/* wheel_sensor 实现：PCNT 4 单元 + 4 路 GPIO 电平监视 + GPIO 下降沿中断（间隔测频）。 */
#include <string.h>

#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "wheel_sensor.h"

/* 有效脉冲最小间隔（us）：小于此值的下降沿视为磁铁贴近时的接触抖动/双探针，
 * 从"有效脉冲计数"与"间隔测频"中剔除。PCNT 硬件仍会计数（兜底），
 * 但本模块对外报告的累计与频率均以 ISR 的有效脉冲为准。
 * 最小真实扫频：高速轮也不低于 ~100Hz（间隔 10ms），这里取 5ms 余量。 */
#define PULSE_DEBOUNCE_US 5000u

typedef struct {
    volatile uint32_t last_interval_us; /* 最近相邻两次"有效"下降沿间隔 */
    volatile uint32_t last_edge_us;     /* 最近一次"有效"下降沿时刻 */
    volatile uint32_t valid_count;    /* 有效脉冲总数（去抖后） */
} pulse_meta_t;

static pulse_meta_t s_pulse_meta[WHEEL_SENSOR_COUNT];

/* ISR：下降沿；<5ms 间隔视为抖动丢弃。单写单读 volatile，无锁安全。 */
static void IRAM_ATTR pulse_isr(void *arg)
{
    uint32_t wheel = (uint32_t)(uintptr_t)arg;
    uint32_t now_us = (uint32_t)esp_timer_get_time();
    uint32_t prev_us = s_pulse_meta[wheel].last_edge_us;
    uint32_t interval = (prev_us && now_us > prev_us) ? (now_us - prev_us) : 0;

    if (interval == 0) {
        s_pulse_meta[wheel].last_edge_us = now_us; /* 首个沿：只设基准 */
    } else if (interval >= PULSE_DEBOUNCE_US) {
        s_pulse_meta[wheel].last_interval_us = interval;
        s_pulse_meta[wheel].valid_count += 1; /* 有效一次扫过 */
        s_pulse_meta[wheel].last_edge_us = now_us;
    } else {
        /* 抖动：不更新间隔（保持上次有效值），也不计入有效计数。
         * last_edge_us 不更新，让"有效间隔"仍以上次为准。 */
    }
}

static const char *TAG = "wheelsensor";

/* A3144 触发脉冲宽 ≥ microsecond 级；200ns 滤毛刺只滤电气噪声。
 * （原在 main/wheel_config.h，为使本组件不依赖板级配置而内联） */
#define WHEEL_SENSOR_GLITCH_NS 200

typedef struct {
    pcnt_unit_handle_t unit;
    pcnt_channel_handle_t chan;
    bool enabled;
} sensor_slot_t;

static sensor_slot_t s_slots[WHEEL_SENSOR_COUNT];
static int s_gpios[WHEEL_SENSOR_COUNT]; /* 由 init 传入（main/wheel_config.h 定义） */

/* GPIO 输入配置：内部上拉使能（面包板外部上拉的冗余）。
 * 若外部上拉到 3.3V，内部上拉只是同向协助，不会拉低。 */
static esp_err_t init_gpio(uint8_t wheel)
{
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << s_gpios[wheel],
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 1,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE,
    };
    return gpio_config(&cfg);
}

esp_err_t wheel_sensor_read_gpio_levels(uint8_t levels[WHEEL_SENSOR_COUNT])
{
    for (int i = 0; i < WHEEL_SENSOR_COUNT; i++) {
        levels[i] = (uint8_t)gpio_get_level(s_gpios[i]);
    }
    return ESP_OK;
}

esp_err_t wheel_sensor_init(const uint8_t gpios[WHEEL_SENSOR_COUNT])
{
    if (gpios == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    for (int i = 0; i < WHEEL_SENSOR_COUNT; i++) {
        s_gpios[i] = gpios[i]; /* 逐个赋值：gpios 是 uint8_t，不能 memcpy 进 int[] */
        s_pulse_meta[i] = (pulse_meta_t){0};
    }

    for (int i = 0; i < WHEEL_SENSOR_COUNT; i++) {
        ESP_RETURN_ON_ERROR(init_gpio((uint8_t)i), TAG, "err");
    }

    /* GPIO ISR service（此前漏装导致间隔恒 0、RPS 不更新）。
     * 注意顺序：先 init_gpio（gpio_config 会置 intr_type=DISABLE），
     * 再装 handler + 设 NEGEDGE，避免被 init 重配覆盖。 */
    ESP_RETURN_ON_ERROR(gpio_install_isr_service(0), TAG, "err");
    for (int i = 0; i < WHEEL_SENSOR_COUNT; i++) {
        ESP_RETURN_ON_ERROR(gpio_isr_handler_add(s_gpios[i], pulse_isr, (void *)(uintptr_t)i), TAG, "err");
        ESP_RETURN_ON_ERROR(gpio_set_intr_type(s_gpios[i], GPIO_INTR_NEGEDGE), TAG, "err");
    }

    for (int i = 0; i < WHEEL_SENSOR_COUNT; i++) {
        pcnt_unit_config_t unit_cfg = {
            .high_limit = 32767, /* ±32767 内递增；回绕由采集器判定 */
            .low_limit = -32768,
        };
        ESP_RETURN_ON_ERROR(pcnt_new_unit(&unit_cfg, &s_slots[i].unit), TAG, "err");

        pcnt_glitch_filter_config_t glitch = {
            .max_glitch_ns = WHEEL_SENSOR_GLITCH_NS,
        };
        ESP_RETURN_ON_ERROR(pcnt_unit_set_glitch_filter(s_slots[i].unit, &glitch), TAG, "err");

        pcnt_chan_config_t chan_cfg = {
            .edge_gpio_num = s_gpios[i],
            .level_gpio_num = -1,  /* 无控制输入，单边沿计数 */
        };
        ESP_RETURN_ON_ERROR(pcnt_new_channel(s_slots[i].unit, &chan_cfg, &s_slots[i].chan), TAG, "err");

        /* A3144 南极靠近→输出拉低（下降沿）；每次下降沿 +1。
         * 上升沿不计（磁铁经过既无重复计数）。 */
        ESP_RETURN_ON_ERROR(pcnt_channel_set_edge_action(
            s_slots[i].chan, PCNT_CHANNEL_EDGE_ACTION_HOLD,
            PCNT_CHANNEL_EDGE_ACTION_INCREASE), TAG, "err");
        ESP_RETURN_ON_ERROR(pcnt_channel_set_level_action(
            s_slots[i].chan,
            PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_KEEP), TAG, "err");

        ESP_RETURN_ON_ERROR(pcnt_unit_enable(s_slots[i].unit), TAG, "err");
        ESP_RETURN_ON_ERROR(pcnt_unit_clear_count(s_slots[i].unit), TAG, "err");
        s_slots[i].enabled = true;
    }
    for (int i = 0; i < WHEEL_SENSOR_COUNT; i++) {
        ESP_RETURN_ON_ERROR(pcnt_unit_start(s_slots[i].unit), TAG, "err");
    }
    return ESP_OK;
}

esp_err_t wheel_sensor_read_count(uint8_t wheel, uint16_t *count)
{
    if (wheel >= WHEEL_SENSOR_COUNT || count == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    int v = 0;
    ESP_RETURN_ON_ERROR(pcnt_unit_get_count(s_slots[wheel].unit, &v), TAG, "err");
    *count = (uint16_t)v; /* 取 16 位低位，回绕由采集器判定 */
    return ESP_OK;
}

esp_err_t wheel_sensor_get_period_us(uint8_t wheel, uint32_t *interval_us)
{
    if (wheel >= WHEEL_SENSOR_COUNT || interval_us == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    /* 间隔过期判定：距最近一次有效脉冲已超"上次间隔×1.5"（即已超过一个
     * 本轮周期仍无新脉冲）→ 判定停止，返回 0。否则把历史间隔当新鲜处理
     * 会导致磁铁离开后 RPS 永久挂在旧值。 */
    uint32_t since_us = (uint32_t)esp_timer_get_time() - s_pulse_meta[wheel].last_edge_us;
    uint32_t last_interval = s_pulse_meta[wheel].last_interval_us;
    if (last_interval == 0 || since_us > last_interval + last_interval / 2) {
        *interval_us = 0;
    } else {
        *interval_us = last_interval;
    }
    return ESP_OK;
}

esp_err_t wheel_sensor_get_valid_count(uint8_t wheel, uint16_t *count)
{
    if (wheel >= WHEEL_SENSOR_COUNT || count == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *count = (uint16_t)s_pulse_meta[wheel].valid_count; /* 低 16 位，回绕由采集器判定 */
    return ESP_OK;
}

esp_err_t wheel_sensor_set_enabled(uint8_t wheel, bool enabled)
{
    if (wheel >= WHEEL_SENSOR_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    if (enabled) {
        ESP_RETURN_ON_ERROR(pcnt_unit_clear_count(s_slots[wheel].unit), TAG, "err");
        ESP_RETURN_ON_ERROR(pcnt_unit_start(s_slots[wheel].unit), TAG, "err");
    } else {
        ESP_RETURN_ON_ERROR(pcnt_unit_stop(s_slots[wheel].unit), TAG, "err");
    }
    s_slots[wheel].enabled = enabled;
    return ESP_OK;
}
