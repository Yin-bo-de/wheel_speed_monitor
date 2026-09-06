/* wheel_sensor 实现：PCNT 4 单元 + 4 路 GPIO 电平监视。 */
#include <string.h>

#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "esp_check.h"
#include "esp_log.h"

#include "wheel_sensor.h"

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
    }
    for (int i = 0; i < WHEEL_SENSOR_COUNT; i++) {
        ESP_RETURN_ON_ERROR(init_gpio((uint8_t)i), TAG, "err");

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
