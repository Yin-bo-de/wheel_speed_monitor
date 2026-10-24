/* rc_capture 实现：双沿中断测脉宽。设计理由见头文件。 */
#include <string.h>

#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_timer.h"

#include "rc_capture.h"
#include "rc_input.h" /* 脉宽量程常量：与模型层共用同一条边界 */

static const char *TAG = "rccap";

/* ISR 上下文只认这一个实例（单板单实例），init 时绑定 */
static rc_capture_t *s_isr_cap;

/* 双沿共用一个处理函数：读当前电平判方向。
 * 不带 IRAM_ATTR——处理函数里调了 gpio_get_level（在 flash 里），而 ISR 服务
 * 是用 gpio_install_isr_service(0) 装的、没开 ESP_INTR_FLAG_IRAM，中断在
 * flash 关缓存窗口本来就会被推迟，此处标 IRAM 只会给出一个守不住的承诺。
 * 悬空沿（没等到上升沿就来的下降沿）与宽度越界的脉冲都在这里丢掉。 */
static void rc_pulse_isr(void *arg)
{
    uint32_t ch = (uint32_t)(uintptr_t)arg;
    uint32_t now_us = (uint32_t)esp_timer_get_time(); /* 71 分钟回绕，无符号减法自洽 */

    if (gpio_get_level((gpio_num_t)s_isr_cap->gpio[ch])) {
        s_isr_cap->rise_us[ch] = now_us;
        return;
    }

    uint32_t rise = s_isr_cap->rise_us[ch];
    s_isr_cap->rise_us[ch] = 0;
    uint32_t width = (rise != 0) ? (now_us - rise) : 0;
    if (width < RC_US_MIN || width > RC_US_MAX) {
        return; /* 噪声毛刺、接收机故障值、长按不成形的电平，都不算一次捕获 */
    }
    s_isr_cap->width_us[ch] = (uint16_t)width;
    s_isr_cap->fresh[ch] = true;
}

esp_err_t rc_capture_init(rc_capture_t *cap, const int gpio[RC_CAPTURE_CH_COUNT])
{
    if (cap == NULL || gpio == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(cap, 0, sizeof(*cap));
    for (int i = 0; i < RC_CAPTURE_CH_COUNT; i++) {
        cap->gpio[i] = gpio[i];
    }

    uint64_t mask = 0;
    for (int i = 0; i < RC_CAPTURE_CH_COUNT; i++) {
        mask |= (uint64_t)1 << cap->gpio[i];
    }
    gpio_config_t io = {
        .pin_bit_mask = mask,
        .mode = GPIO_MODE_INPUT,
        /* 内部下拉：接收机没上电时信号线悬空，不拉住就会乱跳、每个毛刺都
         * 记一次上升沿 */
        .pull_up_en = 0,
        .pull_down_en = 1,
        /* 先 DISABLE：gpio_config 会重写中断类型，装完 handler 再单独设，
         * 与 wheel_sensor 同一顺序，避免被这一步覆盖 */
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io), TAG, "gpio_config failed");

    /* 先直接挂 handler：ISR 服务由 wheel_sensor 早就装好了，这一步会成功。
     * 不去无脑调 gpio_install_isr_service——它对"已安装"只回一行 ESP_LOGE，
     * 启动日志里会多出一条看着像故障的 E 行。真没装（本组件被单独使用时）
     * 才补装一次。 */
    s_isr_cap = cap; /* 绑实例要在挂 handler 之前：handler 一挂上中断随时可能进来 */
    for (int i = 0; i < RC_CAPTURE_CH_COUNT; i++) {
        esp_err_t e =
            gpio_isr_handler_add((gpio_num_t)cap->gpio[i], rc_pulse_isr, (void *)(uintptr_t)i);
        if (e == ESP_ERR_INVALID_STATE) {
            ESP_RETURN_ON_ERROR(gpio_install_isr_service(0), TAG, "isr service install failed");
            e = gpio_isr_handler_add((gpio_num_t)cap->gpio[i], rc_pulse_isr, (void *)(uintptr_t)i);
        }
        ESP_RETURN_ON_ERROR(e, TAG, "isr handler add failed");
        ESP_RETURN_ON_ERROR(gpio_set_intr_type((gpio_num_t)cap->gpio[i], GPIO_INTR_ANYEDGE),
                            TAG, "set intr type failed");
    }
    cap->inited = true;
    ESP_LOGI(TAG, "RC 捕获就绪 模式=GPIO%d 前=%d 后=%d", cap->gpio[0], cap->gpio[1], cap->gpio[2]);
    return ESP_OK;
}

bool rc_capture_take(rc_capture_t *cap, uint8_t ch, uint16_t *us)
{
    if (cap == NULL || us == NULL || ch >= RC_CAPTURE_CH_COUNT || !cap->fresh[ch]) {
        return false;
    }
    *us = cap->width_us[ch];
    cap->fresh[ch] = false;
    return true;
}
