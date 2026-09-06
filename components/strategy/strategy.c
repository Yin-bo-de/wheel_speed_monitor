/* strategy 桩实现：接口占位，v1 无实际操作。 */
#include <stddef.h>

#include "strategy.h"

static strategy_mode_t g_mode = STRATEGY_MODE_MANUAL;

static const strategy_pwm_config_t g_pwm_cfg = {
    .servo_gpio = -1,
    .servo_min_us = 1000,
    .servo_max_us = 2000,
    .servo_hz = 50,
};

esp_err_t strategy_init(void)
{
    g_mode = STRATEGY_MODE_MANUAL;
    return ESP_OK;
}

esp_err_t strategy_get_status(strategy_status_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    out->supported = false;
    out->mode = g_mode;
    return ESP_OK;
}

esp_err_t strategy_set_mode(strategy_mode_t mode)
{
    g_mode = mode;
    return ESP_OK;
}

esp_err_t strategy_feed_wheel_rpm(const float rpm[4])
{
    (void)rpm; /* v1 无策略消费 */
    return ESP_OK;
}

esp_err_t strategy_get_pwm_config(strategy_pwm_config_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *out = g_pwm_cfg;
    return ESP_OK;
}

esp_err_t strategy_set_servo(uint8_t channel, uint32_t us)
{
    (void)channel;
    (void)us;
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t strategy_rc_read(uint8_t channel, uint16_t *us)
{
    (void)channel;
    (void)us;
    return ESP_ERR_NOT_SUPPORTED;
}
