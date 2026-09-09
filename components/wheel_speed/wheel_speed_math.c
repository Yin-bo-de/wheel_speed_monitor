/* wheel_speed_math 实现：见头文件注释。 */
#include <math.h>

#include "wheel_speed_math.h"

esp_err_t wsmath_update_period(wsmath_state_t *st, uint32_t interval_us,
                               uint32_t magnets, uint32_t wheel_diam_mm,
                               float alpha, uint32_t now_ms)
{
    if (magnets == 0 || interval_us == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    /* 频率 = 1/interval；1e6us = 1s，直接反映每秒转圈数 */
    const float freq_hz = 1000000.0f / (float)interval_us;
    const float raw_rpm = freq_hz * 60.0f / (float)magnets;
    const float raw_speed_cm_s = raw_rpm * (float)M_PI * (float)wheel_diam_mm / 600.0f;
    const float raw_speed_km_h = raw_speed_cm_s * 3.6f / 100.0f;

    (void)raw_speed_km_h;

    st->freq_hz = freq_hz;
    st->last_pulse_ms = now_ms;

    /* EMA：首样本直接采纳，避免从 0 缓慢爬升掩盖真实转速台阶。
     * 静止归零（清 has_sample）由上层采集器超时判定后做。 */
    if (!st->has_sample) {
        st->rpm_ema = raw_rpm;
        st->has_sample = true;
    } else {
        st->rpm_ema = alpha * raw_rpm + (1.0f - alpha) * st->rpm_ema;
    }
    st->speed_cm_s = st->rpm_ema * (float)M_PI * (float)wheel_diam_mm / 600.0f;
    st->speed_km_h = st->speed_cm_s * 3.6f / 100.0f;

    return ESP_OK;
}
