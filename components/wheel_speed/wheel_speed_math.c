/* wheel_speed_math 实现：见头文件注释。 */
#include <math.h>
#include <string.h>

#include "wheel_speed_math.h"

#define STATIC_IDLE_TIMEOUT_MS 1000u /* 无脉冲超过 1 秒判定静止，复位 EMA */

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

    /* 静止判定直接在间隔法外做（间隔过时由上层归零），此处仅 EMA */
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

esp_err_t wsmath_update(wsmath_state_t *st, uint32_t pulses_this_window,
                        uint32_t window_ms, uint32_t magnets,
                        uint32_t wheel_diam_mm, float alpha, uint32_t now_ms)
{
    if (magnets == 0 || window_ms == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    const float freq_hz = (float)pulses_this_window * 1000.0f / (float)window_ms;
    /* 频率×60 = 每分钟脉冲数，除以磁铁数 = 转/分（RPM） */
    const float raw_rpm = freq_hz * 60.0f / (float)magnets;
    /* RPM × π × 轮径（mm）= 每分钟行驶 mm；/600 → cm/s（60s * 10mm/cm） */
    const float raw_speed_cm_s = raw_rpm * (float)M_PI * (float)wheel_diam_mm / 600.0f;
    const float raw_speed_km_h = raw_speed_cm_s * 3.6f / 100.0f;

    (void)raw_speed_km_h; /* 保持换算链完整，仅作单元测试锁定口径 */

    st->freq_hz = freq_hz;

    if (pulses_this_window > 0) {
        st->last_pulse_ms = now_ms;
    }

    /* 静止判定：距最后一次脉冲超过阈值 → 显著归零并清状态，
     * 使随后恢复脉冲以首样本直接采纳，避免"回魂"式缓慢爬升。 */
    if (now_ms - st->last_pulse_ms > STATIC_IDLE_TIMEOUT_MS) {
        st->rpm_ema = 0.0f;
        st->speed_cm_s = 0.0f;
        st->speed_km_h = 0.0f;
        st->has_sample = false;
        return ESP_OK;
    }

    if (pulses_this_window > 0) {
        /* EMA：首样本直接采纳，避免从 0 缓慢爬升掩盖真实转速台阶。 */
        if (!st->has_sample) {
            st->rpm_ema = raw_rpm;
            st->has_sample = true;
        } else {
            st->rpm_ema = alpha * raw_rpm + (1.0f - alpha) * st->rpm_ema;
        }
    } else if (st->has_sample) {
        /* 无脉冲窗口：朝 0 衰减，避免旧转速长时间撑住；
         * （首样本不在此采纳，否则 0 脉冲窗口会把转速从 0 起爬） */
        st->rpm_ema = (1.0f - alpha) * st->rpm_ema;
    }

    st->speed_cm_s = st->rpm_ema * (float)M_PI * (float)wheel_diam_mm / 600.0f;
    st->speed_km_h = st->speed_cm_s * 3.6f / 100.0f;

    return ESP_OK;
}
