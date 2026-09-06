/* wheel_speed_collector 实现：见头文件注释。 */
#include "wheel_speed_collector.h"

/* 间隔法静止判定：超过该时长无新脉冲即判定停止（攀爬车低速转动，
 * 一轮可短至几百 ms 到数秒；2s 上限覆盖尾部怠速停顿） */
#define IDLE_TIMEOUT_LONG_MS 2000u

esp_err_t wheel_chan_sample(wheel_chan_state_t *st, const wheel_chan_cfg_t *cfg,
                            uint16_t new_counter, uint32_t now_ms)
{
    if (cfg->magnets == 0 || cfg->window_ms == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t delta = 0;
    if (!st->has_baseline) {
        /* 首窗口只建立基准，不产生脉冲：开机瞬间计数器语义未知 */
        st->last_counter = new_counter;
        st->has_baseline = true;
    } else if (new_counter >= st->last_counter) {
        delta = new_counter - st->last_counter;
    } else {
        /* 计数回退：回绕或清零复位。PCNT 是 16 位，必须区分这两种情况，
         * 否则清零复位会被当成 65536+ 脉冲造成巨大 RPM 毛刺。 */
        if (st->last_counter > WHEEL_WRAP_HIGH && new_counter < WHEEL_WRAP_LOW) {
            delta = WHEEL_COUNTER_WRAP - st->last_counter + new_counter;
        } else {
            delta = 0; /* 清零复位：本窗口 0 脉冲，基准改为新值 */
        }
    }
    st->last_counter = new_counter;

    /* 最小间隔去抖：窗口内探测到多脉冲、且窗口时长大于去抖间隔时，
     * 只保留第一个脉冲，丢弃紧随其后的（磁铁双探针效应）。 */
    if (delta > 1 && cfg->debounce_ms > 0 && cfg->window_ms >= cfg->debounce_ms) {
        delta = 1;
    }

    st->trigger = (delta > 0);
    st->pulses_total += delta;

    /* 窗口法只维护累计脉冲与 trigger；math(频率/RPM) 由间隔法维护。
     * 主采样路径调用顺序：先 period（更新 math）再 sample（只累计），
     * 避免窗口法 0 脉冲/20Hz 覆盖间隔法的低速准确值。 */
    if (st->use_period) {
        if (delta == 0) {
            /* 间隔法下仍要处理静止归零：窗口法不驱动 EMA，
             * 这里只做"长期无脉冲 → 速度清零" */
            if (now_ms - st->math.last_pulse_ms > IDLE_TIMEOUT_LONG_MS) {
                st->math.rpm_ema = 0.0f;
                st->math.speed_cm_s = 0.0f;
                st->math.speed_km_h = 0.0f;
                st->math.freq_hz = 0.0f;
            }
        }
        return ESP_OK;
    }
    return wsmath_update(&st->math, delta, cfg->window_ms, cfg->magnets,
                         cfg->wheel_diam_mm, cfg->alpha, now_ms);
}

esp_err_t wheel_chan_sample_period(wheel_chan_state_t *st, const wheel_chan_cfg_t *cfg,
                                   uint32_t interval_us, uint32_t now_ms)
{
    if (interval_us == 0) {
        return ESP_ERR_INVALID_ARG; /* 无有效脉冲间隔（未检测/光电异常），不虚假推算 */
    }
    st->trigger = true; /* 有间隔即表示本窗口检测到完整脉冲周期 */
    /* 频率 = 1/间隔；注意与窗口法的关系：窗口法在单窗口 1 脉冲恒报
     * 1s/window_ms 倍频，间隔法直接反映两次脉冲真实距离 */
    return wsmath_update_period(&st->math, interval_us, cfg->magnets,
                                cfg->wheel_diam_mm, cfg->alpha, now_ms);
}

void wheel_chan_reset_baseline(wheel_chan_state_t *st, uint16_t current_counter)
{
    st->last_counter = current_counter;
    st->has_baseline = true;
}

void wheel_chan_reset_counts(wheel_chan_state_t *st)
{
    st->pulses_total = 0;
}
