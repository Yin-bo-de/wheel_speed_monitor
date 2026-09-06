/* wheel_speed_collector 实现：见头文件注释。 */
#include "wheel_speed_collector.h"

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

    return wsmath_update(&st->math, delta, cfg->window_ms, cfg->magnets,
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
