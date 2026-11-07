/* wheel_speed_collector 实现：见头文件注释。 */
#include "wheel_speed_collector.h"

/* 静止归零超时：超过该时长无新脉冲即判定停止并清零 math（攀爬车
 * 低速转动一轮可短至几百 ms 到数秒；2s 上限覆盖尾部怠速停顿）。
 * 间隔法本身无法主动归零（无脉冲就不被调用），由本层兜底。 */
#define IDLE_TIMEOUT_LONG_MS 2000u

/* 间隔切分合并阈值。假脉冲（相机 PCLK/XCLK 耦合到霍尔线）只是把一次
 * 真实轮周期切成两段：短的一段低于上次有效间隔的一半，两段之和又落回
 * 原周期附近。50% 让真实缓加速不被误挂起，±25% 覆盖中断/采样计时抖动。 */
#define SPLIT_HOLD_DEN 2u          /* 新间隔 < 上次有效间隔的 1/2 判可疑 */
#define SPLIT_MERGE_TOL_PCT 25u    /* 合并窗口：上次有效间隔 ±25% */
#define SPLIT_TIMEOUT_NUM 3u       /* 挂起超时 = 上次有效间隔 × 3/2 */
#define SPLIT_TIMEOUT_DEN 2u

/* 长段回滚阈值。假脉冲也可能先切出长段（例如真实 1000ms 被切成
 * 857+143），长段先按正常间隔上报会把 EMA 抬高一个台阶。等短段到来时
 * 回头看：长段比上上段短 10% 以上、且长短两段之和回到上上段 ±25% 内，
 * 就撤销长段上报并重放合并周期。物理假设：真实轮速一步突变不超过 10%
 * （低速验证台成立）；真急加速会被误修正一次，下一脉冲后自愈。 */
#define SPLIT_ROLLBACK_SHORTEN_PCT 90u /* 长段 < 上上段 × 90%（严格不等式） */
#define SPLIT_ROLLBACK_TOL_PCT 25u     /* 合并窗口：上上段 ±25% */

/* 回滚参照失效：停转、换源或挂起超时后，"上上段"和旧 math 快照都不再
 * 代表当前转速。快照必须整体清零（含 has_sample=false），否则之后一次
 * 短段回滚会把停转前的旧样本复活，显示一个不存在的转速。 */
static void rollback_baseline_clear(wheel_chan_state_t *st)
{
    st->prev_valid_interval_us = 0;
    st->math_snapshot = (wsmath_state_t){0};
}

/* 挂起超时：若真是假脉冲切出来的，另一半必然在一个真实周期内到来；
 * 等满 1.5×上次有效间隔仍无新事件，说明轮子停了（或之前是真实急加速）。
 * 丢弃挂起并清空合并基准：真实源每 50ms 会重复返回同一个间隔，基准不清
 * 会让下次采样继续把同一个短间隔判成可疑，轮速永远不更新。 */
static void pending_expire_if_needed(wheel_chan_state_t *st, uint32_t now_ms)
{
    if (st->pending_interval_us == 0) {
        return;
    }
    const uint64_t timeout_ms = (uint64_t)st->last_valid_interval_us *
                                SPLIT_TIMEOUT_NUM / SPLIT_TIMEOUT_DEN / 1000u;
    if ((uint64_t)(uint32_t)(now_ms - st->pending_since_ms) > timeout_ms) {
        st->pending_interval_us = 0;
        st->last_valid_interval_us = 0;
        rollback_baseline_clear(st);
    }
}

esp_err_t wheel_chan_sample(wheel_chan_state_t *st, const wheel_chan_cfg_t *cfg,
                            uint16_t new_counter, uint32_t now_ms)
{
    if (cfg->magnets == 0 || cfg->window_ms == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    pending_expire_if_needed(st, now_ms);

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

    /* 有脉冲时刷新 last_pulse_ms（间隔法在 sample_period 里也会刷新，
     * 此处兜底确保即使本周期无有效间隔但有计数增量时静止判定不被误触） */
    if (delta > 0) {
        st->math.last_pulse_ms = now_ms;
    }

    /* 静止归零兜底：间隔法无脉冲时不会被调用，无法主动归零；
     * 此处检查距上次脉冲是否超时，超时则清零 math 全字段。
     * 清 has_sample 使恢复脉冲时首样本直采，避免缓慢爬升。
     * 合并基准一并清空：停转前的周期不再能代表恢复后的转速。 */
    if (delta == 0 && now_ms - st->math.last_pulse_ms > IDLE_TIMEOUT_LONG_MS) {
        st->math.rpm_ema = 0.0f;
        st->math.speed_cm_s = 0.0f;
        st->math.speed_km_h = 0.0f;
        st->math.freq_hz = 0.0f;
        st->math.has_sample = false;
        st->last_valid_interval_us = 0;
        st->pending_interval_us = 0;
        st->pending_since_ms = 0;
        rollback_baseline_clear(st);
    }

    return ESP_OK;
}

esp_err_t wheel_chan_sample_period(wheel_chan_state_t *st, const wheel_chan_cfg_t *cfg,
                                   uint32_t interval_us, uint32_t now_ms)
{
    if (interval_us == 0) {
        return ESP_ERR_INVALID_ARG; /* 无有效脉冲间隔（未检测/光电异常），不虚假推算 */
    }

    /* 调用顺序是先 sample_period 再 sample，超时检查两边都要有：
     * 这里兜住"长时间无新间隔后又来一个"的情况，sample 里兜住纯停转。 */
    pending_expire_if_needed(st, now_ms);

    st->trigger = true; /* 有间隔即表示本窗口检测到完整脉冲周期 */

    uint32_t report_us = interval_us;

    if (st->pending_interval_us != 0) {
        /* 真实源/模拟源每 50ms 重复返回最近一次间隔：值没变就是同一事件
         * 的重复轮询，既不算合并搭档，也不能重新挂起。 */
        if (interval_us == st->pending_interval_us) {
            return ESP_OK;
        }
        /* 下一个不同间隔到来：与挂起片段相加，落回上次有效间隔 ±25% 就
         * 确认是假脉冲切分，合并成一个真实周期上报。 */
        const uint64_t merged = (uint64_t)st->pending_interval_us + interval_us;
        const uint64_t last = st->last_valid_interval_us;
        st->pending_interval_us = 0;
        if (merged <= UINT32_MAX &&
            merged * 100u > last * (100u - SPLIT_MERGE_TOL_PCT) &&
            merged * 100u < last * (100u + SPLIT_MERGE_TOL_PCT)) {
            report_us = (uint32_t)merged;
        }
        /* 合并不成立：挂起值丢弃，当前间隔按正常值上报（下面统一走数学层） */
    } else if (st->last_valid_interval_us != 0 &&
               (uint64_t)interval_us * SPLIT_HOLD_DEN < st->last_valid_interval_us) {
        /* 长段回滚：假脉冲先切出长段时，长段已经被当成真实周期上报过。
         * 等短段到来，若长段明显短于上上段（>10%，真实轮速不会一步突变
         * 这么多）、且两段之和回落到上上段 ±25% 内，就撤销长段那次上报，
         * 用合并后的真实周期重放 math，抹掉 EMA 上的显示台阶。 */
        const uint64_t merged = (uint64_t)st->last_valid_interval_us + interval_us;
        const uint64_t prev = st->prev_valid_interval_us;
        if (prev != 0 && merged <= UINT32_MAX &&
            (uint64_t)st->last_valid_interval_us * 100u <
                prev * SPLIT_ROLLBACK_SHORTEN_PCT &&
            merged * 100u > prev * (100u - SPLIT_ROLLBACK_TOL_PCT) &&
            merged * 100u < prev * (100u + SPLIT_ROLLBACK_TOL_PCT)) {
            st->math = st->math_snapshot; /* 撤销长段上报 */
            st->suspect_count++;
            st->last_valid_interval_us = (uint32_t)merged;
            /* prev 不变：回滚后上上段仍是后续修正的参照 */
            return wsmath_update_period(&st->math, (uint32_t)merged, cfg->magnets,
                                        cfg->wheel_diam_mm, cfg->alpha, now_ms);
        }

        /* 回滚不成立（首样本无参照或数值对不上）：按原逻辑挂起，
         * 等下一个间隔判断能否合并 */
        st->pending_interval_us = interval_us;
        st->pending_since_ms = now_ms;
        st->suspect_count++;
        return ESP_OK;
    }

    /* 上报前留档：若下一条是假脉冲切出的短段，回滚要用"这一段上报前"的
     * math 状态重放合并周期。prev 记下上上段作为回滚判定的参照。 */
    st->math_snapshot = st->math;
    st->prev_valid_interval_us = st->last_valid_interval_us;
    st->last_valid_interval_us = report_us;

    /* 频率 = 1/间隔；间隔法直接反映两次脉冲真实距离，
     * 低速（1Hz 级）也能准确测频，无窗口法的量化跳变。 */
    return wsmath_update_period(&st->math, report_us, cfg->magnets,
                                cfg->wheel_diam_mm, cfg->alpha, now_ms);
}

void wheel_chan_reset_baseline(wheel_chan_state_t *st, uint16_t current_counter)
{
    st->last_counter = current_counter;
    st->has_baseline = true;
    /* 旧源的间隔不再是新源的合理基准：不清会让换源后的首段间隔被误判
     * 为可疑（挂起）或误合并。可疑计数是累计诊断值，保留。 */
    st->last_valid_interval_us = 0;
    st->pending_interval_us = 0;
    st->pending_since_ms = 0;
    rollback_baseline_clear(st);
}

void wheel_chan_reset_counts(wheel_chan_state_t *st)
{
    st->pulses_total = 0;
}
