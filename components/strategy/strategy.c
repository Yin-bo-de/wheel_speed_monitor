/* strategy 实现：见头文件注释（含状态机设计理由）。 */
#include <string.h>

#include "strategy.h"

/* 每轴运行时状态。仅由喂入方（sampler_task）单线程写，无锁。 */
typedef struct {
    strategy_phase_t phase;
    uint32_t phase_since_ms; /* 进入当前阶段的时刻 */
    uint32_t hold_ms;        /* 本次锁定的保持时长（试探失败逐次翻倍） */
    uint32_t probe_since_ms; /* 试探窗口的计时起点（动/停翻转时重置） */
    bool probe_moving;       /* 计时起点对应的轮子状态：在动 / 停着 */
    float ratio;
    bool slip_fast_left;
    uint16_t target_us;
} axle_runtime_t;

static axle_runtime_t s_axle[STRATEGY_SERVO_COUNT];
static strategy_mode_t s_mode = STRATEGY_MODE_AUTO;
static float s_rpm[STRATEGY_WHEEL_COUNT];

void strategy_config_default(strategy_config_t *cfg)
{
    if (cfg == NULL) {
        return;
    }
    memset(cfg, 0, sizeof(*cfg));
    cfg->mode = STRATEGY_MODE_AUTO;
    for (int i = 0; i < STRATEGY_SERVO_COUNT; i++) {
        cfg->ch_enabled[i] = true;
        cfg->unlock_us[i] = 1500;
        cfg->lock_us[i] = 2000;
        cfg->manual_us[i] = cfg->unlock_us[i];
    }
    cfg->slip_engage_ratio = 0.30f;
    cfg->slip_min_rpm = 10.0f;
    cfg->lock_hold_ms = 3000;
    cfg->lock_hold_max_ms = 30000;
    cfg->probe_window_ms = 2000;
}

esp_err_t strategy_init(void)
{
    memset(s_axle, 0, sizeof(s_axle));
    memset(s_rpm, 0, sizeof(s_rpm));
    s_mode = STRATEGY_MODE_AUTO;
    return ESP_OK;
}

esp_err_t strategy_feed_wheel_rpm(const float rpm[STRATEGY_WHEEL_COUNT],
                                  const strategy_config_t *cfg, uint32_t now_ms)
{
    if (rpm == NULL || cfg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    for (int i = 0; i < STRATEGY_WHEEL_COUNT; i++) {
        s_rpm[i] = rpm[i];
    }
    s_mode = cfg->mode;

    for (uint8_t axle = 0; axle < STRATEGY_SERVO_COUNT; axle++) {
        axle_runtime_t *a = &s_axle[axle];

        float l = rpm[axle * 2];
        float r = rpm[axle * 2 + 1];
        float al = (l < 0.0f) ? -l : l;
        float ar = (r < 0.0f) ? -r : r;
        float base = (al > ar) ? al : ar;

        /* 比值法：对速度尺度不敏感，轮子快慢都适用；双轮静止时比值无意义 */
        a->ratio = (base > 0.0f) ? (((al > ar) ? (al - ar) : (ar - al)) / base) : 0.0f;
        a->slip_fast_left = (al > ar);

        bool moving = (base >= cfg->slip_min_rpm);
        bool slip = moving && (a->ratio > cfg->slip_engage_ratio);

        if (!cfg->ch_enabled[axle]) {
            a->phase = STRATEGY_PHASE_IDLE;
            a->hold_ms = 0;
            a->phase_since_ms = now_ms;
            a->target_us = (uint16_t)cfg->unlock_us[axle];
            continue;
        }

        /* 手动模式直设脉宽，不跑打滑判定（也刻意不走两档——
         * 手动是用来验证 PWM 链路本身的，两档映射留到自动模式看） */
        if (cfg->mode == STRATEGY_MODE_MANUAL) {
            a->phase = STRATEGY_PHASE_IDLE;
            a->hold_ms = 0;
            a->phase_since_ms = now_ms;
            a->target_us = (uint16_t)cfg->manual_us[axle];
            continue;
        }

        switch (a->phase) {
        case STRATEGY_PHASE_IDLE:
            if (slip) {
                a->phase = STRATEGY_PHASE_LOCKED;
                a->phase_since_ms = now_ms;
                a->hold_ms = cfg->lock_hold_ms;
            }
            break;

        case STRATEGY_PHASE_LOCKED:
            /* 刻意不看 ratio：锁上之后两轮被强制同步，速差被自己的动作抹平，
             * 据此判定"已脱困"会形成锁-松-锁的稳定振荡。这里只看时间。 */
            if (now_ms - a->phase_since_ms >= a->hold_ms) {
                a->phase = STRATEGY_PHASE_PROBE;
                a->phase_since_ms = now_ms;
                a->probe_moving = moving;
                a->probe_since_ms = now_ms;
            }
            break;

        case STRATEGY_PHASE_PROBE:
            if (slip) {
                /* 地形没走完：锁回去，下次保持更久（退避） */
                a->phase = STRATEGY_PHASE_LOCKED;
                a->phase_since_ms = now_ms;
                a->hold_ms = (a->hold_ms > cfg->lock_hold_max_ms / 2)
                                 ? cfg->lock_hold_max_ms
                                 : (a->hold_ms * 2);
            } else if (moving != a->probe_moving) {
                /* 刚停下或刚起步，状态还没稳住，重新计时 */
                a->probe_moving = moving;
                a->probe_since_ms = now_ms;
            } else if (now_ms - a->probe_since_ms >= cfg->probe_window_ms) {
                /* 连续保持同一状态满整个窗口且没打滑 → 认定为真脱困。
                 * 停着也算：试探阶段舵机本就已松开，转 IDLE 不改变输出，
                 * 只是把退避等级复位。若这里要求"必须一直在动"，
                 * 停一次车退避等级就永远复位不了，下次轻微打滑直接吃满长锁定。 */
                a->phase = STRATEGY_PHASE_IDLE;
                a->hold_ms = 0;
            }
            break;
        }

        /* 两档映射：只有锁定相位去锁定档，其余（解锁/试探）都在解锁档。
         * 试探期本就该"松开看会不会再打滑"，落点与解锁一致是策略的一部分，
         * 不是缺省值。 */
        a->target_us = (uint16_t)((a->phase == STRATEGY_PHASE_LOCKED)
                                      ? cfg->lock_us[axle]
                                      : cfg->unlock_us[axle]);
    }

    return ESP_OK;
}

esp_err_t strategy_get_state(strategy_state_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(out, 0, sizeof(*out));
    out->mode = s_mode;
    for (int i = 0; i < STRATEGY_WHEEL_COUNT; i++) {
        out->rpm[i] = s_rpm[i];
    }
    for (int i = 0; i < STRATEGY_SERVO_COUNT; i++) {
        out->servo[i].phase = s_axle[i].phase;
        out->servo[i].hold_ms = s_axle[i].hold_ms;
        out->servo[i].ratio = s_axle[i].ratio;
        out->servo[i].slip_fast_left = s_axle[i].slip_fast_left;
        out->servo[i].target_us = s_axle[i].target_us;
    }
    return ESP_OK;
}

esp_err_t strategy_rc_read(uint8_t channel, uint16_t *us)
{
    (void)channel;
    (void)us;
    return ESP_ERR_NOT_SUPPORTED; /* 二期：RC 接收机 PWM 捕获（RMT） */
}
