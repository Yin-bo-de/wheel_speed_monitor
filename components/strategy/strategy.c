/* strategy 实现：见头文件注释（含状态机设计理由）。 */
#include <math.h>
#include <string.h>

#include "strategy.h"

/* 手动模式"锁止档命令"的判定容差：手动目标落在锁定档 ±20µs 内才算锁止
 * 命令、才需要过闸门。与前端 index.html 的 DETENT_TOL_US 同一约定，
 * 保证页面显示的"锁定位"与闸门判定一致。 */
#define STRATEGY_DETENT_TOL_US 20

/* 每轴运行时状态。仅由喂入方（sampler_task）单线程写，无锁。 */
typedef struct {
    strategy_phase_t phase;
    uint32_t phase_since_ms; /* 进入当前阶段的时刻 */
    uint32_t slip_since_ms;  /* 打滑确认计时起点；0=当前不在确认中 */
    uint32_t hold_ms;        /* 本次锁定的保持时长（试探失败逐次翻倍） */
    uint32_t probe_since_ms; /* 试探窗口的计时起点（动/停翻转时重置） */
    bool probe_moving;       /* 计时起点对应的轮子状态：在动 / 停着 */
    float ratio;
    bool slip_fast_left;
    uint32_t sync_since_ms;  /* 双闸门窗口连续满足计时起点；0=当前不满足 */
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
    cfg->slip_confirm_ms = 3000;
    cfg->lock_hold_ms = 3000;
    cfg->lock_hold_max_ms = 30000;
    cfg->probe_window_ms = 2000;
    /* 打牙防护起步值偏保守：宁可多等一拍也不打牙。速差闸门 40 取自方案
     * 30~50 区间；车速闸门 60 与稳定期 300 的标定方法见 docs 方案「阈值标定」 */
    cfg->lock_max_speed_rpm = 60.0f;
    cfg->lock_max_delta_rpm = 40.0f;
    cfg->lock_sync_stable_ms = 300;
    cfg->manual_gate_enable = true;
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

        /* 双闸门（每拍计算，al/ar 已取绝对值）：慢轮 ≈ 车身速度（打滑时
         * 空转轮不推车），速差直接对应锁止瞬间的冲击扭矩。两闸门独立、
         * 缺一不可：sync_ok 管啮合冲击，speed_ok 管高速风险与 EMA 滞后兜底。 */
        float slow = (al < ar) ? al : ar;
        bool speed_ok = (slow < cfg->lock_max_speed_rpm);
        bool sync_ok = (fabsf(al - ar) < cfg->lock_max_delta_rpm);

        if (!cfg->ch_enabled[axle]) {
            a->phase = STRATEGY_PHASE_IDLE;
            a->hold_ms = 0;
            a->slip_since_ms = 0;
            a->phase_since_ms = now_ms;
            a->target_us = (uint16_t)cfg->unlock_us[axle];
            continue;
        }

        /* 手动模式直设脉宽，不跑打滑判定（也刻意不走两档——
         * 手动是用来验证 PWM 链路本身的，两档映射留到自动模式看）。
         * 打牙防护同样覆盖手动：锁止档命令且闸门不满足时，输出保持解锁档，
         * 窗口连续稳定满 lock_sync_stable_ms 才放行——机械保护被舵机硬推
         * 绕过，软件在这里补回来。解锁档命令/闸门关闭则原样直通。 */
        if (cfg->mode == STRATEGY_MODE_MANUAL) {
            uint16_t want = (uint16_t)cfg->manual_us[axle];
            int d_lock = (int)want - (int)cfg->lock_us[axle];
            bool lock_cmd = cfg->manual_gate_enable &&
                            (d_lock >= -STRATEGY_DETENT_TOL_US &&
                             d_lock <= STRATEGY_DETENT_TOL_US);
            if (!lock_cmd) {
                a->phase = STRATEGY_PHASE_IDLE;
                a->hold_ms = 0;
                a->slip_since_ms = 0;
                a->sync_since_ms = 0;
                a->phase_since_ms = now_ms;
                a->target_us = want;
                continue;
            }
            if (speed_ok && sync_ok) {
                if (a->sync_since_ms == 0) {
                    a->sync_since_ms = now_ms;
                }
                if (now_ms - a->sync_since_ms >= cfg->lock_sync_stable_ms) {
                    /* 已稳定：持续放行锁止档，直到窗口被尖峰打断再重新计时 */
                    a->target_us = want;
                    a->phase = STRATEGY_PHASE_IDLE;
                } else {
                    /* 等窗口稳定：打牙防护优先于操作者意图，输出扣在解锁档，
                     * phase 报 pending_sync，页面显示"等待同步"提示松油门 */
                    a->phase = STRATEGY_PHASE_PENDING_SYNC;
                    a->target_us = (uint16_t)cfg->unlock_us[axle];
                }
            } else {
                a->sync_since_ms = 0;
                a->phase = STRATEGY_PHASE_PENDING_SYNC;
                a->target_us = (uint16_t)cfg->unlock_us[axle];
            }
            a->hold_ms = 0;
            a->slip_since_ms = 0;
            a->phase_since_ms = now_ms;
            continue;
        }

        switch (a->phase) {
        case STRATEGY_PHASE_IDLE:
            if (slip) {
                /* 单拍速差可能只是过坎/采样毛刺：连续满足整段确认期才带出
                 * 锁止意图，确认期间输出仍是解锁档。 */
                if (a->slip_since_ms == 0) {
                    a->slip_since_ms = now_ms;
                }
                if (now_ms - a->slip_since_ms >= cfg->slip_confirm_ms) {
                    /* 确认期满不直接落锁：高速/大速差下直接锁就是打牙。
                     * 带着锁止意图进等待同步，等司机松油门、速差收敛。 */
                    a->phase = STRATEGY_PHASE_PENDING_SYNC;
                    a->phase_since_ms = now_ms;
                    a->hold_ms = cfg->lock_hold_ms; /* 锁定时长先备好，落锁即用 */
                    a->sync_since_ms = 0;
                    a->slip_since_ms = 0;
                }
            } else {
                /* 速差回落或轮子停下：确认必须从头再来 */
                a->slip_since_ms = 0;
            }
            break;

        case STRATEGY_PHASE_PENDING_SYNC:
            /* 等待的前提是打滑持续，打滑消失自然回 IDLE——因此刻意不设
             * 等待超时，超时没有额外信息量。 */
            if (!slip) {
                a->phase = STRATEGY_PHASE_IDLE;
                a->phase_since_ms = now_ms;
                a->hold_ms = 0; /* 没真正锁过，退避不该累积 */
                a->sync_since_ms = 0;
                break;
            }
            if (speed_ok && sync_ok) {
                if (a->sync_since_ms == 0) {
                    a->sync_since_ms = now_ms;
                }
                if (now_ms - a->sync_since_ms >= cfg->lock_sync_stable_ms) {
                    a->phase = STRATEGY_PHASE_LOCKED;
                    a->phase_since_ms = now_ms;
                    a->sync_since_ms = 0;
                }
            } else {
                a->sync_since_ms = 0;
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
                /* 地形没走完：退避翻倍先算好（下面两条路都要保留）。
                 * 同步窗口在手 → 直接锁回（冲击小，不必再等）；
                 * 窗口不在手 → 落 PENDING_SYNC 等同步，此刻锁回同样是打牙。 */
                a->hold_ms = (a->hold_ms > cfg->lock_hold_max_ms / 2)
                                 ? cfg->lock_hold_max_ms
                                 : (a->hold_ms * 2);
                if (speed_ok && sync_ok) {
                    a->phase = STRATEGY_PHASE_LOCKED;
                    a->phase_since_ms = now_ms;
                } else {
                    a->phase = STRATEGY_PHASE_PENDING_SYNC;
                    a->phase_since_ms = now_ms;
                    a->sync_since_ms = 0;
                }
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

        /* 两档映射：只有锁定相位去锁定档，其余（解锁/等待同步/试探）都在
         * 解锁档。等待同步期本就该扣住不锁，落点与解锁一致是策略的一部分，
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
