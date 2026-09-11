/*
 * strategy：前后差速策略引擎（纯 C，无 IDF 依赖，宿主机可测）。
 *
 * 只负责"该转到什么角度"这一个决策，不碰硬件——真正的 PWM 输出在
 * 舵机执行器层（servo_act 模拟模型 / servo_drv LEDC 驱动），由 main 按
 * 配置二选一。这样打滑逻辑与舵机模型都能在宿主机上直接跑单测。
 *
 * 打滑判定为什么不是"比例低于阈值就解锁"：
 * 锁差速这个动作本身会把该轴左右轮速差抹平（两轮被强制同步），于是
 * "比例低于阈值"在锁定后必然成立，必然松开；松开后打滑重现又必然锁回去，
 * 形成稳定振荡。被控制动作污染的信号不能拿来做控制判断。
 * 因此每轴维护一个三态状态机：IDLE（正常判定）→ LOCKED（只看时间，不看
 * 轮速差）→ PROBE（松开试探，试探失败则退回 LOCKED 并延长保持时长）。
 */
#ifndef STRATEGY_H
#define STRATEGY_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define STRATEGY_SERVO_COUNT 2 /* 0=前差速 1=后差速 */
#define STRATEGY_WHEEL_COUNT 4 /* 左前,右前,左后,右后 */

typedef enum {
    STRATEGY_MODE_MANUAL = 0, /* 手动：直设目标脉宽，不跑打滑判定 */
    STRATEGY_MODE_AUTO,       /* 自动：打滑检测自动锁差速 */
} strategy_mode_t;

/* 每轴状态机阶段。 */
typedef enum {
    STRATEGY_PHASE_IDLE = 0, /* 解锁，正常盯比例 */
    STRATEGY_PHASE_LOCKED,   /* 锁定保持中——只看时间，不看轮速差 */
    STRATEGY_PHASE_PROBE,    /* 松开试探中——动/停状态不变地连续保持满窗口即算脱困 */
} strategy_phase_t;

typedef struct {
    uint32_t min_us;    /* 脉宽下限（满偏一端） */
    uint32_t center_us; /* 中位脉宽 = 解锁位置 */
    uint32_t max_us;    /* 脉宽上限（满偏另一端） */
    bool invert[STRATEGY_SERVO_COUNT];     /* 反向映射（舵机装反时勾） */
    bool ch_enabled[STRATEGY_SERVO_COUNT]; /* 通道使能；关=输出中位且不判定 */
    strategy_mode_t mode;
    uint32_t manual_us[STRATEGY_SERVO_COUNT]; /* 手动模式目标脉宽（不应用 invert） */

    float slip_engage_ratio; /* 打滑进入门槛（左右速差比例） */
    float slip_min_rpm;      /* 低于此转速不判定：静止时比值无意义 */
    uint32_t lock_hold_ms;      /* 初始锁定保持时长（"地形时间"） */
    uint32_t lock_hold_max_ms;  /* 保持时长翻倍上限 */
    uint32_t probe_window_ms;   /* 试探窗口：车连续动满此时长才算脱困 */
} strategy_config_t;

typedef struct {
    strategy_phase_t phase;
    uint32_t hold_ms;    /* 本次锁定的保持时长（试探失败逐次翻倍） */
    float ratio;         /* 诊断：最近一次算出的左右速差比例 */
    bool slip_fast_left; /* 诊断：哪一侧更快 */
    uint16_t target_us;  /* 本周期该输出的目标脉宽；首次 feed 之前为 0（未计算） */
} strategy_servo_state_t;

typedef struct {
    strategy_mode_t mode;
    float rpm[STRATEGY_WHEEL_COUNT]; /* 输入快照（诊断用） */
    strategy_servo_state_t servo[STRATEGY_SERVO_COUNT];
} strategy_state_t;

/* 填入缺省配置（脉宽 1000/1500/2000、正映射、使能、自动模式、
 * 门槛 0.30、最低 10 转/分、保持 3000ms、上限 30000ms、试探 2000ms）。 */
void strategy_config_default(strategy_config_t *cfg);

/* 复位状态机与保持计时。模式/使能/映射/门槛变更后由上层调用，
 * 避免带着上一组参数的锁定状态进入新配置。 */
esp_err_t strategy_init(void);

/* 喂四轮转速（转/分）并推进状态机一步。
 * cfg 由调用方每次传入而非内部保存——上层（main）从自己的配置快照构造，
 * 避免任务间共享可变状态带来的读写撕裂。now_ms 单调递增（可回绕）。 */
esp_err_t strategy_feed_wheel_rpm(const float rpm[STRATEGY_WHEEL_COUNT],
                                  const strategy_config_t *cfg, uint32_t now_ms);

esp_err_t strategy_get_state(strategy_state_t *out);

/* 角度→脉宽（纯函数）：deg∈[-90,+90]，0→center，正角趋向 max 端；
 * invert 先取反；超出 ±90 钳位。ch 越界返回 center。 */
uint32_t strategy_deg_to_us(const strategy_config_t *cfg, uint8_t ch, float deg);

/* 脉宽→角度，与 strategy_deg_to_us 互逆（渲染层回显"到达角度"用）。 */
float strategy_us_to_deg(const strategy_config_t *cfg, uint8_t ch, float us);

/* RC 通道读捕获脉宽（µs）。二期未接 RC 输入，恒返回 ESP_ERR_NOT_SUPPORTED。 */
esp_err_t strategy_rc_read(uint8_t channel, uint16_t *us);

#ifdef __cplusplus
}
#endif

#endif /* STRATEGY_H */
