/*
 * servo_act：舵机执行器抽象（纯 C，无 IDF 依赖，宿主机可测）。
 *
 * 两个同构实现，由上层按配置二选一：
 *   - servo_sim  模拟舵机模型：按速率限制朝目标推进，如实报告"现在转到哪"，
 *                让控制链路在没有真实舵机时也能被完整验证（对应轮速链里的
 *                sim_source）。
 *   - servo_ledc 真实 LEDC 输出（IDF 依赖，不参与宿主机测试）。
 *
 * 位置一律用脉宽 µs 表达：舵机位置对脉宽单调，模型不必再引入"角度"这一层
 * 抽象。策略层给的也是两档脉宽（解锁/锁定），到这里就是"去哪两个点"。
 */
#ifndef SERVO_ACT_H
#define SERVO_ACT_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SERVO_ACT_CHANNELS 2
#define SERVO_ACT_MIN_US 500
#define SERVO_ACT_MAX_US 2500

typedef struct {
    bool enabled;      /* 通道使能；关=不输出信号 */
    bool reached;      /* 是否已到达目标（LEDC 无位置反馈，恒 true） */
    float cur_us;      /* 当前位置 */
    float target_us;   /* 目标位置 */
} servo_act_chan_state_t;

/* 运行时可切换的操作集。init 不在表内：具体实现各有自己的初始化函数
 * （参数不同），在启动时或切换实现时调用一次即可。 */
typedef struct {
    esp_err_t (*set_us)(void *ctx, uint8_t ch, uint32_t us);
    esp_err_t (*set_enabled)(void *ctx, uint8_t ch, bool en);
    /* 推进内部状态到 now_ms。模拟实现据此移动位置；LEDC 实现为空操作
     * （硬件自己会走）。now_ms 单调递增（可回绕）。 */
    esp_err_t (*step)(void *ctx, uint32_t now_ms);
    esp_err_t (*get_state)(void *ctx, servo_act_chan_state_t out[SERVO_ACT_CHANNELS]);
} servo_act_ops_t;

/* ---- 模拟舵机实现 ---- */

typedef struct {
    float cur_us[SERVO_ACT_CHANNELS];
    float target_us[SERVO_ACT_CHANNELS];
    float max_rate_us_s;  /* 行程速率上限（µs/秒）；越小，"到达过程"越慢越可见 */
    uint32_t last_ms;
    bool enabled[SERVO_ACT_CHANNELS];
    bool primed;          /* 首次 step 只建立时间基准，不移动 */
} servo_sim_t;

/* 起始位置逐通道给定，通常取配置里的解锁档脉宽——上电时差速应当是松开的，
 * 且前后两轴的解锁落点可以不同，取一个共用值会让其中一轴开机就偏。 */
void servo_sim_init(servo_sim_t *s, float max_rate_us_s,
                    const uint32_t start_us[SERVO_ACT_CHANNELS]);

extern const servo_act_ops_t servo_sim_ops;

#ifdef __cplusplus
}
#endif

#endif /* SERVO_ACT_H */
