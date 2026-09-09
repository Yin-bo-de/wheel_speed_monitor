/*
 * wheel_speed_collector：每轮采集器的核心状态机，纯 C 无 IDF 依赖。
 * 职责：原始计数器(PCNT 16 位)差值 → 回绕/清零复位判定 → 累计脉冲
 *       → 最小脉冲间隔去抖 → 长期静止归零。
 * 频率/RPM/EMA 由 wheel_chan_sample_period（间隔法）单独维护，
 * 本层的 wheel_chan_sample 只做累计与静止兜底，不计算频率。
 */
#ifndef WHEEL_SPEED_COLLECTOR_H
#define WHEEL_SPEED_COLLECTOR_H

#include "esp_err.h"
#include "wheel_speed_math.h"

#ifdef __cplusplus
extern "C" {
#endif

#define WHEEL_COUNTER_WRAP 65536u   /* 16 位计数器回绕周期 */

/* 回绕判定阈值：旧计数超过 HIGH 且新计数低于 LOW 时视为 16 位回绕；
 * 否则新计数小于旧计数视为计数器被清零复位（非回绕）。PCNT 每窗口增量很小，
 * 这两个阈值把"半圈回绕"这种误判隔在窗口之外。 */
#define WHEEL_WRAP_HIGH 60000u
#define WHEEL_WRAP_LOW 5000u

typedef struct {
    uint32_t window_ms;     /* 采样窗口时长（毫秒），仅用于去抖判定 */
    uint32_t magnets;       /* 每转磁铁数，>= 1 */
    uint32_t wheel_diam_mm; /* 轮径，毫米 */
    float alpha;            /* EMA 系数 0..1，越大越灵敏 */
    uint32_t debounce_ms;   /* 最小脉冲间隔去抖；0=关闭（高于此值认为同窗口内多余脉冲无效） */
} wheel_chan_cfg_t;

typedef struct {
    uint16_t last_counter;        /* 上一窗口原始计数值（回绕判定基准） */
    uint32_t pulses_total;        /* 累计脉冲（跨回绕单调递增） */
    wsmath_state_t math;          /* 数学层状态（EMA/频率/线速度），由 sample_period 维护 */
    bool trigger;                 /* 本窗口是否有脉冲增量 */
    bool has_baseline;            /* 是否已建立首窗口基准 */
} wheel_chan_state_t;

/*
 * 推进一个采样窗口（仅累计脉冲 + 静止归零兜底，不计算频率/RPM）。
 * new_counter 为该轮 PCNT（或模拟源）当前计数值。
 * 返回 ESP_OK；参数非法（magnets==0 / window_ms==0）返回 ESP_ERR_INVALID_ARG
 * 且状态不变。trigger 反映"本窗口有脉冲"。
 * 频率/RPM/EMA 由 wheel_chan_sample_period（间隔法）单独维护。
 */
esp_err_t wheel_chan_sample(wheel_chan_state_t *st, const wheel_chan_cfg_t *cfg,
                            uint16_t new_counter, uint32_t now_ms);

/*
 * 换数据源/启停/配置变更后由调方主动重置基准，避免切换瞬间按回绕/复位误判。
 * 不清累计脉冲与 EMA 历史。
 */
void wheel_chan_reset_baseline(wheel_chan_state_t *st, uint16_t current_counter);

/*
 * 脉冲间隔测频法：interval_us 为相邻两次脉冲的时间间隔（微秒）。
 * 频率 = 1e6/interval_us（Hz），RPM = 频率×60/磁铁数，走 EMA。
 * 间隔为 0 返回 ESP_ERR_INVALID_ARG 且状态不变。
 */
esp_err_t wheel_chan_sample_period(wheel_chan_state_t *st, const wheel_chan_cfg_t *cfg,
                                   uint32_t interval_us, uint32_t now_ms);

/* 清零累计脉冲（网页 reset_counts 命令），基准保留。 */
void wheel_chan_reset_counts(wheel_chan_state_t *st);

#ifdef __cplusplus
}
#endif

#endif /* WHEEL_SPEED_COLLECTOR_H */
