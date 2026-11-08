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

    /* 间隔切分合并：相机时钟耦合进霍尔线产生的假脉冲会把一个真实轮周期
     * 切成两段，短的那段单独测频就是几百 RPM 的尖峰。短片段先挂起，
     * 下一个间隔到来时再决定合并还是丢弃。 */
    uint32_t last_valid_interval_us; /* 最近一次确认有效的间隔（合并基准；0=无基准） */
    uint32_t prev_valid_interval_us; /* 上上段有效间隔：长段回滚判定基准；0=无 */
    wsmath_state_t math_snapshot;    /* 上次正常上报前的 math 副本，回滚时恢复 */
    uint32_t pending_interval_us;    /* 挂起中的可疑短间隔；0=无挂起 */
    uint32_t pending_since_ms;       /* 挂起时刻，用于超时丢弃 */
    uint32_t last_seen_interval_us;  /* 最近一次见到的间隔值：正常路径重复轮询
                                        去重用——源在两次边沿间每窗口重复返回
                                        同一值，放行会覆写回滚参照（prev/快照）
                                        导致长段回滚永不触发；0=尚未见过，
                                        随回滚基准一起清零 */
    uint32_t suspect_count;          /* 可疑间隔累计次数（诊断遥测） */
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
 *
 * 间隔切分合并（防相机时钟耦合产生的假脉冲）：
 * - 重复轮询去重：源在两次边沿之间每窗口重复返回同一间隔，正常路径仅首个
 *   见到的值参与状态机；重复值只保鲜 last_pulse_ms 并按 last_valid 继续推进
 *   EMA（维持每窗口一步的收敛速度），不产生新事件、不覆写回滚参照；
 *   挂起值的重复同样不上报；
 * - 无基准首样本不直采：启动/静止恢复后的第一个间隔不可信（使能瞬间
 *   ISR 锁存的可能是假脉冲切出的短段，直采曾让读数一闪几百 RPM），先
 *   挂起（不计可疑、不上报）；第二个不同间隔到来时静默提升为基准，再
 *   按正常流程判定。恒定间隔的模拟源由第二次不同值或停转兜底；
 * - 新间隔短于上次有效间隔的一半 → 先做长段回滚判定，再挂起不报；
 *   suspect_count++，同一间隔的重复轮询不算新事件；
 * - 长段回滚：上一段比上上段短 10% 以上、且上一段与当前短段之和落在
 *   上上段 ±25% 内 → 恢复上一段上报前的 math 快照，按合并值重放后返回；
 * - 回滚不成立则挂起；下一个间隔到来时，挂起值与它之和落在上次有效间隔
 *   ±25% 内 → 合并上报；
 * - 合并不成立 → 丢弃挂起，当前间隔正常上报；
 * - 挂起超过上次有效间隔的 1.5 倍仍无新事件 → 丢弃挂起并清空合并基准，
 *   之后首个间隔按首样本直采（停转由 IDLE_TIMEOUT_LONG_MS 归零）。
 * - 挂起超时、静止归零、换源重置同时清空回滚参照（last_seen、prev 与 math 快照）。
 */
esp_err_t wheel_chan_sample_period(wheel_chan_state_t *st, const wheel_chan_cfg_t *cfg,
                                   uint32_t interval_us, uint32_t now_ms);

/* 清零累计脉冲（网页 reset_counts 命令），基准保留。 */
void wheel_chan_reset_counts(wheel_chan_state_t *st);

#ifdef __cplusplus
}
#endif

#endif /* WHEEL_SPEED_COLLECTOR_H */
