/*
 * sim_source：模拟数据源，纯 C 无 IDF 依赖。
 * 与 PCNT 源同构——外部只读单调计数器（uint16，自然回绕），使模拟数据
 * 走与真实霍尔完全相同的采集计算链。误差累积取整保证长时间平均频率
 * 精确等于目标频度，切换目标时不清累积器（只做钳位），无漂移。
 * 另提供脉冲间隔（微秒），由虚拟时间跨窗口累计，低速（1 RPS 级）
 * 也如实反映周期，供间隔法测频使用。
 */
#ifndef SIM_SOURCE_H
#define SIM_SOURCE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SIM_SOURCE_CHANNELS 4   /* 四轮独立目标 */

typedef struct {
    float target_rpm[SIM_SOURCE_CHANNELS]; /* 每通道目标转速（转/分） */
    double acc[SIM_SOURCE_CHANNELS];       /* 误差累积器（脉冲当量） */
    uint16_t counter[SIM_SOURCE_CHANNELS]; /* 模拟单调计数器，回绕即 16 位翻转 */
    uint32_t vtime_us;                       /* 虚拟时间（微秒），每次 step 递增 */
    uint32_t last_pulse_vtime[SIM_SOURCE_CHANNELS]; /* 上次脉冲的虚拟时刻 */
    uint32_t last_period_us[SIM_SOURCE_CHANNELS];  /* 最近一次脉冲间隔 */
    bool has_pulse[SIM_SOURCE_CHANNELS];             /* 是否曾 emit 过脉冲 */
} sim_source_t;

/* 目标 RPM 设 0 即停转（增量恒 0，无除零）。 */
void sim_source_init(sim_source_t *src);

void sim_source_set_target_rpm(sim_source_t *src, uint8_t ch, float rpm);

/*
 * 推进一个窗口：把每通道累积误差折算为整数脉冲增量加进 counter。
 * window_ms 是窗口时长（毫秒），rpm 与 磁铁每转 1 的假设一致：
 * 增量 = rpm / 60 * (window_ms / 1000)。
 * 若本窗口有脉冲 emit，按"上次脉冲到当前窗口末尾"的跨度折算间隔，
 * 保证低速跨多窗口才出一个脉冲时仍如实反映真实周期。
 */
void sim_source_step(sim_source_t *src, uint32_t window_ms);

/* 读当前计数器（uint16，外部按采集链回绕判定处理差值）。 */
uint16_t sim_source_counter(const sim_source_t *src, uint8_t ch);

/*
 * 读最近一次脉冲间隔（微秒）；0 表示尚未有脉冲或已过期（距上次脉冲
 * 超过上次间隔×1.5）。语义与 wheel_sensor_get_period_us 对齐，
 * 使模拟源与真实源走同一间隔法采集链。
 */
uint32_t sim_source_period_us(const sim_source_t *src, uint8_t ch);

#ifdef __cplusplus
}
#endif

#endif /* SIM_SOURCE_H */
