/*
 * sim_source：模拟数据源，纯 C 无 IDF 依赖。
 * 与 PCNT 源同构——外部只读单调计数器（uint16，自然回绕），使模拟数据
 * 走与真实霍尔完全相同的采集计算链。误差累积取整保证长时间平均频率
 * 精确等于目标频度，切换目标时不清累积器（只做钳位），无漂移。
 */
#ifndef SIM_SOURCE_H
#define SIM_SOURCE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SIM_SOURCE_CHANNELS 4   /* 四轮独立目标 */

typedef struct {
    float target_rpm[SIM_SOURCE_CHANNELS]; /* 每通道目标转速（转/分） */
    double acc[SIM_SOURCE_CHANNELS];       /* 误差累积器（脉冲当量） */
    uint16_t counter[SIM_SOURCE_CHANNELS]; /* 模拟单调计数器，回绕即 16 位翻转 */
} sim_source_t;

/* 目标 RPM 设 0 即停转（增量恒 0，无除零）。 */
void sim_source_init(sim_source_t *src);

void sim_source_set_target_rpm(sim_source_t *src, uint8_t ch, float rpm);

/*
 * 推进一个窗口：把每通道累积误差折算为整数脉冲增量加进 counter。
 * window_ms 是窗口时长（毫秒），rpm 与 磁铁每转 1 的假设一致：
 * 增量 = rpm / 60 * (window_ms / 1000)。
 */
void sim_source_step(sim_source_t *src, uint32_t window_ms);

/* 读当前计数器（uint16，外部按采集链回绕判定处理差值）。 */
uint16_t sim_source_counter(const sim_source_t *src, uint8_t ch);

#ifdef __cplusplus
}
#endif

#endif /* SIM_SOURCE_H */
