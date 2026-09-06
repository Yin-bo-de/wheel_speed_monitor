/*
 * wheel_speed_math：纯数值链，无 IDF 依赖。
 * 输入：一个采集窗口内的脉冲增量；输出：频率、RPM、线速度（EMA 滤波后）。
 * 静止归零的"是否静止"判定基于 last_pulse_ms，由调用方保证 now_ms 单调递增。
 */
#ifndef WHEEL_SPEED_MATH_H
#define WHEEL_SPEED_MATH_H

#include <stdbool.h>
#include <stdint.h>

/* 宿主机测试命中 tests/native/include/esp_err.h（shim），IDF 构建命中 esp_common 的 esp_err.h */
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float freq_hz;        /* 本窗口脉冲频率（脉冲数 / 窗口秒数） */
    float rpm_ema;        /* EMA 滤波后的转速（转/分） */
    float speed_cm_s;     /* 线速度（厘米/秒），由 rpm * π * 轮径换算 */
    float speed_km_h;     /* 线速度（公里/小时） */
    uint32_t last_pulse_ms; /* 最近一次有脉冲窗口的 now_ms，用于静止判定 */
    bool has_sample;      /* 是否已有一次有效样本（首次采样直接采纳） */
} wsmath_state_t;

/*
 * 推进一次数学计算。
 * pulses_this_window：本窗口（window_ms 毫秒）内采集到的脉冲增量。
 * magnets：每转磁铁数，必须 >= 1。
 * wheel_diam_mm：轮径（毫米），必须 >= 1。
 * alpha：EMA 滤波系数，0..1，越大越灵敏。
 * now_ms：当前单调毫秒时间，必须随窗口递增。
 * 返回 ESP_ERR_INVALID_ARG 时状态不变；否则状态被推进。
 */
esp_err_t wsmath_update(wsmath_state_t *st, uint32_t pulses_this_window,
                        uint32_t window_ms, uint32_t magnets,
                        uint32_t wheel_diam_mm, float alpha, uint32_t now_ms);

/*
 * 脉冲间隔测频版：interval_us 为相邻两次脉冲间隔（微秒）。
 * 频率 = 1e6/interval_us，EMA 平滑；0 间隔返回 ESP_ERR_INVALID_ARG（不产生 NaN）。
 * 用于低速实时观测：低速（1Hz 级）时窗口法会跳 0/20Hz，间隔法直接反映真实转速。
 */
esp_err_t wsmath_update_period(wsmath_state_t *st, uint32_t interval_us,
                               uint32_t magnets, uint32_t wheel_diam_mm,
                               float alpha, uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* WHEEL_SPEED_MATH_H */
