/*
 * wheel_speed_math：纯数值链，无 IDF 依赖。
 * 输入：相邻两次脉冲的间隔（微秒）；输出：频率、RPM、线速度（EMA 滤波后）。
 * 静止归零由上层采集器（wheel_speed_collector）基于 last_pulse_ms 做超时判定，
 * 本层只负责有有效间隔时的 EMA 推进与线速度换算。
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
    float freq_hz;        /* 当前脉冲频率（1e6/间隔us） */
    float rpm_ema;       /* EMA 滤波后的转速（转/分） */
    float speed_cm_s;    /* 线速度（厘米/秒），由 rpm * π * 轮径换算 */
    float speed_km_h;    /* 线速度（公里/小时） */
    uint32_t last_pulse_ms; /* 最近一次有脉冲的 now_ms，供上层静止判定 */
    bool has_sample;      /* 是否已有一次有效样本（首次采样直接采纳） */
} wsmath_state_t;

/*
 * 脉冲间隔测频：interval_us 为相邻两次脉冲间隔（微秒）。
 * 频率 = 1e6/interval_us，RPM = 频率×60/磁铁数，走 EMA。
 * 0 间隔返回 ESP_ERR_INVALID_ARG（不产生 NaN），状态不变。
 * now_ms 为当前单调毫秒时间，用于刷新 last_pulse_ms（供上层静止判定）。
 */
esp_err_t wsmath_update_period(wsmath_state_t *st, uint32_t interval_us,
                               uint32_t magnets, uint32_t wheel_diam_mm,
                               float alpha, uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* WHEEL_SPEED_MATH_H */
