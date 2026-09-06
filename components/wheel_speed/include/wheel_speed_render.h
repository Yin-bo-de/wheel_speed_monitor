/*
 * wheel_speed_render：轮速采集器的遥测 JSON 块渲染，纯 C 无 IDF 依赖。
 * 输入采集器状态快照，输出 {"type":"wheel_speed",...} 对象文本（不含外层包裹），
 * 由 telemetry 框架拼进 collectors 数组。字段名/小数位即前端协议，固定不变。
 */
#ifndef WHEEL_SPEED_RENDER_H
#define WHEEL_SPEED_RENDER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define WHEEL_RENDER_COUNT 4

typedef enum {
    WHEEL_RENDER_SRC_PCNT = 0, /* 真实霍尔 PCNT */
    WHEEL_RENDER_SRC_SIM,      /* 模拟源 */
} wheel_render_src_t;

typedef struct {
    bool enabled;        /* 该轮是否启用 */
    float rpm;           /* EMA 滤波后转速 */
    float speed_cm_s;    /* 线速度 cm/s */
    float speed_km_h;    /* 线速度 km/h */
    float freq_hz;       /* 脉冲频率 */
    uint32_t pulses;     /* 累计脉冲 */
    bool trigger;        /* 最近窗口有脉冲 */
} wheel_render_wheel_t;

/* 渲染输入快照（由采集器 sample 后填充） */
typedef struct {
    wheel_render_src_t source;
    uint32_t magnets;
    uint32_t wheel_diam_mm;
    float alpha;
    bool sim_on;
    float sim_rpm[WHEEL_RENDER_COUNT];
    wheel_render_wheel_t wheels[WHEEL_RENDER_COUNT];
} wheel_render_state_t;

/* 渲染 JSON 块；buf 不足返回 ESP_ERR_INVALID_SIZE 且 used 不变。 */
esp_err_t wheel_render_block(const wheel_render_state_t *rs,
                             char *buf, size_t len, size_t *used);

#ifdef __cplusplus
}
#endif

#endif /* WHEEL_SPEED_RENDER_H */
