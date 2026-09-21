/*
 * strategy_render：舵机状态的遥测 JSON 块渲染，纯 C 无 IDF 依赖。
 * 输出 {"type":"servo",...} 对象文本（不含外层包裹），由 telemetry 框架
 * 拼进 collectors 数组。字段名/小数位即前端协议，固定不变。
 *
 * phase 字段是验证锁定策略的主要观测手段：锁定期间轮速差被自己的动作抹平，
 * 光看数值看不出状态机在做什么，必须把 idle/locked/probe 三态显式传出去。
 *
 * 刻意只吃基本类型（不引用 servo_act 的类型），渲染层不该反向依赖执行器组件。
 */
#ifndef STRATEGY_RENDER_H
#define STRATEGY_RENDER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "strategy.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool enabled;        /* 通道使能 */
    bool reached;        /* 执行器是否已到达目标 */
    uint16_t target_us;  /* 指令脉宽 */
    float cur_us;        /* 到达脉宽（模拟模型给出过程值；LEDC 无反馈=指令值） */
    strategy_phase_t phase;
    uint32_t hold_ms;    /* 本次锁定的保持时长（退避累积，可见翻倍） */
    float ratio;         /* 左右速差比例 */
    bool slip_fast_left; /* 哪一侧更快 */
} servo_render_chan_t;

typedef struct {
    const char *src;          /* "sim" / "ledc" */
    bool sim;                 /* 模拟舵机开关（配置回显） */
    uint32_t sim_speed_us_s;  /* 模拟行程速率 µs/s（配置回显） */
    strategy_config_t cfg;    /* 两档脉宽与策略参数（配置回显） */
    servo_render_chan_t ch[STRATEGY_SERVO_COUNT];
} servo_render_state_t;

/* 渲染 JSON 块；buf 不足返回 ESP_ERR_INVALID_SIZE 且 used 不变。 */
esp_err_t servo_render_block(const servo_render_state_t *rs,
                             char *buf, size_t len, size_t *used);

/* 阶段名（"idle"/"locked"/"probe"），供渲染与测试共用。 */
const char *strategy_phase_name(strategy_phase_t phase);

#ifdef __cplusplus
}
#endif

#endif /* STRATEGY_RENDER_H */
