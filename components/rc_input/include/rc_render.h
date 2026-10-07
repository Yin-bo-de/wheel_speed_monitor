/*
 * rc_render：遥控输入的遥测 JSON 块渲染，纯 C 无 IDF 依赖。
 * 输出 {"type":"rc",...} 对象文本（不含外层包裹），由 telemetry 框架拼进
 * collectors 数组。字段名即前端协议，固定不变。
 *
 * 三路通道的 valid/seen/age 与每轴目标的来源（透传还是回落）都必须传出去：
 * "这根轴到底听谁的"是这套功能唯一的验证面，页面上看不见就没法验。
 * seen 单独报是为了区分"从未收到"（没接线）与"超时"（掉线了）。
 *
 * 只吃基本类型，不引用捕获层的结构体；通道数复用 rc_input 的常量，
 * 避免同一套数量在两处各写一遍。
 */
#ifndef RC_RENDER_H
#define RC_RENDER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "rc_input.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint16_t us;      /* 最近一次脉宽 */
    bool valid;       /* 当前是否有效 */
    bool seen;        /* 是否收到过：false = 从未收到 */
    uint32_t age_ms;  /* 距上次收到的时间；从未收到时为 0 */
} rc_render_chan_t;

typedef struct {
    uint16_t us;  /* 该轴生效目标脉宽 */
    bool from_rc; /* true=遥控透传，false=回落设置页滑杆 */
} rc_render_target_t;

typedef struct {
    const char *src;      /* "gpio" / "sim"：实际在跑的捕获源 */
    const char *mode;     /* 生效模式 "manual" / "auto" */
    const char *mode_src; /* 模式来源 "rc" / "cfg" */
    bool sim;             /* 配置回显：遥控模拟开关 */
    uint32_t sim_us[RC_CH_COUNT];
    rc_render_chan_t ch[RC_CH_COUNT];
    rc_render_target_t target[RC_DIFF_COUNT];
} rc_render_state_t;

/* 渲染 JSON 块；buf 不足返回 ESP_ERR_INVALID_SIZE 且 used 不变。 */
esp_err_t rc_render_block(const rc_render_state_t *rs, char *buf, size_t len, size_t *used);

#ifdef __cplusplus
}
#endif

#endif /* RC_RENDER_H */
