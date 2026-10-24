/*
 * rc_capture：遥控接收机 PWM 的 GPIO 双沿中断捕获层（IDF 依赖，不宿主机测）。
 *
 * 为什么不是 RMT：RMT 接收靠 idle 阈值断帧，适配"突发一串 + 长空闲"那种
 * 信号（红外遥控）；而遥控舵机的 PWM 是永不停歇的 50Hz 脉冲串，帧间隙
 * 十几毫秒与脉宽本身同量级，阈值两头不讨好——取小了挂上接收还没等到脉冲
 * 就误判帧尾，取大了帧尾永远断不开；非 DMA 接收的缓冲粒度（24 符号 ≈ 240ms）
 * 也远粗于 50ms 采样节拍。边沿中断直接测"上升沿到下降沿"的差值，没信号时
 * 一次中断都没有，"有没有信号"天然由时间戳年龄给出。
 *
 * 本层只做一件事：把测到的脉宽交给调用方；有效性、模式判定、仲裁全在
 * rc_input（纯 C）里，模拟注入与真实捕获共用同一份规则。
 */
#ifndef RC_CAPTURE_H
#define RC_CAPTURE_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RC_CAPTURE_CH_COUNT 3 /* 0=模式 1=前差速 2=后差速 */

typedef struct {
    int gpio[RC_CAPTURE_CH_COUNT];
    volatile uint32_t rise_us[RC_CAPTURE_CH_COUNT];  /* 上升沿时刻（ISR 写） */
    volatile uint16_t width_us[RC_CAPTURE_CH_COUNT]; /* 最近一次合法脉宽（ISR 写） */
    volatile bool fresh[RC_CAPTURE_CH_COUNT];        /* 有新脉宽未被取走（ISR 写） */
    bool inited;
} rc_capture_t;

/* 配置三个输入脚并挂上双沿中断。内部下拉：接收机没上电时线不悬空。
 * 重复调用安全（幂等）。 */
esp_err_t rc_capture_init(rc_capture_t *cap, const int gpio[RC_CAPTURE_CH_COUNT]);

/* 取走自上次调用以来捕获到的脉宽；没有新值返回 false 且不改 *us。
 * 值域由 ISR 过滤在 RC_US_MIN..RC_US_MAX 内（见 rc_input.h）。 */
bool rc_capture_take(rc_capture_t *cap, uint8_t ch, uint16_t *us);

#ifdef __cplusplus
}
#endif

#endif /* RC_CAPTURE_H */
