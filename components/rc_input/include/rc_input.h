/*
 * rc_input：遥控接收机三路通道的判定与仲裁，纯 C 无 IDF 依赖（宿主机可测）。
 *
 * 模式通道决定差速锁听谁：手动 = 把前后差速通道的脉宽原样透传；自动 = 忽略
 * 遥控、走轮速状态机。捕获层（rc_capture）与验证台的模拟注入都往这里喂值，
 * "哪边说了算"这条规则只有这一份实现，两边行为不会走样。
 *
 * 有效性只看两点：收到过、且距今不超过超时。脉宽量程在 feed 处校验，越界值
 * 当场丢弃且不刷新时间戳——坏脉冲不该让链路显得"还活着"。
 *
 * 模式判定与差速透传各自独立判定：每根线单独断都有确定行为，不必约定
 * "三根线都有效才认遥控"这种更脆弱的前提。任一路无效就回落设置页配置。
 */
#ifndef RC_INPUT_H
#define RC_INPUT_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RC_CH_COUNT 3   /* 0=模式 1=前差速 2=后差速 */
#define RC_DIFF_COUNT 2 /* 差速通道在 us[] 与结果里的下标：0=前 1=后 */
#define RC_CH_MODE 0
#define RC_CH_DIFF_FRONT 1
#define RC_CH_DIFF_REAR 2

/* 与舵机链路同一量程（servo_act 的 500..2500）：区间外的"脉冲"只可能是
 * 电气噪声或接收机故障，当无效处理 */
#define RC_US_MIN 500
#define RC_US_MAX 2500

/* 手动/自动分界：规格为 [1500,2000] 手动、[1000,1500) 自动 */
#define RC_MODE_THRESHOLD_US 1500

typedef struct {
    uint16_t us[RC_CH_COUNT];      /* 最近一次收到的合法脉宽 */
    uint32_t last_ms[RC_CH_COUNT]; /* 收到时刻 */
    bool seen[RC_CH_COUNT];        /* 是否收到过：区分"从未收到"与"超时" */
} rc_input_t;

typedef struct {
    bool manual;                        /* 生效模式：true=手动 */
    bool mode_from_rc;                  /* 模式来自 RC 通道（否则来自设置页） */
    uint32_t manual_us[RC_DIFF_COUNT];  /* 每轴生效手动目标脉宽 */
    bool target_from_rc[RC_DIFF_COUNT]; /* 该轴目标来自 RC 透传（否则来自滑杆） */
} rc_resolved_t;

void rc_input_init(rc_input_t *rc);

esp_err_t rc_input_feed(rc_input_t *rc, uint8_t ch, uint16_t us, uint32_t now_ms);

/* 通道有效性：收到过，且距今 ≤ timeout_ms */
bool rc_input_valid(const rc_input_t *rc, uint8_t ch, uint32_t now_ms, uint32_t timeout_ms);

/* 距上次收到的时间（页面显示用"从未收到"与"N ms 前"）；从未收到返回 0 */
uint32_t rc_input_age_ms(const rc_input_t *rc, uint8_t ch, uint32_t now_ms);

/* 仲裁合成：RC 通道有效用 RC，否则用 cfg_*（设置页配置） */
esp_err_t rc_input_resolve(const rc_input_t *rc, uint32_t now_ms, uint32_t timeout_ms,
                           bool cfg_manual, const uint32_t cfg_manual_us[RC_DIFF_COUNT],
                           rc_resolved_t *out);

#ifdef __cplusplus
}
#endif

#endif /* RC_INPUT_H */
