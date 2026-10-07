/* rc_input 实现：见头文件注释（含判定边界与仲裁规则的设计理由）。 */
#include <string.h>

#include "rc_input.h"

void rc_input_init(rc_input_t *rc)
{
    if (rc == NULL) {
        return;
    }
    memset(rc, 0, sizeof(*rc)); /* 全零即"从未收到"：不靠 last_ms=0 判断 */
}

esp_err_t rc_input_feed(rc_input_t *rc, uint8_t ch, uint16_t us, uint32_t now_ms)
{
    if (rc == NULL || ch >= RC_CH_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    /* 越界脉宽不是"信号还在"的证据：拒收，时间戳保持不动，让通道按正常
     * 超时判丢。若在这里刷新时间戳，一根出故障的接收机就能把链路永远
     * 撑成"活的"，透传给舵机的还是一串噪声值。 */
    if (us < RC_US_MIN || us > RC_US_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    rc->us[ch] = us;
    rc->last_ms[ch] = now_ms;
    rc->seen[ch] = true;
    return ESP_OK;
}

bool rc_input_valid(const rc_input_t *rc, uint8_t ch, uint32_t now_ms, uint32_t timeout_ms)
{
    if (rc == NULL || ch >= RC_CH_COUNT || !rc->seen[ch]) {
        return false;
    }
    return (uint32_t)(now_ms - rc->last_ms[ch]) <= timeout_ms;
}

uint32_t rc_input_age_ms(const rc_input_t *rc, uint8_t ch, uint32_t now_ms)
{
    if (rc == NULL || ch >= RC_CH_COUNT || !rc->seen[ch]) {
        return 0; /* "从未收到"由 seen 表达，这里不给一个巨大的假年龄 */
    }
    return now_ms - rc->last_ms[ch];
}

esp_err_t rc_input_resolve(const rc_input_t *rc, uint32_t now_ms, uint32_t timeout_ms,
                           bool cfg_manual, const uint32_t cfg_manual_us[RC_DIFF_COUNT],
                           rc_resolved_t *out)
{
    if (rc == NULL || cfg_manual_us == NULL || out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* 模式与透传各自独立判定：模式通道掉线不该连累差速通道的透传，
     * 反之亦然。任一路无效就只回落那一路 */
    out->mode_from_rc = rc_input_valid(rc, RC_CH_MODE, now_ms, timeout_ms);
    out->manual = out->mode_from_rc ? (rc->us[RC_CH_MODE] >= RC_MODE_THRESHOLD_US) : cfg_manual;

    for (uint8_t i = 0; i < RC_DIFF_COUNT; i++) {
        uint8_t ch = (uint8_t)(RC_CH_DIFF_FRONT + i);
        bool valid = rc_input_valid(rc, ch, now_ms, timeout_ms);
        out->target_from_rc[i] = valid;
        out->manual_us[i] = valid ? rc->us[ch] : cfg_manual_us[i];
    }
    return ESP_OK;
}
