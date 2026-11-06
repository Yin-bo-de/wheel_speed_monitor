/* cam_rate 实现：见头文件注释。 */
#include "cam_rate.h"

static uint32_t clamp_cap(uint32_t cap)
{
    if (cap < CAM_RATE_FPS_MIN) {
        return CAM_RATE_FPS_MIN;
    }
    if (cap > CAM_RATE_FPS_MAX) {
        return CAM_RATE_FPS_MAX;
    }
    return cap;
}

void cam_rate_init(cam_rate_t *r, uint32_t cap)
{
    if (r == NULL) {
        return;
    }
    r->cap = clamp_cap(cap);
    r->effective = r->cap;
    r->last_change_ms = 0;
    r->fast_streak = 0;
    r->stream_active = false;
}

void cam_rate_set_cap(cam_rate_t *r, uint32_t cap)
{
    if (r == NULL) {
        return;
    }
    r->cap = clamp_cap(cap);
    if (!r->stream_active || r->effective > r->cap) {
        r->effective = r->cap;
    }
}

void cam_rate_stream_start(cam_rate_t *r, uint32_t now_ms)
{
    if (r == NULL) {
        return;
    }
    r->stream_active = true;
    if (r->effective > r->cap) {
        r->effective = r->cap;
    }
    r->last_change_ms = now_ms;
    r->fast_streak = 0;
}

void cam_rate_stream_stop(cam_rate_t *r, uint32_t now_ms)
{
    if (r == NULL) {
        return;
    }
    r->stream_active = false;
    r->last_change_ms = now_ms;
    r->fast_streak = 0;
}

uint32_t cam_rate_report_send(cam_rate_t *r, uint32_t send_ms, uint32_t now_ms)
{
    if (r == NULL) {
        return 0;
    }
    if (!r->stream_active) {
        return r->effective;
    }

    uint32_t elapsed = now_ms - r->last_change_ms;

    /* 按当前目标帧率算"这一帧是否拖住了节奏"：固定 250ms 对 25fps 太迟钝，
     * 对 5fps 又太敏感。取 1.5 个帧间隔，并夹在 100–400ms。 */
    uint32_t eff = r->effective ? r->effective : CAM_RATE_FPS_MIN;
    uint32_t interval = 1000u / eff;
    uint32_t slow_ms = interval + interval / 2u;
    if (slow_ms < CAM_RATE_SLOW_MIN_MS) {
        slow_ms = CAM_RATE_SLOW_MIN_MS;
    }
    if (slow_ms > CAM_RATE_SLOW_MAX_MS) {
        slow_ms = CAM_RATE_SLOW_MAX_MS;
    }

    if (send_ms >= slow_ms) {
        r->fast_streak = 0;
        if (elapsed >= CAM_RATE_SLOW_COOLDOWN_MS && r->effective > CAM_RATE_FPS_MIN) {
            uint32_t next = r->effective * 3u / 4u;
            if (send_ms >= 1000u) {
                /* 阻塞超过 1 秒不是抖动，直接砍半，尽快把发送缓冲放空。 */
                next = r->effective / 2u;
            }
            if (next >= r->effective) {
                next = r->effective - 1u;
            }
            if (next < CAM_RATE_FPS_MIN) {
                next = CAM_RATE_FPS_MIN;
            }
            r->effective = next;
            r->last_change_ms = now_ms;
        }
    } else if (send_ms <= CAM_RATE_FAST_MS) {
        if (r->fast_streak < CAM_RATE_FAST_STREAK) {
            r->fast_streak++;
        }
        if (r->fast_streak >= CAM_RATE_FAST_STREAK && elapsed >= CAM_RATE_RECOVER_MS &&
            r->effective < r->cap) {
            r->effective++;
            r->last_change_ms = now_ms;
            r->fast_streak = 0;
        }
    } else {
        r->fast_streak = 0;
    }

    return r->effective;
}

uint32_t cam_rate_effective(const cam_rate_t *r)
{
    return (r != NULL) ? r->effective : CAM_RATE_FPS_MIN;
}

uint32_t cam_rate_cap(const cam_rate_t *r)
{
    return (r != NULL) ? r->cap : CAM_RATE_FPS_MIN;
}
