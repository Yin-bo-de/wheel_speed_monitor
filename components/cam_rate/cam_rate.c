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
    r->stream_start_ms = 0;
    r->win_start_ms = 0;
    r->win_sum_ms = 0;
    r->win_frames = 0;
    r->last_avg_ms = 0;
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
    r->stream_start_ms = now_ms;
    r->win_start_ms = 0;
    r->win_sum_ms = 0;
    r->win_frames = 0;
    r->last_avg_ms = 0;
    r->last_change_ms = now_ms;
    r->fast_streak = 0;
}

void cam_rate_stream_stop(cam_rate_t *r, uint32_t now_ms)
{
    if (r == NULL) {
        return;
    }
    r->stream_active = false;
    r->win_start_ms = 0;
    r->win_sum_ms = 0;
    r->win_frames = 0;
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

    /* 流刚起来这一段不降速：曝光收敛、浏览器缓冲、TCP 窗口都还没稳，
     * 这时候的发送耗时不能代表网络。也不累积，免得保护期结束后一结算
     * 就被冷启动的长尾拉降。 */
    if (now_ms - r->stream_start_ms < CAM_RATE_STARTUP_MS) {
        return r->effective;
    }

    if (r->win_start_ms == 0) {
        r->win_start_ms = now_ms;
    }
    r->win_sum_ms += send_ms;
    r->win_frames++;

    /* 1 秒结算一次，单帧长尾被窗口平均掉。 */
    if (now_ms - r->win_start_ms < CAM_RATE_WINDOW_MS) {
        return r->effective;
    }

    uint32_t avg_ms = r->win_sum_ms / r->win_frames;
    r->last_avg_ms = avg_ms;
    r->win_start_ms = now_ms;
    r->win_sum_ms = 0;
    r->win_frames = 0;

    uint32_t elapsed = now_ms - r->last_change_ms;

    /* 按当前目标帧率算"这个窗口是否拖住了节奏"：固定 250ms 对 25fps 太迟钝，
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

    if (avg_ms >= slow_ms) {
        r->fast_streak = 0;
        if (elapsed >= CAM_RATE_SLOW_COOLDOWN_MS && r->effective > CAM_RATE_FPS_MIN) {
            uint32_t next = r->effective * 3u / 4u;
            if (avg_ms >= 1000u) {
                /* 整窗阻塞超过 1 秒不是抖动，直接砍半，尽快把发送缓冲放空。 */
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
    } else if (avg_ms <= CAM_RATE_FAST_MS) {
        if (r->fast_streak < CAM_RATE_FAST_WINDOWS) {
            r->fast_streak++;
        }
        if (r->fast_streak >= CAM_RATE_FAST_WINDOWS && elapsed >= CAM_RATE_RECOVER_MS &&
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
