/*
 * cam_rate：相机网络自适应帧率控制器（纯 C，无 IDF 依赖）。
 *
 * 输入只有"一帧发出去花了多久"；输出是当前有效采集/发送帧率。发送阻塞
 * 说明 TCP 缓冲被下游顶住，按 3/4 退让；连续快速发送一段时间后每 3 秒加
 * 1fps，逐步回到用户配置上限。控制器不直接碰传感器，应用层把 effective
 * 交给 camera_drv 改 CLKRC/VTS。
 */
#ifndef CAM_RATE_H
#define CAM_RATE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CAM_RATE_FPS_MIN 5
#define CAM_RATE_FPS_MAX 25
/* 慢发送阈值随有效帧率变化：约 1.5 个帧间隔，最低 100ms、最高 400ms。 */
#define CAM_RATE_SLOW_MIN_MS 100
#define CAM_RATE_SLOW_MAX_MS 400
#define CAM_RATE_FAST_MS 60
#define CAM_RATE_SLOW_COOLDOWN_MS 2000
#define CAM_RATE_RECOVER_MS 3000
#define CAM_RATE_FAST_STREAK 30

typedef struct {
    uint32_t cap;            /* 用户配置上限 */
    uint32_t effective;      /* 当前生效帧率 */
    uint32_t last_change_ms; /* 上次调整时刻 */
    uint32_t fast_streak;    /* 连续快速发送计数 */
    bool stream_active;
} cam_rate_t;

void cam_rate_init(cam_rate_t *r, uint32_t cap);

/* 用户改配置时调用。流空闲时立即跟随新上限；流进行中只限制不抬高，
 * 抬高交给 recovery，避免网络刚好转时突然冲高。 */
void cam_rate_set_cap(cam_rate_t *r, uint32_t cap);

/* 新流开始：保留上一次的有效帧率（旧流可能是网络差被迫断开的），
 * 重新开始观察；新的相机会话在 deinit 时由 set_cap 恢复用户上限。 */
void cam_rate_stream_start(cam_rate_t *r, uint32_t now_ms);

/* 流结束：只标记停止，保留 effective，宽限期内不再按网络升/降。
 * 需要恢复用户上限时由应用层再调 cam_rate_set_cap(同一个 cap)。 */
void cam_rate_stream_stop(cam_rate_t *r, uint32_t now_ms);

/* 上报一帧的发送耗时（含 TCP 阻塞），返回调整后的 effective。 */
uint32_t cam_rate_report_send(cam_rate_t *r, uint32_t send_ms, uint32_t now_ms);

uint32_t cam_rate_effective(const cam_rate_t *r);
uint32_t cam_rate_cap(const cam_rate_t *r);

#ifdef __cplusplus
}
#endif

#endif /* CAM_RATE_H */
