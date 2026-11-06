/*
 * cam_rate：相机网络自适应帧率控制器（纯 C，无 IDF 依赖）。
 *
 * 输入是每帧发送耗时，按 1 秒窗口取平均后再判断，单帧长尾不再触发降速。
 * 流启动后前 5 秒只观察不降速，给 AEC/AGC 收敛和 TCP 建窗留时间。
 * 窗口平均阻塞说明 TCP 缓冲被下游顶住，按 3/4 退让；连续多个快窗口后
 * 每 3 秒加 1fps，逐步回到用户配置上限。控制器不直接碰传感器，应用层把
 * effective 交给 camera_drv 改 CLKRC/VTS。
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
/* 流启动后先看 5 秒：相机曝光收敛、浏览器建 TCP 窗口都在这一段。 */
#define CAM_RATE_STARTUP_MS 5000
/* 发送耗时按 1 秒窗口结算，窗口内的单帧长尾被平均掉。 */
#define CAM_RATE_WINDOW_MS 1000
/* 连续多少个快窗口才允许恢复 1fps。 */
#define CAM_RATE_FAST_WINDOWS 3

typedef struct {
    uint32_t cap;            /* 用户配置上限 */
    uint32_t effective;      /* 当前生效帧率 */
    uint32_t last_change_ms; /* 上次调整时刻 */
    uint32_t fast_streak;    /* 连续快速窗口计数 */
    uint32_t stream_start_ms;/* 本次流开始时刻 */
    uint32_t win_start_ms;   /* 当前统计窗口起点 */
    uint32_t win_sum_ms;     /* 窗口内发送耗时总和 */
    uint32_t win_frames;     /* 窗口内帧数 */
    uint32_t last_avg_ms;    /* 最近一个结算窗口的平均发送耗时 */
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
