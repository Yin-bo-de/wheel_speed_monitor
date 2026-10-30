/*
 * bench_cam：相机生命周期与发布缓冲（main 内，依赖 IDF + camera_drv）。
 *
 * 唯一的相机句柄持有者是 cam_publish_task：init/deinit/取帧都在这个任务里，
 * httpd 和流任务只发请求、读快照。发布缓冲由短锁保护，网络发送不持锁。
 *
 * 状态机见 docs/CAMERA_DESIGN.md 4.5.1：默认 on_demand，第一个流请求启动；
 * 预览 tab 不可见后保持 30 秒，切回则取消；流断开或快照消费者离开后 30 秒
 * 宽限；always 常开，off 永不启动。
 */
#ifndef BENCH_CAM_H
#define BENCH_CAM_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "config_params.h"
#include "telemetry.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    BENCH_CAM_STREAM_OK = 0,
    BENCH_CAM_STREAM_BUSY, /* 已有客户端 */
    BENCH_CAM_STREAM_OFF,  /* 策略为 off */
} bench_cam_stream_result_t;

typedef enum {
    BENCH_CAM_SNAP_OK = 0,
    BENCH_CAM_SNAP_WARMING, /* 相机启动中或还没出第一帧 */
    BENCH_CAM_SNAP_OFF,     /* 策略为 off */
    BENCH_CAM_SNAP_FAIL,    /* 上次初始化失败；本次已请求重试 */
    BENCH_CAM_SNAP_NO_MEM,  /* 拷贝缓冲分配失败 */
} bench_cam_snap_result_t;

typedef enum {
    BENCH_CAM_STATE_OFF = 0,
    BENCH_CAM_STATE_IDLE,
    BENCH_CAM_STATE_INIT,
    BENCH_CAM_STATE_ON,
    BENCH_CAM_STATE_FAIL,
} bench_cam_state_t;

/* 启动发布任务并应用初始策略。cfg/cfg_mutex 由 main 持有且生命周期贯穿运行期。 */
esp_err_t bench_cam_start(cfg_params_t *cfg, SemaphoreHandle_t cfg_mutex);

/* 流请求入口（httpd 上下文）：抢占唯一流名额、标记预览可见、按需触发 init。 */
bench_cam_stream_result_t bench_cam_stream_acquire(void);
/* 流结束（正常或断开）：释放名额，按需进入 30 秒宽限。 */
void bench_cam_stream_release(void);
/* 流任务每帧前查询：策略关闭、预览超时、改参数重 init、相机失败时返回 true。 */
bool bench_cam_stream_should_stop(void);

/* 把最新发布帧拷到调用方缓冲。没有帧或缓冲不足返回错误，不阻塞。 */
esp_err_t bench_cam_copy_latest(uint8_t *dst, size_t cap, size_t *len, uint32_t *gen);
/* 当前分辨率下 JPEG 缓冲上限，流任务按它分配发送缓冲。 */
size_t bench_cam_max_frame(void);

/* 快照入口（httpd 上下文）：相机 idle 时只触发启动并返回 WARMING，
 * 不在 httpd 里等 init。成功时 *out 由 heap_caps_malloc 分配，调用方负责
 * heap_caps_free。 */
bench_cam_snap_result_t bench_cam_snapshot(uint8_t **out, size_t *len);

/* 预览 tab 可见性（WS cam_preview）。只改运行时状态，不写 NVS。 */
void bench_cam_set_preview(bool active);
/* 最后一个 WS 客户端断开：按预览不可见处理，起 30 秒计时。 */
void bench_cam_on_ws_disconnect(void);

/* sys.cam 快照，ws_push_task 每帧调用。 */
void bench_cam_get_status(telem_cam_status_t *out);

/* 流任务启动等待用：state=on 才发送 MJPEG，fail/off 直接回 503。 */
bench_cam_state_t bench_cam_state(void);

#ifdef __cplusplus
}
#endif

#endif /* BENCH_CAM_H */
