/*
 * bench_cam_stream：MJPEG 流与单帧快照的 HTTP 端点。
 *
 * httpd 是单任务，MJPEG 的发送循环不能跑在 handler 里；/cam/stream 用
 * httpd_req_async_handler_begin() 把请求交给 cam_stream_task。快照只做一次
 * 拷贝和一次发送，短小，留在 httpd 上下文。
 */
#ifndef BENCH_CAM_STREAM_H
#define BENCH_CAM_STREAM_H

#include "esp_err.h"
#include "esp_http_server.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 注册 /cam/stream 与 /cam/snapshot，并启动唯一的流发送任务。 */
esp_err_t bench_cam_stream_start(httpd_handle_t hd);

#ifdef __cplusplus
}
#endif

#endif /* BENCH_CAM_STREAM_H */
