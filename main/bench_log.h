/*
 * bench_log：远程日志观测（设计见 docs/LOGGING_DESIGN.md）。
 * vprintf 钩子把 ESP_LOGx 截进 RAM 环形缓冲（实时视图），
 * esp_timer 周期把脏行追加写入 LittleFS（历史视图），按小时轮转。
 * 端点：/api/logs/live | /api/logs/list | /api/logs/file
 */
#ifndef BENCH_LOG_H
#define BENCH_LOG_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "esp_http_server.h"

#ifdef __cplusplus
extern "C" {
#endif

/* app_main 最早期调用：mount LittleFS、boot 计数、清超配额、装钩子、起刷盘
 * 定时器。存储挂载失败时降级为“只留实时视图”（不落盘，仍返回 ESP_OK）；
 * 只有环形缓冲/锁/定时器装不起来才返回错误。任何失败都不挡启动。 */
esp_err_t bench_log_init(void);

/* httpd 起来后注册 3 个 /api/logs/xxx 端点。 */
esp_err_t bench_log_start(httpd_handle_t hd);

/* WS {type:"log_clock", ts:<ms>}：浏览器对时。首次成功会把当前 b 前缀
 * 文件轮转成日期命名文件。 */
void bench_log_clock_sync(int64_t client_epoch_ms);

/* OTA 写 flash 前强制刷盘并暂停落盘；升级失败恢复时调 suspend(false)。 */
void bench_log_flush_now(void);
void bench_log_suspend(bool susp);

#ifdef __cplusplus
}
#endif

#endif /* BENCH_LOG_H */
