/*
 * bench_http_server：Web 验证台服务。
 * + GET /            → 内嵌 index.html（EMBED_FILES）
 * + GET /api/status  → 最新遥测帧（兜底轮询）
 * + WS /ws           → 100ms 遥测推送 + 命令接收
 * 命令（set_cfg/set_sim/reset_counts/get_cfg）在 httpd 上下文解析、校验、
 * 应用配置并落 NVS。全部 WS 发送走 ws_push_task 单任务（async_send）。
 */
#ifndef BENCH_HTTP_SERVER_H
#define BENCH_HTTP_SERVER_H

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "wheel_bench.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t bench_http_server_start(const bench_server_ctx_t *ctx);

/* 由 ws_push_task 调用（单任务发送），广播遥测帧到所有 WS 客户端。 */
void bench_http_server_broadcast(const char *frame, size_t len, uint32_t seq);

#ifdef __cplusplus
}
#endif

#endif /* BENCH_HTTP_SERVER_H */
