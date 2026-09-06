/*
 * wheel_bench.h：main 内部共享对象（配置、单位数组、锁）。
 * bench_http_server 通过 bench_server_ctx_t 引用，不直接访问 statics。
 */
#ifndef WHEEL_BENCH_H
#define WHEEL_BENCH_H

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "config_params.h"
#include "sim_source.h"
#include "wheel_speed_collector.h"
#include "wheel_speed_render.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 每轮采集器单位：状态链（PCNT 或 sim 源统一入口） + 渲染快照 */
typedef struct {
    wheel_chan_state_t chan;   /* 差值/回绕/EMA */
    sim_source_t sim;          /* 模拟源（sim_on 时启用） */
    wheel_render_state_t disp; /* 渲染快照（ws_push 组装） */
} wheel_unit_t;

typedef struct {
    cfg_params_t *cfg;            /* 当前运行配置（用 cfg_mutex 保护） */
    SemaphoreHandle_t cfg_mutex;  /* 配置与应用互斥 */
    wheel_unit_t *units;          /* WHEEL_COUNT 个单位 */
} bench_server_ctx_t;

/* main 暴露：应用配置到采集器链（httpd 命令处理完调用一次）。
 * cfg 已按校验通过；此函数幂等，可在锁内调用。 */
esp_err_t wheel_bench_apply_config(const cfg_params_t *cfg);

#ifdef __cplusplus
}
#endif

#endif /* WHEEL_BENCH_H */
