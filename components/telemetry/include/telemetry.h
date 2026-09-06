/*
 * telemetry：车载数据采集框架，纯 C 无 IDF 依赖。
 * 每个采集器是一个自包含对象（内部状态+配置+数据源），实现统一的 ops 接口后
 * 注册进框架；框架负责采样泵、遥测帧拼装，却完全不知道各采集器的内部细节。
 * 框架外新增一种采集器（如 IMU）：在 telem_type_t 末尾加枚举 → 实现 ops →
 * main 注册 → 前端加渲染函数。注册表启动期完成，运行期只读，无锁。
 */
#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 采集器类型：真实类型顺序排列，末尾 AUTO_COUNT 是哨兵。
 * TELEM_TYPE_MOCK_BASE 起预留为宿主机测试的 mock 类型（固件不注册）。 */
typedef enum {
    TELEM_TYPE_WHEEL_SPEED = 0,
    TELEM_TYPE_MOCK_BASE = 100,  /* 宿主机测试用，固件构建不出现 */
    TELEM_TYPE_COUNT,            /* 非哨兵：不在前端渲染，只占位 */
} telem_type_t;

#define TELEM_MAX_COLLECTORS 8

typedef struct {
    telem_type_t type;  /* 唯一标识，也是前端渲染分发的键 */
    esp_err_t (*init)(void *ctx);
    /* 推进内部状态：读硬件 + 计算链。每采样窗口调用一次，now_ms 单调递增。 */
    esp_err_t (*sample)(void *ctx, uint32_t now_ms);
    /* 把本类型最新状态渲染为 JSON 对象文本（不含外层包裹）。
     * used 返回写入字节数；buffer 不足返回 ESP_ERR_INVALID_SIZE。 */
    esp_err_t (*render)(void *ctx, char *buf, size_t len, size_t *used);
    /* 换源/启停/配置变更后重置基准，避免切换瞬间的差值毛刺。 */
    esp_err_t (*reset_baseline)(void *ctx);
} telem_collector_ops_t;

/* 注册采集器。type 重复 → ESP_ERR_INVALID_ARG；注册表满 → ESP_ERR_NO_MEM。 */
esp_err_t telem_register(const telem_collector_ops_t *ops, void *ctx);

/* 依次调用注册表里所有采集器的 sample()；返回第一个非 OK 错误（容错隔离：
 * 单个采集器出错不影响其余遍历）。 */
esp_err_t telem_sample_all(uint32_t now_ms);

/* 渲染完整遥测帧：{"type":"telemetry","seq":..,"t":..,"sys":{..},"collectors":[...]}。
 * seq 由调用方传入（保证帧间序号递增）；free_heap 为 sys 块调试信息（宿主机
 * 测试传 0），uptime_s 由 now_ms 换算。 */
esp_err_t telem_render_frame(char *buf, size_t len, size_t *used,
                             uint32_t seq, uint32_t now_ms, uint32_t free_heap);

/* 通知所有采集器重置基准（换源/启停/配置变更后由上层调用）。 */
esp_err_t telem_reset_all_baselines(void);

/* 喂 sys 块原始 IO 电平（4 路 GPIO，采样任务每窗口调用）。 */
void telem_set_gpio_levels(const uint8_t levels[4]);

/* 宿主机测试专用：清空注册表。固件代码不得调用。 */
void telem_reset_for_test(void);

#ifdef __cplusplus
}
#endif

#endif /* TELEMETRY_H */
