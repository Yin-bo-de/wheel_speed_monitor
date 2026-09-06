/*
 * config_params：车载运行参数的纯 C 模型，宿主机可测（无 IDF 依赖）。
 * 配置存储由 main 的 config_store 负责（NVS 薄封装），这里只做：
 * 默认值、逐字段校验、NVS blob 打包/解包。版本号字段留给 v2 迁移。
 */
#ifndef CONFIG_PARAMS_H
#define CONFIG_PARAMS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CFG_WHEEL_COUNT 4

#define CFG_MAGNETS_MIN 1
#define CFG_MAGNETS_MAX 16
#define CFG_DIAM_MM_MIN 10
#define CFG_DIAM_MM_MAX 300
#define CFG_ALPHA_MIN 0.05f
#define CFG_ALPHA_MAX 1.0f

/* blob 定长布局：版本(1B) + magnets(1B) + diam(4B) + alpha(4B float)
 * + wheel_enabled(1B 位图) + sim_on(1B) + sim_rpm[4](16B) + debounce_ms(4B) */
#define CFG_PARAMS_BLOB_VERSION 1
#define CFG_PARAMS_BLOB_SIZE 32

typedef struct {
    uint32_t magnets;              /* 每转磁铁数 1-16 */
    uint32_t wheel_diam_mm;        /* 轮径（毫米）10-300 */
    float alpha;                   /* EMA 系数 0.05-1.0 */
    bool wheel_enabled[CFG_WHEEL_COUNT];
    bool sim_on;                   /* 模拟模式总开关 */
    float sim_rpm[CFG_WHEEL_COUNT];
    uint32_t debounce_ms;          /* 最小脉冲间隔去抖；0=关闭 */
} cfg_params_t;

void cfg_params_default(cfg_params_t *cfg);

/* 校验单字段；字段名与 JSON 命令一致（magnets/diam/alpha/en/sim/sim_rpm/debounce）。
 * 未知字段返回 ESP_OK（命令层忽略）。 */
esp_err_t cfg_params_validate(const cfg_params_t *cfg, const char *field);

esp_err_t cfg_params_pack(const cfg_params_t *cfg, uint8_t *buf, size_t len);
esp_err_t cfg_params_unpack(const uint8_t *buf, size_t len, cfg_params_t *out);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_PARAMS_H */
