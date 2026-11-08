/*
 * config_store：NVS 薄封装（namespace wheelcfg）。
 * load：无记录 → 默认值并回写；save：冗余写 blob。仅 httpd 上下文调用
 * （不阻塞计算链）。
 */
#ifndef CONFIG_STORE_H
#define CONFIG_STORE_H

#include "config_params.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t config_store_load(cfg_params_t *out);
esp_err_t config_store_save(const cfg_params_t *cfg);

/* 日志子系统的启动序号：每次调用读旧值 +1 并写回（bench_log 用于
 * 未对时期间的文件命名与排序）。 */
uint32_t config_store_next_boot_seq(void);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_STORE_H */
