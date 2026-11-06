/*
 * bench_ota：无线固件升级。
 *
 * 上传在 httpd 任务里同步执行：接收 → esp_ota_write → 校验 → 切分区 → 重启。
 * 升级期间相机与遥测暂停，采样和舵机继续跑。
 */
#ifndef BENCH_OTA_H
#define BENCH_OTA_H

#include <stdbool.h>

#include "esp_err.h"
#include "esp_http_server.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 注册 /api/ota/info、/api/ota/upload、/api/ota/rollback。 */
esp_err_t bench_ota_start(httpd_handle_t hd);

/* 本次启动的固件可用：取消 bootloader 的待验证状态。 */
void bench_ota_mark_valid(void);

/* 升级进行中：遥测推送据此跳过广播，避免和上传抢 httpd。 */
bool bench_ota_busy(void);

#ifdef __cplusplus
}
#endif

#endif /* BENCH_OTA_H */
