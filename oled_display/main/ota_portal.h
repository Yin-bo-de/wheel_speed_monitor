/* ota_portal：OTA 升级端点（IDF 依赖，不宿主机测）。
 * 模式照搬车端 main/bench_ota.c：双 OTA 分区 + 回滚，新固件确认可用后
 * 才标记有效，起不来自动回退上一版。C3 没有舵机/相机/日志，静音那些
 * 钩子不搬。升级在 httpd 任务里同步跑，不新增任务。 */
#ifndef OTA_PORTAL_H
#define OTA_PORTAL_H

#include <stdbool.h>

#include "esp_err.h"
#include "esp_http_server.h"

esp_err_t ota_portal_register(httpd_handle_t hd);

/* 新固件启动后 httpd 就绪时调用，确认可用；没确认就崩溃会被回滚 */
void ota_portal_mark_valid(void);

bool ota_portal_busy(void);

#endif /* OTA_PORTAL_H */
