/* dev_cfg_store：dev_cfg 的 NVS 薄封装（IDF 依赖，不宿主机测）。
 * NVS 读写只经本模块，别处不直接调 nvs API。 */
#ifndef DEV_CFG_STORE_H
#define DEV_CFG_STORE_H

#include "dev_cfg.h"

/* 默认值 + NVS 覆盖。首次烧录（NVS 无 ssid）用 Kconfig 的默认 WiFi 凭据。 */
esp_err_t dev_cfg_store_load(dev_cfg_t *c);

esp_err_t dev_cfg_store_save(const dev_cfg_t *c);

#endif /* DEV_CFG_STORE_H */
