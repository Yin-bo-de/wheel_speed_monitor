/* dev_cfg：设备配置的纯 C 模型（不依赖 IDF，宿主机可测）。
 * NVS 读写在 dev_cfg_store（薄封装），别处不直接调 nvs API。 */
#ifndef DEV_CFG_H
#define DEV_CFG_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#define DEV_CFG_SSID_MAX 32
#define DEV_CFG_PASS_MAX 64
#define DEV_CFG_URI_MAX  64

/* 轮径存 0.1mm 整数（NVS 友好），使用时换算毫米浮点 */
#define DEV_CFG_DIAM_X10_MIN 200u   /* 20.0mm */
#define DEV_CFG_DIAM_X10_MAX 2000u  /* 200.0mm */
#define DEV_CFG_DISC_MS_MIN  500u
#define DEV_CFG_DISC_MS_MAX  10000u

typedef struct {
    char wifi_ssid[DEV_CFG_SSID_MAX + 1];
    char wifi_pass[DEV_CFG_PASS_MAX + 1];
    /* 服务端 WS 地址。空 = 自动：取 DHCP 网关 ws://<gw>/ws（车端固定
     * 192.168.4.1，验证时电脑热点的网关就是跑模拟脚本的电脑）；
     * 端口/路径不同时在配置页面填完整 URI。 */
    char server_uri[DEV_CFG_URI_MAX + 1];
    uint32_t diam_x10;      /* 轮胎直径 0.1mm */
    uint32_t disconnect_ms; /* 断开判定阈值 */
} dev_cfg_t;

/* 几何默认值（数值项）；WiFi 凭据默认值由 dev_cfg_store 从 Kconfig 取 */
void dev_cfg_defaults(dev_cfg_t *c);

/* 逐字段校验（长度/范围/字符集），失败字段的错误码含义见实现 */
esp_err_t dev_cfg_validate(const dev_cfg_t *c);

float dev_cfg_diam_mm(const dev_cfg_t *c);

#endif /* DEV_CFG_H */
