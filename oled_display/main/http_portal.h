/* http_portal：C3 自起的极简配置页（IDF 依赖，不宿主机测）。
 * 承载三件事：设备信息、参数配置（WiFi/轮径/断开阈值）、OTA 升级入口。
 * 页面比车端网页更简单直白。OLED 开机小页面给出 IP，手机开
 * http://<ip>/ 就能进本页。 */
#ifndef HTTP_PORTAL_H
#define HTTP_PORTAL_H

#include "dev_cfg.h"
#include "esp_err.h"

/* 启动 httpd 并注册全部端点；就绪后标记固件有效（回滚确认） */
esp_err_t http_portal_start(const dev_cfg_t *cfg);

#endif /* HTTP_PORTAL_H */
