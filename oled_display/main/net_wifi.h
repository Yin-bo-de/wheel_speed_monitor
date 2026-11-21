/* net_wifi：STA 联网 + 常开配置热点（IDF 依赖，不宿主机测）。
 * 开机按配置连热点，掉线自动重连。服务端地址自动取 DHCP 网关
 * （车端 AP 的网关就是车端 S3 192.168.4.1；电脑热点的网关就是电脑）。
 * 同时 APSTA 并行常开一个开放配置热点：手机直连就能进配置页和
 * OTA 页，不用查 DHCP 地址；凭据配错时也有路自救，不用插 USB。 */
#ifndef NET_WIFI_H
#define NET_WIFI_H

#include <stdbool.h>
#include <stddef.h>

#include "dev_cfg.h"
#include "esp_err.h"

/* 出厂固定的配置热点标识。地址选 192.168.7.x：STA 常连的车端热点
 * 占着 192.168.4.x，AP 撞同网段会让 C3 两个接口路由乱套。 */
#define NET_WIFI_AP_SSID "oled-setup"
#define NET_WIFI_AP_IP "192.168.7.1"

esp_err_t net_wifi_start(const dev_cfg_t *cfg);

bool net_wifi_got_ip(void);

/* 当前 STA IP / 网关字符串；没拿到 IP 时写空串 */
void net_wifi_ip(char *buf, size_t cap);
void net_wifi_gw(char *buf, size_t cap);

#endif /* NET_WIFI_H */
