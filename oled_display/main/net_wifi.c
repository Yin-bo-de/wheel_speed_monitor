/* net_wifi 实现：见头文件注释。 */
#include "net_wifi.h"

#include <string.h>

#include "esp_check.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"

static const char *TAG = "netwifi";

static esp_netif_t *s_netif;
static volatile bool s_got_ip;
static char s_ip[16];
static char s_gw[16];

static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data)
{
    (void)arg;
    (void)data;
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        s_got_ip = false;
        ESP_LOGW(TAG, "WiFi 断开，重连中");
        esp_wifi_connect(); /* 无限重试：车上电顺序无关，等热点出现 */
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *ev = (ip_event_got_ip_t *)data;
        snprintf(s_ip, sizeof(s_ip), IPSTR, IP2STR(&ev->ip_info.ip));
        snprintf(s_gw, sizeof(s_gw), IPSTR, IP2STR(&ev->ip_info.gw));
        s_got_ip = true;
        ESP_LOGI(TAG, "got ip %s gw %s", s_ip, s_gw);
    }
}

esp_err_t net_wifi_start(const dev_cfg_t *cfg)
{
    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "netif init");
    ESP_RETURN_ON_ERROR(esp_event_loop_create_default(), TAG, "event loop");
    s_netif = esp_netif_create_default_wifi_sta();
    if (s_netif == NULL) {
        return ESP_ERR_NO_MEM;
    }
    esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();
    if (ap_netif == NULL) {
        return ESP_ERR_NO_MEM;
    }
    /* AP 网段不能沿用默认的 192.168.4.x：STA 常连的车端热点就在
     * 那个网段，撞上后 C3 两个接口同网段、lwIP 路由会乱。 */
    esp_netif_ip_info_t ap_ip = {0};
    ESP_RETURN_ON_ERROR(esp_netif_str_to_ip4(NET_WIFI_AP_IP, &ap_ip.ip), TAG,
                        "ap ip parse");
    ap_ip.netmask.addr = ESP_IP4TOADDR(255, 255, 255, 0);
    ap_ip.gw = ap_ip.ip;
    ESP_RETURN_ON_ERROR(esp_netif_dhcps_stop(ap_netif), TAG, "dhcps stop");
    ESP_RETURN_ON_ERROR(esp_netif_set_ip_info(ap_netif, &ap_ip), TAG, "ap ip");
    ESP_RETURN_ON_ERROR(esp_netif_dhcps_start(ap_netif), TAG, "dhcps start");

    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&init_cfg), TAG, "wifi init");
    ESP_RETURN_ON_ERROR(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                   wifi_event_handler, NULL),
                        TAG, "wifi handler");
    ESP_RETURN_ON_ERROR(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                   wifi_event_handler, NULL),
                        TAG, "ip handler");

    wifi_config_t sta = {0};
    /* strlcpy 截断安全：dev_cfg 允许 32 字节 SSID，wifi_config_t 刚好 32 字节 */
    strlcpy((char *)sta.sta.ssid, cfg->wifi_ssid, sizeof(sta.sta.ssid));
    strlcpy((char *)sta.sta.password, cfg->wifi_pass, sizeof(sta.sta.password));
    sta.sta.threshold.authmode = WIFI_AUTH_OPEN; /* 开放热点也接受 */
    wifi_config_t ap = {0};
    strlcpy((char *)ap.ap.ssid, NET_WIFI_AP_SSID, sizeof(ap.ap.ssid));
    ap.ap.ssid_len = strlen(NET_WIFI_AP_SSID);
    ap.ap.channel = 1; /* APSTA 下 AP 信道跟随 STA，这只是初始值 */
    ap.ap.authmode = WIFI_AUTH_OPEN;
    ap.ap.max_connection = 4;

    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_APSTA), TAG, "mode");
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_STA, &sta), TAG, "sta config");
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_AP, &ap), TAG, "ap config");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "start");

    ESP_LOGI(TAG, "配置热点 \"%s\" 已开启，手机直连后打开 http://%s/",
             NET_WIFI_AP_SSID, NET_WIFI_AP_IP);
    ESP_LOGI(TAG, "connecting to \"%s\"", cfg->wifi_ssid);
    return ESP_OK;
}

bool net_wifi_got_ip(void)
{
    return s_got_ip;
}

void net_wifi_ip(char *buf, size_t cap)
{
    snprintf(buf, cap, "%s", s_got_ip ? s_ip : "");
}

void net_wifi_gw(char *buf, size_t cap)
{
    snprintf(buf, cap, "%s", s_got_ip ? s_gw : "");
}
