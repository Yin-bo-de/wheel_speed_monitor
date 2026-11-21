/* dev_cfg_store 实现：见头文件注释。 */
#include "dev_cfg_store.h"

#include <string.h>

#include "esp_check.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

static const char *TAG = "devcfg";
static const char *NS = "oledcfg";

esp_err_t dev_cfg_store_load(dev_cfg_t *c)
{
    if (c == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    dev_cfg_defaults(c);

    nvs_handle_t h;
    esp_err_t e = nvs_open(NS, NVS_READONLY, &h);
    if (e == ESP_OK) {
        size_t len = sizeof(c->wifi_ssid);
        if (nvs_get_str(h, "ssid", c->wifi_ssid, &len) != ESP_OK) {
            /* NVS 没存过 = 首次烧录：用编译期默认凭据 */
            snprintf(c->wifi_ssid, sizeof(c->wifi_ssid), "%s",
                     CONFIG_OLED_WIFI_SSID);
        }
        len = sizeof(c->wifi_pass);
        if (nvs_get_str(h, "pass", c->wifi_pass, &len) != ESP_OK) {
            snprintf(c->wifi_pass, sizeof(c->wifi_pass), "%s",
                     CONFIG_OLED_WIFI_PASS);
        }
        len = sizeof(c->server_uri);
        if (nvs_get_str(h, "uri", c->server_uri, &len) != ESP_OK) {
            c->server_uri[0] = '\0'; /* 空 = 自动取网关 */
        }
        (void)nvs_get_u32(h, "diam_x10", &c->diam_x10);
        (void)nvs_get_u32(h, "disc_ms", &c->disconnect_ms);
        nvs_close(h);
    } else {
        snprintf(c->wifi_ssid, sizeof(c->wifi_ssid), "%s",
                 CONFIG_OLED_WIFI_SSID);
        snprintf(c->wifi_pass, sizeof(c->wifi_pass), "%s",
                 CONFIG_OLED_WIFI_PASS);
    }

    /* NVS 里的值也可能是旧版本写坏的：校验兜底，坏值回落默认 */
    if (dev_cfg_validate(c) != ESP_OK) {
        ESP_LOGW(TAG, "NVS 配置不合法，回落默认值");
        dev_cfg_defaults(c);
        snprintf(c->wifi_ssid, sizeof(c->wifi_ssid), "%s",
                 CONFIG_OLED_WIFI_SSID);
        snprintf(c->wifi_pass, sizeof(c->wifi_pass), "%s",
                 CONFIG_OLED_WIFI_PASS);
    }
    return ESP_OK;
}

esp_err_t dev_cfg_store_save(const dev_cfg_t *c)
{
    ESP_RETURN_ON_ERROR(dev_cfg_validate(c), TAG, "validate");
    nvs_handle_t h;
    ESP_RETURN_ON_ERROR(nvs_open(NS, NVS_READWRITE, &h), TAG, "nvs open");
    esp_err_t e = ESP_OK;
    if (e == ESP_OK) {
        e = nvs_set_str(h, "ssid", c->wifi_ssid);
    }
    if (e == ESP_OK) {
        e = nvs_set_str(h, "pass", c->wifi_pass);
    }
    if (e == ESP_OK) {
        e = nvs_set_str(h, "uri", c->server_uri);
    }
    if (e == ESP_OK) {
        e = nvs_set_u32(h, "diam_x10", c->diam_x10);
    }
    if (e == ESP_OK) {
        e = nvs_set_u32(h, "disc_ms", c->disconnect_ms);
    }
    if (e == ESP_OK) {
        e = nvs_commit(h);
    }
    nvs_close(h);
    return e;
}
