/* dev_cfg 实现：见头文件注释。 */
#include "dev_cfg.h"

#include <string.h>

void dev_cfg_defaults(dev_cfg_t *c)
{
    memset(c, 0, sizeof(*c));
    c->diam_x10 = 660;      /* 66.0mm，1/10 攀爬车常见轮径 */
    c->disconnect_ms = 2000;
}

static bool has_ctrl(const char *s)
{
    /* 配置来自 HTTP 表单，拒绝控制字符，别把换行带进 WiFi 驱动和页面 */
    for (; *s != '\0'; s++) {
        if ((unsigned char)*s < 0x20) {
            return true;
        }
    }
    return false;
}

esp_err_t dev_cfg_validate(const dev_cfg_t *c)
{
    if (c == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (c->wifi_ssid[0] == '\0' || strlen(c->wifi_ssid) > DEV_CFG_SSID_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    if (strlen(c->wifi_pass) > DEV_CFG_PASS_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    if (strlen(c->server_uri) > DEV_CFG_URI_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    if (has_ctrl(c->wifi_ssid) || has_ctrl(c->wifi_pass) || has_ctrl(c->server_uri)) {
        return ESP_ERR_INVALID_ARG;
    }
    /* 非空 URI 必须是 ws:// 开头：C3 只走局域网明文 WS */
    if (c->server_uri[0] != '\0' && strncmp(c->server_uri, "ws://", 5) != 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (c->diam_x10 < DEV_CFG_DIAM_X10_MIN || c->diam_x10 > DEV_CFG_DIAM_X10_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    if (c->disconnect_ms < DEV_CFG_DISC_MS_MIN || c->disconnect_ms > DEV_CFG_DISC_MS_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    return ESP_OK;
}

float dev_cfg_diam_mm(const dev_cfg_t *c)
{
    return (float)c->diam_x10 / 10.0f;
}
