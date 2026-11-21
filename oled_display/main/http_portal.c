/* http_portal 实现：见头文件注释。 */
#include "http_portal.h"

#include <stdlib.h>
#include <string.h>

#include "cJSON.h"
#include "dev_cfg_store.h"
#include "esp_app_desc.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "feed_ws.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "net_wifi.h"
#include "ota_portal.h"

static const char *TAG = "oledweb";

/* EMBED_FILES 嵌入的页面（链接期符号，名字带文件名） */
extern const uint8_t page_html_start[] asm("_binary_page_html_start");
extern const uint8_t page_html_end[]   asm("_binary_page_html_end");

static dev_cfg_t s_cfg;

static esp_err_t send_json(httpd_req_t *req, cJSON *root)
{
    char *json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (json == NULL) {
        return ESP_ERR_NO_MEM;
    }
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    esp_err_t e = httpd_resp_send(req, json, strlen(json));
    cJSON_free(json);
    return e;
}

static esp_err_t send_error(httpd_req_t *req, const char *status, const char *msg)
{
    cJSON *root = cJSON_CreateObject();
    if (root == NULL) {
        return ESP_ERR_NO_MEM;
    }
    cJSON_AddBoolToObject(root, "ok", false);
    cJSON_AddStringToObject(root, "error", msg);
    httpd_resp_set_status(req, status);
    return send_json(req, root);
}

static void cfg_to_json(cJSON *root, const dev_cfg_t *c)
{
    cJSON_AddStringToObject(root, "ssid", c->wifi_ssid);
    cJSON_AddStringToObject(root, "pass", c->wifi_pass);
    cJSON_AddStringToObject(root, "server_uri", c->server_uri);
    cJSON_AddNumberToObject(root, "diam_mm", (double)dev_cfg_diam_mm(c));
    cJSON_AddNumberToObject(root, "disconnect_ms", (double)c->disconnect_ms);
}

static esp_err_t handle_page(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    return httpd_resp_send(req, (const char *)page_html_start,
                           page_html_end - page_html_start);
}

static esp_err_t handle_info(httpd_req_t *req)
{
    const esp_app_desc_t *desc = esp_app_get_description();
    char ip[16];
    char gw[16];
    net_wifi_ip(ip, sizeof(ip));
    net_wifi_gw(gw, sizeof(gw));

    wifi_ap_record_t ap = {0};
    int8_t rssi = 0;
    if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
        rssi = ap.rssi;
    }

    feed_state_t st;
    feed_ws_snapshot(&st);
    uint32_t now = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    long frame_age = -1;
    if (st.last_frame_ms != 0) {
        frame_age = (long)(now - st.last_frame_ms);
    }

    cJSON *root = cJSON_CreateObject();
    if (root == NULL) {
        return ESP_ERR_NO_MEM;
    }
    cJSON_AddBoolToObject(root, "ok", true);
    cJSON_AddStringToObject(root, "version", desc->version);
    cJSON_AddStringToObject(root, "ip", ip);
    cJSON_AddStringToObject(root, "gw", gw);
    cJSON_AddNumberToObject(root, "rssi", rssi);
    cJSON_AddBoolToObject(root, "ws_connected", st.ws_connected);
    cJSON_AddNumberToObject(root, "frame_age_ms", frame_age);
    cJSON *cfg = cJSON_CreateObject();
    if (cfg != NULL) {
        cfg_to_json(cfg, &s_cfg);
        cJSON_AddItemToObject(root, "cfg", cfg);
    }
    return send_json(req, root);
}

static esp_err_t handle_cfg_get(httpd_req_t *req)
{
    cJSON *root = cJSON_CreateObject();
    if (root == NULL) {
        return ESP_ERR_NO_MEM;
    }
    cJSON_AddBoolToObject(root, "ok", true);
    cJSON *cfg = cJSON_CreateObject();
    if (cfg == NULL) {
        cJSON_Delete(root);
        return ESP_ERR_NO_MEM;
    }
    cfg_to_json(cfg, &s_cfg);
    cJSON_AddItemToObject(root, "cfg", cfg);
    return send_json(req, root);
}

static esp_err_t handle_cfg_set(httpd_req_t *req)
{
    char body[512];
    int total = httpd_req_recv(req, body, sizeof(body) - 1);
    if (total <= 0) {
        return send_error(req, "400 Bad Request", "body required");
    }
    body[total] = '\0';

    cJSON *root = cJSON_Parse(body);
    if (root == NULL) {
        return send_error(req, "400 Bad Request", "bad json");
    }

    dev_cfg_t next = s_cfg;
    cJSON *j;
    if ((j = cJSON_GetObjectItemCaseSensitive(root, "ssid")) != NULL &&
        cJSON_IsString(j)) {
        snprintf(next.wifi_ssid, sizeof(next.wifi_ssid), "%s", j->valuestring);
    }
    if ((j = cJSON_GetObjectItemCaseSensitive(root, "pass")) != NULL &&
        cJSON_IsString(j)) {
        snprintf(next.wifi_pass, sizeof(next.wifi_pass), "%s", j->valuestring);
    }
    if ((j = cJSON_GetObjectItemCaseSensitive(root, "server_uri")) != NULL &&
        cJSON_IsString(j)) {
        snprintf(next.server_uri, sizeof(next.server_uri), "%s", j->valuestring);
    }
    if ((j = cJSON_GetObjectItemCaseSensitive(root, "diam_mm")) != NULL &&
        cJSON_IsNumber(j)) {
        next.diam_x10 = (uint32_t)(j->valuedouble * 10.0 + 0.5);
    }
    if ((j = cJSON_GetObjectItemCaseSensitive(root, "disconnect_ms")) != NULL &&
        cJSON_IsNumber(j)) {
        next.disconnect_ms = (uint32_t)j->valuedouble;
    }
    cJSON_Delete(root);

    if (dev_cfg_validate(&next) != ESP_OK) {
        return send_error(req, "400 Bad Request", "config invalid");
    }
    esp_err_t e = dev_cfg_store_save(&next);
    if (e != ESP_OK) {
        return send_error(req, "500 Internal Server Error", "nvs save failed");
    }
    s_cfg = next;

    cJSON *resp = cJSON_CreateObject();
    if (resp == NULL) {
        return ESP_ERR_NO_MEM;
    }
    cJSON_AddBoolToObject(resp, "ok", true);
    cJSON_AddNumberToObject(resp, "restart_in_ms", 2000);
    e = send_json(req, resp);
    if (e != ESP_OK) {
        ESP_LOGW(TAG, "cfg response failed (%s), restart anyway", esp_err_to_name(e));
    }
    /* WiFi 凭据变更只有重启才干净生效，配置保存后一律重启 */
    vTaskDelay(pdMS_TO_TICKS(2000));
    esp_restart();
    return ESP_OK;
}

esp_err_t http_portal_start(const dev_cfg_t *cfg)
{
    if (cfg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    s_cfg = *cfg;

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    /* 端点：页面 1 + 配置 2 + OTA 3 = 6，默认槽位 8，显式钉住防默认值漂移 */
    config.max_uri_handlers = 8;
    httpd_handle_t hd = NULL;
    ESP_RETURN_ON_ERROR(httpd_start(&hd, &config), TAG, "httpd start");

    httpd_uri_t uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = handle_page,
        .user_ctx = NULL,
    };
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(hd, &uri), TAG, "page uri");
    uri.uri = "/api/info";
    uri.handler = handle_info;
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(hd, &uri), TAG, "info uri");
    uri.uri = "/api/cfg";
    uri.handler = handle_cfg_get;
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(hd, &uri), TAG, "cfg get uri");
    uri.method = HTTP_POST;
    uri.handler = handle_cfg_set;
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(hd, &uri), TAG, "cfg set uri");
    ESP_RETURN_ON_ERROR(ota_portal_register(hd), TAG, "ota uris");

    /* httpd 就绪 = 新固件可用，确认后 bootloader 不再回滚 */
    ota_portal_mark_valid();
    return ESP_OK;
}
