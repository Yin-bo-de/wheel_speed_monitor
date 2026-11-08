/* bench_ota 实现：见头文件与 docs/superpowers/specs/2026-11-06-ota-upgrade-design.md。 */
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "bench_ota.h"

#include "bench_cam.h"
#include "bench_log.h"
#include "cJSON.h"
#include "esp_app_desc.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "benchota";

/* 单任务（httpd）写、单任务（ws_push）读，32 位对齐访问天然原子。 */
static volatile bool s_ota_busy;

/* 4KB 静态接收缓冲：httpd 栈只有 8KB，不能放栈上。 */
static uint8_t s_ota_buf[4096];

/* 客户端卡住时连续超时的上限，避免 handler 永远不返回。 */
#define OTA_RECV_TIMEOUT_MAX 10
/* 整轮上传的绝对上限：只看连续超时挡不住每隔几秒发一个字节的客户端，
 * 那种连接会让 OTA 状态和相机停摆被无限期占住。 */
#define OTA_UPLOAD_TIMEOUT_MS 120000

static const char *part_name(const esp_partition_t *p)
{
    if (p == NULL) {
        return "none";
    }
    switch (p->subtype) {
    case ESP_PARTITION_SUBTYPE_APP_FACTORY:
        return "factory";
    case ESP_PARTITION_SUBTYPE_APP_OTA_0:
        return "ota_0";
    case ESP_PARTITION_SUBTYPE_APP_OTA_1:
        return "ota_1";
    default:
        return "unknown";
    }
}

static const char *ota_state_name(esp_ota_img_states_t st)
{
    switch (st) {
    case ESP_OTA_IMG_NEW:
        return "new";
    case ESP_OTA_IMG_PENDING_VERIFY:
        return "pending";
    case ESP_OTA_IMG_VALID:
        return "valid";
    case ESP_OTA_IMG_INVALID:
        return "invalid";
    case ESP_OTA_IMG_ABORTED:
        return "aborted";
    default:
        return "undefined";
    }
}

/* app_desc 里的字符串是定长字段，不保证以 \0 结尾。 */
static void copy_field(char *dst, size_t cap, const char *src, size_t src_len)
{
    size_t n = strnlen(src, src_len);
    if (n >= cap) {
        n = cap - 1;
    }
    memcpy(dst, src, n);
    dst[n] = '\0';
}

static bool read_part_desc(const esp_partition_t *p, esp_app_desc_t *desc)
{
    return p != NULL && esp_ota_get_partition_description(p, desc) == ESP_OK;
}

static esp_err_t send_error(httpd_req_t *req, const char *status, const char *msg)
{
    cJSON *root = cJSON_CreateObject();
    if (root == NULL) {
        return ESP_ERR_NO_MEM;
    }
    cJSON_AddBoolToObject(root, "ok", false);
    cJSON_AddStringToObject(root, "error", msg);
    char *json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (json == NULL) {
        return ESP_ERR_NO_MEM;
    }
    httpd_resp_set_status(req, status);
    httpd_resp_set_type(req, "application/json");
    esp_err_t e = httpd_resp_send(req, json, strlen(json));
    cJSON_free(json);
    return e;
}

static esp_err_t send_ok_restart(httpd_req_t *req, const char *version, size_t size)
{
    cJSON *root = cJSON_CreateObject();
    if (root == NULL) {
        return ESP_ERR_NO_MEM;
    }
    cJSON_AddBoolToObject(root, "ok", true);
    cJSON_AddStringToObject(root, "version", version);
    cJSON_AddNumberToObject(root, "size", (double)size);
    cJSON_AddNumberToObject(root, "restart_in_ms", 2000);
    char *json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (json == NULL) {
        return ESP_ERR_NO_MEM;
    }
    httpd_resp_set_type(req, "application/json");
    esp_err_t e = httpd_resp_send(req, json, strlen(json));
    cJSON_free(json);
    return e;
}

static bool read_content_length(httpd_req_t *req, size_t *out)
{
    char buf[16];
    if (httpd_req_get_hdr_value_str(req, "Content-Length", buf, sizeof(buf)) != ESP_OK) {
        return false;
    }
    char *end = NULL;
    unsigned long v = strtoul(buf, &end, 10);
    if (end == buf || *end != '\0' || v == 0) {
        return false;
    }
    *out = (size_t)v;
    return true;
}

static esp_err_t handle_info(httpd_req_t *req)
{
    const esp_partition_t *run = esp_ota_get_running_partition();
    const esp_partition_t *other = esp_ota_get_next_update_partition(NULL);
    esp_app_desc_t run_desc = {0};
    esp_app_desc_t other_desc = {0};
    bool run_ok = read_part_desc(run, &run_desc);
    bool other_ok = read_part_desc(other, &other_desc);
    esp_ota_img_states_t other_state = ESP_OTA_IMG_UNDEFINED;
    if (other_ok) {
        (void)esp_ota_get_state_partition(other, &other_state);
    }

    char running[33] = "";
    char version[33] = "";
    char built[36] = "";
    char project[33] = "";
    char other_version[33] = "";
    if (run_ok) {
        copy_field(version, sizeof(version), run_desc.version, sizeof(run_desc.version));
        copy_field(project, sizeof(project), run_desc.project_name, sizeof(run_desc.project_name));
        char date[17];
        char time[17];
        copy_field(date, sizeof(date), run_desc.date, sizeof(run_desc.date));
        copy_field(time, sizeof(time), run_desc.time, sizeof(run_desc.time));
        snprintf(built, sizeof(built), "%s %s", date, time);
    }
    if (other_ok) {
        copy_field(other_version, sizeof(other_version), other_desc.version,
                   sizeof(other_desc.version));
    }
    snprintf(running, sizeof(running), "%s", part_name(run));

    cJSON *root = cJSON_CreateObject();
    if (root == NULL) {
        return ESP_ERR_NO_MEM;
    }
    cJSON_AddStringToObject(root, "running", running);
    cJSON_AddStringToObject(root, "version", version);
    cJSON_AddStringToObject(root, "built", built);
    cJSON_AddStringToObject(root, "project", project);
    cJSON_AddNumberToObject(root, "partition_size", run ? (double)run->size : 0);
    cJSON_AddStringToObject(root, "other", part_name(other));
    cJSON_AddStringToObject(root, "other_version", other_version);
    cJSON_AddStringToObject(root, "other_state",
                            other_ok ? ota_state_name(other_state) : "invalid");
    cJSON_AddBoolToObject(root, "busy", s_ota_busy);
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

static esp_err_t handle_upload(httpd_req_t *req)
{
    if (s_ota_busy) {
        return send_error(req, "409 Conflict", "ota already in progress");
    }
    size_t content_len = 0;
    if (!read_content_length(req, &content_len)) {
        return send_error(req, "400 Bad Request", "content-length required");
    }
    const esp_partition_t *part = esp_ota_get_next_update_partition(NULL);
    if (part == NULL) {
        return send_error(req, "500 Internal Server Error", "no ota partition");
    }
    if (content_len > part->size) {
        return send_error(req, "413 Payload Too Large", "firmware exceeds partition size");
    }

    s_ota_busy = true;
    bench_log_flush_now(); /* 崩溃/重启前的日志都落盘，再停写避开与 OTA 写交错 */
    bench_log_suspend(true);
    bench_cam_suspend_for_ota();

    esp_ota_handle_t handle = 0;
    esp_err_t err = esp_ota_begin(part, OTA_WITH_SEQUENTIAL_WRITES, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ota_begin failed: %s", esp_err_to_name(err));
        bench_cam_resume_after_ota();
        bench_log_suspend(false);
        s_ota_busy = false;
        return send_error(req, "500 Internal Server Error", "ota begin failed");
    }

    size_t received = 0;
    int timeouts = 0;
    const char *fail_msg = NULL;
    TickType_t started = xTaskGetTickCount();
    while (received < content_len) {
        if ((xTaskGetTickCount() - started) > pdMS_TO_TICKS(OTA_UPLOAD_TIMEOUT_MS)) {
            fail_msg = "upload timeout";
            break;
        }
        int r = httpd_req_recv(req, (char *)s_ota_buf, sizeof(s_ota_buf));
        if (r == HTTPD_SOCK_ERR_TIMEOUT) {
            if (++timeouts > OTA_RECV_TIMEOUT_MAX) {
                fail_msg = "receive timeout";
                break;
            }
            continue;
        }
        if (r <= 0) {
            fail_msg = "receive failed";
            break;
        }
        timeouts = 0;
        err = esp_ota_write(handle, s_ota_buf, (size_t)r);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "ota_write failed: %s", esp_err_to_name(err));
            fail_msg = "flash write failed";
            break;
        }
        received += (size_t)r;
    }
    if (fail_msg == NULL && received != content_len) {
        fail_msg = "incomplete upload";
    }

    char new_version[33] = "";
    esp_app_desc_t new_desc = {0};
    if (fail_msg == NULL) {
        err = esp_ota_end(handle);
        handle = 0; /* end 之后无论成败句柄都失效，不能再 abort */
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "ota_end failed: %s", esp_err_to_name(err));
            fail_msg = "image checksum failed";
        }
    }
    if (fail_msg == NULL) {
        err = esp_ota_set_boot_partition(part);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "set_boot_partition failed: %s", esp_err_to_name(err));
            fail_msg = "switch boot partition failed";
        }
    }
    if (fail_msg == NULL && read_part_desc(part, &new_desc)) {
        copy_field(new_version, sizeof(new_version), new_desc.version,
                   sizeof(new_desc.version));
    }

    if (fail_msg != NULL) {
        if (handle != 0) {
            (void)esp_ota_abort(handle);
        }
        bench_cam_resume_after_ota();
        bench_log_suspend(false);
        s_ota_busy = false;
        return send_error(req, "500 Internal Server Error", fail_msg);
    }

    esp_err_t e = send_ok_restart(req, new_version, received);
    if (e != ESP_OK) {
        /* 启动分区已经切过去了，重启是唯一一致的动作。不重启会让页面
         * 以为失败，可设备下次重启却进了新固件。 */
        ESP_LOGW(TAG, "ota response failed (%s), restart anyway", esp_err_to_name(e));
    }
    ESP_LOGI(TAG, "ota success: %lu bytes -> %s (%s), restarting",
             (unsigned long)received, part_name(part), new_version);
    /* 升级写入已结束，恢复日志落盘并把升级结果刷进 storage；否则停机前的
     * 这段日志只留在 RAM 里，重启就没了。 */
    bench_log_suspend(false);
    bench_log_flush_now();
    vTaskDelay(pdMS_TO_TICKS(2000));
    esp_restart();
    return ESP_OK;
}

static esp_err_t handle_rollback(httpd_req_t *req)
{
    if (s_ota_busy) {
        return send_error(req, "409 Conflict", "ota in progress");
    }
    const esp_partition_t *other = esp_ota_get_next_update_partition(NULL);
    esp_app_desc_t desc = {0};
    if (other == NULL || !read_part_desc(other, &desc)) {
        return send_error(req, "409 Conflict", "no valid backup partition");
    }
    /* set_boot_partition 不看 otadata 状态，会把已经因启动失败被标记
     * aborted 的镜像再切回去，白搭一轮重启。只接受验证过的分区。 */
    esp_ota_img_states_t state = ESP_OTA_IMG_UNDEFINED;
    if (esp_ota_get_state_partition(other, &state) != ESP_OK ||
        state != ESP_OTA_IMG_VALID) {
        return send_error(req, "409 Conflict", "backup partition not verified");
    }
    esp_err_t err = esp_ota_set_boot_partition(other);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "rollback set_boot failed: %s", esp_err_to_name(err));
        const char *status = (err == ESP_ERR_OTA_VALIDATE_FAILED)
                                 ? "409 Conflict" : "500 Internal Server Error";
        return send_error(req, status, "switch boot partition failed");
    }
    char version[33] = "";
    copy_field(version, sizeof(version), desc.version, sizeof(desc.version));
    esp_err_t e = send_ok_restart(req, version, 0);
    if (e != ESP_OK) {
        ESP_LOGW(TAG, "rollback response failed (%s), restart anyway", esp_err_to_name(e));
    }
    ESP_LOGI(TAG, "rollback -> %s (%s), restarting", part_name(other), version);
    bench_log_flush_now(); /* 回退也会重启，别把这次操作的日志丢在 RAM 里 */
    vTaskDelay(pdMS_TO_TICKS(2000));
    esp_restart();
    return ESP_OK;
}

esp_err_t bench_ota_start(httpd_handle_t hd)
{
    if (hd == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    httpd_uri_t uri = {
        .uri = "/api/ota/info",
        .method = HTTP_GET,
        .handler = handle_info,
        .user_ctx = NULL,
    };
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(hd, &uri), TAG, "info uri");

    uri.uri = "/api/ota/upload";
    uri.method = HTTP_POST;
    uri.handler = handle_upload;
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(hd, &uri), TAG, "upload uri");

    uri.uri = "/api/ota/rollback";
    uri.method = HTTP_POST;
    uri.handler = handle_rollback;
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(hd, &uri), TAG, "rollback uri");

    return ESP_OK;
}

void bench_ota_mark_valid(void)
{
    esp_err_t e = esp_ota_mark_app_valid_cancel_rollback();
    if (e == ESP_OK) {
        ESP_LOGI(TAG, "running app marked valid");
    } else if (e != ESP_ERR_OTA_ROLLBACK_INVALID_STATE) {
        ESP_LOGW(TAG, "mark valid failed: %s", esp_err_to_name(e));
    }
}

bool bench_ota_busy(void)
{
    return s_ota_busy;
}
