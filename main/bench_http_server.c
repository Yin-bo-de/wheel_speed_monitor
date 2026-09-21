/* bench_http_server 实现：见头文件注释。 */
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "cJSON.h"
#include "esp_check.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_system.h"

#include "bench_http_server.h"
#include "config_store.h"
#include "wheel_config.h"

static const char *TAG = "benchhttp";

/* 内嵌 index.html（EMBED_FILES）：符号 basename 为 index.html → _binary_index_html_* */
extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[] asm("_binary_index_html_end");

/* 按值持有上下文，不存调用方指针：ctx 若是 app_main 的栈变量，函数返回后
 * 栈帧即被回收，此后 httpd 任务再解引用就是读已释放内存（表现为
 * xSemaphoreTake 拿到垃圾句柄、assert pxQueue->uxItemSize == 0）。 */
static bench_server_ctx_t s_ctx;
static httpd_handle_t s_server_handle; /* 广播时直接用，不动态获取 */

/* 最新遥测帧快照（ws_push_task 写入；/api/status 读取） */
static char s_last_frame[2048];
static size_t s_last_frame_len;
static uint32_t s_last_seq;
static SemaphoreHandle_t s_frame_mutex;

/* WS 客户端表：fd 数组（单 WS /ws），0 空槽 */
#define WS_CLIENT_MAX 4

/* 单条 WS 命令上限。v2 舵机全字段配置约 400B，256 不够 */
#define WS_CMD_MAX 1024
static int s_ws_clients[WS_CLIENT_MAX];
static SemaphoreHandle_t s_clients_mutex;

/* ---- 静态页 ---- */

static esp_err_t handle_index(httpd_req_t *req)
{
    size_t len = (size_t)(index_html_end - index_html_start);
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_set_status(req, "200 OK");
    /* 直接回写内嵌文件（体小，无需 chunk） */
    return httpd_resp_send(req, (const char *)index_html_start, len);
}

/* ---- /api/status 兜底 ---- */

static esp_err_t handle_api_status(httpd_req_t *req)
{
    char buf[2048];
    size_t len = 0;
    uint32_t seq = 0;
    if (s_frame_mutex != NULL && xSemaphoreTake(s_frame_mutex, pdMS_TO_TICKS(500)) == pdTRUE) {
        len = s_last_frame_len;
        seq = s_last_seq;
        memcpy(buf, s_last_frame, len);
        xSemaphoreGive(s_frame_mutex);
    } else {
        len = 0;
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_set_status(req, "200 OK");
    if (len < (size_t)snprintf(buf, sizeof(buf), "{\"seq\":%lu,\"error\":\"no frame\"}", (unsigned long)seq)) {
        len = (size_t)snprintf(buf, sizeof(buf), "{\"seq\":%lu}", (unsigned long)seq);
    }
    return httpd_resp_send(req, buf, (len ? len : 1));
}

/* ---- WebSocket 客户端管理：注册/注销 ---- */

static void ws_client_register(int fd)
{
    if (xSemaphoreTake(s_clients_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return;
    }
    for (int i = 0; i < WS_CLIENT_MAX; i++) {
        if (s_ws_clients[i] == 0) {
            s_ws_clients[i] = fd;
            break;
        }
    }
    xSemaphoreGive(s_clients_mutex);
}

static void ws_client_unregister(int fd)
{
    if (xSemaphoreTake(s_clients_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return;
    }
    for (int i = 0; i < WS_CLIENT_MAX; i++) {
        if (s_ws_clients[i] == fd) {
            s_ws_clients[i] = 0;
            break;
        }
    }
    xSemaphoreGive(s_clients_mutex);
    /* 断开此前是静默的：页面掉线后只表现为"设置点了没反应"，日志里
     * 看不出连接何时掉的，排查只能靠猜。留一行好对上时间线。 */
    ESP_LOGW(TAG, "WS client disconnected fd=%d", fd);
}

/* ---- 命令处理（在 httpd 上下文） ---- */

static esp_err_t handle_set_cfg(const char *payload)
{
    /* 解析 JSON：magnets/diam_mm/alpha/en/sim/sim_rpm/debounce_ms */
    /* 使用 cJSON 解析 */
    cJSON *root = cJSON_Parse(payload);
    if (root == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t result = ESP_OK;
    const char *err_msg = NULL;

    cfg_params_t *cfg = s_ctx.cfg;
    xSemaphoreTake(s_ctx.cfg_mutex, portMAX_DELAY);

    cfg_params_t next = *cfg;
    bool changed = false;

    cJSON *j;
    if ((j = cJSON_GetObjectItemCaseSensitive(root, "magnets"))) {
        if (cJSON_IsNumber(j)) {
            next.magnets = (uint32_t)j->valueint;
            if (cfg_params_validate(&next, "magnets") != ESP_OK) {
                err_msg = "magnets out of range";
                result = ESP_ERR_INVALID_ARG;
                goto done;
            }
            changed = true;
        }
    }
    if ((j = cJSON_GetObjectItemCaseSensitive(root, "diam_mm"))) {
        if (cJSON_IsNumber(j)) {
            next.wheel_diam_mm = (uint32_t)j->valueint;
            if (cfg_params_validate(&next, "diam_mm") != ESP_OK) {
                err_msg = "diam_mm out of range";
                result = ESP_ERR_INVALID_ARG;
                goto done;
            }
            changed = true;
        }
    }
    if ((j = cJSON_GetObjectItemCaseSensitive(root, "alpha"))) {
        if (cJSON_IsNumber(j)) {
            next.alpha = (float)j->valuedouble;
            if (cfg_params_validate(&next, "alpha") != ESP_OK) {
                err_msg = "alpha out of range";
                result = ESP_ERR_INVALID_ARG;
                goto done;
            }
            changed = true;
        }
    }
    if ((j = cJSON_GetObjectItemCaseSensitive(root, "en"))) {
        if (cJSON_IsArray(j)) {
            for (int i = 0; i < WHEEL_COUNT; i++) {
                if (i < cJSON_GetArraySize(j)) {
                    cJSON *item = cJSON_GetArrayItem(j, i);
                    if (cJSON_IsBool(item)) {
                        next.wheel_enabled[i] = cJSON_IsTrue(item);
                    }
                }
            }
            changed = true;
        }
    }
    if ((j = cJSON_GetObjectItemCaseSensitive(root, "debounce_ms"))) {
        if (cJSON_IsNumber(j)) {
            next.debounce_ms = (uint32_t)j->valueint;
            if (cfg_params_validate(&next, "debounce_ms") != ESP_OK) {
                err_msg = "debounce_ms out of range";
                result = ESP_ERR_INVALID_ARG;
                goto done;
            }
            changed = true;
        }
    }
    if ((j = cJSON_GetObjectItemCaseSensitive(root, "sim"))) {
        if (cJSON_IsBool(j)) {
            next.sim_on = cJSON_IsTrue(j);
            changed = true;
        }
    }
    if ((j = cJSON_GetObjectItemCaseSensitive(root, "sim_rpm"))) {
        if (cJSON_IsArray(j)) {
            for (int i = 0; i < WHEEL_COUNT; i++) {
                if (i < cJSON_GetArraySize(j)) {
                    cJSON *item = cJSON_GetArrayItem(j, i);
                    if (cJSON_IsNumber(item)) {
                        next.sim_rpm[i] = (float)item->valuedouble;
                    }
                }
            }
            changed = true;
        }
    }

    /* ---- v2：差速舵机与锁定策略 ----
     * 这些字段形态一致、数量多，用三个局部宏收掉样板：
     * 取字段 → 赋值 → 逐字段校验（含跨字段约束）→ 置 changed，
     * 任一不合法即整条命令拒绝，与上面 v1 各字段同一语义。 */
#define TRY_U32(field, member)                                                            \
    do {                                                                                  \
        if ((j = cJSON_GetObjectItemCaseSensitive(root, field)) != NULL &&                \
            cJSON_IsNumber(j)) {                                                          \
            next.member = (uint32_t)j->valueint;                                          \
            if (cfg_params_validate(&next, field) != ESP_OK) {                            \
                err_msg = field " out of range";                                          \
                result = ESP_ERR_INVALID_ARG;                                             \
                goto done;                                                                \
            }                                                                             \
            changed = true;                                                               \
        }                                                                                 \
    } while (0)

#define TRY_F32(field, member)                                                            \
    do {                                                                                  \
        if ((j = cJSON_GetObjectItemCaseSensitive(root, field)) != NULL &&                \
            cJSON_IsNumber(j)) {                                                          \
            next.member = (float)j->valuedouble;                                          \
            if (cfg_params_validate(&next, field) != ESP_OK) {                            \
                err_msg = field " out of range";                                          \
                result = ESP_ERR_INVALID_ARG;                                             \
                goto done;                                                                \
            }                                                                             \
            changed = true;                                                               \
        }                                                                                 \
    } while (0)

#define TRY_BOOLS(field, member, count)                                                   \
    do {                                                                                  \
        if ((j = cJSON_GetObjectItemCaseSensitive(root, field)) != NULL &&                \
            cJSON_IsArray(j)) {                                                           \
            for (int k_ = 0; k_ < (count); k_++) {                                        \
                if (k_ < cJSON_GetArraySize(j)) {                                         \
                    cJSON *it_ = cJSON_GetArrayItem(j, k_);                               \
                    if (cJSON_IsBool(it_)) {                                              \
                        next.member[k_] = cJSON_IsTrue(it_);                              \
                    }                                                                     \
                }                                                                         \
            }                                                                             \
            changed = true;                                                               \
        }                                                                                 \
    } while (0)

    /* 数字数组：全部元素落进 next 后一次性校验（逐元素判边界，见 config_params）。
     * 与 TRY_U32 一样，任一元素不合法即整条命令拒绝。 */
#define TRY_U32_ARRAY(field, member, count)                                               \
    do {                                                                                  \
        if ((j = cJSON_GetObjectItemCaseSensitive(root, field)) != NULL &&                \
            cJSON_IsArray(j)) {                                                           \
            for (int k_ = 0; k_ < (count); k_++) {                                        \
                if (k_ < cJSON_GetArraySize(j)) {                                         \
                    cJSON *it_ = cJSON_GetArrayItem(j, k_);                               \
                    if (cJSON_IsNumber(it_)) {                                            \
                        next.member[k_] = (uint32_t)it_->valueint;                        \
                    }                                                                     \
                }                                                                         \
            }                                                                             \
            if (cfg_params_validate(&next, field) != ESP_OK) {                            \
                err_msg = field " out of range";                                          \
                result = ESP_ERR_INVALID_ARG;                                             \
                goto done;                                                                \
            }                                                                             \
            changed = true;                                                               \
        }                                                                                 \
    } while (0)

    TRY_BOOLS("servo_en", servo_en, CFG_SERVO_COUNT);

    if ((j = cJSON_GetObjectItemCaseSensitive(root, "servo_sim")) != NULL && cJSON_IsBool(j)) {
        next.servo_sim = cJSON_IsTrue(j);
        changed = true;
    }

    /* 模式用可读字符串传，服务端转成枚举；非法取值整条拒绝 */
    if ((j = cJSON_GetObjectItemCaseSensitive(root, "servo_mode")) != NULL &&
        j->valuestring != NULL) {
        if (strcmp(j->valuestring, "manual") == 0) {
            next.servo_mode = CFG_SERVO_MODE_MANUAL;
        } else if (strcmp(j->valuestring, "auto") == 0) {
            next.servo_mode = CFG_SERVO_MODE_AUTO;
        } else {
            err_msg = "servo_mode invalid";
            result = ESP_ERR_INVALID_ARG;
            goto done;
        }
        changed = true;
    }

    /* 三组脉宽都是每通道数组：前后轴各自一组 */
    TRY_U32_ARRAY("servo_unlock_us", servo_unlock_us, CFG_SERVO_COUNT);
    TRY_U32_ARRAY("servo_lock_us", servo_lock_us, CFG_SERVO_COUNT);
    TRY_U32_ARRAY("servo_manual_us", servo_manual_us, CFG_SERVO_COUNT);

    TRY_F32("slip_engage_ratio", slip_engage_ratio);
    TRY_F32("slip_min_rpm", slip_min_rpm);
    TRY_U32("lock_hold_ms", lock_hold_ms);
    TRY_U32("lock_hold_max_ms", lock_hold_max_ms);
    TRY_U32("probe_window_ms", probe_window_ms);
    TRY_U32("servo_sim_speed_us_s", servo_sim_speed_us_s);

#undef TRY_U32
#undef TRY_F32
#undef TRY_BOOLS
#undef TRY_U32_ARRAY

    if (changed) {
        *cfg = next;
        config_store_save(cfg);
        wheel_bench_apply_config(cfg); /* main 层应用采集器配置 */
        (void)WHEEL_SAMPLE_MS;
    }

done:
    xSemaphoreGive(s_ctx.cfg_mutex);
    cJSON_Delete(root);
    if (err_msg) {
        ESP_LOGE(TAG, "set_cfg rejected: %s", err_msg);
    }
    return result;
}

static void handle_reset_counts(void)
{
    xSemaphoreTake(s_ctx.cfg_mutex, portMAX_DELAY);
    for (int i = 0; i < WHEEL_COUNT; i++) {
        wheel_chan_reset_counts(&s_ctx.units[i].chan);
    }
    xSemaphoreGive(s_ctx.cfg_mutex);
}

/* WS 处理（is_websocket=true 注册）：握手后每次收帧调用。
 * req->method == HTTP_GET 表示握手完成（客户端刚连上），此时注册 fd 到广播表；
 * 其余是数据帧，解析文本命令（与官方 ws_echo_server 生命周期一致）。 */
static esp_err_t ws_handler(httpd_req_t *req)
{
    if (req->method == HTTP_GET) {
        int fd = httpd_req_to_sockfd(req);
        ws_client_register(fd);
        ESP_LOGI(TAG, "WS client connected fd=%d", fd);
        return ESP_OK;
    }

    httpd_ws_frame_t frame;
    memset(&frame, 0, sizeof(frame));
    frame.type = HTTPD_WS_TYPE_TEXT;
    /* 先取长度 */
    esp_err_t ret = httpd_ws_recv_frame(req, &frame, 0);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "ws recv len failed: %d", ret);
        return ESP_FAIL;
    }
    /* 1024：v2 舵机全字段配置命令约 400B，旧的 256B 上限会把 set_cfg 整条丢掉 */
    if (frame.len > WS_CMD_MAX) {
        ESP_LOGW(TAG, "ws frame too large: %u > %d", (unsigned)frame.len, WS_CMD_MAX);
        return ESP_ERR_INVALID_ARG; /* 过大丢弃 */
    }
    char payload[WS_CMD_MAX + 1];
    frame.payload = (uint8_t *)payload;
    ret = httpd_ws_recv_frame(req, &frame, WS_CMD_MAX);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "ws recv body failed: %d (len=%u)", ret, (unsigned)frame.len);
        return ESP_FAIL;
    }
    payload[frame.len] = '\0';
    ESP_LOGI(TAG, "WS frame: %s", payload);

    /* 命令路由 */
    cJSON *root = cJSON_Parse(payload);
    if (root == NULL) {
        ESP_LOGW(TAG, "bad ws json");
        return ESP_OK;
    }
    cJSON *jtype = cJSON_GetObjectItem(root, "type");
    if (jtype != NULL && jtype->valuestring != NULL) {
        if (strcmp(jtype->valuestring, "set_cfg") == 0) {
            handle_set_cfg(payload);
        } else if (strcmp(jtype->valuestring, "reset_counts") == 0) {
            handle_reset_counts();
        } else if (strcmp(jtype->valuestring, "get_cfg") == 0) {
            /* cfg 由遥测帧携带，无需专门回包 */
        }
    }
    cJSON_Delete(root);
    return ESP_OK;
}

/* ---- 广播（ws_push_task 调用） ---- */

void bench_http_server_broadcast(const char *frame, size_t len, uint32_t seq)
{
    /* 更新最新帧快照（供 /api/status） */
    if (s_frame_mutex != NULL && xSemaphoreTake(s_frame_mutex, pdMS_TO_TICKS(500)) == pdTRUE) {
        memcpy(s_last_frame, frame, len);
        s_last_frame_len = len;
        s_last_seq = seq;
        xSemaphoreGive(s_frame_mutex);
    }

    /* 拷贝客户端表后解锁再发送（不在锁内做网络 IO） */
    int clients[WS_CLIENT_MAX];
    if (xSemaphoreTake(s_clients_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return;
    }
    memcpy(clients, s_ws_clients, sizeof(clients));
    xSemaphoreGive(s_clients_mutex);

    httpd_handle_t hd = s_server_handle;
    if (hd == NULL) {
        return;
    }
    for (int i = 0; i < WS_CLIENT_MAX; i++) {
        int fd = clients[i];
        if (fd == 0) {
            continue;
        }
        /* 会话仍健康？ */
        httpd_ws_client_info_t info = httpd_ws_get_fd_info(hd, fd);
        if (info == HTTPD_WS_CLIENT_INVALID) {
            ws_client_unregister(fd);
            continue;
        }
        httpd_ws_frame_t wf = {
            .type = HTTPD_WS_TYPE_TEXT,
            .payload = (uint8_t *)frame,
            .len = (size_t)len,
        };
        if (httpd_ws_send_frame_async(hd, fd, &wf) != ESP_OK) {
            ws_client_unregister(fd); /* 发送失败，剔除 */
        }
    }
}

esp_err_t bench_http_server_start(const bench_server_ctx_t *ctx)
{
    /* mutex 必须在广播任务可能运行前就创建：
     * app_main 先 start（创建 mutex + httpd）再启动 ws_push_task，否则
     * 任务先跑会 xSemaphoreTake(NULL) 触发 assert（LoadProhibited 属于次生）。
     * 若想彻底解耦，可改由 app_main 预先创建并传入；此处按调用顺序保证。 */
    if (ctx == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    s_ctx = *ctx; /* 按值拷贝：调用方的 ctx 可以随其栈帧消亡，httpd 不受影响 */
    s_frame_mutex = xSemaphoreCreateMutex();
    s_clients_mutex = xSemaphoreCreateMutex();
    memset(s_ws_clients, 0, sizeof(s_ws_clients));

    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.stack_size = WHEEL_HTTPD_STACK;
    cfg.core_id = WHEEL_HTTPD_CORE;
    cfg.lru_purge_enable = true;
    cfg.uri_match_fn = httpd_uri_match_wildcard;

    httpd_handle_t hd;
    ESP_RETURN_ON_ERROR(httpd_start(&hd, &cfg), TAG, "err");
    s_server_handle = hd; /* 供 ws_push_task 广播用（此前漏赋值导致广播空转） */

    httpd_uri_t uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = handle_index,
        .user_ctx = NULL,
    };
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(hd, &uri), TAG, "err");

    uri.uri = "/index.html";
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(hd, &uri), TAG, "err");

    /* handler 必须显式重设：uri 结构体是从上一项改的，漏设会继续用 handle_index，
     * /api/status 就返回 HTML，前端 2 秒兜底轮询永远拿不到 JSON */
    uri.uri = "/api/status";
    uri.handler = handle_api_status;
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(hd, &uri), TAG, "err");

    uri.uri = "/ws";
    uri.method = HTTP_GET;
    uri.handler = ws_handler;
    uri.is_websocket = true;
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(hd, &uri), TAG, "err");

    ESP_LOGI(TAG, "httpd up; page http://%s/", WHEEL_AP_IP);
    return ESP_OK;
}
