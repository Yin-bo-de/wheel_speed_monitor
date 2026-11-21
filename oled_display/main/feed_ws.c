/* feed_ws 实现：见头文件注释。 */
#include "feed_ws.h"

#include <string.h>

#include "esp_check.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_websocket_client.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "net_wifi.h"

static const char *TAG = "feedws";

static dev_cfg_t s_cfg;
static esp_websocket_client_handle_t s_client;
static SemaphoreHandle_t s_lock;
static feed_state_t s_state;

static void snapshot_set(const feed_state_t *st)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_state = *st;
    xSemaphoreGive(s_lock);
}

void feed_ws_snapshot(feed_state_t *out)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    *out = s_state;
    xSemaphoreGive(s_lock);
}

static void ws_event_handler(void *arg, esp_event_base_t base,
                             int32_t id, void *data)
{
    (void)arg;
    (void)base;
    feed_state_t st;
    feed_ws_snapshot(&st);

    switch (id) {
    case WEBSOCKET_EVENT_CONNECTED:
        ESP_LOGI(TAG, "ws connected");
        st.ws_connected = true;
        break;
    case WEBSOCKET_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "ws disconnected");
        st.ws_connected = false;
        break;
    case WEBSOCKET_EVENT_DATA: {
        esp_websocket_event_data_t *ev = (esp_websocket_event_data_t *)data;
        /* 车端单帧就是完整 JSON（最大约 3KB），分片到达按拼好再解析 */
        static char frame[4096];
        if (ev->payload_offset + ev->data_len > (int)sizeof(frame)) {
            break; /* 超长帧丢弃：契约外的东西不值得为它拼包 */
        }
        memcpy(frame + ev->payload_offset, ev->data_ptr, ev->data_len);
        if (ev->payload_offset + ev->data_len < ev->payload_len) {
            break; /* 还没拼完整帧 */
        }
        telem_frame_view_t view;
        if (telem_frame_parse(frame, ev->payload_len, &view)) {
            st.view = view;
            st.last_frame_ms =
                (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
        }
        break;
    }
    default:
        return;
    }
    snapshot_set(&st);
}

static void feed_task(void *arg)
{
    (void)arg;
    /* 等 IP：验证时连电脑热点、车上连 wheel-bench，顺序无关 */
    while (!net_wifi_got_ip()) {
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    char uri[96];
    if (s_cfg.server_uri[0] != '\0') {
        snprintf(uri, sizeof(uri), "%s", s_cfg.server_uri);
    } else {
        /* 自动定位服务端 = DHCP 网关：车端 AP 网关是 192.168.4.1，
         * 电脑热点的网关就是跑模拟脚本的电脑 */
        char gw[16];
        net_wifi_gw(gw, sizeof(gw));
        snprintf(uri, sizeof(uri), "ws://%s/ws", gw);
    }
    ESP_LOGI(TAG, "ws feed uri: %s", uri);

    esp_websocket_client_config_t cc = {
        .uri = uri,
        /* 车端帧最大约 3KB，默认 1KB 缓冲会截断整帧 */
        .buffer_size = 4096,
        .task_stack = 6144,
        .reconnect_timeout_ms = 2000, /* 车端重启/OTA 后自动重连自愈 */
        .network_timeout_ms = 5000,
    };
    s_client = esp_websocket_client_init(&cc);
    if (s_client == NULL) {
        ESP_LOGE(TAG, "ws client init failed");
        vTaskDelete(NULL);
        return;
    }
    esp_websocket_register_events(s_client, WEBSOCKET_EVENT_ANY,
                                  ws_event_handler, NULL);
    if (esp_websocket_client_start(s_client) != ESP_OK) {
        ESP_LOGE(TAG, "ws client start failed");
    }
    vTaskDelete(NULL); /* 后续生命周期全由 ws 客户端内部任务接管 */
}

esp_err_t feed_ws_start(const dev_cfg_t *cfg)
{
    if (cfg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    s_cfg = *cfg;
    s_lock = xSemaphoreCreateMutex();
    if (s_lock == NULL) {
        return ESP_ERR_NO_MEM;
    }
    /* 5120：本地任务只做一次连接建立，栈需求不大 */
    BaseType_t ok = xTaskCreate(feed_task, "ws_feed", 5120, NULL, 5, NULL);
    return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}
