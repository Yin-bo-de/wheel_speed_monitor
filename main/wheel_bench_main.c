/*
 * wheel_bench_main：入口编排。
 * NVS → 传感器 → 采集器注册 → 任务/队列 → WiFi AP → httpd。
 *
 * 数据流（一期）：sampler_task（Core0）每 50ms 读 4 路 PCNT 计数与 GPIO 电平
 * → wheel 采集器运算 → telemetry 聚合渲染 → ws_push_task（Core0）每 100ms
 * 经 WebSocket 推给验证台网页。命令在 httpd 上下文处理（config 变更 + NVS）。
 *
 * 任务表（优先/栈/核心见 wheel_config.h；固化进项目 CLAUDE.md）：
 *   sampler_task  Core0 22 3072  50ms 周期采样
 *   ws_push_task  Core0 15 4096  100ms 推遥测帧
 *   httpd server  Core1  5 8192  事件驱动
 */
#include <string.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_netif_types.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "bench_http_server.h"
#include "config_store.h"
#include "telemetry.h"
#include "wheel_bench.h"
#include "wheel_config.h"
#include "wheel_sensor.h"

static const char *TAG = "wheelbench";

static cfg_params_t s_cfg;
static SemaphoreHandle_t s_cfg_mutex;
static wheel_unit_t s_units[WHEEL_COUNT];
static wheel_chan_cfg_t s_chan_cfg;

/* ---- wheel_speed 采集器 ops（telemetry 框架注册） ---- */

esp_err_t wheel_bench_apply_config(const cfg_params_t *cfg)
{
    wheel_chan_cfg_t *cc = &s_chan_cfg;
    cc->window_ms = WHEEL_SAMPLE_MS;
    cc->magnets = cfg->magnets;
    cc->wheel_diam_mm = cfg->wheel_diam_mm;
    cc->alpha = cfg->alpha;
    cc->debounce_ms = cfg->debounce_ms;

    /* 换源/启停变更会改变基准，重置避免毛刺 */
    for (int i = 0; i < WHEEL_COUNT; i++) {
        if (cfg->sim_on) {
            sim_source_init(&s_units[i].sim);
        }
        if (cfg->wheel_enabled[i]) {
            uint16_t c = 0;
            if (cfg->sim_on) {
                c = sim_source_counter(&s_units[i].sim, (uint8_t)i);
            } else {
                wheel_sensor_read_count((uint8_t)i, &c);
            }
            wheel_chan_reset_baseline(&s_units[i].chan, c);
        } else {
            wheel_chan_reset_baseline(&s_units[i].chan, 0);
        }
    }
    return ESP_OK;
}

static esp_err_t wheel_collector_sample(void *ctx, uint32_t now_ms)
{
    (void)ctx;
    for (int i = 0; i < WHEEL_COUNT; i++) {
        if (!s_cfg.wheel_enabled[i]) {
            continue;
        }
        uint16_t c = 0;
        if (s_cfg.sim_on) {
            sim_source_set_target_rpm(&s_units[i].sim, (uint8_t)i, s_cfg.sim_rpm[i]);
            sim_source_step(&s_units[i].sim, WHEEL_SAMPLE_MS);
            c = sim_source_counter(&s_units[i].sim, (uint8_t)i);
        } else {
            wheel_sensor_read_count((uint8_t)i, &c);
        }
        wheel_chan_sample(&s_units[i].chan, &s_chan_cfg, c, now_ms);
    }
    return ESP_OK;
}

static esp_err_t wheel_collector_render(void *ctx, char *buf, size_t len, size_t *used)
{
    (void)ctx;
    return wheel_render_block(&s_units[0].disp, buf, len, used);
}

static esp_err_t wheel_collector_reset(void *ctx)
{
    (void)ctx;
    for (int i = 0; i < WHEEL_COUNT; i++) {
        uint16_t c = 0;
        if (s_cfg.sim_on) {
            sim_source_init(&s_units[i].sim);
        } else {
            wheel_sensor_read_count((uint8_t)i, &c);
        }
        wheel_chan_reset_baseline(&s_units[i].chan, c);
    }
    return ESP_OK;
}

static const telem_collector_ops_t s_wheel_ops = {
    .type = TELEM_TYPE_WHEEL_SPEED,
    .init = NULL,
    .sample = wheel_collector_sample,
    .render = wheel_collector_render,
    .reset_baseline = wheel_collector_reset,
};

/* ---- 采样任务 ---- */

static void sampler_task(void *arg)
{
    (void)arg;
    TickType_t last = xTaskGetTickCount();
    uint8_t gpio[WHEEL_COUNT];

    while (1) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(WHEEL_SAMPLE_MS));
        uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

        /* GPIO 电平（sys 块，不受 PCNT 影响） */
        wheel_sensor_read_gpio_levels(gpio);
        telem_set_gpio_levels(gpio);

        /* 采集器运算（帧渲染与其分离，由 ws_push 完成） */
        telem_sample_all(now_ms);
    }
}

/* ---- 遥测推送任务 ---- */

static void ws_push_task(void *arg)
{
    (void)arg;
    TickType_t last = xTaskGetTickCount();
    char frame[2048];
    uint32_t seq = 0;

    while (1) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(WHEEL_WS_PUSH_MS));
        uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

        /* 组装各轮显示快照 */
        for (int i = 0; i < WHEEL_COUNT; i++) {
            wheel_unit_t *u = &s_units[i];
            wheel_render_wheel_t *w = &u->disp.wheels[i];
            wheel_render_state_t *d = &u->disp;
            d->source = s_cfg.sim_on ? WHEEL_RENDER_SRC_SIM : WHEEL_RENDER_SRC_PCNT;
            d->magnets = s_cfg.magnets;
            d->wheel_diam_mm = s_cfg.wheel_diam_mm;
            d->alpha = s_cfg.alpha;
            d->sim_on = s_cfg.sim_on;
            for (int j = 0; j < WHEEL_COUNT; j++) {
                d->sim_rpm[j] = s_cfg.sim_rpm[j];
            }
            w->enabled = s_cfg.wheel_enabled[i];
            w->rpm = u->chan.math.rpm_ema;
            w->speed_cm_s = u->chan.math.speed_cm_s;
            w->speed_km_h = u->chan.math.speed_km_h;
            w->freq_hz = u->chan.math.freq_hz;
            w->pulses = u->chan.pulses_total;
            w->trigger = u->chan.trigger;
        }

        size_t used = 0;
        esp_err_t e = telem_render_frame(frame, sizeof(frame), &used, seq, now_ms,
                                         (uint32_t)esp_get_free_heap_size());
        if (e == ESP_OK) {
            bench_http_server_broadcast(frame, used, seq++);
        }
    }
}

/* ---- WiFi AP ---- */

static void wifi_ap_start(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t ap = {
        .ap = {
            .ssid = WHEEL_AP_SSID,
            .password = WHEEL_AP_PASSWD,
            .ssid_len = 0,
            .channel = 1,
            .max_connection = 4, /* 验证台最多 4 台设备（与 sdkconfig CONFIG_ESP_WIFI_AP_MAX_STA_CONN=4 一致） */
            .authmode = WIFI_AUTH_OPEN,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap));
    ESP_ERROR_CHECK(esp_wifi_start());

    esp_netif_ip_info_t ip = {
        .ip = {.addr = ESP_IP4TOADDR(192, 168, 4, 1)},
        .gw = {.addr = ESP_IP4TOADDR(192, 168, 4, 1)},
        .netmask = {.addr = ESP_IP4TOADDR(255, 255, 255, 0)},
    };
    esp_netif_t *ap_netif = esp_netif_get_handle_from_ifkey("ESP_NETIF_DEFAULT_AP");
    ESP_ERROR_CHECK(esp_netif_set_ip_info(ap_netif, &ip));

    ESP_LOGI(TAG, "WiFi AP up: %s @ %s", WHEEL_AP_SSID, WHEEL_AP_IP);
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());

    cfg_params_t cfg;
    ESP_ERROR_CHECK(config_store_load(&cfg));
    s_cfg = cfg;
    s_cfg_mutex = xSemaphoreCreateMutex();

    /* collector 配置快照 */
    s_chan_cfg.window_ms = WHEEL_SAMPLE_MS;
    s_chan_cfg.magnets = cfg.magnets;
    s_chan_cfg.wheel_diam_mm = cfg.wheel_diam_mm;
    s_chan_cfg.alpha = cfg.alpha;
    s_chan_cfg.debounce_ms = cfg.debounce_ms;

    static const uint8_t hall_gpios[WHEEL_COUNT] = {
        WHEEL_HALL_GPIO_LF, WHEEL_HALL_GPIO_RF, WHEEL_HALL_GPIO_LR, WHEEL_HALL_GPIO_RR,
    };
    ESP_ERROR_CHECK(wheel_sensor_init(hall_gpios));
    for (int i = 0; i < WHEEL_COUNT; i++) {
        sim_source_init(&s_units[i].sim);
        memset(&s_units[i].disp, 0, sizeof(s_units[i].disp));
    }

    /* 注册 wheel 采集器到 telemetry 框架 */
    esp_err_t e = telem_register(&s_wheel_ops, NULL);
    ESP_LOGI(TAG, "telemetry register: %d", e);

    xTaskCreatePinnedToCore(sampler_task, "sampler", WHEEL_SAMPLER_STACK, NULL,
                            WHEEL_SAMPLER_PRIO, NULL, WHEEL_SAMPLER_CORE);
    xTaskCreatePinnedToCore(ws_push_task, "ws_push", WHEEL_WS_PUSH_STACK, NULL,
                            WHEEL_WS_PUSH_PRIO, NULL, WHEEL_WS_PUSH_CORE);

    wifi_ap_start();

    bench_server_ctx_t ctx = {
        .cfg = &s_cfg,
        .cfg_mutex = s_cfg_mutex,
        .units = s_units,
    };
    ESP_ERROR_CHECK(bench_http_server_start(&ctx));
}
