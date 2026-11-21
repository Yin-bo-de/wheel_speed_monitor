/* oled_display：OLED 轮速显示子项目入口（需求方案见
 * docs/2026-11-20-OLED轮速显示子项目-需求方案.md）。
 *
 * 分层（二期 IMU 接入不动这个框架）：
 *   数据接收   feed_ws（WS 遥测帧 → telem_frame_view_t）
 *   指标计算   speed_calc + dev_cfg（车速换算，配置模型）
 *   显示渲染   ui_render（布局/字形，纯 C）→ ssd1306（I2C 硬件层）
 * 另有 http_portal：配置页 + OTA（双分区回滚）。
 *
 * 显示任务每 200ms 刷一帧：开机先出信息小页面（IP/版本/配置摘要，
 * 供查地址），随后进主视图；断线判定用遥测帧到达间隔，超阈值清屏出
 * 断开标记，一帧到达立即恢复。 */
#include <string.h>

#include "dev_cfg.h"
#include "dev_cfg_store.h"
#include "esp_app_desc.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_system.h"
#include "feed_ws.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "http_portal.h"
#include "net_wifi.h"
#include "nvs_flash.h"
#include "ota_portal.h"
#include "speed_calc.h"
#include "ssd1306.h"
#include "ui_render.h"

static const char *TAG = "oledmain";

#define BOOT_INFO_MS 8000   /* 开机信息小页面停留时长 */
#define DISPLAY_PERIOD_MS 200

static dev_cfg_t s_cfg;

static void display_task(void *arg)
{
    (void)arg;
    uint32_t boot_until =
        (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS) + BOOT_INFO_MS;
    static uint8_t fb[UI_FB_SIZE];

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(DISPLAY_PERIOD_MS));
        if (ota_portal_busy()) {
            continue; /* 升级写 flash 期间不刷屏，别让 I2C 传输添乱 */
        }
        uint32_t now = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);

        if (now < boot_until) {
            char ip[16];
            net_wifi_ip(ip, sizeof(ip));
            const esp_app_desc_t *desc = esp_app_get_description();
            ui_info_t info = {
                .ip = (ip[0] != '\0') ? ip : NULL,
                .version = desc->version,
                .wifi_ssid = s_cfg.wifi_ssid,
                .diam_mm = dev_cfg_diam_mm(&s_cfg),
                .disconnect_ms = s_cfg.disconnect_ms,
            };
            ui_render_info(&info, fb);
        } else {
            feed_state_t st;
            feed_ws_snapshot(&st);
            /* 断开判定：连续 disconnect_ms 收不到帧即断开；开机没连上
             * 也走断开画面。恢复无迟滞，一帧到达立即恢复显示。 */
            bool link_ok = st.last_frame_ms != 0 &&
                           (now - st.last_frame_ms) <= s_cfg.disconnect_ms;
            if (!link_ok) {
                ui_render_disconnected(fb);
            } else {
                speed_calc_in_t in = {
                    .diam_mm = dev_cfg_diam_mm(&s_cfg),
                };
                for (int i = 0; i < SPEED_CALC_WHEELS; i++) {
                    in.valid[i] = st.view.wheel_en[i];
                    in.rpm[i] = st.view.wheel_rpm[i];
                }
                ui_view_t v;
                memset(&v, 0, sizeof(v));
                v.speed_valid = speed_calc_kmh(&in, &v.speed_kmh);
                v.mode_valid = st.view.has_mode;
                v.mode_auto = st.view.mode_auto;
                for (int i = 0; i < TELEM_DIFF_COUNT; i++) {
                    v.phase[i] = st.view.phase[i];
                    v.phase_en[i] = st.view.phase_en[i];
                }
                ui_render_main(&v, fb);
            }
        }

        esp_err_t e = ssd1306_write_fb(fb);
        if (e != ESP_OK) {
            ESP_LOGW(TAG, "display flush failed: %s", esp_err_to_name(e));
        }
    }
}

void app_main(void)
{
    esp_err_t e = nvs_flash_init();
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        e = nvs_flash_init();
    }
    ESP_ERROR_CHECK(e);
    ESP_ERROR_CHECK(dev_cfg_store_load(&s_cfg));
    ESP_LOGI(TAG, "cfg: ssid=%s uri=%s diam=%.1fmm disc=%ums",
             s_cfg.wifi_ssid,
             s_cfg.server_uri[0] ? s_cfg.server_uri : "(auto: gw)",
             (double)dev_cfg_diam_mm(&s_cfg), (unsigned)s_cfg.disconnect_ms);

    ESP_ERROR_CHECK(ssd1306_init());
    ESP_ERROR_CHECK(net_wifi_start(&s_cfg));
    ESP_ERROR_CHECK(http_portal_start(&s_cfg));
    ESP_ERROR_CHECK(feed_ws_start(&s_cfg));

    /* 显示任务：周期刷屏 + 断开判定 */
    if (xTaskCreate(display_task, "disp", 4096, NULL, 4, NULL) != pdPASS) {
        ESP_LOGE(TAG, "display task create failed");
        abort();
    }
}
