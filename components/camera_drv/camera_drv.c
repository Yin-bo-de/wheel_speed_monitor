/* camera_drv 实现：见头文件注释。 */
#include <stdio.h>
#include <string.h>

#include "camera_drv.h"
#include "esp_camera.h"
#include "esp_log.h"
#include "sensor.h"

static const char *TAG = "camera_drv";

/* Freenove ESP32-S3 WROOM CAM（FNK0085）板载 DVP 接线，按本地例程
 * camera_pins.h 的 ESP32S3_EYE 段核对：Y2..Y9 依次对应 D0..D7。
 * PWDN/RESET 板上固定为 -1，esp_camera_deinit() 不断模组电。 */
#define CAM_PIN_XCLK 15
#define CAM_PIN_SIOD 4
#define CAM_PIN_SIOC 5
#define CAM_PIN_VSYNC 6
#define CAM_PIN_HREF 7
#define CAM_PIN_PCLK 13
#define CAM_PIN_D0 11
#define CAM_PIN_D1 9
#define CAM_PIN_D2 8
#define CAM_PIN_D3 10
#define CAM_PIN_D4 12
#define CAM_PIN_D5 18
#define CAM_PIN_D6 17
#define CAM_PIN_D7 16

#define CAM_XCLK_HZ 20000000

static bool s_inited;
static char s_sensor_name[16] = "unknown";

static framesize_t res_to_framesize(uint32_t res)
{
    switch (res) {
    case CAMERA_DRV_RES_VGA:
        return FRAMESIZE_VGA;
    case CAMERA_DRV_RES_SVGA:
        return FRAMESIZE_SVGA;
    case CAMERA_DRV_RES_QVGA:
    default:
        return FRAMESIZE_QVGA;
    }
}

size_t camera_drv_max_jpeg(uint32_t res)
{
    /* 按实测留两倍余量：esp32-camera 未显式给 jpeg_buffer_size 时按
     * width×height/5 估算，quality 12 的复杂画面会超过它并报 FB-OVF。 */
    switch (res) {
    case CAMERA_DRV_RES_VGA:
        return 96 * 1024;
    case CAMERA_DRV_RES_SVGA:
        return 160 * 1024;
    case CAMERA_DRV_RES_QVGA:
    default:
        return 32 * 1024;
    }
}

uint16_t camera_drv_width(uint32_t res)
{
    switch (res) {
    case CAMERA_DRV_RES_VGA:
        return 640;
    case CAMERA_DRV_RES_SVGA:
        return 800;
    case CAMERA_DRV_RES_QVGA:
    default:
        return 320;
    }
}

uint16_t camera_drv_height(uint32_t res)
{
    switch (res) {
    case CAMERA_DRV_RES_VGA:
        return 480;
    case CAMERA_DRV_RES_SVGA:
        return 600;
    case CAMERA_DRV_RES_QVGA:
    default:
        return 240;
    }
}

static const char *pid_to_name(uint16_t pid)
{
    switch (pid) {
    case OV2640_PID:
        return "OV2640";
    case OV5640_PID:
        return "OV5640";
    case OV3660_PID:
        return "OV3660";
    case OV3640_PID:
        return "OV3640";
    case OV7670_PID:
        return "OV7670";
    case OV7725_PID:
        return "OV7725";
    case GC2145_PID:
        return "GC2145";
    case GC032A_PID:
        return "GC032A";
    case GC0308_PID:
        return "GC0308";
    default:
        return "unknown";
    }
}

esp_err_t camera_drv_init(const camera_drv_cfg_t *cfg)
{
    if (cfg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (cfg->res > CAMERA_DRV_RES_SVGA) {
        return ESP_ERR_INVALID_ARG;
    }

    /* 重 init 走同一条路：先拆掉旧实例，避免 esp_camera_init 报已初始化。 */
    if (s_inited) {
        (void)camera_drv_deinit();
    }

    camera_config_t config;
    memset(&config, 0, sizeof(config));
    config.pin_pwdn = -1;
    config.pin_reset = -1;
    config.pin_xclk = CAM_PIN_XCLK;
    config.pin_sccb_sda = CAM_PIN_SIOD;
    config.pin_sccb_scl = CAM_PIN_SIOC;
    config.pin_d0 = CAM_PIN_D0;
    config.pin_d1 = CAM_PIN_D1;
    config.pin_d2 = CAM_PIN_D2;
    config.pin_d3 = CAM_PIN_D3;
    config.pin_d4 = CAM_PIN_D4;
    config.pin_d5 = CAM_PIN_D5;
    config.pin_d6 = CAM_PIN_D6;
    config.pin_d7 = CAM_PIN_D7;
    config.pin_vsync = CAM_PIN_VSYNC;
    config.pin_href = CAM_PIN_HREF;
    config.pin_pclk = CAM_PIN_PCLK;
    config.xclk_freq_hz = CAM_XCLK_HZ;
    config.ledc_timer = LEDC_TIMER_0;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.pixel_format = PIXFORMAT_JPEG;
    config.frame_size = res_to_framesize(cfg->res);
    config.jpeg_quality = (int)cfg->quality;
    /* 两个槽：CAMERA_GRAB_LATEST 下完成队列长度 = fb_count - 1 = 1，
     * 上层拿到的就是最新完成帧，不会积压出一帧延迟。 */
    config.fb_count = 2;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.grab_mode = CAMERA_GRAB_LATEST;
    config.jpeg_buffer_size = camera_drv_max_jpeg(cfg->res);

    esp_err_t e = esp_camera_init(&config);
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "esp_camera_init failed: %d（检查 PSRAM 与排线）", e);
        return e;
    }

    sensor_t *sensor = esp_camera_sensor_get();
    if (sensor == NULL) {
        ESP_LOGE(TAG, "esp_camera_sensor_get returned NULL");
        (void)esp_camera_deinit();
        return ESP_FAIL;
    }

    snprintf(s_sensor_name, sizeof(s_sensor_name), "%s", pid_to_name(sensor->id.PID));
    ESP_LOGI(TAG, "sensor %s PID=0x%04x, %ux%u, q=%lu, jpeg_buf=%u B",
             s_sensor_name, (unsigned)sensor->id.PID,
             (unsigned)camera_drv_width(cfg->res), (unsigned)camera_drv_height(cfg->res),
             (unsigned long)cfg->quality, (unsigned)camera_drv_max_jpeg(cfg->res));

    /* 装车方向按 Freenove 例程先上下翻。若仍倒 180°，首版只在页面上做
     * CSS 翻转，不改这里——快照下载的原始 JPEG 保持相机方向。 */
    (void)sensor->set_vflip(sensor, 1);

    s_inited = true;
    return ESP_OK;
}

esp_err_t camera_drv_deinit(void)
{
    if (!s_inited) {
        return ESP_OK;
    }
    esp_err_t e = esp_camera_deinit();
    s_inited = false;
    snprintf(s_sensor_name, sizeof(s_sensor_name), "%s", "unknown");
    if (e != ESP_OK) {
        ESP_LOGW(TAG, "esp_camera_deinit failed: %d", e);
    }
    return e;
}

esp_err_t camera_drv_grab(camera_drv_frame_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(out, 0, sizeof(*out));
    if (!s_inited) {
        return ESP_ERR_INVALID_STATE;
    }

    camera_fb_t *fb = esp_camera_fb_get();
    if (fb == NULL) {
        return ESP_ERR_TIMEOUT;
    }
    if (fb->format != PIXFORMAT_JPEG) {
        /* 配置写死 JPEG；真拿到别的格式说明驱动状态被打乱，别把原始像素当
         * JPEG 发给浏览器。 */
        esp_camera_fb_return(fb);
        return ESP_ERR_INVALID_RESPONSE;
    }
    out->handle = fb;
    out->data = fb->buf;
    out->len = fb->len;
    out->width = (uint16_t)fb->width;
    out->height = (uint16_t)fb->height;
    return ESP_OK;
}

void camera_drv_release(camera_drv_frame_t *frame)
{
    if (frame == NULL || frame->handle == NULL) {
        return;
    }
    esp_camera_fb_return((camera_fb_t *)frame->handle);
    memset(frame, 0, sizeof(*frame));
}

bool camera_drv_is_inited(void)
{
    return s_inited;
}

const char *camera_drv_sensor_name(void)
{
    return s_sensor_name;
}
