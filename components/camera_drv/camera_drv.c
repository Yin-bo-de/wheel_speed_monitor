/* camera_drv 实现：见头文件注释。 */
#include <stdio.h>
#include <string.h>

#include "camera_drv.h"
#include "esp_camera.h"
#include "esp_log.h"
#include "sensor.h"

#if CONFIG_IDF_TARGET_ESP32S3
#include "driver/gpio.h"
#include "esp_rom_gpio.h"
#include "soc/gpio_sig_map.h"
#include "soc/lcd_cam_struct.h"
#endif

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

/* S3 的 XCLK 由 LCD_CAM 产生，esp_camera_deinit() 只停采集和 DMA，
 * 不会关时钟源。模组 PWDN 固定为 -1，若时钟继续输出，停采后传感器
 * 仍会持续成像/压缩并发热。这里显式关闭时钟源并把引脚拉低。 */
static void camera_xclk_disable(void)
{
#if CONFIG_IDF_TARGET_ESP32S3
    LCD_CAM.cam_ctrl.cam_clk_sel = 0; /* 0 = clock source disabled */
    LCD_CAM.cam_ctrl.cam_update = 1;

    esp_rom_gpio_connect_out_signal(CAM_PIN_XCLK, SIG_GPIO_OUT_IDX, false, false);
    gpio_set_direction(CAM_PIN_XCLK, GPIO_MODE_OUTPUT);
    gpio_set_level(CAM_PIN_XCLK, 0);
    ESP_LOGI(TAG, "xclk off");
#endif
}

static bool s_inited;
static char s_sensor_name[16] = "unknown";

/* 帧率控制只改传感器自己的时序寄存器：
 *   OV2640：CLKRC(传感器 bank 0x11) 的分频
 *   OV5640：VTS(0x380E/0x380F) 的帧长
 * 基准寄存器在初始化后读一次，之后按实测基准帧率换算。 */
static sensor_t *s_sensor;
static bool s_fps_supported;
static uint8_t s_base_clkrc;
static uint8_t s_cur_clkrc;
static uint16_t s_base_vts;
static uint16_t s_cur_vts;

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
        /* 初始化可能已经打开过 XCLK，失败返回前也要收尾。 */
        camera_xclk_disable();
        ESP_LOGE(TAG, "esp_camera_init failed: %d（检查 PSRAM 与排线）", e);
        return e;
    }

    sensor_t *sensor = esp_camera_sensor_get();
    if (sensor == NULL) {
        ESP_LOGE(TAG, "esp_camera_sensor_get returned NULL");
        (void)esp_camera_deinit();
        camera_xclk_disable();
        return ESP_FAIL;
    }

    s_sensor = sensor;
    s_fps_supported = false;
    s_base_clkrc = s_cur_clkrc = 0;
    s_base_vts = s_cur_vts = 0;
    if (sensor->get_reg != NULL && sensor->set_reg != NULL) {
        if (sensor->id.PID == OV2640_PID) {
            int clkrc = sensor->get_reg(sensor, 0x0111, 0xFF); /* sensor bank */
            if (clkrc >= 0) {
                s_base_clkrc = s_cur_clkrc = (uint8_t)clkrc;
                s_fps_supported = true;
            }
        } else if (sensor->id.PID == OV5640_PID) {
            int vts = sensor->get_reg(sensor, 0x380E, 0xFFFF);
            if (vts > 0) {
                s_base_vts = s_cur_vts = (uint16_t)vts;
                s_fps_supported = true;
            }
        }
    }

    snprintf(s_sensor_name, sizeof(s_sensor_name), "%s", pid_to_name(sensor->id.PID));
    ESP_LOGI(TAG, "sensor %s PID=0x%04x, %ux%u, q=%lu, jpeg_buf=%u B, fps_ctrl=%d",
             s_sensor_name, (unsigned)sensor->id.PID,
             (unsigned)camera_drv_width(cfg->res), (unsigned)camera_drv_height(cfg->res),
             (unsigned long)cfg->quality, (unsigned)camera_drv_max_jpeg(cfg->res),
             s_fps_supported);

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
    /* esp_camera_deinit() 不关 S3 的 XCLK，必须在它之后主动收尾。 */
    camera_xclk_disable();
    s_inited = false;
    s_sensor = NULL;
    s_fps_supported = false;
    s_base_clkrc = s_cur_clkrc = 0;
    s_base_vts = s_cur_vts = 0;
    snprintf(s_sensor_name, sizeof(s_sensor_name), "%s", "unknown");
    if (e != ESP_OK) {
        ESP_LOGW(TAG, "esp_camera_deinit failed: %d", e);
    }
    return e;
}

bool camera_drv_fps_control_supported(void)
{
    return s_inited && s_fps_supported;
}

esp_err_t camera_drv_set_fps(uint32_t base_fps, uint32_t target_fps)
{
    if (!s_inited || !s_fps_supported || s_sensor == NULL) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    if (base_fps == 0 || target_fps == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_sensor->id.PID == OV2640_PID) {
        /* CLKRC 低 6 位是分频：真实帧率 ≈ 基准 / (div+1)。
         * 目标高于基准时保持基准，不尝试超频。 */
        uint32_t base_div = s_base_clkrc & 0x3Fu;
        uint32_t div = base_div;
        if (target_fps < base_fps) {
            uint64_t num = (uint64_t)(base_div + 1u) * base_fps;
            div = (uint32_t)((num + target_fps / 2u) / target_fps) - 1u;
            if (div < base_div) {
                div = base_div;
            }
            if (div > 0x3Fu) {
                div = 0x3Fu;
            }
        }
        uint8_t value = (uint8_t)((s_base_clkrc & 0xC0u) | (div & 0x3Fu));
        if (value == s_cur_clkrc) {
            return ESP_OK;
        }
        int ret = s_sensor->set_reg(s_sensor, 0x0111, 0xFF, value);
        if (ret != 0) {
            ESP_LOGW(TAG, "set CLKRC=0x%02x failed: %d", value, ret);
            return ESP_FAIL;
        }
        s_cur_clkrc = value;
        ESP_LOGD(TAG, "fps %lu -> %lu: CLKRC=0x%02x div=%lu",
                 (unsigned long)base_fps, (unsigned long)target_fps,
                 (unsigned)value, (unsigned long)div);
        return ESP_OK;
    }

    if (s_sensor->id.PID == OV5640_PID) {
        /* VTS 是帧长：目标低于基准时按比例拉长 VTS。
         * 目标高于基准时保持基准（不缩短到传感器默认以下）。 */
        uint32_t vts = s_base_vts;
        if (target_fps < base_fps) {
            uint64_t num = (uint64_t)s_base_vts * base_fps;
            vts = (uint32_t)((num + target_fps / 2u) / target_fps);
            if (vts < s_base_vts) {
                vts = s_base_vts;
            }
            if (vts > 0xFFFFu) {
                vts = 0xFFFFu;
            }
        }
        if (vts == s_cur_vts) {
            return ESP_OK;
        }
        int ret = s_sensor->set_reg(s_sensor, 0x380E, 0xFFFF, (int)vts);
        if (ret != 0) {
            ESP_LOGW(TAG, "set VTS=%u failed: %d", (unsigned)vts, ret);
            return ESP_FAIL;
        }
        s_cur_vts = (uint16_t)vts;
        ESP_LOGD(TAG, "fps %lu -> %lu: VTS=%u",
                 (unsigned long)base_fps, (unsigned long)target_fps, (unsigned)vts);
        return ESP_OK;
    }

    return ESP_ERR_NOT_SUPPORTED;
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
