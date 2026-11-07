/* camera_drv 实现：见头文件注释。 */
#include <stdio.h>
#include <string.h>

#include "camera_drv.h"
#include "esp_camera.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
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

/* hx 在 S3 + 20MHz XCLK 上验证过的 PLL 参数。上游 esp32-camera 用
 * multiplier=180、pre_div=2，VCO 高达 1.8GHz，超出 OV5640 500–1000MHz
 * 规格，实际帧率只有 11fps。下面的参数把 VCO 压回规格内并抬高 PCLK。 */
#define CAM_OV5640_S3_PLL_FIX 1
/* VGA 40fps 档：hx 的高帧率模式，VCO 810MHz。帧长 25ms 仍跨两个 50Hz
 * 周期，anti-banding 可以保留。先验证 30fps 档稳定后再打开。 */
#define CAM_OV5640_VGA_HIGH_FPS 0

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
static uint32_t s_cur_quality;
/* PLL 生效后 AEC 的防闪烁步长要按新的 SYSCLK/HTS 重算。为 0 表示
 * 仍使用驱动自带的寄存器常量（上游 PLL 档），VTS 变化时不动 banding。 */
static uint32_t s_banding_sysclk_hz;
static uint16_t s_banding_hts;
static bool s_banding_dirty; /* 上次 banding 写失败，下次改帧率时重试 */
/* PLL 和 VTS 算出的理论基准帧率。时钟模型已按实测校准（见
 * ov5640_pll_sysclk_hz），正常应与实测 base 一致；不一致说明传感器
 * 侧还有额外降速（如曝光时间上限），可作为诊断线索。 */
static uint32_t s_theory_fps;

/* OV5640 停掉 XCLK 后仍保持常电偏置，模组会继续发热。deinit 前让它
 * 进入软件掉电（SYSTEM_CTROL0 bit6）；下次 esp_camera_init() 会写
 * 0x3008=0x82 清除掉电位，采集能正常恢复。 */
static void camera_sensor_standby(void)
{
    if (s_sensor == NULL || s_sensor->set_reg == NULL ||
        s_sensor->id.PID != OV5640_PID) {
        return;
    }
    int ret = s_sensor->set_reg(s_sensor, 0x3008, 0xFF, 0x42);
    if (ret == 0) {
        ESP_LOGI(TAG, "sensor standby");
        esp_rom_delay_us(5000);
    } else {
        ESP_LOGW(TAG, "sensor standby failed: %d", ret);
    }
}

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

typedef struct {
    uint8_t multiplier;
    uint8_t sys_div;
    uint8_t pre_div;
    uint8_t pclk_root_div;
    uint8_t pclk_div;
} ov5640_pll_params_t;

/* VGA/SVGA 档：VCO 606MHz、SYSCLK 30.3MHz（实测 15fps）。pclk_div 用
 * 6 而不是 hx 的 3：PCLK 只决定 JPEG 流出的总线速率，帧率由 SYSCLK/VTS
 * 决定，加大分频不降帧率。hx 的 div 3 让 PCLK 到 20.22MHz，S3 的 1KB
 * 节点中断在 Core1 上被 httpd/WiFi 抢占就会丢字节（花屏/NO-EOI）；
 * div 6 把 PCLK 压到 10.11MHz，回到改动前 11.25MHz 已验证能稳的档位，
 * 21KB 的 JPEG 在 66ms 帧长里仍只需约 2ms 就能流完。 */
static const ov5640_pll_params_t OV5640_PLL_30FPS = {91, 1, 3, 2, 6};
/* esp32-camera 上游给 VGA/SVGA 的默认 PLL（180/4/2，VCO 1.8GHz）。
 * 新 PLL 只写了一半失败时用它把传感器恢复到一个已知状态。 */
static const ov5640_pll_params_t OV5640_PLL_UPSTREAM = {180, 4, 2, 2, 4};
#if CAM_OV5640_VGA_HIGH_FPS
/* VGA 40fps：VCO 810MHz，PCLK 20.25MHz。 */
static const ov5640_pll_params_t OV5640_PLL_VGA_40FPS = {81, 1, 2, 2, 4};
#endif

static esp_err_t ov5640_apply_pll(sensor_t *sensor,
                                  const ov5640_pll_params_t *pll,
                                  uint32_t *sysclk_hz);
static esp_err_t ov5640_write_banding(sensor_t *sensor, uint32_t sysclk_hz,
                                      uint16_t hts, uint16_t vts);

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

    /* S3 + 20MHz XCLK 下上游 PLL 参数（VCO 1.8GHz 超规格）让 OV5640
     * 只剩 11fps。这套参数对所有分辨率生效：banding 按寄存器里实际的
     * HTS/VTS 动态重算，QVGA（binning，HTS 2060 × VTS 492）同样适用，
     * 理论基准约 30fps。 */
    s_banding_sysclk_hz = 0;
    s_banding_hts = 0;
    s_banding_dirty = false;
    s_theory_fps = 0;
    if (CAM_OV5640_S3_PLL_FIX && sensor->id.PID == OV5640_PID &&
        sensor->xclk_freq_hz == 20000000) {
#if CAM_OV5640_VGA_HIGH_FPS
        bool high_fps = (cfg->res == CAMERA_DRV_RES_VGA);
        const ov5640_pll_params_t *pll = high_fps ? &OV5640_PLL_VGA_40FPS
                                                  : &OV5640_PLL_30FPS;
#else
        const ov5640_pll_params_t *pll = &OV5640_PLL_30FPS;
#endif
        uint32_t sysclk_hz = 0;
        if (ov5640_apply_pll(sensor, pll, &sysclk_hz) != ESP_OK) {
            /* 写了一半的 PLL 不能当没事：恢复上游参数，恢复失败就让本次
             * init 失败，交给上层重试，不把未知状态报成初始化成功。 */
            ESP_LOGW(TAG, "PLL fix failed, restore upstream PLL");
            uint32_t ignored = 0;
            if (ov5640_apply_pll(sensor, &OV5640_PLL_UPSTREAM, &ignored) != ESP_OK) {
                ESP_LOGE(TAG, "PLL restore failed");
                s_sensor = NULL;
                (void)esp_camera_deinit();
                camera_xclk_disable();
                return ESP_FAIL;
            }
        } else {
            /* 不要靠缩短 VTS 提帧率：QVGA 试过 984→492，传感器读出台+JPEG
             * 编码管道吃不下，FB-OVF 后连续取帧超时（实测结论）。想提帧率
             * 只能抬高 SYSCLK，VTS 保持驱动默认 984。 */
            int hts = sensor->get_reg(sensor, 0x380C, 0xFFFF);
            int vts = sensor->get_reg(sensor, 0x380E, 0xFFFF);
            if (hts > 0 && vts > 4) {
                s_banding_sysclk_hz = sysclk_hz;
                s_banding_hts = (uint16_t)hts;
                /* 四舍五入取整：30.33MHz / (2060×984) = 14.96，截断成 14
                 * 会偏低约 7%，四舍五入成 15 才贴近实测。 */
                uint32_t denom = (uint32_t)hts * (uint32_t)vts;
                s_theory_fps = (sysclk_hz + denom / 2u) / denom;
                if (s_theory_fps == 0) {
                    s_theory_fps = 1;
                }
                if (ov5640_write_banding(sensor, sysclk_hz, (uint16_t)hts,
                                         (uint16_t)vts) == ESP_OK) {
                    ESP_LOGI(TAG,
                             "banding recalibrated: hts=%d vts=%d step50=%lu max50=%lu theory=%lufps",
                             hts, vts,
                             (unsigned long)(sysclk_hz / ((uint32_t)hts * 100u)),
                             (unsigned long)(((uint32_t)vts - 4u) /
                                             (sysclk_hz / ((uint32_t)hts * 100u))),
                             (unsigned long)s_theory_fps);
                } else {
                    s_banding_dirty = true;
                    ESP_LOGW(TAG, "banding write failed, retry on next fps change");
                }
#if CAM_OV5640_VGA_HIGH_FPS
                if (high_fps) {
                    /* 40fps 帧长 25ms，不足两个 50Hz 周期（40ms），
                     * AEC 会按防闪烁把曝光量化到单个 10ms 带；hx 在
                     * VGA 高帧率模式关闭 banding，这里保持一致。 */
                    if (sensor->set_reg(sensor, 0x3a00, 0x20, 0x00) != 0) {
                        ESP_LOGW(TAG, "disable banding for high fps failed");
                    }
                }
#endif
            } else {
                ESP_LOGW(TAG, "read HTS/VTS failed (%d/%d), skip banding", hts, vts);
            }
        }
    }
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

    s_cur_quality = cfg->quality;
    s_inited = true;
    return ESP_OK;
}

esp_err_t camera_drv_deinit(void)
{
    if (!s_inited) {
        return ESP_OK;
    }
    /* 趁 SCCB 还活着、XCLK 还在，先让传感器进 standby。 */
    camera_sensor_standby();
    esp_err_t e = esp_camera_deinit();
    /* esp_camera_deinit() 不关 S3 的 XCLK，必须在它之后主动收尾。 */
    camera_xclk_disable();
    s_inited = false;
    s_sensor = NULL;
    s_fps_supported = false;
    s_base_clkrc = s_cur_clkrc = 0;
    s_base_vts = s_cur_vts = 0;
    s_cur_quality = 0;
    s_banding_sysclk_hz = 0;
    s_banding_hts = 0;
    s_banding_dirty = false;
    s_theory_fps = 0;
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

/* 实测校准过的时钟模型：VCO = XCLK / pre_div × multiplier，
 * PLL_CLK = VCO / sys_div / 5，SYSCLK = PLL_CLK / 4。
 * hx 的 calc_sysclk 用 ×2/5，比真实值大了一倍——本机两套 PLL 都验证过：
 * 上游 180/4/2 公式算 22.2fps 实测 11fps，91/1/3 公式算 29.9fps 实测
 * 15fps，都正好是公式的一半。按 /5 模型：上游 PCLK = 1800/4/5/2/4 =
 * 11.25MHz（与上游日志一致），91/1/3 档 PCLK = 606.7/5/2/3 = 20.22MHz
 * （与 hx 注释一致）。pre_div 在本文件只用 2 和 3，表值就是分频值。 */
static uint32_t ov5640_pll_sysclk_hz(uint32_t xclk_hz, const ov5640_pll_params_t *pll)
{
    uint32_t pre_div = pll->pre_div ? pll->pre_div : 1;
    uint32_t vco_hz = xclk_hz / pre_div * pll->multiplier;
    uint32_t pll_clk_hz = vco_hz / pll->sys_div / 5;
    return pll_clk_hz / 4;
}

/* 只改 PLL，HTS/VTS 保持 set_framesize 写好的分辨率时序。 */
static esp_err_t ov5640_apply_pll(sensor_t *sensor,
                                  const ov5640_pll_params_t *pll,
                                  uint32_t *sysclk_hz)
{
    const struct {
        uint16_t reg;
        uint8_t val;
    } regs[] = {
        {0x3039, 0x00},                                  /* PLL bypass 关 */
        {0x3034, 0x1A},                                  /* 10-bit mode */
        {0x3035, 0x01 | ((pll->sys_div & 0x0F) << 4)},   /* sys_div */
        {0x3036, pll->multiplier},                       /* multiplier */
        {0x3037, pll->pre_div & 0x0F},                   /* root_2x 关 */
        {0x3108, ((pll->pclk_root_div & 0x3) << 4) | 0x06},
        {0x3824, pll->pclk_div & 0x1F},
        {0x460C, 0x22},                                  /* pclk_manual */
        {0x3103, 0x13},                                  /* clock from PLL */
    };

    for (size_t i = 0; i < sizeof(regs) / sizeof(regs[0]); i++) {
        if (sensor->set_reg(sensor, regs[i].reg, 0xFF, regs[i].val) != 0) {
            ESP_LOGE(TAG, "OV5640 PLL reg 0x%04x write failed", regs[i].reg);
            return ESP_FAIL;
        }
    }

    *sysclk_hz = ov5640_pll_sysclk_hz(sensor->xclk_freq_hz, pll);
    /* PCLK 只影响 JPEG 流出的总线速率，帧率看 SYSCLK/VTS；上板时用这条
     * 日志核对分频有没有写进去。 */
    uint32_t pclk_hz = (*sysclk_hz * 4u) / pll->pclk_root_div / pll->pclk_div;
    ESP_LOGI(TAG, "OV5640 PLL: VCO %lu MHz, SYSCLK %lu MHz, PCLK %lu.%02lu MHz",
             (unsigned long)(sensor->xclk_freq_hz / (pll->pre_div ? pll->pre_div : 1)
                             * pll->multiplier / 1000000u),
             (unsigned long)(*sysclk_hz / 1000000u),
             (unsigned long)(pclk_hz / 1000000u),
             (unsigned long)((pclk_hz % 1000000u) / 10000u));
    return ESP_OK;
}

/* AEC 的防闪烁步长按行计，依赖 SYSCLK 和 HTS；上游寄存器常量只对应
 * 它自己的 PLL/分辨率组合。换了 PLL 后必须按 hx 的公式重算，否则
 * 50Hz 工频的积分步长错位，画面会出现横纹或曝光被压暗。 */
static esp_err_t ov5640_write_banding(sensor_t *sensor, uint32_t sysclk_hz,
                                      uint16_t hts, uint16_t vts)
{
    if (sysclk_hz == 0 || hts == 0 || vts <= 4) {
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t max_exposure = (uint32_t)vts - 4u;
    uint32_t step_50 = sysclk_hz / ((uint32_t)hts * 100u);
    uint32_t step_60 = sysclk_hz / ((uint32_t)hts * 120u);
    if (step_50 == 0 || step_60 == 0) {
        return ESP_ERR_INVALID_STATE;
    }
    uint32_t max_band_50 = max_exposure / step_50;
    uint32_t max_band_60 = max_exposure / step_60;
    if (max_band_50 < 1) {
        max_band_50 = 1;
    }
    if (max_band_60 < 1) {
        max_band_60 = 1;
    }
    if (max_band_50 > 255) {
        max_band_50 = 255;
    }
    if (max_band_60 > 255) {
        max_band_60 = 255;
    }

    static const uint16_t regs[] = {
        0x3a02, 0x3a03, /* 60Hz 最大曝光 */
        0x3a14, 0x3a15, /* 50Hz 最大曝光 */
        0x3a08, 0x3a09, /* 50Hz 步长 */
        0x3a0a, 0x3a0b, /* 60Hz 步长 */
        0x3a0d,         /* 60Hz 最大步数 */
        0x3a0e,         /* 50Hz 最大步数 */
    };
    const uint8_t vals[] = {
        (uint8_t)(max_exposure >> 8), (uint8_t)(max_exposure & 0xFF),
        (uint8_t)(max_exposure >> 8), (uint8_t)(max_exposure & 0xFF),
        (uint8_t)(step_50 >> 8), (uint8_t)(step_50 & 0xFF),
        (uint8_t)(step_60 >> 8), (uint8_t)(step_60 & 0xFF),
        (uint8_t)max_band_60, (uint8_t)max_band_50,
    };

    for (size_t i = 0; i < sizeof(regs) / sizeof(regs[0]); i++) {
        if (sensor->set_reg(sensor, regs[i], 0xFF, vals[i]) != 0) {
            ESP_LOGE(TAG, "OV5640 banding reg 0x%04x write failed", regs[i]);
            return ESP_FAIL;
        }
    }
    return ESP_OK;
}

/* VTS 改变后 max_exposure 跟着变，band step 不变但最大步数要重算。
 * 失败只告警：banding 不准影响画质，不该把帧率控制整条链停掉。 */
static void ov5640_refresh_banding(uint16_t vts)
{
    if (s_banding_sysclk_hz == 0 || s_banding_hts == 0 || s_sensor == NULL) {
        return;
    }
    if (ov5640_write_banding(s_sensor, s_banding_sysclk_hz, s_banding_hts, vts) != ESP_OK) {
        s_banding_dirty = true;
        ESP_LOGW(TAG, "banding refresh failed at VTS=%u", (unsigned)vts);
    } else {
        s_banding_dirty = false;
    }
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
         * 目标高于基准时保持基准（不缩短到传感器默认以下）。
         * 换算必须用实测基准：PLL 理论 SYSCLK 与真实值有偏差（实测只有
         * 理论的一半），按理论值算出的 VTS 会让实际帧率正好减半，然后
         * base 观测跟着往下掉，形成递减。s_theory_fps 只做日志参考。 */
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
        if (vts == s_cur_vts && !s_banding_dirty) {
            return ESP_OK;
        }
        if (vts != s_cur_vts) {
            int ret = s_sensor->set_reg(s_sensor, 0x380E, 0xFFFF, (int)vts);
            if (ret != 0) {
                ESP_LOGW(TAG, "set VTS=%u failed: %d", (unsigned)vts, ret);
                return ESP_FAIL;
            }
            s_cur_vts = (uint16_t)vts;
        }
        /* 上次 banding 写失败时，即使 VTS 没变也借这次调用重试。 */
        ov5640_refresh_banding(s_cur_vts);
        ESP_LOGD(TAG, "fps %lu -> %lu: VTS=%u",
                 (unsigned long)base_fps, (unsigned long)target_fps, (unsigned)s_cur_vts);
        return ESP_OK;
    }

    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t camera_drv_reset_fps(void)
{
    if (!s_inited || !s_fps_supported || s_sensor == NULL) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    if (s_sensor->id.PID == OV2640_PID) {
        if (s_cur_clkrc == s_base_clkrc) {
            return ESP_OK;
        }
        int ret = s_sensor->set_reg(s_sensor, 0x0111, 0xFF, s_base_clkrc);
        if (ret != 0) {
            ESP_LOGW(TAG, "reset CLKRC failed: %d", ret);
            return ESP_FAIL;
        }
        s_cur_clkrc = s_base_clkrc;
        return ESP_OK;
    }

    if (s_sensor->id.PID == OV5640_PID) {
        if (s_cur_vts == s_base_vts && !s_banding_dirty) {
            return ESP_OK;
        }
        if (s_cur_vts != s_base_vts) {
            int ret = s_sensor->set_reg(s_sensor, 0x380E, 0xFFFF, s_base_vts);
            if (ret != 0) {
                ESP_LOGW(TAG, "reset VTS failed: %d", ret);
                return ESP_FAIL;
            }
            s_cur_vts = s_base_vts;
        }
        ov5640_refresh_banding(s_cur_vts);
        return ESP_OK;
    }

    return ESP_ERR_NOT_SUPPORTED;
}

/* 网络自适应画质用：只改 JPEG 量化表，不重启相机、不动 DMA。
 * quality 数值越大压缩越强、帧越小；调用方负责把它限制在配置范围内。 */
esp_err_t camera_drv_set_quality(uint32_t quality)
{
    if (!s_inited || s_sensor == NULL || s_sensor->set_quality == NULL) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    if (quality < 4 || quality > 63) {
        return ESP_ERR_INVALID_ARG;
    }
    if (quality == s_cur_quality) {
        return ESP_OK;
    }
    if (s_sensor->set_quality(s_sensor, (int)quality) != 0) {
        ESP_LOGW(TAG, "set quality %lu failed", (unsigned long)quality);
        return ESP_FAIL;
    }
    s_cur_quality = quality;
    return ESP_OK;
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
