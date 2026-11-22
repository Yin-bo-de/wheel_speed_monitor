/* ssd1306 实现：见头文件注释。 */
#include "ssd1306.h"

#include <string.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_check.h"
#include "esp_log.h"

static const char *TAG = "ssd1306";

/* 引脚选型（需求方案「硬件选型与接线」）：避开 GPIO18/19（USB）、
 * GPIO20/21（UART0 console）、GPIO2/8/9（strapping）。C3 的 I2C 走
 * GPIO 矩阵任意脚可映射，6/7 是无冲突的顺手选择。 */
#define SSD1306_SCL_GPIO GPIO_NUM_6
#define SSD1306_SDA_GPIO GPIO_NUM_7
#define SSD1306_ADDR     0x3C

static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_dev;
static bool s_inited;

/* 4KB 静态发送缓冲：1025 字节整帧一次下发，放栈上会爆任务栈 */
static uint8_t s_tx[SSD1306_FB_SIZE + 1];

static esp_err_t send_cmds(const uint8_t *cmds, size_t len)
{
    /* Co=0, D/C#=0：后面全是命令 */
    s_tx[0] = 0x00;
    memcpy(s_tx + 1, cmds, len);
    return i2c_master_transmit(s_dev, s_tx, len + 1, 1000);
}

esp_err_t ssd1306_init(void)
{
    if (s_inited) {
        return ESP_OK;
    }

    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = -1,
        .sda_io_num = SSD1306_SDA_GPIO,
        .scl_io_num = SSD1306_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        /* 模块自带上拉，这里开内部上拉只是兜底（约 45k，弱） */
        .flags.enable_internal_pullup = true,
    };
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_cfg, &s_bus), TAG, "i2c bus");

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SSD1306_ADDR,
        .scl_speed_hz = 400000,
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(s_bus, &dev_cfg, &s_dev),
                        TAG, "i2c dev");

    /* SSD1306 标准初始化序列：电荷泵、128×64、水平翻转（按模组常见排线）、
     * 段/列重映射后原点在左上 */
    static const uint8_t init_cmds[] = {
        0xAE,             /* display off */
        0xD5, 0x80,       /* clock divide */
        0xA8, 0x3F,       /* multiplex = 64-1 */
        0xD3, 0x00,       /* display offset */
        0x40,             /* start line 0 */
        0x8D, 0x14,       /* charge pump on */
        0x20, 0x00,       /* horizontal addressing */
        0xA1,             /* segment remap */
        0xC8,             /* COM scan dec */
        0xDA, 0x12,       /* COM pins config */
        0x81, 0x7F,       /* contrast */
        0xD9, 0xF1,       /* precharge */
        0xDB, 0x40,       /* VCOMH */
        0xA4,             /* display follows RAM */
        0xA6,             /* normal (not inverted) */
        0xAF,             /* display on */
    };
    esp_err_t e = send_cmds(init_cmds, sizeof(init_cmds));
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "init cmds failed: %s", esp_err_to_name(e));
        return e;
    }
    s_inited = true;
    ESP_LOGI(TAG, "SSD1306 ready SCL=%d SDA=%d addr=0x%02X",
             SSD1306_SCL_GPIO, SSD1306_SDA_GPIO, SSD1306_ADDR);
    return ESP_OK;
}

esp_err_t ssd1306_write_fb(const uint8_t fb[SSD1306_FB_SIZE])
{
    if (!s_inited) {
        return ESP_ERR_INVALID_STATE;
    }
    /* 列 0..127、页 0..7 的整屏窗口，然后一次性灌整帧 */
    static const uint8_t window_cmds[] = {0x21, 0x00, 0x7F, 0x22, 0x00, 0x07};
    ESP_RETURN_ON_ERROR(send_cmds(window_cmds, sizeof(window_cmds)),
                        TAG, "window");
    s_tx[0] = 0x40; /* Co=0, D/C#=1：后面全是数据 */
    memcpy(s_tx + 1, fb, SSD1306_FB_SIZE);
    return i2c_master_transmit(s_dev, s_tx, SSD1306_FB_SIZE + 1, 1000);
}
