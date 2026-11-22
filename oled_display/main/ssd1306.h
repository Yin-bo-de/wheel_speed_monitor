/* ssd1306：0.96" OLED（SSD1306 128×64，I2C 0x3C）薄驱动（IDF 依赖，不宿主机测）。
 * 接线：SCL=GPIO6、SDA=GPIO7、VCC=3V3、GND。模块自带上拉，直连。
 * 二期 MPU6050（0x68）与屏共用这条 I2C 总线：总线句柄在本模块创建，
 * 到时候把 device 挂到同一 bus 上即可，接线按共总线规划留位。 */
#ifndef SSD1306_DRIVER_H
#define SSD1306_DRIVER_H

#include <stdint.h>

#include "esp_err.h"

#define SSD1306_W  128
#define SSD1306_H  64
#define SSD1306_FB_SIZE (SSD1306_W * SSD1306_H / 8)

/* 初始化 I2C 总线 + 屏幕（可重复调用，只初始化一次） */
esp_err_t ssd1306_init(void);

/* 整屏刷新：fb 为 ui_render 的纵向寻址帧缓冲 */
esp_err_t ssd1306_write_fb(const uint8_t fb[SSD1306_FB_SIZE]);

#endif /* SSD1306_DRIVER_H */
