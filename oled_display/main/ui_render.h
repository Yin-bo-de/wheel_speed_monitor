/* ui_render：OLED 显示布局与字形渲染（纯 C，不依赖 IDF，宿主机可测）。
 * 输出 SSD1306 GDDRAM 帧缓冲（128×64 单色，纵向寻址）：
 * 像素 (x,y) = fb[(y/8)*128 + x] 的第 (y%8) 位。
 *
 * 内容分区（需求方案「显示规格」）：
 * - 主信息：车速大字，一眼可读；
 * - 状态区：模式 + 前/后差速四态，小字不抢主信息；
 * - 异常态：清屏只留断开标记；
 * - 开机小页面：IP / 版本 / 配置摘要，供查地址用。 */
#ifndef UI_RENDER_H
#define UI_RENDER_H

#include <stdbool.h>
#include <stdint.h>

#include "telem_frame.h"

#define UI_FB_W    128
#define UI_FB_H    64
#define UI_FB_SIZE (UI_FB_W * UI_FB_H / 8)

typedef struct {
    bool speed_valid; /* false = 无有效轮，车速显示 "--" */
    float speed_kmh;
    bool mode_valid;
    bool mode_auto;
    telem_phase_t phase[TELEM_DIFF_COUNT];
    bool phase_en[TELEM_DIFF_COUNT];
} ui_view_t;

typedef struct {
    const char *ip;   /* STA IP，NULL = 尚未拿到 */
    const char *version;
    const char *ap_ssid; /* 配置热点名：手机直连的入口 */
    const char *ap_ip;   /* 配置热点固定地址 */
    float diam_mm;
    uint32_t disconnect_ms;
} ui_info_t;

void ui_render_main(const ui_view_t *v, uint8_t fb[UI_FB_SIZE]);
void ui_render_disconnected(uint8_t fb[UI_FB_SIZE]);
void ui_render_info(const ui_info_t *info, uint8_t fb[UI_FB_SIZE]);

/* 像素读取（宿主机测试用）：越界返回 false */
bool ui_fb_get_pixel(const uint8_t fb[UI_FB_SIZE], int x, int y);

#endif /* UI_RENDER_H */
