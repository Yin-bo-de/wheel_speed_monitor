/* ui_render 单测：布局分区（主信息/状态区/断开标记/开机小页面）出图。 */
#include <string.h>

#include "unity.h"
#include "unity_runner.h"
#include "ui_render.h"

void setUp(void) {}
void tearDown(void) {}

static int count_pixels(const uint8_t fb[UI_FB_SIZE], int x0, int y0,
                        int x1, int y1)
{
    int n = 0;
    for (int y = y0; y <= y1; y++) {
        for (int x = x0; x <= x1; x++) {
            if (ui_fb_get_pixel(fb, x, y)) {
                n++;
            }
        }
    }
    return n;
}

static ui_view_t view_ok(void)
{
    ui_view_t v;
    memset(&v, 0, sizeof(v));
    v.speed_valid = true;
    v.speed_kmh = 42.0f;
    v.mode_valid = true;
    v.mode_auto = true;
    v.phase[0] = TELEM_PHASE_LOCKED;
    v.phase[1] = TELEM_PHASE_IDLE;
    v.phase_en[0] = true;
    v.phase_en[1] = true;
    return v;
}

static void test_main_layout(void)
{
    static uint8_t fb[UI_FB_SIZE];
    ui_view_t v = view_ok();
    ui_render_main(&v, fb);

    /* 主信息：大字车速落在 y=12..32、单位在右侧 */
    TEST_ASSERT_TRUE(count_pixels(fb, 2, 12, 60, 32) > 50);
    TEST_ASSERT_TRUE(count_pixels(fb, 60, 22, 110, 28) > 5);
    /* 状态区：模式行 + 前/后差速行 */
    TEST_ASSERT_TRUE(count_pixels(fb, 0, 0, 127, 6) > 5);
    TEST_ASSERT_TRUE(count_pixels(fb, 0, 38, 127, 44) > 5);
    TEST_ASSERT_TRUE(count_pixels(fb, 0, 48, 127, 54) > 5);
}

static void test_main_no_speed_shows_dash(void)
{
    static uint8_t fb[UI_FB_SIZE];
    ui_view_t v = view_ok();
    v.speed_valid = false;
    ui_render_main(&v, fb);
    /* "--" 也出图，但位数少，像素总量明显小于三位数 */
    int dashes = count_pixels(fb, 2, 12, 60, 32);
    TEST_ASSERT_TRUE(dashes > 10);

    v.speed_valid = true;
    v.speed_kmh = 42.0f;
    ui_render_main(&v, fb);
    TEST_ASSERT_TRUE(count_pixels(fb, 2, 12, 60, 32) > dashes);
}

static void test_disconnected_marker(void)
{
    static uint8_t fb[UI_FB_SIZE];
    memset(fb, 0xFF, sizeof(fb)); /* 先弄脏，确保清屏 */
    ui_render_disconnected(fb);

    /* 边框两条横线 + 两条竖线 */
    TEST_ASSERT_EQUAL_INT(122, count_pixels(fb, 3, 18, 124, 18));
    TEST_ASSERT_EQUAL_INT(122, count_pixels(fb, 3, 45, 124, 45));
    TEST_ASSERT_EQUAL_INT(28, count_pixels(fb, 3, 18, 3, 45));
    /* 框外清干净 */
    TEST_ASSERT_EQUAL_INT(0, count_pixels(fb, 0, 0, 127, 17));
    /* 中央有字 */
    TEST_ASSERT_TRUE(count_pixels(fb, 20, 24, 110, 40) > 30);
}

static void test_info_page_lines(void)
{
    static uint8_t fb[UI_FB_SIZE];
    ui_info_t info = {
        .ip = "192.168.2.10",
        .version = "0.1.0",
        .ap_ssid = "oled-setup",
        .ap_ip = "192.168.7.1",
        .diam_mm = 66.0f,
        .disconnect_ms = 2000,
    };
    ui_render_info(&info, fb);
    /* 五行信息各自出图 */
    TEST_ASSERT_TRUE(count_pixels(fb, 0, 4, 127, 10) > 5);
    TEST_ASSERT_TRUE(count_pixels(fb, 0, 16, 127, 22) > 5);
    TEST_ASSERT_TRUE(count_pixels(fb, 0, 28, 127, 34) > 5);
    TEST_ASSERT_TRUE(count_pixels(fb, 0, 40, 127, 46) > 5);
    TEST_ASSERT_TRUE(count_pixels(fb, 0, 52, 127, 58) > 5);
}

static void test_pixel_bounds(void)
{
    static uint8_t fb[UI_FB_SIZE];
    memset(fb, 0xFF, sizeof(fb));
    TEST_ASSERT_TRUE(ui_fb_get_pixel(fb, 0, 0));
    TEST_ASSERT_TRUE(ui_fb_get_pixel(fb, 127, 63));
    TEST_ASSERT_FALSE(ui_fb_get_pixel(fb, -1, 0));
    TEST_ASSERT_FALSE(ui_fb_get_pixel(fb, 128, 0));
    TEST_ASSERT_FALSE(ui_fb_get_pixel(fb, 0, 64));
}

static int max_lit_x(const uint8_t fb[UI_FB_SIZE], int y0, int y1)
{
    int mx = -1;
    for (int y = y0; y <= y1; y++) {
        for (int x = 0; x < UI_FB_W; x++) {
            if (ui_fb_get_pixel(fb, x, y) && x > mx) {
                mx = x;
            }
        }
    }
    return mx;
}

static ui_view_t view_manual(uint16_t target)
{
    ui_view_t v;
    memset(&v, 0, sizeof(v));
    v.speed_valid = true;
    v.speed_kmh = 30.0f;
    v.mode_valid = true;
    v.mode_auto = false;
    v.has_servo_cmd = true;
    v.phase[0] = TELEM_PHASE_IDLE;
    v.phase_en[0] = true;
    v.diff_target_us[0] = target;
    v.diff_unlock_us[0] = 1100;
    v.diff_lock_us[0] = 1900;
    return v;
}

/* 手动模式下 phase 恒 idle，锁定显示靠指令落点（±20µs），
 * 与车端网页的锁定位/解锁位判定同一约定 */
static void test_manual_lock_cmd_shows_lock(void)
{
    static uint8_t fb_lock[UI_FB_SIZE], fb_unlock[UI_FB_SIZE];
    ui_view_t v = view_manual(1895); /* 锁定档 ±20µs 内 */
    ui_render_main(&v, fb_lock);
    v = view_manual(1100); /* 解锁档 */
    ui_render_main(&v, fb_unlock);

    /* "F: LOCK" 7 字宽、"F: UNLOCK" 9 字宽 */
    TEST_ASSERT_TRUE(max_lit_x(fb_lock, 38, 44) <= 45);
    TEST_ASSERT_TRUE(max_lit_x(fb_unlock, 38, 44) >= 50);
    TEST_ASSERT_TRUE(memcmp(fb_lock, fb_unlock, UI_FB_SIZE) != 0);
}

/* 打牙防护扣住锁止命令（pending_sync）时先显示 WAIT，不被落点判定盖掉 */
static void test_manual_pending_sync_wins(void)
{
    static uint8_t fb_lock[UI_FB_SIZE], fb_unlock[UI_FB_SIZE], fb_idle[UI_FB_SIZE];
    ui_view_t v = view_manual(1895);
    v.phase[0] = TELEM_PHASE_PENDING_SYNC;
    ui_render_main(&v, fb_lock);
    v = view_manual(1100); /* 指令落解锁档也一样：pending_sync 优先 */
    v.phase[0] = TELEM_PHASE_PENDING_SYNC;
    ui_render_main(&v, fb_unlock);

    TEST_ASSERT_EQUAL_MEMORY(fb_lock, fb_unlock, UI_FB_SIZE);
    v = view_manual(1895);
    ui_render_main(&v, fb_idle);
    TEST_ASSERT_TRUE(memcmp(fb_lock, fb_idle, UI_FB_SIZE) != 0);
}

NATIVE_TEST_MAIN(
    UnityDefaultTestRun(test_main_layout, "test_main_layout", __LINE__);
    UnityDefaultTestRun(test_main_no_speed_shows_dash, "test_main_no_speed_shows_dash", __LINE__);
    UnityDefaultTestRun(test_disconnected_marker, "test_disconnected_marker", __LINE__);
    UnityDefaultTestRun(test_info_page_lines, "test_info_page_lines", __LINE__);
    UnityDefaultTestRun(test_pixel_bounds, "test_pixel_bounds", __LINE__);
    UnityDefaultTestRun(test_manual_lock_cmd_shows_lock, "test_manual_lock_cmd_shows_lock", __LINE__);
    UnityDefaultTestRun(test_manual_pending_sync_wins, "test_manual_pending_sync_wins", __LINE__);
)
