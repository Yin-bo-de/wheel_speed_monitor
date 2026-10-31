/* cam_rate 自适应帧率控制器行为测试：降速、冷却、恢复、上限跟随。 */
#include "unity.h"
#include "unity_runner.h"
#include "cam_rate.h"

void setUp(void) {}
void tearDown(void) {}

/* 1. 初始值和配置夹紧；空闲时改上限立即生效 */
static void test_defaults_and_cap_clamp(void)
{
    cam_rate_t r;
    cam_rate_init(&r, 15);
    TEST_ASSERT_EQUAL_INT(15, cam_rate_cap(&r));
    TEST_ASSERT_EQUAL_INT(15, cam_rate_effective(&r));

    cam_rate_init(&r, 1);
    TEST_ASSERT_EQUAL_INT(CAM_RATE_FPS_MIN, cam_rate_cap(&r));
    cam_rate_init(&r, 99);
    TEST_ASSERT_EQUAL_INT(CAM_RATE_FPS_MAX, cam_rate_cap(&r));

    cam_rate_init(&r, 15);
    cam_rate_set_cap(&r, 8);
    TEST_ASSERT_EQUAL_INT(8, cam_rate_effective(&r));
    cam_rate_set_cap(&r, 30);
    TEST_ASSERT_EQUAL_INT(CAM_RATE_FPS_MAX, cam_rate_cap(&r));
    TEST_ASSERT_EQUAL_INT(CAM_RATE_FPS_MAX, cam_rate_effective(&r));
}

/* 2. 持续慢发送按 3/4 退让，受冷却时间限制，最低 5fps */
static void test_slow_send_backs_off(void)
{
    cam_rate_t r;
    cam_rate_init(&r, 20);
    cam_rate_stream_start(&r, 0);

    /* 冷却期内不连续降 */
    TEST_ASSERT_EQUAL_INT(20, cam_rate_report_send(&r, 300, 1000));
    /* 满 2 秒，20 → 15 */
    TEST_ASSERT_EQUAL_INT(15, cam_rate_report_send(&r, 300, 2000));
    TEST_ASSERT_EQUAL_INT(15, cam_rate_report_send(&r, 300, 3000));
    /* 15 → 11 → 8 → 6 → 5，然后封底 */
    TEST_ASSERT_EQUAL_INT(11, cam_rate_report_send(&r, 300, 4000));
    TEST_ASSERT_EQUAL_INT(8, cam_rate_report_send(&r, 300, 6000));
    TEST_ASSERT_EQUAL_INT(6, cam_rate_report_send(&r, 300, 8000));
    TEST_ASSERT_EQUAL_INT(5, cam_rate_report_send(&r, 300, 10000));
    TEST_ASSERT_EQUAL_INT(5, cam_rate_report_send(&r, 300, 12000));
}

/* 3. 阻塞超过 1 秒直接砍半，而不是 3/4 */
static void test_long_block_halves(void)
{
    cam_rate_t r;
    cam_rate_init(&r, 20);
    cam_rate_stream_start(&r, 0);
    TEST_ASSERT_EQUAL_INT(10, cam_rate_report_send(&r, 1200, 2000));
}

/* 4. 连续快速发送 30 帧且满 3 秒，每次加 1fps，封顶用户配置 */
static void test_fast_send_recovers(void)
{
    cam_rate_t r;
    cam_rate_init(&r, 10);
    cam_rate_stream_start(&r, 0);
    TEST_ASSERT_EQUAL_INT(7, cam_rate_report_send(&r, 300, 2000));

    uint32_t eff = 7;
    for (int i = 0; i < CAM_RATE_FAST_STREAK; i++) {
        eff = cam_rate_report_send(&r, 20, 5000);
    }
    TEST_ASSERT_EQUAL_INT(8, eff);
    TEST_ASSERT_EQUAL_INT(8, cam_rate_effective(&r));

    for (int i = 0; i < CAM_RATE_FAST_STREAK; i++) {
        eff = cam_rate_report_send(&r, 20, 8000);
    }
    TEST_ASSERT_EQUAL_INT(9, eff);

    for (int i = 0; i < CAM_RATE_FAST_STREAK; i++) {
        eff = cam_rate_report_send(&r, 20, 11000);
    }
    TEST_ASSERT_EQUAL_INT(10, eff);
    /* 已到用户上限，继续快速发送不再涨 */
    for (int i = 0; i < CAM_RATE_FAST_STREAK; i++) {
        eff = cam_rate_report_send(&r, 20, 14000);
    }
    TEST_ASSERT_EQUAL_INT(10, eff);
}

/* 5. 中等耗时会把连续快速计数清零，避免慢网络里误判恢复 */
static void test_medium_send_breaks_fast_streak(void)
{
    cam_rate_t r;
    cam_rate_init(&r, 10);
    cam_rate_stream_start(&r, 0);
    TEST_ASSERT_EQUAL_INT(7, cam_rate_report_send(&r, 300, 2000));

    for (int i = 0; i < CAM_RATE_FAST_STREAK - 1; i++) {
        cam_rate_report_send(&r, 20, 5000);
    }
    cam_rate_report_send(&r, 120, 5000); /* 中等耗时，清零 */
    uint32_t eff = cam_rate_report_send(&r, 20, 5000);
    TEST_ASSERT_EQUAL_INT(7, eff);
}

/* 6. 流进行中改上限：低于当前立即降，高于当前等恢复；断流保留，deinit 回上限 */
static void test_cap_change_during_stream(void)
{
    cam_rate_t r;
    cam_rate_init(&r, 15);
    cam_rate_stream_start(&r, 0);
    TEST_ASSERT_EQUAL_INT(11, cam_rate_report_send(&r, 300, 2000));

    cam_rate_set_cap(&r, 20);
    TEST_ASSERT_EQUAL_INT(20, cam_rate_cap(&r));
    TEST_ASSERT_EQUAL_INT(11, cam_rate_effective(&r));

    cam_rate_set_cap(&r, 8);
    TEST_ASSERT_EQUAL_INT(8, cam_rate_effective(&r));

    cam_rate_set_cap(&r, 20);
    TEST_ASSERT_EQUAL_INT(8, cam_rate_effective(&r));
    cam_rate_stream_stop(&r, 3000);
    /* 流结束保留降速结果，宽限期内不重新冲高 */
    TEST_ASSERT_EQUAL_INT(8, cam_rate_effective(&r));
    cam_rate_stream_start(&r, 3100);
    TEST_ASSERT_EQUAL_INT(8, cam_rate_effective(&r));
    /* 相机 deinit：先停流，再用同一个 cap 调 set_cap 恢复上限 */
    cam_rate_stream_stop(&r, 3200);
    cam_rate_set_cap(&r, 20);
    TEST_ASSERT_EQUAL_INT(20, cam_rate_effective(&r));
}

NATIVE_TEST_MAIN(
    UnityDefaultTestRun(test_defaults_and_cap_clamp, "test_defaults_and_cap_clamp", __LINE__);
    UnityDefaultTestRun(test_slow_send_backs_off, "test_slow_send_backs_off", __LINE__);
    UnityDefaultTestRun(test_long_block_halves, "test_long_block_halves", __LINE__);
    UnityDefaultTestRun(test_fast_send_recovers, "test_fast_send_recovers", __LINE__);
    UnityDefaultTestRun(test_medium_send_breaks_fast_streak, "test_medium_send_breaks_fast_streak", __LINE__);
    UnityDefaultTestRun(test_cap_change_during_stream, "test_cap_change_during_stream", __LINE__);
)
