/* cam_rate 自适应帧率控制器行为测试：启动保护、1 秒窗口、降速、冷却、恢复。 */
#include "unity.h"
#include "unity_runner.h"
#include "cam_rate.h"

void setUp(void) {}
void tearDown(void) {}

/* 在一个 1 秒窗口内每 100ms 上报一帧，返回窗口结算后的 effective。 */
static uint32_t report_window(cam_rate_t *r, uint32_t send_ms,
                              uint32_t start_ms, uint32_t end_ms)
{
    for (uint32_t t = start_ms; t < end_ms; t += 100) {
        (void)cam_rate_report_send(r, send_ms, t);
    }
    return cam_rate_report_send(r, send_ms, end_ms);
}

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

/* 2. 启动前 5 秒不降速；之后按 1 秒窗口平均，持续慢发送按 3/4 退让 */
static void test_startup_grace_and_slow_backoff(void)
{
    cam_rate_t r;
    cam_rate_init(&r, 20);
    cam_rate_stream_start(&r, 0);

    /* 保护期内单帧再慢也不降 */
    TEST_ASSERT_EQUAL_INT(20, cam_rate_report_send(&r, 300, 1000));
    TEST_ASSERT_EQUAL_INT(20, cam_rate_report_send(&r, 300, 4000));
    TEST_ASSERT_EQUAL_INT(20, cam_rate_report_send(&r, 300, 4999));

    /* t=6000 结算第一个窗口，平均 300ms 超过 25fps 的 100ms 阈值 */
    TEST_ASSERT_EQUAL_INT(15, report_window(&r, 300, 5000, 6000));
    /* 冷却 2 秒：t=7000 的窗口结算不降 */
    TEST_ASSERT_EQUAL_INT(15, report_window(&r, 300, 6000, 7000));
    /* t=8000 冷却满足，15 → 11 */
    TEST_ASSERT_EQUAL_INT(11, report_window(&r, 300, 7000, 8000));
    /* 后面每 2 秒降一档，最低 5fps */
    TEST_ASSERT_EQUAL_INT(11, report_window(&r, 300, 8000, 9000));
    TEST_ASSERT_EQUAL_INT(8, report_window(&r, 300, 9000, 10000));
    TEST_ASSERT_EQUAL_INT(8, report_window(&r, 300, 10000, 11000));
    TEST_ASSERT_EQUAL_INT(6, report_window(&r, 300, 11000, 12000));
    TEST_ASSERT_EQUAL_INT(6, report_window(&r, 300, 12000, 13000));
    TEST_ASSERT_EQUAL_INT(5, report_window(&r, 300, 13000, 14000));
    TEST_ASSERT_EQUAL_INT(5, report_window(&r, 300, 14000, 15000));
}

/* 3. 单帧长尾被 1 秒窗口平均掉，不触发降速 */
static void test_single_spike_is_averaged_out(void)
{
    cam_rate_t r;
    cam_rate_init(&r, 20);
    cam_rate_stream_start(&r, 0);

    /* 窗口内 10 帧 20ms + 最后 1 帧 300ms，平均约 45ms，低于 100ms 阈值 */
    for (uint32_t t = 5000; t < 6000; t += 100) {
        (void)cam_rate_report_send(&r, 20, t);
    }
    TEST_ASSERT_EQUAL_INT(20, cam_rate_report_send(&r, 300, 6000));
    TEST_ASSERT_EQUAL_INT(1, r.fast_streak);
}

/* 4. 整窗平均超过 1 秒直接砍半 */
static void test_long_block_halves(void)
{
    cam_rate_t r;
    cam_rate_init(&r, 20);
    cam_rate_stream_start(&r, 0);

    TEST_ASSERT_EQUAL_INT(20, cam_rate_report_send(&r, 1200, 5000));
    TEST_ASSERT_EQUAL_INT(10, cam_rate_report_send(&r, 1200, 6000));
}

/* 5. 连续 3 个快窗口且满 3 秒，每次加 1fps，封顶用户配置 */
static void test_fast_windows_recover(void)
{
    cam_rate_t r;
    cam_rate_init(&r, 10);
    cam_rate_stream_start(&r, 0);
    TEST_ASSERT_EQUAL_INT(7, report_window(&r, 300, 5000, 6000));

    TEST_ASSERT_EQUAL_INT(7, report_window(&r, 20, 6000, 7000));
    TEST_ASSERT_EQUAL_INT(7, report_window(&r, 20, 7000, 8000));
    TEST_ASSERT_EQUAL_INT(8, report_window(&r, 20, 8000, 9000));

    TEST_ASSERT_EQUAL_INT(8, report_window(&r, 20, 9000, 10000));
    TEST_ASSERT_EQUAL_INT(8, report_window(&r, 20, 10000, 11000));
    TEST_ASSERT_EQUAL_INT(9, report_window(&r, 20, 11000, 12000));

    TEST_ASSERT_EQUAL_INT(9, report_window(&r, 20, 12000, 13000));
    TEST_ASSERT_EQUAL_INT(9, report_window(&r, 20, 13000, 14000));
    TEST_ASSERT_EQUAL_INT(10, report_window(&r, 20, 14000, 15000));
    /* 已到用户上限，继续快速发送不再涨 */
    TEST_ASSERT_EQUAL_INT(10, report_window(&r, 20, 15000, 16000));
}

/* 6. 中等耗时的窗口会把连续快速计数清零，避免慢网络里误判恢复 */
static void test_medium_window_breaks_fast_streak(void)
{
    cam_rate_t r;
    cam_rate_init(&r, 10);
    cam_rate_stream_start(&r, 0);
    TEST_ASSERT_EQUAL_INT(7, report_window(&r, 300, 5000, 6000));

    TEST_ASSERT_EQUAL_INT(7, report_window(&r, 20, 6000, 7000));
    TEST_ASSERT_EQUAL_INT(1, r.fast_streak);
    /* 120ms 在 60ms 快阈值和当前慢阈值之间，清零 */
    TEST_ASSERT_EQUAL_INT(7, report_window(&r, 120, 7000, 8000));
    TEST_ASSERT_EQUAL_INT(0, r.fast_streak);
    TEST_ASSERT_EQUAL_INT(7, report_window(&r, 20, 8000, 9000));
    TEST_ASSERT_EQUAL_INT(1, r.fast_streak);
}

/* 7. 窗口重置只丢未结算样本，不动 effective 和冷却状态 */
static void test_reset_window_drops_pending_samples(void)
{
    cam_rate_t r;
    cam_rate_init(&r, 10);
    cam_rate_stream_start(&r, 0);
    TEST_ASSERT_EQUAL_INT(7, report_window(&r, 300, 5000, 6000));

    /* 窗口里先塞慢样本，未结算就重置 */
    (void)cam_rate_report_send(&r, 300, 6000);
    (void)cam_rate_report_send(&r, 300, 6500);
    TEST_ASSERT_TRUE(cam_rate_avg_valid(&r));
    cam_rate_reset_window(&r);
    TEST_ASSERT_EQUAL_INT(7, cam_rate_effective(&r));
    TEST_ASSERT_EQUAL_INT(0, r.win_frames);
    TEST_ASSERT_EQUAL_INT(0, r.fast_streak);
    TEST_ASSERT_FALSE(cam_rate_avg_valid(&r));

    /* 重置后从快帧重新开始，不会带着旧慢样本降速 */
    TEST_ASSERT_EQUAL_INT(7, report_window(&r, 20, 6600, 7700));
    TEST_ASSERT_EQUAL_INT(1, r.fast_streak);
}

/* 8. 流进行中改上限：低于当前立即降，高于当前等恢复；断流保留，deinit 回上限 */
static void test_cap_change_during_stream(void)
{
    cam_rate_t r;
    cam_rate_init(&r, 15);
    cam_rate_stream_start(&r, 0);
    TEST_ASSERT_EQUAL_INT(11, report_window(&r, 300, 5000, 6000));

    cam_rate_set_cap(&r, 20);
    TEST_ASSERT_EQUAL_INT(20, cam_rate_cap(&r));
    TEST_ASSERT_EQUAL_INT(11, cam_rate_effective(&r));

    cam_rate_set_cap(&r, 8);
    TEST_ASSERT_EQUAL_INT(8, cam_rate_effective(&r));

    cam_rate_set_cap(&r, 20);
    TEST_ASSERT_EQUAL_INT(8, cam_rate_effective(&r));
    cam_rate_stream_stop(&r, 7000);
    /* 流结束保留降速结果，宽限期内不重新冲高 */
    TEST_ASSERT_EQUAL_INT(8, cam_rate_effective(&r));
    cam_rate_stream_start(&r, 7100);
    TEST_ASSERT_EQUAL_INT(8, cam_rate_effective(&r));
    /* 相机 deinit：先停流，再用同一个 cap 调 set_cap 恢复上限 */
    cam_rate_stream_stop(&r, 7200);
    cam_rate_set_cap(&r, 20);
    TEST_ASSERT_EQUAL_INT(20, cam_rate_effective(&r));
}

/* 9. 质量默认等于用户值；断流恢复用户值 */
static void test_quality_defaults_and_restore(void)
{
    cam_quality_t q;
    cam_quality_init(&q, 12);
    TEST_ASSERT_EQUAL_INT(12, q.effective);
    TEST_ASSERT_FALSE(cam_quality_restore(&q));

    TEST_ASSERT_TRUE(cam_quality_evaluate(&q, true, true, 5, 20, 250));
    TEST_ASSERT_EQUAL_INT(14, q.effective);
    TEST_ASSERT_TRUE(cam_quality_restore(&q));
    TEST_ASSERT_EQUAL_INT(12, q.effective);
}

/* 10. 帧率到下限仍发不完：每次加 2，最多比用户值高 MAX_DELTA */
static void test_quality_backs_off_to_limit(void)
{
    cam_quality_t q;
    cam_quality_init(&q, 12);
    for (int i = 0; i < 10; i++) {
        (void)cam_quality_evaluate(&q, true, true, 5, 20, 300);
    }
    TEST_ASSERT_EQUAL_INT(12 + CAM_QUALITY_MAX_DELTA, q.effective);
    TEST_ASSERT_FALSE(cam_quality_evaluate(&q, true, true, 5, 20, 300));
}

/* 11. 网络恢复要连续两个快窗口才还一档，中间窗口打断计数 */
static void test_quality_recovers_after_two_fast_windows(void)
{
    cam_quality_t q;
    cam_quality_init(&q, 12);
    TEST_ASSERT_TRUE(cam_quality_evaluate(&q, true, true, 5, 20, 300));
    TEST_ASSERT_TRUE(cam_quality_evaluate(&q, true, true, 5, 20, 300));
    TEST_ASSERT_EQUAL_INT(16, q.effective);

    TEST_ASSERT_FALSE(cam_quality_evaluate(&q, true, true, 20, 20, 20));
    TEST_ASSERT_EQUAL_INT(16, q.effective);
    TEST_ASSERT_FALSE(cam_quality_evaluate(&q, true, true, 20, 20, 120));
    TEST_ASSERT_EQUAL_INT(0, q.fast_windows);

    TEST_ASSERT_FALSE(cam_quality_evaluate(&q, true, true, 20, 20, 20));
    TEST_ASSERT_TRUE(cam_quality_evaluate(&q, true, true, 20, 20, 20));
    TEST_ASSERT_EQUAL_INT(14, q.effective);
}

/* 12. 用户值接近传感器上限时，自适应不越过 63 */
static void test_quality_clamps_at_sensor_max(void)
{
    cam_quality_t q;
    cam_quality_init(&q, 60);
    TEST_ASSERT_TRUE(cam_quality_evaluate(&q, true, true, 5, 20, 300));
    TEST_ASSERT_TRUE(cam_quality_evaluate(&q, true, true, 5, 20, 300));
    TEST_ASSERT_EQUAL_INT(CAM_QUALITY_MAX, q.effective);
    TEST_ASSERT_FALSE(cam_quality_evaluate(&q, true, true, 5, 20, 300));
}

/* 13. 没有流或没有发送样本时不动画质 */
static void test_quality_ignores_idle_or_no_samples(void)
{
    cam_quality_t q;
    cam_quality_init(&q, 12);
    TEST_ASSERT_FALSE(cam_quality_evaluate(&q, false, false, 5, 20, 300));
    TEST_ASSERT_FALSE(cam_quality_evaluate(&q, true, false, 5, 20, 0));
    TEST_ASSERT_EQUAL_INT(12, q.effective);
}

/* 14. 平均 0ms 是有效的快速样本，画质必须能恢复；不能当成"没有样本" */
static void test_quality_zero_avg_is_valid_fast_sample(void)
{
    cam_quality_t q;
    cam_quality_init(&q, 12);
    TEST_ASSERT_TRUE(cam_quality_evaluate(&q, true, true, 5, 20, 300));
    TEST_ASSERT_TRUE(cam_quality_evaluate(&q, true, true, 5, 20, 300));
    TEST_ASSERT_EQUAL_INT(16, q.effective);

    TEST_ASSERT_FALSE(cam_quality_evaluate(&q, true, true, 20, 20, 0));
    TEST_ASSERT_TRUE(cam_quality_evaluate(&q, true, true, 20, 20, 0));
    TEST_ASSERT_EQUAL_INT(14, q.effective);
}

NATIVE_TEST_MAIN(
    UnityDefaultTestRun(test_defaults_and_cap_clamp, "test_defaults_and_cap_clamp", __LINE__);
    UnityDefaultTestRun(test_startup_grace_and_slow_backoff, "test_startup_grace_and_slow_backoff", __LINE__);
    UnityDefaultTestRun(test_single_spike_is_averaged_out, "test_single_spike_is_averaged_out", __LINE__);
    UnityDefaultTestRun(test_long_block_halves, "test_long_block_halves", __LINE__);
    UnityDefaultTestRun(test_fast_windows_recover, "test_fast_windows_recover", __LINE__);
    UnityDefaultTestRun(test_medium_window_breaks_fast_streak, "test_medium_window_breaks_fast_streak", __LINE__);
    UnityDefaultTestRun(test_reset_window_drops_pending_samples, "test_reset_window_drops_pending_samples", __LINE__);
    UnityDefaultTestRun(test_cap_change_during_stream, "test_cap_change_during_stream", __LINE__);
    UnityDefaultTestRun(test_quality_defaults_and_restore, "test_quality_defaults_and_restore", __LINE__);
    UnityDefaultTestRun(test_quality_backs_off_to_limit, "test_quality_backs_off_to_limit", __LINE__);
    UnityDefaultTestRun(test_quality_recovers_after_two_fast_windows, "test_quality_recovers_after_two_fast_windows", __LINE__);
    UnityDefaultTestRun(test_quality_clamps_at_sensor_max, "test_quality_clamps_at_sensor_max", __LINE__);
    UnityDefaultTestRun(test_quality_ignores_idle_or_no_samples, "test_quality_ignores_idle_or_no_samples", __LINE__);
    UnityDefaultTestRun(test_quality_zero_avg_is_valid_fast_sample, "test_quality_zero_avg_is_valid_fast_sample", __LINE__);
)
