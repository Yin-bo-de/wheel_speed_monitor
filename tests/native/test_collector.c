/* wheel_speed_collector 行为测试：计数差值、16 位回绕/复位判定、累计、换源重置、去抖、
 * 脉冲间隔测频。 */
#include <math.h>
#include <string.h>
#include "unity.h"
#include "unity_runner.h"
#include "wheel_speed_collector.h"

void setUp(void) {}
void tearDown(void) {}

static wheel_chan_state_t st;
static wheel_chan_cfg_t cfg;

static void fresh(void)
{
    memset(&st, 0, sizeof(st));
    memset(&cfg, 0, sizeof(cfg));
    cfg.window_ms = 50;
    cfg.magnets = 1;
    cfg.wheel_diam_mm = 100;
    cfg.alpha = 0.3f;
    cfg.debounce_ms = 0;
}

/* 1. 正常递增：每窗口增量正确反映到累计脉冲 */
static void test_normal_increment(void)
{
    fresh();
    /* 首窗口只建立基准 */
    wheel_chan_sample(&st, &cfg, 100, 1000);
    TEST_ASSERT_EQUAL_UINT32(0, st.pulses_total);
    /* 100→103→106，各 3 脉冲 */
    wheel_chan_sample(&st, &cfg, 103, 1050);
    TEST_ASSERT_EQUAL_UINT32(3, st.pulses_total);
    wheel_chan_sample(&st, &cfg, 106, 1100);
    TEST_ASSERT_EQUAL_UINT32(6, st.pulses_total);
}

/* 2. 回绕：65000→100 判定为回绕，本窗口脉冲 = 65536−65000+100 = 636 */
static void test_wraparound(void)
{
    fresh();
    wheel_chan_sample(&st, &cfg, 65000, 1000);
    wheel_chan_sample(&st, &cfg, 100, 1050);
    TEST_ASSERT_EQUAL_UINT32(636, st.pulses_total);
}

/* 3. 清零复位：300→5 且旧值未过回绕阈值 → 本窗口 0 脉冲，计数基准重置为 5 */
static void test_clear_reset(void)
{
    fresh();
    wheel_chan_sample(&st, &cfg, 300, 1000);
    wheel_chan_sample(&st, &cfg, 5, 1050);
    TEST_ASSERT_EQUAL_UINT32(0, st.pulses_total);
    /* 复位后从 5 继续：5→10 增量 5 */
    wheel_chan_sample(&st, &cfg, 10, 1100);
    TEST_ASSERT_EQUAL_UINT32(5, st.pulses_total);
}

/* 4. 全零序列：RPM=0、无触发、无错误 */
static void test_all_zero_sequence(void)
{
    fresh();
    for (int i = 0; i < 4; i++) {
        TEST_ASSERT_EQUAL_INT(ESP_OK, wheel_chan_sample(&st, &cfg, 0, 1000 + i * 50));
    }
    TEST_ASSERT_TRUE(st.trigger == false);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, st.math.rpm_ema);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, st.math.freq_hz);
}

/* 5. 链路正确性：每窗口 1 脉冲、1 磁铁、50ms → 20Hz → 1200 RPM */
static void test_pipeline_correctness(void)
{
    fresh();
    /* 建立基准 */
    wheel_chan_sample(&st, &cfg, 0, 1000);
    for (int i = 1; i <= 10; i++) {
        wheel_chan_sample(&st, &cfg, i, 1000 + i * 50);
    }
    TEST_ASSERT_TRUE(st.trigger);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 20.0f, st.math.freq_hz);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 1200.0f, st.math.rpm_ema);
}

/* 6. 累计脉冲跨多次回绕单调递增不丢数 */
static void test_pulses_monotonic_across_wraps(void)
{
    fresh();
    wheel_chan_sample(&st, &cfg, 65000, 1000);
    wheel_chan_sample(&st, &cfg, 100, 1050);   /* 回绕 +636 */
    wheel_chan_sample(&st, &cfg, 200, 1100);   /* +100 */
    wheel_chan_sample(&st, &cfg, 65000, 1150); /* +64800 */
    wheel_chan_sample(&st, &cfg, 50, 1200);    /* 回绕 +586 */
    TEST_ASSERT_EQUAL_UINT32(636 + 100 + 64800 + 586, st.pulses_total);
}

/* 7. 换源基准重置：reset_baseline 后首窗口不产生巨大差值 */
static void test_reset_baseline_no_glitch(void)
{
    fresh();
    /* 先跑到 65000，模拟旧源高位计数 */
    wheel_chan_sample(&st, &cfg, 65000, 1000);
    /* 切换源：新源计数从 0 开始——先重置基准再喂新计数 */
    wheel_chan_reset_baseline(&st, 0);
    TEST_ASSERT_EQUAL_UINT32(0, st.pulses_total);
    /* 新源 0→2：正常 2 脉冲，而不是把 0 误判为回绕或复位清零 */
    wheel_chan_sample(&st, &cfg, 2, 1050);
    TEST_ASSERT_EQUAL_UINT32(2, st.pulses_total);
}

/* 8. 去抖：默认 0 关闭时脉冲全部计数；设 20ms 时同窗口第二脉冲被丢弃 */
static void test_debounce_zero_counts_all(void)
{
    fresh();
    /* 100→103（3 脉冲，无去抖） */
    wheel_chan_sample(&st, &cfg, 100, 1000);
    wheel_chan_sample(&st, &cfg, 103, 1050);
    TEST_ASSERT_EQUAL_UINT32(3, st.pulses_total);
}

static void test_debounce_drops_second_pulse(void)
{
    fresh();
    cfg.debounce_ms = 20;
    /* 同一 50ms 窗口内出现 2 个计数（磁铁双脉冲），去抖后只记 1 */
    wheel_chan_sample(&st, &cfg, 100, 1000);
    wheel_chan_sample(&st, &cfg, 102, 1050);
    TEST_ASSERT_EQUAL_UINT32(1, st.pulses_total);
}

/* 9. reset 清零累计（前端 reset_counts 命令） */
static void test_reset_counts(void)
{
    fresh();
    wheel_chan_sample(&st, &cfg, 100, 1000);
    wheel_chan_sample(&st, &cfg, 110, 1050);
    TEST_ASSERT_EQUAL_UINT32(10, st.pulses_total);
    wheel_chan_reset_counts(&st);
    TEST_ASSERT_EQUAL_UINT32(0, st.pulses_total);
    /* 重置基准为当前计数，后续增量正常 */
    wheel_chan_sample(&st, &cfg, 115, 1100);
    TEST_ASSERT_EQUAL_UINT32(5, st.pulses_total);
}

/* 10. 脉冲间隔测频法：间隔 1000000us（1s）→ 频率 1Hz → RPS 1；实时非窗口放大 */
static void test_period_measurement_freq(void)
{
    fresh();
    wheel_chan_sample_period(&st, &cfg, 1000000u, 1000);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 1.0f, st.math.freq_hz);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 60.0f, st.math.rpm_ema);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 1.0f, st.math.rpm_ema / 60.0f);
}

/* 11. 快速脉冲间隔（100ms → 10Hz）→ 10 RPS，窗口法在单窗口会误报 20Hz */
static void test_period_measurement_fast(void)
{
    fresh();
    wheel_chan_sample_period(&st, &cfg, 100000u, 1000);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 10.0f, st.math.freq_hz);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 600.0f, st.math.rpm_ema);
}

/* 12. 间隔为零（中断异常/未检测）→ 返回错误且不产生 NaN */
static void test_period_zero_interval(void)
{
    fresh();
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, wheel_chan_sample_period(&st, &cfg, 0, 1000));
    TEST_ASSERT_TRUE(isfinite(st.math.freq_hz));
}

NATIVE_TEST_MAIN(
    UnityDefaultTestRun(test_normal_increment, "test_normal_increment", __LINE__);
    UnityDefaultTestRun(test_wraparound, "test_wraparound", __LINE__);
    UnityDefaultTestRun(test_clear_reset, "test_clear_reset", __LINE__);
    UnityDefaultTestRun(test_all_zero_sequence, "test_all_zero_sequence", __LINE__);
    UnityDefaultTestRun(test_pipeline_correctness, "test_pipeline_correctness", __LINE__);
    UnityDefaultTestRun(test_pulses_monotonic_across_wraps, "test_pulses_monotonic_across_wraps", __LINE__);
    UnityDefaultTestRun(test_reset_baseline_no_glitch, "test_reset_baseline_no_glitch", __LINE__);
    UnityDefaultTestRun(test_debounce_zero_counts_all, "test_debounce_zero_counts_all", __LINE__);
    UnityDefaultTestRun(test_debounce_drops_second_pulse, "test_debounce_drops_second_pulse", __LINE__);
    UnityDefaultTestRun(test_reset_counts, "test_reset_counts", __LINE__);
    UnityDefaultTestRun(test_period_measurement_freq, "test_period_measurement_freq", __LINE__);
    UnityDefaultTestRun(test_period_measurement_fast, "test_period_measurement_fast", __LINE__);
    UnityDefaultTestRun(test_period_zero_interval, "test_period_zero_interval", __LINE__);
)
