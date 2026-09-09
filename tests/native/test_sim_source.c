/* sim_source 行为测试：模拟源用误差累积取整生成脉冲，长时间平均频率精确等于目标。
 * 模拟源与 PCNT 源同构：外部只读单调计数器（这里也走 16 位回绕），与本项目采集链一致。 */
#include <string.h>
#include "unity.h"
#include "unity_runner.h"
#include "sim_source.h"

void setUp(void) {}
void tearDown(void) {}

static sim_source_t sim;

static void fresh(void)
{
    memset(&sim, 0, sizeof(sim));
}

/* 窗口脉冲差分：读当前计数 − 上窗口计数（含 16 位回绕），返回本窗口实际增量 */
static uint32_t diff_ch(const sim_source_t *s, uint8_t ch, uint16_t prev)
{
    uint16_t cur = sim_source_counter(s, ch);
    if (cur >= prev) {
        return cur - prev;
    }
    return 65536u - prev + cur;
}

/* 1. 目标 60 RPM：每窗口 0.05 脉冲，连续 20 窗口恰好累计 1 脉冲 */
static void test_low_rpm_accumulates_across_windows(void)
{
    fresh();
    sim_source_set_target_rpm(&sim, 0, 60.0f);
    uint32_t total = 0;
    uint16_t prev = 0;
    for (int i = 0; i < 20; i++) {
        sim_source_step(&sim, 50);
        total += diff_ch(&sim, 0, prev);
        prev = sim_source_counter(&sim, 0);
    }
    TEST_ASSERT_EQUAL_UINT32(1, total);
}

/* 2. 目标 1200 RPM：每窗口恰 1 脉冲 */
static void test_integer_pulse_per_window(void)
{
    fresh();
    sim_source_set_target_rpm(&sim, 0, 1200.0f);
    uint16_t prev = 0;
    for (int i = 0; i < 3; i++) {
        sim_source_step(&sim, 50);
        TEST_ASSERT_EQUAL_UINT32(1, diff_ch(&sim, 0, prev));
        prev = sim_source_counter(&sim, 0);
    }
}

/* 3. 长时间平均频率等于目标（1000 窗口累计偏差 ≤ 1 脉冲）：60 RPM × 0.05/窗口 → 50 脉冲 */
static void test_long_term_avg_accuracy(void)
{
    fresh();
    sim_source_set_target_rpm(&sim, 0, 60.0f);
    uint32_t total = 0;
    uint16_t prev = 0;
    for (int i = 0; i < 1000; i++) {
        sim_source_step(&sim, 50);
        total += diff_ch(&sim, 0, prev);
        prev = sim_source_counter(&sim, 0);
    }
    TEST_ASSERT_UINT32_WITHIN(1, 50, total);
}

/* 4. 目标 RPM=0 → 恒 0 增量、无除零 */
static void test_zero_target(void)
{
    fresh();
    sim_source_set_target_rpm(&sim, 0, 0.0f);
    uint16_t prev = 0;
    for (int i = 0; i < 10; i++) {
        sim_source_step(&sim, 50);
        TEST_ASSERT_EQUAL_UINT32(0, diff_ch(&sim, 0, prev));
        prev = sim_source_counter(&sim, 0);
    }
}

/* 5. 4 轮各自目标独立，累计脉冲与各自目标成比例 */
static void test_channels_accumulated_proportions(void)
{
    fresh();
    sim_source_set_target_rpm(&sim, 0, 60.0f);
    sim_source_set_target_rpm(&sim, 1, 1200.0f);
    sim_source_set_target_rpm(&sim, 2, 2400.0f);
    sim_source_set_target_rpm(&sim, 3, 0.0f);

    uint16_t prev[4] = {0};
    uint32_t acc[4] = {0};
    for (int i = 0; i < 100; i++) {
        sim_source_step(&sim, 50);
        for (int c = 0; c < 4; c++) {
            acc[c] += diff_ch(&sim, (uint8_t)c, prev[c]);
            prev[c] = sim_source_counter(&sim, (uint8_t)c);
        }
    }
    /* 100 窗口 × 0.05 = 5；100×1=100；100×2=200；0 */
    TEST_ASSERT_EQUAL_UINT32(5, acc[0]);
    TEST_ASSERT_EQUAL_UINT32(100, acc[1]);
    TEST_ASSERT_EQUAL_UINT32(200, acc[2]);
    TEST_ASSERT_EQUAL_UINT32(0, acc[3]);
}

/* 6. 计数到达 uint16 上限后回绕，永不出界 */
static void test_counter_never_exceeds_u16(void)
{
    fresh();
    /* 高 RPM 快速冲爆 65535 */
    sim_source_set_target_rpm(&sim, 0, 36000.0f);
    uint32_t total = 0;
    uint16_t prev = 0;
    for (int i = 0; i < 100; i++) {
        sim_source_step(&sim, 50);
        TEST_ASSERT_TRUE(sim_source_counter(&sim, 0) <= 0xFFFF);
        total += diff_ch(&sim, 0, prev);
        prev = sim_source_counter(&sim, 0);
    }
    /* 36000 RPM = 600 Hz = 30 脉冲/窗口 → 100 窗口 3000 脉冲，回绕至少一次 */
    TEST_ASSERT_EQUAL_UINT32(3000, total);
}

/* 7. 目标 RPM 中途改变后收敛到新目标（误差累积器不炸）：
 * 前 200 窗口 1200 RPM（每窗口 1），后 200 窗口 30 RPM（每窗口 0.025）→ 合计 200 + 5 = 205 */
static void test_target_change_converges(void)
{
    fresh();
    sim_source_set_target_rpm(&sim, 0, 1200.0f);
    uint16_t prev = 0;
    uint32_t total = 0;
    for (int i = 0; i < 200; i++) {
        sim_source_step(&sim, 50);
        total += diff_ch(&sim, 0, prev);
        prev = sim_source_counter(&sim, 0);
    }
    TEST_ASSERT_UINT32_WITHIN(1, 200, total);

    sim_source_set_target_rpm(&sim, 0, 30.0f); /* 改目标 */
    for (int i = 0; i < 200; i++) {
        sim_source_step(&sim, 50);
        total += diff_ch(&sim, 0, prev);
        prev = sim_source_counter(&sim, 0);
    }
    TEST_ASSERT_UINT32_WITHIN(1, 205, total);
}

/* 8. 间隔产出：60 RPM（1 RPS）跨 20 个 50ms 窗口才出 1 脉冲。
 * 首次 emit 只设基准（与真实 ISR 对齐），第二次 emit 才产间隔。
 * 间隔应如实反映 1s 周期（而非单窗口的 50ms 倍频） */
static void test_period_reflects_cross_window_span(void)
{
    fresh();
    sim_source_set_target_rpm(&sim, 0, 60.0f); /* 1 RPS */
    for (int i = 0; i < 20; i++) {
        sim_source_step(&sim, 50);
    }
    /* 第 20 窗口首次 emit，只设基准，period 仍为 0 */
    TEST_ASSERT_EQUAL_UINT32(1, sim_source_counter(&sim, 0));
    TEST_ASSERT_EQUAL_UINT32(0, sim_source_period_us(&sim, 0));
    /* 再跑 20 窗口到第二次 emit，span = 20×50ms = 1000000us */
    for (int i = 0; i < 20; i++) {
        sim_source_step(&sim, 50);
    }
    TEST_ASSERT_EQUAL_UINT32(1000000u, sim_source_period_us(&sim, 0));
}

/* 9. 高速 1200 RPM：每窗口 1 脉冲。首次只设基准，第二次 emit 间隔 = 50ms → 20Hz */
static void test_period_high_speed(void)
{
    fresh();
    sim_source_set_target_rpm(&sim, 0, 1200.0f);
    sim_source_step(&sim, 50); /* 首次 emit，只设基准 */
    TEST_ASSERT_EQUAL_UINT32(0, sim_source_period_us(&sim, 0));
    sim_source_step(&sim, 50); /* 第二次 emit，span = 50ms */
    TEST_ASSERT_EQUAL_UINT32(50000u, sim_source_period_us(&sim, 0));
}

/* 10. 停转过期：建立间隔后停转，超过 1.5×周期后 period 返回 0 */
static void test_period_expires_after_stop(void)
{
    fresh();
    sim_source_set_target_rpm(&sim, 0, 1200.0f); /* 20Hz, 间隔 50ms */
    sim_source_step(&sim, 50); /* 首次 emit，只设基准 */
    sim_source_step(&sim, 50); /* 第二次 emit，period=50000 */
    TEST_ASSERT_EQUAL_UINT32(50000u, sim_source_period_us(&sim, 0));
    /* 停转：target=0，继续 step 不 emit，since 持续增长 */
    sim_source_set_target_rpm(&sim, 0, 0.0f);
    /* 1.5×50000=75000us = 1.5 窗口；2 个窗口后 since=100000 > 75000 → 过期 */
    sim_source_step(&sim, 50); /* since=50000，未过期 */
    TEST_ASSERT_EQUAL_UINT32(50000u, sim_source_period_us(&sim, 0));
    sim_source_step(&sim, 50); /* since=100000 > 75000，过期 */
    TEST_ASSERT_EQUAL_UINT32(0, sim_source_period_us(&sim, 0));
}

NATIVE_TEST_MAIN(
    UnityDefaultTestRun(test_low_rpm_accumulates_across_windows, "test_low_rpm_accumulates_across_windows", __LINE__);
    UnityDefaultTestRun(test_integer_pulse_per_window, "test_integer_pulse_per_window", __LINE__);
    UnityDefaultTestRun(test_long_term_avg_accuracy, "test_long_term_avg_accuracy", __LINE__);
    UnityDefaultTestRun(test_zero_target, "test_zero_target", __LINE__);
    UnityDefaultTestRun(test_channels_accumulated_proportions, "test_channels_accumulated_proportions", __LINE__);
    UnityDefaultTestRun(test_counter_never_exceeds_u16, "test_counter_never_exceeds_u16", __LINE__);
    UnityDefaultTestRun(test_target_change_converges, "test_target_change_converges", __LINE__);
    UnityDefaultTestRun(test_period_reflects_cross_window_span, "test_period_reflects_cross_window_span", __LINE__);
    UnityDefaultTestRun(test_period_high_speed, "test_period_high_speed", __LINE__);
    UnityDefaultTestRun(test_period_expires_after_stop, "test_period_expires_after_stop", __LINE__);
)
