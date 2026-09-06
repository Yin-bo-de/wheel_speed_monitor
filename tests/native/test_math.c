/* wheel_speed_math 纯数值链行为测试（频率→RPM→线速度→EMA→静止归零）。 */
#include <math.h>
#include "unity.h"
#include "unity_runner.h"
#include "wheel_speed_math.h"

void setUp(void) {}
void tearDown(void) {}

static wsmath_state_t st;

static void fresh_state(void)
{
    st = (wsmath_state_t){0};
}

/* 1. 给定窗口脉冲数和窗口时长，频率计算正确：1 脉冲/50ms → 20Hz */
static void test_freq_calc(void)
{
    fresh_state();
    TEST_ASSERT_EQUAL_INT(ESP_OK, wsmath_update(&st, 1, 50, 1, 100, 0.3f, 1000));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 20.0f, st.freq_hz);
}

/* 2. 频率→RPM 正确除以磁铁数 */
static void test_rpm_magnet_division(void)
{
    fresh_state();
    /* 20Hz、1 磁铁 → 1200 RPM */
    wsmath_update(&st, 1, 50, 1, 100, 0.3f, 1000);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 1200.0f, st.rpm_ema);

    fresh_state();
    /* 20Hz、2 磁铁 → 600 RPM；4 磁铁 → 300 RPM */
    wsmath_update(&st, 1, 50, 2, 100, 0.3f, 1000);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 600.0f, st.rpm_ema);
    fresh_state();
    wsmath_update(&st, 1, 50, 4, 100, 0.3f, 1000);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 300.0f, st.rpm_ema);
}

/* 3. RPM→线速度 cm/s 与 km/h 换算正确
 * 1200 RPM、轮径 100mm：20 转/秒 × π×100mm/转 = 2000π mm/s ≈ 628.3 cm/s；
 * km/h = cm/s ÷ 100 × 3.6 ≈ 22.62 */
static void test_speed_conversion(void)
{
    fresh_state();
    wsmath_update(&st, 1, 50, 1, 100, 0.3f, 1000);
    TEST_ASSERT_FLOAT_WITHIN(0.2f, 628.32f, st.speed_cm_s);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 22.62f, st.speed_km_h);
}

/* 4. 磁铁数为 0 返回错误码，且不产生 NaN/无穷 */
static void test_zero_magnets_rejected(void)
{
    fresh_state();
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, wsmath_update(&st, 1, 50, 0, 100, 0.3f, 1000));
    TEST_ASSERT_TRUE(isfinite(st.freq_hz));
    TEST_ASSERT_TRUE(isfinite(st.rpm_ema));
    TEST_ASSERT_TRUE(isfinite(st.speed_cm_s));
    TEST_ASSERT_TRUE(isfinite(st.speed_km_h));
}

/* 5. EMA 首样本直接采纳（不清零起步缓升） */
static void test_ema_first_sample_taken_directly(void)
{
    fresh_state();
    wsmath_update(&st, 1, 50, 1, 100, 0.05f, 1000);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 1200.0f, st.rpm_ema);
}

/* 6. 常数输入序列下 EMA 收敛到真值（偏差 < 0.5%） */
static void test_ema_converges(void)
{
    fresh_state();
    /* 每窗口 2 脉冲、1 磁铁、50ms → 40Hz → 2400 RPM；alpha=0.3 迭代 100 次 */
    for (int i = 0; i < 100; i++) {
        wsmath_update(&st, 2, 50, 1, 100, 0.3f, 1000 + i * 50);
    }
    TEST_ASSERT_FLOAT_WITHIN(2400.0f * 0.5f / 100.0f, 2400.0f, st.rpm_ema);
}

/* 7. alpha=1 输出即最新样本（一步到位，不保留滤波历史）；alpha=0.05 响应明显滞后 */
static void test_alpha_semantics(void)
{
    fresh_state();
    /* alpha=1：同窗口序列 1 脉冲→2 脉冲，输出直接等于最新 raw（1200→2400） */
    wsmath_update(&st, 1, 50, 1, 100, 1.0f, 1000);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 1200.0f, st.rpm_ema);
    wsmath_update(&st, 2, 50, 1, 100, 1.0f, 1050);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 2400.0f, st.rpm_ema);
}

static void test_alpha_low_lags(void)
{
    fresh_state();
    for (int i = 0; i < 20; i++) {
        wsmath_update(&st, 1, 50, 1, 100, 0.3f, 1000 + i * 50);
    }
    /* alpha=0.05：跳变到 300 RPM 后一个窗口，仍明显高于真值（滞后） */
    wsmath_update(&st, 0, 50, 1, 100, 0.05f, 1000 + 1000);
    wsmath_update(&st, 1, 50, 1, 100, 0.05f, 1000 + 1050);
    TEST_ASSERT_TRUE(st.rpm_ema > 600.0f);
}

/* 8. 连续无脉冲超过 1s 时 RPM 与线速度归零 */
static void test_idle_timeout_zeroes(void)
{
    fresh_state();
    wsmath_update(&st, 1, 50, 1, 100, 0.3f, 1000);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 1200.0f, st.rpm_ema);
    /* now_ms - last_pulse_ms > 1000 → 归零 */
    wsmath_update(&st, 0, 50, 1, 100, 0.3f, 2100);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, st.rpm_ema);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, st.speed_cm_s);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, st.speed_km_h);
}

/* 9. 归零后再次来脉冲，RPM 恢复计算且不继承旧滤波值（首样本直接采纳） */
static void test_revive_after_idle(void)
{
    fresh_state();
    wsmath_update(&st, 1, 50, 1, 100, 0.05f, 1000);
    wsmath_update(&st, 0, 50, 1, 100, 0.05f, 1100);
    /* 静止 */
    wsmath_update(&st, 0, 50, 1, 100, 0.05f, 2100);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, st.rpm_ema);
    /* 恢复：alpha 极小也应一步到 1200（不继承旧值缓升） */
    wsmath_update(&st, 1, 50, 1, 100, 0.05f, 2150);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 1200.0f, st.rpm_ema);
}

/* 10. 窗口时长为 0 返回错误码 */
static void test_zero_window_rejected(void)
{
    fresh_state();
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, wsmath_update(&st, 1, 0, 1, 100, 0.3f, 1000));
}

/* 11. 脉冲间隔测频：1s 间隔 → 1Hz；100ms → 10Hz；0 间隔报错不产生 NaN */
static void test_period_freq(void)
{
    fresh_state();
    TEST_ASSERT_EQUAL_INT(ESP_OK, wsmath_update_period(&st, 1000000u, 1, 100, 0.9f, 1000));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.0f, st.freq_hz);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 60.0f, st.rpm_ema);

    fresh_state();
    TEST_ASSERT_EQUAL_INT(ESP_OK, wsmath_update_period(&st, 100000u, 1, 100, 0.9f, 1000));
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 10.0f, st.freq_hz);

    fresh_state();
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, wsmath_update_period(&st, 0, 1, 100, 0.9f, 1000));
    TEST_ASSERT_TRUE(isfinite(st.freq_hz));
}

NATIVE_TEST_MAIN(
    UnityDefaultTestRun(test_freq_calc, "test_freq_calc", __LINE__);
    UnityDefaultTestRun(test_rpm_magnet_division, "test_rpm_magnet_division", __LINE__);
    UnityDefaultTestRun(test_speed_conversion, "test_speed_conversion", __LINE__);
    UnityDefaultTestRun(test_zero_magnets_rejected, "test_zero_magnets_rejected", __LINE__);
    UnityDefaultTestRun(test_ema_first_sample_taken_directly, "test_ema_first_sample_taken_directly", __LINE__);
    UnityDefaultTestRun(test_ema_converges, "test_ema_converges", __LINE__);
    UnityDefaultTestRun(test_alpha_semantics, "test_alpha_semantics", __LINE__);
    UnityDefaultTestRun(test_alpha_low_lags, "test_alpha_low_lags", __LINE__);
    UnityDefaultTestRun(test_idle_timeout_zeroes, "test_idle_timeout_zeroes", __LINE__);
    UnityDefaultTestRun(test_revive_after_idle, "test_revive_after_idle", __LINE__);
    UnityDefaultTestRun(test_zero_window_rejected, "test_zero_window_rejected", __LINE__);
    UnityDefaultTestRun(test_period_freq, "test_period_freq", __LINE__);
)
