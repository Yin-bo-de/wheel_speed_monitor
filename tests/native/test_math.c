/* wheel_speed_math 间隔法数值链行为测试（频率→RPM→线速度→EMA）。 */
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

/* 1. 脉冲间隔测频：1s 间隔 → 1Hz；100ms → 10Hz */
static void test_freq_calc(void)
{
    fresh_state();
    TEST_ASSERT_EQUAL_INT(ESP_OK, wsmath_update_period(&st, 1000000u, 1, 100, 0.3f, 1000));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.0f, st.freq_hz);

    fresh_state();
    TEST_ASSERT_EQUAL_INT(ESP_OK, wsmath_update_period(&st, 100000u, 1, 100, 0.3f, 1000));
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 10.0f, st.freq_hz);
}

/* 2. 频率→RPM 正确除以磁铁数：1Hz 1 磁铁 → 60 RPM；2 磁铁 → 30；4 磁铁 → 15 */
static void test_rpm_magnet_division(void)
{
    fresh_state();
    wsmath_update_period(&st, 1000000u, 1, 100, 0.3f, 1000);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 60.0f, st.rpm_ema);

    fresh_state();
    wsmath_update_period(&st, 1000000u, 2, 100, 0.3f, 1000);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 30.0f, st.rpm_ema);

    fresh_state();
    wsmath_update_period(&st, 1000000u, 4, 100, 0.3f, 1000);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 15.0f, st.rpm_ema);
}

/* 3. RPM→线速度 cm/s 与 km/h 换算正确
 * 60 RPM、轮径 100mm：1 转/秒 × π×100mm/转 = 100π mm/s ≈ 31.42 cm/s；
 * km/h = cm/s ÷ 100 × 3.6 ≈ 1.131 */
static void test_speed_conversion(void)
{
    fresh_state();
    wsmath_update_period(&st, 1000000u, 1, 100, 0.3f, 1000);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 31.416f, st.speed_cm_s);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.131f, st.speed_km_h);
}

/* 4. 磁铁数为 0 返回错误码，且不产生 NaN/无穷 */
static void test_zero_magnets_rejected(void)
{
    fresh_state();
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, wsmath_update_period(&st, 1000000u, 0, 100, 0.3f, 1000));
    TEST_ASSERT_TRUE(isfinite(st.freq_hz));
    TEST_ASSERT_TRUE(isfinite(st.rpm_ema));
    TEST_ASSERT_TRUE(isfinite(st.speed_cm_s));
    TEST_ASSERT_TRUE(isfinite(st.speed_km_h));
}

/* 5. EMA 首样本直接采纳（不清零起步缓升） */
static void test_ema_first_sample_taken_directly(void)
{
    fresh_state();
    wsmath_update_period(&st, 1000000u, 1, 100, 0.05f, 1000);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 60.0f, st.rpm_ema);
}

/* 6. 常数间隔序列下 EMA 收敛到真值（偏差 < 0.5%） */
static void test_ema_converges(void)
{
    fresh_state();
    /* 100ms 间隔 → 10Hz → 600 RPM；alpha=0.3 迭代 100 次 */
    for (int i = 0; i < 100; i++) {
        wsmath_update_period(&st, 100000u, 1, 100, 0.3f, 1000 + i * 50);
    }
    TEST_ASSERT_FLOAT_WITHIN(600.0f * 0.5f / 100.0f, 600.0f, st.rpm_ema);
}

/* 7. alpha=1 输出即最新样本（一步到位）；alpha 小则明显滞后 */
static void test_alpha_semantics(void)
{
    fresh_state();
    /* alpha=1：1Hz→10Hz，输出直接等于最新 raw（60→600） */
    wsmath_update_period(&st, 1000000u, 1, 100, 1.0f, 1000);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 60.0f, st.rpm_ema);
    wsmath_update_period(&st, 100000u, 1, 100, 1.0f, 1050);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 600.0f, st.rpm_ema);
}

/* 8. 间隔为 0 返回错误码且不产生 NaN */
static void test_zero_interval_rejected(void)
{
    fresh_state();
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, wsmath_update_period(&st, 0, 1, 100, 0.3f, 1000));
    TEST_ASSERT_TRUE(isfinite(st.freq_hz));
}

/* 9. last_pulse_ms 随采样刷新（供上层静止判定） */
static void test_last_pulse_ms_refreshed(void)
{
    fresh_state();
    wsmath_update_period(&st, 1000000u, 1, 100, 0.3f, 5000);
    TEST_ASSERT_EQUAL_UINT32(5000, st.last_pulse_ms);
}

NATIVE_TEST_MAIN(
    UnityDefaultTestRun(test_freq_calc, "test_freq_calc", __LINE__);
    UnityDefaultTestRun(test_rpm_magnet_division, "test_rpm_magnet_division", __LINE__);
    UnityDefaultTestRun(test_speed_conversion, "test_speed_conversion", __LINE__);
    UnityDefaultTestRun(test_zero_magnets_rejected, "test_zero_magnets_rejected", __LINE__);
    UnityDefaultTestRun(test_ema_first_sample_taken_directly, "test_ema_first_sample_taken_directly", __LINE__);
    UnityDefaultTestRun(test_ema_converges, "test_ema_converges", __LINE__);
    UnityDefaultTestRun(test_alpha_semantics, "test_alpha_semantics", __LINE__);
    UnityDefaultTestRun(test_zero_interval_rejected, "test_zero_interval_rejected", __LINE__);
    UnityDefaultTestRun(test_last_pulse_ms_refreshed, "test_last_pulse_ms_refreshed", __LINE__);
)
