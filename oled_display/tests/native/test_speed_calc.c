/* speed_calc 单测：慢轮取最小、坏轮剔除、换算公式。 */
#include "speed_calc.h"
#include "unity.h"
#include "unity_runner.h"

void setUp(void) {}
void tearDown(void) {}

static speed_calc_in_t in_all(float rpm)
{
    speed_calc_in_t in = {.diam_mm = 66.0f};
    for (int i = 0; i < SPEED_CALC_WHEELS; i++) {
        in.valid[i] = true;
        in.rpm[i] = rpm;
    }
    return in;
}

static void test_min_valid_wheel(void)
{
    speed_calc_in_t in = in_all(60.0f);
    in.rpm[0] = 200.0f; /* 打滑轮快，不参与 */
    float v = -1.0f;
    TEST_ASSERT_TRUE(speed_calc_kmh(&in, &v));
    /* 慢轮 60rpm：π × 66mm × 60 × 6e-5 ≈ 0.746 km/h */
    TEST_ASSERT_FLOAT_WITHIN(0.002f, 0.7461f, v);
}

static void test_disabled_wheel_excluded(void)
{
    speed_calc_in_t in = in_all(60.0f);
    in.rpm[2] = 0.0f;  /* 坏轮把最小值拉到 0 */
    in.valid[2] = false; /* 剔除后不受影响 */
    float v = -1.0f;
    TEST_ASSERT_TRUE(speed_calc_kmh(&in, &v));
    TEST_ASSERT_FLOAT_WITHIN(0.002f, 0.7461f, v);
}

static void test_all_invalid(void)
{
    speed_calc_in_t in = in_all(60.0f);
    for (int i = 0; i < SPEED_CALC_WHEELS; i++) {
        in.valid[i] = false;
    }
    float v = -1.0f;
    TEST_ASSERT_FALSE(speed_calc_kmh(&in, &v));
}

static void test_bad_diam(void)
{
    speed_calc_in_t in = in_all(60.0f);
    in.diam_mm = 0.0f;
    float v = -1.0f;
    TEST_ASSERT_FALSE(speed_calc_kmh(&in, &v));
}

static void test_negative_rpm_skipped(void)
{
    speed_calc_in_t in = in_all(60.0f);
    in.rpm[1] = -5.0f; /* 坏数据不当有效轮 */
    float v = -1.0f;
    TEST_ASSERT_TRUE(speed_calc_kmh(&in, &v));
    TEST_ASSERT_FLOAT_WITHIN(0.002f, 0.7461f, v);
}

static void test_zero_rpm_is_valid(void)
{
    /* 车停着也是有效数据：全零显示 0，不显示 "--" */
    speed_calc_in_t in = in_all(0.0f);
    float v = -1.0f;
    TEST_ASSERT_TRUE(speed_calc_kmh(&in, &v));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, v);
}

NATIVE_TEST_MAIN(
    UnityDefaultTestRun(test_min_valid_wheel, "test_min_valid_wheel", __LINE__);
    UnityDefaultTestRun(test_disabled_wheel_excluded, "test_disabled_wheel_excluded", __LINE__);
    UnityDefaultTestRun(test_all_invalid, "test_all_invalid", __LINE__);
    UnityDefaultTestRun(test_bad_diam, "test_bad_diam", __LINE__);
    UnityDefaultTestRun(test_negative_rpm_skipped, "test_negative_rpm_skipped", __LINE__);
    UnityDefaultTestRun(test_zero_rpm_is_valid, "test_zero_rpm_is_valid", __LINE__);
)
