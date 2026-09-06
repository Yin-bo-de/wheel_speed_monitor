/* strategy 桩接口行为测试：模式管理、feed 无副作用、舵机/PWM 未实现、默认脉宽配置。 */
#include <string.h>
#include "unity.h"
#include "unity_runner.h"
#include "strategy.h"

void setUp(void) {}
void tearDown(void) {}

/* 1. init 可重复调用且返回 OK */
static void test_init_repeatable(void)
{
    strategy_status_t st;
    TEST_ASSERT_EQUAL_INT(ESP_OK, strategy_init());
    TEST_ASSERT_EQUAL_INT(ESP_OK, strategy_init());
    TEST_ASSERT_EQUAL_INT(ESP_OK, strategy_get_status(&st));
}

/* 2. set_mode 手动/自动均 OK，get_status 反映最后一次模式 */
static void test_mode_set_get(void)
{
    strategy_status_t st;
    strategy_init();
    TEST_ASSERT_EQUAL_INT(ESP_OK, strategy_set_mode(STRATEGY_MODE_MANUAL));
    strategy_get_status(&st);
    TEST_ASSERT_EQUAL_INT(STRATEGY_MODE_MANUAL, st.mode);

    TEST_ASSERT_EQUAL_INT(ESP_OK, strategy_set_mode(STRATEGY_MODE_AUTO));
    strategy_get_status(&st);
    TEST_ASSERT_EQUAL_INT(STRATEGY_MODE_AUTO, st.mode);
}

/* 3. feed 输入遥测帧后状态不变（v1 无副作用），返回 OK */
static void test_feed_no_side_effect(void)
{
    float wheel_rpm[4] = {12.5f, 13.0f, 11.8f, 12.1f};
    strategy_status_t st_before, st_after;
    strategy_init();
    strategy_get_status(&st_before);
    TEST_ASSERT_EQUAL_INT(ESP_OK, strategy_feed_wheel_rpm(wheel_rpm));
    strategy_get_status(&st_after);
    TEST_ASSERT_EQUAL_INT(st_before.mode, st_after.mode);
    TEST_ASSERT_EQUAL_INT(st_before.supported, st_after.supported);
}

/* 4. set_servo 返回 ESP_ERR_NOT_SUPPORTED（v1 未实现舵机输出） */
static void test_set_servo_not_supported(void)
{
    strategy_init();
    TEST_ASSERT_EQUAL_INT(ESP_ERR_NOT_SUPPORTED, strategy_set_servo(0, 1500));
}

/* 5. rc_read 返回 ESP_ERR_NOT_SUPPORTED（v1 未装 RC 捕获） */
static void test_rc_read_not_supported(void)
{
    strategy_init();
    uint16_t us = 0;
    TEST_ASSERT_EQUAL_INT(ESP_ERR_NOT_SUPPORTED, strategy_rc_read(0, &us));
}

/* 6. get_status 的 supported 标志恒为 false */
static void test_supported_false(void)
{
    strategy_status_t st;
    strategy_init();
    strategy_get_status(&st);
    TEST_ASSERT_FALSE(st.supported);
}

/* 7. PWM 输入输出配置结构存在且有默认值（脉宽 1000–2000µs、周期 50Hz） */
static void test_pwm_config_defaults(void)
{
    strategy_pwm_config_t cfg;
    TEST_ASSERT_EQUAL_INT(ESP_OK, strategy_get_pwm_config(&cfg));
    TEST_ASSERT_EQUAL_INT(1000, cfg.servo_min_us);
    TEST_ASSERT_EQUAL_INT(2000, cfg.servo_max_us);
    TEST_ASSERT_EQUAL_INT(50, cfg.servo_hz);
    /* v1 无舵机引脚，输出引脚为 -1（未分配） */
    TEST_ASSERT_EQUAL_INT(-1, cfg.servo_gpio);
}

NATIVE_TEST_MAIN(
    UnityDefaultTestRun(test_init_repeatable, "test_init_repeatable", __LINE__);
    UnityDefaultTestRun(test_mode_set_get, "test_mode_set_get", __LINE__);
    UnityDefaultTestRun(test_feed_no_side_effect, "test_feed_no_side_effect", __LINE__);
    UnityDefaultTestRun(test_set_servo_not_supported, "test_set_servo_not_supported", __LINE__);
    UnityDefaultTestRun(test_rc_read_not_supported, "test_rc_read_not_supported", __LINE__);
    UnityDefaultTestRun(test_supported_false, "test_supported_false", __LINE__);
    UnityDefaultTestRun(test_pwm_config_defaults, "test_pwm_config_defaults", __LINE__);
)
