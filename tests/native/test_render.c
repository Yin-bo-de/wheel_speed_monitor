/* render 行为测试：轮速 JSON 块字段完整性、数值格式、禁用轮、缓冲不足、确定性。 */
#include <string.h>
#include "unity.h"
#include "unity_runner.h"
#include "wheel_speed_render.h"

/* Unity 2.6 没有 assert_contains，提供本地助手 */
static void assert_contains(const char *haystack, const char *needle)
{
    if (strstr(haystack, needle) == NULL) {
        TEST_FAIL_MESSAGE(needle);
    }
}

void setUp(void) {}
void tearDown(void) {}

static wheel_render_state_t rs;

static void fresh(void)
{
    memset(&rs, 0, sizeof(rs));
    rs.source = WHEEL_RENDER_SRC_PCNT;
}

/* 1. 渲染结果含全部必需字段 */
static void test_all_required_fields(void)
{
    fresh();
    char buf[512];
    size_t used = 0;
    TEST_ASSERT_EQUAL_INT(ESP_OK, wheel_render_block(&rs, buf, sizeof(buf), &used));
    assert_contains(buf, "\"type\":\"wheel_speed\"");
    assert_contains(buf, "\"src\":\"pcnt\"");
    assert_contains(buf, "\"wheels\":");
    assert_contains(buf, "\"cfg\":{");
    assert_contains(buf, "\"magnets\":");
    assert_contains(buf, "\"diam_mm\":");
    assert_contains(buf, "\"alpha\":");
    assert_contains(buf, "\"sim\":");
    assert_contains(buf, "\"sim_rpm\":[");
}

/* 2. 浮点固定小数位（如 %.1f），不随 locale 变化 */
static void test_float_format_fixed(void)
{
    fresh();
    rs.magnets = 1;
    rs.wheel_diam_mm = 100;
    rs.alpha = 0.3f;
    rs.wheels[0].rpm = 42.37f;
    rs.wheels[0].speed_cm_s = 3.456f;
    char buf[512];
    size_t used = 0;
    wheel_render_block(&rs, buf, sizeof(buf), &used);
    assert_contains(buf, "\"rpm\":42.4");
    assert_contains(buf, "\"speed_cms\":3.5");
}

/* 3. 禁用轮仍输出且数值为 0 */
static void test_disabled_wheel_zero_output(void)
{
    fresh();
    rs.wheels[0].enabled = false;
    char buf[512];
    size_t used = 0;
    wheel_render_block(&rs, buf, sizeof(buf), &used);
    /* 渲染 4 轮；禁用轮 rpm 为 0.0 */
    assert_contains(buf, "\"i\":0,\"en\":false,\"rpm\":0.0");
}

/* 4. 缓冲不足返回错误码且不越界写 */
static void test_buffer_too_small(void)
{
    fresh();
    char tiny[8];
    size_t used = 0;
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_SIZE, wheel_render_block(&rs, tiny, sizeof(tiny), &used));
}

/* 5. 同一快照两次渲染字节一致（确定性） */
static void test_deterministic_render(void)
{
    fresh();
    rs.magnets = 2;
    rs.wheels[1].pulses = 123456;
    char b1[512], b2[512];
    size_t u1 = 0, u2 = 0;
    wheel_render_block(&rs, b1, sizeof(b1), &u1);
    wheel_render_block(&rs, b2, sizeof(b2), &u2);
    TEST_ASSERT_EQUAL_INT(u1, u2);
    TEST_ASSERT_EQUAL_STRING(b1, b2);
}

NATIVE_TEST_MAIN(
    UnityDefaultTestRun(test_all_required_fields, "test_all_required_fields", __LINE__);
    UnityDefaultTestRun(test_float_format_fixed, "test_float_format_fixed", __LINE__);
    UnityDefaultTestRun(test_disabled_wheel_zero_output, "test_disabled_wheel_zero_output", __LINE__);
    UnityDefaultTestRun(test_buffer_too_small, "test_buffer_too_small", __LINE__);
    UnityDefaultTestRun(test_deterministic_render, "test_deterministic_render", __LINE__);
)
