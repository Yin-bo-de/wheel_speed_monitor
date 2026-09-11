/*
 * strategy_render 测试：舵机 JSON 块的字段完整性、三态 phase 字符串、
 * 角度反算（含反向映射）、数值格式、缓冲不足、确定性。
 */
#include <string.h>

#include "unity.h"
#include "unity_runner.h"
#include "strategy_render.h"

/* Unity 2.6 没有 assert_contains，提供本地助手 */
static void assert_contains(const char *haystack, const char *needle)
{
    if (strstr(haystack, needle) == NULL) {
        TEST_FAIL_MESSAGE(needle);
    }
}

void setUp(void) {}
void tearDown(void) {}

static servo_render_state_t rs;

static void fresh(void)
{
    memset(&rs, 0, sizeof(rs));
    strategy_config_default(&rs.cfg);
    rs.src = "sim";
    rs.sim = true;
    rs.sim_speed_us_s = 3000;
    for (int i = 0; i < STRATEGY_SERVO_COUNT; i++) {
        rs.ch[i].enabled = true;
        rs.ch[i].target_us = 1500;
        rs.ch[i].cur_us = 1500.0f;
        rs.ch[i].reached = true;
        rs.ch[i].phase = STRATEGY_PHASE_IDLE;
    }
}

/* 1. 必需字段齐全 */
static void test_all_required_fields(void)
{
    fresh();
    char buf[1024];
    size_t used = 0;
    TEST_ASSERT_EQUAL_INT(ESP_OK, servo_render_block(&rs, buf, sizeof(buf), &used));
    assert_contains(buf, "\"type\":\"servo\"");
    assert_contains(buf, "\"src\":\"sim\"");
    assert_contains(buf, "\"mode\":\"auto\"");
    assert_contains(buf, "\"servos\":[");
    assert_contains(buf, "\"cfg\":{");
    assert_contains(buf, "\"i\":0");
    assert_contains(buf, "\"phase\":\"idle\"");
    assert_contains(buf, "\"hold_ms\":");
    assert_contains(buf, "\"ratio\":");
    assert_contains(buf, "\"slip_left\":");
    assert_contains(buf, "\"target_us\":");
    assert_contains(buf, "\"target_deg\":");
    assert_contains(buf, "\"cur_us\":");
    assert_contains(buf, "\"cur_deg\":");
    assert_contains(buf, "\"reached\":");
    /* cfg 回显 */
    assert_contains(buf, "\"sim\":true");
    assert_contains(buf, "\"min_us\":1000");
    assert_contains(buf, "\"center_us\":1500");
    assert_contains(buf, "\"max_us\":2000");
    assert_contains(buf, "\"invert\":[");
    assert_contains(buf, "\"manual_us\":[");
    assert_contains(buf, "\"engage\":0.30");
    assert_contains(buf, "\"min_rpm\":10.0");
    assert_contains(buf, "\"hold_ms\":3000");
    assert_contains(buf, "\"hold_max_ms\":30000");
    assert_contains(buf, "\"probe_ms\":2000");
    assert_contains(buf, "\"sim_speed\":3000");
}

/* 2. 三态 phase 各自渲染成对应字符串 */
static void test_phase_strings(void)
{
    fresh();
    char buf[1024];
    size_t used = 0;

    rs.ch[0].phase = STRATEGY_PHASE_LOCKED;
    rs.ch[0].hold_ms = 6000;
    rs.ch[1].phase = STRATEGY_PHASE_PROBE;
    servo_render_block(&rs, buf, sizeof(buf), &used);
    assert_contains(buf, "\"i\":0,\"en\":true,\"phase\":\"locked\",\"hold_ms\":6000");
    assert_contains(buf, "\"i\":1,\"en\":true,\"phase\":\"probe\"");

    TEST_ASSERT_EQUAL_STRING("idle", strategy_phase_name(STRATEGY_PHASE_IDLE));
    TEST_ASSERT_EQUAL_STRING("locked", strategy_phase_name(STRATEGY_PHASE_LOCKED));
    TEST_ASSERT_EQUAL_STRING("probe", strategy_phase_name(STRATEGY_PHASE_PROBE));
}

/* 3. 角度由脉宽反算：中位 0°、满偏 ±90°、非对称量程也正确 */
static void test_deg_back_calculation(void)
{
    fresh();
    rs.ch[0].target_us = 2000;
    rs.ch[0].cur_us = 1250.0f;
    char buf[1024];
    size_t used = 0;
    servo_render_block(&rs, buf, sizeof(buf), &used);
    assert_contains(buf, "\"target_deg\":90.0");
    assert_contains(buf, "\"cur_deg\":-45.0");

    /* 反向映射：同样的脉宽给出相反的角度 */
    rs.cfg.invert[0] = true;
    servo_render_block(&rs, buf, sizeof(buf), &used);
    assert_contains(buf, "\"target_deg\":-90.0");
    assert_contains(buf, "\"cur_deg\":45.0");
}

/* 4. 打滑诊断字段透出 */
static void test_slip_diagnostics(void)
{
    fresh();
    rs.ch[0].ratio = 0.7f;
    rs.ch[0].slip_fast_left = true;
    rs.ch[0].reached = false;
    char buf[1024];
    size_t used = 0;
    servo_render_block(&rs, buf, sizeof(buf), &used);
    assert_contains(buf, "\"ratio\":0.70");
    assert_contains(buf, "\"slip_left\":true");
    assert_contains(buf, "\"reached\":false");
}

/* 5. LEDC 源与手动模式回显 */
static void test_ledc_and_manual_echo(void)
{
    fresh();
    rs.src = "ledc";
    rs.sim = false;
    rs.cfg.mode = STRATEGY_MODE_MANUAL;
    char buf[1024];
    size_t used = 0;
    servo_render_block(&rs, buf, sizeof(buf), &used);
    assert_contains(buf, "\"src\":\"ledc\"");
    assert_contains(buf, "\"mode\":\"manual\"");
    assert_contains(buf, "\"sim\":false");
}

/* 6. 缓冲不足返回错误码 */
static void test_buffer_too_small(void)
{
    fresh();
    char tiny[16];
    size_t used = 0;
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_SIZE, servo_render_block(&rs, tiny, sizeof(tiny), &used));
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, servo_render_block(NULL, tiny, sizeof(tiny), &used));
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, servo_render_block(&rs, NULL, sizeof(tiny), &used));
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, servo_render_block(&rs, tiny, sizeof(tiny), NULL));
}

/* 7. 同一快照两次渲染字节一致 */
static void test_deterministic(void)
{
    fresh();
    rs.ch[1].phase = STRATEGY_PHASE_LOCKED;
    rs.ch[1].hold_ms = 12000;
    rs.ch[1].ratio = 0.42f;
    char b1[1024], b2[1024];
    size_t u1 = 0, u2 = 0;
    servo_render_block(&rs, b1, sizeof(b1), &u1);
    servo_render_block(&rs, b2, sizeof(b2), &u2);
    TEST_ASSERT_EQUAL_INT(u1, u2);
    TEST_ASSERT_EQUAL_STRING(b1, b2);
}

/* 8. 整块长度在遥测块缓冲（1024）之内 */
static void test_block_fits_budget(void)
{
    fresh();
    char buf[1024];
    size_t used = 0;
    TEST_ASSERT_EQUAL_INT(ESP_OK, servo_render_block(&rs, buf, sizeof(buf), &used));
    TEST_ASSERT_TRUE(used < 900);
}

NATIVE_TEST_MAIN(
    UnityDefaultTestRun(test_all_required_fields, "test_all_required_fields", __LINE__);
    UnityDefaultTestRun(test_phase_strings, "test_phase_strings", __LINE__);
    UnityDefaultTestRun(test_deg_back_calculation, "test_deg_back_calculation", __LINE__);
    UnityDefaultTestRun(test_slip_diagnostics, "test_slip_diagnostics", __LINE__);
    UnityDefaultTestRun(test_ledc_and_manual_echo, "test_ledc_and_manual_echo", __LINE__);
    UnityDefaultTestRun(test_buffer_too_small, "test_buffer_too_small", __LINE__);
    UnityDefaultTestRun(test_deterministic, "test_deterministic", __LINE__);
    UnityDefaultTestRun(test_block_fits_budget, "test_block_fits_budget", __LINE__);
)
