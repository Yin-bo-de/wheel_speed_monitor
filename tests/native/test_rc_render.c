/*
 * rc_render 测试：rc JSON 块的字段完整性、通道有效性渲染、目标来源标记、
 * 缓冲不足、确定性、块长预算。
 */
#include <string.h>

#include "unity.h"
#include "unity_runner.h"
#include "rc_render.h"

/* Unity 2.6 没有 assert_contains，提供本地助手 */
static void assert_contains(const char *haystack, const char *needle)
{
    if (strstr(haystack, needle) == NULL) {
        TEST_FAIL_MESSAGE(needle);
    }
}

void setUp(void) {}
void tearDown(void) {}

static rc_render_state_t rs;

static void fresh(void)
{
    memset(&rs, 0, sizeof(rs));
    rs.src = "gpio";
    rs.mode = "auto";
    rs.mode_src = "cfg";
    rs.sim = false;
    rs.sim_us[0] = 1000;
    rs.sim_us[1] = 1500;
    rs.sim_us[2] = 1500;
    for (int i = 0; i < RC_CH_COUNT; i++) {
        rs.ch[i].us = 1500;
        rs.ch[i].valid = true;
        rs.ch[i].seen = true;
        rs.ch[i].age_ms = 12;
    }
    for (int i = 0; i < RC_DIFF_COUNT; i++) {
        rs.target[i].us = 1500;
        rs.target[i].from_rc = false;
    }
}

/* 1. 必需字段齐全：三路通道、两个目标、cfg 回显 */
static void test_all_required_fields(void)
{
    fresh();
    char buf[512];
    size_t used = 0;
    TEST_ASSERT_EQUAL_INT(ESP_OK, rc_render_block(&rs, buf, sizeof(buf), &used));
    assert_contains(buf, "\"type\":\"rc\"");
    assert_contains(buf, "\"src\":\"gpio\"");
    assert_contains(buf, "\"mode\":\"auto\"");
    assert_contains(buf, "\"mode_src\":\"cfg\"");
    assert_contains(buf, "\"ch\":[");
    assert_contains(buf, "\"us\":1500");
    assert_contains(buf, "\"valid\":true");
    assert_contains(buf, "\"seen\":true");
    assert_contains(buf, "\"age_ms\":12");
    assert_contains(buf, "\"target\":[");
    assert_contains(buf, "\"from_rc\":false");
    assert_contains(buf, "\"cfg\":{\"sim\":false,\"sim_us\":[1000,1500,1500]}");
    /* 三路通道 + 两个目标：分隔符数量对得上（不出现空数组或尾逗号） */
    assert_contains(buf, "\"seen\":true,\"age_ms\":12},{\"us\":1500");
    assert_contains(buf, "\"from_rc\":false},{\"us\":1500");
}

/* 2. 无效与从未收到的通道分开渲染：seen=false 不能显示成一个假年龄 */
static void test_channel_validity_rendering(void)
{
    fresh();
    rs.ch[0].valid = false;
    rs.ch[0].age_ms = 430;
    rs.ch[1].valid = false;
    rs.ch[1].seen = false;
    rs.ch[1].age_ms = 0;
    char buf[512];
    size_t used = 0;
    rc_render_block(&rs, buf, sizeof(buf), &used);
    assert_contains(buf, "{\"us\":1500,\"valid\":false,\"seen\":true,\"age_ms\":430}");
    assert_contains(buf, "{\"us\":1500,\"valid\":false,\"seen\":false,\"age_ms\":0}");
}

/* 3. 目标来源标记：一根轴透传、一根轴回落 */
static void test_target_sources(void)
{
    fresh();
    rs.target[0].us = 1800;
    rs.target[0].from_rc = true;
    rs.target[1].us = 1400;
    rs.target[1].from_rc = false;
    char buf[512];
    size_t used = 0;
    rc_render_block(&rs, buf, sizeof(buf), &used);
    assert_contains(buf, "{\"us\":1800,\"from_rc\":true}");
    assert_contains(buf, "{\"us\":1400,\"from_rc\":false}");
}

/* 4. 生效模式与来源：手动来自遥控 / 模拟源 */
static void test_mode_and_src_variants(void)
{
    fresh();
    rs.src = "sim";
    rs.mode = "manual";
    rs.mode_src = "rc";
    rs.sim = true;
    char buf[512];
    size_t used = 0;
    rc_render_block(&rs, buf, sizeof(buf), &used);
    assert_contains(buf, "\"src\":\"sim\"");
    assert_contains(buf, "\"mode\":\"manual\"");
    assert_contains(buf, "\"mode_src\":\"rc\"");
    assert_contains(buf, "\"cfg\":{\"sim\":true,");
}

/* 5. 缓冲不足与空指针 */
static void test_buffer_too_small(void)
{
    fresh();
    char tiny[16];
    size_t used = 0;
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_SIZE, rc_render_block(&rs, tiny, sizeof(tiny), &used));
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, rc_render_block(NULL, tiny, sizeof(tiny), &used));
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, rc_render_block(&rs, NULL, sizeof(tiny), &used));
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, rc_render_block(&rs, tiny, sizeof(tiny), NULL));
}

/* 6. 同一快照两次渲染字节一致 */
static void test_deterministic(void)
{
    fresh();
    rs.mode = "manual";
    rs.mode_src = "rc";
    rs.target[0].from_rc = true;
    rs.ch[2].valid = false;
    char b1[512], b2[512];
    size_t u1 = 0, u2 = 0;
    rc_render_block(&rs, b1, sizeof(b1), &u1);
    rc_render_block(&rs, b2, sizeof(b2), &u2);
    TEST_ASSERT_EQUAL_INT(u1, u2);
    TEST_ASSERT_EQUAL_STRING(b1, b2);
}

/* 7. 整块长度在遥测块缓冲（1024）之内，且留有余量给后续字段 */
static void test_block_fits_budget(void)
{
    fresh();
    char buf[1024];
    size_t used = 0;
    TEST_ASSERT_EQUAL_INT(ESP_OK, rc_render_block(&rs, buf, sizeof(buf), &used));
    TEST_ASSERT_TRUE(used < 512);
}

NATIVE_TEST_MAIN(
    UnityDefaultTestRun(test_all_required_fields, "test_all_required_fields", __LINE__);
    UnityDefaultTestRun(test_channel_validity_rendering, "test_channel_validity_rendering", __LINE__);
    UnityDefaultTestRun(test_target_sources, "test_target_sources", __LINE__);
    UnityDefaultTestRun(test_mode_and_src_variants, "test_mode_and_src_variants", __LINE__);
    UnityDefaultTestRun(test_buffer_too_small, "test_buffer_too_small", __LINE__);
    UnityDefaultTestRun(test_deterministic, "test_deterministic", __LINE__);
    UnityDefaultTestRun(test_block_fits_budget, "test_block_fits_budget", __LINE__);
)
