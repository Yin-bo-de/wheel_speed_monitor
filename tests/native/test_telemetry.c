/* telemetry 采集框架行为测试：注册表、采样遍历、帧聚合、容错隔离、重复/表满拒绝。 */
#include <string.h>
#include "unity.h"
#include "unity_runner.h"
#include "telemetry.h"


/* Unity 2.6 没有 TEST_ASSERT_STRING_CONTAINS，提供本地助手 */
static void assert_contains(const char *haystack, const char *needle)
{
    if (strstr(haystack, needle) == NULL) {
        TEST_FAIL_MESSAGE(needle);
    }
}

void setUp(void) {}
void tearDown(void) {}

/* ---- mock 采集器：sample 记为一次调用；render 输出固定文本 ---- */
typedef struct {
    int sample_count;
    const char *render_text;
    esp_err_t sample_err;
} mock_ctx_t;

static esp_err_t mock_init(void *ctx) { (void)ctx; return ESP_OK; }

static esp_err_t mock_sample(void *ctx, uint32_t now_ms)
{
    (void)now_ms;
    mock_ctx_t *m = (mock_ctx_t *)ctx;
    m->sample_count++;
    return m->sample_err;
}

static esp_err_t mock_render(void *ctx, char *buf, size_t len, size_t *used)
{
    mock_ctx_t *m = (mock_ctx_t *)ctx;
    size_t n = strlen(m->render_text);
    if (len < n + 1) {
        return ESP_ERR_INVALID_SIZE;
    }
    memcpy(buf, m->render_text, n);
    buf[n] = '\0';
    *used = n;
    return ESP_OK;
}

static esp_err_t mock_reset_baseline(void *ctx) { (void)ctx; return ESP_OK; }

static const telem_collector_ops_t mock_ops = {
    .type = TELEM_TYPE_MOCK_BASE,
    .init = mock_init,
    .sample = mock_sample,
    .render = mock_render,
    .reset_baseline = mock_reset_baseline,
};

/* 1. 注册一个 mock 采集器后 sample_all 逐一遍历调用 */
static void test_register_then_sample_all(void)
{
    telem_reset_for_test();
    mock_ctx_t ctx = {0, "x", ESP_OK};
    TEST_ASSERT_EQUAL_INT(ESP_OK, telem_register(&mock_ops, &ctx));
    TEST_ASSERT_EQUAL_INT(ESP_OK, telem_sample_all(1000));
    TEST_ASSERT_EQUAL_INT(1, ctx.sample_count);
    TEST_ASSERT_EQUAL_INT(ESP_OK, telem_sample_all(1050));
    TEST_ASSERT_EQUAL_INT(2, ctx.sample_count);
}

/* 2. 注册 3 个不同 type 的 mock 采集器，sample_all 全部被遍历（聚合正确） */
static void test_three_collectors_aggregate(void)
{
    telem_reset_for_test();
    telem_collector_ops_t o1 = mock_ops, o2 = mock_ops, o3 = mock_ops;
    o1.type = TELEM_TYPE_MOCK_BASE;
    o2.type = TELEM_TYPE_MOCK_BASE + 1;
    o3.type = TELEM_TYPE_MOCK_BASE + 2;
    mock_ctx_t c1 = {0, "one", ESP_OK};
    mock_ctx_t c2 = {0, "two", ESP_OK};
    mock_ctx_t c3 = {0, "three", ESP_OK};
    TEST_ASSERT_EQUAL_INT(ESP_OK, telem_register(&o1, &c1));
    TEST_ASSERT_EQUAL_INT(ESP_OK, telem_register(&o2, &c2));
    TEST_ASSERT_EQUAL_INT(ESP_OK, telem_register(&o3, &c3));

    TEST_ASSERT_EQUAL_INT(ESP_OK, telem_sample_all(1000));
    TEST_ASSERT_EQUAL_INT(1, c1.sample_count);
    TEST_ASSERT_EQUAL_INT(1, c2.sample_count);
    TEST_ASSERT_EQUAL_INT(1, c3.sample_count);
}

/* 3. render_frame 输出含 sys 块与各采集器块、顺序稳定，且 seq/时间戳回落 */
static void test_render_frame_structure(void)
{
    telem_reset_for_test();
    telem_collector_ops_t o1 = mock_ops, o2 = mock_ops;
    o1.type = TELEM_TYPE_MOCK_BASE;
    o2.type = TELEM_TYPE_MOCK_BASE + 1;
    mock_ctx_t c1 = {0, "WHEEL_BLOCK", ESP_OK};
    mock_ctx_t c2 = {0, "IMU_BLOCK", ESP_OK};
    telem_register(&o1, &c1);
    telem_register(&o2, &c2);

    char buf[512];
    size_t used = 0;
    TEST_ASSERT_EQUAL_INT(ESP_OK, telem_render_frame(buf, sizeof(buf), &used, 42, 778899, 204800));
    assert_contains(buf, "\"seq\":42");
    assert_contains(buf, "\"t\":778899");
    assert_contains(buf, "\"free_heap\":204800");
    assert_contains(buf, "\"uptime_s\":778");
    /* sys 块与两块采集器都出现，顺序：sys 最先 */
    TEST_ASSERT_TRUE(strstr(buf, "\"sys\"") < strstr(buf, "WHEEL_BLOCK"));
    TEST_ASSERT_TRUE(strstr(buf, "WHEEL_BLOCK") < strstr(buf, "IMU_BLOCK"));
    assert_contains(buf, "\"collectors\":[");
}

/* 4. 未注册任何采集器时帧渲染不崩溃（空 collectors） */
static void test_render_empty_registry(void)
{
    telem_reset_for_test();
    char buf[512];
    size_t used = 0;
    TEST_ASSERT_EQUAL_INT(ESP_OK, telem_render_frame(buf, sizeof(buf), &used, 1, 0, 0));
    assert_contains(buf, "\"type\":\"telemetry\"");
    assert_contains(buf, "\"collectors\":[]");
}

/* 5. 重复注册同一 type 拒绝并返回错误码 */
static void test_duplicate_type_rejected(void)
{
    telem_reset_for_test();
    mock_ctx_t c1 = {0, "x", ESP_OK};
    mock_ctx_t c2 = {0, "y", ESP_OK};
    TEST_ASSERT_EQUAL_INT(ESP_OK, telem_register(&mock_ops, &c1));
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, telem_register(&mock_ops, &c2));
}

/* 6. 单个采集器 sample 失败不影响其他采集器（容错隔离） */
static void test_sample_error_isolated(void)
{
    telem_reset_for_test();
    telem_collector_ops_t o_ok = mock_ops, o_err = mock_ops;
    o_ok.type = TELEM_TYPE_MOCK_BASE;
    o_err.type = TELEM_TYPE_MOCK_BASE + 1;
    mock_ctx_t c_ok = {0, "ok", ESP_OK};
    mock_ctx_t c_err = {0, "err", ESP_ERR_TIMEOUT};
    telem_register(&o_ok, &c_ok);
    telem_register(&o_err, &c_err);

    /* 整体返回失败标记（存在采样错误），但好的采集器仍被遍历 */
    TEST_ASSERT_EQUAL_INT(ESP_ERR_TIMEOUT, telem_sample_all(1000));
    TEST_ASSERT_EQUAL_INT(1, c_ok.sample_count);
    TEST_ASSERT_EQUAL_INT(1, c_err.sample_count);
}

/* 7. 注册表满时报错且不越界：超过容量必须返回 ESP_ERR_NO_MEM */
static void test_registry_full(void)
{
    telem_reset_for_test();
    bool got_full_error = false;
    for (int i = 0; i < TELEM_MAX_COLLECTORS + 5; i++) {
        mock_ctx_t c = {0, "x", ESP_OK};
        telem_collector_ops_t o = mock_ops;
        o.type = TELEM_TYPE_MOCK_BASE + (telem_type_t)i;
        esp_err_t e = telem_register(&o, &c);
        if (e == ESP_ERR_NO_MEM) {
            got_full_error = true;
            break;
        }
        TEST_ASSERT_EQUAL_INT(ESP_OK, e);
    }
    TEST_ASSERT_TRUE(got_full_error);
}

NATIVE_TEST_MAIN(
    UnityDefaultTestRun(test_register_then_sample_all, "test_register_then_sample_all", __LINE__);
    UnityDefaultTestRun(test_three_collectors_aggregate, "test_three_collectors_aggregate", __LINE__);
    UnityDefaultTestRun(test_render_frame_structure, "test_render_frame_structure", __LINE__);
    UnityDefaultTestRun(test_render_empty_registry, "test_render_empty_registry", __LINE__);
    UnityDefaultTestRun(test_duplicate_type_rejected, "test_duplicate_type_rejected", __LINE__);
    UnityDefaultTestRun(test_sample_error_isolated, "test_sample_error_isolated", __LINE__);
    UnityDefaultTestRun(test_registry_full, "test_registry_full", __LINE__);
)
