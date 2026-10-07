/*
 * rc_input 测试：通道有效性、模式阈值、仲裁合成（RC 覆盖配置 / 无效回落配置）。
 * 判定规则集中在这里；捕获层与模拟注入只负责喂值，不重复实现规则。
 */
#include "unity.h"
#include "unity_runner.h"
#include "rc_input.h"

void setUp(void) {}
void tearDown(void) {}

#define TIMEOUT_MS 100

/* 设置页的回落值：与下面用的透传值刻意取不同，好分辨结果来自哪边 */
#define CFG_FRONT 1600
#define CFG_REAR 1400

static rc_input_t rc;

static void feed_all(uint32_t now_ms, uint16_t mode, uint16_t front, uint16_t rear)
{
    TEST_ASSERT_EQUAL_INT(ESP_OK, rc_input_feed(&rc, RC_CH_MODE, mode, now_ms));
    TEST_ASSERT_EQUAL_INT(ESP_OK, rc_input_feed(&rc, RC_CH_DIFF_FRONT, front, now_ms));
    TEST_ASSERT_EQUAL_INT(ESP_OK, rc_input_feed(&rc, RC_CH_DIFF_REAR, rear, now_ms));
}

static void resolve(uint32_t now_ms, bool cfg_manual, rc_resolved_t *out)
{
    const uint32_t man[RC_DIFF_COUNT] = {CFG_FRONT, CFG_REAR};
    TEST_ASSERT_EQUAL_INT(ESP_OK,
                          rc_input_resolve(&rc, now_ms, TIMEOUT_MS, cfg_manual, man, out));
}

/* 1. 越界脉宽当场丢弃，且不刷新时间戳（坏脉冲不能把链路撑成"活的"） */
static void test_feed_rejects_out_of_range(void)
{
    rc_input_init(&rc);
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, rc_input_feed(&rc, RC_CH_MODE, 400, 1000));
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, rc_input_feed(&rc, RC_CH_MODE, 2600, 1000));
    TEST_ASSERT_FALSE(rc_input_valid(&rc, RC_CH_MODE, 1000, TIMEOUT_MS));

    /* 合法值之后再喂越界值：时间戳停在合法那次，按正常超时判丢 */
    TEST_ASSERT_EQUAL_INT(ESP_OK, rc_input_feed(&rc, RC_CH_MODE, 1700, 1000));
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, rc_input_feed(&rc, RC_CH_MODE, 9000, 1090));
    TEST_ASSERT_TRUE(rc_input_valid(&rc, RC_CH_MODE, 1100, TIMEOUT_MS));
    TEST_ASSERT_FALSE(rc_input_valid(&rc, RC_CH_MODE, 1101, TIMEOUT_MS));
}

/* 2. 从未收到过的通道判无效：now_ms 很小时也不能靠 last_ms=0 蒙成有效 */
static void test_never_seen_is_invalid(void)
{
    rc_input_init(&rc);
    TEST_ASSERT_FALSE(rc_input_valid(&rc, RC_CH_MODE, 0, TIMEOUT_MS));
    TEST_ASSERT_FALSE(rc_input_valid(&rc, RC_CH_MODE, 50, TIMEOUT_MS));
    TEST_ASSERT_FALSE(rc_input_valid(&rc, RC_CH_DIFF_FRONT, 50, TIMEOUT_MS));
}

/* 3. 超时边界：恰好等于阈值仍有效（now - last <= timeout），多 1ms 失效 */
static void test_timeout_boundary(void)
{
    rc_input_init(&rc);
    rc_input_feed(&rc, RC_CH_DIFF_REAR, 1200, 1000);
    TEST_ASSERT_TRUE(rc_input_valid(&rc, RC_CH_DIFF_REAR, 1100, TIMEOUT_MS));
    TEST_ASSERT_FALSE(rc_input_valid(&rc, RC_CH_DIFF_REAR, 1101, TIMEOUT_MS));
}

/* 4. 连续刷新保持有效（接收机 50Hz 正常工作时的状态） */
static void test_refresh_keeps_valid(void)
{
    rc_input_init(&rc);
    for (uint32_t t = 1000; t <= 5000; t += 50) {
        TEST_ASSERT_EQUAL_INT(ESP_OK, rc_input_feed(&rc, RC_CH_MODE, 1700, t));
        TEST_ASSERT_TRUE(rc_input_valid(&rc, RC_CH_MODE, t + 50, TIMEOUT_MS));
    }
}

/* 5. 模式阈值：1500 含在手动区，1499 归自动；两端极值同规则。
 * 配置值刻意与 RC 相反，验证"RC 有效时 RC 说了算" */
static void test_mode_threshold(void)
{
    rc_input_init(&rc);
    rc_resolved_t r;

    rc_input_feed(&rc, RC_CH_MODE, RC_MODE_THRESHOLD_US, 1000);
    resolve(1000, false, &r);
    TEST_ASSERT_TRUE(r.manual);
    TEST_ASSERT_TRUE(r.mode_from_rc);

    rc_input_feed(&rc, RC_CH_MODE, RC_MODE_THRESHOLD_US - 1, 1000);
    resolve(1000, true, &r);
    TEST_ASSERT_FALSE(r.manual);
    TEST_ASSERT_TRUE(r.mode_from_rc);

    rc_input_feed(&rc, RC_CH_MODE, 2000, 1000);
    resolve(1000, false, &r);
    TEST_ASSERT_TRUE(r.manual);

    rc_input_feed(&rc, RC_CH_MODE, 1000, 1000);
    resolve(1000, true, &r);
    TEST_ASSERT_FALSE(r.manual);
}

/* 6. 模式通道无效：模式回落设置页配置，两种配置各验一次 */
static void test_mode_falls_back_when_invalid(void)
{
    rc_input_init(&rc);
    rc_resolved_t r;

    resolve(1000, true, &r);
    TEST_ASSERT_TRUE(r.manual);
    TEST_ASSERT_FALSE(r.mode_from_rc);

    resolve(1000, false, &r);
    TEST_ASSERT_FALSE(r.manual);
    TEST_ASSERT_FALSE(r.mode_from_rc);
}

/* 7. 差速透传：通道有效原样透传，超时后该轴回落滑杆值（另一轴不受影响） */
static void test_diff_passthrough_and_fallback(void)
{
    rc_input_init(&rc);
    rc_resolved_t r;

    feed_all(1000, 1700, 1800, 1200);
    resolve(1000, false, &r);
    TEST_ASSERT_EQUAL_UINT32(1800, r.manual_us[0]);
    TEST_ASSERT_TRUE(r.target_from_rc[0]);
    TEST_ASSERT_EQUAL_UINT32(1200, r.manual_us[1]);
    TEST_ASSERT_TRUE(r.target_from_rc[1]);

    /* 只喂后轴：前轴超时回落滑杆，后轴换成新值 */
    rc_input_feed(&rc, RC_CH_DIFF_REAR, 1250, 1200);
    resolve(1200, false, &r);
    TEST_ASSERT_EQUAL_UINT32(CFG_FRONT, r.manual_us[0]);
    TEST_ASSERT_FALSE(r.target_from_rc[0]);
    TEST_ASSERT_EQUAL_UINT32(1250, r.manual_us[1]);
    TEST_ASSERT_TRUE(r.target_from_rc[1]);
}

/* 8. 模式与透传互相独立：模式通道掉了不挡透传，差速通道掉了不影响模式 */
static void test_mode_and_diff_are_independent(void)
{
    rc_input_init(&rc);
    rc_resolved_t r;

    /* 模式通道掉线，两根差速通道继续有信号：模式回落配置，透传照旧 */
    feed_all(1000, 1700, 1800, 1200);
    rc_input_feed(&rc, RC_CH_DIFF_FRONT, 1800, 1200);
    rc_input_feed(&rc, RC_CH_DIFF_REAR, 1200, 1200);
    resolve(1200, false, &r);
    TEST_ASSERT_FALSE(r.mode_from_rc);
    TEST_ASSERT_FALSE(r.manual);
    TEST_ASSERT_TRUE(r.target_from_rc[0]);
    TEST_ASSERT_TRUE(r.target_from_rc[1]);

    /* 反过来：模式通道有信号、差速通道全掉，模式仍由遥控说了算 */
    rc_input_init(&rc);
    feed_all(1000, 1900, 1800, 1200);
    rc_input_feed(&rc, RC_CH_MODE, 1900, 1200);
    resolve(1200, false, &r);
    TEST_ASSERT_TRUE(r.mode_from_rc);
    TEST_ASSERT_TRUE(r.manual);
    TEST_ASSERT_FALSE(r.target_from_rc[0]);
    TEST_ASSERT_FALSE(r.target_from_rc[1]);
    TEST_ASSERT_EQUAL_UINT32(CFG_FRONT, r.manual_us[0]);
    TEST_ASSERT_EQUAL_UINT32(CFG_REAR, r.manual_us[1]);
}

/* 9. 参数守卫 */
static void test_arg_guards(void)
{
    rc_input_init(&rc);
    const uint32_t man[RC_DIFF_COUNT] = {CFG_FRONT, CFG_REAR};
    rc_resolved_t r;

    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, rc_input_feed(NULL, 0, 1500, 0));
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, rc_input_feed(&rc, RC_CH_COUNT, 1500, 0));
    TEST_ASSERT_FALSE(rc_input_valid(NULL, 0, 0, TIMEOUT_MS));
    TEST_ASSERT_FALSE(rc_input_valid(&rc, RC_CH_COUNT, 0, TIMEOUT_MS));
    TEST_ASSERT_EQUAL_UINT32(0, rc_input_age_ms(NULL, 0, 5000));
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, rc_input_resolve(NULL, 0, TIMEOUT_MS, false, man, &r));
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, rc_input_resolve(&rc, 0, TIMEOUT_MS, false, NULL, &r));
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, rc_input_resolve(&rc, 0, TIMEOUT_MS, false, man, NULL));
}

/* 10. age 助手：从未收到返回 0，收到后给出距今天数（页面显示用） */
static void test_age_ms(void)
{
    rc_input_init(&rc);
    TEST_ASSERT_EQUAL_UINT32(0, rc_input_age_ms(&rc, RC_CH_MODE, 5000));

    rc_input_feed(&rc, RC_CH_MODE, 1700, 1000);
    TEST_ASSERT_EQUAL_UINT32(0, rc_input_age_ms(&rc, RC_CH_MODE, 1000));
    TEST_ASSERT_EQUAL_UINT32(430, rc_input_age_ms(&rc, RC_CH_MODE, 1430));
}

/* 11. init 清空全部状态（重新选择信号源时用） */
static void test_init_clears_state(void)
{
    feed_all(1000, 1700, 1800, 1200);
    rc_input_init(&rc);

    rc_resolved_t r;
    resolve(1000, true, &r);
    TEST_ASSERT_FALSE(r.mode_from_rc);
    TEST_ASSERT_TRUE(r.manual); /* 回落配置 */
    TEST_ASSERT_FALSE(r.target_from_rc[0]);
    TEST_ASSERT_FALSE(r.target_from_rc[1]);
    TEST_ASSERT_EQUAL_UINT32(CFG_FRONT, r.manual_us[0]);
    TEST_ASSERT_EQUAL_UINT32(CFG_REAR, r.manual_us[1]);
}

NATIVE_TEST_MAIN(
    UnityDefaultTestRun(test_feed_rejects_out_of_range, "test_feed_rejects_out_of_range", __LINE__);
    UnityDefaultTestRun(test_never_seen_is_invalid, "test_never_seen_is_invalid", __LINE__);
    UnityDefaultTestRun(test_timeout_boundary, "test_timeout_boundary", __LINE__);
    UnityDefaultTestRun(test_refresh_keeps_valid, "test_refresh_keeps_valid", __LINE__);
    UnityDefaultTestRun(test_mode_threshold, "test_mode_threshold", __LINE__);
    UnityDefaultTestRun(test_mode_falls_back_when_invalid, "test_mode_falls_back_when_invalid", __LINE__);
    UnityDefaultTestRun(test_diff_passthrough_and_fallback, "test_diff_passthrough_and_fallback", __LINE__);
    UnityDefaultTestRun(test_mode_and_diff_are_independent, "test_mode_and_diff_are_independent", __LINE__);
    UnityDefaultTestRun(test_arg_guards, "test_arg_guards", __LINE__);
    UnityDefaultTestRun(test_age_ms, "test_age_ms", __LINE__);
    UnityDefaultTestRun(test_init_clears_state, "test_init_clears_state", __LINE__);
)
