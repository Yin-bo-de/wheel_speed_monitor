/* config_params 行为测试：默认值、边界校验、逐字段判定、打包往返、版本号。 */
#include <string.h>
#include "unity.h"
#include "unity_runner.h"
#include "config_params.h"

void setUp(void) {}
void tearDown(void) {}

/* 1. 默认值：磁铁 1、轮径 100mm、alpha 0.3、4 轮全启用、模拟关、去抖关 */
static void test_defaults(void)
{
    cfg_params_t cfg;
    cfg_params_default(&cfg);
    TEST_ASSERT_EQUAL_INT(1, cfg.magnets);
    TEST_ASSERT_EQUAL_INT(100, cfg.wheel_diam_mm);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.3f, cfg.alpha);
    for (int i = 0; i < 4; i++) {
        TEST_ASSERT_TRUE(cfg.wheel_enabled[i]);
    }
    TEST_ASSERT_FALSE(cfg.sim_on);
    for (int i = 0; i < 4; i++) {
        TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, cfg.sim_rpm[i]);
    }
    TEST_ASSERT_EQUAL_INT(0, cfg.debounce_ms);
}

/* 2. 合法边界接受、非法拒绝 */
static void test_bounds(void)
{
    cfg_params_t cfg = {0};

    cfg.magnets = 1;
    TEST_ASSERT_EQUAL_INT(ESP_OK, cfg_params_validate(&cfg, "magnets"));
    cfg.magnets = 16;
    TEST_ASSERT_EQUAL_INT(ESP_OK, cfg_params_validate(&cfg, "magnets"));
    cfg.magnets = 0;
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, cfg_params_validate(&cfg, "magnets"));
    cfg.magnets = 17;
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, cfg_params_validate(&cfg, "magnets"));

    cfg.wheel_diam_mm = 10;
    TEST_ASSERT_EQUAL_INT(ESP_OK, cfg_params_validate(&cfg, "diam"));
    cfg.wheel_diam_mm = 300;
    TEST_ASSERT_EQUAL_INT(ESP_OK, cfg_params_validate(&cfg, "diam"));
    cfg.wheel_diam_mm = 9;
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, cfg_params_validate(&cfg, "diam"));
    cfg.wheel_diam_mm = 301;
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, cfg_params_validate(&cfg, "diam"));

    cfg.alpha = 0.05f;
    TEST_ASSERT_EQUAL_INT(ESP_OK, cfg_params_validate(&cfg, "alpha"));
    cfg.alpha = 1.0f;
    TEST_ASSERT_EQUAL_INT(ESP_OK, cfg_params_validate(&cfg, "alpha"));
    cfg.alpha = 0.0f;
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, cfg_params_validate(&cfg, "alpha"));
    cfg.alpha = 1.01f;
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, cfg_params_validate(&cfg, "alpha"));
}

/* 3. 逐字段判定：多字段输入时各字段独立判定并报告非法字段（命令层决定整条拒绝） */
static void test_per_field_validation(void)
{
    cfg_params_t cfg;
    cfg_params_default(&cfg);

    /* magnets=99、diam=0、alpha=0.5 → 逐个鉴 */
    cfg.magnets = 99;
    cfg.wheel_diam_mm = 0;
    cfg.alpha = 0.5f;
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, cfg_params_validate(&cfg, "magnets"));
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, cfg_params_validate(&cfg, "diam"));
    TEST_ASSERT_EQUAL_INT(ESP_OK, cfg_params_validate(&cfg, "alpha"));
    /* 未知字段名不报错（命令层自己处理忽略） */
    TEST_ASSERT_EQUAL_INT(ESP_OK, cfg_params_validate(&cfg, "unknown_field"));
}

/* 4. 打包→解包往返一致（含启用位图） */
static void test_pack_unpack_roundtrip(void)
{
    cfg_params_t a, b;
    cfg_params_default(&a);
    a.magnets = 4;
    a.wheel_diam_mm = 250;
    a.alpha = 0.8f;
    a.wheel_enabled[0] = false;
    a.wheel_enabled[2] = false;
    a.sim_on = true;
    a.sim_rpm[3] = 1234.5f;
    a.debounce_ms = 20;

    uint8_t blob[CFG_PARAMS_BLOB_SIZE];
    TEST_ASSERT_EQUAL_INT(ESP_OK, cfg_params_pack(&a, blob, sizeof(blob)));
    TEST_ASSERT_EQUAL_INT(ESP_OK, cfg_params_unpack(blob, sizeof(blob), &b));

    TEST_ASSERT_EQUAL_INT(a.magnets, b.magnets);
    TEST_ASSERT_EQUAL_INT(a.wheel_diam_mm, b.wheel_diam_mm);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, a.alpha, b.alpha);
    for (int i = 0; i < 4; i++) {
        TEST_ASSERT_EQUAL_INT(a.wheel_enabled[i], b.wheel_enabled[i]);
        TEST_ASSERT_FLOAT_WITHIN(0.0001f, a.sim_rpm[i], b.sim_rpm[i]);
    }
    TEST_ASSERT_EQUAL_INT(a.sim_on, b.sim_on);
    TEST_ASSERT_EQUAL_INT(a.debounce_ms, b.debounce_ms);
}

/* 5. blob 版本号字段存在（v2 迁移预留） */
static void test_blob_version_field(void)
{
    cfg_params_t cfg;
    cfg_params_default(&cfg);
    uint8_t blob[CFG_PARAMS_BLOB_SIZE];
    cfg_params_pack(&cfg, blob, sizeof(blob));
    /* 版本字段在 blob 头部：前 1 字节 = 版本（v1 = 1） */
    TEST_ASSERT_EQUAL_INT(CFG_PARAMS_BLOB_VERSION, blob[0]);
}

NATIVE_TEST_MAIN(
    UnityDefaultTestRun(test_defaults, "test_defaults", __LINE__);
    UnityDefaultTestRun(test_bounds, "test_bounds", __LINE__);
    UnityDefaultTestRun(test_per_field_validation, "test_per_field_validation", __LINE__);
    UnityDefaultTestRun(test_pack_unpack_roundtrip, "test_pack_unpack_roundtrip", __LINE__);
    UnityDefaultTestRun(test_blob_version_field, "test_blob_version_field", __LINE__);
)
