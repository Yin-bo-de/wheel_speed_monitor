/* config_params 行为测试：默认值、边界校验、逐字段判定、打包往返、版本号。 */
#include <string.h>
#include "unity.h"
#include "unity_runner.h"
#include "config_params.h"

void setUp(void) {}
void tearDown(void) {}

/* 1. 默认值：磁铁 1、轮径 105mm、alpha 0.3、4 轮全启用、模拟关、去抖关 */
static void test_defaults(void)
{
    cfg_params_t cfg;
    cfg_params_default(&cfg);
    TEST_ASSERT_EQUAL_INT(1, cfg.magnets);
    TEST_ASSERT_EQUAL_INT(105, cfg.wheel_diam_mm);
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

/* 6. 舵机默认值：两档脉宽前后各一组 */
static void test_servo_defaults(void)
{
    cfg_params_t cfg;
    cfg_params_default(&cfg);
    TEST_ASSERT_TRUE(cfg.servo_sim);
    for (int i = 0; i < CFG_SERVO_COUNT; i++) {
        TEST_ASSERT_TRUE(cfg.servo_en[i]);
        TEST_ASSERT_EQUAL_INT(1500, cfg.servo_unlock_us[i]);
        TEST_ASSERT_EQUAL_INT(2000, cfg.servo_lock_us[i]);
        TEST_ASSERT_EQUAL_INT(1500, cfg.servo_manual_us[i]);
    }
    TEST_ASSERT_EQUAL_INT(CFG_SERVO_MODE_AUTO, cfg.servo_mode);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.30f, cfg.slip_engage_ratio);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 10.0f, cfg.slip_min_rpm);
    TEST_ASSERT_EQUAL_INT(3000, cfg.lock_hold_ms);
    TEST_ASSERT_EQUAL_INT(30000, cfg.lock_hold_max_ms);
    TEST_ASSERT_EQUAL_INT(2000, cfg.probe_window_ms);
    TEST_ASSERT_EQUAL_INT(3000, cfg.servo_sim_speed_us_s);
}

/* 7. 两档脉宽逐通道判边界：越界即拒；一个通道非法不掩盖另一个通道的合法值 */
static void test_servo_pulse_bounds(void)
{
    cfg_params_t cfg;
    cfg_params_default(&cfg);

    cfg.servo_unlock_us[0] = CFG_SERVO_US_MIN;
    cfg.servo_unlock_us[1] = CFG_SERVO_US_MAX;
    TEST_ASSERT_EQUAL_INT(ESP_OK, cfg_params_validate(&cfg, "servo_unlock_us"));

    cfg.servo_unlock_us[1] = CFG_SERVO_US_MIN - 1; /* 后轴越下限 */
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, cfg_params_validate(&cfg, "servo_unlock_us"));

    cfg.servo_lock_us[0] = CFG_SERVO_US_MAX + 1; /* 前轴越上限 */
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, cfg_params_validate(&cfg, "servo_lock_us"));
    cfg.servo_lock_us[0] = 1800;
    TEST_ASSERT_EQUAL_INT(ESP_OK, cfg_params_validate(&cfg, "servo_lock_us"));

    /* 两档相同是允许的：该轴锁定时不动，调 PWM 链路时正是要这个 */
    cfg.servo_lock_us[0] = cfg.servo_unlock_us[0];
    TEST_ASSERT_EQUAL_INT(ESP_OK, cfg_params_validate(&cfg, "servo_lock_us"));

    /* 手动目标与两档同量程、同判法 */
    cfg.servo_manual_us[1] = CFG_SERVO_US_MAX + 1;
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, cfg_params_validate(&cfg, "servo_manual_us"));
    cfg.servo_manual_us[1] = CFG_SERVO_US_MAX;
    TEST_ASSERT_EQUAL_INT(ESP_OK, cfg_params_validate(&cfg, "servo_manual_us"));
}

/* 8. 锁定策略与打滑判定参数边界 */
static void test_lock_policy_bounds(void)
{
    cfg_params_t cfg;
    cfg_params_default(&cfg);

    cfg.lock_hold_ms = 200;
    cfg.lock_hold_max_ms = 200;
    TEST_ASSERT_EQUAL_INT(ESP_OK, cfg_params_validate(&cfg, "lock_hold_ms"));
    TEST_ASSERT_EQUAL_INT(ESP_OK, cfg_params_validate(&cfg, "lock_hold_max_ms"));
    cfg.lock_hold_ms = 199;
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, cfg_params_validate(&cfg, "lock_hold_ms"));

    cfg.lock_hold_ms = 3000;
    cfg.lock_hold_max_ms = 2999; /* 上限低于初始 → 翻倍无意义 */
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, cfg_params_validate(&cfg, "lock_hold_max_ms"));

    cfg.probe_window_ms = 200;
    TEST_ASSERT_EQUAL_INT(ESP_OK, cfg_params_validate(&cfg, "probe_window_ms"));
    cfg.probe_window_ms = 199;
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, cfg_params_validate(&cfg, "probe_window_ms"));
    cfg.probe_window_ms = 10001;
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, cfg_params_validate(&cfg, "probe_window_ms"));

    cfg.slip_engage_ratio = 1.0f;
    TEST_ASSERT_EQUAL_INT(ESP_OK, cfg_params_validate(&cfg, "slip_engage_ratio"));
    cfg.slip_engage_ratio = 0.04f;
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, cfg_params_validate(&cfg, "slip_engage_ratio"));

    cfg.slip_min_rpm = 0.0f;
    TEST_ASSERT_EQUAL_INT(ESP_OK, cfg_params_validate(&cfg, "slip_min_rpm"));
    cfg.slip_min_rpm = 2001.0f;
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, cfg_params_validate(&cfg, "slip_min_rpm"));

    cfg.servo_sim_speed_us_s = 100;
    TEST_ASSERT_EQUAL_INT(ESP_OK, cfg_params_validate(&cfg, "servo_sim_speed_us_s"));
    cfg.servo_sim_speed_us_s = 99;
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, cfg_params_validate(&cfg, "servo_sim_speed_us_s"));

    cfg.servo_mode = CFG_SERVO_MODE_MANUAL;
    TEST_ASSERT_EQUAL_INT(ESP_OK, cfg_params_validate(&cfg, "servo_mode"));
    cfg.servo_mode = 2;
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, cfg_params_validate(&cfg, "servo_mode"));
}

/* 9. v3 打包往返：舵机字段全量保真，前后两组两档脉宽互不串位 */
static void test_pack_unpack_v3_roundtrip(void)
{
    cfg_params_t a, b;
    cfg_params_default(&a);
    a.servo_sim = false;
    a.servo_en[1] = false;
    a.servo_unlock_us[0] = 1450;
    a.servo_unlock_us[1] = 1550;
    a.servo_lock_us[0] = 2200;
    a.servo_lock_us[1] = 800; /* 后轴装反：锁定去下限那一端 */
    a.servo_mode = CFG_SERVO_MODE_MANUAL;
    a.servo_manual_us[0] = 1700;
    a.servo_manual_us[1] = 1300;
    a.slip_engage_ratio = 0.42f;
    a.slip_min_rpm = 25.0f;
    a.lock_hold_ms = 4500;
    a.lock_hold_max_ms = 12000;
    a.probe_window_ms = 2500;
    a.servo_sim_speed_us_s = 800;

    uint8_t blob[CFG_PARAMS_BLOB_SIZE];
    TEST_ASSERT_EQUAL_INT(ESP_OK, cfg_params_pack(&a, blob, sizeof(blob)));
    TEST_ASSERT_EQUAL_INT(CFG_PARAMS_BLOB_VERSION, blob[0]);
    TEST_ASSERT_EQUAL_INT(ESP_OK, cfg_params_unpack(blob, sizeof(blob), &b));

    TEST_ASSERT_EQUAL_INT(a.servo_sim, b.servo_sim);
    TEST_ASSERT_EQUAL_INT(a.servo_en[0], b.servo_en[0]);
    TEST_ASSERT_EQUAL_INT(a.servo_en[1], b.servo_en[1]);
    for (int i = 0; i < CFG_SERVO_COUNT; i++) {
        TEST_ASSERT_EQUAL_INT(a.servo_unlock_us[i], b.servo_unlock_us[i]);
        TEST_ASSERT_EQUAL_INT(a.servo_lock_us[i], b.servo_lock_us[i]);
    }
    TEST_ASSERT_EQUAL_INT(a.servo_mode, b.servo_mode);
    TEST_ASSERT_EQUAL_INT(a.servo_manual_us[0], b.servo_manual_us[0]);
    TEST_ASSERT_EQUAL_INT(a.servo_manual_us[1], b.servo_manual_us[1]);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, a.slip_engage_ratio, b.slip_engage_ratio);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, a.slip_min_rpm, b.slip_min_rpm);
    TEST_ASSERT_EQUAL_INT(a.lock_hold_ms, b.lock_hold_ms);
    TEST_ASSERT_EQUAL_INT(a.lock_hold_max_ms, b.lock_hold_max_ms);
    TEST_ASSERT_EQUAL_INT(a.probe_window_ms, b.probe_window_ms);
    TEST_ASSERT_EQUAL_INT(a.servo_sim_speed_us_s, b.servo_sim_speed_us_s);
}

/* 按一期（v1）布局手工拼一条 32 字节 blob。刻意不复用打包代码——
 * 迁移测试要的是一个独立于当前实现的"旧固件写下的字节流"。 */
static void build_v1_blob(uint8_t buf[32])
{
    memset(buf, 0, 32);
    buf[0] = 1; /* v1 版本号 */
    buf[1] = 6; /* magnets */
    uint32_t diam = 180;
    memcpy(buf + 2, &diam, 4);
    float alpha = 0.55f;
    memcpy(buf + 6, &alpha, 4);
    buf[10] = 0x05; /* 位图：轮 0、轮 2 启用 */
    buf[11] = 1;    /* sim_on */
    float sr[4] = {10.0f, 20.0f, 30.0f, 40.0f};
    memcpy(buf + 12, sr, 16);
    uint32_t deb = 25;
    memcpy(buf + 28, &deb, 4);
}

/* 10. v1 blob 迁移：旧字段照常解出，新字段取默认值而非零值（升级不丢配置） */
static void test_v1_blob_migrates(void)
{
    uint8_t v1[32];
    build_v1_blob(v1);

    cfg_params_t cfg;
    TEST_ASSERT_EQUAL_INT(ESP_OK, cfg_params_unpack(v1, sizeof(v1), &cfg));

    TEST_ASSERT_EQUAL_INT(6, cfg.magnets);
    TEST_ASSERT_EQUAL_INT(180, cfg.wheel_diam_mm);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.55f, cfg.alpha);
    TEST_ASSERT_TRUE(cfg.wheel_enabled[0]);
    TEST_ASSERT_FALSE(cfg.wheel_enabled[1]);
    TEST_ASSERT_TRUE(cfg.wheel_enabled[2]);
    TEST_ASSERT_FALSE(cfg.wheel_enabled[3]);
    TEST_ASSERT_TRUE(cfg.sim_on);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 30.0f, cfg.sim_rpm[2]);
    TEST_ASSERT_EQUAL_INT(25, cfg.debounce_ms);

    cfg_params_t def;
    cfg_params_default(&def);
    for (int i = 0; i < CFG_SERVO_COUNT; i++) {
        TEST_ASSERT_EQUAL_INT(def.servo_unlock_us[i], cfg.servo_unlock_us[i]);
        TEST_ASSERT_EQUAL_INT(def.servo_lock_us[i], cfg.servo_lock_us[i]);
    }
    TEST_ASSERT_EQUAL_INT(def.lock_hold_ms, cfg.lock_hold_ms);
    TEST_ASSERT_EQUAL_INT(def.lock_hold_max_ms, cfg.lock_hold_max_ms);
    TEST_ASSERT_EQUAL_INT(def.probe_window_ms, cfg.probe_window_ms);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, def.slip_engage_ratio, cfg.slip_engage_ratio);
    TEST_ASSERT_EQUAL_INT(def.servo_mode, cfg.servo_mode);
    TEST_ASSERT_TRUE(cfg.servo_sim);
}

/* 按二期（v2）布局手工拼一条 72 字节 blob：共享的 min/center/max + invert 位图。
 * 同样刻意不复用打包代码——要的是"旧固件写下的字节流"。 */
static void build_v2_blob(uint8_t buf[72])
{
    memset(buf, 0, 72);
    buf[0] = 2;    /* v2 版本号 */
    buf[32] = 1;   /* servo_sim */
    buf[33] = 0x03; /* 位图：两通道都使能 */
    buf[34] = (uint8_t)(CFG_SERVO_MODE_MANUAL | (0x02 << 4)); /* 低 4 位模式，高 4 位 invert */
    uint16_t min_us = 800, center_us = 1450, max_us = 2200;
    memcpy(buf + 35, &min_us, 2);
    memcpy(buf + 37, &center_us, 2);
    memcpy(buf + 39, &max_us, 2);
    uint16_t manual[2] = {1700, 1300};
    memcpy(buf + 41, manual, 4);
    float engage = 0.42f, min_rpm = 25.0f;
    memcpy(buf + 45, &engage, 4);
    memcpy(buf + 49, &min_rpm, 4);
    uint32_t hold = 4500, hold_max = 12000, probe = 2500, speed = 800;
    memcpy(buf + 53, &hold, 4);
    memcpy(buf + 57, &hold_max, 4);
    memcpy(buf + 61, &probe, 4);
    memcpy(buf + 65, &speed, 4);
}

/* 11. v2 blob 迁移成两档：旧的共享量程换算规则 = 中位作解锁档、
 * invert 决定锁定档去哪一端。换算前后行为必须逐位等价，升级不用重新标定。 */
static void test_v2_blob_migrates_to_two_detents(void)
{
    uint8_t v2[72];
    build_v2_blob(v2);

    cfg_params_t cfg;
    TEST_ASSERT_EQUAL_INT(ESP_OK, cfg_params_unpack(v2, sizeof(v2), &cfg));

    for (int i = 0; i < CFG_SERVO_COUNT; i++) {
        TEST_ASSERT_EQUAL_INT(1450, cfg.servo_unlock_us[i]); /* 旧中位 */
    }
    TEST_ASSERT_EQUAL_INT(2200, cfg.servo_lock_us[0]); /* 未反向 → 去上限 */
    TEST_ASSERT_EQUAL_INT(800, cfg.servo_lock_us[1]);  /* 反向 → 去下限 */

    /* 同段其余字段照常解出，不受迁移影响 */
    TEST_ASSERT_EQUAL_INT(1700, cfg.servo_manual_us[0]);
    TEST_ASSERT_EQUAL_INT(1300, cfg.servo_manual_us[1]);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.42f, cfg.slip_engage_ratio);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 25.0f, cfg.slip_min_rpm);
    TEST_ASSERT_EQUAL_INT(4500, cfg.lock_hold_ms);
    TEST_ASSERT_EQUAL_INT(12000, cfg.lock_hold_max_ms);
    TEST_ASSERT_EQUAL_INT(2500, cfg.probe_window_ms);
    TEST_ASSERT_EQUAL_INT(800, cfg.servo_sim_speed_us_s);
    TEST_ASSERT_EQUAL_INT(CFG_SERVO_MODE_MANUAL, cfg.servo_mode);
    TEST_ASSERT_TRUE(cfg.servo_sim);
    TEST_ASSERT_TRUE(cfg.servo_en[0]);
    TEST_ASSERT_TRUE(cfg.servo_en[1]);
}

/* 12. 长度不足与未知版本拒绝；各版旧 blob 的合法长度仍照收 */
static void test_unpack_rejects_bad_len_and_version(void)
{
    cfg_params_t cfg;
    uint8_t blob[CFG_PARAMS_BLOB_SIZE];
    cfg_params_default(&cfg);
    cfg_params_pack(&cfg, blob, sizeof(blob));

    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, cfg_params_unpack(blob, CFG_PARAMS_BLOB_SIZE - 1, &cfg));
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, cfg_params_unpack(blob, 0, &cfg));

    blob[0] = 0;
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_VERSION, cfg_params_unpack(blob, sizeof(blob), &cfg));
    blob[0] = 4;
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_VERSION, cfg_params_unpack(blob, sizeof(blob), &cfg));

    /* v1 版本号但只有 31 字节：残缺，拒绝 */
    blob[0] = 1;
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, cfg_params_unpack(blob, 31, &cfg));
    /* v1 版本号 + 32 字节：旧固件写下的正是这个形态，必须能解 */
    TEST_ASSERT_EQUAL_INT(ESP_OK, cfg_params_unpack(blob, 32, &cfg));

    /* v2 同理：71 字节残缺拒绝，72 字节照收 */
    blob[0] = 2;
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, cfg_params_unpack(blob, 71, &cfg));
    TEST_ASSERT_EQUAL_INT(ESP_OK, cfg_params_unpack(blob, 72, &cfg));

    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, cfg_params_unpack(NULL, sizeof(blob), &cfg));
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, cfg_params_unpack(blob, sizeof(blob), NULL));
}

/* 13. 打包缓冲不足与空指针 */
static void test_pack_guards(void)
{
    cfg_params_t cfg;
    cfg_params_default(&cfg);
    uint8_t blob[CFG_PARAMS_BLOB_SIZE];
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG,
                          cfg_params_pack(&cfg, blob, CFG_PARAMS_BLOB_SIZE - 1));
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, cfg_params_pack(NULL, blob, sizeof(blob)));
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, cfg_params_pack(&cfg, NULL, sizeof(blob)));
}

NATIVE_TEST_MAIN(
    UnityDefaultTestRun(test_defaults, "test_defaults", __LINE__);
    UnityDefaultTestRun(test_bounds, "test_bounds", __LINE__);
    UnityDefaultTestRun(test_per_field_validation, "test_per_field_validation", __LINE__);
    UnityDefaultTestRun(test_pack_unpack_roundtrip, "test_pack_unpack_roundtrip", __LINE__);
    UnityDefaultTestRun(test_blob_version_field, "test_blob_version_field", __LINE__);
    UnityDefaultTestRun(test_servo_defaults, "test_servo_defaults", __LINE__);
    UnityDefaultTestRun(test_servo_pulse_bounds, "test_servo_pulse_bounds", __LINE__);
    UnityDefaultTestRun(test_lock_policy_bounds, "test_lock_policy_bounds", __LINE__);
    UnityDefaultTestRun(test_pack_unpack_v3_roundtrip, "test_pack_unpack_v3_roundtrip", __LINE__);
    UnityDefaultTestRun(test_v1_blob_migrates, "test_v1_blob_migrates", __LINE__);
    UnityDefaultTestRun(test_v2_blob_migrates_to_two_detents, "test_v2_blob_migrates_to_two_detents", __LINE__);
    UnityDefaultTestRun(test_unpack_rejects_bad_len_and_version, "test_unpack_rejects_bad_len_and_version", __LINE__);
    UnityDefaultTestRun(test_pack_guards, "test_pack_guards", __LINE__);
)
