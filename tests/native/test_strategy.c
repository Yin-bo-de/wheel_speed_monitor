/*
 * strategy 引擎测试：角度映射、打滑状态机（锁定保持 / 松开试探 / 失败退避）、
 * 模式与使能、参数守卫。
 *
 * 状态机的核心主张——"锁定期间不看轮速差"——由 test_locked_ignores_ratio
 * 直接守住：锁上后把轮速差抹平，锁定必须继续，否则就是自咬闭环。
 */
#include <string.h>

#include "unity.h"
#include "unity_runner.h"
#include "strategy.h"

void setUp(void) {}
void tearDown(void) {}

static strategy_config_t cfg;
static strategy_state_t st;

static void fresh(void)
{
    strategy_config_default(&cfg);
    strategy_init();
}

/* 单次喂入四轮转速（左前,右前,左后,右后）。 */
static void feed(float lf, float rf, float lr, float rr, uint32_t now_ms)
{
    float r[4] = {lf, rf, lr, rr};
    strategy_feed_wheel_rpm(r, &cfg, now_ms);
}

/* 以 50ms 步长推进 total_ms，期间四轮转速恒定（模拟 sampler 周期）。 */
static void run(uint32_t *t, uint32_t total_ms, float lf, float rf, float lr, float rr)
{
    for (uint32_t e = 0; e < total_ms; e += 50) {
        *t += 50;
        feed(lf, rf, lr, rr, *t);
    }
}

/* 1. 缺省配置：脉宽三元组、门槛、各处时长 */
static void test_config_defaults(void)
{
    strategy_config_default(&cfg);
    TEST_ASSERT_EQUAL_UINT32(1000, cfg.min_us);
    TEST_ASSERT_EQUAL_UINT32(1500, cfg.center_us);
    TEST_ASSERT_EQUAL_UINT32(2000, cfg.max_us);
    TEST_ASSERT_FALSE(cfg.invert[0]);
    TEST_ASSERT_FALSE(cfg.invert[1]);
    TEST_ASSERT_TRUE(cfg.ch_enabled[0]);
    TEST_ASSERT_TRUE(cfg.ch_enabled[1]);
    TEST_ASSERT_EQUAL_INT(STRATEGY_MODE_AUTO, cfg.mode);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.30f, cfg.slip_engage_ratio);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 10.0f, cfg.slip_min_rpm);
    TEST_ASSERT_EQUAL_UINT32(3000, cfg.lock_hold_ms);
    TEST_ASSERT_EQUAL_UINT32(30000, cfg.lock_hold_max_ms);
    TEST_ASSERT_EQUAL_UINT32(2000, cfg.probe_window_ms);
}

/* 2. 角度→脉宽：中位、两端、中间值；非对称量程也正确 */
static void test_deg_to_us_mapping(void)
{
    fresh();
    TEST_ASSERT_EQUAL_UINT32(1500, strategy_deg_to_us(&cfg, 0, 0.0f));
    TEST_ASSERT_EQUAL_UINT32(2000, strategy_deg_to_us(&cfg, 0, 90.0f));
    TEST_ASSERT_EQUAL_UINT32(1000, strategy_deg_to_us(&cfg, 0, -90.0f));
    TEST_ASSERT_EQUAL_UINT32(1750, strategy_deg_to_us(&cfg, 0, 45.0f));
    TEST_ASSERT_EQUAL_UINT32(1250, strategy_deg_to_us(&cfg, 0, -45.0f));
}

/* 3. 反向映射：角度取反后再落脉宽 */
static void test_deg_to_us_invert(void)
{
    fresh();
    cfg.invert[0] = true;
    TEST_ASSERT_EQUAL_UINT32(1000, strategy_deg_to_us(&cfg, 0, 90.0f));
    TEST_ASSERT_EQUAL_UINT32(2000, strategy_deg_to_us(&cfg, 0, -90.0f));
    TEST_ASSERT_EQUAL_UINT32(1500, strategy_deg_to_us(&cfg, 0, 0.0f));
    /* 未开反向的通道不受影响 */
    TEST_ASSERT_EQUAL_UINT32(2000, strategy_deg_to_us(&cfg, 1, 90.0f));
}

/* 4. 超出 ±90 钳位；通道越界退回中位（不越界写数组） */
static void test_deg_to_us_clamps(void)
{
    fresh();
    TEST_ASSERT_EQUAL_UINT32(2000, strategy_deg_to_us(&cfg, 0, 200.0f));
    TEST_ASSERT_EQUAL_UINT32(1000, strategy_deg_to_us(&cfg, 0, -200.0f));
    TEST_ASSERT_EQUAL_UINT32(1500, strategy_deg_to_us(&cfg, 9, 90.0f));
}

/* 5. 脉宽→角度与正向映射互逆（渲染层靠它显示"到达角度"） */
static void test_us_to_deg_inverse(void)
{
    fresh();
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, strategy_us_to_deg(&cfg, 0, 1500));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 90.0f, strategy_us_to_deg(&cfg, 0, 2000));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -90.0f, strategy_us_to_deg(&cfg, 0, 1000));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -45.0f, strategy_us_to_deg(&cfg, 0, 1250));

    cfg.invert[0] = true;
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -90.0f, strategy_us_to_deg(&cfg, 0, 2000));
}

/* 6. 打滑超门槛 → 仅打滑那根轴锁定并满偏，另一根保持中位 */
static void test_slip_locks_that_axle_only(void)
{
    fresh();
    uint32_t t = 0;
    run(&t, 50, 200, 60, 60, 60); /* 前轴比例 |200-60|/200 = 0.70 > 0.30 */

    strategy_get_state(&st);
    TEST_ASSERT_EQUAL_INT(STRATEGY_PHASE_LOCKED, st.servo[0].phase);
    TEST_ASSERT_EQUAL_UINT16(2000, st.servo[0].target_us);
    TEST_ASSERT_EQUAL_INT(STRATEGY_PHASE_IDLE, st.servo[1].phase);
    TEST_ASSERT_EQUAL_UINT16(1500, st.servo[1].target_us);
    TEST_ASSERT_FLOAT_WITHIN(0.005f, 0.70f, st.servo[0].ratio);
    TEST_ASSERT_TRUE(st.servo[0].slip_fast_left);
}

/* 7. 锁定期间把轮速差抹平，锁定必须继续（信号被自己的动作污染，不能据此解锁） */
static void test_locked_ignores_ratio(void)
{
    fresh();
    uint32_t t = 0;
    run(&t, 50, 200, 60, 60, 60);      /* 锁上，保持 3000ms */
    run(&t, 2000, 60, 60, 60, 60);     /* 差速锁住 → 两轮同步 → 比例归零 */
    strategy_get_state(&st);
    TEST_ASSERT_EQUAL_INT(STRATEGY_PHASE_LOCKED, st.servo[0].phase);
    TEST_ASSERT_EQUAL_UINT16(2000, st.servo[0].target_us);
}

/* 8. 保持期走完 → 进入试探，目标回中位 */
static void test_hold_expires_into_probe(void)
{
    fresh();
    uint32_t t = 0;
    run(&t, 50, 200, 60, 60, 60);
    run(&t, 3050, 60, 60, 60, 60); /* 累计越过 3000ms 保持期 */
    strategy_get_state(&st);
    TEST_ASSERT_EQUAL_INT(STRATEGY_PHASE_PROBE, st.servo[0].phase);
    TEST_ASSERT_EQUAL_UINT16(1500, st.servo[0].target_us);
}

/* 9. 试探期又打滑 → 立刻回锁，保持时长翻倍 */
static void test_probe_relock_doubles_hold(void)
{
    fresh();
    uint32_t t = 0;
    run(&t, 50, 200, 60, 60, 60);
    run(&t, 3050, 60, 60, 60, 60);
    strategy_get_state(&st);
    TEST_ASSERT_EQUAL_INT(STRATEGY_PHASE_PROBE, st.servo[0].phase);
    TEST_ASSERT_EQUAL_UINT32(3000, st.servo[0].hold_ms);

    run(&t, 50, 200, 60, 60, 60); /* 地形没走完 */
    strategy_get_state(&st);
    TEST_ASSERT_EQUAL_INT(STRATEGY_PHASE_LOCKED, st.servo[0].phase);
    TEST_ASSERT_EQUAL_UINT32(6000, st.servo[0].hold_ms);
    TEST_ASSERT_EQUAL_UINT16(2000, st.servo[0].target_us);
}

/* 10. 反复试探失败：翻倍直到上限就封顶，不会溢出 */
static void test_hold_doubling_caps(void)
{
    fresh();
    cfg.lock_hold_ms = 20000;
    cfg.lock_hold_max_ms = 30000;
    strategy_init();

    uint32_t t = 0;
    run(&t, 50, 200, 60, 60, 60);       /* 锁上，hold=20000 */
    run(&t, 20050, 60, 60, 60, 60);     /* → PROBE */
    run(&t, 50, 200, 60, 60, 60);       /* 回锁，翻倍被上限截住 */
    strategy_get_state(&st);
    TEST_ASSERT_EQUAL_INT(STRATEGY_PHASE_LOCKED, st.servo[0].phase);
    TEST_ASSERT_EQUAL_UINT32(30000, st.servo[0].hold_ms);
}

/* 11. 试探窗口按"状态保持不变的时长"计：动/停翻转要重新计时，
 * 状态稳住满窗口即脱困（动或停都算，见 test_backoff_resets_when_stopped） */
static void test_probe_window_restarts_on_state_change(void)
{
    fresh();
    uint32_t t = 0;
    run(&t, 50, 200, 60, 60, 60);     /* 锁上 */
    run(&t, 3050, 60, 60, 60, 60);    /* → PROBE，在动，开始计时 */
    run(&t, 1000, 0, 0, 0, 0);        /* 刚停下 1 秒：翻转 → 重新计时 */
    strategy_get_state(&st);
    TEST_ASSERT_EQUAL_INT(STRATEGY_PHASE_PROBE, st.servo[0].phase);

    run(&t, 1500, 60, 60, 60, 60);    /* 又动起来：再翻转 → 又从头计时 */
    strategy_get_state(&st);
    TEST_ASSERT_EQUAL_INT(STRATEGY_PHASE_PROBE, st.servo[0].phase);

    run(&t, 2050, 60, 60, 60, 60);    /* 连续动满窗口 → 脱困 */
    strategy_get_state(&st);
    TEST_ASSERT_EQUAL_INT(STRATEGY_PHASE_IDLE, st.servo[0].phase);
}

/* 12. 试探窗口走完无打滑 → 真脱困：回 IDLE 且保持时长重置为初始值 */
static void test_probe_success_resets_hold(void)
{
    fresh();
    uint32_t t = 0;
    run(&t, 50, 200, 60, 60, 60);
    run(&t, 3050, 60, 60, 60, 60);       /* → PROBE */
    run(&t, 50, 200, 60, 60, 60);        /* 回锁，hold 翻到 6000 */
    strategy_get_state(&st);
    TEST_ASSERT_EQUAL_UINT32(6000, st.servo[0].hold_ms);

    run(&t, 6050, 60, 60, 60, 60);       /* 这次保持期走完 */
    run(&t, 2050, 60, 60, 60, 60);       /* 试探期一直不打滑 */
    strategy_get_state(&st);
    TEST_ASSERT_EQUAL_INT(STRATEGY_PHASE_IDLE, st.servo[0].phase);
    TEST_ASSERT_EQUAL_UINT32(0, st.servo[0].hold_ms); /* 已重置，下次从头算 */
    TEST_ASSERT_EQUAL_UINT16(1500, st.servo[0].target_us);
}

/* 13. 低于最低判定转速：比例再大也不判（静止时比值无意义） */
static void test_below_min_rpm_not_judged(void)
{
    fresh();
    uint32_t t = 0;
    run(&t, 500, 5, 1, 1, 1); /* 比例 0.80，但 base=5 < 10 转/分 */
    strategy_get_state(&st);
    TEST_ASSERT_EQUAL_INT(STRATEGY_PHASE_IDLE, st.servo[0].phase);
    TEST_ASSERT_EQUAL_UINT16(1500, st.servo[0].target_us);
}

/* 14. 通道关闭：输出中位且不参与判定，哪怕比例超门槛 */
static void test_disabled_channel_forced_center(void)
{
    fresh();
    cfg.ch_enabled[0] = false;
    uint32_t t = 0;
    run(&t, 500, 200, 60, 60, 60);
    strategy_get_state(&st);
    TEST_ASSERT_EQUAL_INT(STRATEGY_PHASE_IDLE, st.servo[0].phase);
    TEST_ASSERT_EQUAL_UINT16(1500, st.servo[0].target_us);
}

/* 15. 手动模式：目标脉宽直通，打滑也不锁，且不应用反向 */
static void test_manual_passthrough(void)
{
    fresh();
    cfg.mode = STRATEGY_MODE_MANUAL;
    cfg.manual_us[0] = 1800;
    cfg.invert[0] = true; /* 手动模式直设原始脉宽，反向不参与 */
    uint32_t t = 0;
    run(&t, 500, 200, 60, 60, 60);
    strategy_get_state(&st);
    TEST_ASSERT_EQUAL_INT(STRATEGY_MODE_MANUAL, st.mode);
    TEST_ASSERT_EQUAL_INT(STRATEGY_PHASE_IDLE, st.servo[0].phase);
    TEST_ASSERT_EQUAL_UINT16(1800, st.servo[0].target_us);
}

/* 16. 反向映射：锁定方向翻到另一端 */
static void test_lock_direction_follows_invert(void)
{
    fresh();
    cfg.invert[0] = true;
    uint32_t t = 0;
    run(&t, 50, 200, 60, 60, 60);
    strategy_get_state(&st);
    TEST_ASSERT_EQUAL_INT(STRATEGY_PHASE_LOCKED, st.servo[0].phase);
    TEST_ASSERT_EQUAL_UINT16(1000, st.servo[0].target_us);
}

/* 17. strategy_init 复位状态机与保持计时（模式/使能变更后由上层调用） */
static void test_init_resets_state(void)
{
    fresh();
    uint32_t t = 0;
    run(&t, 50, 200, 60, 60, 60);
    strategy_get_state(&st);
    TEST_ASSERT_EQUAL_INT(STRATEGY_PHASE_LOCKED, st.servo[0].phase);

    strategy_init();
    strategy_get_state(&st);
    TEST_ASSERT_EQUAL_INT(STRATEGY_PHASE_IDLE, st.servo[0].phase);
    TEST_ASSERT_EQUAL_UINT32(0, st.servo[0].hold_ms);
    /* 复位后尚未喂入转速，目标未计算（0 = 未定）。上层在首次 feed 之前
     * 自行保持中位，不依赖这里读出有效脉宽。 */
    TEST_ASSERT_EQUAL_UINT16(0, st.servo[0].target_us);
}

/* 18. 参数守卫：空指针拒绝，RC 读取恒未支持 */
static void test_arg_guards(void)
{
    fresh();
    float r[4] = {1.0f, 1.0f, 1.0f, 1.0f};
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, strategy_feed_wheel_rpm(NULL, &cfg, 0));
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, strategy_feed_wheel_rpm(r, NULL, 0));
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, strategy_get_state(NULL));

    uint16_t us = 0;
    TEST_ASSERT_EQUAL_INT(ESP_ERR_NOT_SUPPORTED, strategy_rc_read(0, &us));
}

/* 19. 车轮继续转着清除打滑：退避等级要能复位，下次从初始值重来 */
static void test_backoff_resets_when_slip_clears(void)
{
    fresh();
    uint32_t t = 0;
    run(&t, 50, 200, 60, 60, 60);   /* 锁上 3000 */
    run(&t, 3050, 200, 60, 60, 60); /* 保持走完 → PROBE → 仍打滑，回锁 6000 */
    strategy_get_state(&st);
    TEST_ASSERT_EQUAL_UINT32(6000, st.servo[0].hold_ms);

    run(&t, 6050, 60, 60, 60, 60); /* 保持走完 → PROBE，这次不打滑 */
    run(&t, 2050, 60, 60, 60, 60); /* 连续动满试探窗口 → 脱困 */
    strategy_get_state(&st);
    TEST_ASSERT_EQUAL_INT(STRATEGY_PHASE_IDLE, st.servo[0].phase);

    run(&t, 50, 200, 60, 60, 60); /* 再打滑：必须从 3000 重新开始 */
    strategy_get_state(&st);
    TEST_ASSERT_EQUAL_INT(STRATEGY_PHASE_LOCKED, st.servo[0].phase);
    TEST_ASSERT_EQUAL_UINT32(3000, st.servo[0].hold_ms);
}

/* 20. 车轮停转清除打滑（验证台上最自然的做法）：退避等级同样要能复位。
 * 现在试探窗口在"车停着"时暂停，于是停一次车就永久留在高锁定时长上，
 * 下次轻微打滑直接吃满长锁定——与直觉不符。 */
static void test_backoff_resets_when_stopped(void)
{
    fresh();
    uint32_t t = 0;
    run(&t, 50, 200, 60, 60, 60);
    run(&t, 3050, 200, 60, 60, 60); /* 回锁 6000 */
    strategy_get_state(&st);
    TEST_ASSERT_EQUAL_UINT32(6000, st.servo[0].hold_ms);

    run(&t, 6050, 0, 0, 60, 60); /* 前轴停转 → 保持走完 → PROBE */
    run(&t, 3000, 0, 0, 60, 60); /* 停着不动 3 秒（超过试探窗口） */
    strategy_get_state(&st);
    TEST_ASSERT_EQUAL_INT(STRATEGY_PHASE_IDLE, st.servo[0].phase);

    run(&t, 50, 200, 60, 60, 60); /* 再打滑：从 3000 重新开始 */
    strategy_get_state(&st);
    TEST_ASSERT_EQUAL_UINT32(3000, st.servo[0].hold_ms);
}

NATIVE_TEST_MAIN(
    UnityDefaultTestRun(test_config_defaults, "test_config_defaults", __LINE__);
    UnityDefaultTestRun(test_deg_to_us_mapping, "test_deg_to_us_mapping", __LINE__);
    UnityDefaultTestRun(test_deg_to_us_invert, "test_deg_to_us_invert", __LINE__);
    UnityDefaultTestRun(test_deg_to_us_clamps, "test_deg_to_us_clamps", __LINE__);
    UnityDefaultTestRun(test_us_to_deg_inverse, "test_us_to_deg_inverse", __LINE__);
    UnityDefaultTestRun(test_slip_locks_that_axle_only, "test_slip_locks_that_axle_only", __LINE__);
    UnityDefaultTestRun(test_locked_ignores_ratio, "test_locked_ignores_ratio", __LINE__);
    UnityDefaultTestRun(test_hold_expires_into_probe, "test_hold_expires_into_probe", __LINE__);
    UnityDefaultTestRun(test_probe_relock_doubles_hold, "test_probe_relock_doubles_hold", __LINE__);
    UnityDefaultTestRun(test_hold_doubling_caps, "test_hold_doubling_caps", __LINE__);
    UnityDefaultTestRun(test_probe_window_restarts_on_state_change, "test_probe_window_pauses_when_stopped", __LINE__);
    UnityDefaultTestRun(test_probe_success_resets_hold, "test_probe_success_resets_hold", __LINE__);
    UnityDefaultTestRun(test_below_min_rpm_not_judged, "test_below_min_rpm_not_judged", __LINE__);
    UnityDefaultTestRun(test_disabled_channel_forced_center, "test_disabled_channel_forced_center", __LINE__);
    UnityDefaultTestRun(test_manual_passthrough, "test_manual_passthrough", __LINE__);
    UnityDefaultTestRun(test_lock_direction_follows_invert, "test_lock_direction_follows_invert", __LINE__);
    UnityDefaultTestRun(test_init_resets_state, "test_init_resets_state", __LINE__);
    UnityDefaultTestRun(test_arg_guards, "test_arg_guards", __LINE__);
    UnityDefaultTestRun(test_backoff_resets_when_slip_clears, "test_backoff_resets_when_slip_clears", __LINE__);
    UnityDefaultTestRun(test_backoff_resets_when_stopped, "test_backoff_resets_when_stopped", __LINE__);
)
