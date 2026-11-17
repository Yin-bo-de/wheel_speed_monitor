/*
 * strategy 引擎测试：两档脉宽映射（每通道独立）、打滑状态机（进入确认 /
 * 锁定保持 / 松开试探 / 失败退避）、模式与使能、参数守卫。
 *
 * 状态机的核心主张——"锁定期间不看轮速差"——由 test_locked_ignores_ratio
 * 直接守住：锁上后把轮速差抹平，锁定必须继续，否则就是自咬闭环。
 * 进入 LOCKED 前的持续确认见 test_slip_confirm_*：单拍打滑是毛刺，不是地形。
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

/* 老路径测试专用：关掉双闸门（阈值放到量程顶、稳定期归零）。闸门全开且
 * stable=0 时，窗口满足的第一拍（PENDING_SYNC 的下一拍）即落锁——老测试的
 * "确认期满直接 LOCKED"预期整体顺延一拍 50ms，见下方各测试的计时修改。 */
static void gates_open(void)
{
    cfg.lock_max_speed_rpm = 2000.0f;
    cfg.lock_max_delta_rpm = 2000.0f;
    cfg.lock_sync_stable_ms = 0;
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

/* 1. 缺省配置：两档脉宽、门槛、各处时长 */
static void test_config_defaults(void)
{
    strategy_config_default(&cfg);
    TEST_ASSERT_EQUAL_UINT32(1500, cfg.unlock_us[0]);
    TEST_ASSERT_EQUAL_UINT32(2000, cfg.lock_us[0]);
    TEST_ASSERT_EQUAL_UINT32(1500, cfg.unlock_us[1]);
    TEST_ASSERT_EQUAL_UINT32(2000, cfg.lock_us[1]);
    TEST_ASSERT_TRUE(cfg.ch_enabled[0]);
    TEST_ASSERT_TRUE(cfg.ch_enabled[1]);
    TEST_ASSERT_EQUAL_INT(STRATEGY_MODE_AUTO, cfg.mode);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.30f, cfg.slip_engage_ratio);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 10.0f, cfg.slip_min_rpm);
    TEST_ASSERT_EQUAL_UINT32(3000, cfg.slip_confirm_ms);
    TEST_ASSERT_EQUAL_UINT32(3000, cfg.lock_hold_ms);
    TEST_ASSERT_EQUAL_UINT32(30000, cfg.lock_hold_max_ms);
    TEST_ASSERT_EQUAL_UINT32(2000, cfg.probe_window_ms);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 60.0f, cfg.lock_max_speed_rpm);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 40.0f, cfg.lock_max_delta_rpm);
    TEST_ASSERT_EQUAL_UINT32(300, cfg.lock_sync_stable_ms);
    TEST_ASSERT_TRUE(cfg.manual_gate_enable);
}

/* 2. 解锁档是每通道独立的一档：只有改过的那一轴变，另一轴不受影响 */
static void test_unlock_us_per_channel(void)
{
    fresh();
    cfg.unlock_us[1] = 1200;
    uint32_t t = 0;
    run(&t, 50, 60, 60, 60, 60); /* 两轴都不打滑，都停在解锁档 */
    strategy_get_state(&st);
    TEST_ASSERT_EQUAL_UINT16(1500, st.servo[0].target_us);
    TEST_ASSERT_EQUAL_UINT16(1200, st.servo[1].target_us);
}

/* 3. 锁定档也是每通道独立的一档——两轴舵机装反时靠这个各填一角 */
static void test_lock_us_per_channel(void)
{
    fresh();
    gates_open();
    cfg.lock_us[0] = 2000;
    cfg.lock_us[1] = 1000; /* 后轴装反：锁定去行程另一端 */
    uint32_t t = 0;
    run(&t, 3100, 200, 60, 200, 60); /* 前后轴同时打滑并走满确认期 */
    strategy_get_state(&st);
    TEST_ASSERT_EQUAL_INT(STRATEGY_PHASE_LOCKED, st.servo[0].phase);
    TEST_ASSERT_EQUAL_INT(STRATEGY_PHASE_LOCKED, st.servo[1].phase);
    TEST_ASSERT_EQUAL_UINT16(2000, st.servo[0].target_us);
    TEST_ASSERT_EQUAL_UINT16(1000, st.servo[1].target_us);
}

/* 4. 自动模式的输出只可能是配置的两档之一，不产生中间值：
 * 换一组非标准的脉宽，整个 IDLE→LOCKED→PROBE 走下来都只落在这两个数上 */
static void test_auto_output_is_two_detent_only(void)
{
    fresh();
    gates_open();
    cfg.unlock_us[0] = 1300;
    cfg.lock_us[0] = 1750;
    uint32_t t = 0;

    run(&t, 50, 100, 100, 60, 60); /* 速差 0 → 解锁档 */
    strategy_get_state(&st);
    TEST_ASSERT_EQUAL_UINT16(1300, st.servo[0].target_us);

    run(&t, 3100, 200, 60, 60, 60); /* 打滑满确认期 → 锁定档 */
    strategy_get_state(&st);
    TEST_ASSERT_EQUAL_UINT16(1750, st.servo[0].target_us);

    run(&t, 3050, 60, 60, 60, 60); /* 保持走完 → 试探 → 回解锁档 */
    strategy_get_state(&st);
    TEST_ASSERT_EQUAL_INT(STRATEGY_PHASE_PROBE, st.servo[0].phase);
    TEST_ASSERT_EQUAL_UINT16(1300, st.servo[0].target_us);
}

/* 5. 打滑超门槛 → 仅打滑那根轴锁定，另一根保持自己的解锁档 */
static void test_slip_locks_that_axle_only(void)
{
    fresh();
    gates_open();
    uint32_t t = 0;
    run(&t, 3100, 200, 60, 60, 60); /* 前轴比例 |200-60|/200 = 0.70，走满确认期 */

    strategy_get_state(&st);
    TEST_ASSERT_EQUAL_INT(STRATEGY_PHASE_LOCKED, st.servo[0].phase);
    TEST_ASSERT_EQUAL_UINT16(2000, st.servo[0].target_us);
    TEST_ASSERT_EQUAL_INT(STRATEGY_PHASE_IDLE, st.servo[1].phase);
    TEST_ASSERT_EQUAL_UINT16(1500, st.servo[1].target_us);
    TEST_ASSERT_FLOAT_WITHIN(0.005f, 0.70f, st.servo[0].ratio);
    TEST_ASSERT_TRUE(st.servo[0].slip_fast_left);
}

/* 6. 锁定期间把轮速差抹平，锁定必须继续（信号被自己的动作污染，不能据此解锁） */
static void test_locked_ignores_ratio(void)
{
    fresh();
    gates_open();
    uint32_t t = 0;
    run(&t, 3100, 200, 60, 60, 60);    /* 走满确认期锁上，保持 3000ms */
    run(&t, 2000, 60, 60, 60, 60);     /* 差速锁住 → 两轮同步 → 比例归零 */
    strategy_get_state(&st);
    TEST_ASSERT_EQUAL_INT(STRATEGY_PHASE_LOCKED, st.servo[0].phase);
    TEST_ASSERT_EQUAL_UINT16(2000, st.servo[0].target_us);
}

/* 7. 保持期走完 → 进入试探，目标回到解锁档 */
static void test_hold_expires_into_probe(void)
{
    fresh();
    gates_open();
    uint32_t t = 0;
    run(&t, 3100, 200, 60, 60, 60);
    run(&t, 3050, 60, 60, 60, 60); /* 累计越过 3000ms 保持期 */
    strategy_get_state(&st);
    TEST_ASSERT_EQUAL_INT(STRATEGY_PHASE_PROBE, st.servo[0].phase);
    TEST_ASSERT_EQUAL_UINT16(1500, st.servo[0].target_us);
}

/* 8. 试探期又打滑 → 立刻回锁，保持时长翻倍 */
static void test_probe_relock_doubles_hold(void)
{
    fresh();
    gates_open();
    uint32_t t = 0;
    run(&t, 3100, 200, 60, 60, 60);
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

/* 9. 反复试探失败：翻倍直到上限就封顶，不会溢出 */
static void test_hold_doubling_caps(void)
{
    fresh();
    gates_open();
    cfg.lock_hold_ms = 20000;
    cfg.lock_hold_max_ms = 30000;
    strategy_init();

    uint32_t t = 0;
    run(&t, 3100, 200, 60, 60, 60);     /* 走满确认期锁上，hold=20000 */
    run(&t, 20050, 60, 60, 60, 60);     /* → PROBE */
    run(&t, 50, 200, 60, 60, 60);       /* 回锁，翻倍被上限截住 */
    strategy_get_state(&st);
    TEST_ASSERT_EQUAL_INT(STRATEGY_PHASE_LOCKED, st.servo[0].phase);
    TEST_ASSERT_EQUAL_UINT32(30000, st.servo[0].hold_ms);
}

/* 10. 试探窗口按"状态保持不变的时长"计：动/停翻转要重新计时，
 * 状态稳住满窗口即脱困（动或停都算，见 test_backoff_resets_when_stopped） */
static void test_probe_window_restarts_on_state_change(void)
{
    fresh();
    gates_open();
    uint32_t t = 0;
    run(&t, 3100, 200, 60, 60, 60);   /* 走满确认期锁上 */
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

/* 11. 试探窗口走完无打滑 → 真脱困：回 IDLE 且保持时长重置为初始值 */
static void test_probe_success_resets_hold(void)
{
    fresh();
    gates_open();
    uint32_t t = 0;
    run(&t, 3100, 200, 60, 60, 60);
    run(&t, 3100, 60, 60, 60, 60);       /* → PROBE */
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

/* 12. 低于最低判定转速：比例再大也不判（静止时比值无意义） */
static void test_below_min_rpm_not_judged(void)
{
    fresh();
    uint32_t t = 0;
    run(&t, 500, 5, 1, 1, 1); /* 比例 0.80，但 base=5 < 10 转/分 */
    strategy_get_state(&st);
    TEST_ASSERT_EQUAL_INT(STRATEGY_PHASE_IDLE, st.servo[0].phase);
    TEST_ASSERT_EQUAL_UINT16(1500, st.servo[0].target_us);
}

/* 14. 通道关闭：输出该通道的解锁档且不参与判定，哪怕比例超门槛 */
static void test_disabled_channel_forced_unlock(void)
{
    fresh();
    cfg.ch_enabled[0] = false;
    cfg.unlock_us[0] = 1200; /* 关掉的通道也认自己的解锁档，不是写死的中位 */
    uint32_t t = 0;
    run(&t, 500, 200, 60, 60, 60);
    strategy_get_state(&st);
    TEST_ASSERT_EQUAL_INT(STRATEGY_PHASE_IDLE, st.servo[0].phase);
    TEST_ASSERT_EQUAL_UINT16(1200, st.servo[0].target_us);
}

/* 15. 手动模式：目标脉宽直通，不理两档，打滑也不锁 */
static void test_manual_passthrough(void)
{
    fresh();
    cfg.mode = STRATEGY_MODE_MANUAL;
    cfg.manual_us[0] = 1800;
    cfg.unlock_us[0] = 1200; /* 手动目标既不是解锁档也不是锁定档 */
    cfg.lock_us[0] = 1900;
    uint32_t t = 0;
    run(&t, 500, 200, 60, 60, 60);
    strategy_get_state(&st);
    TEST_ASSERT_EQUAL_INT(STRATEGY_MODE_MANUAL, st.mode);
    TEST_ASSERT_EQUAL_INT(STRATEGY_PHASE_IDLE, st.servo[0].phase);
    TEST_ASSERT_EQUAL_UINT16(1800, st.servo[0].target_us);
}

/* 16. 两档相同 = 该轴锁定时也不动：调试 PWM 链路时想看到的效果 */
static void test_lock_equals_unlock_keeps_still(void)
{
    fresh();
    gates_open();
    cfg.unlock_us[0] = 1600;
    cfg.lock_us[0] = 1600;
    uint32_t t = 0;
    run(&t, 3100, 200, 60, 60, 60);
    strategy_get_state(&st);
    TEST_ASSERT_EQUAL_INT(STRATEGY_PHASE_LOCKED, st.servo[0].phase); /* 判定照跑 */
    TEST_ASSERT_EQUAL_UINT16(1600, st.servo[0].target_us);           /* 只是不动 */
}

/* 17. strategy_init 复位状态机与保持计时（模式/使能变更后由上层调用） */
static void test_init_resets_state(void)
{
    fresh();
    gates_open();
    uint32_t t = 0;
    run(&t, 3100, 200, 60, 60, 60); /* 走满确认期才锁上 */
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

/* 17. 参数守卫：空指针拒绝，RC 读取恒未支持 */
static void test_arg_guards(void)
{
    fresh();
    float r[4] = {1.0f, 1.0f, 1.0f, 1.0f};
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, strategy_feed_wheel_rpm(NULL, &cfg, 0));
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, strategy_feed_wheel_rpm(r, NULL, 0));
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, strategy_get_state(NULL));
}

/* 18. 车轮继续转着清除打滑：退避等级要能复位，下次从初始值重来 */
static void test_backoff_resets_when_slip_clears(void)
{
    fresh();
    gates_open();
    uint32_t t = 0;
    run(&t, 3100, 200, 60, 60, 60); /* 走满确认期锁上 3000 */
    run(&t, 3050, 200, 60, 60, 60); /* 保持走完 → PROBE → 仍打滑，回锁 6000 */
    strategy_get_state(&st);
    TEST_ASSERT_EQUAL_UINT32(6000, st.servo[0].hold_ms);

    run(&t, 6050, 60, 60, 60, 60); /* 保持走完 → PROBE，这次不打滑 */
    run(&t, 2050, 60, 60, 60, 60); /* 连续动满试探窗口 → 脱困 */
    strategy_get_state(&st);
    TEST_ASSERT_EQUAL_INT(STRATEGY_PHASE_IDLE, st.servo[0].phase);

    run(&t, 3100, 200, 60, 60, 60); /* 再打滑：确认期与退避都从头开始 */
    strategy_get_state(&st);
    TEST_ASSERT_EQUAL_INT(STRATEGY_PHASE_LOCKED, st.servo[0].phase);
    TEST_ASSERT_EQUAL_UINT32(3000, st.servo[0].hold_ms);
}

/* 19. 车轮停转清除打滑（验证台上最自然的做法）：退避等级同样要能复位。
 * 现在试探窗口在"车停着"时暂停，于是停一次车就永久留在高锁定时长上，
 * 下次轻微打滑直接吃满长锁定——与直觉不符。 */
static void test_backoff_resets_when_stopped(void)
{
    fresh();
    gates_open();
    uint32_t t = 0;
    run(&t, 3100, 200, 60, 60, 60); /* 走满确认期锁上 */
    run(&t, 3050, 200, 60, 60, 60); /* 回锁 6000 */
    strategy_get_state(&st);
    TEST_ASSERT_EQUAL_UINT32(6000, st.servo[0].hold_ms);

    run(&t, 6050, 0, 0, 60, 60); /* 前轴停转 → 保持走完 → PROBE */
    run(&t, 3000, 0, 0, 60, 60); /* 停着不动 3 秒（超过试探窗口） */
    strategy_get_state(&st);
    TEST_ASSERT_EQUAL_INT(STRATEGY_PHASE_IDLE, st.servo[0].phase);

    run(&t, 3100, 200, 60, 60, 60); /* 再打滑：确认期与退避都从 3000 重新开始 */
    strategy_get_state(&st);
    TEST_ASSERT_EQUAL_UINT32(3000, st.servo[0].hold_ms);
}

/* 20. 打滑出现的第一个采样点不锁定：先进入持续确认，输出仍是解锁档 */
static void test_slip_first_tick_not_locked(void)
{
    fresh();
    uint32_t t = 0;
    run(&t, 50, 200, 60, 60, 60);
    strategy_get_state(&st);
    TEST_ASSERT_EQUAL_INT(STRATEGY_PHASE_IDLE, st.servo[0].phase);
    TEST_ASSERT_EQUAL_UINT16(1500, st.servo[0].target_us);
    TEST_ASSERT_EQUAL_UINT32(0, st.servo[0].hold_ms);
}

/* 21. 连续打滑满确认期（默认 3000ms）才离开 IDLE，先落 PENDING_SYNC 再落锁。
 * 首个打滑样本在 t=50，确认期满在 t=3050；闸门全开后窗口满足的第一拍
 * （t=3100）落锁——改造后 LOCKED 比改造前晚一拍。 */
static void test_slip_confirm_period_required(void)
{
    fresh();
    gates_open();
    uint32_t t = 0;
    run(&t, 3000, 200, 60, 60, 60);
    strategy_get_state(&st);
    TEST_ASSERT_EQUAL_INT(STRATEGY_PHASE_IDLE, st.servo[0].phase);
    TEST_ASSERT_EQUAL_UINT16(1500, st.servo[0].target_us);

    run(&t, 50, 200, 60, 60, 60); /* 确认期满 3000ms → 进 PENDING_SYNC，不直接锁 */
    strategy_get_state(&st);
    TEST_ASSERT_EQUAL_INT(STRATEGY_PHASE_PENDING_SYNC, st.servo[0].phase);
    TEST_ASSERT_EQUAL_UINT16(1500, st.servo[0].target_us);

    run(&t, 50, 200, 60, 60, 60); /* 窗口满足的第一拍 → 锁定 */
    strategy_get_state(&st);
    TEST_ASSERT_EQUAL_INT(STRATEGY_PHASE_LOCKED, st.servo[0].phase);
    TEST_ASSERT_EQUAL_UINT16(2000, st.servo[0].target_us);
    TEST_ASSERT_EQUAL_UINT32(3000, st.servo[0].hold_ms);
}

/* 22. 确认期间速差回落一拍 → 计时清零；再打滑必须重新等满确认期 */
static void test_confirm_resets_when_slip_clears(void)
{
    fresh();
    gates_open();
    uint32_t t = 0;
    run(&t, 2500, 200, 60, 60, 60); /* 确认到一半 */
    run(&t, 50, 60, 60, 60, 60);    /* 速差回落一拍 → 确认作废 */
    strategy_get_state(&st);
    TEST_ASSERT_EQUAL_INT(STRATEGY_PHASE_IDLE, st.servo[0].phase);

    run(&t, 2950, 200, 60, 60, 60); /* 重新打滑：不够 3 秒仍不锁 */
    strategy_get_state(&st);
    TEST_ASSERT_EQUAL_INT(STRATEGY_PHASE_IDLE, st.servo[0].phase);

    run(&t, 150, 200, 60, 60, 60); /* 满 3000ms 进 PENDING，再一拍落锁 */
    strategy_get_state(&st);
    TEST_ASSERT_EQUAL_INT(STRATEGY_PHASE_LOCKED, st.servo[0].phase);
}

/* 23. 确认期间轮子静止（moving 为假）→ 计时同样清零 */
static void test_confirm_resets_when_stopped(void)
{
    fresh();
    gates_open();
    uint32_t t = 0;
    run(&t, 2500, 200, 60, 60, 60);
    run(&t, 50, 0, 0, 60, 60); /* 前轴停转一拍 */
    strategy_get_state(&st);
    TEST_ASSERT_EQUAL_INT(STRATEGY_PHASE_IDLE, st.servo[0].phase);

    run(&t, 2950, 200, 60, 60, 60);
    strategy_get_state(&st);
    TEST_ASSERT_EQUAL_INT(STRATEGY_PHASE_IDLE, st.servo[0].phase);

    run(&t, 150, 200, 60, 60, 60); /* 满 3000ms 进 PENDING，再一拍落锁 */
    strategy_get_state(&st);
    TEST_ASSERT_EQUAL_INT(STRATEGY_PHASE_LOCKED, st.servo[0].phase);
}

/* 24. 确认期配成 0 = 保留旧行为：打滑第一拍立即锁定 */
static void test_confirm_zero_locks_immediately(void)
{
    fresh();
    gates_open();
    cfg.slip_confirm_ms = 0;
    uint32_t t = 0;
    run(&t, 100, 200, 60, 60, 60);
    strategy_get_state(&st);
    TEST_ASSERT_EQUAL_INT(STRATEGY_PHASE_LOCKED, st.servo[0].phase);
    TEST_ASSERT_EQUAL_UINT16(2000, st.servo[0].target_us);
    TEST_ASSERT_EQUAL_UINT32(3000, st.servo[0].hold_ms);
}

/* 25. 陷车大速差：确认期满落 PENDING_SYNC 而非 LOCKED，输出保持解锁档；
 * 不设等待超时——打滑持续时等再久也不落锁、也不放弃意图 */
static void test_stuck_goes_pending_not_locked(void)
{
    fresh();
    uint32_t t = 0;
    run(&t, 3050, 600, 0, 60, 60); /* 快轮空转、慢轮停：速差 600 远超闸门 */
    strategy_get_state(&st);
    TEST_ASSERT_EQUAL_INT(STRATEGY_PHASE_PENDING_SYNC, st.servo[0].phase);
    TEST_ASSERT_EQUAL_UINT16(1500, st.servo[0].target_us);
    TEST_ASSERT_EQUAL_UINT32(3000, st.servo[0].hold_ms); /* 锁定时长已备好，等窗口 */

    run(&t, 30000, 600, 0, 60, 60);
    strategy_get_state(&st);
    TEST_ASSERT_EQUAL_INT(STRATEGY_PHASE_PENDING_SYNC, st.servo[0].phase);
    TEST_ASSERT_EQUAL_UINT16(1500, st.servo[0].target_us);
}

/* 26. 窗口连续稳定满 lock_sync_stable_ms 才落锁：差一拍都不锁 */
static void test_pending_locks_on_stable_window(void)
{
    fresh();
    cfg.lock_max_speed_rpm = 200.0f; /* 只留速差闸门：慢轮 82 必须放行 */
    uint32_t t = 0;
    run(&t, 3050, 120, 82, 60, 60); /* 比例 0.32 打滑，速差 38 < 40 窗口在手 */
    strategy_get_state(&st);
    TEST_ASSERT_EQUAL_INT(STRATEGY_PHASE_PENDING_SYNC, st.servo[0].phase);

    run(&t, 300, 120, 82, 60, 60); /* 窗口从 t=3100 起算，t=3350 时累计 250ms */
    strategy_get_state(&st);
    TEST_ASSERT_EQUAL_INT(STRATEGY_PHASE_PENDING_SYNC, st.servo[0].phase);
    TEST_ASSERT_EQUAL_UINT16(1500, st.servo[0].target_us);

    run(&t, 50, 120, 82, 60, 60); /* t=3400 满 300ms → 落锁 */
    strategy_get_state(&st);
    TEST_ASSERT_EQUAL_INT(STRATEGY_PHASE_LOCKED, st.servo[0].phase);
    TEST_ASSERT_EQUAL_UINT16(2000, st.servo[0].target_us);
    TEST_ASSERT_EQUAL_UINT32(3000, st.servo[0].hold_ms);
}

/* 27. 窗口被速差尖峰打断 → 稳定计时清零重来（松油门过程速差扫过阈值区
 * 来回摆动的典型形态：必须连续稳定，瞬时扫过不算数） */
static void test_pending_window_resets_on_break(void)
{
    fresh();
    cfg.lock_max_speed_rpm = 200.0f;
    uint32_t t = 0;
    run(&t, 3050, 120, 82, 60, 60);  /* → PENDING，窗口从 t=3100 起算 */
    run(&t, 200, 120, 82, 60, 60);   /* t=3250：累计 150ms */
    run(&t, 50, 200, 60, 60, 60);    /* t=3300：速差尖峰一拍，计时清零 */
    strategy_get_state(&st);
    TEST_ASSERT_EQUAL_INT(STRATEGY_PHASE_PENDING_SYNC, st.servo[0].phase);

    run(&t, 300, 120, 82, 60, 60);   /* t=3350 起重新计数，t=3600 累计 250ms */
    strategy_get_state(&st);
    TEST_ASSERT_EQUAL_INT(STRATEGY_PHASE_PENDING_SYNC, st.servo[0].phase);

    run(&t, 50, 120, 82, 60, 60);    /* t=3650 满 300ms → 落锁 */
    strategy_get_state(&st);
    TEST_ASSERT_EQUAL_INT(STRATEGY_PHASE_LOCKED, st.servo[0].phase);
}

/* 28. 等待期间打滑消失 → 回 IDLE，退避与输出都复位 */
static void test_pending_returns_idle_when_slip_clears(void)
{
    fresh();
    uint32_t t = 0;
    run(&t, 3050, 600, 0, 60, 60); /* → PENDING */
    strategy_get_state(&st);
    TEST_ASSERT_EQUAL_INT(STRATEGY_PHASE_PENDING_SYNC, st.servo[0].phase);

    run(&t, 50, 60, 60, 60, 60); /* 速差消失（脱困或轮停）→ IDLE */
    strategy_get_state(&st);
    TEST_ASSERT_EQUAL_INT(STRATEGY_PHASE_IDLE, st.servo[0].phase);
    TEST_ASSERT_EQUAL_UINT32(0, st.servo[0].hold_ms);
    TEST_ASSERT_EQUAL_UINT16(1500, st.servo[0].target_us);
}

/* 29. 速差闸门严格小于：差恰 40 挡、39 放（stable=0 便于边界直测） */
static void test_delta_gate_strict_boundary(void)
{
    fresh();
    cfg.slip_confirm_ms = 0;       /* 立即进 PENDING */
    cfg.lock_sync_stable_ms = 0;   /* 窗口满足第一拍落锁 */
    cfg.lock_max_speed_rpm = 2000.0f;
    uint32_t t = 0;

    run(&t, 50, 120, 80, 60, 60); /* 速差恰 40：不满足 → PENDING */
    strategy_get_state(&st);
    TEST_ASSERT_EQUAL_INT(STRATEGY_PHASE_PENDING_SYNC, st.servo[0].phase);

    run(&t, 50, 120, 81, 60, 60); /* 速差 39：满足 → 下一拍锁定 */
    strategy_get_state(&st);
    TEST_ASSERT_EQUAL_INT(STRATEGY_PHASE_LOCKED, st.servo[0].phase);
}

/* 30. 车速闸门独立生效：速差小但慢轮超速也挡住（行驶中误锁场景——
 * 高转速下 EMA 滤波滞后让实测速差偏小，单靠速差闸门不可靠） */
static void test_speed_gate_blocks_fast_slip(void)
{
    fresh();
    cfg.lock_max_delta_rpm = 500.0f; /* 只留车速闸门 */
    uint32_t t = 0;
    run(&t, 3050, 300, 150, 60, 60); /* 比例 0.5 打滑；慢轮 150 > 60 超速 */
    strategy_get_state(&st);
    TEST_ASSERT_EQUAL_INT(STRATEGY_PHASE_PENDING_SYNC, st.servo[0].phase);
    TEST_ASSERT_EQUAL_UINT16(1500, st.servo[0].target_us);

    cfg.lock_max_speed_rpm = 200.0f; /* 抬闸门后窗口满足，稳定 300ms 落锁 */
    run(&t, 300, 300, 150, 60, 60); /* 窗口从 t=3100 起算，t=3350 累计 250ms */
    strategy_get_state(&st);
    TEST_ASSERT_EQUAL_INT(STRATEGY_PHASE_PENDING_SYNC, st.servo[0].phase);
    run(&t, 50, 300, 150, 60, 60); /* t=3400 满 300ms → 落锁 */
    strategy_get_state(&st);
    TEST_ASSERT_EQUAL_INT(STRATEGY_PHASE_LOCKED, st.servo[0].phase);
}

/* 31. 陷车时间线全程（对应方案「陷车场景时间线」1-4 步）：
 * 600/0 空转 → 确认 3 秒 → 等待（不落锁）→ 松油门速差收敛 → 稳定 300ms 落锁 → 脱困 */
static void test_stuck_timeline(void)
{
    fresh();
    cfg.lock_max_speed_rpm = 200.0f; /* 标定值：日常慢轮分布下限之上留余量 */
    uint32_t t = 0;

    /* 1) 陷车：快轮 600 空转、慢轮 0，打滑确认 3 秒 → PENDING_SYNC */
    run(&t, 3050, 600, 0, 60, 60);
    strategy_get_state(&st);
    TEST_ASSERT_EQUAL_INT(STRATEGY_PHASE_PENDING_SYNC, st.servo[0].phase);
    TEST_ASSERT_EQUAL_UINT16(1500, st.servo[0].target_us);

    /* 2) 速差 600 远超 40：不落锁、输出保持解锁，此刻锁止就是打牙，等 */
    run(&t, 30000, 600, 0, 60, 60);
    strategy_get_state(&st);
    TEST_ASSERT_EQUAL_INT(STRATEGY_PHASE_PENDING_SYNC, st.servo[0].phase);

    /* 3) 司机松油门：快轮减速，速差收敛进阈值、稳定 300ms → 落锁 */
    run(&t, 350, 120, 82, 60, 60);
    strategy_get_state(&st);
    TEST_ASSERT_EQUAL_INT(STRATEGY_PHASE_LOCKED, st.servo[0].phase);
    TEST_ASSERT_EQUAL_UINT16(2000, st.servo[0].target_us);

    /* 4) 再给油：两轮同步驱动脱困，锁定按时间保持 → 试探 */
    run(&t, 3050, 200, 200, 60, 60);
    strategy_get_state(&st);
    TEST_ASSERT_EQUAL_INT(STRATEGY_PHASE_PROBE, st.servo[0].phase);
}

/* 32. 试探期再打滑但窗口不在手 → PENDING_SYNC，退避翻倍保留；
 * 窗口到手后带着翻倍时长落锁 */
static void test_probe_relock_unsynced_waits(void)
{
    fresh();
    cfg.lock_max_speed_rpm = 200.0f;
    uint32_t t = 0;
    run(&t, 3050, 200, 60, 60, 60);  /* 大速差：确认期满 → PENDING */
    run(&t, 350, 120, 82, 60, 60);   /* 窗口满足稳定 → LOCKED（hold=3000） */
    strategy_get_state(&st);
    TEST_ASSERT_EQUAL_INT(STRATEGY_PHASE_LOCKED, st.servo[0].phase);
    run(&t, 3050, 60, 60, 60, 60);   /* 保持走完 → PROBE */
    run(&t, 50, 600, 0, 60, 60);     /* 再打滑、未同步 → PENDING，退避翻倍 */
    strategy_get_state(&st);
    TEST_ASSERT_EQUAL_INT(STRATEGY_PHASE_PENDING_SYNC, st.servo[0].phase);
    TEST_ASSERT_EQUAL_UINT32(6000, st.servo[0].hold_ms);
    TEST_ASSERT_EQUAL_UINT16(1500, st.servo[0].target_us);

    run(&t, 350, 120, 82, 60, 60);   /* 窗口收敛稳定 → 带着翻倍时长落锁 */
    strategy_get_state(&st);
    TEST_ASSERT_EQUAL_INT(STRATEGY_PHASE_LOCKED, st.servo[0].phase);
    TEST_ASSERT_EQUAL_UINT32(6000, st.servo[0].hold_ms);
}

/* 33. 试探期再打滑且窗口在手 → 直接锁回，退避翻倍保留（不绕 PENDING） */
static void test_probe_relock_synced_direct(void)
{
    fresh();
    cfg.lock_max_speed_rpm = 200.0f;
    uint32_t t = 0;
    run(&t, 3050, 200, 60, 60, 60);
    run(&t, 350, 120, 82, 60, 60);
    run(&t, 3050, 60, 60, 60, 60);   /* → PROBE */
    run(&t, 50, 120, 82, 60, 60);    /* 再打滑，窗口在手 */
    strategy_get_state(&st);
    TEST_ASSERT_EQUAL_INT(STRATEGY_PHASE_LOCKED, st.servo[0].phase);
    TEST_ASSERT_EQUAL_UINT32(6000, st.servo[0].hold_ms);
    TEST_ASSERT_EQUAL_UINT16(2000, st.servo[0].target_us);
}

/* 34. 手动锁止档命令过闸门：闸门不满足扣住解锁档并报 PENDING_SYNC，
 * 窗口稳定后持续放行（放行后只要窗口不破就保持锁止档） */
static void test_manual_gate_holds_lock_command(void)
{
    fresh();
    cfg.mode = STRATEGY_MODE_MANUAL;
    cfg.manual_us[0] = cfg.lock_us[0]; /* 2000 = 锁止档命令 */
    uint32_t t = 0;

    run(&t, 500, 600, 0, 60, 60); /* 大速差：闸门不满足 → 扣住 */
    strategy_get_state(&st);
    TEST_ASSERT_EQUAL_INT(STRATEGY_PHASE_PENDING_SYNC, st.servo[0].phase);
    TEST_ASSERT_EQUAL_UINT16(1500, st.servo[0].target_us); /* 输出保持解锁档 */

    cfg.lock_max_speed_rpm = 200.0f;
    run(&t, 300, 120, 82, 60, 60); /* 窗口从 t=550 起算，t=800 累计 250ms */
    strategy_get_state(&st);
    TEST_ASSERT_EQUAL_INT(STRATEGY_PHASE_PENDING_SYNC, st.servo[0].phase);
    TEST_ASSERT_EQUAL_UINT16(1500, st.servo[0].target_us);

    run(&t, 50, 120, 82, 60, 60); /* t=850 满 300ms → 放行锁止档 */
    strategy_get_state(&st);
    TEST_ASSERT_EQUAL_INT(STRATEGY_PHASE_IDLE, st.servo[0].phase);
    TEST_ASSERT_EQUAL_UINT16(2000, st.servo[0].target_us);
}

/* 35. 手动闸门窗口与自动同规则：尖峰打断稳定计时（防松油门扫过阈值区的瞬时触发） */
static void test_manual_gate_window_resets(void)
{
    fresh();
    cfg.mode = STRATEGY_MODE_MANUAL;
    cfg.manual_us[0] = cfg.lock_us[0];
    cfg.lock_max_speed_rpm = 200.0f;
    uint32_t t = 0;
    run(&t, 250, 120, 82, 60, 60);  /* t=250：稳定 200ms */
    run(&t, 50, 200, 60, 60, 60);   /* t=300：尖峰一拍，计时清零 */
    run(&t, 300, 120, 82, 60, 60);  /* t=600：重新累计 250ms，仍差一拍 */
    strategy_get_state(&st);
    TEST_ASSERT_EQUAL_INT(STRATEGY_PHASE_PENDING_SYNC, st.servo[0].phase);
    TEST_ASSERT_EQUAL_UINT16(1500, st.servo[0].target_us);
    run(&t, 50, 120, 82, 60, 60);   /* t=650：满 300ms → 放行 */
    strategy_get_state(&st);
    TEST_ASSERT_EQUAL_UINT16(2000, st.servo[0].target_us);
}

/* 36. 解锁档命令与关闸门都不拦：立即生效（旧行为保留路径） */
static void test_manual_gate_passthrough_cases(void)
{
    fresh();
    cfg.mode = STRATEGY_MODE_MANUAL;
    uint32_t t = 0;

    cfg.manual_us[0] = 1500; /* = 解锁档：不是锁止命令，不闸门 */
    run(&t, 500, 600, 0, 60, 60);
    strategy_get_state(&st);
    TEST_ASSERT_EQUAL_INT(STRATEGY_PHASE_IDLE, st.servo[0].phase);
    TEST_ASSERT_EQUAL_UINT16(1500, st.servo[0].target_us);

    cfg.manual_gate_enable = false; /* 关闸门：锁止命令也直通 */
    cfg.manual_us[0] = 2000;
    run(&t, 500, 600, 0, 60, 60);
    strategy_get_state(&st);
    TEST_ASSERT_EQUAL_INT(STRATEGY_PHASE_IDLE, st.servo[0].phase);
    TEST_ASSERT_EQUAL_UINT16(2000, st.servo[0].target_us);
}

/* 37. 锁止档判定容差 ±20µs：与前端 DETENT_TOL_US 同一约定 */
static void test_manual_gate_detent_tolerance(void)
{
    fresh();
    cfg.mode = STRATEGY_MODE_MANUAL;
    uint32_t t = 0;

    cfg.manual_us[0] = 2020; /* 距锁定档 20µs：算锁止命令 → 闸门扣住 */
    run(&t, 500, 600, 0, 60, 60);
    strategy_get_state(&st);
    TEST_ASSERT_EQUAL_UINT16(1500, st.servo[0].target_us);

    cfg.manual_us[0] = 2021; /* 距锁定档 21µs：不算锁止命令 → 直通 */
    run(&t, 500, 600, 0, 60, 60);
    strategy_get_state(&st);
    TEST_ASSERT_EQUAL_UINT16(2021, st.servo[0].target_us);
}

NATIVE_TEST_MAIN(
    UnityDefaultTestRun(test_config_defaults, "test_config_defaults", __LINE__);
    UnityDefaultTestRun(test_unlock_us_per_channel, "test_unlock_us_per_channel", __LINE__);
    UnityDefaultTestRun(test_lock_us_per_channel, "test_lock_us_per_channel", __LINE__);
    UnityDefaultTestRun(test_auto_output_is_two_detent_only, "test_auto_output_is_two_detent_only", __LINE__);
    UnityDefaultTestRun(test_slip_locks_that_axle_only, "test_slip_locks_that_axle_only", __LINE__);
    UnityDefaultTestRun(test_locked_ignores_ratio, "test_locked_ignores_ratio", __LINE__);
    UnityDefaultTestRun(test_hold_expires_into_probe, "test_hold_expires_into_probe", __LINE__);
    UnityDefaultTestRun(test_probe_relock_doubles_hold, "test_probe_relock_doubles_hold", __LINE__);
    UnityDefaultTestRun(test_hold_doubling_caps, "test_hold_doubling_caps", __LINE__);
    UnityDefaultTestRun(test_probe_window_restarts_on_state_change, "test_probe_window_pauses_when_stopped", __LINE__);
    UnityDefaultTestRun(test_probe_success_resets_hold, "test_probe_success_resets_hold", __LINE__);
    UnityDefaultTestRun(test_below_min_rpm_not_judged, "test_below_min_rpm_not_judged", __LINE__);
    UnityDefaultTestRun(test_disabled_channel_forced_unlock, "test_disabled_channel_forced_unlock", __LINE__);
    UnityDefaultTestRun(test_manual_passthrough, "test_manual_passthrough", __LINE__);
    UnityDefaultTestRun(test_lock_equals_unlock_keeps_still, "test_lock_equals_unlock_keeps_still", __LINE__);
    UnityDefaultTestRun(test_init_resets_state, "test_init_resets_state", __LINE__);
    UnityDefaultTestRun(test_arg_guards, "test_arg_guards", __LINE__);
    UnityDefaultTestRun(test_backoff_resets_when_slip_clears, "test_backoff_resets_when_slip_clears", __LINE__);
    UnityDefaultTestRun(test_backoff_resets_when_stopped, "test_backoff_resets_when_stopped", __LINE__);
    UnityDefaultTestRun(test_slip_first_tick_not_locked, "test_slip_first_tick_not_locked", __LINE__);
    UnityDefaultTestRun(test_slip_confirm_period_required, "test_slip_confirm_period_required", __LINE__);
    UnityDefaultTestRun(test_confirm_resets_when_slip_clears, "test_confirm_resets_when_slip_clears", __LINE__);
    UnityDefaultTestRun(test_confirm_resets_when_stopped, "test_confirm_resets_when_stopped", __LINE__);
    UnityDefaultTestRun(test_confirm_zero_locks_immediately, "test_confirm_zero_locks_immediately", __LINE__);
    UnityDefaultTestRun(test_stuck_goes_pending_not_locked, "test_stuck_goes_pending_not_locked", __LINE__);
    UnityDefaultTestRun(test_pending_locks_on_stable_window, "test_pending_locks_on_stable_window", __LINE__);
    UnityDefaultTestRun(test_pending_window_resets_on_break, "test_pending_window_resets_on_break", __LINE__);
    UnityDefaultTestRun(test_pending_returns_idle_when_slip_clears, "test_pending_returns_idle_when_slip_clears", __LINE__);
    UnityDefaultTestRun(test_delta_gate_strict_boundary, "test_delta_gate_strict_boundary", __LINE__);
    UnityDefaultTestRun(test_speed_gate_blocks_fast_slip, "test_speed_gate_blocks_fast_slip", __LINE__);
    UnityDefaultTestRun(test_stuck_timeline, "test_stuck_timeline", __LINE__);
    UnityDefaultTestRun(test_probe_relock_unsynced_waits, "test_probe_relock_unsynced_waits", __LINE__);
    UnityDefaultTestRun(test_probe_relock_synced_direct, "test_probe_relock_synced_direct", __LINE__);
    UnityDefaultTestRun(test_manual_gate_holds_lock_command, "test_manual_gate_holds_lock_command", __LINE__);
    UnityDefaultTestRun(test_manual_gate_window_resets, "test_manual_gate_window_resets", __LINE__);
    UnityDefaultTestRun(test_manual_gate_passthrough_cases, "test_manual_gate_passthrough_cases", __LINE__);
    UnityDefaultTestRun(test_manual_gate_detent_tolerance, "test_manual_gate_detent_tolerance", __LINE__);
)
