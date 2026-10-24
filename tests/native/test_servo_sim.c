/*
 * servo_sim 测试：速率限制推进、到达吸附、越界拒绝、使能开关、首步保护。
 * 全部经由 servo_act_ops_t 调用（上层真正使用的接口），不直接戳结构体内部。
 */
#include <string.h>

#include "unity.h"
#include "unity_runner.h"
#include "servo_act.h"

void setUp(void) {}
void tearDown(void) {}

static servo_sim_t sim;
static servo_act_chan_state_t st[SERVO_ACT_CHANNELS];

/* 两通道同一起始位置，多数用例只关心单通道行为 */
static void fresh(uint32_t start_us)
{
    const uint32_t start[SERVO_ACT_CHANNELS] = {start_us, start_us};
    servo_sim_init(&sim, 3000.0f, start);
    servo_sim_ops.step(&sim, 1000); /* 建立时间基准 */
}

static void read_state(void)
{
    servo_sim_ops.get_state(&sim, st);
}

/* 1. 初始化后停在该通道给定的起始位置，已视为到达 */
static void test_init_at_start(void)
{
    const uint32_t start[SERVO_ACT_CHANNELS] = {1500, 1200}; /* 前后起始位置可以不同 */
    servo_sim_init(&sim, 3000.0f, start);
    servo_sim_ops.step(&sim, 1000);
    read_state();
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1500.0f, st[0].cur_us);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1500.0f, st[0].target_us);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1200.0f, st[1].cur_us);
    TEST_ASSERT_TRUE(st[0].reached);
    TEST_ASSERT_TRUE(st[1].reached);
}

/* 2. 目标跳变后按速率线性推进：100ms 走 300µs（不是一步到位） */
static void test_moves_at_rate(void)
{
    fresh(1500);
    servo_sim_ops.set_us(&sim, 0, 2000);
    servo_sim_ops.step(&sim, 1100); /* dt = 100ms */
    read_state();
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 1800.0f, st[0].cur_us);
    TEST_ASSERT_FALSE(st[0].reached);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 2000.0f, st[0].target_us);
}

/* 3. 走够时间后吸附到目标并置 reached，不再越冲 */
static void test_reaches_and_snaps(void)
{
    fresh(1500);
    servo_sim_ops.set_us(&sim, 0, 2000);
    servo_sim_ops.step(&sim, 2000); /* 累计 1s，足够走完 500µs */
    read_state();
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 2000.0f, st[0].cur_us);
    TEST_ASSERT_TRUE(st[0].reached);

    servo_sim_ops.step(&sim, 3000); /* 继续推进不会越过目标 */
    read_state();
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 2000.0f, st[0].cur_us);
}

/* 4. 目标小于当前位置时同样按速率回走 */
static void test_moves_downward(void)
{
    fresh(1500);
    servo_sim_ops.set_us(&sim, 0, 1000);
    servo_sim_ops.step(&sim, 1100);
    read_state();
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 1200.0f, st[0].cur_us);
}

/* 5. 越界脉宽拒绝且不改变目标（0 与超上限都挡住） */
static void test_set_us_rejects_out_of_range(void)
{
    fresh(1500);
    servo_sim_ops.set_us(&sim, 0, 2000);
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, servo_sim_ops.set_us(&sim, 0, 400));
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, servo_sim_ops.set_us(&sim, 0, 2600));
    read_state();
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 2000.0f, st[0].target_us);
}

/* 6. 通道越界与空指针拒绝 */
static void test_arg_guards(void)
{
    fresh(1500);
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, servo_sim_ops.set_us(&sim, 9, 1500));
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, servo_sim_ops.set_us(NULL, 0, 1500));
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, servo_sim_ops.get_state(&sim, NULL));
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, servo_sim_ops.step(NULL, 1000));
}

/* 7. 禁用通道不移动；重新使能后继续朝目标走 */
static void test_disabled_channel_holds_position(void)
{
    fresh(1500);
    servo_sim_ops.set_us(&sim, 0, 2000);
    servo_sim_ops.set_enabled(&sim, 0, false);
    servo_sim_ops.step(&sim, 1500); /* 400ms 足够走完，但通道关着 */

    read_state();
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1500.0f, st[0].cur_us); /* 原地不动 */
    TEST_ASSERT_FALSE(st[0].enabled);

    servo_sim_ops.set_enabled(&sim, 0, true);
    servo_sim_ops.step(&sim, 1600);
    read_state();
    TEST_ASSERT_TRUE(st[0].enabled);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 1800.0f, st[0].cur_us);
}

/* 8. 首次 step 只建立时间基准：不会因为 last_ms=0 而一步跳到位 */
static void test_first_step_does_not_jump(void)
{
    servo_sim_t s;
    const uint32_t start[SERVO_ACT_CHANNELS] = {1500, 1500};
    servo_sim_init(&s, 3000.0f, start);
    servo_sim_ops.set_us(&s, 0, 2000);
    servo_sim_ops.step(&s, 999999); /* 若按 now-0 算 dt，会瞬间跑完 */
    servo_act_chan_state_t o[SERVO_ACT_CHANNELS];
    servo_sim_ops.get_state(&s, o);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1500.0f, o[0].cur_us);

    servo_sim_ops.step(&s, 1000099); /* 基准之后的 100ms 才真正开始走 */
    servo_sim_ops.get_state(&s, o);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 1800.0f, o[0].cur_us);
}

/* 9. 两通道互不干扰 */
static void test_channels_independent(void)
{
    fresh(1500);
    servo_sim_ops.set_us(&sim, 0, 2000);
    servo_sim_ops.set_us(&sim, 1, 1000);
    servo_sim_ops.step(&sim, 1100);
    read_state();
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 1800.0f, st[0].cur_us);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 1200.0f, st[1].cur_us);
}

NATIVE_TEST_MAIN(
    UnityDefaultTestRun(test_init_at_start, "test_init_at_start", __LINE__);
    UnityDefaultTestRun(test_moves_at_rate, "test_moves_at_rate", __LINE__);
    UnityDefaultTestRun(test_reaches_and_snaps, "test_reaches_and_snaps", __LINE__);
    UnityDefaultTestRun(test_moves_downward, "test_moves_downward", __LINE__);
    UnityDefaultTestRun(test_set_us_rejects_out_of_range, "test_set_us_rejects_out_of_range", __LINE__);
    UnityDefaultTestRun(test_arg_guards, "test_arg_guards", __LINE__);
    UnityDefaultTestRun(test_disabled_channel_holds_position, "test_disabled_channel_holds_position", __LINE__);
    UnityDefaultTestRun(test_first_step_does_not_jump, "test_first_step_does_not_jump", __LINE__);
    UnityDefaultTestRun(test_channels_independent, "test_channels_independent", __LINE__);
)
