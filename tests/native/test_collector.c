/* wheel_speed_collector 行为测试：计数差值、16 位回绕/复位判定、累计、换源重置、去抖、
 * 脉冲间隔测频、静止归零兜底。 */
#include <math.h>
#include <string.h>
#include "unity.h"
#include "unity_runner.h"
#include "wheel_speed_collector.h"

void setUp(void) {}
void tearDown(void) {}

static wheel_chan_state_t st;
static wheel_chan_cfg_t cfg;

static void fresh(void)
{
    memset(&st, 0, sizeof(st));
    memset(&cfg, 0, sizeof(cfg));
    cfg.window_ms = 50;
    cfg.magnets = 1;
    cfg.wheel_diam_mm = 100;
    cfg.alpha = 0.3f;
    cfg.debounce_ms = 0;
}

/* 1. 正常递增：每窗口增量正确反映到累计脉冲 */
static void test_normal_increment(void)
{
    fresh();
    /* 首窗口只建立基准 */
    wheel_chan_sample(&st, &cfg, 100, 1000);
    TEST_ASSERT_EQUAL_UINT32(0, st.pulses_total);
    /* 100→103→106，各 3 脉冲 */
    wheel_chan_sample(&st, &cfg, 103, 1050);
    TEST_ASSERT_EQUAL_UINT32(3, st.pulses_total);
    wheel_chan_sample(&st, &cfg, 106, 1100);
    TEST_ASSERT_EQUAL_UINT32(6, st.pulses_total);
}

/* 2. 回绕：65000→100 判定为回绕，本窗口脉冲 = 65536−65000+100 = 636 */
static void test_wraparound(void)
{
    fresh();
    wheel_chan_sample(&st, &cfg, 65000, 1000);
    wheel_chan_sample(&st, &cfg, 100, 1050);
    TEST_ASSERT_EQUAL_UINT32(636, st.pulses_total);
}

/* 3. 清零复位：300→5 且旧值未过回绕阈值 → 本窗口 0 脉冲，计数基准重置为 5 */
static void test_clear_reset(void)
{
    fresh();
    wheel_chan_sample(&st, &cfg, 300, 1000);
    wheel_chan_sample(&st, &cfg, 5, 1050);
    TEST_ASSERT_EQUAL_UINT32(0, st.pulses_total);
    /* 复位后从 5 继续：5→10 增量 5 */
    wheel_chan_sample(&st, &cfg, 10, 1100);
    TEST_ASSERT_EQUAL_UINT32(5, st.pulses_total);
}

/* 4. 全零序列：无触发、无累计、math 保持初始 0 */
static void test_all_zero_sequence(void)
{
    fresh();
    for (int i = 0; i < 4; i++) {
        TEST_ASSERT_EQUAL_INT(ESP_OK, wheel_chan_sample(&st, &cfg, 0, 1000 + i * 50));
    }
    TEST_ASSERT_TRUE(st.trigger == false);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, st.math.rpm_ema);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, st.math.freq_hz);
}

/* 5. 累计脉冲跨多次回绕单调递增不丢数 */
static void test_pulses_monotonic_across_wraps(void)
{
    fresh();
    wheel_chan_sample(&st, &cfg, 65000, 1000);
    wheel_chan_sample(&st, &cfg, 100, 1050);   /* 回绕 +636 */
    wheel_chan_sample(&st, &cfg, 200, 1100);   /* +100 */
    wheel_chan_sample(&st, &cfg, 65000, 1150); /* +64800 */
    wheel_chan_sample(&st, &cfg, 50, 1200);    /* 回绕 +586 */
    TEST_ASSERT_EQUAL_UINT32(636 + 100 + 64800 + 586, st.pulses_total);
}

/* 6. 换源基准重置：reset_baseline 后首窗口不产生巨大差值 */
static void test_reset_baseline_no_glitch(void)
{
    fresh();
    /* 先跑到 65000，模拟旧源高位计数 */
    wheel_chan_sample(&st, &cfg, 65000, 1000);
    /* 切换源：新源计数从 0 开始——先重置基准再喂新计数 */
    wheel_chan_reset_baseline(&st, 0);
    TEST_ASSERT_EQUAL_UINT32(0, st.pulses_total);
    /* 新源 0→2：正常 2 脉冲，而不是把 0 误判为回绕或复位清零 */
    wheel_chan_sample(&st, &cfg, 2, 1050);
    TEST_ASSERT_EQUAL_UINT32(2, st.pulses_total);
}

/* 7. 去抖：默认 0 关闭时脉冲全部计数；设 20ms 时同窗口第二脉冲被丢弃 */
static void test_debounce_zero_counts_all(void)
{
    fresh();
    /* 100→103（3 脉冲，无去抖） */
    wheel_chan_sample(&st, &cfg, 100, 1000);
    wheel_chan_sample(&st, &cfg, 103, 1050);
    TEST_ASSERT_EQUAL_UINT32(3, st.pulses_total);
}

static void test_debounce_drops_second_pulse(void)
{
    fresh();
    cfg.debounce_ms = 20;
    /* 同一 50ms 窗口内出现 2 个计数（磁铁双脉冲），去抖后只记 1 */
    wheel_chan_sample(&st, &cfg, 100, 1000);
    wheel_chan_sample(&st, &cfg, 102, 1050);
    TEST_ASSERT_EQUAL_UINT32(1, st.pulses_total);
}

/* 8. reset 清零累计（前端 reset_counts 命令） */
static void test_reset_counts(void)
{
    fresh();
    wheel_chan_sample(&st, &cfg, 100, 1000);
    wheel_chan_sample(&st, &cfg, 110, 1050);
    TEST_ASSERT_EQUAL_UINT32(10, st.pulses_total);
    wheel_chan_reset_counts(&st);
    TEST_ASSERT_EQUAL_UINT32(0, st.pulses_total);
    /* 重置基准为当前计数，后续增量正常 */
    wheel_chan_sample(&st, &cfg, 115, 1100);
    TEST_ASSERT_EQUAL_UINT32(5, st.pulses_total);
}

/* 9. 脉冲间隔测频法：间隔 1000000us（1s）→ 频率 1Hz → 60 RPM → 1 RPS。
 * 首样本挂起不直采：用抖动值触发基准提升，1s 间隔随后正常上报。 */
static void test_period_measurement_freq(void)
{
    fresh();
    wheel_chan_sample_period(&st, &cfg, 1000000u, 1000); /* 首样本挂起 */
    wheel_chan_sample_period(&st, &cfg, 1000001u, 1050); /* 上报，直采 EMA */
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 1.0f, st.math.freq_hz);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 60.0f, st.math.rpm_ema);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 1.0f, st.math.rpm_ema / 60.0f);
}

/* 10. 快速脉冲间隔（100ms → 10Hz）→ 10 RPS → 600 RPM */
static void test_period_measurement_fast(void)
{
    fresh();
    wheel_chan_sample_period(&st, &cfg, 100000u, 1000); /* 首样本挂起 */
    wheel_chan_sample_period(&st, &cfg, 100001u, 1050); /* 上报，直采 EMA */
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 10.0f, st.math.freq_hz);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 600.0f, st.math.rpm_ema);
}

/* 11. 间隔为零（中断异常/未检测）→ 返回错误且不产生 NaN */
static void test_period_zero_interval(void)
{
    fresh();
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, wheel_chan_sample_period(&st, &cfg, 0, 1000));
    TEST_ASSERT_TRUE(isfinite(st.math.freq_hz));
}

/* 12. 静止归零兜底：有脉冲后长期无脉冲 → math 全字段清零 */
static void test_idle_timeout_zeroes(void)
{
    fresh();
    /* 先用间隔法建立转速：首样本挂起，第二个不同间隔提升基准并上报 */
    wheel_chan_sample_period(&st, &cfg, 100000u, 1000);
    wheel_chan_sample_period(&st, &cfg, 100001u, 1050); /* 10Hz → 600 RPM */
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 600.0f, st.math.rpm_ema);
    /* 计数也递增一下，建立 last_pulse_ms */
    wheel_chan_sample(&st, &cfg, 1, 1050);
    /* 长期无脉冲（超过 2000ms），sample 兜底归零 */
    wheel_chan_sample(&st, &cfg, 1, 3150);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, st.math.rpm_ema);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, st.math.speed_cm_s);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, st.math.speed_km_h);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, st.math.freq_hz);
    TEST_ASSERT_FALSE(st.math.has_sample);
}

/* 13. 归零后恢复脉冲：直采不缓升（首样本挂起，第二个间隔提升基准后一步到位） */
static void test_revive_after_idle(void)
{
    fresh();
    wheel_chan_sample_period(&st, &cfg, 100000u, 1000); /* 首样本挂起 */
    wheel_chan_sample_period(&st, &cfg, 100001u, 1050); /* 上报：600 RPM */
    wheel_chan_sample(&st, &cfg, 1, 1050);
    /* 静止 */
    wheel_chan_sample(&st, &cfg, 1, 3150);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, st.math.rpm_ema);
    TEST_ASSERT_FALSE(st.math.has_sample);
    /* 恢复脉冲：首样本挂起，第二个间隔提升基准后上报直采，alpha 极小也一步到 600 */
    wheel_chan_sample_period(&st, &cfg, 100000u, 3200);
    wheel_chan_sample_period(&st, &cfg, 100001u, 3250);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 600.0f, st.math.rpm_ema);
}

/* ---- 间隔切分合并（相机时钟耦合假脉冲） ---- */

static uint32_t g_now;

/* 喂一个间隔，按 50ms 采样节拍推进时间。
 * 返回它是否真的被数学层采纳：未上报（含首样本挂起）时 last_pulse_ms 不刷新。 */
static bool feed_period(uint32_t interval_us)
{
    g_now += 50;
    uint32_t before = st.math.last_pulse_ms;
    (void)wheel_chan_sample_period(&st, &cfg, interval_us, g_now);
    return st.math.last_pulse_ms != before;
}

/* 14. 正常等间隔序列（带 ±µs 抖动）：首样本挂起后每个新间隔都照常上报，
 * 不挂起、不累计可疑计数；末尾回到精确基准值供精确断言 */
static void test_stable_sequence_unchanged(void)
{
    fresh();
    g_now = 1000;
    TEST_ASSERT_FALSE(feed_period(1000000u)); /* 首样本挂起（不直采） */
    for (int i = 1; i < 12; i++) {
        TEST_ASSERT_TRUE(feed_period(1000000u + (uint32_t)(i % 3u)));
    }
    TEST_ASSERT_TRUE(feed_period(1000000u));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.0f, st.math.freq_hz);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 60.0f, st.math.rpm_ema);
    TEST_ASSERT_EQUAL_UINT32(0, st.suspect_count);
    TEST_ASSERT_EQUAL_UINT32(0, st.pending_interval_us);
    TEST_ASSERT_EQUAL_UINT32(1000000u, st.last_valid_interval_us);
}

/* 15. 假脉冲切分 150+850：短片段挂起，与 850 合并回 1000ms，
 * 不把 150ms（400 RPM）当真实转速上报 */
static void test_split_150_850_merged(void)
{
    fresh();
    g_now = 1000;
    TEST_ASSERT_FALSE(feed_period(1000000u)); /* 首样本挂起（不直采） */
    TEST_ASSERT_FALSE(feed_period(150000u)); /* 1000 静默提升基准；150 短段挂起 */
    TEST_ASSERT_EQUAL_UINT32(1, st.suspect_count);
    TEST_ASSERT_EQUAL_UINT32(150000u, st.pending_interval_us);
    TEST_ASSERT_TRUE(feed_period(850000u));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.0f, st.math.freq_hz);
    TEST_ASSERT_EQUAL_UINT32(0, st.pending_interval_us);
    TEST_ASSERT_EQUAL_UINT32(1000000u, st.last_valid_interval_us);
}

/* 16. 假脉冲切分 400+600：400 挂起，600 到来后合并回 1000ms */
static void test_split_400_600_merged(void)
{
    fresh();
    g_now = 1000;
    TEST_ASSERT_FALSE(feed_period(1000000u)); /* 首样本挂起（不直采） */
    TEST_ASSERT_FALSE(feed_period(400000u)); /* 1000 静默提升基准；400 短段挂起 */
    TEST_ASSERT_EQUAL_UINT32(1, st.suspect_count);
    TEST_ASSERT_TRUE(feed_period(600000u));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.0f, st.math.freq_hz);
    TEST_ASSERT_EQUAL_UINT32(0, st.pending_interval_us);
    TEST_ASSERT_EQUAL_UINT32(1000000u, st.last_valid_interval_us);
}

/* 17. 假脉冲落在 850ms 处（850 先到、150 后到）：150 到来时触发长段回滚，
 * 850+150 合并回 1000ms 重放，400 RPM 尖峰与 +3 RPM 台阶都不出现 */
static void test_split_850_150_no_spike(void)
{
    fresh();
    g_now = 1000;
    TEST_ASSERT_FALSE(feed_period(1000000u)); /* 首样本挂起（不直采） */
    TEST_ASSERT_TRUE(feed_period(850000u)); /* 1000 静默提升基准；长片段上报直采约 70 RPM */
    TEST_ASSERT_TRUE(feed_period(150000u)); /* 回滚：撤销 850，按 1000 重放直采 60 */
    TEST_ASSERT_EQUAL_UINT32(1, st.suspect_count);
    TEST_ASSERT_EQUAL_UINT32(0, st.pending_interval_us);
    TEST_ASSERT_EQUAL_UINT32(1000000u, st.last_valid_interval_us);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 60.0f, st.math.rpm_ema);
    /* 回到真实周期后继续保持稳定 */
    TEST_ASSERT_TRUE(feed_period(1000000u));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.0f, st.math.freq_hz);
    TEST_ASSERT_EQUAL_UINT32(0, st.pending_interval_us);
    TEST_ASSERT_EQUAL_UINT32(1000000u, st.last_valid_interval_us);
}

/* 18. 真实缓加速 1000→900→810：每段都超过上一步的一半，不挂起、不合并 */
static void test_slow_accel_not_merged(void)
{
    fresh();
    g_now = 1000;
    TEST_ASSERT_FALSE(feed_period(1000000u)); /* 首样本挂起（不直采） */
    TEST_ASSERT_TRUE(feed_period(900000u)); /* 1000 静默提升基准；900 正常上报 */
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1000000.0f / 900000.0f, st.math.freq_hz);
    TEST_ASSERT_TRUE(feed_period(810000u));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1000000.0f / 810000.0f, st.math.freq_hz);
    TEST_ASSERT_EQUAL_UINT32(0, st.suspect_count);
    TEST_ASSERT_EQUAL_UINT32(0, st.pending_interval_us);
}

/* 19. 挂起后停转：1.5×上次有效间隔后丢弃挂起并清空合并基准，
 * 2s 静止兜底把 RPM 归零；恢复时首样本挂起、第二个间隔提升基准后直采，
 * 不被旧基准误挂起。前两拍先用抖动对建立基准与 EMA（新首样本语义）。 */
static void test_pending_then_stop_zeroes(void)
{
    fresh();
    g_now = 1000;
    TEST_ASSERT_FALSE(feed_period(1000001u)); /* 首样本挂起（不直采） */
    TEST_ASSERT_TRUE(feed_period(1000000u)); /* 提升基准并上报，last_pulse_ms=1100 */
    TEST_ASSERT_FALSE(feed_period(150000u)); /* 挂起，pending_since=1150 */
    TEST_ASSERT_EQUAL_UINT32(1, st.suspect_count);

    /* 停转：计数不变，采样任务继续按 50ms 节拍推进 sample */
    wheel_chan_sample(&st, &cfg, 0, 1150);
    wheel_chan_sample(&st, &cfg, 0, 2700); /* 挂起年龄 1550ms > 1.5s */
    TEST_ASSERT_EQUAL_UINT32(0, st.pending_interval_us);
    TEST_ASSERT_EQUAL_UINT32(0, st.last_valid_interval_us);
    wheel_chan_sample(&st, &cfg, 0, 3200); /* 距上次有效脉冲 2100ms > 2s */
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, st.math.rpm_ema);
    TEST_ASSERT_FALSE(st.math.has_sample);

    /* 恢复：无合并基准，新间隔先挂起，第二个间隔提升基准后直采 600 */
    g_now = 3200;
    TEST_ASSERT_FALSE(feed_period(100000u)); /* 恢复首样本挂起 */
    TEST_ASSERT_TRUE(feed_period(100001u)); /* 上报直采，一步到 600 */
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 600.0f, st.math.rpm_ema);
    TEST_ASSERT_EQUAL_UINT32(1, st.suspect_count); /* 恢复不算可疑 */
}

/* 20. 连续两个假脉冲（150 + 250，真实 1000）：150 挂起后与 250 相加
 * 明显偏离基准，不得错误合并；当前 250 正常上报，随后回到真实周期 */
static void test_two_fakes_no_wrong_merge(void)
{
    fresh();
    g_now = 1000;
    TEST_ASSERT_FALSE(feed_period(1000000u)); /* 首样本挂起（不直采） */
    TEST_ASSERT_FALSE(feed_period(150000u)); /* 1000 静默提升基准；150 挂起 */
    TEST_ASSERT_EQUAL_UINT32(1, st.suspect_count);
    TEST_ASSERT_TRUE(feed_period(250000u)); /* 挂起+250=400，不合并；250 首次上报直采 */
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 4.0f, st.math.freq_hz); /* 250ms → 240 RPM */
    TEST_ASSERT_EQUAL_UINT32(0, st.pending_interval_us);
    TEST_ASSERT_EQUAL_UINT32(250000u, st.last_valid_interval_us);
    TEST_ASSERT_TRUE(feed_period(600000u));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1000000.0f / 600000.0f, st.math.freq_hz);
    TEST_ASSERT_TRUE(feed_period(1000000u));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.0f, st.math.freq_hz);
    TEST_ASSERT_EQUAL_UINT32(1, st.suspect_count); /* 全程只挂起过一次 */
}

/* 21.【回归】首样本（无基准）：不再直采——挂起等待第二个间隔，
 * 不上报、不进 EMA、不计可疑；第二个不同间隔到来时静默提升基准，
 * 新间隔正常上报直采 */
static void test_first_sample_held_pending(void)
{
    fresh();
    g_now = 1000;
    TEST_ASSERT_FALSE(feed_period(100000u)); /* 首样本挂起，不直采 */
    TEST_ASSERT_EQUAL_UINT32(0, st.suspect_count);
    TEST_ASSERT_EQUAL_UINT32(100000u, st.pending_interval_us);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, st.math.rpm_ema); /* EMA 不动 */
    TEST_ASSERT_FALSE(st.math.has_sample);
    /* 第二个不同间隔：静默提升 100000 为基准，100001 正常上报直采 */
    TEST_ASSERT_TRUE(feed_period(100001u));
    TEST_ASSERT_EQUAL_UINT32(0, st.suspect_count);
    TEST_ASSERT_EQUAL_UINT32(0, st.pending_interval_us);
    TEST_ASSERT_EQUAL_UINT32(100001u, st.last_valid_interval_us);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 600.0f, st.math.rpm_ema);
}

/* 22. 挂起期间同一间隔被采样任务重复轮询：不重复计数、不上报；
 * 真搭档到来时仍能合并（真实源每 50ms 都会重读同一个间隔） */
static void test_pending_repeat_poll_ignored(void)
{
    fresh();
    g_now = 1000;
    TEST_ASSERT_FALSE(feed_period(1000000u)); /* 首样本挂起（不直采） */
    TEST_ASSERT_FALSE(feed_period(150000u)); /* 1000 静默提升基准；150 挂起 */
    for (int i = 0; i < 5; i++) {
        TEST_ASSERT_FALSE(feed_period(150000u));
    }
    TEST_ASSERT_EQUAL_UINT32(1, st.suspect_count);
    TEST_ASSERT_TRUE(feed_period(850000u));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.0f, st.math.freq_hz);
}

/* ---- 长段回滚修正（假脉冲先切出长段的场景） ---- */

/* 23. 假脉冲落在 857ms（长段先上报）：长段单独算频会把 EMA 抬到约 63 RPM，
 * 短段 143ms 到来时确认 857+143 回到上上段 1000ms → 撤销长段上报并重放，
 * EMA 立即回到 60，消除 +3 RPM 的显示台阶。 */
static void test_long_segment_rollback(void)
{
    fresh();
    g_now = 1000;
    TEST_ASSERT_FALSE(feed_period(1000001u)); /* 首样本挂起（不直采） */
    TEST_ASSERT_TRUE(feed_period(1000000u)); /* 提升基准并上报，EMA 直采 60 */
    TEST_ASSERT_TRUE(feed_period(857000u));
    /* 台阶来源：长段被当成真实周期，EMA 从 60 抬到约 63 */
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 63.0f, st.math.rpm_ema);
    TEST_ASSERT_TRUE(feed_period(143000u));
    TEST_ASSERT_EQUAL_UINT32(0, st.pending_interval_us);
    TEST_ASSERT_EQUAL_UINT32(1, st.suspect_count);
    TEST_ASSERT_EQUAL_UINT32(1000000u, st.last_valid_interval_us);
    TEST_ASSERT_EQUAL_UINT32(1000000u, st.prev_valid_interval_us); /* 回滚不推进 prev */
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 60.0f, st.math.rpm_ema);
}

/* 24. 双假脉冲：长段 857 上报后 100 触发回滚（合并 957）；
 * 紧跟的 43 只比 1000 短 4.3%，不满足回滚前提，走原挂起与 900 合并成 943。
 * 全程无 240+ RPM 尖峰，终值仍收敛在 60 附近。 */
static void test_rollback_two_fakes(void)
{
    fresh();
    g_now = 1000;
    TEST_ASSERT_FALSE(feed_period(1000001u)); /* 首样本挂起（不直采） */
    TEST_ASSERT_TRUE(feed_period(1000000u)); /* 提升基准并上报，EMA 直采 60 */
    TEST_ASSERT_TRUE(feed_period(857000u));
    TEST_ASSERT_TRUE(feed_period(100000u)); /* 回滚：857+100=957 重放 */
    TEST_ASSERT_EQUAL_UINT32(1, st.suspect_count);
    TEST_ASSERT_EQUAL_UINT32(0, st.pending_interval_us);
    TEST_ASSERT_EQUAL_UINT32(957000u, st.last_valid_interval_us);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 60.0f, st.math.rpm_ema);

    TEST_ASSERT_FALSE(feed_period(43000u)); /* 957 不短于上上段 10%，只挂起 */
    TEST_ASSERT_EQUAL_UINT32(2, st.suspect_count);
    TEST_ASSERT_EQUAL_UINT32(43000u, st.pending_interval_us);
    TEST_ASSERT_TRUE(st.math.rpm_ema < 65.0f);

    TEST_ASSERT_TRUE(feed_period(900000u)); /* 43+900=943 落在 957±25% → 合并 */
    TEST_ASSERT_EQUAL_UINT32(0, st.pending_interval_us);
    TEST_ASSERT_EQUAL_UINT32(943000u, st.last_valid_interval_us);
    /* alpha=0.3 下 EMA 逐步收敛，尚未完全回到 60，但从未出现 240+ 尖峰 */
    TEST_ASSERT_FLOAT_WITHIN(2.0f, 60.0f, st.math.rpm_ema);
}

/* 25. 短段先到：上段 1000 几乎不短于上上段（不满足 10% 缩短前提），
 * 150 只挂起不回滚，math 停在 60；850 到来后按原合并路径回到 1000ms。
 * 前两拍先用抖动对建立基准与 EMA（新首样本语义）。 */
static void test_short_first_no_rollback(void)
{
    fresh();
    g_now = 1000;
    TEST_ASSERT_FALSE(feed_period(1000001u)); /* 首样本挂起（不直采） */
    TEST_ASSERT_TRUE(feed_period(1000000u)); /* 提升基准并上报，EMA 直采 60 */
    TEST_ASSERT_FALSE(feed_period(150000u));
    TEST_ASSERT_EQUAL_UINT32(1, st.suspect_count);
    TEST_ASSERT_EQUAL_UINT32(150000u, st.pending_interval_us);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 60.0f, st.math.rpm_ema); /* 未回滚，math 不动 */
    TEST_ASSERT_TRUE(feed_period(850000u));
    TEST_ASSERT_EQUAL_UINT32(0, st.pending_interval_us);
    TEST_ASSERT_EQUAL_UINT32(1, st.suspect_count);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 60.0f, st.math.rpm_ema);
}

/* 26. 真加速边界：850 上报（比上上段 1000 短 15%），下段 400ms 与它相加
 * 恰好 1250ms，压在上上段 +25% 窗口上沿（1000×5/4=1250，开区间不包含）
 * → 不触发回滚，按原逻辑挂起；600 到来合并成 1000ms。 */
static void test_accel_border_no_rollback(void)
{
    fresh();
    g_now = 1000;
    TEST_ASSERT_FALSE(feed_period(1000001u)); /* 首样本挂起（不直采） */
    TEST_ASSERT_TRUE(feed_period(1000000u)); /* 提升基准并上报，EMA 直采 60 */
    TEST_ASSERT_TRUE(feed_period(850000u)); /* 850 上报后 prev 回到精确 1000000 */
    TEST_ASSERT_FALSE(feed_period(400000u));
    TEST_ASSERT_EQUAL_UINT32(1, st.suspect_count);
    TEST_ASSERT_EQUAL_UINT32(400000u, st.pending_interval_us);
    TEST_ASSERT_EQUAL_UINT32(850000u, st.last_valid_interval_us); /* 未被回滚改写 */
    TEST_ASSERT_TRUE(feed_period(600000u)); /* 400+600=1000 落在 850±25% */
    TEST_ASSERT_EQUAL_UINT32(0, st.pending_interval_us);
    TEST_ASSERT_EQUAL_UINT32(1000000u, st.last_valid_interval_us);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.0f, st.math.freq_hz);
    TEST_ASSERT_EQUAL_UINT32(1, st.suspect_count);
}

/* 27.【回归】长段上报后同值重复轮询不得破坏回滚参照：真实源每 50ms 重读
 * 同一间隔，若重复值被当新事件上报，prev 会被覆写成 last（都为 800），
 * "800 比上上段 1000 短 10%" 永不成立 → 回滚失效 → 开摄像头时 RPM
 * 周期性跳涨且可疑计数同步 +1。重复轮询只允许保鲜 last_pulse_ms，
 * 参照必须原样保留，短段到来时回滚仍然成立。 */
static void test_rollback_survives_repeat_polls(void)
{
    fresh();
    g_now = 1000;
    TEST_ASSERT_FALSE(feed_period(1000001u)); /* 首样本挂起（不直采） */
    TEST_ASSERT_TRUE(feed_period(1000000u)); /* 提升基准并上报，EMA 直采 60 */
    TEST_ASSERT_TRUE(feed_period(800000u)); /* 长段先上报，EMA 抬到约 64.5 */
    for (int i = 0; i < 6; i++) {
        TEST_ASSERT_TRUE(feed_period(800000u)); /* 同值重复轮询：保鲜但不改参照 */
    }
    TEST_ASSERT_EQUAL_UINT32(800000u, st.last_valid_interval_us);
    TEST_ASSERT_EQUAL_UINT32(1000000u, st.prev_valid_interval_us); /* 未被覆写 */
    TEST_ASSERT_TRUE(feed_period(200000u)); /* 短段：回滚 800+200=1000 重放 */
    TEST_ASSERT_EQUAL_UINT32(1, st.suspect_count); /* 重复轮询不产生额外可疑 */
    TEST_ASSERT_EQUAL_UINT32(0, st.pending_interval_us);
    TEST_ASSERT_EQUAL_UINT32(1000000u, st.last_valid_interval_us);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 60.0f, st.math.rpm_ema);
}

/* 28.【回归】挂起态结束（合并/丢弃）后，已消费搭档的重复轮询同样只保鲜：
 * 150 挂起后 850 合并回 1000，随后 850 的重复轮询不得被当成新长段
 * 上报（850<1000 但 > 一半，旧行为会再次推进 EMA）。 */
static void test_consumed_partner_repeat_poll_ignored(void)
{
    fresh();
    g_now = 1000;
    TEST_ASSERT_FALSE(feed_period(1000000u)); /* 首样本挂起（不直采） */
    TEST_ASSERT_FALSE(feed_period(150000u)); /* 1000 静默提升基准；150 挂起 */
    TEST_ASSERT_TRUE(feed_period(850000u)); /* 合并 150+850=1000，直采 60 */
    TEST_ASSERT_EQUAL_UINT32(1000000u, st.last_valid_interval_us);
    for (int i = 0; i < 5; i++) {
        TEST_ASSERT_TRUE(feed_period(850000u)); /* 只保鲜，不再上报 */
    }
    TEST_ASSERT_EQUAL_UINT32(1, st.suspect_count);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.0f, st.math.freq_hz);
    TEST_ASSERT_EQUAL_UINT32(1000000u, st.last_valid_interval_us);
}

/* 29.【回归】启动瞬间首个间隔是垃圾短值：新语义下首样本挂起不直采，
 * 垃圾值不上屏（EMA 保持 0、不计可疑）；真实周期到来时静默提升基准并
 * 直采 60 RPM，全程无 400 RPM 尖峰。随后重复轮询按 50ms 步长把 EMA
 * 保持指向最后有效值——推进参照是 last_valid 而非重复值本身，合并/回滚
 * 后 last_seen 是切分段值，按它推进会重新引入台阶。 */
static void test_garbage_first_sample_not_displayed(void)
{
    fresh();
    g_now = 1000;
    TEST_ASSERT_FALSE(feed_period(150000u)); /* 垃圾首样本挂起：不上报、EMA 不动 */
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, st.math.rpm_ema);
    TEST_ASSERT_TRUE(feed_period(1000000u)); /* 150 静默提升基准；1000 上报直采 60 */
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 60.0f, st.math.rpm_ema);
    for (int i = 0; i < 20; i++) {
        TEST_ASSERT_TRUE(feed_period(1000000u)); /* 重复轮询只保鲜，EMA 稳在 60 */
    }
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 60.0f, st.math.rpm_ema);
    TEST_ASSERT_EQUAL_UINT32(0, st.suspect_count);
    TEST_ASSERT_EQUAL_UINT32(1000000u, st.last_valid_interval_us);
}

NATIVE_TEST_MAIN(
    UnityDefaultTestRun(test_normal_increment, "test_normal_increment", __LINE__);
    UnityDefaultTestRun(test_wraparound, "test_wraparound", __LINE__);
    UnityDefaultTestRun(test_clear_reset, "test_clear_reset", __LINE__);
    UnityDefaultTestRun(test_all_zero_sequence, "test_all_zero_sequence", __LINE__);
    UnityDefaultTestRun(test_pulses_monotonic_across_wraps, "test_pulses_monotonic_across_wraps", __LINE__);
    UnityDefaultTestRun(test_reset_baseline_no_glitch, "test_reset_baseline_no_glitch", __LINE__);
    UnityDefaultTestRun(test_debounce_zero_counts_all, "test_debounce_zero_counts_all", __LINE__);
    UnityDefaultTestRun(test_debounce_drops_second_pulse, "test_debounce_drops_second_pulse", __LINE__);
    UnityDefaultTestRun(test_reset_counts, "test_reset_counts", __LINE__);
    UnityDefaultTestRun(test_period_measurement_freq, "test_period_measurement_freq", __LINE__);
    UnityDefaultTestRun(test_period_measurement_fast, "test_period_measurement_fast", __LINE__);
    UnityDefaultTestRun(test_period_zero_interval, "test_period_zero_interval", __LINE__);
    UnityDefaultTestRun(test_idle_timeout_zeroes, "test_idle_timeout_zeroes", __LINE__);
    UnityDefaultTestRun(test_revive_after_idle, "test_revive_after_idle", __LINE__);
    UnityDefaultTestRun(test_stable_sequence_unchanged, "test_stable_sequence_unchanged", __LINE__);
    UnityDefaultTestRun(test_split_150_850_merged, "test_split_150_850_merged", __LINE__);
    UnityDefaultTestRun(test_split_400_600_merged, "test_split_400_600_merged", __LINE__);
    UnityDefaultTestRun(test_split_850_150_no_spike, "test_split_850_150_no_spike", __LINE__);
    UnityDefaultTestRun(test_slow_accel_not_merged, "test_slow_accel_not_merged", __LINE__);
    UnityDefaultTestRun(test_pending_then_stop_zeroes, "test_pending_then_stop_zeroes", __LINE__);
    UnityDefaultTestRun(test_two_fakes_no_wrong_merge, "test_two_fakes_no_wrong_merge", __LINE__);
    UnityDefaultTestRun(test_first_sample_held_pending, "test_first_sample_held_pending", __LINE__);
    UnityDefaultTestRun(test_pending_repeat_poll_ignored, "test_pending_repeat_poll_ignored", __LINE__);
    UnityDefaultTestRun(test_long_segment_rollback, "test_long_segment_rollback", __LINE__);
    UnityDefaultTestRun(test_rollback_two_fakes, "test_rollback_two_fakes", __LINE__);
    UnityDefaultTestRun(test_short_first_no_rollback, "test_short_first_no_rollback", __LINE__);
    UnityDefaultTestRun(test_accel_border_no_rollback, "test_accel_border_no_rollback", __LINE__);
    UnityDefaultTestRun(test_rollback_survives_repeat_polls, "test_rollback_survives_repeat_polls", __LINE__);
    UnityDefaultTestRun(test_consumed_partner_repeat_poll_ignored, "test_consumed_partner_repeat_poll_ignored", __LINE__);
    UnityDefaultTestRun(test_garbage_first_sample_not_displayed, "test_garbage_first_sample_not_displayed", __LINE__);
)
