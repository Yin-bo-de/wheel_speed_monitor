/* telem_frame 单测：按车端 render 输出的帧结构验证解析。
 * 帧样例刻意带上了 cfg 块里的同名 mode 字段，防止解析器误抓嵌套值。 */
#include <string.h>

#include "telem_frame.h"
#include "unity.h"
#include "unity_runner.h"

void setUp(void) {}
void tearDown(void) {}

static const char *FRAME_FULL =
    "{\"type\":\"telemetry\",\"seq\":7,\"t\":123456,"
    "\"sys\":{\"gpio\":[1,1,1,1],\"free_heap\":100,\"heap_int\":0,\"uptime_s\":9,"
    "\"cam\":{\"policy\":\"on_demand\",\"state\":\"off\",\"fps\":0.0}},"
    "\"collectors\":["
    "{\"type\":\"wheel_speed\",\"src\":\"pcnt\",\"wheels\":["
    "{\"i\":0,\"en\":true,\"rpm\":200.5,\"rps\":3.3,\"speed_cms\":120.0,"
    "\"freq\":3.340,\"pulses\":9,\"suspect\":0,\"trigger\":true},"
    "{\"i\":1,\"en\":false,\"rpm\":0.0,\"rps\":0.0,\"speed_cms\":0.0,"
    "\"freq\":0.000,\"pulses\":0,\"suspect\":2,\"trigger\":false},"
    "{\"i\":2,\"en\":true,\"rpm\":60.0,\"rps\":1.0,\"speed_cms\":36.0,"
    "\"freq\":1.000,\"pulses\":3,\"suspect\":0,\"trigger\":false},"
    "{\"i\":3,\"en\":true,\"rpm\":61.5,\"rps\":1.0,\"speed_cms\":37.0,"
    "\"freq\":1.025,\"pulses\":3,\"suspect\":0,\"trigger\":false}],"
    "\"cfg\":{\"magnets\":1,\"diam_mm\":66,\"alpha\":0.30,\"debounce_ms\":20,"
    "\"sim\":false,\"sim_rpm\":[0,0,0,0],\"gear_ratio\":[1.0,1.0,1.0,1.0]}},"
    "{\"type\":\"servo\",\"src\":\"sim\",\"mode\":\"manual\",\"mode_src\":\"rc\","
    "\"sim\":true,"
    "\"servos\":["
    "{\"i\":0,\"en\":true,\"phase\":\"locked\",\"hold_ms\":1234,\"ratio\":0.75,"
    "\"slip_left\":true,\"target_us\":1900,\"cur_us\":1850.5,\"reached\":false},"
    "{\"i\":1,\"en\":false,\"phase\":\"pending_sync\",\"hold_ms\":0,\"ratio\":0.0,"
    "\"slip_left\":false,\"target_us\":1100,\"cur_us\":1100.0,\"reached\":true}],"
    "\"cfg\":{\"mode\":\"auto\",\"engage\":0.30}},"
    "{\"type\":\"rc\",\"src\":\"sim\",\"mode\":\"auto\",\"mode_src\":\"cfg\","
    "\"ch\":[{\"us\":1500,\"valid\":true,\"seen\":true,\"age_ms\":0},"
    "{\"us\":1200,\"valid\":true,\"seen\":true,\"age_ms\":10},"
    "{\"us\":1700,\"valid\":true,\"seen\":true,\"age_ms\":20}],"
    "\"target\":[{\"us\":1500,\"from_rc\":false},"
    "{\"us\":1200,\"from_rc\":true}],\"cfg\":{\"sim\":true}}]}";

static void test_parse_full_frame(void)
{
    telem_frame_view_t v;
    TEST_ASSERT_TRUE(telem_frame_parse(FRAME_FULL, strlen(FRAME_FULL), &v));

    TEST_ASSERT_TRUE(v.has_wheel);
    TEST_ASSERT_TRUE(v.wheel_en[0]);
    TEST_ASSERT_FALSE(v.wheel_en[1]);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 200.5f, v.wheel_rpm[0]);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, v.wheel_rpm[1]);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 60.0f, v.wheel_rpm[2]);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 61.5f, v.wheel_rpm[3]);

    TEST_ASSERT_TRUE(v.has_mode);
    /* 取顶层 mode（manual），不能误抓 cfg.mode（auto） */
    TEST_ASSERT_FALSE(v.mode_auto);

    TEST_ASSERT_EQUAL_INT(TELEM_PHASE_LOCKED, v.phase[0]);
    TEST_ASSERT_EQUAL_INT(TELEM_PHASE_PENDING_SYNC, v.phase[1]);
    TEST_ASSERT_TRUE(v.phase_en[0]);
    TEST_ASSERT_FALSE(v.phase_en[1]);
}

static void test_parse_wheel_only(void)
{
    static const char *frame =
        "{\"type\":\"telemetry\",\"seq\":1,\"t\":0,\"collectors\":["
        "{\"type\":\"wheel_speed\",\"src\":\"sim\",\"wheels\":["
        "{\"i\":0,\"en\":true,\"rpm\":10.0},{\"i\":1,\"en\":true,\"rpm\":20.0},"
        "{\"i\":2,\"en\":true,\"rpm\":30.0},{\"i\":3,\"en\":true,\"rpm\":40.0}]}]}";
    telem_frame_view_t v;
    TEST_ASSERT_TRUE(telem_frame_parse(frame, strlen(frame), &v));
    TEST_ASSERT_TRUE(v.has_wheel);
    TEST_ASSERT_FALSE(v.has_mode);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 40.0f, v.wheel_rpm[3]);
}

static void test_parse_tolerates_new_fields(void)
{
    static const char *frame =
        "{\"type\":\"telemetry\",\"seq\":2,\"t\":0,\"future_field\":{\"a\":1},"
        "\"collectors\":["
        "{\"type\":\"new_collector\",\"blah\":true},"
        "{\"type\":\"wheel_speed\",\"wheels\":["
        "{\"i\":0,\"en\":true,\"rpm\":1.0,\"future\":\"x\"},"
        "{\"i\":1,\"en\":true,\"rpm\":2.0},{\"i\":2,\"en\":true,\"rpm\":3.0},"
        "{\"i\":3,\"en\":true,\"rpm\":4.0}]}]}";
    telem_frame_view_t v;
    TEST_ASSERT_TRUE(telem_frame_parse(frame, strlen(frame), &v));
    TEST_ASSERT_TRUE(v.has_wheel);
    TEST_ASSERT_FALSE(v.has_mode);
}

static void test_parse_bad_input(void)
{
    telem_frame_view_t v;
    TEST_ASSERT_FALSE(telem_frame_parse("", 0, &v));
    TEST_ASSERT_FALSE(telem_frame_parse("not json", 8, &v));
    TEST_ASSERT_FALSE(telem_frame_parse("{\"type\":\"telemetry\"}", 20, &v));
    TEST_ASSERT_FALSE(telem_frame_parse(NULL, 0, &v));
}

static void test_phase_from_str(void)
{
    TEST_ASSERT_EQUAL_INT(TELEM_PHASE_IDLE, telem_phase_from_str("idle"));
    TEST_ASSERT_EQUAL_INT(TELEM_PHASE_PENDING_SYNC,
                          telem_phase_from_str("pending_sync"));
    TEST_ASSERT_EQUAL_INT(TELEM_PHASE_PROBE, telem_phase_from_str("probe"));
    TEST_ASSERT_EQUAL_INT(TELEM_PHASE_LOCKED, telem_phase_from_str("locked"));
    TEST_ASSERT_EQUAL_INT(TELEM_PHASE_UNKNOWN, telem_phase_from_str("weird"));
    TEST_ASSERT_EQUAL_INT(TELEM_PHASE_UNKNOWN, telem_phase_from_str(NULL));
}

NATIVE_TEST_MAIN(
    UnityDefaultTestRun(test_parse_full_frame, "test_parse_full_frame", __LINE__);
    UnityDefaultTestRun(test_parse_wheel_only, "test_parse_wheel_only", __LINE__);
    UnityDefaultTestRun(test_parse_tolerates_new_fields, "test_parse_tolerates_new_fields", __LINE__);
    UnityDefaultTestRun(test_parse_bad_input, "test_parse_bad_input", __LINE__);
    UnityDefaultTestRun(test_phase_from_str, "test_phase_from_str", __LINE__);
)
