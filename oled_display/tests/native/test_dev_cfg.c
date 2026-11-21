/* dev_cfg 单测：配置模型的默认值与逐字段校验。 */
#include <string.h>

#include "dev_cfg.h"
#include "unity.h"
#include "unity_runner.h"

void setUp(void) {}
void tearDown(void) {}

static dev_cfg_t cfg_ok(void)
{
    dev_cfg_t c;
    dev_cfg_defaults(&c);
    snprintf(c.wifi_ssid, sizeof(c.wifi_ssid), "wheel-bench");
    return c;
}

static void test_defaults(void)
{
    dev_cfg_t c;
    dev_cfg_defaults(&c);
    TEST_ASSERT_EQUAL_UINT32(660, c.diam_x10);
    TEST_ASSERT_EQUAL_UINT32(2000, c.disconnect_ms);
    TEST_ASSERT_EQUAL_STRING("", c.server_uri);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 66.0f, dev_cfg_diam_mm(&c));
}

static void test_valid_config(void)
{
    dev_cfg_t c = cfg_ok();
    TEST_ASSERT_EQUAL_INT(ESP_OK, dev_cfg_validate(&c));
    snprintf(c.server_uri, sizeof(c.server_uri), "ws://192.168.2.1:8080/ws");
    TEST_ASSERT_EQUAL_INT(ESP_OK, dev_cfg_validate(&c));
}

static void test_ssid_required(void)
{
    dev_cfg_t c = cfg_ok();
    c.wifi_ssid[0] = '\0';
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, dev_cfg_validate(&c));
}

static void test_range_checks(void)
{
    dev_cfg_t c = cfg_ok();
    c.diam_x10 = 100; /* 10mm，低于下限 */
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, dev_cfg_validate(&c));
    c = cfg_ok();
    c.diam_x10 = 3000;
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, dev_cfg_validate(&c));
    c = cfg_ok();
    c.disconnect_ms = 100;
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, dev_cfg_validate(&c));
    c = cfg_ok();
    c.disconnect_ms = 60000;
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, dev_cfg_validate(&c));
}

static void test_uri_scheme(void)
{
    dev_cfg_t c = cfg_ok();
    snprintf(c.server_uri, sizeof(c.server_uri), "http://1.2.3.4/ws");
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, dev_cfg_validate(&c));
    snprintf(c.server_uri, sizeof(c.server_uri), "ws://1.2.3.4/ws");
    TEST_ASSERT_EQUAL_INT(ESP_OK, dev_cfg_validate(&c));
}

static void test_rejects_ctrl_chars(void)
{
    dev_cfg_t c = cfg_ok();
    snprintf(c.wifi_ssid, sizeof(c.wifi_ssid), "bad\nssid");
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, dev_cfg_validate(&c));
}

NATIVE_TEST_MAIN(
    UnityDefaultTestRun(test_defaults, "test_defaults", __LINE__);
    UnityDefaultTestRun(test_valid_config, "test_valid_config", __LINE__);
    UnityDefaultTestRun(test_ssid_required, "test_ssid_required", __LINE__);
    UnityDefaultTestRun(test_range_checks, "test_range_checks", __LINE__);
    UnityDefaultTestRun(test_uri_scheme, "test_uri_scheme", __LINE__);
    UnityDefaultTestRun(test_rejects_ctrl_chars, "test_rejects_ctrl_chars", __LINE__);
)
