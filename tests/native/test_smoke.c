/* 脚手架自检：确认 Unity 断言与 ctest 红绿机制真实可用。 */
#include "unity.h"
#include "unity_runner.h"

void setUp(void) {}
void tearDown(void) {}

static void test_unity_assert_macros_work(void)
{
    TEST_ASSERT_TRUE(1 == 1);
    TEST_ASSERT_FALSE(1 == 2);
    TEST_ASSERT_EQUAL_INT(42, 42);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 3.14159f, 3.14159f);
}

NATIVE_TEST_MAIN(
    UnityDefaultTestRun(test_unity_assert_macros_work, "test_unity_assert_macros_work", __LINE__);
)
