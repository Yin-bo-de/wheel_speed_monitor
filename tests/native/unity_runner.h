/*
 * 每个宿主机测试可执行文件共享的 main() 模板。
 * 用法：
 *   NATIVE_TEST_MAIN(
 *       UnityDefaultTestRun(test_xxx, "test_xxx", __LINE__);
 *   )
 */
#ifndef UNITY_RUNNER_H
#define UNITY_RUNNER_H

#include "unity.h"

#define NATIVE_TEST_MAIN(...)                       \
    int main(void)                                  \
    {                                               \
        UnityBegin(__FILE__);                       \
        __VA_ARGS__                                 \
        return UnityEnd();                          \
    }

#endif /* UNITY_RUNNER_H */
