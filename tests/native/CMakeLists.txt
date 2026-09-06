# 宿主机纯逻辑单测工程（macOS clang + Unity + ctest）。
# 构建与运行：
#   cmake -S tests/native -B tests/native/build
#   cmake --build tests/native/build
#   ctest --test-dir tests/native/build --output-on-failure
cmake_minimum_required(VERSION 3.16)
project(wheel_speed_native_tests C)

set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)

# unity 是第三方上游源码（来自 ESP-IDF v5.5.2，Unity 2.6.0），
# 关掉告警选项避免未使用函数等告警升级为错误。
add_library(unity STATIC unity/unity.c)
target_include_directories(unity PUBLIC unity)
target_compile_options(unity PRIVATE -w)

# 组件纯 C 源文件（宿主机直接编译，不走 IDF 组件机制）。
# 各测试按 TDD 阶段增量添加。
# 各测试按 TDD 阶段增量添加：文件实现后立刻加入对应 SRCS 清单。
# 注意：代码还没写完的 .c 不要提前列进来，configure 会找不到文件。
# 从 tests/native 出发回项目根要 ../../
set(WHEEL_SPEED_SRCS
    ${CMAKE_CURRENT_SOURCE_DIR}/../../components/wheel_speed/wheel_speed_math.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../components/wheel_speed/wheel_speed_collector.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../components/wheel_speed/sim_source.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../components/wheel_speed/render.c
)
set(TELEMETRY_SRCS
    ${CMAKE_CURRENT_SOURCE_DIR}/../../components/telemetry/telemetry.c
)
set(STRATEGY_SRCS
    ${CMAKE_CURRENT_SOURCE_DIR}/../../components/strategy/strategy.c
)
set(MAIN_SRCS
    ${CMAKE_CURRENT_SOURCE_DIR}/../../main/config_params.c
)

# include 顺序：先本目录（esp_err_shim.h / unity_runner.h），再组件目录。
set(COMMON_INC_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../../components/wheel_speed/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../components/telemetry/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../components/strategy/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../main
)

function(add_native_test name)
    add_executable(${name} test_${name}.c ${ARGN})
    target_include_directories(${name} PRIVATE ${COMMON_INC_DIRS})
    target_compile_options(${name} PRIVATE -Wall -Wextra -Werror)
    target_link_libraries(${name} PRIVATE unity)
    add_test(NAME ${name} COMMAND ${name})
endfunction()

enable_testing()

add_native_test(smoke)
add_native_test(math ${WHEEL_SPEED_SRCS})
add_native_test(collector ${WHEEL_SPEED_SRCS})
add_native_test(sim_source ${WHEEL_SPEED_SRCS})
add_native_test(telemetry ${TELEMETRY_SRCS})
add_native_test(config_params ${MAIN_SRCS})
add_native_test(render ${WHEEL_SPEED_SRCS})
add_native_test(strategy ${STRATEGY_SRCS})
