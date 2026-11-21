# OLED 子项目宿主机纯逻辑单测（macOS clang + Unity + ctest）。
# 构建与运行：
#   cmake -S oled_display/tests/native -B oled_display/tests/native/build
#   cmake --build oled_display/tests/native/build
#   ctest --test-dir oled_display/tests/native/build --output-on-failure
cmake_minimum_required(VERSION 3.16)
project(oled_display_native_tests C)

set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)

# unity 与 esp_err shim 复用车端宿主机测试基建，一处维护
set(NATIVE_COMMON ${CMAKE_CURRENT_SOURCE_DIR}/../../../tests/native)
add_library(unity STATIC ${NATIVE_COMMON}/unity/unity.c)
target_include_directories(unity PUBLIC ${NATIVE_COMMON}/unity)
target_compile_options(unity PRIVATE -w)

set(TELEM_FRAME_SRCS
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../shared_components/telem_frame/telem_frame.c
)

set(COMMON_INC_DIRS
    ${NATIVE_COMMON}/include                       # esp_err.h shim
    ${NATIVE_COMMON}                               # unity_runner.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../shared_components/telem_frame/include
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

add_native_test(telem_frame ${TELEM_FRAME_SRCS})
add_native_test(speed_calc ${CMAKE_CURRENT_SOURCE_DIR}/../../main/speed_calc.c)
add_native_test(ui_render ${CMAKE_CURRENT_SOURCE_DIR}/../../main/ui_render.c)
add_native_test(dev_cfg ${CMAKE_CURRENT_SOURCE_DIR}/../../main/dev_cfg.c)
