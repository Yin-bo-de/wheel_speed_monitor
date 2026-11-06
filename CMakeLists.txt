cmake_minimum_required(VERSION 3.16)

# 页面显示的固件版本，编进 esp_app_desc_t；发版时改这里。
set(PROJECT_VER "1.0.0")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(wheel_speed_monitor)
