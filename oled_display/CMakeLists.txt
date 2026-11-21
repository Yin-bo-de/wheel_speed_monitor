cmake_minimum_required(VERSION 3.16)

# OLED 显示子项目固件版本：新模块起步 0.1.0，之后每次代码变更 +0.0.1，
# 重大变更（新增模块/配置项）+0.1.0。
set(PROJECT_VER "0.2.3")

# 共享契约组件（telem_frame）：与车端一处维护遥测帧契约
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../shared_components")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(oled_display)
