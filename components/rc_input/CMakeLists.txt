# 纯逻辑层：不依赖 IDF，宿主机可直接编译测试（见 tests/native/CMakeLists.txt）。
# 真实捕获在 components/rc_capture（GPIO 边沿中断），与本组件共用同一个模型。
idf_component_register(
    SRCS "rc_input.c" "rc_render.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_common
)
