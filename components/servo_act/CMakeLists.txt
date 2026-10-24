# 纯逻辑层：不依赖 IDF，宿主机可直接编译测试（见 tests/native/CMakeLists.txt）。
# 真实 LEDC 输出在 components/servo_drv，与本组件共用 servo_act_ops_t 接口。
idf_component_register(
    SRCS "servo_sim.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_common
)
