# 硬件层：依赖 IDF 驱动，不参与宿主机测试（同 wheel_sensor 的约定）。
# 与 servo_act 共用 servo_act_ops_t 接口，由 main 按配置二选一。
idf_component_register(
    SRCS "servo_drv.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_ledc esp_common log servo_act
)
