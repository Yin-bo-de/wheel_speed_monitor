# 硬件层：GPIO 双沿中断捕获遥控脉宽，不宿主机测（见 CLAUDE.md 分层纪律）。
# 依赖 rc_input 只为复用脉宽量程常量（RC_US_MIN/MAX），与 servo_drv 依赖
# servo_act 同理——边界值不在两处各写一遍。
idf_component_register(
    SRCS "rc_capture.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_gpio esp_timer esp_common rc_input
)
