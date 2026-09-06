idf_component_register(
    SRCS "wheel_sensor.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_pcnt esp_driver_gpio esp_common esp_timer
)
