idf_component_register(
    SRCS "wheel_speed_math.c" "wheel_speed_collector.c" "sim_source.c" "render.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_common
)
