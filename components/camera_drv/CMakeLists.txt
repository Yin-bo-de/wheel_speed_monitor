# 硬件层：esp32-camera 的薄封装，依赖 IDF 与 PSRAM，不参与宿主机测试。
idf_component_register(
    SRCS "camera_drv.c"
    INCLUDE_DIRS "include"
    REQUIRES esp32-camera esp_common log
)
