idf_component_register(
    SRCS "wheel_bench_main.c" "config_params.c" "config_store.c"
         "bench_http_server.c" "bench_cam.c" "bench_cam_stream.c"
    INCLUDE_DIRS "."
    EMBED_FILES "www/index.html"
    REQUIRES nvs_flash esp_wifi esp_netif esp_event esp_http_server json heap
              telemetry wheel_speed wheel_sensor strategy servo_act servo_drv
              rc_input rc_capture camera_drv cam_rate
)
