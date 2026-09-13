idf_component_register(
    SRCS "wheel_bench_main.c" "config_params.c" "config_store.c" "bench_http_server.c"
    INCLUDE_DIRS "."
    EMBED_FILES "www/index.html"
    REQUIRES nvs_flash esp_wifi esp_netif esp_event esp_http_server json
              telemetry wheel_speed wheel_sensor strategy servo_act servo_drv
)
