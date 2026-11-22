idf_component_register(
    SRCS "oled_main.c" "dev_cfg.c" "dev_cfg_store.c" "speed_calc.c"
         "ui_render.c" "ssd1306.c" "net_wifi.c" "feed_ws.c"
         "http_portal.c" "ota_portal.c"
    INCLUDE_DIRS "."
    EMBED_FILES "page.html"
    REQUIRES nvs_flash esp_wifi esp_netif esp_event esp_http_server json
              app_update esp_partition esp_driver_i2c esp_driver_gpio esp_timer
              telem_frame espressif__esp_websocket_client
)
