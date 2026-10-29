idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       REQUIRES driver esp_event esp_netif esp_wifi nvs_flash esp_timer
                                navigation gps compass mpu6050 motor configuration
                                wifi_manager store_finder)
