idf_component_register(SRCS "store_finder.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_http_client mbedtls json wifi_manager navigation
                                freertos esp_timer)
