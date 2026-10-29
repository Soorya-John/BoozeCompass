#pragma once

#include "configuration.h"
#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

esp_err_t bc_wifi_manager_init(const bc_runtime_config_t *config);
bool bc_wifi_manager_is_connected(void);
esp_err_t bc_wifi_manager_wait_for_time(uint32_t timeout_ms);
esp_err_t bc_wifi_manager_set_credentials(const char *ssid, const char *password);
