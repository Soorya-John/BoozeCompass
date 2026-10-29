#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stddef.h>

#define BC_WIFI_SSID_MAX 33
#define BC_WIFI_PASSWORD_MAX 65

typedef struct {
    char wifi_ssid[BC_WIFI_SSID_MAX];
    char wifi_password[BC_WIFI_PASSWORD_MAX];
    bool wifi_configured;
} bc_runtime_config_t;

typedef struct {
    double offset_x;
    double offset_y;
    double offset_z;
    double scale_x;
    double scale_y;
    double scale_z;
    double yaw_offset_deg;
    bool calibrated;
} bc_saved_mag_calibration_t;

esp_err_t bc_configuration_load(bc_runtime_config_t *config);
esp_err_t bc_configuration_save_wifi(const char *ssid, const char *password);
esp_err_t bc_configuration_load_mag(bc_saved_mag_calibration_t *calibration);
esp_err_t bc_configuration_save_mag(const bc_saved_mag_calibration_t *calibration);
