#include "configuration.h"

#include "esp_log.h"
#include "nvs.h"
#include <math.h>
#include <string.h>

#define CONFIG_NAMESPACE "boozecompass"
#define KEY_WIFI_SSID "wifi_ssid"
#define KEY_WIFI_PASSWORD "wifi_password"
#define KEY_MAG_CALIBRATION "mag_cal"

static const char *TAG = "configuration";

esp_err_t bc_configuration_load(bc_runtime_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(config, 0, sizeof(*config));
#ifdef CONFIG_BC_WIFI_SSID
    strlcpy(config->wifi_ssid, CONFIG_BC_WIFI_SSID, sizeof(config->wifi_ssid));
#endif
#ifdef CONFIG_BC_WIFI_PASSWORD
    strlcpy(config->wifi_password, CONFIG_BC_WIFI_PASSWORD,
            sizeof(config->wifi_password));
#endif

    nvs_handle_t handle;
    esp_err_t err = nvs_open(CONFIG_NAMESPACE, NVS_READONLY, &handle);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        err = ESP_OK;
    } else if (err == ESP_OK) {
        size_t ssid_size = sizeof(config->wifi_ssid);
        size_t password_size = sizeof(config->wifi_password);
        esp_err_t ssid_err = nvs_get_str(handle, KEY_WIFI_SSID,
                                         config->wifi_ssid, &ssid_size);
        esp_err_t password_err = nvs_get_str(handle, KEY_WIFI_PASSWORD,
                                             config->wifi_password, &password_size);
        if ((ssid_err != ESP_OK && ssid_err != ESP_ERR_NVS_NOT_FOUND) ||
            (password_err != ESP_OK && password_err != ESP_ERR_NVS_NOT_FOUND)) {
            err = ESP_FAIL;
        }
        nvs_close(handle);
    }
    config->wifi_configured = config->wifi_ssid[0] != '\0';
    if (config->wifi_configured) {
        ESP_LOGI(TAG, "Wi-Fi credentials loaded (SSID hidden)");
    } else {
        ESP_LOGW(TAG, "Wi-Fi is not configured; use menuconfig or 'wifi set'");
    }
    return err;
}

esp_err_t bc_configuration_save_wifi(const char *ssid, const char *password)
{
    if (ssid == NULL || password == NULL || ssid[0] == '\0' ||
        strlen(ssid) >= BC_WIFI_SSID_MAX ||
        strlen(password) >= BC_WIFI_PASSWORD_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    nvs_handle_t handle;
    esp_err_t err = nvs_open(CONFIG_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_str(handle, KEY_WIFI_SSID, ssid);
    if (err == ESP_OK) {
        err = nvs_set_str(handle, KEY_WIFI_PASSWORD, password);
    }
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    return err;
}

esp_err_t bc_configuration_load_mag(bc_saved_mag_calibration_t *calibration)
{
    if (calibration == NULL) return ESP_ERR_INVALID_ARG;
    nvs_handle_t handle;
    esp_err_t err = nvs_open(CONFIG_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) return err;
    size_t size = sizeof(*calibration);
    err = nvs_get_blob(handle, KEY_MAG_CALIBRATION, calibration, &size);
    nvs_close(handle);
    if (err == ESP_OK && size != sizeof(*calibration)) {
        return ESP_ERR_INVALID_SIZE;
    }
    return err;
}

esp_err_t bc_configuration_save_mag(const bc_saved_mag_calibration_t *calibration)
{
    if (calibration == NULL || !calibration->calibrated ||
        !isfinite(calibration->offset_x) || !isfinite(calibration->offset_y) ||
        !isfinite(calibration->offset_z) || !isfinite(calibration->scale_x) ||
        !isfinite(calibration->scale_y) || !isfinite(calibration->scale_z)) {
        return ESP_ERR_INVALID_ARG;
    }
    nvs_handle_t handle;
    esp_err_t err = nvs_open(CONFIG_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) return err;
    err = nvs_set_blob(handle, KEY_MAG_CALIBRATION, calibration,
                       sizeof(*calibration));
    if (err == ESP_OK) err = nvs_commit(handle);
    nvs_close(handle);
    return err;
}
