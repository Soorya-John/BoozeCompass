#include "wifi_manager.h"

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_netif_sntp.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include <string.h>
#include <time.h>

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_RETRY_REQUEST_BIT BIT1
#define WIFI_RETRY_MAX_MS 30000

static const char *TAG = "wifi";
static EventGroupHandle_t s_events;
static bc_runtime_config_t s_config;
static bool s_wifi_started;
static bool s_sntp_started;

static esp_err_t apply_configuration(void)
{
    wifi_config_t wifi = {0};
    strlcpy((char *)wifi.sta.ssid, s_config.wifi_ssid, sizeof(wifi.sta.ssid));
    strlcpy((char *)wifi.sta.password, s_config.wifi_password,
            sizeof(wifi.sta.password));
    wifi.sta.threshold.authmode = WIFI_AUTH_OPEN;
    wifi.sta.pmf_cfg.capable = true;
    wifi.sta.pmf_cfg.required = false;
    return esp_wifi_set_config(WIFI_IF_STA, &wifi);
}

static void event_handler(void *argument, esp_event_base_t base,
                          int32_t event_id, void *event_data)
{
    (void)argument;
    (void)event_data;
    if (base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupClearBits(s_events, WIFI_CONNECTED_BIT);
        xEventGroupSetBits(s_events, WIFI_RETRY_REQUEST_BIT);
        ESP_LOGW(TAG, "Hotspot disconnected; reconnect task will retry");
    } else if (base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        if (!s_sntp_started) {
            const esp_sntp_config_t sntp_config =
                ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
            const esp_err_t time_err = esp_netif_sntp_init(&sntp_config);
            s_sntp_started = time_err == ESP_OK;
            if (time_err != ESP_OK) {
                ESP_LOGW(TAG, "SNTP initialization failed: %s",
                         esp_err_to_name(time_err));
            }
        }
        xEventGroupSetBits(s_events, WIFI_CONNECTED_BIT);
        ESP_LOGI(TAG, "Connected to hotspot and received an IP address");
    }
}

static void connection_task(void *argument)
{
    (void)argument;
    uint32_t backoff_ms = 1000;
    for (;;) {
        EventBits_t bits = xEventGroupGetBits(s_events);
        if (!s_config.wifi_configured) {
            xEventGroupWaitBits(s_events, WIFI_RETRY_REQUEST_BIT,
                                pdTRUE, pdFALSE, pdMS_TO_TICKS(1000));
            continue;
        }
        if ((bits & WIFI_CONNECTED_BIT) != 0) {
            backoff_ms = 1000;
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        ESP_LOGI(TAG, "Connecting to configured hotspot (SSID hidden)");
        esp_err_t err = esp_wifi_connect();
        if (err != ESP_OK && err != ESP_ERR_WIFI_CONN) {
            ESP_LOGW(TAG, "esp_wifi_connect: %s", esp_err_to_name(err));
        }
        const EventBits_t connected = xEventGroupWaitBits(
            s_events, WIFI_CONNECTED_BIT, pdFALSE, pdFALSE,
            pdMS_TO_TICKS(backoff_ms));
        xEventGroupClearBits(s_events, WIFI_RETRY_REQUEST_BIT);
        if (connected & WIFI_CONNECTED_BIT) {
            backoff_ms = 1000;
        } else {
            backoff_ms = backoff_ms >= WIFI_RETRY_MAX_MS / 2
                ? WIFI_RETRY_MAX_MS : backoff_ms * 2;
        }
    }
}

esp_err_t bc_wifi_manager_init(const bc_runtime_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    memcpy(&s_config, config, sizeof(s_config));
    s_events = xEventGroupCreate();
    if (s_events == NULL) {
        return ESP_ERR_NO_MEM;
    }
    esp_netif_create_default_wifi_sta();
    const wifi_init_config_t init_config = WIFI_INIT_CONFIG_DEFAULT();
    esp_err_t err = esp_wifi_init(&init_config);
    if (err != ESP_OK && err != ESP_ERR_WIFI_INIT_STATE) {
        return err;
    }
    err = esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                     event_handler, NULL);
    if (err != ESP_OK) return err;
    err = esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                     event_handler, NULL);
    if (err != ESP_OK) return err;
    err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err != ESP_OK) return err;
    err = apply_configuration();
    if (err != ESP_OK) return err;
    err = esp_wifi_start();
    if (err != ESP_OK && err != ESP_ERR_WIFI_NOT_STARTED) return err;
    s_wifi_started = true;
    if (xTaskCreate(connection_task, "wifi_reconnect", 3072, NULL, 4, NULL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

bool bc_wifi_manager_is_connected(void)
{
    return s_events != NULL &&
        (xEventGroupGetBits(s_events) & WIFI_CONNECTED_BIT) != 0;
}

esp_err_t bc_wifi_manager_wait_for_time(uint32_t timeout_ms)
{
    if (!s_sntp_started) return ESP_ERR_INVALID_STATE;
    time_t current_time = 0;
    time(&current_time);
    if (current_time >= 1700000000) return ESP_OK;
    const esp_err_t err = esp_netif_sntp_sync_wait(pdMS_TO_TICKS(timeout_ms));
    if (err != ESP_OK) return err;
    time(&current_time);
    return current_time >= 1700000000 ? ESP_OK : ESP_ERR_INVALID_STATE;
}

esp_err_t bc_wifi_manager_set_credentials(const char *ssid, const char *password)
{
    esp_err_t err = bc_configuration_save_wifi(ssid, password);
    if (err != ESP_OK) {
        return err;
    }
    strlcpy(s_config.wifi_ssid, ssid, sizeof(s_config.wifi_ssid));
    strlcpy(s_config.wifi_password, password, sizeof(s_config.wifi_password));
    s_config.wifi_configured = true;
    if (!s_wifi_started) {
        return ESP_ERR_INVALID_STATE;
    }
    err = apply_configuration();
    if (err != ESP_OK) return err;
    xEventGroupClearBits(s_events, WIFI_CONNECTED_BIT);
    xEventGroupSetBits(s_events, WIFI_RETRY_REQUEST_BIT);
    return esp_wifi_disconnect();
}
