#include "store_finder.h"

#include "cJSON.h"
#include "esp_crt_bundle.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "wifi_manager.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define RESPONSE_LIMIT 24576
#define QUERY_LIMIT 1024
#define POST_LIMIT 3072
#define HTTP_ATTEMPTS 2

static const char *TAG = "store_finder";
static QueueHandle_t s_request_queue;
static QueueHandle_t s_result_queue;

typedef struct {
    double latitude_deg;
    double longitude_deg;
    unsigned radius_m;
} search_request_t;

typedef struct {
    char data[RESPONSE_LIMIT + 1];
    size_t length;
    bool overflow;
} response_buffer_t;

static int64_t monotonic_ms(void)
{
    return esp_timer_get_time() / 1000;
}

static esp_err_t http_event(esp_http_client_event_t *event)
{
    response_buffer_t *response = event->user_data;
    if (event->event_id == HTTP_EVENT_ON_DATA && event->data_len > 0 &&
        response != NULL) {
        const size_t bytes = (size_t)event->data_len;
        if (response->length + bytes > RESPONSE_LIMIT) {
            response->overflow = true;
            return ESP_FAIL;
        }
        memcpy(response->data + response->length, event->data, bytes);
        response->length += bytes;
        response->data[response->length] = '\0';
    }
    return ESP_OK;
}

static bool append_encoded(char *out, size_t capacity, size_t *used,
                           const char *input)
{
    static const char hex[] = "0123456789ABCDEF";
    for (const unsigned char *p = (const unsigned char *)input; *p != '\0'; ++p) {
        const bool safe = (*p >= 'A' && *p <= 'Z') ||
                          (*p >= 'a' && *p <= 'z') ||
                          (*p >= '0' && *p <= '9') ||
                          *p == '-' || *p == '_' || *p == '.' || *p == '~';
        const size_t needed = safe ? 1 : 3;
        if (*used + needed >= capacity) return false;
        if (safe) {
            out[(*used)++] = (char)*p;
        } else {
            out[(*used)++] = '%';
            out[(*used)++] = hex[*p >> 4];
            out[(*used)++] = hex[*p & 0x0F];
        }
    }
    out[*used] = '\0';
    return true;
}

static bool build_post_body(const search_request_t *request,
                            char *post, size_t post_capacity)
{
    char query[QUERY_LIMIT];
    const int length = snprintf(query, sizeof(query),
        "[out:json][timeout:20];nwr(around:%u,%.6f,%.6f)"
        "[shop~\"^(alcohol|wine)$\"];out center tags;",
        request->radius_m, request->latitude_deg, request->longitude_deg);
    if (length < 0 || (size_t)length >= sizeof(query)) return false;
    size_t used = 0;
    const char prefix[] = "data=";
    memcpy(post, prefix, sizeof(prefix) - 1);
    used = sizeof(prefix) - 1;
    post[used] = '\0';
    return append_encoded(post, post_capacity, &used, query);
}

static const char *json_string(const cJSON *object, const char *name)
{
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(object, name);
    return cJSON_IsString(item) ? item->valuestring : NULL;
}

static bool json_number(const cJSON *object, const char *name, double *value)
{
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(object, name);
    if (!cJSON_IsNumber(item) || !isfinite(item->valuedouble)) return false;
    *value = item->valuedouble;
    return true;
}

static bool parse_response(const char *json, bc_store_search_result_t *result)
{
    cJSON *root = cJSON_ParseWithLength(json, strlen(json));
    if (root == NULL) return false;
    cJSON *elements = cJSON_GetObjectItemCaseSensitive(root, "elements");
    if (!cJSON_IsArray(elements)) {
        cJSON_Delete(root);
        return false;
    }
    const int64_t observed = monotonic_ms();
    cJSON *element = NULL;
    cJSON_ArrayForEach(element, elements) {
        if (result->count >= BC_STORE_RESULT_CAPACITY) break;
        const cJSON *tags = cJSON_GetObjectItemCaseSensitive(element, "tags");
        if (!cJSON_IsObject(tags)) continue;
        const char *shop = json_string(tags, "shop");
        if (shop == NULL ||
            (strcmp(shop, "alcohol") != 0 && strcmp(shop, "wine") != 0)) {
            continue;
        }
        double latitude;
        double longitude;
        if (!json_number(element, "lat", &latitude) ||
            !json_number(element, "lon", &longitude)) {
            const cJSON *center = cJSON_GetObjectItemCaseSensitive(element, "center");
            if (!cJSON_IsObject(center) ||
                !json_number(center, "lat", &latitude) ||
                !json_number(center, "lon", &longitude)) {
                continue;
            }
        }
        if (!bc_coordinates_valid(latitude, longitude)) continue;
        bc_store_t *store = &result->stores[result->count];
        memset(store, 0, sizeof(*store));
        store->latitude_deg = latitude;
        store->longitude_deg = longitude;
        store->observed_ms = observed;
        const cJSON *id = cJSON_GetObjectItemCaseSensitive(element, "id");
        if (cJSON_IsNumber(id) && id->valuedouble >= 0.0) {
            store->osm_id = (uint64_t)id->valuedouble;
        }
        const char *name = json_string(tags, "name");
        if (name == NULL || name[0] == '\0') name = json_string(tags, "brand");
        if (name == NULL || name[0] == '\0') name = "Unnamed mapped shop";
        strlcpy(store->name, name, sizeof(store->name));
        ++result->count;
    }
    cJSON_Delete(root);
    result->count = bc_store_deduplicate(result->stores, result->count, 25.0);
    return true;
}

static esp_err_t make_request(const search_request_t *request,
                              bc_store_search_result_t *result)
{
    if (!bc_wifi_manager_is_connected()) return ESP_ERR_INVALID_STATE;
    esp_err_t time_err = bc_wifi_manager_wait_for_time(15000);
    if (time_err != ESP_OK) {
        ESP_LOGW(TAG, "Waiting for network time before TLS: %s",
                 esp_err_to_name(time_err));
        return time_err;
    }
    char post[POST_LIMIT];
    if (!build_post_body(request, post, sizeof(post))) return ESP_ERR_INVALID_SIZE;

    response_buffer_t *response = calloc(1, sizeof(*response));
    if (response == NULL) return ESP_ERR_NO_MEM;
    esp_http_client_config_t config = {
        .url = CONFIG_BC_OVERPASS_URL,
        .method = HTTP_METHOD_POST,
        .timeout_ms = 20000,
        .buffer_size = 2048,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .event_handler = http_event,
        .user_data = response,
        .keep_alive_enable = false
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == NULL) {
        free(response);
        return ESP_ERR_NO_MEM;
    }
    esp_err_t err = esp_http_client_set_header(client, "Content-Type",
                                                "application/x-www-form-urlencoded");
    if (err == ESP_OK) {
        err = esp_http_client_set_header(client, "User-Agent",
                                         "BoozeCompass/1.0 (ESP-IDF embedded client)");
    }
    if (err == ESP_OK) {
        err = esp_http_client_set_post_field(client, post, (int)strlen(post));
    }
    if (err == ESP_OK) {
        err = esp_http_client_perform(client);
    }
    result->http_status = esp_http_client_get_status_code(client);
    if (err == ESP_OK && (result->http_status < 200 || result->http_status >= 300)) {
        err = result->http_status == 429 ? ESP_ERR_INVALID_STATE : ESP_FAIL;
    }
    if (response->overflow) err = ESP_ERR_INVALID_SIZE;
    if (err == ESP_OK && !parse_response(response->data, result)) {
        err = ESP_ERR_INVALID_RESPONSE;
    }
    esp_http_client_cleanup(client);
    free(response);
    return err;
}

static void search_task(void *argument)
{
    (void)argument;
    search_request_t request;
    for (;;) {
        if (xQueueReceive(s_request_queue, &request, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        bc_store_search_result_t result = {0};
        for (unsigned attempt = 0; attempt < HTTP_ATTEMPTS; ++attempt) {
            memset(&result, 0, sizeof(result));
            result.error = make_request(&request, &result);
            if (result.error == ESP_OK) break;
            ESP_LOGW(TAG, "Search attempt %u/%u failed (HTTP %d): %s",
                     attempt + 1, HTTP_ATTEMPTS, result.http_status,
                     esp_err_to_name(result.error));
            if (attempt + 1 < HTTP_ATTEMPTS) {
                TickType_t wait_ticks = pdMS_TO_TICKS(2000U << attempt);
                if (result.http_status == 429 || result.http_status == 406) {
                    wait_ticks = pdMS_TO_TICKS(30000);
                }
                vTaskDelay(wait_ticks);
                if (!bc_wifi_manager_is_connected()) {
                    result.error = ESP_ERR_INVALID_STATE;
                    break;
                }
            }
        }
        result.completed_ms = monotonic_ms();
        if (result.error == ESP_OK) {
            ESP_LOGI(TAG, "Search complete: %u unique mapped candidates",
                     (unsigned)result.count);
        }
        xQueueOverwrite(s_result_queue, &result);
    }
}

esp_err_t bc_store_finder_start(void)
{
    s_request_queue = xQueueCreate(1, sizeof(search_request_t));
    s_result_queue = xQueueCreate(1, sizeof(bc_store_search_result_t));
    if (s_request_queue == NULL || s_result_queue == NULL) {
        return ESP_ERR_NO_MEM;
    }
    if (xTaskCreate(search_task, "store_search", 6144, NULL, 4, NULL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

esp_err_t bc_store_finder_request(double latitude_deg, double longitude_deg,
                                  unsigned radius_m)
{
    if (s_request_queue == NULL ||
        !bc_coordinates_valid(latitude_deg, longitude_deg) ||
        radius_m < 250 || radius_m > 50000) {
        return ESP_ERR_INVALID_ARG;
    }
    const search_request_t request = {
        .latitude_deg = latitude_deg,
        .longitude_deg = longitude_deg,
        .radius_m = radius_m
    };
    return xQueueOverwrite(s_request_queue, &request) == pdTRUE
        ? ESP_OK : ESP_FAIL;
}

bool bc_store_finder_get_result(bc_store_search_result_t *result)
{
    return result != NULL && s_result_queue != NULL &&
           xQueueReceive(s_result_queue, result, 0) == pdTRUE;
}
