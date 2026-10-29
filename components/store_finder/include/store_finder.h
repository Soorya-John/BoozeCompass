#pragma once

#include "esp_err.h"
#include "navigation.h"
#include <stdbool.h>
#include <stddef.h>

#define BC_STORE_RESULT_CAPACITY 32

typedef struct {
    bc_store_t stores[BC_STORE_RESULT_CAPACITY];
    size_t count;
    int http_status;
    esp_err_t error;
    int64_t completed_ms;
} bc_store_search_result_t;

esp_err_t bc_store_finder_start(void);
esp_err_t bc_store_finder_request(double latitude_deg, double longitude_deg,
                                  unsigned radius_m);
bool bc_store_finder_get_result(bc_store_search_result_t *result);
