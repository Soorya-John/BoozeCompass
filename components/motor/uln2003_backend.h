#pragma once

#include "esp_err.h"

esp_err_t bc_uln2003_init(void);
esp_err_t bc_uln2003_apply_phase(unsigned phase);
esp_err_t bc_uln2003_disable(void);
