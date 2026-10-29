#pragma once

#include "esp_err.h"
#include <stdbool.h>

typedef struct {
    double estimated_angle_deg;
    double target_angle_deg;
    bool position_known;
    bool armed;
    bool moving;
    bool dry_run;
    esp_err_t last_error;
} bc_motor_status_t;

esp_err_t bc_motor_init(void);
esp_err_t bc_motor_confirm_zero(void);
esp_err_t bc_motor_arm(void);
void bc_motor_safe_disable(void);
esp_err_t bc_motor_move_to_angle(double target_angle_deg);
void bc_motor_get_status(bc_motor_status_t *status);
