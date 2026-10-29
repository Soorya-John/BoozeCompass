#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
    int64_t observed_ms;
    bool valid;
    bool overflow;
} bc_mag_sample_t;

typedef struct {
    double offset_x;
    double offset_y;
    double offset_z;
    double scale_x;
    double scale_y;
    double scale_z;
    double yaw_offset_deg;
    bool calibrated;
} bc_mag_calibration_t;

esp_err_t bc_compass_init(i2c_master_bus_handle_t bus, uint8_t address,
                          uint32_t clock_hz);
esp_err_t bc_compass_recover(i2c_master_bus_handle_t bus, uint8_t address,
                             uint32_t clock_hz);
esp_err_t bc_compass_read_raw(bc_mag_sample_t *sample);
double bc_compass_heading_deg(const bc_mag_sample_t *mag,
                              double accel_x_g, double accel_y_g,
                              double accel_z_g,
                              const bc_mag_calibration_t *calibration);
uint8_t bc_compass_read_chip_id(esp_err_t *result);
void bc_compass_calibration_begin(void);
void bc_compass_calibration_add(const bc_mag_sample_t *sample);
esp_err_t bc_compass_calibration_finish(bc_mag_calibration_t *calibration);
