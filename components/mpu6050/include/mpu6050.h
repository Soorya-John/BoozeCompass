#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    double accel_x_g;
    double accel_y_g;
    double accel_z_g;
    double gyro_x_dps;
    double gyro_y_dps;
    double gyro_z_dps;
    double tilt_roll_deg;
    double tilt_pitch_deg;
    int64_t observed_ms;
    bool valid;
} bc_imu_sample_t;

typedef struct {
    uint16_t sample_hz;
    uint8_t accel_range_g;
    uint16_t gyro_range_dps;
} bc_mpu6050_config_t;

esp_err_t bc_mpu6050_init(i2c_master_bus_handle_t bus, uint8_t address,
                          const bc_mpu6050_config_t *config,
                          uint32_t clock_hz);
esp_err_t bc_mpu6050_recover(i2c_master_bus_handle_t bus, uint8_t address,
                             const bc_mpu6050_config_t *config,
                             uint32_t clock_hz);
esp_err_t bc_mpu6050_read(bc_imu_sample_t *sample);
esp_err_t bc_mpu6050_calibrate_gyro(unsigned samples);
