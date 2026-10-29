#include "mpu6050.h"

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <string.h>

#define MPU_REG_SMPLRT_DIV 0x19
#define MPU_REG_CONFIG 0x1A
#define MPU_REG_GYRO_CONFIG 0x1B
#define MPU_REG_ACCEL_CONFIG 0x1C
#define MPU_REG_ACCEL_XOUT_H 0x3B
#define MPU_REG_PWR_MGMT_1 0x6B
#define MPU_REG_WHO_AM_I 0x75

static i2c_master_dev_handle_t s_device;
static double s_gyro_bias[3];
static double s_filter[6];
static bool s_filter_initialized;
static double s_accel_lsb_per_g = 16384.0;
static double s_gyro_lsb_per_dps = 131.0;

static esp_err_t write_register(uint8_t reg, uint8_t value)
{
    const uint8_t packet[2] = {reg, value};
    return i2c_master_transmit(s_device, packet, sizeof(packet), 100);
}

static esp_err_t read_registers(uint8_t reg, uint8_t *data, size_t length)
{
    return i2c_master_transmit_receive(s_device, &reg, 1, data, length, 100);
}

static int16_t signed_word(const uint8_t *bytes)
{
    return (int16_t)((uint16_t)bytes[0] << 8 | bytes[1]);
}

esp_err_t bc_mpu6050_init(i2c_master_bus_handle_t bus, uint8_t address,
                          const bc_mpu6050_config_t *config, uint32_t clock_hz)
{
    if (bus == NULL || config == NULL || config->sample_hz == 0 ||
        config->sample_hz > 1000 || clock_hz < 10000 || clock_hz > 400000) {
        return ESP_ERR_INVALID_ARG;
    }
    const i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address,
        .scl_speed_hz = clock_hz
    };
    esp_err_t err = i2c_master_bus_add_device(bus, &device_config, &s_device);
    if (err != ESP_OK) {
        return err;
    }
    uint8_t id = 0;
    err = read_registers(MPU_REG_WHO_AM_I, &id, 1);
    if (err != ESP_OK) {
        return err;
    }
    if ((id & 0x7E) != 0x68) {
        return ESP_ERR_NOT_FOUND;
    }

    err = write_register(MPU_REG_PWR_MGMT_1, 0x01);
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(100));
    const uint8_t divider = (uint8_t)(1000U / config->sample_hz - 1U);
    err = write_register(MPU_REG_CONFIG, 0x03);
    if (err != ESP_OK) return err;
    err = write_register(MPU_REG_SMPLRT_DIV, divider);
    if (err != ESP_OK) return err;

    uint8_t gyro_bits;
    switch (config->gyro_range_dps) {
    case 250: gyro_bits = 0; s_gyro_lsb_per_dps = 131.0; break;
    case 500: gyro_bits = 1; s_gyro_lsb_per_dps = 65.5; break;
    case 1000: gyro_bits = 2; s_gyro_lsb_per_dps = 32.8; break;
    case 2000: gyro_bits = 3; s_gyro_lsb_per_dps = 16.4; break;
    default: return ESP_ERR_INVALID_ARG;
    }
    uint8_t accel_bits;
    switch (config->accel_range_g) {
    case 2: accel_bits = 0; s_accel_lsb_per_g = 16384.0; break;
    case 4: accel_bits = 1; s_accel_lsb_per_g = 8192.0; break;
    case 8: accel_bits = 2; s_accel_lsb_per_g = 4096.0; break;
    case 16: accel_bits = 3; s_accel_lsb_per_g = 2048.0; break;
    default: return ESP_ERR_INVALID_ARG;
    }
    err = write_register(MPU_REG_GYRO_CONFIG, (uint8_t)(gyro_bits << 3));
    if (err != ESP_OK) return err;
    err = write_register(MPU_REG_ACCEL_CONFIG, (uint8_t)(accel_bits << 3));
    memset(s_gyro_bias, 0, sizeof(s_gyro_bias));
    memset(s_filter, 0, sizeof(s_filter));
    s_filter_initialized = false;
    return err;
}

esp_err_t bc_mpu6050_recover(i2c_master_bus_handle_t bus, uint8_t address,
                             const bc_mpu6050_config_t *config,
                             uint32_t clock_hz)
{
    if (s_device != NULL) {
        (void)i2c_master_bus_rm_device(s_device);
        s_device = NULL;
    }
    return bc_mpu6050_init(bus, address, config, clock_hz);
}

esp_err_t bc_mpu6050_read(bc_imu_sample_t *sample)
{
    if (sample == NULL || s_device == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    uint8_t data[14];
    esp_err_t err = read_registers(MPU_REG_ACCEL_XOUT_H, data, sizeof(data));
    if (err != ESP_OK) {
        sample->valid = false;
        return err;
    }
    const double raw[6] = {
        signed_word(&data[0]) / s_accel_lsb_per_g,
        signed_word(&data[2]) / s_accel_lsb_per_g,
        signed_word(&data[4]) / s_accel_lsb_per_g,
        signed_word(&data[8]) / s_gyro_lsb_per_dps - s_gyro_bias[0],
        signed_word(&data[10]) / s_gyro_lsb_per_dps - s_gyro_bias[1],
        signed_word(&data[12]) / s_gyro_lsb_per_dps - s_gyro_bias[2]
    };
    double filtered[6];
    for (size_t i = 0; i < 6; ++i) {
        if (!s_filter_initialized) {
            s_filter[i] = raw[i];
        } else {
            s_filter[i] += 0.25 * (raw[i] - s_filter[i]);
        }
        filtered[i] = s_filter[i];
    }
    s_filter_initialized = true;
    sample->accel_x_g = filtered[0];
    sample->accel_y_g = filtered[1];
    sample->accel_z_g = filtered[2];
    sample->gyro_x_dps = filtered[3];
    sample->gyro_y_dps = filtered[4];
    sample->gyro_z_dps = filtered[5];
    sample->tilt_roll_deg = atan2(filtered[1],
        sqrt(filtered[0] * filtered[0] + filtered[2] * filtered[2])) *
        180.0 / 3.14159265358979323846;
    sample->tilt_pitch_deg = atan2(-filtered[0],
        sqrt(filtered[1] * filtered[1] + filtered[2] * filtered[2])) *
        180.0 / 3.14159265358979323846;
    sample->observed_ms = esp_timer_get_time() / 1000;
    sample->valid = true;
    return ESP_OK;
}

esp_err_t bc_mpu6050_calibrate_gyro(unsigned samples)
{
    if (samples < 32 || samples > 2000 || s_device == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    int64_t sums[3] = {0, 0, 0};
    uint8_t data[14];
    for (unsigned i = 0; i < samples; ++i) {
        esp_err_t err = read_registers(MPU_REG_ACCEL_XOUT_H, data, sizeof(data));
        if (err != ESP_OK) {
            return err;
        }
        sums[0] += signed_word(&data[8]);
        sums[1] += signed_word(&data[10]);
        sums[2] += signed_word(&data[12]);
        vTaskDelay(pdMS_TO_TICKS(4));
    }
    for (size_t i = 0; i < 3; ++i) {
        s_gyro_bias[i] = (double)sums[i] / samples / s_gyro_lsb_per_dps;
    }
    s_filter_initialized = false;
    return ESP_OK;
}
