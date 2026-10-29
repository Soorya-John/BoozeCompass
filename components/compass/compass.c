#include "compass.h"

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

#define QMC_REG_DATA 0x00
#define QMC_REG_STATUS 0x06
#define QMC_REG_CONTROL_1 0x09
#define QMC_REG_CONTROL_2 0x0A
#define QMC_REG_SET_RESET 0x0B
#define QMC_REG_CHIP_ID 0x0D
#define QMC_STATUS_DRDY 0x01
#define QMC_STATUS_OVL 0x02
#define PI_D 3.14159265358979323846

static i2c_master_dev_handle_t s_device;
static int16_t s_cal_min[3];
static int16_t s_cal_max[3];
static unsigned s_cal_samples;

static esp_err_t read_registers(uint8_t reg, uint8_t *data, size_t length)
{
    if (s_device == NULL || data == NULL || length == 0) {
        return ESP_ERR_INVALID_STATE;
    }
    return i2c_master_transmit_receive(s_device, &reg, 1, data, length, 100);
}

static esp_err_t write_register(uint8_t reg, uint8_t value)
{
    const uint8_t packet[2] = {reg, value};
    return i2c_master_transmit(s_device, packet, sizeof(packet), 100);
}

esp_err_t bc_compass_init(i2c_master_bus_handle_t bus, uint8_t address,
                          uint32_t clock_hz)
{
    if (bus == NULL || address == 0 || address > 0x7F ||
        clock_hz < 10000 || clock_hz > 400000) {
        return ESP_ERR_INVALID_ARG;
    }
    const i2c_device_config_t config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address,
        .scl_speed_hz = clock_hz
    };
    esp_err_t err = i2c_master_bus_add_device(bus, &config, &s_device);
    if (err != ESP_OK) {
        return err;
    }
    err = write_register(QMC_REG_CONTROL_2, 0x80);
    if (err != ESP_OK) {
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    err = write_register(QMC_REG_SET_RESET, 0x01);
    if (err != ESP_OK) {
        return err;
    }
    // OSR=512, range=8 G, ODR=50 Hz, continuous mode.
    return write_register(QMC_REG_CONTROL_1, 0x15);
}

esp_err_t bc_compass_recover(i2c_master_bus_handle_t bus, uint8_t address,
                             uint32_t clock_hz)
{
    if (s_device != NULL) {
        (void)i2c_master_bus_rm_device(s_device);
        s_device = NULL;
    }
    return bc_compass_init(bus, address, clock_hz);
}

esp_err_t bc_compass_read_raw(bc_mag_sample_t *sample)
{
    if (sample == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t status = 0;
    esp_err_t err = read_registers(QMC_REG_STATUS, &status, 1);
    if (err != ESP_OK) {
        sample->valid = false;
        return err;
    }
    if ((status & QMC_STATUS_DRDY) == 0) {
        sample->valid = false;
        return ESP_ERR_NOT_FOUND;
    }
    uint8_t data[6];
    err = read_registers(QMC_REG_DATA, data, sizeof(data));
    if (err != ESP_OK) {
        sample->valid = false;
        return err;
    }
    sample->x = (int16_t)((uint16_t)data[1] << 8 | data[0]);
    sample->y = (int16_t)((uint16_t)data[3] << 8 | data[2]);
    sample->z = (int16_t)((uint16_t)data[5] << 8 | data[4]);
    sample->observed_ms = esp_timer_get_time() / 1000;
    sample->overflow = (status & QMC_STATUS_OVL) != 0;
    sample->valid = !sample->overflow;
    return sample->valid ? ESP_OK : ESP_ERR_INVALID_RESPONSE;
}

double bc_compass_heading_deg(const bc_mag_sample_t *mag,
                              double accel_x_g, double accel_y_g,
                              double accel_z_g,
                              const bc_mag_calibration_t *calibration)
{
    if (mag == NULL || !mag->valid ||
        !isfinite(accel_x_g) || !isfinite(accel_y_g) || !isfinite(accel_z_g)) {
        return NAN;
    }
    const double offset_x = calibration ? calibration->offset_x : 0.0;
    const double offset_y = calibration ? calibration->offset_y : 0.0;
    const double offset_z = calibration ? calibration->offset_z : 0.0;
    const double scale_x = calibration ? calibration->scale_x : 1.0;
    const double scale_y = calibration ? calibration->scale_y : 1.0;
    const double scale_z = calibration ? calibration->scale_z : 1.0;
    const double yaw_offset = calibration ? calibration->yaw_offset_deg : 0.0;
    const double mx = (mag->x - offset_x) * scale_x;
    const double my = (mag->y - offset_y) * scale_y;
    const double mz = (mag->z - offset_z) * scale_z;
    const double accel_norm = sqrt(accel_x_g * accel_x_g +
                                   accel_y_g * accel_y_g +
                                   accel_z_g * accel_z_g);
    if (accel_norm < 0.5 || accel_norm > 1.5) {
        return NAN;
    }
    const double ax = accel_x_g / accel_norm;
    const double ay = accel_y_g / accel_norm;
    const double az = accel_z_g / accel_norm;
    const double roll = atan2(ay, az);
    const double pitch = atan2(-ax, sqrt(ay * ay + az * az));
    const double x_horizontal = mx * cos(pitch) + mz * sin(pitch);
    const double y_horizontal = mx * sin(roll) * sin(pitch) +
                                my * cos(roll) -
                                mz * sin(roll) * cos(pitch);
    double degrees = atan2(-y_horizontal, x_horizontal) * 180.0 / PI_D;
    degrees += yaw_offset;
    degrees = fmod(degrees, 360.0);
    return degrees < 0.0 ? degrees + 360.0 : degrees;
}

uint8_t bc_compass_read_chip_id(esp_err_t *result)
{
    uint8_t id = 0;
    esp_err_t err = read_registers(QMC_REG_CHIP_ID, &id, 1);
    if (result != NULL) {
        *result = err;
    }
    return id;
}

void bc_compass_calibration_begin(void)
{
    for (size_t i = 0; i < 3; ++i) {
        s_cal_min[i] = INT16_MAX;
        s_cal_max[i] = INT16_MIN;
    }
    s_cal_samples = 0;
}

void bc_compass_calibration_add(const bc_mag_sample_t *sample)
{
    if (sample == NULL || !sample->valid) return;
    const int16_t values[3] = {sample->x, sample->y, sample->z};
    for (size_t i = 0; i < 3; ++i) {
        if (values[i] < s_cal_min[i]) s_cal_min[i] = values[i];
        if (values[i] > s_cal_max[i]) s_cal_max[i] = values[i];
    }
    ++s_cal_samples;
}

esp_err_t bc_compass_calibration_finish(bc_mag_calibration_t *calibration)
{
    if (calibration == NULL || s_cal_samples < 100) return ESP_ERR_INVALID_STATE;
    double radius[3];
    double average_radius = 0.0;
    for (size_t i = 0; i < 3; ++i) {
        radius[i] = ((double)s_cal_max[i] - s_cal_min[i]) * 0.5;
        if (radius[i] < 100.0) return ESP_ERR_INVALID_RESPONSE;
        average_radius += radius[i] / 3.0;
    }
    calibration->offset_x = ((double)s_cal_max[0] + s_cal_min[0]) * 0.5;
    calibration->offset_y = ((double)s_cal_max[1] + s_cal_min[1]) * 0.5;
    calibration->offset_z = ((double)s_cal_max[2] + s_cal_min[2]) * 0.5;
    calibration->scale_x = average_radius / radius[0];
    calibration->scale_y = average_radius / radius[1];
    calibration->scale_z = average_radius / radius[2];
    calibration->calibrated = true;
    if (calibration->scale_x < 0.3 || calibration->scale_x > 3.0 ||
        calibration->scale_y < 0.3 || calibration->scale_y > 3.0 ||
        calibration->scale_z < 0.3 || calibration->scale_z > 3.0) {
        calibration->calibrated = false;
        return ESP_ERR_INVALID_RESPONSE;
    }
    return ESP_OK;
}
