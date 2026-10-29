#include "compass.h"
#include "configuration.h"
#include "gps.h"
#include "motor.h"
#include "mpu6050.h"
#include "navigation.h"
#include "store_finder.h"
#include "wifi_manager.h"

#include "driver/i2c_master.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_idf_version.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include <math.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define LOOP_PERIOD_MS 20
#define IMU_SAMPLE_PERIOD_MS 50
#define MAG_SAMPLE_PERIOD_MS 50
#define DIAGNOSTIC_PERIOD_MS 2000
#define SENSOR_RECOVERY_PERIOD_MS 30000
#define TARGET_FRESH_MULTIPLIER 3
#define GPS_UART UART_NUM_1
#define MPU6050_ADDRESS CONFIG_BC_MPU6050_I2C_ADDRESS

static const char *TAG = "boozecompass";

typedef enum {
    STATE_BOOT,
    STATE_HARDWARE_INIT,
    STATE_SENSOR_DIAGNOSTICS,
    STATE_WAITING_FOR_GPS,
    STATE_CONNECTING_WIFI,
    STATE_SEARCHING_FOR_TARGET,
    STATE_CALIBRATING_HEADING,
    STATE_TRACKING_TARGET,
    STATE_DEGRADED_MODE,
    STATE_ERROR_RECOVERY
} system_state_t;

typedef enum {
    CMD_STATUS,
    CMD_I2C_SCAN,
    CMD_GPS,
    CMD_COMPASS,
    CMD_IMU,
    CMD_SEARCH,
    CMD_MOTOR_ZERO,
    CMD_MOTOR_ARM,
    CMD_MOTOR_DISABLE,
    CMD_MOTOR_TEST,
    CMD_GYRO_CALIBRATE,
    CMD_MAG_CALIBRATE_START,
    CMD_MAG_CALIBRATE_FINISH,
    CMD_HELP
} command_kind_t;

typedef struct {
    command_kind_t kind;
    double value;
} console_command_t;

static i2c_master_bus_handle_t s_i2c_bus;
static QueueHandle_t s_console_queue;
static system_state_t s_state = STATE_BOOT;
static bc_position_t s_position;
static bc_imu_sample_t s_imu;
static bc_mag_sample_t s_mag;
static bc_heading_t s_heading;
static bc_mag_calibration_t s_mag_calibration;
static bc_store_t s_stores[BC_STORE_RESULT_CAPACITY];
static size_t s_store_count;
static bc_store_t s_target;
static bool s_has_target;
static bool s_mag_calibration_active;
static bool s_compass_ready;
static bool s_imu_ready;
static bool s_gps_ready;
static bool s_motor_ready;
static bool s_wifi_ready;
static bool s_store_finder_ready;
static bool s_search_pending;
static bool s_force_search;
static bool s_search_origin_valid;
static double s_search_origin_lat;
static double s_search_origin_lon;
static bool s_filtered_heading_valid;
static double s_filtered_heading;
static bool s_last_motor_command_valid;
static double s_last_motor_command;
static unsigned s_compass_errors;
static unsigned s_imu_errors;
static int64_t s_next_search_ms;
static int64_t s_last_api_request_ms;
static int64_t s_next_recovery_ms;
static int64_t s_next_imu_ms;
static int64_t s_next_mag_ms;
static int64_t s_next_diagnostics_ms;
static bc_mpu6050_config_t s_imu_config = {
    .sample_hz = 50,
    .accel_range_g = 2,
    .gyro_range_dps = 250
};
static bc_mag_calibration_t s_mag_calibration_candidate;

static int64_t now_ms(void)
{
    return esp_timer_get_time() / 1000;
}

static const char *state_name(system_state_t state)
{
    static const char *names[] = {
        "BOOT", "HARDWARE_INIT", "SENSOR_DIAGNOSTICS", "WAITING_FOR_GPS",
        "CONNECTING_WIFI", "SEARCHING_FOR_TARGET", "CALIBRATING_HEADING",
        "TRACKING_TARGET", "DEGRADED_MODE", "ERROR_RECOVERY"
    };
    return state <= STATE_ERROR_RECOVERY ? names[state] : "INVALID";
}

static bool measurement_fresh(int64_t observed_ms, int64_t max_age_ms)
{
    const int64_t now = now_ms();
    return observed_ms >= 0 && observed_ms <= now &&
           now - observed_ms <= max_age_ms;
}

static bool gps_is_fresh(void)
{
    return s_position.valid &&
           bc_coordinates_valid(s_position.latitude_deg, s_position.longitude_deg) &&
           measurement_fresh(s_position.observed_ms, CONFIG_BC_GPS_MAX_AGE_MS);
}

static bool heading_is_fresh(void)
{
    return s_heading.valid &&
           measurement_fresh(s_heading.observed_ms, CONFIG_BC_HEADING_MAX_AGE_MS);
}

static int64_t target_max_age_ms(void)
{
    return (int64_t)CONFIG_BC_SEARCH_REFRESH_S * 1000 *
           TARGET_FRESH_MULTIPLIER;
}

static bool target_is_fresh(void)
{
    return s_has_target && measurement_fresh(s_target.observed_ms,
                                               target_max_age_ms());
}

static void set_state(system_state_t next)
{
    if (next != s_state) {
        ESP_LOGI(TAG, "STATE %s -> %s", state_name(s_state), state_name(next));
        s_state = next;
    }
}

static void print_status(void)
{
    const int64_t now = now_ms();
    bc_motor_status_t motor = {0};
    if (s_motor_ready) bc_motor_get_status(&motor);
    printf("BoozeCompass 1.0 | ESP-IDF %s | state=%s\n",
           esp_get_idf_version(), state_name(s_state));
    printf("GPS: %s", gps_is_fresh() ? "FIX" : (s_position.valid ? "STALE" : "NO FIX"));
    if (s_position.observed_ms > 0) {
        printf(" age=%" PRId64 "ms lat=%.6f lon=%.6f sats=%u quality=%u",
               now - s_position.observed_ms, s_position.latitude_deg,
               s_position.longitude_deg, s_position.satellites,
               s_position.fix_quality);
    }
    printf("\n");
    printf("Compass: %s heading=%.1f deg magnetic calibration=%s",
           heading_is_fresh() ? "VALID" : "INVALID",
           s_heading.magnetic_heading_deg,
           s_heading.calibrated ? "valid" : "not calibrated");
    if (s_heading.observed_ms > 0) {
        printf(" age=%" PRId64 "ms", now - s_heading.observed_ms);
    }
    printf(" raw=(%d,%d,%d)\n", s_mag.x, s_mag.y, s_mag.z);
    printf("MPU6050: %s accel=(%.2f,%.2f,%.2f)g gyro=(%.1f,%.1f,%.1f)dps "
           "tilt=(%.1f,%.1f)deg\n",
           s_imu.valid && measurement_fresh(s_imu.observed_ms, 1000)
               ? "VALID" : "INVALID",
           s_imu.accel_x_g, s_imu.accel_y_g, s_imu.accel_z_g,
           s_imu.gyro_x_dps, s_imu.gyro_y_dps, s_imu.gyro_z_dps,
           s_imu.tilt_roll_deg, s_imu.tilt_pitch_deg);
    printf("Wi-Fi: %s | search: %s\n",
           bc_wifi_manager_is_connected() ? "connected" : "disconnected",
           s_search_pending ? "in progress" : (target_is_fresh() ? "target cached" : "due/unknown"));
    if (s_has_target) {
        printf("Target: %s | %s | osm=%" PRIu64,
               s_target.name,
               target_is_fresh() ? "CURRENT RESULT" : "STALE CACHE",
               s_target.osm_id);
        if (gps_is_fresh()) {
            printf(" | distance=%.0fm", bc_haversine_distance_m(
                s_position.latitude_deg, s_position.longitude_deg,
                s_target.latitude_deg, s_target.longitude_deg));
        }
        printf("\n");
    } else {
        printf("Target: unknown (no mapped result)\n");
    }
    printf("Motor: %s position=%.1f target=%.1f known=%s armed=%s moving=%s "
           "dry_run=%s error=%s\n",
           s_motor_ready ? "ULN2003 backend" : "unavailable",
           motor.estimated_angle_deg, motor.target_angle_deg,
           motor.position_known ? "yes" : "no",
           motor.armed ? "yes" : "no", motor.moving ? "yes" : "no",
           motor.dry_run ? "yes" : "no", esp_err_to_name(motor.last_error));
}

static void print_i2c_scan(void)
{
    if (s_i2c_bus == NULL) {
        printf("I2C bus is not initialized\n");
        return;
    }
    unsigned found = 0;
    printf("I2C 7-bit scan: ");
    for (uint8_t address = 0x08; address <= 0x77; ++address) {
        if (i2c_master_probe(s_i2c_bus, address, 20) == ESP_OK) {
            printf("0x%02X ", address);
            ++found;
        }
    }
    if (found == 0) printf("no devices");
    printf("\n");
}

static void print_gps(void)
{
    if (s_position.observed_ms == 0) {
        printf("GPS has not produced a complete valid sentence/packet\n");
        return;
    }
    printf("GPS valid=%s age=%" PRId64 "ms lat=%.7f lon=%.7f quality=%u "
           "satellites=%u altitude=%.1fm speed=%.2fm/s\n",
           gps_is_fresh() ? "yes" : "no", now_ms() - s_position.observed_ms,
           s_position.latitude_deg, s_position.longitude_deg,
           s_position.fix_quality, s_position.satellites,
           s_position.altitude_m, s_position.speed_mps);
}

static void print_compass(void)
{
    printf("Compass read=%s address=0x%02X raw=(%d,%d,%d) heading=%.2f deg "
           "calibrated=%s age=%" PRId64 "ms\n",
           s_mag.valid ? "valid" : "invalid", CONFIG_BC_MAG_I2C_ADDRESS,
           s_mag.x, s_mag.y, s_mag.z, s_heading.magnetic_heading_deg,
           s_mag_calibration.calibrated ? "yes" : "no",
           s_heading.observed_ms > 0 ? now_ms() - s_heading.observed_ms : -1);
}

static void print_imu(void)
{
    printf("MPU6050 valid=%s accel=(%.4f,%.4f,%.4f)g gyro=(%.3f,%.3f,%.3f)dps "
           "tilt=(%.2f,%.2f)deg\n",
           s_imu.valid ? "yes" : "no",
           s_imu.accel_x_g, s_imu.accel_y_g, s_imu.accel_z_g,
           s_imu.gyro_x_dps, s_imu.gyro_y_dps, s_imu.gyro_z_dps,
           s_imu.tilt_roll_deg, s_imu.tilt_pitch_deg);
}

static void service_search(void)
{
    bc_store_search_result_t result;
    if (bc_store_finder_get_result(&result)) {
        s_search_pending = false;
        if (result.error != ESP_OK) {
            ESP_LOGW(TAG, "Store search failed (HTTP %d): %s; keeping cache only until stale",
                     result.http_status, esp_err_to_name(result.error));
            s_next_search_ms = now_ms() + 60000;
        } else if (result.count == 0) {
            s_has_target = false;
            s_store_count = 0;
            ESP_LOGW(TAG, "No mapped alcohol/wine shops found within configured radius");
            s_next_search_ms = now_ms() +
                (int64_t)CONFIG_BC_SEARCH_REFRESH_S * 1000;
        } else {
            memcpy(s_stores, result.stores, result.count * sizeof(result.stores[0]));
            s_store_count = result.count;
            s_next_search_ms = now_ms() +
                (int64_t)CONFIG_BC_SEARCH_REFRESH_S * 1000;
            ESP_LOGI(TAG, "Cached %u mapped candidates from OpenStreetMap",
                     (unsigned)s_store_count);
        }
    }

    if (s_store_count > 0 && gps_is_fresh()) {
        bc_store_t nearest;
        if (bc_store_select_nearest(s_stores, s_store_count,
                                    s_position.latitude_deg,
                                    s_position.longitude_deg, &nearest, NULL)) {
            s_target = nearest;
            s_has_target = true;
        }
    }

    bool moved_out_of_area = false;
    if (gps_is_fresh() && s_search_origin_valid) {
        moved_out_of_area = bc_haversine_distance_m(
            s_position.latitude_deg, s_position.longitude_deg,
            s_search_origin_lat, s_search_origin_lon) >
            CONFIG_BC_SEARCH_RADIUS_M * 0.5;
    }
    const int64_t now = now_ms();
    const int64_t minimum_interval_ms = 900000;
    const bool search_due = s_force_search || moved_out_of_area ||
                            now >= s_next_search_ms;
    const bool rate_limit_passed = s_last_api_request_ms == 0 ||
        now - s_last_api_request_ms >= minimum_interval_ms;
    if (!s_search_pending && s_store_finder_ready && search_due &&
        rate_limit_passed && gps_is_fresh() && bc_wifi_manager_is_connected()) {
        esp_err_t err = bc_store_finder_request(
            s_position.latitude_deg, s_position.longitude_deg,
            CONFIG_BC_SEARCH_RADIUS_M);
        if (err == ESP_OK) {
            s_search_pending = true;
            s_last_api_request_ms = now;
            s_search_origin_lat = s_position.latitude_deg;
            s_search_origin_lon = s_position.longitude_deg;
            s_search_origin_valid = true;
            s_force_search = false;
            ESP_LOGI(TAG, "Searching within %d m of latest GPS fix",
                     CONFIG_BC_SEARCH_RADIUS_M);
        } else {
            s_next_search_ms = now_ms() + minimum_interval_ms;
            ESP_LOGW(TAG, "Could not queue store search: %s", esp_err_to_name(err));
        }
    }
}

static void update_target_and_motor(void)
{
    if (!s_has_target) return;
    bc_navigation_solution_t solution;
    const bc_nav_status_t nav_status = bc_navigation_solve(
        &s_position, &s_heading, &s_target, now_ms(),
        CONFIG_BC_GPS_MAX_AGE_MS, CONFIG_BC_HEADING_MAX_AGE_MS,
        target_max_age_ms(), CONFIG_BC_MAG_DECLINATION_DEG,
        CONFIG_BC_POINTER_ZERO_OFFSET_DEG, &solution);
    if (nav_status != BC_NAV_OK || !s_heading.calibrated) {
        if (nav_status == BC_NAV_STALE_TARGET) {
            ESP_LOGW(TAG, "Cached target is stale; pointer updates are inhibited");
        }
        return;
    }
    static bool filtered_valid;
    static double filtered_bearing;
    if (!filtered_valid) {
        filtered_bearing = solution.relative_pointer_deg;
        filtered_valid = true;
    } else {
        const double delta = bc_signed_angle_difference(
            solution.relative_pointer_deg, filtered_bearing);
        filtered_bearing = bc_normalize_360(filtered_bearing + 0.25 * delta);
    }
    const double demand = filtered_bearing;
    if (!s_motor_ready) return;
    bc_motor_status_t status;
    bc_motor_get_status(&status);
    if (!status.position_known || !status.armed) return;
    const double current_demand = status.moving
        ? status.target_angle_deg : status.estimated_angle_deg;
    if (fabs(bc_signed_angle_difference(demand, current_demand)) <
        CONFIG_BC_POINTER_DEADBAND_DEG) {
        return;
    }
    if (s_last_motor_command_valid &&
        fabs(bc_signed_angle_difference(demand, s_last_motor_command)) <
            CONFIG_BC_POINTER_DEADBAND_DEG) {
        return;
    }
    const esp_err_t err = bc_motor_move_to_angle(demand);
    if (err == ESP_OK) {
        s_last_motor_command = demand;
        s_last_motor_command_valid = true;
        ESP_LOGI(TAG, "Navigation: %.0f m, true bearing %.1f deg, pointer %.1f deg",
                 solution.distance_m, solution.true_bearing_deg, demand);
    } else {
        ESP_LOGW(TAG, "Motor command rejected: %s", esp_err_to_name(err));
    }
}

static system_state_t derive_state(void)
{
    if (s_i2c_bus == NULL || !s_gps_ready || !s_wifi_ready ||
        !s_store_finder_ready) return STATE_ERROR_RECOVERY;
    if (!s_compass_ready || !s_imu_ready || !s_mag.valid ||
        !s_imu.valid || !heading_is_fresh()) return STATE_DEGRADED_MODE;
    if (!s_mag_calibration.calibrated) return STATE_CALIBRATING_HEADING;
    if (!gps_is_fresh()) return STATE_WAITING_FOR_GPS;
    if (!bc_wifi_manager_is_connected()) return STATE_CONNECTING_WIFI;
    if (s_search_pending || !s_has_target) return STATE_SEARCHING_FOR_TARGET;
    if (!target_is_fresh()) return STATE_DEGRADED_MODE;
    return STATE_TRACKING_TARGET;
}

static void attempt_sensor_recovery(void)
{
    const int64_t now = now_ms();
    if (now < s_next_recovery_ms) return;
    s_next_recovery_ms = now + SENSOR_RECOVERY_PERIOD_MS;
    if (s_i2c_bus == NULL) return;
    if (!s_compass_ready || s_compass_errors >= 5) {
        esp_err_t err = bc_compass_recover(s_i2c_bus,
            CONFIG_BC_MAG_I2C_ADDRESS, CONFIG_BC_I2C_FREQUENCY_HZ);
        s_compass_ready = err == ESP_OK;
        s_compass_errors = 0;
        ESP_LOGW(TAG, "Compass recovery: %s", esp_err_to_name(err));
    }
    if (!s_imu_ready || s_imu_errors >= 5) {
        esp_err_t err = bc_mpu6050_recover(s_i2c_bus, MPU6050_ADDRESS,
            &s_imu_config, CONFIG_BC_I2C_FREQUENCY_HZ);
        s_imu_ready = err == ESP_OK;
        s_imu_errors = 0;
        ESP_LOGW(TAG, "MPU6050 recovery: %s", esp_err_to_name(err));
    }
}

static void run_command(const console_command_t *command)
{
    esp_err_t err = ESP_OK;
    switch (command->kind) {
    case CMD_STATUS: print_status(); break;
    case CMD_I2C_SCAN: print_i2c_scan(); break;
    case CMD_GPS: print_gps(); break;
    case CMD_COMPASS: print_compass(); break;
    case CMD_IMU: print_imu(); break;
    case CMD_SEARCH:
        s_force_search = true;
        printf("Fresh search queued for the next valid GPS/Wi-Fi window\n");
        break;
    case CMD_MOTOR_ZERO:
        printf("Confirm physical pointer is aligned with the device forward axis first\n");
        err = bc_motor_confirm_zero();
        break;
    case CMD_MOTOR_ARM:
        err = bc_motor_arm();
        break;
    case CMD_MOTOR_DISABLE:
        bc_motor_safe_disable();
        printf("Motor disarmed\n");
        break;
    case CMD_MOTOR_TEST:
        if (command->value < -360.0 || command->value > 360.0) {
            err = ESP_ERR_INVALID_ARG;
        } else {
            err = bc_motor_move_to_angle(command->value);
        }
        break;
    case CMD_GYRO_CALIBRATE:
        if (!s_imu_ready) {
            err = ESP_ERR_INVALID_STATE;
        } else {
            printf("Keep the device motionless while gyro bias is sampled\n");
            err = bc_mpu6050_calibrate_gyro(128);
        }
        break;
    case CMD_MAG_CALIBRATE_START:
        if (!s_compass_ready) {
            err = ESP_ERR_INVALID_STATE;
        } else {
            bc_compass_calibration_begin();
            s_mag_calibration_active = true;
            printf("Rotate the device through full turns and varied tilt for 30-60 seconds\n");
        }
        break;
    case CMD_MAG_CALIBRATE_FINISH:
        if (!s_mag_calibration_active) {
            err = ESP_ERR_INVALID_STATE;
        } else {
            s_mag_calibration_active = false;
            s_mag_calibration_candidate = s_mag_calibration;
            err = bc_compass_calibration_finish(&s_mag_calibration_candidate);
            if (err == ESP_OK) {
                bc_saved_mag_calibration_t saved = {
                    .offset_x = s_mag_calibration_candidate.offset_x,
                    .offset_y = s_mag_calibration_candidate.offset_y,
                    .offset_z = s_mag_calibration_candidate.offset_z,
                    .scale_x = s_mag_calibration_candidate.scale_x,
                    .scale_y = s_mag_calibration_candidate.scale_y,
                    .scale_z = s_mag_calibration_candidate.scale_z,
                    .yaw_offset_deg = s_mag_calibration_candidate.yaw_offset_deg,
                    .calibrated = true
                };
                err = bc_configuration_save_mag(&saved);
                if (err == ESP_OK) s_mag_calibration = s_mag_calibration_candidate;
            }
        }
        break;
    case CMD_HELP:
        printf("Commands: status, scan_i2c, gps, compass, imu, search, "
               "motor_zero confirm, motor_arm, motor_disable, motor_test <deg>, "
               "calibrate_gyro, calibrate_mag start|finish, help\n");
        break;
    }
    if (err != ESP_OK) {
        printf("Command failed: %s\n", esp_err_to_name(err));
    }
}

static void console_task(void *argument)
{
    (void)argument;
    char line[96];
    printf("\nBoozeCompass serial commands; type 'help'\n> ");
    for (;;) {
        if (fgets(line, sizeof(line), stdin) == NULL) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        line[strcspn(line, "\r\n")] = '\0';
        console_command_t command = {0};
        bool valid = true;
        if (strcmp(line, "status") == 0) command.kind = CMD_STATUS;
        else if (strcmp(line, "scan_i2c") == 0) command.kind = CMD_I2C_SCAN;
        else if (strcmp(line, "gps") == 0) command.kind = CMD_GPS;
        else if (strcmp(line, "compass") == 0) command.kind = CMD_COMPASS;
        else if (strcmp(line, "imu") == 0) command.kind = CMD_IMU;
        else if (strcmp(line, "search") == 0) command.kind = CMD_SEARCH;
        else if (strcmp(line, "motor_zero confirm") == 0) command.kind = CMD_MOTOR_ZERO;
        else if (strcmp(line, "motor_arm") == 0) command.kind = CMD_MOTOR_ARM;
        else if (strcmp(line, "motor_disable") == 0) command.kind = CMD_MOTOR_DISABLE;
        else if (strcmp(line, "calibrate_gyro") == 0) command.kind = CMD_GYRO_CALIBRATE;
        else if (strcmp(line, "calibrate_mag start") == 0) command.kind = CMD_MAG_CALIBRATE_START;
        else if (strcmp(line, "calibrate_mag finish") == 0) command.kind = CMD_MAG_CALIBRATE_FINISH;
        else if (strcmp(line, "help") == 0) command.kind = CMD_HELP;
        else if (strncmp(line, "motor_test ", 11) == 0) {
            char *end = NULL;
            command.value = strtod(line + 11, &end);
            command.kind = CMD_MOTOR_TEST;
            valid = end != line + 11 && *end == '\0' && isfinite(command.value);
        } else {
            valid = false;
        }
        if (valid) {
            if (xQueueSend(s_console_queue, &command, 0) != pdTRUE) {
                printf("Command queue full; try again\n");
            }
        } else {
            printf("Unknown or invalid command; type 'help'\n");
        }
        printf("> ");
    }
}

static void initialize_nvs(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}

void app_main(void)
{
    s_state = STATE_BOOT;
    ESP_LOGI(TAG, "BoozeCompass 1.0, ESP-IDF %s, target ESP32-C3",
             esp_get_idf_version());
    set_state(STATE_HARDWARE_INIT);
    initialize_nvs();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    bc_runtime_config_t config;
    esp_err_t config_err = bc_configuration_load(&config);
    if (config_err != ESP_OK) {
        ESP_LOGW(TAG, "Configuration load: %s", esp_err_to_name(config_err));
    }
    s_mag_calibration = (bc_mag_calibration_t) {
        .offset_x = CONFIG_BC_MAG_OFFSET_X,
        .offset_y = CONFIG_BC_MAG_OFFSET_Y,
        .offset_z = CONFIG_BC_MAG_OFFSET_Z,
        .scale_x = CONFIG_BC_MAG_SCALE_X_MILLI / 1000.0,
        .scale_y = CONFIG_BC_MAG_SCALE_Y_MILLI / 1000.0,
        .scale_z = CONFIG_BC_MAG_SCALE_Z_MILLI / 1000.0,
        .yaw_offset_deg = CONFIG_BC_SENSOR_YAW_OFFSET_DEG,
        .calibrated = false
    };
    bc_saved_mag_calibration_t saved_mag = {0};
    if (bc_configuration_load_mag(&saved_mag) == ESP_OK && saved_mag.calibrated) {
        s_mag_calibration = (bc_mag_calibration_t) {
            .offset_x = saved_mag.offset_x, .offset_y = saved_mag.offset_y,
            .offset_z = saved_mag.offset_z, .scale_x = saved_mag.scale_x,
            .scale_y = saved_mag.scale_y, .scale_z = saved_mag.scale_z,
            .yaw_offset_deg = saved_mag.yaw_offset_deg, .calibrated = true
        };
        ESP_LOGI(TAG, "Magnetometer calibration restored from NVS");
    }

    const i2c_master_bus_config_t i2c_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = CONFIG_BC_I2C_SDA_GPIO,
        .scl_io_num = CONFIG_BC_I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = false
    };
    esp_err_t err = i2c_new_master_bus(&i2c_config, &s_i2c_bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C initialization failed: %s", esp_err_to_name(err));
        s_i2c_bus = NULL;
    }
    if (s_i2c_bus != NULL) {
        print_i2c_scan();
        err = bc_compass_init(s_i2c_bus, CONFIG_BC_MAG_I2C_ADDRESS,
                              CONFIG_BC_I2C_FREQUENCY_HZ);
        s_compass_ready = err == ESP_OK;
        ESP_LOGI(TAG, "QMC5883 compass init %s: %s (address 0x%02X)",
                 s_compass_ready ? "complete" : "failed",
                 esp_err_to_name(err), CONFIG_BC_MAG_I2C_ADDRESS);
        if (s_compass_ready) {
            esp_err_t id_err = ESP_OK;
            const uint8_t id = bc_compass_read_chip_id(&id_err);
            ESP_LOGI(TAG, "Compass ID register response 0x%02X (%s); this value is not unique",
                     id, esp_err_to_name(id_err));
        }
        err = bc_mpu6050_init(s_i2c_bus, MPU6050_ADDRESS, &s_imu_config,
                              CONFIG_BC_I2C_FREQUENCY_HZ);
        s_imu_ready = err == ESP_OK;
        ESP_LOGI(TAG, "MPU6050 init %s: %s",
                 s_imu_ready ? "complete" : "failed", esp_err_to_name(err));
    }

    err = bc_gps_init(GPS_UART, CONFIG_BC_GNSS_UART_RX_GPIO,
                      CONFIG_BC_GNSS_UART_TX_GPIO, CONFIG_BC_GNSS_UART_BAUD);
    s_gps_ready = err == ESP_OK;
    ESP_LOGI(TAG, "GNSS UART init %s: %s",
             s_gps_ready ? "complete" : "failed", esp_err_to_name(err));
    err = bc_motor_init();
    s_motor_ready = err == ESP_OK;
    ESP_LOGI(TAG, "Motor backend init %s: %s",
             s_motor_ready ? "complete" : "failed", esp_err_to_name(err));
    err = bc_wifi_manager_init(&config);
    s_wifi_ready = err == ESP_OK;
    if (err != ESP_OK) ESP_LOGE(TAG, "Wi-Fi manager init: %s", esp_err_to_name(err));
    err = bc_store_finder_start();
    s_store_finder_ready = err == ESP_OK;
    if (err != ESP_OK) ESP_LOGE(TAG, "Store finder init: %s", esp_err_to_name(err));

    s_console_queue = xQueueCreate(8, sizeof(console_command_t));
    if (s_console_queue == NULL) {
        ESP_LOGE(TAG, "Console command queue allocation failed");
    } else {
        xTaskCreate(console_task, "serial_console", 4096, NULL, 3, NULL);
    }

    set_state(STATE_SENSOR_DIAGNOSTICS);
    print_i2c_scan();
    ESP_LOGI(TAG, "Boot complete. Motor outputs remain dry-run/disabled until explicitly configured.");

    for (;;) {
        const int64_t now = now_ms();
        if (s_gps_ready) {
            bc_gps_poll();
            bc_gps_get_position(&s_position);
        }
        if (s_imu_ready && now >= s_next_imu_ms) {
            s_next_imu_ms = now + IMU_SAMPLE_PERIOD_MS;
            err = bc_mpu6050_read(&s_imu);
            if (err == ESP_OK) s_imu_errors = 0;
            else ++s_imu_errors;
        }
        if (s_compass_ready && now >= s_next_mag_ms) {
            s_next_mag_ms = now + MAG_SAMPLE_PERIOD_MS;
            err = bc_compass_read_raw(&s_mag);
            if (err == ESP_OK) {
                s_compass_errors = 0;
                if (s_mag_calibration_active) {
                    bc_compass_calibration_add(&s_mag);
                }
                if (s_imu.valid && measurement_fresh(s_imu.observed_ms, 500)) {
                    const double raw_heading = bc_compass_heading_deg(
                        &s_mag, s_imu.accel_x_g, s_imu.accel_y_g,
                        s_imu.accel_z_g, &s_mag_calibration);
                    if (isfinite(raw_heading)) {
                        if (!s_filtered_heading_valid) {
                            s_filtered_heading = raw_heading;
                            s_filtered_heading_valid = true;
                        } else {
                            const double delta = bc_signed_angle_difference(
                                raw_heading, s_filtered_heading);
                            s_filtered_heading = bc_normalize_360(
                                s_filtered_heading + 0.35 * delta);
                        }
                        s_heading.magnetic_heading_deg = s_filtered_heading;
                        s_heading.observed_ms = s_mag.observed_ms;
                        s_heading.valid = true;
                        s_heading.calibrated = s_mag_calibration.calibrated;
                    } else {
                        s_heading.valid = false;
                    }
                } else {
                    s_heading.valid = false;
                }
            } else {
                ++s_compass_errors;
                s_heading.valid = false;
            }
        }

        console_command_t command;
        while (s_console_queue != NULL &&
               xQueueReceive(s_console_queue, &command, 0) == pdTRUE) {
            run_command(&command);
        }
        service_search();
        attempt_sensor_recovery();
        const system_state_t next = derive_state();
        set_state(next);
        if (next == STATE_TRACKING_TARGET) update_target_and_motor();
        if (now >= s_next_diagnostics_ms) {
            s_next_diagnostics_ms = now + DIAGNOSTIC_PERIOD_MS;
            if (s_has_target && !target_is_fresh()) {
                ESP_LOGW(TAG, "Cached target is stale; it is not used for pointer movement");
            } else {
                ESP_LOGI(TAG, "state=%s GPS=%s compass=%s Wi-Fi=%s target=%s",
                         state_name(s_state), gps_is_fresh() ? "fresh" : "invalid",
                         heading_is_fresh() ? "fresh" : "invalid",
                         bc_wifi_manager_is_connected() ? "connected" : "disconnected",
                         target_is_fresh() ? s_target.name : "unknown/stale");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(LOOP_PERIOD_MS));
    }
}
