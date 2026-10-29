#include "motor.h"

#include "motor_math.h"
#include "uln2003_backend.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <math.h>
#include <stdlib.h>

#define MOTOR_COMMAND_QUEUE_LENGTH 1

static const char *TAG = "motor";
static QueueHandle_t s_commands;
static portMUX_TYPE s_status_mux = portMUX_INITIALIZER_UNLOCKED;
static bc_motor_status_t s_status;
static long s_step_count;
static unsigned s_phase;
static bool s_hardware_enabled;
static bool s_dry_run = true;

static int motor_direction(void)
{
#ifdef CONFIG_BC_MOTOR_REVERSE
    return CONFIG_BC_MOTOR_REVERSE ? -1 : 1;
#else
    return 1;
#endif
}

typedef struct {
    double target_deg;
} motor_command_t;

static double current_angle(void)
{
    return bc_motor_steps_to_angle(s_step_count, CONFIG_BC_MOTOR_STEPS_PER_REV,
                                  motor_direction());
}

static void status_update(bool moving, double target, esp_err_t error)
{
    portENTER_CRITICAL(&s_status_mux);
    s_status.estimated_angle_deg = current_angle();
    s_status.target_angle_deg = target;
    s_status.moving = moving;
    s_status.last_error = error;
    portEXIT_CRITICAL(&s_status_mux);
}

static void set_error(esp_err_t error)
{
    portENTER_CRITICAL(&s_status_mux);
    s_status.last_error = error;
    s_status.moving = false;
    portEXIT_CRITICAL(&s_status_mux);
}

static uint32_t step_delay_ms(unsigned step_index, unsigned step_count)
{
    const double steps_per_second =
        (double)CONFIG_BC_MOTOR_SPEED_DPS *
        (double)CONFIG_BC_MOTOR_STEPS_PER_REV / 360.0;
    const double max_period_ms = 1000.0 / fmax(1.0, steps_per_second);
    const double ramp_steps = fmax(1.0,
        (double)CONFIG_BC_MOTOR_SPEED_DPS * CONFIG_BC_MOTOR_SPEED_DPS /
        (2.0 * CONFIG_BC_MOTOR_ACCEL_DPS2) *
        CONFIG_BC_MOTOR_STEPS_PER_REV / 360.0);
    const double ramp_fraction = fmin(1.0,
        fmin((step_index + 1.0) / ramp_steps,
             (step_count - step_index) / ramp_steps));
    const double shaped = fmax(0.2, sqrt(fmax(0.0, ramp_fraction)));
    return (uint32_t)fmax(2.0, max_period_ms / shaped);
}

static void motor_task(void *argument)
{
    (void)argument;
    motor_command_t command;
    for (;;) {
        if (xQueueReceive(s_commands, &command, portMAX_DELAY) != pdTRUE) continue;
        portENTER_CRITICAL(&s_status_mux);
        const bool known = s_status.position_known;
        const bool armed = s_status.armed;
        const double start_angle = current_angle();
        portEXIT_CRITICAL(&s_status_mux);
        if (!known || !armed || !s_hardware_enabled || s_dry_run) {
            set_error(ESP_ERR_INVALID_STATE);
            continue;
        }
        const long delta = bc_motor_angle_delta_steps(
            command.target_deg, start_angle,
            CONFIG_BC_MOTOR_STEPS_PER_REV,
            motor_direction());
        const unsigned steps = (unsigned)labs(delta);
        status_update(steps > 0, command.target_deg, ESP_OK);
        if (steps == 0) continue;
        const int direction = delta >= 0 ? 1 : -1;
        esp_err_t motion_error = ESP_OK;
        for (unsigned i = 0; i < steps; ++i) {
            portENTER_CRITICAL(&s_status_mux);
            const bool still_armed = s_status.armed;
            portEXIT_CRITICAL(&s_status_mux);
            if (!still_armed) {
                (void)bc_uln2003_disable();
                motion_error = ESP_ERR_INVALID_STATE;
                break;
            }
            s_phase = (unsigned)((int)s_phase + direction + 8) & 7U;
            esp_err_t err = bc_uln2003_apply_phase(s_phase);
            if (err != ESP_OK) {
                (void)bc_uln2003_disable();
                motion_error = err;
                break;
            }
            portENTER_CRITICAL(&s_status_mux);
            s_step_count += direction;
            portEXIT_CRITICAL(&s_status_mux);
            status_update(true, command.target_deg, ESP_OK);
            vTaskDelay(pdMS_TO_TICKS(step_delay_ms(i, steps)));
        }
        if (motion_error == ESP_OK) {
            status_update(false, command.target_deg, ESP_OK);
        } else {
            set_error(motion_error);
        }
    }
}

esp_err_t bc_motor_init(void)
{
    s_commands = xQueueCreate(MOTOR_COMMAND_QUEUE_LENGTH, sizeof(motor_command_t));
    if (s_commands == NULL) return ESP_ERR_NO_MEM;
#ifdef CONFIG_BC_MOTOR_ENABLE
    s_hardware_enabled = CONFIG_BC_MOTOR_ENABLE;
#else
    s_hardware_enabled = false;
#endif
    s_dry_run = CONFIG_BC_MOTOR_DRY_RUN || !s_hardware_enabled;
    s_status.dry_run = s_dry_run;
    s_status.last_error = ESP_OK;
    if (s_hardware_enabled) {
        esp_err_t err = bc_uln2003_init();
        if (err != ESP_OK) return err;
    }
    if (xTaskCreate(motor_task, "motor_stepper", 3072, NULL, 4, NULL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    if (s_status.dry_run) {
        ESP_LOGW(TAG, "Dry run enabled; motor outputs will not be energized");
    } else {
        ESP_LOGI(TAG, "ULN2003 backend initialized, but position is unknown");
    }
    return ESP_OK;
}

esp_err_t bc_motor_confirm_zero(void)
{
    if (!s_status.dry_run && !s_hardware_enabled) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    portENTER_CRITICAL(&s_status_mux);
    s_step_count = 0;
    s_status.estimated_angle_deg = 0.0;
    s_status.target_angle_deg = 0.0;
    s_status.position_known = true;
    s_status.moving = false;
    s_status.last_error = ESP_OK;
    portEXIT_CRITICAL(&s_status_mux);
    ESP_LOGW(TAG, "Logical zero accepted; physical pointer alignment is an operator assumption");
    return ESP_OK;
}

esp_err_t bc_motor_arm(void)
{
    portENTER_CRITICAL(&s_status_mux);
    const bool position_known = s_status.position_known;
    portEXIT_CRITICAL(&s_status_mux);
    if (!position_known) return ESP_ERR_INVALID_STATE;
    if (!s_dry_run && !s_hardware_enabled) return ESP_ERR_NOT_SUPPORTED;
    portENTER_CRITICAL(&s_status_mux);
    s_status.armed = true;
    portEXIT_CRITICAL(&s_status_mux);
    return ESP_OK;
}

void bc_motor_safe_disable(void)
{
    portENTER_CRITICAL(&s_status_mux);
    s_status.armed = false;
    const bool moving = s_status.moving;
    portEXIT_CRITICAL(&s_status_mux);
    if (s_hardware_enabled && !moving) {
        const esp_err_t err = bc_uln2003_disable();
        if (err != ESP_OK) set_error(err);
    }
}

esp_err_t bc_motor_move_to_angle(double target_angle_deg)
{
    if (!isfinite(target_angle_deg)) return ESP_ERR_INVALID_ARG;
    target_angle_deg = bc_motor_normalize_angle(target_angle_deg);
    portENTER_CRITICAL(&s_status_mux);
    const bool known = s_status.position_known;
    const bool armed = s_status.armed;
    portEXIT_CRITICAL(&s_status_mux);
    if (!known || !armed) return ESP_ERR_INVALID_STATE;
    if (s_dry_run || !s_hardware_enabled) {
        portENTER_CRITICAL(&s_status_mux);
        s_status.target_angle_deg = target_angle_deg;
        s_status.estimated_angle_deg = target_angle_deg;
        s_step_count = (long)llround(target_angle_deg *
            CONFIG_BC_MOTOR_STEPS_PER_REV / 360.0 *
            motor_direction());
        s_status.moving = false;
        s_status.last_error = ESP_OK;
        portEXIT_CRITICAL(&s_status_mux);
        ESP_LOGI(TAG, "DRY RUN pointer target %.1f degrees", target_angle_deg);
        return ESP_OK;
    }
    const motor_command_t command = {.target_deg = target_angle_deg};
    return xQueueOverwrite(s_commands, &command) == pdTRUE ? ESP_OK : ESP_FAIL;
}

void bc_motor_get_status(bc_motor_status_t *status)
{
    if (status == NULL) return;
    portENTER_CRITICAL(&s_status_mux);
    *status = s_status;
    status->estimated_angle_deg = current_angle();
    portEXIT_CRITICAL(&s_status_mux);
}
