#include "uln2003_backend.h"

#include "driver/gpio.h"

static const gpio_num_t s_pins[4] = {
    CONFIG_BC_MOTOR_IN1_GPIO,
    CONFIG_BC_MOTOR_IN2_GPIO,
    CONFIG_BC_MOTOR_IN3_GPIO,
    CONFIG_BC_MOTOR_IN4_GPIO
};

// Eight half-step phases for a 5-wire unipolar motor through a ULN2003.
static const uint8_t s_sequence[8] = {
    0x01, 0x03, 0x02, 0x06, 0x04, 0x0C, 0x08, 0x09
};

esp_err_t bc_uln2003_init(void)
{
    uint64_t mask = 0;
    for (unsigned i = 0; i < 4; ++i) {
        if (s_pins[i] < 0 || s_pins[i] >= GPIO_NUM_MAX) {
            return ESP_ERR_INVALID_ARG;
        }
        for (unsigned j = 0; j < i; ++j) {
            if (s_pins[i] == s_pins[j]) return ESP_ERR_INVALID_ARG;
        }
        mask |= 1ULL << s_pins[i];
    }
    const gpio_config_t config = {
        .pin_bit_mask = mask,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    esp_err_t err = gpio_config(&config);
    if (err != ESP_OK) return err;
    return bc_uln2003_disable();
}

esp_err_t bc_uln2003_apply_phase(unsigned phase)
{
    const uint8_t state = s_sequence[phase & 7U];
    for (unsigned i = 0; i < 4; ++i) {
        esp_err_t err = gpio_set_level(s_pins[i], (state >> i) & 1U);
        if (err != ESP_OK) return err;
    }
    return ESP_OK;
}

esp_err_t bc_uln2003_disable(void)
{
    for (unsigned i = 0; i < 4; ++i) {
        esp_err_t err = gpio_set_level(s_pins[i], 0);
        if (err != ESP_OK) return err;
    }
    return ESP_OK;
}
