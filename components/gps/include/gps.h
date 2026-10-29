#pragma once

#include "navigation.h"
#include "esp_err.h"
#include "driver/uart.h"

esp_err_t bc_gps_init(uart_port_t port, int rx_gpio, int tx_gpio, int baud);
void bc_gps_poll(void);
void bc_gps_get_position(bc_position_t *position);
void bc_gps_reset_parser(void);
