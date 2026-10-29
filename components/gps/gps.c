#include "gps.h"

#include "esp_timer.h"
#include "esp_log.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define GPS_RX_BUFFER_SIZE 128
#define UBX_MAX_PAYLOAD 100

static const char *TAG = "gps";
static uart_port_t s_port;
static bc_position_t s_position;
static char s_nmea[GPS_RX_BUFFER_SIZE];
static size_t s_nmea_len;
static struct {
    uint8_t state;
    uint8_t message_class;
    uint8_t message_id;
    uint16_t payload_length;
    uint16_t payload_index;
    uint8_t payload[UBX_MAX_PAYLOAD];
    uint8_t checksum_a;
    uint8_t checksum_b;
    uint8_t received_a;
} s_ubx;

static int64_t monotonic_ms(void)
{
    return esp_timer_get_time() / 1000;
}

static bool parse_coordinate(const char *value, const char *hemisphere,
                             bool latitude, double *coordinate)
{
    if (value == NULL || hemisphere == NULL || coordinate == NULL ||
        value[0] == '\0' || hemisphere[0] == '\0') {
        return false;
    }
    char *end = NULL;
    const double degrees_minutes = strtod(value, &end);
    if (end == value || !isfinite(degrees_minutes)) {
        return false;
    }
    const double degrees = floor(degrees_minutes / 100.0);
    const double minutes = degrees_minutes - degrees * 100.0;
    if (minutes < 0.0 || minutes >= 60.0) {
        return false;
    }
    double decimal = degrees + minutes / 60.0;
    if ((latitude && (degrees > 90.0 || (degrees == 90.0 && minutes > 0.0))) ||
        (!latitude && (degrees > 180.0 || (degrees == 180.0 && minutes > 0.0)))) {
        return false;
    }
    if ((latitude && hemisphere[0] == 'S') ||
        (!latitude && hemisphere[0] == 'W')) {
        decimal = -decimal;
    } else if (!((latitude && hemisphere[0] == 'N') ||
                 (!latitude && hemisphere[0] == 'E'))) {
        return false;
    }
    *coordinate = decimal;
    return true;
}

static bool verify_nmea_checksum(char *sentence)
{
    if (sentence == NULL || sentence[0] != '$') {
        return false;
    }
    char *star = strchr(sentence, '*');
    if (star == NULL || star[1] == '\0' || star[2] == '\0') {
        return false;
    }
    char *end = NULL;
    const long expected = strtol(star + 1, &end, 16);
    if (end == star + 1 || expected < 0 || expected > 255) {
        return false;
    }
    uint8_t checksum = 0;
    for (char *p = sentence + 1; p < star; ++p) {
        checksum ^= (uint8_t)*p;
    }
    *star = '\0';
    return checksum == (uint8_t)expected;
}

static size_t split_fields(char *sentence, char **fields, size_t capacity)
{
    size_t count = 0;
    if (capacity == 0) {
        return 0;
    }
    fields[count++] = sentence;
    for (char *p = sentence; *p != '\0' && count < capacity; ++p) {
        if (*p == ',') {
            *p = '\0';
            fields[count++] = p + 1;
        }
    }
    return count;
}

static void parse_nmea(char *sentence)
{
    if (!verify_nmea_checksum(sentence)) {
        return;
    }
    char *fields[20] = {0};
    const size_t count = split_fields(sentence, fields, 20);
    if (count < 10 || strlen(fields[0]) < 6) {
        return;
    }
    const int64_t now = monotonic_ms();
    if (strcmp(fields[0] + 3, "GGA") == 0) {
        double latitude;
        double longitude;
        char *end = NULL;
        const long quality = strtol(fields[6], &end, 10);
        if (end == fields[6]) {
            return;
        }
        const long satellites = strtol(fields[7], NULL, 10);
        if (quality <= 0 ||
            !parse_coordinate(fields[2], fields[3], true, &latitude) ||
            !parse_coordinate(fields[4], fields[5], false, &longitude)) {
            s_position.valid = false;
            s_position.observed_ms = now;
            s_position.fix_quality = quality > 0 ? (unsigned)quality : 0;
            return;
        }
        s_position.latitude_deg = latitude;
        s_position.longitude_deg = longitude;
        s_position.fix_quality = (unsigned)quality;
        s_position.satellites = satellites > 0 ? (unsigned)satellites : 0;
        s_position.altitude_m = strtod(fields[9], NULL);
        s_position.observed_ms = now;
        s_position.valid = true;
        return;
    }
    if (strcmp(fields[0] + 3, "RMC") == 0 && count >= 9) {
        if (fields[2][0] != 'A') {
            s_position.valid = false;
            s_position.observed_ms = now;
            return;
        }
        double latitude;
        double longitude;
        if (!parse_coordinate(fields[3], fields[4], true, &latitude) ||
            !parse_coordinate(fields[5], fields[6], false, &longitude)) {
            s_position.valid = false;
            s_position.observed_ms = now;
            return;
        }
        s_position.latitude_deg = latitude;
        s_position.longitude_deg = longitude;
        s_position.speed_mps = strtod(fields[7], NULL) * 0.514444444;
        s_position.observed_ms = now;
        s_position.valid = true;
        if (s_position.fix_quality == 0) {
            s_position.fix_quality = 1;
        }
    }
}

static uint32_t ubx_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static int32_t ubx_i32(const uint8_t *p)
{
    return (int32_t)ubx_u32(p);
}

static void parse_ubx_packet(void)
{
    if (s_ubx.message_class != 0x01 || s_ubx.message_id != 0x07 ||
        s_ubx.payload_length < 92) {
        return;
    }
    const uint8_t *p = s_ubx.payload;
    const uint8_t fix_type = p[20];
    const bool fix_ok = (p[21] & 0x01) != 0;
    s_position.satellites = p[23];
    s_position.fix_quality = fix_type;
    s_position.longitude_deg = ubx_i32(&p[24]) * 1e-7;
    s_position.latitude_deg = ubx_i32(&p[28]) * 1e-7;
    s_position.altitude_m = ubx_i32(&p[36]) / 1000.0;
    s_position.speed_mps = ubx_i32(&p[60]) / 1000.0;
    s_position.observed_ms = monotonic_ms();
    s_position.valid = fix_ok && fix_type >= 2 &&
        bc_coordinates_valid(s_position.latitude_deg, s_position.longitude_deg);
}

static void ubx_feed(uint8_t byte)
{
    switch (s_ubx.state) {
    case 0: s_ubx.state = byte == 0xB5 ? 1 : 0; break;
    case 1: s_ubx.state = byte == 0x62 ? 2 : (byte == 0xB5 ? 1 : 0); break;
    case 2:
        s_ubx.message_class = byte;
        s_ubx.checksum_a = byte;
        s_ubx.checksum_b = byte;
        s_ubx.state = 3;
        break;
    case 3:
        s_ubx.message_id = byte;
        s_ubx.checksum_a += byte;
        s_ubx.checksum_b += s_ubx.checksum_a;
        s_ubx.state = 4;
        break;
    case 4:
        s_ubx.payload_length = byte;
        s_ubx.checksum_a += byte;
        s_ubx.checksum_b += s_ubx.checksum_a;
        s_ubx.state = 5;
        break;
    case 5:
        s_ubx.payload_length |= (uint16_t)byte << 8;
        s_ubx.checksum_a += byte;
        s_ubx.checksum_b += s_ubx.checksum_a;
        s_ubx.payload_index = 0;
        if (s_ubx.payload_length > UBX_MAX_PAYLOAD) {
            s_ubx.state = 0;
        } else {
            s_ubx.state = s_ubx.payload_length == 0 ? 7 : 6;
        }
        break;
    case 6:
        s_ubx.payload[s_ubx.payload_index++] = byte;
        s_ubx.checksum_a += byte;
        s_ubx.checksum_b += s_ubx.checksum_a;
        if (s_ubx.payload_index == s_ubx.payload_length) s_ubx.state = 7;
        break;
    case 7: s_ubx.received_a = byte; s_ubx.state = 8; break;
    case 8:
        if (s_ubx.received_a == s_ubx.checksum_a &&
            byte == s_ubx.checksum_b) {
            parse_ubx_packet();
        }
        s_ubx.state = 0;
        break;
    default: s_ubx.state = 0; break;
    }
}

static void feed_byte(uint8_t byte)
{
    ubx_feed(byte);
    if (byte == '$') {
        s_nmea_len = 0;
        s_nmea[s_nmea_len++] = '$';
        return;
    }
    if (s_nmea_len == 0) {
        return;
    }
    if (byte == '\n' || byte == '\r') {
        if (s_nmea_len > 6) {
            s_nmea[s_nmea_len] = '\0';
            parse_nmea(s_nmea);
        }
        s_nmea_len = 0;
        return;
    }
    if (s_nmea_len + 1 < sizeof(s_nmea)) {
        s_nmea[s_nmea_len++] = (char)byte;
    } else {
        s_nmea_len = 0;
    }
}

esp_err_t bc_gps_init(uart_port_t port, int rx_gpio, int tx_gpio, int baud)
{
    if (baud < 9600 || baud > 460800) {
        return ESP_ERR_INVALID_ARG;
    }
    s_port = port;
    const uart_config_t config = {
        .baud_rate = baud,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };
    esp_err_t err = uart_param_config(port, &config);
    if (err != ESP_OK) return err;
    err = uart_set_pin(port, tx_gpio, rx_gpio,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) return err;
    err = uart_driver_install(port, 4096, 0, 0, NULL, 0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;
    memset(&s_position, 0, sizeof(s_position));
    memset(&s_ubx, 0, sizeof(s_ubx));
    s_nmea_len = 0;
    ESP_LOGI(TAG, "UART%d configured at %d baud; parsing NMEA and UBX NAV-PVT",
             (int)port, baud);
    return ESP_OK;
}

void bc_gps_poll(void)
{
    uint8_t input[256];
    int length;
    do {
        length = uart_read_bytes(s_port, input, sizeof(input), 0);
        for (int i = 0; i < length; ++i) {
            feed_byte(input[i]);
        }
    } while (length == (int)sizeof(input));
}

void bc_gps_get_position(bc_position_t *position)
{
    if (position != NULL) {
        *position = s_position;
    }
}

void bc_gps_reset_parser(void)
{
    memset(&s_ubx, 0, sizeof(s_ubx));
    s_nmea_len = 0;
    uart_flush_input(s_port);
}
