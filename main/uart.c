/*
 * This file is part of the ESP32-XBee distribution (https://github.com/nebkat/esp32-xbee).
 * Copyright (c) 2019 Nebojsa Cvetkovic.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <driver/uart.h>
#include <driver/gpio.h>
#include <esp_event.h>
#include <esp_log.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <protocol/nmea.h>
#include <stream_stats.h>

#include "uart.h"
#include "config.h"
#include "tasks.h"

static const char *TAG = "UART";

ESP_EVENT_DEFINE_BASE(UART_EVENT_READ);
ESP_EVENT_DEFINE_BASE(UART_EVENT_WRITE);

void uart_register_read_handler(esp_event_handler_t event_handler) {
    ESP_ERROR_CHECK(esp_event_handler_register(UART_EVENT_READ, ESP_EVENT_ANY_ID, event_handler, NULL));
}

void uart_unregister_read_handler(esp_event_handler_t event_handler) {
    ESP_ERROR_CHECK(esp_event_handler_unregister(UART_EVENT_READ, ESP_EVENT_ANY_ID, event_handler));
}

void uart_register_write_handler(esp_event_handler_t event_handler) {
    ESP_ERROR_CHECK(esp_event_handler_register(UART_EVENT_WRITE, ESP_EVENT_ANY_ID, event_handler, NULL));
}

void uart_unregister_write_handler(esp_event_handler_t event_handler) {
    ESP_ERROR_CHECK(esp_event_handler_unregister(UART_EVENT_WRITE, ESP_EVENT_ANY_ID, event_handler));
}

static int uart_port = -1;
static bool uart_log_forward = false;

static stream_stats_handle_t stream_stats;

/* Last MODE response, parsed from incoming UART text. */
static portMUX_TYPE mode_response_mux = portMUX_INITIALIZER_UNLOCKED;
static bool mode_response_received = false;
static char mode_coordinate_b[32] = "";
static char mode_coordinate_l[32] = "";
static char mode_coordinate_h[32] = "";

static void mode_response_process_line(const char *line) {
    if (strstr(line, "#MODE") == NULL &&
        strstr(line, "MODE ROVER") == NULL &&
        strstr(line, "MODE BASE TIME") == NULL) {
        return;
    }

    char b[32] = "";
    char l[32] = "";
    char h[32] = "";
    const char *mode = strstr(line, "MODE BASE ");

    if (mode != NULL) {
        mode += strlen("MODE BASE ");
        char *end_b;
        char *end_l;
        char *end_h;
        double vb = strtod(mode, &end_b);
        double vl = strtod(end_b, &end_l);
        double vh = strtod(end_l, &end_h);

        bool valid = end_b != mode && end_l != end_b && end_h != end_l;
        if (valid) {
            while (*end_h == ' ' || *end_h == '\t') end_h++;
            valid = (*end_h == ',' || *end_h == '*' || *end_h == '\0' ||
                     *end_h == '\r' || *end_h == '\n');
        }
        if (valid) {
            snprintf(b, sizeof(b), "%.11f", vb);
            snprintf(l, sizeof(l), "%.11f", vl);
            snprintf(h, sizeof(h), "%.6f", vh);
        }
    }

    portENTER_CRITICAL(&mode_response_mux);
    mode_response_received = true;
    strlcpy(mode_coordinate_b, b, sizeof(mode_coordinate_b));
    strlcpy(mode_coordinate_l, l, sizeof(mode_coordinate_l));
    strlcpy(mode_coordinate_h, h, sizeof(mode_coordinate_h));
    portEXIT_CRITICAL(&mode_response_mux);
}

void uart_get_mode_response(bool *received, char *b, size_t b_size,
                            char *l, size_t l_size, char *h, size_t h_size) {
    portENTER_CRITICAL(&mode_response_mux);
    if (received) *received = mode_response_received;
    if (b && b_size) strlcpy(b, mode_coordinate_b, b_size);
    if (l && l_size) strlcpy(l, mode_coordinate_l, l_size);
    if (h && h_size) strlcpy(h, mode_coordinate_h, h_size);
    portEXIT_CRITICAL(&mode_response_mux);
}

static void uart_task(void *ctx);

void uart_init() {
    uart_log_forward = config_get_bool1(CONF_ITEM(KEY_CONFIG_UART_LOG_FORWARD));

    uart_port = config_get_u8(CONF_ITEM(KEY_CONFIG_UART_NUM));

    uart_hw_flowcontrol_t flow_ctrl;
    bool flow_ctrl_rts = config_get_bool1(CONF_ITEM(KEY_CONFIG_UART_FLOW_CTRL_RTS));
    bool flow_ctrl_cts = config_get_bool1(CONF_ITEM(KEY_CONFIG_UART_FLOW_CTRL_CTS));
    if (flow_ctrl_rts && flow_ctrl_cts) {
        flow_ctrl = UART_HW_FLOWCTRL_CTS_RTS;
    } else if (flow_ctrl_rts) {
        flow_ctrl = UART_HW_FLOWCTRL_RTS;
    } else if (flow_ctrl_cts) {
        flow_ctrl = UART_HW_FLOWCTRL_CTS;
    } else {
        flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    }

    uart_config_t uart_config = {
            .baud_rate = config_get_u32(CONF_ITEM(KEY_CONFIG_UART_BAUD_RATE)),
            .data_bits = config_get_i8(CONF_ITEM(KEY_CONFIG_UART_DATA_BITS)),
            .parity = config_get_i8(CONF_ITEM(KEY_CONFIG_UART_PARITY)),
            .stop_bits = config_get_i8(CONF_ITEM(KEY_CONFIG_UART_STOP_BITS)),
            .flow_ctrl = flow_ctrl
    };
    ESP_ERROR_CHECK(uart_param_config(uart_port, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(
            uart_port,
            config_get_u8(CONF_ITEM(KEY_CONFIG_UART_TX_PIN)),
            config_get_u8(CONF_ITEM(KEY_CONFIG_UART_RX_PIN)),
            config_get_u8(CONF_ITEM(KEY_CONFIG_UART_RTS_PIN)),
            config_get_u8(CONF_ITEM(KEY_CONFIG_UART_CTS_PIN))
    ));
    ESP_ERROR_CHECK(uart_driver_install(uart_port, UART_BUFFER_SIZE, UART_BUFFER_SIZE, 0, NULL, 0));

    stream_stats = stream_stats_new("uart");

    xTaskCreate(uart_task, "uart_task", 8192, NULL, TASK_PRIORITY_UART, NULL);
}

static void uart_task(void *ctx) {
    uint8_t buffer[UART_BUFFER_SIZE];
    static char response_line[512];
    static size_t response_line_len = 0;

    while (true) {
        int32_t len = uart_read_bytes(uart_port, buffer, sizeof(buffer), pdMS_TO_TICKS(50));
        if (len < 0) {
            ESP_LOGE(TAG, "Error reading from UART");
            continue;
        } else if (len == 0) {
            continue;
        }

        /* Parse complete UART lines independently of read chunk boundaries. */
        for (int32_t i = 0; i < len; i++) {
            char ch = (char) buffer[i];
            if (ch == '\n') {
                response_line[response_line_len] = '\0';
                mode_response_process_line(response_line);
                response_line_len = 0;
            } else if (response_line_len < sizeof(response_line) - 1) {
                response_line[response_line_len++] = ch;
            } else {
                response_line_len = 0;
            }
        }

        stream_stats_increment(stream_stats, len, 0);
        esp_event_post(UART_EVENT_READ, len, &buffer, len, portMAX_DELAY);
    }
}

void uart_inject(void *buf, size_t len) {
    esp_event_post(UART_EVENT_READ, len,  buf, len, portMAX_DELAY);
}

int uart_log(char *buf, size_t len) {
    if (!uart_log_forward) return 0;
    return uart_write(buf, len);
}

int uart_nmea(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);

    char *nmea;
    nmea_vasprintf(&nmea, fmt, args);
    int l = uart_write(nmea, strlen(nmea));
    free(nmea);

    va_end(args);

    return l;
}

int uart_write(char *buf, size_t len) {
    if (uart_port < 0) return 0;
    if (len == 0) return 0;

    int written = uart_write_bytes(uart_port, buf, len);
    if (written < 0) return written;

    stream_stats_increment(stream_stats, 0, len);

    esp_event_post(UART_EVENT_WRITE, len, buf, len, portMAX_DELAY);

    return written;
}
