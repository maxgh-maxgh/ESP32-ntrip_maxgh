#ifndef ESP32_XBEE_UART_H
#define ESP32_XBEE_UART_H

#include <esp_event.h>
#include <stdbool.h>
#include <stddef.h>

ESP_EVENT_DECLARE_BASE(UART_EVENT_READ);
ESP_EVENT_DECLARE_BASE(UART_EVENT_WRITE);

#define UART_BUFFER_SIZE 4096

void uart_init();

void uart_inject(void *data, size_t len);
int uart_log(char *buffer, size_t len);
int uart_nmea(const char *fmt, ...);
int uart_write(char *buffer, size_t len);

void uart_get_mode_response(bool *received, char *b, size_t b_size,
                            char *l, size_t l_size, char *h, size_t h_size);

void uart_register_read_handler(esp_event_handler_t event_handler);
void uart_register_write_handler(esp_event_handler_t event_handler);

#endif //ESP32_XBEE_UART_H
