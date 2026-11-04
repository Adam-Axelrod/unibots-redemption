#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "lidar.h"

#define LIDAR_UART UART_NUM_1
#define LIDAR_TX 17
#define LIDAR_RX 18
#define LIDAR_BAUD 230400

static portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
static uint16_t distances[MS200_BINS];

static void lidar_task(void *arg)
{
    static ms200_parser_t parser;
    uint8_t bytes[128];
    while (true) {
        int n = uart_read_bytes(LIDAR_UART, bytes, sizeof(bytes), pdMS_TO_TICKS(20));
        for (int i = 0; i < n; i++) {
            if (ms200_feed(&parser, bytes[i])) {
                taskENTER_CRITICAL(&lock);
                ms200_store(parser.buf, distances);
                taskEXIT_CRITICAL(&lock);
            }
        }
    }
}

void lidar_init(void)
{
    uart_config_t cfg = {
        .baud_rate = LIDAR_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(LIDAR_UART, 2048, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(LIDAR_UART, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(LIDAR_UART, LIDAR_TX, LIDAR_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    xTaskCreatePinnedToCore(lidar_task, "lidar", 4096, NULL, 8, NULL, 0);
}

void lidar_copy_ranges(float ranges_m[MS200_BINS])
{
    static uint16_t snapshot[MS200_BINS];
    taskENTER_CRITICAL(&lock);
    memcpy(snapshot, distances, sizeof(snapshot));
    taskEXIT_CRITICAL(&lock);
    for (int i = 0; i < MS200_BINS; i++) ranges_m[i] = snapshot[i] / 1000.0f;
}
