// micro-ROS over the Type-C serial port, to the agent on the Pi.
#include "freertos/FreeRTOS.h"
#include "driver/uart.h"
#include "config.h"
#include "transport.h"

#define ROS_UART UART_NUM_0
#define ROS_TX 43
#define ROS_RX 44

bool transport_open(struct uxrCustomTransport *t)
{
    uart_config_t cfg = {
        .baud_rate = ROS_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    return uart_param_config(ROS_UART, &cfg) == ESP_OK
        && uart_set_pin(ROS_UART, ROS_TX, ROS_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) == ESP_OK
        && uart_driver_install(ROS_UART, 4096, 4096, 0, NULL, 0) == ESP_OK;
}

bool transport_close(struct uxrCustomTransport *t)
{
    return uart_driver_delete(ROS_UART) == ESP_OK;
}

size_t transport_write(struct uxrCustomTransport *t, const uint8_t *buf, size_t len, uint8_t *err)
{
    int n = uart_write_bytes(ROS_UART, (const char *)buf, len);
    return n < 0 ? 0 : (size_t)n;
}

size_t transport_read(struct uxrCustomTransport *t, uint8_t *buf, size_t len, int timeout, uint8_t *err)
{
    int n = uart_read_bytes(ROS_UART, buf, len, pdMS_TO_TICKS(timeout));
    return n < 0 ? 0 : (size_t)n;
}
