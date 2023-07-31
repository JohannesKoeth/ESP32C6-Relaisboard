#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "comm_uart.h"
#include "defines.h"

static void echo_send(const int port, const char *str, uint8_t length)
{
    if (uart_write_bytes(port, str, length) != length)
    {
        ESP_LOGE(TAG, "Send data critical failure.");
        // add your code to handle sending failure here
        abort();
    }
}

void uart_init()
{
    uart_config_t uart_config = {
        .baud_rate = 19200, // Set your desired baud rate
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_RTS,
    };
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, BUFFERSIZE * 2, 0, 0, NULL, 0));                                        // Install UART driver
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));                                                         // Configure UART parameters
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, ECHO_TEST_RTS, UART_PIN_NO_CHANGE)); // Set UART pins as per KConfig settings
    // Optionally, you can set UART RX buffer size (in this case, we use the default size)
    // ESP_ERROR_CHECK(uart_set_rx_buffer_size(UART_NUM, BUFFERSIZE));
}

void uart_receive_task(void *arg)
{
    uint8_t data[BUFFERSIZE];
    while (1)
    {
        // Read data from UART
        int len = uart_read_bytes(UART_NUM, data, BUFFERSIZE, 100 / PACKET_READ_TICS); // Timeout of 100ms

        if (len > 0)
        {
            // Process received data here
            // Example: Print received data
            data[len] = '\0'; // Null-terminate the data for printing
            printf("Received: %s\n", data);
        }
    }
}

