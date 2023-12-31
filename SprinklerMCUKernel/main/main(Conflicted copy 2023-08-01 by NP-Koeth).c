#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"
#include "lvgl.h"
#include "esp_lvgl_port.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/timers.h"

#include "esp_lcd_panel_vendor.h"
#include "defines.h"

// extern void example_lvgl_demo_ui(lv_disp_t *disp);

/* The LVGL port component calls esp_lcd_panel_draw_bitmap API for send data to the screen. There must be called
lvgl_port_flush_ready(disp) after each transaction to display. The best way is to use on_color_trans_done
callback from esp_lcd IO config structure. In IDF 5.1 and higher, it is solved inside LVGL port component. */
static bool notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    lv_disp_t *disp = (lv_disp_t *)user_ctx;
    lvgl_port_flush_ready(disp);
    return false;
}

static void echo_send(const int port, const char *str, uint8_t length)
{
    if (uart_write_bytes(port, str, length) != length)
    {
        ESP_LOGE(TAG, "Send data critical failure.");
        // add your code to handle sending failure here
        abort();
    }
}

void parse_string(const char *input, int *num1, int *num2, int *num3)
{
    // Find the last occurrence of '/'
    const char *last_slash = strrchr(input, '/');
    if (last_slash == NULL)
    {
        fprintf(stderr, "Invalid input format: No slashes found.\n");
        return;
    }

    // Convert the third number after the last '/'
    *num3 = atoi(last_slash + 1);

    // Find the second last occurrence of '/'
    const char *second_last_slash = strrchr(input, last_slash[-1]);
    if (second_last_slash == NULL)
    {
        fprintf(stderr, "Invalid input format: Less than three numbers found.\n");
        return;
    }

    // Convert the second number after the second last '/'
    *num2 = atoi(second_last_slash + 1);

    // Find the third last occurrence of '/'
    const char *third_last_slash = strrchr(input, second_last_slash[-1]);
    if (third_last_slash == NULL)
    {
        fprintf(stderr, "Invalid input format: Less than three numbers found.\n");
        return;
    }

    // Convert the first number after the third last '/'
    *num1 = atoi(third_last_slash + 1);
}

// Subroutine to print text using LVGL
void print_text(const char *text, int pos_x, int pos_y)
{
    lv_obj_t *label = lv_label_create(lv_scr_act());     // Create a new label
    lv_label_set_text(label, text);                      // Set the text of the label to the input string
    lv_obj_align(label, LV_ALIGN_DEFAULT, pos_x, pos_y); // Align the label to the center of the screen
    lv_scr_load(lv_scr_act());                           // Refresh the screen
}

void set_gpio_state(int gpio_num, bool state)
{
    #define TAG "set_gpio_state"
    ESP_LOGI(TAG, "Switching Relais %d to %d", gpio_num, state);

    gpio_set_level(gpio_num, state);
}

#define LINEINC 14

// An example of echo test with hardware flow control on UART
static void echo_task(void *arg)
{
    const int uart_num = ECHO_UART_PORT;
    uart_config_t uart_config = {
        .baud_rate = BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // Set UART log level
    esp_log_level_set(TAG, ESP_LOG_INFO);
    // Install UART driver (we don't need an event queue here)
    // In this example we don't even use a buffer for sending data.
    ESP_ERROR_CHECK(uart_driver_install(uart_num, BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config)); // Configure UART parameters
    ESP_LOGI(TAG, "UART set pins, mode and install driver.");
    // ESP_ERROR_CHECK(uart_set_pin(uart_num, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS));     // Set UART pins as per KConfig settings
    ESP_ERROR_CHECK(uart_set_pin(uart_num, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, UART_PIN_NO_CHANGE)); // Set UART pins as per KConfig settings
    ESP_ERROR_CHECK(uart_set_mode(uart_num, UART_MODE_RS485_HALF_DUPLEX));                                    // Set RS485 half duplex mode
    ESP_ERROR_CHECK(uart_set_rx_timeout(uart_num, ECHO_READ_TOUT));                                           // Set read timeout of UART TOUT feature
    uint8_t *data = (uint8_t *)malloc(BUF_SIZE);                                                              // Allocate buffers for UART
    ESP_LOGI(TAG, "UART start recieve loop.\r\n");
    echo_send(uart_num, "Start RS485 UART test.\r\n", 24);

    while (1)
    {
        int linenumber = 0;
        // Read data from UART
        int len = uart_read_bytes(uart_num, data, BUF_SIZE, PACKET_READ_TICS);
        // Write data back to UART
        if (len > 0)
        {
            echo_send(uart_num, "\r\n", 2);
            char prefix[] = "RS485 Received: [";
            echo_send(uart_num, prefix, (sizeof(prefix) - 1));
            ESP_LOGI(TAG, "Received %u bytes:", len);
            printf("[ ");
            for (int i = 0; i < len; i++)
            {
                printf("0x%.2X ", (uint8_t)data[i]);
                echo_send(uart_num, (const char *)&data[i], 1);
                print_text((const char *)&data[i], 0, linenumber);
                linenumber += LINEINC;
                if (linenumber > 64)
                {
                    linenumber = 0;
                }
                // Add a Newline character if you get a return charater from paste (Paste tests multibyte receipt/buffer)
                if (data[i] == '\r')
                {
                    echo_send(uart_num, "\n", 1);
                }
            }
            printf("] \n");
            echo_send(uart_num, "]\r\n", 3);
        }
        else
        {
            ESP_ERROR_CHECK(uart_wait_tx_done(uart_num, 10));
        }
    }
    vTaskDelete(NULL);
}

#define UART_NUM 1 // Use UART 1 (UART_NUM_1) or UART 2 (UART_NUM_2) as per your hardware connections

#define BUF_SIZE 128

void uart_init()
{
    uart_config_t uart_config = {
        .baud_rate = 19200, // Set your desired baud rate
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_RTS,
    };
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0));                                        // Install UART driver
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));                                                         // Configure UART parameters
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, ECHO_TEST_RTS, UART_PIN_NO_CHANGE)); // Set UART pins as per KConfig settings
    // Optionally, you can set UART RX buffer size (in this case, we use the default size)
    // ESP_ERROR_CHECK(uart_set_rx_buffer_size(UART_NUM, BUF_SIZE));
}

void uart_receive_task(void *arg)
{
    uint8_t data[BUF_SIZE];
    while (1)
    {
        // Read data from UART
        int len = uart_read_bytes(UART_NUM, data, BUF_SIZE, 100 / PACKET_READ_TICS); // Timeout of 100ms

        if (len > 0)
        {
            // Process received data here
            // Example: Print received data
            data[len] = '\0'; // Null-terminate the data for printing
            printf("Received: %s\n", data);
        }
    }
}

void timer_callback(TimerHandle_t xTimer)
{
    // Callback function to be executed when the timer expires
    gpio_set_level(17, 0);
    xTimerDelete(xTimer, portMAX_DELAY); // Delete the timer after its use
}

void start_timer()
{
    const TickType_t timer_period_ms = 5000;                                                                      // Timer period in milliseconds (5 seconds in this example)
    TimerHandle_t timer = xTimerCreate("GPIO_Timer", pdMS_TO_TICKS(timer_period_ms), pdFALSE, 0, timer_callback); // Create a one-shot software timer
    if (timer == NULL)
    { // Check if the timer was created successfully
        // Timer creation failed
        return;
    }
    // Start the timer
    if (xTimerStart(timer, 0) != pdPASS)
    {
        // Timer start failed
        xTimerDelete(timer, portMAX_DELAY); // Clean up the timer if start failed
        return;
    }
    printf("Timer started. GPIO will be switched low after %lu ms.\n", timer_period_ms); // Optionally, print a message to indicate the timer has started
}

void app_main(void)
{
    ESP_LOGI(TAG, "Initialize I2C bus");
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = EXAMPLE_PIN_NUM_SDA,
        .scl_io_num = EXAMPLE_PIN_NUM_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = EXAMPLE_LCD_PIXEL_CLOCK_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_HOST, &i2c_conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_HOST, I2C_MODE_MASTER, 0, 0, 0));

    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t io_config = {
        .dev_addr = EXAMPLE_I2C_HW_ADDR,
        .control_phase_bytes = 1,
        .lcd_cmd_bits = EXAMPLE_LCD_CMD_BITS,
        .lcd_param_bits = EXAMPLE_LCD_CMD_BITS,
#if CONFIG_EXAMPLE_LCD_CONTROLLER_SSD1306
        .dc_bit_offset = 6,
#elif CONFIG_EXAMPLE_LCD_CONTROLLER_SH1107
        .dc_bit_offset = 0,
        .flags =
            {
                .disable_control_phase = 1,
            }
#endif
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)I2C_HOST, &io_config, &io_handle));

    ESP_LOGI(TAG, "Install SSD1306 panel driver");
    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .bits_per_pixel = 1,
        .reset_gpio_num = EXAMPLE_PIN_NUM_RST,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(io_handle, &panel_config, &panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

#if CONFIG_EXAMPLE_LCD_CONTROLLER_SH1107
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true));
#endif

    ESP_LOGI(TAG, "Initialize LVGL");
    const lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    lvgl_port_init(&lvgl_cfg);

    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = io_handle,
        .panel_handle = panel_handle,
        .buffer_size = EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES,
        .double_buffer = true,
        .hres = EXAMPLE_LCD_H_RES,
        .vres = EXAMPLE_LCD_V_RES,
        .monochrome = true,
        .rotation = {
            .swap_xy = false,
            .mirror_x = false,
            .mirror_y = false,
        }};
    lv_disp_t *disp = lvgl_port_add_disp(&disp_cfg);
    /* Register done callback for IO */
    const esp_lcd_panel_io_callbacks_t cbs = {
        .on_color_trans_done = notify_lvgl_flush_ready,
    };
    esp_lcd_panel_io_register_event_callbacks(io_handle, &cbs, disp);

    /* Rotation of the screen */
    lv_disp_set_rotation(disp, LV_DISP_ROT_NONE);

    ESP_LOGI(TAG, "Starting RS485 to display");

    print_text("Status overview", 0, 0);
    // start_timer();
    uart_init();
    // xTaskCreate(echo_task, "uart_echo_task", ECHO_TASK_STACK_SIZE, NULL, ECHO_TASK_PRIO, NULL);
    #define SWPIN 19
       // gpio_reset_pin(SWPIN);
    gpio_set_direction(SWPIN, GPIO_MODE_OUTPUT);
    for(int i = 0; i < 50; i++) {
        set_gpio_state(SWPIN, 0);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        set_gpio_state(SWPIN, 1);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    xTaskCreate(uart_receive_task, "uart_receive_task", ECHO_TASK_STACK_SIZE, NULL, ECHO_TASK_PRIO, NULL);
    while (1)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
