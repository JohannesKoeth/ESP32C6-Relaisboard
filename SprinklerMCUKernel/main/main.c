#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "lvgl.h"
#include "esp_lvgl_port.h"

#include "driver/gpio.h"
#include "freertos/timers.h"

#include "esp_lcd_panel_vendor.h"
#include "defines.h"

#include "timeouttimer.h"
#include "comm_uart.h"

// GPIO pins with relais

#define LINESOFDISP 5

static int relais[4] = {18, 19, 21, 22};
static char *linesofdisplay[LINESOFDISP][20];

#define DISPLAYATTACHED 1

#if DISPLAYATTACHED
static bool notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    lv_disp_t *disp = (lv_disp_t *)user_ctx;
    lvgl_port_flush_ready(disp);
    return false;
}
#endif

// Subroutine to print text using LVGL
void print_text(const char *text, int pos_x, int pos_y)
{
#if DISPLAYATTACHED
    lv_obj_t *label = lv_label_create(lv_scr_act());     // Create a new label
    lv_label_set_text(label, text);                      // Set the text of the label to the input string
    lv_obj_align(label, LV_ALIGN_DEFAULT, pos_x, pos_y); // Align the label to the center of the screen
    lv_scr_load(lv_scr_act());                           // Refresh the screen
#else
    printf("%s\n", text);
#endif
}

// write a routine displaying the variable linesofdisplay on the display
void showpage()
{
#if DISPLAYATTACHED
  //  lv_obj_clean(lv_scr_act());
#endif
    for (int i = 0; i < LINESOFDISP; i++)
    {
        print_text(linesofdisplay[i], 0, i * LINEINC);
    }
}

void set_gpio_state(int gpio_num, bool state)
{
#define TAG "set_gpio_state"
    ESP_LOGI(TAG, "Switching Relais %d to %d", gpio_num, state);
    gpio_set_level(gpio_num, state);
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
    *num3 = atoi(last_slash + 1);    // Convert the third number after the last '/'
    const char *second_last_slash = strrchr(input, last_slash[-1]);    // Find the second last occurrence of '/'
    if (second_last_slash == NULL)
    {
        fprintf(stderr, "Invalid input format: Less than three numbers found.\n");
        return;
    }
    *num2 = atoi(second_last_slash + 1);    // Convert the second number after the second last '/'
    const char *third_last_slash = strrchr(input, second_last_slash[-1]);    // Find the third last occurrence of '/'
    if (third_last_slash == NULL)
    {
        fprintf(stderr, "Invalid input format: Less than three numbers found.\n");
        return;
    }
    // Convert the first number after the third last '/'
    *num1 = atoi(third_last_slash + 1);
}

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
    ESP_ERROR_CHECK(uart_driver_install(uart_num, BUFFERSIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config)); // Configure UART parameters
    ESP_LOGI(TAG, "UART set pins, mode and install driver.");
    // ESP_ERROR_CHECK(uart_set_pin(uart_num, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS));     // Set UART pins as per KConfig settings
    ESP_ERROR_CHECK(uart_set_pin(uart_num, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, UART_PIN_NO_CHANGE)); // Set UART pins as per KConfig settings
    ESP_ERROR_CHECK(uart_set_mode(uart_num, UART_MODE_RS485_HALF_DUPLEX));                                    // Set RS485 half duplex mode
    ESP_ERROR_CHECK(uart_set_rx_timeout(uart_num, ECHO_READ_TOUT));                                           // Set read timeout of UART TOUT feature
    uint8_t *data = (uint8_t *)malloc(BUFFERSIZE);                                                            // Allocate buffers for UART
    ESP_LOGI(TAG, "UART start recieve loop.\r\n");
    echo_send(uart_num, "Start RS485 UART test.\r\n", 24);

    while (1)
    {
        int linenumber = 0;
        // Read data from UART
        int len = uart_read_bytes(uart_num, data, BUFFERSIZE, PACKET_READ_TICS);
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

void app_main(void)
{
    bool state = false;
#if DISPLAYATTACHED
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
        .dc_bit_offset = 6,
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
#endif
    ESP_LOGI(TAG, "Starting RS485 to display");
    start_timer();
    uart_init();
// xTaskCreate(echo_task, "uart_echo_task", ECHO_TASK_STACK_SIZE, NULL, ECHO_TASK_PRIO, NULL);
#define SWPIN 19
    // gpio_reset_pin(SWPIN);
    gpio_set_direction(SWPIN, GPIO_MODE_OUTPUT);
    // for(int i = 0; i < 50; i++) {
    //     set_gpio_state(SWPIN, 0);
    //     vTaskDelay(2000 / portTICK_PERIOD_MS);
    //     set_gpio_state(SWPIN, 1);
    //     vTaskDelay(2000 / portTICK_PERIOD_MS);
    //     }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    xTaskCreate(uart_receive_task, "uart_receive_task", ECHO_TASK_STACK_SIZE, NULL, ECHO_TASK_PRIO, NULL);
    int i = 0;
    sprintf(linesofdisplay[0], "Valve 1: %s", state ? "open" : "closed");
    sprintf(linesofdisplay[1], "Valve 2: %s", state ? "open" : "closed");
    sprintf(linesofdisplay[2], "Valve 3: %s", state ? "open" : "closed");
    sprintf(linesofdisplay[3], "Valve 4: %s", state ? "open" : "closed");
    sprintf(linesofdisplay[4], "Statusline");
    while (1)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
#if DISPLAYATTACHED
        lv_obj_clean(lv_scr_act());
#endif
        i++;
        if (i == LINESOFDISP)
        {
            i = 0;
            state = !state;
        }
        set_gpio_state(relais[i], state);
        sprintf(linesofdisplay[i], "Valve %i: %s", i+1, state ? "open" : "closed");
        timer_status(linesofdisplay[4]);
        showpage();
    }
}
