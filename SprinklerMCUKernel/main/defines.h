#ifndef DEFINES_H
#define DEFINES_H

static const char *TAG = "SprinklerMCU";

#define I2C_HOST  0
#define EXAMPLE_LCD_PIXEL_CLOCK_HZ    (400 * 1000)
#define EXAMPLE_PIN_NUM_SDA           6
#define EXAMPLE_PIN_NUM_SCL           7
#define EXAMPLE_PIN_NUM_RST           -1
#define EXAMPLE_I2C_HW_ADDR           0x3C

#define CONFIG_EXAMPLE_LCD_CONTROLLER_SSD1306 1

// The pixel number in horizontal and vertical
#define EXAMPLE_LCD_H_RES              128
#define EXAMPLE_LCD_V_RES              64
// Bit number used to represent command and parameter
#define EXAMPLE_LCD_CMD_BITS           8
#define EXAMPLE_LCD_PARAM_BITS         8

//
// RS485 Configuration
//

#define UART_NUM 1 // Use UART 1 (UART_NUM_1) or UART 2 (UART_NUM_2) as per your hardware connections

#define CONFIG_ENV_GPIO_RANGE_MIN 0
#define CONFIG_ENV_GPIO_RANGE_MAX 30
#define CONFIG_ENV_GPIO_IN_RANGE_MAX 30
#define CONFIG_ENV_GPIO_OUT_RANGE_MAX 30

#define CONFIG_ECHO_UART_PORT_NUM 0
#define CONFIG_ECHO_UART_BAUD_RATE 19200
#define CONFIG_ECHO_UART_RXD 24
#define CONFIG_ECHO_UART_TXD 25
#define CONFIG_ECHO_UART_RTS 10

// Note: Some pins on target chip cannot be assigned for UART communication.
// Please refer to documentation for selected board and target to configure pins using Kconfig.
#define ECHO_TEST_TXD   (CONFIG_ECHO_UART_TXD)
#define ECHO_TEST_RXD   (CONFIG_ECHO_UART_RXD)

// RTS for RS485 Half-Duplex Mode manages DE/~RE
#define ECHO_TEST_RTS   (CONFIG_ECHO_UART_RTS)

// CTS is not used in RS485 Half-Duplex Mode
#define ECHO_TEST_CTS   (UART_PIN_NO_CHANGE)

#define BUFFERSIZE        (127)
#define BAUD_RATE       (CONFIG_ECHO_UART_BAUD_RATE)

// Read packet timeout
#define PACKET_READ_TICS        (100 / portTICK_PERIOD_MS)
#define ECHO_TASK_STACK_SIZE    (2048)
#define ECHO_TASK_PRIO          (10)
#define ECHO_UART_PORT          (CONFIG_ECHO_UART_PORT_NUM)

// Timeout threshold for UART = number of symbols (~10 tics) with unchanged state on receive pin
#define ECHO_READ_TOUT          (3) // 3.5T * 8 = 28 ticks, TOUT=3 -> ~24..33 ticks

//  Timeout for the relais to be switched off
#define TIMEOUT 1000*60*90 // 90 minutes
#define GPIO_TO_BE_SWITCHED 18

// New line increment
#define LINEINC 14

#endif // DEFINES_H