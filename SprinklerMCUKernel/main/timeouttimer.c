#include "defines.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "timeouttimer.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "driver/gpio.h"

#define DISPLAYATTACHED 1

struct TimerInfo {
    int8_t minutes;
    int8_t seconds;
};

struct TimerInfo remaining_time = {0.0, 0.0};

TimerHandle_t xTimer;

void timer_callback() {
    // Callback function to be executed when the timer expires
    gpio_set_level(GPIO_TO_BE_SWITCHED, 0);
}

void start_timer() {
    const TickType_t timer_period_ms = TIMEOUT; // Timer period in milliseconds

    // Create a one-shot software timer
    xTimer = xTimerCreate("GPIO_Timer", pdMS_TO_TICKS(timer_period_ms), pdFALSE, 0, timer_callback);

    // Check if the timer was created successfully
    if (xTimer == NULL) {
        // Timer creation failed
        return;
    }

    // Start the timer
    if (xTimerStart(xTimer, 0) != pdPASS) {
        // Timer start failed
        xTimerDelete(xTimer, portMAX_DELAY); // Clean up the timer if start failed
        return;
    }

    // Optionally, print a message to indicate the timer has started
    ESP_LOGI(TAG, "Timer started. GPIO will be switched low after %lu ms.\n", timer_period_ms);
    print_text("Timer started", 0, 64-16);
}

void timer_status(char* msg[])
{
    // Read the elapsed time and period from the timer
    TickType_t period_ticks = xTimerGetPeriod(xTimer);
    TickType_t elapsed_ticks = xTimerGetExpiryTime(xTimer) - xTaskGetTickCount();
    uint32_t elapsed_time_ms = (elapsed_ticks * portTICK_PERIOD_MS) % (period_ticks * portTICK_PERIOD_MS);

    // Convert elapsed time to minutes and seconds
    uint32_t remaining_time_seconds = elapsed_time_ms / 1000;
    remaining_time.minutes = remaining_time_seconds / 60;
    remaining_time.seconds = remaining_time_seconds % 60;

    // Print the remaining time in minutes and seconds
   // printf("Timer expired! Remaining Time: %lu minutes and %lu seconds\n", remaining_time.minutes, remaining_time.seconds);
    // char* msg[100];
    sprintf(msg, "timeout in: %u:%02u", remaining_time.minutes, remaining_time.seconds);
    
    //linesofdisplay[4] = msg;
    // print_text(msg, 0, 64-16);
}
