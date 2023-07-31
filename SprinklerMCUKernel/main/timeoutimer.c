#include "defines.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"

void timer_callback(TimerHandle_t xTimer) {
    // Callback function to be executed when the timer expires
    gpio_set_level(GPIO_TO_BE_SWITCHED, 0);

   
}

void start_timer() {
    const TickType_t timer_period_ms = TIMEOUT; // Timer period in milliseconds (5 seconds in this example)

    // Create a one-shot software timer
    TimerHandle_t timer = xTimerCreate("GPIO_Timer", pdMS_TO_TICKS(timer_period_ms), pdFALSE, 0, timer_callback);

    // Check if the timer was created successfully
    if (timer == NULL) {
        // Timer creation failed
        return;
    }

    // Start the timer
    if (xTimerStart(timer, 0) != pdPASS) {
        // Timer start failed
        xTimerDelete(timer, portMAX_DELAY); // Clean up the timer if start failed
        return;
    }

    // Optionally, print a message to indicate the timer has started
    printf("Timer started. GPIO will be switched low after %lu ms.\n", timer_period_ms);
    print_text("Timer started", 0, 64-16);
}

void timer_status()
{
     // Read the elapsed time and period from the timer
    TickType_t period_ticks = xTimerGetPeriod(xTimer);
    TickType_t elapsed_ticks = xTimerGetExpiryTime(xTimer) - xTaskGetTickCount();
    uint32_t elapsed_time_ms = (elapsed_ticks * portTICK_PERIOD_MS) % (period_ticks * portTICK_PERIOD_MS);

    // Convert elapsed time to minutes and seconds
    float remaining_time_seconds = elapsed_time_ms / 1000;
    remaining_time.minutes = remaining_time_seconds / 60;
    remaining_time.seconds = remaining_time_seconds % 60;

    // Print the remaining time in minutes and seconds
   // printf("Timer expired! Remaining Time: %lu minutes and %lu seconds\n", remaining_time.minutes, remaining_time.seconds);
    char* msg[100];
    sprintf(msg, "timeout in: %f:%f", remaining_time.minutes, remaining_time.seconds);
    // sprintf(msg, "timeout in: %f:%f", remaining_time.minutes, remaining_time.seconds);
    print_text("msg", 0, 64-16);
}
