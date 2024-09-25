/* Deep sleep wake up example - EXT0 Pin GPIO 15 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_sleep.h"
#include "driver/rtc_io.h"

static RTC_DATA_ATTR struct timeval sleep_enter_time;

void app_main(void)
{
    struct timeval now;
    gettimeofday(&now, NULL);
    int sleep_time_ms = (now.tv_sec - sleep_enter_time.tv_sec) * 1000 + (now.tv_usec - sleep_enter_time.tv_usec) / 1000;

    switch (esp_sleep_get_wakeup_cause()) {
        case ESP_SLEEP_WAKEUP_EXT0: {
            printf("Wake up from EXT0 | sleep time : %d\n", sleep_time_ms);
            break;
        }
        case ESP_SLEEP_WAKEUP_UNDEFINED:
        default:
            printf("Not a deep sleep reset\n");
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    // Configure EXT0 wakeup on GPIO 15
    const int ext_wakeup_pin_0 = 15;  // Using GPIO 15 for EXT0 wakeup
    printf("Enabling EXT0 wakeup on pin GPIO%d\n", ext_wakeup_pin_0);
    ESP_ERROR_CHECK(esp_sleep_enable_ext0_wakeup(ext_wakeup_pin_0, 0));
    ESP_ERROR_CHECK(rtc_gpio_pullup_en(ext_wakeup_pin_0));  
    ESP_ERROR_CHECK(rtc_gpio_pulldown_dis(ext_wakeup_pin_0)); 
    esp_deep_sleep_start();
}
