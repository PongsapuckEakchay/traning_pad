#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"

#define ESP_INTR_FLAG_DEFAULT 0
#define VIBRATION_PIN GPIO_NUM_18
#define SAMPLES 100

static const char *Vibration_tag = "Vibration_data";
TaskHandle_t Vibration_Handle = NULL;
TaskHandle_t task2Handle = NULL;

static volatile bool start_sampling = false;
static int samples[SAMPLES+20];
//static const vibration_threshold = 40;

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint32_t gpio_num = (uint32_t) arg;
    uint8_t  level = gpio_get_level(gpio_num);
    if (gpio_num == VIBRATION_PIN) {
        if (level == 0) {
            gpio_set_intr_type(VIBRATION_PIN, GPIO_INTR_DISABLE);
            vTaskNotifyGiveFromISR(Vibration_Handle, &xHigherPriorityTaskWoken);
            if (xHigherPriorityTaskWoken) {
                portYIELD_FROM_ISR();
             }
        }
    } 
    
}

void vibration_task(void *pvParameters)
{
    while (1) {
        // Wait for notification
        uint32_t ulNotificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (ulNotificationValue > 0) {
            uint8_t high = 0;
            uint8_t low = 0;
            for (int i = 0; i < SAMPLES; i++) {
                samples[i] = gpio_get_level(VIBRATION_PIN);
                if (samples[i] == 1) {
                    high++;
                 } else {
                    low++;
                }
                vTaskDelay(pdMS_TO_TICKS(1));
            }
            ESP_LOGI(Vibration_tag, "High: %d, Low: %d", high, low);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            gpio_set_intr_type(VIBRATION_PIN, GPIO_INTR_NEGEDGE);
        }
    }
}

void app_main(void)
{
    // Configure GPIO pin
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.pin_bit_mask = (1ULL << VIBRATION_PIN);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);


    // Install GPIO ISR service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(VIBRATION_PIN, gpio_isr_handler, (void*) VIBRATION_PIN);
    xTaskCreate(vibration_task, "vibration_task", 2048, NULL, 16, &Vibration_Handle);
    ESP_LOGI(Vibration_tag, "Waiting for trigger on GPIO %d", VIBRATION_PIN);

    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS); // Small delay to prevent watchdog trigger
    }
}