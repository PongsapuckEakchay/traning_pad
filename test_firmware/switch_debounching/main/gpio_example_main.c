#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_timer.h"

#define BUTTON_GPIO 15  // Replace with your GPIO pin number
#define DEBOUNCE_TIME_MS 200    // 200 ms debounce time

static xQueueHandle gpio_evt_queue = NULL;
static uint64_t last_interrupt_time = 0;

static void IRAM_ATTR gpio_isr_handler(void* arg) {
    uint64_t current_time = esp_timer_get_time() / 1000; // Get time in ms
    if (current_time - last_interrupt_time > DEBOUNCE_TIME_MS) {
        last_interrupt_time = current_time;
        uint32_t gpio_num = (uint32_t) arg;
        xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
    }
}

static void gpio_task_example(void* arg) {
    uint32_t io_num;
    for (;;) {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            printf("Button pressed on GPIO %d\n", io_num);
        }
    }
}

void app_main(void) {
    gpio_config_t io_conf;
    
    // Configure button GPIO as input with pull-up
    io_conf.intr_type = GPIO_INTR_NEGEDGE; // Interrupt on falling edge (button press)
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << BUTTON_GPIO);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    // Create a queue to handle GPIO events from the ISR
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));

    // Start a task to process button presses
    xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL);

    // Install ISR service and attach the interrupt handler
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_GPIO, gpio_isr_handler, (void*) BUTTON_GPIO);

    printf("Switch debouncing example started\n");
}
