#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"
#include "driver/rmt.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define RMT_RX_CHANNEL    RMT_CHANNEL_0
#define GPIO_INPUT_PIN    18
#define SAMPLE_COUNT      100
#define SAMPLE_PERIOD     1000  // in microseconds (1ms)
#define BUFFER_SIZE       1000  // Increased buffer size

static const char *TAG = "DIGITAL_BIT_COLLECTION";

static void rmt_rx_init()
{
    rmt_config_t rmt_rx_config = {
        .rmt_mode = RMT_MODE_RX,
        .channel = RMT_RX_CHANNEL,
        .gpio_num = GPIO_INPUT_PIN,
        .clk_div = 80,  // 1us tick
        .mem_block_num = 2,  // Increased memory blocks
        .rx_config = {
            .filter_en = true,
            .filter_ticks_thresh = 10,
            .idle_threshold = SAMPLE_PERIOD * 2
        }
    };

    ESP_ERROR_CHECK(rmt_config(&rmt_rx_config));
    ESP_ERROR_CHECK(rmt_driver_install(RMT_RX_CHANNEL, BUFFER_SIZE, 0));
}

void app_main(void)
{
    rmt_rx_init();
    
    RingbufHandle_t rb = NULL;
    ESP_ERROR_CHECK(rmt_get_ringbuf_handle(RMT_RX_CHANNEL, &rb));
    ESP_LOGI(TAG, "Initialized RMT receiver on GPIO %d", GPIO_INPUT_PIN);

    while (1) {
        ESP_LOGI(TAG, "Waiting for trigger (GPIO %d going low)...", GPIO_INPUT_PIN);
        
        while (gpio_get_level(GPIO_INPUT_PIN) == 1) {
            vTaskDelay(1);
        }

        ESP_LOGI(TAG, "Triggered. Starting data collection...");
        
        ESP_ERROR_CHECK(rmt_rx_start(RMT_RX_CHANNEL, 1));

        int sample_count = 0;
        while (sample_count < SAMPLE_COUNT) {
            size_t rx_size = 0;
            rmt_item32_t* items = (rmt_item32_t*) xRingbufferReceive(rb, &rx_size, pdMS_TO_TICKS(10));
            if (items) {
                size_t num_items = rx_size / sizeof(rmt_item32_t);
                for (int i = 0; i < num_items && sample_count < SAMPLE_COUNT; i++) {
                    ESP_LOGI(TAG, "Bit %3d: %d", sample_count + 1, items[i].level0);
                    sample_count++;
                    if (sample_count < SAMPLE_COUNT) {
                        ESP_LOGI(TAG, "Bit %3d: %d", sample_count + 1, items[i].level1);
                        sample_count++;
                    }
                }
                vRingbufferReturnItem(rb, (void*) items);
            }
        }

        ESP_ERROR_CHECK(rmt_rx_stop(RMT_RX_CHANNEL));
        ESP_LOGI(TAG, "Data collection complete.");

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}