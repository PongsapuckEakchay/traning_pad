#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/rmt.h"
#include "driver/gpio.h"
#include "led_strip.h"
#include "esp_err.h"

static const char *led_tag = "Led strip";
static led_strip_t *strip = NULL;

#define RMT_TX_CHANNEL RMT_CHANNEL_0

void set_all_leds(uint32_t red, uint32_t green, uint32_t blue)
{
    for (int i = 0; i < CONFIG_EXAMPLE_STRIP_LED_NUMBER; i++) {
        ESP_ERROR_CHECK(strip->set_pixel(strip, i, red, green, blue));
    }
    ESP_ERROR_CHECK(strip->refresh(strip, 100));
}

void show_color(uint32_t red, uint32_t green, uint32_t blue)
{
    set_all_leds(red, green, blue);
    vTaskDelay(pdMS_TO_TICKS(1000));  // รอ 1 วินาที
    set_all_leds(0, 0, 0);  // ปิดไฟทั้งหมด
    vTaskDelay(pdMS_TO_TICKS(1000));  // รอ 1 วินาที
}

void led_ble_disconnect()
{
    set_all_leds(255, 0, 0);
    vTaskDelay(pdMS_TO_TICKS(200));  // รอ 1 วินาที
    set_all_leds(0, 0, 0);  // ปิดไฟทั้งหมด
    vTaskDelay(pdMS_TO_TICKS(200));  // รอ 1 วินาที
}

void show_blue()
{
    show_color(0, 0, 255);
}

void show_red()
{
    show_color(255, 0, 0);
}

void show_green()
{
    show_color(0, 255, 0);
}

void app_main(void)
{
    // led setup
    rmt_config_t config = RMT_DEFAULT_CONFIG_TX(CONFIG_EXAMPLE_RMT_TX_GPIO, RMT_TX_CHANNEL);
    config.clk_div = 2;

    ESP_ERROR_CHECK(rmt_config(&config));
    ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));

    led_strip_config_t strip_config = LED_STRIP_DEFAULT_CONFIG(CONFIG_EXAMPLE_STRIP_LED_NUMBER, (led_strip_dev_t)config.channel);
    strip = led_strip_new_rmt_ws2812(&strip_config);
    if (!strip) {
        ESP_LOGE(led_tag, "install WS2812 driver failed");
    }

    // Clear LED strip (turn off all LEDs)
    ESP_ERROR_CHECK(strip->clear(strip, 100));
    // led setup end

    ESP_LOGI(led_tag, "LED Color Sequence Start");
    while (true) {
        
        led_ble_disconnect();
        
    }
}