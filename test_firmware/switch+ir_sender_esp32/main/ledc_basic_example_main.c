#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/rmt.h"
#include "led_strip.h"

#define IR_LED_PIN         16  // GPIO pin for IR LED
#define IR_RECEIVER_PIN    17  // GPIO pin for IR receiver
#define BUTTON_PIN         15   // GPIO pin for button
#define GPIO_INPUT_PIN_SEL  ((1ULL<<BUTTON_PIN) | (1ULL<<IR_RECEIVER_PIN))
#define ESP_INTR_FLAG_DEFAULT 0

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_10_BIT // 10 bit duty resolution
#define LEDC_FREQUENCY          (38000) // 38 kHz
#define LEDC_DUTY               (512)  // 50% duty cycle (2^10 * 0.5)

static const char *led_tag = "Led strip";
static led_strip_t *strip = NULL;
#define RMT_TX_CHANNEL RMT_CHANNEL_0

static xQueueHandle gpio_evt_queue = NULL;

void set_all_leds(uint32_t red, uint32_t green, uint32_t blue)
{
    for (int i = 0; i < CONFIG_EXAMPLE_STRIP_LED_NUMBER; i++) {
        ESP_ERROR_CHECK(strip->set_pixel(strip, i, red, green, blue));
    }
    ESP_ERROR_CHECK(strip->refresh(strip, 100));
}
void ir_received() 
{
    set_all_leds(0, 255, 0);
}
void ir_not_received() 
{
    set_all_leds(255, 0, 0);
}

void led_ble_disconnect()
{
    set_all_leds(255, 0, 0);
    vTaskDelay(pdMS_TO_TICKS(200));  // รอ 0.2 วินาที
    set_all_leds(0, 0, 0);  // ปิดไฟทั้งหมด
    vTaskDelay(pdMS_TO_TICKS(200));  // รอ 0.2 วินาที
}

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void gpio_task_example(void* arg)
{
    uint32_t io_num;
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            //printf("GPIO[%"PRIu32"] intr, val: %d\n", io_num, gpio_get_level(io_num));
            printf("GPIO[%d] intr, val: %d\n", io_num, gpio_get_level(io_num));
            if (io_num == BUTTON_PIN) {
                if (gpio_get_level(io_num) == 0) { // Button pressed (active low)
                    // Start PWM signal
                    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY));
                    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
                    printf("IR LED ON\n");
                } else {
                    // Stop PWM signal
                    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0));
                    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
                    printf("IR LED OFF\n");
                }
            }
            else if (io_num == IR_RECEIVER_PIN) {
                if (gpio_get_level(io_num) == 0) { // Button pressed (active low)
                    // green rgb on
                    ir_received();
                    printf("IR RECEIVED\n");
                } else {

                    // red rgb on
                    ir_not_received();
                    printf("IR NOT RECEIVED\n");
                }
            }
        }
    }
}

void ledc_init(void)
{
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = IR_LED_PIN,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
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

    // Initialize LEDC for IR LED control
    ledc_init();

    // Configure button GPIO
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL ;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    // Create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    
    // Start gpio task
    xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL);

    // Install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    
    // Hook isr handler for specific gpio pin
    gpio_isr_handler_add(BUTTON_PIN, gpio_isr_handler, (void*) BUTTON_PIN);
    gpio_isr_handler_add(IR_RECEIVER_PIN, gpio_isr_handler, (void*) IR_RECEIVER_PIN);
    printf("IR LED Control System Started\n");

    while(1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}