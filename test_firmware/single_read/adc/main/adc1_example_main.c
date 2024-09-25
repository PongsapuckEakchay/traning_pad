#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

#define DEFAULT_VREF    1100        // Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          // Multisampling
#define ALPHA           0.3         // Exponential smoothing factor

static esp_adc_cal_characteristics_t *adc_chars;

static const adc_channel_t percent_bat_channel = ADC_CHANNEL_6;     // GPIO34 if ADC1
static const adc_bits_width_t width = ADC_WIDTH_BIT_12;
static const adc_atten_t atten = ADC_ATTEN_DB_0;
static const adc_unit_t unit = ADC_UNIT_1;

static const adc_channel_t status_bat_channel = ADC_CHANNEL_7;     // GPIO35 if ADC1

static void check_efuse(void)
{
#if CONFIG_IDF_TARGET_ESP32
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("eFuse Two Point: NOT supported\n");
    }
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printf("eFuse Vref: Supported\n");
    } else {
        printf("eFuse Vref: NOT supported\n");
    }
#elif CONFIG_IDF_TARGET_ESP32S2
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("Cannot retrieve eFuse Two Point calibration values. Default calibration values will be used.\n");
    }
#else
#error "This example is configured for ESP32/ESP32S2."
#endif
}

static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("Characterized using eFuse Vref\n");
    } else {
        printf("Characterized using Default Vref\n");
    }
}

// Exponential smoothing function
float exponential_smoothing(float prev_smooth, float new_value)
{
    return ALPHA * new_value + (1 - ALPHA) * prev_smooth;
}

// ADC sampling task
void bat_percent_task(void *pvParameter)
{
    float smoothed_voltage = 0.0;
    while (1) {
        uint32_t adc_reading = 0;
        // Multisampling
        for (int i = 0; i < NO_OF_SAMPLES; i++) {
            if (unit == ADC_UNIT_1) {
                adc_reading += adc1_get_raw((adc1_channel_t)percent_bat_channel);
            }
        }
        adc_reading /= NO_OF_SAMPLES;
        // Convert adc_reading to voltage in mV
        uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
        // Apply exponential smoothing
        smoothed_voltage = exponential_smoothing(smoothed_voltage, voltage);
        // Print the raw and smoothed voltage values
        printf("percent :Raw: %d\tVoltage: %dmV\tSmoothed Voltage: %.2fmV\n", adc_reading, voltage, smoothed_voltage);
        // Delay for 1 second
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
void bat_status_task(void *pvParameter)
{
    float smoothed_voltage = 0.0;
    while (1) {
        uint32_t adc_reading = 0;
        // Multisampling
        for (int i = 0; i < NO_OF_SAMPLES; i++) {
            if (unit == ADC_UNIT_1) {
                adc_reading += adc1_get_raw((adc1_channel_t)status_bat_channel);
            }
        }
        adc_reading /= NO_OF_SAMPLES;
        // Convert adc_reading to voltage in mV
        uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
        // Apply exponential smoothing
        //smoothed_voltage = exponential_smoothing(smoothed_voltage, voltage);
        // Print the raw and smoothed voltage values
        if (voltage > 900) {
            printf("Battery Status: Fully Charged\n");
        } else if (voltage > 300) {
            printf("Battery Status: not Charge\n");
        } else if (voltage < 200) {
            printf("Battery Status: Charging\n");
        }
        printf("status : Raw: %d\tVoltage: %dmV\n", adc_reading, voltage);
        // Delay for 1 second
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void app_main(void)
{
    // ################## adc setup code ##################
    // Check if Two Point or Vref are burned into eFuse
    check_efuse();
    // Configure ADC
    if (unit == ADC_UNIT_1) {
        adc1_config_width(width);
        adc1_config_channel_atten(percent_bat_channel, atten);
        adc1_config_channel_atten(status_bat_channel, atten);
    }
    // Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, width, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);
    // ################## adc setup code end ##################


    // Create ADC sampling task
    xTaskCreate(bat_percent_task, "bat_percent_task", 2048, NULL, 5, NULL);
    xTaskCreate(bat_status_task, "bat_status_task", 2048, NULL, 5, NULL);
}
