/*
    This is just a part of vibation code, need to combine with the full one.
    In this file, it will tell the part of vibration code that had been changed.

*/

/////////////////////////////////
/*Start Marco and variable Part*/

// Old (delete all of this code and replace with new one)
/*
#define NO_OF_VIBRATION_SAMPLES 100
static uint64_t last_vibration_time = 0;
static uint64_t g_vibration_time_data[100] = {0};  // ข้อมูลเวลาที่ตรวจจับจาก vibration
static uint64_t diff_time[100];
static uint8_t g_vibration_time_len = 0;
*/

// New 
/*
#define VIBRATION_TIMEOUT 300 // vibration timeout time
#define VIB_THRESHOLD_MULTIPLIER 1.5 // threshold multiplier
static uint8_t g_vibration_count = 0; // จำนวนค่าที่อ่านได้จาก vibration (จำนวน Negedge)
static uint8_t g_vibration_reading = 0; // แสดงสถานะการอ่าน vibration 1=กำลังอ่านค่า 
static uint8_t g_vib_threshold = 1;  // ค่าเริ่มต้นสำหรับ vibration threshold
TaskHandle_t Vibration_Handle = NULL;
TaskHandle_t Vibration_Timer_Handle =NULL;
*/

/*End Marco and variable Part*/
/////////////////////////////////


/////////////////////////////////
/*Start gpio_isr_handler Part*/


// delete this code
/*
    uint64_t current_time = esp_timer_get_time() / 1000; 
*/

// delete all of "gpio_num == VIBRATION_PIN" part and replace with the new one

// New 
/*
    else if (gpio_num == VIBRATION_PIN) {
        if(g_vibration_reading==0){
            g_vibration_reading=1;
            g_vibration_count=1;
            vTaskNotifyGiveFromISR(Vibration_Timer_Handle,&xHigherPriorityTaskWoken);
        }
        else{
            g_vibration_count++;
        }
    }     
*/

/*End gpio_isr_handler Part*/
/////////////////////////////////


/////////////////////////////////
/*Start vibration_task Part*/

// delete all of vibration_task part and replace with the new one

// New 
/*
void vibration_task(void *pvParameters)
{
    esp_err_t ret;
    uint8_t on = 255;
    uint8_t off = 0;
    uint32_t ulNotificationValue;
    while(1){
        ulNotificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (ulNotificationValue > 0) {
            gpio_set_intr_type(VIBRATION_PIN, GPIO_INTR_DISABLE);
            if(g_vibration_count>=g_vib_threshold*VIB_THRESHOLD_MULTIPLIER){
                ret=esp_ble_gatts_set_attr_value(iwing_training_table[IDX_CHAR_VIBRATION_VAL], sizeof(on), &on);
                ble_send_notify(iwing_training_table[IDX_CHAR_VIBRATION_VAL], &on, sizeof(on));
                if (ret != ESP_OK) {
                    ESP_LOGE(ble_tag, "Failed to set vibration value : %d", ret);
                }
                else {
                    ESP_LOGI(ble_tag, "vibration value set to : 255");
                }
            }
            ESP_LOGI(Vibration_tag, "count = %u", g_vibration_count);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            ret=esp_ble_gatts_set_attr_value(iwing_training_table[IDX_CHAR_VIBRATION_VAL], sizeof(off), &off);
            ble_send_notify(iwing_training_table[IDX_CHAR_VIBRATION_VAL], &off, sizeof(off));
            if (ret != ESP_OK) {
                ESP_LOGE(ble_tag, "Failed to set vibration value : %d", ret);
            }
            else {
                ESP_LOGI(ble_tag, "vibration value set to : 0");
            }
            ESP_LOGI(ble_tag, "threshold = %u", g_vib_threshold);
            g_vibration_reading=0;
            gpio_set_intr_type(VIBRATION_PIN, GPIO_INTR_NEGEDGE);
        }
        vTaskDelay(1/ portTICK_PERIOD_MS);
    }
}  
*/

/*End vibration_task Part*/
/////////////////////////////////


/////////////////////////////////
/*Start vibration_timer Part*/

// this is a new task

// New 
/*
void vibration_timer(void* pvParameters){
    uint32_t ulNotificationValue;
    while(1){
        ulNotificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (ulNotificationValue > 0) {
            ESP_LOGI(Vibration_tag, "Start timer");
            vTaskDelay(VIBRATION_TIMEOUT/ portTICK_PERIOD_MS);
            ESP_LOGI(Vibration_tag, "Stop timer");
            xTaskNotifyGive(Vibration_Handle);
        }
    }
}
*/

/*End vibration_timer Part*/
/////////////////////////////////


/////////////////////////////////
/*Start app_main (Task_init) Part*/

// add vibration_timer task

// New 
/*
    xTaskCreate(vibration_task, "vibration_task", 2048, NULL, 16, &Vibration_Handle);
    xTaskCreate(vibration_timer, "vibration_timer", 2048, NULL, 16, &Vibration_Timer_Handle);
*/

/*End app_main (Task_init) Part*/
/////////////////////////////////