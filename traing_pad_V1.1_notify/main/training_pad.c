#include <stdio.h>
#include <string.h>
#include <ctype.h>
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
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "freertos/semphr.h"
#include <stdlib.h>
#include <sys/time.h>
#include "esp_sleep.h"
#include "driver/rtc_io.h"

#include <esp_http_server.h>
#include <esp_ota_ops.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <sys/param.h>
#include <esp_wifi.h>

// #################### BLE ####################
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "gatts_table_creat_demo.h"
#include "esp_gatt_common_api.h"

#define FIRMWARE_VERSION 0x05
#define WIFI_SSID "Triaing_Pad OTA Update"
#define GATTS_TABLE_TAG "GATTS_TABLE_DEMO"

#define PROFILE_NUM                 1
#define PROFILE_APP_IDX             0
#define ESP_APP_ID                  0x55
#define SAMPLE_DEVICE_NAME          "Trainning_PAD"
#define SVC_INST_ID                 0

/* The max length of characteristic value. When the GATT client performs a write or prepare write operation,
*  the data length must be less than GATTS_DEMO_CHAR_VAL_LEN_MAX.
*/
#define GATTS_DEMO_CHAR_VAL_LEN_MAX 250
#define PREPARE_BUF_MAX_SIZE        512
#define CHAR_DECLARATION_SIZE       (sizeof(uint8_t))

#define ADV_CONFIG_FLAG             (1 << 0)
#define SCAN_RSP_CONFIG_FLAG        (1 << 1)

static uint8_t adv_config_done       = 0;

uint16_t iwing_training_table[IWING_TRAINER_IDX_NB];

typedef struct {
    uint8_t                 *prepare_buf;
    int                     prepare_len;
} prepare_type_env_t;

static prepare_type_env_t prepare_write_env;

// #define CONFIG_SET_RAW_ADV_DATA
#ifdef CONFIG_SET_RAW_ADV_DATA
static uint8_t raw_adv_data[] = {
        /* flags */
        0x02, 0x01, 0x06,
        /* tx power*/
        0x02, 0x0a, 0xeb,
        /* service uuid */
        0x03, 0x03, 0xFF, 0x00,
        /* device name */
        0x0f, 0x09, 'T', 'R', 'A', 'N', 'I', 'N', 'G', '_', 'P', 'A', 'D'
};
static uint8_t raw_scan_rsp_data[] = {
        /* flags */
        0x02, 0x01, 0x06,
        /* tx power */
        0x02, 0x0a, 0xeb,
        /* service uuid */
        0x03, 0x03, 0xFF,0x00
};

#else
static uint8_t service_uuid[16] = { 0x01, 0x00, 0x63, 0xE7, 0x35, 0x9C, 0xE2, 0xB8, 0x29, 0x47, 0x2C, 0x82, 0xA1, 0xFD, 0xE9, 0xB2 };

/* The length of adv data must be less than 31 bytes */
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp        = false,
    .include_name        = true,
    .include_txpower     = true,
    .min_interval        = 0x0006, //slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval        = 0x0010, //slave connection max interval, Time = max_interval * 1.25 msec
    .appearance          = 0x00,
    .manufacturer_len    = 4,    //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data = NULL, //test_manufacturer,
    .service_data_len    = 0,
    .p_service_data      = NULL,
    .service_uuid_len    = sizeof(service_uuid),
    .p_service_uuid      = service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

// scan response data
static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp        = true,
    .include_name        = true,
    .include_txpower     = true,
    .min_interval        = 0x0006,
    .max_interval        = 0x0010,
    .appearance          = 0x00,
    .manufacturer_len    = 0, //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data = NULL, //&test_manufacturer[0],
    .service_data_len    = 0,
    .p_service_data      = NULL,
    .service_uuid_len    = sizeof(service_uuid),
    .p_service_uuid      = service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};
#endif /* CONFIG_SET_RAW_ADV_DATA */

static esp_ble_adv_params_t adv_params = {
    .adv_int_min         = 0x20,
    .adv_int_max         = 0x40,
    .adv_type            = ADV_TYPE_IND,
    .own_addr_type       = BLE_ADDR_TYPE_PUBLIC,
    .channel_map         = ADV_CHNL_ALL,
    .adv_filter_policy   = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

static void gatts_profile_event_handler(esp_gatts_cb_event_t event,
					esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst trainning_pad_profile_tab[PROFILE_NUM] = {
    [PROFILE_APP_IDX] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

/* Service */
static const uint8_t GATTS_SERVICE_UUID_TEST[16]      = { 0x01, 0x00, 0x63, 0xE7, 0x35, 0x9C, 0xE2, 0xB8, 0x29, 0x47, 0x2C, 0x82, 0xA1, 0xFD, 0xE9, 0xB2 }; // IWING_TRAINER

static const uint8_t GATTS_CHAR_UUID_BATT_VOLTAGE[16] = { 0x02, 0x00, 0x63, 0xE7, 0x35, 0x9C, 0xE2, 0xB8, 0x29, 0x47, 0x2C, 0x82, 0xA1, 0xFD, 0xE9, 0xB2 }; // BATT_VOLTAGE
static const uint8_t GATTS_CHAR_UUID_BATT_CHARGING[16] = { 0x03, 0x00, 0x63, 0xE7, 0x35, 0x9C, 0xE2, 0xB8, 0x29, 0x47, 0x2C, 0x82, 0xA1, 0xFD, 0xE9, 0xB2 }; // BATT_CHARGING
static const uint8_t GATTS_CHAR_UUID_BATT_FULL[16]    = { 0x04, 0x00, 0x63, 0xE7, 0x35, 0x9C, 0xE2, 0xB8, 0x29, 0x47, 0x2C, 0x82, 0xA1, 0xFD, 0xE9, 0xB2 }; // BATT_FULL
static const uint8_t GATTS_CHAR_UUID_BUTTONS[16]      = { 0x05, 0x00, 0x63, 0xE7, 0x35, 0x9C, 0xE2, 0xB8, 0x29, 0x47, 0x2C, 0x82, 0xA1, 0xFD, 0xE9, 0xB2 }; // BUTTONS
static const uint8_t GATTS_CHAR_UUID_VIBRATION[16]    = { 0x06, 0x00, 0x63, 0xE7, 0x35, 0x9C, 0xE2, 0xB8, 0x29, 0x47, 0x2C, 0x82, 0xA1, 0xFD, 0xE9, 0xB2 }; // VIBRATION
static const uint8_t GATTS_CHAR_UUID_IR_RX[16]        = { 0x07, 0x00, 0x63, 0xE7, 0x35, 0x9C, 0xE2, 0xB8, 0x29, 0x47, 0x2C, 0x82, 0xA1, 0xFD, 0xE9, 0xB2 }; // IR_RX
static const uint8_t GATTS_CHAR_UUID_VIB_THRES[16]    = { 0x08, 0x00, 0x63, 0xE7, 0x35, 0x9C, 0xE2, 0xB8, 0x29, 0x47, 0x2C, 0x82, 0xA1, 0xFD, 0xE9, 0xB2 }; // VIB_THRES
static const uint8_t GATTS_CHAR_UUID_LED[16]          = { 0x09, 0x00, 0x63, 0xE7, 0x35, 0x9C, 0xE2, 0xB8, 0x29, 0x47, 0x2C, 0x82, 0xA1, 0xFD, 0xE9, 0xB2 }; // LED
static const uint8_t GATTS_CHAR_UUID_IR_TX[16]        = { 0x0A, 0x00, 0x63, 0xE7, 0x35, 0x9C, 0xE2, 0xB8, 0x29, 0x47, 0x2C, 0x82, 0xA1, 0xFD, 0xE9, 0xB2 }; // IR_TX
static const uint8_t GATTS_CHAR_UUID_MUSIC[16]        = { 0x0B, 0x00, 0x63, 0xE7, 0x35, 0x9C, 0xE2, 0xB8, 0x29, 0x47, 0x2C, 0x82, 0xA1, 0xFD, 0xE9, 0xB2 }; // MUSIC
static const uint8_t GATTS_CHAR_UUID_MODE[16]        = { 0x0C, 0x00, 0x63, 0xE7, 0x35, 0x9C, 0xE2, 0xB8, 0x29, 0x47, 0x2C, 0x82, 0xA1, 0xFD, 0xE9, 0xB2 }; // MODE

static const uint16_t primary_service_uuid         = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid   = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
// static const uint8_t char_prop_read                =  ESP_GATT_CHAR_PROP_BIT_READ;
// static const uint8_t char_prop_write               = ESP_GATT_CHAR_PROP_BIT_WRITE;
static uint16_t cccd_value = 0x0000;
static const uint8_t char_prop_read_write_notify   = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t char_prop_read_notify   = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
// static const uint8_t heart_measurement_ccc[2]      = {0x00, 0x00};

static const uint8_t char_value[4]  = {0x11, 0x22, 0x33, 0x44};
static uint8_t manufacturer_data[4] = {0xFF, 0xFF, 0x00, 0x00};
// #################### END BLE ####################
// rgb led pin 19 
#define IR_LED_PIN         16  // GPIO pin for IR LED
#define IR_RECEIVER_PIN    17  // GPIO pin for IR receiver
#define BUTTON_PIN         15   // GPIO pin for button
#define VIBRATION_PIN      21   // GPIO pin for vibration sensor
#define VIBRATION_EN_PIN   32   // GPIO pin for vibration sensor enable
#define GPIO_INPUT_BUTTON_PIN_SEL  1ULL<<BUTTON_PIN
#define GPIO_INPUT_IR_PIN_SEL 1ULL<<IR_RECEIVER_PIN
#define GPIO_INPUT_VIBRATION_PIN_SEL 1ULL<<VIBRATION_PIN
#define GPIO_SONIC_PIN_SEL 1ULL<<VIBRATION_EN_PIN
#define ESP_INTR_FLAG_DEFAULT 0
#define DEBOUNCE_TIME_MS   10 // 200 ms debounce time
#define SAMPLES 100
#define BLE_TIMEOUT 10000 // timeout before sleep
#define LED_INTENSITY 200
#define LED_BREAHTING_STEP 10
static int last_level = 1;

#define IR_PWM_TMER               LEDC_TIMER_0
#define IR_PWM_MODE               LEDC_LOW_SPEED_MODE
#define IR_PWM_CHANNEL            LEDC_CHANNEL_0
#define IR_PWM_DUTY_RES           LEDC_TIMER_10_BIT // 10 bit duty resolution
#define IR_PWM_FREQUENCY          (38000) // 38 kHz
#define IR_PWM_DUTY               (512)  // 50% duty cycle (2^10 * 0.5)

#define DEFAULT_VREF    1100        // Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          // Multisampling
#define ALPHA           0.3         // Exponential smoothing factor

#define BATT_DIVIDED_FACTOR 5 //multiply with batt data before send

#define BUZZER_TIMER              LEDC_TIMER_1
#define BUZZER_MODE               LEDC_LOW_SPEED_MODE
#define BUZZER_OUTPUT_IO          (2) // Define the output GPIO
#define BUZZER_CHANNEL            LEDC_CHANNEL_1
#define BUZZER_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define BUZZER_DUTY               (4095) // Set duty to 50%. ((2 ** 13) - 1) * 50% = 4095
#define MAX_OCTAVE 8
#define MIN_OCTAVE 1
#define DEFAULT_OCTAVE 4
#define DEFAULT_DURATION 4
#define DEFAULT_TEMPO 120
#define MAX_SONG_LENGTH 100

// Base frequencies for octave 4
const int base_frequencies[] = {
    262, 277, 294, 311, 330, 349, 370, 392, 415, 440, 466, 494
};
static int current_octave;
static int default_duration;
static int tempo;
static char current_song[MAX_SONG_LENGTH] = {0};  // ตัวแปร global สำหรับเก็บโน้ตเพลง

static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t percent_bat_channel = ADC_CHANNEL_6;     // GPIO34 if ADC1
static const adc_bits_width_t width = ADC_WIDTH_BIT_12;
static const adc_atten_t atten = ADC_ATTEN_DB_0;
static const adc_unit_t unit = ADC_UNIT_1;
static const adc_channel_t status_bat_channel = ADC_CHANNEL_7;     // GPIO35 if ADC1

static const char *Vibration_tag = "Vibration_data";
static const char *led_tag = "Led strip";
static const char *adc_tag = "ADC";
static const char *ble_tag = "BLE STATUS";
static led_strip_t *strip = NULL;
#define RMT_TX_CHANNEL RMT_CHANNEL_0
#define RMT_TX_GPIO CONFIG_EXAMPLE_RMT_TX_GPIO

static xQueueHandle button_evt_queue = NULL;
static xQueueHandle ir_evt_queue = NULL;
static xQueueHandle ultrasonic_queue = NULL;
static xQueueHandle ble_data_queue = NULL;
static xQueueHandle ble_status_queue = NULL;
TaskHandle_t Button_Handle = NULL;
TaskHandle_t ble_data_task_handle = NULL;
static TaskHandle_t breathing_led_handle = NULL;
static SemaphoreHandle_t g_ble_data_mutex = NULL;

static uint8_t g_led_color[3] = {0, 0, 0};  // RGB color for LED
static uint8_t g_ir_tx_state = 0;  // สถานะของ IR transmitter
static uint8_t g_music_data[100] = {0};  // ข้อมูลเพลงใน Music Macro Language
static size_t g_music_data_len = 0;
static uint8_t ble_is_connected = 1;
static uint8_t g_mode[4] = {FIRMWARE_VERSION, 0, 0, 0};  // [A,B,C,D] - version, reserved, calibrate flag, mode

#define VIBRATION_TIMEOUT 300 // vibration timeout time
#define VIB_THRESHOLD_MULTIPLIER 1.5 // threshold multiplier
static uint8_t g_vibration_count = 0; // จำนวนค่าที่อ่านได้จาก vibration (จำนวน Negedge)
static uint8_t g_vibration_reading = 0; // แสดงสถานะการอ่าน vibration 1=กำลังอ่านค่า 
static uint8_t g_vib_threshold = 1;  // ค่าเริ่มต้นสำหรับ vibration threshold
TaskHandle_t Vibration_Handle = NULL;
TaskHandle_t Vibration_Timer_Handle =NULL;

static esp_gatt_if_t gatt_if = ESP_GATT_IF_NONE;
static uint16_t conn_id = 0;

static bool is_ota_mode = false;
extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[] asm("_binary_index_html_end");

void set_all_leds(uint8_t red, uint8_t green, uint8_t blue);
void sleep_call();
void set_song();
void stop_song();
void start_ota_mode();
typedef struct {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
} rgb_color_t;

esp_err_t index_get_handler(httpd_req_t *req)
{
	httpd_resp_send(req, (const char *) index_html_start, index_html_end - index_html_start);
	return ESP_OK;
}

/*
 * Handle OTA file upload
 */
esp_err_t update_post_handler(httpd_req_t *req)
{
	char buf[1000];
	esp_ota_handle_t ota_handle;
	int remaining = req->content_len;

	const esp_partition_t *ota_partition = esp_ota_get_next_update_partition(NULL);
	ESP_ERROR_CHECK(esp_ota_begin(ota_partition, OTA_SIZE_UNKNOWN, &ota_handle));

	while (remaining > 0) {
		int recv_len = httpd_req_recv(req, buf, MIN(remaining, sizeof(buf)));

		// Timeout Error: Just retry
		if (recv_len == HTTPD_SOCK_ERR_TIMEOUT) {
			continue;

		// Serious Error: Abort OTA
		} else if (recv_len <= 0) {
			httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Protocol Error");
			return ESP_FAIL;
		}

		// Successful Upload: Flash firmware chunk
		if (esp_ota_write(ota_handle, (const void *)buf, recv_len) != ESP_OK) {
			httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Flash Error");
			return ESP_FAIL;
		}

		remaining -= recv_len;
	}

	// Validate and switch to new OTA image and reboot
	if (esp_ota_end(ota_handle) != ESP_OK || esp_ota_set_boot_partition(ota_partition) != ESP_OK) {
			httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Validation / Activation Error");
			return ESP_FAIL;
	}

	httpd_resp_sendstr(req, "Firmware update complete, rebooting now!\n");

	vTaskDelay(500 / portTICK_PERIOD_MS);
	esp_restart();

	return ESP_OK;
}

/*
 * HTTP Server
 */
httpd_uri_t index_get = {
	.uri	  = "/",
	.method   = HTTP_GET,
	.handler  = index_get_handler,
	.user_ctx = NULL
};

httpd_uri_t update_post = {
	.uri	  = "/update",
	.method   = HTTP_POST,
	.handler  = update_post_handler,
	.user_ctx = NULL
};

static esp_err_t http_server_init(void)
{
	static httpd_handle_t http_server = NULL;

	httpd_config_t config = HTTPD_DEFAULT_CONFIG();

	if (httpd_start(&http_server, &config) == ESP_OK) {
		httpd_register_uri_handler(http_server, &index_get);
		httpd_register_uri_handler(http_server, &update_post);
	}

	return http_server == NULL ? ESP_FAIL : ESP_OK;
}

/*
 * WiFi configuration
 */
static esp_err_t softap_init(void)
{
	esp_err_t res = ESP_OK;

	res |= esp_netif_init();
	res |= esp_event_loop_create_default();
	esp_netif_create_default_wifi_ap();

	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	res |= esp_wifi_init(&cfg);

	wifi_config_t wifi_config = {
		.ap = {
			.ssid = WIFI_SSID,
			.ssid_len = strlen(WIFI_SSID),
			.channel = 6,
			.authmode = WIFI_AUTH_OPEN,
			.max_connection = 3
		},
	};

	res |= esp_wifi_set_mode(WIFI_MODE_AP);
	res |= esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config);
	res |= esp_wifi_start();

	return res;
}
// #################### BLE ####################
static const esp_gatts_attr_db_t gatt_db[IWING_TRAINER_IDX_NB] =
{ 
    // Service Declaration for IWING_TRAINER (UUID: B2E9FDA1-822C-4729-B8E2-9C35E7630001)
    [IDX_SVC]        =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
      sizeof(uint16_t), ESP_UUID_LEN_128, (uint8_t *)&GATTS_SERVICE_UUID_TEST}},

    /* Characteristic Declaration for BATT_VOLTAGE (UUID: B2E9FDA1-822C-4729-B8E2-9C35E7630002) */
    [IDX_CHAR_BATT_VOLTAGE_DECL]     =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},

    /* Characteristic Value for BATT_VOLTAGE */
    [IDX_CHAR_BATT_VOLTAGE_VAL] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)&GATTS_CHAR_UUID_BATT_VOLTAGE, ESP_GATT_PERM_READ,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)&char_value}}, // RO: ระดับแรงดันแบตเตอรี่ (mV)

    /* Client Characteristic Configuration Descriptor for BATT_VOLTAGE */
    [IDX_CHAR_BATT_VOLTAGE_CCCD] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid,
      ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(uint16_t), sizeof(uint16_t), (uint8_t *)&cccd_value}}, // RO: สถานะการ notify ของแรงดันแบตเตอรี่
    

    /* Characteristic Declaration for BATT_CHARGING (UUID: B2E9FDA1-822C-4729-B8E2-9C35E7630003) */
    [IDX_CHAR_BATT_CHARGING_DECL]     =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},

    /* Characteristic Value for BATT_CHARGING */
    [IDX_CHAR_BATT_CHARGING_VAL] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)&GATTS_CHAR_UUID_BATT_CHARGING, ESP_GATT_PERM_READ,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)&char_value}}, // RO: สถานะการเสียบชาร์จแบตเตอรี่ (1 = charging)

    /* Client Characteristic Configuration Descriptor for BATT_CHARGING */
    [IDX_CHAR_BATT_CHARGING_CCCD] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid,
      ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(uint16_t), sizeof(uint16_t), (uint8_t *)&cccd_value}}, // RW: สถานะการ notify ของการเสียบชาร์จ

    /* Characteristic Declaration for BATT_FULL (UUID: B2E9FDA1-822C-4729-B8E2-9C35E7630004) */
    [IDX_CHAR_BATT_FULL_DECL]     =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},

    /* Characteristic Value for BATT_FULL */
    [IDX_CHAR_BATT_FULL_VAL] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)&GATTS_CHAR_UUID_BATT_FULL, ESP_GATT_PERM_READ,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)&char_value}}, // RO: สถานะแบตเตอรี่ชาร์จเต็ม (1 = full)

    /* Client Characteristic Configuration Descriptor for BATT_FULL */
    [IDX_CHAR_BATT_FULL_CCCD] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid,
      ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(uint16_t), sizeof(uint16_t), (uint8_t *)&cccd_value}}, // RO: สถานะการ notify ของแบตเตอรี่ชาร์จเต็ม

    /* Characteristic Declaration for BUTTONS (UUID: B2E9FDA1-822C-4729-B8E2-9C35E7630005) */
    [IDX_CHAR_BUTTONS_DECL]     =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},

    /* Characteristic Value for BUTTONS */
    [IDX_CHAR_BUTTONS_VAL] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)&GATTS_CHAR_UUID_BUTTONS, ESP_GATT_PERM_READ,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)&char_value}}, // RO: สถานะปุ่มกด บิตละ 1 ปุ่ม (1 = กด)

    /* Client Characteristic Configuration Descriptor for BUTTONS */
    [IDX_CHAR_BUTTONS_CCCD] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid,
      ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(uint16_t), sizeof(uint16_t), (uint8_t *)&cccd_value}},


    /* Characteristic Declaration for VIBRATION (UUID: B2E9FDA1-822C-4729-B8E2-9C35E7630006) */
    [IDX_CHAR_VIBRATION_DECL]     =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},

    /* Characteristic Value for VIBRATION */
    [IDX_CHAR_VIBRATION_VAL] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)&GATTS_CHAR_UUID_VIBRATION, ESP_GATT_PERM_READ,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)&char_value}}, // RO: สถานะ vibration sensor ไบต์ละ 1 ตัว (255 = เคลื่อนไหวแรงสุด)

    /* Client Characteristic Configuration Descriptor for VIBRATION */
    [IDX_CHAR_VIBRATION_CCCD] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid,
      ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(uint16_t), sizeof(uint16_t), (uint8_t *)&cccd_value}},

    /* Characteristic Declaration for IR_RX (UUID: B2E9FDA1-822C-4729-B8E2-9C35E7630007) */
    [IDX_CHAR_IR_RX_DECL]     =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},

    /* Characteristic Value for IR_RX */
    [IDX_CHAR_IR_RX_VAL] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)&GATTS_CHAR_UUID_IR_RX, ESP_GATT_PERM_READ,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)&char_value}}, // RO: สถานะ IR Receiver (1 = ได้รับสัญญาณ)

    /* Client Characteristic Configuration Descriptor for IR_RX */
    [IDX_CHAR_IR_RX_CCCD] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid,
      ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(uint16_t), sizeof(uint16_t), (uint8_t *)&cccd_value}},

    /* Characteristic Declaration for VIB_THRES (UUID: B2E9FDA1-822C-4729-B8E2-9C35E7630008) */
    [IDX_CHAR_VIB_THRES_DECL]     =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

    /* Characteristic Value for VIB_THRES */
    [IDX_CHAR_VIB_THRES_VAL] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)&GATTS_CHAR_UUID_VIB_THRES, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)&char_value}}, // RW: threshold สำหรับ vibration sensor (8 บิต)

    /* Client Characteristic Configuration Descriptor for VIB_THRES */
    [IDX_CHAR_VIB_THRES_CCCD] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid,
      ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(uint16_t), sizeof(uint16_t), (uint8_t *)&cccd_value}},

    /* Characteristic Declaration for LED (UUID: B2E9FDA1-822C-4729-B8E2-9C35E7630009) */
    [IDX_CHAR_LED_DECL]     =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

    /* Characteristic Value for LED */
    [IDX_CHAR_LED_VAL] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)&GATTS_CHAR_UUID_LED, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)&char_value}}, // RW: สีของ RGB LED ดวงละ 3 ไบต์ (RGB x 8 บิต)

    /* Client Characteristic Configuration Descriptor for LED */
    [IDX_CHAR_LED_CCCD] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid,
      ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(uint16_t), sizeof(uint16_t), (uint8_t *)&cccd_value}},

    /* Characteristic Declaration for IR_TX (UUID: B2E9FDA1-822C-4729-B8E2-9C35E763000A) */
    [IDX_CHAR_IR_TX_DECL]     =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

    /* Characteristic Value for IR_TX */
    [IDX_CHAR_IR_TX_VAL] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)&GATTS_CHAR_UUID_IR_TX, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)&char_value}}, // RW: สถานะ IR Transmitter (1 = เปิดใช้งาน)

    /* Client Characteristic Configuration Descriptor for IR_TX */
    [IDX_CHAR_IR_TX_CCCD] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid,
      ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(uint16_t), sizeof(uint16_t), (uint8_t *)&cccd_value}},

    /* Characteristic Declaration for MUSIC (UUID: B2E9FDA1-822C-4729-B8E2-9C35E763000B) */
    [IDX_CHAR_MUSIC_DECL]     =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

    /* Characteristic Value for MUSIC */
    [IDX_CHAR_MUSIC_VAL] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)&GATTS_CHAR_UUID_MUSIC, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)&char_value}}, // RW: โน้ตเพลงในรูป Music Macro Language

    /* Client Characteristic Configuration Descriptor for MUSIC */
    [IDX_CHAR_MUSIC_CCCD] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid,
      ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(uint16_t), sizeof(uint16_t), (uint8_t *)&cccd_value}},

     /* Characteristic Declaration for MODE (UUID: B2E9FDA1-822C-4729-B8E2-9C35E763000C) */
    [IDX_CHAR_MODE_DECL] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

    /* Characteristic Value for MODE */
    [IDX_CHAR_MODE] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)&GATTS_CHAR_UUID_MODE, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(g_mode), (uint8_t *)&g_mode}}, // RW: สถานะ MODE (1 = เปิดใช้งานการ calibrate  , 0 = ปิดการ calibrate)

    /* Client Characteristic Configuration Descriptor for MODE */
    [IDX_CHAR_MODE_CCCD] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid,
      ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(uint16_t), sizeof(uint16_t), (uint8_t *)&cccd_value}},
    
};



static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    #ifdef CONFIG_SET_RAW_ADV_DATA
        case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
            adv_config_done &= (~ADV_CONFIG_FLAG);
            if (adv_config_done == 0){
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
        case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
            adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
            if (adv_config_done == 0){
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
    #else
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            adv_config_done &= (~ADV_CONFIG_FLAG);
            if (adv_config_done == 0 && !is_ota_mode){
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
        case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
            adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
            if (adv_config_done == 0 && !is_ota_mode){
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
    #endif
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            /* advertising start complete event to indicate advertising start successfully or failed */
            if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(GATTS_TABLE_TAG, "advertising start failed");
            }else{
                ESP_LOGI(GATTS_TABLE_TAG, "advertising start successfully");
            }
            break;
        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(GATTS_TABLE_TAG, "Advertising stop failed");
            }
            else {
                ESP_LOGI(GATTS_TABLE_TAG, "Stop adv successfully");
            }
            break;
        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
                  param->update_conn_params.status,
                  param->update_conn_params.min_int,
                  param->update_conn_params.max_int,
                  param->update_conn_params.conn_int,
                  param->update_conn_params.latency,
                  param->update_conn_params.timeout);
            break;
        default:
            break;
    }
}

void example_prepare_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param)
{
    ESP_LOGI(GATTS_TABLE_TAG, "prepare write, handle = %d, value len = %d", param->write.handle, param->write.len);
    esp_gatt_status_t status = ESP_GATT_OK;
    if (param->write.offset > PREPARE_BUF_MAX_SIZE) {
        status = ESP_GATT_INVALID_OFFSET;
    } else if ((param->write.offset + param->write.len) > PREPARE_BUF_MAX_SIZE) {
        status = ESP_GATT_INVALID_ATTR_LEN;
    }
    if (status == ESP_GATT_OK && prepare_write_env->prepare_buf == NULL) {
        prepare_write_env->prepare_buf = (uint8_t *)malloc(PREPARE_BUF_MAX_SIZE * sizeof(uint8_t));
        prepare_write_env->prepare_len = 0;
        if (prepare_write_env->prepare_buf == NULL) {
            ESP_LOGE(GATTS_TABLE_TAG, "%s, Gatt_server prep no mem", __func__);
            status = ESP_GATT_NO_RESOURCES;
        }
    }

    /*send response when param->write.need_rsp is true */
    if (param->write.need_rsp){
        esp_gatt_rsp_t *gatt_rsp = (esp_gatt_rsp_t *)malloc(sizeof(esp_gatt_rsp_t));
        if (gatt_rsp != NULL){
            gatt_rsp->attr_value.len = param->write.len;
            gatt_rsp->attr_value.handle = param->write.handle;
            gatt_rsp->attr_value.offset = param->write.offset;
            gatt_rsp->attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;
            memcpy(gatt_rsp->attr_value.value, param->write.value, param->write.len);
            esp_err_t response_err = esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, gatt_rsp);
            if (response_err != ESP_OK) {
               ESP_LOGE(GATTS_TABLE_TAG, "Send response error");
            }
            free(gatt_rsp);
        }else{
            ESP_LOGE(GATTS_TABLE_TAG, "%s, malloc failed", __func__);
            status = ESP_GATT_NO_RESOURCES;
        }
    }
    if (status != ESP_GATT_OK){
        return;
    }
    memcpy(prepare_write_env->prepare_buf + param->write.offset,
           param->write.value,
           param->write.len);
    prepare_write_env->prepare_len += param->write.len;

}

void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param){
    if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC && prepare_write_env->prepare_buf){
        esp_log_buffer_hex(GATTS_TABLE_TAG, prepare_write_env->prepare_buf, prepare_write_env->prepare_len);
    }else{
        ESP_LOGI(GATTS_TABLE_TAG,"ESP_GATT_PREP_WRITE_CANCEL");
    }
    if (prepare_write_env->prepare_buf) {
        free(prepare_write_env->prepare_buf);
        prepare_write_env->prepare_buf = NULL;
    }
    prepare_write_env->prepare_len = 0;
}

// void handle_read_ble_event(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
//     if(param->read.handle == iwing_training_table[IDX_CHAR_BATT_VOLTAGE_VAL]) {
//         esp_gatt_rsp_t rsp;
//         memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
//         rsp.attr_value.handle = param->read.handle;
//         rsp.attr_value.len = sizeof(button_state);
//         rsp.attr_value.value[0] = button_state;
//         esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
//     }
// }

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    
    switch (event) {
        case ESP_GATTS_REG_EVT:{
            gatt_if = gatts_if;
            esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(SAMPLE_DEVICE_NAME);
            if (set_dev_name_ret){
                ESP_LOGE(GATTS_TABLE_TAG, "set device name failed, error code = %x", set_dev_name_ret);
            }
    #ifdef CONFIG_SET_RAW_ADV_DATA
            esp_err_t raw_adv_ret = esp_ble_gap_config_adv_data_raw(raw_adv_data, sizeof(raw_adv_data));
            if (raw_adv_ret){
                ESP_LOGE(GATTS_TABLE_TAG, "config raw adv data failed, error code = %x ", raw_adv_ret);
            }
            adv_config_done |= ADV_CONFIG_FLAG;
            esp_err_t raw_scan_ret = esp_ble_gap_config_scan_rsp_data_raw(raw_scan_rsp_data, sizeof(raw_scan_rsp_data));
            if (raw_scan_ret){
                ESP_LOGE(GATTS_TABLE_TAG, "config raw scan rsp data failed, error code = %x", raw_scan_ret);
            }
            adv_config_done |= SCAN_RSP_CONFIG_FLAG;
    #else
            //config adv data
            if(!is_ota_mode) {
                esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
                if (ret){
                    ESP_LOGE(GATTS_TABLE_TAG, "config adv data failed, error code = %x", ret);
                }
                adv_config_done |= ADV_CONFIG_FLAG;
                //config scan response data
                ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
                if (ret){
                    ESP_LOGE(GATTS_TABLE_TAG, "config scan response data failed, error code = %x", ret);
                }
                adv_config_done |= SCAN_RSP_CONFIG_FLAG;
            }
    #endif
            esp_err_t create_attr_ret = esp_ble_gatts_create_attr_tab(gatt_db, gatts_if, IWING_TRAINER_IDX_NB, SVC_INST_ID);
            if (create_attr_ret){
                ESP_LOGE(GATTS_TABLE_TAG, "create attr table failed, error code = %x", create_attr_ret);
            }
        }
       	    break;
        case ESP_GATTS_READ_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_READ_EVT");
            // handle_read_ble_event(gatts_if, param);
       	    break;
        case ESP_GATTS_WRITE_EVT:
            if (!param->write.is_prep){
                xQueueSend(ble_data_queue, &param, portMAX_DELAY);
                // the data length of gattc write  must be less than GATTS_DEMO_CHAR_VAL_LEN_MAX.
                ESP_LOGI(GATTS_TABLE_TAG, "GATT_WRITE_EVT, handle = %d, value len = %d, value :%d", param->write.handle, param->write.len,param->write.value[0]);
                esp_log_buffer_hex(GATTS_TABLE_TAG, param->write.value, param->write.len);
                /* send response when param->write.need_rsp is true*/
                if (param->write.need_rsp){
                    ESP_LOGE(GATTS_TABLE_TAG, "need_rsp : send response");
                    esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
                }
            }else{
                /* handle prepare write */
                example_prepare_write_event_env(gatts_if, &prepare_write_env, param);
            }
      	    break;
        case ESP_GATTS_EXEC_WRITE_EVT:
            // the length of gattc prepare write data must be less than GATTS_DEMO_CHAR_VAL_LEN_MAX.
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_EXEC_WRITE_EVT");
            example_exec_write_event_env(&prepare_write_env, param);
            break;
        case ESP_GATTS_MTU_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
            break;
        case ESP_GATTS_CONF_EVT:
            //ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_CONF_EVT, status = %d, attr_handle %d", param->conf.status, param->conf.handle);
            break;
        case ESP_GATTS_START_EVT:
            ble_is_connected = 1;
            ESP_LOGI(GATTS_TABLE_TAG, "SERVICE_START_EVT, status %d, service_handle %d", param->start.status, param->start.service_handle);
            break;
        case ESP_GATTS_CONNECT_EVT:
            
            ble_is_connected = 0;
            conn_id = param->connect.conn_id;
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_CONNECT_EVT, conn_id = %d", param->connect.conn_id);
            esp_log_buffer_hex(GATTS_TABLE_TAG, param->connect.remote_bda, 6);
            esp_ble_conn_update_params_t conn_params = {0};
            memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            /* For the iOS system, please refer to Apple official documents about the BLE connection parameters restrictions. */
            conn_params.latency = 0;
            conn_params.max_int = 0x20;    // max_int = 0x20*1.25ms = 40ms
            conn_params.min_int = 0x10;    // min_int = 0x10*1.25ms = 20ms
            conn_params.timeout = 400;    // timeout = 400*10ms = 4000ms
            //start sent the update connection parameters to the peer device.
            esp_ble_gap_update_conn_params(&conn_params);
            break;
        case ESP_GATTS_DISCONNECT_EVT:
            ble_is_connected = 1;
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_DISCONNECT_EVT, reason = 0x%x", param->disconnect.reason);
            if (!is_ota_mode) {  // เช็คก่อนว่ากำลังจะเข้า OTA mode หรือไม่
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
        case ESP_GATTS_CREAT_ATTR_TAB_EVT:
            if (param->add_attr_tab.status != ESP_GATT_OK){
                ESP_LOGE(GATTS_TABLE_TAG, "create attribute table failed, error code=0x%x", param->add_attr_tab.status);
            }
            else if (param->add_attr_tab.num_handle != IWING_TRAINER_IDX_NB){
                ESP_LOGE(GATTS_TABLE_TAG, "create attribute table abnormally, num_handle (%d) \
                        doesn't equal to IWING_TRAINER_IDX_NB(%d)", param->add_attr_tab.num_handle, IWING_TRAINER_IDX_NB);
            }
            else {
                ESP_LOGI(GATTS_TABLE_TAG, "create attribute table successfully, the number handle = %d",param->add_attr_tab.num_handle);
                memcpy(iwing_training_table, param->add_attr_tab.handles, sizeof(iwing_training_table));
                ESP_LOGI(GATTS_TABLE_TAG, "memcpy OK");
                esp_ble_gatts_start_service(iwing_training_table[IDX_SVC]);
                ESP_LOGI(GATTS_TABLE_TAG, "Gatts_strat service OK");
            }   
            break;
        case ESP_GATTS_STOP_EVT:
        case ESP_GATTS_OPEN_EVT:
        case ESP_GATTS_CANCEL_OPEN_EVT:
        case ESP_GATTS_CLOSE_EVT:
        case ESP_GATTS_LISTEN_EVT:
        case ESP_GATTS_CONGEST_EVT:
        case ESP_GATTS_UNREG_EVT:
        case ESP_GATTS_DELETE_EVT:
        default:
            break;
    }
}
void update_advertising_data(uint16_t battery_voltage) {
    if(!is_ota_mode) {
        battery_voltage = battery_voltage * BATT_DIVIDED_FACTOR;
        manufacturer_data[3] = (battery_voltage >> 8) & 0xFF; 
        manufacturer_data[2] = battery_voltage & 0xFF;         
        adv_data.p_manufacturer_data = manufacturer_data;
        esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
        if (ret) {
            ESP_LOGE(GATTS_TABLE_TAG, "config adv data failed, error code = %x", ret);
        }
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{

    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            trainning_pad_profile_tab[PROFILE_APP_IDX].gatts_if = gatts_if;
        } else {
            ESP_LOGE(GATTS_TABLE_TAG, "reg app failed, app_id %04x, status %d",
                    param->reg.app_id,
                    param->reg.status);
            return;
        }
    }
    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
            if (gatts_if == ESP_GATT_IF_NONE || gatts_if == trainning_pad_profile_tab[idx].gatts_if) {
                if (trainning_pad_profile_tab[idx].gatts_cb) {
                    trainning_pad_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}

static void ble_send_notify(uint16_t handle, uint8_t *value, size_t len) {
    //esp_ble_gatts_send_indicate(gatt_if, conn_id, handle, len, value, false);
    if (is_ota_mode) {
        return; // ไม่ส่งข้อมูลถ้าอยู่ใน OTA mode
    }
    esp_err_t ret = esp_ble_gatts_send_indicate(gatt_if, conn_id, handle, len, value, false);
    if (ret != ESP_OK) {
        ESP_LOGE(ble_tag, "Failed to send indication: %s", esp_err_to_name(ret));
    }
    else {
        //ESP_LOGI(ble_tag, "Sent indication - handle: 0x%x, value: %d", handle, *value);
    }
}
// #################### END BLE ####################

static void update_global_data(uint16_t handle, uint8_t *value, size_t len) {
    xSemaphoreTake(g_ble_data_mutex, portMAX_DELAY);
    if (handle == iwing_training_table[IDX_CHAR_VIB_THRES_VAL] && len == 1) {
        g_vib_threshold = value[0];
    } else if (handle == iwing_training_table[IDX_CHAR_LED_VAL] && len == 3) {
        memcpy(g_led_color, value, 3);
    } else if (handle == iwing_training_table[IDX_CHAR_IR_TX_VAL] && len == 1) {
        g_ir_tx_state = value[0];
    } else if (handle == iwing_training_table[IDX_CHAR_MUSIC_VAL]) {
        memcpy(g_music_data, value, len);
        g_music_data_len = len;
    } else if (handle == iwing_training_table[IDX_CHAR_MODE] && len == 4) {
        memcpy(&g_mode[1], &value[1], 3); 
        // Mode D handling
        if(g_mode[1] == 1) { // ถ้า mode B = 1
            start_ota_mode();
        }
        if(g_mode[3] == 0) {
            // Normal mode
            //set_all_leds(0, 0, 0);
            ESP_LOGI(ble_tag, "Normal mode");
        } else if(g_mode[3] == 1) {
            // IR calibrate mode
            // LED handling in ir_task
        } else if(g_mode[3] == 2) {
            // Ultrasonic mode
            gpio_set_level(VIBRATION_EN_PIN, 0);
            if(g_mode[2] == 1) {
                // Set default distance logic here
                gpio_set_level(VIBRATION_EN_PIN, 1);
                vTaskDelay(pdMS_TO_TICKS(1000));
                gpio_set_level(VIBRATION_EN_PIN, 0);
                //printf("Distance: set\n");
                ESP_LOGI(ble_tag, "Distance: set");
            }
        } else if(g_mode[3] == 3) {
            // Press only mode
            // Button handling stays in button_task
            ESP_LOGI(ble_tag, "Press only mode");
        }
        esp_err_t ret;
        ret = esp_ble_gatts_set_attr_value(iwing_training_table[IDX_CHAR_MODE], sizeof(g_mode), (const uint8_t *)&g_mode);
        if (ret != ESP_OK) {
            ESP_LOGE(ble_tag, "Failed to set mode: %d", ret);
        }
        else {
            ESP_LOGI(ble_tag, "initial mode value set to : %02X", g_mode[0]);
        }
    }
    xSemaphoreGive(g_ble_data_mutex);
}
static void ble_data_task(void *pvParameter) {
    esp_ble_gatts_cb_param_t *param;
    for (;;) {
        if (is_ota_mode) {
            vTaskDelete(NULL); // ให้ task ยกเลิกตัวเอง
            return;
        }
        if (xQueueReceive(ble_data_queue, &param, portMAX_DELAY)) {
            if (is_ota_mode) continue;
            if (param->write.is_prep == false) {
                update_global_data(param->write.handle, param->write.value, param->write.len);
                
                if (param->write.handle == iwing_training_table[IDX_CHAR_VIB_THRES_VAL]) {
                    printf("New vibration threshold: %d\n", g_vib_threshold);
                    
                } else if (param->write.handle == iwing_training_table[IDX_CHAR_LED_VAL]) {
                    printf("New LED color: R:%d G:%d B:%d\n", g_led_color[0], g_led_color[1], g_led_color[2]);
                    set_all_leds(g_led_color[0], g_led_color[1], g_led_color[2]);
                } else if(param->write.handle == iwing_training_table[IDX_CHAR_IR_TX_VAL]) {
                    if(g_ir_tx_state == 0) {
                        ESP_ERROR_CHECK(ledc_set_duty(IR_PWM_MODE, IR_PWM_CHANNEL, 0));
                        ESP_ERROR_CHECK(ledc_update_duty(IR_PWM_MODE, IR_PWM_CHANNEL));
                        gpio_set_intr_type(IR_RECEIVER_PIN, GPIO_INTR_ANYEDGE);
                        printf("IR LED OFF\n");
                    } else {
                        ESP_ERROR_CHECK(ledc_set_duty(IR_PWM_MODE, IR_PWM_CHANNEL, IR_PWM_DUTY));
                        ESP_ERROR_CHECK(ledc_update_duty(IR_PWM_MODE, IR_PWM_CHANNEL));
                        gpio_set_intr_type(IR_RECEIVER_PIN, GPIO_INTR_DISABLE);
                        printf("IR LED ON\n");
                    }
                } else if(param->write.handle == iwing_training_table[IDX_CHAR_MUSIC_VAL]) {
                    printf("New music data\n");
                    for (int i = 0; i < g_music_data_len; i++) {
                        printf("%c ", (char)g_music_data[i]);
                    }
                    set_song();
                    printf("\n");
                } else if(param->write.handle == iwing_training_table[IDX_CHAR_MODE]) {
                    printf("New mode: A:0x%02X B:0x%02X C:0x%02X D:0x%02X\n",  g_mode[0], g_mode[1], g_mode[2], g_mode[3]);
                }
            }
        }
    }
}




static void check_efuse(void)
{

    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        
        ESP_LOGI(adc_tag, "eFuse Two Point: Supported");
    } else {
        //printf("eFuse Two Point: NOT supported\n");
        ESP_LOGI(adc_tag, "eFuse Two Point: NOT supported");
    }
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        //printf("eFuse Vref: Supported\n");
        ESP_LOGI(adc_tag, "eFuse Vref: Supported");
    } else {
        //printf("eFuse Vref: NOT supported\n");
        ESP_LOGI(adc_tag, "eFuse Vref: NOT supported");
    }

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


float exponential_smoothing(float prev_smooth, float new_value)
{
    return ALPHA * new_value + (1 - ALPHA) * prev_smooth;
}
void bat_percent_task(void *pvParameter)
{
    float smoothed_voltage = 750.0;
    int flag = 0;
    while (1) {
        uint32_t adc_reading = 0;
        for (int i = 0; i < NO_OF_SAMPLES; i++) {
            if (unit == ADC_UNIT_1) {
                adc_reading += adc1_get_raw((adc1_channel_t)percent_bat_channel);
            }
        }
        adc_reading /= NO_OF_SAMPLES;
        uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
        for(int i =0; i<5 ;i++ ){
            smoothed_voltage = exponential_smoothing(smoothed_voltage, voltage);
            if(flag == 0){
                vTaskDelay(pdMS_TO_TICKS(50));
            }
            else{
                vTaskDelay(pdMS_TO_TICKS(500));
            }
        }
        flag = 1;
        uint16_t data = (uint16_t)smoothed_voltage * BATT_DIVIDED_FACTOR;
        if(ble_is_connected == 1){
            update_advertising_data(data);
        }
        //set_batt_voltage((uint8_t)data, sizeof(data));
        uint8_t data_bytes[2];  // อาร์เรย์เพื่อเก็บข้อมูล 2 ไบต์
        data_bytes[0] = (uint8_t)(data & 0xFF);  // ส่วนต่ำ (ต่ำสุด 8 บิต)
        data_bytes[1] = (uint8_t)((data >> 8) & 0xFF);  // ส่วนสูง (สูงสุด 8 บิต)
        esp_ble_gatts_set_attr_value(iwing_training_table[IDX_CHAR_BATT_VOLTAGE_VAL], sizeof(data_bytes), data_bytes);
        ble_send_notify(iwing_training_table[IDX_CHAR_BATT_VOLTAGE_VAL], data_bytes, sizeof(data_bytes));
        //ESP_LOGI(adc_tag, " voltage : %d ,Smoothed Voltage: %.2fmV, data : %d error code : %x",voltage, smoothed_voltage,data,ret);
        //vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
void bat_status_task(void *pvParameter)
{
    uint8_t yes = 1;
    uint8_t no = 0;
    while (1) {
        uint32_t adc_reading = 0;
        for (int i = 0; i < NO_OF_SAMPLES; i++) {
            if (unit == ADC_UNIT_1) {
                adc_reading += adc1_get_raw((adc1_channel_t)status_bat_channel);
            }
        }
        adc_reading /= NO_OF_SAMPLES;
        uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
        if (voltage > 900) {
            //ESP_LOGI(adc_tag, "Battery Status: Fully Charged\tVoltage: %dmV", voltage);
            esp_ble_gatts_set_attr_value(iwing_training_table[IDX_CHAR_BATT_CHARGING_VAL], sizeof(no), &no);
            esp_ble_gatts_set_attr_value(iwing_training_table[IDX_CHAR_BATT_FULL_VAL], sizeof(yes), &yes);
            ble_send_notify(iwing_training_table[IDX_CHAR_BATT_CHARGING_VAL], &no, sizeof(no));
            ble_send_notify(iwing_training_table[IDX_CHAR_BATT_FULL_VAL], &yes, sizeof(yes));
            
        } else if (voltage > 700) {
            //ESP_LOGI(adc_tag, "Battery Status: Not Charged\tVoltage: %dmV", voltage);
            esp_ble_gatts_set_attr_value(iwing_training_table[IDX_CHAR_BATT_CHARGING_VAL], sizeof(no), &no);
            esp_ble_gatts_set_attr_value(iwing_training_table[IDX_CHAR_BATT_FULL_VAL], sizeof(no), &no);
            ble_send_notify(iwing_training_table[IDX_CHAR_BATT_CHARGING_VAL], &no, sizeof(no));
            ble_send_notify(iwing_training_table[IDX_CHAR_BATT_FULL_VAL], &no, sizeof(no));
        } else if (voltage < 500) {
            //ESP_LOGI(adc_tag, "Battery Status: Charging \tVoltage: %dmV", voltage);
            esp_ble_gatts_set_attr_value(iwing_training_table[IDX_CHAR_BATT_CHARGING_VAL], sizeof(yes), &yes);
            esp_ble_gatts_set_attr_value(iwing_training_table[IDX_CHAR_BATT_FULL_VAL], sizeof(no), &no);
            ble_send_notify(iwing_training_table[IDX_CHAR_BATT_CHARGING_VAL], &yes, sizeof(yes));
            ble_send_notify(iwing_training_table[IDX_CHAR_BATT_FULL_VAL], &no, sizeof(no));
        }
        //ESP_LOGI(adc_tag, "voltage \tVoltage: %dmV", voltage);
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void set_all_leds(uint8_t red, uint8_t green, uint8_t blue)
{
    for (int i = 0; i < CONFIG_EXAMPLE_STRIP_LED_NUMBER; i++) {
        ESP_ERROR_CHECK(strip->set_pixel(strip, i, red, green, blue));
    }
    ESP_ERROR_CHECK(strip->refresh(strip, 100));
}
static void breathing_led_task(void *pvParameter) {
    rgb_color_t *color = (rgb_color_t *)pvParameter;
    uint8_t led_intensity = LED_INTENSITY;
    uint8_t increasing = 0;
    
    while(1) {
        set_all_leds(
            (color->red * led_intensity) / LED_INTENSITY,
            (color->green * led_intensity) / LED_INTENSITY,
            (color->blue * led_intensity) / LED_INTENSITY
        );
        
        if(increasing) {
            led_intensity += LED_BREAHTING_STEP;
            if(led_intensity >= LED_INTENSITY) {
                increasing = 0;
            }
        } else {
            led_intensity -= LED_BREAHTING_STEP;
            if(led_intensity <= 0) {
                increasing = 1;
            }
        }
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

// ฟังก์ชันสำหรับเริ่ม breathing effect
void start_breathing_led(uint8_t red, uint8_t green, uint8_t blue) {
    // ถ้ามี task ทำงานอยู่ให้หยุดก่อน
    if(breathing_led_handle != NULL) {
        vTaskDelete(breathing_led_handle);
        breathing_led_handle = NULL;
    }
    
    // สร้าง structure สำหรับเก็บค่าสี
    rgb_color_t *color = malloc(sizeof(rgb_color_t));
    color->red = red;
    color->green = green;
    color->blue = blue;
    
    // สร้าง task ใหม่
    xTaskCreate(breathing_led_task, "breathing_led", 2048, color, 5, &breathing_led_handle);
}

// ฟังก์ชันสำหรับหยุด breathing effect
void stop_breathing_led() {
    if(breathing_led_handle != NULL) {
        vTaskDelete(breathing_led_handle);
        breathing_led_handle = NULL;
        set_all_leds(0, 0, 0);
    }
}
static void ble_disconnect_task(void *pvParameter)
{
    uint64_t last_connected_time=esp_timer_get_time() / 1000;
    uint64_t current_time; 
    while(1){
        if(ble_is_connected == 1){
            if(breathing_led_handle == NULL) {
                start_breathing_led(LED_INTENSITY, 0, 0);
            }
            current_time = esp_timer_get_time() / 1000;
            if(current_time - last_connected_time > BLE_TIMEOUT ){
                stop_breathing_led();
                sleep_call();
            }
        }else{
            if(g_mode[3]==4){
               if(breathing_led_handle == NULL) {
                    start_breathing_led(0, 0, LED_INTENSITY);
                }
            }
            else {
                stop_breathing_led();
                vTaskDelay(1000 / portTICK_PERIOD_MS);
            }
            last_connected_time = esp_timer_get_time() / 1000;
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint32_t gpio_num = (uint32_t) arg;
    uint8_t  level = gpio_get_level(gpio_num);
    if (gpio_num == BUTTON_PIN) {
        if(last_level != level){
            gpio_set_intr_type(BUTTON_PIN, GPIO_INTR_DISABLE);
            last_level = level;
            xQueueSendFromISR(button_evt_queue, &level, NULL);        
            if(level == 0){
                //gpio_set_intr_type(BUTTON_PIN, GPIO_INTR_NEGEDGE);
                vTaskNotifyGiveFromISR(Button_Handle, &xHigherPriorityTaskWoken);
            }
        }
        
    } 
    else if (gpio_num == IR_RECEIVER_PIN) {
        xQueueSendFromISR(ir_evt_queue, &level, NULL);
    }
    else if (gpio_num == VIBRATION_PIN) {
        if (level == 0) {
            if(g_mode[3] == 2){
                xQueueSendFromISR(ultrasonic_queue, &level, NULL); 
            }
            else if(g_mode[3] == 0){
                g_vibration_reading=1;
                g_vibration_count=1;
                vTaskNotifyGiveFromISR(Vibration_Timer_Handle,&xHigherPriorityTaskWoken);
            }
        }
        else if (level == 1) {
            if(g_mode[3] == 0){
                g_vibration_count++;
            }
        }
    } 
}
static void button_timer_task(void *pvParameter)
{
    uint8_t level;
    while (1) {
        uint32_t ulNotificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if(ulNotificationValue > 0){
            //uint64_t time1 = esp_timer_get_time()/1000;
            //ESP_LOGE(ble_tag, "Button pressed notify: %llu ", time1);
            vTaskDelay(DEBOUNCE_TIME_MS / portTICK_PERIOD_MS);
            //uint64_t time2 = esp_timer_get_time()/1000;
            //ESP_LOGE(ble_tag, "Button pressed notify: %llu : %llu",time2, time2-time1);
            //ESP_LOGI(ble_tag, "------------END----------------");
            level = gpio_get_level(BUTTON_PIN);
            if(level == 1){
                level = 2;
                xQueueSend(button_evt_queue, &level, portMAX_DELAY);
                gpio_set_intr_type(BUTTON_PIN, GPIO_INTR_NEGEDGE);
            }
            else {
                gpio_set_intr_type(BUTTON_PIN, GPIO_INTR_POSEDGE); 
            }
        }
    }
}
static void button_task(void* arg)
{
    //esp_err_t ret;
    uint8_t level;
    for(;;) {
        if(xQueueReceive(button_evt_queue, &level, portMAX_DELAY)) {
            // uint64_t time = esp_timer_get_time()/1000;
            // ESP_LOGE(ble_tag, "Button pressed : %d : %llu", level, time);
            // ESP_LOGI(ble_tag, "------------END----------------");
            if (level == 1) {
                if(g_mode[3] == 0){
                    ble_send_notify(iwing_training_table[IDX_CHAR_BUTTONS_VAL], &level, sizeof(level));
                }
                vTaskDelay(DEBOUNCE_TIME_MS / portTICK_PERIOD_MS); // prevent bouncing in rising edge
                gpio_set_intr_type(BUTTON_PIN, GPIO_INTR_NEGEDGE);
            }
            else if (level == 0) {
                ble_send_notify(iwing_training_table[IDX_CHAR_BUTTONS_VAL], &level, sizeof(level));
            }
            else if(level == 2){
                level = 1;
                last_level = 1;
                if(g_mode[3] == 0){
                    ble_send_notify(iwing_training_table[IDX_CHAR_BUTTONS_VAL], &level, sizeof(level));
                }
            }
            
        }
    }
}
static void ir_task(void* arg){
    esp_err_t ret;
    uint8_t level;
    for(;;) {
        if(xQueueReceive(ir_evt_queue, &level, portMAX_DELAY)) {
            //printf("GPIO[IR] intr, val: %d\n", level);
            //ESP_LOGE(ble_tag, "level g_mode  : %d    ,   %d", level,g_mode);
            if(level == 1) {
                level = 0;
                if(g_mode[3] == 1){
                    set_all_leds(0, 0, 0);
                }
            }else if(level == 0) {
                level = 1;
                if(g_mode[3] == 1){
                    set_all_leds(0, 255, 0);
                }
            }
            if(g_mode[3] == 0){
                ret=esp_ble_gatts_set_attr_value(iwing_training_table[IDX_CHAR_IR_RX_VAL], sizeof(level), &level);
                ble_send_notify(iwing_training_table[IDX_CHAR_IR_RX_VAL], &level, sizeof(level));
                if (ret != ESP_OK) {
                    ESP_LOGE(ble_tag, "Failed to set IR value : %d", ret);
                }
                else {
                    ESP_LOGI(ble_tag, "IR RX value set to : %d", level);
                }
            }
        }

    }
}
static void ultrasonic_task(void* arg)
{
    esp_err_t ret;
    uint8_t level;
    for(;;) {
        if(xQueueReceive(ultrasonic_queue, &level, portMAX_DELAY)) {
            if(level == 1) {
                level = 0;
            }else if(level == 0) {
                level = 1;
                //gpio_set_intr_type(VIBRATION_PIN, GPIO_INTR_POSEDGE);
                ret=esp_ble_gatts_set_attr_value(iwing_training_table[IDX_CHAR_VIBRATION_VAL], sizeof(level), &level);
                ble_send_notify(iwing_training_table[IDX_CHAR_VIBRATION_VAL], &level, sizeof(level));
                if (ret != ESP_OK) {
                    ESP_LOGE(ble_tag, "Failed to set sonic value : %d", ret);
                }
                else {
                    ESP_LOGI(ble_tag, "ultrasonic value set to : %d", level);
                }
            }
            
            
        }

    }
}
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
void play_note(int frequency, int duration_ms) {
    if (frequency > 0) {
        ESP_ERROR_CHECK(ledc_set_freq(BUZZER_MODE, BUZZER_TIMER, frequency));
        ESP_ERROR_CHECK(ledc_set_duty(BUZZER_MODE, BUZZER_CHANNEL, BUZZER_DUTY));
        ESP_ERROR_CHECK(ledc_update_duty(BUZZER_MODE, BUZZER_CHANNEL));
        vTaskDelay(duration_ms / portTICK_PERIOD_MS);
        ESP_ERROR_CHECK(ledc_set_duty(BUZZER_MODE, BUZZER_CHANNEL, 0));
        ESP_ERROR_CHECK(ledc_update_duty(BUZZER_MODE, BUZZER_CHANNEL));
    } else {
        vTaskDelay(duration_ms / portTICK_PERIOD_MS);
    }
}

int get_frequency(char note, char accidental)
{
    int base_index;
    switch (tolower(note)) {
        case 'c': base_index = 0; break;
        case 'd': base_index = 2; break;
        case 'e': base_index = 4; break;
        case 'f': base_index = 5; break;
        case 'g': base_index = 7; break;
        case 'a': base_index = 9; break;
        case 'b': base_index = 11; break;
        default: return 0;
    }

    if (accidental == '+' || accidental == '#') base_index++;
    else if (accidental == '-') base_index--;

    int frequency = base_frequencies[base_index % 12];
    for (int i = 4; i < current_octave; i++) frequency *= 2;
    for (int i = 4; i > current_octave; i--) frequency /= 2;
    
    return frequency;
}

void reset_mml_state(void)
{
    current_octave = DEFAULT_OCTAVE;
    default_duration = DEFAULT_DURATION;
    tempo = DEFAULT_TEMPO;
}

void play_mml(const char* mml)
{

    reset_mml_state();  // Reset state before playing

    char note, accidental;
    int duration, dot;
    const char* p = mml;

    while (*p) {
        note = *p++;
        accidental = 0;
        duration = default_duration;
        dot = 0;

        //printf("note: %c\n", note);

        if (*p == '+' || *p == '#' || *p == '-') {
            accidental = *p++;
        }

        if (note == 't' && isdigit(*p)) {  // Change tempo
            tempo = 0;
            while (isdigit(*p)) {
                tempo = tempo * 10 + (*p++ - '0');
            }
            //printf("Tempo changed to: %d\n", tempo);
            continue;
        }

        if (isdigit(*p)) {
            duration = 0;
            while (isdigit(*p)) {
                duration = duration * 10 + (*p++ - '0');
            }
        }

        if (*p == '.') {
            dot = 1;
            p++;
        }

        int note_duration = (60000 / tempo) * (4.0 / duration);
        if (dot) note_duration += note_duration / 2;

        switch (tolower(note)) {
            case 'c':
            case 'd':
            case 'e':
            case 'f':
            case 'g':
            case 'a':
            case 'b':
                play_note(get_frequency(note, accidental), note_duration);
                break;
            case 'p':
            case 'r':
                play_note(0, note_duration);
                break;
            case 'o':
                if (isdigit(*p)) {
                    current_octave = *p++ - '0';
                    if (current_octave < MIN_OCTAVE) current_octave = MIN_OCTAVE;
                    if (current_octave > MAX_OCTAVE) current_octave = MAX_OCTAVE;
                }
                break;
            case '>':
                if (current_octave < MAX_OCTAVE) current_octave++;
                break;
            case '<':
                if (current_octave > MIN_OCTAVE) current_octave--;
                break;
            case 'l':
                if (isdigit(*p)) {
                    default_duration = 0;
                    while (isdigit(*p)) {
                        default_duration = default_duration * 10 + (*p++ - '0');
                    }
                }
                break;
        }
    }
}
void song_task(void* pvParameters)
{
    while(1) {
       if (strlen(current_song) > 0) {  // เช็คว่ามีเพลงอยู่ใน buffer หรือไม่
            play_mml(current_song);
            vTaskDelay(2000 / portTICK_PERIOD_MS); // รอ 2 วินาทีก่อนเล่นซ้ำ
        } else {
            vTaskDelay(100 / portTICK_PERIOD_MS); // ถ้าไม่มีเพลง รอสักครู่แล้วเช็คใหม่
        }
    }
}
void set_song() {
    char new_song[g_music_data_len];
    for(int i =0;i<g_music_data_len;i++){
        new_song[i] = (char)g_music_data[i];
    }
    strncpy(current_song, new_song, MAX_SONG_LENGTH - 1);
    current_song[MAX_SONG_LENGTH - 1] = '\0';  // ป้องกันการ overflow
}

// ฟังก์ชันสำหรับหยุดเพลง
void stop_song(void) {
    current_song[0] = '\0';  // เคลียร์เพลงโดยการตั้งค่าสตริงว่าง
    // หยุดเสียงที่กำลังเล่นอยู่
    ESP_ERROR_CHECK(ledc_set_duty(BUZZER_MODE, BUZZER_CHANNEL, 0));
    ESP_ERROR_CHECK(ledc_update_duty(BUZZER_MODE, BUZZER_CHANNEL));
}

void ledc_init(void)
{
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = IR_PWM_MODE,
        .timer_num        = IR_PWM_TMER,
        .duty_resolution  = IR_PWM_DUTY_RES,
        .freq_hz          = IR_PWM_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = IR_PWM_MODE,
        .channel        = IR_PWM_CHANNEL,
        .timer_sel      = IR_PWM_TMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = IR_LED_PIN,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}
static void buzzer_init(void) {
    ledc_timer_config_t buzzer_timer = {
        .speed_mode       = BUZZER_MODE,
        .timer_num        = BUZZER_TIMER,
        .duty_resolution  = BUZZER_DUTY_RES,
        .freq_hz          = 440,  // Default frequency (A4)
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&buzzer_timer));

    ledc_channel_config_t buzzer_channel = {
        .speed_mode     = BUZZER_MODE,
        .channel        = BUZZER_CHANNEL,
        .timer_sel      = BUZZER_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = BUZZER_OUTPUT_IO,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&buzzer_channel));
}
void rgb_led_init(void){
    rmt_config_t config = RMT_DEFAULT_CONFIG_TX(CONFIG_EXAMPLE_RMT_TX_GPIO, RMT_TX_CHANNEL);
    config.clk_div = 2;
    ESP_ERROR_CHECK(rmt_config(&config));
    ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));
    led_strip_config_t strip_config = LED_STRIP_DEFAULT_CONFIG(CONFIG_EXAMPLE_STRIP_LED_NUMBER, (led_strip_dev_t)config.channel);
    strip = led_strip_new_rmt_ws2812(&strip_config);
    if (!strip) {
        ESP_LOGE(led_tag, "install WS2812 driver failed");
    }
    ESP_ERROR_CHECK(strip->clear(strip, 100));
}
void gpio_led_init(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << RMT_TX_GPIO),
        .mode = GPIO_MODE_OUTPUT_OD,        // ตั้งเป็น open-drain mode
        .pull_up_en = GPIO_PULLUP_DISABLE,  
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
}

void gpio_init(void){
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.pin_bit_mask = GPIO_INPUT_IR_PIN_SEL ;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.pin_bit_mask = GPIO_INPUT_BUTTON_PIN_SEL ;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.pin_bit_mask = GPIO_INPUT_VIBRATION_PIN_SEL ;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    io_conf.intr_type = GPIO_INTR_DISABLE;  // Outputs typically don't need interrupts
    io_conf.pin_bit_mask = GPIO_SONIC_PIN_SEL; // Define GPIO_OUTPUT_PIN_SEL for your pin
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE; // No pull-up needed for output
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE; // No pull-down needed for output
    gpio_config(&io_conf);

    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(BUTTON_PIN, gpio_isr_handler, (void*) BUTTON_PIN);
    gpio_isr_handler_add(IR_RECEIVER_PIN, gpio_isr_handler, (void*) IR_RECEIVER_PIN);
    gpio_isr_handler_add(VIBRATION_PIN, gpio_isr_handler, (void*) VIBRATION_PIN);
}
void adc_init(){
    check_efuse();
    if (unit == ADC_UNIT_1) {
        adc1_config_width(width);
        adc1_config_channel_atten(percent_bat_channel, atten);
        adc1_config_channel_atten(status_bat_channel, atten);
    }
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, width, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);
}
void led_power_off(void) {
    ESP_ERROR_CHECK(strip->clear(strip, 100));
    ESP_ERROR_CHECK(rmt_driver_uninstall(RMT_TX_CHANNEL));
    // gpio_led_init();
    // gpio_set_level(RMT_TX_GPIO, 0);

}

void sleep_call(){
    if(is_ota_mode) {
        // ถ้าอยู่ใน OTA mode จะไม่เข้า deep sleep
        return;
    }
    led_power_off();
    ESP_ERROR_CHECK(esp_sleep_enable_ext0_wakeup(BUTTON_PIN, 0));
    ESP_ERROR_CHECK(rtc_gpio_pulldown_dis(BUTTON_PIN));
    ESP_ERROR_CHECK(rtc_gpio_pullup_en(BUTTON_PIN));
    esp_deep_sleep_start();
}
void start_ota_mode() {
    is_ota_mode = true;  // set flag ว่าอยู่ใน OTA mode
    vTaskDelay(100 / portTICK_PERIOD_MS);
    // Stop BLE 
    esp_bluedroid_disable();
    vTaskDelay(100 / portTICK_PERIOD_MS);
    esp_bluedroid_deinit();
    vTaskDelay(100 / portTICK_PERIOD_MS);
    esp_bt_controller_disable();
    vTaskDelay(100 / portTICK_PERIOD_MS);
    esp_bt_controller_deinit();
    vTaskDelay(100 / portTICK_PERIOD_MS);
    esp_bt_controller_mem_release(ESP_BT_MODE_BLE);
    // Start OTA
    ESP_ERROR_CHECK(softap_init());
    ESP_ERROR_CHECK(http_server_init());
    
    printf("OTA Update Mode Started\n");
    start_breathing_led(0, LED_INTENSITY, 0);
    while(1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

}


void app_main(void)
{
    //##################### BLE #####################
    esp_err_t ret;

    /* Initialize NVS. */
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret){
        ESP_LOGE(GATTS_TABLE_TAG, "gatts register error, error code = %x", ret);
        return;
    }

    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret){
        ESP_LOGE(GATTS_TABLE_TAG, "gap register error, error code = %x", ret);
        return;
    }

    ret = esp_ble_gatts_app_register(ESP_APP_ID);
    if (ret){
        ESP_LOGE(GATTS_TABLE_TAG, "gatts app register error, error code = %x", ret);
        return;
    }

    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret){
        ESP_LOGE(GATTS_TABLE_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    }
    //##################### END BLE #################
    switch (esp_sleep_get_wakeup_cause()) {
        case ESP_SLEEP_WAKEUP_EXT0: {
            printf("Wake up from EXT0\n");
            rtc_gpio_deinit(BUTTON_PIN);
            break;
        }
        case ESP_SLEEP_WAKEUP_UNDEFINED:
        default:
            printf("Not a deep sleep reset\n");
    }
    ret = esp_ble_gatts_set_attr_value(iwing_training_table[IDX_CHAR_MODE], sizeof(g_mode), g_mode);
    if (ret != ESP_OK) {
        ESP_LOGE(ble_tag, "Failed to set mode: %d", ret);
    }
    else {
        ESP_LOGI(ble_tag, "initial mode value set to : %02X", g_mode[0]);
    }
    rgb_led_init();
    ledc_init();
    gpio_init();
    adc_init();
    buzzer_init();
    //ble_data_inint();
    
    g_ble_data_mutex = xSemaphoreCreateMutex();

    button_evt_queue = xQueueCreate(10, sizeof(uint8_t));
    ir_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    ultrasonic_queue = xQueueCreate(10, sizeof(uint32_t));
    ble_data_queue = xQueueCreate(20, sizeof(esp_ble_gatts_cb_param_t *));
    ble_status_queue = xQueueCreate(10, sizeof(uint8_t));   

    //mode_queue = xQueueCreate(10, sizeof(uint8_t));

    xTaskCreate(button_task, "button_task", 2048, NULL, 10, NULL);
    xTaskCreate(button_timer_task, "button_timer_task", 2048, NULL, 10, &Button_Handle);
    xTaskCreate(ir_task, "ir_task", 2048, NULL, 10, NULL);
    xTaskCreate(vibration_task, "vibration_task", 2048, NULL, 16, &Vibration_Handle);
    xTaskCreate(vibration_timer, "vibration_timer", 2048, NULL, 16, &Vibration_Timer_Handle);
    xTaskCreate(ultrasonic_task, "ultrasonic_task", 2048, NULL, 10, NULL);

    xTaskCreate(bat_percent_task, "bat_percent_task", 2048, NULL, 5, NULL);
    xTaskCreate(bat_status_task, "bat_status_task", 2048, NULL, 5, NULL);
    xTaskCreate(ble_data_task, "ble_data_task", 4096, NULL, 10, &ble_data_task_handle);
    xTaskCreate(ble_disconnect_task, "ble_disconnect_task", 2048, NULL, 5, NULL);
    xTaskCreate(song_task, "song_task", 2048, NULL, 5, NULL);
    //xTaskCreate(calibrate_task, "calibrate_task", 2048, NULL, 10, NULL);
    const esp_partition_t *partition = esp_ota_get_running_partition();
	printf("Currently running partition: %s\r\n", partition->label);

	esp_ota_img_states_t ota_state;
	if (esp_ota_get_state_partition(partition, &ota_state) == ESP_OK) {
		if (ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
			esp_ota_mark_app_valid_cancel_rollback();
		}
	}
    while(1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}