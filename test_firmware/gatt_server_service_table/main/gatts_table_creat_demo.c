/*
 * SPDX-FileCopyrightText: 2021-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

/****************************************************************************
*
* This demo showcases creating a GATT database using a predefined attribute table.
* It acts as a GATT server and can send adv data, be connected by client.
* Run the gatt_client demo, the client demo will automatically connect to the gatt_server_service_table demo.
* Client demo will enable GATT server's notify after connection. The two devices will then exchange
* data.
*
****************************************************************************/


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

#include "driver/gpio.h"

#define GATTS_TABLE_TAG "GATTS_TABLE_DEMO"

#define PROFILE_NUM                 1
#define PROFILE_APP_IDX             0
#define ESP_APP_ID                  0x55
#define SAMPLE_DEVICE_NAME          "Trainning_PAD"
#define SVC_INST_ID                 0

/* The max length of characteristic value. When the GATT client performs a write or prepare write operation,
*  the data length must be less than GATTS_DEMO_CHAR_VAL_LEN_MAX.
*/
#define GATTS_DEMO_CHAR_VAL_LEN_MAX 500
#define PREPARE_BUF_MAX_SIZE        1024
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
    .manufacturer_len    = 0,    //TEST_MANUFACTURER_DATA_LEN,
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


static const uint16_t primary_service_uuid         = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid   = ESP_GATT_UUID_CHAR_DECLARE;
// static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static const uint8_t char_prop_read                =  ESP_GATT_CHAR_PROP_BIT_READ;
// static const uint8_t char_prop_write               = ESP_GATT_CHAR_PROP_BIT_WRITE;
static const uint8_t char_prop_read_write_notify   = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
// static const uint8_t heart_measurement_ccc[2]      = {0x00, 0x00};
static const uint8_t char_value[4]                 = {0x11, 0x22, 0x33, 0x44};

static uint8_t button_state = 1;
static uint8_t led_state = 0;


/* Full Database Description - Used to add attributes into the database */
static const esp_gatts_attr_db_t gatt_db[IWING_TRAINER_IDX_NB] =
{
    // Service Declaration for IWING_TRAINER (UUID: B2E9FDA1-822C-4729-B8E2-9C35E7630001)
    [IDX_SVC]        =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
      sizeof(uint16_t), ESP_UUID_LEN_128, (uint8_t *)&GATTS_SERVICE_UUID_TEST}},

    /* Characteristic Declaration for BATT_VOLTAGE (UUID: B2E9FDA1-822C-4729-B8E2-9C35E7630002) */
    [IDX_CHAR_BATT_VOLTAGE_DECL]     =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read}},

    /* Characteristic Value for BATT_VOLTAGE */
    [IDX_CHAR_BATT_VOLTAGE_VAL] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)&GATTS_CHAR_UUID_BATT_VOLTAGE, ESP_GATT_PERM_READ,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(button_state), (uint8_t *)&button_state}}, // RO: ระดับแรงดันแบตเตอรี่ (mV)

    /* Characteristic Declaration for BATT_CHARGING (UUID: B2E9FDA1-822C-4729-B8E2-9C35E7630003) */
    [IDX_CHAR_BATT_CHARGING_DECL]     =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read}},

    /* Characteristic Value for BATT_CHARGING */
    [IDX_CHAR_BATT_CHARGING_VAL] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)&GATTS_CHAR_UUID_BATT_CHARGING, ESP_GATT_PERM_READ,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}}, // RO: สถานะการเสียบชาร์จแบตเตอรี่ (1 = charging)

    /* Characteristic Declaration for BATT_FULL (UUID: B2E9FDA1-822C-4729-B8E2-9C35E7630004) */
    [IDX_CHAR_BATT_FULL_DECL]     =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read}},

    /* Characteristic Value for BATT_FULL */
    [IDX_CHAR_BATT_FULL_VAL] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)&GATTS_CHAR_UUID_BATT_FULL, ESP_GATT_PERM_READ,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}}, // RO: สถานะแบตเตอรี่ชาร์จเต็ม (1 = full)

    /* Characteristic Declaration for BUTTONS (UUID: B2E9FDA1-822C-4729-B8E2-9C35E7630005) */
    [IDX_CHAR_BUTTONS_DECL]     =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read}},

    /* Characteristic Value for BUTTONS */
    [IDX_CHAR_BUTTONS_VAL] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)&GATTS_CHAR_UUID_BUTTONS, ESP_GATT_PERM_READ,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}}, // RO: สถานะปุ่มกด บิตละ 1 ปุ่ม (1 = กด)

    /* Characteristic Declaration for VIBRATION (UUID: B2E9FDA1-822C-4729-B8E2-9C35E7630006) */
    [IDX_CHAR_VIBRATION_DECL]     =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read}},

    /* Characteristic Value for VIBRATION */
    [IDX_CHAR_VIBRATION_VAL] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)&GATTS_CHAR_UUID_VIBRATION, ESP_GATT_PERM_READ,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}}, // RO: สถานะ vibration sensor ไบต์ละ 1 ตัว (255 = เคลื่อนไหวแรงสุด)

    /* Characteristic Declaration for IR_RX (UUID: B2E9FDA1-822C-4729-B8E2-9C35E7630007) */
    [IDX_CHAR_IR_RX_DECL]     =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read}},

    /* Characteristic Value for IR_RX */
    [IDX_CHAR_IR_RX_VAL] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)&GATTS_CHAR_UUID_IR_RX, ESP_GATT_PERM_READ,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}}, // RO: สถานะ IR Receiver (1 = ได้รับสัญญาณ)

    /* Characteristic Declaration for VIB_THRES (UUID: B2E9FDA1-822C-4729-B8E2-9C35E7630008) */
    [IDX_CHAR_VIB_THRES_DECL]     =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

    /* Characteristic Value for VIB_THRES */
    [IDX_CHAR_VIB_THRES_VAL] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)&GATTS_CHAR_UUID_VIB_THRES, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(led_state), (uint8_t *)&led_state}}, // RW: threshold สำหรับ vibration sensor (8 บิต)

    /* Characteristic Declaration for LED (UUID: B2E9FDA1-822C-4729-B8E2-9C35E7630009) */
    [IDX_CHAR_LED_DECL]     =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

    /* Characteristic Value for LED */
    [IDX_CHAR_LED_VAL] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)&GATTS_CHAR_UUID_LED, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}}, // RW: สีของ RGB LED ดวงละ 3 ไบต์ (RGB x 8 บิต)

    /* Characteristic Declaration for IR_TX (UUID: B2E9FDA1-822C-4729-B8E2-9C35E763000A) */
    [IDX_CHAR_IR_TX_DECL]     =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

    /* Characteristic Value for IR_TX */
    [IDX_CHAR_IR_TX_VAL] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)&GATTS_CHAR_UUID_IR_TX, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}}, // RW: สถานะ IR Transmitter (1 = เปิดใช้งาน)

    /* Characteristic Declaration for MUSIC (UUID: B2E9FDA1-822C-4729-B8E2-9C35E763000B) */
    [IDX_CHAR_MUSIC_DECL]     =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

    /* Characteristic Value for MUSIC */
    [IDX_CHAR_MUSIC_VAL] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)&GATTS_CHAR_UUID_MUSIC, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}}, // RW: โน้ตเพลงในรูป Music Macro Language
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
            if (adv_config_done == 0){
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
        case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
            adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
            if (adv_config_done == 0){
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
                // the data length of gattc write  must be less than GATTS_DEMO_CHAR_VAL_LEN_MAX.
                ESP_LOGI(GATTS_TABLE_TAG, "GATT_WRITE_EVT, handle = %d, value len = %d, value :", param->write.handle, param->write.len);
                esp_log_buffer_hex(GATTS_TABLE_TAG, param->write.value, param->write.len);
                // For NOTIFY
                // if (iwing_training_table[IDX_CHAR_CFG_A] == param->write.handle && param->write.len == 2){
                //     uint16_t descr_value = param->write.value[1]<<8 | param->write.value[0];
                //     if (descr_value == 0x0001){
                //         ESP_LOGI(GATTS_TABLE_TAG, "notify enable");
                //         uint8_t notify_data[15];
                //         for (int i = 0; i < sizeof(notify_data); ++i)
                //         {
                //             notify_data[i] = i % 0xff;
                //         }
                //         //the size of notify_data[] need less than MTU size
                //         esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, iwing_training_table[IDX_CHAR_VAL_A],
                //                                 sizeof(notify_data), notify_data, false);
                //     }else if (descr_value == 0x0002){
                //         ESP_LOGI(GATTS_TABLE_TAG, "indicate enable");
                //         uint8_t indicate_data[15];
                //         for (int i = 0; i < sizeof(indicate_data); ++i)
                //         {
                //             indicate_data[i] = i % 0xff;
                //         }

                //         // if want to change the value in server database, call:
                //         // esp_ble_gatts_set_attr_value(iwing_training_table[IDX_CHAR_VAL_A], sizeof(indicate_data), indicate_data);


                //         //the size of indicate_data[] need less than MTU size
                //         esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, iwing_training_table[IDX_CHAR_VAL_A],
                //                             sizeof(indicate_data), indicate_data, true);
                //     }
                //     else if (descr_value == 0x0000){
                //         ESP_LOGI(GATTS_TABLE_TAG, "notify/indicate disable ");
                //     }else{
                //         ESP_LOGE(GATTS_TABLE_TAG, "unknown descr value");
                //         esp_log_buffer_hex(GATTS_TABLE_TAG, param->write.value, param->write.len);
                //     }

                // }
                // if (param->write.handle == iwing_training_table[IDX_CHAR_VIB_THRES_VAL]){
                //     led_state = param->write.value[0];
                //     ESP_LOGI(GATTS_TABLE_TAG, "write LED %d", led_state);
                // }
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
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_CONF_EVT, status = %d, attr_handle %d", param->conf.status, param->conf.handle);
            break;
        case ESP_GATTS_START_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "SERVICE_START_EVT, status %d, service_handle %d", param->start.status, param->start.service_handle);
            break;
        case ESP_GATTS_CONNECT_EVT:
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
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_DISCONNECT_EVT, reason = 0x%x", param->disconnect.reason);
            esp_ble_gap_start_advertising(&adv_params);
            break;
        case ESP_GATTS_CREAT_ATTR_TAB_EVT:{
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
        }
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

void set_batt_voltage(uint8_t* data, uint16_t length) {
    esp_ble_gatts_set_attr_value(iwing_training_table[IDX_CHAR_BATT_VOLTAGE_VAL], length, data);
}

esp_err_t get_batt_voltage(uint16_t* length, uint8_t** data) {
    return esp_ble_gatts_get_attr_value(iwing_training_table[IDX_CHAR_BATT_VOLTAGE_VAL], length, data);
}

void set_batt_charging(uint8_t* data, uint16_t length) {
    esp_ble_gatts_set_attr_value(iwing_training_table[IDX_CHAR_BATT_CHARGING_VAL], length, data);
}

esp_err_t get_batt_charging(uint16_t* length, uint8_t** data) {
    return esp_ble_gatts_get_attr_value(iwing_training_table[IDX_CHAR_BATT_CHARGING_VAL], length, data);
}

void set_batt_full(uint8_t* data, uint16_t length) {
    esp_ble_gatts_set_attr_value(iwing_training_table[IDX_CHAR_BATT_FULL_VAL], length, data);
}

esp_err_t get_batt_full(uint16_t* length, uint8_t** data) {
    return esp_ble_gatts_get_attr_value(iwing_training_table[IDX_CHAR_BATT_FULL_VAL], length, data);
}

void set_buttons(uint8_t* data, uint16_t length) {
    esp_ble_gatts_set_attr_value(iwing_training_table[IDX_CHAR_BUTTONS_VAL], length, data);
}

esp_err_t get_buttons(uint16_t* length, uint8_t** data) {
    return esp_ble_gatts_get_attr_value(iwing_training_table[IDX_CHAR_BUTTONS_VAL], length, data);
}

void set_vibration(uint8_t* data, uint16_t length) {
    esp_ble_gatts_set_attr_value(iwing_training_table[IDX_CHAR_VIBRATION_VAL], length, data);
}

esp_err_t get_vibration(uint16_t* length, uint8_t** data) {
    return esp_ble_gatts_get_attr_value(iwing_training_table[IDX_CHAR_VIBRATION_VAL], length, data);
}

void set_ir_rx(uint8_t* data, uint16_t length) {
    esp_ble_gatts_set_attr_value(iwing_training_table[IDX_CHAR_IR_RX_VAL], length, data);
}

esp_err_t get_ir_rx(uint16_t* length, uint8_t** data) {
    return esp_ble_gatts_get_attr_value(iwing_training_table[IDX_CHAR_IR_RX_VAL], length, data);
}

void set_vib_thres(uint8_t* data, uint16_t length) {
    esp_ble_gatts_set_attr_value(iwing_training_table[IDX_CHAR_VIB_THRES_VAL], length, data);
}

esp_err_t get_vib_thres(uint16_t* length, uint8_t** data) {
    return esp_ble_gatts_get_attr_value(iwing_training_table[IDX_CHAR_VIB_THRES_VAL], length, data);
}

void set_led(uint8_t* data, uint16_t length) {
    esp_ble_gatts_set_attr_value(iwing_training_table[IDX_CHAR_LED_VAL], length, data);
}

esp_err_t get_led(uint16_t* length, uint8_t** data) {
    return esp_ble_gatts_get_attr_value(iwing_training_table[IDX_CHAR_LED_VAL], length, data);
}

void set_ir_tx(uint8_t* data, uint16_t length) {
    esp_ble_gatts_set_attr_value(iwing_training_table[IDX_CHAR_IR_TX_VAL], length, data);
}

esp_err_t get_ir_tx(uint16_t* length, uint8_t** data) {
    return esp_ble_gatts_get_attr_value(iwing_training_table[IDX_CHAR_IR_TX_VAL], length, data);
}

void set_music(uint8_t* data, uint16_t length) {
    esp_ble_gatts_set_attr_value(iwing_training_table[IDX_CHAR_MUSIC_VAL], length, data);
}

esp_err_t get_music(uint16_t* length, uint8_t** data) {
    return esp_ble_gatts_get_attr_value(iwing_training_table[IDX_CHAR_MUSIC_VAL], length, data);
}

void app_main(void)
{
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
}