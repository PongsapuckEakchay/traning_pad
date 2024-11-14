/*
 * SPDX-FileCopyrightText: 2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */


#include <stdio.h>
#include <stdlib.h>
#include <string.h>


/* Attributes State Machine */
enum
{
    IDX_SVC,                          // Service Declaration

    IDX_CHAR_BATT_VOLTAGE_DECL,       // Characteristic Declaration for BATT_VOLTAGE
    IDX_CHAR_BATT_VOLTAGE_VAL,         // Characteristic Value for BATT_VOLTAGE

    IDX_CHAR_BATT_CHARGING_DECL,       // Characteristic Declaration for BATT_CHARGING
    IDX_CHAR_BATT_CHARGING_VAL,        // Characteristic Value for BATT_CHARGING

    IDX_CHAR_BATT_FULL_DECL,           // Characteristic Declaration for BATT_FULL
    IDX_CHAR_BATT_FULL_VAL,            // Characteristic Value for BATT_FULL

    IDX_CHAR_BUTTONS_DECL,             // Characteristic Declaration for BUTTONS
    IDX_CHAR_BUTTONS_VAL,              // Characteristic Value for BUTTONS

    IDX_CHAR_VIBRATION_DECL,           // Characteristic Declaration for VIBRATION
    IDX_CHAR_VIBRATION_VAL,            // Characteristic Value for VIBRATION

    IDX_CHAR_IR_RX_DECL,               // Characteristic Declaration for IR_RX
    IDX_CHAR_IR_RX_VAL,                // Characteristic Value for IR_RX

    IDX_CHAR_VIB_THRES_DECL,           // Characteristic Declaration for VIB_THRES
    IDX_CHAR_VIB_THRES_VAL,            // Characteristic Value for VIB_THRES

    IDX_CHAR_LED_DECL,                 // Characteristic Declaration for LED
    IDX_CHAR_LED_VAL,                  // Characteristic Value for LED

    IDX_CHAR_IR_TX_DECL,               // Characteristic Declaration for IR_TX
    IDX_CHAR_IR_TX_VAL,                // Characteristic Value for IR_TX

    IDX_CHAR_MUSIC_DECL,               // Characteristic Declaration for MUSIC
    IDX_CHAR_MUSIC_VAL,                // Characteristic Value for MUSIC

    IDX_CHAR_MODE_DECL,                // Characteristic Declaration for MODE
    IDX_CHAR_MODE,                     // Characteristic Value for MODE

    IWING_TRAINER_IDX_NB               // Number of attributes
};
