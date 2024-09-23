# IWING Training Pad

## Overview

The IWING Trainer Pad is an ESP32-based device that incorporates various sensors and actuators, managed through a custom BLE service. This project is designed for interactive training applications, featuring vibration detection, button input, IR communication, RGB LED control, and battery management.

## Hardware Setup

Connect the following components to your ESP32:

1. Button: GPIO15
2. Vibration Sensor: GPIO21
3. IR Receiver: GPIO17
4. IR LED Transmitter: GPIO16
5. RGB LED Strip (WS2812): GPIO19
6. Battery Voltage Sensor: GPIO34 (ADC1 Channel 6)
7. Battery Charging Status: GPIO35 (ADC1 Channel 7)

## BLE Service: IWING_TRAINER

UUID: B2E9FDA1-822C-4729-B8E2-9C35E7630001

### Characteristics

1. BATT_VOLTAGE (UUID: ...0002)
   - Battery voltage level (mV)
   - Read Only

2. BATT_CHARGING (UUID: ...0003)
   - Battery charging status (1 = charging)
   - Read Only

3. BATT_FULL (UUID: ...0004)
   - Battery fully charged status (1 = full)
   - Read Only

4. BUTTONS (UUID: ...0005)
   - Button status, 1 bit per button (1 = pressed)
   - Read Only

5. VIBRATION (UUID: ...0006)
   - Vibration sensor status, 1 byte per sensor (255 = maximum movement)
   - Read Only

6. IR_RX (UUID: ...0007)
   - IR Receiver status (1 = signal received)
   - Read Only

7. VIB_THRES (UUID: ...0008)
   - Threshold for vibration sensor (8-bit)
   - Read/Write

8. LED (UUID: ...0009)
   - RGB LED color, 3 bytes per LED (RGB x 8-bit)
   - Read/Write

9. IR_TX (UUID: ...000A)
   - IR Transmitter status (1 = enabled)
   - Read/Write

10. MUSIC (UUID: ...000B)
    - Music notes in Music Macro Language format
    - Read/Write

## Getting Started

1. Install the ESP-IDF development framework.
2. Clone this repository.
3. Configure the project: `idf.py menuconfig`
4. Build the project: `idf.py build`
5. Flash to your ESP32: `idf.py -p PORT flash`

## Usage

After flashing, the device will start advertising its BLE service. Connect to the device using a BLE client to interact with the various characteristics for monitoring and control.

## Contributing

We welcome contributions to improve the IWING Trainer Pad. Please submit pull requests or open issues on our GitHub repository.

## License


## Contact

