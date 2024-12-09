# Training Pad Project

A firmware project for Training Pad device developed using ESP-IDF v4.4.5 on ESP32.

## Features
- Bluetooth Low Energy (BLE) connectivity
- Vibration detection
- Button input handling
- IR communication (transmitter and receiver)
- RGB LED control 
- Music playback via buzzer
- Battery monitoring
- Power saving with deep sleep mode

## Prerequisites
- ESP-IDF v4.4.5
- Compatible ESP32 development board
- LED strip (WS2812)
- IR LED and receiver
- Vibration sensor
- Push button
- Buzzer
- Battery monitoring circuit

## Getting Started

1. Install ESP-IDF v4.4.5
```bash
git clone -b v4.4.5 --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
./install.sh
. ./export.sh
```

2. Clone this repository
```bash
git clone [repository-url]
cd training-pad
```

3. Configure the project
```bash
idf.py menuconfig
```
- Set WiFi credentials if needed
- Configure pin assignments
- Adjust other parameters as needed

4. Build and flash
```bash 
idf.py build
idf.py -p [PORT] flash monitor
```

## Hardware Setup
- Connect RGB LED strip to GPIO19
- Connect IR LED to GPIO16
- Connect IR receiver to GPIO17 
- Connect push button to GPIO15
- Connect vibration sensor to GPIO21
- Connect vibration enable pin to GPIO32
- Connect buzzer to GPIO2
- Connect battery voltage divider to GPIO34
- Connect battery status to GPIO35

## BLE Services
The device advertises as "Training_PAD" and provides following services:
- Battery monitoring (voltage, charging status)  
- Button state
- Vibration detection
- IR communication
- LED control
- Music playback
- Mode control

## Power Management
- Device enters deep sleep when disconnected from BLE for 10 seconds
- Can be woken up by button press
- LED breathing effect indicates device status
