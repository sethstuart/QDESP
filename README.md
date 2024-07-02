# QDESP
 A simple esp32 robotics project using DriveCells and bluepad32

 This project uses an ESP32 and the Bluepad32 library to control a simple robot with an Xbox controller. 
 The project also features dynamic NeoPixel effects that change based on controller inputs.
 
 NOTE: THIS README WAS MADE BY AI, IT MIGHT NOT BE ACCURATE!

## Features

- **Motor Control**: Control the motors of the battle bot using the brake and throttle triggers on the Xbox controller.
- **NeoPixel Effects**: Dynamic visual effects on a single NeoPixel LED, including:
  - Breathing White
  - Rainbow
  - Police Strobe (Red and Blue)
  - Dynamic color based on brake and throttle values
  - Off state

## Hardware Requirements

- ESP32
- Xbox controller
- Two DriveCell modules
- NeoPixel LED
- Additional components: Resistors, capacitors, wiring, etc.

## Software Requirements

- Arduino IDE
- Bluepad32 library
- Adafruit NeoPixel library

## Pin Configuration

- **DriveCell 1**: IN1_pin (GPIO 25), IN2_pin (GPIO 27)
- **DriveCell 2**: IN3_pin (GPIO 26), IN4_pin (GPIO 18)
- **NeoPixel LED**: GPIO 19

## Installation

1. **Clone this repository**:
    ```bash
    git clone https://github.com/sethstuart/QDESP.git
    ```
2. **Open the project in Arduino IDE**:
    ```bash
    arduino-cli core install esp32:esp32
    arduino-cli lib install "Adafruit NeoPixel"
    arduino-cli lib install "Bluepad32"
    ```
3. **Configure the ESP32**:
    - Select the correct board (`ESP32 Dev Module`)
    - Select the correct port

## Usage

1. **Setup**:
    - Connect the hardware as per the pin configuration.
    - Upload the code to the ESP32.

2. **Controller**:
    - Pair the Xbox controller with the ESP32.
    - Use the brake and throttle triggers to control the motors.
    - Use the center button to cycle through NeoPixel effects.

3. **Effects**:
    - **Off**: No light.
    - **Breathing White**: Smooth pulsing white light.
    - **Rainbow**: Continuous rainbow cycle.
    - **Police Strobe**: Alternating red and blue lights.
    - **Dynamic Brake/Throttle**: Color changes based on brake and throttle values.

## Code Explanation

### Main Code Sections

#### Setup Function

```cpp
void setup() {
    Serial.begin(115200);
    Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
    const uint8_t* addr = BP32.localBdAddress();
    Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

    BP32.setup(&onConnectedController, &onDisconnectedController);
    BP32.forgetBluetoothKeys();
    BP32.enableVirtualDevice(false);

    ledcSetup(0, 20000, 8);      // Channel 0, 20 kHz frequency, 8-bit resolution - left motor
    ledcAttachPin(IN1_pin, 0);  // Attach pin 25 to channel 0

    ledcSetup(1, 20000, 8);      // Channel 1, 20 kHz frequency, 8-bit resolution - left motor
    ledcAttachPin(IN2_pin, 1);  // Attach pin 27 to channel 1

    ledcSetup(2, 20000, 8);      // Channel 2, 20 kHz frequency, 8-bit resolution - right motor
    ledcAttachPin(IN3_pin, 2);  // Attach pin 26 to channel 2

    ledcSetup(3, 20000, 8);      // Channel 3, 20 kHz frequency, 8-bit resolution - right motor
    ledcAttachPin(IN4_pin, 3);  // Attach pin 18 to channel 3

    pixel.begin();
    pixel.setBrightness(50);
    pixel.show(); // Initialize pixel to 'off'
}
