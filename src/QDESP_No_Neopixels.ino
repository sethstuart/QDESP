#include <OneWire.h>
#include "sdkconfig.h"
#include <Arduino.h>
#include <Bluepad32.h>
#include <vector>
#include <string>
#include <sstream>

#define BP32_MAX_GAMEPADS 2

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

const int ledPin = 2; // GPIO pin for the flash LED

const int IN1_pin = 25; // Assign ESP32 PWM Pin1 for DriveCell
const int IN2_pin = 27; // Assign ESP32 PWM Pin2 for DriveCell

const int IN3_pin = 26; // Assign ESP32 PWM Pin1 for DriveCell 2
const int IN4_pin = 18; // Assign ESP32 PWM Pin2 for DriveCell 2

// Button bitmask constants
const int BUTTON_1 = 0x0010;
const int BUTTON_2 = 0x0020;
const int BUTTON_3 = 0x0030;

// Menu button constant
const int MISC_BUTTON_MENU = 0x02;

void onConnectedController(ControllerPtr ctl) {
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
            ControllerProperties properties = ctl->getProperties();
            Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id,
                           properties.product_id);
            myControllers[i] = ctl;
            foundEmptySlot = true;
            break;
        }
    }
    if (!foundEmptySlot) {
        Serial.println("CALLBACK: Controller connected, but could not found empty slot");
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    bool foundController = false;

    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
            myControllers[i] = nullptr;
            foundController = true;
            break;
        }
    }

    if (!foundController) {
        Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
    }
}

std::vector<std::string> getButtonMaskValues(int buttonMask) {
    std::vector<std::string> pressedButtons;
    
    if (buttonMask & 0x0001) pressedButtons.push_back("A");
    if (buttonMask & 0x0002) pressedButtons.push_back("B");
    if (buttonMask & 0x0004) pressedButtons.push_back("X");
    if (buttonMask & 0x0008) pressedButtons.push_back("Y");
    if (buttonMask & 0x0100) pressedButtons.push_back("StickOne");
    if (buttonMask & 0x0200) pressedButtons.push_back("StickTwo");
    if (buttonMask & 0x0040) pressedButtons.push_back("LT");
    if (buttonMask & 0x0010) pressedButtons.push_back("LB");
    if (buttonMask & 0x0080) pressedButtons.push_back("RT");
    if (buttonMask & 0x0020) pressedButtons.push_back("RB");

    return pressedButtons;
}

std::vector<std::string> getMiscMaskValues(int miscMask) {
    std::vector<std::string> pressedButtons;

    if (miscMask & 0x02) pressedButtons.push_back("left");
    if (miscMask & 0x08) pressedButtons.push_back("center");
    if (miscMask & 0x04) pressedButtons.push_back("right");
    if (miscMask & 0x01) pressedButtons.push_back("xbox");

    return pressedButtons;
}

std::vector<std::string> getDpadMaskValues(int dpadMask) {
    std::vector<std::string> pressedButtons;

    if (dpadMask & 0x01) pressedButtons.push_back("up");
    if (dpadMask & 0x02) pressedButtons.push_back("down");
    if (dpadMask & 0x08) pressedButtons.push_back("left");
    if (dpadMask & 0x04) pressedButtons.push_back("right");

    return pressedButtons;
}

// Helper function to convert a vector of strings to a single comma-separated string
std::string join(const std::vector<std::string>& vec, const std::string& delimiter) {
    std::ostringstream oss;
    for (size_t i = 0; i < vec.size(); ++i) {
        if (i != 0)
            oss << delimiter;
        oss << vec[i];
    }
    return oss.str();
}

void dumpGamepad(ControllerPtr ctl, int rightDC, int leftDC) {
    // Get the list of pressed buttons
    std::vector<std::string> pressedButtons = getButtonMaskValues(ctl->buttons());
    std::vector<std::string> pressedMiscButtons = getMiscMaskValues(ctl->miscButtons());
    std::vector<std::string> pressedDpadButtons = getDpadMaskValues(ctl->dpad());

    // Convert the lists to comma-separated strings
    std::string pressedButtonsStr = join(pressedButtons, ", ");
    std::string pressedMiscButtonsStr = join(pressedMiscButtons, ", ");
    std::string pressedDpadButtonsStr = join(pressedDpadButtons, ", ");

    Serial.printf(
        "idx=%d, dpad: 0x%02x (%s), buttons: 0x%04x (%s), axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
        "misc: 0x%02x (%s), gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d, rightDuty: %d, leftDuty: %d\n",
        ctl->index(),                // Controller Index
        ctl->dpad(),                 // D-pad
        pressedDpadButtonsStr.c_str(), // List of pressed D-pad buttons
        ctl->buttons(),              // bitmask of pressed buttons
        pressedButtonsStr.c_str(),   // List of pressed buttons
        ctl->axisX(),                // (-511 - 512) left X Axis
        ctl->axisY(),                // (-511 - 512) left Y axis
        ctl->axisRX(),               // (-511 - 512) right X axis
        ctl->axisRY(),               // (-511 - 512) right Y axis
        ctl->brake(),                // (0 - 1023): brake button
        ctl->throttle(),             // (0 - 1023): throttle (AKA gas) button
        ctl->miscButtons(),          // bitmask of pressed "misc" buttons
        pressedMiscButtonsStr.c_str(), // List of pressed misc buttons
        ctl->gyroX(),                // Gyro X
        ctl->gyroY(),                // Gyro Y
        ctl->gyroZ(),                // Gyro Z
        ctl->accelX(),               // Accelerometer X
        ctl->accelY(),               // Accelerometer Y
        ctl->accelZ(),               // Accelerometer Z
        rightDC,                     // Right Drivecell Duty Cycle
        leftDC                       // Left Drivecell Duty Cycle
    );
}

void processControllers() {
    for (auto myController : myControllers) {
        if (myController && myController->isConnected() && myController->hasData()) {
            if (myController->isGamepad()) {
                processGamepad(myController);
            }
        }
    }
}

void processGamepad(ControllerPtr ctl) {
    int brakeValue = ctl->brake(); // Get value from "brake" (left trigger)
    int throttleValue = ctl->throttle(); // Get value from "throttle" (right trigger)
    int buttonMask = ctl->buttons(); // Get button mask
    int miscMask = ctl->miscButtons(); // Get misc button mask
    static bool menu = false; // Initialize menu to false

    std::vector<std::string> pressedButtons = getButtonMaskValues(buttonMask);
    std::vector<std::string> pressedMiscButtons = getMiscMaskValues(miscMask);

    int rightDC = map(brakeValue, 0, 1024, 0, 255);
    int leftDC = map(throttleValue, 0, 1024, 0, 255);

    // Check if specific buttons are pressed
    bool button1Pressed = buttonMask & 0x0010;
    bool button2Pressed = buttonMask & 0x0020;

    if (button1Pressed && !button2Pressed) {
        // Only BUTTON_1 pressed
        ledcWrite(0, leftDC); // (channel, duty cycle 0 - 255)
        ledcWrite(1, !leftDC);
        ledcWrite(2, rightDC);
        ledcWrite(3, !rightDC);
    } 
    if (button2Pressed && !button1Pressed) {
        // Only BUTTON_2 pressed
        ledcWrite(0, !leftDC); // (channel, duty cycle 0 - 255)
        ledcWrite(1, leftDC);
        ledcWrite(2, !rightDC);
        ledcWrite(3, rightDC);
    } 
    if (button1Pressed && button2Pressed) {
        // Both BUTTON_1 and BUTTON_2 pressed
        ledcWrite(0, leftDC); // (channel, duty cycle 0 - 255)
        ledcWrite(1, !leftDC);
        ledcWrite(2, !rightDC);
        ledcWrite(3, rightDC);
    } 
    if (!button1Pressed && !button2Pressed) {
        // No specific button pressed
        ledcWrite(0, !leftDC); // (channel, duty cycle 0 - 255)
        ledcWrite(1, leftDC);
        ledcWrite(2, rightDC);
        ledcWrite(3, !rightDC);
    }

    dumpGamepad(ctl, rightDC, leftDC);
}

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

}

void loop() {
    bool dataUpdated = BP32.update();
    if (dataUpdated) {
        processControllers();
    }
}