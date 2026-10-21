// RCJ Line robot firmware for the Cytron Motion 2350 Pro.
// The Pi drives everything over USB serial; see docs/PROTOCOL.md.

#include <Arduino.h>
#include <Wire.h>

#include "BoardPins.h"
#include "Claw.h"
#include "Commands.h"
#include "Config.h"
#include "Imu.h"
#include "Lights.h"
#include "Motion.h"
#include "Motors.h"
#include "Telemetry.h"
#include "Tof.h"

void setupI2c() {
    Wire.setSDA(I2C0_SDA);
    Wire.setSCL(I2C0_SCL);
    Wire.setClock(I2C_HZ);
    Wire.begin();
    Wire1.setSDA(I2C1_SDA);
    Wire1.setSCL(I2C1_SCL);
    Wire1.setClock(I2C_HZ);
    Wire1.begin();
}

void checkEstopButton() {
    static bool wasPressed = false;
    static unsigned long lastPressMs = 0;
    bool pressed = digitalRead(ESTOP_BUTTON_PIN) == LOW;
    if (pressed && !wasPressed && millis() - lastPressMs > 200) {
        lastPressMs = millis();
        motion::stop();
        claw::cancel();
        lights::flashEstop();
        Serial.println("EVT ESTOP");
    }
    wasPressed = pressed;
}

void setup() {
    Serial.begin(115200);
    config::begin();
    motors::begin();
    lights::begin();
    pinMode(ESTOP_BUTTON_PIN, INPUT_PULLUP);

    setupI2c();
    tof::begin();  // first, so the ToFs are off the bus before the IMU is probed
    imu::begin();
    claw::begin();

    Serial.printf("READY imu=%d tof=%d\n", imu::ok(), tof::presentCount());
}

void loop() {
    commands::poll();
    checkEstopButton();
    tof::update();
    claw::update();

    static unsigned long lastTickMs = 0;
    if (millis() - lastTickMs >= motion::TICK_MS) {
        lastTickMs = millis();
        motion::update();
        motors::update();
    }

    telemetry::update();
    lights::update();
}
