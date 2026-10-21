#pragma once

#include <Arduino.h>

// Cytron Motion 2350 Pro pin map. PIN_* names come from the core's board variant.

// Onboard motor driver. Left side = M1 + M2, right side = M3 + M4.
const uint8_t MOTOR_PINS[4][2] = {
    {PIN_M1A, PIN_M1B},
    {PIN_M2A, PIN_M2B},
    {PIN_M3A, PIN_M3B},
    {PIN_M4A, PIN_M4B},
};
const uint32_t MOTOR_PWM_HZ = 20000;

// I2C0 (Wire): BNO055 + ToF 1/2. I2C1 (Wire1): ToF 3/4.
const uint8_t I2C0_SDA = PIN_WIRE0_SDA;  // GP16
const uint8_t I2C0_SCL = PIN_WIRE0_SCL;  // GP17
const uint8_t I2C1_SDA = PIN_WIRE1_SDA;  // GP26
const uint8_t I2C1_SCL = PIN_WIRE1_SCL;  // GP27
const uint32_t I2C_HZ = 400000;

// VL53L0X XSHUT lines, one per ToF slot (see Tof.cpp for bus + address).
const uint8_t TOF_XSHUT_PINS[4] = {3, 4, 6, 19};

// Claw: two mirrored jaw servos and the arm that lowers/raises them.
const uint8_t SERVO_JAW_LEFT_PIN = 2;
const uint8_t SERVO_JAW_RIGHT_PIN = 7;
const uint8_t SERVO_ARM_PIN = 5;

const uint8_t STATUS_PIXEL_PIN = PIN_NEOPIXEL;  // GP23, 2 onboard WS2812
const uint8_t STATUS_PIXEL_COUNT = NUM_NEOPIXEL;
const uint8_t LIGHT_BAR_PIN = 18;
const uint8_t LIGHT_BAR_COUNT = 8;

const uint8_t ESTOP_BUTTON_PIN = 20;  // onboard user button 1, active low
