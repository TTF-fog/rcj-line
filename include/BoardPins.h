#ifndef BOARD_PINS_H
#define BOARD_PINS_H

#include <Arduino.h>

/**
 * @brief Hardware mapping for the Cytron Motion 2350 Pro (RP2350A).
 *
 * Board: Cytron Motion 2350 Pro with its onboard 4-channel DC motor driver,
 * onboard 2x WS2812 (GP23, status indicators), passive buzzer (GP22),
 * user buttons (GP20/GP21), LF white light bar (GP18),
 * BNO055 IMU on hardware I2C0 (SDA=GP16, SCL=GP17), UART1 on GP0/GP1,
 * claw servos on GP2/GP7 and the pickup servo on GP5 (PIO, 50 Hz).
 */

#define BOARD_NAME_TEXT       "Cytron Motion 2350 Pro (RP2350)"
#define PIN_DESC_TEXT         "M1={8,9}, M2={10,11}, M3={12,13}, M4={14,15} (onboard 4ch driver, 3A/5A peak per ch)"
#define MOTOR_HEADER_TEXT     "Motion 2350 Pro onboard 4-ch motor driver"
#define IMU_WIRING_TEXT       "BNO055 on I2C0 (SDA=GP16, SCL=GP17, 100kHz)"
#define IMU_SDA_PIN_TEXT      "GP16"
#define IMU_SCL_PIN_TEXT      "GP17"
#define UART_TEXT             "USB-CDC Serial + UART1 (TX=GP0, RX=GP1)"

#define IMU_WIRE              Wire
#define IMU_SDA_PIN           PIN_WIRE0_SDA   // 16
#define IMU_SCL_PIN           PIN_WIRE0_SCL   // 17

// Status LEDs = the 2 onboard WS2812 near the USB connector.
#define STATUS_PIXEL_PIN      PIN_NEOPIXEL    // 23
#define STATUS_PIXEL_COUNT    NUM_NEOPIXEL    // 2

// LF white light bar: RGB pixel sink for LIGHT commands.
#define EXT_NEOPIXEL_PIN      18
#define EXT_NEOPIXEL_COUNT    8               // adjust to the actual bar length

// Claw + pickup servos (PIO-driven 50 Hz hobby servos, tunable via the CLAW
// serial command). Wired to the free header pins, away from the motor driver.
#define SERVO_CLAW_LEFT_PIN   2               // claw jaw (left)
#define SERVO_CLAW_RIGHT_PIN  7               // claw jaw (right)
#define SERVO_PICKUP_PIN      5               // pickup arm (up/down)
#define SERVO_WIRING_TEXT     "Claw servos: left=GP2, right=GP7, pickup=GP5 (PIO 50Hz)"

#define BUZZER_PIN            PIN_BUZZER      // 22 (passive piezo, mute switch onboard)

#define USER_BTN1_PIN         20              // momentary low-active: ESTOP
#define USER_BTN2_PIN         21              // spare

#define PWM_FREQ_HZ           20000           // silent motor PWM

// NOTE: the motor pin pairs (PIN_M1A..PIN_M4B) are turned into the
// MotorPinConfig array in main.cpp, so this header stays free of any
// MotorController.h dependency (see main.cpp: motorConfigs).

#endif // BOARD_PINS_H
