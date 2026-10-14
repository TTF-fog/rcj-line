#ifndef SUBSCRIPTS_H
#define SUBSCRIPTS_H

#include <Arduino.h>
#include <Wire.h>
#include <7Semi_BNO055.h>
#include "MotorController.h"

/**
 * @brief Display interactive startup menu over USB Serial with a countdown timeout.
 * @param timeoutSeconds Countdown time in seconds before auto-selecting Main Script.
 * @return int Selected mode: 1 = I2C Scanner, 2 = Motor Test, 3 = BNO Test, 4 = Main Script.
 */
int promptStartupMenu(int timeoutSeconds = 5);

/**
 * @brief Subscript 1: I2C Bus Scanner
 * Scans standard I2C bus (Wire on pins 18/A4 and 19/A5) for connected peripherals.
 */
void runI2CScanner();

/**
 * @brief Subscript 2: Four-Motor Test Sequence
 * Tests each motor (M1..M4) forward and reverse as demonstrated in example.txt.
 * @param motors Reference to MotorController instance.
 */
void runMotorTest(MotorController &motors);

/**
 * @brief Subscript 3: BNO055 IMU Diagnostics & Live Stream
 * Tests chip detection, reads chip ID, temperature, calibration status, and streams Euler angles.
 * @param bno Reference to BNO055_7Semi IMU instance.
 */
void runBNOTest(BNO055_7Semi &bno);

#endif // SUBSCRIPTS_H
