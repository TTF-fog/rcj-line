/***************************************************************
 * @file    Quaternion.ino
 * @brief   Example demonstrating how to read orientation in
 *          quaternion format from the 7Semi BNO055 sensor.
 *
 * Features Demonstrated:
 * - Sensor initialization via I2C (auto-detect 0x28/0x29)
 * - Reads W, X, Y, Z components of quaternion output
 * - Prints quaternion data to Serial Monitor
 *
 * Notes:
 * - Quaternion representation avoids gimbal lock.
 * - Useful for sensor fusion, VR/AR, robotics, and 3D rotation tracking.
 * - Output unit is unitless (normalized quaternion).
 *
 * Connections:
 * - VIN  -> 3.3V / 5V
 * - GND  -> GND
 * - SDA  -> A4 (Uno) or custom SDA
 * - SCL  -> A5 (Uno) or custom SCL
 *
 * Library   : 7Semi_BNO055
 * Author    : 7Semi
 * Version   : 1.0
 * Date      : 20 September 2025
 * License   : MIT
 ***************************************************************/
#include <Wire.h>
#include <7Semi_BNO055.h>

BNO055_7Semi imu;

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println(F("\n7Semi_BNO055 Quaternion"));

  // begin(Wire, SDA, SCL, useExtCrystal)
  if (!imu.begin(Wire, SDA, SCL, /*useExtCrystal=*/true)) {
    Serial.println(F("BNO055 not found"));
    while (1) delay(1000);
  }

  Serial.println(F("Reading quaternion (W, X, Y, Z)..."));
}

void loop() {
  float w, x, y, z;
  if (imu.readQuat(w, x, y, z)) {
    Serial.print(F("Quat W,X,Y,Z: "));
    Serial.print(w, 5); Serial.print(", ");
    Serial.print(x, 5); Serial.print(", ");
    Serial.print(y, 5); Serial.print(", ");
    Serial.println(z, 5);
  } else {
    Serial.println(F("readQuat failed"));
  }

  delay(200);
}
