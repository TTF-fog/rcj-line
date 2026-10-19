/***************************************************************
 * @file    Euler.ino
 * @brief   Example to read Euler angles (Heading, Roll, Pitch)
 *          from the 7Semi BNO055 9-axis IMU using I2C.
 *
 * Features Demonstrated:
 * - Sensor initialization via I2C
 * - Fusion mode setup (NDOF)
 * - Reading orientation as Euler angles
 * - Conversion from degrees to radians
 *
 * Sensor Configuration:
 * - Mode         : NDOF (Accel + Gyro + Mag fusion)
 * - Output Units : Degrees (default)
 * - Interface    : I2C (auto-detect 0x28 or 0x29)
 * - External Crystal : Enabled
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
inline float degToRad(float deg) {
  return deg * 3.14159265358979323846f / 180.0f;
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println(F("\n7Semi_BNO055 Euler Angles"));

  // begin(Wire, SDA, SCL, useExtCrystal, i2cClockHz)
  if (!imu.begin(Wire, SDA, SCL, /*useExtCrystal=*/true)) {
    Serial.println(F("BNO055 not found"));
    while (1) delay(1000);
  }

  /* 
  Fusion Mode Configuration
  You must be in a fusion mode to get Euler angles.
  Common modes:
    NDOF          - Full fusion (accel + gyro + mag)
    IMUPLUS       - Fusion (accel + gyro)
    COMPASS       - Fusion (accel + mag)
  */

  imu.setMode(Mode::NDOF);  // Switch to full fusion mode

  Serial.println(F("Reading Euler angles (Heading / Roll / Pitch)..."));
}

void loop() {
  float heading, roll, pitch;

  if (imu.readEuler(heading, roll, pitch)) {
    // Output in degrees
    Serial.print(F("Euler H/R/P Degree: "));
    Serial.print(heading, 2);
    Serial.print(',');
    Serial.print(roll, 2);
    Serial.print(',');
    Serial.print(pitch, 2);
    Serial.print("\t  radians: ");
    Serial.print(degToRad(heading), 4);
    Serial.print(',');
    Serial.print(degToRad(roll), 4);
    Serial.print(',');
    Serial.println(degToRad(pitch), 4);

  } else {
    Serial.println(F("readEuler failed"));
  }

  delay(200);
}
