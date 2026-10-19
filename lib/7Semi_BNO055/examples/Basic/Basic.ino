/***************************************************************
 * @file    Basic.ino
 * @brief   Comprehensive example for the 7Semi BNO055 IMU sensor
 *          demonstrating orientation, raw data, calibration, and
 *          quaternion features via I2C.
 *
 * Features demonstrated:
 * - Initialization and NDOF mode selection
 * - Reading Euler angles (heading, roll, pitch)
 * - Reading raw accelerometer, gyroscope, magnetometer data
 * - Reading linear acceleration and gravity vector
 * - Reading quaternion data
 * - Reading temperature and calibration status
 *
 * Sensor Configuration:
 * - Operation Mode : NDOF (Fusion mode)
 * - I2C Address     : Auto-detect (0x28 or 0x29)
 * - External Crystal: Enabled
 *
 * Connections:
 * - VIN  -> 3.3V / 5V
 * - GND  -> GND
 * - SDA  -> A4 (Uno) or custom SDA pin
 * - SCL  -> A5 (Uno) or custom SCL pin
 * - ADR  -> GND (for 0x28) or VCC (for 0x29)
 *
 * Library     : 7Semi_BNO055
 * Author      : 7Semi
 * Version     : 1.0
 * Date        : 19 September 2025
 * License     : MIT
 ***************************************************************/

#include <Wire.h>
#include <7Semi_BNO055.h>

BNO055_7Semi imu;  

void printCalib() {
  uint8_t sys, gyr, acc, mag;
  imu.calibBreakdown(sys, gyr, acc, mag);
  Serial.print(F("Calib SYS:"));
  Serial.print(sys);
  Serial.print(F(" G:"));
  Serial.print(gyr);
  Serial.print(F(" A:"));
  Serial.print(acc);
  Serial.print(F(" M:"));
  Serial.println(mag);
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println(F("\nBNO055 demo"));

  // Initialize (Wire, address, use external crystal)
  if (!imu.begin()) {
    Serial.println(F("ERROR: BNO055 not found"));
    while (1) { delay(1000); }
  }

  imu.setMode(Mode::NDOF);

  // Optional: wait for calibration
  Serial.print(F("Calibrating"));
  if (!imu.waitCalibrated(10000, 200)) {
    Serial.println(F(" - timeout"));
  } else {
    Serial.println(F(" - done"));
  }
  printCalib();

  Serial.println(F("Ready!\n"));
}

void loop() {
  // // Orientation (degrees or radians depending on UNIT_SEL; defaults to degrees)
  float heading, roll, pitch;
  if (imu.readEuler(heading, roll, pitch)) {
    Serial.print(F("Euler H/R/P: "));
    Serial.print(heading, 1);
    Serial.print('\t');
    Serial.print(roll, 1);
    Serial.print('\t');
    Serial.println(pitch, 1);
  }

  // Raw sensor data
  int16_t ax, ay, az, gx, gy, gz, mx, my, mz;
  if (imu.readAccel(ax, ay, az)) {
    Serial.print(F("Accel raw: "));
    Serial.print(ax);
    Serial.print(", ");
    Serial.print(ay);
    Serial.print(", ");
    Serial.println(az);
  }
  if (imu.readGyro(gx, gy, gz)) {
    Serial.print(F("Gyro raw : "));
    Serial.print(gx);
    Serial.print(", ");
    Serial.print(gy);
    Serial.print(", ");
    Serial.println(gz);
  }
  if (imu.readMag(mx, my, mz)) {
    Serial.print(F("Mag  raw : "));
    Serial.print(mx);
    Serial.print(", ");
    Serial.print(my);
    Serial.print(", ");
    Serial.println(mz);
  }

  // Optional: linear acceleration / gravity
  int16_t lx, ly, lz, gxv, gyv, gzv;
  if (imu.readLinear(lx, ly, lz)) {
    Serial.print(F("LinAcc raw: "));
    Serial.print(lx);
    Serial.print(", ");
    Serial.print(ly);
    Serial.print(", ");
    Serial.println(lz);
  }
  if (imu.readGravity(gxv, gyv, gzv)) {
    Serial.print(F("Grav  raw: "));
    Serial.print(gxv);
    Serial.print(", ");
    Serial.print(gyv);
    Serial.print(", ");
    Serial.println(gzv);
  }

  // Optional: quaternion
  float qw, qx, qy, qz;
  if (imu.readQuat(qw, qx, qy, qz)) {
    Serial.print(F("Quat WXYZ: "));
    Serial.print(qw, 4);
    Serial.print(", ");
    Serial.print(qx, 4);
    Serial.print(", ");
    Serial.print(qy, 4);
    Serial.print(", ");
    Serial.println(qz, 4);
  }
  int temp = imu.temperatureC();
  Serial.print("Temperature: ");
  Serial.print(temp);
  Serial.println(" °C");
  // Show calibration every ~2s
  static uint32_t tCal = 0;
  if (millis() - tCal > 2000) {
    tCal = millis();
    printCalib();
  }

  Serial.println();
  delay(500);
}
