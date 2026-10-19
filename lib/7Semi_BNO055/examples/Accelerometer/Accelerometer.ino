/***************************************************************
 * @file    Accelerometer.ino
 * @brief   Example for reading raw and converted accelerometer data 
 *          from the 7Semi BNO055 IMU sensor using I2C.
 *
 * Features demonstrated:
 * - Initialization and configuration of the accelerometer
 * - Setting range, bandwidth, and power mode
 * - Reading raw (mg) and converted (m/s²) values
 * - Register-based configuration validation
 *
 * Sensor configuration used:
 * - Accel Range   : ±4g
 * - Bandwidth     : 125 Hz
 * - Power Mode    : Normal
 * - Communication : I2C (auto-detect 0x28 or 0x29)
 *
 * Connections:
 * - SDA -> A4 (UNO) or custom SDA
 * - SCL -> A5 (UNO) or custom SCL
 * - VIN -> 3.3V / 5V
 * - GND -> GND
 *
 * @author   7Semi
 * @license  MIT
 * @version  1.0
 * @date     19 September 2025
 ***************************************************************/

#include <Wire.h>
#include <7Semi_BNO055.h>

BNO055_7Semi imu;

// Convert mg accel  to m/s² 
static inline float accMS2(int16_t v) {
  return v * 0.00981f;
}

// Print back current accel config from registers
void printAccelConfig() {
  uint8_t range, bw, pwr;
  if (imu.getAccelConfig(range, bw, pwr)) {
    Serial.print(F("Accel Config  range="));
    Serial.print(range);
    Serial.print(F("  bw="));
    Serial.print(bw);
    Serial.print(F("  power="));
    Serial.println(pwr);
  } else {
    Serial.println(F("getAccelConfig failed"));
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println(F("\n7Semi_BNO055 Accel"));

  // Initialize IMU (auto-detects 0x28/0x29 I2C addr)
  if (!imu.begin(Wire, SDA, SCL, /*useExtCrystal=*/true)) {
    Serial.println(F("BNO055 not found"));
    while (1) delay(1000);
  }

 /*
 Accelerometer Configuration 
  ranges:
    g2
    g4
    g8
    g16
  
  bandwidths:
    BW_7_81
    BW_15_63
    BW_31_25
    BW_62_5
    BW_125
    BW_250
    BW_500
    BW_1000
  
  power modes:
    Accel_Normal
    Accel_Suspend
    Accel_LowPower1
    Accel_Standby
    Accel_LowPower2
    Accel_DeepSuspend
    */

  if (!imu.configAccel(g4, BW_125, Accel_Normal)) {
    Serial.println(F("configAccel failed"));
  }

  if (!imu.configAccel(g4, BW_125, Accel_Normal)) {
    Serial.println(F("configAccel failed"));
  }

  // Print actual register config to confirm
  printAccelConfig();
  Serial.println(F("Reading accel..."));
}

void loop() {
  int16_t axr, ayr, azr;
  if (imu.readAccel(axr, ayr, azr)) {
    float ax = accMS2(axr);
    float ay = accMS2(ayr);
    float az = accMS2(azr);

    // Print both raw LSB and converted m/s²
    Serial.print(F("mg: "));
    Serial.print(axr);
    Serial.print(',');
    Serial.print(ayr);
    Serial.print(',');
    Serial.print(azr);

    Serial.print(F("   m/s^2: "));
    Serial.print(ax, 3);
    Serial.print(',');
    Serial.print(ay, 3);
    Serial.print(',');
    Serial.println(az, 3);
  } else {
    Serial.println(F("readAccel failed"));
  }

  delay(500);
}
