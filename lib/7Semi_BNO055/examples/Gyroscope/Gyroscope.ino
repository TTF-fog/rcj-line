/***************************************************************
 * @file    Gyroscope.ino
 * @brief   Example for reading gyroscope data from the 
 *          7Semi BNO055 IMU sensor using I2C.
 *
 * Features Demonstrated:
 * - Initialization and sensor configuration
 * - Gyroscope configuration: range, bandwidth, power mode
 * - Reading raw gyro data in DSP units
 * - Converting DSP (1/16°/s) to RPS (rad/s)
 * - Real-time monitoring via Serial
 *
 * Sensor Configuration:
 * - Range       : ±500 dps
 * - Bandwidth   : 47 Hz
 * - Power Mode  : Normal
 * - Communication: I2C (auto-detect 0x28 / 0x29)
 *
 * Connections:
 * - VIN  -> 3.3V / 5V
 * - GND  -> GND
 * - SDA  -> A4 (UNO) or custom SDA
 * - SCL  -> A5 (UNO) or custom SCL
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

// 1 dsp = 1/16 rps
static inline float gyroRPS(int16_t v) {
  return v / 16.0f;
}

void printGyroConfig() {
  uint8_t range, bw, pwr;
  if (imu.getGyroConfig(range, bw, pwr)) {
    Serial.print(F("Gyro Config  range="));
    Serial.print(range);
    Serial.print(F("  bw="));
    Serial.print(bw);
    Serial.print(F("  power="));
    Serial.println(pwr);
  } else {
    Serial.println(F("getGyroConfig failed"));
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println(F("\n7Semi_BNO055 Gyroscope-only"));

  // begin(Wire, SDA, SCL, useExtCrystal, i2cClockHz)
  if (!imu.begin(Wire, SDA, SCL, /*useExtCrystal=*/true)) {
    Serial.println(F("BNO055 not found"));
    while (1) delay(1000);
  }
  /*
    Gyroscope Configuration 
    ranges:
      dps2000, dps1000, dps500, dps250, dps125
    
    bandwidths:
      BW_523, BW_230, BW_116, BW_47, BW_23, BW_12, BW_64, BW_32
    
    power modes:
      Gyro_Normal, Gyro_FastPowerUp, Gyro_DeepSuspend,
      Gyro_Suspend, Gyro_AdvancedPowersave
    */

  if (!imu.configGyro(dps500, BW_47, Gyro_Normal)) {
    Serial.println(F("configGyro failed"));
  }

  printGyroConfig();
  Serial.println(F("Reading gyro..."));
}

void loop() {
  int16_t gxr, gyr, gzr;
  if (imu.readGyro(gxr, gyr, gzr)) {
    float gx = gyroRPS(gxr);
    float gy = gyroRPS(gyr);
    float gz = gyroRPS(gzr);

    Serial.print(F("DSP: "));
    Serial.print(gxr);
    Serial.print(',');
    Serial.print(gyr);
    Serial.print(',');
    Serial.print(gzr);

    Serial.print(F("   RSP: "));
    Serial.print(gx, 2);
    Serial.print(',');
    Serial.print(gy, 2);
    Serial.print(',');
    Serial.println(gz, 2);
  } else {
    Serial.println(F("readGyro failed"));
  }

  delay(500);
}
