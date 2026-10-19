/***************************************************************
 * @file    Linear_Acceleration.ino
 * @brief   Example demonstrating how to read linear acceleration
 *          data from the 7Semi BNO055 sensor over I2C.
 *
 * Features Demonstrated:
 * - Sensor initialization using I2C
 * - Optional accelerometer configuration (range, bandwidth, power)
 * - Reading linear acceleration in raw (mg) and converted (m/s²)
 *
 * Notes:
 * - Linear acceleration is acceleration with gravity removed.
 * - 1 LSB = 1 mg = 0.00981 m/s² (default unit settings)
 *
 * Sensor Configuration:
 * - Accel Range    : ±4g
 * - Bandwidth      : 125 Hz
 * - Power Mode     : Normal
 * - Communication  : I2C (auto-detect 0x28 / 0x29)
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

// 1 LSB = 1 mg = 0.00981 m/s² (when UNIT_SEL accel bit = 0)
static inline float toMS2(int16_t v) { return v * 0.00981f; }

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println(F("\n7Semi_BNO055 Linear Acceleration"));

  // begin(Wire, SDA, SCL, useExtCrystal)
  if (!imu.begin(Wire, SDA, SCL, /*useExtCrystal=*/true)) {
    Serial.println(F("BNO055 not found"));
    while (1) delay(1000);
  }

  // (Optional) configure accel if you want specific range/BW/power
  imu.configAccel(g4, BW_125, Accel_Normal);

  Serial.println(F("Reading linear acceleration..."));
}

void loop() {
  int16_t lx_r, ly_r, lz_r;
  if (imu.readLinear(lx_r, ly_r, lz_r)) {
    float lx = toMS2(lx_r);
    float ly = toMS2(ly_r);
    float lz = toMS2(lz_r);

    Serial.print(F("linear accel mg: "));
    Serial.print(lx_r); Serial.print(',');
    Serial.print(ly_r); Serial.print(',');
    Serial.print(lz_r);

    Serial.print(F("   m/s^2: "));
    Serial.print(lx, 3); Serial.print(',');
    Serial.print(ly, 3); Serial.print(',');
    Serial.println(lz, 3);
  } else {
    Serial.println(F("readLinear failed"));
  }

  delay(200);
}
