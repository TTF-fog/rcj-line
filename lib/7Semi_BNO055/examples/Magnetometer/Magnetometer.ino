/***************************************************************
 * @file    Magnetometer.ino
 * @brief   Example for reading magnetic field data from the
 *          7Semi BNO055 9-axis IMU sensor in MAGONLY mode.
 *
 * Features Demonstrated:
 * - Initializes BNO055 in MAGONLY (magnetometer-only) mode
 * - Configures magnetometer: data rate, power mode, and op mode
 * - Reads raw magnetic field values (X/Y/Z)
 * - Converts readings to microtesla (µT)
 *
 * Sensor Configuration:
 * - Mode       : MAGONLY
 * - Rate       : 20 Hz
 * - Power Mode : Normal
 * - Op Mode    : High Accuracy
 * - Units      : 1 LSB = 1/16 µT
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

static inline float magUT(int16_t v) {
  return v / 16.0f;
}  // 1 LSB = 1/16 µT

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println(F("\n7Semi_BNO055 Magnetometer"));

  // begin(Wire, SDA, SCL, useExtCrystal, i2cClockHz)
  if (!imu.begin(Wire, SDA, SCL, /*useExtCrystal=*/true)) {
    Serial.println(F("BNO055 not found"));
    while (1) delay(1000);
  }

  // IMPORTANT: use a mode that enables the magnetometer by itself
  imu.setMode(Mode::MAGONLY);  // or Mode::AMG

  // Configure MAG: (rate, power, op-mode)
  // Note: In fusion modes these bits may be overridden. In MAGONLY they take effect.
  imu.configMag(Hz20, Mag_Normal, HighAccuracy);

  // Small warm-up after mode/config changes
  delay(50);

  /*
  Magnetometer Configuration 
  rates:       Hz2, Hz6, Hz8, Hz10, Hz15, Hz20, Hz25, Hz30
  power modes: Mag_Normal, Mag_Sleep, Mag_Suspend, Mag_ForceMode
  op modes:    LowPower, Regular, EnhancedRegular, HighAccuracy
  */
  uint8_t rate, pwr, op;
  imu.getMagConfig(rate, pwr, op);
  Serial.print(F("Mag Config  rate="));
  Serial.print(rate);
  Serial.print(F("  power="));
  Serial.print(pwr);
  Serial.print(F("  op="));
  Serial.println(op);

  Serial.println(F("Reading magnetometer..."));
}

void loop() {
  int16_t mxr, myr, mzr;
  if (imu.readMag(mxr, myr, mzr)) {
    float mx = magUT(mxr), my = magUT(myr), mz = magUT(mzr);

    Serial.print(F("RAW: "));
    Serial.print(mxr);
    Serial.print(',');
    Serial.print(myr);
    Serial.print(',');
    Serial.print(mzr);

    Serial.print(F("   µT: "));
    Serial.print(mx, 2);
    Serial.print(',');
    Serial.print(my, 2);
    Serial.print(',');
    Serial.println(mz, 2);
  } else {
    Serial.println(F("readMag failed"));
  }
  delay(300);
}
