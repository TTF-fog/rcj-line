/*************************************************************** 
 * @file   7SemiVLab_QuaternionStreamer.ino
 * @brief   Streams quaternion orientation and calibration status 
 *          from the 7Semi BNO055 IMU sensor over Serial.
 *
 * Features demonstrated:
 * - Initialization and NDOF mode selection
 * - Reading quaternion data (w, x, y, z)
 * - Reading sensor calibration status (sys, gyro, accel, mag)
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
 * Output Format:
 * - Serial CSV: Q,w,x,y,z,CAL,sys,g,a,m (baud: 115200)
 *
 * Library     : 7Semi_BNO055
 * Author      : 7Semi
 * Version     : 1.0
 * Date        : 03 October 2025
 * License     : MIT
 ***************************************************************/

#include <7Semi_BNO055.h>

// Create BNO055 IMU object
BNO055_7Semi bno;

void setup() {
  // Initialize built-in LED for heartbeat
  pinMode(LED_BUILTIN, OUTPUT);

  // Start serial communication
  Serial.begin(115200);

  // Initialize IMU: Wire interface, A4/A5, external crystal, 100kHz I2C
  if (!bno.begin(Wire, A4, A5, true, 100000)) {
    Serial.println(F("ERR: BNO055 not detected."));
    // Blink LED rapidly on error
    while (1) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(150);
      digitalWrite(LED_BUILTIN, LOW);
      delay(150);
    }
  }

  delay(1000);  // Allow sensor to settle

  // Set IMU to NDOF fusion mode
  bno.setMode(Mode::NDOF);

  // Startup message
  Serial.println(F("# 7SemiVLab Quaternion Streamer Ready (UNO)"));
  Serial.println(F("# Format: Q,w,x,y,z,CAL,sys,g,a,m"));
  Serial.println(F("# Commands: SAVE | RESTORE | RESET | STATUS"));  // commands not enabled in this version
}

void loop() {
  // Calibration status variables
  uint8_t sys, g, a, m;
  bno.calibBreakdown(sys, g, a, m);

  // Quaternion components
  float w, x, y, z;
  if (bno.readQuat(w, x, y, z)) {
    // Output CSV line
    Serial.print(F("Q,"));
    Serial.print(w, 6); Serial.print(',');
    Serial.print(x, 6); Serial.print(',');
    Serial.print(y, 6); Serial.print(',');
    Serial.print(z, 6);
    Serial.print(F(",CAL,"));
    Serial.print(sys); Serial.print(',');
    Serial.print(g); Serial.print(',');
    Serial.print(a); Serial.print(',');
    Serial.println(m);

    // LED heartbeat: toggle once per second
    static bool led = false;
    static unsigned hb = 0;
    if ((++hb % 50) == 0) {
      led = !led;
      digitalWrite(LED_BUILTIN, led ? HIGH : LOW);
    }

    // Run loop at ~50 Hz
    delay(20);
  }
}
