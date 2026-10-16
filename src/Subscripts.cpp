#include "Subscripts.h"

int promptStartupMenu(int timeoutSeconds) {
    Serial.println();
    Serial.println(F("===================================================="));
    Serial.println(F("            ROBOCUP ROBOT STARTUP MENU              "));
    Serial.println(F("===================================================="));
    Serial.println(F("  [1] I2C Bus Scanner"));
    Serial.println(F("  [2] Four-Motor Test Sequence (Maker Drive)"));
    Serial.println(F("  [3] BNO055 IMU Diagnostics & Live Stream"));
    Serial.println(F("  [4] Main Robot Script (IMU + Dual-UART Control)"));
    Serial.println(F("===================================================="));
    Serial.printf("Auto-booting to [4] in %d seconds...\r\n", timeoutSeconds);
    Serial.print(F("Countdown: "));

    unsigned long startMillis = millis();
    int lastRemaining = timeoutSeconds;
    Serial.printf("%ds ", lastRemaining);

    while ((millis() - startMillis) < (unsigned long)(timeoutSeconds * 1000)) {
        // Check for incoming user keystroke
        if (Serial.available() > 0) {
            char c = static_cast<char>(Serial.read());

            // Flush extra trailing newline / carriage return
            while (Serial.available() > 0 && (Serial.peek() == '\n' || Serial.peek() == '\r')) {
                Serial.read();
            }

            if (c >= '1' && c <= '4') {
                Serial.println();
                Serial.printf(">> Selected Mode [%c]\r\n\n", c);
                return c - '0';
            }
        }

        // Print countdown update once per second
        int remaining = timeoutSeconds - static_cast<int>((millis() - startMillis) / 1000);
        if (remaining != lastRemaining && remaining > 0) {
            lastRemaining = remaining;
            Serial.printf("%ds ", remaining);
        }

        delay(15);
    }

    Serial.println();
    Serial.println(F(">> Timeout elapsed. Proceeding to Main Script [4]...\r\n"));
    return 4;
}

void runI2CScanner() {
    Serial.println(F("----------------------------------------------------"));
    Serial.println(F("             [1] I2C BUS SCANNER                    "));
    Serial.println(F("----------------------------------------------------"));
    Serial.println(F("Scanning I2C "));

    Wire.begin();

    byte count = 0;
    for (byte address = 1; address < 127; ++address) {
        Wire.beginTransmission(address);
        byte error = Wire.endTransmission();

        if (error == 0) {
            Serial.printf(" -> Found device at 0x%02X", address);
            if (address == 0x28) {
                Serial.print(F(" [BNO055 IMU default]"));
            } else if (address == 0x29) {
                Serial.print(F(" [BNO055 IMU alt / VL53L0X]"));
            } else if (address == 0x48 || address == 0x49) {
                Serial.print(F(" [ADS1115 ADC / TMP102]"));
            } else if (address == 0x68) {
                Serial.print(F(" [MPU6050 / DS3231 RTC]"));
            }
            Serial.println();
            count++;
        } else if (error == 4) {
            Serial.printf(" -> Unknown error at address 0x%02X\r\n", address);
        }
    }

    if (count == 0) {
        Serial.println(F("No I2C devices found. Check SDA/SCL wiring, pull-ups, and 3.3V power."));
    } else {
        Serial.printf("Scan complete. %d device(s) detected.\r\n", count);
    }

    Serial.println(F("\nOptions: [r] Rescan  |  [m] Startup Menu  |  [x] Main Script"));
    while (true) {
        if (Serial.available() > 0) {
            char c = static_cast<char>(Serial.read());
            if (c == 'r' || c == 'R') {
                runI2CScanner();
                return;
            } else if (c == 'm' || c == 'M' || c == 'x' || c == 'X' || c == 'q' || c == 'Q' || c == '4') {
                return;
            }
        }
        delay(15);
    }
}

void runMotorTest(MotorController &motors) {
    Serial.println(F("----------------------------------------------------"));
    Serial.println(F("        [2] FOUR-MOTOR TEST SEQUENCE                "));
    Serial.println(F("----------------------------------------------------"));
    Serial.println(F("Testing Motors 1..4 (Cytron Maker Drive dual-PWM)"));
    Serial.println(F("Motors: M1={23,22}, M2={12,11}, M3={9,8}, M4={7,6}"));
    Serial.println(F("Note: M1/M2/M3 run INVERTED, M4 normal (setInverted)"));
    Serial.println(F("Press ANY key to abort sequence at any time.\n"));

    const int TEST_SPEED = 100;
    const int RUN_TIME_MS = 1500;
    const int PAUSE_TIME_MS = 1000;

    for (int i = 0; i < MOTOR_COUNT; i++) {
        // Forward test
        Serial.printf("Motor %d Forward (PWM: %d)...\r\n", i + 1, TEST_SPEED);
        motors.driveMotor(i, TEST_SPEED);

        unsigned long start = millis();
        while (millis() - start < (unsigned long)RUN_TIME_MS) {
            if (Serial.available() > 0) {
                Serial.read();
                motors.stop();
                Serial.println(F("Aborted by user input.\n"));
                return;
            }
            delay(10);
        }

        motors.stopMotor(i);
        delay(PAUSE_TIME_MS);

        // Reverse test
        Serial.printf("Motor %d Reverse (PWM: %d)...\r\n", i + 1, -TEST_SPEED);
        motors.driveMotor(i, -TEST_SPEED);

        start = millis();
        while (millis() - start < (unsigned long)RUN_TIME_MS) {
            if (Serial.available() > 0) {
                Serial.read();
                motors.stop();
                Serial.println(F("Aborted by user input.\n"));
                return;
            }
            delay(10);
        }

        motors.stopMotor(i);
        delay(PAUSE_TIME_MS);
    }

    motors.stop();
    Serial.println(F("Four-motor test sequence finished successfully.\n"));
    Serial.println(F("Options: [r] Repeat Test  |  [m] Startup Menu  |  [x] Main Script"));
    while (true) {
        if (Serial.available() > 0) {
            char c = static_cast<char>(Serial.read());
            if (c == 'r' || c == 'R') {
                runMotorTest(motors);
                return;
            } else if (c == 'm' || c == 'M' || c == 'x' || c == 'X' || c == 'q' || c == 'Q' || c == '4') {
                return;
            }
        }
        delay(15);
    }
}

void runBNOTest(BNO055_7Semi &bno) {
    Serial.println(F("----------------------------------------------------"));
    Serial.println(F("    [3] BNO055 IMU DIAGNOSTICS & STREAM TEST        "));
    Serial.println(F("----------------------------------------------------"));
    Serial.println(F("Connecting to BNO055 (Wire: A4/A5, 100kHz I2C)..."));

    if (!bno.begin(Wire, A4, A5, true, 100000)) {
        Serial.println(F("ERR: BNO055 not responding at address 0x28 or 0x29!"));
        Serial.println(F("Troubleshooting tips:"));
        Serial.println(F("  - Check SDA (Pin 18) and SCL (Pin 19) wiring."));
        Serial.println(F("  - Ensure 3.3V and GND are firmly connected."));
        Serial.println(F("  - Run [1] I2C Scanner to check if any peripheral is visible."));
        Serial.println(F("\nOptions: [m] Startup Menu  |  [x] Main Script"));
        while (true) {
            if (Serial.available() > 0) {
                char c = static_cast<char>(Serial.read());
                if (c == 'm' || c == 'M' || c == 'x' || c == 'X' || c == 'q' || c == 'Q') {
                    return;
                }
            }
            delay(15);
        }
    }

    delay(100);
    bno.setMode(Mode::NDOF);

    uint8_t chipId = bno.readReg(REG_CHIP_ID);
    int8_t temp = bno.temperatureC();
    Serial.printf("BNO055 Initialized! Chip ID: 0x%02X (Expected: 0xA0) | Temp: %d°C\r\n", chipId, temp);
    Serial.println(F("Streaming live Euler angles & calibration breakdown."));
    Serial.println(F("Press 'q' or 'x' to stop streaming and exit.\n"));

    while (true) {
        if (Serial.available() > 0) {
            char c = static_cast<char>(Serial.read());
            if (c == 'q' || c == 'Q' || c == 'x' || c == 'X' || c == 'm' || c == 'M' || c == '4') {
                break;
            }
        }

        float heading, roll, pitch;
        uint8_t sys, gyr, acc, mag;
        bno.calibBreakdown(sys, gyr, acc, mag);

        if (bno.readEuler(heading, roll, pitch)) {
            Serial.printf("Heading: %6.1f° | Roll: %6.1f° | Pitch: %6.1f° | CAL: [Sys:%d G:%d A:%d M:%d]\r\n",
                          heading, roll, pitch, sys, gyr, acc, mag);
        }
        delay(50); // ~20 Hz readable diagnostic stream
    }

    Serial.println(F("\nBNO test stream exited.\n"));
}
