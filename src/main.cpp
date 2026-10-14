#define WIRE WIRE
#include <Arduino.h>
#include <7Semi_BNO055.h>
#include "MotorController.h"
#include "Subscripts.h"
#define WIRE Wire2

// =============================================================================
// UART Configuration
// =============================================================================
// Primary UART (USB Serial) for CLI commands, startup menu & IMU telemetry
#define IMU_UART    Serial

// Second UART bus (Teensy 4.1 Serial1: Pin 0 RX1, Pin 1 TX1) for external controller
#define MOTOR_UART  Serial1
constexpr uint32_t MOTOR_UART_BAUD = 115200;

// =============================================================================
// Four-Motor Pin Configuration from src/example.txt
// Two Cytron Maker Drive boards (Dual-PWM control, no external libraries)
// =============================================================================
const MotorPinConfig motorConfigs[MOTOR_COUNT] = {
    {23, 22},  // Motor 1 (Left)
    {12, 11},  // Motor 2 (Left)
    {9, 8},    // Motor 3 (Right)
    {7, 6}     // Motor 4 (Right)
};

// Motor Controller Instance (Library-less)
MotorController motors(motorConfigs);

// Create BNO055 IMU object
BNO055_7Semi bno;
bool imuAvailable = false;

void initMainScript() {
    IMU_UART.println(F("----------------------------------------------------"));
    IMU_UART.println(F("       [4] MAIN ROBOT SCRIPT OPERATIONAL            "));
    IMU_UART.println(F("----------------------------------------------------"));
    IMU_UART.println(F("# 4-Motor Controller Ready (Library-less)"));
    IMU_UART.println(F("# Pins: M1={23,22}, M2={12,11}, M3={9,8}, M4={7,6}"));
    IMU_UART.println(F("# Dual-UART active: USB Serial + Serial1"));

    MOTOR_UART.println(F("# 4-Motor Controller Ready (Library-less)"));
    MOTOR_UART.println(F("# Pins: M1={23,22}, M2={12,11}, M3={9,8}, M4={7,6}"));
    MOTOR_UART.println(F("# Type 'HELP' for available motor commands"));

    // Initialize IMU non-blockingly
    imuAvailable = bno.begin(WIRE, A4, A5, true, 100000);
    if (!imuAvailable) {
        IMU_UART.println(F("# WARN: BNO055 not detected. Motors will continue to operate."));
        MOTOR_UART.println(F("# WARN: BNO055 not detected. Motors will continue to operate."));
    } else {
        delay(100);
        bno.setMode(Mode::NDOF);
        IMU_UART.println(F("# 7SemiVLab Quaternion Streamer Ready"));
        IMU_UART.println(F("# Format: Q,w,x,y,z,CAL,sys,g,a,m"));
    }
}

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);

    // Initialize USB and Hardware serial buses
    IMU_UART.begin(115200);
    MOTOR_UART.begin(MOTOR_UART_BAUD);

    // Initialize Motor pins (pinMode OUTPUT, stopped)
    motors.begin();

    // Brief settling delay for USB enumeration if plugged into PC
    unsigned long waitSerial = millis();
    while (!Serial && (millis() - waitSerial < 1000));

    // Interactive Startup Menu with 5-second countdown timeout
    int selectedMode = promptStartupMenu(5);
    while (selectedMode != 4) {
        if (selectedMode == 1) {
            runI2CScanner();
        } else if (selectedMode == 2) {
            runMotorTest(motors);
        } else if (selectedMode == 3) {
            runBNOTest(bno);
        }
        // After completing diagnostic subscript, allow choosing another or proceeding
        selectedMode = promptStartupMenu(5);
    }

    // Launch Main Script
    initMainScript();
}

void loop() {
    // 1. Service incoming motor commands from BOTH USB Serial AND Hardware Serial1!
    motors.processUartCommands(Serial);
    motors.processUartCommands(MOTOR_UART);

    // 2. Read IMU data and stream over primary UART at ~50 Hz (every 20 ms)
    static unsigned long lastImuTime = 0;
    unsigned long currentMillis = millis();

    if (currentMillis - lastImuTime >= 20) {
        lastImuTime = currentMillis;

        if (imuAvailable) {
            uint8_t sys, g, a, m;
            bno.calibBreakdown(sys, g, a, m);

            float w, x, y, z;
            if (bno.readQuat(w, x, y, z)) {
                IMU_UART.print(F("Q,"));
                IMU_UART.print(w, 6); IMU_UART.print(',');
                IMU_UART.print(x, 6); IMU_UART.print(',');
                IMU_UART.print(y, 6); IMU_UART.print(',');
                IMU_UART.print(z, 6);
                IMU_UART.print(F(",CAL,"));
                IMU_UART.print(sys); IMU_UART.print(',');
                IMU_UART.print(g); IMU_UART.print(',');
                IMU_UART.print(a); IMU_UART.print(',');
                IMU_UART.println(m);
            }
        }

        // LED heartbeat: toggle once per second (50 iterations @ 50 Hz)
        static bool led = false;
        static unsigned hb = 0;
        if ((++hb % 50) == 0) {
            led = !led;
            digitalWrite(LED_BUILTIN, led ? HIGH : LOW);
        }
    }
}