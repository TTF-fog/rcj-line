#include <Arduino.h>
#include <7Semi_BNO055.h>
#include <Adafruit_NeoPixel.h>
#include "MotorController.h"
#include "ServoClaw.h"
#include "Subscripts.h"
#include "BoardPins.h"

// =============================================================================
// UART Configuration (per-board pins in BoardPins.h)
// =============================================================================
// Primary UART (USB Serial) for CLI commands, startup menu & IMU telemetry
#define IMU_UART    Serial

// Second UART bus for external controller (Motion 2350 Pro: GP0/GP1)
#define MOTOR_UART  Serial1
constexpr uint32_t MOTOR_UART_BAUD = 115200;

// =============================================================================
// RGB pixels (BoardPins.h):
//  - Onboard 2x WS2812 on GP23 = STATUS indicators (color-coded: boot white,
//    cyan gyro-hold, amber spin, green TDONE flash, red ESTOP,
//    magenta blink = IMU missing)
//  - LF white light bar on GP18 = LIGHT-command sink
// ============================================================================
Adafruit_NeoPixel statusPix(STATUS_PIXEL_COUNT, STATUS_PIXEL_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel barPix(EXT_NEOPIXEL_COUNT, EXT_NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// Last LIGHT-commanded color (mirrored to the onboard status pixels when idle)
static uint8_t userR = 255, userG = 255, userB = 255, userBri = 255;
static bool userOn = true;
// Status-engine event latches
static unsigned long estopLatchMs = 0;     // red solid after ESTOP / all-stop
static unsigned long tdoneFlashMs = 0;     // green flash after a gyro turn completes
static unsigned long imuFailFlashMs = 0;   // magenta flash on IMU-loss turn abort

// Motor pin configuration per board (BoardPins.h).
static const MotorPinConfig motorConfigs[MOTOR_COUNT] = {
    {PIN_M1B, PIN_M1A},  // Motor 1
    {PIN_M2A, PIN_M2B},  // Motor 2
    {PIN_M3A, PIN_M3B},  // Motor 3
    {PIN_M4B, PIN_M4A}   // Motor 4
};

// Motor Controller Instance (Library-less)
MotorController motors(motorConfigs);

// Claw jaws (GP2/GP7) + pickup arm (GP5) servo group; angles tunable via CLAW.
ServoClaw claw;

// Create BNO055 IMU object
BNO055_7Semi bno;
bool imuAvailable = false;

/**
 * @brief Heading source for the gyro straight-hold P controller.
 * Supplies fused Euler yaw (0..360 deg, NDOF mode) to MotorController.
 */
bool imuHeadingProvider(float &headingDegOut) {
    if (!imuAvailable) return false;
    float heading, roll, pitch;
    if (!bno.readEuler(heading, roll, pitch)) return false;
    headingDegOut = heading;
    return true;
}

/**
 * @brief LIGHT-command sink: repaints the LF light bar (GP18) with the user
 * color and remembers it so the onboard status LEDs can mirror it while idle.
 */
bool applyPixel(uint8_t r, uint8_t g, uint8_t b, uint8_t bri) {
    userR = r; userG = g; userB = b; userBri = bri; userOn = bri > 0;
    barPix.setBrightness(bri);
    barPix.fill(barPix.Color(r, g, b));
    barPix.show();
    return true;
}

/**
 * @brief Onboard status-LED engine (2x WS2812 on GP23), evaluated at ~50 Hz.
 *
 * Priority: red ESTOP latch > green TDONE flash > magenta FL hang flash >
 * amber while a gyro turn is active > cyan while gyro straight-hold active >
 * magenta blink when the IMU is missing > user LIGHT color when idle.
 */
void updateStatusLeds(bool straightActive, bool turnActive) {
    static uint8_t lastR = 99, lastG = 99, lastB = 99, lastBri = 99;
    unsigned long now = millis();

    uint8_t r = 0, g = 0, b = 0, bri = 0;
    if (estopLatchMs > now)            { r = 255;   bri = 255; }             // ESTOP: red
    else if (tdoneFlashMs > now)       { g = 255;   bri = 255; }             // TDONE: green flash
    else if (imuFailFlashMs > now)     { r = 255; b = 255; bri = 255; }      // FL hang: magenta
    else if (turnActive)               { r = 255; g = 200; b = 0; bri = 255; }  // spinning: amber
    else if (straightActive)           { r = 0; g = 180; b = 255; bri = 255; }  // gyro hold: cyan
    else if (!imuAvailable) {                                               // no IMU: magenta blink
        if ((millis() / 400) & 1) { r = 255; b = 255; bri = 255; }
    } else {                                                                // idle: user color
        if (userOn) { r = userR; g = userG; b = userB; bri = userBri; }
    }

    if (r == lastR && g == lastG && b == lastB && bri == lastBri) return;
    lastR = r; lastG = g; lastB = b; lastBri = bri;
    statusPix.setBrightness(bri);
    statusPix.fill(statusPix.Color(r, g, b));
    statusPix.show();
}

void initMainScript() {
    IMU_UART.println(F("----------------------------------------------------"));
    IMU_UART.println(F("       [4] MAIN ROBOT SCRIPT OPERATIONAL            "));
    IMU_UART.println(F("----------------------------------------------------"));
    IMU_UART.println(F("# 4-Motor Controller Ready (Library-less)"));
    Serial.printf("# Board: %s\r\n", BOARD_NAME_TEXT);
    Serial.printf("# Pins: %s\r\n", PIN_DESC_TEXT);
    Serial.printf("# IMU: %s\r\n", IMU_WIRING_TEXT);
    Serial.printf("# UART: %s\r\n", UART_TEXT);
    IMU_UART.println(F("# Motors 1/2/3 INVERTED, Motor 4 normal"));
    IMU_UART.println(F("# Gyro straight-hold: GFWD/GBACK/GSTOP/GKP/GMAX/GSTATUS"));
    IMU_UART.println(F("# Gyro turns: GTURN/GLEFT/GRIGHT/GTL90/GTR90/GTL45/GTR45"));
    IMU_UART.printf("# Speed cap: %d/%d (all sources, incl. gyro mix)\r\n", MAX_SPEED, 255);
    IMU_UART.printf("# Gyro turn PID: GTKP/GTKI/GTKD/GTIMAX, tune per docs/PID_TUNING.md\r\n");
    IMU_UART.println(F(""));
    IMU_UART.printf("# LIGHT -> white bar on GP%d | status LEDs GP%d (onboard)\r\n", EXT_NEOPIXEL_PIN, STATUS_PIXEL_PIN);
    IMU_UART.printf("# Buzzer GP%d: BEEP <ms> [hz] | User btn GP%d = ESTOP (red LEDs + chirps)\r\n", BUZZER_PIN, USER_BTN1_PIN);
    IMU_UART.println(F("# BOOTLOAD: reboot into UF2 bootloader for flashing"));
    IMU_UART.println(F("# Claw servos: left=GP2, right=GP7, pickup=GP5 (CLAW OPEN/CLOSE/PICK/DROP/SEQ)"));

    MOTOR_UART.println(F("# 4-Motor Controller Ready (Library-less)"));
    MOTOR_UART.printf("# Board: %s\r\n", BOARD_NAME_TEXT);
    MOTOR_UART.printf("# Pins: %s\r\n", PIN_DESC_TEXT);
    MOTOR_UART.println(F("# Claw: left=GP2, right=GP7, pickup=GP5 (CLAW OPEN/CLOSE/PICK/DROP/SEQ)"));
    MOTOR_UART.println(F("# Type 'HELP' for available motor commands"));

    // Initialize IMU non-blockingly on the board's default I2C0 (GP16/GP17)
    imuAvailable = bno.begin(IMU_WIRE, IMU_SDA_PIN, IMU_SCL_PIN, true, 100000);
    if (!imuAvailable) {
        IMU_UART.println(F("# WARN: BNO055 not detected. Motors will continue to operate."));
        MOTOR_UART.println(F("# WARN: BNO055 not detected. Motors will continue to operate."));
        motors.beep(600, 990);   // long low tone: robot ready but blind
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

    // Boot lights: status LEDs white (onboard), light bar white
    statusPix.begin();
    statusPix.setBrightness(255);
    statusPix.fill(statusPix.Color(255, 255, 255));
    statusPix.show();
    barPix.begin();
    barPix.setBrightness(255);
    barPix.fill(barPix.Color(255, 255, 255));
    barPix.show();

    // Initialize Motor pins (pinMode OUTPUT, stopped)
    motors.begin();

    // Silent, high-resolution motor PWM on the RP2350 slices (before any write).
    analogWriteFreq(PWM_FREQ_HZ);
    analogWriteResolution(8);
    // Passive buzzer + user button 1 = momentary ESTOP.
    motors.setBuzzerPin(BUZZER_PIN);
    pinMode(USER_BTN1_PIN, INPUT_PULLUP);   // low-active (their pull-up is fine)
    pinMode(USER_BTN2_PIN, INPUT_PULLUP);

    // Claw jaws + pickup arm: attach and go to the stowed pose.
    motors.setClaw(&claw);
    claw.begin();

    // Flip Motor 1/2/3 direction; Motor 4 stays normal.
    motors.setInverted(true, true, true, false);

    // Wire gyro straight-hold P controller to the BNO055 fused heading.
    // Default Kp = 2.0 PWM per degree of heading error.
    motors.setHeadingProvider(imuHeadingProvider);
    motors.setPixelHandler(applyPixel);
    motors.setStraightKp(2.0f);
    motors.setStraightMaxCorrection(80.0f);

    // Gyro turn PID gains (tuning procedure: docs/PID_TUNING.md).
    // Anti-wobble set: brake-only Kd damps the approach, a modest Kp/Ki
    // avoids pumping energy into the swing, and the stall floor only acts
    // far from the target (see GyroTurn.h).
    motors.setTurnKp(3.0f);
    motors.setTurnKi(1.0f);
    motors.setTurnKd(1.0f);
    motors.setTurnIntegralMax(20.0f);
    motors.setTurnMinPwm(60.0f);
    motors.setTurnTolerance(2.0f);

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

    // 1a. End beep patterns when their time is up, and treat a momentary
    // press of user button 1 (GP20) as a physical ESTOP (red LEDs + 3 chirps).
    motors.serviceBuzzer();
    claw.service();     // scoop-sequence stage machine (close->angle->lift->open)
    static unsigned long btnDebounceMs = 0;
    static bool btnArmed = true;
    bool btnDown = (digitalRead(USER_BTN1_PIN) == LOW);
    if (btnDown && btnArmed && (millis() - btnDebounceMs) > 50) {
        btnArmed = false;
        motors.stop();
        estopLatchMs = millis() + 1500;   // onboard LEDs: solid red 1.5 s
        motors.beep(110, 2730, 3);        // three urgent chirps
        IMU_UART.println(F("ESTOP BTN: user button pressed - all motors stopped"));
        MOTOR_UART.println(F("ESTOP BTN"));
    } else if (!btnDown) {
        btnArmed = true;
        btnDebounceMs = millis();
    }

    // 1b. Report a finished/aborted gyro turn once, on both UARTs.
    GyroTurnResult tr;
    if (motors.pollTurnDone(tr)) {
        if (tr.completed) {
            // printf %f is unavailable on this toolchain; chain prints instead.
            auto printTdone = [](Stream &s, const GyroTurnResult &t) {
                s.print(F("TDONE err="));
                s.print(t.finalErr, 2);
                s.print(F(" target="));
                s.print(t.target, 1);
                s.print(F(" head="));
                s.println(t.finalHeading, 1);
            };
            printTdone(IMU_UART, tr);
            printTdone(MOTOR_UART, tr);
            motors.beep(120, 3135);       // single chirp on turn completion
            tdoneFlashMs = millis() + 650;  // onboard LEDs: green flash
        } else {
            IMU_UART.println(F("TABORT imu"));
            MOTOR_UART.println(F("TABORT imu"));
            imuFailFlashMs = millis() + 700;  // onboard LEDs: magenta flash
        }
    }

    // 1c. Publish scoop-sequence completion once, on both UARTs.
    if (claw.pollSeqDone()) {
        IMU_UART.println(F("CLAWSEQ done"));
        MOTOR_UART.println(F("CLAWSEQ done"));
    }

    // 2. Read IMU data and stream over primary UART at ~50 Hz (every 20 ms)
    static unsigned long lastImuTime = 0;
    unsigned long currentMillis = millis();

    if (currentMillis - lastImuTime >= 20) {
        lastImuTime = currentMillis;

        // Gyro straight-hold P step: reads heading via imuHeadingProvider,
        // computes error = target - current, applies Kp correction to L/R.
        // No-op when idle or IMU offline.
        motors.updateStraight();

        // Gyro turn P step: spins in place toward the latched target heading,
        // brakes + publishes TDONE once inside tolerance. No-op when idle.
        motors.updateTurn();

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

        // Gyro-hold telemetry at ~5 Hz while active: G,heading,target,err,L,R
        if (motors.isStraightActive()) {
            static unsigned gTick = 0;
            if ((++gTick % 10) == 0) {
                float headingNow = 0.0f;
                float target = motors.getStraightTarget();
                if (imuHeadingProvider(headingNow)) {
                    float err = GyroStraight::wrapError(target, headingNow);
                    IMU_UART.print(F("G,"));
                    IMU_UART.print(headingNow, 1);
                    IMU_UART.print(',');
                    IMU_UART.print(target, 1);
                    IMU_UART.print(',');
                    IMU_UART.print(err, 1);
                    IMU_UART.print(F(",L="));
                    IMU_UART.print(motors.getLeftSpeed());
                    IMU_UART.print(F(",R="));
                    IMU_UART.print(motors.getRightSpeed());
                    IMU_UART.print(F(",Kp="));
                    IMU_UART.println(motors.getStraightKp(), 2);
                }
            }
        }

        // Gyro-turn telemetry at ~5 Hz while active: T,heading,target,err,L,R
        if (motors.isTurnActive()) {
            static unsigned tTick = 0;
            if ((++tTick % 10) == 0) {
                float headingNow = 0.0f;
                if (imuHeadingProvider(headingNow)) {
                    float err = GyroStraight::wrapError(motors.getTurnTarget(), headingNow);
                    IMU_UART.print(F("T,"));
                    IMU_UART.print(headingNow, 1);
                    IMU_UART.print(',');
                    IMU_UART.print(motors.getTurnTarget(), 1);
                    IMU_UART.print(',');
                    IMU_UART.print(err, 1);
                    IMU_UART.print(F(",L="));
                    IMU_UART.print(motors.getLeftSpeed());
                    IMU_UART.print(F(",R="));
                    IMU_UART.print(motors.getRightSpeed());
                    IMU_UART.print(F(",Kp="));
                    IMU_UART.println(motors.getTurnKp(), 2);
                }
            }
        }

        // LED heartbeat: toggle once per second (50 iterations @ 50 Hz)
        static bool led = false;
        static unsigned hb = 0;
        if ((++hb % 50) == 0) {
            led = !led;
            digitalWrite(LED_BUILTIN, led ? HIGH : LOW);
        }

        // Onboard status-LED engine: color by robot state (see updateStatusLeds)
        updateStatusLeds(motors.isStraightActive(), motors.isTurnActive());
    }
}