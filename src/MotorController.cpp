#include "MotorController.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

MotorController::MotorController(const MotorPinConfig configs[MOTOR_COUNT]) {
    for (int i = 0; i < MOTOR_COUNT; i++) {
        _motors[i] = configs[i];
        _speeds[i] = 0;
        _inverted[i] = false;
    }
    memset(_usbCmdBuffer, 0, sizeof(_usbCmdBuffer));
    _usbCmdBufferPos = 0;
    memset(_hwCmdBuffer, 0, sizeof(_hwCmdBuffer));
    _hwCmdBufferPos = 0;
}

void MotorController::begin() {
    for (int i = 0; i < MOTOR_COUNT; i++) {
        pinMode(_motors[i].pin1, OUTPUT);
        pinMode(_motors[i].pin2, OUTPUT);
        stopMotor(i);
    }
}

void MotorController::setInverted(bool inv1, bool inv2, bool inv3, bool inv4) {
    _inverted[0] = inv1;
    _inverted[1] = inv2;
    _inverted[2] = inv3;
    _inverted[3] = inv4;
}

void MotorController::setHeadingProvider(HeadingProviderFn fn) {
    _headingProvider = fn;
}

void MotorController::setStraightKp(float kp) {
    _straight.setKp(kp);
}

float MotorController::getStraightKp() const {
    return _straight.kp();
}

void MotorController::setStraightMaxCorrection(float maxCorrection) {
    _straight.setMaxCorrection(maxCorrection);
}

float MotorController::getStraightMaxCorrection() const {
    return _straight.maxCorrection();
}

void MotorController::startStraight(int16_t baseSpeed, float targetHeadingDeg) {
    _straight.start(baseSpeed, targetHeadingDeg);
    // Immediately apply zero-error mix so motors start moving without
    // waiting for the next updateStraight() tick.
    int16_t left, right;
    if (_straight.compute(targetHeadingDeg, left, right)) {
        if (baseSpeed > 0 && allStopped()) {
            // Forward kick-off from standstill: stagger power-ups to
            // spread the inrush peak.
            driveFourStaggered(left, left, right, right);
        } else {
            setMotorSpeeds(left, right);
        }
    }
}

bool MotorController::startStraightHere(int16_t baseSpeed) {
    if (!_headingProvider) return false;
    float heading = 0.0f;
    if (!_headingProvider(heading)) return false;
    startStraight(baseSpeed, heading);
    return true;
}

void MotorController::stopStraight(bool stopMotors) {
    _straight.stop();
    if (stopMotors) {
        stop();
    }
}

bool MotorController::isStraightActive() const {
    return _straight.isActive();
}

int16_t MotorController::getStraightBase() const {
    return _straight.baseSpeed();
}

float MotorController::getStraightTarget() const {
    return _straight.target();
}

void MotorController::updateStraight() {
    if (!_straight.isActive()) return;
    if (!_headingProvider) return;
    float heading = 0.0f;
    if (!_headingProvider(heading)) return;  // IMU glitch: hold last output
    int16_t left, right;
    if (_straight.compute(heading, left, right)) {
        setMotorSpeeds(left, right);
    }
}

void MotorController::stopMotor(int motorIndex) {
    if (motorIndex < 0 || motorIndex >= MOTOR_COUNT) return;
    _speeds[motorIndex] = 0;
    analogWrite(_motors[motorIndex].pin1, 0);
    analogWrite(_motors[motorIndex].pin2, 0);
}

void MotorController::brakeMotor(int motorIndex) {
    if (motorIndex < 0 || motorIndex >= MOTOR_COUNT) return;
    _speeds[motorIndex] = 0;
    // Active short-brake: both inputs HIGH
    analogWrite(_motors[motorIndex].pin1, 255);
    analogWrite(_motors[motorIndex].pin2, 255);
}

void MotorController::driveMotor(int motorIndex, int16_t speed) {
    if (motorIndex < 0 || motorIndex >= MOTOR_COUNT) return;

    if (speed > 255) speed = 255;
    if (speed < -255) speed = -255;

    _speeds[motorIndex] = speed;

    int16_t appliedSpeed = _inverted[motorIndex] ? -speed : speed;

    // Library-less dual-PWM control (Cytron Maker Drive logic matching src/example.txt)
    if (appliedSpeed > 0) {
        analogWrite(_motors[motorIndex].pin1, appliedSpeed);
        analogWrite(_motors[motorIndex].pin2, 0);
    } else if (appliedSpeed < 0) {
        analogWrite(_motors[motorIndex].pin1, 0);
        analogWrite(_motors[motorIndex].pin2, -appliedSpeed);
    } else {
        analogWrite(_motors[motorIndex].pin1, 0);
        analogWrite(_motors[motorIndex].pin2, 0);
    }
}

void MotorController::setMotorSpeed(int motorIndex, int16_t speed) {
    driveMotor(motorIndex, speed);
}

void MotorController::setFourMotorSpeeds(int16_t m1, int16_t m2, int16_t m3, int16_t m4) {
    driveMotor(0, m1);
    driveMotor(1, m2);
    driveMotor(2, m3);
    driveMotor(3, m4);
}

bool MotorController::allStopped() const {
    for (int i = 0; i < MOTOR_COUNT; i++) {
        if (_speeds[i] != 0) return false;
    }
    return true;
}

void MotorController::driveFourStaggered(int16_t m1, int16_t m2, int16_t m3, int16_t m4) {
    driveMotor(0, m1);
    delay(START_STAGGER_MS);
    driveMotor(1, m2);
    delay(START_STAGGER_MS);
    driveMotor(2, m3);
    delay(START_STAGGER_MS);
    driveMotor(3, m4);
}

void MotorController::setLeftSpeed(int16_t speed) {
    // Left side: Motor 1 (idx 0) and Motor 2 (idx 1)
    driveMotor(0, speed);
    driveMotor(1, speed);
}

void MotorController::setRightSpeed(int16_t speed) {
    // Right side: Motor 3 (idx 2) and Motor 4 (idx 3)
    driveMotor(2, speed);
    driveMotor(3, speed);
}

void MotorController::setMotorSpeeds(int16_t leftSpeed, int16_t rightSpeed) {
    setLeftSpeed(leftSpeed);
    setRightSpeed(rightSpeed);
}

void MotorController::moveForward(int16_t speed) {
    int16_t s = abs(speed);
    if (allStopped() && s != 0) {
        // From-standstill forward start: stagger power-ups (M1..M4 with
        // START_STAGGER_MS gaps) to spread the inrush peak.
        driveFourStaggered(s, s, s, s);
    } else {
        setMotorSpeeds(s, s);
    }
}

void MotorController::moveBackward(int16_t speed) {
    int16_t s = abs(speed);
    setMotorSpeeds(-s, -s);
}

void MotorController::steer(int16_t speed, float turnRatio) {
    if (speed > 255) speed = 255;
    if (speed < -255) speed = -255;

    // Clamp turnRatio between -1.0 (full left spin) and +1.0 (full right spin)
    if (turnRatio > 1.0f) turnRatio = 1.0f;
    if (turnRatio < -1.0f) turnRatio = -1.0f;

    float leftFactor = 1.0f;
    float rightFactor = 1.0f;

    if (turnRatio > 0.0f) {
        // Turning right: scale down / reverse right side
        rightFactor = 1.0f - 2.0f * turnRatio;
    } else if (turnRatio < 0.0f) {
        // Turning left: scale down / reverse left side
        leftFactor = 1.0f + 2.0f * turnRatio;
    }

    int16_t leftSpeed = static_cast<int16_t>(roundf(speed * leftFactor));
    int16_t rightSpeed = static_cast<int16_t>(roundf(speed * rightFactor));

    setMotorSpeeds(leftSpeed, rightSpeed);
}

void MotorController::turnLeft(int16_t speed) {
    int16_t s = abs(speed);
    setMotorSpeeds(-s, s);
}

void MotorController::turnRight(int16_t speed) {
    int16_t s = abs(speed);
    setMotorSpeeds(s, -s);
}

void MotorController::stop() {
    for (int i = 0; i < MOTOR_COUNT; i++) {
        stopMotor(i);
    }
}

void MotorController::brake() {
    for (int i = 0; i < MOTOR_COUNT; i++) {
        brakeMotor(i);
    }
}

int16_t MotorController::getMotorSpeed(int motorIndex) const {
    if (motorIndex < 0 || motorIndex >= MOTOR_COUNT) return 0;
    return _speeds[motorIndex];
}

int16_t MotorController::getLeftSpeed() const {
    return (_speeds[0] + _speeds[1]) / 2;
}

int16_t MotorController::getRightSpeed() const {
    return (_speeds[2] + _speeds[3]) / 2;
}

void MotorController::processUartCommands(Stream &serialStream) {
    char *buf;
    size_t *pos;

    // Use stream-specific buffer to avoid interleaving between USB and Hardware UART
    if (&serialStream == &Serial) {
        buf = _usbCmdBuffer;
        pos = &_usbCmdBufferPos;
    } else {
        buf = _hwCmdBuffer;
        pos = &_hwCmdBufferPos;
    }

    while (serialStream.available() > 0) {
        char c = static_cast<char>(serialStream.read());
        if (c == '\r') {
            continue; // Skip CR
        }
        if (c == '\n') {
            buf[*pos] = '\0';
            if (*pos > 0) {
                parseAndExecuteCommand(buf, serialStream);
                *pos = 0;
            }
        } else {
            if (*pos < (CMD_BUF_LEN - 1)) {
                buf[(*pos)++] = c;
            } else {
                *pos = 0;
            }
        }
    }
}

void MotorController::parseAndExecuteCommand(const char *cmd, Stream &serialStream) {
    while (*cmd == ' ' || *cmd == '\t') cmd++;
    if (*cmd == '\0') return;

    char cmdCopy[CMD_BUF_LEN];
    strncpy(cmdCopy, cmd, sizeof(cmdCopy) - 1);
    cmdCopy[sizeof(cmdCopy) - 1] = '\0';

    char *savePtr = nullptr;
    char *token = strtok_r(cmdCopy, " \t", &savePtr);
    if (!token) return;

    if (strcasecmp(token, "SPEED") == 0) {
        char *arg1 = strtok_r(nullptr, " \t", &savePtr);
        char *arg2 = strtok_r(nullptr, " \t", &savePtr);
        if (arg1 && arg2) {
            int left = atoi(arg1);
            int right = atoi(arg2);
            _straight.stop();  // manual drive cancels gyro straight-hold
            setMotorSpeeds(left, right);
            serialStream.printf("OK: SPEED %d %d\r\n", left, right);
        } else {
            serialStream.println(F("ERR: Usage: SPEED <left [-255..255]> <right [-255..255]>"));
        }
    } else if (strcasecmp(token, "MOTOR") == 0 || strcasecmp(token, "M") == 0) {
        char *argIdx = strtok_r(nullptr, " \t", &savePtr);
        char *argSpd = strtok_r(nullptr, " \t", &savePtr);
        if (argIdx && argSpd) {
            int idx = atoi(argIdx);
            int spd = atoi(argSpd);
            // Accept 1-based (1..4) or 0-based (0..3)
            int internalIdx = (idx >= 1 && idx <= 4) ? (idx - 1) : idx;
            if (internalIdx >= 0 && internalIdx < MOTOR_COUNT) {
                _straight.stop();  // manual drive cancels gyro straight-hold
                driveMotor(internalIdx, spd);
                serialStream.printf("OK: MOTOR %d %d\r\n", internalIdx + 1, spd);
            } else {
                serialStream.println(F("ERR: Motor index must be 1..4"));
            }
        } else {
            serialStream.println(F("ERR: Usage: MOTOR <1..4> <speed>"));
        }
    } else if (strcasecmp(token, "MOTORS") == 0) {
        char *a1 = strtok_r(nullptr, " \t", &savePtr);
        char *a2 = strtok_r(nullptr, " \t", &savePtr);
        char *a3 = strtok_r(nullptr, " \t", &savePtr);
        char *a4 = strtok_r(nullptr, " \t", &savePtr);
        if (a1 && a2 && a3 && a4) {
            int m1 = atoi(a1);
            int m2 = atoi(a2);
            int m3 = atoi(a3);
            int m4 = atoi(a4);
            _straight.stop();  // manual drive cancels gyro straight-hold
            setFourMotorSpeeds(m1, m2, m3, m4);
            serialStream.printf("OK: MOTORS %d %d %d %d\r\n", m1, m2, m3, m4);
        } else {
            serialStream.println(F("ERR: Usage: MOTORS <m1> <m2> <m3> <m4>"));
        }
    } else if (strcasecmp(token, "FWD") == 0 || strcasecmp(token, "FORWARD") == 0) {
        char *arg = strtok_r(nullptr, " \t", &savePtr);
        if (arg) {
            int spd = atoi(arg);
            _straight.stop();  // manual drive cancels gyro straight-hold
            moveForward(spd);
            serialStream.printf("OK: FWD %d\r\n", spd);
        } else {
            serialStream.println(F("ERR: Usage: FWD <speed [0..255]>"));
        }
    } else if (strcasecmp(token, "BACK") == 0 || strcasecmp(token, "REV") == 0 || strcasecmp(token, "BACKWARD") == 0) {
        char *arg = strtok_r(nullptr, " \t", &savePtr);
        if (arg) {
            int spd = atoi(arg);
            _straight.stop();  // manual drive cancels gyro straight-hold
            moveBackward(spd);
            serialStream.printf("OK: BACK %d\r\n", spd);
        } else {
            serialStream.println(F("ERR: Usage: BACK <speed [0..255]>"));
        }
    } else if (strcasecmp(token, "STEER") == 0) {
        char *arg1 = strtok_r(nullptr, " \t", &savePtr);
        char *arg2 = strtok_r(nullptr, " \t", &savePtr);
        if (arg1 && arg2) {
            int spd = atoi(arg1);
            float ratio = static_cast<float>(atof(arg2));
            _straight.stop();  // manual drive cancels gyro straight-hold
            steer(spd, ratio);
            serialStream.printf("OK: STEER %d %.2f (L=%d, R=%d)\r\n", spd, ratio, getLeftSpeed(), getRightSpeed());
        } else {
            serialStream.println(F("ERR: Usage: STEER <speed [-255..255]> <ratio [-1.0..1.0]>"));
        }
    } else if (strcasecmp(token, "LEFT") == 0) {
        char *arg = strtok_r(nullptr, " \t", &savePtr);
        if (arg) {
            int spd = atoi(arg);
            _straight.stop();  // manual drive cancels gyro straight-hold
            turnLeft(spd);
            serialStream.printf("OK: LEFT %d\r\n", spd);
        } else {
            serialStream.println(F("ERR: Usage: LEFT <speed [0..255]>"));
        }
    } else if (strcasecmp(token, "RIGHT") == 0) {
        char *arg = strtok_r(nullptr, " \t", &savePtr);
        if (arg) {
            int spd = atoi(arg);
            _straight.stop();  // manual drive cancels gyro straight-hold
            turnRight(spd);
            serialStream.printf("OK: RIGHT %d\r\n", spd);
        } else {
            serialStream.println(F("ERR: Usage: RIGHT <speed [0..255]>"));
        }
    } else if (strcasecmp(token, "STOP") == 0) {
        _straight.stop();
        stop();
        serialStream.println(F("OK: STOP"));
    } else if (strcasecmp(token, "BRAKE") == 0) {
        _straight.stop();
        brake();
        serialStream.println(F("OK: BRAKE"));
    } else if (strcasecmp(token, "STATUS") == 0) {
        serialStream.printf("STATUS: M1=%d M2=%d M3=%d M4=%d L=%d R=%d\r\n",
                            _speeds[0], _speeds[1], _speeds[2], _speeds[3],
                            getLeftSpeed(), getRightSpeed());
    } else if (strcasecmp(token, "GFWD") == 0) {
        // Gyro straight-hold forward: latch current heading as target.
        char *arg = strtok_r(nullptr, " \t", &savePtr);
        char *argKp = strtok_r(nullptr, " \t", &savePtr);
        if (arg) {
            int spd = atoi(arg);
            if (spd < 0) spd = -spd;
            if (spd > 255) spd = 255;
            if (argKp) _straight.setKp(static_cast<float>(atof(argKp)));
            if (startStraightHere(spd)) {
                serialStream.printf("OK: GFWD %d (target=%.1f, Kp=%.2f)\r\n",
                                    spd, getStraightTarget(), getStraightKp());
            } else {
                serialStream.println(F("ERR: GFWD failed (no heading: IMU offline?)"));
            }
        } else {
            serialStream.println(F("ERR: Usage: GFWD <speed [0..255]> [kp]"));
        }
    } else if (strcasecmp(token, "GBACK") == 0) {
        // Gyro straight-hold reverse: latch current heading as target.
        char *arg = strtok_r(nullptr, " \t", &savePtr);
        char *argKp = strtok_r(nullptr, " \t", &savePtr);
        if (arg) {
            int spd = atoi(arg);
            if (spd < 0) spd = -spd;
            if (spd > 255) spd = 255;
            if (argKp) _straight.setKp(static_cast<float>(atof(argKp)));
            if (startStraightHere(-spd)) {
                serialStream.printf("OK: GBACK %d (target=%.1f, Kp=%.2f)\r\n",
                                    spd, getStraightTarget(), getStraightKp());
            } else {
                serialStream.println(F("ERR: GBACK failed (no heading: IMU offline?)"));
            }
        } else {
            serialStream.println(F("ERR: Usage: GBACK <speed [0..255]> [kp]"));
        }
    } else if (strcasecmp(token, "GSTOP") == 0) {
        stopStraight(true);
        serialStream.println(F("OK: GSTOP"));
    } else if (strcasecmp(token, "GKP") == 0) {
        char *arg = strtok_r(nullptr, " \t", &savePtr);
        if (arg) {
            float kp = static_cast<float>(atof(arg));
            if (kp < 0.0f) kp = 0.0f;
            if (kp > 20.0f) kp = 20.0f;
            setStraightKp(kp);
            serialStream.printf("OK: GKP %.2f\r\n", getStraightKp());
        } else {
            serialStream.println(F("ERR: Usage: GKP <0.0..20.0>"));
        }
    } else if (strcasecmp(token, "GMAX") == 0) {
        char *arg = strtok_r(nullptr, " \t", &savePtr);
        if (arg) {
            float m = static_cast<float>(atof(arg));
            if (m < 0.0f) m = 0.0f;
            if (m > 255.0f) m = 255.0f;
            setStraightMaxCorrection(m);
            serialStream.printf("OK: GMAX %.1f\r\n", getStraightMaxCorrection());
        } else {
            serialStream.println(F("ERR: Usage: GMAX <0..255>"));
        }
    } else if (strcasecmp(token, "GSTATUS") == 0) {
        float heading = 0.0f;
        bool haveHeading = (_headingProvider && _headingProvider(heading));
        if (isStraightActive()) {
            float err = haveHeading ? GyroStraight::wrapError(getStraightTarget(), heading) : 0.0f;
            serialStream.printf("GSTATUS: ACTIVE base=%d target=%.1f Kp=%.2f max=%.1f head=%.1f err=%.1f L=%d R=%d\r\n",
                                getStraightBase(), getStraightTarget(), getStraightKp(),
                                getStraightMaxCorrection(),
                                haveHeading ? heading : -1.0f, err,
                                getLeftSpeed(), getRightSpeed());
        } else {
            serialStream.printf("GSTATUS: IDLE Kp=%.2f max=%.1f head=%.1f L=%d R=%d\r\n",
                                getStraightKp(), getStraightMaxCorrection(),
                                haveHeading ? heading : -1.0f,
                                getLeftSpeed(), getRightSpeed());
        }
    } else if (strcasecmp(token, "PING") == 0) {
        serialStream.println(F("PONG"));
    } else if (strcasecmp(token, "HELP") == 0) {
        serialStream.println(F("--- 4-Motor Driver Commands (No Library) ---"));
        serialStream.println(F("  SPEED <L> <R>        : Set left & right side speeds (-255..255)"));
        serialStream.println(F("  MOTOR <1..4> <spd>   : Set individual motor speed (-255..255)"));
        serialStream.println(F("  MOTORS <m1..m4>      : Set all 4 motor speeds directly"));
        serialStream.println(F("  FWD <spd>            : Move forward (0..255)"));
        serialStream.println(F("  BACK <spd>           : Move backward (0..255)"));
        serialStream.println(F("  STEER <spd> <ratio>  : Steer (ratio -1.0=Left .. +1.0=Right)"));
        serialStream.println(F("  LEFT <spd>           : Spin left in place"));
        serialStream.println(F("  RIGHT <spd>          : Spin right in place"));
        serialStream.println(F("  STOP                 : Coast all motors to stop"));
        serialStream.println(F("  BRAKE                : Active short-brake all motors"));
        serialStream.println(F("  STATUS               : Query motor speeds"));
        serialStream.println(F("  --- Gyro straight-hold (P on heading error) ---"));
        serialStream.println(F("  GFWD <spd> [kp]      : Drive straight fwd, heading latched"));
        serialStream.println(F("  GBACK <spd> [kp]     : Drive straight rev, heading latched"));
        serialStream.println(F("  GSTOP                : Exit gyro hold + stop motors"));
        serialStream.println(F("  GKP <kp>             : Set P gain (PWM per deg, 0..20)"));
        serialStream.println(F("  GMAX <max>           : Set max correction PWM (0..255)"));
        serialStream.println(F("  GSTATUS              : Query gyro hold state + heading"));
        serialStream.println(F("  PING                 : Test communication"));
        serialStream.println(F("--------------------------------------------"));
    } else {
        serialStream.printf("ERR: Unknown command '%s'. Send HELP for usage.\r\n", token);
    }
}

// Static helper methods for sending commands over UART
void MotorController::sendSpeedCommand(Stream &serialStream, int16_t left, int16_t right) {
    serialStream.printf("SPEED %d %d\r\n", left, right);
}

void MotorController::sendMotorCommand(Stream &serialStream, int motor1Based, int16_t speed) {
    serialStream.printf("MOTOR %d %d\r\n", motor1Based, speed);
}

void MotorController::sendFourMotorsCommand(Stream &serialStream, int16_t m1, int16_t m2, int16_t m3, int16_t m4) {
    serialStream.printf("MOTORS %d %d %d %d\r\n", m1, m2, m3, m4);
}

void MotorController::sendForwardCommand(Stream &serialStream, int16_t speed) {
    serialStream.printf("FWD %d\r\n", speed);
}

void MotorController::sendBackwardCommand(Stream &serialStream, int16_t speed) {
    serialStream.printf("BACK %d\r\n", speed);
}

void MotorController::sendSteerCommand(Stream &serialStream, int16_t speed, float turnRatio) {
    serialStream.printf("STEER %d %.2f\r\n", speed, turnRatio);
}

void MotorController::sendTurnLeftCommand(Stream &serialStream, int16_t speed) {
    serialStream.printf("LEFT %d\r\n", speed);
}

void MotorController::sendTurnRightCommand(Stream &serialStream, int16_t speed) {
    serialStream.printf("RIGHT %d\r\n", speed);
}

void MotorController::sendStopCommand(Stream &serialStream) {
    serialStream.println(F("STOP"));
}

void MotorController::sendBrakeCommand(Stream &serialStream) {
    serialStream.println(F("BRAKE"));
}
