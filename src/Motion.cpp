#include "Motion.h"

#include <Arduino.h>

#include "Config.h"
#include "Control.h"
#include "Imu.h"
#include "Motors.h"

namespace {

const uint8_t MAX_IMU_MISSES = 5;  // consecutive failed reads before giving up

motion::Mode current = motion::Mode::Idle;
bool reportsEnd = false;  // a timed move or a turn, so the Pi waits for DONE/FAIL
uint32_t startMs = 0;
uint32_t durationMs = 0;

float speed = 0;
float maxTurnPower = 0;

float target = 0;       // heading to hold (Straight)
float turnGoal = 0;     // degrees to rotate (Turn)
float turned = 0;       // degrees rotated so far, unwrapped (Turn)
float lastHeading = 0;
float rate = 0;         // filtered heading rate, deg/s
float error = 0;
uint8_t imuMisses = 0;

// End the current move. Prints the report line when the Pi is waiting for one.
void finish(const char *verb, const char *detail = "") {
    motors::brake();
    if (reportsEnd) Serial.printf("%s %s%s\n", verb, motion::modeName(), detail);
    current = motion::Mode::Idle;
    reportsEnd = false;
}

void begin(motion::Mode mode, uint32_t ms, bool promisesEnd) {
    if (reportsEnd) finish("CANCEL");
    current = mode;
    startMs = millis();
    durationMs = ms;
    reportsEnd = promisesEnd;
}

void startHeadingTracking(float heading) {
    lastHeading = heading;
    rate = 0;
    error = 0;
    imuMisses = 0;
}

// Reads the heading and updates `rate`. Returns false (and FAILs the move) when the IMU is gone.
bool trackHeading(float &heading, float &delta) {
    if (!imu::readHeading(heading)) {
        if (++imuMisses >= MAX_IMU_MISSES) {
            char detail[32];
            snprintf(detail, sizeof(detail), " imu err=%.1f", error);
            finish("FAIL", detail);
        }
        return false;
    }
    imuMisses = 0;
    delta = control::wrap180(heading - lastHeading);
    lastHeading = heading;
    rate = 0.5f * rate + 0.5f * (delta * 1000.0f / motion::TICK_MS);
    return true;
}

void updateStraight() {
    float heading, delta;
    if (!trackHeading(heading, delta)) return;
    error = control::wrap180(target - heading);
    float left, right;
    control::straightMix(speed, error, rate, cfg.straightKp, cfg.straightKd,
                         cfg.straightMaxCorrection, left, right);
    motors::setSides(left, right);
}

void updateTurn() {
    float heading, delta;
    if (!trackHeading(heading, delta)) return;
    turned += delta;
    error = turnGoal - turned;

    char detail[32];
    if (fabsf(error) <= cfg.turnTolerance) {
        snprintf(detail, sizeof(detail), " err=%.1f", error);
        finish("DONE", detail);
        return;
    }
    if (millis() - startMs >= cfg.turnTimeoutMs) {
        snprintf(detail, sizeof(detail), " timeout err=%.1f", error);
        finish("FAIL", detail);
        return;
    }

    control::TurnGains gains = {cfg.turnKp, cfg.turnKd, cfg.turnMinPower, maxTurnPower};
    float power = control::turnPower(error, rate, gains);
    motors::setSides(power, -power);
}

}  // namespace

namespace motion {

void drive(float speed, float steer, uint32_t ms) {
    begin(Mode::Drive, ms, ms > 0);
    float left, right;
    control::steerMix(speed, steer, left, right);
    motors::setSides(left, right);
}

bool straight(float newSpeed, uint32_t ms) {
    // Re-sending an open-ended STRAIGHT only changes the speed, so the Pi can
    // stream speed updates without resetting the heading being held.
    if (current == Mode::Straight && !reportsEnd && ms == 0) {
        speed = newSpeed;
        return true;
    }
    float heading;
    if (!imu::readHeading(heading)) return false;
    begin(Mode::Straight, ms, ms > 0);
    startHeadingTracking(heading);
    target = heading;
    speed = newSpeed;
    return true;
}

bool turn(float degrees, float power) {
    float heading;
    if (!imu::readHeading(heading)) return false;
    begin(Mode::Turn, 0, true);
    startHeadingTracking(heading);
    turnGoal = degrees;
    turned = 0;
    error = degrees;
    target = control::wrap180(heading + degrees - 180.0f) + 180.0f;  // 0..360
    maxTurnPower = power;
    return true;
}

void stop() { finish("CANCEL"); }

void update() {
    if (current == Mode::Idle) return;
    if (durationMs > 0 && millis() - startMs >= durationMs) {
        finish("DONE");
        return;
    }
    if (current == Mode::Straight) updateStraight();
    if (current == Mode::Turn) updateTurn();
}

Mode mode() { return current; }

const char *modeName() {
    switch (current) {
        case Mode::Drive: return "DRIVE";
        case Mode::Straight: return "STRAIGHT";
        case Mode::Turn: return "TURN";
        default: return "IDLE";
    }
}

static bool usesHeading() { return current == Mode::Straight || current == Mode::Turn; }
float targetHeading() { return usesHeading() ? target : NAN; }
float headingError() { return usesHeading() ? error : NAN; }

uint32_t remainingMs() {
    if (current == Mode::Idle || durationMs == 0) return 0;
    uint32_t elapsed = millis() - startMs;
    return elapsed >= durationMs ? 0 : durationMs - elapsed;
}

}  // namespace motion
