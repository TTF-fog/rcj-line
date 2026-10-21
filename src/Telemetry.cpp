#include "Telemetry.h"

#include <Arduino.h>

#include "Claw.h"
#include "Config.h"
#include "Imu.h"
#include "Motion.h"
#include "Motors.h"
#include "Tof.h"

namespace {

// Longest stream line, worst case. The USB TX buffer is only 256 bytes, and a
// write to a full buffer blocks for up to a second, so when the Pi stops reading
// we skip lines instead of stalling the control loop.
const int MAX_LINE_BYTES = 160;

unsigned long lastStreamMs = 0;

bool roomForLine() { return Serial && Serial.availableForWrite() >= MAX_LINE_BYTES; }

}  // namespace

namespace telemetry {

void printImu() {
    imu::Sample s;
    if (!imu::readSample(s)) {
        Serial.println("IMU none");
        return;
    }
    Serial.printf("IMU heading=%.2f roll=%.2f pitch=%.2f gx=%.2f gy=%.2f gz=%.2f "
                  "qw=%.4f qx=%.4f qy=%.4f qz=%.4f cal=%u,%u,%u,%u\n",
                  s.heading, s.roll, s.pitch, s.gyroX, s.gyroY, s.gyroZ,
                  s.qw, s.qx, s.qy, s.qz, s.calSys, s.calGyro, s.calAccel, s.calMag);
}

void printStatus() {
    Serial.printf("STAT mode=%s left=%.0f right=%.0f target=%.1f err=%.1f remain=%lu grab=%s tof=%d,%d,%d,%d\n",
                  motion::modeName(), motors::left(), motors::right(),
                  motion::targetHeading(), motion::headingError(),
                  (unsigned long)motion::remainingMs(), claw::stageName(),
                  tof::distanceMm(0), tof::distanceMm(1), tof::distanceMm(2), tof::distanceMm(3));
}

void update() {
    if (cfg.streamHz <= 0) return;
    unsigned long now = millis();
    if (now - lastStreamMs < 1000.0f / cfg.streamHz) return;
    lastStreamMs = now;
    if (roomForLine()) printImu();
    if (roomForLine()) printStatus();
}

}  // namespace telemetry
