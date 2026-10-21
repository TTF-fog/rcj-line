#pragma once

#include <Arduino.h>

// Everything that can be fixed over serial with SET, and persisted with SAVE.
// Flags are stored as floats too (0 = off, 1 = on) so one table handles all of them.
struct Config {
    // Motors: flip a single motor, or swap which pair counts as the left side.
    float invertM1, invertM2, invertM3, invertM4;
    float swapSides;
    float rampPerTick;  // max output change per 10 ms tick, in %. 0 = no ramp.

    // Flip the heading if TURN spins forever / STRAIGHT veers off instead of correcting.
    float invertHeading;

    float straightKp, straightKd, straightMaxCorrection;

    float turnKp, turnKd;
    float turnMinPower, turnMaxPower;  // %
    float turnTolerance;               // deg
    float turnTimeoutMs;

    // Claw angles in servo degrees.
    float armUp, armDown;
    float jawOpenLeft, jawOpenRight;
    float jawNeutralLeft, jawNeutralRight;
    float jawBias;  // added to both jaws for GRAB R, subtracted for GRAB L
    float armMs, jawMs;  // wait after each arm / jaw move during GRAB
    float jawCreepDegPerS;  // speed of the slow open at the end of GRAB

    float streamHz;  // IMU + STAT lines per second, 0 = off
};

extern Config cfg;

namespace config {

struct Setting {
    const char *key;
    float *value;
    float min, max;
};

void begin();  // load from flash, or defaults if nothing valid is saved
void resetToDefaults();
bool save();

const Setting *find(const char *key);  // nullptr if unknown
void printAll();

inline bool on(float flag) { return flag != 0.0f; }

}  // namespace config
