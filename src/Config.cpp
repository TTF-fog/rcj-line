#include "Config.h"

#include <EEPROM.h>

Config cfg;

namespace {

const Config DEFAULTS = {
    .invertM1 = 0, .invertM2 = 1, .invertM3 = 1, .invertM4 = 1,
    .swapSides = 0,
    .rampPerTick = 10,
    .invertHeading = 0,
    .straightKp = 1.5f, .straightKd = 0.05f, .straightMaxCorrection = 30,
    .turnKp = 1.0f, .turnKd = 0.05f,
    .turnMinPower = 25, .turnMaxPower = 60,
    .turnTolerance = 5,
    .turnTimeoutMs = 5000,
    .armUp = 130, .armDown = 40,
    .jawOpenLeft = 20, .jawOpenRight = 160,
    .jawNeutralLeft = 80, .jawNeutralRight = 100,
    .jawBias = 15,
    .armMs = 450, .jawMs = 300,
    .jawCreepDegPerS = 120,
    .streamHz = 20,
};

const config::Setting SETTINGS[] = {
    {"motor.invert1", &cfg.invertM1, 0, 1},
    {"motor.invert2", &cfg.invertM2, 0, 1},
    {"motor.invert3", &cfg.invertM3, 0, 1},
    {"motor.invert4", &cfg.invertM4, 0, 1},
    {"motor.swap_sides", &cfg.swapSides, 0, 1},
    {"motor.ramp", &cfg.rampPerTick, 0, 100},
    {"heading.invert", &cfg.invertHeading, 0, 1},
    {"straight.kp", &cfg.straightKp, 0, 20},
    {"straight.kd", &cfg.straightKd, 0, 5},
    {"straight.max", &cfg.straightMaxCorrection, 0, 100},
    {"turn.kp", &cfg.turnKp, 0, 20},
    {"turn.kd", &cfg.turnKd, 0, 5},
    {"turn.min", &cfg.turnMinPower, 0, 100},
    {"turn.max", &cfg.turnMaxPower, 1, 100},
    {"turn.tol", &cfg.turnTolerance, 0.5f, 45},
    {"turn.timeout", &cfg.turnTimeoutMs, 500, 30000},
    {"claw.arm_up", &cfg.armUp, 0, 180},
    {"claw.arm_down", &cfg.armDown, 0, 180},
    {"claw.open_l", &cfg.jawOpenLeft, 0, 180},
    {"claw.open_r", &cfg.jawOpenRight, 0, 180},
    {"claw.neutral_l", &cfg.jawNeutralLeft, 0, 180},
    {"claw.neutral_r", &cfg.jawNeutralRight, 0, 180},
    {"claw.bias", &cfg.jawBias, -90, 90},
    {"claw.arm_ms", &cfg.armMs, 0, 5000},
    {"claw.jaw_ms", &cfg.jawMs, 0, 5000},
    {"claw.creep", &cfg.jawCreepDegPerS, 1, 1000},
    {"stream.hz", &cfg.streamHz, 0, 50},
};

// Bump VERSION whenever the Config layout changes, so old flash contents are ignored.
const uint32_t MAGIC = 0x52434A4C;  // "RCJL"
const uint32_t VERSION = 1;

struct Stored {
    uint32_t magic;
    uint32_t version;
    Config config;
};

}  // namespace

namespace config {

void begin() {
    EEPROM.begin(sizeof(Stored));
    Stored stored;
    EEPROM.get(0, stored);
    if (stored.magic == MAGIC && stored.version == VERSION) {
        cfg = stored.config;
    } else {
        cfg = DEFAULTS;
    }
}

void resetToDefaults() { cfg = DEFAULTS; }

bool save() {
    Stored stored = {MAGIC, VERSION, cfg};
    EEPROM.put(0, stored);
    return EEPROM.commit();
}

const Setting *find(const char *key) {
    for (const Setting &s : SETTINGS) {
        if (strcasecmp(s.key, key) == 0) return &s;
    }
    return nullptr;
}

void printAll() {
    for (const Setting &s : SETTINGS) Serial.printf("CFG %s=%g\n", s.key, *s.value);
}

}  // namespace config
