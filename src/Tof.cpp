#include "Tof.h"

#include <VL53L0X.h>
#include <Wire.h>

#include "BoardPins.h"

namespace {

struct Slot {
    TwoWire *bus;
    const char *busName;
    uint8_t address;
};

// Addresses avoid the BNO055 (0x28) and the VL53L0X power-on default (0x29).
const Slot SLOTS[tof::COUNT] = {
    {&Wire, "I2C0", 0x30},
    {&Wire, "I2C0", 0x31},
    {&Wire1, "I2C1", 0x32},
    {&Wire1, "I2C1", 0x33},
};

const uint8_t DEFAULT_ADDRESS = 0x29;
const uint8_t MODEL_ID = 0xEE;
const unsigned long LOST_AFTER_MS = 500;
const uint16_t FAILED_READING = 65535;  // the library's timeout value, also what a failed read returns

enum class State {
    Missing,        // nothing answered: not connected (normal for unused slots)
    InitFailed,     // answered but wouldn't initialise
    AddressFailed,  // didn't take its new address
    Ok,
    Lost,           // worked, then stopped giving readings
};

const char *stateName(State s) {
    switch (s) {
        case State::Missing: return "missing";
        case State::InitFailed: return "init_failed";
        case State::AddressFailed: return "address_failed";
        case State::Ok: return "ok";
        case State::Lost: return "lost";
    }
    return "?";
}

VL53L0X sensors[tof::COUNT];
State states[tof::COUNT];
int distances[tof::COUNT];
unsigned long lastReadingMs[tof::COUNT];

void holdOff(int i) {
    pinMode(TOF_XSHUT_PINS[i], OUTPUT);
    digitalWrite(TOF_XSHUT_PINS[i], LOW);
}

bool answers(TwoWire &bus, uint8_t address) {
    bus.beginTransmission(address);
    return bus.endTransmission() == 0;
}

// With every XSHUT low nothing should be at 0x29. If something is, a sensor's
// XSHUT isn't wired (or the BNO055 has ADR high), and it will confuse discovery.
void warnAboutStrays() {
    if (answers(Wire, DEFAULT_ADDRESS)) Serial.println("EVT TOF stray device at 0x29 on I2C0, check XSHUT wiring");
    if (answers(Wire1, DEFAULT_ADDRESS)) Serial.println("EVT TOF stray device at 0x29 on I2C1, check XSHUT wiring");
}

State bringUp(int i) {
    // Release XSHUT by letting it float: breakouts pull it up to their own 2.8 V.
    pinMode(TOF_XSHUT_PINS[i], INPUT);
    delay(10);

    // A fresh driver object talks to the power-on address again, which matters on
    // a rescan: the old one still points at the address we assigned last time.
    VL53L0X &sensor = sensors[i];
    sensor = VL53L0X();
    sensor.setBus(SLOTS[i].bus);
    sensor.setTimeout(50);
    if (!answers(*SLOTS[i].bus, DEFAULT_ADDRESS)) return State::Missing;
    if (!sensor.init()) return State::InitFailed;

    sensor.setAddress(SLOTS[i].address);
    if (sensor.readReg(VL53L0X::IDENTIFICATION_MODEL_ID) != MODEL_ID || sensor.last_status != 0) {
        return State::AddressFailed;
    }
    sensor.startContinuous();
    return State::Ok;
}

// Returns true if a fresh reading was stored.
bool readIfReady(int i) {
    VL53L0X &sensor = sensors[i];
    uint8_t interrupt = sensor.readReg(VL53L0X::RESULT_INTERRUPT_STATUS);
    if (sensor.last_status != 0 || (interrupt & 0x07) == 0) return false;

    uint16_t mm = sensor.readRangeContinuousMillimeters();
    if (sensor.last_status != 0 || mm == FAILED_READING) return false;
    distances[i] = mm;
    return true;
}

}  // namespace

namespace tof {

void scan() {
    for (int i = 0; i < COUNT; i++) {
        holdOff(i);
        distances[i] = -1;
    }
    delay(10);
    warnAboutStrays();

    for (int i = 0; i < COUNT; i++) {
        states[i] = bringUp(i);
        lastReadingMs[i] = millis();
        if (states[i] == State::Ok) continue;
        holdOff(i);  // so a half-working sensor can't sit on 0x29 and block the next one
        if (states[i] != State::Missing) {
            Serial.printf("EVT TOF %d %s\n", i + 1, stateName(states[i]));
        }
    }
}

void update() {
    unsigned long now = millis();
    for (int i = 0; i < COUNT; i++) {
        if (states[i] != State::Ok) continue;
        if (readIfReady(i)) {
            lastReadingMs[i] = now;
        } else if (now - lastReadingMs[i] > LOST_AFTER_MS) {
            states[i] = State::Lost;
            distances[i] = -1;
            Serial.printf("EVT TOF %d lost\n", i + 1);
        }
    }
}

int distanceMm(int index) { return distances[index]; }

int okCount() {
    int count = 0;
    for (State s : states) count += s == State::Ok;
    return count;
}

void printStatus() {
    for (int i = 0; i < COUNT; i++) {
        Serial.printf("TOF %d state=%s bus=%s addr=0x%02X xshut=GP%u mm=%d\n", i + 1,
                      stateName(states[i]), SLOTS[i].busName, SLOTS[i].address,
                      TOF_XSHUT_PINS[i], distances[i]);
    }
}

}  // namespace tof
