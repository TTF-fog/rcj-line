#include "Tof.h"

#include <VL53L0X.h>
#include <Wire.h>

#include "BoardPins.h"

namespace {

struct Slot {
    TwoWire *bus;
    uint8_t address;
};

// Addresses avoid the BNO055 (0x28) and the VL53L0X power-on default (0x29).
const Slot SLOTS[tof::COUNT] = {
    {&Wire, 0x30},
    {&Wire, 0x31},
    {&Wire1, 0x32},
    {&Wire1, 0x33},
};

VL53L0X sensors[tof::COUNT];
bool found[tof::COUNT];
int distances[tof::COUNT];

bool resultReady(VL53L0X &sensor) {
    return (sensor.readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07) != 0;
}

}  // namespace

namespace tof {

void begin() {
    for (int i = 0; i < COUNT; i++) {
        pinMode(TOF_XSHUT_PINS[i], OUTPUT);
        digitalWrite(TOF_XSHUT_PINS[i], LOW);
        distances[i] = -1;
    }
    delay(10);

    for (int i = 0; i < COUNT; i++) {
        // Release XSHUT by letting it float: breakouts pull it up to their own 2.8 V.
        pinMode(TOF_XSHUT_PINS[i], INPUT);
        delay(10);
        VL53L0X &sensor = sensors[i];
        sensor.setBus(SLOTS[i].bus);
        sensor.setTimeout(50);
        found[i] = sensor.init();
        if (!found[i]) {
            // Keep a sensor that failed init off the bus, so it can't sit on 0x29.
            pinMode(TOF_XSHUT_PINS[i], OUTPUT);
            digitalWrite(TOF_XSHUT_PINS[i], LOW);
            continue;
        }
        sensor.setAddress(SLOTS[i].address);
        sensor.startContinuous();
    }
}

void update() {
    for (int i = 0; i < COUNT; i++) {
        if (found[i] && resultReady(sensors[i])) {
            distances[i] = sensors[i].readRangeContinuousMillimeters();
        }
    }
}

int distanceMm(int index) { return distances[index]; }

int presentCount() {
    int count = 0;
    for (bool f : found) count += f;
    return count;
}

}  // namespace tof
