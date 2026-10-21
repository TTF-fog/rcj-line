#include "Imu.h"

#include <Arduino.h>
#include <Wire.h>

#include "Config.h"

namespace {

// Register map, BNO055 datasheet section 4.
const uint8_t REG_CHIP_ID = 0x00;
const uint8_t REG_GYRO_DATA = 0x14;   // gyro, euler, quaternion follow back to back
const uint8_t REG_EULER_DATA = 0x1A;
const uint8_t REG_CALIB_STAT = 0x35;
const uint8_t REG_OPR_MODE = 0x3D;
const uint8_t REG_PWR_MODE = 0x3E;
const uint8_t REG_SYS_TRIGGER = 0x3F;

const uint8_t CHIP_ID = 0xA0;
const uint8_t MODE_CONFIG = 0x00;
const uint8_t MODE_IMUPLUS = 0x08;
const uint8_t POWER_NORMAL = 0x00;
const uint8_t USE_EXTERNAL_CRYSTAL = 0x80;

// ADR pin low. 0x29 is taken by the VL53L0X power-on address.
const uint8_t ADDRESS = 0x28;

bool found = false;

bool writeReg(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(ADDRESS);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

bool readRegs(uint8_t reg, uint8_t *buffer, uint8_t length) {
    Wire.beginTransmission(ADDRESS);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom(ADDRESS, length) != length) return false;
    for (uint8_t i = 0; i < length; i++) buffer[i] = Wire.read();
    return true;
}

int16_t int16At(const uint8_t *buffer, int index) {
    return (int16_t)(buffer[index] | (buffer[index + 1] << 8));
}

float applyHeadingInvert(float heading) {
    if (config::on(cfg.invertHeading)) heading = 360.0f - heading;
    return heading >= 360.0f ? heading - 360.0f : heading;
}

bool findChip() {
    // The chip needs up to ~850 ms after power-on before it answers.
    unsigned long start = millis();
    while (millis() - start < 1000) {
        uint8_t id = 0;
        if (readRegs(REG_CHIP_ID, &id, 1) && id == CHIP_ID) return true;
        delay(50);
    }
    return false;
}

}  // namespace

namespace imu {

bool begin() {
    // Mode switches need settling time: ~19 ms into config mode, ~7 ms out of it.
    found = findChip() && writeReg(REG_OPR_MODE, MODE_CONFIG);
    if (!found) return false;
    delay(25);
    found = writeReg(REG_PWR_MODE, POWER_NORMAL) && writeReg(REG_SYS_TRIGGER, USE_EXTERNAL_CRYSTAL);
    delay(10);
    found = found && writeReg(REG_OPR_MODE, MODE_IMUPLUS);
    delay(20);
    return found;
}

bool ok() { return found; }

bool readHeading(float &heading) {
    uint8_t raw[2];
    if (!found || !readRegs(REG_EULER_DATA, raw, 2)) return false;
    heading = applyHeadingInvert(int16At(raw, 0) / 16.0f);
    return true;
}

bool readSample(Sample &s) {
    uint8_t raw[20];
    uint8_t calib;
    if (!found || !readRegs(REG_GYRO_DATA, raw, 20) || !readRegs(REG_CALIB_STAT, &calib, 1)) {
        return false;
    }
    s.gyroX = int16At(raw, 0) / 16.0f;
    s.gyroY = int16At(raw, 2) / 16.0f;
    s.gyroZ = int16At(raw, 4) / 16.0f;
    s.heading = applyHeadingInvert(int16At(raw, 6) / 16.0f);
    s.roll = int16At(raw, 8) / 16.0f;
    s.pitch = int16At(raw, 10) / 16.0f;
    const float quatScale = 1.0f / 16384.0f;
    s.qw = int16At(raw, 12) * quatScale;
    s.qx = int16At(raw, 14) * quatScale;
    s.qy = int16At(raw, 16) * quatScale;
    s.qz = int16At(raw, 18) * quatScale;
    s.calSys = (calib >> 6) & 0x03;
    s.calGyro = (calib >> 4) & 0x03;
    s.calAccel = (calib >> 2) & 0x03;
    s.calMag = calib & 0x03;
    return true;
}

}  // namespace imu
