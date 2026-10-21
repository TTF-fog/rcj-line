#pragma once

// Up to four VL53L0X distance sensors, two per I2C bus. Every sensor powers up
// at address 0x29, so they are held off with XSHUT and woken one at a time to be
// given their own address.
//
// Any slot can fail without affecting the others. A slot that fails discovery is
// held off the bus with XSHUT; a sensor that stops answering later is marked lost.
// Both read as -1 until the next scan.
namespace tof {

const int COUNT = 4;

// Blocks for up to ~0.5 s. Run at boot before the IMU, or via TOF SCAN while stopped.
void scan();
void update();  // call every loop, never blocks for long

int distanceMm(int index);  // -1 unless Ok; 8190+ means nothing in range
int okCount();

void printStatus();  // one TOF line per slot

}  // namespace tof
