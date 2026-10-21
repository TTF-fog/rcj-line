#pragma once

// Up to four VL53L0X distance sensors, two per I2C bus. Every sensor powers up
// at address 0x29, so they are held off with XSHUT and woken one at a time to be
// given their own address. Sensors that don't answer are simply reported missing.
namespace tof {

const int COUNT = 4;

void begin();   // call before anything else talks on the I2C buses
void update();  // call every loop, never blocks

int distanceMm(int index);  // -1 when missing; 8190+ means nothing in range
int presentCount();

}  // namespace tof
