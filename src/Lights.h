#pragma once

#include <stdint.h>

// Light bar (GP18) is whatever the Pi asks for with LIGHT; boots white.
// The two onboard pixels show what the robot is doing:
//   red = ESTOP (1.5 s) or IMU missing (blinking), amber = TURN, cyan = STRAIGHT,
//   blue = DRIVE, purple = GRAB, dim green = idle.
namespace lights {

void begin();
void setBar(uint8_t r, uint8_t g, uint8_t b);
void flashEstop();
void update();  // call every loop, repaints at most every 50 ms

}  // namespace lights
