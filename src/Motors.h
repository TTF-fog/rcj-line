#pragma once

// Four DC motors as a skid-steer drive. All speeds are percent, -100..100.
namespace motors {

void begin();

void setSides(float left, float right);
void setEach(const float speeds[4]);  // M1..M4, still honours motor.invertN
void brake();                         // short-brake all motors, skips the ramp

void update();  // call every motion::TICK_MS: steps outputs toward their targets (motor.ramp)

float left();  // what the side is actually driven at right now, after the ramp
float right();

}  // namespace motors
