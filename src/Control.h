#pragma once

// Pure control math, no Arduino dependencies, so it runs in host tests.
//
// Conventions used everywhere:
//   - Speeds and powers are percent, -100..100.
//   - Heading grows clockwise (BNO055 convention), so a positive angle means "to the right".
//   - Yaw rate is proportional to (left - right): left faster = turning right.

namespace control {

float clamp(float value, float lo, float hi);

// Wrap any angle into -180..180. wrap180(target - current) is the shortest turn.
float wrap180(float deg);

// steer 0 = straight, +1 = spin right in place, -1 = spin left in place.
void steerMix(float speed, float steer, float &left, float &right);

// Heading hold: errorDeg = target - heading, rate = heading change in deg/s.
void straightMix(float speed, float errorDeg, float rateDegPerS,
                 float kp, float kd, float maxCorrection, float &left, float &right);

struct TurnGains {
    float kp, kd;
    float minPower, maxPower;
};

// Spin power for an in-place turn. Positive = spin right (left wheels forward).
// The minimum power only pushes toward the target, so the D term can still brake.
float turnPower(float errorDeg, float rateDegPerS, const TurnGains &gains);

}  // namespace control
