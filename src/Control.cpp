#include "Control.h"

#include <math.h>

namespace control {

float clamp(float value, float lo, float hi) {
    if (value < lo) return lo;
    if (value > hi) return hi;
    return value;
}

float wrap180(float deg) {
    deg = fmodf(deg + 180.0f, 360.0f);
    if (deg < 0) deg += 360.0f;
    return deg - 180.0f;
}

void steerMix(float speed, float steer, float &left, float &right) {
    steer = clamp(steer, -1.0f, 1.0f);
    left = speed;
    right = speed;
    if (steer > 0) {
        right = speed * (1.0f - 2.0f * steer);
    } else {
        left = speed * (1.0f + 2.0f * steer);
    }
}

void straightMix(float speed, float errorDeg, float rateDegPerS,
                 float kp, float kd, float maxCorrection, float &left, float &right) {
    float correction = clamp(kp * errorDeg - kd * rateDegPerS, -maxCorrection, maxCorrection);
    left = clamp(speed + correction, -100.0f, 100.0f);
    right = clamp(speed - correction, -100.0f, 100.0f);
}

float turnPower(float errorDeg, float rateDegPerS, const TurnGains &gains) {
    float power = gains.kp * errorDeg - gains.kd * rateDegPerS;
    power = clamp(power, -gains.maxPower, gains.maxPower);

    bool pushingTowardTarget = (power > 0) == (errorDeg > 0);
    if (pushingTowardTarget && fabsf(power) < gains.minPower) {
        power = errorDeg > 0 ? gains.minPower : -gains.minPower;
    }
    return power;
}

}  // namespace control
