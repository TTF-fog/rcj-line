#include "GyroStraight.h"

GyroStraight::GyroStraight(float kp, float maxCorrection, float deadbandDeg)
    : _kp(kp), _maxCorrection(maxCorrection), _deadband(deadbandDeg) {
    if (_kp < 0.0f) _kp = 0.0f;
    if (_maxCorrection < 0.0f) _maxCorrection = 0.0f;
    if (_deadband < 0.0f) _deadband = 0.0f;
}

void GyroStraight::setKp(float kp) {
    _kp = (kp < 0.0f) ? 0.0f : kp;
}

void GyroStraight::setMaxCorrection(float maxCorrection) {
    _maxCorrection = (maxCorrection < 0.0f) ? 0.0f : maxCorrection;
}

void GyroStraight::setDeadband(float deadbandDeg) {
    _deadband = (deadbandDeg < 0.0f) ? 0.0f : deadbandDeg;
}

void GyroStraight::start(int16_t baseSpeed, float targetHeadingDeg) {
    if (baseSpeed > 255) baseSpeed = 255;
    if (baseSpeed < -255) baseSpeed = -255;
    _base = baseSpeed;
    _target = normalize360(targetHeadingDeg);
    _active = true;
}

void GyroStraight::stop() {
    _active = false;
    _base = 0;
}

float GyroStraight::normalize360(float headingDeg) {
    while (headingDeg < 0.0f) headingDeg += 360.0f;
    while (headingDeg >= 360.0f) headingDeg -= 360.0f;
    return headingDeg;
}

float GyroStraight::wrapError(float targetDeg, float currentDeg) {
    float err = normalize360(targetDeg) - normalize360(currentDeg);
    while (err > 180.0f) err -= 360.0f;
    while (err < -180.0f) err += 360.0f;
    return err;
}

float GyroStraight::correctionFor(float currentHeadingDeg) const {
    if (!_active) return 0.0f;
    float err = wrapError(_target, currentHeadingDeg);
    // Deadband: ignore sub-degree jitter / sensor noise.
    if (err > -_deadband && err < _deadband) return 0.0f;
    float corr = _kp * err;
    return clampF(corr, -_maxCorrection, _maxCorrection);
}

void GyroStraight::mix(int16_t baseSpeed, float correction, int16_t &leftOut, int16_t &rightOut) {
    // correction is float PWM; round toward nearest int via truncation offset.
    int32_t c = static_cast<int32_t>(correction >= 0.0f ? correction + 0.5f : correction - 0.5f);
    leftOut = clampS(static_cast<int32_t>(baseSpeed) + c);
    rightOut = clampS(static_cast<int32_t>(baseSpeed) - c);
}

bool GyroStraight::compute(float currentHeadingDeg, int16_t &leftOut, int16_t &rightOut) const {
    if (!_active) return false;
    float corr = correctionFor(currentHeadingDeg);
    mix(_base, corr, leftOut, rightOut);
    return true;
}

float GyroStraight::clampF(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

int16_t GyroStraight::clampS(int32_t v) {
    if (v > 255) return 255;
    if (v < -255) return -255;
    return static_cast<int16_t>(v);
}
