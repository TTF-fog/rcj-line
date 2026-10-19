#include "GyroTurn.h"
#include <math.h>

GyroTurn::GyroTurn(float kp, float minPwm, float toleranceDeg,
                   float ki, float kd, float integralMaxPwm)
    : _kp(kp), _minPwm(minPwm), _tolerance(toleranceDeg),
      _ki(ki), _kd(kd), _integralMax(integralMaxPwm) {
    setKp(_kp);
    setKi(_ki);
    setKd(_kd);
    setMinPwm(_minPwm);
    setTolerance(_tolerance);
    setIntegralMax(_integralMax);
}

void GyroTurn::setKp(float kp) {
    _kp = (kp < 0.0f) ? 0.0f : kp;
}

void GyroTurn::setKi(float ki) {
    _ki = (ki < 0.0f) ? 0.0f : ki;
}

void GyroTurn::setKd(float kd) {
    _kd = (kd < 0.0f) ? 0.0f : kd;
}

void GyroTurn::setMinPwm(float minPwm) {
    _minPwm = clampF(minPwm, 0.0f, 255.0f);
}

void GyroTurn::setTolerance(float toleranceDeg) {
    _tolerance = (toleranceDeg < 0.0f) ? 0.0f : toleranceDeg;
}

void GyroTurn::setIntegralMax(float maxPwm) {
    _integralMax = (maxPwm < 0.0f) ? 0.0f : maxPwm;
}

void GyroTurn::start(float currentHeadingDeg, float deltaDeg, int16_t speed) {
    if (speed < 0) speed = -speed;
    if (speed > 255) speed = 255;
    _speed = speed;
    _target = GyroStraight::normalize360(currentHeadingDeg + deltaDeg);
    _lastError = GyroStraight::wrapError(_target, currentHeadingDeg);
    // Fresh turn: no residual integral bias, no stale derivative estimate.
    _integral = 0.0f;
    _dFilt = 0.0f;
    // Note: do NOT latch finished here even if already on target; the first
    // compute() must run so the caller still publishes a completion result.
    _active = true;
}

void GyroTurn::stop() {
    _active = false;
    _speed = 0;
    _integral = 0.0f;
    _dFilt = 0.0f;
}

void GyroTurn::abort() {
    _active = false;
    _speed = 0;
    _integral = 0.0f;
    _dFilt = 0.0f;
}

bool GyroTurn::compute(float currentHeadingDeg, int16_t &leftOut, int16_t &rightOut,
                       float dtSec) {
    if (!_active) {
        leftOut = 0;
        rightOut = 0;
        return false;
    }

    float err = GyroStraight::wrapError(_target, currentHeadingDeg);

    // Within tolerance: latch finished so the caller can short-brake and
    // publish the one-shot GyroTurnResult.
    if (fabsf(err) <= _tolerance) {
        _lastError = err;
        _active = false;
        _integral = 0.0f;
        _dFilt = 0.0f;
        leftOut = 0;
        rightOut = 0;
        return false;
    }

    // dt is caller-supplied; guard against 0 (double integration) and huge
    // gaps (stale samples) that would blow up the I and D terms.
    float dt = clampF(dtSec, 0.001f, 0.1f);

    // Integral: accumulate error*dt; the TERM (Ki * integral) is later
    // clamped, so a stuck robot can't wind more than integralMax PWM of bias.
    _integral += err * dt;
    float iTerm = _ki * _integral;
    if (iTerm > _integralMax) iTerm = _integralMax;
    else if (iTerm < -_integralMax) iTerm = -_integralMax;

    // Derivative on measurement with wrap-safe error delta, light IIR
    // filtration to keep gyro noise out of the output.
    float dE = err - _lastError;
    if (dE > 180.0f) dE -= 360.0f;
    else if (dE < -180.0f) dE += 360.0f;
    float rawRate = dE / dt;
    _dFilt += 0.5f * (rawRate - _dFilt);  // alpha = 0.5 at 50 Hz stepping
    _lastError = err;

    // Direction always toward the target (error sign): +dir = spin CCW/left.
    // The brake-only D below reduces the drive; it can NEVER flip this.
    float dir = (err >= 0.0f) ? 1.0f : -1.0f;

    // Drive magnitude: P + |I| toward the target.
    float drive = _kp * fabsf(err) + fabsf(iTerm);

    // BRAKE-ONLY D (anti-wobble): when the heading is closing on the target,
    // the error RATE has the opposite sign of err; subtract that rate from
    // the drive, but never reverse and never boost. Cap the brake punch at
    // 90% of the drive and always keep a 10% nudge so we never fully coast
    // inside the swing; the MCU's TURN_BRAKE_MS short-brake lands the stop.
    float brake = (dir * _dFilt > 0.0f) ? 0.0f : (dir < 0.0f ? _dFilt : -_dFilt);
    if (brake < 0.0f) brake = 0.0f;
    float mag = drive - _kd * brake;
    if (mag < 0.10f * drive) mag = 0.10f * drive;

    // Stall floor applies ONLY well away from the target: inside the near
    // band (4x tolerance, at least 8 deg) a strong floor is what shoves the
    // robot back and forth across the landing window ("the wiggle"). Close
    // in, let the small PID outputs (and the finish brake) do the work.
    float band = 4.0f * _tolerance;
    if (band < 8.0f) band = 8.0f;
    bool far = fabsf(err) > band;

    // Cap: commanded turn speed, never below the stall floor (a tiny speed
    // with a big floor still honours the floor).
    float cap = static_cast<float>(_speed);
    if (cap > 255.0f) cap = 255.0f;
    if (cap < _minPwm) cap = _minPwm;

    if (far && mag < _minPwm) mag = _minPwm;
    if (mag > cap) mag = cap;

    int32_t pwm = static_cast<int32_t>(mag + 0.5f);
    leftOut = static_cast<int16_t>(pwm);
    rightOut = static_cast<int16_t>(pwm);
    if (dir > 0.0f) {
        // Target CCW (left) of current heading: spin left (left rev, right fwd).
        leftOut = static_cast<int16_t>(-pwm);
    } else {
        // Target CW (right) of current heading: spin right (left fwd, right rev).
        rightOut = static_cast<int16_t>(-pwm);
    }
    return true;
}

float GyroTurn::clampF(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}
