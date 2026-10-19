#ifndef GYRO_TURN_H
#define GYRO_TURN_H

#include <stdint.h>
#include "GyroStraight.h"

/**
 * @brief Gyro (heading) based in-place turn controller using PID.
 *
 * Intended sensor: BNO055 fused Euler heading (0..360 deg, NDOF mode),
 * read via BNO055_7Semi::readEuler(). The controller only cares about the
 * shortest signed heading error toward a latched target heading.
 *
 * Sign convention: a relative turn of +deg means left/CCW (target is CCW of
 * the current heading), -deg means right/CW. A positive heading error
 * (target to the left of current) produces a left spin:
 *   left = -pwm, right = +pwm   (matches MotorController::turnLeft()).
 *
 * Control law per step (magnitude form, anti-wobble):
 *   error      = wrap180(targetHeading - currentHeading)   // deg, -180..+180
 *   deltaE     = clamp(error - lastError, +/-180)           // wrap-safe
 *   integral  += error * dt                                 // clamped term
 *   drive      = Kp * |error| + |Ki * integral|             // toward target
 *   brake      = opposing part of dE/dt (IIR-filtered)      // closing on target
 *   mag        = drive - Kd * brake, floored at 10% of drive
 *   pwm        = clamp(mag, [minPwm if far], speed)
 *   left = -pwm, right = +pwm when error > 0 (spin CCW), opposite when < 0.
 *
 *  - Kp: proportional pull toward the target (PWM per degree).
 *  - Kd: BRAKE-ONLY damping (PWM-seconds per degree). It can only reduce the
 *        toward-target drive while the heading closes on the target — it can
 *        never reverse or boost the spin. (A signed D term that subtracts
 *        hundreds of PWM mid-approach flips the motor direction and causes
 *        the classic left/right/left wobble; brake-only D cannot.)
 *  - Ki: integral bias (PWM per degree-second) for systematic imbalance;
 *        the integral TERM is clamped to integralMax PWM, and reset on
 *        start/finish/abort so a lost IMU can't wind it up.
 *  - minPwm stall floor: applies ONLY when |error| is beyond the near band
 *        (4x tolerance, at least 8 deg). Inside the band a floor-sized push
 *        shoves the robot back and forth across the landing window, so there
 *        the (small) PID output and the finish short-brake do the work.
 *
 * The defaults are pure P (Ki = Kd = 0); enable the I and D terms via
 * setKi()/setKd() (the firmware enables them in setup(), see main.cpp and
 * docs/PID_TUNING.md).
 *
 * finish: when |error| <= tolerance the step latches finished (outputs 0),
 * so MotorController can short-brake rotational momentum and publish the
 * one-shot GyroTurnResult via pollTurnDone().
 */
class GyroTurn {
public:
    explicit GyroTurn(float kp = 4.0f, float minPwm = 60.0f, float toleranceDeg = 2.0f,
                      float ki = 0.0f, float kd = 0.0f, float integralMaxPwm = 40.0f);

    void setKp(float kp);
    void setKi(float ki);
    void setKd(float kd);
    void setMinPwm(float minPwm);
    void setTolerance(float toleranceDeg);
    void setIntegralMax(float maxPwm);

    float kp() const { return _kp; }
    float ki() const { return _ki; }
    float kd() const { return _kd; }
    float minPwm() const { return _minPwm; }
    float tolerance() const { return _tolerance; }
    float integralMax() const { return _integralMax; }

    /**
     * @brief Latch a new in-place turn relative to the current heading.
     * Resets the integrator and derivative filter.
     * @param currentHeadingDeg Heading now (0..360).
     * @param deltaDeg + = left/CCW, - = right/CW (sign carries the direction).
     * @param speed Turn speed 1..255 (clamped; negative made positive).
     */
    void start(float currentHeadingDeg, float deltaDeg, int16_t speed);
    /** @brief End the turn without a result (active flag cleared). */
    void stop();
    /** @brief End the turn after an external abort (e.g. IMU lost). */
    void abort();

    bool isActive() const { return _active; }
    /** @brief Target heading (deg, 0..360). */
    float target() const { return _target; }
    /** @brief Clamped turn speed cap (0..255). */
    int16_t turnSpeed() const { return _speed; }
    /** @brief Heading error of the most recent compute()/start() (deg, -180..+180). */
    float lastError() const { return _lastError; }

    /**
     * @brief One control step: compute left/right spin outputs for a heading.
     * Latches inactive and zeroes the outputs on the step that reaches the
     * tolerance window (the "finished" step).
     * @param currentHeadingDeg Heading now (0..360).
     * @param dtSec Seconds since the previous step (>0). Defaults to 0.02
     *        when the caller doesn't track time (test convenience).
     * @return true while the turn is still running (outputs valid),
     *         false when inactive or just finished (outputs set to 0).
     */
    bool compute(float currentHeadingDeg, int16_t &leftOut, int16_t &rightOut,
                 float dtSec = 0.02f);

private:
    static float clampF(float v, float lo, float hi);

    bool _active = false;
    float _target = 0.0f;
    int16_t _speed = 0;
    float _lastError = 0.0f;
    float _integral = 0.0f;   // accumulated error * dt (deg*s)
    float _dFilt = 0.0f;      // low-passed error rate (deg/s)

    // Gains / envelope
    float _kp = 4.0f;
    float _ki = 0.0f;
    float _kd = 0.0f;
    float _minPwm = 60.0f;
    float _tolerance = 2.0f;
    float _integralMax = 40.0f;  // clamp on the integral TERM (PWM)
};

#endif // GYRO_TURN_H
