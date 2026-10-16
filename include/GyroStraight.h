#ifndef GYRO_STRAIGHT_H
#define GYRO_STRAIGHT_H

#include <stdint.h>

/**
 * @brief Gyro (heading) based straight-line holder using a P controller.
 *
 * Intended sensor: BNO055 fused Euler heading (0..360 deg, NDOF mode),
 * read via BNO055_7Semi::readEuler(). Raw gyro Z could also be integrated
 * externally and fed here as a 0..360 heading — the controller only cares
 * about heading deviation.
 *
 * Control law:
 *   error      = wrap180(targetHeading - currentHeading)   // degrees, -180..+180
 *   correction = Kp * error, clamped to +/- maxCorrection, 0 inside deadband
 *   left       = baseSpeed + correction
 *   right      = baseSpeed - correction
 *   (both clamped to -255..255)
 *
 * Sign convention: positive error (target to the left / CCW of current)
 * produces positive correction -> left faster, right slower -> turns left,
 * which drives error back toward zero. This holds for forward and reverse
 * base speeds because yaw rate ~ (right - left), independent of base sign.
 */
class GyroStraight {
public:
    explicit GyroStraight(float kp = 2.0f, float maxCorrection = 80.0f, float deadbandDeg = 1.0f);

    void setKp(float kp);
    void setMaxCorrection(float maxCorrection);
    void setDeadband(float deadbandDeg);

    float kp() const { return _kp; }
    float maxCorrection() const { return _maxCorrection; }
    float deadband() const { return _deadband; }

    /** @brief Latch a new straight run. baseSpeed -255..255, target 0..360. */
    void start(int16_t baseSpeed, float targetHeadingDeg);
    void stop();

    bool isActive() const { return _active; }
    int16_t baseSpeed() const { return _base; }
    float target() const { return _target; }

    /** @brief Normalize any angle to [0, 360). */
    static float normalize360(float headingDeg);

    /** @brief Shortest signed error target - current, in [-180, +180]. */
    static float wrapError(float targetDeg, float currentDeg);

    /** @brief P-term for a heading (0 if inactive / inside deadband). */
    float correctionFor(float currentHeadingDeg) const;

    /** @brief Mix base speed + correction into clamped left/right outputs. */
    static void mix(int16_t baseSpeed, float correction, int16_t &leftOut, int16_t &rightOut);

    /**
     * @brief Full step: compute left/right for current heading.
     * @return false when inactive (outputs untouched).
     */
    bool compute(float currentHeadingDeg, int16_t &leftOut, int16_t &rightOut) const;

private:
    static float clampF(float v, float lo, float hi);
    static int16_t clampS(int32_t v);

    bool _active = false;
    int16_t _base = 0;
    float _target = 0.0f;
    float _kp = 2.0f;
    float _maxCorrection = 80.0f;
    float _deadband = 1.0f;
};

#endif // GYRO_STRAIGHT_H
