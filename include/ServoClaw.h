#ifndef SERVO_CLAW_H
#define SERVO_CLAW_H

#include <Arduino.h>
#include <Servo.h>

/**
 * @brief Claw + pickup servo group with runtime-tunable action angles.
 *
 * Three 50 Hz hobby servos (pins in BoardPins.h, PIO-driven so they are
 * independent of the 20 kHz motor PWM):
 *   - claw left  @ GP2  \  mirrored jaws -> shared OPEN / CLOSE actions
 *   - claw right @ GP7  /
 *   - pickup     @ GP5     arm -> PICKUP (down/grab) / DROP (up/release)
 *
 * Each action stores its own angle in degrees (0..180). The `CLAW` serial
 * command tunes them at runtime (see MotorController::parseAndExecuteCommand)
 * so the mechanical endpoints can be dialled in without reflashing. Values
 * live in RAM only: once the mechanics are happy, bake the final numbers into
 * the DEF_* constants below.
 */
class ServoClaw {
public:
    // Fallback action angles (degrees). TUNE THESE to the real mechanics.
    static constexpr int DEF_OPEN_L  = 20;
    static constexpr int DEF_OPEN_R  = 160;
    static constexpr int DEF_CLOSE_L = 80;
    static constexpr int DEF_CLOSE_R = 100;
    static constexpr int DEF_PICKUP  = 40;
    static constexpr int DEF_DROP    = 130;

    static constexpr int SERVO_LEFT   = 1;   // claw jaw, GP2
    static constexpr int SERVO_RIGHT  = 2;   // claw jaw, GP7
    static constexpr int SERVO_PICKUP = 3;   // pickup arm, GP5

    /**
     * @brief Attach the three servos and move to the stowed power-on pose
     * (jaws closed, pickup arm up) so the robot boots in a known state.
     */
    void begin();

    /** True when all three servos attached (PIO state machines available). */
    bool attached() const { return _attached; }

    // --- Actions -----------------------------------------------------------
    void open();      // jaws apart   (both claws to their OPEN angles)
    void close();     // jaws shut    (both claws to their CLOSE angles)
    void pickup();    // arm down     (grab position)
    void drop();      // arm up       (release position)

    /** Restore the DEF_* action angles (does not move the servos). */
    void resetDefaults();

    // --- Tuning: store an action angle (an action command applies it) -------
    void setOpenLeft(int deg)    { _openL  = clampDeg(deg); }
    void setOpenRight(int deg)   { _openR  = clampDeg(deg); }
    void setCloseLeft(int deg)   { _closeL = clampDeg(deg); }
    void setCloseRight(int deg)  { _closeR = clampDeg(deg); }
    void setPickupAngle(int deg) { _pick   = clampDeg(deg); }
    void setDropAngle(int deg)   { _drop   = clampDeg(deg); }

    int openLeft() const    { return _openL; }
    int openRight() const   { return _openR; }
    int closeLeft() const   { return _closeL; }
    int closeRight() const  { return _closeR; }
    int pickupAngle() const { return _pick; }
    int dropAngle() const   { return _drop; }

    // --- Direct control / lifecycle ----------------------------------------
    /**
     * @brief Nudge a single servo to an absolute angle (tuning aid).
     * @param servo1Based SERVO_LEFT (1), SERVO_RIGHT (2) or SERVO_PICKUP (3)
     * @param deg 0..180 (clamped)
     * @return false when servo1Based is out of range
     */
    bool moveRaw(int servo1Based, int deg);

    /** Last commanded angle of one servo (SERVO_LEFT/RIGHT/PICKUP), or -1. */
    int angleOf(int servo1Based) const;

    /** Stop pulsing / free the PIO state machines (pins go idle). */
    void detachAll();

    /** Print the tunables, the live pose and a paste-ready defaults line. */
    void printStatus(Stream &s) const;

    // --- Scheduled scoop sequence (the real grab) ---------------------------
    /**
     * @brief Start the pickup sequence (non-blocking; service() runs it):
     *   1. FULL CLOSE - jaws to the calibrated close pose, hold to grip.
     *   2. ANGLE - jaws settle to the Pi-supplied grip angles (skip when
     *      both are < 0, i.e. no info from the Pi).
     *   3. PICKUP - arm down to the grab angle, hold so the lift settles.
     *   4. OPEN CAREFULLY - jaws creep step-by-step to the calibrated open
     *      pose. Finishes by publishing one-shot via pollSeqDone().
     */
    void startSeq(int piLeftDeg, int piRightDeg);
    /** @brief Advance the running sequence. Call every loop() pass. */
    void service();
    /** @brief True while the sequence is still running. */
    bool seqActive() const { return _seqStage != SEQ_IDLE; }
    /** @brief Human-readable stage: IDLE/CLOSE/ANGLE/PICKUP/OPEN. */
    const char *seqStageName() const;
    /** @brief Fetch and clear the one-shot "sequence finished" event. */
    bool pollSeqDone();

private:
    enum SeqStage : uint8_t {
        SEQ_IDLE = 0,    /**< No sequence running. */
        SEQ_STAGE_CLOSE, /**< Full close, gripping dwell. */
        SEQ_STAGE_ANGLE, /**< Settle to the Pi-supplied jaw angles. */
        SEQ_STAGE_PICKUP,/**< Arm down, lift settle dwell. */
        SEQ_STAGE_OPEN,  /**< Slow open to the calibrated open pose. */
    };

    static int clampDeg(int deg) { return deg < 0 ? 0 : (deg > 180 ? 180 : deg); }
    static int stepToward(int cur, int tgt, int step);
    void writeAngle(int servo1Based, int deg);

    Servo _left, _right, _pickupServo;
    bool _attached = false;

    // Action angles (degrees)
    int _openL  = DEF_OPEN_L;
    int _openR  = DEF_OPEN_R;
    int _closeL = DEF_CLOSE_L;
    int _closeR = DEF_CLOSE_R;
    int _pick   = DEF_PICKUP;
    int _drop   = DEF_DROP;

    // Last commanded angle per servo (for CLAW status / re-attach)
    int _curL = DEF_CLOSE_L;
    int _curR = DEF_CLOSE_R;
    int _curP = DEF_DROP;

    // Scheduled sequence state (ServoClaw stage machine), clocked in service()
    uint8_t _seqStage = SEQ_IDLE;
    unsigned long _seqStageMs = 0;
    int _piL = -1, _piR = -1;         // Pi-supplied grip angles (<0 = skip step)
    bool _seqDonePending = false;     // one-shot: sequence completed

    static constexpr unsigned SEQ_CLOSE_MS   = 300;  // grip dwell after close
    static constexpr unsigned SEQ_ANGLE_MS   = 100;  // jaws settle on Pi angles
    static constexpr unsigned SEQ_PICK_MS    = 450;  // lift settle dwell
    static constexpr unsigned SEQ_OPEN_MS    = 40;   // slow-open step cadence
    static constexpr int      SEQ_OPEN_STEP  = 6;    // degrees per slow-open step
};

#endif // SERVO_CLAW_H
