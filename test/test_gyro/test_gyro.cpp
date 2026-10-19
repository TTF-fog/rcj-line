#include <unity.h>
#include "GyroStraight.h"
#include "GyroTurn.h"

// =============================================================================
// GyroTurn: in-place heading turn controller
// =============================================================================

// Start latches a target relative to the current heading, + = left/CCW.
void test_turn_target_latches_relative_heading(void) {
    GyroTurn t;
    t.start(10.0f, 90.0f, 150);
    TEST_ASSERT_TRUE(t.isActive());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 100.0f, t.target());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 90.0f, t.lastError());
}

// Wrapping: target crosses the 0/360 seam (350 + 20 -> 10).
void test_turn_target_wraps_past_360(void) {
    GyroTurn t;
    t.start(350.0f, 20.0f, 150);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 10.0f, t.target());
}

// Positive error (target CCW) must spin left: left reverse, right forward.
void test_turn_positive_error_spins_left(void) {
    GyroTurn t(4.0f, 60.0f, 2.0f);
    t.start(0.0f, 90.0f, 150);
    int16_t l = 0, r = 0;
    TEST_ASSERT_TRUE(t.compute(0.0f, l, r));
    TEST_ASSERT_EQUAL_INT16(-150, l);
    TEST_ASSERT_EQUAL_INT16(150, r);
}

// Negative error (target CW) must spin right: left forward, right reverse.
void test_turn_negative_error_spins_right(void) {
    GyroTurn t(4.0f, 60.0f, 2.0f);
    t.start(0.0f, -90.0f, 150);
    int16_t l = 0, r = 0;
    TEST_ASSERT_TRUE(t.compute(0.0f, l, r));
    TEST_ASSERT_EQUAL_INT16(150, l);
    TEST_ASSERT_EQUAL_INT16(-150, r);
}

// The commanded turn speed caps the P output.
void test_turn_speed_caps_pwm(void) {
    GyroTurn t(4.0f, 5.0f, 2.0f);
    t.start(0.0f, 90.0f, 100);  // Kp*err = 360 -> capped at 100
    int16_t l = 0, r = 0;
    TEST_ASSERT_TRUE(t.compute(0.0f, l, r));
    TEST_ASSERT_EQUAL_INT16(-100, l);
    TEST_ASSERT_EQUAL_INT16(100, r);
}

// Far from the target a weak output still clears the stall floor (min PWM).
void test_turn_min_pwm_floor(void) {
    GyroTurn t(1.0f, 70.0f, 2.0f);
    t.start(0.0f, 90.0f, 150);
    int16_t l = 0, r = 0;
    TEST_ASSERT_TRUE(t.compute(70.0f, l, r));  // err=20 (far) -> Kp*err=20, floor 70
    TEST_ASSERT_EQUAL_INT16(-70, l);
    TEST_ASSERT_EQUAL_INT16(70, r);
}

// Inside the near band (4x tol, min 8 deg) the floor is DISABLED: a small
// output glides instead of shoving the robot across the landing window.
void test_turn_floor_skipped_near_target(void) {
    GyroTurn t(1.0f, 70.0f, 2.0f);
    t.start(0.0f, 5.0f, 255);
    int16_t l = 0, r = 0;
    TEST_ASSERT_TRUE(t.compute(0.0f, l, r));  // err=5 < band 8 -> no floor
    TEST_ASSERT_EQUAL_INT16(-5, l);           // small glide, NOT -70
    TEST_ASSERT_EQUAL_INT16(5, r);
}

// Kd is brake-only: however fast the heading closes on the target, the D
// term can damp the drive to its 10% floor but NEVER reverse the spin.
void test_turn_brake_d_never_reverses(void) {
    GyroTurn t(2.0f, 0.0f, 2.0f, 0.0f, 2.0f);
    t.start(0.0f, 40.0f, 255);
    int16_t l = 0, r = 0;
    TEST_ASSERT_TRUE(t.compute(20.0f, l, r));
    // err 20, dE=-20 -> rate -1000 -> dFilt -500. Old signed-D output would
    // be u = 40 - 2*500 = -960 (spin CW, hard reverse = wobble). Brake-only
    // D clamps the brake punch to 90% of drive: mag = 10% of 40 = 4, same dir.
    TEST_ASSERT_EQUAL_INT16(-4, l);
    TEST_ASSERT_EQUAL_INT16(4, r);
}

// Reaching tolerance latches finished: compute false + outputs zeroed.
void test_turn_finishes_inside_tolerance(void) {
    GyroTurn t(4.0f, 60.0f, 2.0f);
    t.start(0.0f, 90.0f, 150);
    int16_t l = -1, r = -1;
    TEST_ASSERT_FALSE(t.compute(88.5f, l, r));  // err=1.5 <= 2.0
    TEST_ASSERT_FALSE(t.isActive());
    TEST_ASSERT_EQUAL_INT16(0, l);
    TEST_ASSERT_EQUAL_INT16(0, r);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.5f, t.lastError());
}

// Wrapped error is chosen as the shortest path (err 15, not -345).
void test_turn_uses_shortest_error(void) {
    GyroTurn t(4.0f, 60.0f, 2.0f);
    t.start(350.0f, 20.0f, 150);  // target 10
    int16_t l = 0, r = 0;
    TEST_ASSERT_TRUE(t.compute(355.0f, l, r));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 15.0f, t.lastError());
    TEST_ASSERT_EQUAL_INT16(-60, l);  // still spinning left, short way
}

// Once active a 0-error start still publishes one "finished" step.
void test_turn_zero_delta_finishes_on_first_step(void) {
    GyroTurn t(4.0f, 60.0f, 2.0f);
    t.start(45.0f, 0.0f, 150);
    int16_t l = -1, r = -1;
    TEST_ASSERT_FALSE(t.compute(45.0f, l, r));
    TEST_ASSERT_FALSE(t.isActive());
    TEST_ASSERT_EQUAL_INT16(0, l);
    TEST_ASSERT_EQUAL_INT16(0, r);
}

// stop()/abort() end the turn without producing drive output.
void test_turn_stop_and_abort(void) {
    GyroTurn t(4.0f, 60.0f, 2.0f);
    t.start(0.0f, 90.0f, 150);
    t.abort();
    TEST_ASSERT_FALSE(t.isActive());
    int16_t l = -1, r = -1;
    TEST_ASSERT_FALSE(t.compute(0.0f, l, r));
    TEST_ASSERT_EQUAL_INT16(0, l);
    TEST_ASSERT_EQUAL_INT16(0, r);

    t.start(0.0f, 90.0f, 150);
    t.stop();
    TEST_ASSERT_FALSE(t.isActive());

    // Setters clamp to sane ranges.
    t.setKp(-5.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, t.kp());
    t.setMinPwm(500.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 255.0f, t.minPwm());
    t.setTolerance(-1.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, t.tolerance());
}

// =============================================================================
// GyroStraight: heading straight-line holder
// =============================================================================

void test_straight_wrap_and_normalize(void) {
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 350.0f, GyroStraight::normalize360(-10.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, GyroStraight::normalize360(720.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -20.0f, GyroStraight::wrapError(350.0f, 10.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 20.0f, GyroStraight::wrapError(10.0f, 350.0f));
}

void test_straight_p_mix_and_deadband(void) {
    GyroStraight g(2.0f, 80.0f, 1.0f);
    g.start(150, 0.0f);
    int16_t l = 0, r = 0;
    TEST_ASSERT_TRUE(g.compute(10.0f, l, r));  // err -10 -> corr -20
    TEST_ASSERT_EQUAL_INT16(130, l);
    TEST_ASSERT_EQUAL_INT16(170, r);

    // Inside the deadband the correction is zero: straight at base speed.
    TEST_ASSERT_TRUE(g.compute(0.5f, l, r));
    TEST_ASSERT_EQUAL_INT16(150, l);
    TEST_ASSERT_EQUAL_INT16(150, r);

    // Correction is clamped to maxCorrection (err +160 -> 320 -> 80):
    // target is CCW of current, so this turns left (left faster).
    TEST_ASSERT_TRUE(g.compute(200.0f, l, r));
    TEST_ASSERT_EQUAL_INT16(230, l);
    TEST_ASSERT_EQUAL_INT16(70, r);

    g.stop();
    TEST_ASSERT_FALSE(g.compute(0.0f, l, r));
}

// =============================================================================
// GyroTurn: PID terms
// =============================================================================

// Ki integrates a sustained error: still "stalled" so each step pushes harder.
void test_turn_ki_integrates_when_stalled(void) {
    GyroTurn t(0.0f, 0.0f, 2.0f, 2.0f, 0.0f);  // pure I
    t.start(0.0f, 10.0f, 255);
    int16_t l = 0, r = 0;
    // 100 stalled steps @ dt=0.02: integral = 2.0 deg*s, u = 0.4 * 100 = 40
    for (int i = 0; i < 100; i++) {
        TEST_ASSERT_TRUE(t.compute(0.0f, l, r));
    }
    TEST_ASSERT_EQUAL_INT16(-40, l);
    TEST_ASSERT_EQUAL_INT16(40, r);
}

// The integral TERM is clamped: windup can never exceed GTIMAX PWM.
void test_turn_integral_clamped_by_imax(void) {
    GyroTurn t(0.0f, 0.0f, 2.0f, 5.0f, 0.0f, 40.0f);  // pure I, iMax 40
    t.start(0.0f, 50.0f, 255);
    int16_t l = 0, r = 0;
    for (int i = 0; i < 100; i++) {
        TEST_ASSERT_TRUE(t.compute(0.0f, l, r));
    }
    TEST_ASSERT_EQUAL_INT16(-40, l);  // saturated at iMax
    for (int i = 0; i < 100; i++) {
        TEST_ASSERT_TRUE(t.compute(0.0f, l, r));
    }
    TEST_ASSERT_EQUAL_INT16(-40, l);  // still 40: windup bounded
}

// A fresh start clears the integrator (no residual bias from the last turn).
void test_turn_integral_resets_on_new_start(void) {
    GyroTurn t(0.0f, 0.0f, 2.0f, 2.0f, 0.0f);
    t.start(0.0f, 10.0f, 255);
    int16_t l = 0, r = 0;
    for (int i = 0; i < 100; i++) {
        TEST_ASSERT_TRUE(t.compute(0.0f, l, r));
    }
    TEST_ASSERT_EQUAL_INT16(-40, l);  // wound up
    t.stop();
    t.start(0.0f, 10.0f, 255);        // fresh turn resets state
    TEST_ASSERT_TRUE(t.compute(0.0f, l, r));  // integral = 0.2 -> u = 0.4
    TEST_ASSERT_EQUAL_INT16(0, l);    // rounds to zero drive
    TEST_ASSERT_TRUE(t.isActive());   // still stepping, not finished
}

// Kd damps the output as the error shrinks (no stall floor).
void test_turn_kd_damps_output_while_approaching(void) {
    GyroTurn t(4.0f, 0.0f, 2.0f, 0.0f, 0.5f);
    t.start(0.0f, 10.0f, 255);
    int16_t l1 = 0, r1 = 0, l2 = 0, r2 = 0;
    TEST_ASSERT_TRUE(t.compute(0.0f, l1, r1));  // err 10: u = 40 (dFilt 0)
    TEST_ASSERT_TRUE(t.compute(2.0f, l2, r2));  // err 8:  dE=-2 -> dFilt=-50
    // Step 1: u = 40. Step 2: u = 32 + 0.5*(-50) = 7 -> heavily damped.
    TEST_ASSERT_EQUAL_INT16(-40, l1);
    TEST_ASSERT_EQUAL_INT16(-7, l2);
    // Direction still CCW toward the target in both steps.
    TEST_ASSERT_EQUAL_INT16(40, r1);
    TEST_ASSERT_EQUAL_INT16(7, r2);
}

void setUp(void) {}
void tearDown(void) {}

int main(int argc, char **argv) {
    (void)argc;
    (void)argv;
    UNITY_BEGIN();
    RUN_TEST(test_turn_target_latches_relative_heading);
    RUN_TEST(test_turn_target_wraps_past_360);
    RUN_TEST(test_turn_positive_error_spins_left);
    RUN_TEST(test_turn_negative_error_spins_right);
    RUN_TEST(test_turn_speed_caps_pwm);
    RUN_TEST(test_turn_min_pwm_floor);
    RUN_TEST(test_turn_floor_skipped_near_target);
    RUN_TEST(test_turn_brake_d_never_reverses);
    RUN_TEST(test_turn_finishes_inside_tolerance);
    RUN_TEST(test_turn_uses_shortest_error);
    RUN_TEST(test_turn_zero_delta_finishes_on_first_step);
    RUN_TEST(test_turn_stop_and_abort);
    RUN_TEST(test_turn_ki_integrates_when_stalled);
    RUN_TEST(test_turn_integral_clamped_by_imax);
    RUN_TEST(test_turn_integral_resets_on_new_start);
    RUN_TEST(test_turn_kd_damps_output_while_approaching);
    RUN_TEST(test_straight_wrap_and_normalize);
    RUN_TEST(test_straight_p_mix_and_deadband);
    return UNITY_END();
}
