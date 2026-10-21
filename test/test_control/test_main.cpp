#include <unity.h>

#include "Control.h"

using namespace control;

void setUp() {}
void tearDown() {}

void test_wrap180_takes_the_short_way() {
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -20.0f, wrap180(10.0f - 30.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 20.0f, wrap180(10.0f - 350.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -20.0f, wrap180(350.0f - 10.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 90.0f, wrap180(450.0f));
}

void test_steer_mix() {
    float left, right;
    steerMix(50, 0, left, right);
    TEST_ASSERT_EQUAL_FLOAT(50, left);
    TEST_ASSERT_EQUAL_FLOAT(50, right);

    steerMix(50, 0.5f, left, right);  // right side stops: arc right
    TEST_ASSERT_EQUAL_FLOAT(50, left);
    TEST_ASSERT_EQUAL_FLOAT(0, right);

    steerMix(50, -1, left, right);  // spin left in place
    TEST_ASSERT_EQUAL_FLOAT(-50, left);
    TEST_ASSERT_EQUAL_FLOAT(50, right);
}

void test_straight_steers_toward_target_both_ways() {
    float left, right;
    straightMix(40, 10, 0, 1, 0, 30, left, right);  // target is to the right
    TEST_ASSERT_TRUE(left > right);
    straightMix(-40, 10, 0, 1, 0, 30, left, right);  // same, reversing
    TEST_ASSERT_TRUE(left > right);
    straightMix(40, 100, 0, 1, 0, 30, left, right);
    TEST_ASSERT_EQUAL_FLOAT(70, left);
    TEST_ASSERT_EQUAL_FLOAT(10, right);
}

void test_turn_power() {
    TurnGains gains = {1.0f, 0.1f, 25, 60};
    TEST_ASSERT_EQUAL_FLOAT(60, turnPower(90, 0, gains));     // far: capped
    TEST_ASSERT_EQUAL_FLOAT(-25, turnPower(-8, 0, gains));    // near: minimum power
    TEST_ASSERT_EQUAL_FLOAT(-10, turnPower(10, 200, gains));  // closing fast: D brakes
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_wrap180_takes_the_short_way);
    RUN_TEST(test_steer_mix);
    RUN_TEST(test_straight_steers_toward_target_both_ways);
    RUN_TEST(test_turn_power);
    return UNITY_END();
}
