#include "Motors.h"

#include "BoardPins.h"
#include "Config.h"
#include "Control.h"

namespace {

const int PWM_RANGE = 1000;

float targets[4];
float outputs[4];
bool braked = true;

bool inverted(int motor) {
    const float flags[4] = {cfg.invertM1, cfg.invertM2, cfg.invertM3, cfg.invertM4};
    return config::on(flags[motor]);
}

void writeMotor(int motor, float percent) {
    if (inverted(motor)) percent = -percent;
    int duty = (int)(fabsf(percent) * PWM_RANGE / 100.0f);
    uint8_t pinA = MOTOR_PINS[motor][0];
    uint8_t pinB = MOTOR_PINS[motor][1];
    analogWrite(pinA, percent > 0 ? duty : 0);
    analogWrite(pinB, percent < 0 ? duty : 0);
}

}  // namespace

namespace motors {

void begin() {
    analogWriteFreq(MOTOR_PWM_HZ);
    analogWriteRange(PWM_RANGE);
    brake();
}

void setSides(float left, float right) {
    if (config::on(cfg.swapSides)) {
        float swapped = left;
        left = right;
        right = swapped;
    }
    float speeds[4] = {left, left, right, right};
    setEach(speeds);
}

void setEach(const float speeds[4]) {
    for (int i = 0; i < 4; i++) targets[i] = control::clamp(speeds[i], -100, 100);
    braked = false;
}

void brake() {
    for (int i = 0; i < 4; i++) {
        targets[i] = 0;
        outputs[i] = 0;
        analogWrite(MOTOR_PINS[i][0], PWM_RANGE);
        analogWrite(MOTOR_PINS[i][1], PWM_RANGE);
    }
    braked = true;
}

void update() {
    if (braked) return;
    float step = cfg.rampPerTick > 0 ? cfg.rampPerTick : 200;
    for (int i = 0; i < 4; i++) {
        outputs[i] += control::clamp(targets[i] - outputs[i], -step, step);
        writeMotor(i, outputs[i]);
    }
}

float left() { return config::on(cfg.swapSides) ? outputs[2] : outputs[0]; }
float right() { return config::on(cfg.swapSides) ? outputs[0] : outputs[2]; }

}  // namespace motors
