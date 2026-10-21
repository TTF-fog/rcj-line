#include "Claw.h"

#include <Arduino.h>
#include <Servo.h>

#include "BoardPins.h"
#include "Config.h"

namespace {

enum class Stage { Idle, ArmDown, Open, Neutral, Bias, ArmUp, Creep };

Servo jawLeft, jawRight, arm;
float leftAngle, rightAngle;  // last written, so the creep knows where it is

Stage stage = Stage::Idle;
claw::Side grabSide = claw::Side::Center;
unsigned long stageStartMs = 0;
unsigned long lastCreepMs = 0;

void writeJaws(float left, float right) {
    leftAngle = constrain(left, 0, 180);
    rightAngle = constrain(right, 0, 180);
    jawLeft.write((int)leftAngle);
    jawRight.write((int)rightAngle);
}

void enter(Stage next) {
    stage = next;
    stageStartMs = millis();
    lastCreepMs = stageStartMs;
    switch (next) {
        case Stage::ArmDown: claw::armDown(); break;
        case Stage::Open: claw::jawsOpen(); break;
        case Stage::Neutral: claw::jawsNeutral(); break;
        case Stage::Bias: {
            // The jaws are mirrored, so moving both angles the same way shifts the grip sideways.
            float bias = grabSide == claw::Side::Left ? -cfg.jawBias : cfg.jawBias;
            writeJaws(cfg.jawNeutralLeft + bias, cfg.jawNeutralRight + bias);
            break;
        }
        case Stage::ArmUp: claw::armUp(); break;
        default: break;
    }
}

bool waited(float ms) { return millis() - stageStartMs >= ms; }

float stepToward(float from, float to, float maxStep) {
    if (fabsf(to - from) <= maxStep) return to;
    return from + (to > from ? maxStep : -maxStep);
}

// Returns true once both jaws reached the open pose.
bool creepOpen() {
    unsigned long now = millis();
    float maxStep = cfg.jawCreepDegPerS * (now - lastCreepMs) / 1000.0f;
    if (maxStep < 1.0f) return false;  // servo writes are whole degrees
    lastCreepMs = now;
    writeJaws(stepToward(leftAngle, cfg.jawOpenLeft, maxStep),
              stepToward(rightAngle, cfg.jawOpenRight, maxStep));
    return leftAngle == cfg.jawOpenLeft && rightAngle == cfg.jawOpenRight;
}

}  // namespace

namespace claw {

void begin() {
    jawLeft.attach(SERVO_JAW_LEFT_PIN);
    jawRight.attach(SERVO_JAW_RIGHT_PIN);
    arm.attach(SERVO_ARM_PIN);
    armUp();
    jawsNeutral();
}

void armUp() { arm.write((int)cfg.armUp); }
void armDown() { arm.write((int)cfg.armDown); }
void jawsOpen() { writeJaws(cfg.jawOpenLeft, cfg.jawOpenRight); }
void jawsNeutral() { writeJaws(cfg.jawNeutralLeft, cfg.jawNeutralRight); }

void moveServo(int servo, float degrees) {
    if (servo == 1) writeJaws(degrees, rightAngle);
    if (servo == 2) writeJaws(leftAngle, degrees);
    if (servo == 3) arm.write((int)constrain(degrees, 0, 180));
}

void grab(Side side) {
    grabSide = side;
    enter(Stage::ArmDown);
}

bool busy() { return stage != Stage::Idle; }

void cancel() {
    if (!busy()) return;
    stage = Stage::Idle;
    Serial.println("CANCEL GRAB");
}

const char *stageName() {
    switch (stage) {
        case Stage::ArmDown: return "ARM_DOWN";
        case Stage::Open: return "OPEN";
        case Stage::Neutral: return "NEUTRAL";
        case Stage::Bias: return "BIAS";
        case Stage::ArmUp: return "ARM_UP";
        case Stage::Creep: return "CREEP";
        default: return "IDLE";
    }
}

void update() {
    switch (stage) {
        case Stage::ArmDown:
            if (waited(cfg.armMs)) enter(Stage::Open);
            break;
        case Stage::Open:
            if (waited(cfg.jawMs)) enter(Stage::Neutral);
            break;
        case Stage::Neutral:
            if (waited(cfg.jawMs)) enter(grabSide == Side::Center ? Stage::ArmUp : Stage::Bias);
            break;
        case Stage::Bias:
            if (waited(cfg.jawMs)) enter(Stage::ArmUp);
            break;
        case Stage::ArmUp:
            if (waited(cfg.armMs)) enter(Stage::Creep);
            break;
        case Stage::Creep:
            if (creepOpen()) {
                stage = Stage::Idle;
                Serial.println("DONE GRAB");
            }
            break;
        case Stage::Idle:
            break;
    }
}

}  // namespace claw
