#pragma once

// Claw = an arm servo that lowers/raises two mirrored jaw servos.
// All angles and timings live in cfg (claw.* settings).
namespace claw {

enum class Side { Left, Center, Right };

void begin();  // attach servos and go to the rest pose: arm up, jaws neutral

void armUp();
void armDown();
void jawsOpen();
void jawsNeutral();
void moveServo(int servo, float degrees);  // 1 = left jaw, 2 = right jaw, 3 = arm

// The full pickup. Prints DONE GRAB when finished:
//   arm down -> jaws open -> jaws neutral -> jaws biased to `side`
//   -> arm up -> jaws creep open
void grab(Side side);
bool busy();       // a GRAB is running; manual moves are refused
void cancel();     // abort a running GRAB where it is (prints CANCEL GRAB)
const char *stageName();

void update();  // call every loop

}  // namespace claw
