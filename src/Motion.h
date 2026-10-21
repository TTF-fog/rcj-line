#pragma once

#include <stdint.h>

// Driving modes. Each one runs until replaced, STOP, or (when given) its duration.
// Moves that promise an end report it with one line:
//   DONE <MODE> ...   finished normally
//   FAIL <MODE> ...   gave up (IMU lost, turn timeout)
//   CANCEL <MODE>     replaced by another command or STOP before finishing
namespace motion {

const uint32_t TICK_MS = 10;

enum class Mode { Idle, Drive, Straight, Turn };

void drive(float speed, float steer, uint32_t durationMs);  // 0 ms = until replaced
bool straight(float speed, uint32_t durationMs);            // false if no heading
bool turn(float degrees, float maxPower);                   // false if no heading
void stop();

void update();  // call every TICK_MS

Mode mode();
const char *modeName();
float targetHeading();   // NAN unless in Straight/Turn
float headingError();    // NAN unless in Straight/Turn. deg, + = target is to the right
uint32_t remainingMs();  // 0 when the move has no duration

}  // namespace motion
