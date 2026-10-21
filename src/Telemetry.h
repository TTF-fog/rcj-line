#pragma once

namespace telemetry {

void printImu();     // IMU heading=.. roll=.. ...
void printStatus();  // STAT mode=.. left=.. ...

void update();  // call every loop: sends both lines at stream.hz

}  // namespace telemetry
