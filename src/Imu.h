#pragma once

#include <stdint.h>

// BNO055 in IMUPLUS mode (gyro + accelerometer fusion, magnetometer off).
// The magnetometer is left out on purpose: motors and steel on the field bend it,
// which makes the heading jump. Heading is relative to the power-on orientation.
namespace imu {

struct Sample {
    float heading, roll, pitch;  // deg, heading 0..360 clockwise
    float gyroX, gyroY, gyroZ;   // deg/s
    float qw, qx, qy, qz;
    uint8_t calSys, calGyro, calAccel, calMag;  // 0..3
};

bool begin();
bool ok();  // found at boot

bool readHeading(float &heading);  // false on I2C error
bool readSample(Sample &sample);

}  // namespace imu
