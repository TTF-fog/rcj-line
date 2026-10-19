# 7Semi_BNO055

A lightweight and minimal Arduino library for the Bosch BNO055 9-DOF IMU sensor.

Supports initialization, raw data reading (Accel / Gyro / Mag / Linear / Gravity), Euler angles, quaternions, calibration helpers, and axis remapping.  
Designed for low flash usage — compatible with AVR, ESP32, ESP8266, RP2040, STM32, and other Arduino platforms.

![Platform](https://img.shields.io/badge/platform-arduino-blue.svg)
![License](https://img.shields.io/badge/license-MIT-green.svg)
![Status](https://img.shields.io/badge/status-active-brightgreen.svg)

---

## ✨ Features

- I2C communication with auto-detection at address `0x28` or `0x29`
- Works on standard and custom I2C pins (for ESP32, STM32, etc.)
- Orientation data:
  - Euler angles (heading, roll, pitch)
  - Quaternion (w, x, y, z)
- Raw data reading:
  - Accelerometer
  - Gyroscope
  - Magnetometer
  - Linear acceleration
  - Gravity vector
- Sensor configuration:
  - Operation modes
  - Power modes
  - Bandwidth & range (per sensor)
  - Axis remapping
- Calibration helpers and sensor offset saving/restoring

---

## ⚡ Getting Started

### 1. Installation via Arduino Library Manager

1. Open the **Arduino IDE**
2. Go to:
   - `Sketch > Include Library > Manage Libraries…` (IDE 1.x), or  
   - Use the 📚 **Library Manager** in the sidebar (IDE 2.x)
3. In the search bar, type:
   -7Semi BNO055

4. Click the **Install** button

Once installed, include the library in your sketch:

#include <7semi_bno055.h>
### 2.Wiring (I2C)
| BNO055 Pin | Arduino Pin                      |
| ---------- | -------------------------------- |
| VIN        | 3.3V or 5V                       |
| GND        | GND                              |
| SDA        | A4 (Uno) or custom SDA           |
| SCL        | A5 (Uno) or custom SCL           |

### 3.Applications

Robotics and drones

Motion tracking and stabilization

AR/VR and head-tracking

Inertial navigation

Wearables and IoT motion sensing

