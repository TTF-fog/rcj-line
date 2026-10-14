#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <Arduino.h>
#include <stdint.h>

/**
 * @brief Representation of a single H-bridge motor channel using 2 PWM/DIR pins.
 * Matches the hardware layout in src/example.txt (Cytron Maker Drive dual-PWM).
 */
struct MotorPinConfig {
    int pin1;
    int pin2;
};

constexpr int MOTOR_COUNT = 4;

/**
 * @brief Library-less Motor Controller for 4-motor differential drive.
 * Controls 4 motors directly via analogWrite / pinMode without external libraries.
 */
class MotorController {
public:
    /**
     * @brief Construct a new MotorController using an array of 4 motor pin configurations.
     * @param configs Array of 4 MotorPinConfig structs.
     */
    explicit MotorController(const MotorPinConfig configs[MOTOR_COUNT]);

    /**
     * @brief Configure GPIO pin modes and stop all motors.
     */
    void begin();

    /**
     * @brief Set speed for an individual motor (0-indexed: 0, 1, 2, 3 or 1-based via helper).
     * @param motorIndex Motor index (0 = Motor 1, 1 = Motor 2, 2 = Motor 3, 3 = Motor 4)
     * @param speed Speed value between -255 (reverse) and +255 (forward)
     */
    void driveMotor(int motorIndex, int16_t speed);

    /**
     * @brief Coast an individual motor to stop (0-indexed).
     * @param motorIndex Motor index (0..3)
     */
    void stopMotor(int motorIndex);

    /**
     * @brief Active short-brake an individual motor (0-indexed).
     * @param motorIndex Motor index (0..3)
     */
    void brakeMotor(int motorIndex);

    /**
     * @brief Set individual motor speed by index (alias for driveMotor).
     */
    void setMotorSpeed(int motorIndex, int16_t speed);

    /**
     * @brief Set speeds for all 4 motors individually.
     * @param m1 Speed for Motor 1 (-255..255)
     * @param m2 Speed for Motor 2 (-255..255)
     * @param m3 Speed for Motor 3 (-255..255)
     * @param m4 Speed for Motor 4 (-255..255)
     */
    void setFourMotorSpeeds(int16_t m1, int16_t m2, int16_t m3, int16_t m4);

    /**
     * @brief Set left and right side motor speeds for differential drive.
     * Left side = Motor 1 & Motor 2. Right side = Motor 3 & Motor 4.
     * @param leftSpeed Left side speed (-255..255)
     * @param rightSpeed Right side speed (-255..255)
     */
    void setMotorSpeeds(int16_t leftSpeed, int16_t rightSpeed);

    /**
     * @brief Set speed for left side motors (Motor 1 & 2).
     */
    void setLeftSpeed(int16_t speed);

    /**
     * @brief Set speed for right side motors (Motor 3 & 4).
     */
    void setRightSpeed(int16_t speed);

    /**
     * @brief Move forward at given speed (all 4 motors forward).
     * @param speed Speed value (0..255)
     */
    void moveForward(int16_t speed);

    /**
     * @brief Move backward at given speed (all 4 motors backward).
     * @param speed Speed value (0..255)
     */
    void moveBackward(int16_t speed);

    /**
     * @brief Differential drive steering with speed and turn ratio.
     * @param speed Forward/backward base speed (-255..255)
     * @param turnRatio Steering ratio from -1.0f (full left spin) to +1.0f (full right spin).
     */
    void steer(int16_t speed, float turnRatio);

    /**
     * @brief Spin in place to the left (left reverse, right forward).
     * @param speed Speed value (0..255)
     */
    void turnLeft(int16_t speed);

    /**
     * @brief Spin in place to the right (left forward, right reverse).
     * @param speed Speed value (0..255)
     */
    void turnRight(int16_t speed);

    /**
     * @brief Stop all 4 motors (coast to stop).
     */
    void stop();

    /**
     * @brief Active short-brake all 4 motors.
     */
    void brake();

    /**
     * @brief Configure direction inversion for individual motors.
     */
    void setInverted(bool inv1, bool inv2, bool inv3, bool inv4);

    /**
     * @brief Get speed of an individual motor (0..3).
     */
    int16_t getMotorSpeed(int motorIndex) const;

    /**
     * @brief Get aggregate left side speed.
     */
    int16_t getLeftSpeed() const;

    /**
     * @brief Get aggregate right side speed.
     */
    int16_t getRightSpeed() const;

    /**
     * @brief Process incoming command strings from a UART stream (non-blocking).
     * Supports concurrent processing from both USB Serial and Hardware Serial1.
     * @param serialStream UART stream to read from (e.g., Serial or Serial1)
     */
    void processUartCommands(Stream &serialStream);

    // =========================================================================
    // Static helper methods for sending commands over UART to another node
    // =========================================================================
    static void sendSpeedCommand(Stream &serialStream, int16_t left, int16_t right);
    static void sendMotorCommand(Stream &serialStream, int motor1Based, int16_t speed);
    static void sendFourMotorsCommand(Stream &serialStream, int16_t m1, int16_t m2, int16_t m3, int16_t m4);
    static void sendForwardCommand(Stream &serialStream, int16_t speed);
    static void sendBackwardCommand(Stream &serialStream, int16_t speed);
    static void sendSteerCommand(Stream &serialStream, int16_t speed, float turnRatio);
    static void sendTurnLeftCommand(Stream &serialStream, int16_t speed);
    static void sendTurnRightCommand(Stream &serialStream, int16_t speed);
    static void sendStopCommand(Stream &serialStream);
    static void sendBrakeCommand(Stream &serialStream);

private:
    MotorPinConfig _motors[MOTOR_COUNT];
    int16_t _speeds[MOTOR_COUNT] = {0, 0, 0, 0};
    bool _inverted[MOTOR_COUNT] = {false, false, false, false};

    static constexpr size_t CMD_BUF_LEN = 64;
    // Independent buffers for USB Serial and Hardware Serial streams
    char _usbCmdBuffer[CMD_BUF_LEN];
    size_t _usbCmdBufferPos = 0;
    char _hwCmdBuffer[CMD_BUF_LEN];
    size_t _hwCmdBufferPos = 0;

    void parseAndExecuteCommand(const char *cmd, Stream &serialStream);
};

#endif // MOTOR_CONTROLLER_H
