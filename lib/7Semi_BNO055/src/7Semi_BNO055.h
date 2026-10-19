#ifndef _7Semi_BNO055_H_
#define _7Semi_BNO055_H_

#pragma once
#include <Arduino.h>
#include <Wire.h>

// ================= I2C addresses =================
// #define I2C_ADDR_A 0x28
// #define I2C_ADDR_B 0x29

// // ================= Register map ==================
// // PAGE ID
#define REG_PAGE_ID 0x07

// // ---- PAGE 0 ----
// // IDs / revisions
#define REG_CHIP_ID 0x00
#define REG_ACCEL_REV_ID 0x01
#define REG_MAG_REV_ID 0x02
#define REG_GYRO_REV_ID 0x03
#define REG_SW_REV_LSB 0x04
#define REG_SW_REV_MSB 0x05
#define REG_BL_REV_ID 0x06

// // Accel
#define REG_ACCEL_X_LSB 0x08
#define REG_ACCEL_X_MSB 0x09
#define REG_ACCEL_Y_LSB 0x0A
#define REG_ACCEL_Y_MSB 0x0B
#define REG_ACCEL_Z_LSB 0x0C
#define REG_ACCEL_Z_MSB 0x0D

// // Mag
#define REG_MAG_X_LSB 0x0E
#define REG_MAG_X_MSB 0x0F
#define REG_MAG_Y_LSB 0x10
#define REG_MAG_Y_MSB 0x11
#define REG_MAG_Z_LSB 0x12
#define REG_MAG_Z_MSB 0x13

// // Gyro
#define REG_GYRO_X_LSB 0x14
#define REG_GYRO_X_MSB 0x15
#define REG_GYRO_Y_LSB 0x16
#define REG_GYRO_Y_MSB 0x17
#define REG_GYRO_Z_LSB 0x18
#define REG_GYRO_Z_MSB 0x19

// // Euler (H, R, P)
#define REG_EULER_H_LSB 0x1A
#define REG_EULER_H_MSB 0x1B
#define REG_EULER_R_LSB 0x1C
#define REG_EULER_R_MSB 0x1D
#define REG_EULER_P_LSB 0x1E
#define REG_EULER_P_MSB 0x1F

// // Quaternion (W, X, Y, Z)
#define REG_QUAT_W_LSB 0x20
#define REG_QUAT_W_MSB 0x21
#define REG_QUAT_X_LSB 0x22
#define REG_QUAT_X_MSB 0x23
#define REG_QUAT_Y_LSB 0x24
#define REG_QUAT_Y_MSB 0x25
#define REG_QUAT_Z_LSB 0x26
#define REG_QUAT_Z_MSB 0x27

// // Linear acceleration
#define REG_LINACC_X_LSB 0x28
#define REG_LINACC_X_MSB 0x29
#define REG_LINACC_Y_LSB 0x2A
#define REG_LINACC_Y_MSB 0x2B
#define REG_LINACC_Z_LSB 0x2C
#define REG_LINACC_Z_MSB 0x2D

// // Gravity vector
#define REG_GRAV_X_LSB 0x2E
#define REG_GRAV_X_MSB 0x2F
#define REG_GRAV_Y_LSB 0x30
#define REG_GRAV_Y_MSB 0x31
#define REG_GRAV_Z_LSB 0x32
#define REG_GRAV_Z_MSB 0x33

// // Temperature
#define REG_TEMP 0x34

// // Status
#define REG_CALIB_STAT 0x35
#define REG_SELFTEST_RESULT 0x36
#define REG_INTR_STAT 0x37
#define REG_SYS_CLK_STAT 0x38
#define REG_SYS_STATUS 0x39
#define REG_SYS_ERR 0x3A

// // Units & data select
#define REG_UNIT_SEL 0x3B
#define REG_DATA_SELECT 0x3C

// // Modes & power
#define REG_OPR_MODE 0x3D
#define REG_PWR_MODE 0x3E
#define REG_SYS_TRIGGER 0x3F
#define REG_TEMP_SOURCE 0x40

// // Axis remap
#define REG_AXIS_MAP_CONFIG 0x41
#define REG_AXIS_MAP_SIGN 0x42

// // SIC matrix
#define REG_SIC_0_LSB 0x43
#define REG_SIC_0_MSB 0x44
#define REG_SIC_1_LSB 0x45
#define REG_SIC_1_MSB 0x46
#define REG_SIC_2_LSB 0x47
#define REG_SIC_2_MSB 0x48
#define REG_SIC_3_LSB 0x49
#define REG_SIC_3_MSB 0x4A
#define REG_SIC_4_LSB 0x4B
#define REG_SIC_4_MSB 0x4C
#define REG_SIC_5_LSB 0x4D
#define REG_SIC_5_MSB 0x4E
#define REG_SIC_6_LSB 0x4F
#define REG_SIC_6_MSB 0x50
#define REG_SIC_7_LSB 0x51
#define REG_SIC_7_MSB 0x52
#define REG_SIC_8_LSB 0x53
#define REG_SIC_8_MSB 0x54

// // Offsets block (22 bytes total, 0x55..0x6A)
#define REG_OFFSETS_START 0x55
#define REG_ACCEL_OFF_X_LSB 0x55
#define REG_ACCEL_OFF_X_MSB 0x56
#define REG_ACCEL_OFF_Y_LSB 0x57
#define REG_ACCEL_OFF_Y_MSB 0x58
#define REG_ACCEL_OFF_Z_LSB 0x59
#define REG_ACCEL_OFF_Z_MSB 0x5A
#define REG_MAG_OFF_X_LSB 0x5B
#define REG_MAG_OFF_X_MSB 0x5C
#define REG_MAG_OFF_Y_LSB 0x5D
#define REG_MAG_OFF_Y_MSB 0x5E
#define REG_MAG_OFF_Z_LSB 0x5F
#define REG_MAG_OFF_Z_MSB 0x60
#define REG_GYRO_OFF_X_LSB 0x61
#define REG_GYRO_OFF_X_MSB 0x62
#define REG_GYRO_OFF_Y_LSB 0x63
#define REG_GYRO_OFF_Y_MSB 0x64
#define REG_GYRO_OFF_Z_LSB 0x65
#define REG_GYRO_OFF_Z_MSB 0x66
#define REG_ACCEL_RADIUS_LSB 0x67
#define REG_ACCEL_RADIUS_MSB 0x68
#define REG_MAG_RADIUS_LSB 0x69
#define REG_MAG_RADIUS_MSB 0x6A

// // ---- PAGE 1 ----
#define P1_ACCEL_CONFIG 0x08
#define P1_MAG_CONFIG 0x09
#define P1_GYRO_CONFIG0 0x0A
#define P1_GYRO_CONFIG1 0x0B
#define P1_ACCEL_SLEEP_CONFIG 0x0C
#define P1_GYRO_SLEEP_CONFIG 0x0D
#define P1_MAG_SLEEP_CONFIG 0x0E

// Interrupts (Page 1)
#define P1_INT_MASK 0x0F
#define P1_INT_EN 0x10
#define P1_ACCEL_ANY_MOTION_THRES 0x11
#define P1_ACCEL_INTR_SETTINGS 0x12
#define P1_ACCEL_HIGH_G_DURN 0x13
#define P1_ACCEL_HIGH_G_THRES 0x14
#define P1_ACCEL_NO_MOTION_THRES 0x15
#define P1_ACCEL_NO_MOTION_SET 0x16
#define P1_GYRO_INTR_SETTING 0x17
#define P1_GYRO_HIGHRATE_X_SET 0x18
#define P1_GYRO_DURN_X 0x19
#define P1_GYRO_HIGHRATE_Y_SET 0x1A
#define P1_GYRO_DURN_Y 0x1B
#define P1_GYRO_HIGHRATE_Z_SET 0x1C
#define P1_GYRO_DURN_Z 0x1D
#define P1_GYRO_ANY_MOTION_THRES 0x1E
#define P1_GYRO_ANY_MOTION_SET 0x1F

// --- Power ---
#define Power_Normal 0x00
#define Power_LowPower 0x01
#define Power_Suspend 0x02

// --- Sensor ranges / bandwidths ---
#define g2 0
#define g4 1
#define g8 2
#define g16 3

#define BW_7_81 0
#define BW_15_63 1
#define BW_31_25 2
#define BW_62_5 3
#define BW_125 4
#define BW_250 5
#define BW_500 6
#define BW_1000 7

#define dps2000 0
#define dps1000 1
#define dps500 2
#define dps250 3
#define dps125 4

#define BW_523 0
#define BW_230 1
#define BW_116 2
#define BW_47 3
#define BW_23 4
#define BW_12 5
#define BW_64 6
#define BW_32 7

#define Hz2 0
#define Hz6 1
#define Hz8 2
#define Hz10 3
#define Hz15 4
#define Hz20 5
#define Hz25 6
#define Hz30 7

#define LowPower 0
#define Regular 1
#define EnhancedRegular 2
#define HighAccuracy 3

// --- Power modes per sensor ---
#define Gyro_Normal 0b000
#define Gyro_FastPowerUp 0b001
#define Gyro_DeepSuspend 0b010
#define Gyro_Suspend 0b011
#define Gyro_AdvancedPowersave 0b100

#define Accel_Normal 0b000
#define Accel_Suspend 0b001
#define Accel_LowPower1 0b010
#define Accel_Standby 0b011
#define Accel_LowPower2 0b100
#define Accel_DeepSuspend 0b101

#define Mag_Normal 0b00
#define Mag_Sleep 0b01
#define Mag_Suspend 0b10
#define Mag_ForceMode 0b11

// --- Interrupt bits on P1_INT_EN (0x10) ---
#define ACC_NM 7
#define ACC_AM 6
#define ACC_HIGH_G 5
#define GYR_DRDY 4
#define GYR_HIGH_RATE 3
#define GYR_AM 2
#define MAG_DRDY 1
#define ACC_BSX_DRDY 0

struct AxisRemap {
  uint8_t config = 0, sign = 0;
};

// // --- Operation modes ---
enum class Mode : uint8_t {
  CONFIG = 0x00,
  ACCONLY = 0x01,
  MAGONLY = 0x02,
  GYRONLY = 0x03,
  ACCMAG = 0x04,
  ACCGYRO = 0x05,
  MAGGYRO = 0x06,
  AMG = 0x07,
  IMUPLUS = 0x08,
  COMPASS = 0x09,
  M4G = 0x0A,
  NDOF_FMC_OFF = 0x0B,
  NDOF = 0x0C
};

// ================= BNO055_7Semi class =================
class BNO055_7Semi {
public:
    BNO055_7Semi();              

  // - wire : I2C interface
  // - sda  : I2C SDA pin
  // - scl  : I2C SCL pin
  // - useExtCrystal : enable external crystal
  // return : true if initialized successfully
  // NOTE(vendored patch): the registry copy defaults the pins to fixed A4/A5
  // analog pin macros, which don't exist on arduino-pico boards. Explicit
  bool begin(TwoWire& wire = Wire, uint8_t sda = 16, uint8_t scl = 17, bool useExtCrystal = false, uint32_t i2cClockHz = 100000);

  // - m : mode to set
  // return : true if set
  bool setMode(Mode m);
  Mode getMode() const {
    return _mode;
  }

  // - mode : power mode
  // return : true if set
  bool setPower(uint8_t mode);

  // - enable : enable external crystal
  // return : true if set
  bool enableExternalCrystal(bool enable = true);

  // return : true if reset OK
  bool softReset();

  // - mapConfig : axis config
  // - mapSign : axis sign
  // return : true if set
  bool setAxis(uint8_t mapConfig, uint8_t mapSign);

  // Sensor configuration
  bool configAccel(uint8_t range, uint8_t bw);
  bool configAccel(uint8_t range, uint8_t bw, uint8_t pwr);
  bool getAccelConfig(uint8_t &range, uint8_t &bw, uint8_t &pwr) const;

  bool configGyro(uint8_t range, uint8_t bw, uint8_t pwr);
  bool getGyroConfig(uint8_t &range, uint8_t &bw, uint8_t &pwr) const;

  bool configMag(uint8_t rate, uint8_t pwr, uint8_t op);
  bool getMagConfig(uint8_t& rate, uint8_t& pwr, uint8_t& op) const;

  // Per-sensor offsets
  bool setAccelOffset(int16_t x, int16_t y, int16_t z);
  bool getAccelOffset(int16_t& x, int16_t& y, int16_t& z) const;
  bool setGyroOffset(int16_t x, int16_t y, int16_t z);
  bool getGyroOffset(int16_t& x, int16_t& y, int16_t& z) const;
  bool setMagOffset(int16_t x, int16_t y, int16_t z);
  bool getMagOffset(int16_t& x, int16_t& y, int16_t& z) const;

  // Calibration status helpers
  uint8_t readCalibStatus() const;
  bool isCalibrated() const;
  void calibBreakdown(uint8_t& sys, uint8_t& gyr, uint8_t& acc, uint8_t& mag) const;
  bool saveOffsets(uint8_t out22[22]);
  bool loadOffsets(const uint8_t in22[22]);
  bool waitCalibrated(uint32_t timeout_ms = 8000, uint32_t poll_ms = 100);

  // Raw reads
  bool readAccel(int16_t& x, int16_t& y, int16_t& z);
  bool readGyro(int16_t& x, int16_t& y, int16_t& z);
  bool readMag(int16_t& x, int16_t& y, int16_t& z);
  bool readLinear(int16_t& x, int16_t& y, int16_t& z);
  bool readGravity(int16_t& x, int16_t& y, int16_t& z);

  // Euler & Quaternion
  bool readEuler(float& heading, float& roll, float& pitch);
  bool readQuat(float& w, float& x, float& y, float& z);

  // return : temperature in °C
  int8_t temperatureC() {
    return int8_t(readReg(REG_TEMP));
  }

  // Interrupt helpers
  bool setInterruptMask(uint8_t mask, bool enable);
  bool enableInterrupt(uint8_t bit, bool enable);
  bool readInterruptMask(uint8_t& mask) const;

  // Low-level helpers
  uint8_t readReg(uint8_t reg);
  bool writeReg(uint8_t reg, uint8_t val);
  bool ensurePage(uint8_t page);
  bool readNReg(uint8_t reg, uint8_t* data, uint8_t n);

  static int16_t read2Reg(BNO055_7Semi* d, uint8_t reg_lsb);
  static bool write2Reg(BNO055_7Semi* d, uint8_t reg_lsb, int16_t v);

private:
  TwoWire* i2c = nullptr;
  uint8_t _addr = 0x28;
  Mode _mode = Mode::CONFIG;
  bool probeAddr(uint8_t addr);
};
#endif