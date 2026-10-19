#include "7Semi_BNO055.h"

// ---------- RAW helpers (optional, faster for fusion) ----------
/**
- Read raw accelerometer xyz (page 0)
- x,y,z : out raw values
- return : true if read ok
*/
BNO055_7Semi::BNO055_7Semi() {} 

bool BNO055_7Semi::readAccel(int16_t& x, int16_t& y, int16_t& z) {
  uint8_t b[6];
  if (!readNReg(REG_ACCEL_X_LSB, b, 6)) return false;
  x = int16_t(b[1] << 8 | b[0]);
  y = int16_t(b[3] << 8 | b[2]);
  z = int16_t(b[5] << 8 | b[4]);
  return true;
}

/**
- Read raw gyroscope xyz (page 0)
- x,y,z : out raw values
- return : true if read ok
*/
bool BNO055_7Semi::readGyro(int16_t& x, int16_t& y, int16_t& z) {
  uint8_t b[6];
  if (!readNReg(REG_GYRO_X_LSB, b, 6)) return false;
  x = int16_t(b[1] << 8 | b[0]);
  y = int16_t(b[3] << 8 | b[2]);
  z = int16_t(b[5] << 8 | b[4]);
  return true;
}

/**
- Read raw magnetometer xyz (page 0)
- x,y,z : out raw values
- return : true if read ok
*/
bool BNO055_7Semi::readMag(int16_t& x, int16_t& y, int16_t& z) {
  uint8_t b[6];
  if (!readNReg(REG_MAG_X_LSB, b, 6)) return false;
  x = int16_t(b[1] << 8 | b[0]);
  y = int16_t(b[3] << 8 | b[2]);
  z = int16_t(b[5] << 8 | b[4]);
  return true;
}
/* 
 - heading : Output heading (Yaw)
 - roll    : Output roll
 - pitch   : Output pitch
 - return    : true if data read successfully
*/
bool BNO055_7Semi::readEuler(float& heading, float& roll, float& pitch) {
  uint8_t buf[6];

  // Read 6 bytes from the Euler angle registers (H, R, P)
  if (!readNReg(REG_EULER_H_LSB, buf, 6)) return false;

  // Combine LSB and MSB for each axis
  int16_t h = int16_t(buf[1] << 8 | buf[0]);
  int16_t r = int16_t(buf[3] << 8 | buf[2]);
  int16_t p = int16_t(buf[5] << 8 | buf[4]);

  // radians: 1 LSB = 1/900 rad
  const float s = 1.0f / 16.0f;
  heading = h * s;
  roll = r * s;
  pitch = p * s;
  return true;
}

/**
- Read raw linear acceleration xyz (page 0)
- x,y,z : out raw values
- return : true if read ok
*/
bool BNO055_7Semi::readLinear(int16_t& x, int16_t& y, int16_t& z) {
  uint8_t b[6];
  if (!readNReg(REG_LINACC_X_LSB, b, 6)) return false;
  x = int16_t(b[1] << 8 | b[0]);
  y = int16_t(b[3] << 8 | b[2]);
  z = int16_t(b[5] << 8 | b[4]);
  return true;
}

/**
- Read raw gravity vector xyz (page 0)
- x,y,z : out raw values
- return : true if read ok
*/
bool BNO055_7Semi::readGravity(int16_t& x, int16_t& y, int16_t& z) {
  uint8_t b[6];
  if (!readNReg(REG_GRAV_X_LSB, b, 6)) return false;
  x = int16_t(b[1] << 8 | b[0]);
  y = int16_t(b[3] << 8 | b[2]);
  z = int16_t(b[5] << 8 | b[4]);
  return true;
}

/**
- Read quaternion as floats
- w,x,y,z : out components (scaled by 1/16384)
- return : true if read ok
*/
bool BNO055_7Semi::readQuat(float& w, float& x, float& y, float& z) {
  uint8_t b[8];
  if (!readNReg(REG_QUAT_W_LSB, b, 8)) return false;

  int16_t rw = int16_t(b[1] << 8 | b[0]);
  int16_t rx = int16_t(b[3] << 8 | b[2]);
  int16_t ry = int16_t(b[5] << 8 | b[4]);
  int16_t rz = int16_t(b[7] << 8 | b[6]);

  // 1 LSB = 1/16384
  const float s = 1.0f / 16384.0f;
  w = rw * s;
  x = rx * s;
  y = ry * s;
  z = rz * s;
  return true;
}

// ================== Mode ==================
/**
- Set operation mode
- m : target mode (CONFIG/NDOF/…)
- return : true if mode write ok
*/
bool BNO055_7Semi::setMode(Mode m) {
  if (_mode == m) return true;
  if (!writeReg(REG_OPR_MODE, static_cast<uint8_t>(m))) return false;
  delay(30);  // Bosch recommends 19–30 ms
  _mode = m;
  return true;
}

// ================== Begin ==================
/**
- Initialize sensor (Wire begin, chip check, CONFIG, power, optional XTal, enter NDOF)
- wire : I2C interface
- addr : I2C address
- useExtCrystal : true to enable external crystal
- return : true if sensor responds and init succeeds
*/
bool BNO055_7Semi::begin(TwoWire& wire, uint8_t sda, uint8_t scl, bool useExtCrystal, uint32_t i2cClockHz /*= 400000*/) {
  i2c = &wire;
  _addr = 0x28;
  // Configure I2C pins and clock
  // ----- I2C init (pins only where supported) -----
  // NOTE(vendored patch): earlephilhower's arduino-pico core (used by PlatformIO
  // for the Cytron Motion 2350 Pro / RP2350) has no TwoWire::begin(sda, scl)
  // overload; pins are assigned with setSDA/setSCL before begin(). Calling them
  // is safe even when Wire is already running with the same pins (the variant
  // defaults match). mbed-RP2040 and other cores keep the original call.
#if defined(ARDUINO_ARCH_RP2040) && !defined(ARDUINO_ARCH_MBED)
  if (i2c) {
    i2c->setSDA(sda);
    i2c->setSCL(scl);
    i2c->begin();
  }
#elif defined(ESP32) || defined(ESP8266) || defined(ARDUINO_ARCH_STM32) || defined(ARDUINO_ARCH_NRF5) || defined(ARDUINO_ARCH_RP2040)
  i2c->begin(sda, scl);
#else
  // AVR (UNO/Nano/Mega, etc.): pins are fixed by hardware
  (void)sda;
  (void)scl;
  i2c->begin();
#endif

// Set I2C clock where supported
#if defined(TWBR) || defined(ESP32) || defined(ESP8266) || defined(ARDUINO_ARCH_STM32) || defined(ARDUINO_ARCH_RP2040)
  i2c->setClock(i2cClockHz);
#endif
  if (probeAddr(0x28)) _addr = 0x28;
  else if (probeAddr(0x29)) _addr = 0x29;
  else return false;
  delay(700);
  // Chip ID check only (do not fail on revision IDs)
  if (readReg(REG_CHIP_ID) != 0xA0) return false;

  // Enter CONFIG mode
  if (!setMode(Mode ::CONFIG)) return false;
  delay(20);

  // Normal power
  if (!writeReg(REG_PWR_MODE, static_cast<uint8_t>(Power_Normal))) return false;
  delay(10);

  // External crystal
  if (useExtCrystal) {
    uint8_t t = readReg(REG_SYS_TRIGGER);
    if (!writeReg(REG_SYS_TRIGGER, t | 0x80)) return false;  // CLK_SEL
    delay(10);
  }
  if (!setMode(Mode::NDOF)) return false;
  delay(20);
  return true;
}
bool BNO055_7Semi::probeAddr(uint8_t addr) {
  i2c->beginTransmission(addr);
  return i2c->endTransmission() == 0;
}

/**
- Enable/disable external crystal (requires CONFIG mode)
- enable : true to enable
- return : true if write ok
*/
bool BNO055_7Semi::enableExternalCrystal(bool enable) {
  Mode prev = _mode;
  if (!setMode(Mode ::CONFIG)) return false;
  delay(20);
  uint8_t t = readReg(REG_SYS_TRIGGER);
  if (enable) t |= 0x80;
  else t &= ~uint8_t(0x80);
  bool ok = writeReg(REG_SYS_TRIGGER, t);
  setMode(prev);
  delay(20);
  return ok;
}

/**
- Issue software reset (RST_SYS bit)
- return : true if write ok
*/
bool BNO055_7Semi::softReset() {
  Mode prev = _mode;
  if (!setMode(Mode ::CONFIG)) return false;
  delay(20);
  uint8_t t = readReg(REG_SYS_TRIGGER);
  bool ok = writeReg(REG_SYS_TRIGGER, uint8_t(t | 0x20));  // RST_SYS bit
  delay(650);
  setMode(prev);
  delay(20);
  return ok;
}

/**
- Set axis remap via raw config/sign values (requires CONFIG mode)
- mapConfig : value for REG_AXIS_MAP_CONFIG
- mapSign : value for REG_AXIS_MAP_SIGN
- return : true if both writes ok
*/
bool BNO055_7Semi::setAxis(uint8_t mapConfig, uint8_t mapSign) {
  AxisRemap ar;
  ar.config = mapConfig;
  ar.sign = mapSign;
  Mode prev = _mode;

  // Must be in CONFIG mode to change axis mapping
  if (!setMode(Mode::CONFIG)) return false;
  delay(20);

  bool ok = writeReg(REG_AXIS_MAP_CONFIG, ar.config)
            && writeReg(REG_AXIS_MAP_SIGN, ar.sign);

  // Restore previous mode
  setMode(prev);
  delay(20);

  return ok;
}

/**
- Configure accelerometer (legacy 2-param version)
- range : ±2g..±16g
- bw : bandwidth
- return : true if write ok
*/
bool BNO055_7Semi::configAccel(uint8_t range, uint8_t bw) {
  Mode prev = _mode;
  if (!setMode(Mode::CONFIG)) return false;
  delay(20);

  if (!ensurePage(1)) return false;

  uint8_t value = (uint8_t(range) & 0x03) | ((uint8_t(bw) & 0x07) << 2);
  bool ok = writeReg(P1_ACCEL_CONFIG, value);

  ensurePage(0);
  setMode(prev);
  delay(20);
  return ok;
}

/**
- Check if fully calibrated (all four fields == 3)
- return : true if fully calibrated
*/
bool BNO055_7Semi::isCalibrated() const {
  uint8_t c = readCalibStatus();
  return ((c >> 6) & 3) == 3 && ((c >> 4) & 3) == 3 && ((c >> 2) & 3) == 3 && (c & 3) == 3;
}

/**
- Decode calibration status byte into components
- sys,gyr,acc,mag : out values (0..3)
*/
void BNO055_7Semi::calibBreakdown(uint8_t& sys, uint8_t& gyr, uint8_t& acc, uint8_t& mag) const {
  uint8_t c = readCalibStatus();
  sys = (c >> 6) & 3;
  gyr = (c >> 4) & 3;
  acc = (c >> 2) & 3;
  mag = (c >> 0) & 3;
}

/**
- Wait until fully calibrated or timeout
- timeout_ms : max wait time
- poll_ms : polling delay
- return : true if calibrated in time
*/
bool BNO055_7Semi::waitCalibrated(uint32_t timeout_ms, uint32_t poll_ms) {
  uint32_t start = millis();
  while (millis() - start < timeout_ms) {
    if (isCalibrated()) return true;
    delay(poll_ms);
  }
  return false;
}

/**
- Read 8-bit register
- reg : address
- return : value read
*/
uint8_t BNO055_7Semi::readReg(uint8_t reg) {
  i2c->beginTransmission(_addr);
  i2c->write(reg);
  i2c->endTransmission();
  i2c->requestFrom(_addr, (uint8_t)1);
  return i2c->available() ? i2c->read() : 0;
}

/**
- Write 8-bit register
- reg : address
- val : value
- return : true if I2C success
*/
bool BNO055_7Semi::writeReg(uint8_t reg, uint8_t val) {
  i2c->beginTransmission(_addr);
  i2c->write(reg);
  i2c->write(val);
  return i2c->endTransmission() == 0;
}

/**
- Read N bytes starting at register
- reg : start address
- data : destination buffer
- n : number of bytes
- return : true if exact N bytes read
*/
bool BNO055_7Semi::readNReg(uint8_t reg, uint8_t* data, uint8_t n) {
  i2c->beginTransmission(_addr);
  i2c->write(reg);
  i2c->endTransmission();
  uint8_t got = i2c->requestFrom(_addr, n);
  for (uint8_t i = 0; i < n && i2c->available(); ++i) data[i] = i2c->read();
  return got == n;
}

/**
- Ensure target register page selected
- page : 0 or 1
- return : true if page already set or write ok
*/
bool BNO055_7Semi::ensurePage(uint8_t page) {
  uint8_t cur = readReg(REG_PAGE_ID);
  if (cur == page) return true;
  return writeReg(REG_PAGE_ID, page);
}

/**
- Configure magnetometer (rate, power, op mode)
- rate : output data rate
- pwr : power mode
- op : operation mode (accuracy)
- return : true if write ok
*/
bool BNO055_7Semi::configMag(uint8_t rate, uint8_t pwr, uint8_t op) {
  Mode prev = _mode;
  if (!setMode(Mode::CONFIG)) return false;
  delay(20);

  if (!ensurePage(1)) {
    setMode(prev);
    delay(20);
    return false;
  }

  uint8_t v = ((static_cast<uint8_t>(rate) & 0x07)) | ((static_cast<uint8_t>(pwr) & 0x03) << 5) | ((static_cast<uint8_t>(op) & 0x03) << 3);

  bool ok = writeReg(P1_MAG_CONFIG, v);

  ensurePage(0);
  setMode(prev);
  delay(20);
  return ok;
}

/**
- Read back magnetometer configuration
- rate : out ODR
- pwr : out power mode
- op : out operation mode
- return : true
*/
bool BNO055_7Semi::getMagConfig(uint8_t& rate, uint8_t& pwr, uint8_t& op) const {
  // Switch to Page 1 to read sensor config
  const_cast<BNO055_7Semi*>(this)->ensurePage(1);
  uint8_t v = const_cast<BNO055_7Semi*>(this)->readReg(P1_MAG_CONFIG);
  const_cast<BNO055_7Semi*>(this)->ensurePage(0);

  // Extract raw values (no enum cast)
  rate = (v & 0x07);        // bits 0..2 → data rate
  pwr = ((v >> 3) & 0x03);  // bits 3..4 → power mode
  op = ((v >> 5) & 0x07);   // bits 5..7 → operation mode

  return true;
}


/**
- Configure accelerometer (range, bandwidth, power)
- range : ±2g..±16g
- bw : bandwidth
- pwr : Accel power mode
- return : true if write ok
*/
bool BNO055_7Semi::configAccel(uint8_t range, uint8_t bw, uint8_t pwr) {
  Mode prev = _mode;
  if (!setMode(Mode::CONFIG)) return false;
  delay(20);
  if (!ensurePage(1)) {
    setMode(prev);
    delay(20);
    return false;
  }

  uint8_t v = (range & 0x03)
             | ((bw & 0x07) << 2)
             | ((pwr & 0x07) << 5);

  bool ok = writeReg(P1_ACCEL_CONFIG, v);

  ensurePage(0);
  setMode(prev);
  delay(20);
  return ok;
}


/**
- Read back accelerometer configuration
- range : out range
- bw : out bandwidth
- pwr : out power mode
- return : true
*/
bool BNO055_7Semi::getAccelConfig(uint8_t& range, uint8_t& bw, uint8_t& pwr) const {
  const_cast<BNO055_7Semi*>(this)->ensurePage(1);
  uint8_t v = const_cast<BNO055_7Semi*>(this)->readReg(P1_ACCEL_CONFIG);
  const_cast<BNO055_7Semi*>(this)->ensurePage(0);

  range = (v & 0x03);
  bw    = ((v >> 2) & 0x07);
  pwr   = ((v >> 5) & 0x07);
  return true;
}


/**
- Configure gyroscope (range, bandwidth, power)
- range : dps125..dps2000
- bw : bandwidth
- pwr : gyro power mode
- return : true if write ok
*/
bool BNO055_7Semi::configGyro(uint8_t range, uint8_t bw, uint8_t pwr) {
  Mode prev = _mode;
  if (!setMode(Mode::CONFIG)) return false;
  delay(20);
  if (!ensurePage(1)) {
    setMode(prev);
    delay(20);
    return false;
  }

  // CONFIG0: range+BW
  uint8_t v0 = (range & 0x07) | ((bw & 0x07) << 3);
  bool ok = writeReg(P1_GYRO_CONFIG0, v0);

  // CONFIG1: power mode bits [2:0]
  uint8_t v1 = readReg(P1_GYRO_CONFIG1);
  v1 = (v1 & ~uint8_t(0b00000111)) | (pwr & 0x07);
  ok &= writeReg(P1_GYRO_CONFIG1, v1);

  ensurePage(0);
  setMode(prev);
  delay(20);
  return ok;
}


/**
- Read back gyroscope configuration
- range : out range
- bw : out bandwidth
- pwr : out power mode
- return : true
*/
bool BNO055_7Semi::getGyroConfig(uint8_t& range, uint8_t& bw, uint8_t& pwr) const {
  const_cast<BNO055_7Semi*>(this)->ensurePage(1);

  uint8_t v0 = const_cast<BNO055_7Semi*>(this)->readReg(P1_GYRO_CONFIG0);
  uint8_t v1 = const_cast<BNO055_7Semi*>(this)->readReg(P1_GYRO_CONFIG1);

  const_cast<BNO055_7Semi*>(this)->ensurePage(0);

  range = v0 & 0x07;         // bits 2:0
  bw    = (v0 >> 3) & 0x07;   // bits 5:3
  pwr   = v1 & 0x07;          // bits 2:0
  return true;
}


/**
- Write gyroscope raw offsets (page 0)
- x,y,z : raw register values
- return : true if writes ok
*/
bool BNO055_7Semi::setGyroOffset(int16_t x, int16_t y, int16_t z) {
  Mode prev = _mode;
  if (!setMode(Mode::CONFIG)) return false;
  delay(20);
  ensurePage(0);

  bool ok = write2Reg(this, REG_GYRO_OFF_X_LSB, x)
            && write2Reg(this, REG_GYRO_OFF_Y_LSB, y)
            && write2Reg(this, REG_GYRO_OFF_Z_LSB, z);

  setMode(prev);
  delay(20);
  return ok;
}

/**
- Read gyroscope raw offsets (page 0)
- x,y,z : out raw register values
- return : true
*/
bool BNO055_7Semi::getGyroOffset(int16_t& x, int16_t& y, int16_t& z) const {
  const_cast<BNO055_7Semi*>(this)->ensurePage(0);
  x = read2Reg(const_cast<BNO055_7Semi*>(this), REG_GYRO_OFF_X_LSB);
  y = read2Reg(const_cast<BNO055_7Semi*>(this), REG_GYRO_OFF_Y_LSB);
  z = read2Reg(const_cast<BNO055_7Semi*>(this), REG_GYRO_OFF_Z_LSB);
  return true;
}

/**
- Write magnetometer raw offsets (page 0)
- x,y,z : raw register values
- return : true if writes ok
*/
bool BNO055_7Semi::setMagOffset(int16_t x, int16_t y, int16_t z) {
  Mode prev = _mode;
  if (!setMode(Mode::CONFIG)) return false;
  delay(20);
  ensurePage(0);

  bool ok = write2Reg(this, REG_MAG_OFF_X_LSB, x)
            && write2Reg(this, REG_MAG_OFF_Y_LSB, y)
            && write2Reg(this, REG_MAG_OFF_Z_LSB, z);

  setMode(prev);
  delay(20);
  return ok;
}

/**
- Read magnetometer raw offsets (page 0)
- x,y,z : out raw register values
- return : true
*/
bool BNO055_7Semi::getMagOffset(int16_t& x, int16_t& y, int16_t& z) const {
  const_cast<BNO055_7Semi*>(this)->ensurePage(0);
  x = read2Reg(const_cast<BNO055_7Semi*>(this), REG_MAG_OFF_X_LSB);
  y = read2Reg(const_cast<BNO055_7Semi*>(this), REG_MAG_OFF_Y_LSB);
  z = read2Reg(const_cast<BNO055_7Semi*>(this), REG_MAG_OFF_Z_LSB);
  return true;
}

/**
- Write accelerometer raw offsets (page 0)
- x,y,z : raw register values
- return : true if writes ok
*/
bool BNO055_7Semi::setAccelOffset(int16_t x, int16_t y, int16_t z) {
  Mode prev = _mode;
  if (!setMode(Mode::CONFIG)) return false;
  delay(20);
  ensurePage(0);

  bool ok = write2Reg(this, REG_ACCEL_OFF_X_LSB, x)
            && write2Reg(this, REG_ACCEL_OFF_Y_LSB, y)
            && write2Reg(this, REG_ACCEL_OFF_Z_LSB, z);

  setMode(prev);
  delay(20);
  return ok;
}

/**
- Read accelerometer raw offsets (page 0)
- x,y,z : out raw register values
- return : true
*/
bool BNO055_7Semi::getAccelOffset(int16_t& x, int16_t& y, int16_t& z) const {
  const_cast<BNO055_7Semi*>(this)->ensurePage(0);
  x = read2Reg(const_cast<BNO055_7Semi*>(this), REG_ACCEL_OFF_X_LSB);
  y = read2Reg(const_cast<BNO055_7Semi*>(this), REG_ACCEL_OFF_Y_LSB);
  z = read2Reg(const_cast<BNO055_7Semi*>(this), REG_ACCEL_OFF_Z_LSB);
  return true;
}

/**
- Read signed 16-bit from LSB/MSB pair
- d : BNO055_7Semi pointer
- reg_lsb : LSB register
- return : combined int16_t
*/
int16_t BNO055_7Semi::read2Reg(BNO055_7Semi* d, uint8_t reg_lsb) {
  uint8_t b[2];
  if (!d->readNReg(reg_lsb, b, 2)) return 0;
  return int16_t(int16_t(b[1]) << 8 | b[0]);
}

/**
- Write signed 16-bit to LSB/MSB pair
- d : BNO055_7Semi pointer
- reg_lsb : LSB register
- v : value
- return : true if both bytes written
*/
bool BNO055_7Semi::write2Reg(BNO055_7Semi* d, uint8_t reg_lsb, int16_t v) {
  return d->writeReg(reg_lsb, uint8_t(v & 0xFF)) && d->writeReg(uint8_t(reg_lsb + 1), uint8_t((uint16_t(v) >> 8) & 0xFF));
}

/**
- Read calibration status register (0x35)
- return : [7:6]=SYS, [5:4]=GYR, [3:2]=ACC, [1:0]=MAG
*/
uint8_t BNO055_7Semi::readCalibStatus() const {
  return const_cast<BNO055_7Semi*>(this)->readReg(REG_CALIB_STAT);
}
