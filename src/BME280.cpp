/**
 * @file BME280.cpp
 * @brief BME280 driver implementation.
 */

#include "BME280/BME280.h"

#include <Arduino.h>
#include <cstring>
#include <limits>

namespace BME280 {
namespace {

static constexpr size_t MAX_WRITE_LEN = 16;
static constexpr uint32_t RESET_TIMEOUT_MS = 10;
static constexpr uint32_t MEASUREMENT_MARGIN_US = 1000;

static uint8_t osrsToReg(Oversampling osrs) {
  return static_cast<uint8_t>(osrs) & 0x07;
}

static uint8_t modeToReg(Mode mode) {
  return static_cast<uint8_t>(mode) & 0x03;
}

static uint8_t filterToReg(Filter filter) {
  return static_cast<uint8_t>(filter) & 0x07;
}

static uint8_t standbyToReg(Standby standby) {
  return static_cast<uint8_t>(standby) & 0x07;
}

static uint8_t osrsMultiplier(Oversampling osrs) {
  switch (osrs) {
    case Oversampling::SKIP: return 0;
    case Oversampling::X1: return 1;
    case Oversampling::X2: return 2;
    case Oversampling::X4: return 4;
    case Oversampling::X8: return 8;
    case Oversampling::X16: return 16;
    default: return 0;
  }
}

static bool isValidOversampling(Oversampling osrs) {
  const uint8_t value = osrsToReg(osrs);
  return value <= 5;
}

static bool isValidFilter(Filter filter) {
  const uint8_t value = filterToReg(filter);
  return value <= 4;
}

static bool isValidStandby(Standby standby) {
  const uint8_t value = standbyToReg(standby);
  return value <= 7;
}

static bool isValidMode(Mode mode) {
  return mode == Mode::SLEEP || mode == Mode::FORCED || mode == Mode::NORMAL;
}

static uint8_t buildCtrlHum(Oversampling osrsH) {
  return static_cast<uint8_t>(osrsToReg(osrsH) << cmd::BIT_CTRL_HUM_OSRS_H);
}

static uint8_t buildCtrlMeas(Oversampling osrsT, Oversampling osrsP, Mode mode) {
  return static_cast<uint8_t>((osrsToReg(osrsT) << cmd::BIT_CTRL_MEAS_OSRS_T) |
                              (osrsToReg(osrsP) << cmd::BIT_CTRL_MEAS_OSRS_P) |
                              (modeToReg(mode) << cmd::BIT_CTRL_MEAS_MODE));
}

static uint8_t buildConfig(Standby standby, Filter filter) {
  return static_cast<uint8_t>((standbyToReg(standby) << cmd::BIT_CONFIG_T_SB) |
                              (filterToReg(filter) << cmd::BIT_CONFIG_FILTER));
}

static int16_t signExtend12(int16_t value) {
  if (value & 0x0800) {
    value |= 0xF000;
  }
  return value;
}

}  // namespace

Status BME280::begin(const Config& config) {
  _initialized = false;
  _driverState = DriverState::UNINIT;

  _lastOkMs = 0;
  _lastErrorMs = 0;
  _lastError = Status::Ok();
  _consecutiveFailures = 0;
  _totalFailures = 0;
  _totalSuccess = 0;

  _measurementRequested = false;
  _measurementReady = false;
  _measurementStartMs = 0;
  _tFine = 0;
  _rawSample = RawSample{};
  _compSample = CompensatedSample{};

  if (config.i2cWrite == nullptr || config.i2cWriteRead == nullptr) {
    return Status::Error(Err::INVALID_CONFIG, "I2C callbacks not set");
  }
  if (config.i2cTimeoutMs == 0) {
    return Status::Error(Err::INVALID_CONFIG, "I2C timeout must be > 0");
  }
  if (config.i2cAddress != 0x76 && config.i2cAddress != 0x77) {
    return Status::Error(Err::INVALID_CONFIG, "Invalid I2C address");
  }
  if (!isValidOversampling(config.osrsT) ||
      !isValidOversampling(config.osrsP) ||
      !isValidOversampling(config.osrsH) ||
      !isValidFilter(config.filter) ||
      !isValidStandby(config.standby) ||
      !isValidMode(config.mode)) {
    return Status::Error(Err::INVALID_CONFIG, "Invalid configuration value");
  }

  _config = config;
  if (_config.offlineThreshold == 0) {
    _config.offlineThreshold = 1;
  }

  uint8_t chipId = 0;
  Status st = _readRegisterRaw(cmd::REG_CHIP_ID, chipId);
  if (!st.ok()) {
    return Status::Error(Err::DEVICE_NOT_FOUND, "Device not responding", st.detail);
  }
  if (chipId != cmd::CHIP_ID_BME280) {
    return Status::Error(Err::CHIP_ID_MISMATCH, "Chip ID mismatch", chipId);
  }

  st = _readCalibration();
  if (!st.ok()) {
    return st;
  }
  st = _validateCalibration();
  if (!st.ok()) {
    return st;
  }
  st = _applyConfig();
  if (!st.ok()) {
    return st;
  }

  _initialized = true;
  _driverState = DriverState::READY;

  return Status::Ok();
}

void BME280::tick(uint32_t nowMs) {
  if (!_initialized || !_measurementRequested) {
    return;
  }

  if (_config.mode == Mode::SLEEP) {
    return;
  }

  if (_config.mode == Mode::FORCED) {
    const uint32_t deadline = _measurementStartMs + estimateMeasurementTimeMs();
    if (static_cast<int32_t>(nowMs - deadline) < 0) {
      return;
    }
  }

  bool measuring = false;
  Status st = isMeasuring(measuring);
  if (!st.ok() || measuring) {
    return;
  }

  st = _readRawData();
  if (!st.ok()) {
    return;
  }

  st = _compensate();
  if (!st.ok()) {
    return;
  }

  _measurementReady = true;
  _measurementRequested = false;
}

void BME280::end() {
  _initialized = false;
  _driverState = DriverState::UNINIT;
}

Status BME280::probe() {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }

  uint8_t chipId = 0;
  Status st = _readRegisterRaw(cmd::REG_CHIP_ID, chipId);
  if (!st.ok()) {
    return Status::Error(Err::DEVICE_NOT_FOUND, "Device not responding", st.detail);
  }
  if (chipId != cmd::CHIP_ID_BME280) {
    return Status::Error(Err::CHIP_ID_MISMATCH, "Chip ID mismatch", chipId);
  }

  return Status::Ok();
}

Status BME280::recover() {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }

  uint8_t chipId = 0;
  Status st = readRegister(cmd::REG_CHIP_ID, chipId);
  if (!st.ok()) {
    return st;
  }
  if (chipId != cmd::CHIP_ID_BME280) {
    return Status::Error(Err::CHIP_ID_MISMATCH, "Chip ID mismatch", chipId);
  }

  return Status::Ok();
}

Status BME280::requestMeasurement() {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  if (_config.mode == Mode::SLEEP) {
    return Status::Error(Err::INVALID_PARAM, "Device is in sleep mode");
  }
  if (_measurementRequested && !_measurementReady) {
    return Status::Error(Err::BUSY, "Measurement in progress");
  }

  _measurementReady = false;

  if (_config.mode == Mode::FORCED) {
    bool measuring = false;
    Status st = isMeasuring(measuring);
    if (!st.ok()) {
      return st;
    }
    if (measuring) {
      return Status::Error(Err::BUSY, "Device is measuring");
    }

    const uint8_t ctrlMeas = buildCtrlMeas(_config.osrsT, _config.osrsP, Mode::FORCED);
    st = writeRegister(cmd::REG_CTRL_MEAS, ctrlMeas);
    if (!st.ok()) {
      return st;
    }

    _measurementRequested = true;
    _measurementStartMs = millis();

    return Status::Error(Err::IN_PROGRESS, "Measurement started");
  }

  _measurementRequested = true;
  return Status::Error(Err::IN_PROGRESS, "Measurement scheduled");
}

Status BME280::getMeasurement(Measurement& out) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  if (!_measurementReady) {
    return Status::Error(Err::MEASUREMENT_NOT_READY, "Measurement not ready");
  }

  out.temperatureC = static_cast<float>(_compSample.tempC_x100) / 100.0f;
  out.pressurePa = static_cast<float>(_compSample.pressurePa);
  out.humidityPct = static_cast<float>(_compSample.humidityPct_x1024) / 1024.0f;

  _measurementReady = false;
  return Status::Ok();
}

Status BME280::getRawSample(RawSample& out) const {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  if (!_measurementReady) {
    return Status::Error(Err::MEASUREMENT_NOT_READY, "Measurement not ready");
  }

  out = _rawSample;
  return Status::Ok();
}

Status BME280::getCompensatedSample(CompensatedSample& out) const {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  if (!_measurementReady) {
    return Status::Error(Err::MEASUREMENT_NOT_READY, "Measurement not ready");
  }

  out = _compSample;
  return Status::Ok();
}

Status BME280::getCalibration(Calibration& out) const {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }

  out.digT1 = _digT1;
  out.digT2 = _digT2;
  out.digT3 = _digT3;

  out.digP1 = _digP1;
  out.digP2 = _digP2;
  out.digP3 = _digP3;
  out.digP4 = _digP4;
  out.digP5 = _digP5;
  out.digP6 = _digP6;
  out.digP7 = _digP7;
  out.digP8 = _digP8;
  out.digP9 = _digP9;

  out.digH1 = _digH1;
  out.digH2 = _digH2;
  out.digH3 = _digH3;
  out.digH4 = _digH4;
  out.digH5 = _digH5;
  out.digH6 = _digH6;

  return Status::Ok();
}

Status BME280::readCalibrationRaw(CalibrationRaw& out) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }

  Status st = readRegs(cmd::REG_CALIB_TP_START, out.tp, sizeof(out.tp));
  if (!st.ok()) {
    return st;
  }

  st = readRegs(cmd::REG_CALIB_H1, &out.h1, 1);
  if (!st.ok()) {
    return st;
  }

  return readRegs(cmd::REG_CALIB_H_START, out.h, sizeof(out.h));
}

Status BME280::setMode(Mode mode) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  if (!isValidMode(mode)) {
    return Status::Error(Err::INVALID_PARAM, "Invalid mode");
  }

  if (mode == Mode::SLEEP) {
    _measurementRequested = false;
  }

  _config.mode = mode;
  const uint8_t ctrlMeas = buildCtrlMeas(_config.osrsT, _config.osrsP, _config.mode);
  return writeRegister(cmd::REG_CTRL_MEAS, ctrlMeas);
}

Status BME280::getMode(Mode& out) const {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  out = _config.mode;
  return Status::Ok();
}

Status BME280::setOversamplingT(Oversampling osrs) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  if (!isValidOversampling(osrs)) {
    return Status::Error(Err::INVALID_PARAM, "Invalid oversampling");
  }

  _config.osrsT = osrs;
  const uint8_t ctrlMeas = buildCtrlMeas(_config.osrsT, _config.osrsP, _config.mode);
  return writeRegister(cmd::REG_CTRL_MEAS, ctrlMeas);
}

Status BME280::setOversamplingP(Oversampling osrs) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  if (!isValidOversampling(osrs)) {
    return Status::Error(Err::INVALID_PARAM, "Invalid oversampling");
  }

  _config.osrsP = osrs;
  const uint8_t ctrlMeas = buildCtrlMeas(_config.osrsT, _config.osrsP, _config.mode);
  return writeRegister(cmd::REG_CTRL_MEAS, ctrlMeas);
}

Status BME280::setOversamplingH(Oversampling osrs) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  if (!isValidOversampling(osrs)) {
    return Status::Error(Err::INVALID_PARAM, "Invalid oversampling");
  }

  _config.osrsH = osrs;
  const uint8_t ctrlHum = buildCtrlHum(_config.osrsH);
  Status st = writeRegister(cmd::REG_CTRL_HUM, ctrlHum);
  if (!st.ok()) {
    return st;
  }

  const uint8_t ctrlMeas = buildCtrlMeas(_config.osrsT, _config.osrsP, _config.mode);
  return writeRegister(cmd::REG_CTRL_MEAS, ctrlMeas);
}

Status BME280::setFilter(Filter filter) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  if (!isValidFilter(filter)) {
    return Status::Error(Err::INVALID_PARAM, "Invalid filter");
  }

  _config.filter = filter;
  const uint8_t config = buildConfig(_config.standby, _config.filter);
  const uint8_t ctrlMeasSleep = buildCtrlMeas(_config.osrsT, _config.osrsP, Mode::SLEEP);
  const uint8_t ctrlMeas = buildCtrlMeas(_config.osrsT, _config.osrsP, _config.mode);

  Status st = writeRegister(cmd::REG_CTRL_MEAS, ctrlMeasSleep);
  if (!st.ok()) {
    return st;
  }
  st = writeRegister(cmd::REG_CONFIG, config);
  if (!st.ok()) {
    return st;
  }
  return writeRegister(cmd::REG_CTRL_MEAS, ctrlMeas);
}

Status BME280::setStandby(Standby standby) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  if (!isValidStandby(standby)) {
    return Status::Error(Err::INVALID_PARAM, "Invalid standby");
  }

  _config.standby = standby;
  const uint8_t config = buildConfig(_config.standby, _config.filter);
  const uint8_t ctrlMeasSleep = buildCtrlMeas(_config.osrsT, _config.osrsP, Mode::SLEEP);
  const uint8_t ctrlMeas = buildCtrlMeas(_config.osrsT, _config.osrsP, _config.mode);

  Status st = writeRegister(cmd::REG_CTRL_MEAS, ctrlMeasSleep);
  if (!st.ok()) {
    return st;
  }
  st = writeRegister(cmd::REG_CONFIG, config);
  if (!st.ok()) {
    return st;
  }
  return writeRegister(cmd::REG_CTRL_MEAS, ctrlMeas);
}

Status BME280::getOversamplingT(Oversampling& out) const {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  out = _config.osrsT;
  return Status::Ok();
}

Status BME280::getOversamplingP(Oversampling& out) const {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  out = _config.osrsP;
  return Status::Ok();
}

Status BME280::getOversamplingH(Oversampling& out) const {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  out = _config.osrsH;
  return Status::Ok();
}

Status BME280::getFilter(Filter& out) const {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  out = _config.filter;
  return Status::Ok();
}

Status BME280::getStandby(Standby& out) const {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  out = _config.standby;
  return Status::Ok();
}

Status BME280::softReset() {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }

  Status st = writeRegister(cmd::REG_RESET, cmd::RESET_VALUE);
  if (!st.ok()) {
    return st;
  }

  const uint32_t deadline = millis() + RESET_TIMEOUT_MS;
  while (true) {
    uint8_t status = 0;
    st = readRegister(cmd::REG_STATUS, status);
    if (!st.ok()) {
      return st;
    }
    if ((status & cmd::MASK_STATUS_IM_UPDATE) == 0) {
      break;
    }
    if (static_cast<int32_t>(millis() - deadline) >= 0) {
      return Status::Error(Err::TIMEOUT, "Reset timeout");
    }
  }

  st = _readCalibration();
  if (!st.ok()) {
    return st;
  }
  st = _validateCalibration();
  if (!st.ok()) {
    return st;
  }
  return _applyConfig();
}

Status BME280::readChipId(uint8_t& id) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  return readRegister(cmd::REG_CHIP_ID, id);
}

Status BME280::readStatus(uint8_t& status) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  return readRegister(cmd::REG_STATUS, status);
}

Status BME280::readCtrlHum(uint8_t& value) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  return readRegister(cmd::REG_CTRL_HUM, value);
}

Status BME280::readCtrlMeas(uint8_t& value) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  return readRegister(cmd::REG_CTRL_MEAS, value);
}

Status BME280::readConfig(uint8_t& value) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  return readRegister(cmd::REG_CONFIG, value);
}

Status BME280::isMeasuring(bool& measuring) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }

  uint8_t status = 0;
  Status st = readRegister(cmd::REG_STATUS, status);
  if (!st.ok()) {
    return st;
  }

  measuring = (status & cmd::MASK_STATUS_MEASURING) != 0;
  return Status::Ok();
}

uint32_t BME280::estimateMeasurementTimeMs() const {
  const uint8_t t_osrs = osrsMultiplier(_config.osrsT);
  const uint8_t p_osrs = osrsMultiplier(_config.osrsP);
  const uint8_t h_osrs = osrsMultiplier(_config.osrsH);

  uint32_t timeUs = 1250;
  if (t_osrs > 0) {
    timeUs += 2300U * t_osrs;
  }
  if (p_osrs > 0) {
    timeUs += 2300U * p_osrs + 575U;
  }
  if (h_osrs > 0) {
    timeUs += 2300U * h_osrs + 575U;
  }
  timeUs += MEASUREMENT_MARGIN_US;

  return (timeUs + 999U) / 1000U;
}

Status BME280::_i2cWriteReadRaw(const uint8_t* txBuf, size_t txLen,
                                uint8_t* rxBuf, size_t rxLen) {
  if (_config.i2cWriteRead == nullptr) {
    return Status::Error(Err::INVALID_CONFIG, "I2C write-read not set");
  }
  return _config.i2cWriteRead(_config.i2cAddress, txBuf, txLen, rxBuf, rxLen,
                              _config.i2cTimeoutMs, _config.i2cUser);
}

Status BME280::_i2cWriteRaw(const uint8_t* buf, size_t len) {
  if (_config.i2cWrite == nullptr) {
    return Status::Error(Err::INVALID_CONFIG, "I2C write not set");
  }
  return _config.i2cWrite(_config.i2cAddress, buf, len, _config.i2cTimeoutMs,
                          _config.i2cUser);
}

Status BME280::_i2cWriteReadTracked(const uint8_t* txBuf, size_t txLen,
                                    uint8_t* rxBuf, size_t rxLen) {
  if (txBuf == nullptr || txLen == 0 || (rxLen > 0 && rxBuf == nullptr)) {
    return Status::Error(Err::INVALID_PARAM, "Invalid I2C buffer");
  }

  Status st = _i2cWriteReadRaw(txBuf, txLen, rxBuf, rxLen);
  if (st.code == Err::INVALID_CONFIG || st.code == Err::INVALID_PARAM) {
    return st;
  }
  return _updateHealth(st);
}

Status BME280::_i2cWriteTracked(const uint8_t* buf, size_t len) {
  if (buf == nullptr || len == 0) {
    return Status::Error(Err::INVALID_PARAM, "Invalid I2C buffer");
  }

  Status st = _i2cWriteRaw(buf, len);
  if (st.code == Err::INVALID_CONFIG || st.code == Err::INVALID_PARAM) {
    return st;
  }
  return _updateHealth(st);
}

Status BME280::readRegs(uint8_t startReg, uint8_t* buf, size_t len) {
  if (buf == nullptr || len == 0) {
    return Status::Error(Err::INVALID_PARAM, "Invalid read buffer");
  }

  uint8_t reg = startReg;
  return _i2cWriteReadTracked(&reg, 1, buf, len);
}

Status BME280::writeRegs(uint8_t startReg, const uint8_t* buf, size_t len) {
  if (buf == nullptr || len == 0) {
    return Status::Error(Err::INVALID_PARAM, "Invalid write buffer");
  }
  if (len > MAX_WRITE_LEN) {
    return Status::Error(Err::INVALID_PARAM, "Write length too large");
  }

  uint8_t payload[MAX_WRITE_LEN + 1] = {};
  payload[0] = startReg;
  std::memcpy(&payload[1], buf, len);

  return _i2cWriteTracked(payload, len + 1);
}

Status BME280::readRegister(uint8_t reg, uint8_t& value) {
  return readRegs(reg, &value, 1);
}

Status BME280::writeRegister(uint8_t reg, uint8_t value) {
  return writeRegs(reg, &value, 1);
}

Status BME280::_readRegisterRaw(uint8_t reg, uint8_t& value) {
  uint8_t addr = reg;
  return _i2cWriteReadRaw(&addr, 1, &value, 1);
}

Status BME280::_updateHealth(const Status& st) {
  if (!_initialized) {
    return st;
  }

  const uint32_t now = millis();
  const uint32_t maxU32 = std::numeric_limits<uint32_t>::max();
  const uint8_t maxU8 = std::numeric_limits<uint8_t>::max();

  if (st.ok()) {
    _lastOkMs = now;
    if (_totalSuccess < maxU32) {
      _totalSuccess++;
    }
    _consecutiveFailures = 0;
    _driverState = DriverState::READY;
    return st;
  }

  _lastError = st;
  _lastErrorMs = now;
  if (_totalFailures < maxU32) {
    _totalFailures++;
  }
  if (_consecutiveFailures < maxU8) {
    _consecutiveFailures++;
  }

  if (_consecutiveFailures >= _config.offlineThreshold) {
    _driverState = DriverState::OFFLINE;
  } else {
    _driverState = DriverState::DEGRADED;
  }

  return st;
}

Status BME280::_applyConfig() {
  const uint8_t ctrlHum = buildCtrlHum(_config.osrsH);
  const uint8_t ctrlMeasSleep = buildCtrlMeas(_config.osrsT, _config.osrsP, Mode::SLEEP);
  const uint8_t ctrlMeas = buildCtrlMeas(_config.osrsT, _config.osrsP, _config.mode);
  const uint8_t config = buildConfig(_config.standby, _config.filter);

  Status st = writeRegister(cmd::REG_CTRL_MEAS, ctrlMeasSleep);
  if (!st.ok()) {
    return st;
  }

  st = writeRegister(cmd::REG_CONFIG, config);
  if (!st.ok()) {
    return st;
  }

  st = writeRegister(cmd::REG_CTRL_HUM, ctrlHum);
  if (!st.ok()) {
    return st;
  }

  return writeRegister(cmd::REG_CTRL_MEAS, ctrlMeas);
}

Status BME280::_readCalibration() {
  uint8_t calibTP[cmd::REG_CALIB_TP_LEN] = {};
  Status st = readRegs(cmd::REG_CALIB_TP_START, calibTP, sizeof(calibTP));
  if (!st.ok()) {
    return st;
  }

  _digT1 = static_cast<uint16_t>((calibTP[1] << 8) | calibTP[0]);
  _digT2 = static_cast<int16_t>((calibTP[3] << 8) | calibTP[2]);
  _digT3 = static_cast<int16_t>((calibTP[5] << 8) | calibTP[4]);

  _digP1 = static_cast<uint16_t>((calibTP[7] << 8) | calibTP[6]);
  _digP2 = static_cast<int16_t>((calibTP[9] << 8) | calibTP[8]);
  _digP3 = static_cast<int16_t>((calibTP[11] << 8) | calibTP[10]);
  _digP4 = static_cast<int16_t>((calibTP[13] << 8) | calibTP[12]);
  _digP5 = static_cast<int16_t>((calibTP[15] << 8) | calibTP[14]);
  _digP6 = static_cast<int16_t>((calibTP[17] << 8) | calibTP[16]);
  _digP7 = static_cast<int16_t>((calibTP[19] << 8) | calibTP[18]);
  _digP8 = static_cast<int16_t>((calibTP[21] << 8) | calibTP[20]);
  _digP9 = static_cast<int16_t>((calibTP[23] << 8) | calibTP[22]);

  uint8_t h1 = 0;
  st = readRegs(cmd::REG_CALIB_H1, &h1, 1);
  if (!st.ok()) {
    return st;
  }
  _digH1 = h1;

  uint8_t calibH[cmd::REG_CALIB_H_LEN] = {};
  st = readRegs(cmd::REG_CALIB_H_START, calibH, sizeof(calibH));
  if (!st.ok()) {
    return st;
  }

  _digH2 = static_cast<int16_t>((calibH[1] << 8) | calibH[0]);
  _digH3 = calibH[2];

  int16_t h4 = static_cast<int16_t>((calibH[3] << 4) | (calibH[4] & 0x0F));
  int16_t h5 = static_cast<int16_t>((calibH[5] << 4) | (calibH[4] >> 4));
  _digH4 = signExtend12(h4);
  _digH5 = signExtend12(h5);
  _digH6 = static_cast<int8_t>(calibH[6]);

  return Status::Ok();
}

Status BME280::_validateCalibration() {
  if (_digT1 == 0 || _digT1 == 0xFFFF) {
    return Status::Error(Err::CALIBRATION_INVALID, "Invalid temperature calibration");
  }
  if (_digP1 == 0 || _digP1 == 0xFFFF) {
    return Status::Error(Err::CALIBRATION_INVALID, "Invalid pressure calibration");
  }

  return Status::Ok();
}

Status BME280::_readRawData() {
  uint8_t data[cmd::DATA_LEN] = {};
  Status st = readRegs(cmd::REG_DATA_START, data, sizeof(data));
  if (!st.ok()) {
    return st;
  }

  _rawSample.adcP = (static_cast<int32_t>(data[0]) << 12) |
                    (static_cast<int32_t>(data[1]) << 4) |
                    (static_cast<int32_t>(data[2]) >> 4);
  _rawSample.adcT = (static_cast<int32_t>(data[3]) << 12) |
                    (static_cast<int32_t>(data[4]) << 4) |
                    (static_cast<int32_t>(data[5]) >> 4);
  _rawSample.adcH = (static_cast<int32_t>(data[6]) << 8) |
                    static_cast<int32_t>(data[7]);

  return Status::Ok();
}

Status BME280::_compensate() {
  const int32_t adcT = _rawSample.adcT;
  const int32_t adcP = _rawSample.adcP;
  const int32_t adcH = _rawSample.adcH;

  int32_t var1 = (((adcT >> 3) - (static_cast<int32_t>(_digT1) << 1)) *
                  static_cast<int32_t>(_digT2)) >> 11;
  int32_t var2 = (((((adcT >> 4) - static_cast<int32_t>(_digT1)) *
                    ((adcT >> 4) - static_cast<int32_t>(_digT1))) >> 12) *
                  static_cast<int32_t>(_digT3)) >> 14;

  _tFine = var1 + var2;
  _compSample.tempC_x100 = (_tFine * 5 + 128) >> 8;

  int64_t pVar1 = static_cast<int64_t>(_tFine) - 128000;
  int64_t pVar2 = pVar1 * pVar1 * static_cast<int64_t>(_digP6);
  pVar2 = pVar2 + ((pVar1 * static_cast<int64_t>(_digP5)) << 17);
  pVar2 = pVar2 + (static_cast<int64_t>(_digP4) << 35);
  pVar1 = ((pVar1 * pVar1 * static_cast<int64_t>(_digP3)) >> 8) +
          ((pVar1 * static_cast<int64_t>(_digP2)) << 12);
  pVar1 = (((static_cast<int64_t>(1) << 47) + pVar1) *
           static_cast<int64_t>(_digP1)) >> 33;
  if (pVar1 == 0) {
    return Status::Error(Err::COMPENSATION_ERROR, "Pressure div by zero");
  }

  int64_t p = 1048576 - static_cast<int64_t>(adcP);
  p = (((p << 31) - pVar2) * 3125) / pVar1;
  pVar1 = (static_cast<int64_t>(_digP9) * (p >> 13) * (p >> 13)) >> 25;
  pVar2 = (static_cast<int64_t>(_digP8) * p) >> 19;
  p = ((p + pVar1 + pVar2) >> 8) + (static_cast<int64_t>(_digP7) << 4);
  _compSample.pressurePa = static_cast<uint32_t>(p >> 8);

  int32_t h = _tFine - 76800;
  h = (((((adcH << 14) - (static_cast<int32_t>(_digH4) << 20) -
          (static_cast<int32_t>(_digH5) * h)) + 16384) >> 15) *
       (((((((h * static_cast<int32_t>(_digH6)) >> 10) *
            (((h * static_cast<int32_t>(_digH3)) >> 11) + 32768)) >> 10) +
           2097152) * static_cast<int32_t>(_digH2) + 8192) >> 14));
  h = (h - (((((h >> 15) * (h >> 15)) >> 7) *
             static_cast<int32_t>(_digH1)) >> 4));
  if (h < 0) {
    h = 0;
  }
  if (h > 419430400) {
    h = 419430400;
  }
  _compSample.humidityPct_x1024 = static_cast<uint32_t>(h >> 12);

  return Status::Ok();
}

}  // namespace BME280
