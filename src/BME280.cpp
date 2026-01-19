/**
 * @file BME280.cpp
 * @brief Minimal driver implementation scaffold for BME280.
 */

#include "BME280/BME280.h"

#include <Arduino.h>
#include <cstring>
#include <limits>

namespace BME280 {
namespace {
static constexpr size_t MAX_WRITE_LEN = 16;
}

Status BME280::begin(const Config& config) {
  _initialized = false;
  _driverState = DriverState::UNINIT;
  _lastOkMs = 0;
  _lastErrorMs = 0;
  _lastError = Status::Ok();
  _consecutiveFailures = 0;
  _totalFailures = 0;
  _totalSuccess = 0;

  if (config.i2cWrite == nullptr || config.i2cWriteRead == nullptr) {
    return Status::Error(Err::INVALID_CONFIG, "I2C callbacks not set");
  }
  if (config.i2cTimeoutMs == 0) {
    return Status::Error(Err::INVALID_CONFIG, "I2C timeout must be > 0");
  }

  _config = config;
  if (_config.offlineThreshold == 0) {
    _config.offlineThreshold = 1;
  }

  _initialized = true;
  _driverState = DriverState::READY;

  return Status::Ok();
}

void BME280::tick(uint32_t nowMs) {
  (void)nowMs;
}

void BME280::end() {
  _initialized = false;
  _driverState = DriverState::UNINIT;
}

Status BME280::probe() {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }

  uint8_t reg = cmd::REG_CHIP_ID;
  uint8_t id = 0;
  Status st = _i2cWriteReadRaw(&reg, 1, &id, 1);
  if (!st.ok()) {
    return st;
  }
  if (id != cmd::CHIP_ID_BME280) {
    return Status::Error(Err::CHIP_ID_MISMATCH, "Chip ID mismatch", id);
  }

  return Status::Ok();
}

Status BME280::recover() {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }

  uint8_t reg = cmd::REG_CHIP_ID;
  uint8_t id = 0;
  Status st = _i2cWriteReadTracked(&reg, 1, &id, 1);
  if (!st.ok()) {
    return st;
  }
  if (id != cmd::CHIP_ID_BME280) {
    return Status::Error(Err::CHIP_ID_MISMATCH, "Chip ID mismatch", id);
  }

  return Status::Ok();
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
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  if (txBuf == nullptr || txLen == 0 || (rxLen > 0 && rxBuf == nullptr)) {
    return Status::Error(Err::INVALID_PARAM, "Invalid I2C buffer");
  }

  Status st = _i2cWriteReadRaw(txBuf, txLen, rxBuf, rxLen);
  if (st.code == Err::INVALID_CONFIG || st.code == Err::INVALID_PARAM ||
      st.code == Err::NOT_INITIALIZED) {
    return st;
  }
  return _updateHealth(st);
}

Status BME280::_i2cWriteTracked(const uint8_t* buf, size_t len) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  if (buf == nullptr || len == 0) {
    return Status::Error(Err::INVALID_PARAM, "Invalid I2C buffer");
  }

  Status st = _i2cWriteRaw(buf, len);
  if (st.code == Err::INVALID_CONFIG || st.code == Err::INVALID_PARAM ||
      st.code == Err::NOT_INITIALIZED) {
    return st;
  }
  return _updateHealth(st);
}

Status BME280::readRegs(uint8_t startReg, uint8_t* buf, size_t len) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  if (buf == nullptr || len == 0) {
    return Status::Error(Err::INVALID_PARAM, "Invalid read buffer");
  }

  uint8_t reg = startReg;
  return _i2cWriteReadTracked(&reg, 1, buf, len);
}

Status BME280::writeRegs(uint8_t startReg, const uint8_t* buf, size_t len) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
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

}  // namespace BME280
