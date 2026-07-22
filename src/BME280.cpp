/**
 * @file BME280.cpp
 * @brief BME280 driver implementation.
 */

#include "BME280/BME280.h"

#include <cstring>
#include <limits>

namespace BME280 {
namespace {

static constexpr size_t MAX_WRITE_LEN = 16;
static constexpr uint16_t NVM_READY_MAX_POLLS = 255;
static constexpr uint16_t MEASURING_READY_MAX_POLLS = 255;
static constexpr uint32_t MEASUREMENT_MARGIN_US = 1000;
static constexpr uint32_t MAX_STANDBY_TIME_MS = 1000;
static constexpr int64_t HUMIDITY_MAX_X4096 = 419430400;

class ScopedTimeContext {
public:
  ScopedTimeContext(bool& active, uint32_t& value, uint32_t nowMs)
      : _active(active), _value(value), _priorActive(active), _priorValue(value) {
    _active = true;
    _value = nowMs;
  }

  ~ScopedTimeContext() {
    _active = _priorActive;
    _value = _priorValue;
  }

  ScopedTimeContext(const ScopedTimeContext&) = delete;
  ScopedTimeContext& operator=(const ScopedTimeContext&) = delete;

private:
  bool& _active;
  uint32_t& _value;
  bool _priorActive;
  uint32_t _priorValue;
};

static bool deadlineReached(uint32_t nowMs, uint32_t deadlineMs) {
  return static_cast<int32_t>(nowMs - deadlineMs) >= 0;
}

static bool checkedAddI64(int64_t lhs, int64_t rhs, int64_t& out) {
  const int64_t maximum = std::numeric_limits<int64_t>::max();
  const int64_t minimum = std::numeric_limits<int64_t>::min();
  if ((rhs > 0 && lhs > maximum - rhs) ||
      (rhs < 0 && lhs < minimum - rhs)) {
    return false;
  }
  out = lhs + rhs;
  return true;
}

static bool checkedSubI64(int64_t lhs, int64_t rhs, int64_t& out) {
  const int64_t maximum = std::numeric_limits<int64_t>::max();
  const int64_t minimum = std::numeric_limits<int64_t>::min();
  if ((rhs > 0 && lhs < minimum + rhs) ||
      (rhs < 0 && lhs > maximum + rhs)) {
    return false;
  }
  out = lhs - rhs;
  return true;
}

static bool checkedMulI64(int64_t lhs, int64_t rhs, int64_t& out) {
  const int64_t maximum = std::numeric_limits<int64_t>::max();
  const int64_t minimum = std::numeric_limits<int64_t>::min();
  if (lhs > 0) {
    if ((rhs > 0 && lhs > maximum / rhs) ||
        (rhs < 0 && rhs < minimum / lhs)) {
      return false;
    }
  } else if (lhs < 0) {
    if ((rhs > 0 && lhs < minimum / rhs) ||
        (rhs < 0 && rhs < maximum / lhs)) {
      return false;
    }
  }
  out = lhs * rhs;
  return true;
}

static uint8_t osrsToReg(Oversampling osrs) {
  return static_cast<uint8_t>(osrs);
}

static uint8_t modeToReg(Mode mode) {
  return static_cast<uint8_t>(mode) & 0x03;
}

static uint8_t filterToReg(Filter filter) {
  return static_cast<uint8_t>(filter);
}

static uint8_t standbyToReg(Standby standby) {
  return static_cast<uint8_t>(standby);
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
  switch (osrs) {
    case Oversampling::SKIP:
    case Oversampling::X1:
    case Oversampling::X2:
    case Oversampling::X4:
    case Oversampling::X8:
    case Oversampling::X16:
      return true;
    default:
      return false;
  }
}

static bool isValidFilter(Filter filter) {
  switch (filter) {
    case Filter::OFF:
    case Filter::X2:
    case Filter::X4:
    case Filter::X8:
    case Filter::X16:
      return true;
    default:
      return false;
  }
}

static bool isValidStandby(Standby standby) {
  switch (standby) {
    case Standby::MS_0_5:
    case Standby::MS_62_5:
    case Standby::MS_125:
    case Standby::MS_250:
    case Standby::MS_500:
    case Standby::MS_1000:
    case Standby::MS_10:
    case Standby::MS_20:
      return true;
    default:
      return false;
  }
}

static bool isValidMode(Mode mode) {
  return mode == Mode::SLEEP || mode == Mode::FORCED || mode == Mode::NORMAL;
}

static Status mapPresenceError(const Status& st) {
  if (st.code == Err::I2C_NACK_ADDR) {
    return Status::Error(Err::DEVICE_NOT_FOUND, "Device not responding", st.detail);
  }
  return st;
}

static bool isTransportFailure(const Status& st) {
  switch (st.code) {
    case Err::I2C_ERROR:
    case Err::I2C_NACK_ADDR:
    case Err::I2C_NACK_DATA:
    case Err::I2C_TIMEOUT:
    case Err::I2C_BUS:
    case Err::I2C_SHORT_TRANSFER:
      return true;
    default:
      return false;
  }
}

static bool mayHaveReachedDeviceAfterDiagnosticWrite(const Status& st) {
  switch (st.code) {
    case Err::I2C_ERROR:
    case Err::I2C_NACK_DATA:
    case Err::I2C_TIMEOUT:
    case Err::I2C_BUS:
    case Err::I2C_SHORT_TRANSFER:
      return true;
    default:
      return false;
  }
}

static bool registerRangeContains(uint8_t startReg, size_t len, uint8_t reg) {
  if (len == 0) {
    return false;
  }
  const size_t start = startReg;
  const size_t target = reg;
  return target >= start && (target - start) < len;
}

static bool touchesDiagnosticConfigRegister(uint8_t startReg, size_t len) {
  return registerRangeContains(startReg, len, cmd::REG_CTRL_HUM) ||
         registerRangeContains(startReg, len, cmd::REG_CTRL_MEAS) ||
         registerRangeContains(startReg, len, cmd::REG_CONFIG) ||
         registerRangeContains(startReg, len, cmd::REG_RESET);
}

static Status diagnosticRawWriteDirtyStatus(uint8_t startReg) {
  return Status::Error(Err::INVALID_CONFIG,
                       "Diagnostic raw write may desync cached config",
                       startReg);
}

static bool isValidMeasurementSelection(Oversampling osrsT,
                                        Oversampling osrsP,
                                        Oversampling osrsH) {
  const bool tempSkipped = (osrsT == Oversampling::SKIP);
  const bool pressSkipped = (osrsP == Oversampling::SKIP);
  const bool humSkipped = (osrsH == Oversampling::SKIP);

  if (tempSkipped && (!pressSkipped || !humSkipped)) {
    return false;
  }
  return !(tempSkipped && pressSkipped && humSkipped);
}

static Mode registerModeForConfig(Mode mode) {
  return (mode == Mode::FORCED) ? Mode::SLEEP : mode;
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

static Status mapTransportResult(const TransportResult& result,
                                 size_t expectedWriteCount,
                                 size_t expectedReadCount) {
  switch (result.code) {
    case TransportErr::OK:
      if (result.writeCount != expectedWriteCount ||
          result.readCount != expectedReadCount) {
        return Status::Error(Err::I2C_SHORT_TRANSFER, result.detail);
      }
      return Status::Ok();
    case TransportErr::NACK_ADDRESS:
      return Status::Error(Err::I2C_NACK_ADDR, result.detail);
    case TransportErr::NACK_DATA:
      return Status::Error(Err::I2C_NACK_DATA, result.detail);
    case TransportErr::TIMEOUT:
      return Status::Error(Err::I2C_TIMEOUT, result.detail);
    case TransportErr::BUS:
      return Status::Error(Err::I2C_BUS, result.detail);
    case TransportErr::OTHER:
      return Status::Error(Err::I2C_ERROR, result.detail);
    default:
      return Status::Error(Err::I2C_ERROR, result.detail);
  }
}

static bool bytesAllEqual(const uint8_t* data, size_t len, uint8_t value) {
  if (data == nullptr || len == 0) {
    return false;
  }
  for (size_t i = 0; i < len; ++i) {
    if (data[i] != value) {
      return false;
    }
  }
  return true;
}

static bool humidityCalibrationBlockValid(const uint8_t* data, size_t len) {
  return !bytesAllEqual(data, len, 0x00) &&
         !bytesAllEqual(data, len, 0xFF);
}

}  // namespace

Status validateSettings(const SensorSettings& settings) {
  if (!isValidOversampling(settings.osrsT) ||
      !isValidOversampling(settings.osrsP) ||
      !isValidOversampling(settings.osrsH) ||
      !isValidFilter(settings.filter) ||
      !isValidStandby(settings.standby) ||
      !isValidMode(settings.mode) ||
      !isValidMeasurementSelection(settings.osrsT, settings.osrsP,
                                   settings.osrsH)) {
    return Status::Error(Err::INVALID_PARAM);
  }
  return Status::Ok();
}

uint32_t estimateMeasurementTimeUs(const SensorSettings& settings) {
  if (!validateSettings(settings).ok()) {
    return 0;
  }
  const uint8_t tOsrs = osrsMultiplier(settings.osrsT);
  const uint8_t pOsrs = osrsMultiplier(settings.osrsP);
  const uint8_t hOsrs = osrsMultiplier(settings.osrsH);

  uint32_t timeUs = 1250;
  if (tOsrs > 0) {
    timeUs += 2300U * tOsrs;
  }
  if (pOsrs > 0) {
    timeUs += 2300U * pOsrs + 575U;
  }
  if (hOsrs > 0) {
    timeUs += 2300U * hOsrs + 575U;
  }
  return timeUs;
}

uint32_t estimateMeasurementTimeMs(const SensorSettings& settings) {
  const uint32_t boschTimeUs = estimateMeasurementTimeUs(settings);
  if (boschTimeUs == 0) {
    return 0;
  }
  return (boschTimeUs + MEASUREMENT_MARGIN_US + 999U) / 1000U;
}

Status temperatureX100ToMilliC(int32_t tempC_x100, int32_t& outMilliC) {
  const int64_t candidate = static_cast<int64_t>(tempC_x100) * 10;
  if (candidate < std::numeric_limits<int32_t>::min() ||
      candidate > std::numeric_limits<int32_t>::max()) {
    return Status::Error(Err::COMPENSATION_ERROR);
  }
  outMilliC = static_cast<int32_t>(candidate);
  return Status::Ok();
}

Status humidityX1024ToMilliPercent(uint32_t humidityPct_x1024,
                                  int32_t& outMilliPercent) {
  static constexpr uint32_t HUMIDITY_100_PERCENT_X1024 = 100U * 1024U;
  if (humidityPct_x1024 > HUMIDITY_100_PERCENT_X1024) {
    return Status::Error(Err::INVALID_PARAM);
  }
  const uint64_t candidate =
      (static_cast<uint64_t>(humidityPct_x1024) * 1000U) / 1024U;
  if (candidate > static_cast<uint64_t>(std::numeric_limits<int32_t>::max())) {
    return Status::Error(Err::COMPENSATION_ERROR);
  }
  outMilliPercent = static_cast<int32_t>(candidate);
  return Status::Ok();
}

void BME280::_resetRuntime(bool preserveHistory) {
  const bool priorHardwareConfigDirty =
      preserveHistory && hardwareConfigDirty();
  const Status priorHardwareConfigDirtyError = preserveHistory
      ? _hardwareConfigDirtyError
      : Status::Ok();
  const uint32_t priorConfigGeneration = preserveHistory
      ? _configGeneration
      : 0;
  const uint32_t priorSampleSequence = preserveHistory
      ? _sampleSequence
      : 0;

  _config = Config{};
  _initialized = false;
  _driverState = DriverState::UNINIT;

  _lastOkMs = 0;
  _lastErrorMs = 0;
  _lastOkTimeValid = false;
  _lastErrorTimeValid = false;
  _lastError = Status::Ok();
  _consecutiveFailures = 0;
  _totalFailures = 0;
  _totalSuccess = 0;
  _timeContextActive = false;
  _timeContextMs = 0;
  _configSyncState = ConfigSyncState::RESYNC_REQUIRED;
  _calibrationState = CalibrationState::INVALID;
  _humidityCalibrationValid = false;
  _hardwareConfigDirtyError = priorHardwareConfigDirty
      ? priorHardwareConfigDirtyError
      : Status::Ok();
  _configGeneration = priorConfigGeneration;

  _measurementRequested = false;
  _measurementReady = false;
  _lastMeasurementStatus = Status::Ok();
  _conversionState = ConversionState::IDLE;
  _hasSample = false;
  _measurementStartMs = 0;
  _measurementDeadlineMs = 0;
  _measurementStatusPolls = 0;
  _sampleTimestampMs = 0;
  _sampleSequence = priorSampleSequence;
  _sampleConfigGeneration = 0;
  _sampleGenerationStale = false;
  _tFine = 0;
  _rawSample = RawSample{};
  _compSample = CompensatedSample{};

  _digT1 = 0;
  _digT2 = 0;
  _digT3 = 0;
  _digP1 = 0;
  _digP2 = 0;
  _digP3 = 0;
  _digP4 = 0;
  _digP5 = 0;
  _digP6 = 0;
  _digP7 = 0;
  _digP8 = 0;
  _digP9 = 0;
  _digH1 = 0;
  _digH2 = 0;
  _digH3 = 0;
  _digH4 = 0;
  _digH5 = 0;
  _digH6 = 0;

  _clearJob();
}

Status BME280::_prepareBeginConfig(const Config& config) {
  _resetRuntime();

  if (config.i2cWrite == nullptr || config.i2cWriteRead == nullptr) {
    return Status::Error(Err::INVALID_CONFIG, "I2C callbacks not set");
  }
  if (config.i2cTimeoutMs == 0) {
    return Status::Error(Err::INVALID_CONFIG, "I2C timeout must be > 0");
  }
  if (config.i2cTimeoutMs >=
      static_cast<uint32_t>(std::numeric_limits<int32_t>::max())) {
    return Status::Error(Err::INVALID_CONFIG, "I2C timeout too large");
  }
  if (config.nvmReadyTimeoutMs == 0) {
    return Status::Error(Err::INVALID_CONFIG, "NVM timeout must be > 0");
  }
  if (config.nvmReadyTimeoutMs >=
      static_cast<uint32_t>(std::numeric_limits<int32_t>::max())) {
    return Status::Error(Err::INVALID_CONFIG, "NVM timeout too large");
  }
  if (config.conversionReadyTimeoutMs == 0) {
    return Status::Error(Err::INVALID_CONFIG,
                         "Conversion ready timeout must be > 0");
  }
  const SensorSettings maximumDurationSettings = {
    Oversampling::X16, Oversampling::X16, Oversampling::X16,
    Filter::OFF, Standby::MS_1000, Mode::NORMAL
  };
  const uint32_t maximumBaseIntervalMs =
      ::BME280::estimateMeasurementTimeMs(maximumDurationSettings) +
      MAX_STANDBY_TIME_MS;
  const uint32_t maximumWrapSafeIntervalMs =
      static_cast<uint32_t>(std::numeric_limits<int32_t>::max());
  if (config.conversionReadyTimeoutMs >
      maximumWrapSafeIntervalMs - maximumBaseIntervalMs) {
    return Status::Error(Err::INVALID_CONFIG,
                         "Conversion ready timeout too large");
  }
  if (config.i2cAddress != 0x76 && config.i2cAddress != 0x77) {
    return Status::Error(Err::INVALID_CONFIG, "Invalid I2C address");
  }
  const SensorSettings settings = {
    config.osrsT, config.osrsP, config.osrsH,
    config.filter, config.standby, config.mode
  };
  const Status settingsStatus = validateSettings(settings);
  if (!settingsStatus.ok()) {
    return Status::Error(Err::INVALID_CONFIG, settingsStatus.detail);
  }

  _config = config;
  if (_config.offlineThreshold == 0) {
    _config.offlineThreshold = 1;
  }

  return Status::Ok();
}

Status BME280::begin(const Config& config) {
  const Status admission = _hardwareOperationAdmission();
  if (!admission.ok()) {
    return admission;
  }
  Status st = _prepareBeginConfig(config);
  if (!st.ok()) {
    return st;
  }

  uint8_t chipId = 0;
  st = _readRegisterRaw(cmd::REG_CHIP_ID, chipId);
  if (!st.ok()) {
    return mapPresenceError(st);
  }
  if (!isBme280ChipId(chipId)) {
    return Status::Error(Err::CHIP_ID_MISMATCH, "Chip ID mismatch", chipId);
  }

  st = _waitForNvmReady(false);
  if (!st.ok()) {
    return mapPresenceError(st);
  }

  Calibration calibrationCandidate;
  bool humidityCalibrationValid = false;
  bool calibrationEvidenceChanged = false;
  st = _readCalibrationCandidate(calibrationCandidate,
                                 humidityCalibrationValid,
                                 calibrationEvidenceChanged);
  if (!st.ok()) {
    return mapPresenceError(st);
  }
  st = _applyConfig();
  if (!st.ok()) {
    return mapPresenceError(st);
  }

  _commitCalibration(calibrationCandidate, humidityCalibrationValid);
  _initialized = true;
  _driverState = DriverState::READY;

  return Status::Ok();
}

void BME280::tick(uint32_t nowMs) {
  ScopedTimeContext timeContext(_timeContextActive, _timeContextMs, nowMs);
  if (!_initialized || !_measurementRequested) {
    return;
  }
  if (_jobActive()) {
    return;
  }
  if (_configSyncState != ConfigSyncState::SYNCHRONIZED ||
      _calibrationState != CalibrationState::VALID) {
    _cancelMeasurementTrackingForStateChange();
    return;
  }

  if (_config.mode == Mode::SLEEP) {
    _measurementRequested = false;
    _lastMeasurementStatus = Status::Error(Err::INVALID_PARAM, "Device is in sleep mode");
    return;
  }

  if (_config.mode == Mode::FORCED || _config.mode == Mode::NORMAL) {
    const uint32_t waitMs = (_config.mode == Mode::NORMAL)
        ? estimateNormalCycleMs()
        : estimateMeasurementTimeMs();
    const uint32_t deadline = _measurementStartMs + waitMs;
    if (!deadlineReached(nowMs, deadline)) {
      return;
    }
  }

  bool measuring = false;
  Status st = isMeasuring(measuring);
  if (!st.ok()) {
    _lastMeasurementStatus = st;
    if (_config.mode == Mode::FORCED &&
        _conversionState != ConversionState::IDLE) {
      _conversionState = ConversionState::UNKNOWN_AFTER_TRIGGER_ERROR;
      _measurementRequested = false;
      _measurementDeadlineMs = 0;
      _measurementStatusPolls = 0;
    }
    return;
  }
  if (measuring) {
    if (_config.mode == Mode::FORCED) {
      _conversionState = ConversionState::IN_PROGRESS;
    }
    if (deadlineReached(nowMs, _measurementDeadlineMs) ||
        _measurementStatusPolls >= MEASURING_READY_MAX_POLLS) {
      _measurementRequested = false;
      _measurementDeadlineMs = 0;
      _measurementStatusPolls = 0;
      _lastMeasurementStatus =
          Status::Error(Err::TIMEOUT, "Measurement ready timeout");
      if (_config.mode == Mode::FORCED) {
        _conversionState = ConversionState::UNKNOWN_AFTER_TRIGGER_ERROR;
      }
      return;
    }
    _measurementStatusPolls++;
    _lastMeasurementStatus = Status::Error(Err::IN_PROGRESS, "Measurement still running");
    return;
  }

  if (_config.mode == Mode::FORCED) {
    _conversionState = ConversionState::IDLE;
  }

  RawSample candidateRaw{};
  CompensatedSample candidateCompensated{};
  int32_t candidateTFine = 0;

  st = _readRawData(candidateRaw);
  if (!st.ok()) {
    _lastMeasurementStatus = st;
    return;
  }

  st = _compensate(candidateRaw, candidateCompensated, candidateTFine);
  if (!st.ok()) {
    _lastMeasurementStatus = st;
    _measurementRequested = false;
    _measurementDeadlineMs = 0;
    _measurementStatusPolls = 0;
    return;
  }

  _commitSample(candidateRaw, candidateCompensated, candidateTFine, nowMs);
  _measurementRequested = false;
  _measurementDeadlineMs = 0;
  _measurementStatusPolls = 0;
  _lastMeasurementStatus = Status::Ok();
}

void BME280::end() {
  _resetRuntime(false);
}

Status BME280::probe() {
  const Status admission = _hardwareOperationAdmission();
  if (!admission.ok()) {
    return admission;
  }
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }

  uint8_t chipId = 0;
  Status st = _readRegisterRaw(cmd::REG_CHIP_ID, chipId);
  if (!st.ok()) {
    return mapPresenceError(st);
  }
  if (!isBme280ChipId(chipId)) {
    return Status::Error(Err::CHIP_ID_MISMATCH, "Chip ID mismatch", chipId);
  }

  return Status::Ok();
}

Status BME280::recover() {
  const Status admission = _hardwareOperationAdmission();
  if (!admission.ok()) {
    return admission;
  }
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }

  Status result = [&]() -> Status {
    uint8_t chipId = 0;
    Status st = readRegister(cmd::REG_CHIP_ID, chipId);
    if (!st.ok()) {
      return st;
    }
    if (!isBme280ChipId(chipId)) {
      const Status mismatch = _recordFailure(
          Status::Error(Err::CHIP_ID_MISMATCH, "Chip ID mismatch", chipId));
      _calibrationState = CalibrationState::INVALID;
      _humidityCalibrationValid = false;
      _markHardwareConfigDirty(mismatch);
      _cancelMeasurementTrackingForStateChange();
      return mismatch;
    }

    st = _waitForNvmReady(true);
    if (!st.ok()) {
      return isTransportFailure(st) ? st : _recordFailure(st);
    }

    Calibration calibrationCandidate;
    bool humidityCalibrationValid = false;
    bool calibrationEvidenceChanged = false;
    st = _readCalibrationCandidate(calibrationCandidate,
                                   humidityCalibrationValid,
                                   calibrationEvidenceChanged);
    if (!st.ok()) {
      if (calibrationEvidenceChanged || st.code == Err::CALIBRATION_INVALID) {
        _calibrationState = CalibrationState::INVALID;
        _humidityCalibrationValid = false;
        _markHardwareConfigDirty(st);
        _cancelMeasurementTrackingForStateChange();
      }
      return st.code == Err::CALIBRATION_INVALID ? _recordFailure(st) : st;
    }
    const bool calibrationChanged = calibrationEvidenceChanged;

    // Re-apply configuration: after a power glitch or external reset the
    // device registers revert to defaults and calibration registers may have
    // just been copied from NVM.
    st = _applyConfig();
    if (!st.ok()) {
      if (calibrationChanged) {
        _calibrationState = CalibrationState::INVALID;
        _humidityCalibrationValid = false;
        _markHardwareConfigDirty(st);
        _cancelMeasurementTrackingForStateChange();
      }
      if (st.code == Err::BUSY) {
        return _recordFailure(st);
      }
      return st;
    }

    _commitCalibration(calibrationCandidate, humidityCalibrationValid);
    return Status::Ok();
  }();
  if (result.ok()) {
    _invalidateSampleCache();
  }
  return result;
}

SensorSettings BME280::sensorSettings() const {
  return SensorSettings{
    _config.osrsT,
    _config.osrsP,
    _config.osrsH,
    _config.filter,
    _config.standby,
    _config.mode
  };
}

void BME280::_setSensorSettings(const SensorSettings& settings) {
  _config.osrsT = settings.osrsT;
  _config.osrsP = settings.osrsP;
  _config.osrsH = settings.osrsH;
  _config.filter = settings.filter;
  _config.standby = settings.standby;
  _config.mode = settings.mode;
}

Status BME280::getSettings(SettingsSnapshot& out) const {
  out.initialized = _initialized;
  out.state = _driverState;
  out.i2cAddress = _config.i2cAddress;
  out.i2cTimeoutMs = _config.i2cTimeoutMs;
  out.nvmReadyTimeoutMs = _config.nvmReadyTimeoutMs;
  out.conversionReadyTimeoutMs = _config.conversionReadyTimeoutMs;
  out.offlineThreshold = _config.offlineThreshold;
  out.hasNowMsHook = (_config.nowMs != nullptr);
  const SensorSettings settings = sensorSettings();
  out.mode = settings.mode;
  out.osrsT = settings.osrsT;
  out.osrsP = settings.osrsP;
  out.osrsH = settings.osrsH;
  out.filter = settings.filter;
  out.standby = settings.standby;
  out.measurementRequested = _measurementRequested;
  out.measurementReady = _measurementReady;
  out.conversionState = _conversionState;
  out.lastMeasurementStatus = _lastMeasurementStatus;
  out.hasSample = _hasSample;
  out.sampleFreshness = sampleFreshness();
  out.configSyncState = _configSyncState;
  out.calibrationState = _calibrationState;
  out.hardwareConfigDirty = hardwareConfigDirty();
  out.hardwareConfigDirtyError = _hardwareConfigDirtyError;
  out.configGeneration = _configGeneration;
  out.sampleSequence = _hasSample ? _sampleSequence : 0;
  out.sampleConfigGeneration = _hasSample ? _sampleConfigGeneration : 0;
  out.measurementStartMs = _measurementStartMs;
  out.sampleTimestampMs = _sampleTimestampMs;
  out.tFine = _tFine;
  out.rawSample = _rawSample;
  out.compSample = _compSample;
  out.sample.rawSample = _rawSample;
  out.sample.compensatedSample = _compSample;
  out.sample.tFine = _tFine;
  out.sample.timestampMs = _sampleTimestampMs;
  out.sample.sampleSequence = _hasSample ? _sampleSequence : 0;
  out.sample.configGeneration = _hasSample ? _sampleConfigGeneration : 0;
  out.calibration.digT1 = _digT1;
  out.calibration.digT2 = _digT2;
  out.calibration.digT3 = _digT3;
  out.calibration.digP1 = _digP1;
  out.calibration.digP2 = _digP2;
  out.calibration.digP3 = _digP3;
  out.calibration.digP4 = _digP4;
  out.calibration.digP5 = _digP5;
  out.calibration.digP6 = _digP6;
  out.calibration.digP7 = _digP7;
  out.calibration.digP8 = _digP8;
  out.calibration.digP9 = _digP9;
  out.calibration.digH1 = _digH1;
  out.calibration.digH2 = _digH2;
  out.calibration.digH3 = _digH3;
  out.calibration.digH4 = _digH4;
  out.calibration.digH5 = _digH5;
  out.calibration.digH6 = _digH6;
  out.lastOkTimeValid = _lastOkTimeValid;
  out.lastErrorTimeValid = _lastErrorTimeValid;
  return Status::Ok();
}

SampleFreshness BME280::sampleFreshness() const {
  if (!_hasSample) {
    return SampleFreshness::NONE;
  }
  if (_configSyncState != ConfigSyncState::SYNCHRONIZED ||
      _calibrationState != CalibrationState::VALID) {
    return SampleFreshness::STALE_AFTER_CONFIG_DIRTY;
  }
  if (_sampleGenerationStale ||
      _sampleConfigGeneration != _configGeneration) {
    return SampleFreshness::STALE_AFTER_CONFIG_CHANGE;
  }
  if (!_lastMeasurementStatus.ok()) {
    return SampleFreshness::STALE_AFTER_ERROR;
  }
  return SampleFreshness::FRESH;
}

bool BME280::sampleFresh(uint32_t nowMs, uint32_t maxAgeMs) const {
  return sampleFreshness() == SampleFreshness::FRESH &&
         sampleAgeMs(nowMs) <= maxAgeMs;
}

bool BME280::_jobActive() const {
  return _jobKind != JobKind::NONE &&
         (_jobState == JobState::RUNNING || _jobState == JobState::WAITING);
}

Status BME280::_jobStartAdmission() const {
  if (_jobActive()) {
    return Status::Error(
        Err::BUSY, "Staged job already active",
        static_cast<int32_t>(BusyReason::STAGED_JOB_ACTIVE));
  }
  if (_jobTerminalResultPending) {
    return Status::Error(
        Err::BUSY, "Terminal job result pending",
        static_cast<int32_t>(BusyReason::TERMINAL_RESULT_PENDING));
  }
  return Status::Ok();
}

Status BME280::_hardwareOperationAdmission() const {
  return _jobStartAdmission();
}

void BME280::_clearJob() {
  _jobKind = JobKind::NONE;
  _jobState = JobState::IDLE;
  _jobPhase = JobPhase::NONE;
  _jobStatus = Status::Ok();
  _jobId = 0;
  _jobTerminalResultPending = false;
  _jobDeadlineMs = 0;
  _jobDeadlineActive = false;
  _jobNvmPolls = 0;
  _jobWaitPolls = 0;
  _jobHardwareConfigTouched = false;
  _jobResetMayHaveReached = false;
  _jobForcedTriggerMayHaveReached = false;
  _jobSettingsStaged = false;
  _jobPriorSettings = SensorSettings{};
  _jobCalibration = Calibration{};
  _jobHumidityCalibrationValid = false;
  _jobCalibrationChanged = false;
  _jobDeviceIdentityMismatch = false;
  _jobPriorConfigSyncState = ConfigSyncState::RESYNC_REQUIRED;
  _jobPriorHardwareConfigDirtyError = Status::Ok();
  _jobRawSample = RawSample{};
  _jobCompSample = CompensatedSample{};
  _jobTFine = 0;
}

Status BME280::_startJob(JobKind kind, JobPhase phase) {
  const Status admission = _jobStartAdmission();
  if (!admission.ok()) {
    return admission;
  }

  ++_nextJobId;
  if (_nextJobId == 0) {
    ++_nextJobId;
  }
  _jobPriorConfigSyncState = _configSyncState;
  _jobPriorHardwareConfigDirtyError = _hardwareConfigDirtyError;
  _jobId = _nextJobId;
  _jobKind = kind;
  _jobState = JobState::RUNNING;
  _jobPhase = phase;
  _jobStatus = Status::Error(Err::IN_PROGRESS, "Job in progress");
  _jobTerminalResultPending = false;
  _jobDeadlineMs = 0;
  _jobDeadlineActive = false;
  _jobNvmPolls = 0;
  _jobWaitPolls = 0;
  _jobHardwareConfigTouched = false;
  _jobResetMayHaveReached = false;
  _jobForcedTriggerMayHaveReached = false;
  _jobSettingsStaged = false;
  _jobPriorSettings = SensorSettings{};
  _jobCalibration = Calibration{};
  _jobHumidityCalibrationValid = false;
  _jobCalibrationChanged = false;
  _jobDeviceIdentityMismatch = false;
  _jobRawSample = RawSample{};
  _jobCompSample = CompensatedSample{};
  _jobTFine = 0;
  return Status::Error(Err::IN_PROGRESS, "Job started");
}

JobPollResult BME280::_idleJobResult() const {
  JobPollResult result;
  result.conversionState = _conversionState;
  return result;
}

JobPollResult BME280::_jobResult(uint8_t instructionsUsed) const {
  JobPollResult result;
  result.jobId = _jobId;
  result.kind = _jobKind;
  result.phase = _jobPhase;
  result.state = _jobState;
  result.status = _jobStatus;
  result.conversionState = _conversionState;
  result.phaseDeadlineActive = _jobDeadlineActive;
  result.phaseDeadlineMs = _jobDeadlineActive ? _jobDeadlineMs : 0;
  result.callbacksUsed = instructionsUsed;
  result.instructionsUsed = instructionsUsed;
  return result;
}

JobPollResult BME280::_failJob(const Status& st, uint8_t instructionsUsed) {
  Status finalStatus = st;
  const bool configJob = _jobKind == JobKind::INIT ||
                         _jobKind == JobKind::APPLY_CONFIG ||
                         _jobKind == JobKind::RESYNC ||
                         _jobKind == JobKind::SOFT_RESET;
  if ((_jobKind == JobKind::RESYNC ||
       _jobKind == JobKind::SOFT_RESET) &&
      !st.ok() && !st.inProgress() &&
      !isTransportFailure(st)) {
    finalStatus = _recordFailure(st);
  }
  const bool changedDeviceStateObserved =
      _jobKind == JobKind::RESYNC &&
      (_jobCalibrationChanged || _jobDeviceIdentityMismatch);
  if (_jobResetMayHaveReached || changedDeviceStateObserved) {
    _calibrationState = CalibrationState::INVALID;
    _humidityCalibrationValid = false;
  }
  if ((_jobHardwareConfigTouched || changedDeviceStateObserved) &&
      !finalStatus.ok() && !finalStatus.inProgress()) {
    _markHardwareConfigDirty(finalStatus);
  } else {
    if (_jobSettingsStaged) {
      _setSensorSettings(_jobPriorSettings);
    }
    if (configJob &&
        _configSyncState == ConfigSyncState::UPDATE_IN_PROGRESS) {
      _configSyncState = _jobPriorConfigSyncState;
      _hardwareConfigDirtyError = _jobPriorHardwareConfigDirtyError;
    }
  }
  if (_jobKind == JobKind::FORCED_MEASUREMENT &&
      !finalStatus.ok() && !finalStatus.inProgress()) {
    _measurementRequested = false;
    _measurementReady = false;
    _measurementDeadlineMs = 0;
    _measurementStatusPolls = 0;
    _lastMeasurementStatus = finalStatus;
  }
  _jobState = JobState::FAILED;
  _jobStatus = finalStatus;
  _jobDeadlineActive = false;
  _jobTerminalResultPending = false;
  return _jobResult(instructionsUsed);
}

JobPollResult BME280::_completeJob(uint8_t instructionsUsed) {
  _jobState = JobState::DONE;
  _jobStatus = Status::Ok();
  _jobPhase = JobPhase::COMPLETE;
  _jobDeadlineActive = false;
  _jobTerminalResultPending = false;
  _jobNvmPolls = 0;
  _jobWaitPolls = 0;
  _jobHardwareConfigTouched = false;
  return _jobResult(instructionsUsed);
}

void BME280::_trackJobConfigWriteResult(const Status& st) {
  if (st.ok() || mayHaveReachedDeviceAfterDiagnosticWrite(st)) {
    _jobHardwareConfigTouched = true;
  }
  if (mayHaveReachedDeviceAfterDiagnosticWrite(st)) {
    _markHardwareConfigDirty(st);
  }
}

Status BME280::startInitJob(const Config& config) {
  const Status admission = _jobStartAdmission();
  if (!admission.ok()) {
    return admission;
  }
  Status st = _prepareBeginConfig(config);
  if (!st.ok()) {
    return st;
  }
  st = _startJob(JobKind::INIT, JobPhase::INIT_READ_CHIP_ID);
  if (st.inProgress()) {
    _configSyncState = ConfigSyncState::UPDATE_IN_PROGRESS;
  }
  return st;
}

Status BME280::startForcedMeasurementJob() {
  const Status admission = _jobStartAdmission();
  if (!admission.ok()) {
    return admission;
  }
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  if (_configSyncState != ConfigSyncState::SYNCHRONIZED ||
      _calibrationState != CalibrationState::VALID) {
    const Status st = Status::Error(Err::RESYNC_REQUIRED,
                                    "Device state requires resynchronization");
    _lastMeasurementStatus = st;
    return st;
  }
  if (_config.mode != Mode::FORCED) {
    return Status::Error(Err::INVALID_PARAM, "Device is not in forced mode");
  }
  if (_measurementRequested && !_measurementReady) {
    return Status::Error(
        Err::BUSY, static_cast<int32_t>(BusyReason::MEASUREMENT_ACTIVE));
  }

  const JobPhase firstPhase = (_conversionState == ConversionState::IDLE)
      ? JobPhase::FORCE_TRIGGER
      : JobPhase::FORCE_RECONCILE_STATUS;
  Status st = _startJob(JobKind::FORCED_MEASUREMENT, firstPhase);
  if (st.inProgress()) {
    _measurementReady = false;
    _lastMeasurementStatus = Status::Error(Err::IN_PROGRESS,
                                           "Measurement job started");
  }
  return st;
}

Status BME280::startApplyConfigJob() {
  const Status admission = _jobStartAdmission();
  if (!admission.ok()) {
    return admission;
  }
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  Status st = _startJob(JobKind::APPLY_CONFIG, JobPhase::APPLY_WAIT_IDLE);
  if (st.inProgress()) {
    _cancelMeasurementTrackingForStateChange();
    _configSyncState = ConfigSyncState::UPDATE_IN_PROGRESS;
  }
  return st;
}

Status BME280::startApplySettingsJob(const SensorSettings& settings) {
  const Status admission = _jobStartAdmission();
  if (!admission.ok()) {
    return admission;
  }
  const Status validation = validateSettings(settings);
  if (!validation.ok()) {
    return validation;
  }
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED);
  }
  if (settings.osrsH != Oversampling::SKIP &&
      !_humidityCalibrationValid) {
    return Status::Error(Err::CALIBRATION_INVALID);
  }

  const SensorSettings priorSettings = sensorSettings();
  Status st = _startJob(JobKind::APPLY_CONFIG, JobPhase::APPLY_WAIT_IDLE);
  if (st.inProgress()) {
    _jobSettingsStaged = true;
    _jobPriorSettings = priorSettings;
    _cancelMeasurementTrackingForStateChange();
    _setSensorSettings(settings);
    _configSyncState = ConfigSyncState::UPDATE_IN_PROGRESS;
  }
  return st;
}

Status BME280::startResyncJob() {
  const Status admission = _jobStartAdmission();
  if (!admission.ok()) {
    return admission;
  }
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }

  Status st = _startJob(JobKind::RESYNC, JobPhase::RESYNC_READ_CHIP_ID);
  if (st.inProgress()) {
    _configSyncState = ConfigSyncState::UPDATE_IN_PROGRESS;
    _measurementRequested = false;
    _measurementReady = false;
    _measurementStartMs = 0;
    _measurementDeadlineMs = 0;
    _measurementStatusPolls = 0;
    if (_conversionState == ConversionState::IN_PROGRESS) {
      _conversionState = ConversionState::UNKNOWN_AFTER_TRIGGER_ERROR;
    }
  }
  return st;
}

Status BME280::startRecoveryJob() {
  return startResyncJob();
}

Status BME280::startSoftResetJob() {
  const Status admission = _jobStartAdmission();
  if (!admission.ok()) {
    return admission;
  }
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }

  Status st = _startJob(JobKind::SOFT_RESET, JobPhase::SOFT_RESET_WRITE);
  if (st.inProgress()) {
    _configSyncState = ConfigSyncState::UPDATE_IN_PROGRESS;
    _measurementRequested = false;
    _measurementReady = false;
    _measurementStartMs = 0;
    _measurementDeadlineMs = 0;
    _measurementStatusPolls = 0;
    if (_conversionState == ConversionState::IN_PROGRESS) {
      _conversionState = ConversionState::UNKNOWN_AFTER_TRIGGER_ERROR;
    }
  }
  return st;
}

Status BME280::cancelJob(CancelReason reason) {
  if (!_jobActive()) {
    return Status::Error(Err::INVALID_PARAM, "No active job to cancel");
  }

  Status cancellation;
  JobState terminalState = JobState::CANCELLED;
  switch (reason) {
    case CancelReason::OWNER_REQUEST:
      cancellation = Status::Error(Err::CANCELLED, "Job cancelled by owner");
      break;
    case CancelReason::DEADLINE_EXPIRED:
      cancellation = Status::Error(Err::DEADLINE_EXPIRED,
                                   "Job owner deadline expired");
      terminalState = JobState::TIMED_OUT;
      break;
    default:
      return Status::Error(Err::INVALID_PARAM, "Invalid cancellation reason");
  }

  const bool configJob = _jobKind == JobKind::INIT ||
                         _jobKind == JobKind::APPLY_CONFIG ||
                         _jobKind == JobKind::RESYNC ||
                         _jobKind == JobKind::SOFT_RESET;
  const bool changedDeviceStateObserved =
      _jobKind == JobKind::RESYNC &&
      (_jobCalibrationChanged || _jobDeviceIdentityMismatch);
  if (_jobResetMayHaveReached || changedDeviceStateObserved) {
    _calibrationState = CalibrationState::INVALID;
    _humidityCalibrationValid = false;
    _markHardwareConfigDirty(cancellation);
  } else if (_jobHardwareConfigTouched) {
    _markHardwareConfigDirty(cancellation);
  } else {
    if (_jobSettingsStaged) {
      _setSensorSettings(_jobPriorSettings);
    }
    if (configJob &&
        _configSyncState == ConfigSyncState::UPDATE_IN_PROGRESS) {
      _configSyncState = _jobPriorConfigSyncState;
      _hardwareConfigDirtyError = _jobPriorHardwareConfigDirtyError;
    }
  }

  if (_jobKind == JobKind::FORCED_MEASUREMENT &&
      (_jobForcedTriggerMayHaveReached ||
       _conversionState == ConversionState::IN_PROGRESS)) {
    _conversionState = ConversionState::UNKNOWN_AFTER_TRIGGER_ERROR;
  }
  if (_jobKind == JobKind::FORCED_MEASUREMENT) {
    _lastMeasurementStatus = cancellation;
  }

  _measurementRequested = false;
  _measurementReady = false;
  _measurementStartMs = 0;
  _measurementDeadlineMs = 0;
  _measurementStatusPolls = 0;
  _jobRawSample = RawSample{};
  _jobCompSample = CompensatedSample{};
  _jobTFine = 0;
  _jobCalibration = Calibration{};
  _jobHumidityCalibrationValid = false;
  _jobCalibrationChanged = false;
  _jobDeviceIdentityMismatch = false;
  _jobState = terminalState;
  _jobStatus = cancellation;
  _jobDeadlineMs = 0;
  _jobDeadlineActive = false;
  _jobNvmPolls = 0;
  _jobWaitPolls = 0;
  _jobTerminalResultPending = true;
  return cancellation;
}

JobPollResult BME280::pollJob(uint32_t nowMs, uint8_t maxInstructions) {
  ScopedTimeContext timeContext(_timeContextActive, _timeContextMs, nowMs);
  if (_jobTerminalResultPending) {
    const JobPollResult terminal = _jobResult(0);
    _jobTerminalResultPending = false;
    return terminal;
  }
  if (!_jobActive()) {
    return _idleJobResult();
  }

  uint8_t instructionsUsed = 0;
  for (uint8_t guard = 0; guard < 32; ++guard) {
    _jobState = JobState::RUNNING;
    _jobStatus = Status::Error(Err::IN_PROGRESS, "Job in progress");

    switch (_jobPhase) {
      case JobPhase::INIT_READ_CHIP_ID: {
        if (instructionsUsed >= maxInstructions) {
          return _jobResult(instructionsUsed);
        }
        uint8_t chipId = 0;
        const Status st = _readRegisterRaw(cmd::REG_CHIP_ID, chipId);
        ++instructionsUsed;
        if (!st.ok()) {
          return _failJob(mapPresenceError(st), instructionsUsed);
        }
        if (!isBme280ChipId(chipId)) {
          return _failJob(
              Status::Error(Err::CHIP_ID_MISMATCH, "Chip ID mismatch", chipId),
              instructionsUsed);
        }
        _jobPhase = JobPhase::INIT_NVM_START;
        break;
      }

      case JobPhase::INIT_NVM_START:
        _jobDeadlineMs = nowMs + _config.nvmReadyTimeoutMs;
        _jobDeadlineActive = true;
        _jobNvmPolls = 0;
        _jobPhase = JobPhase::NVM_POLL;
        break;

      case JobPhase::RESYNC_NVM_START:
        _jobDeadlineMs = nowMs + _config.nvmReadyTimeoutMs;
        _jobDeadlineActive = true;
        _jobNvmPolls = 0;
        _jobPhase = JobPhase::NVM_POLL;
        break;

      case JobPhase::NVM_POLL: {
        if (_jobNvmPolls >= NVM_READY_MAX_POLLS) {
          return _failJob(
              Status::Error(Err::TIMEOUT, "NVM ready polling limit reached",
                            NVM_READY_MAX_POLLS),
              instructionsUsed);
        }
        if (deadlineReached(nowMs, _jobDeadlineMs)) {
          return _failJob(Status::Error(Err::TIMEOUT, "NVM ready timeout",
                                        static_cast<int32_t>(_config.nvmReadyTimeoutMs)),
                          instructionsUsed);
        }
        if (instructionsUsed >= maxInstructions) {
          return _jobResult(instructionsUsed);
        }

        uint8_t status = 0;
        const Status st = (_jobKind == JobKind::INIT)
            ? _readRegisterRaw(cmd::REG_STATUS, status)
            : readRegs(cmd::REG_STATUS, &status, 1);
        ++instructionsUsed;
        ++_jobNvmPolls;
        if (!st.ok()) {
          return _failJob(st, instructionsUsed);
        }
        if ((status & cmd::MASK_STATUS_IM_UPDATE) != 0) {
          _jobState = JobState::WAITING;
          _jobStatus = Status::Error(Err::IN_PROGRESS, "NVM update in progress");
          return _jobResult(instructionsUsed);
        }
        _jobDeadlineActive = false;
        _jobPhase = JobPhase::CALIB_TP;
        break;
      }

      case JobPhase::CALIB_TP: {
        if (instructionsUsed >= maxInstructions) {
          return _jobResult(instructionsUsed);
        }
        const Status st = _readCalibrationTp();
        ++instructionsUsed;
        if (!st.ok()) {
          return _failJob(st, instructionsUsed);
        }
        if (_jobKind == JobKind::RESYNC &&
            !_calibrationTpMatchesCommitted(_jobCalibration)) {
          _jobCalibrationChanged = true;
        }
        _jobPhase = JobPhase::CALIB_H;
        break;
      }

      case JobPhase::CALIB_H: {
        if (instructionsUsed >= maxInstructions) {
          return _jobResult(instructionsUsed);
        }
        const Status st = _readCalibrationH();
        ++instructionsUsed;
        if (!st.ok()) {
          if (_jobKind == JobKind::RESYNC &&
              st.code == Err::CALIBRATION_INVALID) {
            _jobCalibrationChanged = true;
          }
          return _failJob(st, instructionsUsed);
        }
        _jobPhase = JobPhase::VALIDATE_CALIBRATION;
        break;
      }

      case JobPhase::VALIDATE_CALIBRATION: {
        const Status st = _validateCalibrationValues(_jobCalibration.digT1,
                                                     _jobCalibration.digP1);
        if (!st.ok()) {
          if (_jobKind == JobKind::RESYNC) {
            _jobCalibrationChanged = true;
          }
          return _failJob(st, instructionsUsed);
        }
        _jobCalibrationChanged = _jobCalibrationChanged ||
            !_calibrationMatchesCommitted(
                _jobCalibration, _jobHumidityCalibrationValid);
        _jobPhase = JobPhase::APPLY_CTRL_MEAS_SLEEP;
        break;
      }

      case JobPhase::APPLY_WAIT_IDLE: {
        if (instructionsUsed >= maxInstructions) {
          return _jobResult(instructionsUsed);
        }
        uint8_t status = 0;
        const Status st = readRegs(cmd::REG_STATUS, &status, 1);
        ++instructionsUsed;
        if (!st.ok()) {
          return _failJob(st, instructionsUsed);
        }
        if ((status & cmd::MASK_STATUS_MEASURING) != 0) {
          if (_jobWaitPolls == 0) {
            uint32_t waitEstimateMs = estimateMeasurementTimeMs();
            if (_jobSettingsStaged) {
              const uint32_t priorEstimateMs =
                  ::BME280::estimateMeasurementTimeMs(_jobPriorSettings);
              if (priorEstimateMs > waitEstimateMs) {
                waitEstimateMs = priorEstimateMs;
              }
            }
            _jobDeadlineMs = nowMs + waitEstimateMs +
                             _config.conversionReadyTimeoutMs;
            _jobDeadlineActive = true;
          }
          if (deadlineReached(nowMs, _jobDeadlineMs) ||
              _jobWaitPolls >= MEASURING_READY_MAX_POLLS) {
            return _failJob(Status::Error(Err::TIMEOUT,
                                          "Measurement idle wait timeout"),
                            instructionsUsed);
          }
          ++_jobWaitPolls;
          _jobState = JobState::WAITING;
          _jobStatus = Status::Error(Err::IN_PROGRESS, "Measurement in progress");
          return _jobResult(instructionsUsed);
        }
        _jobWaitPolls = 0;
        _jobDeadlineActive = false;
        _jobPhase = JobPhase::APPLY_CTRL_MEAS_SLEEP;
        break;
      }

      case JobPhase::APPLY_CTRL_MEAS_SLEEP: {
        if (instructionsUsed >= maxInstructions) {
          return _jobResult(instructionsUsed);
        }
        const uint8_t value = buildCtrlMeas(_config.osrsT, _config.osrsP,
                                            Mode::SLEEP);
        const Status st = writeRegs(cmd::REG_CTRL_MEAS, &value, 1);
        ++instructionsUsed;
        _trackJobConfigWriteResult(st);
        if (!st.ok()) {
          return _failJob(st, instructionsUsed);
        }
        _jobPhase = JobPhase::APPLY_WAIT_AFTER_SLEEP;
        break;
      }

      case JobPhase::APPLY_WAIT_AFTER_SLEEP: {
        if (instructionsUsed >= maxInstructions) {
          return _jobResult(instructionsUsed);
        }
        uint8_t status = 0;
        const Status st = readRegs(cmd::REG_STATUS, &status, 1);
        ++instructionsUsed;
        if (!st.ok()) {
          return _failJob(st, instructionsUsed);
        }
        if ((status & cmd::MASK_STATUS_MEASURING) != 0) {
          if (_jobWaitPolls == 0) {
            uint32_t waitEstimateMs = estimateMeasurementTimeMs();
            if (_jobSettingsStaged) {
              const uint32_t priorEstimateMs =
                  ::BME280::estimateMeasurementTimeMs(_jobPriorSettings);
              if (priorEstimateMs > waitEstimateMs) {
                waitEstimateMs = priorEstimateMs;
              }
            }
            _jobDeadlineMs = nowMs + waitEstimateMs +
                             _config.conversionReadyTimeoutMs;
            _jobDeadlineActive = true;
          }
          if (deadlineReached(nowMs, _jobDeadlineMs) ||
              _jobWaitPolls >= MEASURING_READY_MAX_POLLS) {
            return _failJob(Status::Error(Err::TIMEOUT,
                                          "Measurement sleep wait timeout"),
                            instructionsUsed);
          }
          ++_jobWaitPolls;
          _markHardwareConfigDirty(
              Status::Error(Err::BUSY,
                            static_cast<int32_t>(BusyReason::DEVICE_MEASURING)));
          _jobState = JobState::WAITING;
          _jobStatus = Status::Error(Err::IN_PROGRESS, "Measurement in progress");
          return _jobResult(instructionsUsed);
        }
        _jobWaitPolls = 0;
        _jobDeadlineActive = false;
        _jobPhase = JobPhase::APPLY_CONFIG;
        break;
      }

      case JobPhase::APPLY_CONFIG: {
        if (instructionsUsed >= maxInstructions) {
          return _jobResult(instructionsUsed);
        }
        const uint8_t value = buildConfig(_config.standby, _config.filter);
        const Status st = writeRegs(cmd::REG_CONFIG, &value, 1);
        ++instructionsUsed;
        _trackJobConfigWriteResult(st);
        if (!st.ok()) {
          return _failJob(st, instructionsUsed);
        }
        _jobPhase = JobPhase::APPLY_CTRL_HUM;
        break;
      }

      case JobPhase::APPLY_CTRL_HUM: {
        if (instructionsUsed >= maxInstructions) {
          return _jobResult(instructionsUsed);
        }
        const uint8_t value = buildCtrlHum(_config.osrsH);
        const Status st = writeRegs(cmd::REG_CTRL_HUM, &value, 1);
        ++instructionsUsed;
        _trackJobConfigWriteResult(st);
        if (!st.ok()) {
          return _failJob(st, instructionsUsed);
        }
        _jobPhase = JobPhase::APPLY_CTRL_MEAS;
        break;
      }

      case JobPhase::APPLY_CTRL_MEAS: {
        if (instructionsUsed >= maxInstructions) {
          return _jobResult(instructionsUsed);
        }
        const uint8_t value = buildCtrlMeas(_config.osrsT, _config.osrsP,
                                            registerModeForConfig(_config.mode));
        const Status st = writeRegs(cmd::REG_CTRL_MEAS, &value, 1);
        ++instructionsUsed;
        _trackJobConfigWriteResult(st);
        if (!st.ok()) {
          return _failJob(st, instructionsUsed);
        }
        _jobPhase = JobPhase::COMPLETE;
        break;
      }

      case JobPhase::FORCE_RECONCILE_STATUS: {
        if (instructionsUsed >= maxInstructions) {
          return _jobResult(instructionsUsed);
        }
        uint8_t status = 0;
        const Status st = readRegs(cmd::REG_STATUS, &status, 1);
        ++instructionsUsed;
        if (!st.ok()) {
          _conversionState = ConversionState::UNKNOWN_AFTER_TRIGGER_ERROR;
          return _failJob(st, instructionsUsed);
        }
        if ((status & cmd::MASK_STATUS_MEASURING) != 0) {
          _conversionState = ConversionState::IN_PROGRESS;
          if (!_jobDeadlineActive) {
            _jobDeadlineMs = nowMs + _config.conversionReadyTimeoutMs;
            _jobDeadlineActive = true;
            _jobWaitPolls = 0;
          }
          if (deadlineReached(nowMs, _jobDeadlineMs) ||
              _jobWaitPolls >= MEASURING_READY_MAX_POLLS) {
            _conversionState = ConversionState::UNKNOWN_AFTER_TRIGGER_ERROR;
            return _failJob(Status::Error(Err::TIMEOUT,
                                          "Conversion reconcile timeout"),
                            instructionsUsed);
          }
          ++_jobWaitPolls;
          _jobState = JobState::WAITING;
          _jobStatus = Status::Error(Err::IN_PROGRESS,
                                     "Existing conversion in progress");
          _lastMeasurementStatus = _jobStatus;
          return _jobResult(instructionsUsed);
        }

        _conversionState = ConversionState::IDLE;
        _jobForcedTriggerMayHaveReached = false;
        _jobDeadlineActive = false;
        _jobWaitPolls = 0;
        _jobPhase = JobPhase::FORCE_TRIGGER;
        break;
      }

      case JobPhase::FORCE_TRIGGER: {
        if (instructionsUsed >= maxInstructions) {
          return _jobResult(instructionsUsed);
        }
        const uint8_t value = buildCtrlMeas(_config.osrsT, _config.osrsP,
                                            Mode::FORCED);
        const Status st = writeRegs(cmd::REG_CTRL_MEAS, &value, 1);
        ++instructionsUsed;
        if (!st.ok()) {
          if (mayHaveReachedDeviceAfterDiagnosticWrite(st)) {
            _jobForcedTriggerMayHaveReached = true;
            _conversionState = ConversionState::UNKNOWN_AFTER_TRIGGER_ERROR;
          } else {
            _conversionState = ConversionState::IDLE;
          }
          return _failJob(st, instructionsUsed);
        }
        _jobForcedTriggerMayHaveReached = true;
        _conversionState = ConversionState::IN_PROGRESS;
        _measurementRequested = true;
        _measurementReady = false;
        _measurementStartMs = nowMs;
        _measurementDeadlineMs = nowMs + estimateMeasurementTimeMs() +
                                 _config.conversionReadyTimeoutMs;
        _measurementStatusPolls = 0;
        _jobDeadlineMs = nowMs + estimateMeasurementTimeMs();
        _jobDeadlineActive = true;
        _jobPhase = JobPhase::FORCE_WAIT_TIME;
        break;
      }

      case JobPhase::FORCE_WAIT_TIME:
        if (!deadlineReached(nowMs, _jobDeadlineMs)) {
          _jobState = JobState::WAITING;
          _jobStatus = Status::Error(Err::IN_PROGRESS, "Measurement delay active");
          _lastMeasurementStatus = _jobStatus;
          return _jobResult(instructionsUsed);
        }
        _jobDeadlineMs = nowMs + _config.conversionReadyTimeoutMs;
        _jobDeadlineActive = true;
        _jobWaitPolls = 0;
        _jobPhase = JobPhase::FORCE_READ_STATUS;
        break;

      case JobPhase::FORCE_READ_STATUS: {
        if (instructionsUsed >= maxInstructions) {
          return _jobResult(instructionsUsed);
        }
        uint8_t status = 0;
        const Status st = readRegs(cmd::REG_STATUS, &status, 1);
        ++instructionsUsed;
        if (!st.ok()) {
          _conversionState = ConversionState::UNKNOWN_AFTER_TRIGGER_ERROR;
          return _failJob(st, instructionsUsed);
        }
        if ((status & cmd::MASK_STATUS_MEASURING) != 0) {
          _conversionState = ConversionState::IN_PROGRESS;
          if (deadlineReached(nowMs, _jobDeadlineMs) ||
              _jobWaitPolls >= MEASURING_READY_MAX_POLLS) {
            _conversionState = ConversionState::UNKNOWN_AFTER_TRIGGER_ERROR;
            return _failJob(Status::Error(Err::TIMEOUT,
                                          "Measurement ready timeout"),
                            instructionsUsed);
          }
          ++_jobWaitPolls;
          _jobState = JobState::WAITING;
          _jobStatus = Status::Error(Err::IN_PROGRESS, "Measurement in progress");
          _lastMeasurementStatus = _jobStatus;
          return _jobResult(instructionsUsed);
        }
        _conversionState = ConversionState::IDLE;
        _jobForcedTriggerMayHaveReached = false;
        _jobDeadlineActive = false;
        _jobWaitPolls = 0;
        _jobPhase = JobPhase::FORCE_READ_DATA;
        break;
      }

      case JobPhase::FORCE_READ_DATA: {
        if (instructionsUsed >= maxInstructions) {
          return _jobResult(instructionsUsed);
        }
        const Status st = _readRawData(_jobRawSample);
        ++instructionsUsed;
        if (!st.ok()) {
          return _failJob(st, instructionsUsed);
        }
        _jobPhase = JobPhase::FORCE_COMPENSATE;
        break;
      }

      case JobPhase::FORCE_COMPENSATE: {
        const Status st = _compensate(_jobRawSample, _jobCompSample, _jobTFine);
        if (!st.ok()) {
          _measurementRequested = false;
          return _failJob(st, instructionsUsed);
        }
        _commitSample(_jobRawSample, _jobCompSample, _jobTFine, nowMs);
        _measurementRequested = false;
        _measurementDeadlineMs = 0;
        _measurementStatusPolls = 0;
        _lastMeasurementStatus = Status::Ok();
        _conversionState = ConversionState::IDLE;
        _jobForcedTriggerMayHaveReached = false;
        _jobPhase = JobPhase::COMPLETE;
        break;
      }

      case JobPhase::SOFT_RESET_WRITE: {
        if (instructionsUsed >= maxInstructions) {
          return _jobResult(instructionsUsed);
        }
        const uint8_t value = cmd::RESET_VALUE;
        const Status st = writeRegs(cmd::REG_RESET, &value, 1);
        ++instructionsUsed;
        _trackJobConfigWriteResult(st);
        if (st.ok() || mayHaveReachedDeviceAfterDiagnosticWrite(st)) {
          _jobResetMayHaveReached = true;
          _calibrationState = CalibrationState::INVALID;
          _humidityCalibrationValid = false;
        }
        if (!st.ok()) {
          return _failJob(st, instructionsUsed);
        }
        _jobPhase = JobPhase::RESYNC_READ_CHIP_ID;
        break;
      }

      case JobPhase::RESYNC_READ_CHIP_ID: {
        if (instructionsUsed >= maxInstructions) {
          return _jobResult(instructionsUsed);
        }
        uint8_t chipId = 0;
        const Status st = readRegs(cmd::REG_CHIP_ID, &chipId, 1);
        ++instructionsUsed;
        if (!st.ok()) {
          return _failJob(st, instructionsUsed);
        }
        if (!isBme280ChipId(chipId)) {
          if (_jobKind == JobKind::RESYNC) {
            _jobDeviceIdentityMismatch = true;
          }
          return _failJob(
              Status::Error(Err::CHIP_ID_MISMATCH, "Chip ID mismatch", chipId),
              instructionsUsed);
        }
        _jobPhase = JobPhase::RESYNC_NVM_START;
        break;
      }

      case JobPhase::COMPLETE:
        if (_jobKind == JobKind::INIT) {
          _commitCalibration(_jobCalibration, _jobHumidityCalibrationValid);
          _clearHardwareConfigDirty();
          _initialized = true;
          _driverState = DriverState::READY;
        } else if (_jobKind == JobKind::APPLY_CONFIG) {
          _clearHardwareConfigDirty();
          if (_config.mode == Mode::FORCED) {
            _conversionState = ConversionState::IDLE;
          }
        } else if (_jobKind == JobKind::RESYNC ||
                   _jobKind == JobKind::SOFT_RESET) {
          _commitCalibration(_jobCalibration, _jobHumidityCalibrationValid);
          _clearHardwareConfigDirty();
          _driverState = DriverState::READY;
          _consecutiveFailures = 0;
          _conversionState = ConversionState::IDLE;
        }
        return _completeJob(instructionsUsed);

      case JobPhase::NONE:
      default:
        return _failJob(Status::Error(
                            Err::BUSY,
                            static_cast<int32_t>(BusyReason::INVALID_JOB_STATE)),
                        instructionsUsed);
    }
  }

  return _failJob(Status::Error(
                      Err::BUSY,
                      static_cast<int32_t>(BusyReason::JOB_STATE_MACHINE_STALLED)),
                  instructionsUsed);
}

Status BME280::requestMeasurement() {
  const Status admission = _hardwareOperationAdmission();
  if (!admission.ok()) {
    return admission;
  }
  if (!_initialized) {
    Status st = Status::Error(Err::NOT_INITIALIZED, "begin() not called");
    _lastMeasurementStatus = st;
    return st;
  }
  if (_configSyncState != ConfigSyncState::SYNCHRONIZED ||
      _calibrationState != CalibrationState::VALID) {
    Status st = Status::Error(Err::RESYNC_REQUIRED,
                              "Device state requires resynchronization");
    _lastMeasurementStatus = st;
    return st;
  }
  if (_config.mode == Mode::SLEEP) {
    Status st = Status::Error(Err::INVALID_PARAM, "Device is in sleep mode");
    _lastMeasurementStatus = st;
    return st;
  }
  if (_config.nowMs == nullptr) {
    Status st = Status::Error(Err::INVALID_CONFIG, "nowMs required for measurement scheduling");
    _lastMeasurementStatus = st;
    return st;
  }
  if (_measurementRequested && !_measurementReady) {
    Status st = Status::Error(
        Err::BUSY, static_cast<int32_t>(BusyReason::MEASUREMENT_ACTIVE));
    _lastMeasurementStatus = st;
    return st;
  }

  _measurementReady = false;

  if (_config.mode == Mode::FORCED) {
    const bool conversionMayBeActive =
        _conversionState != ConversionState::IDLE;
    bool measuring = false;
    Status st = isMeasuring(measuring);
    if (!st.ok()) {
      if (conversionMayBeActive) {
        _conversionState = ConversionState::UNKNOWN_AFTER_TRIGGER_ERROR;
      }
      _lastMeasurementStatus = st;
      return st;
    }
    if (measuring) {
      // A conversion can still be running after config writes in forced mode.
      // Track completion instead of forcing the caller to re-issue the request.
      _measurementRequested = true;
      const uint32_t nowMs = _nowMs();
      _measurementStartMs = nowMs - estimateMeasurementTimeMs();
      _measurementDeadlineMs = nowMs + _config.conversionReadyTimeoutMs;
      _measurementStatusPolls = 0;
      _conversionState = ConversionState::IN_PROGRESS;
      st = Status::Error(Err::IN_PROGRESS, "Measurement already in progress");
      _lastMeasurementStatus = st;
      return st;
    }

    _conversionState = ConversionState::IDLE;

    const uint8_t ctrlMeas = buildCtrlMeas(_config.osrsT, _config.osrsP, Mode::FORCED);
    st = writeRegs(cmd::REG_CTRL_MEAS, &ctrlMeas, 1);
    if (!st.ok()) {
      if (mayHaveReachedDeviceAfterDiagnosticWrite(st)) {
        _conversionState = ConversionState::UNKNOWN_AFTER_TRIGGER_ERROR;
      }
      _lastMeasurementStatus = st;
      return st;
    }

    _measurementRequested = true;
    _conversionState = ConversionState::IN_PROGRESS;
    _measurementStartMs = _nowMs();
    _measurementDeadlineMs =
        _measurementStartMs + estimateMeasurementTimeMs() +
        _config.conversionReadyTimeoutMs;
    _measurementStatusPolls = 0;

    st = Status::Error(Err::IN_PROGRESS, "Measurement started");
    _lastMeasurementStatus = st;
    return st;
  }

  _measurementRequested = true;
  _measurementStartMs = _nowMs();
  _measurementDeadlineMs =
      _measurementStartMs + estimateNormalCycleMs() +
      _config.conversionReadyTimeoutMs;
  _measurementStatusPolls = 0;
  Status st = Status::Error(Err::IN_PROGRESS, "Measurement scheduled");
  _lastMeasurementStatus = st;
  return st;
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
  out.temperatureValid = _compSample.temperatureValid;
  out.pressureValid = _compSample.pressureValid;
  out.humidityValid = _compSample.humidityValid;

  _measurementReady = false;
  return Status::Ok();
}

Status BME280::getRawSample(RawSample& out) const {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  if (!_hasSample) {
    return Status::Error(Err::MEASUREMENT_NOT_READY, "Measurement not ready");
  }

  out = _rawSample;
  return Status::Ok();
}

Status BME280::getCompensatedSample(CompensatedSample& out) const {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  if (!_hasSample) {
    return Status::Error(Err::MEASUREMENT_NOT_READY, "Measurement not ready");
  }

  out = _compSample;
  return Status::Ok();
}

Status BME280::getSampleEnvelope(SampleEnvelope& out) const {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  if (!_hasSample) {
    return Status::Error(Err::MEASUREMENT_NOT_READY, "Measurement not ready");
  }

  out.rawSample = _rawSample;
  out.compensatedSample = _compSample;
  out.tFine = _tFine;
  out.timestampMs = _sampleTimestampMs;
  out.sampleSequence = _sampleSequence;
  out.configGeneration = _sampleConfigGeneration;
  return Status::Ok();
}

Status BME280::getCalibration(Calibration& out) const {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  if (_calibrationState != CalibrationState::VALID) {
    return Status::Error(Err::RESYNC_REQUIRED,
                         "Calibration must be reloaded");
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

Status BME280::invalidateDeviceState() {
  const Status admission = _jobStartAdmission();
  if (!admission.ok()) {
    return admission;
  }

  const Status invalid = Status::Error(
      Err::RESYNC_REQUIRED, "Device state invalidated by owner");
  _calibrationState = CalibrationState::INVALID;
  _humidityCalibrationValid = false;
  _markHardwareConfigDirty(invalid);
  _measurementRequested = false;
  _measurementReady = false;
  _conversionState = ConversionState::UNKNOWN_AFTER_TRIGGER_ERROR;
  _measurementStartMs = 0;
  _measurementDeadlineMs = 0;
  _measurementStatusPolls = 0;
  _lastMeasurementStatus = invalid;
  return Status::Ok();
}

Status BME280::readCalibrationRaw(CalibrationRaw& out) {
  const Status admission = _hardwareOperationAdmission();
  if (!admission.ok()) {
    return admission;
  }
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }

  Status st = readRegs(cmd::REG_CALIB_TP_START, out.tp, sizeof(out.tp));
  if (!st.ok()) {
    return st;
  }

  return readRegs(cmd::REG_CALIB_H_START, out.h, sizeof(out.h));
}

Status BME280::setMode(Mode mode) {
  const Status admission = _hardwareOperationAdmission();
  if (!admission.ok()) {
    return admission;
  }
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  if (!isValidMode(mode)) {
    return Status::Error(Err::INVALID_PARAM, "Invalid mode");
  }
  if (mode != Mode::SLEEP &&
      !isValidMeasurementSelection(_config.osrsT, _config.osrsP, _config.osrsH)) {
    return Status::Error(Err::INVALID_PARAM, "Invalid oversampling combination");
  }

  const uint8_t ctrlMeas = buildCtrlMeas(_config.osrsT, _config.osrsP,
                                         registerModeForConfig(mode));
  Status st = writeRegs(cmd::REG_CTRL_MEAS, &ctrlMeas, 1);
  if (!st.ok()) {
    if (mayHaveReachedDeviceAfterDiagnosticWrite(st)) {
      _markHardwareConfigDirty(st);
    }
    return st;
  }

  _config.mode = mode;
  _conversionState = ConversionState::IDLE;
  _advanceConfigGeneration();
  _invalidateSampleCache();
  if (mode == Mode::SLEEP) {
    _measurementRequested = false;
    _measurementDeadlineMs = 0;
    _measurementStatusPolls = 0;
  }
  return Status::Ok();
}

Status BME280::getMode(Mode& out) const {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  out = _config.mode;
  return Status::Ok();
}

Status BME280::setOversamplingT(Oversampling osrs) {
  const Status admission = _hardwareOperationAdmission();
  if (!admission.ok()) {
    return admission;
  }
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  if (!isValidOversampling(osrs)) {
    return Status::Error(Err::INVALID_PARAM, "Invalid oversampling");
  }
  if (!isValidMeasurementSelection(osrs, _config.osrsP, _config.osrsH)) {
    return Status::Error(Err::INVALID_PARAM, "Invalid oversampling combination");
  }

  const uint8_t ctrlMeas = buildCtrlMeas(osrs, _config.osrsP,
                                         registerModeForConfig(_config.mode));
  Status st = writeRegs(cmd::REG_CTRL_MEAS, &ctrlMeas, 1);
  if (!st.ok()) {
    if (mayHaveReachedDeviceAfterDiagnosticWrite(st)) {
      _markHardwareConfigDirty(st);
    }
    return st;
  }
  _config.osrsT = osrs;
  _advanceConfigGeneration();
  _invalidateSampleCache();
  return Status::Ok();
}

Status BME280::setOversamplingP(Oversampling osrs) {
  const Status admission = _hardwareOperationAdmission();
  if (!admission.ok()) {
    return admission;
  }
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  if (!isValidOversampling(osrs)) {
    return Status::Error(Err::INVALID_PARAM, "Invalid oversampling");
  }
  if (!isValidMeasurementSelection(_config.osrsT, osrs, _config.osrsH)) {
    return Status::Error(Err::INVALID_PARAM, "Invalid oversampling combination");
  }

  const uint8_t ctrlMeas = buildCtrlMeas(_config.osrsT, osrs,
                                         registerModeForConfig(_config.mode));
  Status st = writeRegs(cmd::REG_CTRL_MEAS, &ctrlMeas, 1);
  if (!st.ok()) {
    if (mayHaveReachedDeviceAfterDiagnosticWrite(st)) {
      _markHardwareConfigDirty(st);
    }
    return st;
  }
  _config.osrsP = osrs;
  _advanceConfigGeneration();
  _invalidateSampleCache();
  return Status::Ok();
}

Status BME280::setOversamplingH(Oversampling osrs) {
  const Status admission = _hardwareOperationAdmission();
  if (!admission.ok()) {
    return admission;
  }
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  if (!isValidOversampling(osrs)) {
    return Status::Error(Err::INVALID_PARAM, "Invalid oversampling");
  }
  if (!isValidMeasurementSelection(_config.osrsT, _config.osrsP, osrs)) {
    return Status::Error(Err::INVALID_PARAM, "Invalid oversampling combination");
  }
  if (osrs != Oversampling::SKIP && !_humidityCalibrationValid) {
    return Status::Error(Err::CALIBRATION_INVALID,
                         "Humidity calibration block invalid");
  }

  const uint8_t ctrlHum = buildCtrlHum(osrs);
  Status st = writeRegs(cmd::REG_CTRL_HUM, &ctrlHum, 1);
  if (!st.ok()) {
    if (mayHaveReachedDeviceAfterDiagnosticWrite(st)) {
      _markHardwareConfigDirty(st);
    }
    return st;
  }

  const uint8_t ctrlMeas = buildCtrlMeas(_config.osrsT, _config.osrsP,
                                         registerModeForConfig(_config.mode));
  st = writeRegs(cmd::REG_CTRL_MEAS, &ctrlMeas, 1);
  if (!st.ok()) {
    // ctrl_hum already reached the device. Even a definite address NACK on
    // this second transaction leaves the cached setting out of sync with the
    // register image and requires a full verified resynchronization.
    _markHardwareConfigDirty(st);
    return st;
  }

  _config.osrsH = osrs;
  _advanceConfigGeneration();
  _invalidateSampleCache();
  return Status::Ok();
}

Status BME280::setFilter(Filter filter) {
  const Status admission = _hardwareOperationAdmission();
  if (!admission.ok()) {
    return admission;
  }
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  if (!isValidFilter(filter)) {
    return Status::Error(Err::INVALID_PARAM, "Invalid filter");
  }

  const uint8_t config = buildConfig(_config.standby, filter);
  const uint8_t ctrlMeasSleep = buildCtrlMeas(_config.osrsT, _config.osrsP, Mode::SLEEP);
  const uint8_t ctrlMeas = buildCtrlMeas(_config.osrsT, _config.osrsP,
                                         registerModeForConfig(_config.mode));

  Status st = _ensureConfigWriteReady();
  if (!st.ok()) {
    return st;
  }

  st = writeRegs(cmd::REG_CTRL_MEAS, &ctrlMeasSleep, 1);
  if (!st.ok()) {
    _markHardwareConfigDirty(st);
    return st;
  }

  st = _ensureConfigWriteReady();
  if (!st.ok()) {
    _markHardwareConfigDirty(st);
    const Status restore = writeRegs(cmd::REG_CTRL_MEAS, &ctrlMeas, 1);
    if (!restore.ok()) {
      _markHardwareConfigDirty(restore);
    }
    return st;
  }

  st = writeRegs(cmd::REG_CONFIG, &config, 1);
  if (!st.ok()) {
    _markHardwareConfigDirty(st);
    // Best-effort restore: always return the original config-write error.
    const Status restore = writeRegs(cmd::REG_CTRL_MEAS, &ctrlMeas, 1);
    (void)restore;
    return st;
  }
  st = writeRegs(cmd::REG_CTRL_MEAS, &ctrlMeas, 1);
  if (!st.ok()) {
    _markHardwareConfigDirty(st);
    return st;
  }

  _config.filter = filter;
  _advanceConfigGeneration();
  _invalidateSampleCache();
  return Status::Ok();
}

Status BME280::setStandby(Standby standby) {
  const Status admission = _hardwareOperationAdmission();
  if (!admission.ok()) {
    return admission;
  }
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  if (!isValidStandby(standby)) {
    return Status::Error(Err::INVALID_PARAM, "Invalid standby");
  }

  const uint8_t config = buildConfig(standby, _config.filter);
  const uint8_t ctrlMeasSleep = buildCtrlMeas(_config.osrsT, _config.osrsP, Mode::SLEEP);
  const uint8_t ctrlMeas = buildCtrlMeas(_config.osrsT, _config.osrsP,
                                         registerModeForConfig(_config.mode));

  Status st = _ensureConfigWriteReady();
  if (!st.ok()) {
    return st;
  }

  st = writeRegs(cmd::REG_CTRL_MEAS, &ctrlMeasSleep, 1);
  if (!st.ok()) {
    _markHardwareConfigDirty(st);
    return st;
  }

  st = _ensureConfigWriteReady();
  if (!st.ok()) {
    _markHardwareConfigDirty(st);
    const Status restore = writeRegs(cmd::REG_CTRL_MEAS, &ctrlMeas, 1);
    if (!restore.ok()) {
      _markHardwareConfigDirty(restore);
    }
    return st;
  }

  st = writeRegs(cmd::REG_CONFIG, &config, 1);
  if (!st.ok()) {
    _markHardwareConfigDirty(st);
    // Best-effort restore: always return the original config-write error.
    const Status restore = writeRegs(cmd::REG_CTRL_MEAS, &ctrlMeas, 1);
    (void)restore;
    return st;
  }
  st = writeRegs(cmd::REG_CTRL_MEAS, &ctrlMeas, 1);
  if (!st.ok()) {
    _markHardwareConfigDirty(st);
    return st;
  }

  _config.standby = standby;
  _advanceConfigGeneration();
  _invalidateSampleCache();
  return Status::Ok();
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
  const Status admission = _hardwareOperationAdmission();
  if (!admission.ok()) {
    return admission;
  }
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }

  Status result = [&]() -> Status {
    _invalidateSampleCache();

    const uint8_t resetValue = cmd::RESET_VALUE;
    Status st = writeRegs(cmd::REG_RESET, &resetValue, 1);
    if (st.ok() || mayHaveReachedDeviceAfterDiagnosticWrite(st)) {
      _calibrationState = CalibrationState::INVALID;
      _humidityCalibrationValid = false;
      _conversionState = ConversionState::UNKNOWN_AFTER_TRIGGER_ERROR;
    }
    if (!st.ok()) {
      if (st.code != Err::I2C_NACK_ADDR && st.code != Err::DEVICE_NOT_FOUND) {
        _markHardwareConfigDirty(st);
      }
      return st;
    }

    st = _waitForNvmReady(true);
    if (!st.ok()) {
      _markHardwareConfigDirty(st);
      return isTransportFailure(st) ? st : _recordFailure(st);
    }

    Calibration calibrationCandidate;
    bool humidityCalibrationValid = false;
    bool calibrationEvidenceChanged = false;
    st = _readCalibrationCandidate(calibrationCandidate,
                                   humidityCalibrationValid,
                                   calibrationEvidenceChanged);
    if (!st.ok()) {
      _markHardwareConfigDirty(st);
      return st.code == Err::CALIBRATION_INVALID ? _recordFailure(st) : st;
    }
    st = _applyConfig();
    if (!st.ok()) {
      _markHardwareConfigDirty(st);
      if (st.code == Err::BUSY) {
        return _recordFailure(st);
      }
      return st;
    }
    _commitCalibration(calibrationCandidate, humidityCalibrationValid);
    _conversionState = ConversionState::IDLE;
    return Status::Ok();
  }();
  return result;
}

Status BME280::readChipId(uint8_t& id) {
  const Status admission = _hardwareOperationAdmission();
  if (!admission.ok()) {
    return admission;
  }
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  return readRegister(cmd::REG_CHIP_ID, id);
}

Status BME280::readStatus(uint8_t& status) {
  const Status admission = _hardwareOperationAdmission();
  if (!admission.ok()) {
    return admission;
  }
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  return readRegister(cmd::REG_STATUS, status);
}

Status BME280::readCtrlHum(uint8_t& value) {
  const Status admission = _hardwareOperationAdmission();
  if (!admission.ok()) {
    return admission;
  }
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  return readRegister(cmd::REG_CTRL_HUM, value);
}

Status BME280::readCtrlMeas(uint8_t& value) {
  const Status admission = _hardwareOperationAdmission();
  if (!admission.ok()) {
    return admission;
  }
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  return readRegister(cmd::REG_CTRL_MEAS, value);
}

Status BME280::readConfig(uint8_t& value) {
  const Status admission = _hardwareOperationAdmission();
  if (!admission.ok()) {
    return admission;
  }
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  return readRegister(cmd::REG_CONFIG, value);
}

Status BME280::isMeasuring(bool& measuring) {
  const Status admission = _hardwareOperationAdmission();
  if (!admission.ok()) {
    return admission;
  }
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

uint32_t BME280::estimateMeasurementTimeUs() const {
  return ::BME280::estimateMeasurementTimeUs(sensorSettings());
}

uint32_t BME280::estimateMeasurementTimeMs() const {
  return ::BME280::estimateMeasurementTimeMs(sensorSettings());
}

uint32_t BME280::getStandbyTimeMs() const {
  switch (_config.standby) {
    case Standby::MS_0_5:  return 1;    // rounded up from 0.5
    case Standby::MS_62_5: return 63;   // rounded up from 62.5
    case Standby::MS_125:  return 125;
    case Standby::MS_250:  return 250;
    case Standby::MS_500:  return 500;
    case Standby::MS_1000: return 1000;
    case Standby::MS_10:   return 10;
    case Standby::MS_20:   return 20;
    default:               return 125;  // safe fallback
  }
}

uint32_t BME280::estimateNormalCycleMs() const {
  return estimateMeasurementTimeMs() + getStandbyTimeMs();
}

Status BME280::_i2cWriteReadRaw(const uint8_t* txBuf, size_t txLen,
                                uint8_t* rxBuf, size_t rxLen) {
  if (txBuf == nullptr || txLen == 0 || (rxLen > 0 && rxBuf == nullptr)) {
    return Status::Error(Err::INVALID_PARAM, "Invalid I2C buffer");
  }
  if (_config.i2cWriteRead == nullptr) {
    return Status::Error(Err::INVALID_CONFIG, "I2C write-read not set");
  }
  const TransportResult result = _config.i2cWriteRead(
      _config.i2cAddress, txBuf, txLen, rxBuf, rxLen,
      _config.i2cTimeoutMs, _config.i2cUser);
  return mapTransportResult(result, txLen, rxLen);
}

Status BME280::_i2cWriteRaw(const uint8_t* buf, size_t len) {
  if (buf == nullptr || len == 0) {
    return Status::Error(Err::INVALID_PARAM, "Invalid I2C buffer");
  }
  if (_config.i2cWrite == nullptr) {
    return Status::Error(Err::INVALID_CONFIG, "I2C write not set");
  }
  const TransportResult result = _config.i2cWrite(
      _config.i2cAddress, buf, len, _config.i2cTimeoutMs, _config.i2cUser);
  return mapTransportResult(result, len, 0);
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

Status BME280::readRegisters(uint8_t startReg, uint8_t* buf, size_t len) {
  const Status admission = _hardwareOperationAdmission();
  if (!admission.ok()) {
    return admission;
  }
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  return readRegs(startReg, buf, len);
}

Status BME280::writeRegisters(uint8_t startReg, const uint8_t* buf, size_t len) {
  const Status admission = _hardwareOperationAdmission();
  if (!admission.ok()) {
    return admission;
  }
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  const bool touchesConfig = touchesDiagnosticConfigRegister(startReg, len);
  const Status st = writeRegs(startReg, buf, len);
  if (touchesConfig) {
    if (st.ok()) {
      _invalidateSampleCache();
      _markHardwareConfigDirty(diagnosticRawWriteDirtyStatus(startReg));
    } else if (mayHaveReachedDeviceAfterDiagnosticWrite(st)) {
      _invalidateSampleCache();
      _markHardwareConfigDirty(st);
    }
  }
  return st;
}

Status BME280::readRegister(uint8_t reg, uint8_t& value) {
  const Status admission = _hardwareOperationAdmission();
  if (!admission.ok()) {
    return admission;
  }
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  return readRegs(reg, &value, 1);
}

Status BME280::writeRegister(uint8_t reg, uint8_t value) {
  const Status admission = _hardwareOperationAdmission();
  if (!admission.ok()) {
    return admission;
  }
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  return writeRegisters(reg, &value, 1);
}

Status BME280::_readRegisterRaw(uint8_t reg, uint8_t& value) {
  uint8_t addr = reg;
  return _i2cWriteReadRaw(&addr, 1, &value, 1);
}

Status BME280::_updateHealth(const Status& st) {
  if (!_initialized) {
    return st;
  }
  if (st.inProgress()) {
    return st;
  }

  const uint32_t now = _nowMs();
  const bool timeValid = _timeValid();
  const uint32_t maxU32 = std::numeric_limits<uint32_t>::max();
  const uint8_t maxU8 = std::numeric_limits<uint8_t>::max();

  if (st.ok()) {
    _lastOkMs = now;
    _lastOkTimeValid = timeValid;
    if (_totalSuccess < maxU32) {
      _totalSuccess++;
    }
    _consecutiveFailures = 0;

    _driverState = DriverState::READY;
    return st;
  }

  _lastError = st;
  _lastErrorMs = now;
  _lastErrorTimeValid = timeValid;
  if (_totalFailures < maxU32) {
    _totalFailures++;
  }
  if (_consecutiveFailures < maxU8) {
    _consecutiveFailures++;
  }

  if (_initialized) {
    if (_consecutiveFailures >= _config.offlineThreshold) {
      _driverState = DriverState::OFFLINE;
    } else {
      _driverState = DriverState::DEGRADED;
    }
  }

  return st;
}

Status BME280::_recordFailure(const Status& st) {
  if (st.ok() || st.inProgress() ||
      st.code == Err::INVALID_CONFIG ||
      st.code == Err::INVALID_PARAM ||
      st.code == Err::NOT_INITIALIZED) {
    return st;
  }

  const uint32_t now = _nowMs();
  const bool timeValid = _timeValid();
  const uint32_t maxU32 = std::numeric_limits<uint32_t>::max();
  const uint8_t maxU8 = std::numeric_limits<uint8_t>::max();

  _lastError = st;
  _lastErrorMs = now;
  _lastErrorTimeValid = timeValid;
  if (_totalFailures < maxU32) {
    _totalFailures++;
  }
  if (_consecutiveFailures < maxU8) {
    _consecutiveFailures++;
  }

  if (_initialized) {
    if (_consecutiveFailures >= _config.offlineThreshold) {
      _driverState = DriverState::OFFLINE;
    } else {
      _driverState = DriverState::DEGRADED;
    }
  }

  return st;
}

void BME280::_markHardwareConfigDirty(const Status& st) {
  if (st.ok() || st.inProgress()) {
    return;
  }
  if (_hardwareConfigDirtyError.ok()) {
    _hardwareConfigDirtyError = st;
  }
  _configSyncState = ConfigSyncState::RESYNC_REQUIRED;
}

void BME280::_clearHardwareConfigDirty() {
  _configSyncState = ConfigSyncState::SYNCHRONIZED;
  _hardwareConfigDirtyError = Status::Ok();
  _advanceConfigGeneration();
}

Status BME280::_ensureConfigWriteReady() {
  uint8_t status = 0;
  const Status st = _initialized
      ? readRegister(cmd::REG_STATUS, status)
      : _readRegisterRaw(cmd::REG_STATUS, status);
  if (!st.ok()) {
    return st;
  }
  if ((status & cmd::MASK_STATUS_MEASURING) != 0) {
    return Status::Error(
        Err::BUSY, static_cast<int32_t>(BusyReason::DEVICE_MEASURING));
  }
  return Status::Ok();
}

Status BME280::_applyConfig() {
  const uint8_t ctrlHum = buildCtrlHum(_config.osrsH);
  const uint8_t ctrlMeasSleep = buildCtrlMeas(_config.osrsT, _config.osrsP, Mode::SLEEP);
  const uint8_t ctrlMeas = buildCtrlMeas(_config.osrsT, _config.osrsP,
                                         registerModeForConfig(_config.mode));
  const uint8_t config = buildConfig(_config.standby, _config.filter);

  Status st = _ensureConfigWriteReady();
  if (!st.ok()) {
    return st;
  }

  st = writeRegs(cmd::REG_CTRL_MEAS, &ctrlMeasSleep, 1);
  if (!st.ok()) {
    _markHardwareConfigDirty(st);
    return st;
  }
  _configSyncState = ConfigSyncState::UPDATE_IN_PROGRESS;

  st = _ensureConfigWriteReady();
  if (!st.ok()) {
    _markHardwareConfigDirty(st);
    const Status restore = writeRegs(cmd::REG_CTRL_MEAS, &ctrlMeas, 1);
    if (!restore.ok()) {
      _markHardwareConfigDirty(restore);
    }
    return st;
  }

  st = writeRegs(cmd::REG_CONFIG, &config, 1);
  if (!st.ok()) {
    _markHardwareConfigDirty(st);
    return st;
  }

  st = writeRegs(cmd::REG_CTRL_HUM, &ctrlHum, 1);
  if (!st.ok()) {
    _markHardwareConfigDirty(st);
    return st;
  }

  st = writeRegs(cmd::REG_CTRL_MEAS, &ctrlMeas, 1);
  if (!st.ok()) {
    _markHardwareConfigDirty(st);
    return st;
  }

  _clearHardwareConfigDirty();
  if (_config.mode == Mode::FORCED) {
    _conversionState = ConversionState::IDLE;
  }
  return Status::Ok();
}

Status BME280::_waitForNvmReady(bool tracked) {
  const uint32_t deadline = _nowMs() + _config.nvmReadyTimeoutMs;

  uint8_t status = 0;
  const Status st = tracked ? readRegister(cmd::REG_STATUS, status)
                            : _readRegisterRaw(cmd::REG_STATUS, status);
  if (!st.ok()) {
    return st;
  }
  if ((status & cmd::MASK_STATUS_IM_UPDATE) == 0) {
    return Status::Ok();
  }

  if (deadlineReached(_nowMs(), deadline)) {
    return Status::Error(Err::TIMEOUT, "NVM ready timeout",
                         static_cast<int32_t>(_config.nvmReadyTimeoutMs));
  }

  return Status::Error(
      Err::BUSY, static_cast<int32_t>(BusyReason::NVM_UPDATE));
}

Status BME280::_readCalibrationTp() {
  uint8_t calibTP[cmd::REG_CALIB_TP_LEN] = {};
  Status st = readRegs(cmd::REG_CALIB_TP_START, calibTP, sizeof(calibTP));
  if (!st.ok()) {
    return st;
  }

  _jobCalibration.digT1 = static_cast<uint16_t>((calibTP[1] << 8) | calibTP[0]);
  _jobCalibration.digT2 = static_cast<int16_t>((calibTP[3] << 8) | calibTP[2]);
  _jobCalibration.digT3 = static_cast<int16_t>((calibTP[5] << 8) | calibTP[4]);

  _jobCalibration.digP1 = static_cast<uint16_t>((calibTP[7] << 8) | calibTP[6]);
  _jobCalibration.digP2 = static_cast<int16_t>((calibTP[9] << 8) | calibTP[8]);
  _jobCalibration.digP3 = static_cast<int16_t>((calibTP[11] << 8) | calibTP[10]);
  _jobCalibration.digP4 = static_cast<int16_t>((calibTP[13] << 8) | calibTP[12]);
  _jobCalibration.digP5 = static_cast<int16_t>((calibTP[15] << 8) | calibTP[14]);
  _jobCalibration.digP6 = static_cast<int16_t>((calibTP[17] << 8) | calibTP[16]);
  _jobCalibration.digP7 = static_cast<int16_t>((calibTP[19] << 8) | calibTP[18]);
  _jobCalibration.digP8 = static_cast<int16_t>((calibTP[21] << 8) | calibTP[20]);
  _jobCalibration.digP9 = static_cast<int16_t>((calibTP[23] << 8) | calibTP[22]);
  _jobCalibration.digH1 = calibTP[25];

  return Status::Ok();
}

Status BME280::_readCalibrationH() {
  uint8_t calibH[cmd::REG_CALIB_H_LEN] = {};
  Status st = readRegs(cmd::REG_CALIB_H_START, calibH, sizeof(calibH));
  if (!st.ok()) {
    return st;
  }

  _jobHumidityCalibrationValid =
      humidityCalibrationBlockValid(calibH, sizeof(calibH));
  if (_config.osrsH != Oversampling::SKIP &&
      !_jobHumidityCalibrationValid) {
    return Status::Error(Err::CALIBRATION_INVALID,
                         "Humidity calibration block erased");
  }

  _jobCalibration.digH2 = static_cast<int16_t>((calibH[1] << 8) | calibH[0]);
  _jobCalibration.digH3 = calibH[2];

  int16_t h4 = static_cast<int16_t>((calibH[3] << 4) | (calibH[4] & 0x0F));
  int16_t h5 = static_cast<int16_t>((calibH[5] << 4) | (calibH[4] >> 4));
  _jobCalibration.digH4 = signExtend12(h4);
  _jobCalibration.digH5 = signExtend12(h5);
  _jobCalibration.digH6 = static_cast<int8_t>(calibH[6]);

  return Status::Ok();
}

Status BME280::_readCalibrationCandidate(
    Calibration& calibration, bool& humidityCalibrationValid,
    bool& calibrationEvidenceChanged) {
  calibrationEvidenceChanged = false;
  uint8_t calibTP[cmd::REG_CALIB_TP_LEN] = {};
  Status st = readRegs(cmd::REG_CALIB_TP_START, calibTP, sizeof(calibTP));
  if (!st.ok()) {
    return st;
  }

  Calibration candidate;
  candidate.digT1 = static_cast<uint16_t>((calibTP[1] << 8) | calibTP[0]);
  candidate.digT2 = static_cast<int16_t>((calibTP[3] << 8) | calibTP[2]);
  candidate.digT3 = static_cast<int16_t>((calibTP[5] << 8) | calibTP[4]);
  candidate.digP1 = static_cast<uint16_t>((calibTP[7] << 8) | calibTP[6]);
  candidate.digP2 = static_cast<int16_t>((calibTP[9] << 8) | calibTP[8]);
  candidate.digP3 = static_cast<int16_t>((calibTP[11] << 8) | calibTP[10]);
  candidate.digP4 = static_cast<int16_t>((calibTP[13] << 8) | calibTP[12]);
  candidate.digP5 = static_cast<int16_t>((calibTP[15] << 8) | calibTP[14]);
  candidate.digP6 = static_cast<int16_t>((calibTP[17] << 8) | calibTP[16]);
  candidate.digP7 = static_cast<int16_t>((calibTP[19] << 8) | calibTP[18]);
  candidate.digP8 = static_cast<int16_t>((calibTP[21] << 8) | calibTP[20]);
  candidate.digP9 = static_cast<int16_t>((calibTP[23] << 8) | calibTP[22]);
  candidate.digH1 = calibTP[cmd::REG_CALIB_TP_LEN - 1];
  calibrationEvidenceChanged = !_calibrationTpMatchesCommitted(candidate);

  uint8_t calibH[cmd::REG_CALIB_H_LEN] = {};
  st = readRegs(cmd::REG_CALIB_H_START, calibH, sizeof(calibH));
  if (!st.ok()) {
    return st;
  }

  candidate.digH2 = static_cast<int16_t>((calibH[1] << 8) | calibH[0]);
  candidate.digH3 = calibH[2];
  int16_t h4 = static_cast<int16_t>((calibH[3] << 4) | (calibH[4] & 0x0F));
  int16_t h5 = static_cast<int16_t>((calibH[5] << 4) | (calibH[4] >> 4));
  candidate.digH4 = signExtend12(h4);
  candidate.digH5 = signExtend12(h5);
  candidate.digH6 = static_cast<int8_t>(calibH[6]);

  st = _validateCalibrationValues(candidate.digT1, candidate.digP1);
  if (!st.ok()) {
    calibrationEvidenceChanged = true;
    return st;
  }
  const bool candidateHumidityCalibrationValid =
      humidityCalibrationBlockValid(calibH, sizeof(calibH));
  if (_config.osrsH != Oversampling::SKIP &&
      !candidateHumidityCalibrationValid) {
    calibrationEvidenceChanged = true;
    return Status::Error(Err::CALIBRATION_INVALID,
                         "Humidity calibration block erased");
  }

  calibrationEvidenceChanged = calibrationEvidenceChanged ||
      !_calibrationMatchesCommitted(candidate,
                                    candidateHumidityCalibrationValid);
  calibration = candidate;
  humidityCalibrationValid = candidateHumidityCalibrationValid;
  return Status::Ok();
}

Status BME280::_validateCalibrationValues(uint16_t digT1, uint16_t digP1) const {
  if (digT1 == 0 || digT1 == 0xFFFF) {
    return Status::Error(Err::CALIBRATION_INVALID, "Invalid temperature calibration");
  }
  if (digP1 == 0 || digP1 == 0xFFFF) {
    return Status::Error(Err::CALIBRATION_INVALID, "Invalid pressure calibration");
  }

  return Status::Ok();
}

void BME280::_commitCalibration(const Calibration& calibration,
                                bool humidityCalibrationValid) {
  _digT1 = calibration.digT1;
  _digT2 = calibration.digT2;
  _digT3 = calibration.digT3;
  _digP1 = calibration.digP1;
  _digP2 = calibration.digP2;
  _digP3 = calibration.digP3;
  _digP4 = calibration.digP4;
  _digP5 = calibration.digP5;
  _digP6 = calibration.digP6;
  _digP7 = calibration.digP7;
  _digP8 = calibration.digP8;
  _digP9 = calibration.digP9;
  _digH1 = calibration.digH1;
  _digH2 = calibration.digH2;
  _digH3 = calibration.digH3;
  _digH4 = calibration.digH4;
  _digH5 = calibration.digH5;
  _digH6 = calibration.digH6;
  _humidityCalibrationValid = humidityCalibrationValid;
  _calibrationState = CalibrationState::VALID;
}

bool BME280::_calibrationMatchesCommitted(
    const Calibration& calibration, bool humidityCalibrationValid) const {
  return _calibrationState == CalibrationState::VALID &&
         _humidityCalibrationValid == humidityCalibrationValid &&
         _digT1 == calibration.digT1 &&
         _digT2 == calibration.digT2 &&
         _digT3 == calibration.digT3 &&
         _digP1 == calibration.digP1 &&
         _digP2 == calibration.digP2 &&
         _digP3 == calibration.digP3 &&
         _digP4 == calibration.digP4 &&
         _digP5 == calibration.digP5 &&
         _digP6 == calibration.digP6 &&
         _digP7 == calibration.digP7 &&
         _digP8 == calibration.digP8 &&
         _digP9 == calibration.digP9 &&
         _digH1 == calibration.digH1 &&
         _digH2 == calibration.digH2 &&
         _digH3 == calibration.digH3 &&
         _digH4 == calibration.digH4 &&
         _digH5 == calibration.digH5 &&
         _digH6 == calibration.digH6;
}

bool BME280::_calibrationTpMatchesCommitted(
    const Calibration& calibration) const {
  return _calibrationState == CalibrationState::VALID &&
         _digT1 == calibration.digT1 &&
         _digT2 == calibration.digT2 &&
         _digT3 == calibration.digT3 &&
         _digP1 == calibration.digP1 &&
         _digP2 == calibration.digP2 &&
         _digP3 == calibration.digP3 &&
         _digP4 == calibration.digP4 &&
         _digP5 == calibration.digP5 &&
         _digP6 == calibration.digP6 &&
         _digP7 == calibration.digP7 &&
         _digP8 == calibration.digP8 &&
         _digP9 == calibration.digP9 &&
         _digH1 == calibration.digH1;
}

Status BME280::_readRawData(RawSample& out) {
  uint8_t data[cmd::DATA_LEN] = {};
  Status st = readRegs(cmd::REG_DATA_START, data, sizeof(data));
  if (!st.ok()) {
    return st;
  }

  RawSample candidate{};
  candidate.adcP = (static_cast<int32_t>(data[0]) << 12) |
                   (static_cast<int32_t>(data[1]) << 4) |
                   (static_cast<int32_t>(data[2]) >> 4);
  candidate.adcT = (static_cast<int32_t>(data[3]) << 12) |
                   (static_cast<int32_t>(data[4]) << 4) |
                   (static_cast<int32_t>(data[5]) >> 4);
  candidate.adcH = (static_cast<int32_t>(data[6]) << 8) |
                   static_cast<int32_t>(data[7]);
  candidate.pressureValid = (_config.osrsP != Oversampling::SKIP) &&
                            (candidate.adcP != cmd::RAW_PRESSURE_SKIPPED);
  candidate.temperatureValid = (_config.osrsT != Oversampling::SKIP) &&
                               (candidate.adcT != cmd::RAW_TEMPERATURE_SKIPPED);
  candidate.humidityValid = (_config.osrsH != Oversampling::SKIP) &&
                            (candidate.adcH != cmd::RAW_HUMIDITY_SKIPPED);

  out = candidate;
  return Status::Ok();
}

Status BME280::_compensate(const RawSample& raw,
                           CompensatedSample& compensated,
                           int32_t& tFine) const {
  CompensatedSample candidate{};
  int32_t candidateTFine = 0;
  const bool pressSkipped = (_config.osrsP == Oversampling::SKIP);
  const bool humSkipped = (_config.osrsH == Oversampling::SKIP);

  // Temperature compensation is required for pressure and humidity.
  if (!raw.temperatureValid) {
    return Status::Error(Err::COMPENSATION_ERROR, "Temperature sample skipped");
  }

  // --- Temperature (Bosch int32 reference) ---
  const int32_t adcT = raw.adcT;

  const int64_t tDelta1 = (static_cast<int64_t>(adcT) >> 3) -
                          (static_cast<int64_t>(_digT1) << 1);
  const int64_t tVar1 = (tDelta1 * static_cast<int64_t>(_digT2)) >> 11;
  const int64_t tDelta2 = (static_cast<int64_t>(adcT) >> 4) -
                          static_cast<int64_t>(_digT1);
  const int64_t tVar2 = (((tDelta2 * tDelta2) >> 12) *
                         static_cast<int64_t>(_digT3)) >> 14;
  const int64_t tFine64 = tVar1 + tVar2;
  if (tFine64 < static_cast<int64_t>(std::numeric_limits<int32_t>::min()) ||
      tFine64 > static_cast<int64_t>(std::numeric_limits<int32_t>::max())) {
    return Status::Error(Err::COMPENSATION_ERROR, "Temperature compensation overflow");
  }
  const int64_t tempC_x100 = (tFine64 * 5 + 128) >> 8;
  if (tempC_x100 < static_cast<int64_t>(std::numeric_limits<int32_t>::min()) ||
      tempC_x100 > static_cast<int64_t>(std::numeric_limits<int32_t>::max())) {
    return Status::Error(Err::COMPENSATION_ERROR, "Temperature output overflow");
  }

  candidateTFine = static_cast<int32_t>(tFine64);
  candidate.tempC_x100 = static_cast<int32_t>(tempC_x100);
  candidate.temperatureValid = true;

  // --- Pressure (Bosch int64 reference) ---
  if (pressSkipped) {
    candidate.pressurePa = 0;
    candidate.pressureValid = false;
  } else {
    if (!raw.pressureValid) {
      return Status::Error(Err::COMPENSATION_ERROR, "Pressure sample skipped");
    }
    const int32_t adcP = raw.adcP;

    int64_t pVar1 = static_cast<int64_t>(candidateTFine) - 128000;
    int64_t pSquared = 0;
    int64_t pVar2 = 0;
    int64_t term = 0;
    int64_t sum = 0;
    if (!checkedMulI64(pVar1, pVar1, pSquared) ||
        !checkedMulI64(pSquared, static_cast<int64_t>(_digP6), pVar2) ||
        !checkedMulI64(pVar1, static_cast<int64_t>(_digP5), term) ||
        !checkedMulI64(term, 131072, term) ||
        !checkedAddI64(pVar2, term, pVar2) ||
        !checkedMulI64(static_cast<int64_t>(_digP4), 34359738368LL, term) ||
        !checkedAddI64(pVar2, term, pVar2)) {
      return Status::Error(Err::COMPENSATION_ERROR,
                           "Pressure compensation overflow");
    }

    if (!checkedMulI64(pSquared, static_cast<int64_t>(_digP3), term)) {
      return Status::Error(Err::COMPENSATION_ERROR,
                           "Pressure compensation overflow");
    }
    term >>= 8;
    if (!checkedMulI64(pVar1, static_cast<int64_t>(_digP2), sum) ||
        !checkedMulI64(sum, 4096, sum) ||
        !checkedAddI64(term, sum, pVar1) ||
        !checkedAddI64(140737488355328LL, pVar1, sum) ||
        !checkedMulI64(sum, static_cast<int64_t>(_digP1), pVar1)) {
      return Status::Error(Err::COMPENSATION_ERROR,
                           "Pressure compensation overflow");
    }
    pVar1 >>= 33;
    if (pVar1 == 0) {
      return Status::Error(Err::COMPENSATION_ERROR, "Pressure div by zero");
    }

    int64_t p = 1048576 - static_cast<int64_t>(adcP);
    if (!checkedMulI64(p, 2147483648LL, p) ||
        !checkedSubI64(p, pVar2, p) ||
        !checkedMulI64(p, 3125, p) ||
        (p == std::numeric_limits<int64_t>::min() && pVar1 == -1)) {
      return Status::Error(Err::COMPENSATION_ERROR,
                           "Pressure compensation overflow");
    }
    p /= pVar1;

    const int64_t pressureScaled = p >> 13;
    if (!checkedMulI64(static_cast<int64_t>(_digP9), pressureScaled, pVar1) ||
        !checkedMulI64(pVar1, pressureScaled, pVar1)) {
      return Status::Error(Err::COMPENSATION_ERROR,
                           "Pressure compensation overflow");
    }
    pVar1 >>= 25;
    if (!checkedMulI64(static_cast<int64_t>(_digP8), p, pVar2)) {
      return Status::Error(Err::COMPENSATION_ERROR,
                           "Pressure compensation overflow");
    }
    pVar2 >>= 19;
    if (!checkedAddI64(p, pVar1, sum) ||
        !checkedAddI64(sum, pVar2, sum) ||
        !checkedMulI64(static_cast<int64_t>(_digP7), 16, term)) {
      return Status::Error(Err::COMPENSATION_ERROR,
                           "Pressure compensation overflow");
    }
    sum >>= 8;
    if (!checkedAddI64(sum, term, p)) {
      return Status::Error(Err::COMPENSATION_ERROR,
                           "Pressure compensation overflow");
    }
    int64_t pressurePa = p >> 8;
    if (pressurePa < 0) {
      pressurePa = 0;
    } else if (pressurePa > static_cast<int64_t>(std::numeric_limits<uint32_t>::max())) {
      pressurePa = static_cast<int64_t>(std::numeric_limits<uint32_t>::max());
    }
    candidate.pressurePa = static_cast<uint32_t>(pressurePa);
    candidate.pressureValid = true;
  }

  // --- Humidity (Bosch int32 reference, widened to int64 for safety) ---
  if (humSkipped) {
    candidate.humidityPct_x1024 = 0;
    candidate.humidityValid = false;
  } else {
    if (!raw.humidityValid) {
      return Status::Error(Err::COMPENSATION_ERROR, "Humidity sample skipped");
    }
    const int32_t adcH = raw.adcH;

    int64_t h = static_cast<int64_t>(candidateTFine) - 76800;
    int64_t hTerm1 = 0;
    int64_t hTerm2 = 0;
    int64_t hFactor = 0;
    int64_t hCorrection = 0;
    int64_t hIntermediate = 0;
    if (!checkedMulI64(static_cast<int64_t>(adcH), 16384, hTerm1) ||
        !checkedMulI64(static_cast<int64_t>(_digH4), 1048576,
                       hIntermediate) ||
        !checkedSubI64(hTerm1, hIntermediate, hTerm1) ||
        !checkedMulI64(static_cast<int64_t>(_digH5), h, hIntermediate) ||
        !checkedSubI64(hTerm1, hIntermediate, hTerm1) ||
        !checkedAddI64(hTerm1, 16384, hTerm1)) {
      return Status::Error(Err::COMPENSATION_ERROR,
                           "Humidity compensation overflow");
    }
    if (!checkedMulI64(h, static_cast<int64_t>(_digH6), hTerm2) ||
        !checkedMulI64(h, static_cast<int64_t>(_digH3), hFactor)) {
      return Status::Error(Err::COMPENSATION_ERROR,
                           "Humidity compensation overflow");
    }
    hTerm2 >>= 10;
    hFactor >>= 11;
    if (!checkedAddI64(hFactor, 32768, hFactor) ||
        !checkedMulI64(hTerm2, hFactor, hTerm2)) {
      return Status::Error(Err::COMPENSATION_ERROR,
                           "Humidity compensation overflow");
    }
    hTerm2 >>= 10;
    if (!checkedAddI64(hTerm2, 2097152, hTerm2) ||
        !checkedMulI64(hTerm2, static_cast<int64_t>(_digH2), hTerm2) ||
        !checkedAddI64(hTerm2, 8192, hTerm2)) {
      return Status::Error(Err::COMPENSATION_ERROR,
                           "Humidity compensation overflow");
    }
    hTerm2 >>= 14;
    if (!checkedMulI64(hTerm1 >> 15, hTerm2, h)) {
      return Status::Error(Err::COMPENSATION_ERROR,
                           "Humidity compensation overflow");
    }
    hCorrection = h >> 15;
    if (!checkedMulI64(hCorrection, hCorrection, hCorrection)) {
      return Status::Error(Err::COMPENSATION_ERROR,
                           "Humidity compensation overflow");
    }
    hCorrection >>= 7;
    if (!checkedMulI64(hCorrection, static_cast<int64_t>(_digH1), hCorrection)) {
      return Status::Error(Err::COMPENSATION_ERROR,
                           "Humidity compensation overflow");
    }
    hCorrection >>= 4;
    if (!checkedSubI64(h, hCorrection, h)) {
      return Status::Error(Err::COMPENSATION_ERROR,
                           "Humidity compensation overflow");
    }
    if (h < 0) {
      h = 0;
    }
    if (h > HUMIDITY_MAX_X4096) {
      h = HUMIDITY_MAX_X4096;
    }
    candidate.humidityPct_x1024 = static_cast<uint32_t>(h >> 12);
    candidate.humidityValid = true;
  }

  compensated = candidate;
  tFine = candidateTFine;
  return Status::Ok();
}

void BME280::_commitSample(const RawSample& raw,
                           const CompensatedSample& compensated,
                           int32_t tFine, uint32_t timestampMs) {
  if (_sampleSequence < std::numeric_limits<uint32_t>::max()) {
    ++_sampleSequence;
  }
  _rawSample = raw;
  _compSample = compensated;
  _tFine = tFine;
  _sampleTimestampMs = timestampMs;
  _sampleConfigGeneration = _configGeneration;
  _sampleGenerationStale = false;
  _hasSample = true;
  _measurementReady = true;
}

void BME280::_advanceConfigGeneration() {
  if (_configGeneration < std::numeric_limits<uint32_t>::max()) {
    ++_configGeneration;
  }
  if (_hasSample) {
    _sampleGenerationStale = true;
  }
}

void BME280::_cancelMeasurementTrackingForStateChange() {
  if (_conversionState == ConversionState::IN_PROGRESS) {
    _conversionState = ConversionState::UNKNOWN_AFTER_TRIGGER_ERROR;
  }
  _measurementRequested = false;
  _measurementReady = false;
  _measurementStartMs = 0;
  _measurementDeadlineMs = 0;
  _measurementStatusPolls = 0;
  _lastMeasurementStatus = Status::Error(Err::RESYNC_REQUIRED);
}

void BME280::_invalidateSampleCache() {
  _measurementRequested = false;
  _measurementReady = false;
  _lastMeasurementStatus = Status::Ok();
  _hasSample = false;
  _measurementStartMs = 0;
  _measurementDeadlineMs = 0;
  _measurementStatusPolls = 0;
  _sampleTimestampMs = 0;
  _sampleConfigGeneration = 0;
  _sampleGenerationStale = false;
  _tFine = 0;
  _rawSample = RawSample{};
  _compSample = CompensatedSample{};
}

uint32_t BME280::_nowMs() const {
  if (_timeContextActive) {
    return _timeContextMs;
  }
  if (_config.nowMs != nullptr) {
    return _config.nowMs(_config.timeUser);
  }
  return 0;
}

bool BME280::_timeValid() const {
  return _timeContextActive || _config.nowMs != nullptr;
}

}  // namespace BME280
