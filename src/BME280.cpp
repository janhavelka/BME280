/**
 * @file BME280.cpp
 * @brief BME280 driver implementation.
 */

#include "BME280/BME280.h"

#include "PlatformTime.h"

#include <cstring>
#include <limits>

namespace BME280 {
namespace {

static constexpr size_t MAX_WRITE_LEN = 16;
static constexpr uint16_t NVM_READY_MAX_POLLS = 255;
static constexpr uint32_t MEASUREMENT_MARGIN_US = 1000;
static constexpr int64_t HUMIDITY_MAX_X4096 = 419430400;

class ScopedOfflineI2cAllowance {
public:
  explicit ScopedOfflineI2cAllowance(bool& flag, bool allow) : _flag(flag), _old(flag) {
    _flag = allow;
  }

  ~ScopedOfflineI2cAllowance() {
    _flag = _old;
  }

  ScopedOfflineI2cAllowance(const ScopedOfflineI2cAllowance&) = delete;
  ScopedOfflineI2cAllowance& operator=(const ScopedOfflineI2cAllowance&) = delete;

private:
  bool& _flag;
  bool _old;
};

static bool deadlineReached(uint32_t nowMs, uint32_t deadlineMs) {
  return static_cast<int32_t>(nowMs - deadlineMs) >= 0;
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

}  // namespace

void BME280::_resetRuntime() {
  const bool priorHardwareConfigDirty = _hardwareConfigDirty;
  const Status priorHardwareConfigDirtyError = _hardwareConfigDirtyError;

  _config = Config{};
  _initialized = false;
  _driverState = DriverState::UNINIT;
  _allowOfflineI2c = false;

  _lastOkMs = 0;
  _lastErrorMs = 0;
  _lastError = Status::Ok();
  _consecutiveFailures = 0;
  _totalFailures = 0;
  _totalSuccess = 0;
  _hardwareConfigDirty = priorHardwareConfigDirty;
  _hardwareConfigDirtyError = priorHardwareConfigDirtyError;

  _measurementRequested = false;
  _measurementReady = false;
  _lastMeasurementStatus = Status::Ok();
  _hasSample = false;
  _measurementStartMs = 0;
  _sampleTimestampMs = 0;
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
  if (config.nvmReadyTimeoutMs == 0) {
    return Status::Error(Err::INVALID_CONFIG, "NVM timeout must be > 0");
  }
  if (config.nvmReadyTimeoutMs >
      static_cast<uint32_t>(std::numeric_limits<int32_t>::max())) {
    return Status::Error(Err::INVALID_CONFIG, "NVM timeout too large");
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
  if (!isValidMeasurementSelection(config.osrsT, config.osrsP, config.osrsH)) {
    return Status::Error(Err::INVALID_CONFIG, "Invalid oversampling combination");
  }

  _config = config;
  if (_config.offlineThreshold == 0) {
    _config.offlineThreshold = 1;
  }

  return Status::Ok();
}

Status BME280::begin(const Config& config) {
  Status st = _prepareBeginConfig(config);
  if (!st.ok()) {
    return st;
  }

  uint8_t chipId = 0;
  st = _readRegisterRaw(cmd::REG_CHIP_ID, chipId);
  if (!st.ok()) {
    return mapPresenceError(st);
  }
  if (chipId != cmd::CHIP_ID_BME280) {
    return Status::Error(Err::CHIP_ID_MISMATCH, "Chip ID mismatch", chipId);
  }

  st = _waitForNvmReady(false);
  if (!st.ok()) {
    return mapPresenceError(st);
  }

  st = _readCalibration();
  if (!st.ok()) {
    return mapPresenceError(st);
  }
  st = _validateCalibration();
  if (!st.ok()) {
    return st;
  }
  st = _applyConfig();
  if (!st.ok()) {
    return mapPresenceError(st);
  }

  _initialized = true;
  _driverState = DriverState::READY;

  return Status::Ok();
}

void BME280::tick(uint32_t nowMs) {
  if (!_initialized || !_measurementRequested) {
    return;
  }
  if (_jobActive()) {
    return;
  }

  if (_driverState == DriverState::OFFLINE) {
    _measurementRequested = false;
    _lastMeasurementStatus = Status::Error(Err::BUSY, "Driver is offline; call recover()");
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
    if (_driverState == DriverState::OFFLINE) {
      _measurementRequested = false;
    }
    return;
  }
  if (measuring) {
    _lastMeasurementStatus = Status::Error(Err::IN_PROGRESS, "Measurement still running");
    return;
  }

  st = _readRawData();
  if (!st.ok()) {
    _lastMeasurementStatus = st;
    if (_driverState == DriverState::OFFLINE) {
      _measurementRequested = false;
    }
    return;
  }

  st = _compensate();
  if (!st.ok()) {
    _lastMeasurementStatus = st;
    _measurementRequested = false;
    return;
  }

  _measurementReady = true;
  _hasSample = true;
  _sampleTimestampMs = nowMs;
  _measurementRequested = false;
  _lastMeasurementStatus = Status::Ok();
}

void BME280::end() {
  if (_initialized) {
    // Best-effort: put device to sleep to save power.
    // Uses raw I2C to avoid health tracking during shutdown.
    const uint8_t payload[2] = {
      cmd::REG_CTRL_MEAS,
      buildCtrlMeas(_config.osrsT, _config.osrsP, Mode::SLEEP)
    };
    (void)_i2cWriteRaw(payload, sizeof(payload));
  }

  _initialized = false;
  _driverState = DriverState::UNINIT;
  _measurementRequested = false;
  _measurementReady = false;
  _lastMeasurementStatus = Status::Ok();
  _hasSample = false;
  _measurementStartMs = 0;
  _sampleTimestampMs = 0;
  _tFine = 0;
  _rawSample = RawSample{};
  _compSample = CompensatedSample{};
  _clearJob();
}

Status BME280::probe() {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }

  uint8_t chipId = 0;
  Status st = _readRegisterRaw(cmd::REG_CHIP_ID, chipId);
  if (!st.ok()) {
    return mapPresenceError(st);
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

  const bool startedOffline = (_driverState == DriverState::OFFLINE);
  ScopedOfflineI2cAllowance allowOfflineI2c(_allowOfflineI2c, true);
  Status result = [&]() -> Status {
    uint8_t chipId = 0;
    Status st = readRegister(cmd::REG_CHIP_ID, chipId);
    if (!st.ok()) {
      return st;
    }
    if (chipId != cmd::CHIP_ID_BME280) {
      return _recordFailure(
          Status::Error(Err::CHIP_ID_MISMATCH, "Chip ID mismatch", chipId));
    }

    st = _waitForNvmReady(true);
    if (!st.ok()) {
      return isTransportFailure(st) ? st : _recordFailure(st);
    }

    st = _readCalibration();
    if (!st.ok()) {
      return st.code == Err::CALIBRATION_INVALID ? _recordFailure(st) : st;
    }
    st = _validateCalibration();
    if (!st.ok()) {
      return _recordFailure(st);
    }

    // Re-apply configuration: after a power glitch or external reset the
    // device registers revert to defaults and calibration registers may have
    // just been copied from NVM.
    st = _applyConfig();
    if (!st.ok()) {
      if (st.code == Err::BUSY) {
        return _recordFailure(st);
      }
      return st;
    }

    return Status::Ok();
  }();
  if (result.ok()) {
    _invalidateSampleCache();
  }
  if (startedOffline && !result.ok() && !result.inProgress()) {
    _reassertOfflineLatch();
  }
  return result;
}

Status BME280::getSettings(SettingsSnapshot& out) const {
  out.initialized = _initialized;
  out.state = _driverState;
  out.i2cAddress = _config.i2cAddress;
  out.i2cTimeoutMs = _config.i2cTimeoutMs;
  out.nvmReadyTimeoutMs = _config.nvmReadyTimeoutMs;
  out.offlineThreshold = _config.offlineThreshold;
  out.hasNowMsHook = (_config.nowMs != nullptr);
  out.mode = _config.mode;
  out.osrsT = _config.osrsT;
  out.osrsP = _config.osrsP;
  out.osrsH = _config.osrsH;
  out.filter = _config.filter;
  out.standby = _config.standby;
  out.measurementRequested = _measurementRequested;
  out.measurementReady = _measurementReady;
  out.lastMeasurementStatus = _lastMeasurementStatus;
  out.hasSample = _hasSample;
  out.hardwareConfigDirty = _hardwareConfigDirty;
  out.hardwareConfigDirtyError = _hardwareConfigDirtyError;
  out.measurementStartMs = _measurementStartMs;
  out.sampleTimestampMs = _sampleTimestampMs;
  out.tFine = _tFine;
  out.rawSample = _rawSample;
  out.compSample = _compSample;
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
  return Status::Ok();
}

bool BME280::_jobActive() const {
  return _jobKind != JobKind::NONE &&
         (_jobState == JobState::RUNNING || _jobState == JobState::WAITING);
}

void BME280::_clearJob() {
  _jobKind = JobKind::NONE;
  _jobState = JobState::IDLE;
  _jobPhase = JobPhase::NONE;
  _jobStatus = Status::Ok();
  _jobDeadlineMs = 0;
  _jobNvmPolls = 0;
  _jobStartedOffline = false;
  _jobHardwareConfigTouched = false;
  _jobCalibration = Calibration{};
}

Status BME280::_startJob(JobKind kind, JobPhase phase) {
  if (_jobActive()) {
    return Status::Error(Err::BUSY, "Job already running");
  }

  _jobKind = kind;
  _jobState = JobState::RUNNING;
  _jobPhase = phase;
  _jobStatus = Status::Error(Err::IN_PROGRESS, "Job in progress");
  _jobDeadlineMs = 0;
  _jobNvmPolls = 0;
  _jobStartedOffline = false;
  _jobHardwareConfigTouched = false;
  _jobCalibration = Calibration{};
  return Status::Error(Err::IN_PROGRESS, "Job started");
}

JobPollResult BME280::_jobResult(uint8_t instructionsUsed) const {
  JobPollResult result;
  result.state = _jobState;
  result.status = _jobStatus;
  result.instructionsUsed = instructionsUsed;
  return result;
}

JobPollResult BME280::_failJob(const Status& st, uint8_t instructionsUsed) {
  Status finalStatus = st;
  if (_jobKind == JobKind::RECOVERY && !st.ok() && !st.inProgress() &&
      !isTransportFailure(st)) {
    finalStatus = _recordFailure(st);
  }
  if (_jobHardwareConfigTouched && !finalStatus.ok() && !finalStatus.inProgress()) {
    _markHardwareConfigDirty(finalStatus);
  }
  if (_jobKind == JobKind::FORCED_MEASUREMENT &&
      !finalStatus.ok() && !finalStatus.inProgress()) {
    _measurementRequested = false;
    _measurementReady = false;
    _lastMeasurementStatus = finalStatus;
  }
  _jobState = JobState::FAILED;
  _jobStatus = finalStatus;
  _jobPhase = JobPhase::NONE;
  if (_jobKind == JobKind::RECOVERY && _jobStartedOffline &&
      !finalStatus.ok() && !finalStatus.inProgress()) {
    _reassertOfflineLatch();
  }
  return _jobResult(instructionsUsed);
}

JobPollResult BME280::_completeJob(uint8_t instructionsUsed) {
  _jobState = JobState::DONE;
  _jobStatus = Status::Ok();
  _jobPhase = JobPhase::COMPLETE;
  _jobNvmPolls = 0;
  _jobStartedOffline = false;
  _jobHardwareConfigTouched = false;
  return _jobResult(instructionsUsed);
}

void BME280::_trackJobConfigWriteResult(const Status& st) {
  if (st.ok()) {
    _jobHardwareConfigTouched = true;
    return;
  }
  if (mayHaveReachedDeviceAfterDiagnosticWrite(st)) {
    _markHardwareConfigDirty(st);
  }
}

Status BME280::startInitJob(const Config& config) {
  if (_jobActive()) {
    return Status::Error(Err::BUSY, "Job already running");
  }
  Status st = _prepareBeginConfig(config);
  if (!st.ok()) {
    return st;
  }
  return _startJob(JobKind::INIT, JobPhase::INIT_READ_CHIP_ID);
}

Status BME280::startForcedMeasurementJob() {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  if (_driverState == DriverState::OFFLINE) {
    return Status::Error(Err::BUSY, "Driver is offline; call recover()");
  }
  if (_config.mode != Mode::FORCED) {
    return Status::Error(Err::INVALID_PARAM, "Device is not in forced mode");
  }
  if (_measurementRequested && !_measurementReady) {
    return Status::Error(Err::BUSY, "Measurement in progress");
  }

  Status st = _startJob(JobKind::FORCED_MEASUREMENT, JobPhase::FORCE_WRITE_CTRL_HUM);
  if (st.inProgress()) {
    _measurementReady = false;
  }
  return st;
}

Status BME280::startApplyConfigJob() {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  if (_driverState == DriverState::OFFLINE) {
    return Status::Error(Err::BUSY, "Driver is offline; call recover()");
  }
  return _startJob(JobKind::APPLY_CONFIG, JobPhase::APPLY_WAIT_IDLE);
}

Status BME280::startRecoveryJob() {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }

  const bool startedOffline = (_driverState == DriverState::OFFLINE);
  Status st = _startJob(JobKind::RECOVERY, JobPhase::RECOVERY_WRITE_RESET);
  if (st.inProgress()) {
    _jobStartedOffline = startedOffline;
    _measurementRequested = false;
    _measurementReady = false;
    _hasSample = false;
    _measurementStartMs = 0;
    _sampleTimestampMs = 0;
  }
  return st;
}

JobPollResult BME280::pollJob(uint32_t nowMs, uint8_t maxInstructions) {
  if (_jobKind == JobKind::NONE || _jobState == JobState::IDLE ||
      _jobState == JobState::DONE || _jobState == JobState::FAILED) {
    return _jobResult(0);
  }
  if (maxInstructions == 0) {
    return _jobResult(0);
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
        if (chipId != cmd::CHIP_ID_BME280) {
          return _failJob(
              Status::Error(Err::CHIP_ID_MISMATCH, "Chip ID mismatch", chipId),
              instructionsUsed);
        }
        _jobPhase = JobPhase::INIT_NVM_START;
        break;
      }

      case JobPhase::INIT_NVM_START:
        _jobDeadlineMs = nowMs + _config.nvmReadyTimeoutMs;
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
        const Status st = (_jobKind == JobKind::RECOVERY)
            ? readRegister(cmd::REG_STATUS, status)
            : _readRegisterRaw(cmd::REG_STATUS, status);
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
          return _failJob(st, instructionsUsed);
        }
        _jobPhase = JobPhase::VALIDATE_CALIBRATION;
        break;
      }

      case JobPhase::VALIDATE_CALIBRATION: {
        const Status st = _validateCalibrationValues(_jobCalibration.digT1,
                                                     _jobCalibration.digP1);
        if (!st.ok()) {
          return _failJob(st, instructionsUsed);
        }
        _commitCalibration(_jobCalibration);
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
          _jobState = JobState::WAITING;
          _jobStatus = Status::Error(Err::IN_PROGRESS, "Measurement in progress");
          return _jobResult(instructionsUsed);
        }
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
          _markHardwareConfigDirty(
              Status::Error(Err::BUSY, "Device still measuring after sleep write"));
          _jobState = JobState::WAITING;
          _jobStatus = Status::Error(Err::IN_PROGRESS, "Measurement in progress");
          return _jobResult(instructionsUsed);
        }
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

      case JobPhase::FORCE_WRITE_CTRL_HUM: {
        if (instructionsUsed >= maxInstructions) {
          return _jobResult(instructionsUsed);
        }
        const uint8_t value = buildCtrlHum(_config.osrsH);
        const Status st = writeRegs(cmd::REG_CTRL_HUM, &value, 1);
        ++instructionsUsed;
        if (!st.ok()) {
          return _failJob(st, instructionsUsed);
        }
        _jobPhase = JobPhase::FORCE_WRITE_CTRL_MEAS;
        break;
      }

      case JobPhase::FORCE_WRITE_CTRL_MEAS: {
        if (instructionsUsed >= maxInstructions) {
          return _jobResult(instructionsUsed);
        }
        const uint8_t value = buildCtrlMeas(_config.osrsT, _config.osrsP,
                                            Mode::FORCED);
        const Status st = writeRegs(cmd::REG_CTRL_MEAS, &value, 1);
        ++instructionsUsed;
        if (!st.ok()) {
          return _failJob(st, instructionsUsed);
        }
        _measurementRequested = true;
        _measurementReady = false;
        _measurementStartMs = nowMs;
        _jobDeadlineMs = nowMs + estimateMeasurementTimeMs();
        _jobPhase = JobPhase::FORCE_WAIT_TIME;
        break;
      }

      case JobPhase::FORCE_WAIT_TIME:
        if (!deadlineReached(nowMs, _jobDeadlineMs)) {
          _jobState = JobState::WAITING;
          _jobStatus = Status::Error(Err::IN_PROGRESS, "Measurement delay active");
          return _jobResult(instructionsUsed);
        }
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
          return _failJob(st, instructionsUsed);
        }
        if ((status & cmd::MASK_STATUS_MEASURING) != 0) {
          _jobState = JobState::WAITING;
          _jobStatus = Status::Error(Err::IN_PROGRESS, "Measurement in progress");
          return _jobResult(instructionsUsed);
        }
        _jobPhase = JobPhase::FORCE_READ_DATA;
        break;
      }

      case JobPhase::FORCE_READ_DATA: {
        if (instructionsUsed >= maxInstructions) {
          return _jobResult(instructionsUsed);
        }
        const Status st = _readRawData();
        ++instructionsUsed;
        if (!st.ok()) {
          return _failJob(st, instructionsUsed);
        }
        _jobPhase = JobPhase::FORCE_COMPENSATE;
        break;
      }

      case JobPhase::FORCE_COMPENSATE: {
        const Status st = _compensate();
        if (!st.ok()) {
          _measurementRequested = false;
          return _failJob(st, instructionsUsed);
        }
        _measurementReady = true;
        _hasSample = true;
        _sampleTimestampMs = nowMs;
        _measurementRequested = false;
        _jobPhase = JobPhase::COMPLETE;
        break;
      }

      case JobPhase::RECOVERY_WRITE_RESET: {
        if (instructionsUsed >= maxInstructions) {
          return _jobResult(instructionsUsed);
        }
        const uint8_t value = cmd::RESET_VALUE;
        ScopedOfflineI2cAllowance allowOfflineI2c(_allowOfflineI2c, true);
        const Status st = writeRegs(cmd::REG_RESET, &value, 1);
        ++instructionsUsed;
        _trackJobConfigWriteResult(st);
        if (!st.ok()) {
          return _failJob(st, instructionsUsed);
        }
        _jobPhase = JobPhase::INIT_NVM_START;
        break;
      }

      case JobPhase::COMPLETE:
        if (_jobKind == JobKind::INIT) {
          _clearHardwareConfigDirty();
          _initialized = true;
          _driverState = DriverState::READY;
        } else if (_jobKind == JobKind::APPLY_CONFIG) {
          _clearHardwareConfigDirty();
        } else if (_jobKind == JobKind::RECOVERY) {
          _clearHardwareConfigDirty();
          _driverState = DriverState::READY;
          _consecutiveFailures = 0;
        }
        return _completeJob(instructionsUsed);

      case JobPhase::NONE:
      default:
        return _failJob(Status::Error(Err::BUSY, "Invalid job state"),
                        instructionsUsed);
    }
  }

  return _failJob(Status::Error(Err::BUSY, "Job state machine stalled"),
                  instructionsUsed);
}

Status BME280::requestMeasurement() {
  if (!_initialized) {
    Status st = Status::Error(Err::NOT_INITIALIZED, "begin() not called");
    _lastMeasurementStatus = st;
    return st;
  }
  if (_driverState == DriverState::OFFLINE) {
    Status st = Status::Error(Err::BUSY, "Driver is offline; call recover()");
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
    Status st = Status::Error(Err::BUSY, "Measurement in progress");
    _lastMeasurementStatus = st;
    return st;
  }

  _measurementReady = false;

  if (_config.mode == Mode::FORCED) {
    bool measuring = false;
    Status st = isMeasuring(measuring);
    if (!st.ok()) {
      _lastMeasurementStatus = st;
      return st;
    }
    if (measuring) {
      // A conversion can still be running after config writes in forced mode.
      // Track completion instead of forcing the caller to re-issue the request.
      _measurementRequested = true;
      _measurementStartMs = _nowMs();
      st = Status::Error(Err::IN_PROGRESS, "Measurement already in progress");
      _lastMeasurementStatus = st;
      return st;
    }

    const uint8_t ctrlMeas = buildCtrlMeas(_config.osrsT, _config.osrsP, Mode::FORCED);
    st = writeRegs(cmd::REG_CTRL_MEAS, &ctrlMeas, 1);
    if (!st.ok()) {
      _lastMeasurementStatus = st;
      return st;
    }

    _measurementRequested = true;
    _measurementStartMs = _nowMs();

    st = Status::Error(Err::IN_PROGRESS, "Measurement started");
    _lastMeasurementStatus = st;
    return st;
  }

  _measurementRequested = true;
  _measurementStartMs = _nowMs();
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
  if (mode != Mode::SLEEP &&
      !isValidMeasurementSelection(_config.osrsT, _config.osrsP, _config.osrsH)) {
    return Status::Error(Err::INVALID_PARAM, "Invalid oversampling combination");
  }

  const uint8_t ctrlMeas = buildCtrlMeas(_config.osrsT, _config.osrsP,
                                         registerModeForConfig(mode));
  Status st = writeRegs(cmd::REG_CTRL_MEAS, &ctrlMeas, 1);
  if (!st.ok()) {
    _markHardwareConfigDirty(st);
    return st;
  }

  _config.mode = mode;
  _invalidateSampleCache();
  if (mode == Mode::SLEEP) {
    _measurementRequested = false;
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
    _markHardwareConfigDirty(st);
    return st;
  }
  _config.osrsT = osrs;
  _invalidateSampleCache();
  return Status::Ok();
}

Status BME280::setOversamplingP(Oversampling osrs) {
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
    _markHardwareConfigDirty(st);
    return st;
  }
  _config.osrsP = osrs;
  _invalidateSampleCache();
  return Status::Ok();
}

Status BME280::setOversamplingH(Oversampling osrs) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  if (!isValidOversampling(osrs)) {
    return Status::Error(Err::INVALID_PARAM, "Invalid oversampling");
  }
  if (!isValidMeasurementSelection(_config.osrsT, _config.osrsP, osrs)) {
    return Status::Error(Err::INVALID_PARAM, "Invalid oversampling combination");
  }

  const uint8_t ctrlHum = buildCtrlHum(osrs);
  Status st = writeRegs(cmd::REG_CTRL_HUM, &ctrlHum, 1);
  if (!st.ok()) {
    _markHardwareConfigDirty(st);
    return st;
  }

  const uint8_t ctrlMeas = buildCtrlMeas(_config.osrsT, _config.osrsP,
                                         registerModeForConfig(_config.mode));
  st = writeRegs(cmd::REG_CTRL_MEAS, &ctrlMeas, 1);
  if (!st.ok()) {
    _markHardwareConfigDirty(st);
    return st;
  }

  _config.osrsH = osrs;
  _invalidateSampleCache();
  return Status::Ok();
}

Status BME280::setFilter(Filter filter) {
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
  _invalidateSampleCache();
  return Status::Ok();
}

Status BME280::setStandby(Standby standby) {
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
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }

  const bool startedOffline = (_driverState == DriverState::OFFLINE);
  ScopedOfflineI2cAllowance allowOfflineI2c(_allowOfflineI2c, true);
  Status result = [&]() -> Status {
    _measurementRequested = false;
    _measurementReady = false;
    _lastMeasurementStatus = Status::Ok();
    _hasSample = false;
    _measurementStartMs = 0;
    _sampleTimestampMs = 0;

    const uint8_t resetValue = cmd::RESET_VALUE;
    Status st = writeRegs(cmd::REG_RESET, &resetValue, 1);
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

    st = _readCalibration();
    if (!st.ok()) {
      _markHardwareConfigDirty(st);
      return st.code == Err::CALIBRATION_INVALID ? _recordFailure(st) : st;
    }
    st = _validateCalibration();
    if (!st.ok()) {
      _markHardwareConfigDirty(st);
      return _recordFailure(st);
    }
    st = _applyConfig();
    if (!st.ok()) {
      _markHardwareConfigDirty(st);
      if (st.code == Err::BUSY) {
        return _recordFailure(st);
      }
      return st;
    }
    return Status::Ok();
  }();
  if (startedOffline && !result.ok() && !result.inProgress()) {
    _reassertOfflineLatch();
  }
  return result;
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
  return _config.i2cWriteRead(_config.i2cAddress, txBuf, txLen, rxBuf, rxLen,
                              _config.i2cTimeoutMs, _config.i2cUser);
}

Status BME280::_i2cWriteRaw(const uint8_t* buf, size_t len) {
  if (buf == nullptr || len == 0) {
    return Status::Error(Err::INVALID_PARAM, "Invalid I2C buffer");
  }
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
  if (!_allowOfflineI2c && _initialized && _driverState == DriverState::OFFLINE) {
    return _offlineStatus();
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
  if (!_allowOfflineI2c && _initialized && _driverState == DriverState::OFFLINE) {
    return _offlineStatus();
  }

  Status st = _i2cWriteRaw(buf, len);
  if (st.code == Err::INVALID_CONFIG || st.code == Err::INVALID_PARAM) {
    return st;
  }
  return _updateHealth(st);
}

Status BME280::_offlineStatus() const {
  return Status::Error(Err::BUSY, "Driver is offline; call recover()");
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
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  return readRegs(startReg, buf, len);
}

Status BME280::writeRegisters(uint8_t startReg, const uint8_t* buf, size_t len) {
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
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  return readRegs(reg, &value, 1);
}

Status BME280::writeRegister(uint8_t reg, uint8_t value) {
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
  const uint32_t maxU32 = std::numeric_limits<uint32_t>::max();
  const uint8_t maxU8 = std::numeric_limits<uint8_t>::max();

  _lastError = st;
  _lastErrorMs = now;
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

void BME280::_reassertOfflineLatch() {
  _driverState = DriverState::OFFLINE;
  const uint8_t threshold = _config.offlineThreshold == 0 ? 1 : _config.offlineThreshold;
  if (_consecutiveFailures < threshold) {
    _consecutiveFailures = threshold;
  }
}

void BME280::_markHardwareConfigDirty(const Status& st) {
  if (st.ok() || st.inProgress()) {
    return;
  }
  if (!_hardwareConfigDirty) {
    _hardwareConfigDirtyError = st;
  }
  _hardwareConfigDirty = true;
}

void BME280::_clearHardwareConfigDirty() {
  _hardwareConfigDirty = false;
  _hardwareConfigDirtyError = Status::Ok();
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
    return Status::Error(Err::BUSY, "Device measuring; config write deferred");
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

  return Status::Error(Err::BUSY, "NVM update in progress",
                       static_cast<int32_t>(_config.nvmReadyTimeoutMs));
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

  _jobCalibration.digH2 = static_cast<int16_t>((calibH[1] << 8) | calibH[0]);
  _jobCalibration.digH3 = calibH[2];

  int16_t h4 = static_cast<int16_t>((calibH[3] << 4) | (calibH[4] & 0x0F));
  int16_t h5 = static_cast<int16_t>((calibH[5] << 4) | (calibH[4] >> 4));
  _jobCalibration.digH4 = signExtend12(h4);
  _jobCalibration.digH5 = signExtend12(h5);
  _jobCalibration.digH6 = static_cast<int8_t>(calibH[6]);

  return Status::Ok();
}

Status BME280::_readCalibration() {
  uint8_t calibTP[cmd::REG_CALIB_TP_LEN] = {};
  Status st = readRegs(cmd::REG_CALIB_TP_START, calibTP, sizeof(calibTP));
  if (!st.ok()) {
    return st;
  }

  const uint16_t digT1 = static_cast<uint16_t>((calibTP[1] << 8) | calibTP[0]);
  const int16_t digT2 = static_cast<int16_t>((calibTP[3] << 8) | calibTP[2]);
  const int16_t digT3 = static_cast<int16_t>((calibTP[5] << 8) | calibTP[4]);

  const uint16_t digP1 = static_cast<uint16_t>((calibTP[7] << 8) | calibTP[6]);
  const int16_t digP2 = static_cast<int16_t>((calibTP[9] << 8) | calibTP[8]);
  const int16_t digP3 = static_cast<int16_t>((calibTP[11] << 8) | calibTP[10]);
  const int16_t digP4 = static_cast<int16_t>((calibTP[13] << 8) | calibTP[12]);
  const int16_t digP5 = static_cast<int16_t>((calibTP[15] << 8) | calibTP[14]);
  const int16_t digP6 = static_cast<int16_t>((calibTP[17] << 8) | calibTP[16]);
  const int16_t digP7 = static_cast<int16_t>((calibTP[19] << 8) | calibTP[18]);
  const int16_t digP8 = static_cast<int16_t>((calibTP[21] << 8) | calibTP[20]);
  const int16_t digP9 = static_cast<int16_t>((calibTP[23] << 8) | calibTP[22]);

  uint8_t h1 = 0;
  st = readRegs(cmd::REG_CALIB_H1, &h1, 1);
  if (!st.ok()) {
    return st;
  }

  uint8_t calibH[cmd::REG_CALIB_H_LEN] = {};
  st = readRegs(cmd::REG_CALIB_H_START, calibH, sizeof(calibH));
  if (!st.ok()) {
    return st;
  }

  const int16_t digH2 = static_cast<int16_t>((calibH[1] << 8) | calibH[0]);
  const uint8_t digH3 = calibH[2];

  int16_t h4 = static_cast<int16_t>((calibH[3] << 4) | (calibH[4] & 0x0F));
  int16_t h5 = static_cast<int16_t>((calibH[5] << 4) | (calibH[4] >> 4));
  const int16_t digH4 = signExtend12(h4);
  const int16_t digH5 = signExtend12(h5);
  const int8_t digH6 = static_cast<int8_t>(calibH[6]);

  if (digT1 == 0 || digT1 == 0xFFFF) {
    return Status::Error(Err::CALIBRATION_INVALID, "Invalid temperature calibration");
  }
  if (digP1 == 0 || digP1 == 0xFFFF) {
    return Status::Error(Err::CALIBRATION_INVALID, "Invalid pressure calibration");
  }

  _digT1 = digT1;
  _digT2 = digT2;
  _digT3 = digT3;
  _digP1 = digP1;
  _digP2 = digP2;
  _digP3 = digP3;
  _digP4 = digP4;
  _digP5 = digP5;
  _digP6 = digP6;
  _digP7 = digP7;
  _digP8 = digP8;
  _digP9 = digP9;
  _digH1 = h1;
  _digH2 = digH2;
  _digH3 = digH3;
  _digH4 = digH4;
  _digH5 = digH5;
  _digH6 = digH6;

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

void BME280::_commitCalibration(const Calibration& calibration) {
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
}

Status BME280::_validateCalibration() {
  return _validateCalibrationValues(_digT1, _digP1);
}

Status BME280::_readRawData() {
  uint8_t data[cmd::DATA_LEN] = {};
  Status st = readRegs(cmd::REG_DATA_START, data, sizeof(data));
  if (!st.ok()) {
    return st;
  }

  _rawSample = RawSample{};
  _rawSample.adcP = (static_cast<int32_t>(data[0]) << 12) |
                    (static_cast<int32_t>(data[1]) << 4) |
                    (static_cast<int32_t>(data[2]) >> 4);
  _rawSample.adcT = (static_cast<int32_t>(data[3]) << 12) |
                    (static_cast<int32_t>(data[4]) << 4) |
                    (static_cast<int32_t>(data[5]) >> 4);
  _rawSample.adcH = (static_cast<int32_t>(data[6]) << 8) |
                    static_cast<int32_t>(data[7]);
  _rawSample.pressureValid = (_config.osrsP != Oversampling::SKIP) &&
                             (_rawSample.adcP != cmd::RAW_PRESSURE_SKIPPED);
  _rawSample.temperatureValid = (_config.osrsT != Oversampling::SKIP) &&
                                (_rawSample.adcT != cmd::RAW_TEMPERATURE_SKIPPED);
  _rawSample.humidityValid = (_config.osrsH != Oversampling::SKIP) &&
                             (_rawSample.adcH != cmd::RAW_HUMIDITY_SKIPPED);

  return Status::Ok();
}

Status BME280::_compensate() {
  _compSample = CompensatedSample{};
  const bool pressSkipped = (_config.osrsP == Oversampling::SKIP);
  const bool humSkipped = (_config.osrsH == Oversampling::SKIP);

  // Temperature compensation is required for pressure and humidity.
  if (!_rawSample.temperatureValid) {
    _tFine = 0;
    return Status::Error(Err::COMPENSATION_ERROR, "Temperature sample skipped");
  }

  // --- Temperature (Bosch int32 reference) ---
  const int32_t adcT = _rawSample.adcT;

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
    _tFine = 0;
    return Status::Error(Err::COMPENSATION_ERROR, "Temperature compensation overflow");
  }
  const int64_t tempC_x100 = (tFine64 * 5 + 128) >> 8;
  if (tempC_x100 < static_cast<int64_t>(std::numeric_limits<int32_t>::min()) ||
      tempC_x100 > static_cast<int64_t>(std::numeric_limits<int32_t>::max())) {
    _tFine = 0;
    return Status::Error(Err::COMPENSATION_ERROR, "Temperature output overflow");
  }

  _tFine = static_cast<int32_t>(tFine64);
  _compSample.tempC_x100 = static_cast<int32_t>(tempC_x100);
  _compSample.temperatureValid = true;

  // --- Pressure (Bosch int64 reference) ---
  if (pressSkipped) {
    _compSample.pressurePa = 0;
    _compSample.pressureValid = false;
  } else {
    if (!_rawSample.pressureValid) {
      return Status::Error(Err::COMPENSATION_ERROR, "Pressure sample skipped");
    }
    const int32_t adcP = _rawSample.adcP;

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
    int64_t pressurePa = p >> 8;
    if (pressurePa < 0) {
      pressurePa = 0;
    } else if (pressurePa > static_cast<int64_t>(std::numeric_limits<uint32_t>::max())) {
      pressurePa = static_cast<int64_t>(std::numeric_limits<uint32_t>::max());
    }
    _compSample.pressurePa = static_cast<uint32_t>(pressurePa);
    _compSample.pressureValid = true;
  }

  // --- Humidity (Bosch int32 reference, widened to int64 for safety) ---
  if (humSkipped) {
    _compSample.humidityPct_x1024 = 0;
    _compSample.humidityValid = false;
  } else {
    if (!_rawSample.humidityValid) {
      return Status::Error(Err::COMPENSATION_ERROR, "Humidity sample skipped");
    }
    const int32_t adcH = _rawSample.adcH;

    int64_t h = static_cast<int64_t>(_tFine) - 76800;
    const int64_t hTerm1 = (static_cast<int64_t>(adcH) << 14) -
                           (static_cast<int64_t>(_digH4) << 20) -
                           (static_cast<int64_t>(_digH5) * h) + 16384;
    int64_t hTerm2 = ((((h * static_cast<int64_t>(_digH6)) >> 10) *
                       (((h * static_cast<int64_t>(_digH3)) >> 11) + 32768)) >> 10) +
                     2097152;
    hTerm2 = ((hTerm2 * static_cast<int64_t>(_digH2)) + 8192) >> 14;
    h = (hTerm1 >> 15) * hTerm2;
    h = h - (((((h >> 15) * (h >> 15)) >> 7) *
              static_cast<int64_t>(_digH1)) >> 4);
    if (h < 0) {
      h = 0;
    }
    if (h > HUMIDITY_MAX_X4096) {
      h = HUMIDITY_MAX_X4096;
    }
    _compSample.humidityPct_x1024 = static_cast<uint32_t>(h >> 12);
    _compSample.humidityValid = true;
  }

  return Status::Ok();
}

void BME280::_invalidateSampleCache() {
  _measurementRequested = false;
  _measurementReady = false;
  _lastMeasurementStatus = Status::Ok();
  _hasSample = false;
  _measurementStartMs = 0;
  _sampleTimestampMs = 0;
  _tFine = 0;
  _rawSample = RawSample{};
  _compSample = CompensatedSample{};
}

uint32_t BME280::_nowMs() const {
  if (_config.nowMs != nullptr) {
    return _config.nowMs(_config.timeUser);
  }
  return platform::nowMs();
}

}  // namespace BME280
