/// @file BME280.h
/// @brief Framework-neutral BME280 I2C driver API.
///
/// The core driver owns no I2C bus, pins, locks, reset GPIOs, or platform
/// timers. Applications inject transport and time callbacks through
/// BME280::Config. Public APIs are task-context APIs; driver instances are not
/// internally thread-safe and are not ISR-safe.
#pragma once

#include <cstddef>
#include <cstdint>
#include "BME280/Status.h"
#include "BME280/Config.h"
#include "BME280/CommandTable.h"
#include "BME280/Version.h"

namespace BME280 {

/// Driver state for health monitoring
enum class DriverState : uint8_t {
  UNINIT,    ///< No successful session, end(), or an admitted initialization failed
  READY,     ///< Operational, consecutiveFailures == 0
  DEGRADED,  ///< 1 <= consecutiveFailures < offlineThreshold
  OFFLINE    ///< Diagnostic: consecutiveFailures >= offlineThreshold; does not block I2C
};

/// Return the canonical string for a driver health state.
/// @param value Driver state to describe.
/// @return Static canonical state name; invalid values return
///         `UNKNOWN_DRIVER_STATE`.
constexpr const char* toString(DriverState value) {
  switch (value) {
    case DriverState::UNINIT: return "UNINIT";
    case DriverState::READY: return "READY";
    case DriverState::DEGRADED: return "DEGRADED";
    case DriverState::OFFLINE: return "OFFLINE";
    default: return "UNKNOWN_DRIVER_STATE";
  }
}

/// Active staged job type.
enum class JobKind : uint8_t {
  NONE,                ///< No staged job is active
  INIT,                ///< Initialization job started by startInitJob()
  FORCED_MEASUREMENT,  ///< Forced conversion job started by startForcedMeasurementJob()
  APPLY_CONFIG,        ///< Config apply job started by startApplyConfigJob()
  RESYNC,              ///< Non-reset resynchronization started by startResyncJob()
  RECOVERY = RESYNC,   ///< Compatibility name for RESYNC
  SOFT_RESET           ///< Explicit reset and resynchronization job
};

/// Return the canonical string for a staged job kind.
/// @param value Job kind to describe.
/// @return Static canonical kind name; invalid values return `UNKNOWN_JOB_KIND`.
constexpr const char* toString(JobKind value) {
  switch (value) {
    case JobKind::NONE: return "NONE";
    case JobKind::INIT: return "INIT";
    case JobKind::FORCED_MEASUREMENT: return "FORCED_MEASUREMENT";
    case JobKind::APPLY_CONFIG: return "APPLY_CONFIG";
    case JobKind::RESYNC: return "RESYNC";
    case JobKind::SOFT_RESET: return "SOFT_RESET";
    default: return "UNKNOWN_JOB_KIND";
  }
}

/// State of the staged job runner.
enum class JobState : uint8_t {
  IDLE,     ///< No staged job is active
  RUNNING,  ///< Job can make more progress when instruction budget is available
  WAITING,  ///< Job is waiting for time or chip status before more progress
  DONE,     ///< Job completed successfully
  FAILED,   ///< Job stopped on error; inspect status
  CANCELLED, ///< Job was cancelled by its owner
  TIMED_OUT  ///< Job was cancelled because its owner deadline expired
};

/// Return the canonical string for a staged job state.
/// @param value Job state to describe.
/// @return Static canonical state name; invalid values return
///         `UNKNOWN_JOB_STATE`.
constexpr const char* toString(JobState value) {
  switch (value) {
    case JobState::IDLE: return "IDLE";
    case JobState::RUNNING: return "RUNNING";
    case JobState::WAITING: return "WAITING";
    case JobState::DONE: return "DONE";
    case JobState::FAILED: return "FAILED";
    case JobState::CANCELLED: return "CANCELLED";
    case JobState::TIMED_OUT: return "TIMED_OUT";
    default: return "UNKNOWN_JOB_STATE";
  }
}

/// Reason supplied by the owner when cancelling a staged job.
enum class CancelReason : uint8_t {
  OWNER_REQUEST,   ///< Owner no longer needs the operation
  DEADLINE_EXPIRED ///< Owner's external deadline expired
};

/// Return the canonical string for a cancellation reason.
/// @param value Cancellation reason to describe.
/// @return Static canonical reason name; invalid values return
///         `UNKNOWN_CANCEL_REASON`.
constexpr const char* toString(CancelReason value) {
  switch (value) {
    case CancelReason::OWNER_REQUEST: return "OWNER_REQUEST";
    case CancelReason::DEADLINE_EXPIRED: return "DEADLINE_EXPIRED";
    default: return "UNKNOWN_CANCEL_REASON";
  }
}

/// Knowledge of a forced-mode conversion that may be running in hardware.
enum class ConversionState : uint8_t {
  IDLE,                        ///< No forced conversion is known to be active
  IN_PROGRESS,                 ///< A trigger succeeded or status reports measuring
  UNKNOWN_AFTER_TRIGGER_ERROR  ///< A trigger/cancellation error left hardware ambiguous
};

/// Return the canonical string for forced-conversion knowledge.
/// @param value Conversion state to describe.
/// @return Static canonical state name; invalid values return
///         `UNKNOWN_CONVERSION_STATE`.
constexpr const char* toString(ConversionState value) {
  switch (value) {
    case ConversionState::IDLE: return "IDLE";
    case ConversionState::IN_PROGRESS: return "IN_PROGRESS";
    case ConversionState::UNKNOWN_AFTER_TRIGGER_ERROR:
      return "UNKNOWN_AFTER_TRIGGER_ERROR";
    default: return "UNKNOWN_CONVERSION_STATE";
  }
}

/// Reason encoded in Status::detail when a job start or hardware operation is busy.
enum class BusyReason : int32_t {
  NONE = 0,
  STAGED_JOB_ACTIVE = 1,      ///< A staged job currently owns hardware access
  TERMINAL_RESULT_PENDING = 2, ///< A cancellation result must be retrieved first
  MEASUREMENT_ACTIVE = 3,      ///< A measurement request is already active
  DEVICE_MEASURING = 4,        ///< Hardware is measuring during configuration
  NVM_UPDATE = 5,              ///< Hardware NVM copy is still active
  INVALID_JOB_STATE = 6,       ///< Staged runner encountered an invalid phase
  JOB_STATE_MACHINE_STALLED = 7 ///< Local transition guard was exhausted
};

/// Return the canonical string for a BUSY detail reason.
/// @param value Busy reason to describe.
/// @return Static canonical reason name; invalid values return
///         `UNKNOWN_BUSY_REASON`.
constexpr const char* toString(BusyReason value) {
  switch (value) {
    case BusyReason::NONE: return "NONE";
    case BusyReason::STAGED_JOB_ACTIVE: return "STAGED_JOB_ACTIVE";
    case BusyReason::TERMINAL_RESULT_PENDING: return "TERMINAL_RESULT_PENDING";
    case BusyReason::MEASUREMENT_ACTIVE: return "MEASUREMENT_ACTIVE";
    case BusyReason::DEVICE_MEASURING: return "DEVICE_MEASURING";
    case BusyReason::NVM_UPDATE: return "NVM_UPDATE";
    case BusyReason::INVALID_JOB_STATE: return "INVALID_JOB_STATE";
    case BusyReason::JOB_STATE_MACHINE_STALLED: return "JOB_STATE_MACHINE_STALLED";
    default: return "UNKNOWN_BUSY_REASON";
  }
}

/// Current phase of the fixed-memory staged state machine.
enum class JobPhase : uint8_t {
  NONE,                    ///< No staged phase is active
  INIT_READ_CHIP_ID,       ///< Initialization identity read
  INIT_NVM_START,          ///< Initialization NVM deadline setup
  NVM_POLL,                ///< Bounded NVM-copy readiness polling
  CALIB_TP,                ///< Temperature/pressure calibration burst read
  CALIB_H,                 ///< Humidity calibration burst read
  VALIDATE_CALIBRATION,    ///< Local-only calibration validation and commit
  APPLY_WAIT_IDLE,         ///< Wait for an existing conversion before settings apply
  APPLY_CTRL_MEAS_SLEEP,   ///< Request sleep before changing configuration
  APPLY_WAIT_AFTER_SLEEP,  ///< Verify the device is idle after the sleep request
  APPLY_CONFIG,            ///< Write filter and standby settings
  APPLY_CTRL_HUM,          ///< Write humidity oversampling
  APPLY_CTRL_MEAS,         ///< Latch oversampling and configured operating mode
  FORCE_RECONCILE_STATUS,  ///< Resolve an ambiguous prior trigger through status
  FORCE_TRIGGER,           ///< Issue exactly one forced ctrl_meas write
  FORCE_WAIT_TIME,         ///< Local-only wait for the estimated conversion time
  FORCE_READ_STATUS,       ///< Bounded conversion-ready status polling
  FORCE_READ_DATA,         ///< Coherent raw data burst read
  FORCE_COMPENSATE,        ///< Local-only validation, compensation, and sample commit
  SOFT_RESET_WRITE,        ///< Explicit soft-reset register write
  RESYNC_READ_CHIP_ID,     ///< Non-reset identity verification
  RESYNC_NVM_START,        ///< Resynchronization NVM deadline setup
  COMPLETE                 ///< Local-only successful terminal transition
};

/// Return the canonical string for a staged job phase.
/// @param value Job phase to describe.
/// @return Static canonical phase name; invalid values return
///         `UNKNOWN_JOB_PHASE`.
constexpr const char* toString(JobPhase value) {
  switch (value) {
    case JobPhase::NONE: return "NONE";
    case JobPhase::INIT_READ_CHIP_ID: return "INIT_READ_CHIP_ID";
    case JobPhase::INIT_NVM_START: return "INIT_NVM_START";
    case JobPhase::NVM_POLL: return "NVM_POLL";
    case JobPhase::CALIB_TP: return "CALIB_TP";
    case JobPhase::CALIB_H: return "CALIB_H";
    case JobPhase::VALIDATE_CALIBRATION: return "VALIDATE_CALIBRATION";
    case JobPhase::APPLY_WAIT_IDLE: return "APPLY_WAIT_IDLE";
    case JobPhase::APPLY_CTRL_MEAS_SLEEP: return "APPLY_CTRL_MEAS_SLEEP";
    case JobPhase::APPLY_WAIT_AFTER_SLEEP: return "APPLY_WAIT_AFTER_SLEEP";
    case JobPhase::APPLY_CONFIG: return "APPLY_CONFIG";
    case JobPhase::APPLY_CTRL_HUM: return "APPLY_CTRL_HUM";
    case JobPhase::APPLY_CTRL_MEAS: return "APPLY_CTRL_MEAS";
    case JobPhase::FORCE_RECONCILE_STATUS: return "FORCE_RECONCILE_STATUS";
    case JobPhase::FORCE_TRIGGER: return "FORCE_TRIGGER";
    case JobPhase::FORCE_WAIT_TIME: return "FORCE_WAIT_TIME";
    case JobPhase::FORCE_READ_STATUS: return "FORCE_READ_STATUS";
    case JobPhase::FORCE_READ_DATA: return "FORCE_READ_DATA";
    case JobPhase::FORCE_COMPENSATE: return "FORCE_COMPENSATE";
    case JobPhase::SOFT_RESET_WRITE: return "SOFT_RESET_WRITE";
    case JobPhase::RESYNC_READ_CHIP_ID: return "RESYNC_READ_CHIP_ID";
    case JobPhase::RESYNC_NVM_START: return "RESYNC_NVM_START";
    case JobPhase::COMPLETE: return "COMPLETE";
    default: return "UNKNOWN_JOB_PHASE";
  }
}

/// Result returned by pollJob().
struct JobPollResult {
  uint32_t jobId = 0;              ///< Nonzero identity, or zero when no result exists
  JobKind kind = JobKind::NONE;    ///< Kind associated with jobId
  JobPhase phase = JobPhase::NONE; ///< Phase at the return boundary
  JobState state = JobState::IDLE; ///< Job state after this poll
  Status status = Status::Ok();    ///< OK, IN_PROGRESS, or terminal error
  ConversionState conversionState = ConversionState::IDLE; ///< Post-poll forced state
  bool phaseDeadlineActive = false; ///< True when phaseDeadlineMs is meaningful
  uint32_t phaseDeadlineMs = 0;     ///< Active chip-phase deadline, not an owner deadline
  uint8_t callbacksUsed = 0;       ///< Transport callbacks issued by this poll
  uint8_t instructionsUsed = 0;    ///< Compatibility alias of callbacksUsed
};

/// Measurement result (float)
struct Measurement {
  float temperatureC = 0.0f; ///< Temperature in Celsius
  float pressurePa = 0.0f;   ///< Pressure in Pascals
  float humidityPct = 0.0f;  ///< Relative humidity in percent
  bool temperatureValid = false; ///< True when temperature was measured and compensated
  bool pressureValid = false;    ///< True when pressure was measured and compensated
  bool humidityValid = false;    ///< True when humidity was measured and compensated
};

/// Raw ADC values
struct RawSample {
  int32_t adcT = 0; ///< Raw temperature ADC (20-bit; 0x80000 when skipped)
  int32_t adcP = 0; ///< Raw pressure ADC (20-bit; 0x80000 when skipped)
  int32_t adcH = 0; ///< Raw humidity ADC (16-bit; 0x8000 when skipped)
  bool temperatureValid = false; ///< True when adcT is a measured value
  bool pressureValid = false;    ///< True when adcP is a measured value
  bool humidityValid = false;    ///< True when adcH is a measured value
};

/// Fixed-point compensated values (no float)
struct CompensatedSample {
  int32_t tempC_x100 = 0;        ///< Temperature * 100 (e.g., 2534 = 25.34 degC)
  uint32_t pressurePa = 0;       ///< Pressure in Pa
  uint32_t humidityPct_x1024 = 0; ///< Humidity * 1024 (Q22.10 format)
  bool temperatureValid = false; ///< True when tempC_x100 is usable
  bool pressureValid = false;    ///< True when pressurePa is usable
  bool humidityValid = false;    ///< True when humidityPct_x1024 is usable
};

/// Cached calibration coefficients from the device
struct Calibration {
  // Temperature
  uint16_t digT1 = 0; ///< Temperature compensation coefficient T1
  int16_t digT2 = 0;  ///< Temperature compensation coefficient T2
  int16_t digT3 = 0;  ///< Temperature compensation coefficient T3
  // Pressure
  uint16_t digP1 = 0; ///< Pressure compensation coefficient P1
  int16_t digP2 = 0;  ///< Pressure compensation coefficient P2
  int16_t digP3 = 0;  ///< Pressure compensation coefficient P3
  int16_t digP4 = 0;  ///< Pressure compensation coefficient P4
  int16_t digP5 = 0;  ///< Pressure compensation coefficient P5
  int16_t digP6 = 0;  ///< Pressure compensation coefficient P6
  int16_t digP7 = 0;  ///< Pressure compensation coefficient P7
  int16_t digP8 = 0;  ///< Pressure compensation coefficient P8
  int16_t digP9 = 0;  ///< Pressure compensation coefficient P9
  // Humidity
  uint8_t digH1 = 0;  ///< Humidity compensation coefficient H1
  int16_t digH2 = 0;  ///< Humidity compensation coefficient H2
  uint8_t digH3 = 0;  ///< Humidity compensation coefficient H3
  int16_t digH4 = 0;  ///< Humidity compensation coefficient H4
  int16_t digH5 = 0;  ///< Humidity compensation coefficient H5
  int8_t digH6 = 0;   ///< Humidity compensation coefficient H6
};

/// Raw calibration register blocks
struct CalibrationRaw {
  /// Raw 0x88..0xA1 burst; tp[25] is register 0xA1 / dig_H1.
  uint8_t tp[cmd::REG_CALIB_TP_LEN] = {};
  uint8_t h[cmd::REG_CALIB_H_LEN] = {};   ///< Raw 0xE1..0xE7 humidity block
};

/// Freshness classification for the latest cached sample.
enum class SampleFreshness : uint8_t {
  NONE,                    ///< No cached sample exists
  FRESH,                   ///< Cached sample is clean and latest measurement status is OK
  STALE_AFTER_ERROR,       ///< Cached sample exists but latest measurement status is not OK
  STALE_AFTER_CONFIG_DIRTY, ///< Cached sample exists but hardware config may be out of sync
  STALE_AFTER_CONFIG_CHANGE ///< Cached sample belongs to an older configuration generation
};

/// Return the canonical string for cached-sample freshness.
/// @param value Freshness state to describe.
/// @return Static canonical state name; invalid values return
///         `UNKNOWN_SAMPLE_FRESHNESS`.
constexpr const char* toString(SampleFreshness value) {
  switch (value) {
    case SampleFreshness::NONE: return "NONE";
    case SampleFreshness::FRESH: return "FRESH";
    case SampleFreshness::STALE_AFTER_ERROR: return "STALE_AFTER_ERROR";
    case SampleFreshness::STALE_AFTER_CONFIG_DIRTY:
      return "STALE_AFTER_CONFIG_DIRTY";
    case SampleFreshness::STALE_AFTER_CONFIG_CHANGE:
      return "STALE_AFTER_CONFIG_CHANGE";
    default: return "UNKNOWN_SAMPLE_FRESHNESS";
  }
}

/// Synchronization state between cached settings and device registers.
enum class ConfigSyncState : uint8_t {
  SYNCHRONIZED,       ///< Cached settings are fully applied to the device
  UPDATE_IN_PROGRESS, ///< A multi-step settings update is active
  RESYNC_REQUIRED     ///< Device settings may differ from the cached settings
};

/// Return the canonical string for cached/hardware synchronization state.
/// @param value Synchronization state to describe.
/// @return Static canonical state name; invalid values return
///         `UNKNOWN_CONFIG_SYNC_STATE`.
constexpr const char* toString(ConfigSyncState value) {
  switch (value) {
    case ConfigSyncState::SYNCHRONIZED: return "SYNCHRONIZED";
    case ConfigSyncState::UPDATE_IN_PROGRESS: return "UPDATE_IN_PROGRESS";
    case ConfigSyncState::RESYNC_REQUIRED: return "RESYNC_REQUIRED";
    default: return "UNKNOWN_CONFIG_SYNC_STATE";
  }
}

/// Validity of the cached device-specific compensation coefficients.
enum class CalibrationState : uint8_t {
  INVALID, ///< Calibration must be reloaded before measurement
  VALID    ///< Calibration is valid for the configured measurement channels
};

/// Return the canonical string for cached calibration validity.
/// @param value Calibration state to describe.
/// @return Static canonical state name; invalid values return
///         `UNKNOWN_CALIBRATION_STATE`.
constexpr const char* toString(CalibrationState value) {
  switch (value) {
    case CalibrationState::INVALID: return "INVALID";
    case CalibrationState::VALID: return "VALID";
    default: return "UNKNOWN_CALIBRATION_STATE";
  }
}

/// Atomically committed sample and its provenance.
struct SampleEnvelope {
  RawSample rawSample = {};                  ///< Coherent raw ADC sample
  CompensatedSample compensatedSample = {}; ///< Values compensated from rawSample
  int32_t tFine = 0;                         ///< Temperature intermediate for this sample
  uint32_t timestampMs = 0;                  ///< Caller time when the sample was committed
  uint32_t sampleSequence = 0;               ///< Nonzero sequence of the committed sample
  uint32_t configGeneration = 0;             ///< Settings generation used for compensation
};

/// Snapshot of driver configuration and runtime state without I2C access.
struct SettingsSnapshot {
  bool initialized = false;                   ///< True after begin() succeeds
  DriverState state = DriverState::UNINIT;    ///< Current driver state
  uint8_t i2cAddress = 0x76;                  ///< Active 7-bit I2C address
  uint32_t i2cTimeoutMs = 0;                  ///< Active I2C timeout
  uint32_t nvmReadyTimeoutMs = 0;             ///< Active NVM ready timeout
  uint32_t conversionReadyTimeoutMs = 0;      ///< Conversion/idle readiness grace period
  uint8_t offlineThreshold = 0;               ///< Failure threshold for OFFLINE
  bool hasNowMsHook = false;                  ///< True when Config::nowMs is set
  Mode mode = Mode::SLEEP;                    ///< Active measurement mode
  Oversampling osrsT = Oversampling::SKIP;    ///< Temperature oversampling
  Oversampling osrsP = Oversampling::SKIP;    ///< Pressure oversampling
  Oversampling osrsH = Oversampling::SKIP;    ///< Humidity oversampling
  Filter filter = Filter::OFF;                ///< IIR filter coefficient
  Standby standby = Standby::MS_0_5;         ///< Standby time for normal mode
  bool measurementRequested = false;          ///< True while the scheduler has a pending capture
  bool measurementReady = false;              ///< True when an unread cached measurement is ready
  ConversionState conversionState = ConversionState::IDLE; ///< Forced-conversion knowledge
  Status lastMeasurementStatus = Status::Ok(); ///< Last request/tick status for measurement scheduling
  bool hasSample = false;                     ///< True when a compensated sample is cached
  SampleFreshness sampleFreshness = SampleFreshness::NONE; ///< Freshness of the cached sample
  ConfigSyncState configSyncState = ConfigSyncState::RESYNC_REQUIRED; ///< Cached/device config relation
  CalibrationState calibrationState = CalibrationState::INVALID; ///< Cached calibration validity
  bool hardwareConfigDirty = false;           ///< True when hardware config may differ from cache
  Status hardwareConfigDirtyError = Status::Ok(); ///< First error that made config state uncertain
  uint32_t configGeneration = 0;               ///< Current synchronized config generation
  uint32_t sampleSequence = 0;                 ///< Sequence of the latest committed sample
  uint32_t sampleConfigGeneration = 0;         ///< Settings generation tagged on the sample
  uint32_t measurementStartMs = 0;            ///< Timestamp of last measurement trigger
  uint32_t sampleTimestampMs = 0;             ///< Timestamp of the last cached sample
  int32_t tFine = 0;                          ///< Last t_fine intermediate value
  RawSample rawSample = {};                   ///< Last raw ADC sample
  CompensatedSample compSample = {};          ///< Last compensated sample
  SampleEnvelope sample = {};                 ///< Atomic sample/provenance snapshot
  Calibration calibration = {};               ///< Cached compensation coefficients
  bool lastOkTimeValid = false;                ///< True when lastOkMs() has a real timebase
  bool lastErrorTimeValid = false;             ///< True when lastErrorMs() has a real timebase
};

/// BME280 driver class.
///
/// Instances are not internally thread-safe and public APIs are not ISR-safe.
/// Applications sharing a driver or I2C bus across tasks must serialize access
/// externally. Transport callbacks must not recursively call into the same
/// driver instance.
///
/// Device contract:
/// - Supported I2C addresses are 0x76 and 0x77.
/// - begin() and probe() verify BME280 identity by reading chip ID 0x60.
/// - The application owns I2C bus setup, pins, pull-ups, locks, reset/power
///   control, timeout policy, and the monotonic clock used by Config::nowMs.
/// - Multi-register configuration failures are reported through
///   hardwareConfigDirty() and hardwareConfigDirtyError().
///
/// Hardware-operation admission contract:
/// - A running/waiting staged job exclusively owns hardware access.
/// - A retained cancellation result must be retrieved exactly once with
///   pollJob() before another hardware-facing operation can start.
/// - Fallible APIs that could access hardware return BUSY without I2C in either
///   case. Status::detail is BusyReason::STAGED_JOB_ACTIVE or
///   BusyReason::TERMINAL_RESULT_PENDING respectively.
class BME280 {
public:
  /// Construct an uninitialized driver instance.
  BME280() = default;

  /// Destroy the driver object. Does not perform I2C; call end() for cleanup.
  ~BME280() = default;

  /// Driver instances are not copyable because they hold runtime state.
  /// @param other Other driver instance.
  BME280(const BME280& other) = delete;

  /// Driver instances are not copy-assignable because they hold runtime state.
  /// @param other Other driver instance.
  /// @return Reference to this driver.
  BME280& operator=(const BME280& other) = delete;

  /// Driver instances are not movable because transport callbacks are non-owning.
  /// @param other Other driver instance.
  BME280(BME280&& other) = delete;

  /// Driver instances are not move-assignable because transport callbacks are non-owning.
  /// @param other Other driver instance.
  /// @return Reference to this driver.
  BME280& operator=(BME280&& other) = delete;

  // =========================================================================
  // Lifecycle
  // =========================================================================
  
  /// Initialize the driver with configuration.
  /// Once admitted, calling this on an initialized instance clears its local
  /// runtime state before configuration validation or I2C. Any later failure
  /// leaves the instance uninitialized; previous cached state is not restored.
  /// Verifies chip ID 0x60, checks NVM readiness once, reads calibration,
  /// validates coefficients, and applies cached config. Call after device POR
  /// and I2C bus readiness; address NACK maps to DEVICE_NOT_FOUND while other
  /// transport errors are preserved. Starts a new health session and resets
  /// tracked I2C counters. If NVM is still busy, returns BUSY or TIMEOUT
  /// instead of hiding a polling loop; use startInitJob()/pollJob() when the
  /// owner needs staged NVM polling.
  /// @param config Configuration including transport callbacks
  /// @return Status::Ok() on success, error otherwise
  Status begin(const Config& config);
  
  /// Process pending measurement operations (call regularly from task context).
  /// @param nowMs Current monotonic timestamp in milliseconds from the same
  ///              timebase as Config::nowMs
  /// Measurement errors are retained in lastMeasurementStatus() and
  /// SettingsSnapshot::lastMeasurementStatus because tick() itself is void.
  /// A status-read, raw-read, or compensation failure terminates the current
  /// request; later tick() calls perform no I2C until requestMeasurement() is
  /// called again.
  void tick(uint32_t nowMs);
  
  /// Unbind the driver and clear all cached state without performing I2C.
  /// This operation is idempotent and may be used during teardown even when a
  /// staged job is active.
  void end();

  /// Check whether the current initialization session is valid.
  /// A later admitted begin() or startInitJob() clears the prior session; if
  /// that initialization fails, this remains false until initialization
  /// succeeds again.
  /// @return true only while the current initialization session is valid
  bool isInitialized() const { return _initialized; }

  /// Get the cached configuration snapshot currently owned by the driver
  /// @return Const reference to the active driver configuration copy
  const Config& getConfig() const { return _config; }

  /// Return the compact cached sensor settings without accessing I2C.
  /// During startApplySettingsJob(), this reports the staged desired settings;
  /// an untouched failed/cancelled apply restores the prior snapshot.
  /// @return Current cached sensor settings by value
  SensorSettings sensorSettings() const;

  // =========================================================================
  // Staged I2C Jobs
  // =========================================================================

  /// Start a staged initialization job. Poll with pollJob() until DONE/FAILED.
  /// Like begin(), this clears prior local runtime state before validating the
  /// new configuration. Admission failure caused by an active operation or a
  /// pending retained cancellation result preserves the current session.
  /// Successful completion starts a new health session and clears dirty config
  /// state after the cached configuration has been applied.
  /// @param config Configuration including transport callbacks
  /// @return IN_PROGRESS when accepted, error otherwise
  Status startInitJob(const Config& config);

  /// Start a staged forced-mode measurement job.
  /// @return IN_PROGRESS when accepted, RESYNC_REQUIRED when cached device
  ///         state is not valid for measurement, or another error
  Status startForcedMeasurementJob();

  /// Start a staged re-apply of the cached configuration.
  /// Partial write failures preserve dirty config diagnostics; successful
  /// completion clears dirty config state.
  /// @return IN_PROGRESS when accepted, error otherwise
  Status startApplyConfigJob();

  /// Validate and stage a desired settings snapshot using the existing
  /// APPLY_CONFIG state machine. This start performs no transport callback.
  /// Before the first mutating/ambiguous write, failure or cancellation restores
  /// prior cached settings and sync state. Afterwards desired settings remain
  /// cached with RESYNC_REQUIRED until a successful full apply/resync.
  /// @param settings Desired sensor settings
  /// @return IN_PROGRESS when accepted, validation/precondition error otherwise
  Status startApplySettingsJob(const SensorSettings& settings);

  /// Start a staged non-reset resynchronization job. The job verifies identity
  /// and NVM readiness, reloads calibration, and reapplies cached settings.
  /// @return IN_PROGRESS when accepted, error otherwise
  Status startResyncJob();

  /// Compatibility alias for startResyncJob(). This method does not reset the
  /// sensor and reports JobKind::RESYNC.
  /// @return IN_PROGRESS when accepted, error otherwise
  Status startRecoveryJob();

  /// Start an explicit staged soft reset followed by full resynchronization.
  /// The first transport callback writes exactly reset value 0xB6 to register
  /// 0xE0. Reset uncertainty invalidates cached calibration.
  /// @return IN_PROGRESS when accepted, error otherwise
  Status startSoftResetJob();

  /// Cancel the active job without accessing I2C. Cancellation is delivered
  /// exactly once by pollJob(); a new start remains BUSY until that retrieval.
  /// @param reason Owner-requested or owner-deadline cancellation reason
  /// @return CANCELLED/DEADLINE_EXPIRED when accepted, INVALID_PARAM otherwise
  Status cancelJob(CancelReason reason);

  /// Advance the active staged job.
  /// @param nowMs Current timestamp in milliseconds
  /// A zero callback budget still permits bounded local-only phase transitions.
  /// Natural terminal results are returned only by the poll that reaches them;
  /// cancellation results are retained until exactly one poll retrieves them.
  /// @param maxInstructions Maximum I2C callbacks to issue this poll; defaults to 1
  /// @return Identity, phase, state, status, deadline and callback usage
  JobPollResult pollJob(uint32_t nowMs, uint8_t maxInstructions = 1);

  /// Current staged job type.
  /// @return Active/last terminal job kind, or JobKind::NONE.
  JobKind jobKind() const { return _jobKind; }

  /// Current staged job state.
  /// @return Current staged runner state.
  JobState jobState() const { return _jobState; }

  /// Last staged job status.
  /// @return Current or most recent staged job status.
  Status jobStatus() const { return _jobStatus; }

  /// Identity of the active or last terminal staged job.
  /// @return Nonzero accepted job identity, or zero before any accepted job.
  uint32_t jobId() const { return _jobId; }

  /// Current staged job phase.
  /// @return Current staged state-machine phase.
  JobPhase jobPhase() const { return _jobPhase; }
  
  // =========================================================================
  // Diagnostics
  // =========================================================================
  
  /// Check if device is present on the bus (raw I2C, no health tracking).
  /// Address NACK maps to DEVICE_NOT_FOUND; other transport errors and chip-ID
  /// mismatch are preserved. Does not clear OFFLINE.
  /// Requires a successful begin() so the transport callbacks and address are
  /// configured.
  /// @return Status::Ok() if device responds with chip ID 0x60, error otherwise
  Status probe();
  
  /// Attempt to recover from DEGRADED/OFFLINE state by verifying chip ID,
  /// checking NVM readiness once, reloading calibration, validating it, and
  /// reapplying cached config.
  /// A successful recovery invalidates cached raw/compensated samples and any
  /// pending measurement state; a failed recovery leaves pre-existing cached
  /// samples unchanged.
  /// @return Status::Ok() if device now responsive, error otherwise
  Status recover();

  /// Populate a snapshot of cached configuration and runtime state without I2C.
  /// @param[out] out Snapshot to fill
  /// @return Status::Ok() always
  Status getSettings(SettingsSnapshot& out) const;

  /// True while a multi-step update is in progress, or when a failed or
  /// uncertain config/resync/reset operation may have left sensor registers
  /// different from cached settings. Cleared only by a complete successful
  /// configuration resync.
  /// @return true while configuration is updating or requires a full resync
  bool hardwareConfigDirty() const {
    return _configSyncState == ConfigSyncState::UPDATE_IN_PROGRESS ||
           (_configSyncState == ConfigSyncState::RESYNC_REQUIRED &&
            !_hardwareConfigDirtyError.ok());
  }

  /// First transport/status error that made hardware config state uncertain.
  /// @return Root-cause status for hardwareConfigDirty(), or Status::Ok()
  Status hardwareConfigDirtyError() const { return _hardwareConfigDirtyError; }

  /// Current synchronization state between cached settings and hardware.
  /// @return Cached/hardware configuration synchronization state.
  ConfigSyncState configSyncState() const { return _configSyncState; }

  /// Current cached-calibration validity.
  /// @return Current cached calibration validity state.
  CalibrationState calibrationState() const { return _calibrationState; }

  /// Current cached-settings generation. Zero means no apply has completed.
  /// @return Current settings generation, or zero before a completed apply.
  uint32_t configGeneration() const { return _configGeneration; }

  /// Last measurement scheduler/capture status recorded by requestMeasurement()
  /// or tick(). This is Status::Ok() after a sample is captured, IN_PROGRESS
  /// while a request is pending, and the original error when polling, raw read,
  /// or compensation fails.
  /// @return Last measurement scheduling or capture status
  Status lastMeasurementStatus() const { return _lastMeasurementStatus; }
  
  // =========================================================================
  // Driver State
  // =========================================================================
  
  /// Get current driver state
  /// @return Current health/lifecycle state
  DriverState state() const { return _driverState; }

  /// Alias for state() used by shared diagnostics.
  /// @return Current health/lifecycle state
  DriverState driverState() const { return state(); }
  
  /// Check if driver is ready for operations
  /// This is an observational health classification; OFFLINE does not block an
  /// explicit owner-directed operation.
  /// @return true for READY or DEGRADED, false for UNINIT or OFFLINE
  bool isOnline() const { 
    return _driverState == DriverState::READY || 
           _driverState == DriverState::DEGRADED; 
  }
  
  // =========================================================================
  // Health Tracking
  // =========================================================================
  
  /// Timestamp of last successful I2C operation
  /// @return Last successful tracked I2C timestamp in milliseconds, or 0
  uint32_t lastOkMs() const { return _lastOkMs; }

  /// True when lastOkMs() was captured from Config::nowMs or explicit poll/tick time.
  /// @return true when lastOkMs() came from a real monotonic time source.
  bool lastOkTimeValid() const { return _lastOkTimeValid; }
  
  /// Timestamp of last failed I2C operation
  /// @return Last failed tracked I2C timestamp in milliseconds, or 0
  uint32_t lastErrorMs() const { return _lastErrorMs; }

  /// True when lastErrorMs() was captured from a real injected/explicit timebase.
  /// @return true when lastErrorMs() came from a real monotonic time source.
  bool lastErrorTimeValid() const { return _lastErrorTimeValid; }
  
  /// Most recent tracked transport or device-resynchronization error.
  /// @return Last health-recorded error status.
  Status lastError() const { return _lastError; }
  
  /// Consecutive health-recorded failures since the last tracked transfer
  /// success. Recovery/reset semantic failures may be recorded after a
  /// successful transfer when identity, calibration, NVM, or configuration
  /// readiness is invalid.
  /// Saturates at UINT8_MAX.
  /// @return Consecutive health-recorded failures.
  uint8_t consecutiveFailures() const { return _consecutiveFailures; }
  
  /// Total transport and device-resynchronization failures recorded in the
  /// current health session.
  /// Saturates at UINT32_MAX and is reset by begin(); it does not wrap.
  /// @return Tracked failure count since the most recent begin()
  uint32_t totalFailures() const { return _totalFailures; }
  
  /// Total successful tracked transport callbacks in the current health
  /// session.
  /// Saturates at UINT32_MAX and is reset by begin(); it does not wrap.
  /// @return Tracked success count since the most recent begin()
  uint32_t totalSuccess() const { return _totalSuccess; }
  
  // =========================================================================
  // Measurement API
  // =========================================================================
  
  /// Request a measurement (non-blocking).
  /// In FORCED mode: triggers measurement if idle.
  /// If an existing forced conversion is detected, the scheduler reserves one
  /// complete configured conversion interval plus readiness grace from the
  /// detection time. In NORMAL mode it reserves the worst phase interval (two
  /// conversions plus one standby interval) before reading, so the sample is
  /// fresh relative to the request even when the request arrives just after a
  /// conversion starts. All intervals use wrap-safe unsigned deadlines.
  /// Returns IN_PROGRESS if a request is accepted or an already-running forced
  /// conversion can be tracked, BUSY if a driver request is already pending,
  /// or admission BUSY as described by the class hardware-operation contract,
  /// INVALID_CONFIG if Config::nowMs is missing, or
  /// INVALID_PARAM in sleep mode. Returns RESYNC_REQUIRED without I2C when
  /// cached configuration or calibration is not synchronized with the device.
  /// @return Scheduling status
  Status requestMeasurement();

  /// Check if measurement is ready to read
  /// @return true when getMeasurement() can consume a pending fresh sample
  bool measurementReady() const { return _measurementReady; }

  /// Current knowledge of forced-mode conversion activity.
  /// @return Current forced-conversion knowledge state.
  ConversionState conversionState() const { return _conversionState; }

  /// True after at least one sample has been cached.
  /// @return true when raw and compensated cached sample data exists
  bool hasSample() const { return _hasSample; }

  /// Timestamp of the last cached sample, or 0 if none exists.
  /// @return Last cached sample timestamp in milliseconds
  uint32_t sampleTimestampMs() const { return _sampleTimestampMs; }

  /// Sequence number of the latest committed sample, or zero before capture.
  /// @return Latest nonzero committed sequence, or zero when no sample exists.
  uint32_t sampleSequence() const { return _hasSample ? _sampleSequence : 0; }

  /// Age of the cached sample in milliseconds.
  /// @param nowMs Current monotonic timestamp in milliseconds
  /// @return `nowMs - sampleTimestampMs()` when a sample exists, otherwise 0
  uint32_t sampleAgeMs(uint32_t nowMs) const {
    return _hasSample ? (nowMs - _sampleTimestampMs) : 0;
  }

  /// Classify the cached sample without touching I2C.
  /// @return Freshness state for the latest cached raw/compensated sample
  SampleFreshness sampleFreshness() const;

  /// Check whether the cached sample is fresh and within an age budget.
  /// @param nowMs Current monotonic timestamp in milliseconds
  /// @param maxAgeMs Maximum acceptable cached-sample age in milliseconds
  /// @return true only when sampleFreshness() is FRESH and age is within budget
  bool sampleFresh(uint32_t nowMs, uint32_t maxAgeMs) const;

  /// Get measurement result (float).
  /// Returns MEASUREMENT_NOT_READY until an unread fresh measurement is ready.
  /// Dirty configuration, a configuration-generation change, or a terminal
  /// refresh error prevents stale cached data from being returned as success.
  /// Clears ready flag after successful read
  /// Does not invalidate cached raw/fixed-point samples.
  /// Numeric fields remain zero for skipped/invalid channels; check the
  /// matching validity flag before using a channel.
  /// @param[out] out Last cached compensated measurement as floats.
  /// @return Status::Ok() only for a fresh unread sample, otherwise
  ///         MEASUREMENT_NOT_READY.
  Status getMeasurement(Measurement& out);

  /// Get raw ADC values.
  /// @param[out] out Last cached raw ADC sample
  /// @return Status::Ok() on success, MEASUREMENT_NOT_READY until a sample has been captured
  /// Channels skipped by configuration or reported as Bosch skipped sentinels
  /// have their validity flag set false.
  Status getRawSample(RawSample& out) const;

  /// Get fixed-point compensated values.
  /// @param[out] out Last cached fixed-point compensated sample
  /// @return Status::Ok() on success, MEASUREMENT_NOT_READY until a sample has been captured
  /// Numeric fields remain zero for skipped/invalid channels; check the
  /// matching validity flag before using a channel.
  Status getCompensatedSample(CompensatedSample& out) const;

  /// Get the atomically committed sample and its provenance.
  /// @param[out] out Last committed sample envelope
  /// @return Status::Ok() on success, MEASUREMENT_NOT_READY before capture
  Status getSampleEnvelope(SampleEnvelope& out) const;

  /// Get cached calibration coefficients.
  /// @param[out] out Cached coefficients read by the most recent successful
  ///             synchronous or staged initialization, resynchronization, or reset
  /// @return Status::Ok() on success, NOT_INITIALIZED before begin(), or
  ///         RESYNC_REQUIRED after device-state invalidation
  Status getCalibration(Calibration& out) const;

  /// Invalidate cached device-specific state without accessing I2C.
  /// Preserves the last sample for explicitly stale diagnostics. A later
  /// initialization or resynchronization must reload calibration and fully
  /// apply settings before another measurement can start.
  /// @return Status::Ok(), or admission BUSY as described by the class
  ///         hardware-operation contract
  Status invalidateDeviceState();

  /// Read raw calibration registers using exactly two bounded bursts:
  /// 0x88..0xA1 and 0xE1..0xE7. Transport errors are preserved.
  /// @param[out] out Raw register blocks
  /// @return Status::Ok() on success, error otherwise
  Status readCalibrationRaw(CalibrationRaw& out);

  // =========================================================================
  // Configuration
  // =========================================================================

  /// Set operating mode (SLEEP, FORCED, NORMAL).
  /// FORCED is an on-demand policy and does not trigger a conversion until
  /// requestMeasurement() is called. A successful mode change invalidates the
  /// cached sample.
  /// @param mode New operating mode
  /// @return Status::Ok() on success, error otherwise
  Status setMode(Mode mode);

  /// Get current mode
  /// @param[out] out Cached mode
  /// @return Status::Ok() on success, NOT_INITIALIZED before begin()
  Status getMode(Mode& out) const;

  /// Set oversampling for temperature.
  /// Temperature must be enabled when pressure or humidity is enabled. A
  /// successful change invalidates the cached sample.
  /// @param osrs New temperature oversampling
  /// @return Status::Ok() on success, error otherwise
  Status setOversamplingT(Oversampling osrs);

  /// Set oversampling for pressure.
  /// Temperature must be enabled when pressure is enabled. A successful change
  /// invalidates the cached sample.
  /// @param osrs New pressure oversampling
  /// @return Status::Ok() on success, error otherwise
  Status setOversamplingP(Oversampling osrs);

  /// Set oversampling for humidity.
  /// Temperature must be enabled when humidity is enabled. The hardware
  /// sequence writes ctrl_hum first, then ctrl_meas to latch the humidity
  /// setting. A successful change invalidates the cached sample.
  /// @param osrs New humidity oversampling
  /// @return Status::Ok() on success, error otherwise
  Status setOversamplingH(Oversampling osrs);

  /// Set IIR filter coefficient. The driver verifies the device is not
  /// currently measuring, switches to sleep, verifies again before writing
  /// config, then restores the cached mode. A successful change invalidates the
  /// cached sample. The BME280 IIR filter applies to pressure and temperature,
  /// not humidity, and changing it resets the hardware filter memory. If
  /// measuring appears after the sleep write, config is skipped and dirty state
  /// is set.
  /// @param filter New IIR filter coefficient
  /// @return Status::Ok() on success, error otherwise
  Status setFilter(Filter filter);

  /// Set standby time (normal mode only). The driver verifies the device is not
  /// currently measuring, switches to sleep, verifies again before writing
  /// config, then restores the cached mode. A successful change invalidates the
  /// cached sample. If measuring appears after the sleep write, config is
  /// skipped and dirty state is set.
  /// @param standby New normal-mode standby interval
  /// @return Status::Ok() on success, error otherwise
  Status setStandby(Standby standby);

  /// Get oversampling for temperature
  /// @param[out] out Cached temperature oversampling
  /// @return Status::Ok() on success, NOT_INITIALIZED before begin()
  Status getOversamplingT(Oversampling& out) const;

  /// Get oversampling for pressure
  /// @param[out] out Cached pressure oversampling
  /// @return Status::Ok() on success, NOT_INITIALIZED before begin()
  Status getOversamplingP(Oversampling& out) const;

  /// Get oversampling for humidity
  /// @param[out] out Cached humidity oversampling
  /// @return Status::Ok() on success, NOT_INITIALIZED before begin()
  Status getOversamplingH(Oversampling& out) const;

  /// Get IIR filter coefficient
  /// @param[out] out Cached IIR filter coefficient
  /// @return Status::Ok() on success, NOT_INITIALIZED before begin()
  Status getFilter(Filter& out) const;

  /// Get standby time
  /// @param[out] out Cached standby interval enum
  /// @return Status::Ok() on success, NOT_INITIALIZED before begin()
  Status getStandby(Standby& out) const;

  /// Soft reset device. Writes 0xB6 to 0xE0, checks status.im_update once,
  /// reloads calibration, validates it, and reapplies cached config. A reset
  /// attempt invalidates cached samples before touching hardware. If reset
  /// write succeeds but a later step fails, hardwareConfigDirty() remains set
  /// with the root-cause status. If NVM is still busy, returns BUSY or TIMEOUT
  /// instead of hiding a polling loop; use startSoftResetJob()/pollJob() when
  /// the owner needs staged NVM polling.
  /// @return Status::Ok() on success, error otherwise
  Status softReset();

  /// Read chip ID
  /// @param[out] id Chip-ID register value
  /// @return Status::Ok() on success, error otherwise
  Status readChipId(uint8_t& id);

  /// Read status register
  /// @param[out] status Status register value
  /// @return Status::Ok() on success, error otherwise
  Status readStatus(uint8_t& status);

  /// Read ctrl_hum register
  /// @param[out] value ctrl_hum register value
  /// @return Status::Ok() on success, error otherwise
  Status readCtrlHum(uint8_t& value);

  /// Read ctrl_meas register
  /// @param[out] value ctrl_meas register value
  /// @return Status::Ok() on success, error otherwise
  Status readCtrlMeas(uint8_t& value);

  /// Read config register
  /// @param[out] value config register value
  /// @return Status::Ok() on success, error otherwise
  Status readConfig(uint8_t& value);

  /// Check if device is currently measuring
  /// @param[out] measuring true when status.measuring is set
  /// @return Status::Ok() on success, error otherwise
  Status isMeasuring(bool& measuring);

  // =========================================================================
  // Raw Register Access
  // =========================================================================

  /// Read a contiguous register block through tracked I2C.
  /// @param startReg First register address to read
  /// @param[out] buf Destination buffer; must not be null
  /// @param len Number of bytes to read; must be nonzero
  /// @return Status::Ok() on success, NOT_INITIALIZED before begin(),
  ///         INVALID_PARAM for null/zero buffers, admission BUSY as described
  ///         by the class contract, or the original tracked transport status.
  Status readRegisters(uint8_t startReg, uint8_t* buf, size_t len);

  /// Write a contiguous register block through tracked I2C.
  /// @param startReg First register address to write
  /// @param buf Source buffer; must not be null
  /// @param len Number of bytes to write; must be nonzero and fit the internal
  ///            bounded stack payload
  /// @return Status::Ok() on success, NOT_INITIALIZED before begin(),
  ///         INVALID_PARAM for null/zero/oversized writes, admission BUSY as
  ///         described by the class contract, or the original tracked
  ///         transport status.
  /// @note Diagnostic writes that overlap ctrl_hum (0xF2), ctrl_meas (0xF4),
  ///       config (0xF5), or reset (0xE0) mark hardwareConfigDirty() on
  ///       success. Transport failures that may have partially reached those
  ///       registers preserve the original status as hardwareConfigDirtyError().
  ///       Call recover(), begin(), or a successful softReset() to resync after
  ///       manual config-register edits.
  Status writeRegisters(uint8_t startReg, const uint8_t* buf, size_t len);

  /// Read a single register through tracked I2C.
  /// @param reg Register address to read
  /// @param[out] value Register value
  /// @return Status::Ok() on success, NOT_INITIALIZED before begin(), admission
  ///         BUSY as described by the class contract, or the original transport
  ///         status.
  Status readRegister(uint8_t reg, uint8_t& value);

  /// Write a single register through tracked I2C.
  /// @param reg Register address to write
  /// @param value Value to write
  /// @return Status::Ok() on success, NOT_INITIALIZED before begin(), admission
  ///         BUSY as described by the class contract, or the original transport
  ///         status.
  /// @note Diagnostic writes to ctrl_hum (0xF2), ctrl_meas (0xF4), config
  ///       (0xF5), or reset (0xE0) mark hardwareConfigDirty() on success.
  ///       Transport failures that may have partially reached those registers
  ///       preserve the original status as hardwareConfigDirtyError(). Call
  ///       recover(), begin(), or a successful softReset() to resync after
  ///       manual config-register edits.
  Status writeRegister(uint8_t reg, uint8_t value);

  // =========================================================================
  // Timing
  // =========================================================================

  /// Estimate maximum measurement time for cached settings in microseconds.
  /// @return Exact Bosch maximum without the scheduler margin
  uint32_t estimateMeasurementTimeUs() const;

  /// Estimate max measurement time based on current oversampling.
  /// @return Estimated measurement duration in milliseconds
  uint32_t estimateMeasurementTimeMs() const;

  /// Get the configured standby interval in milliseconds (rounded up)
  /// @return Standby interval in milliseconds
  uint32_t getStandbyTimeMs() const;

  /// Estimate full normal-mode cycle time (measurement + standby) in ms
  /// @return Estimated normal-mode cycle in milliseconds
  uint32_t estimateNormalCycleMs() const;

private:
  // =========================================================================
  // Transport Wrappers
  // =========================================================================
  
  /// Raw I2C write-read (no health tracking)
  Status _i2cWriteReadRaw(const uint8_t* txBuf, size_t txLen, 
                          uint8_t* rxBuf, size_t rxLen);
  
  /// Raw I2C write (no health tracking)
  Status _i2cWriteRaw(const uint8_t* buf, size_t len);
  
  /// Tracked I2C write-read (updates health)
  Status _i2cWriteReadTracked(const uint8_t* txBuf, size_t txLen, 
                              uint8_t* rxBuf, size_t rxLen);
  
  /// Tracked I2C write (updates health)
  Status _i2cWriteTracked(const uint8_t* buf, size_t len);

  // =========================================================================
  // Register Access
  // =========================================================================
  
  /// Read registers (uses tracked path)
  Status readRegs(uint8_t startReg, uint8_t* buf, size_t len);
  
  /// Write registers (uses tracked path)
  Status writeRegs(uint8_t startReg, const uint8_t* buf, size_t len);

  /// Read single register (raw path)
  Status _readRegisterRaw(uint8_t reg, uint8_t& value);
  
  // =========================================================================
  // Health Management
  // =========================================================================
  
  /// Update health counters and state based on operation result
  /// Called ONLY from tracked transport wrappers
  Status _updateHealth(const Status& st);

  /// Record non-transport semantic failures that make recovery unsuccessful.
  Status _recordFailure(const Status& st);
  void _markHardwareConfigDirty(const Status& st);
  void _clearHardwareConfigDirty();

  // =========================================================================
  // Internal
  // =========================================================================

  void _resetRuntime(bool preserveHistory = true);
  Status _prepareBeginConfig(const Config& config);
  void _setSensorSettings(const SensorSettings& settings);
  bool _jobActive() const;
  Status _jobStartAdmission() const;
  Status _hardwareOperationAdmission() const;
  void _clearJob();
  Status _startJob(JobKind kind, JobPhase phase);
  JobPollResult _idleJobResult() const;
  JobPollResult _jobResult(uint8_t instructionsUsed) const;
  JobPollResult _failJob(const Status& st, uint8_t instructionsUsed);
  JobPollResult _completeJob(uint8_t instructionsUsed);
  void _trackJobConfigWriteResult(const Status& st);
  Status _readCalibrationTp();
  Status _readCalibrationH();
  Status _validateCalibrationValues(uint16_t digT1, uint16_t digP1) const;
  Status _readCalibrationCandidate(Calibration& calibration,
                                   bool& humidityCalibrationValid,
                                   bool& calibrationEvidenceChanged);
  bool _calibrationTpMatchesCommitted(const Calibration& calibration) const;
  bool _calibrationMatchesCommitted(
      const Calibration& calibration, bool humidityCalibrationValid) const;
  void _commitCalibration(const Calibration& calibration,
                          bool humidityCalibrationValid);
  Status _applyConfig();
  Status _ensureConfigWriteReady();
  Status _waitForNvmReady(bool tracked);
  Status _readRawData(RawSample& out);
  Status _compensate(const RawSample& raw, CompensatedSample& compensated,
                     int32_t& tFine) const;
  void _commitSample(const RawSample& raw,
                     const CompensatedSample& compensated,
                     int32_t tFine, uint32_t timestampMs);
  void _cancelMeasurementTrackingForStateChange();
  uint32_t _measurementReadinessIntervalMs() const;
  void _startMeasurementTracking(uint32_t startMs);
  void _finishMeasurementRequest(const Status& status);
  void _invalidateSampleCache();
  void _advanceConfigGeneration();
  uint32_t _nowMs() const;
  bool _timeValid() const;
  
  // =========================================================================
  // State
  // =========================================================================
  
  Config _config;
  bool _initialized = false;
  DriverState _driverState = DriverState::UNINIT;
  
  // Health counters
  uint32_t _lastOkMs = 0;
  uint32_t _lastErrorMs = 0;
  bool _lastOkTimeValid = false;
  bool _lastErrorTimeValid = false;
  Status _lastError = Status::Ok();
  uint8_t _consecutiveFailures = 0;
  uint32_t _totalFailures = 0;
  uint32_t _totalSuccess = 0;
  ConfigSyncState _configSyncState = ConfigSyncState::RESYNC_REQUIRED;
  CalibrationState _calibrationState = CalibrationState::INVALID;
  bool _humidityCalibrationValid = false;
  Status _hardwareConfigDirtyError = Status::Ok();
  uint32_t _configGeneration = 0;

  // Staged job state
  JobKind _jobKind = JobKind::NONE;
  JobState _jobState = JobState::IDLE;
  JobPhase _jobPhase = JobPhase::NONE;
  Status _jobStatus = Status::Ok();
  uint32_t _jobId = 0;
  uint32_t _nextJobId = 0;
  bool _jobTerminalResultPending = false;
  uint32_t _jobDeadlineMs = 0;
  bool _jobDeadlineActive = false;
  uint16_t _jobNvmPolls = 0;
  uint16_t _jobWaitPolls = 0;
  bool _jobHardwareConfigTouched = false;
  bool _jobResetMayHaveReached = false;
  bool _jobForcedTriggerMayHaveReached = false;
  bool _jobSettingsStaged = false;
  SensorSettings _jobPriorSettings = {};
  Calibration _jobCalibration = {};
  bool _jobHumidityCalibrationValid = false;
  bool _jobCalibrationChanged = false;
  bool _jobDeviceIdentityMismatch = false;
  ConfigSyncState _jobPriorConfigSyncState = ConfigSyncState::RESYNC_REQUIRED;
  Status _jobPriorHardwareConfigDirtyError = Status::Ok();
  RawSample _jobRawSample = {};
  CompensatedSample _jobCompSample = {};
  int32_t _jobTFine = 0;

  // Explicit poll/tick time context for health timestamps.
  bool _timeContextActive = false;
  uint32_t _timeContextMs = 0;

  // Calibration data
  uint16_t _digT1 = 0;
  int16_t _digT2 = 0;
  int16_t _digT3 = 0;
  uint16_t _digP1 = 0;
  int16_t _digP2 = 0;
  int16_t _digP3 = 0;
  int16_t _digP4 = 0;
  int16_t _digP5 = 0;
  int16_t _digP6 = 0;
  int16_t _digP7 = 0;
  int16_t _digP8 = 0;
  int16_t _digP9 = 0;
  uint8_t _digH1 = 0;
  int16_t _digH2 = 0;
  uint8_t _digH3 = 0;
  int16_t _digH4 = 0;
  int16_t _digH5 = 0;
  int8_t _digH6 = 0;

  // Measurement state
  bool _measurementRequested = false;
  bool _measurementReady = false;
  ConversionState _conversionState = ConversionState::IDLE;
  Status _lastMeasurementStatus = Status::Ok();
  bool _hasSample = false;
  uint32_t _measurementStartMs = 0;
  uint32_t _measurementDeadlineMs = 0;
  uint16_t _measurementStatusPolls = 0;
  uint32_t _sampleTimestampMs = 0;
  uint32_t _sampleSequence = 0;
  uint32_t _sampleConfigGeneration = 0;
  bool _sampleGenerationStale = false;
  int32_t _tFine = 0;
  RawSample _rawSample;
  CompensatedSample _compSample;
};

} // namespace BME280
