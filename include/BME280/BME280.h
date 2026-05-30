/// @file BME280.h
/// @brief Main driver class for BME280
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
  UNINIT,    ///< begin() not called or end() called
  READY,     ///< Operational, consecutiveFailures == 0
  DEGRADED,  ///< 1 <= consecutiveFailures < offlineThreshold
  OFFLINE    ///< consecutiveFailures >= offlineThreshold
};

/// Measurement result (float)
struct Measurement {
  float temperatureC = 0.0f; ///< Temperature in Celsius
  float pressurePa = 0.0f;   ///< Pressure in Pascals
  float humidityPct = 0.0f;  ///< Relative humidity in percent
};

/// Raw ADC values
struct RawSample {
  int32_t adcT = 0; ///< Raw temperature ADC (20-bit)
  int32_t adcP = 0; ///< Raw pressure ADC (20-bit)
  int32_t adcH = 0; ///< Raw humidity ADC (16-bit)
};

/// Fixed-point compensated values (no float)
struct CompensatedSample {
  int32_t tempC_x100 = 0;        ///< Temperature * 100 (e.g., 2534 = 25.34 degC)
  uint32_t pressurePa = 0;       ///< Pressure in Pa
  uint32_t humidityPct_x1024 = 0; ///< Humidity * 1024 (Q22.10 format)
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
  uint8_t tp[cmd::REG_CALIB_TP_LEN] = {}; ///< Raw 0x88..0xA1 temperature/pressure block
  uint8_t h1 = 0;                         ///< Raw 0xA1 humidity byte
  uint8_t h[cmd::REG_CALIB_H_LEN] = {};   ///< Raw 0xE1..0xE7 humidity block
};

/// Snapshot of driver configuration and runtime state without I2C access.
struct SettingsSnapshot {
  bool initialized = false;                   ///< True after begin() succeeds
  DriverState state = DriverState::UNINIT;    ///< Current driver state
  uint8_t i2cAddress = 0x76;                  ///< Active 7-bit I2C address
  uint32_t i2cTimeoutMs = 0;                  ///< Active I2C timeout
  uint8_t offlineThreshold = 0;               ///< Failure threshold for OFFLINE
  bool hasNowMsHook = false;                  ///< True when Config::nowMs is set
  Mode mode = Mode::SLEEP;                    ///< Active measurement mode
  Oversampling osrsT = Oversampling::SKIP;    ///< Temperature oversampling
  Oversampling osrsP = Oversampling::SKIP;    ///< Pressure oversampling
  Oversampling osrsH = Oversampling::SKIP;    ///< Humidity oversampling
  Filter filter = Filter::OFF;                ///< IIR filter coefficient
  Standby standby = Standby::MS_0_5;         ///< Standby time for normal mode
  bool measurementRequested = false;          ///< True after forced-mode trigger
  bool measurementReady = false;              ///< True when data registers are ready
  bool hasSample = false;                     ///< True when a compensated sample is cached
  bool hardwareConfigDirty = false;           ///< True when hardware config may differ from cache
  Status hardwareConfigDirtyError = Status::Ok(); ///< First error that made config state uncertain
  uint32_t measurementStartMs = 0;            ///< Timestamp of last measurement trigger
  uint32_t sampleTimestampMs = 0;             ///< Timestamp of the last cached sample
  int32_t tFine = 0;                          ///< Last t_fine intermediate value
  RawSample rawSample = {};                   ///< Last raw ADC sample
  CompensatedSample compSample = {};          ///< Last compensated sample
  Calibration calibration = {};               ///< Cached compensation coefficients
};

/// BME280 driver class
class BME280 {
public:
  BME280() = default;
  ~BME280() = default;
  BME280(const BME280&) = delete;
  BME280& operator=(const BME280&) = delete;
  BME280(BME280&&) = delete;
  BME280& operator=(BME280&&) = delete;

  // =========================================================================
  // Lifecycle
  // =========================================================================
  
  /// Initialize the driver with configuration
  /// @param config Configuration including transport callbacks
  /// @return Status::Ok() on success, error otherwise
  Status begin(const Config& config);
  
  /// Process pending operations (call regularly from loop)
  /// @param nowMs Current timestamp in milliseconds
  void tick(uint32_t nowMs);
  
  /// Shutdown the driver, put the sensor to sleep best-effort, and clear runtime state
  void end();

  /// Check if begin() completed successfully and end() has not been called
  bool isInitialized() const { return _initialized; }

  /// Get the cached configuration snapshot currently owned by the driver
  const Config& getConfig() const { return _config; }
  
  // =========================================================================
  // Diagnostics
  // =========================================================================
  
  /// Check if device is present on the bus (no health tracking)
  /// @return Status::Ok() if device responds, error otherwise
  Status probe();
  
  /// Attempt to recover from DEGRADED/OFFLINE state by probing and reapplying cached config
  /// @return Status::Ok() if device now responsive, error otherwise
  Status recover();

  /// Populate a snapshot of cached configuration and runtime state without I2C.
  /// @param[out] out Snapshot to fill
  /// @return Status::Ok() always
  Status getSettings(SettingsSnapshot& out) const;

  /// True when a failed multi-register config operation may have left sensor
  /// registers different from the cached settings. Cleared only by a complete
  /// successful config resync in begin(), recover(), or softReset().
  bool hardwareConfigDirty() const { return _hardwareConfigDirty; }

  /// First transport/status error that made hardware config state uncertain.
  Status hardwareConfigDirtyError() const { return _hardwareConfigDirtyError; }
  
  // =========================================================================
  // Driver State
  // =========================================================================
  
  /// Get current driver state
  DriverState state() const { return _driverState; }

  /// Alias for state() used by shared diagnostics.
  DriverState driverState() const { return state(); }
  
  /// Check if driver is ready for operations
  bool isOnline() const { 
    return _driverState == DriverState::READY || 
           _driverState == DriverState::DEGRADED; 
  }
  
  // =========================================================================
  // Health Tracking
  // =========================================================================
  
  /// Timestamp of last successful I2C operation
  uint32_t lastOkMs() const { return _lastOkMs; }
  
  /// Timestamp of last failed I2C operation
  uint32_t lastErrorMs() const { return _lastErrorMs; }
  
  /// Most recent error status
  Status lastError() const { return _lastError; }
  
  /// Consecutive failures since last success
  uint8_t consecutiveFailures() const { return _consecutiveFailures; }
  
  /// Total failure count (lifetime)
  uint32_t totalFailures() const { return _totalFailures; }
  
  /// Total success count (lifetime)
  uint32_t totalSuccess() const { return _totalSuccess; }
  
  // =========================================================================
  // Measurement API
  // =========================================================================
  
  /// Request a measurement (non-blocking)
  /// In FORCED mode: triggers measurement if idle.
  /// In NORMAL mode: waits one estimated normal cycle before reading, so the
  /// sample is fresh relative to the request.
  /// Returns IN_PROGRESS if measurement started, BUSY if already measuring or OFFLINE
  Status requestMeasurement();

  /// Check if measurement is ready to read
  bool measurementReady() const { return _measurementReady; }

  /// True after at least one sample has been cached.
  bool hasSample() const { return _hasSample; }

  /// Timestamp of the last cached sample, or 0 if none exists.
  uint32_t sampleTimestampMs() const { return _sampleTimestampMs; }

  /// Age of the cached sample in milliseconds.
  /// @param nowMs Current monotonic timestamp in milliseconds
  /// @return `nowMs - sampleTimestampMs()` when a sample exists, otherwise 0
  uint32_t sampleAgeMs(uint32_t nowMs) const {
    return _hasSample ? (nowMs - _sampleTimestampMs) : 0;
  }

  /// Get measurement result (float)
  /// Returns MEASUREMENT_NOT_READY if not available
  /// Clears ready flag after successful read
  /// Does not invalidate cached raw/fixed-point samples.
  Status getMeasurement(Measurement& out);

  /// Get raw ADC values.
  /// @param[out] out Last cached raw ADC sample
  /// @return Status::Ok() on success, MEASUREMENT_NOT_READY until a sample has been captured
  Status getRawSample(RawSample& out) const;

  /// Get fixed-point compensated values.
  /// @param[out] out Last cached fixed-point compensated sample
  /// @return Status::Ok() on success, MEASUREMENT_NOT_READY until a sample has been captured
  Status getCompensatedSample(CompensatedSample& out) const;

  /// Get cached calibration coefficients.
  /// @param[out] out Cached coefficients read during begin() or softReset()
  /// @return Status::Ok() on success, NOT_INITIALIZED before begin()
  Status getCalibration(Calibration& out) const;

  /// Read raw calibration registers from the device.
  /// @param[out] out Raw register blocks
  /// @return Status::Ok() on success, error otherwise
  Status readCalibrationRaw(CalibrationRaw& out);

  // =========================================================================
  // Configuration
  // =========================================================================

  /// Set operating mode (SLEEP, FORCED, NORMAL).
  /// FORCED is an on-demand policy and does not trigger a conversion until
  /// requestMeasurement() is called.
  Status setMode(Mode mode);

  /// Get current mode
  Status getMode(Mode& out) const;

  /// Set oversampling for temperature.
  /// Temperature must be enabled when pressure or humidity is enabled.
  Status setOversamplingT(Oversampling osrs);

  /// Set oversampling for pressure.
  /// Temperature must be enabled when pressure is enabled.
  Status setOversamplingP(Oversampling osrs);

  /// Set oversampling for humidity.
  /// Temperature must be enabled when humidity is enabled.
  Status setOversamplingH(Oversampling osrs);

  /// Set IIR filter coefficient
  Status setFilter(Filter filter);

  /// Set standby time (normal mode only)
  Status setStandby(Standby standby);

  /// Get oversampling for temperature
  Status getOversamplingT(Oversampling& out) const;

  /// Get oversampling for pressure
  Status getOversamplingP(Oversampling& out) const;

  /// Get oversampling for humidity
  Status getOversamplingH(Oversampling& out) const;

  /// Get IIR filter coefficient
  Status getFilter(Filter& out) const;

  /// Get standby time
  Status getStandby(Standby& out) const;

  /// Soft reset device
  Status softReset();

  /// Read chip ID
  Status readChipId(uint8_t& id);

  /// Read status register
  Status readStatus(uint8_t& status);

  /// Read ctrl_hum register
  Status readCtrlHum(uint8_t& value);

  /// Read ctrl_meas register
  Status readCtrlMeas(uint8_t& value);

  /// Read config register
  Status readConfig(uint8_t& value);

  /// Check if device is currently measuring
  Status isMeasuring(bool& measuring);

  // =========================================================================
  // Raw Register Access
  // =========================================================================

  /// Read a contiguous register block
  Status readRegisters(uint8_t startReg, uint8_t* buf, size_t len);

  /// Write a contiguous register block
  Status writeRegisters(uint8_t startReg, const uint8_t* buf, size_t len);

  /// Read a single register
  Status readRegister(uint8_t reg, uint8_t& value);

  /// Write a single register
  Status writeRegister(uint8_t reg, uint8_t value);

  // =========================================================================
  // Timing
  // =========================================================================

  /// Estimate max measurement time based on current oversampling
  /// Returns time in milliseconds
  uint32_t estimateMeasurementTimeMs() const;

  /// Get the configured standby interval in milliseconds (rounded up)
  uint32_t getStandbyTimeMs() const;

  /// Estimate full normal-mode cycle time (measurement + standby) in ms
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

  /// Return BUSY when normal operations try I2C while OFFLINE.
  Status _offlineStatus() const;
  
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
  void _reassertOfflineLatch();
  void _markHardwareConfigDirty(const Status& st);
  void _clearHardwareConfigDirty();

  // =========================================================================
  // Internal
  // =========================================================================

  Status _applyConfig();
  Status _waitForNvmReady();
  Status _readCalibration();
  Status _validateCalibration();
  Status _readRawData();
  Status _compensate();
  uint32_t _nowMs() const;
  
  // =========================================================================
  // State
  // =========================================================================
  
  Config _config;
  bool _initialized = false;
  DriverState _driverState = DriverState::UNINIT;
  
  // Health counters
  uint32_t _lastOkMs = 0;
  uint32_t _lastErrorMs = 0;
  Status _lastError = Status::Ok();
  uint8_t _consecutiveFailures = 0;
  uint32_t _totalFailures = 0;
  uint32_t _totalSuccess = 0;
  bool _allowOfflineI2c = false;
  bool _hardwareConfigDirty = false;
  Status _hardwareConfigDirtyError = Status::Ok();

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
  bool _hasSample = false;
  uint32_t _measurementStartMs = 0;
  uint32_t _sampleTimestampMs = 0;
  int32_t _tFine = 0;
  RawSample _rawSample;
  CompensatedSample _compSample;
};

} // namespace BME280
