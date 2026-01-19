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

/// BME280 driver class
class BME280 {
public:
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
  
  /// Shutdown the driver and release resources
  void end();
  
  // =========================================================================
  // Diagnostics
  // =========================================================================
  
  /// Check if device is present on the bus (no health tracking)
  /// @return Status::Ok() if device responds, error otherwise
  Status probe();
  
  /// Attempt to recover from DEGRADED/OFFLINE state
  /// @return Status::Ok() if device now responsive, error otherwise
  Status recover();
  
  // =========================================================================
  // Driver State
  // =========================================================================
  
  /// Get current driver state
  DriverState state() const { return _driverState; }
  
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
  /// In FORCED mode: triggers measurement if idle
  /// In NORMAL mode: marks intent to read next available
  /// Returns IN_PROGRESS if measurement started, BUSY if already measuring
  Status requestMeasurement();

  /// Check if measurement is ready to read
  bool measurementReady() const { return _measurementReady; }

  /// Get measurement result (float)
  /// Returns MEASUREMENT_NOT_READY if not available
  /// Clears ready flag after successful read
  Status getMeasurement(Measurement& out);

  /// Get raw ADC values
  Status getRawSample(RawSample& out) const;

  /// Get fixed-point compensated values
  Status getCompensatedSample(CompensatedSample& out) const;

  // =========================================================================
  // Configuration
  // =========================================================================

  /// Set operating mode (SLEEP, FORCED, NORMAL)
  Status setMode(Mode mode);

  /// Get current mode
  Status getMode(Mode& out) const;

  /// Set oversampling for temperature
  Status setOversamplingT(Oversampling osrs);

  /// Set oversampling for pressure
  Status setOversamplingP(Oversampling osrs);

  /// Set oversampling for humidity
  Status setOversamplingH(Oversampling osrs);

  /// Set IIR filter coefficient
  Status setFilter(Filter filter);

  /// Set standby time (normal mode only)
  Status setStandby(Standby standby);

  /// Soft reset device
  Status softReset();

  /// Read chip ID
  Status readChipId(uint8_t& id);

  /// Read status register
  Status readStatus(uint8_t& status);

  /// Check if device is currently measuring
  Status isMeasuring(bool& measuring);

  // =========================================================================
  // Timing
  // =========================================================================

  /// Estimate max measurement time based on current oversampling
  /// Returns time in milliseconds
  uint32_t estimateMeasurementTimeMs() const;

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

  /// Read single register (uses tracked path)
  Status readRegister(uint8_t reg, uint8_t& value);

  /// Write single register (uses tracked path)
  Status writeRegister(uint8_t reg, uint8_t value);

  /// Read single register (raw path)
  Status _readRegisterRaw(uint8_t reg, uint8_t& value);
  
  // =========================================================================
  // Health Management
  // =========================================================================
  
  /// Update health counters and state based on operation result
  /// Called ONLY from tracked transport wrappers
  Status _updateHealth(const Status& st);

  // =========================================================================
  // Internal
  // =========================================================================

  Status _applyConfig();
  Status _readCalibration();
  Status _validateCalibration();
  Status _readRawData();
  Status _compensate();
  
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
  uint32_t _measurementStartMs = 0;
  int32_t _tFine = 0;
  RawSample _rawSample;
  CompensatedSample _compSample;
};

} // namespace BME280
