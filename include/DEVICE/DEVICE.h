/// @file {DEVICE}.h
/// @brief Main driver class for {DEVICE_NAME}
#pragma once

#include <cstdint>
#include "{NAMESPACE}/Status.h"
#include "{NAMESPACE}/Config.h"
#include "{NAMESPACE}/CommandTable.h"
#include "{NAMESPACE}/Version.h"

namespace {NAMESPACE} {

/// Driver state for health monitoring
enum class DriverState : uint8_t {
  UNINIT,    ///< begin() not called or end() called
  READY,     ///< Operational, consecutiveFailures == 0
  DEGRADED,  ///< 1 <= consecutiveFailures < offlineThreshold
  OFFLINE    ///< consecutiveFailures >= offlineThreshold
};

/// {DEVICE_NAME} driver class
class {DEVICE} {
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
  // Device-Specific API
  // =========================================================================
  
  // TODO: Add your device-specific methods here
  // Examples:
  // Status readValue(int16_t& value);
  // Status writeConfig(uint8_t config);
  // Status setMode(Mode mode);

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
  
  // =========================================================================
  // Health Management
  // =========================================================================
  
  /// Update health counters and state based on operation result
  /// Called ONLY from tracked transport wrappers
  Status _updateHealth(const Status& st);
  
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
};

} // namespace {NAMESPACE}
