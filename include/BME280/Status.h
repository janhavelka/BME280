/// @file Status.h
/// @brief Error codes and status handling for BME280 driver
#pragma once

#include <cstdint>

namespace BME280 {

/// Error codes for all BME280 operations
enum class Err : uint8_t {
  OK = 0,                    ///< Operation successful
  NOT_INITIALIZED,           ///< begin() not called
  INVALID_CONFIG,            ///< Invalid configuration parameter
  I2C_ERROR,                 ///< I2C communication failure
  TIMEOUT,                   ///< Operation timed out
  INVALID_PARAM,             ///< Invalid parameter value
  DEVICE_NOT_FOUND,          ///< Definite address NACK / device absent on I2C bus
  CHIP_ID_MISMATCH,          ///< Chip ID != 0x60 (not a BME280)
  CALIBRATION_INVALID,       ///< Compensation data failed validation
  MEASUREMENT_NOT_READY,     ///< Sample not yet available
  CONVERSION_NOT_READY = MEASUREMENT_NOT_READY, ///< Alias for cross-library uniformity
  COMPENSATION_ERROR,        ///< Compensation math failed
  BUSY,                      ///< Device is busy
  IN_PROGRESS,               ///< Operation scheduled; call tick() to complete

  // I2C transport details (append-only to preserve existing values)
  I2C_NACK_ADDR,             ///< I2C address not acknowledged
  I2C_NACK_DATA,             ///< I2C data byte not acknowledged
  I2C_TIMEOUT,               ///< I2C transaction timeout
  I2C_BUS,                   ///< I2C bus error (arbitration lost, etc.)
  RESYNC_REQUIRED            ///< Cached device state must be reconciled before use
};

/// Status structure returned by all fallible operations
struct Status {
  Err code = Err::OK;          ///< Repository-standard error code.
  int32_t detail = 0;        ///< Implementation-specific detail (e.g., I2C error code)
  const char* msg = "";      ///< Static string describing the error

  /// Create an OK status.
  constexpr Status() = default;

  /// Create a status with explicit fields.
  /// @param c Error code.
  /// @param d Implementation-specific detail value.
  /// @param m Static status message.
  constexpr Status(Err c, int32_t d, const char* m) : code(c), detail(d), msg(m) {}
  
  /// @return true if operation succeeded
  constexpr bool ok() const { return code == Err::OK; }

  /// @param err Error code to compare against.
  /// @return true if the status matches the provided error code
  constexpr bool is(Err err) const { return code == err; }

  /// @return true if operation in progress (not a failure)
  constexpr bool inProgress() const { return code == Err::IN_PROGRESS; }

  /// @return true if operation succeeded
  explicit constexpr operator bool() const { return ok(); }

  /// Create a success status
  /// @return Success status with Err::OK.
  static constexpr Status Ok() { return Status{Err::OK, 0, "OK"}; }
  
  /// Create an error status
  /// @param err Error code.
  /// @param message Static error string.
  /// @param detailCode Optional implementation-specific detail code.
  /// @return Status carrying the supplied error fields.
  static constexpr Status Error(Err err, const char* message, int32_t detailCode = 0) {
    return Status{err, detailCode, message};
  }
};

} // namespace BME280
