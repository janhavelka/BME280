/// @file Status.h
/// @brief Error codes and status handling for {DEVICE_NAME} driver
#pragma once

#include <cstdint>

namespace {NAMESPACE} {

/// Error codes for all {DEVICE_NAME} operations
enum class Err : uint8_t {
  OK = 0,                    ///< Operation successful
  NOT_INITIALIZED,           ///< begin() not called
  INVALID_CONFIG,            ///< Invalid configuration parameter
  I2C_ERROR,                 ///< I2C communication failure
  TIMEOUT,                   ///< Operation timed out
  INVALID_PARAM,             ///< Invalid parameter value
  DEVICE_NOT_FOUND,          ///< Device not responding on I2C bus
  BUSY,                      ///< Device is busy
  IN_PROGRESS                ///< Operation scheduled; call tick() to complete
  // Add device-specific error codes here
};

/// Status structure returned by all fallible operations
struct Status {
  Err code = Err::OK;
  int32_t detail = 0;        ///< Implementation-specific detail (e.g., I2C error code)
  const char* msg = "";      ///< Static string describing the error
  
  /// @return true if operation succeeded
  constexpr bool ok() const { return code == Err::OK; }
  
  /// @return true if operation in progress (not a failure)
  constexpr bool inProgress() const { return code == Err::IN_PROGRESS; }
  
  /// Create a success status
  static constexpr Status Ok() { return Status{Err::OK, 0, "OK"}; }
  
  /// Create an error status
  static constexpr Status Error(Err err, const char* message, int32_t detailCode = 0) {
    return Status{err, detailCode, message};
  }
};

} // namespace {NAMESPACE}
