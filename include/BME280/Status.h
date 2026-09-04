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
  IN_PROGRESS,               ///< Operation accepted; advance with tick() or pollJob() as documented

  // I2C transport details (append-only to preserve existing values)
  I2C_NACK_ADDR,             ///< I2C address not acknowledged
  I2C_NACK_DATA,             ///< I2C data byte not acknowledged
  I2C_TIMEOUT,               ///< I2C transaction timeout
  I2C_BUS,                   ///< I2C bus error (arbitration lost, etc.)
  RESYNC_REQUIRED,           ///< Cached device state must be reconciled before use.
                             ///< From post-write settings readback, Status::detail
                             ///< is packed as `0x00RREEAA`: register address in
                             ///< bits 16-23 (0xF2 ctrl_hum, 0xF4 ctrl_meas, 0xF5
                             ///< config), expected driver-owned bits in 8-15, and
                             ///< actual driver-owned bits in 0-7. Always
                             ///< non-negative. Other producers leave detail zero.
  CANCELLED,                 ///< Staged job cancelled by its owner
  DEADLINE_EXPIRED,          ///< Staged job cancelled because its owner deadline expired
  I2C_SHORT_TRANSFER         ///< Transport reported OK with incomplete byte counts
};

/// Return the library-owned canonical string for an error code.
/// @param err Error code to describe
/// @return Static storage string; invalid enum values return "UNKNOWN_ERROR"
constexpr const char* toString(Err err) {
  switch (err) {
    case Err::OK: return "OK";
    case Err::NOT_INITIALIZED: return "NOT_INITIALIZED";
    case Err::INVALID_CONFIG: return "INVALID_CONFIG";
    case Err::I2C_ERROR: return "I2C_ERROR";
    case Err::TIMEOUT: return "TIMEOUT";
    case Err::INVALID_PARAM: return "INVALID_PARAM";
    case Err::DEVICE_NOT_FOUND: return "DEVICE_NOT_FOUND";
    case Err::CHIP_ID_MISMATCH: return "CHIP_ID_MISMATCH";
    case Err::CALIBRATION_INVALID: return "CALIBRATION_INVALID";
    case Err::MEASUREMENT_NOT_READY: return "MEASUREMENT_NOT_READY";
    case Err::COMPENSATION_ERROR: return "COMPENSATION_ERROR";
    case Err::BUSY: return "BUSY";
    case Err::IN_PROGRESS: return "IN_PROGRESS";
    case Err::I2C_NACK_ADDR: return "I2C_NACK_ADDR";
    case Err::I2C_NACK_DATA: return "I2C_NACK_DATA";
    case Err::I2C_TIMEOUT: return "I2C_TIMEOUT";
    case Err::I2C_BUS: return "I2C_BUS";
    case Err::RESYNC_REQUIRED: return "RESYNC_REQUIRED";
    case Err::CANCELLED: return "CANCELLED";
    case Err::DEADLINE_EXPIRED: return "DEADLINE_EXPIRED";
    case Err::I2C_SHORT_TRANSFER: return "I2C_SHORT_TRANSFER";
    default: return "UNKNOWN_ERROR";
  }
}

/// Status structure returned by all fallible operations
struct Status {
  Err code = Err::OK;          ///< Repository-standard error code.
  int32_t detail = 0;        ///< Code-dependent detail. BUSY carries a BusyReason.
                             ///< INVALID_PARAM from validateSettings() and
                             ///< INVALID_CONFIG from begin() carry a
                             ///< SettingsValidationReason. CHIP_ID_MISMATCH carries
                             ///< the observed chip-ID byte. RESYNC_REQUIRED from
                             ///< settings readback carries the packed triple
                             ///< documented on Err::RESYNC_REQUIRED. Transport
                             ///< errors carry the adapter's TransportResult detail.
  const char* msg = toString(Err::OK); ///< Library-owned canonical error string

  /// Create an OK status.
  constexpr Status() = default;

  /// Create a status with explicit code/detail and a canonical library message.
  /// @param c Error code.
  /// @param d Implementation-specific detail value.
  /// @param m Compatibility parameter; ignored to prevent borrowed storage.
  constexpr Status(Err c, int32_t d, const char* m)
      : code(c), detail(d), msg(toString(c)) {
    (void)m;
  }

  /// Create a status from canonical code and detail only.
  /// @param c Error code.
  /// @param d Implementation-specific detail value.
  constexpr Status(Err c, int32_t d) : code(c), detail(d), msg(toString(c)) {}

  /// Copy while re-canonicalizing message ownership.
  /// @param other Status to copy.
  constexpr Status(const Status& other)
      : code(other.code), detail(other.detail), msg(toString(other.code)) {}

  /// Assign while re-canonicalizing message ownership.
  /// @param other Status to copy.
  /// @return Reference to this status.
  constexpr Status& operator=(const Status& other) {
    code = other.code;
    detail = other.detail;
    msg = toString(other.code);
    return *this;
  }
  
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
  static constexpr Status Ok() { return Status{Err::OK, 0}; }

  /// Create an error using only library-owned canonical message storage.
  /// @param err Error code.
  /// @param detailCode Optional implementation-specific detail code.
  /// @return Status carrying the supplied code/detail and canonical message.
  static constexpr Status Error(Err err, int32_t detailCode = 0) {
    return Status{err, detailCode};
  }
  
  /// Create an error status while retaining the legacy call signature.
  /// @param err Error code.
  /// @param message Ignored; msg always points to toString(err).
  /// @param detailCode Optional implementation-specific detail code.
  /// @return Status carrying code/detail and a library-owned canonical message.
  static constexpr Status Error(Err err, const char* message, int32_t detailCode = 0) {
    return Status{err, detailCode, message};
  }
};

} // namespace BME280
