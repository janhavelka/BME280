/// @file Config.h
/// @brief Configuration structure for BME280 driver
#pragma once

#include <cstddef>
#include <cstdint>
#include "BME280/Status.h"

namespace BME280 {

/// Terminal outcome of one physical transport attempt.
enum class TransportErr : uint8_t {
  OK,           ///< Requested physical transaction completed
  NACK_ADDRESS, ///< Address phase was not acknowledged
  NACK_DATA,    ///< A data byte was not acknowledged
  TIMEOUT,      ///< Transport attempt exceeded its bounded timeout
  BUS,          ///< Arbitration, electrical, or controller bus error
  OTHER         ///< Other terminal transport failure
};

/// Return the library-owned canonical string for a transport outcome.
/// @param err Transport outcome to describe
/// @return Static storage string; invalid enum values return "UNKNOWN_TRANSPORT_ERROR"
constexpr const char* toString(TransportErr err) {
  switch (err) {
    case TransportErr::OK: return "OK";
    case TransportErr::NACK_ADDRESS: return "NACK_ADDRESS";
    case TransportErr::NACK_DATA: return "NACK_DATA";
    case TransportErr::TIMEOUT: return "TIMEOUT";
    case TransportErr::BUS: return "BUS";
    case TransportErr::OTHER: return "OTHER";
    default: return "UNKNOWN_TRANSPORT_ERROR";
  }
}

/// Fixed-memory result from exactly one physical transport attempt.
struct TransportResult {
  TransportErr code = TransportErr::OK; ///< Terminal transport outcome
  int32_t detail = 0;                   ///< Adapter-owned numeric diagnostic
  size_t writeCount = 0;                ///< Bytes physically written
  size_t readCount = 0;                 ///< Bytes physically read

  /// @return true only for a terminal TransportErr::OK result
  constexpr bool ok() const { return code == TransportErr::OK; }

  /// Construct a completed transfer result.
  /// @param written Number of bytes physically written
  /// @param read Number of bytes physically read
  /// @return Successful terminal result with the supplied physical counts.
  static constexpr TransportResult Complete(size_t written, size_t read = 0) {
    return TransportResult{TransportErr::OK, 0, written, read};
  }

  /// Construct a terminal transport error result.
  /// @param error Terminal transport error
  /// @param detailCode Adapter-owned numeric diagnostic
  /// @param written Number of bytes physically written before failure
  /// @param read Number of bytes physically read before failure
  /// @return Terminal error result carrying the supplied code, detail, and
  ///         physical counts.
  static constexpr TransportResult Error(TransportErr error,
                                         int32_t detailCode = 0,
                                         size_t written = 0,
                                         size_t read = 0) {
    return TransportResult{error, detailCode, written, read};
  }
};

/// I2C write callback signature.
///
/// The application owns bus handles, locking, pins, and timeout policy. This
/// callback must perform exactly one physical attempt, must not retry or recover
/// the bus, and must return a terminal TransportResult within the supplied
/// timeout. It must not recursively call into the same driver instance.
/// @param addr     I2C device address (7-bit)
/// @param data     Pointer to data to write
/// @param len      Number of bytes to write
/// @param timeoutMs Maximum time to wait for completion
/// @param user     User context pointer passed through from Config
/// @return Terminal result with writeCount equal to len on success
using I2cWriteFn = TransportResult (*)(uint8_t addr, const uint8_t* data,
                                       size_t len, uint32_t timeoutMs,
                                       void* user);

/// I2C write-then-read callback signature.
///
/// The application owns bus handles, locking, pins, and timeout policy. This
/// callback must perform exactly one combined physical register-pointer
/// transaction: write txData, issue a repeated START without a STOP, then read
/// rxData. It must not retry, recover the bus, insert a STOP between pointer and
/// read phases, or recursively call into the same driver instance. It must
/// return a terminal TransportResult within the supplied timeout.
/// @param addr     I2C device address (7-bit)
/// @param txData   Pointer to data to write
/// @param txLen    Number of bytes to write
/// @param rxData   Pointer to buffer for read data
/// @param rxLen    Number of bytes to read
/// @param timeoutMs Maximum time to wait for completion
/// @param user     User context pointer passed through from Config
/// @return Terminal result with exact txLen/rxLen counts on success
using I2cWriteReadFn = TransportResult (*)(uint8_t addr,
                                           const uint8_t* txData, size_t txLen,
                                           uint8_t* rxData, size_t rxLen,
                                           uint32_t timeoutMs, void* user);

/// Millisecond timestamp callback.
///
/// requestMeasurement(), pollJob(nowMs), and tick(nowMs) must use the same
/// monotonic timebase.
/// @param user User context pointer passed through from Config
/// @return Current monotonic milliseconds
using NowMsFn = uint32_t (*)(void* user);

/// Oversampling settings
enum class Oversampling : uint8_t {
  SKIP = 0,  ///< Measurement skipped
  X1 = 1,    ///< 1x oversampling
  X2 = 2,    ///< 2x oversampling
  X4 = 3,    ///< 4x oversampling
  X8 = 4,    ///< 8x oversampling
  X16 = 5    ///< 16x oversampling
};

/// Return the canonical string for an oversampling setting.
/// @param value Oversampling setting to describe.
/// @return Static canonical setting name; invalid values return
///         `UNKNOWN_OVERSAMPLING`.
constexpr const char* toString(Oversampling value) {
  switch (value) {
    case Oversampling::SKIP: return "SKIP";
    case Oversampling::X1: return "X1";
    case Oversampling::X2: return "X2";
    case Oversampling::X4: return "X4";
    case Oversampling::X8: return "X8";
    case Oversampling::X16: return "X16";
    default: return "UNKNOWN_OVERSAMPLING";
  }
}

/// Measurement mode
enum class Mode : uint8_t {
  SLEEP = 0,  ///< No measurements, lowest power
  FORCED = 1, ///< On-demand single measurement; requestMeasurement() triggers conversion
  NORMAL = 3  ///< Continuous measurements
};

/// Return the canonical string for a measurement mode.
/// @param value Measurement mode to describe.
/// @return Static canonical mode name; invalid values return `UNKNOWN_MODE`.
constexpr const char* toString(Mode value) {
  switch (value) {
    case Mode::SLEEP: return "SLEEP";
    case Mode::FORCED: return "FORCED";
    case Mode::NORMAL: return "NORMAL";
    default: return "UNKNOWN_MODE";
  }
}

/// IIR filter coefficient
enum class Filter : uint8_t {
  OFF = 0, ///< Filter disabled
  X2 = 1,  ///< 2x filter coefficient
  X4 = 2,  ///< 4x filter coefficient
  X8 = 3,  ///< 8x filter coefficient
  X16 = 4  ///< 16x filter coefficient
};

/// Return the canonical string for an IIR filter setting.
/// @param value Filter setting to describe.
/// @return Static canonical setting name; invalid values return
///         `UNKNOWN_FILTER`.
constexpr const char* toString(Filter value) {
  switch (value) {
    case Filter::OFF: return "OFF";
    case Filter::X2: return "X2";
    case Filter::X4: return "X4";
    case Filter::X8: return "X8";
    case Filter::X16: return "X16";
    default: return "UNKNOWN_FILTER";
  }
}

/// Standby time between measurements (normal mode)
enum class Standby : uint8_t {
  MS_0_5 = 0,  ///< 0.5 ms
  MS_62_5 = 1, ///< 62.5 ms
  MS_125 = 2,  ///< 125 ms
  MS_250 = 3,  ///< 250 ms
  MS_500 = 4,  ///< 500 ms
  MS_1000 = 5, ///< 1000 ms
  MS_10 = 6,   ///< 10 ms
  MS_20 = 7    ///< 20 ms
};

/// Return the canonical string for a standby setting.
/// @param value Standby setting to describe.
/// @return Static canonical setting name; invalid values return
///         `UNKNOWN_STANDBY`.
constexpr const char* toString(Standby value) {
  switch (value) {
    case Standby::MS_0_5: return "MS_0_5";
    case Standby::MS_62_5: return "MS_62_5";
    case Standby::MS_125: return "MS_125";
    case Standby::MS_250: return "MS_250";
    case Standby::MS_500: return "MS_500";
    case Standby::MS_1000: return "MS_1000";
    case Standby::MS_10: return "MS_10";
    case Standby::MS_20: return "MS_20";
    default: return "UNKNOWN_STANDBY";
  }
}

/// Compact hardware measurement settings independent of transport policy.
struct SensorSettings {
  Oversampling osrsT = Oversampling::X1; ///< Temperature oversampling
  Oversampling osrsP = Oversampling::X1; ///< Pressure oversampling
  Oversampling osrsH = Oversampling::X1; ///< Humidity oversampling
  Filter filter = Filter::OFF;           ///< IIR filter coefficient
  Standby standby = Standby::MS_125;     ///< Normal-mode standby interval
  Mode mode = Mode::FORCED;              ///< Measurement mode policy
};

/// Deterministic reason encoded in Status::detail when settings validation
/// returns INVALID_PARAM (or INVALID_CONFIG while admitting Config).
enum class SettingsValidationReason : int32_t {
  NONE = 0,  ///< Settings are valid
  OSRS_T,    ///< Temperature oversampling encoding is invalid
  OSRS_P,    ///< Pressure oversampling encoding is invalid
  OSRS_H,    ///< Humidity oversampling encoding is invalid
  FILTER,    ///< IIR filter encoding is invalid
  STANDBY,   ///< Standby-time encoding is invalid
  MODE,      ///< Measurement-mode encoding is invalid
  SELECTION  ///< Channel selection cannot be compensated
};

/// Return the canonical string for a settings-validation reason.
/// @param value Validation reason to describe.
/// @return Static canonical reason name; invalid values return
///         `UNKNOWN_SETTINGS_VALIDATION_REASON`.
constexpr const char* toString(SettingsValidationReason value) {
  switch (value) {
    case SettingsValidationReason::NONE: return "NONE";
    case SettingsValidationReason::OSRS_T: return "OSRS_T";
    case SettingsValidationReason::OSRS_P: return "OSRS_P";
    case SettingsValidationReason::OSRS_H: return "OSRS_H";
    case SettingsValidationReason::FILTER: return "FILTER";
    case SettingsValidationReason::STANDBY: return "STANDBY";
    case SettingsValidationReason::MODE: return "MODE";
    case SettingsValidationReason::SELECTION: return "SELECTION";
    default: return "UNKNOWN_SETTINGS_VALIDATION_REASON";
  }
}

/// Validate a settings tuple without accessing hardware.
/// Temperature must be enabled whenever pressure or humidity is enabled, and
/// at least one measurement channel must be selected.
/// @param settings Settings to validate
/// @return OK for a supported tuple, or INVALID_PARAM with detail set to the
///         first failing SettingsValidationReason in declaration order
Status validateSettings(const SensorSettings& settings);

/// Estimate the exact Bosch maximum conversion duration for a settings tuple.
/// This microsecond value has no library scheduling margin and performs no
/// hardware access.
/// @param settings Settings whose oversampling controls conversion duration
/// @return Maximum conversion duration in microseconds, or 0 if invalid
uint32_t estimateMeasurementTimeUs(const SensorSettings& settings);

/// Estimate scheduler duration rounded up to whole milliseconds after adding
/// the driver's fixed one-millisecond margin to the Bosch maximum.
/// @param settings Settings whose oversampling controls conversion duration
/// @return Rounded-up maximum conversion duration in milliseconds, or 0 if invalid
uint32_t estimateMeasurementTimeMs(const SensorSettings& settings);

/// Convert signed centi-degrees Celsius to signed milli-degrees Celsius.
/// The output is unchanged when the multiplication would overflow int32_t.
/// @param tempC_x100 Temperature in 0.01 degree Celsius units
/// @param[out] outMilliC Temperature in 0.001 degree Celsius units
/// @return OK on success, COMPENSATION_ERROR on overflow
Status temperatureX100ToMilliC(int32_t tempC_x100, int32_t& outMilliC);

/// Convert unsigned Q22.10 percent to signed milli-percent with truncation.
/// The output is unchanged for input above 100% or signed overflow.
/// @param humidityPct_x1024 Relative humidity percent multiplied by 1024
/// @param[out] outMilliPercent Relative humidity in 0.001 percent units
/// @return OK, INVALID_PARAM above 100%, or COMPENSATION_ERROR on overflow
Status humidityX1024ToMilliPercent(uint32_t humidityPct_x1024,
                                  int32_t& outMilliPercent);

/// Check the mandatory BME280 chip identity value.
/// @param chipId Value read from the chip-ID register
/// @return true only for chip ID 0x60
constexpr bool isBme280ChipId(uint8_t chipId) { return chipId == 0x60; }

/// Configuration for BME280 driver.
///
/// Driver instances are non-owning: callbacks and user pointers must remain
/// valid for the lifetime of the driver configuration.
/// The application owns shared-bus serialization, timeout policy, recovery, and
/// all platform resources; the core driver never resets or reconfigures the bus.
///
/// `nowMs` is optional for `begin()` but required for compatibility
/// measurement scheduling. `pollJob(nowMs)`, `tick(nowMs)`, and `nowMs(user)`
/// must use the same monotonic clock.
///
/// The address must be 0x76 or 0x77. I2C and NVM timeouts must be nonzero and
/// less than INT32_MAX. Conversion-ready grace must be nonzero and small enough
/// that the worst normal-mode freshness interval (two maximum conversions plus
/// one standby interval) remains within the signed wrap-safe half range. An
/// offlineThreshold of zero is normalized to one when the configuration is
/// accepted.
struct Config {
  // === I2C Transport (required) ===
  I2cWriteFn i2cWrite = nullptr;        ///< I2C write function pointer
  I2cWriteReadFn i2cWriteRead = nullptr; ///< I2C write-read function pointer
  void* i2cUser = nullptr;               ///< User context for callbacks

  // === Timing Hooks (optional for begin, required for measurement scheduling) ===
  NowMsFn nowMs = nullptr;               ///< Monotonic millisecond source; required by requestMeasurement()
  void* timeUser = nullptr;              ///< User context for timing hook
  
  // === Device Settings ===
  uint8_t i2cAddress = 0x76;             ///< 0x76 (SDO=GND) or 0x77 (SDO=VDDIO)
  uint32_t i2cTimeoutMs = 50;            ///< Per-callback timeout: 1..INT32_MAX-1 ms
  uint32_t nvmReadyTimeoutMs = 10;       ///< NVM deadline: 1..INT32_MAX-1 ms
  uint32_t conversionReadyTimeoutMs = 20; ///< Nonzero chip-ready grace; maximum is wrap-safety constrained

  // === Measurement Settings ===
  Oversampling osrsT = Oversampling::X1; ///< Temperature oversampling
  Oversampling osrsP = Oversampling::X1; ///< Pressure oversampling
  Oversampling osrsH = Oversampling::X1; ///< Humidity oversampling
  Filter filter = Filter::OFF;           ///< IIR filter coefficient
  Standby standby = Standby::MS_125;     ///< Standby time (normal mode)
  Mode mode = Mode::FORCED;              ///< Operating mode
  
  // === Health Tracking ===
  uint8_t offlineThreshold = 5;          ///< Failures before OFFLINE; zero is normalized to one
};

} // namespace BME280
