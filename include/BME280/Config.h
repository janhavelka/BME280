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
  static constexpr TransportResult Complete(size_t written, size_t read = 0) {
    return TransportResult{TransportErr::OK, 0, written, read};
  }

  /// Construct a terminal transport error result.
  /// @param error Terminal transport error
  /// @param detailCode Adapter-owned numeric diagnostic
  /// @param written Number of bytes physically written before failure
  /// @param read Number of bytes physically read before failure
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
/// requestMeasurement() and tick(nowMs) must use the same monotonic timebase.
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

/// Measurement mode
enum class Mode : uint8_t {
  SLEEP = 0,  ///< No measurements, lowest power
  FORCED = 1, ///< On-demand single measurement; requestMeasurement() triggers conversion
  NORMAL = 3  ///< Continuous measurements
};

/// IIR filter coefficient
enum class Filter : uint8_t {
  OFF = 0, ///< Filter disabled
  X2 = 1,  ///< 2x filter coefficient
  X4 = 2,  ///< 4x filter coefficient
  X8 = 3,  ///< 8x filter coefficient
  X16 = 4  ///< 16x filter coefficient
};

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

/// Configuration for BME280 driver.
///
/// Driver instances are non-owning: callbacks and user pointers must remain
/// valid for the lifetime of the driver configuration.
/// The application owns shared-bus serialization, timeout policy, recovery, and
/// all platform resources; the core driver never resets or reconfigures the bus.
///
/// `nowMs` is optional for `begin()` but required for measurement scheduling.
/// `tick(nowMs)` and `nowMs(user)` should use the same monotonic clock.
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
  uint32_t i2cTimeoutMs = 50;            ///< I2C transaction timeout in ms
  uint32_t nvmReadyTimeoutMs = 10;        ///< NVM ready timeout after POR/reset in ms
  uint32_t conversionReadyTimeoutMs = 20; ///< Grace period for conversion/idle readiness

  // === Measurement Settings ===
  Oversampling osrsT = Oversampling::X1; ///< Temperature oversampling
  Oversampling osrsP = Oversampling::X1; ///< Pressure oversampling
  Oversampling osrsH = Oversampling::X1; ///< Humidity oversampling
  Filter filter = Filter::OFF;           ///< IIR filter coefficient
  Standby standby = Standby::MS_125;     ///< Standby time (normal mode)
  Mode mode = Mode::FORCED;              ///< Operating mode
  
  // === Health Tracking ===
  uint8_t offlineThreshold = 5;          ///< Consecutive failures before OFFLINE state
};

} // namespace BME280
