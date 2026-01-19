/// @file Config.h
/// @brief Configuration structure for BME280 driver
#pragma once

#include <cstddef>
#include <cstdint>
#include "BME280/Status.h"

namespace BME280 {

/// I2C write callback signature
/// @param addr     I2C device address (7-bit)
/// @param data     Pointer to data to write
/// @param len      Number of bytes to write
/// @param timeoutMs Maximum time to wait for completion
/// @param user     User context pointer passed through from Config
/// @return Status indicating success or failure
using I2cWriteFn = Status (*)(uint8_t addr, const uint8_t* data, size_t len,
                              uint32_t timeoutMs, void* user);

/// I2C write-then-read callback signature
/// @param addr     I2C device address (7-bit)
/// @param txData   Pointer to data to write
/// @param txLen    Number of bytes to write
/// @param rxData   Pointer to buffer for read data
/// @param rxLen    Number of bytes to read
/// @param timeoutMs Maximum time to wait for completion
/// @param user     User context pointer passed through from Config
/// @return Status indicating success or failure
using I2cWriteReadFn = Status (*)(uint8_t addr, const uint8_t* txData, size_t txLen,
                                  uint8_t* rxData, size_t rxLen, uint32_t timeoutMs,
                                  void* user);

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
  FORCED = 1, ///< Single measurement, returns to sleep
  NORMAL = 3  ///< Continuous measurements
};

/// IIR filter coefficient
enum class Filter : uint8_t {
  OFF = 0,
  X2 = 1,
  X4 = 2,
  X8 = 3,
  X16 = 4
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

/// Configuration for BME280 driver
struct Config {
  // === I2C Transport (required) ===
  I2cWriteFn i2cWrite = nullptr;        ///< I2C write function pointer
  I2cWriteReadFn i2cWriteRead = nullptr; ///< I2C write-read function pointer
  void* i2cUser = nullptr;               ///< User context for callbacks
  
  // === Device Settings ===
  uint8_t i2cAddress = 0x76;             ///< 0x76 (SDO=GND) or 0x77 (SDO=VDD)
  uint32_t i2cTimeoutMs = 50;            ///< I2C transaction timeout in ms

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
