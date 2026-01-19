/**
 * @file BoardConfig.h
 * @brief Example board configuration for ESP32-S2 / ESP32-S3 reference hardware.
 *
 * These are convenience defaults for reference designs only.
 * NOT part of the library API. Override for your hardware.
 *
 * @warning The library itself is board-agnostic. All pins are passed via Config.
 *          These defaults are provided for examples only.
 */

#pragma once

#include <stdint.h>

#include "examples/common/I2cTransport.h"

namespace board {

// ====================================================================
// EXAMPLE DEFAULTS - ESP32-S2 / ESP32-S3 REFERENCE HARDWARE
// ====================================================================
// These values are NOT library defaults. They are example-only values.
// Override them for your board by creating your own BoardConfig.h or
// passing explicit values to Config structs in your application.
// ====================================================================

/// @brief I2C SDA pin (data line). Example default for ESP32-S2/S3.
static constexpr int I2C_SDA = 8;

/// @brief I2C SCL pin (clock line). Example default for ESP32-S2/S3.
static constexpr int I2C_SCL = 9;

/// @brief I2C clock frequency in Hz.
static constexpr uint32_t I2C_FREQ_HZ = 400000;

/// @brief I2C timeout in milliseconds for example transactions.
static constexpr uint16_t I2C_TIMEOUT_MS = 50;

/// @brief LED pin. Example default for ESP32-S3 (RGB LED on GPIO48).
/// Set to -1 to disable.
static constexpr int LED = 48;

/// @brief Initialize I2C for examples using the default config.
inline bool initI2c() {
  return transport::initWire(I2C_SDA, I2C_SCL, I2C_FREQ_HZ, I2C_TIMEOUT_MS);
}

}  // namespace board
