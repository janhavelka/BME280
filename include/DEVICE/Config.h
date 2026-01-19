/// @file Config.h
/// @brief Configuration structure for {DEVICE_NAME} driver
#pragma once

#include <cstdint>
#include "{NAMESPACE}/Status.h"

namespace {NAMESPACE} {

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

/// Configuration for {DEVICE_NAME} driver
struct Config {
  // === I2C Transport (required) ===
  I2cWriteFn i2cWrite = nullptr;        ///< I2C write function pointer
  I2cWriteReadFn i2cWriteRead = nullptr; ///< I2C write-read function pointer
  void* i2cUser = nullptr;               ///< User context for callbacks
  
  // === Device Settings ===
  uint8_t i2cAddress = 0x00;             ///< I2C device address (set to your device default)
  uint32_t i2cTimeoutMs = 50;            ///< I2C transaction timeout in ms
  
  // === Health Tracking ===
  uint8_t offlineThreshold = 5;          ///< Consecutive failures before OFFLINE state
  
  // === Device-Specific Settings ===
  // Add your device-specific configuration fields here
  // Examples:
  // uint8_t sampleRate = 0;
  // bool enableFeatureX = false;
};

} // namespace {NAMESPACE}
