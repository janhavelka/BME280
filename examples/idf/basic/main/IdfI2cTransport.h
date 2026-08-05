/// @file IdfI2cTransport.h
/// @brief ESP-IDF I2C transport adapter for the BME280 example.
#pragma once

#include <cstddef>
#include <cstdint>

#include <driver/i2c_master.h>
#include <esp_err.h>

#include "BME280/Config.h"

struct IdfI2cContext {
  i2c_master_bus_handle_t bus = nullptr;
  i2c_master_dev_handle_t device = nullptr;
  uint8_t address = 0x76;
  esp_err_t lastError = ESP_OK;
};

/// Example-only transport callback counters used by CLI/HIL diagnostics.
struct IdfI2cTransferStats {
  uint32_t read = 0;   ///< Combined write/read callback attempts
  uint32_t write = 0;  ///< Write-only callback attempts
  uint32_t total = 0;  ///< All validated transport callback attempts
};

IdfI2cContext& bme280IdfI2cContext();
bool bme280IdfInitI2c(int sda, int scl, uint32_t freqHz, uint8_t address);
void bme280IdfDeinitI2c();
void bme280IdfResetTransferStats();
IdfI2cTransferStats bme280IdfTransferStats();

BME280::TransportResult idfI2cWrite(uint8_t addr, const uint8_t* data,
                                    size_t len, uint32_t timeoutMs, void* user);

BME280::TransportResult idfI2cWriteRead(uint8_t addr,
                                        const uint8_t* txData, size_t txLen,
                                        uint8_t* rxData, size_t rxLen,
                                        uint32_t timeoutMs, void* user);
