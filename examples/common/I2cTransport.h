/**
 * @file I2cTransport.h
 * @brief Wire-based I2C transport adapter for BME280 examples.
 *
 * This file provides Wire-compatible I2C callbacks that can be
 * used with the BME280 driver. The library does not depend on Wire
 * directly; this adapter bridges them.
 *
 * NOT part of the library API. Example-only.
 */

#pragma once

#include <Arduino.h>
#include <Wire.h>

#include "BME280/Config.h"

namespace transport {

/// Example-only transport callback counters used by CLI/HIL diagnostics.
struct TransferStats {
  uint32_t read = 0;   ///< Combined write/read callback attempts
  uint32_t write = 0;  ///< Write-only callback attempts
  uint32_t total = 0;  ///< All validated transport callback attempts
};

inline TransferStats& transferStatsStorage() {
  static TransferStats stats{};
  return stats;
}

inline void resetTransferStats() {
  transferStatsStorage() = {};
}

inline TransferStats transferStats() {
  return transferStatsStorage();
}

inline void incrementSaturating(uint32_t& value) {
  if (value != UINT32_MAX) {
    ++value;
  }
}

inline BME280::TransportResult mapWireResult(uint8_t result,
                                             size_t writeCount,
                                             size_t readCount = 0) {
  switch (result) {
    case 0:
      return BME280::TransportResult::Complete(writeCount, readCount);
    case 1:
      return BME280::TransportResult::Error(BME280::TransportErr::OTHER,
                                            result);
    case 2:
      return BME280::TransportResult::Error(
          BME280::TransportErr::NACK_ADDRESS, result);
    case 3:
      return BME280::TransportResult::Error(BME280::TransportErr::NACK_DATA,
                                            result);
    case 4:
      return BME280::TransportResult::Error(BME280::TransportErr::BUS, result);
    case 5:
      return BME280::TransportResult::Error(BME280::TransportErr::TIMEOUT,
                                            result);
    default:
      return BME280::TransportResult::Error(BME280::TransportErr::OTHER,
                                            result);
  }
}

/**
 * @brief Wire-based I2C write implementation.
 *
 * Pass to Config::i2cWrite, and pass &Wire (or a custom TwoWire*) to i2cUser.
 * The timeout parameter is advisory; bus timeout ownership stays with initWire().
 *
 * @param addr I2C 7-bit address
 * @param data Data buffer to send
 * @param len Number of bytes
 * @param timeoutMs Timeout requested by the driver (advisory only)
 * @param user Pointer to TwoWire instance
 * This callback performs exactly one physical transaction and never retries or
 * recovers the bus.
 *
 * @return Terminal transport result with exact counts on success
 */
inline BME280::TransportResult wireWrite(uint8_t addr, const uint8_t* data,
                                         size_t len, uint32_t timeoutMs,
                                         void* user) {
  (void)timeoutMs;

  TwoWire* wire = static_cast<TwoWire*>(user);
  if (wire == nullptr) {
    return BME280::TransportResult::Error(BME280::TransportErr::OTHER, -1);
  }
  if (!data || len == 0) {
    return BME280::TransportResult::Error(BME280::TransportErr::OTHER, -2);
  }

  // Check for oversized writes (ESP32 Wire buffer is 128 bytes)
  if (len > 128) {
    return BME280::TransportResult::Error(BME280::TransportErr::OTHER,
                                          static_cast<int32_t>(len));
  }

  incrementSaturating(transferStatsStorage().write);
  incrementSaturating(transferStatsStorage().total);
  wire->beginTransmission(addr);
  size_t written = wire->write(data, len);
  if (written != len) {
    // No bus attempt has occurred; detail records bytes accepted by Wire.
    return BME280::TransportResult::Error(BME280::TransportErr::OTHER,
                                          static_cast<int32_t>(written));
  }

  uint8_t result = wire->endTransmission(true);  // Send STOP
  return mapWireResult(result, len);
}

/**
 * @brief Wire-based I2C write-read implementation.
 *
 * Pass to Config::i2cWriteRead, and pass &Wire (or a custom TwoWire*) to i2cUser.
 * The timeout parameter is advisory; bus timeout ownership stays with initWire().
 *
 * @param addr I2C 7-bit address
 * @param tx TX buffer to send
 * @param txLen TX length
 * @param rx RX buffer for readback
 * @param rxLen RX length
 * @param timeoutMs Timeout requested by the driver (advisory only)
 * @param user Pointer to TwoWire instance
 * `endTransmission(false)` retains the pointer write without a STOP and the
 * following `requestFrom()` completes one combined repeated-start transaction.
 * The callback never retries or recovers the bus.
 *
 * @return Terminal transport result with exact counts on success
 */
inline BME280::TransportResult wireWriteRead(uint8_t addr, const uint8_t* tx,
                                             size_t txLen, uint8_t* rx,
                                             size_t rxLen, uint32_t timeoutMs,
                                             void* user) {
  (void)timeoutMs;

  TwoWire* wire = static_cast<TwoWire*>(user);
  if (wire == nullptr) {
    return BME280::TransportResult::Error(BME280::TransportErr::OTHER, -1);
  }
  if ((txLen > 0 && tx == nullptr) || (rxLen > 0 && rx == nullptr)) {
    return BME280::TransportResult::Error(BME280::TransportErr::OTHER, -2);
  }
  if (txLen == 0 || rxLen == 0) {
    return BME280::TransportResult::Error(BME280::TransportErr::OTHER, -3);
  }
  if (txLen > 128 || rxLen > 128) {
    return BME280::TransportResult::Error(BME280::TransportErr::OTHER, -4);
  }

  incrementSaturating(transferStatsStorage().read);
  incrementSaturating(transferStatsStorage().total);
  wire->beginTransmission(addr);
  size_t written = wire->write(tx, txLen);
  if (written != txLen) {
    // No bus attempt has occurred; detail records bytes accepted by Wire.
    return BME280::TransportResult::Error(BME280::TransportErr::OTHER,
                                          static_cast<int32_t>(written));
  }

  uint8_t result = wire->endTransmission(false);  // Repeated start
  if (result != 0) {
    return mapWireResult(result, 0);
  }

  size_t read = wire->requestFrom(addr, static_cast<uint8_t>(rxLen));
  if (read != rxLen) {
    // Wire exposes the combined operation's received byte count but not its
    // underlying error. Preserve the exact counts so the core reports
    // I2C_SHORT_TRANSFER instead of inventing an address/data-NACK cause.
    return BME280::TransportResult{BME280::TransportErr::OK, 0, txLen, read};
  }

  for (size_t i = 0; i < rxLen; ++i) {
    if (wire->available()) {
      rx[i] = static_cast<uint8_t>(wire->read());
    } else {
      return BME280::TransportResult{BME280::TransportErr::OK, 0, txLen, i};
    }
  }

  return BME280::TransportResult::Complete(txLen, rxLen);
}

/**
 * @brief Initialize Wire with default pins and frequency.
 *
 * @param sda SDA pin number
 * @param scl SCL pin number
 * @param freq I2C clock frequency in Hz (default 400kHz)
 * @param timeoutMs I2C timeout in milliseconds (default 50ms)
 * @return true on success
 */
inline bool initWire(int sda, int scl, uint32_t freq = 400000, uint16_t timeoutMs = 50) {
#if defined(ARDUINO_ARCH_ESP32)
  // Bus recovery belongs to explicit application setup. Transport callbacks
  // above never invoke this procedure or retry a transaction.
  // Toggle SCL to release any stuck slave.
  pinMode(scl, OUTPUT);
  pinMode(sda, INPUT_PULLUP);
  for (int i = 0; i < 9; i++) {
    digitalWrite(scl, LOW);
    delayMicroseconds(5);
    digitalWrite(scl, HIGH);
    delayMicroseconds(5);
  }
  // Generate STOP condition
  pinMode(sda, OUTPUT);
  digitalWrite(sda, LOW);
  delayMicroseconds(5);
  digitalWrite(scl, HIGH);
  delayMicroseconds(5);
  digitalWrite(sda, HIGH);
  delayMicroseconds(5);
#endif

  Wire.begin(sda, scl);
  Wire.setClock(freq);
  Wire.setTimeOut(timeoutMs);
  return true;
}

}  // namespace transport
