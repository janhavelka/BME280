/// @file I2cScanner.h
/// @brief Simple I2C bus scanner for debugging
/// @note NOT part of the library - examples only
#pragma once

#include <Arduino.h>
#include <Wire.h>
#include "Log.h"

namespace i2c {

/// Scan I2C bus and print found devices
/// @return Number of devices found
inline int scan() {
  LOGI("Scanning I2C bus...");
  
  int count = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    uint8_t error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.printf("  Found device at 0x%02X\n", addr);
      count++;
    }
  }
  
  if (count == 0) {
    LOGW("No I2C devices found");
  } else {
    LOGI("Found %d device(s)", count);
  }
  
  return count;
}

/// Check if a specific address responds
/// @param addr I2C address to check
/// @return true if device responds
inline bool checkAddress(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

} // namespace i2c
