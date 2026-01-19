/// @file Wire.h
/// @brief Minimal Wire stub for native testing
#pragma once

#include <cstdint>
#include <cstddef>

class TwoWire {
public:
  void begin(int sda = -1, int scl = -1) { (void)sda; (void)scl; }
  void setClock(uint32_t freq) { (void)freq; }
  void setTimeOut(uint32_t timeoutMs) { (void)timeoutMs; }
  
  void beginTransmission(uint8_t addr) { _addr = addr; _txLen = 0; }
  size_t write(uint8_t data) { _txBuf[_txLen++] = data; return 1; }
  size_t write(const uint8_t* data, size_t len) { 
    for (size_t i = 0; i < len && _txLen < sizeof(_txBuf); i++) {
      _txBuf[_txLen++] = data[i];
    }
    return len;
  }
  uint8_t endTransmission(bool stop = true) { (void)stop; return 0; }
  
  size_t requestFrom(uint8_t addr, size_t len) { 
    (void)addr;
    _rxLen = len;
    _rxIdx = 0;
    return len;
  }
  
  int available() { return _rxLen - _rxIdx; }
  int read() { 
    if (_rxIdx < _rxLen) {
      return _rxBuf[_rxIdx++];
    }
    return -1;
  }

  // Test helper: set data to return on next read
  void _setReadData(const uint8_t* data, size_t len) {
    for (size_t i = 0; i < len && i < sizeof(_rxBuf); i++) {
      _rxBuf[i] = data[i];
    }
  }

private:
  uint8_t _addr = 0;
  uint8_t _txBuf[64] = {};
  size_t _txLen = 0;
  uint8_t _rxBuf[64] = {};
  size_t _rxLen = 0;
  size_t _rxIdx = 0;
};

extern TwoWire Wire;
