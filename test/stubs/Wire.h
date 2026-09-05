/// @file Wire.h
/// @brief Minimal Wire stub for native testing
#pragma once

#include <cstdint>
#include <cstddef>

#ifndef I2C_BUFFER_LENGTH
#define I2C_BUFFER_LENGTH 128
#endif

class TwoWire {
public:
  bool begin(int sda = -1, int scl = -1) { (void)sda; (void)scl; return _beginResult; }
  bool setClock(uint32_t freq) { (void)freq; return _clockResult; }
  void setTimeOut(uint32_t timeoutMs) { _timeoutMs = timeoutMs; }
  uint32_t getTimeOut() const { return _timeoutMs; }
  
  void beginTransmission(uint8_t addr) {
    (void)addr;
    _txLen = 0;
  }
  size_t write(const uint8_t* data, size_t len) {
    (void)data;
    const size_t available = _txLen < I2C_BUFFER_LENGTH
        ? I2C_BUFFER_LENGTH - _txLen
        : 0;
    const size_t accepted = (len < available) ? len : available;
    _txLen += accepted;
    return accepted;
  }
  uint8_t endTransmission(bool stop = true) { (void)stop; return _endTransmissionResult; }
  
  size_t requestFrom(uint8_t addr, size_t len) { 
    (void)addr;
    if (_requestFromOverrideEnabled) {
      _rxLen = _requestFromOverride;
    } else {
      _rxLen = len;
    }
    _rxIdx = 0;
    return _rxLen;
  }
  
  int available() { return _rxLen - _rxIdx; }
  int read() { 
    if (_rxIdx < _rxLen) {
      ++_rxIdx;
      return 0;
    }
    return -1;
  }

  void _setBeginResult(bool result) { _beginResult = result; }
  void _clearBeginResult() { _beginResult = true; }
  void _setClockResult(bool result) { _clockResult = result; }
  void _clearClockResult() { _clockResult = true; }
  void _setEndTransmissionResult(uint8_t result) { _endTransmissionResult = result; }
  void _clearEndTransmissionResult() { _endTransmissionResult = 0; }
  void _setRequestFromResult(size_t len) {
    _requestFromOverrideEnabled = true;
    _requestFromOverride = len;
  }
  void _clearRequestFromOverride() { _requestFromOverrideEnabled = false; }

private:
  size_t _txLen = 0;
  size_t _rxLen = 0;
  size_t _rxIdx = 0;
  uint32_t _timeoutMs = 0;
  bool _beginResult = true;
  bool _clockResult = true;
  uint8_t _endTransmissionResult = 0;
  bool _requestFromOverrideEnabled = false;
  size_t _requestFromOverride = 0;
};

extern TwoWire Wire;
