/// @file Arduino.h
/// @brief Minimal Arduino stub for native testing
#pragma once

#include <cstdint>
#include <cstddef>
#include <cstring>

// Basic types
using byte = uint8_t;

// Timing stubs
inline uint32_t millis() { return 0; }
inline uint32_t micros() { return 0; }
inline void delay(uint32_t ms) { (void)ms; }
inline void delayMicroseconds(uint32_t us) { (void)us; }

// Serial stub
class SerialClass {
public:
  void begin(uint32_t baud) { (void)baud; }
  void print(const char* s) { (void)s; }
  void println(const char* s = "") { (void)s; }
  void printf(const char* fmt, ...) { (void)fmt; }
  int available() { return 0; }
  int read() { return -1; }
  operator bool() { return true; }
};

extern SerialClass Serial;

// String class (minimal stub)
class String {
public:
  String() = default;
  String(const char* s) : _data(s ? s : "") {}
  const char* c_str() const { return _data.c_str(); }
  size_t length() const { return _data.length(); }
  void trim() {}
  bool startsWith(const char* prefix) const {
    return _data.find(prefix) == 0;
  }
  String substring(size_t start) const {
    return String(_data.substr(start).c_str());
  }
  int toInt() const { return std::stoi(_data); }
  String& operator+=(char c) { _data += c; return *this; }
private:
  std::string _data;
};
