/**
 * @file IdfArduinoCompat.h
 * @brief ESP-IDF compatibility layer for the Arduino-shaped BME280 CLI.
 *
 * Example-only glue. It provides the small Serial/timing/String surface used by
 * examples/01_basic_bringup_cli so the native ESP-IDF example can expose the
 * same user-visible CLI without linking the Arduino framework.
 */

#pragma once

#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifndef BME280_EXAMPLE_PLATFORM_IDF
#define BME280_EXAMPLE_PLATFORM_IDF 1
#endif

#ifndef F
#define F(value) value
#endif

inline uint32_t millis() {
  return static_cast<uint32_t>(esp_timer_get_time() / 1000LL);
}

inline uint32_t micros() {
  return static_cast<uint32_t>(esp_timer_get_time());
}

inline void delayMicroseconds(uint32_t us) {
  esp_rom_delay_us(us);
}

inline TickType_t idfExampleDelayTicks(uint32_t ms) {
  TickType_t ticks = pdMS_TO_TICKS(ms);
  if (ticks == 0 && ms > 0U) {
    ticks = 1;
  }
  return ticks;
}

inline void delay(uint32_t ms) {
  vTaskDelay(idfExampleDelayTicks(ms));
}

inline void yield() {
  vTaskDelay(1);
}

class String {
 public:
  String() = default;
  String(const char* value) { assign(value); }

  String& operator=(const char* value) {
    assign(value);
    return *this;
  }

  String& operator=(const String& other) {
    if (this != &other) {
      assign(other.c_str());
    }
    return *this;
  }

  size_t length() const { return _len; }
  const char* c_str() const { return _data; }
  bool reserve(size_t size) const { return size <= kCapacity; }

  void trim() {
    size_t start = 0U;
    while (start < _len && isspace(static_cast<unsigned char>(_data[start]))) {
      ++start;
    }
    size_t end = _len;
    while (end > start && isspace(static_cast<unsigned char>(_data[end - 1U]))) {
      --end;
    }
    if (start > 0U || end < _len) {
      const size_t newLen = end - start;
      memmove(_data, _data + start, newLen);
      _len = newLen;
      _data[_len] = '\0';
    }
  }

  bool startsWith(const char* prefix) const {
    if (prefix == nullptr) {
      return false;
    }
    const size_t prefixLen = strlen(prefix);
    return prefixLen <= _len && strncmp(_data, prefix, prefixLen) == 0;
  }

  String substring(size_t start) const {
    return substring(start, _len);
  }

  String substring(size_t start, size_t end) const {
    String out;
    if (start >= _len) {
      return out;
    }
    if (end > _len) {
      end = _len;
    }
    if (end < start) {
      end = start;
    }
    out.assign(_data + start, end - start);
    return out;
  }

  int indexOf(char needle) const {
    const char* found = strchr(_data, needle);
    if (found == nullptr) {
      return -1;
    }
    return static_cast<int>(found - _data);
  }

  long toInt() const {
    return strtol(_data, nullptr, 10);
  }

  String& operator+=(char c) {
    if (_len < kCapacity) {
      _data[_len++] = c;
      _data[_len] = '\0';
    }
    return *this;
  }

  void remove(size_t index) {
    if (index >= _len) {
      return;
    }
    _len = index;
    _data[_len] = '\0';
  }

 private:
  static constexpr size_t kCapacity = 160U;

  void assign(const char* value) {
    assign(value, value ? strlen(value) : 0U);
  }

  void assign(const char* value, size_t len) {
    if (value == nullptr) {
      _len = 0U;
      _data[0] = '\0';
      return;
    }
    if (len > kCapacity) {
      len = kCapacity;
    }
    memcpy(_data, value, len);
    _len = len;
    _data[_len] = '\0';
  }

  char _data[kCapacity + 1U] = {};
  size_t _len = 0U;
};

inline bool operator==(const String& lhs, const char* rhs) {
  return strcmp(lhs.c_str(), rhs ? rhs : "") == 0;
}

inline bool operator==(const char* lhs, const String& rhs) {
  return rhs == lhs;
}

inline bool operator!=(const String& lhs, const char* rhs) {
  return !(lhs == rhs);
}

class IdfConsole {
 public:
  void begin(unsigned long) {
    setvbuf(stdout, nullptr, _IONBF, 0);
    const int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    if (flags >= 0) {
      (void)fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
    }
  }

  explicit operator bool() const { return true; }

  int available() {
    pollInput();
    return static_cast<int>(_count);
  }

  int read() {
    pollInput();
    if (_count == 0U) {
      return -1;
    }
    const uint8_t value = _rx[_tail];
    _tail = (_tail + 1U) % kRxCapacity;
    --_count;
    return static_cast<int>(value);
  }

  size_t write(uint8_t value) {
    return fwrite(&value, 1U, 1U, stdout);
  }

  int printf(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    const int written = vfprintf(stdout, fmt, args);
    va_end(args);
    return written;
  }

  void print(const char* value) {
    if (value != nullptr) {
      fputs(value, stdout);
    }
  }

  void print(char value) {
    (void)write(static_cast<uint8_t>(value));
  }

  void println() {
    (void)write(static_cast<uint8_t>('\n'));
  }

  void println(const char* value) {
    print(value);
    println();
  }

  void flush() {
    fflush(stdout);
  }

 private:
  static constexpr size_t kRxCapacity = 256U;

  void pollInput() {
    while (_count < kRxCapacity) {
      uint8_t value = 0U;
      const ssize_t readCount = ::read(STDIN_FILENO, &value, 1U);
      if (readCount == 1) {
        _rx[_head] = value;
        _head = (_head + 1U) % kRxCapacity;
        ++_count;
        continue;
      }
      if (readCount < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
        return;
      }
      return;
    }
  }

  uint8_t _rx[kRxCapacity] = {};
  size_t _head = 0U;
  size_t _tail = 0U;
  size_t _count = 0U;
};

extern IdfConsole Serial;
