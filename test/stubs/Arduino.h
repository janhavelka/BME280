/// @file Arduino.h
/// @brief Minimal Arduino include stub for native transport-adapter tests.
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

inline uint32_t gMillisValue = 0;
inline void setMillis(uint32_t value) { gMillisValue = value; }
inline uint32_t millis() { return gMillisValue; }

// Native model of wired-AND GPIO levels during example startup bus clear.
inline uint32_t gMicrosValue = 0;
static constexpr int INPUT_PULLUP = 2;
static constexpr int OUTPUT = 3;
static constexpr int OUTPUT_OPEN_DRAIN = 0x13;
static constexpr int LOW = 0;
static constexpr int HIGH = 1;
struct StubPin {
  int mode = INPUT_PULLUP;
  int level = HIGH;
  bool heldLow = false;
  uint32_t lowReadsRemaining = 0;
};
inline StubPin gStubPins[64];
inline uint32_t gActiveHighWrites = 0;
inline void resetStubPins() {
  for (auto& pin : gStubPins) pin = StubPin{};
  gActiveHighWrites = 0;
}
inline void pinMode(int pin, int mode) { gStubPins[pin].mode = mode; }
inline void digitalWrite(int pin, int level) {
  gStubPins[pin].level = level;
  if (level == HIGH && gStubPins[pin].mode == OUTPUT) ++gActiveHighWrites;
}
inline int digitalRead(int pin) {
  auto& value = gStubPins[pin];
  if (value.heldLow) return LOW;
  if (value.lowReadsRemaining > 0) { --value.lowReadsRemaining; return LOW; }
  return value.level;
}


inline uint32_t micros() { return gMicrosValue; }
inline void delay(uint32_t ms) { gMicrosValue += ms * 1000U; }
inline void delayMicroseconds(uint32_t us) { gMicrosValue += us; }
