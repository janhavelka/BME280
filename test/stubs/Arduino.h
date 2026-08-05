/// @file Arduino.h
/// @brief Minimal Arduino include stub for native transport-adapter tests.
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

inline uint32_t gMillisValue = 0;
inline void setMillis(uint32_t value) { gMillisValue = value; }
inline uint32_t millis() { return gMillisValue; }
