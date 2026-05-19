/// @file PlatformTime.h
/// @brief Private framework-neutral time fallback for the BME280 core.
#pragma once

#include <cstdint>

namespace BME280 {
namespace platform {

inline uint32_t nowMs() {
  // Framework examples inject real time through Config::nowMs. The fallback is
  // intentionally inert so the core never includes Arduino or ESP-IDF headers.
  return 0U;
}

}  // namespace platform
}  // namespace BME280
