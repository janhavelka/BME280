/// @file PlatformTime.h
/// @brief Private framework-specific time fallback for the BME280 core.
#pragma once

#include <cstdint>

#if defined(ESP_PLATFORM)
#include <esp_timer.h>
#elif defined(ARDUINO)
#define BME280_PLATFORM_HAS_ARDUINO_TIME 1
#elif defined(__has_include)
#if __has_include(<Arduino.h>)
#define BME280_PLATFORM_HAS_ARDUINO_TIME 1
#endif
#endif

#if defined(BME280_PLATFORM_HAS_ARDUINO_TIME)
#include <Arduino.h>
#endif

namespace BME280 {
namespace platform {

inline uint32_t nowMs() {
#if defined(ESP_PLATFORM)
  return static_cast<uint32_t>(esp_timer_get_time() / 1000LL);
#elif defined(ARDUINO) || defined(BME280_PLATFORM_HAS_ARDUINO_TIME)
  return millis();
#else
  return 0U;
#endif
}

}  // namespace platform
}  // namespace BME280
