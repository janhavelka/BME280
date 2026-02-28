#pragma once

#include <Arduino.h>

template <typename DriverT>
inline void printHealthView(const DriverT& driver) {
  Serial.printf("state=%d online=%s failures=%u totalFail=%lu totalOk=%lu\n",
                static_cast<int>(driver.state()), driver.isOnline() ? "true" : "false",
                static_cast<unsigned>(driver.consecutiveFailures()),
                static_cast<unsigned long>(driver.totalFailures()),
                static_cast<unsigned long>(driver.totalSuccess()));
}