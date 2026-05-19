#pragma once

#include <Arduino.h>

namespace health_view {

inline const char* colorGreen() { return "\033[32m"; }
inline const char* colorYellow() { return "\033[33m"; }
inline const char* colorRed() { return "\033[31m"; }
inline const char* colorGray() { return "\033[90m"; }
inline const char* colorReset() { return "\033[0m"; }

inline const char* boolColor(bool value) {
  return value ? colorGreen() : colorRed();
}

inline const char* failureColor(uint32_t failures) {
  if (failures == 0U) {
    return colorGreen();
  }
  if (failures < 3U) {
    return colorYellow();
  }
  return colorRed();
}

inline const char* successColor(uint32_t successes) {
  return (successes > 0U) ? colorGreen() : colorGray();
}

inline const char* successRateColor(float pct) {
  if (pct >= 99.9f) {
    return colorGreen();
  }
  if (pct >= 80.0f) {
    return colorYellow();
  }
  return colorRed();
}

inline const char* stateToString(int state) {
  switch (state) {
    case 0: return "UNINIT";
    case 1: return "READY";
    case 2: return "DEGRADED";
    case 3: return "OFFLINE";
    default: return "UNKNOWN";
  }
}

template <typename DriverT>
struct Snapshot {
  int state = 0;
  bool online = false;
  uint8_t consecutiveFailures = 0;
  uint32_t totalFailures = 0;
  uint32_t totalSuccess = 0;

  void capture(const DriverT& driver) {
    state = static_cast<int>(driver.state());
    online = driver.isOnline();
    consecutiveFailures = driver.consecutiveFailures();
    totalFailures = driver.totalFailures();
    totalSuccess = driver.totalSuccess();
  }
};

template <typename DriverT>
inline void printHealthView(const DriverT& driver) {
  Snapshot<DriverT> snap;
  snap.capture(driver);
  const uint32_t total = snap.totalSuccess + snap.totalFailures;
  const float pct = (total > 0U)
                        ? (100.0f * static_cast<float>(snap.totalSuccess) /
                           static_cast<float>(total))
                        : 0.0f;

  Serial.printf("Health: state=%s%s%s online=%s%s%s consec=%s%u%s ok=%s%lu%s fail=%s%lu%s rate=%s%.1f%%%s\n",
                failureColor(static_cast<uint32_t>(snap.consecutiveFailures)),
                stateToString(snap.state),
                colorReset(),
                boolColor(snap.online),
                snap.online ? "true" : "false",
                colorReset(),
                failureColor(static_cast<uint32_t>(snap.consecutiveFailures)),
                static_cast<unsigned>(snap.consecutiveFailures),
                colorReset(),
                successColor(snap.totalSuccess),
                static_cast<unsigned long>(snap.totalSuccess),
                colorReset(),
                failureColor(snap.totalFailures),
                static_cast<unsigned long>(snap.totalFailures),
                colorReset(),
                successRateColor(pct),
                pct,
                colorReset());
}

template <typename DriverT>
inline void printHealthDiff(const Snapshot<DriverT>& before,
                            const Snapshot<DriverT>& after) {
  bool changed = false;

  if (before.state != after.state) {
    Serial.printf("  State: %s%s%s -> %s%s%s\n",
                  failureColor(static_cast<uint32_t>(before.consecutiveFailures)),
                  stateToString(before.state),
                  colorReset(),
                  failureColor(static_cast<uint32_t>(after.consecutiveFailures)),
                  stateToString(after.state),
                  colorReset());
    changed = true;
  }
  if (before.online != after.online) {
    Serial.printf("  Online: %s%s%s -> %s%s%s\n",
                  boolColor(before.online),
                  before.online ? "true" : "false",
                  colorReset(),
                  boolColor(after.online),
                  after.online ? "true" : "false",
                  colorReset());
    changed = true;
  }
  if (before.consecutiveFailures != after.consecutiveFailures) {
    Serial.printf("  ConsecFail: %s%u -> %u%s\n",
                  failureColor(static_cast<uint32_t>(after.consecutiveFailures)),
                  static_cast<unsigned>(before.consecutiveFailures),
                  static_cast<unsigned>(after.consecutiveFailures),
                  colorReset());
    changed = true;
  }
  if (before.totalSuccess != after.totalSuccess) {
    Serial.printf("  TotalOK: %lu -> %s%lu (+%lu)%s\n",
                  static_cast<unsigned long>(before.totalSuccess),
                  colorGreen(),
                  static_cast<unsigned long>(after.totalSuccess),
                  static_cast<unsigned long>(after.totalSuccess - before.totalSuccess),
                  colorReset());
    changed = true;
  }
  if (before.totalFailures != after.totalFailures) {
    Serial.printf("  TotalFail: %lu -> %s%lu (+%lu)%s\n",
                  static_cast<unsigned long>(before.totalFailures),
                  colorRed(),
                  static_cast<unsigned long>(after.totalFailures),
                  static_cast<unsigned long>(after.totalFailures - before.totalFailures),
                  colorReset());
    changed = true;
  }
  if (!changed) {
    Serial.println("  (no health changes)");
  }
}

} // namespace health_view

template <typename DriverT>
using HealthSnapshot = health_view::Snapshot<DriverT>;

using health_view::printHealthDiff;
using health_view::printHealthView;
