/**
 * @file CliStyle.h
 * @brief Shared CLI styling helpers for examples.
 *
 * NOT part of the library API. The library itself does not log or format
 * terminal output.
 */

#pragma once

#include <Arduino.h>

#include "Log.h"

namespace cli {

inline constexpr size_t HELP_COMMAND_WIDTH = 32U;

inline const char* resetColor() { return LOG_COLOR_RESET; }
inline const char* okColor(bool ok) { return LOG_COLOR_RESULT(ok); }
inline const char* enabledColor(bool enabled) {
  return enabled ? LOG_COLOR_GREEN : LOG_COLOR_RESET;
}
inline const char* yesNoColor(bool value) {
  return value ? LOG_COLOR_GREEN : LOG_COLOR_YELLOW;
}
inline const char* zeroGoodColor(uint32_t value) {
  return (value == 0U) ? LOG_COLOR_GREEN : LOG_COLOR_RED;
}
inline const char* nonZeroGoodColor(uint32_t value) {
  return (value > 0U) ? LOG_COLOR_GREEN : LOG_COLOR_YELLOW;
}
inline const char* warningIfNonZeroColor(uint32_t value) {
  return (value > 0U) ? LOG_COLOR_YELLOW : LOG_COLOR_RESET;
}
inline const char* successRateColor(float pct) {
  if (pct >= 99.9f) return LOG_COLOR_GREEN;
  if (pct >= 80.0f) return LOG_COLOR_YELLOW;
  return LOG_COLOR_RED;
}

inline void printHelpHeader(const char* title) {
  LOG_SERIAL.printf("%s=== %s ===%s\n", LOG_COLOR_CYAN, title, LOG_COLOR_RESET);
}

inline void printHelpSection(const char* title) {
  LOG_SERIAL.printf("\n%s[%s]%s\n", LOG_COLOR_GREEN, title, LOG_COLOR_RESET);
}

inline void printHelpItem(const char* command, const char* description) {
  LOG_SERIAL.printf("  %s%-*s%s - %s\n",
                    LOG_COLOR_CYAN,
                    static_cast<int>(HELP_COMMAND_WIDTH),
                    command,
                    LOG_COLOR_RESET,
                    description);
}

inline void printPrompt() {
  LOG_SERIAL.print("> ");
}

inline void printUnknownCommand(const char* command) {
  LOGW("Unknown command: %s", command ? command : "(null)");
}

}  // namespace cli
