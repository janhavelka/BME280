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

}  // namespace cli
