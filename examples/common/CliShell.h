#pragma once

#include <Arduino.h>

#include "Log.h"

namespace cli_shell {

inline constexpr size_t MAX_LINE_LENGTH = 127U;

inline bool readLine(String& outLine) {
  static String buffer;
  static bool reserved = false;
  static bool overflowed = false;

  if (!reserved) {
    (void)buffer.reserve(MAX_LINE_LENGTH);
    reserved = true;
  }

  while (LOG_SERIAL.available() > 0) {
    const char c = static_cast<char>(LOG_SERIAL.read());

    if (c == '\b' || c == 0x7F) {
      if (!overflowed && buffer.length() > 0U) {
        buffer.remove(buffer.length() - 1U);
      }
      continue;
    }

    if (c == '\r' || c == '\n') {
      if (overflowed) {
        buffer = "";
        overflowed = false;
        continue;
      }
      if (buffer.length() == 0U) {
        continue;
      }
      outLine = buffer;
      buffer = "";
      outLine.trim();
      return outLine.length() > 0U;
    }

    if (overflowed) {
      continue;
    }

    if (buffer.length() < MAX_LINE_LENGTH) {
      buffer += c;
    } else {
      buffer = "";
      overflowed = true;
    }
  }
  return false;
}

}  // namespace cli_shell
