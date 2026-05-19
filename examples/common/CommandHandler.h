/**
 * @file CommandHandler.h
 * @brief Simple serial command parser for interactive examples.
 *
 * NOT part of the library API. This is a helper for examples.
 */

#pragma once

#include <Arduino.h>

#include "examples/common/Log.h"

namespace cmd {

/**
 * @brief Check if serial data is available and read a line.
 * @param buffer Buffer to store the command (null-terminated).
 * @param bufSize Size of the buffer.
 * @return true if a complete line was read, false otherwise.
 */
inline bool readLine(char* buffer, size_t bufSize) {
  static char cmdBuf[128];
  static size_t cmdLen = 0;

  while (LOG_SERIAL.available()) {
    int c = LOG_SERIAL.read();
    if (c < 0) break;

    if (c == '\n' || c == '\r') {
      if (cmdLen > 0) {
        cmdBuf[cmdLen] = '\0';
        strncpy(buffer, cmdBuf, bufSize - 1);
        buffer[bufSize - 1] = '\0';
        cmdLen = 0;
        return true;
      }
    } else if (cmdLen < sizeof(cmdBuf) - 1) {
      cmdBuf[cmdLen++] = (char)c;
    }
  }

  return false;
}

/**
 * @brief Parse integer argument from command string.
 * @param cmd Command string (e.g., "filter 4").
 * @param keyword Keyword to match (e.g., "filter").
 * @param outValue Pointer to store parsed value.
 * @return true if keyword matched and value parsed.
 */
inline bool parseInt(const char* cmd, const char* keyword, int* outValue) {
  size_t kwLen = strlen(keyword);
  if (strncmp(cmd, keyword, kwLen) != 0) return false;

  const char* valueStr = cmd + kwLen;
  while (*valueStr == ' ' || *valueStr == '\t') valueStr++;

  if (*valueStr == '\0') return false;

  *outValue = atoi(valueStr);
  return true;
}

/**
 * @brief Check if command matches a keyword (case-insensitive).
 * @param cmd Command string.
 * @param keyword Keyword to match.
 * @return true if command starts with keyword.
 */
inline bool match(const char* cmd, const char* keyword) {
  return strncasecmp(cmd, keyword, strlen(keyword)) == 0;
}

}  // namespace cmd
