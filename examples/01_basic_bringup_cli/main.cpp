/// @file main.cpp
/// @brief Basic bringup example for {DEVICE_NAME}
/// @note This is an EXAMPLE, not part of the library

#include <Arduino.h>
#include "common/Log.h"
#include "common/BoardConfig.h"
#include "common/I2cTransport.h"
#include "common/I2cScanner.h"

#include "{NAMESPACE}/{DEVICE}.h"

// ============================================================================
// Globals
// ============================================================================

{NAMESPACE}::{DEVICE} device;
bool verboseMode = false;

// ============================================================================
// Helper Functions
// ============================================================================

/// Convert error code to string
const char* errToStr({NAMESPACE}::Err err) {
  using namespace {NAMESPACE};
  switch (err) {
    case Err::OK:                return "OK";
    case Err::NOT_INITIALIZED:   return "NOT_INITIALIZED";
    case Err::INVALID_CONFIG:    return "INVALID_CONFIG";
    case Err::I2C_ERROR:         return "I2C_ERROR";
    case Err::TIMEOUT:           return "TIMEOUT";
    case Err::INVALID_PARAM:     return "INVALID_PARAM";
    case Err::DEVICE_NOT_FOUND:  return "DEVICE_NOT_FOUND";
    case Err::BUSY:              return "BUSY";
    case Err::IN_PROGRESS:       return "IN_PROGRESS";
    default:                     return "UNKNOWN";
  }
}

/// Convert driver state to string
const char* stateToStr({NAMESPACE}::DriverState st) {
  using namespace {NAMESPACE};
  switch (st) {
    case DriverState::UNINIT:   return "UNINIT";
    case DriverState::READY:    return "READY";
    case DriverState::DEGRADED: return "DEGRADED";
    case DriverState::OFFLINE:  return "OFFLINE";
    default:                    return "UNKNOWN";
  }
}

/// Print status details
void printStatus(const {NAMESPACE}::Status& st) {
  Serial.printf("  Status: %s (code=%u, detail=%ld)\n", 
                errToStr(st.code), 
                static_cast<unsigned>(st.code), 
                static_cast<long>(st.detail));
  if (st.msg && st.msg[0]) {
    Serial.printf("  Message: %s\n", st.msg);
  }
}

/// Print driver health information
void printDriverHealth() {
  Serial.println("=== Driver State ===");
  Serial.printf("  State: %s\n", stateToStr(device.state()));
  Serial.printf("  Consecutive failures: %u\n", device.consecutiveFailures());
  Serial.printf("  Total failures: %lu\n", static_cast<unsigned long>(device.totalFailures()));
  Serial.printf("  Total success: %lu\n", static_cast<unsigned long>(device.totalSuccess()));
  Serial.printf("  Last OK at: %lu ms\n", static_cast<unsigned long>(device.lastOkMs()));
  Serial.printf("  Last error at: %lu ms\n", static_cast<unsigned long>(device.lastErrorMs()));
  if (device.lastError().code != {NAMESPACE}::Err::OK) {
    Serial.printf("  Last error: %s\n", errToStr(device.lastError().code));
  }
}

/// Print help
void printHelp() {
  Serial.println("=== Commands ===");
  Serial.println("  help              - Show this help");
  Serial.println("  scan              - Scan I2C bus");
  Serial.println("  probe             - Probe device (no health tracking)");
  Serial.println("  drv               - Show driver state and health");
  Serial.println("  recover           - Attempt recovery");
  Serial.println("  verbose 0|1       - Set verbose mode");
  // TODO: Add your device-specific commands
  // Serial.println("  read              - Read value");
}

// ============================================================================
// Command Processing
// ============================================================================

void processCommand(const String& cmd) {
  cmd.trim();
  
  if (cmd == "help" || cmd == "?") {
    printHelp();
  }
  else if (cmd == "scan") {
    i2c::scan();
  }
  else if (cmd == "probe") {
    LOGI("Probing device (no health tracking)...");
    auto st = device.probe();
    printStatus(st);
  }
  else if (cmd == "drv") {
    printDriverHealth();
  }
  else if (cmd == "recover") {
    LOGI("Attempting recovery...");
    auto st = device.recover();
    printStatus(st);
    printDriverHealth();
  }
  else if (cmd.startsWith("verbose ")) {
    int val = cmd.substring(8).toInt();
    verboseMode = (val != 0);
    LOGI("Verbose mode: %s", verboseMode ? "ON" : "OFF");
  }
  // TODO: Add your device-specific command handlers
  else {
    LOGW("Unknown command: %s", cmd.c_str());
  }
}

// ============================================================================
// Setup and Loop
// ============================================================================

void setup() {
  board::initSerial();
  delay(100);
  
  LOGI("=== {DEVICE_NAME} Bringup Example ===");
  
  // Initialize I2C
  if (!board::initI2c()) {
    LOGE("Failed to initialize I2C");
    return;
  }
  LOGI("I2C initialized (SDA=%d, SCL=%d)", board::I2C_SDA, board::I2C_SCL);
  
  // Scan bus
  i2c::scan();
  
  // Configure driver
  {NAMESPACE}::Config cfg;
  cfg.i2cWrite = transport::wireWrite;
  cfg.i2cWriteRead = transport::wireWriteRead;
  cfg.i2cAddress = 0x00;  // TODO: Set your device address
  cfg.i2cTimeoutMs = board::I2C_TIMEOUT_MS;
  cfg.offlineThreshold = 5;
  
  // Initialize driver
  auto st = device.begin(cfg);
  if (!st.ok()) {
    LOGE("Failed to initialize device");
    printStatus(st);
    return;
  }
  
  LOGI("Device initialized successfully");
  printDriverHealth();
  
  Serial.println("\nType 'help' for commands");
  Serial.print("> ");
}

void loop() {
  // Process tick
  device.tick(millis());
  
  // Process serial input
  static String inputBuffer;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (inputBuffer.length() > 0) {
        processCommand(inputBuffer);
        inputBuffer = "";
        Serial.print("> ");
      }
    } else {
      inputBuffer += c;
    }
  }
}
