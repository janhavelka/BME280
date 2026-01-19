/// @file main.cpp
/// @brief Basic bringup example for BME280
/// @note This is an EXAMPLE, not part of the library

#include <Arduino.h>
#include "common/Log.h"
#include "common/BoardConfig.h"
#include "common/I2cTransport.h"
#include "common/I2cScanner.h"

#include "BME280/BME280.h"

// ============================================================================
// Globals
// ============================================================================

BME280::BME280 device;
bool verboseMode = false;
bool pendingRead = false;
uint32_t pendingStartMs = 0;
int stressRemaining = 0;

// ============================================================================
// Helper Functions
// ============================================================================

const char* errToStr(BME280::Err err) {
  using namespace BME280;
  switch (err) {
    case Err::OK:                  return "OK";
    case Err::NOT_INITIALIZED:     return "NOT_INITIALIZED";
    case Err::INVALID_CONFIG:      return "INVALID_CONFIG";
    case Err::I2C_ERROR:           return "I2C_ERROR";
    case Err::TIMEOUT:             return "TIMEOUT";
    case Err::INVALID_PARAM:       return "INVALID_PARAM";
    case Err::DEVICE_NOT_FOUND:    return "DEVICE_NOT_FOUND";
    case Err::CHIP_ID_MISMATCH:    return "CHIP_ID_MISMATCH";
    case Err::CALIBRATION_INVALID: return "CALIBRATION_INVALID";
    case Err::MEASUREMENT_NOT_READY: return "MEASUREMENT_NOT_READY";
    case Err::COMPENSATION_ERROR:  return "COMPENSATION_ERROR";
    case Err::BUSY:                return "BUSY";
    case Err::IN_PROGRESS:         return "IN_PROGRESS";
    default:                       return "UNKNOWN";
  }
}

const char* stateToStr(BME280::DriverState st) {
  using namespace BME280;
  switch (st) {
    case DriverState::UNINIT:   return "UNINIT";
    case DriverState::READY:    return "READY";
    case DriverState::DEGRADED: return "DEGRADED";
    case DriverState::OFFLINE:  return "OFFLINE";
    default:                    return "UNKNOWN";
  }
}

const char* modeToStr(BME280::Mode mode) {
  using namespace BME280;
  switch (mode) {
    case Mode::SLEEP:  return "SLEEP";
    case Mode::FORCED: return "FORCED";
    case Mode::NORMAL: return "NORMAL";
    default:           return "UNKNOWN";
  }
}

void printStatus(const BME280::Status& st) {
  Serial.printf("  Status: %s (code=%u, detail=%ld)\n",
                errToStr(st.code),
                static_cast<unsigned>(st.code),
                static_cast<long>(st.detail));
  if (st.msg && st.msg[0]) {
    Serial.printf("  Message: %s\n", st.msg);
  }
}

void printDriverHealth() {
  Serial.println("=== Driver State ===");
  Serial.printf("  State: %s\n", stateToStr(device.state()));
  Serial.printf("  Consecutive failures: %u\n", device.consecutiveFailures());
  Serial.printf("  Total failures: %lu\n", static_cast<unsigned long>(device.totalFailures()));
  Serial.printf("  Total success: %lu\n", static_cast<unsigned long>(device.totalSuccess()));
  Serial.printf("  Last OK at: %lu ms\n", static_cast<unsigned long>(device.lastOkMs()));
  Serial.printf("  Last error at: %lu ms\n", static_cast<unsigned long>(device.lastErrorMs()));
  if (device.lastError().code != BME280::Err::OK) {
    Serial.printf("  Last error: %s\n", errToStr(device.lastError().code));
  }
}

void printMeasurement(const BME280::Measurement& m) {
  Serial.printf("Temp: %.2f C, Pressure: %.2f Pa, Humidity: %.2f %%\n",
                m.temperatureC, m.pressurePa, m.humidityPct);
}

void cancelPending() {
  pendingRead = false;
  stressRemaining = 0;
}

BME280::Status scheduleMeasurement() {
  BME280::Status st = device.requestMeasurement();
  if (st.code == BME280::Err::IN_PROGRESS) {
    pendingRead = true;
    pendingStartMs = millis();
    if (verboseMode) {
      Serial.printf("Measurement requested at %lu ms\n",
                    static_cast<unsigned long>(pendingStartMs));
    }
  }
  return st;
}

void handleMeasurementReady() {
  if (!pendingRead || !device.measurementReady()) {
    return;
  }

  BME280::Measurement m;
  BME280::Status st = device.getMeasurement(m);
  if (!st.ok()) {
    printStatus(st);
    pendingRead = false;
    return;
  }

  printMeasurement(m);
  pendingRead = false;

  if (stressRemaining > 0) {
    stressRemaining--;
    if (stressRemaining == 0) {
      LOGI("Stress test complete");
    }
  }
}

bool parseOversampling(const String& token, BME280::Oversampling& out) {
  const int value = token.toInt();
  if (value < 0 || value > 5) {
    return false;
  }
  out = static_cast<BME280::Oversampling>(value);
  return true;
}

bool parseFilter(const String& token, BME280::Filter& out) {
  const int value = token.toInt();
  if (value < 0 || value > 4) {
    return false;
  }
  out = static_cast<BME280::Filter>(value);
  return true;
}

bool parseStandby(const String& token, BME280::Standby& out) {
  const int value = token.toInt();
  if (value < 0 || value > 7) {
    return false;
  }
  out = static_cast<BME280::Standby>(value);
  return true;
}

void printHelp() {
  Serial.println("=== Commands ===");
  Serial.println("  help                    - Show this help");
  Serial.println("  scan                    - Scan I2C bus");
  Serial.println("  read                    - Request and display measurement");
  Serial.println("  mode sleep|forced|normal - Set operating mode");
  Serial.println("  osrs t|p|h <0..5>        - Set oversampling (0=skip, 1=x1, .., 5=x16)");
  Serial.println("  filter <0..4>            - Set IIR filter");
  Serial.println("  standby <0..7>           - Set standby time");
  Serial.println("  status                  - Read status register");
  Serial.println("  chipid                  - Read chip ID");
  Serial.println("  reset                   - Soft reset device");
  Serial.println("  drv                     - Show driver state and health");
  Serial.println("  probe                   - Probe device (no health tracking)");
  Serial.println("  recover                 - Manual recovery attempt");
  Serial.println("  verbose 0|1             - Enable/disable verbose output");
  Serial.println("  stress [N]              - Run N measurement cycles");
}

// ============================================================================
// Command Processing
// ============================================================================

void processCommand(const String& cmdLine) {
  String cmd = cmdLine;
  cmd.trim();
  if (cmd.length() == 0) {
    return;
  }

  if (cmd == "help" || cmd == "?") {
    printHelp();
    return;
  }

  if (cmd == "scan") {
    i2c::scan();
    return;
  }

  if (cmd == "read") {
    cancelPending();
    const BME280::Status st = scheduleMeasurement();
    if (st.code != BME280::Err::IN_PROGRESS) {
      printStatus(st);
    }
    return;
  }

  if (cmd.startsWith("mode ")) {
    String arg = cmd.substring(5);
    arg.trim();

    BME280::Mode mode;
    if (arg == "sleep") {
      mode = BME280::Mode::SLEEP;
    } else if (arg == "forced") {
      mode = BME280::Mode::FORCED;
    } else if (arg == "normal") {
      mode = BME280::Mode::NORMAL;
    } else {
      LOGW("Invalid mode: %s", arg.c_str());
      return;
    }

    cancelPending();
    BME280::Status st = device.setMode(mode);
    printStatus(st);
    return;
  }

  if (cmd.startsWith("osrs ")) {
    String args = cmd.substring(5);
    args.trim();

    const int split = args.indexOf(' ');
    if (split < 0) {
      LOGW("Usage: osrs t|p|h <0..5>");
      return;
    }

    const String which = args.substring(0, split);
    String value = args.substring(split + 1);
    value.trim();

    BME280::Oversampling osrs;
    if (!parseOversampling(value, osrs)) {
      LOGW("Invalid oversampling value");
      return;
    }

    BME280::Status st;
    if (which == "t") {
      st = device.setOversamplingT(osrs);
    } else if (which == "p") {
      st = device.setOversamplingP(osrs);
    } else if (which == "h") {
      st = device.setOversamplingH(osrs);
    } else {
      LOGW("Invalid osrs target: %s", which.c_str());
      return;
    }

    printStatus(st);
    return;
  }

  if (cmd.startsWith("filter ")) {
    String value = cmd.substring(7);
    value.trim();

    BME280::Filter filter;
    if (!parseFilter(value, filter)) {
      LOGW("Invalid filter value");
      return;
    }

    BME280::Status st = device.setFilter(filter);
    printStatus(st);
    return;
  }

  if (cmd.startsWith("standby ")) {
    String value = cmd.substring(8);
    value.trim();

    BME280::Standby standby;
    if (!parseStandby(value, standby)) {
      LOGW("Invalid standby value");
      return;
    }

    BME280::Status st = device.setStandby(standby);
    printStatus(st);
    return;
  }

  if (cmd == "status") {
    uint8_t status = 0;
    BME280::Status st = device.readStatus(status);
    if (!st.ok()) {
      printStatus(st);
      return;
    }

    const bool measuring = (status & BME280::cmd::MASK_STATUS_MEASURING) != 0;
    const bool imUpdate = (status & BME280::cmd::MASK_STATUS_IM_UPDATE) != 0;
    Serial.printf("Status: 0x%02X (measuring=%d, im_update=%d)\n",
                  status, measuring ? 1 : 0, imUpdate ? 1 : 0);
    return;
  }

  if (cmd == "chipid") {
    uint8_t id = 0;
    BME280::Status st = device.readChipId(id);
    if (!st.ok()) {
      printStatus(st);
      return;
    }
    Serial.printf("Chip ID: 0x%02X\n", id);
    return;
  }

  if (cmd == "reset") {
    cancelPending();
    BME280::Status st = device.softReset();
    printStatus(st);
    return;
  }

  if (cmd == "drv") {
    printDriverHealth();
    BME280::Mode mode;
    if (device.getMode(mode).ok()) {
      Serial.printf("  Mode: %s\n", modeToStr(mode));
    }
    return;
  }

  if (cmd == "probe") {
    LOGI("Probing device (no health tracking)...");
    BME280::Status st = device.probe();
    printStatus(st);
    return;
  }

  if (cmd == "recover") {
    LOGI("Attempting recovery...");
    BME280::Status st = device.recover();
    printStatus(st);
    printDriverHealth();
    return;
  }

  if (cmd.startsWith("verbose ")) {
    const int val = cmd.substring(8).toInt();
    verboseMode = (val != 0);
    LOGI("Verbose mode: %s", verboseMode ? "ON" : "OFF");
    return;
  }

  if (cmd.startsWith("stress")) {
    int count = 10;
    if (cmd.length() > 6) {
      count = cmd.substring(6).toInt();
    }
    if (count <= 0) {
      LOGW("Invalid stress count");
      return;
    }

    cancelPending();
    stressRemaining = count;
    LOGI("Starting stress test: %d cycles", stressRemaining);
    return;
  }

  LOGW("Unknown command: %s", cmd.c_str());
}

// ============================================================================
// Setup and Loop
// ============================================================================

void setup() {
  log_begin(115200);

  LOGI("=== BME280 Bringup Example ===");

  if (!board::initI2c()) {
    LOGE("Failed to initialize I2C");
    return;
  }
  LOGI("I2C initialized (SDA=%d, SCL=%d)", board::I2C_SDA, board::I2C_SCL);

  i2c::scan();

  BME280::Config cfg;
  cfg.i2cWrite = transport::wireWrite;
  cfg.i2cWriteRead = transport::wireWriteRead;
  cfg.i2cAddress = 0x76;
  cfg.i2cTimeoutMs = board::I2C_TIMEOUT_MS;
  cfg.offlineThreshold = 5;

  BME280::Status st = device.begin(cfg);
  if (!st.ok()) {
    LOGE("Failed to initialize device");
    printStatus(st);
    return;
  }

  LOGI("Device initialized successfully");
  printDriverHealth();
  printHelp();
  Serial.print("> ");
}

void loop() {
  device.tick(millis());

  if (stressRemaining > 0 && !pendingRead) {
    const BME280::Status st = scheduleMeasurement();
    if (st.code != BME280::Err::IN_PROGRESS && st.code != BME280::Err::BUSY) {
      printStatus(st);
      stressRemaining = 0;
    }
  }

  handleMeasurementReady();

  static String inputBuffer;
  while (Serial.available()) {
    const char c = static_cast<char>(Serial.read());
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
