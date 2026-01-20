/// @file main.cpp
/// @brief Basic bringup example for BME280
/// @note This is an EXAMPLE, not part of the library

#include <Arduino.h>
#include <limits>
#include "common/Log.h"
#include "common/BoardConfig.h"
#include "common/I2cTransport.h"
#include "common/I2cScanner.h"

#include "BME280/BME280.h"

// ============================================================================
// Globals
// ============================================================================

struct StressStats {
  bool active = false;
  uint32_t startMs = 0;
  uint32_t endMs = 0;
  int target = 0;
  int attempts = 0;
  int success = 0;
  uint32_t errors = 0;
  bool hasSample = false;
  float minTemp = 0.0f;
  float maxTemp = 0.0f;
  float minPressure = 0.0f;
  float maxPressure = 0.0f;
  float minHumidity = 0.0f;
  float maxHumidity = 0.0f;
  double sumTemp = 0.0;
  double sumPressure = 0.0;
  double sumHumidity = 0.0;
  BME280::Status lastError = BME280::Status::Ok();
};

struct ChipSettings {
  uint8_t ctrlHum = 0;
  uint8_t ctrlMeas = 0;
  uint8_t config = 0;
  uint8_t osrsH = 0;
  uint8_t osrsT = 0;
  uint8_t osrsP = 0;
  uint8_t modeBits = 0;
  uint8_t filter = 0;
  uint8_t standby = 0;
  bool spi3wEnabled = false;
};

struct InternalSettings {
  BME280::Oversampling osrsT = BME280::Oversampling::SKIP;
  BME280::Oversampling osrsP = BME280::Oversampling::SKIP;
  BME280::Oversampling osrsH = BME280::Oversampling::SKIP;
  BME280::Filter filter = BME280::Filter::OFF;
  BME280::Standby standby = BME280::Standby::MS_0_5;
  BME280::Mode mode = BME280::Mode::SLEEP;
};

BME280::BME280 device;
bool verboseMode = false;
bool pendingRead = false;
uint32_t pendingStartMs = 0;
int stressRemaining = 0;
StressStats stressStats;

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

const char* modeBitsToStr(uint8_t modeBits) {
  switch (modeBits) {
    case 0: return "SLEEP";
    case 1: return "FORCED";
    case 2: return "FORCED";
    case 3: return "NORMAL";
    default: return "UNKNOWN";
  }
}

const char* osrsToStr(uint8_t value) {
  switch (value) {
    case 0: return "SKIP";
    case 1: return "X1";
    case 2: return "X2";
    case 3: return "X4";
    case 4: return "X8";
    case 5: return "X16";
    default: return "UNKNOWN";
  }
}

const char* filterToStr(uint8_t value) {
  switch (value) {
    case 0: return "OFF";
    case 1: return "X2";
    case 2: return "X4";
    case 3: return "X8";
    case 4: return "X16";
    default: return "UNKNOWN";
  }
}

const char* standbyToStr(uint8_t value) {
  switch (value) {
    case 0: return "0.5ms";
    case 1: return "62.5ms";
    case 2: return "125ms";
    case 3: return "250ms";
    case 4: return "500ms";
    case 5: return "1000ms";
    case 6: return "10ms";
    case 7: return "20ms";
    default: return "UNKNOWN";
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

void printCalibration() {
  BME280::Calibration calib;
  BME280::Status st = device.getCalibration(calib);
  if (!st.ok()) {
    printStatus(st);
    return;
  }

  Serial.println("=== Calibration (Cached) ===");
  Serial.printf("  T1=%u T2=%d T3=%d\n",
                static_cast<unsigned>(calib.digT1),
                static_cast<int>(calib.digT2),
                static_cast<int>(calib.digT3));
  Serial.printf("  P1=%u P2=%d P3=%d P4=%d P5=%d P6=%d P7=%d P8=%d P9=%d\n",
                static_cast<unsigned>(calib.digP1),
                static_cast<int>(calib.digP2),
                static_cast<int>(calib.digP3),
                static_cast<int>(calib.digP4),
                static_cast<int>(calib.digP5),
                static_cast<int>(calib.digP6),
                static_cast<int>(calib.digP7),
                static_cast<int>(calib.digP8),
                static_cast<int>(calib.digP9));
  Serial.printf("  H1=%u H2=%d H3=%u H4=%d H5=%d H6=%d\n",
                static_cast<unsigned>(calib.digH1),
                static_cast<int>(calib.digH2),
                static_cast<unsigned>(calib.digH3),
                static_cast<int>(calib.digH4),
                static_cast<int>(calib.digH5),
                static_cast<int>(calib.digH6));
}

void printCalibrationRaw() {
  BME280::CalibrationRaw raw;
  BME280::Status st = device.readCalibrationRaw(raw);
  if (!st.ok()) {
    printStatus(st);
    return;
  }

  Serial.println("=== Calibration (Raw Registers) ===");
  Serial.print("  TP: ");
  for (size_t i = 0; i < sizeof(raw.tp); ++i) {
    Serial.printf("%02X", raw.tp[i]);
    if (i + 1 < sizeof(raw.tp)) {
      Serial.print(' ');
    }
  }
  Serial.println();
  Serial.printf("  H1: %02X\n", raw.h1);
  Serial.print("  H: ");
  for (size_t i = 0; i < sizeof(raw.h); ++i) {
    Serial.printf("%02X", raw.h[i]);
    if (i + 1 < sizeof(raw.h)) {
      Serial.print(' ');
    }
  }
  Serial.println();
}

void printVerboseState() {
  Serial.printf("  Verbose: %s\n", verboseMode ? "ON" : "OFF");
}

bool readChipSettings(ChipSettings& out) {
  BME280::Status st = device.readCtrlHum(out.ctrlHum);
  if (!st.ok()) {
    printStatus(st);
    return false;
  }
  st = device.readCtrlMeas(out.ctrlMeas);
  if (!st.ok()) {
    printStatus(st);
    return false;
  }
  st = device.readConfig(out.config);
  if (!st.ok()) {
    printStatus(st);
    return false;
  }

  out.osrsH = (out.ctrlHum & BME280::cmd::MASK_CTRL_HUM_OSRS_H) >>
              BME280::cmd::BIT_CTRL_HUM_OSRS_H;
  out.osrsT = (out.ctrlMeas & BME280::cmd::MASK_CTRL_MEAS_OSRS_T) >>
              BME280::cmd::BIT_CTRL_MEAS_OSRS_T;
  out.osrsP = (out.ctrlMeas & BME280::cmd::MASK_CTRL_MEAS_OSRS_P) >>
              BME280::cmd::BIT_CTRL_MEAS_OSRS_P;
  out.modeBits = (out.ctrlMeas & BME280::cmd::MASK_CTRL_MEAS_MODE) >>
                 BME280::cmd::BIT_CTRL_MEAS_MODE;
  out.filter = (out.config & BME280::cmd::MASK_CONFIG_FILTER) >>
               BME280::cmd::BIT_CONFIG_FILTER;
  out.standby = (out.config & BME280::cmd::MASK_CONFIG_T_SB) >>
                BME280::cmd::BIT_CONFIG_T_SB;
  out.spi3wEnabled = (out.config & BME280::cmd::MASK_CONFIG_SPI3W_EN) != 0;
  return true;
}

bool readInternalSettings(InternalSettings& out) {
  BME280::Status st = device.getMode(out.mode);
  if (!st.ok()) {
    printStatus(st);
    return false;
  }
  st = device.getOversamplingT(out.osrsT);
  if (!st.ok()) {
    printStatus(st);
    return false;
  }
  st = device.getOversamplingP(out.osrsP);
  if (!st.ok()) {
    printStatus(st);
    return false;
  }
  st = device.getOversamplingH(out.osrsH);
  if (!st.ok()) {
    printStatus(st);
    return false;
  }
  st = device.getFilter(out.filter);
  if (!st.ok()) {
    printStatus(st);
    return false;
  }
  st = device.getStandby(out.standby);
  if (!st.ok()) {
    printStatus(st);
    return false;
  }
  return true;
}

void printChipSettings(const ChipSettings& chip) {
  Serial.println("=== Chip Settings ===");
  Serial.printf("  ctrl_hum: 0x%02X (osrs_h=%u %s)\n",
                chip.ctrlHum,
                static_cast<unsigned>(chip.osrsH),
                osrsToStr(chip.osrsH));
  Serial.printf("  ctrl_meas: 0x%02X (osrs_t=%u %s, osrs_p=%u %s, mode=%u %s)\n",
                chip.ctrlMeas,
                static_cast<unsigned>(chip.osrsT),
                osrsToStr(chip.osrsT),
                static_cast<unsigned>(chip.osrsP),
                osrsToStr(chip.osrsP),
                static_cast<unsigned>(chip.modeBits),
                modeBitsToStr(chip.modeBits));
  Serial.printf("  config: 0x%02X (standby=%u %s, filter=%u %s, spi3w_en=%u)\n",
                chip.config,
                static_cast<unsigned>(chip.standby),
                standbyToStr(chip.standby),
                static_cast<unsigned>(chip.filter),
                filterToStr(chip.filter),
                chip.spi3wEnabled ? 1u : 0u);
}

void printInternalSettings(const InternalSettings& internal) {
  Serial.println("=== Internal Settings ===");
  Serial.printf("  Mode: %s\n", modeToStr(internal.mode));
  Serial.printf("  osrs_t: %s (%u)\n",
                osrsToStr(static_cast<uint8_t>(internal.osrsT)),
                static_cast<unsigned>(internal.osrsT));
  Serial.printf("  osrs_p: %s (%u)\n",
                osrsToStr(static_cast<uint8_t>(internal.osrsP)),
                static_cast<unsigned>(internal.osrsP));
  Serial.printf("  osrs_h: %s (%u)\n",
                osrsToStr(static_cast<uint8_t>(internal.osrsH)),
                static_cast<unsigned>(internal.osrsH));
  Serial.printf("  Filter: %s (%u)\n",
                filterToStr(static_cast<uint8_t>(internal.filter)),
                static_cast<unsigned>(internal.filter));
  Serial.printf("  Standby: %s (%u)\n",
                standbyToStr(static_cast<uint8_t>(internal.standby)),
                static_cast<unsigned>(internal.standby));
  printVerboseState();
}

void printAllSettings() {
  ChipSettings chip;
  if (readChipSettings(chip)) {
    printChipSettings(chip);
  }

  InternalSettings internal;
  if (readInternalSettings(internal)) {
    printInternalSettings(internal);
  } else {
    printVerboseState();
  }
}

void printModeSettings() {
  ChipSettings chip;
  if (readChipSettings(chip)) {
    Serial.printf("Chip mode: %s (%u)\n",
                  modeBitsToStr(chip.modeBits),
                  static_cast<unsigned>(chip.modeBits));
  }

  InternalSettings internal;
  if (readInternalSettings(internal)) {
    Serial.printf("Internal mode: %s\n", modeToStr(internal.mode));
  }
  printVerboseState();
}

void printOsrsSettings() {
  ChipSettings chip;
  if (readChipSettings(chip)) {
    Serial.printf("Chip osrs: T=%s (%u), P=%s (%u), H=%s (%u)\n",
                  osrsToStr(chip.osrsT), static_cast<unsigned>(chip.osrsT),
                  osrsToStr(chip.osrsP), static_cast<unsigned>(chip.osrsP),
                  osrsToStr(chip.osrsH), static_cast<unsigned>(chip.osrsH));
  }

  InternalSettings internal;
  if (readInternalSettings(internal)) {
    Serial.printf("Internal osrs: T=%s (%u), P=%s (%u), H=%s (%u)\n",
                  osrsToStr(static_cast<uint8_t>(internal.osrsT)),
                  static_cast<unsigned>(internal.osrsT),
                  osrsToStr(static_cast<uint8_t>(internal.osrsP)),
                  static_cast<unsigned>(internal.osrsP),
                  osrsToStr(static_cast<uint8_t>(internal.osrsH)),
                  static_cast<unsigned>(internal.osrsH));
  }
  printVerboseState();
}

void printFilterSettings() {
  ChipSettings chip;
  if (readChipSettings(chip)) {
    Serial.printf("Chip filter: %s (%u)\n",
                  filterToStr(chip.filter),
                  static_cast<unsigned>(chip.filter));
  }

  InternalSettings internal;
  if (readInternalSettings(internal)) {
    Serial.printf("Internal filter: %s (%u)\n",
                  filterToStr(static_cast<uint8_t>(internal.filter)),
                  static_cast<unsigned>(internal.filter));
  }
  printVerboseState();
}

void printStandbySettings() {
  ChipSettings chip;
  if (readChipSettings(chip)) {
    Serial.printf("Chip standby: %s (%u)\n",
                  standbyToStr(chip.standby),
                  static_cast<unsigned>(chip.standby));
  }

  InternalSettings internal;
  if (readInternalSettings(internal)) {
    Serial.printf("Internal standby: %s (%u)\n",
                  standbyToStr(static_cast<uint8_t>(internal.standby)),
                  static_cast<unsigned>(internal.standby));
  }
  printVerboseState();
}

void resetStressStats(int target) {
  stressStats.active = true;
  stressStats.startMs = millis();
  stressStats.endMs = 0;
  stressStats.target = target;
  stressStats.attempts = 0;
  stressStats.success = 0;
  stressStats.errors = 0;
  stressStats.hasSample = false;
  stressStats.minTemp = std::numeric_limits<float>::max();
  stressStats.maxTemp = std::numeric_limits<float>::lowest();
  stressStats.minPressure = std::numeric_limits<float>::max();
  stressStats.maxPressure = std::numeric_limits<float>::lowest();
  stressStats.minHumidity = std::numeric_limits<float>::max();
  stressStats.maxHumidity = std::numeric_limits<float>::lowest();
  stressStats.sumTemp = 0.0;
  stressStats.sumPressure = 0.0;
  stressStats.sumHumidity = 0.0;
  stressStats.lastError = BME280::Status::Ok();
}

void noteStressError(const BME280::Status& st) {
  stressStats.errors++;
  stressStats.lastError = st;
}

void updateStressStats(const BME280::Measurement& m) {
  if (!stressStats.hasSample) {
    stressStats.minTemp = m.temperatureC;
    stressStats.maxTemp = m.temperatureC;
    stressStats.minPressure = m.pressurePa;
    stressStats.maxPressure = m.pressurePa;
    stressStats.minHumidity = m.humidityPct;
    stressStats.maxHumidity = m.humidityPct;
    stressStats.hasSample = true;
  } else {
    if (m.temperatureC < stressStats.minTemp) {
      stressStats.minTemp = m.temperatureC;
    }
    if (m.temperatureC > stressStats.maxTemp) {
      stressStats.maxTemp = m.temperatureC;
    }
    if (m.pressurePa < stressStats.minPressure) {
      stressStats.minPressure = m.pressurePa;
    }
    if (m.pressurePa > stressStats.maxPressure) {
      stressStats.maxPressure = m.pressurePa;
    }
    if (m.humidityPct < stressStats.minHumidity) {
      stressStats.minHumidity = m.humidityPct;
    }
    if (m.humidityPct > stressStats.maxHumidity) {
      stressStats.maxHumidity = m.humidityPct;
    }
  }

  stressStats.sumTemp += m.temperatureC;
  stressStats.sumPressure += m.pressurePa;
  stressStats.sumHumidity += m.humidityPct;
  stressStats.success++;
}

void finishStressStats() {
  stressStats.active = false;
  stressStats.endMs = millis();
  const uint32_t durationMs = stressStats.endMs - stressStats.startMs;

  Serial.println("=== Stress Summary ===");
  Serial.printf("  Target: %d\n", stressStats.target);
  Serial.printf("  Attempts: %d\n", stressStats.attempts);
  Serial.printf("  Success: %d\n", stressStats.success);
  Serial.printf("  Errors: %lu\n", static_cast<unsigned long>(stressStats.errors));
  Serial.printf("  Duration: %lu ms\n", static_cast<unsigned long>(durationMs));
  if (durationMs > 0) {
    const float rate = 1000.0f * static_cast<float>(stressStats.attempts) /
                       static_cast<float>(durationMs);
    Serial.printf("  Rate: %.2f samples/s\n", rate);
  }

  if (stressStats.success > 0) {
    const float avgTemp = static_cast<float>(stressStats.sumTemp / stressStats.success);
    const float avgPressure = static_cast<float>(stressStats.sumPressure / stressStats.success);
    const float avgHumidity = static_cast<float>(stressStats.sumHumidity / stressStats.success);
    Serial.printf("  Temp C: min=%.2f avg=%.2f max=%.2f\n",
                  stressStats.minTemp, avgTemp, stressStats.maxTemp);
    Serial.printf("  Pressure Pa: min=%.2f avg=%.2f max=%.2f\n",
                  stressStats.minPressure, avgPressure, stressStats.maxPressure);
    Serial.printf("  Humidity %%: min=%.2f avg=%.2f max=%.2f\n",
                  stressStats.minHumidity, avgHumidity, stressStats.maxHumidity);
  } else {
    Serial.println("  No valid samples");
  }

  if (!stressStats.lastError.ok()) {
    Serial.printf("  Last error: %s\n", errToStr(stressStats.lastError.code));
    if (stressStats.lastError.msg && stressStats.lastError.msg[0]) {
      Serial.printf("  Message: %s\n", stressStats.lastError.msg);
    }
  }
}

void cancelPending() {
  pendingRead = false;
  stressRemaining = 0;
  stressStats.active = false;
}

BME280::Status scheduleMeasurement() {
  BME280::Status st = device.requestMeasurement();
  if (st.code == BME280::Err::IN_PROGRESS) {
    pendingRead = true;
    pendingStartMs = millis();
    if (verboseMode && !stressStats.active) {
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
  pendingRead = false;

  if (!st.ok()) {
    if (stressStats.active) {
      noteStressError(st);
      stressStats.attempts++;
      if (stressRemaining > 0) {
        stressRemaining--;
      }
      if (stressRemaining == 0 && stressStats.active) {
        finishStressStats();
      }
    } else {
      printStatus(st);
    }
    return;
  }

  if (stressStats.active) {
    updateStressStats(m);
    stressStats.attempts++;
    if (stressRemaining > 0) {
      stressRemaining--;
    }
    if (stressRemaining == 0 && stressStats.active) {
      finishStressStats();
    }
    return;
  }

  printMeasurement(m);
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
  Serial.println("  mode [sleep|forced|normal] - Set or show operating mode");
  Serial.println("  osrs [t|p|h <0..5>]        - Set or show oversampling");
  Serial.println("  filter [0..4]              - Set or show IIR filter");
  Serial.println("  standby [0..7]             - Set or show standby time");
  Serial.println("  settings                  - Show chip + internal settings");
  Serial.println("  calib [raw]              - Show cached or raw calibration coefficients");
  Serial.println("  status                  - Read status register");
  Serial.println("  chipid                  - Read chip ID");
  Serial.println("  reset                   - Soft reset device");
  Serial.println("  drv                     - Show driver state and health");
  Serial.println("  probe                   - Probe device (no health tracking)");
  Serial.println("  recover                 - Manual recovery attempt");
  Serial.println("  verbose [0|1]            - Enable/disable verbose output");
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

  if (cmd == "settings" || cmd == "cfg") {
    printAllSettings();
    return;
  }

  if (cmd == "calib raw") {
    printCalibrationRaw();
    return;
  }

  if (cmd == "calib") {
    printCalibration();
    return;
  }

  if (cmd == "mode") {
    printModeSettings();
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

  if (cmd == "osrs") {
    printOsrsSettings();
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

  if (cmd == "filter") {
    printFilterSettings();
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

  if (cmd == "standby") {
    printStandbySettings();
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

  if (cmd == "verbose") {
    printVerboseState();
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
    resetStressStats(count);
    LOGI("Starting stress test: %d cycles", count);
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

  if (stressStats.active && stressRemaining > 0 && !pendingRead) {
    const BME280::Status st = scheduleMeasurement();
    if (st.code != BME280::Err::IN_PROGRESS && st.code != BME280::Err::BUSY) {
      noteStressError(st);
      stressStats.attempts++;
      stressRemaining--;
      if (stressRemaining == 0) {
        finishStressStats();
      }
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
