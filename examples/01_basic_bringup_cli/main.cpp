/// @file main.cpp
/// @brief Basic bringup example for BME280
/// @note This is an EXAMPLE, not part of the library

#include <Arduino.h>
#include <limits>
#include "examples/common/CliStyle.h"
#include "examples/common/Log.h"
#include "examples/common/BoardConfig.h"
#include "examples/common/BusDiag.h"
#include "examples/common/HealthView.h"
#include "examples/common/I2cTransport.h"
#include "examples/common/I2cScanner.h"

#include "BME280/BME280.h"

// ============================================================================
// Globals
// ============================================================================

struct StressStats {
  bool active = false;
  uint32_t startMs = 0;
  uint32_t endMs = 0;
  uint32_t successBefore = 0;
  uint32_t failBefore = 0;
  int target = 0;
  int attempts = 0;
  int success = 0;
  uint32_t errors = 0;
  bool hasFailure = false;
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
  BME280::Status firstError = BME280::Status::Ok();
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
uint8_t activeAddress = 0x76;
static constexpr uint32_t STRESS_PROGRESS_UPDATES = 10U;

void cancelPending();

// ============================================================================
// Helper Functions
// ============================================================================

uint32_t exampleNowMs(void*) {
  return millis();
}

BME280::Config makeDefaultConfig() {
  BME280::Config cfg;
  cfg.i2cWrite = transport::wireWrite;
  cfg.i2cWriteRead = transport::wireWriteRead;
  cfg.i2cUser = transport::configUser();
  cfg.i2cAddress = activeAddress;
  cfg.i2cTimeoutMs = board::I2C_TIMEOUT_MS;
  cfg.nowMs = exampleNowMs;
  cfg.offlineThreshold = 5;
  return cfg;
}

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
    case Err::I2C_NACK_ADDR:       return "I2C_NACK_ADDR";
    case Err::I2C_NACK_DATA:       return "I2C_NACK_DATA";
    case Err::I2C_TIMEOUT:         return "I2C_TIMEOUT";
    case Err::I2C_BUS:             return "I2C_BUS";
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

const char* stateColor(BME280::DriverState st, bool online, uint8_t consecutiveFailures) {
  if (st == BME280::DriverState::UNINIT) {
    return LOG_COLOR_RESET;
  }
  return LOG_COLOR_STATE(online, consecutiveFailures);
}

const char* goodIfZeroColor(uint32_t value) {
  return (value == 0U) ? LOG_COLOR_GREEN : LOG_COLOR_RED;
}

const char* goodIfNonZeroColor(uint32_t value) {
  return (value > 0U) ? LOG_COLOR_GREEN : LOG_COLOR_YELLOW;
}

const char* onOffColor(bool enabled) {
  return enabled ? LOG_COLOR_GREEN : LOG_COLOR_RESET;
}

const char* skipCountColor(uint32_t value) {
  return (value > 0U) ? LOG_COLOR_YELLOW : LOG_COLOR_RESET;
}

const char* successRateColor(float pct) {
  if (pct >= 99.9f) return LOG_COLOR_GREEN;
  if (pct >= 80.0f) return LOG_COLOR_YELLOW;
  return LOG_COLOR_RED;
}

uint32_t stressProgressStep(uint32_t total) {
  if (total == 0U) {
    return 0U;
  }
  const uint32_t step = total / STRESS_PROGRESS_UPDATES;
  return (step == 0U) ? 1U : step;
}

void printStressProgress(uint32_t completed, uint32_t total, uint32_t okCount, uint32_t failCount) {
  if (completed == 0U || total == 0U) {
    return;
  }
  const uint32_t step = stressProgressStep(total);
  if (step == 0U || (completed != total && (completed % step) != 0U)) {
    return;
  }
  const float pct = (100.0f * static_cast<float>(completed)) / static_cast<float>(total);
  Serial.printf("  Progress: %lu/%lu (%.0f%%, ok=%s%lu%s, fail=%s%lu%s)\n",
                static_cast<unsigned long>(completed),
                static_cast<unsigned long>(total),
                pct,
                goodIfNonZeroColor(okCount),
                static_cast<unsigned long>(okCount),
                LOG_COLOR_RESET,
                goodIfZeroColor(failCount),
                static_cast<unsigned long>(failCount),
                LOG_COLOR_RESET);
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
  Serial.printf("  Status: %s%s%s (code=%u, detail=%ld)\n",
                LOG_COLOR_RESULT(st.ok()),
                errToStr(st.code),
                LOG_COLOR_RESET,
                static_cast<unsigned>(st.code),
                static_cast<long>(st.detail));
  if (st.msg && st.msg[0]) {
    Serial.printf("  Message: %s%s%s\n", LOG_COLOR_YELLOW, st.msg, LOG_COLOR_RESET);
  }
}

void printDirtyState() {
  const BME280::Status dirty = device.hardwareConfigDirtyError();
  Serial.printf("  Hardware config dirty: %s%s%s\n",
                device.hardwareConfigDirty() ? LOG_COLOR_YELLOW : LOG_COLOR_GREEN,
                device.hardwareConfigDirty() ? "true" : "false",
                LOG_COLOR_RESET);
  if (device.hardwareConfigDirty() && !dirty.ok()) {
    Serial.printf("  Dirty cause: %s (detail=%ld)\n",
                  errToStr(dirty.code),
                  static_cast<long>(dirty.detail));
  }
}

void printValidity(bool temperatureValid, bool pressureValid, bool humidityValid) {
  Serial.printf("  Valid channels: T=%d P=%d H=%d\n",
                temperatureValid ? 1 : 0,
                pressureValid ? 1 : 0,
                humidityValid ? 1 : 0);
}

void printSampleAge() {
  if (!device.hasSample()) {
    Serial.println("  Cached sample: none");
    return;
  }
  Serial.printf("  Cached sample age: %lu ms (timestamp=%lu ms)\n",
                static_cast<unsigned long>(device.sampleAgeMs(millis())),
                static_cast<unsigned long>(device.sampleTimestampMs()));
}

void printDriverHealth() {
  const uint32_t now = millis();
  const uint32_t totalOk = device.totalSuccess();
  const uint32_t totalFail = device.totalFailures();
  const uint32_t total = totalOk + totalFail;
  const float successRate = (total > 0U)
                                ? (100.0f * static_cast<float>(totalOk) / static_cast<float>(total))
                                : 0.0f;
  const BME280::Status lastErr = device.lastError();
  const BME280::DriverState st = device.state();
  const bool online = device.isOnline();

  Serial.println("=== Driver Health ===");
  Serial.printf("  State: %s%s%s\n",
                stateColor(st, online, device.consecutiveFailures()),
                stateToStr(st),
                LOG_COLOR_RESET);
  Serial.printf("  Online: %s%s%s\n",
                online ? LOG_COLOR_GREEN : LOG_COLOR_RED,
                log_bool_str(online),
                LOG_COLOR_RESET);
  Serial.printf("  Active I2C address: 0x%02X\n", activeAddress);
  printDirtyState();
  Serial.printf("  Consecutive failures: %s%u%s\n",
                goodIfZeroColor(device.consecutiveFailures()),
                device.consecutiveFailures(),
                LOG_COLOR_RESET);
  Serial.printf("  Total success: %s%lu%s\n",
                goodIfNonZeroColor(totalOk),
                static_cast<unsigned long>(totalOk),
                LOG_COLOR_RESET);
  Serial.printf("  Total failures: %s%lu%s\n",
                goodIfZeroColor(totalFail),
                static_cast<unsigned long>(totalFail),
                LOG_COLOR_RESET);
  Serial.printf("  Success rate: %s%.1f%%%s\n",
                successRateColor(successRate),
                successRate,
                LOG_COLOR_RESET);

  const uint32_t lastOkMs = device.lastOkMs();
  if (lastOkMs > 0U) {
    Serial.printf("  Last OK: %lu ms ago (at %lu ms)\n",
                  static_cast<unsigned long>(now - lastOkMs),
                  static_cast<unsigned long>(lastOkMs));
  } else {
    Serial.println("  Last OK: never");
  }

  const uint32_t lastErrorMs = device.lastErrorMs();
  if (lastErrorMs > 0U) {
    Serial.printf("  Last error: %lu ms ago (at %lu ms)\n",
                  static_cast<unsigned long>(now - lastErrorMs),
                  static_cast<unsigned long>(lastErrorMs));
  } else {
    Serial.println("  Last error: never");
  }

  if (!lastErr.ok()) {
    Serial.printf("  Error code: %s%s%s\n",
                  LOG_COLOR_RED,
                  errToStr(lastErr.code),
                  LOG_COLOR_RESET);
    Serial.printf("  Error detail: %ld\n", static_cast<long>(lastErr.detail));
    if (lastErr.msg && lastErr.msg[0]) {
      Serial.printf("  Error msg: %s\n", lastErr.msg);
    }
  }
}

void printMeasurement(const BME280::Measurement& m) {
  Serial.printf("Temp: %.2f C, Pressure: %.2f Pa, Humidity: %.2f %%\n",
                m.temperatureC, m.pressurePa, m.humidityPct);
  printValidity(m.temperatureValid, m.pressureValid, m.humidityValid);
}

void printRawSample() {
  BME280::RawSample raw;
  const BME280::Status st = device.getRawSample(raw);
  if (!st.ok()) {
    printStatus(st);
    return;
  }
  Serial.printf("Raw ADC: T=%ld P=%ld H=%ld\n",
                static_cast<long>(raw.adcT),
                static_cast<long>(raw.adcP),
                static_cast<long>(raw.adcH));
  printValidity(raw.temperatureValid, raw.pressureValid, raw.humidityValid);
  printSampleAge();
}

void printCompensatedSample() {
  BME280::CompensatedSample sample;
  const BME280::Status st = device.getCompensatedSample(sample);
  if (!st.ok()) {
    printStatus(st);
    return;
  }
  const float humidityPct =
      static_cast<float>(sample.humidityPct_x1024) / 1024.0f;
  Serial.printf("Compensated: T=%ld x0.01C, P=%lu Pa, RH=%.2f %%\n",
                static_cast<long>(sample.tempC_x100),
                static_cast<unsigned long>(sample.pressurePa),
                humidityPct);
  printValidity(sample.temperatureValid, sample.pressureValid, sample.humidityValid);
  printSampleAge();
}

void printDataRegisters() {
  uint8_t data[BME280::cmd::DATA_LEN] = {};
  const BME280::Status st =
      device.readRegisters(BME280::cmd::REG_DATA_START, data, sizeof(data));
  if (!st.ok()) {
    printStatus(st);
    return;
  }

  const int32_t adcP = (static_cast<int32_t>(data[0]) << 12) |
                       (static_cast<int32_t>(data[1]) << 4) |
                       (static_cast<int32_t>(data[2]) >> 4);
  const int32_t adcT = (static_cast<int32_t>(data[3]) << 12) |
                       (static_cast<int32_t>(data[4]) << 4) |
                       (static_cast<int32_t>(data[5]) >> 4);
  const int32_t adcH = (static_cast<int32_t>(data[6]) << 8) |
                       static_cast<int32_t>(data[7]);

  Serial.println("=== Live Data Registers ===");
  Serial.print("  0xF7..0xFE: ");
  for (size_t i = 0; i < sizeof(data); ++i) {
    Serial.printf("%02X", data[i]);
    if (i + 1 < sizeof(data)) {
      Serial.print(' ');
    }
  }
  Serial.println();
  Serial.printf("  Decoded raw ADC: P=%ld T=%ld H=%ld\n",
                static_cast<long>(adcP),
                static_cast<long>(adcT),
                static_cast<long>(adcH));
  Serial.printf("  Sentinel check: P_skip=%d T_skip=%d H_skip=%d\n",
                adcP == BME280::cmd::RAW_PRESSURE_SKIPPED ? 1 : 0,
                adcT == BME280::cmd::RAW_TEMPERATURE_SKIPPED ? 1 : 0,
                adcH == BME280::cmd::RAW_HUMIDITY_SKIPPED ? 1 : 0);
}

void printTimingInfo() {
  bool measuring = false;
  const BME280::Status st = device.isMeasuring(measuring);
  if (!st.ok()) {
    printStatus(st);
    return;
  }
  Serial.printf("Measuring: %s\n", measuring ? "YES" : "NO");
  Serial.printf("Estimated measurement time: %lu ms\n",
                static_cast<unsigned long>(device.estimateMeasurementTimeMs()));
  Serial.printf("Configured standby: %lu ms\n",
                static_cast<unsigned long>(device.getStandbyTimeMs()));
  Serial.printf("Estimated normal cycle: %lu ms\n",
                static_cast<unsigned long>(device.estimateNormalCycleMs()));
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
  const bool plausible = calib.digT1 != 0U && calib.digP1 != 0U &&
                         (calib.digH1 != 0U || calib.digH2 != 0 ||
                          calib.digH3 != 0U || calib.digH4 != 0 ||
                          calib.digH5 != 0 || calib.digH6 != 0);
  Serial.printf("  Plausibility: %s%s%s (T1/P1 nonzero, humidity coeffs not all zero)\n",
                plausible ? LOG_COLOR_GREEN : LOG_COLOR_RED,
                plausible ? "PASS" : "CHECK",
                LOG_COLOR_RESET);
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
  Serial.printf("  Verbose: %s%s%s\n",
                onOffColor(verboseMode),
                verboseMode ? "ON" : "OFF",
                LOG_COLOR_RESET);
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
  BME280::SettingsSnapshot snap;
  BME280::Status st = device.getSettings(snap);
  if (!st.ok()) {
    printStatus(st);
    return false;
  }
  out.mode = snap.mode;
  out.osrsT = snap.osrsT;
  out.osrsP = snap.osrsP;
  out.osrsH = snap.osrsH;
  out.filter = snap.filter;
  out.standby = snap.standby;
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
  printDirtyState();
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
  stressStats.successBefore = device.totalSuccess();
  stressStats.failBefore = device.totalFailures();
  stressStats.target = target;
  stressStats.attempts = 0;
  stressStats.success = 0;
  stressStats.errors = 0;
  stressStats.hasFailure = false;
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
  stressStats.firstError = BME280::Status::Ok();
  stressStats.lastError = BME280::Status::Ok();
}

void noteStressError(const BME280::Status& st) {
  stressStats.errors++;
  if (!stressStats.hasFailure) {
    stressStats.firstError = st;
    stressStats.hasFailure = true;
  }
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

bool driverMeasurementPending() {
  BME280::SettingsSnapshot snapshot;
  if (!device.getSettings(snapshot).ok()) {
    return false;
  }
  return snapshot.initialized && snapshot.measurementRequested && !snapshot.measurementReady;
}

BME280::Status ensureForcedMeasurementMode() {
  BME280::Mode mode = BME280::Mode::SLEEP;
  BME280::Status st = device.getMode(mode);
  if (!st.ok()) {
    return st;
  }
  if (mode == BME280::Mode::FORCED) {
    return BME280::Status::Ok();
  }
  return device.setMode(BME280::Mode::FORCED);
}

void finishStressStats() {
  stressStats.active = false;
  stressStats.endMs = millis();
  const uint32_t successDelta = device.totalSuccess() - stressStats.successBefore;
  const uint32_t failDelta = device.totalFailures() - stressStats.failBefore;
  const uint32_t durationMs = stressStats.endMs - stressStats.startMs;
  const float successPct =
      (stressStats.attempts > 0)
          ? (100.0f * static_cast<float>(stressStats.success) /
             static_cast<float>(stressStats.attempts))
          : 0.0f;

  Serial.println("=== Stress Summary ===");
  Serial.printf("  Target: %d\n", stressStats.target);
  Serial.printf("  Attempts: %d\n", stressStats.attempts);
  Serial.printf("  Success: %s%d%s\n",
                goodIfNonZeroColor(static_cast<uint32_t>(stressStats.success)),
                stressStats.success,
                LOG_COLOR_RESET);
  Serial.printf("  Errors: %s%lu%s\n",
                goodIfZeroColor(stressStats.errors),
                static_cast<unsigned long>(stressStats.errors),
                LOG_COLOR_RESET);
  Serial.printf("  Success rate: %s%.2f%%%s\n",
                successRateColor(successPct),
                successPct,
                LOG_COLOR_RESET);
  Serial.printf("  Duration: %lu ms\n", static_cast<unsigned long>(durationMs));
  if (durationMs > 0) {
    const float rate = 1000.0f * static_cast<float>(stressStats.attempts) /
                       static_cast<float>(durationMs);
    Serial.printf("  Rate: %.2f samples/s\n", rate);
  }
  Serial.printf("  Health delta: %ssuccess +%lu%s, %sfailures +%lu%s\n",
                goodIfNonZeroColor(successDelta),
                static_cast<unsigned long>(successDelta),
                LOG_COLOR_RESET,
                goodIfZeroColor(failDelta),
                static_cast<unsigned long>(failDelta),
                LOG_COLOR_RESET);

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

  if (stressStats.hasFailure) {
    Serial.println("  First failure:");
    printStatus(stressStats.firstError);
    if (stressStats.errors > 1U) {
      Serial.println("  Last failure:");
      printStatus(stressStats.lastError);
    }
  }
}

BME280::Status performMeasurementBlocking(BME280::Measurement& out, uint32_t timeoutMs = 500) {
  BME280::Status st = device.requestMeasurement();
  if (st.code == BME280::Err::BUSY && !driverMeasurementPending()) {
    return st;
  }
  if (st.code != BME280::Err::IN_PROGRESS && st.code != BME280::Err::BUSY) {
    return st;
  }

  const uint32_t start = millis();
  while ((millis() - start) < timeoutMs) {
    device.tick(millis());
    if (device.measurementReady()) {
      return device.getMeasurement(out);
    }
    yield();
  }
  return BME280::Status::Error(BME280::Err::TIMEOUT, "measurement timeout", timeoutMs);
}

struct StressMixSettings {
  bool valid = false;
  BME280::Mode mode = BME280::Mode::FORCED;
  BME280::Filter filter = BME280::Filter::OFF;
  BME280::Standby standby = BME280::Standby::MS_125;
};

StressMixSettings captureStressMixSettings() {
  StressMixSettings settings;
  settings.valid = device.getMode(settings.mode).ok() &&
                   device.getFilter(settings.filter).ok() &&
                   device.getStandby(settings.standby).ok();
  return settings;
}

BME280::Status restoreStressMixSettings(const StressMixSettings& settings) {
  if (!settings.valid) {
    return BME280::Status::Ok();
  }

  BME280::Status st = device.setMode(BME280::Mode::SLEEP);
  if (!st.ok()) {
    return st;
  }
  st = device.setFilter(settings.filter);
  if (!st.ok()) {
    return st;
  }
  st = device.setStandby(settings.standby);
  if (!st.ok()) {
    return st;
  }
  return device.setMode(settings.mode);
}

void runStressMix(int count) {
  struct OpStats {
    const char* name;
    uint32_t ok;
    uint32_t fail;
  };
  OpStats stats[] = {
      {"measure", 0, 0},
      {"readStatus", 0, 0},
      {"readChipId", 0, 0},
      {"readCalRaw", 0, 0},
      {"setMode", 0, 0},
      {"setFilter", 0, 0},
      {"setStandby", 0, 0},
  };
  const int opCount = static_cast<int>(sizeof(stats) / sizeof(stats[0]));

  cancelPending();
  const StressMixSettings originalSettings = captureStressMixSettings();
  HealthSnapshot<BME280::BME280> healthBefore;
  healthBefore.capture(device);
  const uint32_t succBefore = device.totalSuccess();
  const uint32_t failBefore = device.totalFailures();
  const uint32_t startMs = millis();
  uint32_t okTotal = 0;
  uint32_t failTotal = 0;
  bool hasFailure = false;
  BME280::Status firstFailure = BME280::Status::Ok();
  BME280::Status lastFailure = BME280::Status::Ok();

  for (int i = 0; i < count; ++i) {
    const int op = i % opCount;
    BME280::Status st = BME280::Status::Ok();

    switch (op) {
      case 0: {
        BME280::Measurement m;
        st = ensureForcedMeasurementMode();
        if (st.ok()) {
          st = performMeasurementBlocking(m);
        }
        break;
      }
      case 1: {
        uint8_t status = 0;
        st = device.readStatus(status);
        break;
      }
      case 2: {
        uint8_t id = 0;
        st = device.readChipId(id);
        if (st.ok() && id != BME280::cmd::CHIP_ID_BME280) {
          st = BME280::Status::Error(BME280::Err::CHIP_ID_MISMATCH, "unexpected chip id", id);
        }
        break;
      }
      case 3: {
        BME280::CalibrationRaw raw;
        st = device.readCalibrationRaw(raw);
        break;
      }
      case 4: {
        const BME280::Mode mode =
            ((i / opCount) % 2 == 0) ? BME280::Mode::FORCED : BME280::Mode::SLEEP;
        st = device.setMode(mode);
        break;
      }
      case 5: {
        st = device.setFilter(static_cast<BME280::Filter>((i / opCount) % 5));
        break;
      }
      case 6: {
        st = device.setStandby(static_cast<BME280::Standby>((i / opCount) % 8));
        break;
      }
      default:
        break;
    }

    if (st.ok()) {
      stats[op].ok++;
      okTotal++;
    } else {
      stats[op].fail++;
      failTotal++;
      if (!hasFailure) {
        firstFailure = st;
        hasFailure = true;
      }
      lastFailure = st;
      if (verboseMode) {
        Serial.printf("  [%d] %s failed: %s\n", i, stats[op].name, errToStr(st.code));
      }
    }

    printStressProgress(static_cast<uint32_t>(i + 1),
                        static_cast<uint32_t>(count),
                        okTotal,
                        failTotal);
  }

  const uint32_t elapsed = millis() - startMs;
  HealthSnapshot<BME280::BME280> healthAfter;
  healthAfter.capture(device);

  Serial.println("=== stress_mix summary ===");
  const float successPct =
      (count > 0) ? (100.0f * static_cast<float>(okTotal) / static_cast<float>(count)) : 0.0f;
  Serial.printf("  Total: %sok=%lu%s %sfail=%lu%s (%s%.2f%%%s)\n",
                goodIfNonZeroColor(okTotal),
                static_cast<unsigned long>(okTotal),
                LOG_COLOR_RESET,
                goodIfZeroColor(failTotal),
                static_cast<unsigned long>(failTotal),
                LOG_COLOR_RESET,
                successRateColor(successPct),
                successPct,
                LOG_COLOR_RESET);
  Serial.printf("  Duration: %lu ms\n", static_cast<unsigned long>(elapsed));
  if (elapsed > 0) {
    Serial.printf("  Rate: %.2f ops/s\n", (1000.0f * static_cast<float>(count)) / elapsed);
  }
  for (int i = 0; i < opCount; ++i) {
    const uint32_t opTotal = stats[i].ok + stats[i].fail;
    const float opPct = (opTotal > 0U)
                            ? (100.0f * static_cast<float>(stats[i].ok) /
                               static_cast<float>(opTotal))
                            : 0.0f;
    Serial.printf("  %-10s %sok=%lu%s %sfail=%lu%s (%s%.1f%%%s)\n",
                  stats[i].name,
                  goodIfNonZeroColor(stats[i].ok),
                  static_cast<unsigned long>(stats[i].ok),
                  LOG_COLOR_RESET,
                  goodIfZeroColor(stats[i].fail),
                  static_cast<unsigned long>(stats[i].fail),
                  LOG_COLOR_RESET,
                  successRateColor(opPct),
                  opPct,
                  LOG_COLOR_RESET);
  }
  const uint32_t successDelta = device.totalSuccess() - succBefore;
  const uint32_t failDelta = device.totalFailures() - failBefore;
  Serial.printf("  Health delta: %ssuccess +%lu%s, %sfailures +%lu%s\n",
                goodIfNonZeroColor(successDelta),
                static_cast<unsigned long>(successDelta),
                LOG_COLOR_RESET,
                goodIfZeroColor(failDelta),
                static_cast<unsigned long>(failDelta),
                LOG_COLOR_RESET);
  Serial.println("  Health changes:");
  printHealthDiff(healthBefore, healthAfter);
  if (hasFailure) {
    Serial.println("  First failure:");
    printStatus(firstFailure);
    if (failTotal > 1U) {
      Serial.println("  Last failure:");
      printStatus(lastFailure);
    }
  }

  const BME280::Status restoreStatus = restoreStressMixSettings(originalSettings);
  if (!restoreStatus.ok()) {
    LOGW("Could not restore pre-stress settings: %s", errToStr(restoreStatus.code));
  }
}

void runSelfTest() {
  struct Result {
    uint32_t pass = 0;
    uint32_t fail = 0;
    uint32_t skip = 0;
  } result;

  enum class SelftestOutcome : uint8_t { PASS, FAIL, SKIP };
  auto report = [&](const char* name, SelftestOutcome outcome, const char* note) {
    const bool ok = (outcome == SelftestOutcome::PASS);
    const bool skip = (outcome == SelftestOutcome::SKIP);
    const char* color = skip ? LOG_COLOR_YELLOW : LOG_COLOR_RESULT(ok);
    const char* tag = skip ? "SKIP" : (ok ? "PASS" : "FAIL");
    Serial.printf("  [%s%s%s] %s", color, tag, LOG_COLOR_RESET, name);
    if (note && note[0]) {
      Serial.printf(" - %s", note);
    }
    Serial.println();
    if (skip) {
      result.skip++;
    } else if (ok) {
      result.pass++;
    } else {
      result.fail++;
    }
  };
  auto reportCheck = [&](const char* name, bool ok, const char* note) {
    report(name, ok ? SelftestOutcome::PASS : SelftestOutcome::FAIL, note);
  };
  auto reportSkip = [&](const char* name, const char* note) {
    report(name, SelftestOutcome::SKIP, note);
  };

  Serial.println("=== BME280 selftest (safe command smoke check) ===");
  Serial.println("  Plausibility ranges are loose and environment-dependent; this is not factory calibration.");
  cancelPending();

  BME280::Mode origMode = BME280::Mode::SLEEP;
  BME280::Oversampling origT = BME280::Oversampling::X1;
  BME280::Oversampling origP = BME280::Oversampling::X1;
  BME280::Oversampling origH = BME280::Oversampling::X1;
  BME280::Filter origFilter = BME280::Filter::OFF;
  BME280::Standby origStandby = BME280::Standby::MS_0_5;
  bool haveSnapshot = device.getMode(origMode).ok() &&
                      device.getOversamplingT(origT).ok() &&
                      device.getOversamplingP(origP).ok() &&
                      device.getOversamplingH(origH).ok() &&
                      device.getFilter(origFilter).ok() &&
                      device.getStandby(origStandby).ok();
  reportCheck("capture baseline settings", haveSnapshot, haveSnapshot ? "" : "could not read one or more fields");

  const uint32_t succBefore = device.totalSuccess();
  const uint32_t failBefore = device.totalFailures();
  const uint8_t consBefore = device.consecutiveFailures();

  BME280::Status st = device.probe();
  if (st.code == BME280::Err::NOT_INITIALIZED) {
    reportSkip("probe responds", "driver not initialized");
    reportSkip("remaining checks", "selftest aborted");
    Serial.printf("Selftest result: pass=%s%lu%s fail=%s%lu%s skip=%s%lu%s\n",
                  goodIfNonZeroColor(result.pass), static_cast<unsigned long>(result.pass), LOG_COLOR_RESET,
                  goodIfZeroColor(result.fail), static_cast<unsigned long>(result.fail), LOG_COLOR_RESET,
                  skipCountColor(result.skip), static_cast<unsigned long>(result.skip), LOG_COLOR_RESET);
    return;
  }
  reportCheck("probe responds", st.ok(), st.ok() ? "" : errToStr(st.code));
  const bool probeNoTrack = device.totalSuccess() == succBefore &&
                            device.totalFailures() == failBefore &&
                            device.consecutiveFailures() == consBefore;
  reportCheck("probe no-health-side-effects", probeNoTrack, "");

  uint8_t id = 0;
  st = device.readChipId(id);
  reportCheck("readChipId", st.ok(), st.ok() ? "" : errToStr(st.code));
  reportCheck("chip id matches 0x60", st.ok() && id == BME280::cmd::CHIP_ID_BME280, "");

  st = device.setMode(BME280::Mode::FORCED);
  reportCheck("setMode(FORCED)", st.ok(), st.ok() ? "" : errToStr(st.code));
  BME280::Mode modeNow = BME280::Mode::SLEEP;
  st = device.getMode(modeNow);
  reportCheck("getMode", st.ok(), st.ok() ? "" : errToStr(st.code));
  reportCheck("verify mode=FORCED", st.ok() && modeNow == BME280::Mode::FORCED, "");

  st = device.setOversamplingT(BME280::Oversampling::X2);
  reportCheck("setOversamplingT(X2)", st.ok(), st.ok() ? "" : errToStr(st.code));
  BME280::Oversampling os = BME280::Oversampling::SKIP;
  st = device.getOversamplingT(os);
  reportCheck("verify osrs_t=X2", st.ok() && os == BME280::Oversampling::X2, st.ok() ? "" : errToStr(st.code));

  st = device.setOversamplingP(BME280::Oversampling::X4);
  reportCheck("setOversamplingP(X4)", st.ok(), st.ok() ? "" : errToStr(st.code));
  st = device.getOversamplingP(os);
  reportCheck("verify osrs_p=X4", st.ok() && os == BME280::Oversampling::X4, st.ok() ? "" : errToStr(st.code));

  st = device.setOversamplingH(BME280::Oversampling::X2);
  reportCheck("setOversamplingH(X2)", st.ok(), st.ok() ? "" : errToStr(st.code));
  st = device.getOversamplingH(os);
  reportCheck("verify osrs_h=X2", st.ok() && os == BME280::Oversampling::X2, st.ok() ? "" : errToStr(st.code));

  st = device.setFilter(BME280::Filter::X4);
  reportCheck("setFilter(X4)", st.ok(), st.ok() ? "" : errToStr(st.code));
  BME280::Filter fil = BME280::Filter::OFF;
  st = device.getFilter(fil);
  reportCheck("verify filter=X4", st.ok() && fil == BME280::Filter::X4, st.ok() ? "" : errToStr(st.code));

  st = device.setStandby(BME280::Standby::MS_125);
  reportCheck("setStandby(125ms)", st.ok(), st.ok() ? "" : errToStr(st.code));
  BME280::Standby sb = BME280::Standby::MS_0_5;
  st = device.getStandby(sb);
  reportCheck("verify standby=125ms", st.ok() && sb == BME280::Standby::MS_125, st.ok() ? "" : errToStr(st.code));

  BME280::Measurement m;
  st = performMeasurementBlocking(m);
  reportCheck("measurement cycle", st.ok(), st.ok() ? "" : errToStr(st.code));
  const bool mRange = m.temperatureValid && m.pressureValid && m.humidityValid &&
                      (m.temperatureC > -60.0f && m.temperatureC < 130.0f) &&
                      (m.humidityPct >= 0.0f && m.humidityPct <= 100.0f) &&
                      (m.pressurePa > 20000.0f && m.pressurePa < 130000.0f);
  reportCheck("measurement in plausible range", st.ok() && mRange, "");

  BME280::RawSample raw;
  st = device.getRawSample(raw);
  reportCheck("getRawSample", st.ok(), st.ok() ? "" : errToStr(st.code));

  BME280::CompensatedSample comp;
  st = device.getCompensatedSample(comp);
  reportCheck("getCompensatedSample", st.ok(), st.ok() ? "" : errToStr(st.code));

  bool measuring = false;
  st = device.isMeasuring(measuring);
  reportCheck("isMeasuring", st.ok(), st.ok() ? "" : errToStr(st.code));

  const uint32_t measMs = device.estimateMeasurementTimeMs();
  const uint32_t standbyMs = device.getStandbyTimeMs();
  const uint32_t cycleMs = device.estimateNormalCycleMs();
  reportCheck("estimateMeasurementTimeMs>0", measMs > 0U, "");
  reportCheck("estimateNormalCycleMs>=meas", cycleMs >= measMs, "");
  reportCheck("getStandbyTimeMs valid", standbyMs > 0U || modeNow != BME280::Mode::NORMAL, "");

  uint8_t statusReg = 0;
  st = device.readStatus(statusReg);
  reportCheck("readStatus", st.ok(), st.ok() ? "" : errToStr(st.code));

  st = device.recover();
  reportCheck("recover", st.ok(), st.ok() ? "" : errToStr(st.code));
  reportCheck("isOnline", device.isOnline(), "");

  if (haveSnapshot) {
    device.setMode(origMode);
    device.setOversamplingT(origT);
    device.setOversamplingP(origP);
    device.setOversamplingH(origH);
    device.setFilter(origFilter);
    device.setStandby(origStandby);
  }

  Serial.printf("Selftest result: pass=%s%lu%s fail=%s%lu%s skip=%s%lu%s\n",
                goodIfNonZeroColor(result.pass), static_cast<unsigned long>(result.pass), LOG_COLOR_RESET,
                goodIfZeroColor(result.fail), static_cast<unsigned long>(result.fail), LOG_COLOR_RESET,
                skipCountColor(result.skip), static_cast<unsigned long>(result.skip), LOG_COLOR_RESET);
}

void cancelPending() {
  pendingRead = false;
  stressRemaining = 0;
  stressStats.active = false;

  BME280::SettingsSnapshot snapshot;
  if (!device.getSettings(snapshot).ok() || !snapshot.initialized) {
    return;
  }

  if (snapshot.measurementReady) {
    BME280::Measurement discarded;
    (void)device.getMeasurement(discarded);
    return;
  }

  if (snapshot.measurementRequested) {
    const BME280::Mode restoreMode = snapshot.mode;
    BME280::Status st = device.setMode(BME280::Mode::SLEEP);
    if (st.ok() && restoreMode != BME280::Mode::SLEEP) {
      (void)device.setMode(restoreMode);
    }
  }
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
  } else if (st.code == BME280::Err::BUSY && driverMeasurementPending()) {
    BME280::SettingsSnapshot snapshot;
    (void)device.getSettings(snapshot);
    pendingRead = true;
    pendingStartMs = snapshot.measurementStartMs;
    st = BME280::Status::Error(BME280::Err::IN_PROGRESS,
                               "Measurement already in progress");
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
      printStressProgress(static_cast<uint32_t>(stressStats.attempts),
                          static_cast<uint32_t>(stressStats.target),
                          static_cast<uint32_t>(stressStats.success),
                          stressStats.errors);
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
    printStressProgress(static_cast<uint32_t>(stressStats.attempts),
                        static_cast<uint32_t>(stressStats.target),
                        static_cast<uint32_t>(stressStats.success),
                        stressStats.errors);
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

bool parseU32(const String& token, uint32_t& out) {
  char* end = nullptr;
  const unsigned long value = strtoul(token.c_str(), &end, 0);
  if (end == token.c_str() || *end != '\0') {
    return false;
  }
  out = static_cast<uint32_t>(value);
  return true;
}

bool parseI2cAddress(const String& token, uint8_t& out) {
  uint32_t value = 0;
  if (!parseU32(token, value) || (value != 0x76U && value != 0x77U)) {
    return false;
  }
  out = static_cast<uint8_t>(value);
  return true;
}

void printActiveAddress() {
  Serial.printf("Active I2C address: 0x%02X (%s)\n",
                activeAddress,
                activeAddress == 0x76 ? "SDO=GND" : "SDO=VDDIO");
}

void printHelp() {
  Serial.println();
  cli::printHelpHeader("BME280 CLI Help");
  cli::printHelpSection("Common");
  cli::printHelpItem("help / ?", "Show this help");
  cli::printHelpItem("version / ver", "Print firmware and library version info");
  cli::printHelpItem("scan", "Scan I2C bus");
  cli::printHelpItem("addr [0x76|0x77]", "Show or select diagnostic I2C address");
  cli::printHelpItem("begin", "Run begin() with the default example config");
  cli::printHelpItem("read", "Request and display measurement");
  cli::printHelpItem("force", "Trigger one forced-mode measurement");
  cli::printHelpItem("normal on/off", "Enable normal mode or return to sleep");
  cli::printHelpItem("raw", "Show cached raw ADC sample and validity flags");
  cli::printHelpItem("comp", "Show cached compensated sample and validity flags");
  cli::printHelpItem("data", "Burst-read and decode live data registers");
  cli::printHelpItem("measuring", "Show measuring flag");
  cli::printHelpItem("timing", "Show measurement and cycle timing estimates");

  cli::printHelpSection("Configuration");
  cli::printHelpItem("mode [sleep|forced|normal]", "Set or show operating mode");
  cli::printHelpItem("osrs [t|p|h <0..5>]", "Set or show oversampling");
  cli::printHelpItem("filter [0..4]", "Set or show IIR filter (temperature/pressure only)");
  cli::printHelpItem("standby [0..7]", "Set or show standby time");
  cli::printHelpItem("cfg / settings", "Show chip and internal settings");
  cli::printHelpItem("calib [raw]", "Show cached or raw calibration");
  cli::printHelpItem("status", "Read status register");
  cli::printHelpItem("id / chipid", "Read chip ID");
  cli::printHelpItem("reset", "Soft reset device");

  cli::printHelpSection("Registers");
  cli::printHelpItem("reg <addr>", "Read 8-bit register (hex address)");
  cli::printHelpItem("wreg <addr> <val>", "Write 8-bit register (diagnostic only; may desync cached config)");

  cli::printHelpSection("Diagnostics");
  cli::printHelpItem("drv", "Show driver state and health");
  cli::printHelpItem("state", "Show compact one-line health summary");
  cli::printHelpItem("probe", "Probe device (no health tracking)");
  cli::printHelpItem("recover", "Manual recovery attempt");
  cli::printHelpItem("verbose [0|1]", "Enable/disable verbose output");
  cli::printHelpItem("stress [N]", "Run N measurement cycles");
  cli::printHelpItem("stress_mix [N]", "Run N mixed-operation cycles");
  cli::printHelpItem("selftest", "Run safe command smoke-test report");
}

void printVersionInfo() {
  Serial.println("=== Version Info ===");
  Serial.printf("  Example firmware build: %s %s\n", __DATE__, __TIME__);
  Serial.printf("  BME280 library version: %s\n", BME280::VERSION);
  Serial.printf("  BME280 library full: %s\n", BME280::VERSION_FULL);
  Serial.printf("  BME280 library build: %s\n", BME280::BUILD_TIMESTAMP);
  Serial.printf("  BME280 library commit: %s (%s)\n", BME280::GIT_COMMIT, BME280::GIT_STATUS);
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

  if (cmd == "version" || cmd == "ver") {
    printVersionInfo();
    return;
  }

  if (cmd == "scan") {
    bus_diag::scan();
    return;
  }

  if (cmd == "addr") {
    printActiveAddress();
    return;
  }

  if (cmd.startsWith("addr ")) {
    String arg = cmd.substring(5);
    arg.trim();
    uint8_t address = 0;
    if (!parseI2cAddress(arg, address)) {
      LOGW("Usage: addr 0x76|0x77");
      return;
    }

    LOGI("Selecting BME280 address 0x%02X", address);
    cancelPending();
    device.end();
    activeAddress = address;
    BME280::Status st = device.begin(makeDefaultConfig());
    printStatus(st);
    if (st.ok()) {
      printDriverHealth();
    }
    return;
  }

  if (cmd == "begin") {
    LOGI("Initializing BME280...");
    cancelPending();
    device.end();
    BME280::Status st = device.begin(makeDefaultConfig());
    printStatus(st);
    if (st.ok()) {
      printDriverHealth();
    }
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

  if (cmd == "force") {
    cancelPending();
    BME280::Status st = device.setMode(BME280::Mode::FORCED);
    if (st.ok()) {
      st = scheduleMeasurement();
    }
    if (st.code != BME280::Err::IN_PROGRESS) {
      printStatus(st);
    }
    return;
  }

  if (cmd == "normal") {
    printModeSettings();
    return;
  }

  if (cmd == "normal on" || cmd == "normal off") {
    cancelPending();
    const BME280::Mode mode =
        (cmd == "normal on") ? BME280::Mode::NORMAL : BME280::Mode::SLEEP;
    BME280::Status st = device.setMode(mode);
    printStatus(st);
    return;
  }

  if (cmd == "raw") {
    printRawSample();
    return;
  }

  if (cmd == "comp") {
    printCompensatedSample();
    return;
  }

  if (cmd == "data") {
    printDataRegisters();
    return;
  }

  if (cmd == "measuring") {
    bool measuring = false;
    BME280::Status st = device.isMeasuring(measuring);
    if (!st.ok()) {
      printStatus(st);
      return;
    }
    Serial.printf("Measuring: %s\n", measuring ? "YES" : "NO");
    return;
  }

  if (cmd == "timing") {
    printTimingInfo();
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
    Serial.printf("Driver: state=%s online=%s dirty=%s\n",
                  stateToStr(device.state()),
                  device.isOnline() ? "true" : "false",
                  device.hardwareConfigDirty() ? "true" : "false");
    return;
  }

  if (cmd == "chipid" || cmd == "id") {
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

  if (cmd.startsWith("wreg ")) {
    String args = cmd.substring(5);
    args.trim();
    const int split = args.indexOf(' ');
    if (split < 0) {
      LOGW("Usage: wreg <addr> <val>");
      return;
    }

    uint32_t addr = 0;
    uint32_t value = 0;
    if (!parseU32(args.substring(0, split), addr) ||
        !parseU32(args.substring(split + 1), value) ||
        addr > 0xFFu || value > 0xFFu) {
      LOGW("Usage: wreg <addr> <val>");
      return;
    }

    BME280::Status st = device.writeRegister(static_cast<uint8_t>(addr), static_cast<uint8_t>(value));
    printStatus(st);
    return;
  }

  if (cmd.startsWith("reg ")) {
    uint32_t addr = 0;
    if (!parseU32(cmd.substring(4), addr) || addr > 0xFFu) {
      LOGW("Usage: reg <addr>");
      return;
    }

    uint8_t value = 0;
    BME280::Status st = device.readRegister(static_cast<uint8_t>(addr), value);
    if (!st.ok()) {
      printStatus(st);
      return;
    }

    Serial.printf("Reg 0x%02lX = 0x%02X (%u)\n",
                  static_cast<unsigned long>(addr),
                  value,
                  value);
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

  if (cmd == "state") {
    printHealthView(device);
    return;
  }

  if (cmd == "probe") {
    LOGI("Probing device (no health tracking)...");
    HealthSnapshot<BME280::BME280> before;
    before.capture(device);
    BME280::Status st = device.probe();
    printStatus(st);
    HealthSnapshot<BME280::BME280> after;
    after.capture(device);
    Serial.println("  Health changes:");
    printHealthDiff(before, after);
    return;
  }

  if (cmd == "recover") {
    LOGI("Attempting recovery...");
    HealthSnapshot<BME280::BME280> before;
    before.capture(device);
    BME280::Status st = device.recover();
    printStatus(st);
    HealthSnapshot<BME280::BME280> after;
    after.capture(device);
    Serial.println("  Health changes:");
    printHealthDiff(before, after);
    printDriverHealth();
    Serial.println("  Note: cached raw/comp samples may predate recovery; run read before using them.");
    return;
  }

  if (cmd == "verbose") {
    printVerboseState();
    return;
  }

  if (cmd.startsWith("verbose ")) {
    const int val = cmd.substring(8).toInt();
    verboseMode = (val != 0);
    LOGI("Verbose mode: %s%s%s",
         onOffColor(verboseMode),
         verboseMode ? "ON" : "OFF",
         LOG_COLOR_RESET);
    return;
  }

  if (cmd == "selftest") {
    runSelfTest();
    return;
  }

  if (cmd == "stress_mix") {
    runStressMix(50);
    return;
  }

  if (cmd.startsWith("stress_mix ")) {
    int count = cmd.substring(11).toInt();
    if (count <= 0 || count > 100000) {
      LOGW("Invalid stress_mix count");
      return;
    }
    runStressMix(count);
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
    const BME280::Status st = ensureForcedMeasurementMode();
    if (!st.ok()) {
      printStatus(st);
      return;
    }
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
  LOGI("BME280 diagnostic address 0x%02X", activeAddress);

  bus_diag::scan();

  BME280::Config cfg = makeDefaultConfig();
  BME280::Status st = device.begin(cfg);
  if (!st.ok()) {
    LOGE("Failed to initialize device");
    printStatus(st);
    return;
  }

  LOGI("Device initialized successfully");
  printDriverHealth();
  printHelp();
  cli::printPrompt();
}

void loop() {
  device.tick(millis());

  if (stressStats.active && stressRemaining > 0 && !pendingRead) {
    const BME280::Status st = scheduleMeasurement();
    if (st.code != BME280::Err::IN_PROGRESS) {
      noteStressError(st);
      stressStats.attempts++;
      stressRemaining--;
      printStressProgress(static_cast<uint32_t>(stressStats.attempts),
                          static_cast<uint32_t>(stressStats.target),
                          static_cast<uint32_t>(stressStats.success),
                          stressStats.errors);
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
        cli::printPrompt();
      }
    } else {
      inputBuffer += c;
    }
  }
}
