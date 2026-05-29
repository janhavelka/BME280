/// @file main.cpp
/// @brief ESP-IDF native bringup CLI example for BME280.
/// @note This is an EXAMPLE, not part of the library.

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <driver/i2c_master.h>
#include <esp_err.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include "BME280/BME280.h"
#include "IdfI2cTransport.h"

#ifndef BME280_IDF_I2C_SDA
#define BME280_IDF_I2C_SDA 8
#endif

#ifndef BME280_IDF_I2C_SCL
#define BME280_IDF_I2C_SCL 9
#endif

#ifndef BME280_IDF_I2C_FREQ_HZ
#define BME280_IDF_I2C_FREQ_HZ 400000
#endif

namespace {

constexpr char LOG_COLOR_RESET[] = "\033[0m";
constexpr char LOG_COLOR_RED[] = "\033[31m";
constexpr char LOG_COLOR_GREEN[] = "\033[32m";
constexpr char LOG_COLOR_YELLOW[] = "\033[33m";
constexpr char LOG_COLOR_BLUE[] = "\033[34m";
constexpr char LOG_COLOR_CYAN[] = "\033[36m";
constexpr char LOG_COLOR_GRAY[] = "\033[90m";
constexpr size_t HELP_COMMAND_WIDTH = 32U;
constexpr uint8_t BME280_ADDR = 0x76;
constexpr uint32_t I2C_TIMEOUT_MS = 50;
constexpr uint32_t CLI_TICK_MS = 5;
constexpr size_t CLI_LINE_LEN = 160U;
constexpr int CLI_QUEUE_DEPTH = 4;
constexpr uint32_t DEFAULT_STRESS_COUNT = 10U;
constexpr uint32_t DEFAULT_STRESS_MIX_COUNT = 50U;
constexpr uint32_t MAX_STRESS_COUNT = 100000U;

#define LOG_COLOR_RESULT(ok) ((ok) ? LOG_COLOR_GREEN : LOG_COLOR_RED)
#define LOG_PRINT(tagColor, tag, fmt, ...) \
  do { \
    std::printf(tagColor "[" tag "]" LOG_COLOR_RESET " " fmt "\n", ##__VA_ARGS__); \
    std::fflush(stdout); \
  } while (0)
#define LOGE(fmt, ...) LOG_PRINT(LOG_COLOR_RED, "E", fmt, ##__VA_ARGS__)
#define LOGW(fmt, ...) LOG_PRINT(LOG_COLOR_YELLOW, "W", fmt, ##__VA_ARGS__)
#define LOGI(fmt, ...) LOG_PRINT(LOG_COLOR_CYAN, "I", fmt, ##__VA_ARGS__)

struct CliLine {
  char text[CLI_LINE_LEN] = {};
};

struct HealthSnapshot {
  BME280::DriverState state = BME280::DriverState::UNINIT;
  bool online = false;
  uint8_t consecutiveFailures = 0;
  uint32_t totalFailures = 0;
  uint32_t totalSuccess = 0;

  void capture(const BME280::BME280& driver) {
    state = driver.state();
    online = driver.isOnline();
    consecutiveFailures = driver.consecutiveFailures();
    totalFailures = driver.totalFailures();
    totalSuccess = driver.totalSuccess();
  }
};

struct StressStats {
  bool active = false;
  uint32_t startMs = 0;
  int target = 0;
  int attempts = 0;
  int success = 0;
  uint32_t errors = 0;
  bool hasSample = false;
  bool hasFailure = false;
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

BME280::BME280 device;
QueueHandle_t gLineQueue = nullptr;
bool gVerbose = false;
bool gPendingRead = false;
uint32_t gPendingStartMs = 0;
int gStressRemaining = 0;
StressStats gStress;

uint32_t nowMs(void*) {
  return static_cast<uint32_t>(esp_timer_get_time() / 1000LL);
}

uint32_t currentMs() {
  return nowMs(nullptr);
}

const char* boolStr(bool value) {
  return value ? "true" : "false";
}

const char* errToStr(BME280::Err err) {
  using namespace BME280;
  switch (err) {
    case Err::OK: return "OK";
    case Err::NOT_INITIALIZED: return "NOT_INITIALIZED";
    case Err::INVALID_CONFIG: return "INVALID_CONFIG";
    case Err::I2C_ERROR: return "I2C_ERROR";
    case Err::TIMEOUT: return "TIMEOUT";
    case Err::INVALID_PARAM: return "INVALID_PARAM";
    case Err::DEVICE_NOT_FOUND: return "DEVICE_NOT_FOUND";
    case Err::CHIP_ID_MISMATCH: return "CHIP_ID_MISMATCH";
    case Err::CALIBRATION_INVALID: return "CALIBRATION_INVALID";
    case Err::MEASUREMENT_NOT_READY: return "MEASUREMENT_NOT_READY";
    case Err::COMPENSATION_ERROR: return "COMPENSATION_ERROR";
    case Err::BUSY: return "BUSY";
    case Err::IN_PROGRESS: return "IN_PROGRESS";
    case Err::I2C_NACK_ADDR: return "I2C_NACK_ADDR";
    case Err::I2C_NACK_DATA: return "I2C_NACK_DATA";
    case Err::I2C_TIMEOUT: return "I2C_TIMEOUT";
    case Err::I2C_BUS: return "I2C_BUS";
    default: return "UNKNOWN";
  }
}

const char* stateToStr(BME280::DriverState state) {
  switch (state) {
    case BME280::DriverState::UNINIT: return "UNINIT";
    case BME280::DriverState::READY: return "READY";
    case BME280::DriverState::DEGRADED: return "DEGRADED";
    case BME280::DriverState::OFFLINE: return "OFFLINE";
    default: return "UNKNOWN";
  }
}

const char* modeToStr(BME280::Mode mode) {
  switch (mode) {
    case BME280::Mode::SLEEP: return "SLEEP";
    case BME280::Mode::FORCED: return "FORCED";
    case BME280::Mode::NORMAL: return "NORMAL";
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

const char* zeroGoodColor(uint32_t value) {
  return (value == 0U) ? LOG_COLOR_GREEN : LOG_COLOR_RED;
}

const char* nonZeroGoodColor(uint32_t value) {
  return (value > 0U) ? LOG_COLOR_GREEN : LOG_COLOR_YELLOW;
}

const char* successRateColor(float pct) {
  if (pct >= 99.9f) return LOG_COLOR_GREEN;
  if (pct >= 80.0f) return LOG_COLOR_YELLOW;
  return LOG_COLOR_RED;
}

char* trim(char* value) {
  if (value == nullptr) {
    return value;
  }
  while (*value == ' ' || *value == '\t') {
    ++value;
  }
  char* end = value + std::strlen(value);
  while (end > value && (end[-1] == ' ' || end[-1] == '\t' || end[-1] == '\r' || end[-1] == '\n')) {
    *--end = '\0';
  }
  return value;
}

char* nextToken(char*& cursor) {
  cursor = trim(cursor);
  if (cursor == nullptr || *cursor == '\0') {
    return nullptr;
  }
  char* token = cursor;
  while (*cursor != '\0' && *cursor != ' ' && *cursor != '\t') {
    ++cursor;
  }
  if (*cursor != '\0') {
    *cursor++ = '\0';
  }
  return token;
}

bool parseU32(const char* token, uint32_t& out) {
  if (token == nullptr || *token == '\0') {
    return false;
  }
  char* end = nullptr;
  const unsigned long value = std::strtoul(token, &end, 0);
  if (end == token || *end != '\0') {
    return false;
  }
  out = static_cast<uint32_t>(value);
  return true;
}

BME280::Config makeDefaultConfig() {
  BME280::Config cfg;
  cfg.i2cWrite = idfI2cWrite;
  cfg.i2cWriteRead = idfI2cWriteRead;
  cfg.i2cUser = &bme280IdfI2cContext();
  cfg.i2cAddress = BME280_ADDR;
  cfg.i2cTimeoutMs = I2C_TIMEOUT_MS;
  cfg.nowMs = nowMs;
  cfg.offlineThreshold = 5;
  return cfg;
}

void printPrompt() {
  std::printf("> ");
  std::fflush(stdout);
}

void printHelpItem(const char* command, const char* description) {
  std::printf("  %s%-*s%s - %s\n",
              LOG_COLOR_CYAN,
              static_cast<int>(HELP_COMMAND_WIDTH),
              command,
              LOG_COLOR_RESET,
              description);
}

void printStatus(const BME280::Status& st) {
  std::printf("  Status: %s%s%s (code=%u, detail=%ld)\n",
              LOG_COLOR_RESULT(st.ok()),
              errToStr(st.code),
              LOG_COLOR_RESET,
              static_cast<unsigned>(st.code),
              static_cast<long>(st.detail));
  if (st.msg != nullptr && st.msg[0] != '\0') {
    std::printf("  Message: %s%s%s\n", LOG_COLOR_YELLOW, st.msg, LOG_COLOR_RESET);
  }
}

void printHealthDiff(const HealthSnapshot& before, const HealthSnapshot& after) {
  bool changed = false;
  if (before.state != after.state) {
    std::printf("  State: %s -> %s\n", stateToStr(before.state), stateToStr(after.state));
    changed = true;
  }
  if (before.online != after.online) {
    std::printf("  Online: %s -> %s\n", boolStr(before.online), boolStr(after.online));
    changed = true;
  }
  if (before.consecutiveFailures != after.consecutiveFailures) {
    std::printf("  ConsecFail: %u -> %u\n",
                static_cast<unsigned>(before.consecutiveFailures),
                static_cast<unsigned>(after.consecutiveFailures));
    changed = true;
  }
  if (before.totalSuccess != after.totalSuccess) {
    std::printf("  TotalOK: %lu -> %lu (+%lu)\n",
                static_cast<unsigned long>(before.totalSuccess),
                static_cast<unsigned long>(after.totalSuccess),
                static_cast<unsigned long>(after.totalSuccess - before.totalSuccess));
    changed = true;
  }
  if (before.totalFailures != after.totalFailures) {
    std::printf("  TotalFail: %lu -> %lu (+%lu)\n",
                static_cast<unsigned long>(before.totalFailures),
                static_cast<unsigned long>(after.totalFailures),
                static_cast<unsigned long>(after.totalFailures - before.totalFailures));
    changed = true;
  }
  if (!changed) {
    std::printf("  (no health changes)\n");
  }
}

void printCompactHealth() {
  const uint32_t totalOk = device.totalSuccess();
  const uint32_t totalFail = device.totalFailures();
  const uint32_t total = totalOk + totalFail;
  const float pct = (total > 0U) ? (100.0f * static_cast<float>(totalOk) / total) : 0.0f;
  std::printf("Health: state=%s online=%s consec=%u ok=%lu fail=%lu rate=%.1f%%\n",
              stateToStr(device.state()),
              boolStr(device.isOnline()),
              static_cast<unsigned>(device.consecutiveFailures()),
              static_cast<unsigned long>(totalOk),
              static_cast<unsigned long>(totalFail),
              pct);
}

void printDriverHealth() {
  const uint32_t now = currentMs();
  const uint32_t totalOk = device.totalSuccess();
  const uint32_t totalFail = device.totalFailures();
  const uint32_t total = totalOk + totalFail;
  const float successRate =
      (total > 0U) ? (100.0f * static_cast<float>(totalOk) / static_cast<float>(total)) : 0.0f;
  const BME280::Status lastErr = device.lastError();

  std::printf("=== Driver Health ===\n");
  std::printf("  State: %s%s%s\n",
              device.consecutiveFailures() == 0 ? LOG_COLOR_GREEN : LOG_COLOR_YELLOW,
              stateToStr(device.state()),
              LOG_COLOR_RESET);
  std::printf("  Online: %s%s%s\n",
              device.isOnline() ? LOG_COLOR_GREEN : LOG_COLOR_RED,
              boolStr(device.isOnline()),
              LOG_COLOR_RESET);
  std::printf("  Consecutive failures: %s%u%s\n",
              zeroGoodColor(device.consecutiveFailures()),
              static_cast<unsigned>(device.consecutiveFailures()),
              LOG_COLOR_RESET);
  std::printf("  Total success: %s%lu%s\n",
              nonZeroGoodColor(totalOk),
              static_cast<unsigned long>(totalOk),
              LOG_COLOR_RESET);
  std::printf("  Total failures: %s%lu%s\n",
              zeroGoodColor(totalFail),
              static_cast<unsigned long>(totalFail),
              LOG_COLOR_RESET);
  std::printf("  Success rate: %s%.1f%%%s\n", successRateColor(successRate), successRate, LOG_COLOR_RESET);
  std::printf("  Last OK: %s\n", device.lastOkMs() == 0U ? "never" : "");
  if (device.lastOkMs() != 0U) {
    std::printf("    %lu ms ago (at %lu ms)\n",
                static_cast<unsigned long>(now - device.lastOkMs()),
                static_cast<unsigned long>(device.lastOkMs()));
  }
  std::printf("  Last error: %s\n", device.lastErrorMs() == 0U ? "never" : "");
  if (device.lastErrorMs() != 0U) {
    std::printf("    %lu ms ago (at %lu ms)\n",
                static_cast<unsigned long>(now - device.lastErrorMs()),
                static_cast<unsigned long>(device.lastErrorMs()));
  }
  if (!lastErr.ok()) {
    std::printf("  Error code: %s%s%s\n", LOG_COLOR_RED, errToStr(lastErr.code), LOG_COLOR_RESET);
    std::printf("  Error detail: %ld\n", static_cast<long>(lastErr.detail));
    if (lastErr.msg != nullptr && lastErr.msg[0] != '\0') {
      std::printf("  Error msg: %s\n", lastErr.msg);
    }
  }
}

void printMeasurement(const BME280::Measurement& sample) {
  std::printf("Temp: %.2f C, Pressure: %.2f Pa, Humidity: %.2f %%\n",
              sample.temperatureC,
              sample.pressurePa,
              sample.humidityPct);
}

void printRawSample() {
  BME280::RawSample raw;
  const BME280::Status st = device.getRawSample(raw);
  if (!st.ok()) {
    printStatus(st);
    return;
  }
  std::printf("Raw ADC: T=%ld P=%ld H=%ld\n",
              static_cast<long>(raw.adcT),
              static_cast<long>(raw.adcP),
              static_cast<long>(raw.adcH));
}

void printCompensatedSample() {
  BME280::CompensatedSample sample;
  const BME280::Status st = device.getCompensatedSample(sample);
  if (!st.ok()) {
    printStatus(st);
    return;
  }
  std::printf("Compensated: T=%ld x0.01C, P=%lu Pa, RH=%.2f %%\n",
              static_cast<long>(sample.tempC_x100),
              static_cast<unsigned long>(sample.pressurePa),
              static_cast<float>(sample.humidityPct_x1024) / 1024.0f);
}

void printDataRegisters() {
  uint8_t data[BME280::cmd::DATA_LEN] = {};
  const BME280::Status st = device.readRegisters(BME280::cmd::REG_DATA_START, data, sizeof(data));
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
  const int32_t adcH = (static_cast<int32_t>(data[6]) << 8) | static_cast<int32_t>(data[7]);

  std::printf("=== Live Data Registers ===\n  0xF7..0xFE: ");
  for (size_t i = 0; i < sizeof(data); ++i) {
    std::printf("%02X%s", data[i], (i + 1U < sizeof(data)) ? " " : "");
  }
  std::printf("\n  Decoded raw ADC: P=%ld T=%ld H=%ld\n",
              static_cast<long>(adcP),
              static_cast<long>(adcT),
              static_cast<long>(adcH));
  std::printf("  Sentinel check: P_skip=%d T_skip=%d H_skip=%d\n",
              adcP == 0x80000L ? 1 : 0,
              adcT == 0x80000L ? 1 : 0,
              adcH == 0x8000L ? 1 : 0);
}

void printTimingInfo() {
  bool measuring = false;
  const BME280::Status st = device.isMeasuring(measuring);
  if (!st.ok()) {
    printStatus(st);
    return;
  }
  std::printf("Measuring: %s\n", measuring ? "YES" : "NO");
  std::printf("Estimated measurement time: %lu ms\n",
              static_cast<unsigned long>(device.estimateMeasurementTimeMs()));
  std::printf("Configured standby: %lu ms\n",
              static_cast<unsigned long>(device.getStandbyTimeMs()));
  std::printf("Estimated normal cycle: %lu ms\n",
              static_cast<unsigned long>(device.estimateNormalCycleMs()));
}

void printCalibration() {
  BME280::Calibration calib;
  const BME280::Status st = device.getCalibration(calib);
  if (!st.ok()) {
    printStatus(st);
    return;
  }
  std::printf("=== Calibration (Cached) ===\n");
  std::printf("  T1=%u T2=%d T3=%d\n", calib.digT1, calib.digT2, calib.digT3);
  std::printf("  P1=%u P2=%d P3=%d P4=%d P5=%d P6=%d P7=%d P8=%d P9=%d\n",
              calib.digP1,
              calib.digP2,
              calib.digP3,
              calib.digP4,
              calib.digP5,
              calib.digP6,
              calib.digP7,
              calib.digP8,
              calib.digP9);
  std::printf("  H1=%u H2=%d H3=%u H4=%d H5=%d H6=%d\n",
              calib.digH1,
              calib.digH2,
              calib.digH3,
              calib.digH4,
              calib.digH5,
              calib.digH6);
}

void printCalibrationRaw() {
  BME280::CalibrationRaw raw;
  const BME280::Status st = device.readCalibrationRaw(raw);
  if (!st.ok()) {
    printStatus(st);
    return;
  }
  std::printf("=== Calibration (Raw Registers) ===\n  TP: ");
  for (size_t i = 0; i < sizeof(raw.tp); ++i) {
    std::printf("%02X%s", raw.tp[i], (i + 1U < sizeof(raw.tp)) ? " " : "");
  }
  std::printf("\n  H1: %02X\n  H: ", raw.h1);
  for (size_t i = 0; i < sizeof(raw.h); ++i) {
    std::printf("%02X%s", raw.h[i], (i + 1U < sizeof(raw.h)) ? " " : "");
  }
  std::printf("\n");
}

void printAllSettings() {
  uint8_t ctrlHum = 0;
  uint8_t ctrlMeas = 0;
  uint8_t config = 0;
  BME280::SettingsSnapshot snap;
  BME280::Status st = device.readCtrlHum(ctrlHum);
  if (!st.ok()) {
    printStatus(st);
    return;
  }
  st = device.readCtrlMeas(ctrlMeas);
  if (!st.ok()) {
    printStatus(st);
    return;
  }
  st = device.readConfig(config);
  if (!st.ok()) {
    printStatus(st);
    return;
  }
  (void)device.getSettings(snap);
  const uint8_t osrsH = (ctrlHum & BME280::cmd::MASK_CTRL_HUM_OSRS_H) >>
                        BME280::cmd::BIT_CTRL_HUM_OSRS_H;
  const uint8_t osrsT = (ctrlMeas & BME280::cmd::MASK_CTRL_MEAS_OSRS_T) >>
                        BME280::cmd::BIT_CTRL_MEAS_OSRS_T;
  const uint8_t osrsP = (ctrlMeas & BME280::cmd::MASK_CTRL_MEAS_OSRS_P) >>
                        BME280::cmd::BIT_CTRL_MEAS_OSRS_P;
  const uint8_t modeBits = (ctrlMeas & BME280::cmd::MASK_CTRL_MEAS_MODE) >>
                           BME280::cmd::BIT_CTRL_MEAS_MODE;
  const uint8_t filter = (config & BME280::cmd::MASK_CONFIG_FILTER) >>
                         BME280::cmd::BIT_CONFIG_FILTER;
  const uint8_t standby = (config & BME280::cmd::MASK_CONFIG_T_SB) >>
                          BME280::cmd::BIT_CONFIG_T_SB;

  std::printf("=== Chip Settings ===\n");
  std::printf("  ctrl_hum: 0x%02X (osrs_h=%u %s)\n", ctrlHum, osrsH, osrsToStr(osrsH));
  std::printf("  ctrl_meas: 0x%02X (osrs_t=%u %s, osrs_p=%u %s, mode=%u %s)\n",
              ctrlMeas,
              osrsT,
              osrsToStr(osrsT),
              osrsP,
              osrsToStr(osrsP),
              modeBits,
              modeToStr(static_cast<BME280::Mode>(modeBits)));
  std::printf("  config: 0x%02X (standby=%u %s, filter=%u %s, spi3w_en=%u)\n",
              config,
              standby,
              standbyToStr(standby),
              filter,
              filterToStr(filter),
              (config & BME280::cmd::MASK_CONFIG_SPI3W_EN) != 0 ? 1U : 0U);
  std::printf("=== Internal Settings ===\n");
  std::printf("  Mode: %s\n", modeToStr(snap.mode));
  std::printf("  osrs_t: %s (%u)\n", osrsToStr(static_cast<uint8_t>(snap.osrsT)), static_cast<unsigned>(snap.osrsT));
  std::printf("  osrs_p: %s (%u)\n", osrsToStr(static_cast<uint8_t>(snap.osrsP)), static_cast<unsigned>(snap.osrsP));
  std::printf("  osrs_h: %s (%u)\n", osrsToStr(static_cast<uint8_t>(snap.osrsH)), static_cast<unsigned>(snap.osrsH));
  std::printf("  Filter: %s (%u)\n", filterToStr(static_cast<uint8_t>(snap.filter)), static_cast<unsigned>(snap.filter));
  std::printf("  Standby: %s (%u)\n", standbyToStr(static_cast<uint8_t>(snap.standby)), static_cast<unsigned>(snap.standby));
  std::printf("  Verbose: %s\n", gVerbose ? "ON" : "OFF");
}

void scanBus() {
  IdfI2cContext& ctx = bme280IdfI2cContext();
  if (ctx.bus == nullptr) {
    LOGW("I2C scan unavailable");
    return;
  }
  LOGI("Scanning I2C bus...");
  int count = 0;
  for (uint8_t addr = 1; addr < 127; ++addr) {
    const esp_err_t err = i2c_master_probe(ctx.bus, addr, I2C_TIMEOUT_MS);
    if (err == ESP_OK) {
      std::printf("  Found device at 0x%02X\n", static_cast<unsigned>(addr));
      ++count;
    }
  }
  if (count == 0) {
    LOGW("No I2C devices found");
  } else {
    LOGI("Found %d device(s)", count);
  }
}

void cancelPending() {
  gPendingRead = false;
  gStressRemaining = 0;
  gStress.active = false;
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
    const BME280::Status st = device.setMode(BME280::Mode::SLEEP);
    if (st.ok() && restoreMode != BME280::Mode::SLEEP) {
      (void)device.setMode(restoreMode);
    }
  }
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

BME280::Status scheduleMeasurement() {
  BME280::Status st = device.requestMeasurement();
  if (st.code == BME280::Err::IN_PROGRESS) {
    gPendingRead = true;
    gPendingStartMs = currentMs();
    if (gVerbose && !gStress.active) {
      LOGI("Measurement requested at %lu ms", static_cast<unsigned long>(gPendingStartMs));
    }
  } else if (st.code == BME280::Err::BUSY && gPendingRead) {
    st = BME280::Status::Error(BME280::Err::IN_PROGRESS, "Measurement already in progress");
  }
  return st;
}

BME280::Status performMeasurementBlocking(BME280::Measurement& out, uint32_t timeoutMs = 500U) {
  BME280::Status st = ensureForcedMeasurementMode();
  if (!st.ok()) {
    return st;
  }
  st = device.requestMeasurement();
  if (st.code != BME280::Err::IN_PROGRESS) {
    return st;
  }
  const uint32_t start = currentMs();
  while (!device.measurementReady()) {
    const uint32_t now = currentMs();
    device.tick(now);
    if (static_cast<uint32_t>(now - start) > timeoutMs) {
      return BME280::Status::Error(BME280::Err::TIMEOUT, "Measurement wait timeout");
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  return device.getMeasurement(out);
}

void resetStressStats(int target) {
  gStress = StressStats{};
  gStress.active = true;
  gStress.startMs = currentMs();
  gStress.target = target;
}

void noteStressError(const BME280::Status& st) {
  ++gStress.errors;
  gStress.lastError = st;
  if (!gStress.hasFailure) {
    gStress.hasFailure = true;
    gStress.firstError = st;
  }
}

void updateStressStats(const BME280::Measurement& sample) {
  if (!gStress.hasSample) {
    gStress.minTemp = sample.temperatureC;
    gStress.maxTemp = sample.temperatureC;
    gStress.minPressure = sample.pressurePa;
    gStress.maxPressure = sample.pressurePa;
    gStress.minHumidity = sample.humidityPct;
    gStress.maxHumidity = sample.humidityPct;
    gStress.hasSample = true;
  } else {
    if (sample.temperatureC < gStress.minTemp) gStress.minTemp = sample.temperatureC;
    if (sample.temperatureC > gStress.maxTemp) gStress.maxTemp = sample.temperatureC;
    if (sample.pressurePa < gStress.minPressure) gStress.minPressure = sample.pressurePa;
    if (sample.pressurePa > gStress.maxPressure) gStress.maxPressure = sample.pressurePa;
    if (sample.humidityPct < gStress.minHumidity) gStress.minHumidity = sample.humidityPct;
    if (sample.humidityPct > gStress.maxHumidity) gStress.maxHumidity = sample.humidityPct;
  }
  ++gStress.success;
  gStress.sumTemp += sample.temperatureC;
  gStress.sumPressure += sample.pressurePa;
  gStress.sumHumidity += sample.humidityPct;
}

void finishStressStats() {
  const uint32_t elapsed = currentMs() - gStress.startMs;
  const float successRate =
      (gStress.attempts > 0) ? (100.0f * static_cast<float>(gStress.success) / gStress.attempts) : 0.0f;
  std::printf("=== Stress Summary ===\n");
  std::printf("  Attempts: %d\n", gStress.attempts);
  std::printf("  Success: %s%d%s\n", nonZeroGoodColor(gStress.success), gStress.success, LOG_COLOR_RESET);
  std::printf("  Errors: %s%lu%s\n", zeroGoodColor(gStress.errors), static_cast<unsigned long>(gStress.errors), LOG_COLOR_RESET);
  std::printf("  Success rate: %s%.1f%%%s\n", successRateColor(successRate), successRate, LOG_COLOR_RESET);
  std::printf("  Elapsed: %lu ms\n", static_cast<unsigned long>(elapsed));
  if (gStress.hasSample && gStress.success > 0) {
    const double denom = static_cast<double>(gStress.success);
    std::printf("  Temp C: min=%.2f avg=%.2f max=%.2f\n",
                gStress.minTemp,
                gStress.sumTemp / denom,
                gStress.maxTemp);
    std::printf("  Pressure Pa: min=%.2f avg=%.2f max=%.2f\n",
                gStress.minPressure,
                gStress.sumPressure / denom,
                gStress.maxPressure);
    std::printf("  Humidity %%: min=%.2f avg=%.2f max=%.2f\n",
                gStress.minHumidity,
                gStress.sumHumidity / denom,
                gStress.maxHumidity);
  }
  if (gStress.hasFailure) {
    std::printf("  First error: %s\n", errToStr(gStress.firstError.code));
    std::printf("  Last error: %s\n", errToStr(gStress.lastError.code));
  }
  gStress.active = false;
}

void handleMeasurementReady() {
  if (!gPendingRead || !device.measurementReady()) {
    return;
  }
  BME280::Measurement sample;
  const BME280::Status st = device.getMeasurement(sample);
  gPendingRead = false;
  if (!st.ok()) {
    if (gStress.active) {
      noteStressError(st);
      ++gStress.attempts;
      --gStressRemaining;
      if (gStressRemaining == 0) {
        finishStressStats();
      }
    } else {
      printStatus(st);
    }
    return;
  }
  if (gStress.active) {
    updateStressStats(sample);
    ++gStress.attempts;
    --gStressRemaining;
    if (gStressRemaining == 0) {
      finishStressStats();
    }
    return;
  }
  printMeasurement(sample);
}

void runStressMix(int count) {
  int success = 0;
  int failures = 0;
  LOGI("Starting mixed stress: %d cycles", count);
  for (int i = 0; i < count; ++i) {
    BME280::Status st;
    switch (i % 4) {
      case 0: {
        BME280::Measurement sample;
        st = performMeasurementBlocking(sample);
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
        break;
      }
      default: {
        BME280::RawSample raw;
        st = device.getRawSample(raw);
        if (st.code == BME280::Err::MEASUREMENT_NOT_READY) {
          st = BME280::Status::Ok();
        }
        break;
      }
    }
    if (st.ok()) {
      ++success;
    } else {
      ++failures;
      if (gVerbose) {
        printStatus(st);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  std::printf("Stress mix result: ok=%s%d%s fail=%s%d%s\n",
              nonZeroGoodColor(success),
              success,
              LOG_COLOR_RESET,
              zeroGoodColor(static_cast<uint32_t>(failures)),
              failures,
              LOG_COLOR_RESET);
}

void reportCheck(const char* label, bool pass, const char* detail = "") {
  std::printf("  [%s%s%s] %s",
              pass ? LOG_COLOR_GREEN : LOG_COLOR_RED,
              pass ? "PASS" : "FAIL",
              LOG_COLOR_RESET,
              label);
  if (detail != nullptr && detail[0] != '\0') {
    std::printf(" - %s", detail);
  }
  std::printf("\n");
}

void runSelfTest() {
  std::printf("=== BME280 Selftest ===\n");
  uint8_t id = 0;
  BME280::Status st = device.readChipId(id);
  reportCheck("readChipId", st.ok(), st.ok() ? "" : errToStr(st.code));
  reportCheck("chip ID is BME280", st.ok() && id == BME280::cmd::CHIP_ID_BME280);
  BME280::Calibration calib;
  st = device.getCalibration(calib);
  reportCheck("getCalibration", st.ok(), st.ok() ? "" : errToStr(st.code));
  BME280::Measurement sample;
  st = performMeasurementBlocking(sample);
  reportCheck("measurement cycle", st.ok(), st.ok() ? "" : errToStr(st.code));
  const bool plausible = st.ok() && sample.temperatureC > -60.0f && sample.temperatureC < 130.0f &&
                         sample.humidityPct >= 0.0f && sample.humidityPct <= 100.0f &&
                         sample.pressurePa > 20000.0f && sample.pressurePa < 130000.0f;
  reportCheck("measurement in plausible range", plausible);
  bool measuring = false;
  st = device.isMeasuring(measuring);
  reportCheck("isMeasuring", st.ok(), st.ok() ? "" : errToStr(st.code));
  st = device.recover();
  reportCheck("recover", st.ok(), st.ok() ? "" : errToStr(st.code));
  reportCheck("isOnline", device.isOnline());
}

void printHelp() {
  std::printf("\n%s=== BME280 CLI Help ===%s\n", LOG_COLOR_CYAN, LOG_COLOR_RESET);
  std::printf("\n%s[Common]%s\n", LOG_COLOR_GREEN, LOG_COLOR_RESET);
  printHelpItem("help / ?", "Show this help");
  printHelpItem("version / ver", "Print firmware and library version info");
  printHelpItem("scan", "Scan I2C bus");
  printHelpItem("begin", "Run begin() with the default example config");
  printHelpItem("read", "Request and display measurement");
  printHelpItem("raw", "Show last raw ADC sample");
  printHelpItem("comp", "Show last compensated sample");
  printHelpItem("data", "Burst-read and decode live data registers");
  printHelpItem("measuring", "Show measuring flag");
  printHelpItem("timing", "Show measurement and cycle timing estimates");
  std::printf("\n%s[Configuration]%s\n", LOG_COLOR_GREEN, LOG_COLOR_RESET);
  printHelpItem("mode [sleep|forced|normal]", "Set or show operating mode");
  printHelpItem("osrs [t|p|h <0..5>]", "Set or show oversampling");
  printHelpItem("filter [0..4]", "Set or show IIR filter");
  printHelpItem("standby [0..7]", "Set or show standby time");
  printHelpItem("cfg / settings", "Show chip and internal settings");
  printHelpItem("calib [raw]", "Show cached or raw calibration");
  printHelpItem("status", "Read status register");
  printHelpItem("chipid", "Read chip ID");
  printHelpItem("reset", "Soft reset device");
  std::printf("\n%s[Registers]%s\n", LOG_COLOR_GREEN, LOG_COLOR_RESET);
  printHelpItem("reg <addr>", "Read 8-bit register (hex address)");
  printHelpItem("wreg <addr> <val>", "Write 8-bit register (diagnostic only; may desync cached config)");
  std::printf("\n%s[Diagnostics]%s\n", LOG_COLOR_GREEN, LOG_COLOR_RESET);
  printHelpItem("drv", "Show driver state and health");
  printHelpItem("state", "Show compact one-line health summary");
  printHelpItem("probe", "Probe device (no health tracking)");
  printHelpItem("recover", "Manual recovery attempt");
  printHelpItem("verbose [0|1]", "Enable/disable verbose output");
  printHelpItem("stress [N]", "Run N measurement cycles");
  printHelpItem("stress_mix [N]", "Run N mixed-operation cycles");
  printHelpItem("selftest", "Run safe command self-test report");
}

void printVersionInfo() {
  std::printf("=== Version Info ===\n");
  std::printf("  Example firmware build: %s %s\n", __DATE__, __TIME__);
  std::printf("  BME280 library version: %s\n", BME280::VERSION);
  std::printf("  BME280 library full: %s\n", BME280::VERSION_FULL);
  std::printf("  BME280 library build: %s\n", BME280::BUILD_TIMESTAMP);
  std::printf("  BME280 library commit: %s (%s)\n", BME280::GIT_COMMIT, BME280::GIT_STATUS);
}

void processCommand(char* line) {
  char* cursor = trim(line);
  char* head = nextToken(cursor);
  if (head == nullptr) {
    return;
  }

  if (std::strcmp(head, "help") == 0 || std::strcmp(head, "?") == 0) {
    printHelp();
  } else if (std::strcmp(head, "version") == 0 || std::strcmp(head, "ver") == 0) {
    printVersionInfo();
  } else if (std::strcmp(head, "scan") == 0) {
    scanBus();
  } else if (std::strcmp(head, "begin") == 0) {
    LOGI("Initializing BME280...");
    cancelPending();
    device.end();
    const BME280::Status st = device.begin(makeDefaultConfig());
    printStatus(st);
    if (st.ok()) {
      printDriverHealth();
    }
  } else if (std::strcmp(head, "read") == 0) {
    cancelPending();
    const BME280::Status st = scheduleMeasurement();
    if (st.code != BME280::Err::IN_PROGRESS) {
      printStatus(st);
    }
  } else if (std::strcmp(head, "raw") == 0) {
    printRawSample();
  } else if (std::strcmp(head, "comp") == 0) {
    printCompensatedSample();
  } else if (std::strcmp(head, "data") == 0) {
    printDataRegisters();
  } else if (std::strcmp(head, "measuring") == 0) {
    bool measuring = false;
    const BME280::Status st = device.isMeasuring(measuring);
    if (st.ok()) {
      std::printf("Measuring: %s\n", measuring ? "YES" : "NO");
    } else {
      printStatus(st);
    }
  } else if (std::strcmp(head, "timing") == 0) {
    printTimingInfo();
  } else if (std::strcmp(head, "settings") == 0 || std::strcmp(head, "cfg") == 0) {
    printAllSettings();
  } else if (std::strcmp(head, "calib") == 0) {
    char* arg = nextToken(cursor);
    if (arg != nullptr && std::strcmp(arg, "raw") == 0) {
      printCalibrationRaw();
    } else {
      printCalibration();
    }
  } else if (std::strcmp(head, "mode") == 0) {
    char* arg = nextToken(cursor);
    if (arg == nullptr) {
      BME280::Mode mode = BME280::Mode::SLEEP;
      const BME280::Status st = device.getMode(mode);
      if (st.ok()) {
        std::printf("Mode: %s\n", modeToStr(mode));
      } else {
        printStatus(st);
      }
      return;
    }
    BME280::Mode mode = BME280::Mode::SLEEP;
    if (std::strcmp(arg, "sleep") == 0) mode = BME280::Mode::SLEEP;
    else if (std::strcmp(arg, "forced") == 0) mode = BME280::Mode::FORCED;
    else if (std::strcmp(arg, "normal") == 0) mode = BME280::Mode::NORMAL;
    else {
      LOGW("Invalid mode: %s", arg);
      return;
    }
    cancelPending();
    printStatus(device.setMode(mode));
  } else if (std::strcmp(head, "osrs") == 0) {
    char* which = nextToken(cursor);
    if (which == nullptr) {
      BME280::SettingsSnapshot snap;
      (void)device.getSettings(snap);
      std::printf("Oversampling: t=%s p=%s h=%s\n",
                  osrsToStr(static_cast<uint8_t>(snap.osrsT)),
                  osrsToStr(static_cast<uint8_t>(snap.osrsP)),
                  osrsToStr(static_cast<uint8_t>(snap.osrsH)));
      return;
    }
    char* valueTok = nextToken(cursor);
    uint32_t value = 0;
    if (valueTok == nullptr || !parseU32(valueTok, value) || value > 5U) {
      LOGW("Usage: osrs t|p|h <0..5>");
      return;
    }
    const auto osrs = static_cast<BME280::Oversampling>(value);
    BME280::Status st;
    if (std::strcmp(which, "t") == 0) st = device.setOversamplingT(osrs);
    else if (std::strcmp(which, "p") == 0) st = device.setOversamplingP(osrs);
    else if (std::strcmp(which, "h") == 0) st = device.setOversamplingH(osrs);
    else {
      LOGW("Usage: osrs t|p|h <0..5>");
      return;
    }
    printStatus(st);
  } else if (std::strcmp(head, "filter") == 0) {
    char* arg = nextToken(cursor);
    if (arg == nullptr) {
      BME280::Filter filter = BME280::Filter::OFF;
      const BME280::Status st = device.getFilter(filter);
      if (st.ok()) std::printf("Filter: %s\n", filterToStr(static_cast<uint8_t>(filter)));
      else printStatus(st);
      return;
    }
    uint32_t value = 0;
    if (!parseU32(arg, value) || value > 4U) {
      LOGW("Invalid filter value");
      return;
    }
    printStatus(device.setFilter(static_cast<BME280::Filter>(value)));
  } else if (std::strcmp(head, "standby") == 0) {
    char* arg = nextToken(cursor);
    if (arg == nullptr) {
      BME280::Standby standby = BME280::Standby::MS_0_5;
      const BME280::Status st = device.getStandby(standby);
      if (st.ok()) std::printf("Standby: %s\n", standbyToStr(static_cast<uint8_t>(standby)));
      else printStatus(st);
      return;
    }
    uint32_t value = 0;
    if (!parseU32(arg, value) || value > 7U) {
      LOGW("Invalid standby value");
      return;
    }
    printStatus(device.setStandby(static_cast<BME280::Standby>(value)));
  } else if (std::strcmp(head, "status") == 0) {
    uint8_t status = 0;
    const BME280::Status st = device.readStatus(status);
    if (!st.ok()) {
      printStatus(st);
      return;
    }
    std::printf("Status: 0x%02X (measuring=%d, im_update=%d)\n",
                status,
                (status & BME280::cmd::MASK_STATUS_MEASURING) != 0 ? 1 : 0,
                (status & BME280::cmd::MASK_STATUS_IM_UPDATE) != 0 ? 1 : 0);
  } else if (std::strcmp(head, "chipid") == 0) {
    uint8_t id = 0;
    const BME280::Status st = device.readChipId(id);
    if (st.ok()) std::printf("Chip ID: 0x%02X\n", id);
    else printStatus(st);
  } else if (std::strcmp(head, "reset") == 0) {
    cancelPending();
    printStatus(device.softReset());
  } else if (std::strcmp(head, "reg") == 0) {
    uint32_t addr = 0;
    if (!parseU32(nextToken(cursor), addr) || addr > 0xFFU) {
      LOGW("Usage: reg <addr>");
      return;
    }
    uint8_t value = 0;
    const BME280::Status st = device.readRegister(static_cast<uint8_t>(addr), value);
    if (st.ok()) std::printf("Reg 0x%02lX = 0x%02X (%u)\n", static_cast<unsigned long>(addr), value, value);
    else printStatus(st);
  } else if (std::strcmp(head, "wreg") == 0) {
    uint32_t addr = 0;
    uint32_t value = 0;
    if (!parseU32(nextToken(cursor), addr) || !parseU32(nextToken(cursor), value) ||
        addr > 0xFFU || value > 0xFFU) {
      LOGW("Usage: wreg <addr> <val>");
      return;
    }
    printStatus(device.writeRegister(static_cast<uint8_t>(addr), static_cast<uint8_t>(value)));
  } else if (std::strcmp(head, "drv") == 0) {
    printDriverHealth();
    BME280::Mode mode;
    if (device.getMode(mode).ok()) {
      std::printf("  Mode: %s\n", modeToStr(mode));
    }
  } else if (std::strcmp(head, "state") == 0) {
    printCompactHealth();
  } else if (std::strcmp(head, "probe") == 0) {
    LOGI("Probing device (no health tracking)...");
    HealthSnapshot before;
    before.capture(device);
    const BME280::Status st = device.probe();
    printStatus(st);
    HealthSnapshot after;
    after.capture(device);
    std::printf("  Health changes:\n");
    printHealthDiff(before, after);
  } else if (std::strcmp(head, "recover") == 0) {
    LOGI("Attempting recovery...");
    HealthSnapshot before;
    before.capture(device);
    const BME280::Status st = device.recover();
    printStatus(st);
    HealthSnapshot after;
    after.capture(device);
    std::printf("  Health changes:\n");
    printHealthDiff(before, after);
    printDriverHealth();
  } else if (std::strcmp(head, "verbose") == 0) {
    char* arg = nextToken(cursor);
    if (arg != nullptr) {
      uint32_t value = 0;
      if (!parseU32(arg, value)) {
        LOGW("Usage: verbose [0|1]");
        return;
      }
      gVerbose = value != 0U;
    }
    LOGI("Verbose mode: %s", gVerbose ? "ON" : "OFF");
  } else if (std::strcmp(head, "selftest") == 0) {
    runSelfTest();
  } else if (std::strcmp(head, "stress_mix") == 0) {
    uint32_t count = DEFAULT_STRESS_MIX_COUNT;
    char* arg = nextToken(cursor);
    if (arg != nullptr && (!parseU32(arg, count) || count == 0U || count > MAX_STRESS_COUNT)) {
      LOGW("Invalid stress_mix count");
      return;
    }
    runStressMix(static_cast<int>(count));
  } else if (std::strcmp(head, "stress") == 0) {
    uint32_t count = DEFAULT_STRESS_COUNT;
    char* arg = nextToken(cursor);
    if (arg != nullptr && (!parseU32(arg, count) || count == 0U || count > MAX_STRESS_COUNT)) {
      LOGW("Invalid stress count");
      return;
    }
    cancelPending();
    const BME280::Status st = ensureForcedMeasurementMode();
    if (!st.ok()) {
      printStatus(st);
      return;
    }
    gStressRemaining = static_cast<int>(count);
    resetStressStats(static_cast<int>(count));
    LOGI("Starting stress test: %lu cycles", static_cast<unsigned long>(count));
  } else {
    LOGW("Unknown command: %s", head);
  }
}

void inputTask(void* arg) {
  QueueHandle_t queue = static_cast<QueueHandle_t>(arg);
  char buffer[CLI_LINE_LEN] = {};
  while (true) {
    if (std::fgets(buffer, sizeof(buffer), stdin) == nullptr) {
      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }
    size_t len = std::strlen(buffer);
    while (len > 0U && (buffer[len - 1U] == '\n' || buffer[len - 1U] == '\r')) {
      buffer[--len] = '\0';
    }
    if (len == 0U) {
      continue;
    }
    CliLine line{};
    std::strncpy(line.text, buffer, sizeof(line.text) - 1U);
    (void)xQueueSend(queue, &line, portMAX_DELAY);
  }
}

void tickApp() {
  device.tick(currentMs());
  if (gStress.active && gStressRemaining > 0 && !gPendingRead) {
    const BME280::Status st = scheduleMeasurement();
    if (st.code != BME280::Err::IN_PROGRESS) {
      noteStressError(st);
      ++gStress.attempts;
      --gStressRemaining;
      if (gStressRemaining == 0) {
        finishStressStats();
      }
    }
  }
  handleMeasurementReady();
}

}  // namespace

extern "C" void app_main(void) {
  LOGI("=== BME280 Bringup Example ===");

  if (!bme280IdfInitI2c(BME280_IDF_I2C_SDA,
                        BME280_IDF_I2C_SCL,
                        BME280_IDF_I2C_FREQ_HZ,
                        I2C_TIMEOUT_MS,
                        BME280_ADDR)) {
    LOGE("Failed to initialize I2C: %s", esp_err_to_name(bme280IdfI2cContext().lastError));
    return;
  }

  LOGI("I2C initialized (SDA=%d, SCL=%d)", BME280_IDF_I2C_SDA, BME280_IDF_I2C_SCL);
  scanBus();

  BME280::Status st = device.begin(makeDefaultConfig());
  if (!st.ok()) {
    LOGE("Failed to initialize device");
    printStatus(st);
  } else {
    LOGI("Device initialized successfully");
    printDriverHealth();
  }

  gLineQueue = xQueueCreate(CLI_QUEUE_DEPTH, sizeof(CliLine));
  if (gLineQueue == nullptr) {
    LOGE("Failed to create CLI input queue");
    bme280IdfDeinitI2c();
    return;
  }
  (void)xTaskCreate(inputTask, "bme280_cli_input", 4096, gLineQueue, 5, nullptr);

  printHelp();
  printPrompt();

  while (true) {
    tickApp();

    CliLine line{};
    while (xQueueReceive(gLineQueue, &line, 0) == pdTRUE) {
      processCommand(line.text);
      printPrompt();
    }

    vTaskDelay(pdMS_TO_TICKS(CLI_TICK_MS));
  }
}
