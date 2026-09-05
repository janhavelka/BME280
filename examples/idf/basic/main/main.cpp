/// @file main.cpp
/// @brief ESP-IDF native bringup CLI example for BME280.
/// @note This is an EXAMPLE, not part of the library.

#include <cstdint>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>

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
constexpr char LOG_COLOR_CYAN[] = "\033[36m";
constexpr size_t HELP_COMMAND_WIDTH = 32U;
constexpr uint8_t BME280_DEFAULT_ADDR = 0x76;
constexpr uint32_t I2C_TIMEOUT_MS = 50;
constexpr uint32_t CLI_TICK_MS = 5;
constexpr size_t CLI_INPUT_MAX_LEN = 127U;
constexpr size_t CLI_LINE_LEN = CLI_INPUT_MAX_LEN + 2U;
static_assert(CLI_LINE_LEN == 129U, "CLI line buffer contract changed");
constexpr int CLI_QUEUE_DEPTH = 4;
constexpr uint32_t DEFAULT_STRESS_COUNT = 10U;
constexpr uint32_t DEFAULT_STRESS_MIX_COUNT = 50U;
constexpr uint32_t MAX_STRESS_COUNT = 100000U;
constexpr uint32_t STRESS_PROGRESS_UPDATES = 10U;
constexpr uint8_t JOB_CLI_DEFAULT_BUDGET = 1U;
constexpr uint8_t JOB_CLI_MAX_BUDGET = 8U;
constexpr uint16_t JOB_CLI_MAX_POLLS = 1024U;
constexpr uint32_t JOB_CLI_POLL_DELAY_MS = 1U;

#define LOG_COLOR_RESULT(ok) ((ok) ? LOG_COLOR_GREEN : LOG_COLOR_RED)
#define LOG_PRINT(tagColor, tag, fmt, ...) \
  do { \
    std::printf("%s[%s]%s " fmt "\n", tagColor, tag, LOG_COLOR_RESET, ##__VA_ARGS__); \
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
  bool hardwareConfigDirty = false;

  void capture(const BME280::BME280& driver) {
    state = driver.state();
    online = driver.isOnline();
    consecutiveFailures = driver.consecutiveFailures();
    totalFailures = driver.totalFailures();
    totalSuccess = driver.totalSuccess();
    hardwareConfigDirty = driver.hardwareConfigDirty();
  }
};

struct ChipSettings {
  uint8_t ctrlHum = 0;
  uint8_t ctrlMeas = 0;
  uint8_t config = 0;
  uint8_t osrsT = 0;
  uint8_t osrsP = 0;
  uint8_t osrsH = 0;
  uint8_t modeBits = 0;
  uint8_t filter = 0;
  uint8_t standby = 0;
  bool spi3wEnabled = false;
};

struct InternalSettings {
  BME280::Mode mode = BME280::Mode::SLEEP;
  BME280::Oversampling osrsT = BME280::Oversampling::X1;
  BME280::Oversampling osrsP = BME280::Oversampling::X1;
  BME280::Oversampling osrsH = BME280::Oversampling::X1;
  BME280::Filter filter = BME280::Filter::OFF;
  BME280::Standby standby = BME280::Standby::MS_0_5;
};

struct StressStats {
  bool active = false;
  uint32_t startMs = 0;
  int target = 0;
  int attempts = 0;
  int success = 0;
  uint32_t successBefore = 0;
  uint32_t failBefore = 0;
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
uint8_t gActiveAddress = BME280_DEFAULT_ADDR;

void clearPendingBookkeeping();
BME280::Status cancelPending();
bool cancelPendingForCommand();

uint32_t nowMs(void*) {
  return static_cast<uint32_t>(esp_timer_get_time() / 1000LL);
}

uint32_t currentMs() {
  return nowMs(nullptr);
}

TickType_t delayTicksAtLeastOne(uint32_t delayMs) {
  const TickType_t ticks = pdMS_TO_TICKS(delayMs);
  return ticks > 0 ? ticks : 1;
}

const char* boolStr(bool value) {
  return value ? "true" : "false";
}

const char* errToStr(BME280::Err err) {
  return BME280::toString(err);
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

uint32_t stressProgressStep(uint32_t total) {
  if (total == 0U) {
    return 0U;
  }
  const uint32_t step = total / STRESS_PROGRESS_UPDATES;
  return (step == 0U) ? 1U : step;
}

void printStressProgress(uint32_t completed, uint32_t total,
                         uint32_t okCount, uint32_t failCount) {
  if (completed == 0U || total == 0U) {
    return;
  }
  const uint32_t step = stressProgressStep(total);
  if (step == 0U || (completed != total && (completed % step) != 0U)) {
    return;
  }
  const float pct =
      (100.0f * static_cast<float>(completed)) / static_cast<float>(total);
  std::printf("  Progress: %lu/%lu (%.0f%%, ok=%s%lu%s, fail=%s%lu%s)\n",
              static_cast<unsigned long>(completed),
              static_cast<unsigned long>(total),
              pct,
              nonZeroGoodColor(okCount),
              static_cast<unsigned long>(okCount),
              LOG_COLOR_RESET,
              zeroGoodColor(failCount),
              static_cast<unsigned long>(failCount),
              LOG_COLOR_RESET);
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
  if (token == nullptr || *token == '\0' || *token == '-' || *token == '+') {
    return false;
  }
  errno = 0;
  char* end = nullptr;
  const unsigned long value = std::strtoul(token, &end, 0);
  if (errno == ERANGE || end == token || *end != '\0' ||
      value > static_cast<unsigned long>(std::numeric_limits<uint32_t>::max())) {
    return false;
  }
  out = static_cast<uint32_t>(value);
  return true;
}

const char* modeBitsToStr(uint8_t modeBits) {
  switch (modeBits) {
    case 0: return "SLEEP";
    case 1:
    case 2: return "FORCED";
    case 3: return "NORMAL";
    default: return "UNKNOWN";
  }
}

bool requireNoArguments(char*& cursor, const char* usage) {
  if (nextToken(cursor) == nullptr) {
    return true;
  }
  LOGW("Usage: %s", usage);
  return false;
}

bool parseMode(const char* token, BME280::Mode& out) {
  if (token == nullptr) return false;
  if (std::strcmp(token, "sleep") == 0 || std::strcmp(token, "0") == 0) {
    out = BME280::Mode::SLEEP;
  } else if (std::strcmp(token, "forced") == 0 || std::strcmp(token, "1") == 0) {
    out = BME280::Mode::FORCED;
  } else if (std::strcmp(token, "normal") == 0 || std::strcmp(token, "3") == 0) {
    out = BME280::Mode::NORMAL;
  } else {
    return false;
  }
  return true;
}

bool parseOversampling(const char* token, BME280::Oversampling& out) {
  if (token == nullptr) return false;
  if (std::strcmp(token, "skip") == 0) out = BME280::Oversampling::SKIP;
  else if (std::strcmp(token, "x1") == 0) out = BME280::Oversampling::X1;
  else if (std::strcmp(token, "x2") == 0) out = BME280::Oversampling::X2;
  else if (std::strcmp(token, "x4") == 0) out = BME280::Oversampling::X4;
  else if (std::strcmp(token, "x8") == 0) out = BME280::Oversampling::X8;
  else if (std::strcmp(token, "x16") == 0) out = BME280::Oversampling::X16;
  else {
    uint32_t value = 0;
    if (!parseU32(token, value) || value > 5U) return false;
    out = static_cast<BME280::Oversampling>(value);
  }
  return true;
}

bool parseFilter(const char* token, BME280::Filter& out) {
  if (token == nullptr) return false;
  if (std::strcmp(token, "off") == 0) out = BME280::Filter::OFF;
  else if (std::strcmp(token, "x2") == 0) out = BME280::Filter::X2;
  else if (std::strcmp(token, "x4") == 0) out = BME280::Filter::X4;
  else if (std::strcmp(token, "x8") == 0) out = BME280::Filter::X8;
  else if (std::strcmp(token, "x16") == 0) out = BME280::Filter::X16;
  else {
    uint32_t value = 0;
    if (!parseU32(token, value) || value > 4U) return false;
    out = static_cast<BME280::Filter>(value);
  }
  return true;
}

bool parseStandby(const char* token, BME280::Standby& out) {
  if (token == nullptr) return false;
  if (std::strcmp(token, "ms_0_5") == 0 || std::strcmp(token, "0.5ms") == 0) out = BME280::Standby::MS_0_5;
  else if (std::strcmp(token, "ms_62_5") == 0 || std::strcmp(token, "62.5ms") == 0) out = BME280::Standby::MS_62_5;
  else if (std::strcmp(token, "ms_125") == 0 || std::strcmp(token, "125ms") == 0) out = BME280::Standby::MS_125;
  else if (std::strcmp(token, "ms_250") == 0 || std::strcmp(token, "250ms") == 0) out = BME280::Standby::MS_250;
  else if (std::strcmp(token, "ms_500") == 0 || std::strcmp(token, "500ms") == 0) out = BME280::Standby::MS_500;
  else if (std::strcmp(token, "ms_1000") == 0 || std::strcmp(token, "1000ms") == 0) out = BME280::Standby::MS_1000;
  else if (std::strcmp(token, "ms_10") == 0 || std::strcmp(token, "10ms") == 0) out = BME280::Standby::MS_10;
  else if (std::strcmp(token, "ms_20") == 0 || std::strcmp(token, "20ms") == 0) out = BME280::Standby::MS_20;
  else {
    uint32_t value = 0;
    if (!parseU32(token, value) || value > 7U) return false;
    out = static_cast<BME280::Standby>(value);
  }
  return true;
}

BME280::Config makeDefaultConfig() {
  BME280::Config cfg;
  cfg.i2cWrite = idfI2cWrite;
  cfg.i2cWriteRead = idfI2cWriteRead;
  cfg.i2cUser = &bme280IdfI2cContext();
  cfg.i2cAddress = gActiveAddress;
  cfg.i2cTimeoutMs = I2C_TIMEOUT_MS;
  cfg.nowMs = nowMs;
  cfg.offlineThreshold = 5;
  return cfg;
}

bool parseI2cAddress(const char* token, uint8_t& out) {
  uint32_t value = 0;
  if (!parseU32(token, value) || (value != 0x76U && value != 0x77U)) {
    return false;
  }
  out = static_cast<uint8_t>(value);
  return true;
}

bool parseJobBudget(const char* token, uint8_t& out, bool allowZero) {
  uint32_t value = 0;
  const uint32_t minimum = allowZero ? 0U : 1U;
  if (!parseU32(token, value) || value < minimum || value > JOB_CLI_MAX_BUDGET) {
    return false;
  }
  out = static_cast<uint8_t>(value);
  return true;
}

BME280::Status initBusForActiveAddress() {
  if (!bme280IdfInitI2c(BME280_IDF_I2C_SDA,
                        BME280_IDF_I2C_SCL,
                        BME280_IDF_I2C_FREQ_HZ,
                        gActiveAddress)) {
    return BME280::Status::Error(BME280::Err::I2C_BUS,
                                 static_cast<int32_t>(bme280IdfI2cContext().lastError));
  }
  return BME280::Status::Ok();
}

BME280::Status beginAtActiveAddress() {
  BME280::Status st = initBusForActiveAddress();
  if (!st.ok()) {
    return st;
  }
  return device.begin(makeDefaultConfig());
}

void printActiveAddress() {
  std::printf("Active I2C address: 0x%02X (%s)\n",
              static_cast<unsigned>(gActiveAddress),
              gActiveAddress == 0x76 ? "SDO=GND" : "SDO=VDDIO");
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

void printDirtyState() {
  const BME280::Status dirty = device.hardwareConfigDirtyError();
  std::printf("  Hardware config dirty: %s%s%s\n",
              device.hardwareConfigDirty() ? LOG_COLOR_YELLOW : LOG_COLOR_GREEN,
              boolStr(device.hardwareConfigDirty()),
              LOG_COLOR_RESET);
  if (device.hardwareConfigDirty() && !dirty.ok()) {
    std::printf("  Dirty cause: %s (detail=%ld)\n",
                errToStr(dirty.code),
                static_cast<long>(dirty.detail));
  }
}

void printValidity(bool temperatureValid, bool pressureValid, bool humidityValid) {
  std::printf("  Valid channels: T=%d P=%d H=%d\n",
              temperatureValid ? 1 : 0,
              pressureValid ? 1 : 0,
              humidityValid ? 1 : 0);
}

void printSampleAge() {
  if (!device.hasSample()) {
    std::printf("  Cached sample: none\n");
    return;
  }
  std::printf("  Cached sample age: %lu ms (timestamp=%lu ms)\n",
              static_cast<unsigned long>(device.sampleAgeMs(currentMs())),
              static_cast<unsigned long>(device.sampleTimestampMs()));
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
  if (before.hardwareConfigDirty != after.hardwareConfigDirty) {
    std::printf("  Dirty: %s -> %s\n",
                boolStr(before.hardwareConfigDirty),
                boolStr(after.hardwareConfigDirty));
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
  std::printf("Health: state=%s online=%s dirty=%s consec=%u ok=%lu fail=%lu rate=%.1f%%\n",
              stateToStr(device.state()),
              boolStr(device.isOnline()),
              boolStr(device.hardwareConfigDirty()),
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
  std::printf("  Active I2C address: 0x%02X\n", static_cast<unsigned>(gActiveAddress));
  printDirtyState();
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

void printJobUsage() {
  LOGW("Usage: job status | job start <init|force|apply|resync|reset|recover> | job cancel <owner|deadline> | job poll [0..8] | job <init|force|apply|resync|reset|recover> [1..8]");
}

bool isTerminalJobState(BME280::JobState state) {
  return state == BME280::JobState::DONE ||
         state == BME280::JobState::FAILED ||
         state == BME280::JobState::CANCELLED ||
         state == BME280::JobState::TIMED_OUT;
}

BME280::JobPollResult makeJobBoundaryResult(const BME280::Status& status) {
  BME280::JobPollResult result;
  result.jobId = device.jobId();
  result.kind = device.jobKind();
  result.phase = device.jobPhase();
  result.state = device.jobState();
  result.status = status;
  result.conversionState = device.conversionState();
  return result;
}

void printJobResult(const BME280::JobPollResult& result, const char* boundary) {
  std::printf("=== Job Status ===\n");
  std::printf("Boundary: %s\n", boundary);
  std::printf("Job ID: %lu\n", static_cast<unsigned long>(result.jobId));
  std::printf("Job kind: %s\n", BME280::toString(result.kind));
  std::printf("Job phase: %s\n", BME280::toString(result.phase));
  std::printf("Job state: %s\n", BME280::toString(result.state));
  std::printf("Terminal state: %s\n", isTerminalJobState(result.state) ? "true" : "false");
  std::printf("Status: %s (code=%u, detail=%ld)\n",
              BME280::toString(result.status.code),
              static_cast<unsigned>(result.status.code),
              static_cast<long>(result.status.detail));
  std::printf("Conversion state: %s\n", BME280::toString(result.conversionState));
  std::printf("Phase deadline active: %s\n", result.phaseDeadlineActive ? "true" : "false");
  std::printf("Phase deadline ms: %lu\n", static_cast<unsigned long>(result.phaseDeadlineMs));
  std::printf("Callbacks used: %u\n", static_cast<unsigned>(result.callbacksUsed));
  std::printf("Instructions: %u\n", static_cast<unsigned>(result.instructionsUsed));
  std::printf("Driver: %s\n", stateToStr(device.state()));
  std::printf("Hardware config dirty: %s\n", boolStr(device.hardwareConfigDirty()));
  std::printf("Consecutive failures: %u\n", static_cast<unsigned>(device.consecutiveFailures()));
}

void printJobStatus() {
  printJobResult(makeJobBoundaryResult(device.jobStatus()), "SNAPSHOT");
}

BME280::Status startJobByName(const char* action) {
  if (std::strcmp(action, "init") == 0) {
    return device.startInitJob(makeDefaultConfig());
  }
  if (std::strcmp(action, "force") == 0) {
    return device.startForcedMeasurementJob();
  }
  if (std::strcmp(action, "apply") == 0) {
    return device.startApplyConfigJob();
  }
  if (std::strcmp(action, "resync") == 0) {
    return device.startResyncJob();
  }
  if (std::strcmp(action, "reset") == 0) {
    return device.startSoftResetJob();
  }
  if (std::strcmp(action, "recover") == 0) {
    return device.startRecoveryJob();
  }
  return BME280::Status::Error(BME280::Err::INVALID_PARAM);
}

void pollJobOnce(uint8_t budget) {
  const BME280::JobPollResult result = device.pollJob(currentMs(), budget);
  printJobResult(result, "POLL");
}

void startJobNonBlocking(const char* action) {
  const BME280::Status status = startJobByName(action);
  if (status.inProgress()) {
    clearPendingBookkeeping();
  }
  printJobResult(makeJobBoundaryResult(status), "START");
}

void cancelJobByName(const char* reason) {
  BME280::CancelReason cancelReason;
  if (std::strcmp(reason, "owner") == 0) {
    cancelReason = BME280::CancelReason::OWNER_REQUEST;
  } else if (std::strcmp(reason, "deadline") == 0) {
    cancelReason = BME280::CancelReason::DEADLINE_EXPIRED;
  } else {
    printJobUsage();
    return;
  }
  const BME280::Status status = device.cancelJob(cancelReason);
  printJobResult(makeJobBoundaryResult(status), "CANCEL");
}

void runStartedJobToTerminal(const BME280::Status& st, uint8_t budget) {
  if (!st.inProgress()) {
    printJobResult(makeJobBoundaryResult(st), "START");
    return;
  }

  BME280::JobPollResult result{};
  const TickType_t pollDelayTicks = delayTicksAtLeastOne(JOB_CLI_POLL_DELAY_MS);
  for (uint16_t poll = 0; poll < JOB_CLI_MAX_POLLS; ++poll) {
    result = device.pollJob(currentMs(), budget);
    if (isTerminalJobState(result.state)) {
      printJobResult(result, "POLL");
      return;
    }
    vTaskDelay(pollDelayTicks);
  }

  LOGW("Job poll limit reached");
  const BME280::Status cancelStatus = device.cancelJob(BME280::CancelReason::OWNER_REQUEST);
  if (!isTerminalJobState(device.jobState())) {
    printJobResult(makeJobBoundaryResult(cancelStatus), "CANCEL");
    return;
  }
  printJobResult(device.pollJob(currentMs(), 0U), "POLL");
}

void runJobToTerminal(const char* action, uint8_t budget) {
  if (!cancelPendingForCommand()) {
    return;
  }
  runStartedJobToTerminal(startJobByName(action), budget);
}

void handleJobCommand(char*& cursor) {
  char* action = nextToken(cursor);
  if (action == nullptr) {
    printJobUsage();
    return;
  }

  char* argument = nextToken(cursor);
  if (nextToken(cursor) != nullptr) {
    printJobUsage();
    return;
  }

  if (std::strcmp(action, "status") == 0) {
    if (argument != nullptr) {
      printJobUsage();
      return;
    }
    printJobStatus();
    return;
  }
  if (std::strcmp(action, "start") == 0) {
    if (argument == nullptr ||
        (std::strcmp(argument, "init") != 0 &&
         std::strcmp(argument, "force") != 0 &&
         std::strcmp(argument, "apply") != 0 &&
         std::strcmp(argument, "resync") != 0 &&
         std::strcmp(argument, "reset") != 0 &&
         std::strcmp(argument, "recover") != 0)) {
      printJobUsage();
      return;
    }
    startJobNonBlocking(argument);
    return;
  }
  if (std::strcmp(action, "cancel") == 0) {
    if (argument == nullptr) {
      printJobUsage();
      return;
    }
    cancelJobByName(argument);
    return;
  }
  if (std::strcmp(action, "poll") == 0) {
    uint8_t budget = JOB_CLI_DEFAULT_BUDGET;
    if (argument != nullptr && !parseJobBudget(argument, budget, true)) {
      printJobUsage();
      return;
    }
    pollJobOnce(budget);
    return;
  }
  if (std::strcmp(action, "init") == 0 ||
      std::strcmp(action, "force") == 0 ||
      std::strcmp(action, "apply") == 0 ||
      std::strcmp(action, "resync") == 0 ||
      std::strcmp(action, "reset") == 0 ||
      std::strcmp(action, "recover") == 0) {
    uint8_t budget = JOB_CLI_DEFAULT_BUDGET;
    if (argument != nullptr && !parseJobBudget(argument, budget, false)) {
      printJobUsage();
      return;
    }
    runJobToTerminal(action, budget);
    return;
  }
  printJobUsage();
}

void printMeasurement(const BME280::Measurement& sample) {
  std::printf("Temp: %.2f C, Pressure: %.2f Pa, Humidity: %.2f %%\n",
              sample.temperatureC,
              sample.pressurePa,
              sample.humidityPct);
  printValidity(sample.temperatureValid, sample.pressureValid, sample.humidityValid);
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
  std::printf("Compensated: T=%ld x0.01C, P=%lu Pa, RH=%.2f %%\n",
              static_cast<long>(sample.tempC_x100),
              static_cast<unsigned long>(sample.pressurePa),
              static_cast<float>(sample.humidityPct_x1024) / 1024.0f);
  printValidity(sample.temperatureValid, sample.pressureValid, sample.humidityValid);
  printSampleAge();
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
    std::printf("%02X%s", static_cast<unsigned>(data[i]), (i + 1U < sizeof(data)) ? " " : "");
  }
  std::printf("\n  Decoded raw ADC: P=%ld T=%ld H=%ld\n",
              static_cast<long>(adcP),
              static_cast<long>(adcT),
              static_cast<long>(adcH));
  std::printf("  Sentinel check: P_skip=%d T_skip=%d H_skip=%d\n",
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
  std::printf("  T1=%u T2=%d T3=%d\n",
              static_cast<unsigned>(calib.digT1),
              calib.digT2,
              calib.digT3);
  std::printf("  P1=%u P2=%d P3=%d P4=%d P5=%d P6=%d P7=%d P8=%d P9=%d\n",
              static_cast<unsigned>(calib.digP1),
              calib.digP2,
              calib.digP3,
              calib.digP4,
              calib.digP5,
              calib.digP6,
              calib.digP7,
              calib.digP8,
              calib.digP9);
  std::printf("  H1=%u H2=%d H3=%u H4=%d H5=%d H6=%d\n",
              static_cast<unsigned>(calib.digH1),
              calib.digH2,
              static_cast<unsigned>(calib.digH3),
              calib.digH4,
              calib.digH5,
              calib.digH6);
  const bool plausible = calib.digT1 != 0U && calib.digP1 != 0U &&
                         (calib.digH1 != 0U || calib.digH2 != 0 ||
                          calib.digH3 != 0U || calib.digH4 != 0 ||
                          calib.digH5 != 0 || calib.digH6 != 0);
  std::printf("  Plausibility: %s%s%s (T1/P1 nonzero, humidity coeffs not all zero)\n",
              plausible ? LOG_COLOR_GREEN : LOG_COLOR_RED,
              plausible ? "PASS" : "CHECK",
              LOG_COLOR_RESET);
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
    std::printf("%02X%s", static_cast<unsigned>(raw.tp[i]), (i + 1U < sizeof(raw.tp)) ? " " : "");
  }
  std::printf("\n  H1: %02X\n  H: ",
              static_cast<unsigned>(raw.tp[BME280::cmd::REG_CALIB_TP_LEN - 1U]));
  for (size_t i = 0; i < sizeof(raw.h); ++i) {
    std::printf("%02X%s", static_cast<unsigned>(raw.h[i]), (i + 1U < sizeof(raw.h)) ? " " : "");
  }
  std::printf("\n");
}

void printVerboseState() {
  std::printf("  Verbose: %s\n", gVerbose ? "ON" : "OFF");
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
  out.spi3wEnabled =
      (out.config & BME280::cmd::MASK_CONFIG_SPI3W_EN) != 0;
  return true;
}

bool readInternalSettings(InternalSettings& out) {
  BME280::SettingsSnapshot snapshot;
  const BME280::Status st = device.getSettings(snapshot);
  if (!st.ok()) {
    printStatus(st);
    return false;
  }
  out.mode = snapshot.mode;
  out.osrsT = snapshot.osrsT;
  out.osrsP = snapshot.osrsP;
  out.osrsH = snapshot.osrsH;
  out.filter = snapshot.filter;
  out.standby = snapshot.standby;
  return true;
}

void printChipSettings(const ChipSettings& chip) {
  std::printf("=== Chip Settings ===\n");
  std::printf("  ctrl_hum: 0x%02X (osrs_h=%u %s)\n",
              static_cast<unsigned>(chip.ctrlHum),
              static_cast<unsigned>(chip.osrsH),
              osrsToStr(chip.osrsH));
  std::printf("  ctrl_meas: 0x%02X (osrs_t=%u %s, osrs_p=%u %s, mode=%u %s)\n",
              static_cast<unsigned>(chip.ctrlMeas),
              static_cast<unsigned>(chip.osrsT),
              osrsToStr(chip.osrsT),
              static_cast<unsigned>(chip.osrsP),
              osrsToStr(chip.osrsP),
              static_cast<unsigned>(chip.modeBits),
              modeBitsToStr(chip.modeBits));
  std::printf("  config: 0x%02X (standby=%u %s, filter=%u %s, spi3w_en=%u)\n",
              static_cast<unsigned>(chip.config),
              static_cast<unsigned>(chip.standby),
              standbyToStr(chip.standby),
              static_cast<unsigned>(chip.filter),
              filterToStr(chip.filter),
              chip.spi3wEnabled ? 1U : 0U);
}

void printInternalSettings(const InternalSettings& internal) {
  std::printf("=== Internal Settings ===\n");
  std::printf("  Mode: %s\n", modeToStr(internal.mode));
  std::printf("  osrs_t: %s (%u)\n",
              osrsToStr(static_cast<uint8_t>(internal.osrsT)),
              static_cast<unsigned>(internal.osrsT));
  std::printf("  osrs_p: %s (%u)\n",
              osrsToStr(static_cast<uint8_t>(internal.osrsP)),
              static_cast<unsigned>(internal.osrsP));
  std::printf("  osrs_h: %s (%u)\n",
              osrsToStr(static_cast<uint8_t>(internal.osrsH)),
              static_cast<unsigned>(internal.osrsH));
  std::printf("  Filter: %s (%u)\n",
              filterToStr(static_cast<uint8_t>(internal.filter)),
              static_cast<unsigned>(internal.filter));
  std::printf("  Standby: %s (%u)\n",
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
    std::printf("Chip mode: %s (%u)\n",
                modeBitsToStr(chip.modeBits),
                static_cast<unsigned>(chip.modeBits));
  }
  InternalSettings internal;
  if (readInternalSettings(internal)) {
    std::printf("Internal mode: %s\n", modeToStr(internal.mode));
  }
  printVerboseState();
}

void printOsrsSettings() {
  ChipSettings chip;
  if (readChipSettings(chip)) {
    std::printf("Chip osrs: T=%s (%u), P=%s (%u), H=%s (%u)\n",
                osrsToStr(chip.osrsT), static_cast<unsigned>(chip.osrsT),
                osrsToStr(chip.osrsP), static_cast<unsigned>(chip.osrsP),
                osrsToStr(chip.osrsH), static_cast<unsigned>(chip.osrsH));
  }
  InternalSettings internal;
  if (readInternalSettings(internal)) {
    std::printf("Internal osrs: T=%s (%u), P=%s (%u), H=%s (%u)\n",
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
    std::printf("Chip filter: %s (%u)\n",
                filterToStr(chip.filter),
                static_cast<unsigned>(chip.filter));
  }
  InternalSettings internal;
  if (readInternalSettings(internal)) {
    std::printf("Internal filter: %s (%u)\n",
                filterToStr(static_cast<uint8_t>(internal.filter)),
                static_cast<unsigned>(internal.filter));
  }
  printVerboseState();
}

void printStandbySettings() {
  ChipSettings chip;
  if (readChipSettings(chip)) {
    std::printf("Chip standby: %s (%u)\n",
                standbyToStr(chip.standby),
                static_cast<unsigned>(chip.standby));
  }
  InternalSettings internal;
  if (readInternalSettings(internal)) {
    std::printf("Internal standby: %s (%u)\n",
                standbyToStr(static_cast<uint8_t>(internal.standby)),
                static_cast<unsigned>(internal.standby));
  }
  printVerboseState();
}

void printSettingsUsage() {
  LOGW("Usage: settings [values|validate|start|set] <mode> <t> <p> <h> <filter> <standby>");
}

void printSettingsValues() {
  std::printf("=== Sensor Settings Values ===\n");
  std::printf("  mode: sleep|forced|normal (0|1|3)\n");
  std::printf("  t/p/h: skip|x1|x2|x4|x8|x16 (0..5)\n");
  std::printf("  filter: off|x2|x4|x8|x16 (0..4)\n");
  std::printf("  standby canonical: ms_0_5|ms_62_5|ms_125|ms_250|ms_500|ms_1000|ms_10|ms_20 (0..7)\n");
  std::printf("  standby aliases: 0.5ms|62.5ms|125ms|250ms|500ms|1000ms|10ms|20ms\n");
  std::printf("  constraint: temperature must be enabled when pressure or humidity is enabled; all channels cannot be skipped\n");
}

void printRequestedSettings(const BME280::SensorSettings& settings) {
  std::printf("=== Requested Sensor Settings ===\n");
  std::printf("  Mode: %s (%u)\n", BME280::toString(settings.mode),
              static_cast<unsigned>(settings.mode));
  std::printf("  Oversampling: T=%s (%u) P=%s (%u) H=%s (%u)\n",
              BME280::toString(settings.osrsT), static_cast<unsigned>(settings.osrsT),
              BME280::toString(settings.osrsP), static_cast<unsigned>(settings.osrsP),
              BME280::toString(settings.osrsH), static_cast<unsigned>(settings.osrsH));
  std::printf("  Filter: %s (%u)\n", BME280::toString(settings.filter),
              static_cast<unsigned>(settings.filter));
  std::printf("  Standby: %s (%u)\n", BME280::toString(settings.standby),
              static_cast<unsigned>(settings.standby));
}

void handleSettingsCommand(char*& cursor) {
  char* action = nextToken(cursor);
  if (action == nullptr) {
    printAllSettings();
    return;
  }
  if (std::strcmp(action, "values") == 0) {
    if (nextToken(cursor) != nullptr) {
      printSettingsUsage();
      return;
    }
    printSettingsValues();
    return;
  }
  if (std::strcmp(action, "validate") != 0 &&
      std::strcmp(action, "start") != 0 &&
      std::strcmp(action, "set") != 0) {
    printSettingsUsage();
    return;
  }

  char* tokens[6] = {};
  for (size_t i = 0; i < 6U; ++i) {
    tokens[i] = nextToken(cursor);
    if (tokens[i] == nullptr) {
      printSettingsUsage();
      return;
    }
  }
  if (nextToken(cursor) != nullptr) {
    printSettingsUsage();
    return;
  }

  BME280::SensorSettings settings;
  if (!parseMode(tokens[0], settings.mode) ||
      !parseOversampling(tokens[1], settings.osrsT) ||
      !parseOversampling(tokens[2], settings.osrsP) ||
      !parseOversampling(tokens[3], settings.osrsH) ||
      !parseFilter(tokens[4], settings.filter) ||
      !parseStandby(tokens[5], settings.standby)) {
    printSettingsUsage();
    return;
  }
  printRequestedSettings(settings);

  const BME280::Status validation = BME280::validateSettings(settings);
  if (std::strcmp(action, "validate") == 0) {
    printStatus(validation);
    return;
  }
  if (!validation.ok()) {
    printStatus(validation);
    return;
  }
  if (std::strcmp(action, "start") == 0) {
    const BME280::Status status = device.startApplySettingsJob(settings);
    if (status.inProgress()) {
      clearPendingBookkeeping();
    }
    printJobResult(makeJobBoundaryResult(status), "START");
    return;
  }

  if (!cancelPendingForCommand()) {
    return;
  }
  runStartedJobToTerminal(device.startApplySettingsJob(settings),
                          JOB_CLI_DEFAULT_BUDGET);
}

void printSampleFreshness(bool hasMaxAge, uint32_t maxAgeMs) {
  BME280::SettingsSnapshot snapshot;
  (void)device.getSettings(snapshot);
  const uint32_t now = currentMs();
  std::printf("=== Sample Freshness ===\n");
  std::printf("  Initialized: %s\n", snapshot.initialized ? "true" : "false");
  std::printf("  Has sample: %s\n", snapshot.hasSample ? "true" : "false");
  std::printf("  Freshness: %s\n", BME280::toString(snapshot.sampleFreshness));
  std::printf("  Sequence: %lu\n", static_cast<unsigned long>(snapshot.sampleSequence));
  std::printf("  Config generation: %lu\n",
              static_cast<unsigned long>(snapshot.configGeneration));
  std::printf("  Sample config generation: %lu\n",
              static_cast<unsigned long>(snapshot.sampleConfigGeneration));
  std::printf("  Timestamp ms: %lu\n",
              static_cast<unsigned long>(snapshot.sampleTimestampMs));
  if (snapshot.hasSample) {
    std::printf("  Age ms: %lu\n",
                static_cast<unsigned long>(device.sampleAgeMs(now)));
  } else {
    std::printf("  Age ms: unavailable\n");
  }
  std::printf("  Last measurement status: %s\n",
              BME280::toString(snapshot.lastMeasurementStatus.code));
  if (hasMaxAge) {
    std::printf("  Max age ms: %lu\n", static_cast<unsigned long>(maxAgeMs));
    std::printf("  Within max age: %s\n",
                device.sampleFresh(now, maxAgeMs) ? "true" : "false");
  }
}

void printRegisterBlock(uint8_t start, const uint8_t* data, size_t length) {
  std::printf("=== Register Block ===\n");
  std::printf("  Start: 0x%02X\n", static_cast<unsigned>(start));
  std::printf("  Length: %u\n", static_cast<unsigned>(length));
  for (size_t offset = 0; offset < length; offset += 8U) {
    std::printf("  0x%02X:", static_cast<unsigned>(start + offset));
    const size_t lineEnd = (offset + 8U < length) ? offset + 8U : length;
    for (size_t i = offset; i < lineEnd; ++i) {
      std::printf(" %02X", static_cast<unsigned>(data[i]));
    }
    std::printf("\n");
  }
}

void printTransferStats() {
  const IdfI2cTransferStats stats = bme280IdfTransferStats();
  std::printf("XFER_STATS read=%lu write=%lu total=%lu\n",
              static_cast<unsigned long>(stats.read),
              static_cast<unsigned long>(stats.write),
              static_cast<unsigned long>(stats.total));
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

void clearPendingBookkeeping() {
  gPendingRead = false;
  gStressRemaining = 0;
  gStress.active = false;
}

BME280::Status cancelPending() {
  BME280::SettingsSnapshot snapshot;
  const BME280::Status snapshotStatus = device.getSettings(snapshot);
  if (!snapshotStatus.ok()) {
    clearPendingBookkeeping();
    return snapshotStatus;
  }
  if (!snapshot.initialized) {
    clearPendingBookkeeping();
    return BME280::Status::Ok();
  }
  if (snapshot.measurementReady) {
    BME280::Measurement discarded;
    const BME280::Status status = device.getMeasurement(discarded);
    clearPendingBookkeeping();
    return status;
  }
  if (snapshot.measurementRequested) {
    const BME280::Mode restoreMode = snapshot.mode;
    BME280::Status st = device.setMode(BME280::Mode::SLEEP);
    if (st.ok() && restoreMode != BME280::Mode::SLEEP) {
      st = device.setMode(restoreMode);
    }
    clearPendingBookkeeping();
    return st;
  }
  clearPendingBookkeeping();
  return BME280::Status::Ok();
}

bool cancelPendingForCommand() {
  const BME280::Status status = cancelPending();
  if (status.ok()) {
    return true;
  }
  printStatus(status);
  return false;
}

bool cancelPendingForRecovery() {
  const BME280::JobState state = device.jobState();
  if (state == BME280::JobState::RUNNING ||
      state == BME280::JobState::WAITING) {
    const BME280::Status cancelStatus =
        device.cancelJob(BME280::CancelReason::OWNER_REQUEST);
    if (cancelStatus.code != BME280::Err::CANCELLED) {
      printStatus(cancelStatus);
      return false;
    }
    (void)device.pollJob(currentMs(), 0U);
  } else if (state == BME280::JobState::CANCELLED ||
             state == BME280::JobState::TIMED_OUT) {
    // Consume the retained terminal result before starting recovery.
    (void)device.pollJob(currentMs(), 0U);
  }
  return cancelPendingForCommand();
}

bool driverMeasurementPending() {
  BME280::SettingsSnapshot snapshot;
  return device.getSettings(snapshot).ok() && snapshot.initialized &&
         snapshot.measurementRequested && !snapshot.measurementReady;
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
  } else if (st.code == BME280::Err::BUSY && driverMeasurementPending()) {
    BME280::SettingsSnapshot snapshot;
    (void)device.getSettings(snapshot);
    gPendingRead = true;
    gPendingStartMs = snapshot.measurementStartMs;
    st = BME280::Status::Error(BME280::Err::IN_PROGRESS);
  }
  return st;
}

BME280::Status performMeasurementBlocking(BME280::Measurement& out, uint32_t timeoutMs = 500U) {
  BME280::Status st = device.requestMeasurement();
  if (st.code == BME280::Err::BUSY && !driverMeasurementPending()) {
    return st;
  }
  if (st.code != BME280::Err::IN_PROGRESS && st.code != BME280::Err::BUSY) {
    return st;
  }
  const uint32_t start = currentMs();
  while (!device.measurementReady()) {
    const uint32_t now = currentMs();
    device.tick(now);
    const BME280::Status pollStatus = device.lastMeasurementStatus();
    if (!pollStatus.ok() && !pollStatus.inProgress()) {
      const BME280::Status cancelStatus = cancelPending();
      return cancelStatus.ok() ? pollStatus : cancelStatus;
    }
    if (static_cast<uint32_t>(now - start) > timeoutMs) {
      const BME280::Status cancelStatus = cancelPending();
      if (!cancelStatus.ok()) {
        return cancelStatus;
      }
      return BME280::Status::Error(BME280::Err::TIMEOUT);
    }
    vTaskDelay(delayTicksAtLeastOne(1U));
  }
  return device.getMeasurement(out);
}

void resetStressStats(int target) {
  gStress = StressStats{};
  gStress.active = true;
  gStress.startMs = currentMs();
  gStress.target = target;
  gStress.successBefore = device.totalSuccess();
  gStress.failBefore = device.totalFailures();
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
  const uint32_t successDelta = device.totalSuccess() - gStress.successBefore;
  const uint32_t failDelta = device.totalFailures() - gStress.failBefore;
  const float successRate =
      (gStress.attempts > 0) ? (100.0f * static_cast<float>(gStress.success) / gStress.attempts) : 0.0f;
  std::printf("=== Stress Summary ===\n");
  std::printf("  Target: %d\n", gStress.target);
  std::printf("  Attempts: %d\n", gStress.attempts);
  std::printf("  Success: %s%d%s\n", nonZeroGoodColor(gStress.success), gStress.success, LOG_COLOR_RESET);
  std::printf("  Errors: %s%lu%s\n", zeroGoodColor(gStress.errors), static_cast<unsigned long>(gStress.errors), LOG_COLOR_RESET);
  std::printf("  Success rate: %s%.1f%%%s\n", successRateColor(successRate), successRate, LOG_COLOR_RESET);
  std::printf("  Duration: %lu ms\n", static_cast<unsigned long>(elapsed));
  if (elapsed > 0U) {
    std::printf("  Rate: %.2f samples/s\n",
                1000.0f * static_cast<float>(gStress.attempts) / static_cast<float>(elapsed));
  }
  std::printf("  Health delta: %ssuccess +%lu%s, %sfailures +%lu%s\n",
              nonZeroGoodColor(successDelta),
              static_cast<unsigned long>(successDelta),
              LOG_COLOR_RESET,
              zeroGoodColor(failDelta),
              static_cast<unsigned long>(failDelta),
              LOG_COLOR_RESET);
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
  if (!gPendingRead) {
    return;
  }
  if (!device.measurementReady()) {
    const BME280::Status terminalStatus = device.lastMeasurementStatus();
    if (terminalStatus.ok() || terminalStatus.inProgress() ||
        driverMeasurementPending()) {
      return;
    }
    gPendingRead = false;
    if (gStress.active) {
      noteStressError(terminalStatus);
      ++gStress.attempts;
      if (gStressRemaining > 0) {
        --gStressRemaining;
      }
      printStressProgress(static_cast<uint32_t>(gStress.attempts),
                          static_cast<uint32_t>(gStress.target),
                          static_cast<uint32_t>(gStress.success),
                          gStress.errors);
      if (gStressRemaining == 0) {
        finishStressStats();
      }
    } else {
      printStatus(terminalStatus);
    }
    return;
  }
  BME280::Measurement sample;
  const BME280::Status st = device.getMeasurement(sample);
  gPendingRead = false;
  if (!st.ok()) {
    if (gStress.active) {
      noteStressError(st);
      ++gStress.attempts;
      if (gStressRemaining > 0) {
        --gStressRemaining;
      }
      printStressProgress(static_cast<uint32_t>(gStress.attempts),
                          static_cast<uint32_t>(gStress.target),
                          static_cast<uint32_t>(gStress.success),
                          gStress.errors);
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
    if (gStressRemaining > 0) {
      --gStressRemaining;
    }
    printStressProgress(static_cast<uint32_t>(gStress.attempts),
                        static_cast<uint32_t>(gStress.target),
                        static_cast<uint32_t>(gStress.success),
                        gStress.errors);
    if (gStressRemaining == 0) {
      finishStressStats();
    }
    return;
  }
  printMeasurement(sample);
}

BME280::Status captureSensorSettings(BME280::SensorSettings& settings) {
  BME280::SettingsSnapshot snapshot;
  const BME280::Status st = device.getSettings(snapshot);
  if (!st.ok()) return st;
  if (!snapshot.initialized) {
    return BME280::Status::Error(BME280::Err::NOT_INITIALIZED);
  }
  settings.mode = snapshot.mode;
  settings.osrsT = snapshot.osrsT;
  settings.osrsP = snapshot.osrsP;
  settings.osrsH = snapshot.osrsH;
  settings.filter = snapshot.filter;
  settings.standby = snapshot.standby;
  return BME280::validateSettings(settings);
}

bool sensorSettingsMatch(const BME280::SensorSettings& expected,
                         const BME280::SettingsSnapshot& actual) {
  return actual.initialized && !actual.hardwareConfigDirty &&
         actual.mode == expected.mode && actual.osrsT == expected.osrsT &&
         actual.osrsP == expected.osrsP && actual.osrsH == expected.osrsH &&
         actual.filter == expected.filter &&
         actual.standby == expected.standby;
}

BME280::Status restoreSensorSettings(const BME280::SensorSettings& settings) {
  BME280::Status st = device.setMode(BME280::Mode::SLEEP);
  if (!st.ok()) return st;
  st = device.setOversamplingT(settings.osrsT);
  if (!st.ok()) return st;
  st = device.setOversamplingP(settings.osrsP);
  if (!st.ok()) return st;
  st = device.setOversamplingH(settings.osrsH);
  if (!st.ok()) return st;
  st = device.setFilter(settings.filter);
  if (!st.ok()) return st;
  st = device.setStandby(settings.standby);
  if (!st.ok()) return st;
  st = device.setMode(settings.mode);
  if (!st.ok()) return st;

  BME280::SettingsSnapshot restored;
  st = device.getSettings(restored);
  if (!st.ok()) return st;
  if (!sensorSettingsMatch(settings, restored)) {
    return BME280::Status::Error(BME280::Err::RESYNC_REQUIRED);
  }
  return BME280::Status::Ok();
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
  constexpr int opCount = static_cast<int>(sizeof(stats) / sizeof(stats[0]));

  if (!cancelPendingForCommand()) {
    return;
  }
  BME280::SensorSettings originalSettings;
  const BME280::Status captureStatus = captureSensorSettings(originalSettings);
  if (!captureStatus.ok()) {
    printStatus(captureStatus);
    return;
  }

  HealthSnapshot healthBefore;
  healthBefore.capture(device);
  const uint32_t successBefore = device.totalSuccess();
  const uint32_t failBefore = device.totalFailures();
  const uint32_t startMs = currentMs();
  uint32_t okTotal = 0;
  uint32_t failTotal = 0;
  bool hasFailure = false;
  BME280::Status firstFailure = BME280::Status::Ok();
  BME280::Status lastFailure = BME280::Status::Ok();

  LOGI("Starting mixed stress: %d cycles", count);
  for (int i = 0; i < count; ++i) {
    const int op = i % opCount;
    BME280::Status st = BME280::Status::Ok();
    switch (op) {
      case 0: {
        BME280::Measurement sample;
        st = ensureForcedMeasurementMode();
        if (st.ok()) {
          st = performMeasurementBlocking(sample);
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
          st = BME280::Status::Error(BME280::Err::CHIP_ID_MISMATCH, id);
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
            ((i / opCount) % 2 == 0) ? BME280::Mode::FORCED
                                     : BME280::Mode::SLEEP;
        st = device.setMode(mode);
        break;
      }
      case 5:
        st = device.setFilter(
            static_cast<BME280::Filter>((i / opCount) % 5));
        break;
      case 6:
        st = device.setStandby(
            static_cast<BME280::Standby>((i / opCount) % 8));
        break;
      default:
        break;
    }

    if (st.ok()) {
      ++stats[op].ok;
      ++okTotal;
    } else {
      ++stats[op].fail;
      ++failTotal;
      if (!hasFailure) {
        firstFailure = st;
        hasFailure = true;
      }
      lastFailure = st;
      if (gVerbose) {
        std::printf("  [%d] %s failed: %s\n", i, stats[op].name,
                    errToStr(st.code));
      }
    }
    printStressProgress(static_cast<uint32_t>(i + 1),
                        static_cast<uint32_t>(count),
                        okTotal,
                        failTotal);
    vTaskDelay(delayTicksAtLeastOne(1U));
  }

  std::printf("=== stress_mix summary ===\n");
  const float successPct =
      count > 0 ? 100.0f * static_cast<float>(okTotal) /
                      static_cast<float>(count)
                : 0.0f;
  std::printf("  Total: %sok=%lu%s %sfail=%lu%s (%s%.2f%%%s)\n",
              nonZeroGoodColor(okTotal),
              static_cast<unsigned long>(okTotal),
              LOG_COLOR_RESET,
              zeroGoodColor(failTotal),
              static_cast<unsigned long>(failTotal),
              LOG_COLOR_RESET,
              successRateColor(successPct),
              successPct,
              LOG_COLOR_RESET);
  for (int i = 0; i < opCount; ++i) {
    const uint32_t total = stats[i].ok + stats[i].fail;
    const float opPct = total > 0U
                            ? 100.0f * static_cast<float>(stats[i].ok) /
                                  static_cast<float>(total)
                            : 0.0f;
    std::printf("  %-10s %sok=%lu%s %sfail=%lu%s (%s%.1f%%%s)\n",
                stats[i].name,
                nonZeroGoodColor(stats[i].ok),
                static_cast<unsigned long>(stats[i].ok),
                LOG_COLOR_RESET,
                zeroGoodColor(stats[i].fail),
                static_cast<unsigned long>(stats[i].fail),
                LOG_COLOR_RESET,
                successRateColor(opPct),
                opPct,
                LOG_COLOR_RESET);
  }
  if (hasFailure) {
    std::printf("  First failure:\n");
    printStatus(firstFailure);
    if (failTotal > 1U) {
      std::printf("  Last failure:\n");
      printStatus(lastFailure);
    }
  }

  const BME280::Status restoreStatus = restoreSensorSettings(originalSettings);
  std::printf("  Restore status: %s\n", errToStr(restoreStatus.code));
  if (!restoreStatus.ok()) {
    printStatus(restoreStatus);
  }

  const uint32_t elapsed = currentMs() - startMs;
  const uint32_t successDelta = device.totalSuccess() - successBefore;
  const uint32_t failDelta = device.totalFailures() - failBefore;
  HealthSnapshot healthAfter;
  healthAfter.capture(device);
  std::printf("  Duration: %lu ms\n", static_cast<unsigned long>(elapsed));
  if (elapsed > 0U) {
    std::printf("  Rate: %.2f ops/s\n",
                1000.0f * static_cast<float>(count) /
                    static_cast<float>(elapsed));
  }
  std::printf("  Health changes:\n");
  printHealthDiff(healthBefore, healthAfter);
  std::printf("  Health delta: %ssuccess +%lu%s, %sfailures +%lu%s\n",
              nonZeroGoodColor(successDelta),
              static_cast<unsigned long>(successDelta),
              LOG_COLOR_RESET,
              zeroGoodColor(failDelta),
              static_cast<unsigned long>(failDelta),
              LOG_COLOR_RESET);
}

void runSelfTest() {
  struct Result {
    uint32_t pass = 0;
    uint32_t fail = 0;
    uint32_t skip = 0;
  } result;

  enum class Outcome : uint8_t { PASS, FAIL, SKIP };
  auto report = [&](const char* name, Outcome outcome, const char* note) {
    const bool ok = outcome == Outcome::PASS;
    const bool skip = outcome == Outcome::SKIP;
    const char* color = skip ? LOG_COLOR_YELLOW : LOG_COLOR_RESULT(ok);
    const char* tag = skip ? "SKIP" : (ok ? "PASS" : "FAIL");
    std::printf("  [%s%s%s] %s", color, tag, LOG_COLOR_RESET, name);
    if (note != nullptr && note[0] != '\0') {
      std::printf(" - %s", note);
    }
    std::printf("\n");
    if (skip) ++result.skip;
    else if (ok) ++result.pass;
    else ++result.fail;
  };
  auto reportCheck = [&](const char* name, bool ok, const char* note) {
    report(name, ok ? Outcome::PASS : Outcome::FAIL, note);
  };
  auto reportSkip = [&](const char* name, const char* note) {
    report(name, Outcome::SKIP, note);
  };

  std::printf("=== BME280 selftest (safe command smoke check) ===\n");
  std::printf("  Plausibility ranges are loose and environment-dependent; this is not factory calibration.\n");
  const BME280::Status cancelStatus = cancelPending();
  if (!cancelStatus.ok()) {
    printStatus(cancelStatus);
    std::printf("Selftest result: pass=0 fail=1 skip=0\n");
    return;
  }

  BME280::SensorSettings baselineSettings;
  const BME280::Status baselineStatus =
      captureSensorSettings(baselineSettings);
  const bool haveSnapshot = baselineStatus.ok();
  reportCheck("capture baseline settings", haveSnapshot,
              haveSnapshot ? "" : "could not read one or more fields");

  const uint32_t successBefore = device.totalSuccess();
  const uint32_t failBefore = device.totalFailures();
  const uint8_t consecutiveBefore = device.consecutiveFailures();

  BME280::Status st = device.probe();
  if (st.code == BME280::Err::NOT_INITIALIZED) {
    reportSkip("probe responds", "driver not initialized");
    reportSkip("remaining checks", "selftest aborted");
    std::printf("Selftest result: pass=%s%lu%s fail=%s%lu%s skip=%s%lu%s\n",
                nonZeroGoodColor(result.pass),
                static_cast<unsigned long>(result.pass),
                LOG_COLOR_RESET,
                zeroGoodColor(result.fail),
                static_cast<unsigned long>(result.fail),
                LOG_COLOR_RESET,
                result.skip > 0U ? LOG_COLOR_YELLOW : LOG_COLOR_GREEN,
                static_cast<unsigned long>(result.skip),
                LOG_COLOR_RESET);
    return;
  }
  reportCheck("probe responds", st.ok(), st.ok() ? "" : errToStr(st.code));
  const bool probeNoTrack = device.totalSuccess() == successBefore &&
                            device.totalFailures() == failBefore &&
                            device.consecutiveFailures() == consecutiveBefore;
  reportCheck("probe no-health-side-effects", probeNoTrack, "");

  uint8_t id = 0;
  st = device.readChipId(id);
  reportCheck("readChipId", st.ok(), st.ok() ? "" : errToStr(st.code));
  reportCheck("chip id matches 0x60",
              st.ok() && id == BME280::cmd::CHIP_ID_BME280, "");

  st = device.setMode(BME280::Mode::FORCED);
  reportCheck("setMode(FORCED)", st.ok(), st.ok() ? "" : errToStr(st.code));
  BME280::Mode modeNow = BME280::Mode::SLEEP;
  st = device.getMode(modeNow);
  reportCheck("getMode", st.ok(), st.ok() ? "" : errToStr(st.code));
  reportCheck("verify mode=FORCED",
              st.ok() && modeNow == BME280::Mode::FORCED, "");

  st = device.setOversamplingT(BME280::Oversampling::X2);
  reportCheck("setOversamplingT(X2)", st.ok(), st.ok() ? "" : errToStr(st.code));
  BME280::Oversampling osrs = BME280::Oversampling::SKIP;
  st = device.getOversamplingT(osrs);
  reportCheck("verify osrs_t=X2", st.ok() && osrs == BME280::Oversampling::X2,
              st.ok() ? "" : errToStr(st.code));

  st = device.setOversamplingP(BME280::Oversampling::X4);
  reportCheck("setOversamplingP(X4)", st.ok(), st.ok() ? "" : errToStr(st.code));
  st = device.getOversamplingP(osrs);
  reportCheck("verify osrs_p=X4", st.ok() && osrs == BME280::Oversampling::X4,
              st.ok() ? "" : errToStr(st.code));

  st = device.setOversamplingH(BME280::Oversampling::X2);
  reportCheck("setOversamplingH(X2)", st.ok(), st.ok() ? "" : errToStr(st.code));
  st = device.getOversamplingH(osrs);
  reportCheck("verify osrs_h=X2", st.ok() && osrs == BME280::Oversampling::X2,
              st.ok() ? "" : errToStr(st.code));

  st = device.setFilter(BME280::Filter::X4);
  reportCheck("setFilter(X4)", st.ok(), st.ok() ? "" : errToStr(st.code));
  BME280::Filter filter = BME280::Filter::OFF;
  st = device.getFilter(filter);
  reportCheck("verify filter=X4", st.ok() && filter == BME280::Filter::X4,
              st.ok() ? "" : errToStr(st.code));

  st = device.setStandby(BME280::Standby::MS_125);
  reportCheck("setStandby(125ms)", st.ok(), st.ok() ? "" : errToStr(st.code));
  BME280::Standby standby = BME280::Standby::MS_0_5;
  st = device.getStandby(standby);
  reportCheck("verify standby=125ms",
              st.ok() && standby == BME280::Standby::MS_125,
              st.ok() ? "" : errToStr(st.code));

  BME280::Measurement sample;
  st = performMeasurementBlocking(sample);
  reportCheck("measurement cycle", st.ok(), st.ok() ? "" : errToStr(st.code));
  const bool plausible = st.ok() && sample.temperatureValid &&
                         sample.pressureValid && sample.humidityValid &&
                         sample.temperatureC > -60.0f &&
                         sample.temperatureC < 130.0f &&
                         sample.humidityPct >= 0.0f &&
                         sample.humidityPct <= 100.0f &&
                         sample.pressurePa > 20000.0f &&
                         sample.pressurePa < 130000.0f;
  reportCheck("measurement in plausible range", plausible, "");

  BME280::RawSample raw;
  st = device.getRawSample(raw);
  reportCheck("getRawSample", st.ok(), st.ok() ? "" : errToStr(st.code));
  BME280::CompensatedSample compensated;
  st = device.getCompensatedSample(compensated);
  reportCheck("getCompensatedSample", st.ok(),
              st.ok() ? "" : errToStr(st.code));

  bool measuring = false;
  st = device.isMeasuring(measuring);
  reportCheck("isMeasuring", st.ok(), st.ok() ? "" : errToStr(st.code));
  const uint32_t measurementMs = device.estimateMeasurementTimeMs();
  const uint32_t standbyMs = device.getStandbyTimeMs();
  const uint32_t cycleMs = device.estimateNormalCycleMs();
  reportCheck("estimateMeasurementTimeMs>0", measurementMs > 0U, "");
  reportCheck("estimateNormalCycleMs>=meas", cycleMs >= measurementMs, "");
  reportCheck("getStandbyTimeMs valid",
              standbyMs > 0U || modeNow != BME280::Mode::NORMAL, "");

  uint8_t statusReg = 0;
  st = device.readStatus(statusReg);
  reportCheck("readStatus", st.ok(), st.ok() ? "" : errToStr(st.code));
  st = device.recover();
  reportCheck("recover", st.ok(), st.ok() ? "" : errToStr(st.code));
  reportCheck("isOnline", device.isOnline(), "");

  if (haveSnapshot) {
    const BME280::Status restoreStatus = restoreSensorSettings(baselineSettings);
    reportCheck("restore baseline settings", restoreStatus.ok(),
                restoreStatus.ok() ? "" : errToStr(restoreStatus.code));
    if (!restoreStatus.ok()) {
      printStatus(restoreStatus);
    }
  } else {
    reportSkip("restore baseline settings", "baseline unavailable");
  }

  std::printf("Selftest result: pass=%s%lu%s fail=%s%lu%s skip=%s%lu%s\n",
              nonZeroGoodColor(result.pass),
              static_cast<unsigned long>(result.pass),
              LOG_COLOR_RESET,
              zeroGoodColor(result.fail),
              static_cast<unsigned long>(result.fail),
              LOG_COLOR_RESET,
              result.skip > 0U ? LOG_COLOR_YELLOW : LOG_COLOR_GREEN,
              static_cast<unsigned long>(result.skip),
              LOG_COLOR_RESET);
}

void printHelp() {
  std::printf("\n%s=== BME280 CLI Help ===%s\n", LOG_COLOR_CYAN, LOG_COLOR_RESET);
  std::printf("\n%s[Common]%s\n", LOG_COLOR_GREEN, LOG_COLOR_RESET);
  printHelpItem("help / ?", "Show this help");
  printHelpItem("version / ver", "Print firmware and library version info");
  printHelpItem("scan", "Scan I2C bus");
  printHelpItem("addr [0x76|0x77]", "Show or select diagnostic I2C address");
  printHelpItem("begin", "Run begin() with the default example config");
  printHelpItem("end", "Zero-I2C unbind and clear cached driver state");
  printHelpItem("read", "Request and display measurement");
  printHelpItem("force", "Trigger one forced-mode measurement");
  printHelpItem("normal on/off", "Enable normal mode or return to sleep");
  printHelpItem("raw", "Show cached raw ADC sample and validity flags");
  printHelpItem("comp", "Show cached compensated sample and validity flags");
  printHelpItem("data", "Burst-read and decode live data registers");
  printHelpItem("measuring", "Show measuring flag");
  printHelpItem("timing", "Show measurement and cycle timing estimates");
  std::printf("\n%s[Configuration]%s\n", LOG_COLOR_GREEN, LOG_COLOR_RESET);
  printHelpItem("mode [sleep|forced|normal|0|1|3]", "Set or show operating mode");
  printHelpItem("osrs [t|p|h <name|code>]", "Names: skip,x1,x2,x4,x8,x16; temperature cannot skip");
  printHelpItem("filter [off|x2|x4|x8|x16|0..4]", "Set or show IIR filter (temperature/pressure only)");
  printHelpItem("standby [time|0..7]", "Times: 0.5ms,10ms,20ms,62.5ms,125ms,250ms,500ms,1000ms");
  printHelpItem("cfg / settings", "Show chip and internal settings");
  printHelpItem("settings values", "Show every accepted sensor-setting value");
  printHelpItem("settings validate <6 values>", "Validate mode,t,p,h,filter,standby without I2C");
  printHelpItem("settings start <6 values>", "Start zero-I2C staged whole-settings apply");
  printHelpItem("settings set <6 values>", "Apply complete settings through the staged job");
  printHelpItem("calib [raw]", "Show cached or raw calibration");
  printHelpItem("status", "Read status register");
  printHelpItem("id / chipid", "Read chip ID");
  printHelpItem("reset", "Soft reset device");
  std::printf("\n%s[Registers]%s\n", LOG_COLOR_GREEN, LOG_COLOR_RESET);
  printHelpItem("reg <addr>", "Read 8-bit register (hex address)");
  printHelpItem("dump / rregs <addr> <1..32>", "Read one bounded contiguous register block");
  printHelpItem("wreg <addr> <val>", "Write 8-bit register (diagnostic; config/reset writes mark dirty)");
  std::printf("\n%s[Diagnostics]%s\n", LOG_COLOR_GREEN, LOG_COLOR_RESET);
  printHelpItem("drv", "Show driver state and health");
  printHelpItem("state", "Show compact one-line health summary");
  printHelpItem("probe", "Probe device (no health tracking)");
  printHelpItem("recover", "Manual recovery attempt");
  printHelpItem("invalidate", "Zero-I2C invalidation for hotplug/replacement handling");
  printHelpItem("freshness [max_age_ms]", "Show cached sample provenance and optional age decision");
  printHelpItem("job status|start|cancel|poll|init|force|apply|resync|reset|recover", "Run staged job API diagnostics");
  printHelpItem("verbose [0|1]", "Enable/disable verbose output");
  printHelpItem("stress [N]", "Run N measurement cycles");
  printHelpItem("stress_mix [N]", "Run N mixed-operation cycles");
  printHelpItem("selftest", "Run safe command smoke-test report");
  printHelpItem("xfer_reset", "Reset example transport callback counters");
  printHelpItem("xfer_stats", "Show example transport callback counters");
  printHelpItem("xfer_assert <r> <w> <t>", "Assert read/write/total callback counters");
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
    if (!requireNoArguments(cursor, "help|?")) return;
    printHelp();
  } else if (std::strcmp(head, "version") == 0 || std::strcmp(head, "ver") == 0) {
    if (!requireNoArguments(cursor, "version|ver")) return;
    printVersionInfo();
  } else if (std::strcmp(head, "scan") == 0) {
    if (!requireNoArguments(cursor, "scan")) return;
    scanBus();
  } else if (std::strcmp(head, "job") == 0) {
    handleJobCommand(cursor);
  } else if (std::strcmp(head, "addr") == 0) {
    char* arg = nextToken(cursor);
    if (arg == nullptr) {
      printActiveAddress();
      return;
    }
    uint8_t address = 0;
    if (nextToken(cursor) != nullptr || !parseI2cAddress(arg, address)) {
      LOGW("Usage: addr 0x76|0x77");
      return;
    }
    LOGI("Selecting BME280 address 0x%02X", static_cast<unsigned>(address));
    clearPendingBookkeeping();
    device.end();
    gActiveAddress = address;
    const BME280::Status st = beginAtActiveAddress();
    printStatus(st);
    if (st.ok()) {
      printDriverHealth();
    }
  } else if (std::strcmp(head, "begin") == 0) {
    if (!requireNoArguments(cursor, "begin")) return;
    LOGI("Initializing BME280...");
    clearPendingBookkeeping();
    device.end();
    const BME280::Status st = beginAtActiveAddress();
    printStatus(st);
    if (st.ok()) {
      printDriverHealth();
    }
  } else if (std::strcmp(head, "end") == 0) {
    if (nextToken(cursor) != nullptr) {
      LOGW("Usage: end");
      return;
    }
    clearPendingBookkeeping();
    device.end();
    std::printf("=== Lifecycle ===\n");
    std::printf("Action: END\n");
    printStatus(BME280::Status::Ok());
    std::printf("Driver: state=%s initialized=%s\n",
                stateToStr(device.state()),
                device.isInitialized() ? "true" : "false");
  } else if (std::strcmp(head, "read") == 0) {
    if (!requireNoArguments(cursor, "read")) return;
    if (!cancelPendingForCommand()) {
      return;
    }
    const BME280::Status st = scheduleMeasurement();
    if (st.code != BME280::Err::IN_PROGRESS) {
      printStatus(st);
    }
  } else if (std::strcmp(head, "force") == 0) {
    if (!requireNoArguments(cursor, "force")) return;
    if (!cancelPendingForCommand()) {
      return;
    }
    BME280::Status st = device.setMode(BME280::Mode::FORCED);
    if (st.ok()) {
      st = scheduleMeasurement();
    }
    if (st.code != BME280::Err::IN_PROGRESS) {
      printStatus(st);
    }
  } else if (std::strcmp(head, "normal") == 0) {
    char* arg = nextToken(cursor);
    if (arg == nullptr) {
      printModeSettings();
      return;
    }
    if (nextToken(cursor) != nullptr ||
        (std::strcmp(arg, "on") != 0 && std::strcmp(arg, "off") != 0)) {
      LOGW("Usage: normal on|off");
      return;
    }
    if (!cancelPendingForCommand()) {
      return;
    }
    const BME280::Mode mode =
        (std::strcmp(arg, "on") == 0) ? BME280::Mode::NORMAL : BME280::Mode::SLEEP;
    printStatus(device.setMode(mode));
  } else if (std::strcmp(head, "raw") == 0) {
    if (!requireNoArguments(cursor, "raw")) return;
    printRawSample();
  } else if (std::strcmp(head, "comp") == 0) {
    if (!requireNoArguments(cursor, "comp")) return;
    printCompensatedSample();
  } else if (std::strcmp(head, "data") == 0) {
    if (!requireNoArguments(cursor, "data")) return;
    printDataRegisters();
  } else if (std::strcmp(head, "measuring") == 0) {
    if (!requireNoArguments(cursor, "measuring")) return;
    bool measuring = false;
    const BME280::Status st = device.isMeasuring(measuring);
    if (st.ok()) {
      std::printf("Measuring: %s\n", measuring ? "YES" : "NO");
    } else {
      printStatus(st);
    }
  } else if (std::strcmp(head, "timing") == 0) {
    if (!requireNoArguments(cursor, "timing")) return;
    printTimingInfo();
  } else if (std::strcmp(head, "freshness") == 0) {
    char* argument = nextToken(cursor);
    if (nextToken(cursor) != nullptr) {
      LOGW("Usage: freshness [max_age_ms]");
      return;
    }
    uint32_t maxAgeMs = 0;
    if (argument != nullptr && !parseU32(argument, maxAgeMs)) {
      LOGW("Usage: freshness [max_age_ms]");
      return;
    }
    printSampleFreshness(argument != nullptr, maxAgeMs);
  } else if (std::strcmp(head, "settings") == 0) {
    handleSettingsCommand(cursor);
  } else if (std::strcmp(head, "cfg") == 0) {
    if (nextToken(cursor) != nullptr) {
      LOGW("Usage: cfg");
      return;
    }
    printAllSettings();
  } else if (std::strcmp(head, "calib") == 0) {
    char* arg = nextToken(cursor);
    if (nextToken(cursor) != nullptr ||
        (arg != nullptr && std::strcmp(arg, "raw") != 0)) {
      LOGW("Usage: calib [raw]");
      return;
    }
    if (arg == nullptr) {
      printCalibration();
    } else {
      printCalibrationRaw();
    }
  } else if (std::strcmp(head, "mode") == 0) {
    char* arg = nextToken(cursor);
    if (nextToken(cursor) != nullptr) {
      LOGW("Usage: mode [sleep|forced|normal|0|1|3]");
      return;
    }
    if (arg == nullptr) {
      printModeSettings();
      return;
    }
    BME280::Mode mode = BME280::Mode::SLEEP;
    if (!parseMode(arg, mode)) {
      LOGW("Invalid mode: %s", arg);
      return;
    }
    if (!cancelPendingForCommand()) {
      return;
    }
    printStatus(device.setMode(mode));
  } else if (std::strcmp(head, "osrs") == 0) {
    char* which = nextToken(cursor);
    if (which == nullptr) {
      printOsrsSettings();
      return;
    }
    char* valueTok = nextToken(cursor);
    if (valueTok == nullptr || nextToken(cursor) != nullptr) {
      LOGW("Usage: osrs t <1..5> | osrs p|h <0..5>");
      return;
    }
    BME280::Oversampling osrs;
    if (!parseOversampling(valueTok, osrs)) {
      LOGW("Invalid oversampling value");
      return;
    }
    if (std::strcmp(which, "t") == 0 && osrs == BME280::Oversampling::SKIP) {
      LOGW("Temperature oversampling cannot be skipped in a valid configuration");
      return;
    }
    BME280::Status st;
    if (std::strcmp(which, "t") != 0 &&
        std::strcmp(which, "p") != 0 &&
        std::strcmp(which, "h") != 0) {
      LOGW("Invalid osrs target: %s", which);
      return;
    }
    if (!cancelPendingForCommand()) {
      return;
    }
    if (std::strcmp(which, "t") == 0) st = device.setOversamplingT(osrs);
    else if (std::strcmp(which, "p") == 0) st = device.setOversamplingP(osrs);
    else st = device.setOversamplingH(osrs);
    printStatus(st);
  } else if (std::strcmp(head, "filter") == 0) {
    char* arg = nextToken(cursor);
    if (nextToken(cursor) != nullptr) {
      LOGW("Usage: filter [off|x2|x4|x8|x16|0..4]");
      return;
    }
    if (arg == nullptr) {
      printFilterSettings();
      return;
    }
    BME280::Filter filter;
    if (!parseFilter(arg, filter)) {
      LOGW("Invalid filter value");
      return;
    }
    if (!cancelPendingForCommand()) {
      return;
    }
    printStatus(device.setFilter(filter));
  } else if (std::strcmp(head, "standby") == 0) {
    char* arg = nextToken(cursor);
    if (nextToken(cursor) != nullptr) {
      LOGW("Usage: standby [0.5ms|62.5ms|125ms|250ms|500ms|1000ms|10ms|20ms|0..7]");
      return;
    }
    if (arg == nullptr) {
      printStandbySettings();
      return;
    }
    BME280::Standby standby;
    if (!parseStandby(arg, standby)) {
      LOGW("Invalid standby value");
      return;
    }
    if (!cancelPendingForCommand()) {
      return;
    }
    printStatus(device.setStandby(standby));
  } else if (std::strcmp(head, "status") == 0) {
    if (!requireNoArguments(cursor, "status")) return;
    uint8_t status = 0;
    const BME280::Status st = device.readStatus(status);
    if (!st.ok()) {
      printStatus(st);
      return;
    }
    std::printf("Status: 0x%02X (measuring=%d, im_update=%d)\n",
                static_cast<unsigned>(status),
                (status & BME280::cmd::MASK_STATUS_MEASURING) != 0 ? 1 : 0,
                (status & BME280::cmd::MASK_STATUS_IM_UPDATE) != 0 ? 1 : 0);
    std::printf("Driver: state=%s online=%s dirty=%s\n",
                stateToStr(device.state()),
                boolStr(device.isOnline()),
                boolStr(device.hardwareConfigDirty()));
  } else if (std::strcmp(head, "chipid") == 0 || std::strcmp(head, "id") == 0) {
    if (!requireNoArguments(cursor, "chipid|id")) return;
    uint8_t id = 0;
    const BME280::Status st = device.readChipId(id);
    if (st.ok()) std::printf("Chip ID: 0x%02X\n", static_cast<unsigned>(id));
    else printStatus(st);
  } else if (std::strcmp(head, "reset") == 0) {
    if (!requireNoArguments(cursor, "reset")) return;
    if (!cancelPendingForCommand()) {
      return;
    }
    printStatus(device.softReset());
  } else if (std::strcmp(head, "dump") == 0 || std::strcmp(head, "rregs") == 0) {
    char* startToken = nextToken(cursor);
    char* lengthToken = nextToken(cursor);
    uint32_t start = 0;
    uint32_t length = 0;
    if (nextToken(cursor) != nullptr ||
        !parseU32(startToken, start) || !parseU32(lengthToken, length) ||
        start > 0xFFU || length == 0U || length > 32U ||
        start + length > 0x100U) {
      LOGW("Usage: dump|rregs <addr> <1..32>");
      return;
    }
    uint8_t data[32] = {};
    const BME280::Status st = device.readRegisters(
        static_cast<uint8_t>(start), data, static_cast<size_t>(length));
    if (!st.ok()) {
      printStatus(st);
      return;
    }
    printRegisterBlock(static_cast<uint8_t>(start), data,
                       static_cast<size_t>(length));
  } else if (std::strcmp(head, "reg") == 0) {
    char* addressToken = nextToken(cursor);
    uint32_t addr = 0;
    if (nextToken(cursor) != nullptr || !parseU32(addressToken, addr) || addr > 0xFFU) {
      LOGW("Usage: reg <addr>");
      return;
    }
    uint8_t value = 0;
    const BME280::Status st = device.readRegister(static_cast<uint8_t>(addr), value);
    if (st.ok()) {
      std::printf("Reg 0x%02lX = 0x%02X (%u)\n",
                  static_cast<unsigned long>(addr),
                  static_cast<unsigned>(value),
                  static_cast<unsigned>(value));
    }
    else printStatus(st);
  } else if (std::strcmp(head, "wreg") == 0) {
    char* addressToken = nextToken(cursor);
    char* valueToken = nextToken(cursor);
    uint32_t addr = 0;
    uint32_t value = 0;
    if (nextToken(cursor) != nullptr ||
        !parseU32(addressToken, addr) || !parseU32(valueToken, value) ||
        addr > 0xFFU || value > 0xFFU) {
      LOGW("Usage: wreg <addr> <val>");
      return;
    }
    printStatus(device.writeRegister(static_cast<uint8_t>(addr), static_cast<uint8_t>(value)));
  } else if (std::strcmp(head, "drv") == 0) {
    if (!requireNoArguments(cursor, "drv")) return;
    printDriverHealth();
    BME280::Mode mode;
    if (device.getMode(mode).ok()) {
      std::printf("  Mode: %s\n", modeToStr(mode));
    }
  } else if (std::strcmp(head, "state") == 0) {
    if (!requireNoArguments(cursor, "state")) return;
    printCompactHealth();
  } else if (std::strcmp(head, "probe") == 0) {
    if (!requireNoArguments(cursor, "probe")) return;
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
    if (!requireNoArguments(cursor, "recover")) return;
    if (!cancelPendingForRecovery()) {
      return;
    }
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
    std::printf("  Note: successful recover invalidates cached samples; run read before using them.\n");
  } else if (std::strcmp(head, "invalidate") == 0) {
    if (nextToken(cursor) != nullptr) {
      LOGW("Usage: invalidate");
      return;
    }
    const BME280::Status st = device.invalidateDeviceState();
    printStatus(st);
    if (st.ok()) {
      clearPendingBookkeeping();
    }
    printSampleFreshness(false, 0U);
  } else if (std::strcmp(head, "xfer_reset") == 0) {
    if (nextToken(cursor) != nullptr) {
      LOGW("Usage: xfer_reset");
      return;
    }
    bme280IdfResetTransferStats();
    std::printf("XFER_RESET read=0 write=0 total=0\n");
  } else if (std::strcmp(head, "xfer_stats") == 0) {
    if (nextToken(cursor) != nullptr) {
      LOGW("Usage: xfer_stats");
      return;
    }
    printTransferStats();
  } else if (std::strcmp(head, "xfer_assert") == 0) {
    char* readToken = nextToken(cursor);
    char* writeToken = nextToken(cursor);
    char* totalToken = nextToken(cursor);
    uint32_t expectedRead = 0;
    uint32_t expectedWrite = 0;
    uint32_t expectedTotal = 0;
    if (nextToken(cursor) != nullptr ||
        !parseU32(readToken, expectedRead) ||
        !parseU32(writeToken, expectedWrite) ||
        !parseU32(totalToken, expectedTotal)) {
      LOGW("Usage: xfer_assert <read> <write> <total>");
      return;
    }
    const IdfI2cTransferStats stats = bme280IdfTransferStats();
    const bool pass = stats.read == expectedRead &&
                      stats.write == expectedWrite &&
                      stats.total == expectedTotal;
    if (pass) {
      std::printf("XFER_ASSERT PASS read=%lu write=%lu total=%lu\n",
                  static_cast<unsigned long>(stats.read),
                  static_cast<unsigned long>(stats.write),
                  static_cast<unsigned long>(stats.total));
    } else {
      std::printf("XFER_ASSERT FAIL expected_read=%lu expected_write=%lu expected_total=%lu actual_read=%lu actual_write=%lu actual_total=%lu\n",
                  static_cast<unsigned long>(expectedRead),
                  static_cast<unsigned long>(expectedWrite),
                  static_cast<unsigned long>(expectedTotal),
                  static_cast<unsigned long>(stats.read),
                  static_cast<unsigned long>(stats.write),
                  static_cast<unsigned long>(stats.total));
    }
  } else if (std::strcmp(head, "verbose") == 0) {
    char* arg = nextToken(cursor);
    if (nextToken(cursor) != nullptr) {
      LOGW("Usage: verbose [0|1]");
      return;
    }
    if (arg == nullptr) {
      printVerboseState();
      return;
    }
    uint32_t value = 0;
    if (!parseU32(arg, value) || value > 1U) {
      LOGW("Usage: verbose [0|1]");
      return;
    }
    gVerbose = value == 1U;
    LOGI("Verbose mode: %s", gVerbose ? "ON" : "OFF");
  } else if (std::strcmp(head, "selftest") == 0) {
    if (!requireNoArguments(cursor, "selftest")) return;
    runSelfTest();
  } else if (std::strcmp(head, "stress_mix") == 0) {
    uint32_t count = DEFAULT_STRESS_MIX_COUNT;
    char* arg = nextToken(cursor);
    if (nextToken(cursor) != nullptr ||
        (arg != nullptr && (!parseU32(arg, count) || count == 0U || count > MAX_STRESS_COUNT))) {
      LOGW("Invalid stress_mix count");
      return;
    }
    runStressMix(static_cast<int>(count));
  } else if (std::strcmp(head, "stress") == 0) {
    uint32_t count = DEFAULT_STRESS_COUNT;
    char* arg = nextToken(cursor);
    if (nextToken(cursor) != nullptr ||
        (arg != nullptr && (!parseU32(arg, count) || count == 0U || count > MAX_STRESS_COUNT))) {
      LOGW("Invalid stress count");
      return;
    }
    if (!cancelPendingForCommand()) {
      return;
    }
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
      vTaskDelay(delayTicksAtLeastOne(20U));
      continue;
    }
    size_t len = std::strlen(buffer);
    const bool terminated =
        len > 0U && (buffer[len - 1U] == '\n' || buffer[len - 1U] == '\r');
    if (!terminated && len == sizeof(buffer) - 1U) {
      int next = 0;
      do {
        next = std::fgetc(stdin);
      } while (next != '\n' && next != '\r' && next != EOF);
      std::printf("Command too long\n");
      std::fflush(stdout);
      continue;
    }
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
      printStressProgress(static_cast<uint32_t>(gStress.attempts),
                          static_cast<uint32_t>(gStress.target),
                          static_cast<uint32_t>(gStress.success),
                          gStress.errors);
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

  BME280::Status st = initBusForActiveAddress();
  if (!st.ok()) {
    LOGE("Failed to initialize I2C");
    printStatus(st);
  } else {
    LOGI("I2C initialized (SDA=%d, SCL=%d, addr=0x%02X)",
         BME280_IDF_I2C_SDA,
         BME280_IDF_I2C_SCL,
         static_cast<unsigned>(gActiveAddress));
    scanBus();
    st = device.begin(makeDefaultConfig());
    if (!st.ok()) {
      LOGE("Failed to initialize device");
      printStatus(st);
    }
  }

  if (st.ok()) {
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

    vTaskDelay(delayTicksAtLeastOne(CLI_TICK_MS));
  }
}
