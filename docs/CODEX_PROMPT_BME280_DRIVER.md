# Codex Prompt: Production-Grade BME280 I2C Driver Library (ESP32 Arduino / PlatformIO)

You are an expert embedded engineer. Implement a **production-grade, reusable BME280 I2C driver library** for **ESP32-S2 / ESP32-S3** using **Arduino framework** under **PlatformIO**.

---

## Baseline Contracts (DO NOT RESTATE)

- **`AGENTS.md`** defines: repository layout, non-blocking architecture, injected I2C transport rules, deterministic tick behavior, managed synchronous driver pattern, transport wrapper architecture, health tracking, and DriverState model.
- **`BME280_Register_Reference.md`** is the authoritative register map.
- **Do not duplicate those requirements.** Implement them and only document what is *not already covered*.

---

## Deliverables

Generate a complete library repository:
- All headers in `include/BME280/` (Status.h, Config.h, BME280.h, CommandTable.h, Version.h)
- Implementation in `src/BME280.cpp`
- Examples in `examples/01_basic_bringup_cli/`
- Example helpers in `examples/common/` (Log.h, BoardConfig.h, I2cTransport.h)
- README.md, CHANGELOG.md, library.json, platformio.ini
- Version generation script in `scripts/generate_version.py`

**This is a library, not an application.**

---

## Namespace

All types in `namespace BME280 { }`.

---

## Status.h — Error Codes

Follow the RV3032 pattern exactly:

```cpp
enum class Err : uint8_t {
  OK = 0,                  ///< Operation successful
  NOT_INITIALIZED,         ///< begin() not called
  INVALID_CONFIG,          ///< Invalid configuration parameter
  I2C_ERROR,               ///< I2C communication failure
  TIMEOUT,                 ///< Operation timed out
  INVALID_PARAM,           ///< Invalid parameter value
  DEVICE_NOT_FOUND,        ///< BME280 not responding on I2C bus
  CHIP_ID_MISMATCH,        ///< Chip ID != 0x60 (not a BME280)
  CALIBRATION_INVALID,     ///< Compensation data failed validation
  MEASUREMENT_NOT_READY,   ///< Sample not yet available
  COMPENSATION_ERROR,      ///< Compensation math failed (overflow/div-by-zero)
  BUSY,                    ///< Device is measuring
  IN_PROGRESS              ///< Operation scheduled; call tick() to complete
};

struct Status {
  Err code = Err::OK;
  int32_t detail = 0;
  const char* msg = "";
  
  constexpr bool ok() const { return code == Err::OK; }
  static constexpr Status Ok() { return Status{Err::OK, 0, "OK"}; }
  static constexpr Status Error(Err err, const char* message, int32_t detailCode = 0);
};
```

---

## Config.h — Configuration

```cpp
/// I2C write callback signature
using I2cWriteFn = Status (*)(uint8_t addr, const uint8_t* data, size_t len,
                              uint32_t timeoutMs, void* user);

/// I2C write-read callback signature  
using I2cWriteReadFn = Status (*)(uint8_t addr, const uint8_t* tx, size_t txLen,
                                  uint8_t* rx, size_t rxLen, uint32_t timeoutMs,
                                  void* user);

/// Oversampling settings
enum class Oversampling : uint8_t {
  SKIP = 0,  ///< Measurement skipped
  X1   = 1,  ///< 1x oversampling
  X2   = 2,  ///< 2x oversampling
  X4   = 3,  ///< 4x oversampling
  X8   = 4,  ///< 8x oversampling
  X16  = 5   ///< 16x oversampling
};

/// Measurement mode
enum class Mode : uint8_t {
  SLEEP  = 0,  ///< No measurements, lowest power
  FORCED = 1,  ///< Single measurement, returns to sleep
  NORMAL = 3   ///< Continuous measurements
};

/// IIR filter coefficient
enum class Filter : uint8_t {
  OFF = 0,
  X2  = 1,
  X4  = 2,
  X8  = 3,
  X16 = 4
};

/// Standby time between measurements (normal mode)
enum class Standby : uint8_t {
  MS_0_5  = 0,   ///< 0.5 ms
  MS_62_5 = 1,   ///< 62.5 ms
  MS_125  = 2,   ///< 125 ms
  MS_250  = 3,   ///< 250 ms
  MS_500  = 4,   ///< 500 ms
  MS_1000 = 5,   ///< 1000 ms
  MS_10   = 6,   ///< 10 ms
  MS_20   = 7    ///< 20 ms
};

struct Config {
  // === I2C Transport (required) ===
  I2cWriteFn i2cWrite = nullptr;
  I2cWriteReadFn i2cWriteRead = nullptr;
  void* i2cUser = nullptr;
  
  // === Device Settings ===
  uint8_t i2cAddress = 0x76;       ///< 0x76 (SDO=GND) or 0x77 (SDO=VDD)
  uint32_t i2cTimeoutMs = 50;      ///< I2C transaction timeout
  
  // === Measurement Settings ===
  Oversampling osrsT = Oversampling::X1;   ///< Temperature oversampling
  Oversampling osrsP = Oversampling::X1;   ///< Pressure oversampling
  Oversampling osrsH = Oversampling::X1;   ///< Humidity oversampling
  Filter filter = Filter::OFF;              ///< IIR filter coefficient
  Standby standby = Standby::MS_125;        ///< Standby time (normal mode)
  Mode mode = Mode::FORCED;                 ///< Operating mode
  
  // === Health Tracking ===
  uint8_t offlineThreshold = 5;    ///< Consecutive failures before OFFLINE
};
```

---

## CommandTable.h — Register Definitions

Define all BME280 registers from `BME280_Register_Reference.md`:
- Register addresses as `static constexpr uint8_t REG_*`
- Bit masks as `static constexpr uint8_t MASK_*`
- Bit positions as `static constexpr uint8_t BIT_*`

Example structure:
```cpp
namespace cmd {
  // Chip identification
  static constexpr uint8_t REG_CHIP_ID = 0xD0;
  static constexpr uint8_t CHIP_ID_BME280 = 0x60;
  
  // Reset
  static constexpr uint8_t REG_RESET = 0xE0;
  static constexpr uint8_t RESET_VALUE = 0xB6;
  
  // Status
  static constexpr uint8_t REG_STATUS = 0xF3;
  static constexpr uint8_t STATUS_MEASURING = 0x08;
  static constexpr uint8_t STATUS_IM_UPDATE = 0x01;
  
  // Control registers
  static constexpr uint8_t REG_CTRL_HUM = 0xF2;
  static constexpr uint8_t REG_CTRL_MEAS = 0xF4;
  static constexpr uint8_t REG_CONFIG = 0xF5;
  
  // Data registers (burst read 0xF7-0xFE)
  static constexpr uint8_t REG_DATA_START = 0xF7;
  static constexpr uint8_t DATA_LEN = 8;
  
  // Calibration data
  static constexpr uint8_t REG_CALIB_T_P = 0x88;   // T1-T3, P1-P9 (26 bytes)
  static constexpr uint8_t REG_CALIB_H = 0xE1;     // H2-H6 (7 bytes)
  static constexpr uint8_t REG_CALIB_H1 = 0xA1;    // H1 (1 byte)
  
  // ... complete from register reference
}
```

---

## BME280.h — Main Driver Class

### DriverState (same as RV3032)

```cpp
enum class DriverState : uint8_t {
  UNINIT,    ///< begin() not called or end() called
  READY,     ///< Operational, consecutiveFailures == 0
  DEGRADED,  ///< 1 <= consecutiveFailures < offlineThreshold
  OFFLINE    ///< consecutiveFailures >= offlineThreshold
};
```

### Data Structures

```cpp
/// Measurement result (float)
struct Measurement {
  float temperatureC = 0.0f;   ///< Temperature in Celsius
  float pressurePa = 0.0f;     ///< Pressure in Pascals
  float humidityPct = 0.0f;    ///< Relative humidity in percent
};

/// Raw ADC values
struct RawSample {
  int32_t adcT = 0;   ///< Raw temperature ADC (20-bit)
  int32_t adcP = 0;   ///< Raw pressure ADC (20-bit)
  int32_t adcH = 0;   ///< Raw humidity ADC (16-bit)
};

/// Fixed-point compensated values (no float)
struct CompensatedSample {
  int32_t tempC_x100 = 0;        ///< Temperature * 100 (e.g., 2534 = 25.34°C)
  uint32_t pressurePa = 0;       ///< Pressure in Pa
  uint32_t humidityPct_x1024 = 0; ///< Humidity * 1024 (Q22.10 format)
};
```

### Public API

```cpp
class BME280 {
public:
  // === Lifecycle ===
  Status begin(const Config& config);
  void tick(uint32_t nowMs);
  void end();
  
  // === Diagnostics (no health tracking) ===
  Status probe();
  Status recover();
  
  // === Driver State ===
  DriverState state() const;
  bool isOnline() const;
  
  // === Health Tracking ===
  uint32_t lastOkMs() const;
  uint32_t lastErrorMs() const;
  Status lastError() const;
  uint8_t consecutiveFailures() const;
  uint32_t totalFailures() const;
  uint32_t totalSuccess() const;
  
  // === Measurement API ===
  
  /// Request a measurement (non-blocking)
  /// In FORCED mode: triggers measurement if idle
  /// In NORMAL mode: marks intent to read next available
  /// Returns IN_PROGRESS if measurement started, BUSY if already measuring
  Status requestMeasurement();
  
  /// Check if measurement is ready to read
  bool measurementReady() const;
  
  /// Get measurement result (float)
  /// Returns MEASUREMENT_NOT_READY if not available
  /// Clears ready flag after successful read
  Status getMeasurement(Measurement& out);
  
  /// Get raw ADC values
  Status getRawSample(RawSample& out) const;
  
  /// Get fixed-point compensated values
  Status getCompensatedSample(CompensatedSample& out) const;
  
  // === Configuration ===
  
  /// Set operating mode (SLEEP, FORCED, NORMAL)
  Status setMode(Mode mode);
  
  /// Get current mode
  Status getMode(Mode& out);
  
  /// Set oversampling for temperature
  Status setOversamplingT(Oversampling osrs);
  
  /// Set oversampling for pressure
  Status setOversamplingP(Oversampling osrs);
  
  /// Set oversampling for humidity
  Status setOversamplingH(Oversampling osrs);
  
  /// Set IIR filter coefficient
  Status setFilter(Filter filter);
  
  /// Set standby time (normal mode only)
  Status setStandby(Standby standby);
  
  /// Soft reset device
  Status softReset();
  
  /// Read chip ID
  Status readChipId(uint8_t& id);
  
  /// Read status register
  Status readStatus(uint8_t& status);
  
  /// Check if device is currently measuring
  Status isMeasuring(bool& measuring);
  
  // === Timing ===
  
  /// Estimate max measurement time based on current oversampling
  /// Returns time in milliseconds
  uint32_t estimateMeasurementTimeMs() const;

private:
  // === Transport Wrappers (tracked vs raw) ===
  Status _i2cWriteReadRaw(const uint8_t* txBuf, size_t txLen, uint8_t* rxBuf, size_t rxLen);
  Status _i2cWriteRaw(const uint8_t* buf, size_t len);
  Status _i2cWriteReadTracked(const uint8_t* txBuf, size_t txLen, uint8_t* rxBuf, size_t rxLen);
  Status _i2cWriteTracked(const uint8_t* buf, size_t len);
  
  // === Register Access ===
  Status readRegs(uint8_t reg, uint8_t* buf, size_t len);
  Status writeRegs(uint8_t reg, const uint8_t* buf, size_t len);
  Status readRegister(uint8_t reg, uint8_t& value);
  Status writeRegister(uint8_t reg, uint8_t value);
  Status _readRegisterRaw(uint8_t reg, uint8_t& value);
  
  // === Health Tracking ===
  Status _updateHealth(const Status& st);
  
  // === Internal ===
  Status _applyConfig();
  Status _readCalibration();
  Status _validateCalibration();
  Status _readRawData();
  void _compensate();
  
  // === State ===
  Config _config;
  bool _initialized = false;
  DriverState _driverState = DriverState::UNINIT;
  
  // === Health Counters ===
  uint32_t _lastOkMs = 0;
  uint32_t _lastErrorMs = 0;
  Status _lastError = Status::Ok();
  uint8_t _consecutiveFailures = 0;
  uint32_t _totalFailures = 0;
  uint32_t _totalSuccess = 0;
  
  // === Calibration Data (cached from device) ===
  // Temperature
  uint16_t _digT1 = 0;
  int16_t _digT2 = 0;
  int16_t _digT3 = 0;
  // Pressure
  uint16_t _digP1 = 0;
  int16_t _digP2 = 0;
  int16_t _digP3 = 0;
  int16_t _digP4 = 0;
  int16_t _digP5 = 0;
  int16_t _digP6 = 0;
  int16_t _digP7 = 0;
  int16_t _digP8 = 0;
  int16_t _digP9 = 0;
  // Humidity
  uint8_t _digH1 = 0;
  int16_t _digH2 = 0;
  uint8_t _digH3 = 0;
  int16_t _digH4 = 0;
  int16_t _digH5 = 0;
  int8_t _digH6 = 0;
  
  // === Measurement State ===
  bool _measurementRequested = false;
  bool _measurementReady = false;
  uint32_t _measurementStartMs = 0;
  int32_t _tFine = 0;  ///< Shared temperature compensation value
  RawSample _rawSample;
  CompensatedSample _compSample;
};
```

---

## Implementation Requirements

### Transport Wrapper Architecture

Follow RV3032 pattern exactly:

```
Public API (getMeasurement, setMode, etc.)
    ↓
Register helpers (readRegs, writeRegs)
    ↓
TRACKED wrappers (_i2cWriteReadTracked, _i2cWriteTracked)
    ↓  ← _updateHealth() called here ONLY
RAW wrappers (_i2cWriteReadRaw, _i2cWriteRaw)
    ↓
Transport callbacks (Config::i2cWrite, i2cWriteRead)
```

**Rules:**
- Public API methods NEVER call `_updateHealth()` directly
- `readRegs()`/`writeRegs()` use TRACKED wrappers
- `probe()` uses RAW wrappers (no health tracking)
- `recover()` tracks probe failures via `_updateHealth()`
- State transitions in `_updateHealth()` guarded by `_initialized`

### begin() Flow

1. Store config, reset state, reset health counters
2. Set `_initialized = false`, `_driverState = UNINIT`
3. Validate config (callbacks not null, timeout > 0)
4. Clamp `offlineThreshold` to minimum 1
5. `probe()` — verify device responds (raw path, no health tracking)
6. Verify chip ID == 0x60
7. `_readCalibration()` — read and parse compensation data
8. `_validateCalibration()` — sanity check calibration values
9. `_applyConfig()` — write ctrl_hum, ctrl_meas, config registers
10. Set `_initialized = true`, `_driverState = READY`

### Compensation Math

Use Bosch reference formulas (32-bit integer math):
- Temperature compensation produces `_tFine` (shared)
- Pressure compensation uses `_tFine`
- Humidity compensation uses `_tFine`
- Detect divide-by-zero and return `COMPENSATION_ERROR`

### ctrl_hum Latch Behavior

BME280 requires:
1. Write `ctrl_hum` first
2. Write `ctrl_meas` to latch humidity setting

Always use this sequence when changing humidity oversampling.

### Measurement Timing

```cpp
uint32_t estimateMeasurementTimeMs() const {
  // Base measurement time from datasheet
  // T_measure = 1.25 + (2.3 * osrs_t) + (2.3 * osrs_p + 0.575) + (2.3 * osrs_h + 0.575)
  // Add safety margin
}
```

### tick() Behavior

In forced mode with pending measurement:
1. Check if measurement time elapsed
2. Read status register to confirm not measuring
3. Burst read data registers
4. Run compensation
5. Set `_measurementReady = true`

In normal mode:
- `tick()` can poll for new data if requested

---

## Example: 01_basic_bringup_cli

Interactive CLI demonstrating all features:

```
Commands:
  help              - Show this help
  read              - Request and display measurement
  mode sleep|forced|normal - Set operating mode
  osrs t|p|h <0..5> - Set oversampling (0=skip, 1=x1, ..., 5=x16)
  filter <0..4>     - Set IIR filter
  standby <0..7>    - Set standby time
  status            - Read status register
  chipid            - Read chip ID
  reset             - Soft reset device
  
Driver Debugging:
  drv               - Show driver state and health
  probe             - Probe device (no health tracking)
  recover           - Manual recovery attempt
  verbose 0|1       - Enable/disable verbose output
  stress [N]        - Run N measurement cycles
```

Example must:
- Use non-blocking scheduling via `tick()`
- Never `delay()` while waiting for measurement
- Show driver state transitions and health tracking
- Demonstrate recovery from failures

---

## examples/common/ Files

### Log.h
```cpp
#define LOGI(fmt, ...) Serial.printf("[I] " fmt "\n", ##__VA_ARGS__)
#define LOGW(fmt, ...) Serial.printf("[W] " fmt "\n", ##__VA_ARGS__)
#define LOGE(fmt, ...) Serial.printf("[E] " fmt "\n", ##__VA_ARGS__)
```

### BoardConfig.h
```cpp
namespace board {
  #if defined(CONFIG_IDF_TARGET_ESP32S3)
    static constexpr int I2C_SDA = 8;
    static constexpr int I2C_SCL = 9;
  #elif defined(CONFIG_IDF_TARGET_ESP32S2)
    static constexpr int I2C_SDA = 8;
    static constexpr int I2C_SCL = 9;
  #endif
  
  bool initI2c();
}
```

### I2cTransport.h
```cpp
namespace transport {
  BME280::Status wireWrite(uint8_t addr, const uint8_t* data, size_t len,
                           uint32_t timeoutMs, void* user);
  BME280::Status wireWriteRead(uint8_t addr, const uint8_t* tx, size_t txLen,
                               uint8_t* rx, size_t rxLen, uint32_t timeoutMs,
                               void* user);
}
```

---

## platformio.ini

```ini
[platformio]
default_envs = ex_bringup_s3

[env]
framework = arduino
monitor_speed = 115200
lib_deps = 
build_flags = 
  -std=c++17
  -Wall
  -Wextra

[env:ex_bringup_s3]
platform = espressif32
board = esp32-s3-devkitc-1
build_src_filter = +<examples/01_basic_bringup_cli/>
extra_scripts = pre:scripts/generate_version.py

[env:ex_bringup_s2]
platform = espressif32
board = esp32-s2-saola-1
build_src_filter = +<examples/01_basic_bringup_cli/>
extra_scripts = pre:scripts/generate_version.py
```

---

## library.json

```json
{
  "name": "BME280",
  "version": "1.0.0",
  "description": "Production-grade BME280 environmental sensor driver",
  "keywords": ["bme280", "sensor", "temperature", "pressure", "humidity", "i2c"],
  "repository": {
    "type": "git",
    "url": "https://github.com/user/BME280.git"
  },
  "authors": [
    {
      "name": "Your Name"
    }
  ],
  "license": "MIT",
  "frameworks": ["arduino"],
  "platforms": ["espressif32"]
}
```

---

## Implementation Constraints (DO NOT VIOLATE)

- No heap allocation in steady-state
- No Wire dependency in library code
- No logging in library code (examples may log)
- Bounded deterministic work per tick()
- Injected transport only
- Never `delay()` in library code
- All public I2C operations are blocking
- Health tracking only via tracked transport wrappers
- State transitions guarded by `_initialized`

---

## Final Output

Generate all repository files with correct, compile-ready, production-quality code.
Ensure examples build for ESP32-S3.

You have at hand also boardconfig, i2cscanner and i2ctransport inspired by a different example - edit it to fit this library example, but keep the inner workings the same

**Now implement the full library.**
