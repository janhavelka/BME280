# AGENTS.md - BME280 Production Embedded Guidelines

## Role and Target
You are a professional embedded software engineer building a production-grade BME280 environmental sensor library.

- Target: ESP32-S2 / ESP32-S3, Arduino framework, PlatformIO.
- Goals: deterministic behavior, long-term stability, clean API contracts, portability, no surprises in the field.
- These rules are binding.

---

## Repository Model (Single Library)

```
include/BME280/         - Public API headers only (Doxygen)
  CommandTable.h        - Register addresses and bit masks
  Status.h
  Config.h
  BME280.h
  Version.h             - Auto-generated (do not edit)
src/                    - Implementation (.cpp)
examples/
  01_*/
  common/               - Example-only helpers (Log.h, BoardConfig.h, I2cTransport.h,
                          I2cScanner.h, CommandHandler.h)
platformio.ini
library.json
README.md
CHANGELOG.md
AGENTS.md
```

Rules:
- `examples/common/` is NOT part of the library. It simulates project glue and keeps examples self-contained.
- No board-specific pins/bus in library code; only in `Config`.
- Public headers only in `include/BME280/`.
- Examples demonstrate usage and may use `examples/common/BoardConfig.h`.
- Keep the layout boring and predictable.

---

## Core Engineering Rules (Mandatory)

- Deterministic: no unbounded loops/waits; all timeouts via deadlines, never `delay()` in library code.
- Non-blocking lifecycle: `Status begin(const Config&)`, `void tick(uint32_t nowMs)`, `void end()`.
- Any I/O that can exceed ~1-2 ms must be split into state machine steps driven by `tick()`.
- No heap allocation in steady state (no `String`, `std::vector`, `new` in normal ops).
- No logging in library code; examples may log.
- No macros for constants; use `static constexpr`. Macros only for conditional compile or logging helpers.

---

## I2C Manager + Transport (Required)

- The library MUST NOT own I2C. It never touches `Wire` directly.
- `Config` MUST accept a transport adapter (function pointers or abstract interface).
- Transport errors MUST map to `Status` (no leaking `Wire`, `esp_err_t`, etc.).
- The library MUST NOT configure bus timeouts or pins.

---

## Status / Error Handling (Mandatory)

All fallible APIs return `Status`:

```cpp
struct Status {
  Err code;
  int32_t detail;
  const char* msg;  // static string only
};
```

- Silent failure is unacceptable.
- No exceptions.

---

## BME280 Driver Requirements

- I2C address 0x76 (default) or 0x77 (configurable via SDO pin); check chip ID (0x60) in `begin()`.
- Read compensation data (calibration coefficients) once in `begin()` and cache in RAM.
- Support measurement modes:
  - **Sleep mode**: No measurements, lowest power.
  - **Forced mode**: Single measurement on demand, returns to sleep.
  - **Normal mode**: Continuous measurements at configured standby interval.
- Configurable oversampling (skip, 1x, 2x, 4x, 8x, 16x) for temperature, pressure, humidity.
- Configurable IIR filter coefficient (off, 2, 4, 8, 16).
- Configurable standby time for normal mode (0.5ms to 1000ms).
- Proper measurement time calculation based on oversampling settings.
- Apply Bosch compensation formulas (32-bit integer or 64-bit for pressure).
- Burst read all data registers (0xF7-0xFE) in single I2C transaction.
- Soft reset via register write (0xB6 to 0xE0).

---

## Driver Architecture: Managed Synchronous Driver

The driver follows a **managed synchronous** model with health tracking:

- All public I2C operations are **blocking** (no async state machine needed - BME280 has no EEPROM writes).
- `tick()` may be used for normal-mode polling or measurement-ready checks.
- Health is tracked via **tracked transport wrappers** — public API never calls `_updateHealth()` directly.
- Recovery is **manual** via `recover()` - the application controls retry strategy.

### DriverState (4 states only)

```cpp
enum class DriverState : uint8_t {
  UNINIT,    // begin() not called or end() called
  READY,     // Operational, consecutiveFailures == 0
  DEGRADED,  // 1 <= consecutiveFailures < offlineThreshold
  OFFLINE    // consecutiveFailures >= offlineThreshold
};
```

State transitions:
- `begin()` success → READY
- Any I2C failure in READY → DEGRADED
- Success in DEGRADED/OFFLINE → READY
- Failures reach `offlineThreshold` → OFFLINE
- `end()` → UNINIT

### Transport Wrapper Architecture

All I2C goes through layered wrappers:

```
Public API (readMeasurement, setMode, etc.)
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
- `readRegs()`/`writeRegs()` use TRACKED wrappers → health updated automatically
- `probe()` uses RAW wrappers → no health tracking (diagnostic only)
- `recover()` tracks probe failures (driver is initialized, so failures count)

### Health Tracking Rules

- `_updateHealth()` called ONLY inside tracked transport wrappers.
- State transitions guarded by `_initialized` (no DEGRADED/OFFLINE before `begin()` succeeds).
- NOT called for config/param validation errors (INVALID_CONFIG, INVALID_PARAM).
- NOT called for precondition errors (NOT_INITIALIZED).
- `probe()` uses raw I2C and does NOT update health (diagnostic only).

### Health Tracking Fields

- `_lastOkMs` - timestamp of last successful I2C operation
- `_lastErrorMs` - timestamp of last failed I2C operation
- `_lastError` - most recent error Status
- `_consecutiveFailures` - failures since last success (resets on success)
- `_totalFailures` / `_totalSuccess` - lifetime counters (wrap at max)

---

## Versioning and Releases

Single source of truth: `library.json`. `Version.h` is auto-generated and must never be edited.

SemVer:
- MAJOR: breaking API/Config/enum changes.
- MINOR: new backward-compatible features or error codes (append only).
- PATCH: bug fixes, refactors, docs.

Release steps:
1. Update `library.json`.
2. Update `CHANGELOG.md` (Added/Changed/Fixed/Removed).
3. Update `README.md` if API or examples changed.
4. Commit and tag: `Release vX.Y.Z`.

---

## Naming Conventions

- Member variables: `_camelCase`
- Methods/Functions: `camelCase`
- Constants: `CAPS_CASE`
- Enum values: `CAPS_CASE` or `X1`, `X2` for short forms
- Locals/params: `camelCase`
- Config fields: `camelCase`
