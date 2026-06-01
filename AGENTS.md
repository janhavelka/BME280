# AGENTS.md - BME280 Production Embedded Guidelines

## Role and Target
You are a professional embedded software engineer building a production-grade BME280 environmental sensor library.

- Target: ESP32-S2 / ESP32-S3, Arduino and ESP-IDF consumers, PlatformIO/ESP-IDF.
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
- Managed synchronous lifecycle: `Status begin(const Config&)`, `void tick(uint32_t nowMs)`, `void end()`.
- Public I2C calls may block for a bounded transport timeout and documented poll limit. Long-running measurement waits must be scheduled through `requestMeasurement()` and `tick()`, not hidden inside unbounded loops.
- No heap allocation in steady state (no `String`, `std::vector`, `new` in normal ops).
- No logging in library code; examples may log.
- No macros for constants; use `static constexpr`. Macros only for conditional compile or logging helpers.
- Core/public headers and `src/` must be framework-neutral: no Arduino or ESP-IDF framework headers unless a rare exception is justified in docs and enforced by tooling.
- Arduino APIs (`Arduino.h`, `Wire.h`, `Serial`, `String`, `TwoWire`) are allowed only in Arduino examples or example-only Arduino adapters.
- ESP-IDF examples must be native IDF examples using `app_main`, `driver/i2c_master.h`, `esp_timer`, FreeRTOS timing, and fixed C buffers or native console APIs.
- ESP-IDF examples must not include Arduino CLI source or use `ArduinoCompat`, `IdfArduinoCompat`, `Arduino.h`, `Wire.h`, `String`, `Serial`, or `TwoWire` facades.
- Preserve Arduino/ESP-IDF CLI parity through a repo-local command contract/checker, not by sharing Arduino implementation in IDF builds.

---

## I2C Manager + Transport (Required)

- The library MUST NOT own I2C. It never touches `Wire` directly.
- `Config` MUST accept a transport adapter (function pointers or abstract interface).
- Transport errors MUST map to `Status` (no leaking `Wire`, `esp_err_t`, etc.).
- The library MUST NOT configure bus timeouts or pins.
- The library must be transport-injected and non-owning. Application transport owns bus handles, pins, locks, and timeout policy.
- Transport callbacks must not recursively call into the same driver instance.

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
- Do not collapse distinguishable transport errors. Use `DEVICE_NOT_FOUND` only for definite absence/address NACK; preserve timeout, data NACK, bus, and generic I2C statuses when the transport can distinguish them.
- Public fallible APIs must return `Status` or explicitly document best-effort behavior.

---

## Concurrency, ISR, and Partial Hardware State

- Driver instances are not thread-safe. Applications must externally serialize access when multiple tasks share a driver or I2C bus.
- Public APIs are not ISR-safe unless a specific API explicitly documents and proves otherwise. I2C, measurement scheduling, and health bookkeeping are task-context operations.
- Multi-register hardware updates must either keep cache and hardware synchronized or expose an explicit dirty/resync-needed diagnostic.
- Dirty or partial hardware state may be cleared only after a successful full resync, recover, or documented verification path.
- Tests, reports, README, and hardware validation matrices must not invent results. If hardware, ESP-IDF, or fault-path validation was not run, say so.
- Examples must be labeled honestly as diagnostic, bring-up, or production templates. A production shared-bus example must show ownership, locking, timeout policy, and scheduling.

---

## BME280 hardening rules

- Core code in `include/` and `src/` must stay framework-neutral: no Arduino, Wire, ESP-IDF, FreeRTOS, platform logging, or platform timing calls.
- The core must not own or mutate the I2C bus; bus setup, locking, recovery, and timeouts belong to the injected transport or application bus manager.
- Fallible APIs return `Status`; no exceptions and no uncontrolled heap allocation in core.
- Measurement scheduling must use the injected/application timebase. `tick(uint32_t nowMs)` is caller-driven, and any public operation that needs monotonic time must explicitly require `Config::nowMs` or document best-effort behavior.
- Public methods are not internally thread-safe and are not ISR-safe unless explicitly implemented and tested.
- BME280 device facts are mandatory: chip ID `0x60`, addresses `0x76/0x77`, SDO not floating, CSB tied high for I2C, soft reset `0xB6`, status `measuring/im_update` semantics.
- `ctrl_hum` writes only become effective after `ctrl_meas`; config/filter/standby changes must avoid normal-mode ignored writes.
- Raw data must be burst-read when coherency matters.
- Calibration and compensation changes must state provenance. Compensation math must match Bosch behavior, including humidity packing, `t_fine`, 64-bit pressure, divide-by-zero guard, and humidity clamp.
- Reset and NVM-copy polling must be bounded, deadline/poll-limit based, and must preserve distinguishable transport failures where the transport can report them.
- Multi-register configuration writes that may partially reach hardware must set dirty hardware-config diagnostics and preserve the original error.
- Samples must not be misleading across config changes; use invalidation or config-generation tagging.
- Documentation must not claim hardware or ESP-IDF validation unless it was actually run.
- Hardening phases use scoped review roles: datasheet, core contracts, fault injection, compensation, IDF/CI, docs/hardware, and integration review. Each role must inspect actual repository files before recommendations are accepted.

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
- Health is tracked via **tracked transport wrappers** -- public API never calls `_updateHealth()` directly.
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
- `begin()` success -> READY
- Any I2C failure in READY -> DEGRADED
- Success in DEGRADED/OFFLINE -> READY
- Failures reach `offlineThreshold` -> OFFLINE
- `end()` -> UNINIT

### Transport Wrapper Architecture

All I2C goes through layered wrappers:

```
Public API (readMeasurement, setMode, etc.)
    |
Register helpers (readRegs, writeRegs)
    |
TRACKED wrappers (_i2cWriteReadTracked, _i2cWriteTracked)
    |  <- _updateHealth() called here ONLY
RAW wrappers (_i2cWriteReadRaw, _i2cWriteRaw)
    |
Transport callbacks (Config::i2cWrite, i2cWriteRead)
```

**Rules:**
- Public API methods NEVER call `_updateHealth()` directly
- `readRegs()`/`writeRegs()` use TRACKED wrappers -> health updated automatically
- `probe()` uses RAW wrappers -> no health tracking (diagnostic only)
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
1. Update `library.json` with `scripts/generate_version.py set X.Y.Z` or `bump`.
2. Regenerate and check `include/BME280/Version.h`.
3. Update `idf_component.yml` and `Doxyfile` to the same version.
4. Update `CHANGELOG.md` (Added/Changed/Fixed/Removed).
5. Update `README.md` and Doxygen-facing comments if API, examples, validation, or release checks changed.
6. Run local validation, commit as `Release vX.Y.Z`, push, wait for CI, then tag the exact release commit.

---

## Naming Conventions

- Member variables: `_camelCase`
- Methods/Functions: `camelCase`
- Constants: `CAPS_CASE`
- Enum values: `CAPS_CASE` or `X1`, `X2` for short forms
- Locals/params: `camelCase`
- Config fields: `camelCase`
