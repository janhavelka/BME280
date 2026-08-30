# AGENTS.md - BME280 Production Embedded Guidelines

## Scope and Target

This repository is a production-grade BME280 environmental sensor library.

- Target: ESP32-S2 / ESP32-S3, Arduino and ESP-IDF consumers, PlatformIO/ESP-IDF.
- Goals: deterministic behavior, long-term stability, clean API contracts, portability, no surprises in the field.
- The rules in this file are binding for any change to this repository.

---

## Working in this repository

Before editing, fetch remotes and fast-forward the newest intended working
branch to its upstream. Stop and report dirty, divergent, or conflicted state;
never overwrite work to force a sync.

On Windows, invoke PlatformIO as `.\scripts\pio.cmd <arguments>`; it selects the
current user's VS Code-managed installation. Never install another PlatformIO
Core; if the wrapper cannot find it, stop and report the missing installation.

---

## Repository Model (Single Library)

```
include/BME280/         - Public API headers only (Doxygen)
  BME280.h              - Driver class, staged jobs, sample/health accessors
  CommandTable.h        - Register addresses, bit masks, and bit positions
  Config.h              - Transport callbacks, SensorSettings, free helpers
  Status.h              - Err enum and Status
  Version.h             - Auto-generated from library.json (do not edit)
src/BME280.cpp          - The entire implementation
examples/
  01_basic_bringup_cli/ - Arduino bring-up/diagnostic CLI
  idf/basic/            - Native ESP-IDF example with the same CLI contract
  common/               - Example-only helpers (BoardConfig.h, BuildConfig.h,
                          CliStyle.h, HealthView.h, I2cScanner.h,
                          I2cTransport.h, Log.h)
test/                   - Native Unity suite plus Arduino/Wire stubs
tools/                  - Contract checkers and the I2C HIL runner
scripts/                - generate_version.py, pio.cmd
docs/                   - Datasheet, register reference, port/validation guides
.github/workflows/      - CI
CMakeLists.txt          - ESP-IDF component build
Doxyfile
idf_component.yml
library.json
platformio.ini
```

Rules:
- `examples/common/` is NOT part of the library. It simulates project glue and keeps examples self-contained.
- No board-specific pins/bus in library code; only in `Config`.
- Public headers only in `include/BME280/`.
- Examples demonstrate usage and may use `examples/common/BoardConfig.h`.
- Keep the layout boring and predictable.

---

## Core Engineering Rules (Mandatory)

- Prefer simplicity, clarity, correctness, robustness, safety, and readability over clever abstractions or speculative flexibility.
- Before coding, inspect whether existing code can be simplified, reused, or deleted.
- Prefer deleting unnecessary code over adding new code.
- Keep changes tightly scoped to the user's request, and preserve dirty user changes; never revert unrelated work.
- Prefer extending existing owners, modules, and API contracts over creating parallel abstractions.
- Before adding a service, class, file, interface, or abstraction, prove a concrete current need and a clear caller or test.
- Do not add placeholder classes, future stubs, empty managers, broad frameworks, plugin systems, service registries, generic layers, or speculative extension points.
- Prefer explicit state, explicit ownership, and small local helpers over hidden global state.
- Deterministic: no unbounded loops, waits, retries, allocations, queues, or buffers in steady paths; all timeouts via deadlines, never `delay()` in library code.
- Every hardware operation that can block must have a timeout and an observable failure path.
- Recovery logic must be bounded, deterministic, and testable.
- Do not hide hardware failures behind silent retries or fake success.
- Managed synchronous lifecycle: `Status begin(const Config&)`, `void tick(uint32_t nowMs)`, `void end()`.
- Public I2C calls may block for a bounded transport timeout and documented poll limit. Long-running measurement waits must be scheduled through `requestMeasurement()` and `tick()`, not hidden inside unbounded loops.
- No heap allocation in steady state (no `String`, `std::vector`, `new` in normal ops). Avoid dynamic allocation in steady embedded paths unless it is already an accepted local pattern and the bound is clear.
- No logging in library code; examples may log.
- No macros for constants; use `static constexpr`. Macros only for conditional compile or logging helpers.
- Core/public headers and `src/` must be framework-neutral: no Arduino or ESP-IDF framework headers unless a rare exception is justified in docs and enforced by tooling.
- Arduino APIs (`Arduino.h`, `Wire.h`, `Serial`, `String`, `TwoWire`) are allowed only in Arduino examples or example-only Arduino adapters.
- ESP-IDF examples must be native IDF examples using `app_main`, `driver/i2c_master.h`, `esp_timer`, FreeRTOS timing, and fixed C buffers or native console APIs.
- ESP-IDF examples must not include Arduino CLI source or use `ArduinoCompat`, `IdfArduinoCompat`, `Arduino.h`, `Wire.h`, `String`, `Serial`, or `TwoWire` facades.
- Preserve Arduino/ESP-IDF CLI parity through a repo-local command contract/checker, not by sharing Arduino implementation in IDF builds.

---

## I2C Manager + Transport (Required)

- The I2C bus must have one clear owner.
- The library MUST NOT own I2C. It never touches `Wire` directly.
- `Config` MUST accept a transport adapter (function pointers or abstract interface).
- Transport errors MUST map to `Status` (no leaking `Wire`, `esp_err_t`, etc.).
- The library MUST NOT configure bus timeouts or pins.
- The library must be transport-injected and non-owning. Application transport owns bus handles, pins, locks, and timeout policy.
- Device drivers must not directly own or reconfigure a shared bus unless this repository architecture explicitly says so.
- I2C transactions must be timeout-bounded and report errors clearly.
- Transport callbacks must not recursively call into the same driver instance.
- Keep chip-level protocol code inside the driver or wrapper. Keep application policy outside the chip driver.
- Do not implement chip protocols manually if an existing hardened project library already provides the needed timeout, recovery, and testability behavior.
- Do not add fake devices, simulated buses, or test doubles to production paths.

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

## Driver Architecture

The driver exposes two ways to reach the hardware over one shared admission
rule: a **synchronous** API for simple callers, and a **staged job** API for
callers that must bound how much bus time a single call consumes.

- Synchronous calls (`begin`, `recover`, `softReset`, the `set*`/`read*`
  helpers) block only for the injected transport timeout. They never loop on a
  chip-ready condition: if the chip is not ready they return `BUSY` or
  `TIMEOUT` and let the caller decide.
- Staged jobs (`startInitJob`, `startForcedMeasurementJob`,
  `startApplyConfigJob`, `startApplySettingsJob`, `startResyncJob`,
  `startSoftResetJob`) are advanced by `pollJob(nowMs, maxCallbacks)`. Each poll
  issues at most `maxCallbacks` transport callbacks; waiting is expressed as
  deadlines, never as sleeps.
- `requestMeasurement()` + `tick(nowMs)` is the scheduling path for a single
  capture; it uses the caller's monotonic timebase.
- A running staged job exclusively owns hardware access. Every hardware-facing
  API goes through the same admission check and returns `BUSY` with a
  `BusyReason` detail rather than interleaving.
- Recovery is always caller-driven: `recover()`, `startResyncJob()`, or
  `startSoftResetJob()`. The driver never self-heals in the background.

### DriverState

`DriverState` is an observational health classification, not a gate:
`UNINIT`, `READY`, `DEGRADED`, `OFFLINE`. `OFFLINE` is diagnostic only and does
not block an explicit owner-directed operation. The authoritative definitions
and transitions live in `include/BME280/BME280.h`; do not duplicate the enum
here.

Device-state correctness is tracked separately from health by `ConfigSyncState`
and `CalibrationState`. Only those two block a measurement (`RESYNC_REQUIRED`).

### Transport Wrapper Architecture

All I2C goes through layered wrappers:

```
Public API (getMeasurement, setMode, readRegisters, ...)
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
- Public API methods never call `_updateHealth()` directly.
- `readRegs()`/`writeRegs()` use the tracked wrappers, so health updates happen
  automatically.
- `probe()` and the pre-`begin()` steps of initialization use the raw wrappers,
  so they do not perturb health counters.
- `_recordFailure()` is the one deliberate exception: it records *semantic*
  failures (identity, calibration, NVM, config readiness) that a successful
  transfer would otherwise hide.

### Health Tracking Rules

- `_updateHealth()` is called only inside the tracked transport wrappers.
- State transitions are guarded by `_initialized`; there is no DEGRADED/OFFLINE
  before `begin()` succeeds.
- Not called for config/param validation errors (`INVALID_CONFIG`,
  `INVALID_PARAM`) or precondition errors (`NOT_INITIALIZED`).

### Health Tracking Fields

- `_lastOkMs` / `_lastErrorMs` - timestamps of the last tracked success/failure.
  Each has a `*TimeValid` companion flag because `Config::nowMs` is optional.
- `_lastError` - most recent tracked error `Status`.
- `_consecutiveFailures` - failures since the last tracked success; saturates at
  `UINT8_MAX`.
- `_totalFailures` / `_totalSuccess` - per-health-session counters. They
  **saturate** at `UINT32_MAX` and never wrap; `begin()` starts a new session by
  resetting both.

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
5. Update `SECURITY.md` supported versions on every MAJOR or MINOR release.
6. Update `README.md` and Doxygen-facing comments if API, examples, validation, or release checks changed.
7. Run the validation gate in `CONTRIBUTING.md`, commit as `Release vX.Y.Z`, push,
   wait for CI, then tag the exact release commit.

---

## Naming Conventions

- Member variables: `_camelCase`
- Methods/Functions: `camelCase`
- Constants: `CAPS_CASE`
- Enum values: `CAPS_CASE` or `X1`, `X2` for short forms
- Locals/params: `camelCase`
- Config fields: `camelCase`
