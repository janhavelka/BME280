# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [1.5.0] - 2026-05-14

### Added
- `SettingsSnapshot` struct for reading cached configuration and runtime state without I2C.
- `getSettings(SettingsSnapshot&)` method to populate a settings snapshot.
- `Status::is(Err)` method for type-safe error code comparison.
- `Status::operator bool()` explicit conversion for concise success checks.
- Native tests for forced-mode on-demand behavior, normal-mode fresh-cycle gating, invalid oversampling combinations, and chip-ID mismatch recovery health.
- Native coverage proving latched `OFFLINE` blocks normal I2C operations without touching the bus while `recover()` remains the explicit recovery path.

### Changed
- Doxyfile project metadata now matches `library.json`, and archived prompt
  metadata no longer contains placeholder ownership values.
- Explicit recovery/reset bypass internals now use the shared `ScopedOfflineI2cAllowance` / `_reassertOfflineLatch()` procedure so failed recovery attempts that begin from `OFFLINE` keep the latch asserted.
- Forced mode is now treated as an on-demand policy: `begin()` and `setMode(FORCED)` leave the hardware in sleep until `requestMeasurement()` triggers a conversion.
- Normal-mode `requestMeasurement()` now waits one estimated normal cycle before `tick()` reads data registers, avoiding stale immediate samples.
- README API docs now include measurement, configuration, calibration, status, and oversampling constraints.
- Reference documentation now uses human-readable vendor PDF names and separates compact chip notes from full PDF extractions under `docs/extracted-md/` and `docs/pdf-extracted-md/`.
- Health behavior is now standardized on latched `OFFLINE`: normal public I2C operations return `BUSY` with `Driver is offline; call recover()` and do not touch I2C until `recover()` succeeds.

### Fixed
- Bring-up CLI stress runs now clear stale pending measurement state before
  restarting, and `stress_mix` no longer counts expected sleep-mode measurement
  attempts as failures.
- Stress progress output now leaves the progress percentage uncolored and only
  colorizes the ok/fail counters.
- Invalid oversampling combinations are rejected in `begin()` and typed setters before I2C, instead of failing later during compensation inside `tick()`.
- `recover()` now records a chip-ID mismatch as a health failure instead of leaving the driver marked healthy after a failed recovery.
- Example diagnostic error strings now include granular `I2C_*` status codes.

## [1.4.0] - 2026-04-05

### Added
- Public lifecycle/config introspection helpers: `isInitialized()` and `getConfig()`.
- Public tracked raw-register helpers: `readRegisters()`, `writeRegisters()`, `readRegister()`, and `writeRegister()`.
- `Err::CONVERSION_NOT_READY` alias for cross-library uniformity.

### Changed
- Health tracking now treats `IN_PROGRESS` as non-failure activity and keeps pre-`begin()` validation errors from forcing state transitions.
- Bringup CLI now exposes `reg` / `wreg` register diagnostics for tracked low-level access.

## [1.3.0] - 2026-04-03

### Added
- Granular I2C transport error codes: `I2C_NACK_ADDR`, `I2C_NACK_DATA`, `I2C_TIMEOUT`, and `I2C_BUS`.
- Native tests covering the `millis()` fallback and example transport error mapping.

### Changed
- `examples/common/I2cTransport.h` now uses `TwoWire*` from `Config::i2cUser` and treats per-call `timeoutMs` as advisory.
- README quick start, transport notes, and documentation section now match the example adapter and shipped datasheet/register notes.

### Fixed
- `BME280::_nowMs()` now falls back to `millis()` when `Config::nowMs` is not injected, matching the documented behavior.

## [1.2.2] - 2026-04-03

### Added
- `inProgress()` convenience method on `Status` struct.
- `CommandHandler.h` example helper for serial command parsing (`cmd::readLine`, `cmd::match`, `cmd::parseInt`).
- `HealthDiag.h` example helper with verbose health diagnostics, color-coded output, snapshots, diffs, and `HealthMonitor` class for continuous monitoring.

### Changed
- `I2cScanner.h` upgraded: advanced table-format scan, bus recovery via `recoverBus()`, timeout support, yield() calls, common address hints, `LOG_SERIAL` macro usage.
- `I2cTransport.h` upgraded: `TwoWire*` via user pointer (no global Wire), null pointer checks, 128-byte buffer validation, detailed per-error-code mapping, `ARDUINO_ARCH_ESP32` guards.
- `BusDiag.h` updated to use `i2c_scanner::scan(Wire)` and include `<Wire.h>`.
- `Log.h`: added `LOGV` runtime-verbose macro, ESP32-S3 USB CDC delay in `log_begin()`, fixed include path.
- `BoardConfig.h`: fixed include path for `I2cTransport.h`.
- `main.cpp` example updated to use `bus_diag::scan()`, set `cfg.i2cUser = &Wire`, corrected include paths.

## [1.2.1] - 2026-03-01

### Changed
- Quick start and bringup example now set `Config.nowMs` explicitly for portable timing/health timestamps.
- `docs/IDF_PORT.md` and timing-guard policy now reflect zero direct Arduino timing calls in library core.

### Fixed
- Restored injected time source in core driver (`_nowMs()`); removed direct `millis()` usage from library internals.
- Forced-mode `requestMeasurement()` now tracks an already-running conversion as `IN_PROGRESS` instead of returning `BUSY`, preventing avoidable measurement-cycle timeouts.
- `getRawSample()` and `getCompensatedSample()` now return the latest cached sample even after `getMeasurement()` consumes the ready flag.

## [1.2.0] - 2026-03-01

### Changed
- Updated `docs/IDF_PORT.md` to reflect the actual timing abstraction and portability flow.

### Fixed
- Core timing guard compliance updates in `src/BME280.cpp`.

### Removed
- Stale auxiliary documentation templates not used by the current release flow.

## [1.1.1] - 2026-02-28

### Added
- Unified bringup helper layer under `examples/common/*` (`BusDiag`, `CliShell`, `HealthView`, `TransportAdapter`).
- `docs/IDF_PORT.md` and `docs/UNIFICATION_STANDARD.md` as baseline portability/unification references.
- CLI/timing contract validation scripts in `tools/`.

### Changed
- `examples/01_basic_bringup_cli` aligned to the shared I2C CLI command/help/reporting scheme.
- CI and native test layout standardized (`test/test_basic.cpp`, workflow/profile normalization).

## [1.1.0] - 2026-02-22

### Added
- `getStandbyTimeMs()` — returns configured standby interval in milliseconds (rounded up)
- `estimateNormalCycleMs()` — returns full normal-mode cycle time (measurement + standby)

### Fixed
- **`tick()` used `millis()` directly** — broke determinism; now uses only the caller-supplied `nowMs` parameter
- **`softReset()` polling used tracked reads** — during POR (~2 ms) the BME280 may NACK, which inflated health-failure counters and could abort reset prematurely; now uses raw reads and tolerates transient I2C errors
- **`setFilter()`/`setStandby()` masked original error** — if config-register write failed and restore-to-original-mode also failed, the restore error was returned instead of the root cause; restore is now best-effort
- **`end()` didn't put device to sleep** — device continued measuring in normal mode after shutdown; now sends best-effort sleep command via raw I2C before clearing state
- **`_compensate()` didn't handle skipped channels** — running compensation on sentinel ADC values when `osrsT/P/H == SKIP` produced garbage; now guards each channel and returns `COMPENSATION_ERROR` if temperature is skipped while P/H are enabled
- **`recover()` didn't re-apply configuration** — after a power glitch, device registers revert to defaults; `recover()` now calls `_applyConfig()` after successful probe

## [1.0.0] - 2026-01-20

### Added
- **First stable release** 🎉
- Complete BME280 driver with Bosch compensation formulas (32-bit/64-bit)
- Injected I2C transport architecture (no Wire dependency in library)
- Health monitoring with automatic state tracking (READY/DEGRADED/OFFLINE)
- Configurable oversampling (SKIP, X1, X2, X4, X8, X16) for T/P/H
- Configurable IIR filter coefficient (OFF, X2, X4, X8, X16)
- Configurable standby time for normal mode (0.5ms to 1000ms)
- Support for all measurement modes: Sleep, Forced, Normal
- Non-blocking tick-based architecture for async operations
- Soft reset with proper timeout handling
- Calibration data validation
- Raw and compensated sample access
- Measurement time estimation
- Register-level read/write access for diagnostics
- Basic CLI example (`01_basic_bringup_cli`)
- Comprehensive Doxygen documentation in public headers
- MIT License

## [0.1.0] - 2026-01-19

### Added
- Initial development version
- Production BME280 driver with injected I2C transport
- Health monitoring and tracked transport wrappers
- Basic CLI example (`01_basic_bringup_cli`)
- Doxygen-style documentation in public headers

[Unreleased]: https://github.com/janhavelka/BME280/compare/v1.5.0...HEAD
[1.5.0]: https://github.com/janhavelka/BME280/compare/v1.4.0...v1.5.0
[1.4.0]: https://github.com/janhavelka/BME280/compare/v1.3.0...v1.4.0
[1.3.0]: https://github.com/janhavelka/BME280/compare/v1.2.2...v1.3.0
[1.2.1]: https://github.com/janhavelka/BME280/compare/v1.2.0...v1.2.1
[1.2.0]: https://github.com/janhavelka/BME280/compare/v1.1.1...v1.2.0
[1.1.1]: https://github.com/janhavelka/BME280/compare/v1.1.0...v1.1.1
[1.1.0]: https://github.com/janhavelka/BME280/compare/v1.0.0...v1.1.0
[1.0.0]: https://github.com/janhavelka/BME280/releases/tag/v1.0.0
[0.1.0]: https://github.com/janhavelka/BME280/releases/tag/v0.1.0
