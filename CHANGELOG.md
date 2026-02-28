# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

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

[Unreleased]: https://github.com/janhavelka/BME280/compare/v1.1.0...HEAD
[1.1.0]: https://github.com/janhavelka/BME280/compare/v1.0.0...v1.1.0
[1.0.0]: https://github.com/janhavelka/BME280/releases/tag/v1.0.0
[0.1.0]: https://github.com/janhavelka/BME280/releases/tag/v0.1.0
