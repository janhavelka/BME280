# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- Nothing yet

### Changed
- Nothing yet

### Deprecated
- Nothing yet

### Removed
- Nothing yet

### Fixed
- Nothing yet

### Security
- Nothing yet

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

[Unreleased]: https://github.com/janhavelka/BME280/compare/v1.0.0...HEAD
[1.0.0]: https://github.com/janhavelka/BME280/releases/tag/v1.0.0
[0.1.0]: https://github.com/janhavelka/BME280/releases/tag/v0.1.0
