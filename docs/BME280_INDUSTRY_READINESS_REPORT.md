# BME280 Industry Readiness Audit

Branch: `hardening/bme280-industry-readiness`

Note: this is a historical baseline audit. Several findings about dirty
configuration diagnostics, compensation vectors, and reset/NVM fault handling
are superseded by the dedicated Phase 01-04 reports.

## Executive Summary

The BME280 library has a clean framework-neutral core, injected/non-owning I2C transport, chip-ID validation, calibration parsing, health tracking, and a real native ESP-IDF example. It is not yet fully industry-grade because multi-register configuration writes can leave hardware and cached settings divergent without an explicit dirty diagnostic, ESP-IDF builds are not covered in CI/local validation, and several device-specific fault tests are missing.

Readiness classification: good architecture foundation, but needs partial-state diagnostics and stronger validation before production merge.

## Baseline Checks

- `python tools/check_core_timing_guard.py`: pass
- `python tools/check_cli_contract.py`: pass
- `python tools/check_idf_example_contract.py`: pass
- `python scripts/generate_version.py check`: pass
- `python -m platformio test -e native`: pass, 33 test cases
- ESP-IDF build: not yet run in this pass.
- Hardware validation: not run in this audit.

## Scorecard

| Area | Status | Notes |
| --- | --- | --- |
| Core framework neutrality | Mostly ready | Core uses no Arduino/ESP-IDF headers. |
| I2C ownership | Mostly ready | Transport callbacks and opaque user context are injected. |
| Timing contracts | Partial | Measurement timing is explicit; missing `nowMs` behavior needs clearer contract. |
| Status/error precision | Partial | Begin/probe collapse transport failures into `DEVICE_NOT_FOUND`. |
| Health/recovery | Mostly ready | Tracked wrappers and offline behavior are covered by tests. |
| Partial hardware state | Not ready | Multi-register config writes lack dirty-state exposure. |
| Device correctness | Mostly ready | Chip ID, calibration parsing, forced/normal timing, and 64-bit pressure path exist. |
| Tests | Partial | Missing partial-write dirty-state and golden compensation vectors. |
| Docs/examples | Partial | README has IDF commands, but CI does not prove them. |

## Strengths

- Core/public headers are framework-neutral.
- BME280 chip ID is checked.
- Calibration register parsing includes humidity packing.
- Measurement timing is estimated from oversampling settings.
- Raw data is burst-read in one transaction.
- Native tests cover health/recovery, NVM wait, forced/normal timing, offline latch, and Arduino adapter mapping.

## High Findings

1. `_applyConfig()`, `setOversamplingH()`, `setFilter()`, and `setStandby()` write multiple hardware registers; later failures can leave hardware partly updated while cached settings remain old.
2. CI does not build the pure ESP-IDF example even though ESP-IDF support is advertised.

## Medium Findings

- `Config::nowMs` is optional and fallback time is constant zero; measurement APIs should document this clearly or reject missing clocks where freshness depends on it.
- `softReset()` NVM polling uses raw I2C and can collapse repeated transport errors into timeout.
- Cached samples remain available across configuration changes; consumers must treat the timestamp/config context carefully.
- Copy/move construction is not explicitly disabled for the driver object.
- Begin/probe do not preserve distinct timeout/bus/generic transport errors.

## Recommended Remediation Plan

- Add `hardwareConfigDirty` diagnostics and preserve the original status that caused dirty state.
- Clear dirty state only after a complete config resync and read-back verification.
- Preserve probe/begin transport errors except definite address NACK.
- Disable copy/move and add compile-time tests.
- Add CI coverage for IDF contract and ESP32-S2/S3 pure IDF builds.
- Document sample freshness, thread/ISR limits, and remaining hardware validation.

## Exit Criteria For Industry Grade

- Native fake-transport tests cover partial writes at each config step, dirty clearing, probe mapping, and copy/move prevention.
- Native tests and PlatformIO Arduino builds pass.
- Pure ESP-IDF example builds locally or in CI for ESP32-S2/S3.
- Hardware validation records chip ID, calibration plausibility, forced/normal timing, reset/NVM behavior, and representative I2C fault paths.
