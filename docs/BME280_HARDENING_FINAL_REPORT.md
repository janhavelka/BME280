# BME280 Hardening Final Report

> Superseded historical report: this report captured an earlier hardening pass.
> For the current Phase 06 final state, validation results, and merge verdict,
> use `docs/BME280_INDUSTRY_STANDARD_FINAL_REPORT.md`.

Branch: `hardening/bme280-industry-readiness`

## Summary

This pass addressed the highest-risk BME280 gap: partial multi-register configuration writes could leave sensor hardware and cached settings divergent without diagnostics. The driver now exposes dirty hardware-config state, preserves probe/begin transport errors, and requires a real timebase for measurement scheduling.

## Public API / Behavior Changes

- `BME280` copy and move construction/assignment are now deleted.
- Added `hardwareConfigDirty()` and `hardwareConfigDirtyError()`.
- Added `SettingsSnapshot::hardwareConfigDirty` and `SettingsSnapshot::hardwareConfigDirtyError`.
- `requestMeasurement()` returns `INVALID_CONFIG` when `Config::nowMs` is missing. `begin()` still does not globally fail for a missing clock.
- `begin()` and `probe()` map only definite address NACK to `DEVICE_NOT_FOUND`; timeout, data NACK, bus, and generic I2C errors are preserved.

## Code Changes

- Added dirty-state tracking for `_applyConfig()`, `setOversamplingH()`, `setFilter()`, and `setStandby()` partial write failures.
- Dirty state preserves the first error that made hardware config uncertain.
- Dirty state is cleared only after a complete successful `_applyConfig()` resync through `begin()`, `recover()`, or `softReset()`.
- Added native fake-transport tests for copy/move prevention, no-clock measurement scheduling, begin/probe error mapping, partial config failures, dirty-state exposure, and dirty clearing after recovery.
- Added ESP-IDF example contract and ESP32-S2/S3 IDF build jobs to CI.
- Updated `AGENTS.md` and `README.md` with timebase, dirty-state, thread/ISR, transport ownership, and validation-honesty contracts.

## Tests And Checks Run

- `python tools/check_core_timing_guard.py`: pass
- `python tools/check_cli_contract.py`: pass
- `python tools/check_idf_example_contract.py`: pass
- `python scripts/generate_version.py check`: pass
- `python -m platformio test -e native`: pass, 41/41 test cases
- `python -m platformio run -e esp32s3dev`: pass
- `python -m platformio run -e esp32s2dev`: pass
- `python -m platformio pkg pack`: pass; generated tarball was removed after validation

## Not Run

- Local pure ESP-IDF build: not run because `idf.py` is not installed in this shell.
- Hardware validation: not run.

## Remaining Work

- Confirm new CI ESP-IDF jobs on GitHub.
- Add Bosch datasheet golden compensation vectors and more humidity calibration packing edge cases.
- Decide whether config changes should invalidate cached samples or tag samples with a config generation.
- Preserve NVM polling transport errors more precisely during initialized `softReset()` paths.
- Record hardware validation for chip ID, calibration plausibility, forced/normal timing, reset/NVM behavior, and representative I2C fault paths.

## Readiness Assessment

Closer to industry-grade and materially safer around partial hardware state. Merge should be gated on CI ESP-IDF confirmation and hardware validation before production claims.
