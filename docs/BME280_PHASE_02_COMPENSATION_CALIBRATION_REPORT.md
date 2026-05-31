# BME280 Phase 02 Compensation and Calibration Report

Date: 2026-05-31
Branch: `hardening/bme280-industry-readiness`
Baseline commit: `5c0cf27960ad237af7382b1bd2ee745f79edd91f`

## Scope

Phase 02 inspected and hardened BME280 calibration parsing, raw burst decoding,
integer compensation, skipped-channel handling, cached-sample validity, and
native golden-vector coverage. No hardware validation was run. No local ESP-IDF
build or target execution was run.

## Start State

- `git status --short`: clean
- `git branch --show-current`: `hardening/bme280-industry-readiness`
- `git log --oneline -8` started at:
  - `5c0cf27 hardening: lock BME280 baseline facts`
  - `a750935 hardening: lock BME280 readiness facts`
  - `0fd07f8 feat: enhance BME280 driver with hardware config dirty state tracking and ESP-IDF CI support`

## Review Roles

The requested Phase 02 roles inspected actual repository files before changes
were accepted:

- `compensation-agent` (`019e7cf5-0c37-7e32-aec0-c722e59c250e`)
- `bme280-datasheet-agent` (`019e7cf5-43cf-7cb1-bcc5-32c97bcc4951`)
- `fault-injection-agent` (`019e7cf5-7258-7a43-8211-bc4c61eb3fa7`)
- `core-contracts-agent` (`019e7cf5-a583-7620-8617-64429eb44982`)
- `integration-review-agent` (`019e7d19-84df-7fd1-8e4d-c1a88f9bac38`)

Key findings accepted:

- Existing calibration parsing matched the Bosch layout, including signed H4/H5
  12-bit sign extension, but tests did not prove it.
- The fake bus hard-coded calibration bytes, preventing meaningful coefficient,
  H4/H5, invalid `dig_P1`, and raw-vector tests.
- Raw skipped sentinels were only visible in examples and not modeled in the
  core sample contract.
- Numeric zero outputs for skipped pressure/humidity were ambiguous without
  validity flags.
- Cached samples needed invalidation after successful typed configuration
  changes to avoid reuse under a new measurement configuration.
- Integration review found and fixed one weak skipped-sentinel assertion. It
  also confirmed the remaining blocker is the unresolved Arduino PlatformIO
  build caused by user-cache permissions.

## Changes Made

- Added public Bosch skipped-sentinel constants:
  - `cmd::RAW_PRESSURE_SKIPPED == 0x80000`
  - `cmd::RAW_TEMPERATURE_SKIPPED == 0x80000`
  - `cmd::RAW_HUMIDITY_SKIPPED == 0x8000`
- Appended per-channel validity flags to `Measurement`, `RawSample`, and
  `CompensatedSample`.
- Derived raw validity at the burst-read boundary from both configured
  oversampling and Bosch skipped sentinels.
- Kept skipped pressure/humidity as successful samples with zero numeric
  compatibility fields and `pressureValid` / `humidityValid` false.
- Rejected enabled-channel raw skipped sentinels during compensation so they do
  not become plausible compensated values.
- Preserved Bosch integer compensation behavior:
  - temperature first, producing `t_fine`
  - 64-bit pressure path
  - pressure denominator zero guard
  - humidity clamp to `0..100%RH`
- Invalidated cached samples after successful typed configuration changes.
- Made the native fake bus calibration reads register-driven.
- Updated README, Doxygen comments, changelog, and example skipped-sentinel
  diagnostics.

## Golden Vector Provenance

The Phase 02 vectors are synthetic and datasheet-derived. They are not claimed
as hardware validation and are not claimed as an official Bosch golden-vector
suite.

- Calibration parser vectors use synthetic register bytes chosen to exercise
  unsigned, signed, signed-boundary, and H4/H5 nibble-packing cases.
- Raw burst vector bytes `AB CD EF 54 32 1F BE EF` prove pressure `0xABCDE`,
  temperature `0x54321`, humidity `0xBEEF`, and ignored XLSB low nibbles.
- Compensation vector expected values were generated from an independent
  transcription of the Bosch integer formulas:
  - calibration: `T1=27504`, `T2=26435`, `T3=-1000`, `P1=36477`,
    `P2=-10685`, `P3=3024`, `P4=2855`, `P5=140`, `P6=-7`, `P7=15500`,
    `P8=-14600`, `P9=6000`, `H1=75`, `H2=362`, `H3=0`, `H4=325`,
    `H5=50`, `H6=30`
  - raw: `adcT=519888`, `adcP=415148`, `adcH=30000`
  - expected: `t_fine=128422`, `tempC_x100=2508`,
    `pressurePa=100653`, `humidityPct_x1024=51941`
- Humidity clamp fixtures use the same calibration with `adcH=0` for the low
  clamp and `adcH=40000` for the high clamp.
- Pressure denominator guard fixture is synthetic and chosen so the Bosch
  pressure denominator evaluates to zero without relying on invalid `dig_P1`.

## Native Test Coverage Added

- `test_calibration_parses_bosch_synthetic_coefficients`
- `test_calibration_parses_signed_boundaries_and_humidity_nibbles`
- `test_invalid_pressure_calibration_is_rejected`
- `test_read_calibration_raw_uses_register_bytes_and_preserves_error`
- `test_raw_burst_reconstructs_20_and_16_bit_samples`
- `test_compensation_matches_datasheet_derived_synthetic_vector`
- `test_humidity_compensation_clamps_low_and_high`
- `test_skipped_sentinels_are_explicit_validity_flags`
- `test_enabled_raw_sentinel_rejects_compensated_sample`
- `test_pressure_compensation_divide_by_zero_guard_blocks_sample`
- `test_config_change_invalidates_cached_samples`

## Validation Results

| Check | Result | Exact observed result |
| --- | --- | --- |
| `python tools/check_core_timing_guard.py` | PASS | `Core timing guard PASSED` |
| `python tools/check_cli_contract.py` | PASS | `CLI contract PASSED` |
| `python tools/check_idf_example_contract.py` | PASS | `IDF example contract PASSED` |
| `python scripts/generate_version.py check` | PASS | `Up to date: ...\\include\\BME280\\Version.h` |
| `python -m platformio test -e native` | PASS | `52 test cases: 52 succeeded` |
| `python -m platformio run -e esp32s3dev -e esp32s2dev` | FAIL | `PermissionError: [Errno 13] Permission denied: 'C:\\Users\\HonzovoSpectre\\.platformio\\platforms.lock'`; escalated rerun requests were aborted |
| Hardware validation | NOT RUN | No physical BME280 hardware was exercised |
| Local ESP-IDF build/target validation | NOT RUN | Only the repo-local IDF example contract script was run |

## Residual Notes

- The public sample structs gained appended validity fields. This is intended to
  preserve normal source compatibility while making skipped channels explicit,
  but it is still a public layout change and should be treated as a minor
  release item when the project cuts a release.
- The Arduino PlatformIO build failure was environmental permission state in
  the user PlatformIO cache, not a compiler failure from this patch. The build
  must be rerun with PlatformIO cache access before claiming Arduino build
  validation for this phase.
