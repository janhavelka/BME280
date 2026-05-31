# BME280 Industry-Standard Final Report

Date: 2026-05-31
Branch: `hardening/bme280-industry-readiness`
Base reviewed: `origin/main` at `b3bc2151b14891736878f42cc6238c58cf772817`
Final range: `b3bc2151b14891736878f42cc6238c58cf772817..HEAD`

This is the authoritative Phase 06 final report. It supersedes earlier
historical summaries that predate Phase 05 or Phase 06.

## Executive Summary

The BME280 library is now software-hardened for a production-oriented embedded
use case: framework-neutral core, injected non-owning I2C transport, explicit
status errors, bounded reset/NVM handling, datasheet-aware configuration
sequencing, sample validity tracking, health/offline diagnostics, Arduino and
native ESP-IDF examples, CI definitions, and package validation.

The branch is ready for pull-request review and current-head CI. It should not
be described as hardware-qualified, field-proven, or locally ESP-IDF validated:
no physical BME280 hardware validation was run in this phase, and `idf.py` was
not installed locally.

Merge verdict: conditional PASS for software integration. Merge after current
branch/PR CI passes. Release claims must remain constrained until the hardware
validation matrix is executed on real boards and sensors.

## Phase 06 Agent Review

Six read-only review agents were spawned/emulated for the final pass.

| Agent | Result |
| --- | --- |
| integration-review-agent | No core blockers. Found stale historical reports and missing final report before this pass. Confirmed core neutrality, transport injection, datasheet-sensitive sequencing, and honest hardware wording. |
| docs-hardware-agent | No hardware overclaim found. Confirmed hardware validation matrix remains `NOT RUN`. Requested the final report and superseding stale historical reports. |
| idf-ci-agent | Confirmed CI coverage for PlatformIO S2/S3, native tests, package validation, guards, and ESP-IDF v6.0.1 S2/S3 builds. Noted `idf.py` and `gh` unavailable locally. |
| fault-injection-agent | Native fault suite passed 84/84. Coverage includes transport failures, dirty state, reset/NVM, recover/offline, measurement, and example transport mapping. |
| bme280-datasheet-agent | No datasheet-contract blockers. Verified chip ID, register constants, ctrl_hum latch ordering, config/mode behavior, raw burst read, calibration parsing, compensation, and timing. |
| core-contracts-agent | No core-contract blockers. Verified framework neutrality, non-owning transport, Status-returning fallible APIs, timebase docs, copy/move deletion, thread/ISR docs, and dirty diagnostics. |

## Implemented Hardening By Phase

| Phase | Commit(s) | Primary outcome |
| --- | --- | --- |
| 00/01 | `a750935`, `5c0cf27` | Workflow/baseline fact lock and repository truth source. |
| 02 | `1c0d409` | Calibration parsing, Bosch-style compensation, sample validity, and golden-vector style native tests. |
| 03 | `efecfe8`, `70c5ab9` | Config sequencing, timebase and measurement scheduling semantics, sample cache semantics, and phase report updates. |
| 04 | `3dd72de` | Reset/NVM polling, recovery, dirty hardware config diagnostics, and fault-path precision. |
| 05 | `bdab940` | Arduino/ESP-IDF example hardening, CI docs, packaging checks, README honesty, and hardware matrix. |
| 06 | `HEAD` | Final integration review, stale-report superseding, package checker tightening for ESP-IDF packaging files, and this final report. |

## Public API And Behavioral Contracts

- Core/public headers and `src/` remain framework-neutral. No Arduino, Wire,
  ESP-IDF, FreeRTOS, logging, or delay APIs were found in `include/` or `src/`.
- I2C is injected through `Config` callbacks; the driver stores address,
  timeout, callback pointers, and user context, but no pins or bus handles.
- Transport errors are preserved as `Status` values. Definite address NACK maps
  to `DEVICE_NOT_FOUND`; timeout, data NACK, bus, and generic I2C errors remain
  distinguishable.
- Public fallible APIs return `Status`, except explicitly documented best-effort
  behavior such as `end()` and `tick()` status reporting through
  `lastMeasurementStatus()`.
- Driver instances are non-copyable and non-movable.
- Health state is tracked through raw/tracked wrapper layering; validation and
  precondition errors do not pollute I2C health.
- Dirty hardware config diagnostics are exposed and are cleared only after a
  successful full resync/recover/reset path. Manual diagnostic raw register
  writes are tracked for health but intentionally do not mark dirty state.
- `requestMeasurement()` requires an application timebase. `tick(nowMs)` uses
  the caller-provided monotonic time and has wraparound coverage.
- Datasheet-sensitive behavior is enforced: chip ID `0x60`, addresses `0x76`
  and `0x77`, reset command `0xB6` to `0xE0`, bounded `im_update` polling,
  ctrl_hum then ctrl_meas latch ordering, config writes outside normal-mode
  ignored-write windows, and one coherent `0xF7..0xFE` data burst.

## Local Validation Results

Primary workspace commands were run on 2026-05-31.

| Check | Result | Evidence |
| --- | --- | --- |
| `git status --short` at Phase 06 start | PASS | Clean. |
| `python tools/check_core_timing_guard.py` | PASS | `Core timing guard PASSED`. |
| `python tools/check_cli_contract.py` | PASS | `CLI contract PASSED`. |
| `python tools/check_idf_example_contract.py` | PASS | `IDF example contract PASSED`. |
| `python scripts/generate_version.py check` | PASS | `include/BME280/Version.h` up to date. |
| `rg -n "Arduino\.h|Wire\.h|Serial|String|TwoWire|driver/i2c|freertos|esp_timer|vTaskDelay|delay\(" include src` | PASS | No matches. |
| `python -m platformio project config --json-output` | PASS | Native env has `framework=[]` and `lib_deps=[]`. |
| `python -m platformio test -e native` | PASS | 84 tests, 84 passed, duration `00:00:21.377`. |
| `python -m platformio run -e esp32s3dev` | PASS after clean | Final clean rerun passed in `00:00:29.992`. Earlier local attempts failed while compiling Arduino framework objects from `.pio` state (`esp32-hal-*`, `ColorFormat.c.o`, `Stream.cpp.o`) with no BME280 source diagnostic. |
| `python -m platformio run -e esp32s2dev` | PASS | Passed in `00:00:28.306`. |
| `python -m platformio pkg pack` | PASS | Wrote `BME280-1.5.0.tar.gz`. |
| `python tools/check_package_contents.py` | PASS | `Package contents PASSED (BME280-1.5.0.tar.gz)`. Archive removed after validation. |
| `idf.py --version` | FAIL | `idf.py` is not installed locally. No local ESP-IDF build was run. |
| `gh --version` | FAIL | GitHub CLI is not installed locally. |

## CI Status

CI configuration exists for:

- PlatformIO Arduino builds: `esp32s3dev`, `esp32s2dev`.
- Native PlatformIO tests and core timing guard.
- CLI contract, ESP-IDF example contract, generated version header, package
  packing, and package content validation.
- ESP-IDF example builds through `espressif/esp-idf-ci-action@v1` for
  `esp32s3` and `esp32s2` with `esp_idf_version: v6.0.1`.

Current-branch CI was not proven in this Phase 06 run. The workflow triggers on
pushes to `main` and pull requests targeting `main`; no open PR existed for
`hardening/bme280-industry-readiness`, and the GitHub API reported zero Actions
runs for this branch. The latest observed `main` CI run was successful for
`b3bc2151b14891736878f42cc6238c58cf772817` at:

`https://github.com/janhavelka/BME280/actions/runs/26627761400`

That CI result predates this branch's hardening commits and must not be treated
as validation of the current branch head.

## Hardware Validation Status

Hardware validation was NOT RUN.

No ESP32-S2 board, ESP32-S3 board, physical BME280 module, serial transcript,
fault-injection wiring, humidity reference, pressure/temperature reference, or
soak test was exercised in this Phase 06 run. The hardware validation matrix
remains an execution plan and result ledger, not a completed validation record.

Allowed claim: software architecture, native tests, local PlatformIO Arduino
builds, packaging, and CI configuration have been validated as listed above.

Forbidden claim from this work: field validation, hardware qualification,
hardware soak, humidity accuracy validation, physical bus fault validation, or
local pure ESP-IDF validation.

## Remaining Risks And Gaps

- Current branch CI must be run through a PR or adjusted workflow trigger before
  final merge.
- Local `idf.py` validation is unavailable on this machine.
- Physical hardware validation is still pending and should record board model,
  sensor module, wiring, pullups, supply voltage, SDO/CSB state, command
  transcript, environmental reference, duration, and exact pass/fail result.
- Native fault tests cover the main software fault paths, but remaining low-risk
  gaps include an explicit `begin()` address-NACK mapping test, single-I2C-call
  partial mutation modeling, FakeBus address discrimination, a focused
  `_waitForNvmReady()` wrap/deadline branch, and a dedicated recover/cache
  preservation assertion.
- Extracted historical docs contain a few stale internal notes and are excluded
  from Doxygen. Public README/API docs and this report are the current sources.

## Merge And Release Verdict

Software merge verdict: conditional PASS.

The branch is suitable to open as a PR and run current-head CI. Merge should
wait for that CI to pass. If the project requires hardware-qualified release
language, the next required work is to execute and commit the hardware
validation matrix results. Until then, release notes should say the library is
software-hardened and CI-configured, with hardware validation pending.

Next action: open a PR against `main` to trigger CI, then run the hardware
validation matrix on real ESP32-S2/ESP32-S3 plus BME280 hardware before making
field-validation claims.
