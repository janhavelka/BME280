# BME280 Industry Hardening Summary

Last updated: 2026-06-01

This document is the maintained summary for the merged industry-readiness work.
It replaces the temporary prompt, phase, and self-test reports that were created
while that work was being built.

## Scope

The hardening work prepares the BME280 library for production-style embedded
use on ESP32-S2 and ESP32-S3 with Arduino/PlatformIO and ESP-IDF consumers.

No physical BME280 hardware validation is claimed here. Local ESP-IDF `idf.py`
validation is claimed only when the exact commands are run and recorded.

## Active Documentation Set

- `README.md`: public usage, API, build, validation, and limitations.
- `CHANGELOG.md`: release-facing change history.
- `AGENTS.md`: repository engineering rules.
- `CONTRIBUTING.md`: contribution workflow.
- `docs/README.md`: map of maintained docs, source evidence, and local
  artifacts.
- `docs/IDF_PORT.md`: ESP-IDF component/example boundary and validation notes.
- `docs/BME280_Register_Reference.md`: register and bitfield reference.
- `docs/BME280_HARDWARE_VALIDATION_MATRIX.md`: hardware result ledger.
- `docs/I2C_HIL_RUNBOOK.md`: serial HIL procedure.
- `docs/I2C_HIL_TARGET_TEMPLATE.md`: per-target HIL evidence template.
- `docs/BME280_datasheet.pdf` and extracted markdown: source evidence.

The old prompt files, intermediate hardening reports, split ESP-IDF
implementation note, and self-test reports are no longer active documentation.

## What Changed

Core and API contracts:

- The core stays framework-neutral: no Arduino, Wire, ESP-IDF, FreeRTOS, log, or
  delay APIs in `include/` or `src/`.
- I2C remains injected through `Config`; the library does not own pins, buses,
  locks, or timeout policy.
- Driver objects are non-copyable and non-movable.
- Public error reporting keeps distinguishable transport errors when the
  adapter can provide them.
- Measurement scheduling now requires an injected timebase for fresh samples.

Measurement and compensation:

- Calibration parsing, raw burst reconstruction, Bosch-style compensation, and
  humidity clamp behavior have native test coverage.
- Raw and compensated sample structs expose per-channel validity flags.
- Bosch skipped-channel sentinel constants are named in the public command
  table.
- Configuration changes invalidate cached samples so callers do not reuse data
  captured under older settings.

Configuration, reset, and recovery:

- Multi-register config paths expose `hardwareConfigDirty()` and the original
  dirty-state error when hardware and cache may diverge.
- Dirty state is cleared only after a complete successful config resync through
  `begin()`, `recover()`, or `softReset()`.
- `setFilter()` and `setStandby()` avoid config writes while the sensor reports
  `measuring`.
- Reset and NVM polling are bounded and preserve useful root-cause errors where
  possible.

Examples, ESP-IDF, and CI:

- The Arduino bring-up CLI and native ESP-IDF CLI share the same command
  contract without sharing Arduino source in IDF builds.
- The ESP-IDF example uses `app_main`, `driver/i2c_master.h`, `esp_timer`, and
  fixed C command buffers.
- Guard scripts check core framework neutrality, CLI parity, IDF example
  boundaries, HIL runner/docs consistency, generated version state, and package
  contents.
- CI is configured for PlatformIO Arduino builds, native tests, package checks,
  and ESP-IDF example builds.

HIL preparation:

- `docs/BME280_HARDWARE_VALIDATION_MATRIX.md` is the hardware result ledger.
- `docs/I2C_HIL_RUNBOOK.md` describes the serial HIL procedure.
- `docs/I2C_HIL_TARGET_TEMPLATE.md` captures per-target setup and evidence.
- `tools/run_i2c_hil.py` captures serial transcripts and summaries.

## Validation Boundary

Software checks can show that the driver builds, tests pass, examples compile,
and docs/tool contracts are coherent. They do not prove wiring, pull-up values,
bus margin, sensor accuracy, humidity handling, long soak behavior, or physical
fault recovery.

Before any hardware row is changed from `NOT RUN`, record board, sensor module,
wiring, address, pull-ups, voltage rails, command transcript, environmental
reference, result, and operator notes in the hardware validation matrix or an
attached HIL artifact package.

Generated Doxygen output is a local artifact. Publish API docs only from a
release commit where the documented public API matches `library.json` and
`include/BME280/Version.h`.

## Current HIL Baseline

The intended first HIL software baseline is the commit used to build the test
firmware. Record the exact `git rev-parse HEAD` value in the HIL target
template before flashing.

`scan` is only I2C ACK evidence. Use `chipid` or `reg 0xD0` reading `0x60` as
the BME280 identity check.

The CLI does not support counted commands such as `read 10` or `read 20`.
Use repeated `read` commands for normal-mode evidence and `stress N` only as a
forced-measurement stress substitute.

## Release Gate

- Merge status: the hardening work is present in the maintained release
  history. Check the current branch and release tag before publishing new
  evidence.
- Release status: version metadata, changelog entries, generated version
  header, and Doxygen project metadata must be aligned on the exact release
  commit before tagging. Publishing still requires the pushed tag and
  successful CI for that commit.
- Hardware status: not complete until `BME280_HARDWARE_VALIDATION_MATRIX.md` or
  an attached HIL artifact package records real board, wiring, transcript,
  reference, fault, and soak evidence. The software release does not claim
  completed hardware validation.
