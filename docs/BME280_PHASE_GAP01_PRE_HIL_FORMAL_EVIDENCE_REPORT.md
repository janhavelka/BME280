# BME280 Gap 01 Pre-HIL Formal Evidence Report

Historical prompt report. This file records a completed gap-closure pass and is
not current user documentation or hardware validation evidence. Use
`README.md`, `docs/README.md`, `docs/I2C_HIL_RUNBOOK.md`, and
`docs/BME280_HARDWARE_VALIDATION_MATRIX.md` for maintained guidance.

Date: 2026-06-01

## Scope

This pass closes pre-HIL evidence and tooling gaps only. It does not run
physical HIL, change core driver behavior, change the public API, or claim
hardware validation.

## Gaps Fixed

- Forced-mode sleep-return evidence was missing from the default HIL sequence.
  The runner now records post-`force` `reg 0xF4`, parses `ctrl_meas[1:0]`, and
  fails the command if the mode bits are not `00` sleep. It also records a
  post-`force` `status` command requiring `measuring=0`.
- HIL setup/evidence fields were aligned across the hardware matrix, runbook,
  and target template. The docs now consistently require firmware version,
  library version, git commit, dirty flag, runner command/arguments, board and
  MCU target, sensor module, rails, pull-ups, SDO/CSB, pins, bus speed, serial
  port, environmental references/readings/tolerances, exact transcript, runner
  verdict, and operator sign-off.
- The package checker no longer relies on suffix-only matching for required
  files. It validates package-root-normalized exact paths and derives required
  ESP-IDF example sources and local quoted headers from the IDF example CMake
  and includes.
- CI already runs `python tools/check_hil_contract.py` in the
  `validate-library` job next to the CLI and ESP-IDF contract checks. No
  physical hardware is required in CI.

## Runner And Docs Changes

Default HIL sequence additions:

```text
force
reg 0xF4
status
read
```

No new CLI command was added. `reg 0xF4` and `status` are existing diagnostic
commands in both Arduino and ESP-IDF CLIs.

The runner summary artifacts now include the runner command and argument list,
and `summary.md` lists its own markdown path alongside the transcript, JSON,
and operator checklist.

## Package Checker Changes

`tools/check_package_contents.py` now requires exact package-relative paths for:

- `examples/idf/basic/CMakeLists.txt`
- `examples/idf/basic/main/CMakeLists.txt`
- every source listed in `examples/idf/basic/main/CMakeLists.txt`
- local quoted headers included by those IDF example sources, including
  `examples/idf/basic/main/IdfI2cTransport.h`

This specifically covers the required IDF transport files:

- `examples/idf/basic/main/IdfI2cTransport.cpp`
- `examples/idf/basic/main/IdfI2cTransport.h`

## CI Changes

`.github/workflows/ci.yml` already contains:

```text
python tools/check_hil_contract.py
```

in the `validate-library` job between the CLI and ESP-IDF contract checks.
This was verified and left unchanged.

## Checks

Final check results are recorded after execution:

| Check | Result |
| --- | --- |
| `python tools/check_hil_contract.py` | PASS: `HIL contract PASSED` |
| `python tools/check_cli_contract.py` | PASS: `CLI contract PASSED` |
| `python tools/check_idf_example_contract.py` | PASS: `IDF example contract PASSED` |
| `python tools/check_core_timing_guard.py` | PASS: `Core timing guard PASSED` |
| `python scripts/generate_version.py check` | PASS: `Version.h` up to date |
| `python -m platformio test -e native` | PASS: 88 test cases, 88 succeeded in 5.245 seconds |
| `python -m platformio run -e esp32s3dev` | PASS: `esp32s3dev` succeeded in 26.010 seconds |
| `python -m platformio run -e esp32s2dev` | PASS: `esp32s2dev` succeeded in 18.399 seconds |
| `python -m platformio pkg pack` | PASS: wrote `BME280-1.6.1.tar.gz` |
| `python tools/check_package_contents.py` | PASS: `Package contents PASSED (BME280-1.6.1.tar.gz)` |
| Negative package simulation without `examples/idf/basic/main/IdfI2cTransport.h` | PASS: checker failed with `missing required files: examples/idf/basic/main/IdfI2cTransport.h` |
| Restored package re-check | PASS: `Package contents PASSED (BME280-1.6.1.tar.gz)` |
| `git diff --check` | PASS: exit code 0; Git printed CRLF replacement warnings for dirty files |
| Pure ESP-IDF example builds with `idf.py` | NOT RUN: `idf.py` was not available on `PATH` in this shell |

The generated `BME280-1.6.1.tar.gz` archive was removed after package
validation and negative-package simulation.

## Remaining HIL-Only Risks

- No physical BME280 HIL was run in this pass.
- Electrical wiring, pull-up values, rail levels, bus margin, SDO/CSB wiring,
  and fault injection remain unvalidated until a protected bench run records
  real evidence.
- Environmental accuracy and humidity handling remain unvalidated until
  reference instruments, tolerances, timestamps, and operator sign-off are
  recorded.
- Long normal-mode soak evidence still requires repeated normal-mode reads with
  timestamps and references; `stress N` remains only a forced-measurement stress
  substitute.
