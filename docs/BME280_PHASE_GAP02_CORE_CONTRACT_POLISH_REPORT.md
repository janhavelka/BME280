# BME280 Phase GAP02 Core Contract Polish Report

Historical prompt report. This file records a completed gap-closure pass and is
not current user documentation or hardware validation evidence. Use
`README.md`, `docs/README.md`, `docs/I2C_HIL_RUNBOOK.md`, and
`docs/BME280_HARDWARE_VALIDATION_MATRIX.md` for maintained guidance.

Date: 2026-06-01

Branch: `hardening/bme280-industry-gap-closure`

Starting HEAD: `499bdaaec84ae99e82da552fb94ed15116df8ff7`

Scope: Prompt 02 only. No HIL was run. No release version metadata was changed.

## Starting Worktree

`git status --short` reported one expected untracked file from the Prompt 00
baseline recovery request:

- `docs/BME280_GAP_CLOSURE_BASELINE_REPORT.md` - left untracked and excluded
  from this Prompt 02 commit.

## Behavior Decisions

### Diagnostic Raw Register Writes

Policy A was implemented.

Public diagnostic `writeRegister()` and `writeRegisters()` remain health-tracked
raw register writes. Writes that overlap these config-affecting registers now
mark `hardwareConfigDirty()`:

- `0xE0` reset
- `0xF2` ctrl_hum
- `0xF4` ctrl_meas
- `0xF5` config

Successful diagnostic writes mark dirty with `Err::INVALID_CONFIG` and the
starting register in `Status::detail`. Transport failures that may have reached
the device, such as data NACK, timeout, bus error, or generic I2C error, mark
dirty while preserving the original transport status. Address NACK does not mark
dirty because the write did not reach the target device.

Internal typed config/reset paths were changed to use private `writeRegs()` so
normal typed setters do not self-mark dirty on successful writes.

### Health Counters

Policy B was selected.

The existing behavior is production-safe: `begin()` starts a new operational
session and resets tracked I2C success/failure counters. README and public
Doxygen now describe `totalFailures()` and `totalSuccess()` as current
health-session counters since the most recent `begin()`, not lifetime object
counters.

### NVM Polling Without `nowMs`

The poll-count fallback was kept.

`begin()` still does not require `Config::nowMs`. NVM polling is bounded by a
10 ms deadline only when `Config::nowMs` supplies an advancing monotonic clock;
otherwise the framework-neutral fallback is bounded by the 255-poll cap and the
transport timeout supplied to each status read.

### Sample Freshness After Recovery/Reset

Option A was implemented.

Successful `recover()` now invalidates cached raw/compensated samples and any
pending measurement state after the full resync succeeds. Failed recovery leaves
pre-existing cached samples unchanged. `softReset()` already invalidated sample
cache before attempting reset; tests now lock that behavior down.

## Files Changed

- `src/BME280.cpp`
  - Added diagnostic dirty-state detection for raw config/control/reset writes.
  - Kept typed config/reset paths on private register writes.
  - Invalidated cached samples after successful `recover()`.
- `include/BME280/BME280.h`
  - Updated Doxygen for raw diagnostic write dirty semantics, recovery/reset
    cache invalidation, health-session counters, and NVM timing bounds.
- `test/test_basic.cpp`
  - Added native tests for diagnostic dirty semantics, session counter resets,
    NVM deadline versus no-clock poll-cap behavior, and recovery/reset sample
    invalidation.
- `README.md`
  - Updated public behavior docs for health counters, raw writes, NVM bounds,
    recovery/reset cache invalidation, and CLI raw-write guidance.
- `docs/BME280_INDUSTRY_HARDENING_SUMMARY.md`
  - Updated maintained hardening summary with Prompt 02 core-contract decisions.
- `CHANGELOG.md`
  - Added `[Unreleased]` entries for Prompt 01 and Prompt 02 changes without a
    release version bump.
- `examples/01_basic_bringup_cli/main.cpp`
  - Updated CLI help and recovery note wording.
- `examples/idf/basic/main/main.cpp`
  - Updated matching native IDF CLI help and recovery note wording.

## Checks Run

- `python tools/check_core_timing_guard.py` - PASS: `Core timing guard PASSED`
- `python tools/check_cli_contract.py` - PASS: `CLI contract PASSED`
- `python tools/check_idf_example_contract.py` - PASS: `IDF example contract PASSED`
- `python tools/check_hil_contract.py` - PASS: `HIL contract PASSED`
- `python scripts/generate_version.py check` - PASS: `Version.h` up to date
- `python tools/check_release_metadata.py` - PASS: release metadata remains at `1.6.1`
- `python -m platformio test -e native` - PASS: 97 test cases, 97 succeeded
- `python -m platformio run -e esp32s3dev` - PASS: `esp32s3dev SUCCESS`
- `python -m platformio run -e esp32s2dev` - PASS: `esp32s2dev SUCCESS`
- `git diff --check` - PASS: no whitespace errors; Git emitted expected CRLF
  working-copy warnings only

## Remaining Work for Prompt 03

- Run formal HIL only when real hardware is connected and the operator
  explicitly requests it.
- Populate target-specific hardware evidence with board/module identity, wiring,
  rails, pull-ups, SDO/CSB state, command transcript, environmental references,
  and operator sign-off.
- Keep hardware matrix rows at `NOT RUN` until physical validation evidence is
  actually captured.
- Perform release-version metadata work only in a dedicated release prompt.
