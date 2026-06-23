# AI Coder Prompt: BME280 Production-Grade Closure Pass

You are working inside the BME280 repository. The goal is to close the remaining
substantial gaps before the library can honestly be called production-grade.

Prefer simple, functional, robust, readable design. Reuse existing code,
helpers, tests, CLI patterns, and HIL runner structure wherever feasible. Do not
create broad frameworks, placeholder abstractions, generic service registries,
or speculative extension points.

You may spawn subagents for read-only audit work. Suggested splits:

- Core contracts: staged jobs, recovery, sample freshness, status codes.
- HIL/CLI parity: Arduino CLI, ESP-IDF CLI, runner validators and result rules.
- Docs/evidence: hardware matrix, report, runbook, release/security docs.
- IDF/build: native ESP-IDF example contract and local/CI validation gaps.

Integrate the results yourself. Do not delegate final judgment.

## Required Starting Steps

1. Read `AGENTS.md` first and obey it.
2. Check `git status --short`; preserve dirty user changes.
3. Read these current evidence and contract files before editing:
   - `docs/reports/hil-validation-COM28-20260622.md`
   - `README.md`
   - `docs/I2C_HIL_RUNBOOK.md`
   - `docs/BME280_HARDWARE_VALIDATION_MATRIX.md`
   - `docs/PRODUCTION_SHARED_BUS_GUIDE.md`
   - `docs/IDF_PORT.md`
   - `include/BME280/BME280.h`
   - `include/BME280/Status.h`
   - `src/BME280.cpp`
   - `tools/run_i2c_hil.py`
   - `tools/check_hil_contract.py`
   - `tools/check_idf_example_contract.py`
   - `examples/01_basic_bringup_cli/src/main.cpp`
   - `examples/idf/basic/main/main.cpp`
   - `test/test_basic.cpp`
4. Do not claim hardware, ESP-IDF, fault, electrical, or calibrated accuracy
   validation unless the exact command and artifact evidence exists.

## Current Known State

The latest recorded HIL evidence is one ESP32-S2 board on `COM28`, PlatformIO
Arduino `esp32s2dev`, BME280 address `0x76`, chip ID `0x60`.

Recorded result summary:

- Functional HIL: `61 PASS`, `22 OPERATOR_CHECK_REQUIRED`, `1 UNKNOWN`,
  `0 FAIL`, `0 TIMEOUT`.
- 8-hour soak: `11497 PASS`, `7297 OPERATOR_CHECK_REQUIRED`, `35 UNKNOWN`,
  `0 FAIL`, `0 TIMEOUT`.
- `UNKNOWN` rows were bounded reset/NVM `BUSY` observations. Follow-up evidence
  showed `READY`, `dirty=false`, `consecutiveFailures=0`.

Not yet proven:

- Native ESP-IDF HIL runtime behavior.
- ESP32-S3 hardware behavior.
- Address `0x77`.
- Calibrated environmental accuracy.
- Protected electrical/fault injection.
- Shared-bus contention under real application load.

## Production-Grade Closure Work

### 1. Fix Staged Recovery From OFFLINE

Problem:

`startRecoveryJob()` starts from `OFFLINE`, but only the reset write has a local
offline-I2C allowance. Later recovery job phases use tracked reads/writes that
can hit `_offlineStatus()` before touching hardware.

Required fix:

- Make `JobKind::RECOVERY` allow I2C for all recovery-job transport phases while
  preserving health tracking and the OFFLINE latch semantics on failure.
- Keep the existing synchronous `recover()` contract: failed recovery from
  OFFLINE must leave the latch in OFFLINE unless a complete successful recovery
  occurs.
- Do not allow non-recovery jobs to bypass OFFLINE.

Concrete test names:

- `test_recovery_job_from_offline_clears_latch_and_allows_i2c`
- `test_recovery_job_from_offline_failure_reasserts_offline_latch`
- `test_non_recovery_job_still_blocked_while_offline`

Acceptance:

- Recovery job from OFFLINE can read/write the device and finish `READY`.
- Failed recovery job from OFFLINE ends with state `OFFLINE`.
- `probe()` remains raw diagnostic and does not clear OFFLINE.

### 2. Align Staged Recovery With Synchronous Recover

Problem:

Synchronous `recover()` verifies chip ID before NVM/calibration/config resync.
The staged recovery path currently starts with reset and can skip the explicit
chip-ID check.

Required fix:

- Add a dedicated staged recovery chip-ID phase before declaring recovery
  trustworthy.
- Recommended enum name:

```cpp
JobPhase::RECOVERY_READ_CHIP_ID
```

- The staged recovery success sequence must prove:
  1. chip ID is `0x60`;
  2. NVM is ready through bounded polling;
  3. calibration reload succeeds;
  4. calibration validation succeeds;
  5. cached config is fully reapplied;
  6. dirty state clears only after all prior steps succeed.

Concrete tests:

- `test_recovery_job_chip_id_mismatch_records_health_failure`
- `test_recovery_job_dirty_clears_only_after_full_resync`
- `test_recovery_job_nvm_busy_uses_one_status_read_per_poll`

Expected status:

- Chip mismatch returns `Err::CHIP_ID_MISMATCH` with observed ID in `detail`.
- NVM poll cap returns `Err::TIMEOUT`.
- Transport errors preserve `I2C_TIMEOUT`, `I2C_BUS`, `I2C_NACK_DATA`,
  `I2C_ERROR`, or `DEVICE_NOT_FOUND` where distinguishable.

### 3. Close Forced-Measurement Job Status Contract

Problem:

`requestMeasurement()` updates `lastMeasurementStatus()` to `IN_PROGRESS` and
then `OK`. `startForcedMeasurementJob()` should present the same observable
measurement lifecycle.

Required fix:

- When a forced job is accepted, set `_lastMeasurementStatus` to
  `Err::IN_PROGRESS` with message `"Measurement job started"`.
- While waiting, keep `IN_PROGRESS` with a useful static message.
- On successful compensation, set `_lastMeasurementStatus = Status::Ok()`.
- On terminal failure, preserve the original error.

Concrete tests:

- `test_forced_measurement_job_reports_in_progress_status`
- `test_forced_measurement_job_success_clears_last_measurement_status`
- `test_forced_measurement_job_failure_preserves_last_measurement_status`

### 4. Make Cached Sample Freshness Explicit

Problem:

The driver intentionally keeps the latest successful cached sample after later
failures in some paths. That can be useful, but production consumers need a
clear way to know whether a sample is fresh for the current request.

Recommended minimal public API addition:

```cpp
enum class SampleFreshness : uint8_t {
  NONE,
  FRESH,
  STALE_AFTER_ERROR,
  STALE_AFTER_CONFIG_DIRTY
};
```

Add to `SettingsSnapshot`:

```cpp
SampleFreshness sampleFreshness = SampleFreshness::NONE;
```

Add public helpers:

```cpp
SampleFreshness sampleFreshness() const;
bool sampleFresh(uint32_t nowMs, uint32_t maxAgeMs) const;
```

Rules:

- `NONE`: no cached sample exists.
- `FRESH`: last measurement status is `OK`, hardware config is clean, and a
  cached sample exists.
- `STALE_AFTER_ERROR`: cached sample exists but the latest measurement request,
  tick, raw read, or compensation status is not `OK`.
- `STALE_AFTER_CONFIG_DIRTY`: cached sample exists but hardware config dirty is
  true.
- `sampleFresh(nowMs, maxAgeMs)` returns true only for `FRESH` and
  `sampleAgeMs(nowMs) <= maxAgeMs`.

Do not change `getRawSample()` or `getCompensatedSample()` to fail for stale
samples unless you also update all docs and tests. The preferred simple contract
is "latest successful sample remains readable, freshness is explicit."

Optional append-only error code only if needed by callers:

```cpp
Err::MEASUREMENT_STALE
```

If added, append it at the end of `Err` to preserve existing numeric values.

Concrete tests:

- `test_sample_freshness_none_before_capture`
- `test_sample_freshness_fresh_after_successful_capture`
- `test_sample_freshness_stale_after_failed_refresh`
- `test_sample_freshness_stale_when_hardware_config_dirty`
- `test_sample_fresh_uses_wrap_safe_age_check`

Docs to update:

- `include/BME280/BME280.h`
- `README.md`
- `docs/PRODUCTION_SHARED_BUS_GUIDE.md`
- `docs/TUNNELMONITOR_FIT_REPORT.md`, if affected

### 5. Add CLI/HIL Coverage For Chunked Job APIs

Problem:

The public staged APIs are mostly covered by native tests. Hardware HIL mainly
exercises synchronous flows.

Add diagnostic CLI commands to both Arduino and native ESP-IDF examples with
matching output tokens:

```text
job status
job init <budget>
job force <budget>
job apply <budget>
job recover <budget>
job poll <budget>
```

Recommended thresholds:

- `JOB_CLI_DEFAULT_BUDGET = 1`
- `JOB_CLI_MAX_BUDGET = 8`
- `JOB_CLI_MAX_POLLS = 512`
- `JOB_CLI_POLL_DELAY_MS = 1` in examples only

Behavior:

- `job <kind> <budget>` starts the job and polls until DONE/FAILED or
  `JOB_CLI_MAX_POLLS`.
- `job poll <budget>` advances an already active job once.
- `job status` prints `kind`, `state`, `status code`, `instructionsUsed` from
  the last poll where available, and driver dirty/health state.
- Invalid budget values are rejected without touching hardware.

Required common output tokens:

```text
=== Job Status ===
Job kind:
Job state:
Status:
Instructions:
Driver:
```

HIL runner work:

- Add `--include-job-api`.
- Add a `job-api` command group with at least:
  - `job status`
  - `job force 1`
  - `raw`
  - `comp`
  - `job recover 1`
  - `cfg`
  - `status`
  - `job force 3`
- Add parser validators:
  - `job_done_or_failed`
  - `job_zero_consecutive_failures`
  - `job_instruction_budget_respected`

Contract checks:

- Update `tools/check_cli_contract.py` and/or `tools/check_idf_example_contract.py`
  so Arduino and IDF command surfaces stay aligned.
- Add parser tests in `tools/test_run_i2c_hil_parser.py`.

### 6. Make ESP-IDF CLI Output HIL-Compatible

Problem:

The ESP-IDF example is source-contract checked but not proven to emit the same
tokens expected by `tools/run_i2c_hil.py`.

Required fix:

- Align ESP-IDF CLI output with Arduino CLI for all runner-validated commands:
  `selftest`, `stress`, `stress_mix`, `drv`, `state`, `cfg`, `calib`,
  `calib raw`, `status`, `reset`, `recover`, and new `job` commands.
- Do not include Arduino headers, Arduino source, `String`, `Serial`, `Wire`,
  `TwoWire`, or compatibility facades in IDF code.

Concrete checker:

```text
test_idf_cli_output_tokens_match_hil_runner_contract
```

Implement this as a repo-local checker or unit-style parser test using source
tokens if executable IDF HIL is not available.

Required tokens to preserve:

```text
Selftest result:
=== Stress Summary ===
Errors:
Health delta:
=== stress_mix summary ===
Total:
=== Driver Health ===
Hardware config dirty:
Consecutive failures:
```

### 7. Add Release-Gating HIL Exit Mode

Problem:

The HIL runner currently exits successfully for evidence runs that end in
`OPERATOR_REVIEW_REQUIRED` or `UNKNOWN`. That is useful for collection, but not
strong enough for release gating.

Add flags:

```text
--require-pass
--fail-on-review
```

Recommended behavior:

- Default behavior remains evidence-friendly and backward-compatible.
- With `--require-pass`, exit `0` only when final verdict is `PASS`.
- With `--fail-on-review`, treat `OPERATOR_REVIEW_REQUIRED`, `REVIEW_REQUIRED`,
  `SERIAL_OK_OR_REVIEW`, and `UNKNOWN` as nonzero.

Exit codes:

- `0`: complete pass.
- `1`: `FAIL`, `TIMEOUT`, serial exception, or hard validator failure.
- `2`: CLI usage/safety-gate/configuration error. Let `argparse` keep this.
- `3`: review/unknown verdict under `--require-pass` or `--fail-on-review`.

Tests:

- `test_require_pass_exits_zero_for_pass`
- `test_require_pass_exits_three_for_unknown`
- `test_fail_on_review_exits_three_for_operator_review`

### 8. Define Reset/NVM UNKNOWN Acceptance Rule

Problem:

The COM28 run recorded bounded reset `BUSY` / NVM-busy rows. Production gating
needs a deterministic rule.

Recommended rule:

`UNKNOWN` is not a production PASS. The only acceptable release-gate exception
is soft reset returning `Status: BUSY` with message `"NVM update in progress"`
when immediate follow-up evidence proves all of:

- post-reset `status` has `im_update=0`;
- `recover` returns `OK`;
- `cfg` is readable;
- final `status` has `dirty=false`;
- `drv` has `State: READY` and `Consecutive failures: 0`.

Do not add a new error code unless machine consumers need to distinguish NVM
busy from other `BUSY` cases. If needed, append only:

```cpp
Err::NVM_BUSY
```

If added, update tests, docs, CLI output, and HIL runner parsing.

### 9. Add Fault And Accuracy Evidence Gates

The repo should distinguish three evidence levels:

1. Serial protocol pass.
2. Bench plausibility pass.
3. Production accuracy/fault pass.

Add HIL runner options for reference readings:

```text
--reference-temp-c <float>
--reference-pressure-pa <float>
--reference-humidity-pct <float>
--accuracy-temp-c <float>        default 2.0
--accuracy-pressure-pa <float>   default 200
--accuracy-humidity-pct <float>  default 5.0
```

Add validator name:

```text
VALIDATOR_SAMPLE_ACCURACY
```

Bring-up plausibility default thresholds:

- Temperature: `2.0 C`
- Pressure: `200 Pa`
- Humidity: `5.0 %RH`

Production matrix thresholds:

- Temperature: `0.5 C + reference uncertainty + mounting allowance`
- Pressure: `100 Pa + reference uncertainty + altitude/local pressure setup uncertainty`
- Humidity: `3.0 %RH + reference uncertainty`, only in `20..80 %RH`
  non-condensing conditions

Add safe absence/fault options:

```text
--include-absence-check
--fault-fixture none|manual|gpio
```

Rules:

- `--include-absence-check` may test the opposite BME280 address and must
  restore the requested address afterward.
- Expected wrong-address outcome: `DEVICE_NOT_FOUND` or definite address NACK
  mapped to absence.
- `--fault-fixture none` records manual checklist only.
- `--fault-fixture manual` marks manual unplug/stuck-bus actions as requested
  but does not automate unsafe GPIO actions.
- `--fault-fixture gpio` must not be implemented unless a safe fixture API and
  pin ownership are explicitly present in examples; otherwise reject with a
  clear message.

Fault-status acceptance matrix:

| Condition | Expected code/state |
| --- | --- |
| Wrong address / definite address NACK | `DEVICE_NOT_FOUND` for presence checks, or `I2C_NACK_ADDR` in raw transport contexts |
| Transfer timeout | `I2C_TIMEOUT` |
| Data NACK | `I2C_NACK_DATA` |
| Bus/arbitration/stuck bus | `I2C_BUS` |
| Generic platform I2C failure | `I2C_ERROR` or `I2C_BUS` with raw detail |
| Wrong chip ID | `CHIP_ID_MISMATCH`, observed ID in `detail` |
| Consecutive tracked failures reach threshold | `DriverState::OFFLINE` |
| Successful recovery | `READY`, `hardwareConfigDirty=false`, `consecutiveFailures=0` |

### 10. Update Hardware Matrix And Evidence Ledger

Problem:

The maintained hardware matrix still says all rows are `NOT RUN`, while the
COM28 report records real ESP32-S2 Arduino HIL evidence.

Required update:

- Do not mark COM28 rows as full `PASS`.
- Add target-specific rows or columns so the matrix can say:
  - ESP32-S2 Arduino `0x76`: `OPERATOR_REVIEW_REQUIRED`, with links to
    `docs/reports/hil-validation-COM28-20260622.md` and local HIL artifact
    paths.
  - ESP32-S2 ESP-IDF: `NOT RUN`.
  - ESP32-S3 Arduino: build only, hardware `NOT RUN`.
  - ESP32-S3 ESP-IDF: `NOT RUN`.
  - Address `0x77`: `NOT RUN`.
  - Calibrated accuracy: `NOT RUN`.
  - Fault injection: `NOT RUN`.
  - Shared-bus contention: `NOT RUN`.

Also add a short rule: HIL artifacts under `hil_logs/` are local unless packaged
with a manifest and hashes.

### 11. Make HIL Evidence Portable

Problem:

Reports reference ignored local `hil_logs/` paths. That is acceptable during
development, but not enough for a release evidence package.

Required work:

- Add a documented packaging command, preferably reusing existing
  `manifest.json`.
- Recommended script name:

```text
tools/package_hil_artifacts.py
```

Minimum behavior:

```text
python tools/package_hil_artifacts.py hil_logs/i2c_20260622_205926 hil_logs/i2c_20260622_210228 --out docs/reports/hil-validation-COM28-20260622-artifacts
```

The script should:

- read each run's `manifest.json`;
- verify all listed files exist;
- compute SHA256 for each artifact;
- write a compact Markdown index;
- optionally create a `.zip` only when requested with `--zip`;
- not commit large logs or zips by default.

If this is too much for the current pass, at least update the runbook and report
with exact packaging instructions and SHA256 generation commands.

### 12. Add Electrical And Shared-Bus Production Gates

Docs currently ask operators to record electrical setup but do not define
pass/fail gates.

Add conservative gates to docs and matrix:

Electrical:

- VDD within `1.71..3.6 V`.
- VDDIO within `1.2..3.6 V`.
- SDA/SCL, SDO, and CSB never driven above VDDIO when VDDIO is off.
- SDO tied to GND for `0x76` or VDDIO for `0x77`; not floating.
- CSB tied high to VDDIO for I2C before POR.
- I2C bus capacitance `<= 400 pF`.
- Rise time verified for the selected I2C speed and pull-ups.
- Stable single-transaction burst read `0xF7..0xFE` at production bus speed.

Shared bus:

- At least one other device active during BME280 sampling.
- Bus lock timeout test returns `I2C_TIMEOUT`.
- Scheduler evidence shows `tick()` is called within the application sample
  budget.
- Fault in another bus client does not cause hidden BME280 success.
- No ISR directly calls public BME280 APIs.

### 13. Fix Process/Release Docs

Audit and update:

- `CHANGELOG.md`
- `library.json`
- `idf_component.yml`
- `Doxyfile`
- `include/BME280/Version.h` through `scripts/generate_version.py`
- `SECURITY.md`
- `CONTRIBUTING.md`
- `README.md`

Rules:

- If public API changes are made, bump MINOR to `1.7.0`.
- Use:

```text
python scripts/generate_version.py set 1.7.0
```

- Update `idf_component.yml` and `Doxyfile` to the same version.
- Move relevant `[Unreleased]` changes into the new release section.
- Update `SECURITY.md` supported versions so it does not claim stale `1.3.x`
  support if that is no longer true.
- Update `CONTRIBUTING.md` to require the current validation gate and honest HIL
  claim rules.

Do not bump version just to edit docs. Bump only when public API/behavior
changes justify it.

## Validation Required Before Final Answer

Run the smallest checks after each local fix. Before final response, run:

```powershell
git diff --check
python tools\test_run_i2c_hil_parser.py
python tools\check_hil_contract.py
python tools\check_cli_contract.py
python tools\check_idf_example_contract.py
python tools\check_core_timing_guard.py
python scripts\generate_version.py check
python tools\check_release_metadata.py
python -m py_compile tools\run_i2c_hil.py tools\check_hil_contract.py tools\check_release_metadata.py
python -m platformio test -e native
python -m platformio run -e esp32s2dev
python -m platformio run -e esp32s3dev
python -m platformio pkg pack
python tools\check_package_contents.py
doxygen Doxyfile
```

Remove generated package tarballs after checking them.

If `idf.py` is available, also run:

```powershell
idf.py -C examples/idf/basic set-target esp32s2
idf.py -C examples/idf/basic build
idf.py -C examples/idf/basic set-target esp32s3
idf.py -C examples/idf/basic build
```

If hardware is available, run at least:

```powershell
python tools\run_i2c_hil.py --parser-self-test
python tools\run_i2c_hil.py --port COM28 --baud 115200 --timeout-s 8 --boot-settle-s 2 --include-job-api --include-config-matrix --include-invalid-inputs --include-benchmarks --operator <name> --board ESP32-S2 --mcu-target esp32s2 --framework PlatformIO-Arduino --build-target esp32s2dev --sda-pin GPIO8 --scl-pin GPIO9 --bus-speed 400000Hz --sdo-state GND-inferred-from-0x76 --csb-state high-inferred-from-I2C-mode
```

Only run fault/accuracy validation when the required fixture/reference
equipment exists. Otherwise document `NOT RUN`.

## Final Response Requirements

Report:

- Files changed.
- Findings fixed, ordered by severity.
- Any API/version changes.
- Exact checks run and results.
- HIL commands run and result counts, if any.
- Hardware/fault/accuracy/ESP-IDF items still `NOT RUN`.
- Whether the library can now be called production-grade, and under which
  evidence boundary.

Do not say "production-grade" without qualifying the target and evidence, for
example:

```text
Production-grade core contract for host/native and Arduino ESP32-S2 0x76 HIL:
yes/no.
Full hardware production qualification across ESP-IDF, ESP32-S3, 0x77,
fault injection, and calibrated accuracy: yes/no.
```
