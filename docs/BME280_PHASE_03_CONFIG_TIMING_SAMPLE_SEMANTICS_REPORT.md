# BME280 Phase 03 Config Timing and Sample Semantics Report

Date: 2026-05-31

Branch: `hardening/bme280-industry-readiness`

Scope: Phase 03 only. This report covers BME280 configuration write sequencing,
measurement scheduling deadlines, sample cache semantics, dirty partial config
diagnostics, native fake-transport coverage, and related README/Doxygen updates.
No hardware validation was run. No local ESP-IDF `idf.py` build was run.

## Start State

The phase started from a clean working tree on
`hardening/bme280-industry-readiness`.

Recent commits at phase start:

- `70c5ab9 docs: add comprehensive report for BME280 prompts 00-06`
- `1c0d409 feat(bme280): Enhance calibration and compensation handling`
- `5c0cf27 hardening: lock BME280 baseline facts`
- `a750935 hardening: lock BME280 readiness facts`

## Subagent Reviews

Required Phase 03 review roles inspected repository files before recommendations
were accepted.

### bme280-datasheet-agent

Findings:

- Humidity oversampling sequencing was correct in intent: `ctrl_hum` must be
  followed by `ctrl_meas` for the setting to take effect.
- `config` register changes must not be written while the device reports
  `status.measuring`; normal-mode config writes may be ignored.
- Multi-register config sequences needed consistent dirty diagnostics when an
  earlier write may have reached hardware and a later write failed.
- Raw data coherency required a single burst read from `0xF7..0xFE`.

Actions taken:

- Kept `setOversamplingH()` as `ctrl_hum` then `ctrl_meas`.
- Added a `status.measuring` guard before `config` write sequences.
- Preserved dirty-state marking for partial config failures.
- Added a test asserting the raw sample read is one burst from `REG_DATA_START`
  with `DATA_LEN`.

### core-contracts-agent

Findings:

- `tick()` is void but performs fallible I/O, so failures needed observable
  diagnostics.
- `requestMeasurement()` uses `Config::nowMs` while `tick(nowMs)` receives a
  caller timestamp; docs needed to require the same timebase.
- Public contracts needed explicit thread/ISR/reentrancy language.
- Successful typed config changes needed documented sample invalidation.

Actions taken:

- Added `lastMeasurementStatus()` and
  `SettingsSnapshot::lastMeasurementStatus`.
- `requestMeasurement()` and `tick()` now retain pending, success, transport,
  and compensation statuses for later inspection.
- Public Doxygen and README now state that `tick(nowMs)` and `Config::nowMs`
  must use the same monotonic timebase.
- Public Doxygen now states the driver is not thread-safe or ISR-safe and that
  callbacks must not recursively call into the same driver instance.

### fault-injection-agent

Findings:

- Partial config failures needed tests at each meaningful write step.
- Tests should preserve the first transport error that makes hardware config
  uncertain.
- Reset/NVM polling findings were noted as Phase 04 scope and were not expanded
  in this phase.

Actions taken:

- Added fake-bus write logging so tests can assert register write order.
- Added tests for `ctrl_hum` then `ctrl_meas`, normal-mode sleep/config/restore,
  config-write BUSY while measuring, sleep-step failure, config-step failure,
  humidity-step failure, recover/apply failure, and measurement status on a raw
  burst read failure.

### docs-hardware-agent

Findings:

- README needed a public transaction/latency shape table.
- Sample invalidation, `ctrl_hum` latching, config-in-sleep behavior, timebase
  requirements, and raw-write caveats needed clearer public documentation.
- Documentation must not claim hardware or ESP-IDF validation that was not run.

Actions taken:

- README now documents `ctrl_hum`/`ctrl_meas` latch behavior, config writes in
  sleep, dirty partial config state, sample invalidation, burst-read coherency,
  transaction shape, and Bosch-derived timing formula.
- The report explicitly records that hardware validation and local ESP-IDF
  `idf.py` validation were not run.

### integration-review-agent

Findings before commit:

- The first implementation guarded `REG_CONFIG` before the sleep write but did
  not re-check `status.measuring` after writing sleep.
- `_applyConfig()` BUSY behavior during `begin()`, `recover()`, and
  `softReset()` needed tests/docs.
- The older comprehensive 00-06 report still described Phase 03 as pending.

Actions taken:

- Added a second `status.measuring` check after the sleep write and before
  `REG_CONFIG`.
- Added native tests for begin-time measuring BUSY and post-sleep measuring
  dirty-state behavior.
- Updated `docs/BME280_PROMPTS_00_06_COMPREHENSIVE_REPORT.md` to mark Phase 03
  complete and point at this report.

## Previous Behavior

- `setFilter()` and `setStandby()` wrote `ctrl_meas` sleep, `config`, and
  restored mode, but did not first check whether the device was currently
  measuring.
- `tick()` could observe transport or compensation errors but callers had no
  direct way to inspect the last measurement failure because `tick()` is void.
- Some partial config failures did not mark `hardwareConfigDirty()` if the first
  write in a multi-register sequence failed or if a typed config write failed.
- README/Doxygen did not fully document timebase requirements, config register
  sequencing, sample invalidation, transaction shape, or raw-write resync
  expectations.

## New Behavior

### Register Sequencing

- `setOversamplingH()` writes `REG_CTRL_HUM` (`0xF2`) first and then writes
  `REG_CTRL_MEAS` (`0xF4`) so the humidity setting is latched by hardware.
- `setFilter()` and `setStandby()`:
  - read `REG_STATUS` (`0xF3`) first;
  - return `BUSY` without writes if `MASK_STATUS_MEASURING` is set;
  - otherwise write `REG_CTRL_MEAS` sleep;
  - read `REG_STATUS` again before `REG_CONFIG`;
  - skip `REG_CONFIG` and mark dirty if the second status read still reports
    measuring;
  - otherwise write `REG_CONFIG` (`0xF5`), then restore the cached mode through
    `REG_CTRL_MEAS`.
- `_applyConfig()` uses the same status guard before its config resync write
  sequence. During `begin()`, an already-measuring device returns `BUSY` before
  config writes; callers can retry `begin()` when the conversion is expected to
  be complete.

### Dirty Partial Config State

- Any failed typed config write that may have reached or partially changed
  hardware marks `hardwareConfigDirty()`.
- The original failing `Status` is preserved in `hardwareConfigDirtyError()` and
  `SettingsSnapshot::hardwareConfigDirtyError`.
- A config-write BUSY result from the status guard does not mark dirty because
  no config write was attempted.
- A BUSY result after the sleep write does mark dirty because `ctrl_meas` may
  have reached hardware while cached mode/config were not updated.
- Dirty state clears only after a complete successful resync.

### Sample Cache Decision

The driver uses invalidation, not generation tagging, for Phase 03:

- Successful typed config changes invalidate cached raw and compensated samples.
- `setMode(SLEEP)` cancels pending measurement state.
- After invalidation, `getRawSample()` and `getCompensatedSample()` return
  `MEASUREMENT_NOT_READY` until a new capture completes.

### Measurement Status Semantics

- `requestMeasurement()` records its returned status in
  `lastMeasurementStatus()`.
- `tick()` records `IN_PROGRESS` while the sensor still reports measuring,
  `OK` after a sample is captured, and the original transport or compensation
  error if status polling, burst read, or compensation fails.
- `SettingsSnapshot` exposes the same `lastMeasurementStatus` without I2C.

### Timing Model

- `begin()` does not require `Config::nowMs`.
- `requestMeasurement()` returns `INVALID_CONFIG` before starting work if
  `Config::nowMs` is missing.
- `tick(nowMs)` and `Config::nowMs` must use the same monotonic millisecond
  timebase.
- Forced-mode deadlines use `estimateMeasurementTimeMs()`.
- Normal-mode freshness waits one estimated normal cycle:
  `estimateMeasurementTimeMs() + getStandbyTimeMs()`.
- Measurement estimate follows the Bosch-style oversampling terms and rounds up
  with a 1000 us safety margin:

```text
t_meas_us = 1250
          + (temperature enabled ? 2300 * osrs_t : 0)
          + (pressure enabled ? 2300 * osrs_p + 575 : 0)
          + (humidity enabled ? 2300 * osrs_h + 575 : 0)
          + 1000
estimateMeasurementTimeMs = ceil(t_meas_us / 1000)
```

## Tests Added or Updated

Native tests now cover:

- Humidity oversampling writes `ctrl_hum` then `ctrl_meas`.
- Normal-mode filter change writes sleep, config, then restore.
- Config write while `status.measuring` is set returns `BUSY` without writes.
- Config write that sees `measuring` after the sleep write skips `REG_CONFIG`,
  restores best-effort, marks dirty, and preserves the BUSY status.
- `begin()` returns `BUSY` without config writes if the device is already
  measuring during `_applyConfig()`.
- Sleep-step, config-step, humidity-step, and recover/apply failures mark dirty
  state and preserve the original error.
- Measurement time estimates use oversampling-derived timing.
- `tick()` raw burst read failure is visible through
  `lastMeasurementStatus()` and `SettingsSnapshot`.
- Raw sample capture reads exactly `0xF7..0xFE` in one burst.
- Existing tests continue to cover missing `nowMs`, timestamp wraparound,
  normal-mode freshness, forced-mode request semantics, cache invalidation after
  config change, and successful dirty-state recovery.

## API and Documentation Changes

- Added `Status lastMeasurementStatus() const`.
- Added `Status lastMeasurementStatus` to `SettingsSnapshot`.
- Expanded public Doxygen for:
  - thread/ISR safety,
  - transport callback ownership and non-recursion,
  - `tick()` timebase and status observation,
  - request preconditions and statuses,
  - sample invalidation after typed config changes,
  - humidity latch sequencing,
  - filter/standby sleep/config/restore behavior,
  - raw write resync caveats.
- Expanded README with the same public contracts plus a transaction shape table.

## Checks and Results

All requested checks available in this environment passed:

| Command | Result |
|---------|--------|
| `python tools/check_core_timing_guard.py` | PASS: `Core timing guard PASSED` |
| `python tools/check_cli_contract.py` | PASS: `CLI contract PASSED` |
| `python tools/check_idf_example_contract.py` | PASS: `IDF example contract PASSED` |
| `python scripts/generate_version.py check` | PASS: `Version.h` up to date |
| `python -m platformio test -e native` | PASS: 63 test cases, 63 succeeded |
| `python -m platformio run -e esp32s3dev` | PASS: build succeeded |
| `python -m platformio run -e esp32s2dev` | PASS: build succeeded |

Not run:

- Hardware validation on a physical BME280.
- Local ESP-IDF `idf.py` builds.

## Remaining Risks and Deferred Items

- The status guard avoids known `config` writes while the sensor reports
  measuring, but there is no physical hardware validation in this phase.
- Reset/NVM-copy polling hardening and deeper reset fault injection remain Phase
  04 scope.
- ESP-IDF contract scripts passed, but native `idf.py` builds were not run in
  this phase.
- Driver instances remain externally serialized by contract; internal locking
  was intentionally not added.
