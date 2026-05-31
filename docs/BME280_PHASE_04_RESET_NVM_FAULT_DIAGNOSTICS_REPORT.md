# BME280 Phase 04 Reset NVM and Fault Diagnostics Report

Date: 2026-05-31

Branch: `hardening/bme280-industry-readiness`

Scope: Phase 04 only. This report covers reset/NVM polling, probe/begin/recover
diagnostics, calibration reload, dirty-state handling, health/offline fault
semantics, and related native fake-transport tests/docs. No hardware validation
was run. No local pure ESP-IDF `idf.py` build was run.

## Start State

Phase 04 started from a clean working tree on
`hardening/bme280-industry-readiness`.

Recent commits at phase start:

- `efecfe8 hardening: enforce BME280 config timing semantics`
- `70c5ab9 docs: add comprehensive report for BME280 prompts 00-06`
- `1c0d409 feat(bme280): Enhance calibration and compensation handling`
- `5c0cf27 hardening: lock BME280 baseline facts`

## Subagent Reviews

Formal agents were used where session capacity allowed. Remaining required roles
were emulated through direct repository inspection and are recorded here.

### bme280-datasheet-agent

Findings:

- `recover()` verified chip ID and reapplied config but did not wait for NVM
  `im_update` to clear, so it could write config while trimming data was still
  being copied.
- `_waitForNvmReady()` ignored failed status reads and eventually returned a
  vague timeout instead of preserving timeout, bus, data-NACK, or generic I2C
  errors.
- `begin()` reads chip ID once; this phase documents that applications should
  call `begin()` after BME280 POR/I2C readiness rather than adding chip-ID
  polling.

Actions:

- `recover()` now waits for NVM readiness, reloads calibration, validates it,
  and then reapplies cached config.
- NVM polling now preserves the first useful transport error if readiness never
  arrives.
- README/Doxygen document chip-ID behavior and POR readiness expectations.

### fault-injection-agent

Findings:

- Soft reset lacked focused tests for reset write failure, NVM timeout,
  transport failure during status polling, calibration read failure, apply
  failure, dirty-state behavior, and successful calibration reload.
- Probe coverage missed non-address transport mappings and chip-ID mismatch
  without health changes.
- Offline coverage missed probe while offline and successful recover from
  OFFLINE.

Actions:

- Native fake-transport coverage was expanded to 84 passing tests.
- Tests now cover probe mappings, begin wrong-device and transport errors,
  reset write ambiguity, NVM timeout and transport errors, atomic calibration
  reload, reset/recover dirty clearing, and offline latch behavior.

### core-contracts-agent

Findings:

- Runtime `recover()`/`softReset()` used raw NVM polling even though public
  initialized operations should update health through tracked wrappers.
- `_readCalibration()` committed fields as it parsed, so runtime calibration
  reload could leave mixed old/new coefficients after a later read failure.
- Reset write timeout/bus/data-NACK is ambiguous and could mean the reset byte
  reached hardware; dirty state should be set except for definite address NACK.

Actions:

- `_waitForNvmReady(false)` is used before initialization in `begin()`, and
  `_waitForNvmReady(true)` is used by initialized `recover()`/`softReset()` so
  status-read transport failures update health.
- Calibration reload now reads and parses all blocks into locals, validates
  required coefficients, and commits cached coefficients only after full success.
- Ambiguous reset-write failures now mark `hardwareConfigDirty()`; definite
  address NACK does not.

### docs-hardware-agent

Findings:

- README and metadata used stronger "production-grade" wording than the
  recorded validation supports.
- README should distinguish blocking bounded reset/NVM polling from tick-driven
  measurement polling.
- ESP-IDF validation remains inconsistent: metadata advertises `idf >=6.0.1`,
  while CI context observed by the agent was a v5.3 smoke path. This is deferred
  to Phase 05.
- Historical readiness report language could confuse readers because some
  baseline findings are now superseded.

Actions:

- README and package metadata now say "production-oriented".
- README has a validation-status note near the top.
- README now says reset/NVM polling is blocking and bounded, while measurement
  polling is tick-driven.
- The older industry-readiness report is marked as a historical baseline
  superseded by later phase reports.

### integration-review-agent

Findings:

- Runtime invalid calibration returned by `_readCalibration()` during
  `recover()`/`softReset()` needed to update health as a semantic recovery/reset
  failure.
- Tracked NVM polling transport failures were already counted by tracked I2C
  wrappers, so callers should not count the same transport failure again.
- NVM polling needed to return `TIMEOUT` when at least one status read succeeded
  but `im_update` stayed set, even if an earlier transient transport error was
  observed.

Actions:

- `recover()` and `softReset()` now record runtime `CALIBRATION_INVALID` as a
  health failure.
- Tracked NVM transport failures are returned without a second `_recordFailure()`
  call.
- `_waitForNvmReady()` now distinguishes "no status read ever succeeded" from
  "status reads succeeded but `im_update` stayed set".
- Regression tests were added for invalid runtime calibration and mixed
  transport/status-success NVM timeout behavior.
- A second integration-review pass found no remaining blockers before commit.

## Reset/NVM Behavior Before and After

Before Phase 04:

- `_waitForNvmReady()` tolerated reset/POR I2C errors but discarded them and
  returned `TIMEOUT` when readiness never arrived.
- `recover()` did not wait for `im_update` to clear and did not reload
  calibration before reapplying config.
- `softReset()` returned NVM/calibration failures without consistently marking
  dirty hardware-config state after a successful reset write.
- Runtime calibration reload could partially update cached coefficients before a
  later calibration read failed.

After Phase 04:

- NVM polling remains bounded by a 10 ms deadline when a real clock is injected
  and by a fixed poll limit when the framework-neutral fallback clock is inert.
- If no status read succeeds, the first useful transport error is returned.
- If status reads succeed but `im_update` stays set, the result is `TIMEOUT`.
- `recover()` waits for NVM, reloads calibration, validates it, and reapplies
  cached config.
- `softReset()` writes `0xB6` to `0xE0`, waits for NVM, reloads calibration,
  validates it, and reapplies cached config.
- Calibration cache updates are atomic across the full calibration read/parse
  path.

## Error Mapping

| Operation | Condition | Result |
| --- | --- | --- |
| `probe()` | address NACK | `DEVICE_NOT_FOUND` |
| `probe()` | timeout, bus, data NACK, generic I2C | original transport `Status` |
| `probe()` | chip ID not `0x60` | `CHIP_ID_MISMATCH` with observed ID in `detail` |
| `begin()` | address NACK during chip-ID read | `DEVICE_NOT_FOUND` |
| `begin()` | timeout, bus, data NACK, generic I2C during chip-ID read | original transport `Status` |
| `begin()` | chip ID not `0x60` | `CHIP_ID_MISMATCH` with observed ID in `detail` |
| NVM polling | all status reads fail | first useful transport error |
| NVM polling | status reads succeed but `im_update` stays set | `TIMEOUT` |
| `softReset()` | reset write timeout/bus/data-NACK/generic I2C | original error and dirty state set |
| `softReset()` | reset write address NACK | original `I2C_NACK_ADDR`, dirty state not set |

## Health and Dirty-State Behavior

- `probe()` remains raw diagnostic I2C. It does not update health counters and
  does not clear an `OFFLINE` latch.
- `recover()` is allowed while `OFFLINE` through scoped offline I2C allowance.
  On success it clears the offline latch through tracked I2C success.
- Runtime NVM polling in `recover()` and `softReset()` is health-tracked.
  Repeated status-read transport failures can move the driver to `OFFLINE`
  while preserving the root-cause status without a second caller-side health
  count for the same tracked transport failure.
- Runtime invalid calibration during `recover()` or `softReset()` is recorded as
  a semantic health failure.
- Dirty state clears only after a full successful config reapply in
  `begin()`, `recover()`, or `softReset()`.
- Reset write success followed by NVM, calibration, validation, or config
  reapply failure leaves dirty state set.
- Ambiguous reset write failures set dirty state because hardware may have
  accepted the reset command before the transport reported failure.

## Tests Added or Updated

Native tests now cover:

- Probe address-NACK mapping to `DEVICE_NOT_FOUND`.
- Probe timeout, bus, data-NACK, generic I2C, and chip-ID mismatch preservation
  without health updates.
- Begin wrong chip ID, chip-ID transport errors, NVM transport errors,
  calibration read failure, and apply-config failure.
- Soft reset reset-write timeout and address-NACK distinction.
- Soft reset NVM `im_update` timeout, status-read transport error preservation,
  mixed transient status-read failure followed by stuck `im_update`,
  calibration read failure, invalid runtime calibration, config reapply failure,
  successful calibration reload, and dirty-state clearing.
- Recover NVM wait, calibration reload, NVM transport error health transitions,
  invalid runtime calibration, successful recover from `OFFLINE`, and probe
  while `OFFLINE`.
- Atomic calibration reload by asserting failed runtime calibration reload does
  not overwrite cached coefficients.

## API and Documentation Changes

- `begin()`, `probe()`, `recover()`, `softReset()`, and dirty-state Doxygen were
  expanded to describe chip-ID behavior, NVM polling, calibration reload, and
  dirty-state semantics.
- README now includes reset/NVM diagnostics, error mapping, validation status,
  transaction shape updates, and health/offline notes.
- `library.json` and `idf_component.yml` descriptions now use
  "production-oriented" instead of "production-grade".
- `docs/BME280_INDUSTRY_READINESS_REPORT.md` is marked as a historical baseline
  superseded by later phase reports.

## Checks and Results

| Command | Result |
| --- | --- |
| `python tools/check_core_timing_guard.py` | PASS: `Core timing guard PASSED` |
| `python tools/check_cli_contract.py` | PASS: `CLI contract PASSED` |
| `python tools/check_idf_example_contract.py` | PASS: `IDF example contract PASSED` |
| `python scripts/generate_version.py check` | PASS: `include\BME280\Version.h` up to date |
| `python -m platformio test -e native` | PASS: 84 test cases, 84 succeeded in `00:00:01.578` |
| `python -m platformio run -e esp32s3dev` | PASS: `esp32s3dev SUCCESS` in `00:00:10.485` |
| `python -m platformio run -e esp32s2dev` | PASS: `esp32s2dev SUCCESS` in `00:00:12.721` |

## Remaining Risks

- Physical BME280 hardware reset/fault validation was not run.
- Local pure ESP-IDF `idf.py` builds were not run.
- CI/ESP-IDF target-version alignment is deferred to Phase 05.
- Chip-ID polling during early POR was not added; README/Doxygen document that
  applications should call `begin()` after BME280 POR and I2C readiness.
