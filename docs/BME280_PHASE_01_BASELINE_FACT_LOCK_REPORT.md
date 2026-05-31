# BME280 Phase 01 Baseline Fact Lock Report

Branch: `hardening/bme280-industry-readiness`

Baseline at phase start: `a75093507c9c3ee68934a89932e2dfe9d4a96fdb`

Phase scope: baseline audit, datasheet fact lock, `AGENTS.md` hardening rules, and a durable plan for later phases. No compensation rewrite, reset rewrite, CLI expansion, CI repair, package repair, or hardware validation was performed in this phase.

## 1. Branch And Repo Identity

- Current branch: `hardening/bme280-industry-readiness`.
- Repository identity evidence: `library.json` name `BME280`, public headers under `include/BME280/`, implementation in `src/BME280.cpp`, examples under `examples/`, tests under `test/`, guard scripts under `tools/`, and BME280 reports/datasheet references under `docs/`.
- Start-of-phase tree state: clean.
- Recent history at phase start includes `a750935 hardening: lock BME280 readiness facts` and prior IDF/hardening commits.

## 2. Current Baseline Checks

Start-of-phase commands:

```text
git status --short
git branch --show-current
git log --oneline -8
```

Start result: clean tree, branch `hardening/bme280-industry-readiness`, latest commit `a750935`.

Required verification commands for this prompt:

```text
python tools/check_core_timing_guard.py
python tools/check_cli_contract.py
python tools/check_idf_example_contract.py
python scripts/generate_version.py check
python -m platformio test -e native
python -m platformio run -e esp32s3dev
python -m platformio run -e esp32s2dev
```

Results are recorded in the "Commands Run" section after the phase verification pass.

## 3. Datasheet Facts Locked Into The Plan

Primary authority: Bosch Sensortec BME280 data sheet, document `BST-BME280-DS001-24`, revision 1.24, February 2024. Repository source copy: `docs/BME280_datasheet.pdf`. Extracted local references are in `docs/extracted-md/` and `docs/pdf-extracted-md/BME280_datasheet.md`.

Locked facts:

1. Chip ID register `0xD0` reads `0x60`.
2. Soft reset writes `0xB6` to register `0xE0`.
3. Status register `0xF3` exposes `measuring` on bit 3 and `im_update` on bit 0; `im_update` is set while NVM data is copied and clears when complete.
4. Valid 7-bit I2C addresses are `0x76` and `0x77`, selected by SDO. SDO must not float.
5. CSB must be tied high to VDDIO for I2C. If CSB is pulled low, SPI selection can interfere with I2C until power-on reset.
6. VDD and VDDIO are separate. Do not drive interface pins high while VDDIO is off.
7. Sleep mode is startup/default; all registers are accessible in sleep.
8. Forced mode performs one measurement, stores results, and returns to sleep.
9. Normal mode cycles active measurement and standby.
10. `ctrl_hum` changes take effect only after writing `ctrl_meas`.
11. `config` writes in normal mode may be ignored; filter/standby/SPI3W changes must be applied from sleep and then normal mode restored if needed.
12. Pressure and temperature raw outputs are unsigned 20-bit; humidity raw output is unsigned 16-bit.
13. Pressure/temperature skipped sentinel is `0x80000`; humidity skipped sentinel is `0x8000`.
14. Coherent samples require one burst read from `0xF7` through `0xFE`.
15. Calibration coefficients occupy `0x88..0xA1` and `0xE1..0xE7`; `dig_H4` and `dig_H5` are signed 12-bit values nibble-packed across `0xE4`, `0xE5`, and `0xE6`.
16. Compensation uses `t_fine`; pressure and humidity depend on the temperature value from the same sample context.
17. Best pressure accuracy uses the Bosch 64-bit pressure path.
18. Pressure compensation must guard divide-by-zero.
19. Humidity compensation must clamp to the valid range, normally 0..100 %RH.
20. The IIR filter applies to pressure and temperature, not humidity; changing filter settings resets filter memory.
21. Measurement time depends on humidity, pressure, and temperature oversampling, plus margin. Normal-mode period is measurement time plus standby.
22. Bosch self-test flow is reset, chip ID, trimming data, measurement, and plausibility checks. Plausibility limits must be configurable or documented because the ambient environment varies.
23. Humidity performance can be temporarily degraded by soldering/reflow, contamination, condensation, or exposure outside operating range. User-facing docs must include handling/reconditioning caveats.

## 4. Audit Claim Verification

| Claim | Result | Evidence / correction |
| --- | --- | --- |
| Framework-neutral core | True | `include/` and `src/` use standard/project headers only; guard script passes. Arduino/ESP-IDF use is confined to examples/test stubs. |
| Injected/non-owning I2C transport | True | `Config` exposes `i2cWrite`, `i2cWriteRead`, and `i2cUser`; raw wrappers call only injected callbacks. Bus handles, pins, locks, and timeouts remain application-owned. |
| Chip-ID validation | True | `begin()`, `probe()`, and `recover()` read `REG_CHIP_ID` and require `CHIP_ID_BME280`. |
| Calibration parsing | True, test coverage partial | Implementation reads T/P and humidity calibration blocks and sign-extends H4/H5. Boundary and Bosch-vector tests are deferred to Phase 02. |
| Explicit measurement timing | True | `estimateMeasurementTimeMs()` implements oversampling-derived max timing plus margin; `estimateNormalCycleMs()` adds standby. |
| Dirty hardware-config state for partial writes | Partial | Dirty diagnostics exist and tests cover several multi-step failures. Ambiguous first-write failures and some restore-success cases still may leave hardware changed without dirty state. |
| `requestMeasurement()` rejects missing `Config::nowMs` | True | `requestMeasurement()` returns `INVALID_CONFIG` before scheduling or I2C writes when `nowMs` is missing. |
| `begin()` / `probe()` preserve transport errors except definite address NACK | Partial | Initial chip-ID failures preserve non-address-NACK transport errors. NVM polling can still collapse repeated I2C errors into generic `TIMEOUT`; probe lacks full timeout/bus/data-NACK test matrix. |
| Copy/move deleted | True | `BME280` copy/move construction and assignment are deleted and covered by native compile-time tests. |
| Native tests pass | True | Native test command passes with 41/41 tests. |
| Arduino ESP32-S2/S3 builds pass | True | PlatformIO `esp32s3dev` and `esp32s2dev` builds pass locally. |
| ESP-IDF CI jobs exist | True with mismatch | Workflow includes `esp-idf-build` for `esp32s3` and `esp32s2`, but it uses ESP-IDF `release-v5.3` while metadata/docs claim `>=6.0.1`. |
| Hardware validation not yet run | True | No hardware validation matrix/log exists, and this phase ran no hardware commands. |

## 5. Current Public API Surface Summary

- Lifecycle: `begin(const Config&)`, `tick(uint32_t nowMs)`, `end()`, `isInitialized()`, and `getConfig()`.
- Diagnostics and health: `probe()`, `recover()`, `getSettings()`, `hardwareConfigDirty()`, `hardwareConfigDirtyError()`, `state()`, `driverState()`, `isOnline()`, health timestamps/counters, and `lastError()`.
- Measurement: `requestMeasurement()`, `measurementReady()`, `hasSample()`, `sampleTimestampMs()`, `sampleAgeMs()`, `getMeasurement()`, `getRawSample()`, `getCompensatedSample()`, `getCalibration()`, and `readCalibrationRaw()`.
- Configuration: `setMode()`, oversampling setters/getters, `setFilter()`, `setStandby()`, `softReset()`, register status/config helpers, and `isMeasuring()`.
- Raw access: tracked `readRegisters()`, `writeRegisters()`, `readRegister()`, and `writeRegister()`.
- Timing: `estimateMeasurementTimeMs()`, `getStandbyTimeMs()`, and `estimateNormalCycleMs()`.
- Current API gaps: `tick()` is `void` while performing fallible I/O/compensation work; thread/ISR and recursive-callback cautions are not fully Doxygen-facing; samples are not invalidated or generation-tagged across config changes; skipped raw sentinels do not yet have a first-class validity/status model.

## 6. Current Test, Build, CI, And Package Surface

- Native tests: one Unity test source, `test/test_basic.cpp`, built by PlatformIO native env.
- Native coverage includes lifecycle, config validation, copy/move deletion, NVM wait, no-clock behavior, probe health behavior, recover/offline behavior, dirty-state diagnostics, forced/normal timing, and sample cache availability.
- Test gaps include byte-level partial writes, full error-code matrices through begin/probe/tick/softReset, `tick()` failure injection, and Bosch-reference compensation vectors.
- Arduino PlatformIO builds exist for `esp32s3dev` and `esp32s2dev`.
- ESP-IDF component files exist: root `CMakeLists.txt`, `idf_component.yml`, and native example under `examples/idf/basic`.
- CI jobs exist for PlatformIO Arduino builds, native tests, guard scripts, package pack, and ESP-IDF example build.
- CI/package gaps: ESP-IDF CI version does not match metadata, generated `Version.h` freshness is not checked in CI, `pio pkg pack` can pass while omitting ignored generated `include/BME280/Version.h`, and native PlatformIO inherits shared `Wire` dependency.

## 7. Current Docs And Examples State

- README documents framework-neutral core, injected I2C transport, health behavior, timing APIs, behavior contracts, and examples.
- Arduino and IDF examples are honestly bring-up/diagnostic CLI examples, not production shared-bus templates.
- Existing docs include BME280 register/timing references, ESP-IDF port notes, extracted datasheet notes, and prior BME280 reports.
- Stale docs remain: `docs/BME280_INDUSTRY_READINESS_REPORT.md` still contains claims that later commits partially corrected.
- User-facing wiring notes are incomplete: the best SDO/CSB/VDDIO cautions are in extracted docs, not README/Doxygen-facing API docs. `Config.h` says `SDO=VDD`; later docs/API cleanup should prefer `VDDIO`.
- Humidity environmental/reconditioning caveats are mostly buried in extracted notes and should be surfaced in README/Doxygen.
- Production wording is stronger than validation evidence in `README.md`, `library.json`, and `idf_component.yml`; final release wording must be adjusted unless hardware validation is recorded.

## 8. Phase-By-Phase Implementation Plan

### Phase 02 - Calibration and Bosch-reference compensation vectors

- Add calibration parsing tests for every coefficient, signed boundaries, H4/H5 nibble packing, all-zero/all-`0xFF` plausibility where supported, and `dig_P1 == 0`.
- Add fixed Bosch-reference compensation vectors with provenance and without importing Bosch source unless license and attribution are settled.
- Add raw burst decoding tests and decide/document skipped sentinel behavior.

### Phase 03 - Config sequencing, timing, and sample semantics

- Tighten dirty diagnostics for all ambiguous multi-register write failures.
- Add config generation or invalidate cached samples across config changes.
- Expand timing tests for oversampling combinations, standby rounding, forced deadlines, normal-mode freshness, and filter-memory reset implications.

### Phase 04 - Reset/NVM and fault diagnostics

- Preserve NVM/status-read transport errors where possible instead of collapsing them into generic timeout.
- Add reset/NVM/softReset fault tests and full transport error matrices for begin/probe/recover/tick paths.
- Decide whether `tick()` needs status surfacing or documented best-effort diagnostics.

### Phase 05 - Examples, ESP-IDF/CI, package, and docs

- Align ESP-IDF CI version with component metadata or correct metadata/docs.
- Expand IDF guard coverage to all IDF example files.
- Add CI `scripts/generate_version.py check`.
- Fix package validation so `Version.h` is included or generated for consumers.
- Remove native env inheritance of Arduino `Wire` if feasible.
- Surface SDO/CSB/VDDIO wiring, humidity caveats, thread/ISR cautions, recursive-callback warnings, and production shared-bus guidance in README/Doxygen.
- Correct production wording unless hardware validation is available.

### Phase 06 - Hardware validation and final report

- Create/update `docs/BME280_INDUSTRY_STANDARD_FINAL_REPORT.md`.
- Record real hardware validation only if actual BME280 hardware commands are run.
- Include commit range, phase summaries, exact command results, hardware status, remaining risks, merge readiness, and release wording that does not overclaim.

## 9. Known Blockers And Deferred Items

- No real BME280 hardware validation has been run.
- No local pure ESP-IDF build was run because `idf.py` is not available in this shell.
- ESP-IDF metadata/docs require `>=6.0.1`, while CI builds `release-v5.3`.
- Package archive validation can pass while omitting ignored generated `Version.h`.
- Compensation correctness lacks Bosch-reference fixed vectors and boundary coverage.
- Skipped raw sentinels lack first-class core status/validity semantics.
- Dirty hardware-config diagnostics are incomplete for ambiguous first-write and partial-transaction cases.
- Samples are not invalidated or generation-tagged across config changes.
- `tick()` has fallible behavior with no returned `Status`.
- True byte-level partial writes and short `Wire::write()` behavior are not simulated.
- Doxygen/README do not yet carry all public safety, wiring, and humidity caveats.
- Existing docs contain stale audit claims and stronger production wording than validation supports.

## 10. Subagent Findings

### bme280-datasheet-agent

- True: chip ID, reset/status registers, modes, calibration packing, compensation, and measurement timing are represented in code/docs.
- Partial: SDO/CSB/VDDIO wiring facts are present in extracted docs but not sufficiently user-facing.
- Partial: raw burst read and widths are implemented, but skipped sentinels are only documented/example-visible, not a core validity/status contract.
- Partial: filter behavior and self-test facts are documented, but filter memory reset/stale-sample implications and Bosch self-test API are not user-facing/core-complete.
- Partial: humidity caveats exist in extracted docs but not README/Doxygen-facing docs.

### core-contracts-agent

- True: core framework neutrality and injected/non-owning transport are intact.
- True: copy/move deletion exists.
- Partial/false: `tick()` performs fallible I/O but returns `void`.
- Partial/false: dirty hardware-config diagnostics do not cover every ambiguous partial-write case.
- Missing: sample cache generation/invalidation across config changes.
- Missing: thread/ISR and recursive-callback cautions in public Doxygen headers.
- Partial: NVM wait can collapse transport errors into generic timeout.

### fault-injection-agent

- Partial: fake transport supports whole-call and call-indexed failures but not byte-level partial writes.
- Partial: NACK/timeout/bus/data paths are tested in several places, but not as full begin/probe/tick/softReset matrices.
- True with gaps: dirty-state tests cover restore failures, recover clearing, and preserved original errors, but not begin/softReset config-apply failures.
- True: offline/recover behavior is tested.
- True: `requestMeasurement()` no-clock behavior is tested.
- False/gap: measurement `tick()` failure injection is absent.

### idf-ci-agent

- True: ESP-IDF component metadata and native IDF example exist, and the IDF boundary is currently clean.
- Partial/false: CI builds ESP-IDF `release-v5.3` while metadata/docs claim `>=6.0.1`.
- Missing: CI does not run `scripts/generate_version.py check`.
- Partial/false: `pio pkg pack` can pass while omitting ignored generated `Version.h`.
- Partial: native PlatformIO env inherits shared Arduino `Wire` dependency.

### docs-hardware-agent

- High: production-grade wording is stronger than recorded validation evidence.
- High: stale readiness report remains in docs and Doxygen input.
- Medium: wiring and humidity caveats are not README/Doxygen-facing enough.
- Medium: no production shared-bus template exists.
- Medium: Doxygen warning settings are permissive and public API comments miss some safety contracts.
- Observation: no dedicated hardware validation matrix/log artifact exists.

### integration-review-agent

- Reviewed the final Phase 01 diff before commit; no blocking findings.

## Commands Run

Start-of-phase:

```text
git status --short
git branch --show-current
git log --oneline -8
```

Inspection commands included:

```text
rg --files
rg -n "Arduino|Wire|Serial|String|TwoWire|esp_|freertos|driver/|millis|delay\(" include src
rg -n "0xD0|0x60|0xE0|0xB6|0xF3|0xF7|0xFE|0x88|0xA1|0xE1|0xE7|0x76|0x77|CHIP|RESET|STATUS|CTRL_HUM|CTRL_MEAS|CONFIG|DATA|CALIB" include src
rg -n "hardwareConfigDirty|dirty|requestMeasurement|nowMs|probe|recover|DEVICE_NOT_FOUND|ADDRESS_NACK|I2C_|offlineThreshold|copy|operator=|BME280\(" include src test
rg -n "esp32s2|esp32s3|idf|platformio|check_core|check_cli|check_idf|Arduino|Wire|TwoWire|ArduinoCompat|IdfArduinoCompat|driver/i2c_master|app_main" .github platformio.ini CMakeLists.txt idf_component.yml examples tools test
rg -n "SDO|CSB|VDDIO|reflow|humidity|reconditioning|hardware validation|validation|self-test|self test|burst|0x80000|0x8000|ctrl_hum|config|IIR|filter|t_fine|Bosch|datasheet|skipped|sample|generation" README.md AGENTS.md docs examples include src test
Get-Command idf.py -ErrorAction SilentlyContinue
```

Verification results:

```text
python tools/check_core_timing_guard.py
PASS: Core timing guard PASSED

python tools/check_cli_contract.py
PASS: CLI contract PASSED

python tools/check_idf_example_contract.py
PASS: IDF example contract PASSED

python scripts/generate_version.py check
PASS: include/BME280/Version.h is up to date

python -m platformio test -e native
PASS: native:* PASSED, 41 test cases, 41 succeeded in 00:00:02.296

python -m platformio run -e esp32s3dev
PASS: esp32s3dev SUCCESS in 00:00:38.824

python -m platformio run -e esp32s2dev
PASS: esp32s2dev SUCCESS in 00:00:37.164
```

Commands not run:

```text
idf.py -C examples/idf/basic set-target esp32s3
idf.py -C examples/idf/basic build
idf.py -C examples/idf/basic set-target esp32s2
idf.py -C examples/idf/basic build
hardware validation commands
```

Reason: `idf.py` is not available in this shell, and this phase explicitly does not run hardware tests.
