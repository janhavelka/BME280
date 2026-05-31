# BME280 Phase 01 Baseline Fact Lock Report

Branch: `hardening/bme280-industry-readiness`

Baseline HEAD: `0fd07f8130f9431711afe050c10246f8426fc7fb`

Phase scope: baseline audit, datasheet fact lock, AGENTS.md hardening rules, and durable plan for later phases. No compensation rewrites, reset rewrites, CLI expansion, or hardware validation were performed in this phase.

## Files Inspected

- `AGENTS.md`
- `README.md`
- `include/BME280/BME280.h`
- `include/BME280/CommandTable.h`
- `include/BME280/Config.h`
- `include/BME280/Status.h`
- `include/BME280/Version.h`
- `src/BME280.cpp`
- `src/PlatformTime.h`
- `test/test_basic.cpp`
- `test/stubs/Arduino.h`
- `test/stubs/Wire.h`
- `examples/01_basic_bringup_cli/main.cpp`
- `examples/common/*`
- `examples/idf/basic/*`
- `tools/check_core_timing_guard.py`
- `tools/check_cli_contract.py`
- `tools/check_idf_example_contract.py`
- `.github/workflows/ci.yml`
- `library.json`
- `platformio.ini`
- `CMakeLists.txt`
- `idf_component.yml`
- Existing BME280 reports and references under `docs/`, including `docs/BME280_INDUSTRY_READINESS_REPORT.md`, `docs/BME280_HARDENING_FINAL_REPORT.md`, `docs/BME280_Register_Reference.md`, extracted datasheet notes, and `docs/BME280_datasheet.pdf`.

## Subagents Used

- `bme280-datasheet-agent`: verified register, mode, raw-data, calibration, compensation, timing, self-test, and humidity facts against implementation and docs.
- `core-contracts-agent`: verified framework neutrality, transport ownership, timebase, Status behavior, copy/move deletion, dirty-state semantics, and sample-cache semantics.
- `fault-injection-agent`: inspected fake transport tests, transport error preservation, dirty-state tests, offline/recover behavior, and no-clock behavior.
- `idf-ci-agent`: inspected ESP-IDF component metadata, native IDF example boundaries, CI jobs, guard scripts, and stale validation claims.
- `docs-hardware-agent`: inspected README/docs/examples for diagnostic wording, wiring notes, hardware validation status, shared-bus guidance, and humidity caveats.
- `integration-review-agent`: reviewed the final Phase 01 diff before commit; no blocking findings.

## Datasheet Source

Primary authority: Bosch Sensortec BME280 data sheet, document `BST-BME280-DS001-24`, revision 1.24, February 2024. The repository copy is `docs/BME280_datasheet.pdf`; the official source URL checked during this phase was:

`https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme280-ds002.pdf`

## Datasheet Facts Locked For Later Phases

1. Chip ID register `0xD0` reads `0x60`.
2. Soft reset is register `0xE0`, value `0xB6`.
3. Status register `0xF3` exposes `measuring` on bit 3 and `im_update` on bit 0; `im_update` is set during NVM copy and clears when copying is complete.
4. Valid I2C addresses are `0x76` and `0x77`, selected by SDO; SDO must not float.
5. CSB must be tied to VDDIO for I2C. If CSB is pulled low once, I2C can remain disabled until the next power-on reset.
6. VDD and VDDIO are separate supplies. Interface pins must not be held high while VDDIO is off.
7. Sleep mode is the startup/default mode and all registers are accessible in sleep.
8. Forced mode performs one measurement, stores results, and returns to sleep.
9. Normal mode cycles between active measurement and standby.
10. `ctrl_hum` changes only become effective after a write to `ctrl_meas`.
11. `config` writes in normal mode may be ignored; config/filter/standby changes must be applied from sleep, then the prior mode restored if needed.
12. Pressure and temperature raw outputs are unsigned 20-bit values; humidity raw output is unsigned 16-bit.
13. Pressure/temperature skipped sentinel is `0x80000`; humidity skipped sentinel is `0x8000`.
14. Coherent pressure/temperature/humidity samples require one burst read of `0xF7..0xFE`.
15. Calibration coefficients occupy `0x88..0xA1` and `0xE1..0xE7`; `dig_H4` and `dig_H5` are signed 12-bit nibble-packed values across `0xE4`, `0xE5`, and `0xE6`.
16. Compensation uses `t_fine`; pressure and humidity depend on the temperature value from the same sample context.
17. Best pressure accuracy uses the Bosch 64-bit pressure path.
18. Humidity compensation must clamp to the valid range.
19. The IIR filter applies to pressure and temperature, not humidity; writing filter settings resets filter memory.
20. Measurement time is computed from selected oversampling, and normal-mode rate includes standby time.
21. Bosch self-test starts with reset, chip ID, trimming-data read, measurement, and plausibility checks; default plausibility limits may not match every environment.
22. Humidity performance may be temporarily degraded by reflow or operating outside the valid range; docs must include handling/reconditioning caveats.

## Current Audit Claim Verification

| Claim | Result | Evidence / correction |
| --- | --- | --- |
| Framework-neutral core | True | `rg` found no Arduino, Wire, Serial, String, ESP-IDF, or FreeRTOS includes/calls in `include/` or `src/`; `tools/check_core_timing_guard.py` passed. |
| Injected/non-owning I2C transport | True | `Config` exposes `i2cWrite`, `i2cWriteRead`, `i2cUser`; raw wrappers call callbacks only. Core does not create buses, pins, locks, or timeouts. |
| Chip-ID validation | True | `begin()` reads `cmd::REG_CHIP_ID` and requires `cmd::CHIP_ID_BME280`; `probe()` performs the same check diagnostically. |
| Calibration parsing | True, test coverage partial | Implementation reads `0x88..0xA1`, `0xA1`, and `0xE1..0xE7`; H4/H5 sign extension exists. Boundary/golden-vector tests are deferred to Phase 02. |
| Explicit measurement timing | True | `estimateMeasurementTimeMs()` uses oversampling-derived max timing plus margin; `estimateNormalCycleMs()` adds standby. Native tests cover forced wraparound and normal cycle wait. |
| Dirty hardware-config state for partial writes | Partial | Dirty diagnostics exist and tests cover later multi-step failures. First-write failures in multi-register paths are not marked dirty even though a transport error may hide a completed write. Later phase must tighten this. |
| `requestMeasurement()` rejects missing `Config::nowMs` | True | Returns `INVALID_CONFIG` before scheduling or I2C writes. `begin()` remains allowed without a clock. |
| `begin()` / `probe()` preserve transport errors except definite address NACK | Partial | Initial chip-ID failures preserve timeout, bus, data-NACK, and generic errors except address NACK maps to `DEVICE_NOT_FOUND`. `begin()` NVM polling currently swallows repeated I2C errors and returns generic `TIMEOUT`. |
| Copy/move deleted | True | `BME280` copy/move construction and assignment are deleted; native compile-time test covers this. |
| Native tests pass | True | `python -m platformio test -e native` passed 41/41 tests in this phase. |
| Arduino ESP32-S2/S3 builds pass | True | `python -m platformio run -e esp32s3dev` and `python -m platformio run -e esp32s2dev` both passed in this phase. |
| ESP-IDF CI jobs exist | True with mismatch | `.github/workflows/ci.yml` has an `esp-idf-build` job for `esp32s3` and `esp32s2`, but it uses `release-v5.3` while docs/metadata claim ESP-IDF `>=6.0.1`. |
| Hardware validation not yet run | True | Existing reports state no hardware validation; no hardware commands were run in this phase. |

## Corrected Assumptions

- The older readiness report says CI does not build the pure ESP-IDF example. That is stale: the workflow now includes an ESP-IDF S2/S3 job, but it targets ESP-IDF `release-v5.3`, not the declared v6.0.1 line.
- Dirty hardware-config diagnostics exist, but they are not yet complete enough to claim every possible partial hardware write is diagnosed.
- Sentinel handling is documented and visible in example diagnostics, but the core does not yet expose a first-class `DATA_SKIPPED`/validity model and does not reject raw sentinels before compensation.
- `tick()` is intentionally `void` in the current lifecycle, but it performs I2C and compensation work. I2C errors are reflected through health state; semantic compensation failures are not returned to the caller. This remains a public contract gap to address deliberately.
- Hardware validation and local pure ESP-IDF validation were not performed. `idf.py` was not available in this shell.

## Implementation Plan For Phases 02-06

### Phase 02 - Calibration and compensation vectors

- Add native tests covering all calibration coefficients, signed boundaries, H4/H5 nibble packing, all-zero/all-`0xFF` plausibility where supported, and `dig_P1 == 0`.
- Add Bosch-reference fixed compensation vectors with provenance and no imported Bosch source unless license/attribution is settled.
- Add raw burst decoding tests for pressure, temperature, and humidity.
- Decide and document skipped sentinel behavior so `0x80000`/`0x8000` cannot be silently presented as normal environmental data.

### Phase 03 - Config, timing, and sample semantics

- Tighten dirty-state behavior for every multi-register configuration failure, including first write failures where hardware state may be unknown.
- Decide whether config changes invalidate cached samples or tag samples with a config generation.
- Expand timing tests for oversampling combinations, standby rounding, forced-mode deadlines, and normal-mode freshness.
- Document any public API migration needed for sample validity/config generation.

### Phase 04 - Reset, NVM, fault diagnostics

- Preserve NVM/status-read transport errors more precisely instead of collapsing repeated I2C errors into generic timeout.
- Add reset/NVM fault-injection tests for timeout, address NACK, data NACK, bus, and generic I2C errors.
- Verify dirty-state clearing only after full successful resync/recover/reset paths.
- Reassess `tick()` status visibility and diagnostics for compensation or data-read failures.

### Phase 05 - Examples, IDF CI, docs

- Align ESP-IDF CI with the declared ESP-IDF version or correct the declared version/docs.
- Expand IDF guard coverage to scan all IDF example files, including headers and top-level CMake files.
- Add `scripts/generate_version.py check` to CI.
- Improve README/Doxygen-facing wiring notes for SDO, CSB, VDD/VDDIO, and humidity reconditioning caveats.
- Add production shared-bus guidance or an example showing ownership, locking, timeout policy, and scheduling.

### Phase 06 - Hardware validation and final report

- Create the final readiness report with exact commit range and phase summaries.
- Run real BME280 validation only if hardware is available, and record board, target, address, commands, raw/compensated sample ranges, reset/NVM behavior, and representative bus fault tests.
- Do not make strong production/field-grade claims until real hardware and representative bus fault validation are recorded.

## Risks And Blockers

- No local pure ESP-IDF build was run because `idf.py` was not present in this shell.
- Existing ESP-IDF CI target version appears inconsistent with the component metadata/docs.
- Compensation correctness still lacks Bosch-reference fixed vectors and boundary tests.
- Raw skipped sentinels are not yet a core Status/validity contract.
- True intra-transaction partial writes are not simulated by the current fake transport or Wire stub.
- Public Doxygen headers do not yet carry all thread/ISR, recursive callback, and wiring cautions.
- No real BME280 hardware validation has been run.

## Commands Run

Start-of-phase:

```text
git status --short
git branch --show-current
git log --oneline -5
```

Initial result: clean tree, branch `hardening/bme280-industry-readiness`, HEAD `0fd07f8`.

Inspection commands included:

```text
rg --files
rg -n "Arduino|Wire|Serial|String|TwoWire|esp_|freertos|driver/|millis|delay\(" include src
rg -n "0xD0|0x60|0xE0|0xB6|0xF3|0xF7|0xFE|0x88|0xA1|0xE1|0xE7|0x76|0x77|CHIP|RESET|STATUS|CTRL_HUM|CTRL_MEAS|CONFIG|DATA|CALIB" include src
rg -n "hardwareConfigDirty|dirty|requestMeasurement|nowMs|probe|recover|DEVICE_NOT_FOUND|ADDRESS_NACK|I2C_|offlineThreshold|copy|operator=|BME280\(" include src test
rg -n "esp32s2|esp32s3|idf|platformio|check_core|check_cli|check_idf|Arduino|Wire|TwoWire|ArduinoCompat|IdfArduinoCompat|driver/i2c_master|app_main" .github platformio.ini CMakeLists.txt idf_component.yml examples tools test
rg -n "SDO|CSB|VDDIO|reflow|humidity|reconditioning|hardware validation|validation|self-test|self test|burst|0x80000|0x8000|ctrl_hum|config|IIR|filter|t_fine|Bosch|datasheet|skipped|sample|generation" README.md AGENTS.md docs examples include src test
git rev-parse HEAD
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
PASS: native:* PASSED, 41 test cases, 41 succeeded

python -m platformio run -e esp32s3dev
PASS: esp32s3dev SUCCESS

python -m platformio run -e esp32s2dev
PASS: esp32s2dev SUCCESS

Get-Command idf.py -ErrorAction SilentlyContinue
NOT AVAILABLE: no idf.py found in this shell
```

Commands not run:

```text
idf.py -C examples/idf/basic set-target esp32s3
idf.py -C examples/idf/basic build
idf.py -C examples/idf/basic set-target esp32s2
idf.py -C examples/idf/basic build
```

Reason: `idf.py` was not available in this shell. No hardware validation commands were run because this phase is audit/report-only and no hardware execution was requested or available.
