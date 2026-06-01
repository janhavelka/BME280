# BME280 Industrial Readiness Exploration Report

Audit date: 2026-06-01

Scope: exploration/audit only. No source code, tests, docs, metadata, or CI files were intentionally changed except this report. PlatformIO commands created ignored build artifacts under `.pio`; `python -m platformio pkg pack` created `BME280-1.5.0.tar.gz`, which was removed after package inspection as requested.

## Git State

Branch: `main`

HEAD: `32663b5d747a25a54376e7b2b79f14338afbf4fd`

Describe: `v1.5.0-22-g32663b5-dirty`

Worktree before writing this report: dirty.

Dirty files recorded at audit start:

```text
 M CHANGELOG.md
 M Doxyfile
 M README.md
 M docs/BME280_HARDWARE_VALIDATION_MATRIX.md
 M docs/BME280_INDUSTRY_HARDENING_SUMMARY.md
 M docs/I2C_HIL_RUNBOOK.md
 M docs/I2C_HIL_TARGET_TEMPLATE.md
 M docs/IDF_PORT.md
 M include/BME280/BME280.h
 M include/BME280/Config.h
 M tools/check_hil_contract.py
?? docs/README.md
```

Recent commits:

```text
32663b5 Merge pull request #2 from janhavelka:hardening/bme280-industry-readiness
047cbcb Fix I2C HIL runner output matching
d7929ec docs: consolidate BME280 industry readiness docs
4683368 delete obsolete documentation files: BME280_PHASE_05_EXAMPLES_IDF_CI_DOCS_REPORT.md, BME280_PRE_HIL_READINESS_REPORT.md, BME280_PROMPTS_00_06_COMPREHENSIVE_REPORT.md, I2C_HIL_SELFTEST_REPORT.md; add BME280_INDUSTRY_HARDENING_SUMMARY.md to summarize industry readiness; update check_hil_contract.py to include new summary checks and remove references to deleted reports; modify IDF_PORT_IMPLEMENTATION.md for clarity on ESP-IDF checks; ensure consistency in command execution across documentation and scripts.
ceec963 Add I2C HIL self-test runner
da5946a hardening: update I2C HIL self-test report with new dry-run timestamp
038df90 Add BME280 I2C HIL runner and documentation
e4e859c hardening: prepare BME280 for HIL validation
2c4098e hardening: finalize BME280 industry-standard report
bdab940 hardening: improve BME280 examples and CI docs
```

## Executive Summary

The repository is strong as a software-hardened BME280 library. The core is framework-neutral, transport-injected, bounded, and well covered by native fault-injection tests. Local PlatformIO native tests passed 88/88, Arduino ESP32-S3 and ESP32-S2 builds passed, the version header check passed, the CLI and IDF example contract checks passed, the HIL contract check passed, and package creation plus current package-content checking passed.

The repository is not ready to claim "industry-grade" without qualification. The main blocker is missing physical hardware evidence: the hardware validation matrix still records all physical rows as `NOT RUN`. Release readiness is also blocked by dirty/uncommitted files, post-`v1.5.0` unreleased API changes while version metadata still says `1.5.0`, and unresolved SemVer/migration questions around public struct layout and copy/move behavior.

I recommend proceeding to controlled exploratory HIL, but not using the current default HIL run as complete formal evidence until the forced-mode post-trigger sleep-return capture gap is fixed or manually added. I do not recommend merging the current dirty changes until the package checker, HIL contract CI coverage, untracked docs, and release-note/migration issues are handled. I do not recommend a public "industry-grade" release before hardware validation.

## Scorecard

| Area | Rating | Findings |
| --- | --- | --- |
| Core architecture | Strong, with medium diagnostic caveats | Core/public headers and `src/` are framework-neutral by scan and guard. I2C is injected through `Config` callbacks. Copy/move are deleted. Public APIs are not thread/ISR-safe by contract. Risks: diagnostic raw writes can desynchronize config without setting dirty state, and health counters are documented as lifetime but reset in `begin()` (`src/BME280.cpp:165`). |
| Datasheet correctness | Strong | Chip ID `0x60`, address validation `0x76/0x77`, reset `0xB6`, `im_update` polling, `ctrl_hum` then `ctrl_meas`, sleep/config/restore sequencing, raw burst read `0xF7..0xFE`, skipped sentinels, Bosch-style compensation, humidity clamp, pressure denominator guard, and timing formulas are implemented and tested. |
| Tests/fault injection | Strong software coverage | Native test run passed 88/88. FakeBus covers address NACK, data NACK, timeout, bus error, reset/NVM failure, partial config failure, dirty clearing, recover/offline, compensation, and cache behavior. Remaining gap: FakeBus is call-level, not byte-level inside a multi-byte write. |
| CI/build/package | Good, not release-complete | Local PlatformIO native/ESP32-S3/ESP32-S2 checks passed. CI defines Arduino builds, native tests, package validation, and ESP-IDF example builds. Local `idf.py` is unavailable. Package checker passed after packaging, but it does not require IDF transport `.cpp`/`.h` files even though the IDF example needs them. Tooling versions are partly unpinned. |
| Docs/examples | Conservative and mostly accurate | README and docs avoid claiming hardware validation and label examples as diagnostic bring-up CLIs. Doxygen input includes public docs and headers, but also `AGENTS.md`; Doxygen warning strictness is off. `docs/README.md` is referenced but currently untracked. |
| HIL readiness | Procedure-ready, evidence not ready | HIL runner/runbook/template/matrix exist and `check_hil_contract.py` passed. Hardware matrix is all `NOT RUN`. Default sequence lacks the matrix-required post-forced-mode sleep-return capture (`mode`, `status`, or `reg 0xF4`). Evidence fields differ between matrix/template/runbook. |
| Release readiness | Not ready | Current tree is `v1.5.0-22-g32663b5-dirty`; metadata still says `1.5.0`; changelog has substantial `[Unreleased]` content. Public layout/API changes need explicit release notes and likely a SemVer decision. Generated local Doxygen may contain unreleased API under a `1.5.0` label. |

## Top 10 Remaining Risks

1. **Blocker - no physical hardware validation evidence.** `docs/BME280_HARDWARE_VALIDATION_MATRIX.md` still records setup fields and all physical rows as `NOT RUN`, including address, chip ID, reset, forced/normal mode, burst coherency, calibration, compensation, fault mapping, shared bus, and long soak.

2. **Blocker - current tree is not releaseable as `1.5.0`.** `git describe` reports `v1.5.0-22-g32663b5-dirty`, while `library.json`, `idf_component.yml`, `Doxyfile`, and `Version.h` still say `1.5.0`. `CHANGELOG.md` contains substantial `[Unreleased]` API/package/docs changes.

3. **Blocker - worktree is dirty and release-critical docs are untracked.** There are 11 modified tracked files plus untracked `docs/README.md`, which is referenced by README, Doxyfile, and the hardening summary. This blocks clean merge/release decisions.

4. **High - public API/layout compatibility is not resolved.** `Measurement`, `RawSample`, `CompensatedSample`, and `SettingsSnapshot` have new fields; `BME280` copy and move operations are deleted (`include/BME280/BME280.h:135`). These are source/layout compatibility changes that need explicit migration wording and SemVer treatment.

5. **High - package checker can miss required IDF example files.** `examples/idf/basic/main/CMakeLists.txt` builds `IdfI2cTransport.cpp`, and `main.cpp` includes `IdfI2cTransport.h`, but `tools/check_package_contents.py` does not require those files. A future broken package could pass the checker.

6. **Medium - HIL forced-mode evidence sequence is incomplete.** The matrix requires sleep-return evidence after forced mode, but the default runner/runbook sequence goes `force`, `read`, `normal on` without `mode`, `status`, or `reg 0xF4`.

7. **Medium - HIL contract is not in CI and evidence fields are inconsistent.** `python tools/check_hil_contract.py` passed locally, but `.github/workflows/ci.yml` does not run it. The runbook/template ask for firmware/library/runner evidence that the hardware matrix setup table does not fully mirror.

8. **Medium - diagnostic raw write APIs can desynchronize config without setting dirty state.** `writeRegisters()` and `writeRegister()` are documented as diagnostic and warn users to recover/resync, but they call tracked register helpers without marking `hardwareConfigDirty()` for config/control writes. This weakens strict "cache synchronized or dirty" claims.

9. **Medium - NVM polling deadline is only real with an injected timebase.** `_waitForNvmReady()` is bounded by both deadline and poll count, but without `Config::nowMs`, `platform::nowMs()` returns `0`, so the path relies on `NVM_READY_MAX_POLLS`. It remains bounded but can exceed the nominal 10 ms wall time if transport callbacks use long timeouts.

10. **Medium - sample freshness after recovery is documented but not strongly encoded.** `recover()` preserves pre-recovery cached samples and getters still return them when `_hasSample` is true. Docs warn users to request a fresh sample, but there is no generation tag or automatic invalidation on successful recovery.

## Specific Questions

| Question | Answer |
| --- | --- |
| 1. Is the core truly framework-neutral? | Yes by scan and `check_core_timing_guard.py`. `include/` and `src/` had no Arduino/Wire/Serial/String/TwoWire/ESP-IDF/FreeRTOS/delay hits. `src/PlatformTime.h` is framework-neutral and returns `0` fallback time. |
| 2. Does the driver ever own or reset the I2C bus internally? | No. The core uses injected callbacks from `Config`. It issues BME280 soft reset to register `0xE0`, but does not create, configure, reset, or recover the I2C bus. |
| 3. Are all fallible operations status-returning and diagnostically useful? | Mostly yes. Fallible public APIs return `Status`; `tick()` is intentionally void and records result in `lastMeasurementStatus()`. `end()` uses a best-effort raw sleep write. |
| 4. Are transport errors preserved? | Core mostly preserves timeout, bus, data NACK, and generic I2C errors. Definite address NACK maps to `DEVICE_NOT_FOUND` in begin/probe identity paths. The ESP-IDF example adapter cannot distinguish all NACK types and maps `ESP_ERR_INVALID_RESPONSE` to generic `I2C_ERROR`. |
| 5. Are validation/precondition errors separated from hardware health failures? | Yes. Tracked wrappers skip health updates for `INVALID_CONFIG`/`INVALID_PARAM`, precondition paths return `NOT_INITIALIZED`, and native tests cover these cases. |
| 6. Is dirty hardware-config state observable and cleared only after real resync? | Yes for typed config/reset/recover paths. Dirty state is exposed via `hardwareConfigDirty()` and cleared after successful `begin()`, `recover()`, or `softReset()` resync. Caveat: diagnostic raw writes do not set dirty state. |
| 7. Are BME280 register sequencing rules enforced and tested? | Yes. `ctrl_hum` is followed by `ctrl_meas`; config/filter/standby writes use sleep/config/restore; raw sample read bursts `0xF7..0xFE`; tests cover these sequences. |
| 8. Are forced, normal, and sleep mode semantics correct and documented? | Yes in software. Forced mode is on-demand and hardware remains sleep until `requestMeasurement()`; normal mode schedules fresh-cycle polling; sleep cancels pending measurement. Docs describe this. |
| 9. Is sample freshness/cache validity clear enough for production users? | Mostly. Per-channel validity flags and sample timestamps are documented. Recovery-preserved cached samples remain a medium risk without a generation tag. |
| 10. Is compensation math covered well enough? | Strongly for software. Tests cover calibration parsing, H4/H5 nibble packing, `t_fine`, fixed-point compensation, humidity clamp, skipped sentinels, and pressure denominator guard. Hardware plausibility is not validated. |
| 11. Are reset/NVM paths bounded and diagnostically precise? | Yes, bounded by deadline and poll cap. First transport error is preserved when no status read succeeds; stuck `im_update` becomes `TIMEOUT`. Caveat: deadline requires injected `nowMs` for real wall-time semantics. |
| 12. Are public structs/API changes documented for release notes? | Partly. Changelog `[Unreleased]` mentions added features, but migration/SemVer impact is not resolved for copy/move deletion and public struct layout changes. |
| 13. Are examples honest: diagnostic vs production/shared-bus integration? | Yes. README labels Arduino and IDF examples as diagnostic bring-up CLIs and states production shared-bus users must provide ownership, locking, scheduling, and timeout policy. A full production shared-bus example is still absent. |
| 14. Is ESP-IDF support actually built in CI, or only documented? | CI is configured to build `examples/idf/basic` for ESP32-S3 and ESP32-S2 using ESP-IDF v6.0.1. Local `idf.py` was unavailable, so local pure IDF builds were not run. |
| 15. Is the package artifact clean and complete? | Current package creation and current checker passed, and the archive was removed. However, the checker misses required IDF transport files, so package completeness enforcement is insufficient. |
| 16. Are stale reports or old claims likely to confuse users? | Reduced but not eliminated. Docs consolidate prior reports and warn about no hardware validation. Local generated Doxygen under `docs/doxygen/` may contain unreleased API labeled `1.5.0`; `docs/README.md` is untracked but referenced. |
| 17. Is HIL ready, and what evidence is still missing? | Ready for exploratory HIL, not formal industry-grade evidence. Missing: real board details, wiring, pullups, voltages, address/CSB/SDO, chip ID, reset/NVM, forced sleep-return, normal repeated reads, logic-analyzer/trace for burst read, calibration output, environmental references, fault bench, shared-bus integration, and soak. |
| 18. What blocks "industry-grade" rather than "software-hardened"? | Physical HIL evidence, fault-bench evidence, environmental/reference measurement evidence, shared-bus integration proof, long soak data, SemVer/release cleanup, package-check hardening, and CI coverage for HIL docs. |

## Evidence Table

| Command | Result |
| --- | --- |
| `git status --short` | INFO: dirty tree with 11 modified tracked files and untracked `docs/README.md` before this report. |
| `git branch --show-current` | PASS: `main`. |
| `git rev-parse HEAD` | PASS: `32663b5d747a25a54376e7b2b79f14338afbf4fd`. |
| `git log --oneline -10` | PASS: recorded recent commits above. |
| `git describe --tags --dirty --always` | INFO: `v1.5.0-22-g32663b5-dirty`. |
| `bash -lc "find . -maxdepth 3 -type f \| sort"` | FAIL: WSL has no installed distributions on this Windows host. |
| PowerShell file inventory fallback | PASS: repository files were enumerated with `Get-ChildItem` and `rg --files`. |
| `rg -n "Arduino\.h\|Wire\.h\|Serial\|String\|TwoWire\|driver/i2c\|freertos\|esp_timer\|vTaskDelay\|delay\(" include src` | PASS: no core hits; exit code 1 from no matches. |
| `rg -n "TODO\|FIXME\|HACK\|XXX\|pending\|not run\|not tested\|stub\|placeholder\|TBD" .` | REVIEWED: hits are docs/examples/test stubs/platformio text; no core TODO/FIXME/HACK blockers found. |
| `rg -n "hardwareConfigDirty\|dirty\|recover\|softReset\|im_update\|ctrl_hum\|ctrl_meas\|config\|t_fine\|RAW_.*SKIPPED" include src test docs README.md` | PASS/INFO: matched expected implementation, tests, and docs. |
| `rg -n "idf.py\|esp-idf\|idf_component\|CMakeLists\|platformio\|library.json\|Version.h" .` | PASS/INFO: matched expected CI/package/version metadata. |
| `python tools/check_core_timing_guard.py` | PASS: `Core timing guard PASSED`. |
| `python tools/check_cli_contract.py` | PASS: `CLI contract PASSED`. |
| `python tools/check_idf_example_contract.py` | PASS: `IDF example contract PASSED`. |
| `python tools/check_hil_contract.py` | PASS: `HIL contract PASSED`. |
| `python scripts/generate_version.py check` | PASS: `Version.h` up to date. |
| `python -m platformio test -e native` | PASS: 88 test cases, 88 succeeded, 49.30 s. |
| `python -m platformio run -e esp32s3dev` | PASS: build success, 68.24 s; RAM 6.9%, flash 29.5%. |
| `python -m platformio run -e esp32s2dev` | PASS: build success, 45.03 s; RAM 11.3%, flash 28.7%. |
| `python -m platformio pkg pack` | PASS: wrote `BME280-1.5.0.tar.gz`. |
| `python tools/check_package_contents.py` | PASS: `Package contents PASSED (BME280-1.5.0.tar.gz)`. |
| Archive cleanup | PASS: `BME280-1.5.0.tar.gz` removed after inspection. |
| `git diff --check` | PASS: exit code 0; Git printed CRLF replacement warnings for dirty files. |
| `Get-Command idf.py` / `idf.py --version` | FAIL/UNAVAILABLE: `idf.py` not found on PATH. Pure local ESP-IDF builds were not run. |
| `python tools/run_i2c_hil.py --help` | PASS: CLI help rendered. No serial HIL was run by this parent audit. |

## Safe Claims

Safe for README/release notes if kept evidence-bound:

- "Framework-neutral core: no Arduino or ESP-IDF framework headers in `include/` or `src/`."
- "Transport-injected driver; the library does not own the I2C bus."
- "Native fault-injection tests pass locally: 88/88."
- "PlatformIO Arduino builds pass locally for `esp32s3dev` and `esp32s2dev`."
- "CI is configured to build the native ESP-IDF example for ESP32-S3 and ESP32-S2 with ESP-IDF v6.0.1."
- "ESP-IDF local `idf.py` validation is not claimed unless exact command results are recorded."
- "Adds HIL runner/runbook/template and a conservative hardware validation matrix with unrun rows marked `NOT RUN`."
- "Examples are diagnostic bring-up CLIs, not production shared-bus firmware templates."
- "Compensation, skipped sentinels, dirty config diagnostics, reset/NVM error paths, and granular core transport errors have native test coverage."

## Unsafe Claims

Unsafe unless additional evidence or fixes exist:

- "Industry-grade", "production-qualified", or "hardware validated."
- "Released as v1.5.0" for the current dirty post-tag tree.
- "Fully backward compatible" without addressing public layout and copy/move changes.
- "Local pure ESP-IDF builds passed" on this machine.
- "Package completeness is fully enforced" until IDF transport files are required by the checker.
- "HIL validated" or "HIL ready as formal evidence" using the current default sequence without post-forced-mode sleep-return capture.
- "Lifetime health counters" if `begin()` reset semantics remain unchanged or undocumented.
- "Dirty state covers all public writes" while diagnostic raw writes can bypass dirty marking.
- "ESP-IDF adapter preserves address/data NACK distinctions" when the example adapter maps some IDF errors generically.

## Required Fixes Before HIL

For exploratory bench HIL, proceed only if the operator records the dirty tree state and manually adds the missing post-force `mode`, `status`, or `reg 0xF4` capture.

For formal HIL evidence:

1. Add post-forced-mode sleep-return evidence to `tools/run_i2c_hil.py` and `docs/I2C_HIL_RUNBOOK.md`, or adjust the matrix to match the real sequence.
2. Align evidence fields across `docs/BME280_HARDWARE_VALIDATION_MATRIX.md`, `docs/I2C_HIL_TARGET_TEMPLATE.md`, and the runbook for firmware version, library version, runner command, commit, dirty state, target, board, wiring, pullups, voltages, and references.
3. Use a clean committed baseline for formal HIL, or record every dirty file and diff hash in the evidence package.
4. Record physical setup: ESP32-S2/S3 board, BME280 module, VDD/VDDIO, SDA/SCL pins, pull-up values, SDO state, CSB state, bus speed, and serial port.

## Required Fixes Before Merge

1. Resolve the dirty worktree intentionally: stage/commit intended docs/header/tool changes, and either track or remove `docs/README.md`.
2. Update `tools/check_package_contents.py` to require `examples/idf/basic/main/IdfI2cTransport.cpp` and `examples/idf/basic/main/IdfI2cTransport.h`.
3. Add `python tools/check_hil_contract.py` to CI.
4. Fix or explicitly document the HIL forced-mode evidence gap.
5. Decide whether public raw writes to config/control registers should set `hardwareConfigDirty()` or remain strictly diagnostic with narrower claims.
6. Correct "lifetime" health counter wording or preserve counters across `begin()`.

## Required Fixes Before Public Release

1. Decide SemVer. The public API/layout changes and deleted copy/move operations need explicit release classification and migration notes.
2. Bump `library.json` and regenerate `Version.h`; synchronize `idf_component.yml` and `Doxyfile PROJECT_NUMBER`, or extend tooling/CI to check them.
3. Move `[Unreleased]` changelog entries into the chosen release section with safe wording and migration guidance.
4. Ensure generated Doxygen, if published, is regenerated from the release tree and does not label unreleased API as `1.5.0`.
5. Decide whether `AGENTS.md` should be part of published Doxygen input.
6. Fix the package checker gap and rerun `pkg pack` plus package contents check.
7. Do not claim "industry-grade" until the hardware validation matrix or attached HIL artifacts show real physical evidence.

## Optional Polish Items

- Pin PlatformIO/espressif32 versions or record expected versions in CI to reduce build drift.
- Strengthen Doxygen gates with `WARN_IF_UNDOCUMENTED`, `WARN_NO_PARAMDOC`, and possibly `WARN_AS_ERROR`.
- Add sample generation/config generation tagging so callers can distinguish pre-recovery samples without relying only on docs.
- Add byte-level FakeBus fault injection for multi-byte diagnostic writes.
- Add an optional production shared-bus example showing external locking, timeout policy, and scheduler ownership.
- Extend the ESP-IDF example adapter docs to be explicit about NACK mapping limitations.
- Fix historical changelog link reference coverage, including the missing `[1.2.2]` link.

## Follow-Up Implementation Prompts

### Must Do Before HIL

```text
Update the HIL runner and docs so the default sequence captures forced-mode sleep-return evidence. Add a `mode`, `status`, or `reg 0xF4` step after `force`/`read`, update the matrix/runbook/template consistently, run `python tools/check_hil_contract.py`, and do not change unrelated source code.
```

```text
Align HIL evidence fields across `docs/BME280_HARDWARE_VALIDATION_MATRIX.md`, `docs/I2C_HIL_RUNBOOK.md`, and `docs/I2C_HIL_TARGET_TEMPLATE.md` for firmware version, library version, runner command, commit, worktree state, hardware wiring, pullups, voltage rails, address, references, and operator sign-off.
```

### Must Do Before Merge

```text
Harden package validation by adding `examples/idf/basic/main/IdfI2cTransport.cpp` and `examples/idf/basic/main/IdfI2cTransport.h` to `tools/check_package_contents.py`, then run `python -m platformio pkg pack`, `python tools/check_package_contents.py`, and remove the generated archive.
```

```text
Add `python tools/check_hil_contract.py` to the GitHub Actions validation job. Keep it near the CLI and IDF example contract checks, then verify the local command still passes.
```

```text
Review public raw register write semantics. Either mark hardware config dirty when raw writes target `0xF2`, `0xF4`, `0xF5`, or other config/control ranges, or narrow the documentation so the strict dirty-state guarantee applies only to typed configuration APIs.
```

### Must Do Before Release

```text
Prepare release metadata for the current post-v1.5.0 API surface. Choose the SemVer version, update `library.json`, regenerate `Version.h`, synchronize `idf_component.yml` and `Doxyfile`, convert `[Unreleased]` changelog entries into the release section, and add migration notes for public struct layout and copy/move deletion.
```

```text
Run the full release check suite on a clean tree: `python tools/check_core_timing_guard.py`, `python tools/check_cli_contract.py`, `python tools/check_hil_contract.py`, `python tools/check_idf_example_contract.py`, `python scripts/generate_version.py check`, `python -m platformio test -e native`, `python -m platformio run -e esp32s3dev`, `python -m platformio run -e esp32s2dev`, `python -m platformio pkg pack`, `python tools/check_package_contents.py`, `git diff --check`, and local `idf.py` builds when available.
```

### Optional Later Hardening

```text
Add sample generation tracking to `RawSample`, `CompensatedSample`, and `SettingsSnapshot`, increment the generation after config changes/recover/reset, and document how callers can reject stale cached samples.
```

```text
Pin CI build tooling versions or document the accepted floating-toolchain policy. Include PlatformIO, espressif32 platform, and ESP-IDF action version rationale.
```

```text
Add a production shared-bus example that shows application-owned I2C bus setup, mutex/lock ownership, timeout policy, scheduled `tick()`, and no driver calls from ISR context.
```
