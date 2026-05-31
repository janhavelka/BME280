# BME280 Prompts 00-06 Comprehensive Report

Date: 2026-05-31
Branch: `hardening/bme280-industry-readiness`
Current HEAD: `1c0d409807d43ad2790306d3bff6d20112e5b084`
Upstream HEAD: `1c0d409807d43ad2790306d3bff6d20112e5b084`

## Executive Summary

Prompts 00, 01, and 02 have repository evidence of execution on
`hardening/bme280-industry-readiness`. Prompts 03, 04, 05, and 06 were pasted
after Phase 02 work had started, but they were not executed as phases because
the workflow prompt requires each phase to finish before later phases begin.

Current repository state is clean and synced with upstream at
`1c0d409807d43ad2790306d3bff6d20112e5b084`.

No hardware validation was run. No local ESP-IDF build was run because `idf.py`
is not available in this shell. Do not claim field validation, hardware soak,
or local ESP-IDF validation from this work.

## Prompt Inventory

| Prompt | Title | Status | Primary artifact |
| --- | --- | --- | --- |
| 00 | Workflow orchestrator | Completed as branch/workflow setup; no code hardening phase executed | No dedicated report required by prompt |
| 01 | Baseline audit, fact lock, AGENTS rules | Completed | `docs/BME280_PHASE_01_BASELINE_FACT_LOCK_REPORT.md` |
| 02 | Compensation, calibration parsing, golden vectors | Completed in current HEAD | `docs/BME280_PHASE_02_COMPENSATION_CALIBRATION_REPORT.md` |
| 03 | Config sequencing, timing, sample semantics | Received, not executed as Phase 03 | Pending |
| 04 | Reset/NVM handling and fault diagnostics | Received, not executed as Phase 04 | Pending |
| 05 | Examples, ESP-IDF CI, packaging, documentation | Received, not executed as Phase 05 | Pending |
| 06 | Hardware validation, final integration review, final report | Received, not executed as Phase 06 | Pending |

## Commit Record

| Commit | Role in prompt sequence | Message |
| --- | --- | --- |
| `a75093507c9c3ee68934a89932e2dfe9d4a96fdb` | Initial Phase 01 readiness/fact-lock work | `hardening: lock BME280 readiness facts` |
| `5c0cf27960ad237af7382b1bd2ee745f79edd91f` | Revised Phase 01 baseline fact lock | `hardening: lock BME280 baseline facts` |
| `1c0d409807d43ad2790306d3bff6d20112e5b084` | Phase 02 calibration/compensation hardening | `feat(bme280): Enhance calibration and compensation handling` |

Note: Phase 02 requested commit message
`hardening: verify BME280 compensation vectors`; the current repository commit
uses `feat(bme280): Enhance calibration and compensation handling`.

## Prompt 00 - Workflow Orchestrator

Objective:

- Confirm the repository and branch.
- Establish the copy-pasted phased workflow.
- Avoid implementing hardening logic before Phase 01.

Observed outcome:

- Work proceeded on `hardening/bme280-industry-readiness`.
- The repository identity was confirmed as BME280: public headers under
  `include/BME280/`, implementation under `src/`, PlatformIO config,
  examples, tests, tools, docs, and BME280 metadata.
- No later-phase code work was started during this orchestrator pass.

Status: complete.

## Prompt 01 - Baseline Audit, Fact Lock, And AGENTS Rules

Objective:

- Audit the current repository against BME280 production-readiness facts.
- Add durable BME280 hardening rules to `AGENTS.md`.
- Produce a baseline report and defer implementation work to later phases.

Artifacts:

- `AGENTS.md`
- `docs/BME280_PHASE_01_BASELINE_FACT_LOCK_REPORT.md`

Key outcomes:

- Locked Bosch/BME280 facts for chip ID, reset, status bits, valid addresses,
  SDO/CSB/VDDIO wiring, sleep/forced/normal mode behavior, `ctrl_hum` latching,
  config-in-sleep behavior, raw widths, skipped sentinels, burst reads,
  calibration ranges, `t_fine`, 64-bit pressure compensation, humidity clamp,
  filter behavior, timing, self-test caveats, and humidity handling caveats.
- Verified the core was framework-neutral and transport-injected.
- Identified remaining gaps: compensation vectors, skipped-channel semantics,
  dirty-state edge cases, sample cache semantics, reset/NVM error precision,
  docs/hardware validation, package/CI alignment, and local ESP-IDF validation.

Phase 01 checks recorded in the phase report:

- `python tools/check_core_timing_guard.py` - PASS
- `python tools/check_cli_contract.py` - PASS
- `python tools/check_idf_example_contract.py` - PASS
- `python scripts/generate_version.py check` - PASS
- `python -m platformio test -e native` - PASS, 41/41 tests
- `python -m platformio run -e esp32s3dev` - PASS
- `python -m platformio run -e esp32s2dev` - PASS

Status: complete and committed.

## Prompt 02 - Compensation, Calibration Parsing, And Golden Vectors

Objective:

- Prove calibration parsing and compensation math.
- Add golden/synthetic vectors with documented provenance.
- Expose skipped-channel validity instead of silently returning plausible zeros.

Artifacts:

- `include/BME280/CommandTable.h`
- `include/BME280/BME280.h`
- `src/BME280.cpp`
- `test/test_basic.cpp`
- `README.md`
- `CHANGELOG.md`
- `examples/01_basic_bringup_cli/main.cpp`
- `examples/idf/basic/main/main.cpp`
- `docs/BME280_PHASE_02_COMPENSATION_CALIBRATION_REPORT.md`

Key changes in current HEAD:

- Added public skipped-sentinel constants:
  - `cmd::RAW_PRESSURE_SKIPPED`
  - `cmd::RAW_TEMPERATURE_SKIPPED`
  - `cmd::RAW_HUMIDITY_SKIPPED`
- Appended per-channel validity flags to `Measurement`, `RawSample`, and
  `CompensatedSample`.
- Derived raw sample validity at the burst-read boundary.
- Kept intentionally skipped pressure/humidity as successful samples with
  zero numeric compatibility fields and false validity flags.
- Rejected enabled-channel skipped sentinels during compensation.
- Preserved Bosch-style integer compensation flow:
  - temperature first
  - `t_fine` generated from temperature
  - 64-bit pressure path
  - pressure denominator zero guard
  - humidity clamp to 0..100 percent RH
- Invalidated cached samples after successful typed configuration changes.
- Made the native fake bus calibration reads register-driven.
- Added native tests for calibration parsing, H4/H5 nibble packing, raw burst
  reconstruction, compensation vector outputs, humidity clamps, skipped
  sentinels, denominator guard, and cache invalidation.

Vector provenance:

- Vectors are synthetic and datasheet-derived.
- They are not claimed as official Bosch vectors.
- They are not hardware validation.

Phase-time validation recorded in the Phase 02 report:

- Core timing guard - PASS
- CLI contract - PASS
- IDF example contract - PASS
- Version check - PASS
- Native tests - PASS, 52/52 tests
- Arduino PlatformIO builds - recorded as FAIL/environment-blocked at phase
  time due `C:\Users\HonzovoSpectre\.platformio\platforms.lock` permission
  error and aborted escalated reruns.
- Hardware validation - NOT RUN
- Local ESP-IDF build/target validation - NOT RUN

Later verification refresh in this consolidated-report pass:

- `python -m platformio run -e esp32s3dev -e esp32s2dev` - PASS for both
  environments.

Status: complete in current HEAD.

## Prompt 03 - Config Sequencing, Timing, And Sample Semantics

Objective from pasted prompt:

- Enforce `ctrl_hum` before `ctrl_meas`.
- Apply `config` changes in sleep mode and restore normal mode when needed.
- Preserve dirty-state diagnostics and original transport error on partial
  multi-register config failures.
- Define sample cache invalidation or config-generation semantics.
- Verify measurement timing and `nowMs` requirements.
- Add native tests and docs for config sequencing, timing, burst read, forced
  and normal-mode freshness.

Execution status:

- Not executed as Phase 03.
- No `docs/BME280_PHASE_03_CONFIG_TIMING_SAMPLE_SEMANTICS_REPORT.md` exists.
- Some Phase 03-relevant work was partially addressed by Phase 02 because
  Phase 02 added sample cache invalidation after successful typed config
  changes, but the full Phase 03 scope remains pending.

Required future artifact:

- `docs/BME280_PHASE_03_CONFIG_TIMING_SAMPLE_SEMANTICS_REPORT.md`

Required future commit message:

- `hardening: enforce BME280 config timing semantics`

Status: pending.

## Prompt 04 - Reset/NVM Handling And Fault Diagnostics

Objective from pasted prompt:

- Tighten `probe()`, `begin()`, `recover()`, `softReset()`, NVM polling, chip
  ID validation, calibration reload, dirty-state clearing, and health/offline
  diagnostics.
- Preserve useful transport errors, especially timeout, bus, data NACK, and
  generic I2C errors.
- Add native fault-injection tests for reset write failure, `im_update`
  timeout, NVM polling transport errors, reset calibration reload, recover
  dirty-state behavior, and offline latch behavior.

Execution status:

- Not executed as Phase 04.
- No `docs/BME280_PHASE_04_RESET_NVM_FAULT_DIAGNOSTICS_REPORT.md` exists.
- Phase 01 identified reset/NVM and transport-error precision as remaining
  work; Phase 04 is still the intended owner for that work.

Required future artifact:

- `docs/BME280_PHASE_04_RESET_NVM_FAULT_DIAGNOSTICS_REPORT.md`

Required future commit message:

- `hardening: improve BME280 reset and fault diagnostics`

Status: pending.

## Prompt 05 - Examples, ESP-IDF CI, Packaging, And Documentation

Objective from pasted prompt:

- Improve examples, CLI honesty, ESP-IDF support, CI guards, packaging,
  README/Doxygen/docs, and validation documentation.
- Verify package metadata, generated version synchronization, PlatformIO
  package packing, and local/CI build coverage.

Execution status:

- Not executed as Phase 05.
- No `docs/BME280_PHASE_05_EXAMPLES_IDF_CI_DOCS_REPORT.md` exists.
- A packaging command was run during this consolidated-report pass for current
  state only, not as Phase 05 implementation.

Current packaging refresh:

- `python -m platformio pkg pack` - PASS
- Generated artifact: `BME280-1.5.0.tar.gz`
- Artifact was removed and not committed.

Required future artifact:

- `docs/BME280_PHASE_05_EXAMPLES_IDF_CI_DOCS_REPORT.md`

Required future commit message:

- `hardening: improve BME280 examples and CI docs`

Status: pending.

## Prompt 06 - Hardware Validation, Final Integration Review, And Final Report

Objective from pasted prompt:

- Run final integration review.
- Run full local checks.
- Run hardware validation only if actual BME280 hardware is connected and
  commands are executed.
- Create `docs/BME280_INDUSTRY_STANDARD_FINAL_REPORT.md`.
- Provide merge readiness and release-claim guidance.

Execution status:

- Not executed as Phase 06.
- No real BME280 hardware commands were run.
- No `docs/BME280_INDUSTRY_STANDARD_FINAL_REPORT.md` exists.
- This file is a consolidated prompt-status report, not the Phase 06 final
  industry-standard report.

Required future artifact:

- `docs/BME280_INDUSTRY_STANDARD_FINAL_REPORT.md`

Required future commit message:

- `hardening: finalize BME280 industry-standard report`

Status: pending.

## Consolidated Verification Refresh

These checks were run on 2026-05-31 against current HEAD
`1c0d409807d43ad2790306d3bff6d20112e5b084`.

| Command | Result | Exact observed summary |
| --- | --- | --- |
| `python tools/check_core_timing_guard.py` | PASS | `Core timing guard PASSED` |
| `python tools/check_cli_contract.py` | PASS | `CLI contract PASSED` |
| `python tools/check_idf_example_contract.py` | PASS | `IDF example contract PASSED` |
| `python scripts/generate_version.py check` | PASS | `include\BME280\Version.h` up to date |
| `python -m platformio test -e native` | PASS | `52 test cases: 52 succeeded` |
| `python -m platformio run -e esp32s3dev -e esp32s2dev` | PASS | `esp32s3dev SUCCESS`; `esp32s2dev SUCCESS` |
| `python -m platformio pkg pack` | PASS | Wrote `BME280-1.5.0.tar.gz`; artifact removed after check |
| `where.exe idf.py` | FAIL/UNAVAILABLE | `INFO: Could not find files for the given pattern(s).` |

Not run:

- Local pure ESP-IDF builds with `idf.py`.
- Hardware validation.
- CI inspection, trigger, or remote workflow verification.

## Current Public API Changes From Recent Work

Recent public API changes are from Phase 02:

- `Measurement` now includes:
  - `temperatureValid`
  - `pressureValid`
  - `humidityValid`
- `RawSample` now includes:
  - `temperatureValid`
  - `pressureValid`
  - `humidityValid`
- `CompensatedSample` now includes:
  - `temperatureValid`
  - `pressureValid`
  - `humidityValid`
- `CommandTable.h` now exposes:
  - `RAW_PRESSURE_SKIPPED`
  - `RAW_TEMPERATURE_SKIPPED`
  - `RAW_HUMIDITY_SKIPPED`
- Cached samples are invalidated after successful typed configuration changes.

Migration note:

- Numeric fields remain zero for skipped/invalid channels for compatibility.
  Callers should check the corresponding validity flag before using a channel.
- The validity fields were appended to existing structs, but this is still a
  public layout change and should be treated as a release-note item.

## Hardware Validation Status

Hardware validation status: NOT RUN.

No BME280 device, wiring setup, board address, pull-up configuration, supply
configuration, serial log, CLI command transcript, soak run, fault-path run, or
environmental reference comparison was recorded during these prompts.

Safe statement:

- The current branch has host/native, Arduino PlatformIO, guard-script, and
  package-pack validation as listed above.

Unsafe statements to avoid:

- Do not say the driver is field-proven.
- Do not say both I2C addresses are hardware-validated.
- Do not say ESP-IDF local builds pass.
- Do not say reset/fault paths are hardware-validated.
- Do not say long soak or environmental humidity behavior has been validated.

## Remaining Work By Prompt

Phase 03 pending:

- Full config sequencing tests for `ctrl_hum`, `ctrl_meas`, and `config`.
- Normal-mode safe config writes and restoration tests.
- Full timing/deadline and sample freshness semantics.

Phase 04 pending:

- Full reset/NVM transport-error matrix.
- `softReset()` and recover dirty-state precision.
- More precise probe/begin/recover diagnostics.

Phase 05 pending:

- Examples/CLI docs review and potential expansion.
- ESP-IDF CI/version alignment.
- README/Doxygen hardware and production integration guidance.
- Package/version inclusion review.

Phase 06 pending:

- Final integration review after Phases 03-05.
- Local ESP-IDF build if `idf.py` is available.
- Hardware validation if actual BME280 hardware is connected.
- Final industry-standard report and merge/release verdict.

## Readiness Verdict

Current readiness after prompts 00-02:

- Ready to proceed to Phase 03.
- Not ready to claim completion of the full 00-06 hardening workflow.
- Not ready for final industry-standard release claims until Phases 03-06 are
  actually executed and reported.

Recommended next action:

- Start Phase 03 from clean, synced HEAD
  `1c0d409807d43ad2790306d3bff6d20112e5b084`.
