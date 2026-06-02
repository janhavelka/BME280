# BME280 Phase Gap 05 Production Docs Report

Date: 2026-06-01

Branch: `hardening/bme280-industry-gap-closure`

Starting HEAD: `ccf37940856a5aa8d595296cf799d941091d8adf`

## Scope

This pass updated production-facing documentation, public API comments, Doxygen
inputs, and stale documentation wording. It did not run physical HIL, did not
tag a release, and did not change core driver behavior.

## Documentation Updated

- `README.md`
  - Clarified framework-neutral, injected, non-owning I2C ownership guidance.
  - Added a maintained pointer to `docs/PRODUCTION_SHARED_BUS_GUIDE.md`.
  - Documented health counter saturation and current-session reset behavior.
  - Clarified diagnostic raw write resync paths: `recover()`, `begin()`, or a
    successful `softReset()`.
  - Repeated that shipped examples are diagnostic bring-up CLIs, not production
    shared-bus firmware templates.
- `include/BME280/Config.h`
  - Clarified synchronous task-context transport callback expectations.
  - Documented that the application owns shared-bus serialization, timeout
    policy, recovery, and platform resources.
- `include/BME280/BME280.h`
  - Documented health counter saturation.
  - Clarified cached calibration provenance after `begin()`, `recover()`, or
    `softReset()`.
  - Clarified raw diagnostic write dirty-state resync paths.
- `docs/README.md`
  - Added the production shared-bus guide to the maintained documentation map.
  - Classified `BME280_PHASE_*.md` files as historical prompt records, not
    current user documentation or hardware validation evidence.
- `docs/IDF_PORT.md`
  - Pointed production shared-bus users to the new guide.
  - Repeated that shipped Arduino and ESP-IDF examples are diagnostic CLIs.
- `CHANGELOG.md`
  - Added unreleased documentation entries for this pass.
  - Corrected prior raw diagnostic write resync wording to include successful
    `softReset()`.
  - Corrected prior wording about prompt reports so it matches the tracked docs
    tree and Doxygen exclusion policy.

## Shared-Bus Guidance

Added `docs/PRODUCTION_SHARED_BUS_GUIDE.md`.

The guide covers:

- application-owned I2C bus handles, pins, pullups, bus speed, and device reset
  policy;
- external bus locking and driver-instance serialization;
- bounded synchronous transport callbacks with finite timeout policy at the
  transport boundary;
- scheduled `tick()` and measurement-task structure;
- no driver calls from ISRs;
- no bus reset or bus reconfiguration inside the core driver;
- shared bus use with other devices;
- application-owned recovery strategy;
- HIL evidence expectations before claiming production hardware readiness.

No buildable production firmware example was added in this pass. The repository
already ships diagnostic Arduino and ESP-IDF CLIs; adding a full production
shared-bus firmware example would increase CI and maintenance scope without
adding hardware evidence.

## Doxygen Changes

Updated `Doxyfile`:

- Added `docs/PRODUCTION_SHARED_BUS_GUIDE.md` to published Doxygen inputs.
- Kept `AGENTS.md` out of published Doxygen inputs.
- Excluded local/generated artifacts and historical prompt reports, including
  `.pio`, `.git`, `docs/doxygen`, extracted docs, `hil_logs`,
  `BME280-*.tar.gz`, `docs/BME280_PHASE_*`, and
  `docs/BME280_GAP_CLOSURE_BASELINE_REPORT.md`.
- Left strict warnings enabled where already practical:
  `WARN_IF_UNDOCUMENTED=YES`, `WARN_NO_PARAMDOC=YES`.
- Left `WARN_AS_ERROR=NO`; promoting warnings to errors should wait for a
  deliberate Doxygen warning-clean baseline in a later gate.
- Confirmed `PROJECT_NUMBER = 1.7.0`, matching release metadata from Prompt 03.

Doxygen result:

- Command: `doxygen Doxyfile`
- Tool version: `1.15.0`
- Result: pass
- Warnings printed: none
- Generated output: local `docs/doxygen/`, not committed

## Stale Docs And Links

- Historical prompt reports were marked as prompt-scoped records and excluded
  from published Doxygen inputs.
- The maintained documentation map now distinguishes current user docs from
  local/generated artifacts and historical prompt reports.
- The changelog `[1.2.2]` reference link already exists; no link repair was
  required for that release reference.
- README/docs continue to state that physical BME280 hardware validation is not
  claimed until the hardware matrix or HIL artifacts record real board, wiring,
  commands, and results.

## Checks

- `python tools/check_core_timing_guard.py`
  - Pass: `Core timing guard PASSED`
- `python tools/check_cli_contract.py`
  - Pass: `CLI contract PASSED`
- `python tools/check_idf_example_contract.py`
  - Pass: `IDF example contract PASSED`
- `python tools/check_hil_contract.py`
  - Pass: `HIL contract PASSED`
- `python scripts/generate_version.py check`
  - Pass: `Up to date: include/BME280/Version.h`
- `python -m platformio test -e native`
  - Pass: 97 test cases, 97 succeeded in `00:00:03.688`
- `python -m platformio run -e esp32s3dev`
  - Pass: `esp32s3dev SUCCESS`, duration `00:00:26.853`
- `python -m platformio run -e esp32s2dev`
  - Pass: `esp32s2dev SUCCESS`, duration `00:00:28.439`
- `python -m platformio pkg pack`
  - Pass: created `BME280-1.7.0.tar.gz`
- `python tools/check_package_contents.py`
  - Pass: `Package contents PASSED (BME280-1.7.0.tar.gz)`
- `git diff --check`
  - Pass: no whitespace errors. Git printed CRLF working-copy conversion
    warnings on Windows.

## Remaining Docs And Release Risks

- Physical HIL remains a separate gate; this pass did not claim hardware
  validation.
- `WARN_AS_ERROR` remains disabled until a deliberate Doxygen warning-clean gate
  is scheduled.
- A stale untracked `docs/BME280_GAP_CLOSURE_BASELINE_REPORT.md` was present
  before this prompt. It is intentionally not staged in this Prompt 05 docs
  commit because it belongs to Prompt 00 baseline cleanup, not production
  integration guidance.
- Production shared-bus behavior is documented as architecture and pseudocode.
  A buildable production firmware template may be added later if CI and
  maintenance ownership are defined.
