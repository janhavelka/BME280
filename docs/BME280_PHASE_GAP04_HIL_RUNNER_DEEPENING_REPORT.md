# BME280 Gap 04 HIL Runner Deepening Report

Historical prompt report. This file records a completed gap-closure pass and is
not current user documentation or hardware validation evidence. Use
`README.md`, `docs/README.md`, `docs/I2C_HIL_RUNBOOK.md`, and
`docs/BME280_HARDWARE_VALIDATION_MATRIX.md` for maintained guidance.

Date: 2026-06-01

Scope: Prompt 04 only. No physical HIL was run, no sibling repository was
modified, no driver behavior was changed, and no hardware validation is claimed.

## Starting Point

- Branch: `hardening/bme280-industry-gap-closure`
- Starting HEAD: `e4602a6f532b6a782b87200309b99c9f75ac6cc8`
- Initial dirty state: only pre-existing untracked
  `docs/BME280_GAP_CLOSURE_BASELINE_REPORT.md`

## Reference Repositories Inspected

- `../SSD1315`
  - Branch: `fix/ssd1315-industrial-gap-closure`
  - HEAD: `ccb5f87417d1212d6b3fa73e6ff5451854562a71`
  - Relevant patterns: grouped HIL plans, timestamped artifact directories,
    CSV/JSON/metadata outputs, command plans, matrix fragments, failure
    analysis, operator checklist separation, strict claim boundaries.
- `../SHT3x-main`
  - Relevant patterns: safe default sequence, opt-in groups, parsed evidence,
    environment records, final verdict boundaries, dry-run artifact generation,
    and contract checking against the documented default sequence.

Both reference repos were inspected read-only.

## Gaps Fixed

- Added command groups to `tools/run_i2c_hil.py`: `provenance`,
  `bus-reachability`, `identity-calibration`, `forced-mode`, `normal-mode`,
  `reset-recover`, `stress-health`, `soak-forced`, `soak-normal`, `raw-write`,
  `manual-fault`, and `custom-command-file`.
- Strengthened BME280 evidence parsing:
  - `reg 0xF4` parses `ctrl_meas[1:0]` and fails if forced mode did not return
    to sleep.
  - `status` parses `measuring` and `im_update`.
  - `selftest` parses `fail=0`.
  - `stress` parses `Errors=0`.
  - `drv` parses final consecutive failures.
- Deepened the default safe sequence with:
  - a second normal-mode `read`,
  - post-reset `status` evidence for `im_update`,
  - post-`recover` `cfg` and `status` evidence for resync/dirty-state review.
- Added artifact discipline:
  - `results.csv`,
  - `command_plan.json`,
  - `environment.txt`,
  - timestamped Markdown summary,
  - `hardware_matrix_fragment.md`,
  - `failure_analysis.md`,
  - `manifest.json` with SHA256 hashes.
- Added metadata flags for operator, board, MCU target, framework, build target,
  module, rails, pull-ups, pins, bus speed, SDO/CSB, references, and notes.
- Added opt-in normal-mode soak generation with existing CLI commands:
  `--include-normal-soak`, `--normal-soak-count`, and
  `--normal-soak-interval-s`.
- Split forced stress soak from normal-mode soak with `--soak-count`.
- Required explicit `--confirm-raw-write BME280_RAW_WRITE` before non-dry-run
  raw-write execution.
- Safety-scanned `--commands` files so raw `wreg` requires destructive opt-in
  and large `stress N` requires soak opt-in.
- Updated the HIL runbook, target template, hardware validation matrix, and
  hardening summary for the new runner evidence flow.
- Extended `tools/check_hil_contract.py` to enforce the new grouped sequence,
  artifacts, raw-write confirmation string, normal soak path, and docs parity.

## HIL Commands Added Or Changed

No CLI command was added and no firmware command behavior was changed.

Runner default sequence changes:

- Added a second `read` while in normal mode.
- Added `status` immediately after `reset`.
- Added `cfg` and `status` immediately after `recover`.

Optional runner-generated sequences now use existing CLI commands:

- Forced stress soak: `stress N` through `--include-soak --soak-count N`.
- Normal-mode soak: `normal on`, repeated `read`, `normal off`, `cfg`,
  `status` through `--include-normal-soak`.
- Destructive raw write remains gated: `wreg 0xF4 0x00`, then `recover`, `cfg`,
  `status`.

## Dry-Run Evidence

Command run:

```bash
python tools/run_i2c_hil.py --dry-run --operator dry-run --board unknown --mcu-target unknown --module unknown --vdd unknown --vddio unknown --pullups unknown --sdo-state unknown --csb-state unknown
```

Result:

- PASS as a runner dry-run.
- Final verdict: `INCOMPLETE`.
- No serial port opened and no physical HIL validation performed.
- Artifact directory inspected:
  `hil_logs/i2c_20260601_204758`.
- Generated artifacts inspected: `serial_transcript.txt`, `summary.md`,
  `i2c_20260601_204758_summary.md`, `summary.json`, `results.csv`,
  `command_plan.json`, `environment.txt`, `operator_checklist.md`,
  `hardware_matrix_fragment.md`, `failure_analysis.md`, and `manifest.json`.

## Checks Run

| Check | Result |
|------|--------|
| `python tools/check_hil_contract.py` | PASS: `HIL contract PASSED` |
| `python tools/run_i2c_hil.py --help` | PASS: help printed new metadata, soak, normal-soak, and raw-write confirmation flags |
| `python tools/check_cli_contract.py` | PASS: `CLI contract PASSED` |
| `python tools/check_idf_example_contract.py` | PASS: `IDF example contract PASSED` |
| `python tools/check_core_timing_guard.py` | PASS: `Core timing guard PASSED` |
| `python scripts/generate_version.py check` | PASS: `include/BME280/Version.h` up to date |
| `python -m platformio test -e native` | PASS: 97 test cases, 97 succeeded in 00:00:03.008 |
| `python -m platformio run -e esp32s3dev` | First attempt FAIL: `*** [.pio\build\esp32s3dev\FrameworkArduino\esp32-hal-matrix.c.o] Error 1` with no compiler diagnostic in normal output |
| `python -m platformio run -e esp32s3dev -v` | PASS: `esp32s3dev SUCCESS` in 00:00:19.710 |
| `python -m platformio run -e esp32s3dev` | PASS on exact rerun: `esp32s3dev SUCCESS` in 00:00:30.655 |
| `python -m platformio run -e esp32s2dev` | PASS: `esp32s2dev SUCCESS` in 00:00:21.557 |
| `git diff --check` | PASS with line-ending warnings that Git will replace LF with CRLF on touched files |

## Remaining HIL-Only Risks

- No real BME280 HIL run was performed.
- No hardware matrix row should move out of `NOT RUN` until a physical target
  artifact package and operator sign-off exist.
- Environmental plausibility still requires calibrated/reference instruments.
- Fault injection remains manual and requires a protected bench fixture.
- Long-duration stability and production soak remain unproven until real
  normal-mode and/or forced-stress artifacts are captured.
- Pure local `idf.py` example builds were not part of this prompt's required
  checks and were not run here.
