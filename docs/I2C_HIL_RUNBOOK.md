# BME280 I2C HIL Runbook

Date: 2026-06-01

This runbook describes the serial hardware-in-the-loop procedure for the BME280
bring-up CLIs. It is a procedure and evidence format, not a completed hardware
result. No physical HIL validation is claimed by this document.

## Scope

The runner drives the existing Arduino/PlatformIO and native ESP-IDF diagnostic
CLIs over a serial port. It does not flash firmware, change library APIs, or
verify sensor accuracy by itself. It captures command output, classifies serial
evidence, and leaves environmental plausibility and unsafe fault work to an
operator.

`scan` shows only an I2C ACK from an address. BME280 identity is established
only when `chipid` or `reg 0xD0` records `0x60`. Environmental accuracy requires
reference instruments and recorded limits.

## Operator Flow

1. Confirm the repository is on the intended commit and the worktree is clean.
2. Fill `docs/I2C_HIL_TARGET_TEMPLATE.md` for the board and sensor module.
3. Build both supported Arduino targets and, when available, both ESP-IDF
   targets.
4. Flash only the firmware target that matches the connected board.
5. Dry-run the HIL runner.
6. Run the serial HIL plan and save the generated artifacts.
7. Add environmental reference readings, photos, logic-analyzer captures, and
   fault-test notes if they were actually collected.
8. Update `docs/BME280_HARDWARE_VALIDATION_MATRIX.md` only with observed
   results.

## Required Setup Record

Record these fields before marking a hardware run PASS or FAIL:

- Operator name.
- Date/time and timezone.
- Branch, git commit hash, and dirty/clean worktree state.
- Framework: Arduino/PlatformIO or ESP-IDF.
- Build target: `esp32s3dev`, `esp32s2dev`, `esp32s3`, or `esp32s2`.
- Serial port and baud rate.
- Firmware version as printed by `version`.
- Library version as printed by `version`.
- HIL runner command and exact arguments.
- Command groups executed and any opt-in groups requested.
- MCU board model.
- MCU target.
- BME280 module or sensor board model.
- Chip marking, if visible.
- Fixture description.
- Supply voltage for VDD and VDDIO.
- I2C pull-up values and whether they are on-module or external.
- BME280 address, SDO state, and CSB state.
- SDA/SCL pins and bus speed.
- Reset wiring or `N/A`.
- Interrupt wiring as `N/A` unless the bench fixture adds one.
- Temperature, humidity, and pressure reference instruments and calibration
  status, if plausibility is evaluated.
- Reference readings, BME280 readings, tolerance or uncertainty, reading
  timestamp, and stability notes, if plausibility is evaluated.
- Exact serial command transcript path.
- Manifest path and artifact hashes.
- Runner final verdict: `INCOMPLETE`, `FAIL`, `OPERATOR_REVIEW_REQUIRED`, or
  `PASS`.
- Logic analyzer capture path, if used.
- Photo or video evidence path, if used.
- Operator notes and sign-off.

## Build And Flash

PlatformIO Arduino builds:

```bash
python -m platformio run -e esp32s3dev
python -m platformio run -e esp32s2dev
```

PlatformIO upload examples:

```bash
python -m platformio run -e esp32s3dev -t upload --upload-port <PORT>
python -m platformio run -e esp32s2dev -t upload --upload-port <PORT>
```

PlatformIO monitor example:

```bash
python -m platformio device monitor --port <PORT> --baud 115200
```

ESP-IDF build examples, when `idf.py` is installed:

```bash
idf.py -C examples/idf/basic set-target esp32s3
idf.py -C examples/idf/basic build
idf.py -C examples/idf/basic set-target esp32s2
idf.py -C examples/idf/basic build
```

ESP-IDF flash/monitor example:

```bash
idf.py -C examples/idf/basic -p <PORT> flash monitor
```

## Runner Commands

Dry-run the plan without opening serial:

```bash
python tools/run_i2c_hil.py --dry-run
```

Run the default serial HIL plan:

```bash
python tools/run_i2c_hil.py --port <PORT> --baud 115200 --address 0x76 --out hil_logs
```

Run with the longer soak command included:

```bash
python tools/run_i2c_hil.py --port <PORT> --baud 115200 --address 0x76 --out hil_logs --include-soak --soak-count 500
```

Run an opt-in normal-mode repeated-read soak using only existing CLI commands:

```bash
python tools/run_i2c_hil.py --port <PORT> --baud 115200 --address 0x76 --out hil_logs --include-normal-soak --normal-soak-count 10 --normal-soak-interval-s 1.0
```

Run parser checks without hardware:

```bash
python tools/run_i2c_hil.py --parser-self-test
```

Run additional safe coverage groups:

```bash
python tools/run_i2c_hil.py --port <PORT> --baud 115200 --address 0x76 --include-config-matrix --include-invalid-inputs --include-benchmarks
```

Run a bounded duration soak after the fixed plan:

```bash
python tools/run_i2c_hil.py --port <PORT> --baud 115200 --address 0x76 --soak-duration-s 28800 --soak-cycle-stress-count 50 --soak-cycle-mix-count 70
```

Serial timing can be adjusted without editing the runner using
`--timeout-s`, `--boot-settle-s`, `--idle-timeout-s`,
`--idle-after-match-s`, `--command-pacing-s`, `--reconnect-attempts`, and
`--reconnect-delay-s`. Use `--verbose` to echo transcript chunks to the console
while preserving the artifact files.

Use `--address 0x77` when SDO is tied to VDDIO. The runner writes artifacts to a
new directory under `hil_logs/`:

- `serial_transcript.txt` - raw serial transcript with command boundaries.
- `summary.md` - auditor-readable result summary.
- `i2c_<timestamp>_summary.md` - timestamped copy of the summary.
- `summary.json` - machine-readable equivalent.
- `results.csv` - flat command result table.
- `command_plan.json` - executable and skipped/manual command plan.
- `environment.txt` - setup metadata copied from runner arguments.
- `operator_checklist.md` - manual checks and skipped unsafe/fault work.
- `hardware_matrix_fragment.md` - generated fragment for operator review.
- `failure_analysis.md` - failure/review rows extracted from results.
- `manifest.json` - artifact paths and SHA256 hashes.

These files are evidence inputs. A serial `PASS` is not a sensor-accuracy pass
unless the setup record and reference readings also support that claim.

The runner accepts metadata flags such as `--operator`, `--board`,
`--mcu-target`, `--module`, `--vdd`, `--vddio`, `--pullups`,
`--pullup-location`, `--sda-pin`, `--scl-pin`, `--bus-speed`, `--sdo-state`,
`--csb-state`, and `--environment-ref`. Unknown fields should be left blank or
recorded as `unknown`, not guessed.

Install `pyserial` only for non-dry-run serial execution:

```bash
python -m pip install pyserial
```

## Default Command Sequence

The guarded default sequence below must match `tools/run_i2c_hil.py` for address
`0x76`. With `--address 0x77`, only the `addr` command changes.

<!-- HIL_DEFAULT_SEQUENCE_START -->
```text
version
help
scan
addr 0x76
begin
probe
chipid
cfg
calib
calib raw
status
timing
reg 0xD0
read
raw
comp
data
force
reg 0xF4
status
read
normal on
read
read
normal off
reset
status
recover
cfg
status
selftest
stress 10
drv
state
```
<!-- HIL_DEFAULT_SEQUENCE_END -->

`read` entries are intentional repeated commands. There is no `read 10` or
`read 20` CLI contract.

Default command groups:

| Group | Evidence purpose |
|------|------------------|
| `provenance` | Firmware/library version, git commit, dirty flag, and CLI surface |
| `bus-reachability` | Address ACK, selected address, initialization, and raw probe |
| `identity-calibration` | BME280 chip ID `0x60`, config, calibration, timing, and initial dirty-state evidence |
| `forced-mode` | Baseline sample, raw/comp validity flags, burst data, forced conversion, post-force sleep return |
| `normal-mode` | Bounded normal-mode entry, repeated reads, and return to sleep |
| `reset-recover` | Soft reset, post-reset `im_update`/status evidence, recover/resync, and dirty-state evidence |
| `stress-health` | Safe selftest, short forced stress, and final driver health |
| `job-api` | Opt-in staged start/poll/cancel retrieval, init/apply/forced, non-reset resync, and explicit reset jobs with callback-budget validation |

The post-`force` `reg 0xF4` capture records `ctrl_meas`; for formal forced-mode
sleep-return evidence, verify mode bits `[1:0]` are `00`. The following
post-`force` `status` capture records `measuring=0` after the forced sample is
available.

Formal BME280 serial acceptance checks:

- `chipid` or `reg 0xD0` must show chip ID `0x60`.
- Post-`force` `reg 0xF4` must parse `ctrl_meas[1:0] == 00`.
- Post-`force` `status` should parse `measuring=0`.
- Post-reset `status` should parse `im_update=0`; otherwise reset/NVM evidence
  remains incomplete or failed.
- A synchronous `reset` that returns `Status: BUSY` with `NVM update in
  progress` is Bosch-compatible transient reset behavior. The runner may
  reclassify it as `PASS_WITH_RESET_BUSY_RECOVERED` only when the immediate
  follow-up `recover` and `status` evidence proves `READY`, `dirty=false`,
  `im_update=0`, and `measuring=0`.
- `selftest` must parse `fail=0`; skipped rows still require operator review.
- `stress` rows must parse `stress Errors=0`; sample plausibility still
  requires operator review.
- `drv` should parse `Consecutive failures: 0` at the end of the run.
- `job` rows added by `--include-job-api` must report the exact command
  boundary, job ID/kind/phase/state, canonical status code/detail, conversion
  state, phase-deadline state/value, callback count, and terminal flag.
- The staged cancellation sequence must show `job start init` using zero
  callbacks, `job poll 1` using exactly one callback, `job cancel deadline`
  using zero callbacks, one `job poll 0` returning the retained `TIMED_OUT` /
  `DEADLINE_EXPIRED` result, and the next `job poll 0` returning the empty
  `IDLE` result. This is the exactly-once retrieval check.
- Natural job terminals must be printed from the `JobPollResult` returned by
  the terminal poll. `Callbacks used:` and its compatibility `Instructions:`
  alias must agree and remain within the requested budget.
- `job resync 1` must report kind `RESYNC` without a soft-reset phase;
  `job reset 1` separately reports kind `SOFT_RESET`. Both must finish with
  `Consecutive failures: 0` on a passing bench run.

## Gated Work

The default run excludes raw writes, long soak, and physical fault injection.

- `--include-soak --soak-count N` adds `stress N` as forced-measurement stress
  evidence. It is not normal-mode soak evidence.
- `--include-normal-soak --normal-soak-count N --normal-soak-interval-s S`
  adds `normal on`, repeated `read` commands, `normal off`, `cfg`, and
  `status`. This remains operator-reviewed because serial output alone does not
  prove environmental accuracy.
- `--include-config-matrix` adds a bounded smoke matrix for safe oversampling,
  filter, and standby boundary values, restores the default example settings,
  and captures `cfg`, timing, raw, and compensated evidence.
- `--include-invalid-inputs` adds safe CLI validation checks for unknown
  commands, invalid address/mode/config values, and malformed register commands.
- `--include-benchmarks` or `--sample-rate-benchmark` adds forced-measurement
  and mixed-operation sampling-rate benchmarks.
- `--include-job-api` adds a bounded staged-job sequence: zero-I2C start,
  one-callback progress, deadline cancellation, zero-budget exactly-once
  terminal retrieval, then natural init/apply/forced terminals, explicit
  non-reset resync, explicit soft reset, and post-job raw/comp/cfg evidence.
  The group remains opt-in and makes no physical hardware-validation claim.
- `--soak-duration-s N` runs a duration-based command mix after the fixed plan.
  Each command remains individually timeout-bounded, and the loop stops early on
  `FAIL` or `TIMEOUT`. `--soak-reset-interval N` controls periodic soft
  reset/recover cycles; use `0` to disable them.
- `--require-pass` exits `0` only for final verdict `PASS`; review/unknown
  verdicts exit `3`.
- `--fail-on-review` keeps evidence collection behavior but exits `3` when the
  final verdict requires operator review or contains unknown serial evidence.

## Portable Evidence Package

Runner artifacts under `hil_logs/` are local development evidence unless they
are packaged with a manifest and hashes. For a release evidence bundle, copy the
selected run directories into a dated report folder and record SHA256 values for
each `manifest.json`, `summary.json`, `results.csv`, and
`serial_transcript.txt`:

```powershell
$artifactDir = "docs\reports\hil-validation-COM28-YYYYMMDD-artifacts"
New-Item -ItemType Directory -Force $artifactDir
Copy-Item -Recurse hil_logs\<run-id-1> $artifactDir\
Copy-Item -Recurse hil_logs\<run-id-2> $artifactDir\
Get-FileHash "$artifactDir\*\manifest.json" -Algorithm SHA256
Get-FileHash "$artifactDir\*\summary.json" -Algorithm SHA256
Get-FileHash "$artifactDir\*\results.csv" -Algorithm SHA256
Get-FileHash "$artifactDir\*\serial_transcript.txt" -Algorithm SHA256
```

Do not commit large transcripts or zip files by default. Commit only a compact
index with paths, hashes, runner commands, and final verdicts unless the release
process explicitly requires bundled raw logs.
- `--include-destructive --confirm-raw-write BME280_RAW_WRITE` adds a
  diagnostic `wreg 0xF4 0x00` raw write and follow-up `recover`, `cfg`, and
  `status` evidence. Raw writes can desynchronize cached driver settings and
  are unsafe for default automation.
- `--include-fault-tests` marks manual checklist items for wrong-address,
  unplug/replug, and safe SDA/SCL fault evidence as intentionally requested.
  The runner does not perform those actions, and the checklist always lists
  them so skipped fault work remains visible.

An operator-supplied command file is loaded with `--commands <path>`. Command
files are safety-scanned: raw `wreg` commands require the destructive opt-in and
confirmation, and `stress` counts above 100 require `--include-soak`. The
operator remains responsible for ending command files in a known safe state,
for example with `normal off`, `cfg`, and `status`.

## Result Rules

- `PASS` means required serial tokens were captured for that command.
- `OPERATOR_CHECK_REQUIRED` means serial output was captured but a human must
  review environmental plausibility or bench evidence.
- `REVIEW_REQUIRED` means output was captured, but the runner could not classify
  it as a deterministic pass or failure. The operator must inspect the
  transcript before using it as evidence.
- `SERIAL_OK_OR_REVIEW` means serial output exists but did not contain all
  expected tokens.
- `UNKNOWN` means a bounded but incomplete hardware state was observed, for
  example soft-reset returning `Status: BUSY` while NVM copy is still in
  progress. Review the following status/recover evidence before using it.
- `FAIL` or `TIMEOUT` means the transcript contains a precise failure token or a
  command exceeded its deadline.
- `SKIPPED_DRY_RUN` means no serial command was sent.
- `SKIPPED_UNSAFE` means a gated destructive command was excluded.

Hardware validation is complete only after the transcript, summary, manual
checklist, and setup record are reviewed together.

## Manual Normal-Mode And Environmental Evidence

The default runner sequence exercises and records a bounded bring-up path. It
does not by itself verify normal-mode soak behavior or environmental accuracy.

For normal-mode evidence, run and record repeated samples with stable timing:

```text
normal on
read
read
read
normal off
cfg
status
```

Record the timestamp, BME280 reading, reference instrument reading, tolerance or
uncertainty, stability notes, and pass/fail decision for temperature, pressure,
and humidity. The runner can generate that command shape with
`--include-normal-soak`; use `--include-soak` only for an additional
forced-measurement stress loop.

## Fault Evidence

Fault tests are manual because unsafe wiring actions can damage boards or
sensors. Run them only on a protected bench fixture.

Record at least:

- wrong-address probe result;
- safe unplug/replug result, if the fixture supports it;
- SDA/SCL fault or timeout result, if safely injectable;
- recovery command and final state after the fault;
- whether `hardwareConfigDirty()` or health counters changed.

Skipped fault tests are acceptable, but they must stay visible in the checklist
and hardware matrix.
