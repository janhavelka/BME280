# BME280 Hardware Validation

Last updated: 2026-07-22

This is the single maintained hardware-in-the-loop (HIL) procedure, evidence
schema, and result ledger for the library. It replaces the former HIL runbook,
target template, validation matrix, and per-board summary.

Host tests, builds, and serial parsing do not prove sensor accuracy, electrical
margin, wiring, pull-ups, humidity handling, shared-bus behavior, protected
fault recovery, or long-term stability. Record `NOT RUN` or `unknown` when the
corresponding physical evidence is absent.

## Current Validation Status

No physical HIL run tied to a clean `v2.0.0` commit and complete evidence
schema is committed. A local one-hour ESP32-S2 run completed on 2026-07-22, but
its firmware and host worktree were dirty and several fixture fields were not
verified. It is useful regression evidence, not a release-qualification claim.

| Target or condition | Current status | Evidence boundary |
| --- | --- | --- |
| ESP32-S2, Arduino/PlatformIO, address `0x76` | LOCAL ONE-HOUR SERIAL HIL / NOT RELEASE-QUALIFYING | The 2026-07-22 COM5 run passed every deterministic row for a dirty `2.0.0` build. Exact clean-build provenance, module/rail/pull-up/strap evidence, references, protected faults, and shared-bus behavior remain open. |
| ESP32-S2, native ESP-IDF | NOT RUN | No physical runtime artifact is committed. |
| ESP32-S3, Arduino/PlatformIO | NOT RUN | Build evidence is not hardware evidence. |
| ESP32-S3, native ESP-IDF | NOT RUN | No physical runtime artifact is committed. |
| Address `0x77` | NOT RUN | Requires SDO tied to VDDIO and a recorded identity check. |
| Calibrated environmental accuracy | NOT RUN | No controlled reference-instrument artifact is committed. |
| Protected fault injection | NOT RUN | No protected address-NACK, timeout, bus/data fault, or recovery artifact is committed. |
| Production shared-bus contention | NOT RUN | No application-owned locking/scheduling coexistence artifact is committed. |
| Logic-analyzer burst coherency | NOT RUN | No capture proving one `0xF7..0xFE` transaction is committed. |

### Local ESP32-S2 `2.0.0` Run

Run `i2c_20260722_163553` used the Arduino `esp32s2dev` diagnostic CLI on
`COM5`, address `0x76`, GPIO8/GPIO9, and the configured 400 kHz bus. The
firmware reported library `2.0.0`, base commit `f6fc7bf`, and a dirty build.
The dirty firmware included post-tag `Unreleased` fixes and therefore is not
the published `v2.0.0` artifact despite reporting the same base version. The
host stayed on the same dirty commit for the run. The artifact directory is
local under `hil_logs/` and is intentionally ignored by Git.

| Recorded item | Result |
| --- | --- |
| Duration soak | 3,603.663 seconds, 1,201 cycles; stopped before starting the next group whose reserved window would exceed the deadline |
| Result rows | 4,282 PASS; 2,614 OPERATOR_CHECK_REQUIRED; 61 PASS_WITH_RESET_BUSY_RECOVERED; 1 provenance REVIEW_REQUIRED; 0 FAIL/TIMEOUT/UNKNOWN |
| Workload totals | 61,110 forced samples; 84,140 mixed operations; 203 normal-mode reads; exhaustive legal oversampling/filter/standby matrix; invalid-input, benchmark, staged-job, and opted-in raw-write/recovery groups |
| Reset evidence | All 61 reset NVM-BUSY observations were followed immediately by successful recover/status evidence proving READY, `dirty=false`, `measuring=0`, and `im_update=0` |
| Observed batch ranges | 27.38 to 28.50 degC; 99,143 to 99,189 Pa; 27.67 to 31.72% RH; these are plausibility observations only, not accuracy results |
| Integrity | The manifest hashes for all ten listed artifacts were rechecked after the run and matched |

The audit found that the runner version used for the hour did not append its
intended final safe-state rows after an ordinary duration deadline stop. That
host-tool defect did not alter the completed workload or its row
classifications. The runner was fixed and run `i2c_20260722_174458` then passed
the exact bounded cleanup sequence: `normal off`, `recover`, `cfg`, `status`,
and `drv`. Its final evidence proved clean configuration, `measuring=0`,
`im_update=0`, and zero consecutive failures.

This local evidence does not qualify `v2.0.0`: the source was dirty, the exact
board/module, rails, pull-ups, SDO/CSB straps, and reference instruments were
not independently recorded, protected physical fault injection was not
performed, and the test was a standalone ESP32-S2 diagnostic fixture rather
than the ESP32-S3 TunnelMonitor shared-bus runtime.

### Historical ESP32-S2 Runs

These results are retained only as pre-v2 development history. They were run on
one ESP32-S2 Arduino fixture on `COM28`, address `0x76`, chip ID `0x60`,
SDA/SCL GPIO8/GPIO9, and 400 kHz. SDO and CSB state were inferred rather than
independently recorded. Because the raw packages and release identity are
missing, none of these rows qualifies `v2.0.0` or any untested target.

| Date | Run | Recorded classifier result |
| --- | --- | --- |
| 2026-06-22 | Functional serial HIL | 61 PASS, 22 OPERATOR_CHECK_REQUIRED, 1 reset/NVM UNKNOWN, 0 FAIL, 0 TIMEOUT |
| 2026-06-22 to 2026-06-23 | 8-hour soak | 11,497 PASS, 7,297 OPERATOR_CHECK_REQUIRED, 35 reset/NVM UNKNOWN, 0 FAIL, 0 TIMEOUT |
| 2026-06-23 to 2026-06-24 | 20-hour soak | 83,313 PASS, 51,701 OPERATOR_CHECK_REQUIRED, 1,206 reset/NVM UNKNOWN, 0 FAIL, 0 TIMEOUT |
| 2026-06-24 | 180-second reset classifier | 354 PASS, 127 OPERATOR_CHECK_REQUIRED, 54 PASS_WITH_RESET_BUSY_RECOVERED, 0 UNKNOWN/FAIL/TIMEOUT |
| 2026-06-24 | 60-second reset classifier | 132 PASS, 48 OPERATOR_CHECK_REQUIRED, 19 PASS_WITH_RESET_BUSY_RECOVERED, 0 UNKNOWN/FAIL/TIMEOUT |

The recovered reset classification is valid only when an immediate `recover`
and status capture prove `im_update=0`, `measuring=0`, driver `READY`, and clean
hardware configuration. It is not an accuracy or fault-injection result.

## HIL Procedure

1. Select the exact clean commit to test and record it before flashing. The
   runner compares the firmware-reported library version, commit, and clean
   state with host provenance captured at run start. A mismatch fails the
   version row; dirty source, unavailable provenance, or a repository change
   during the run requires review and cannot qualify exact build provenance.
2. Record the setup fields below. Do not infer wiring or electrical facts from
   a successful I2C exchange.
3. Build and flash the diagnostic CLI that matches the connected target.
4. Run the parser self-test and a dry run.
5. Run the serial plan with setup metadata and `--include-job-api` for the
   staged v2 API.
6. Review `summary.md`, `results.csv`, `operator_checklist.md`, the transcript,
   and manual reference/fault evidence together.
7. Preserve the selected artifact directory and its `manifest.json`. Add a new
   ledger row here only when the evidence package is available.

Install `pyserial` only for a real serial run:

```bash
python -m pip install pyserial
```

Build and upload the Arduino examples:

```bash
python -m platformio run -e esp32s3dev
python -m platformio run -e esp32s2dev
python -m platformio run -e esp32s3dev -t upload --upload-port <PORT>
python -m platformio run -e esp32s2dev -t upload --upload-port <PORT>
```

Build or flash the native ESP-IDF example when `idf.py` is installed:

```bash
idf.py -C examples/idf/basic set-target esp32s3
idf.py -C examples/idf/basic build
idf.py -C examples/idf/basic set-target esp32s2
idf.py -C examples/idf/basic build
idf.py -C examples/idf/basic -p <PORT> flash monitor
```

Verify the runner before hardware access:

```bash
python tools/run_i2c_hil.py --parser-self-test
python tools/run_i2c_hil.py --dry-run --include-job-api
```

Example evidence run (PowerShell line continuation shown):

```powershell
python tools/run_i2c_hil.py `
  --port <PORT> --baud 115200 --address 0x76 --out hil_logs `
  --include-job-api --fail-on-review `
  --operator "<name>" --board "<board>" --mcu-target esp32s2 `
  --framework arduino --build-target esp32s2dev --module "<module>" `
  --vdd "3.3 V" --vddio "3.3 V" --pullups "4.7 kohm" `
  --pullup-location external --sda-pin GPIO8 --scl-pin GPIO9 `
  --bus-speed "400 kHz" --sdo-state GND --csb-state VDDIO `
  --environment-ref "<instrument/readings artifact>" `
  --operator-notes "<fixture and scope>"
```

Use `--address 0x77` only when SDO is tied to VDDIO. `scan` proves only that an
address acknowledged; `chipid` or `reg 0xD0` must record `0x60` to identify a
BME280.

The default plan deliberately includes environmental plausibility rows that
require human review. Consequently, its final verdict is normally
`OPERATOR_REVIEW_REQUIRED` even when every deterministic serial check passes.
`--fail-on-review` is the honest automation gate for that plan. `--require-pass`
is useful only for a custom plan containing no operator-review rows; it is not a
complete production-release gate.

## Default Command Sequence

This guarded sequence must match `tools/run_i2c_hil.py` for address `0x76`.
With `--address 0x77`, only the `addr` command changes.

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

Repeated `read` entries are intentional; the CLI has no `read 10` or `read 20`
command. A valid deterministic serial run records at least:

- chip ID `0x60`;
- post-`force` `reg 0xF4` mode bits `[1:0] == 00`;
- post-`force` `status` with `measuring=0`;
- reset/recover status with `im_update=0`, `READY`, and clean config;
- `selftest` with `fail=0`, subject to environmental review;
- `stress` with `Errors: 0`, subject to environmental review;
- final consecutive failure count of zero;
- when `--include-job-api` is used, bounded callback counts, distinct resync
  and reset jobs, and exactly-once cancellation-result retrieval.

## Optional and Unsafe Coverage

- `--include-normal-soak --normal-soak-count N
  --normal-soak-interval-s S` adds normal-mode repeated reads.
- `--include-soak --soak-count N` adds forced-measurement stress; it is not
  normal-mode soak.
- `--soak-duration-s N` is a planning budget for a bounded duration mix. The
  runner reserves complete timeout windows and groups `normal on` with
  `normal off`, and `reset` with `recover`, before starting those transitions.
  It may stop short of `N`, and small host/serial scheduling overhead can make
  wall time differ; use the recorded exact elapsed time in any claim. After
  every requested duration stop, the runner makes a bounded best-effort final
  cleanup in the fixed order `normal off`, `recover`, `cfg`, `status`, `drv`.
  It makes the same attempt after a fixed-plan FAIL/TIMEOUT. Cleanup rows remain
  visible evidence and never replace the root verdict; inspect them explicitly.
  A state-changing duration group also performs its declared recovery before a
  hard stop when that command may have taken effect.
- `--include-config-matrix` exercises every legal public oversampling, filter,
  and standby enum encoding with register readback, then restores the example's
  forced-mode defaults. `--include-invalid-inputs` and `--include-benchmarks`
  add the other safe diagnostic groups.
- `--include-destructive --confirm-raw-write BME280_RAW_WRITE` enables a raw
  control-register write followed by resync evidence. Never include it in an
  unattended default run.
- `--include-fault-tests` marks manual fault checks as requested. The runner
  does not unplug hardware or short bus lines.
- A command file supplied with `--commands <path>` remains operator-owned and
  must end in a known safe state. The runner prepends its deterministic
  `version` evidence row when the file does not contain one.

Fault injection is manual and may damage hardware. Perform it only on a
protected, current-limited fixture. Record the action, expected mapping, actual
status, recovery, and final health/config state.

## Evidence Record

Every qualifying run must include:

| Category | Required fields |
| --- | --- |
| Provenance | Operator; date/time and timezone; branch; Git commit; worktree state / dirty flag; runner command and arguments; command groups and opt-ins; firmware `version` output; library version; runner final verdict; sign-off. |
| Target | Framework; build target; MCU board model and target; serial port/baud; BME280 module and visible marking; fixture description. |
| Electrical | VDD; VDDIO; supply/current limit; SDA/SCL pins and speed; pull-up values and location; address; verified SDO and CSB state; reset/interrupt wiring or `N/A`. |
| Environmental | Reference instrument models/calibration state; reference and BME280 readings; tolerances/uncertainty; timestamp; stability/altitude notes; per-channel decision. Use `NOT RUN` when no reference was used. |
| Artifacts | `serial_transcript.txt`, `summary.md`, timestamped summary, `summary.json`, `results.csv`, `command_plan.json`, `environment.txt`, `operator_checklist.md`, `hardware_matrix_fragment.md`, `failure_analysis.md`, and `manifest.json`; optional photo or logic-analyzer paths. |
| Manual review | Wiring/pull-up/strap verification; forced sleep return; normal-mode/soak scope; protected fault scope; environmental plausibility; remaining untested rows; blocking issues. |

The runner accepts the setup values through `--operator`, `--board`,
`--mcu-target`, `--framework`, `--build-target`, `--module`, `--vdd`,
`--vddio`, `--pullups`, `--pullup-location`, `--sda-pin`, `--scl-pin`,
`--bus-speed`, `--sdo-state`, `--csb-state`, `--environment-ref`, and
`--operator-notes`. Blank values become `unknown`; metadata completeness is an
operator qualification decision, not part of the serial classifier.

## Result Vocabulary

- `PASS`: required deterministic serial tokens were captured for that row.
- `PASS_WITH_RESET_BUSY_RECOVERED`: reset reported NVM busy and immediate
  follow-up evidence proved complete recovery.
- `OPERATOR_CHECK_REQUIRED`: serial output matched, but a human must evaluate
  environmental or bench evidence.
- `REVIEW_REQUIRED`: output was captured but was not deterministically
  classified.
- `SERIAL_OK_OR_REVIEW`: serial output exists but expected tokens are missing.
- `UNKNOWN`: bounded but incomplete hardware state was observed.
- `FAIL` / `TIMEOUT`: a failure token was observed or a command exceeded its
  deadline.
- `SKIPPED_DRY_RUN` / `SKIPPED_UNSAFE`: no physical command was executed.

A runner verdict is an evidence input, not a hardware qualification. A
qualifying result requires the serial artifacts, setup record, relevant manual
checks, and operator sign-off to agree.
