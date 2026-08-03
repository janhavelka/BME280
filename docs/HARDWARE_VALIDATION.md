# BME280 Hardware Validation

Last updated: 2026-08-03

This is the single maintained hardware-in-the-loop (HIL) procedure, evidence
schema, and result ledger for the library. It replaces the former HIL runbook,
target template, validation matrix, and per-board summary.

Host tests, builds, and serial parsing do not prove sensor accuracy, electrical
margin, wiring, pull-ups, humidity handling, shared-bus behavior, protected
fault recovery, or long-term stability. Record `NOT RUN` or `unknown` when the
corresponding physical evidence is absent.

## Current Validation Status

A physical Arduino/PlatformIO HIL campaign was run on an ESP32-S2 at address
`0x76` using the clean library source commit
`dc5df8e6735bd21f52dea3e5ae7dcf54ec2b498d`. Its raw serial transcript is
retained at
`../hil_logs/i2c_20260803_144215/serial_transcript.txt`. Deterministic serial
checks passed, but the run remains operator-review evidence rather than formal
hardware qualification because electrical metadata, calibrated environmental
reference data, manual fault injection, and operator sign-off are incomplete.

| Target or condition | Current status | Evidence boundary |
| --- | --- | --- |
| ESP32-S2, Arduino/PlatformIO, address `0x76` | FUNCTIONAL SERIAL PASS / OPERATOR REVIEW REQUIRED | Clean source commit `dc5df8e`; 61-minute COM10 campaign; raw transcript retained. No calibrated accuracy or complete electrical qualification claim. |
| ESP32-S2, native ESP-IDF | NOT RUN | No physical runtime artifact is committed. |
| ESP32-S3, Arduino/PlatformIO | NOT RUN | Build evidence is not hardware evidence. |
| ESP32-S3, native ESP-IDF | NOT RUN | No physical runtime artifact is committed. |
| Address `0x77` | NOT RUN | The safe unstrapped-address negative check is not coverage of a device with SDO tied to VDDIO. |
| Calibrated environmental accuracy | NOT RUN | No controlled reference-instrument artifact is committed. |
| Protected fault injection | NOT RUN | No protected address-NACK, timeout, bus/data fault, or recovery artifact is committed. |
| Production shared-bus contention | NOT RUN | No application-owned locking/scheduling coexistence artifact is committed. |
| Logic-analyzer burst coherency | NOT RUN | No capture proving one `0xF7..0xFE` transaction is committed. |

### Retained ESP32-S2 Run: `i2c_20260803_144215`

- Target: ESP32-S2, Arduino/PlatformIO, COM10 at 115200 baud, BME280 address
  `0x76`, reported SDA GPIO8, SCL GPIO9, and 400 kHz.
- Provenance: branch `main`, clean source commit
  `dc5df8e6735bd21f52dea3e5ae7dcf54ec2b498d`, library version `2.0.0`, and
  firmware build time `2026-08-03 14:39:57`.
- Raw artifact: 5,048,514 bytes, 105,839 lines, SHA-256
  `7b858255bfc5053e18c224f3d14621e18b3db21ed1cfef16d92f727f4ba55415`;
  repository attributes disable text normalization for retained transcripts.
- Runtime: 3,677.9 seconds for the full campaign, including a 3,545.5-second
  duration soak; 6,979 classified result rows contained no `FAIL` or `TIMEOUT`.
- Stress coverage: 58,860 measurement samples with zero reported errors and
  81,340 mixed safe operations with zero reported failures.
- Reset coverage: 117 reset/NVM-busy observations all recovered; the final
  cleanup reported `READY`, online, zero consecutive failures, clean hardware
  configuration, `measuring=0`, and `im_update=0`.
- Follow-up negative check: selecting unstrapped address `0x77` returned
  `I2C_SHORT_TRANSFER`; the Arduino transport could not prove a definite
  address NACK. Restoring `0x76` succeeded and a subsequent measurement was
  valid. This separately observed follow-up has no retained transcript, so it
  is an adapter diagnostic only, not qualification evidence or address-`0x77`
  device validation.
- Accuracy boundary: observed values were plausible, but no calibrated
  reference instrument was recorded. Accuracy, electrical margin, protected
  fault injection, native ESP-IDF behavior, production shared-bus contention,
  and logic-analyzer burst shape remain `NOT RUN`.

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
7. Retain the raw `serial_transcript.txt` and record the provenance, result,
   and evidence boundary in this ledger. The repository ignore policy permits
   selected raw transcripts while ignoring derived runner output.
8. For a formal qualification claim, preserve the complete selected artifact
   directory and its `manifest.json` in durable tracked storage or an immutable
   release asset. Derived summaries may be cleaned after ledger verification
   only when the run is explicitly retained as functional evidence rather than
   a complete qualification package.

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
python tools/run_i2c_hil.py --dry-run --include-job-api --out .pio/hil_dry_runs
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

A formal qualifying run must include:

| Category | Required fields |
| --- | --- |
| Provenance | Operator; date/time and timezone; branch; Git commit; worktree state / dirty flag; runner command and arguments; command groups and opt-ins; firmware `version` output; library version; exact pioarduino platform, Arduino core, and ESP-IDF versions where applicable; runner final verdict; sign-off. |
| Target | Framework; build target; MCU board model and target; serial port/baud; BME280 module and visible marking; fixture description. |
| Electrical | VDD; VDDIO; supply/current limit; SDA/SCL pins and speed; pull-up values and location; address; verified SDO and CSB state; reset/interrupt wiring or `N/A`. |
| Environmental | Reference instrument models/calibration state; reference and BME280 readings; tolerances/uncertainty; timestamp; stability/altitude notes; per-channel decision. Use `NOT RUN` when no reference was used. |
| Artifacts | `serial_transcript.txt`, `summary.md`, timestamped summary, `summary.json`, `results.csv`, `command_plan.json`, `environment.txt`, `operator_checklist.md`, `hardware_matrix_fragment.md`, `failure_analysis.md`, and `manifest.json`; optional photo or logic-analyzer paths. |
| Manual review | Wiring/pull-up/strap verification; forced sleep return; normal-mode/soak scope; protected fault scope; environmental plausibility; remaining untested rows; blocking issues. |

A functional run that is not claimed as formal qualification may retain the
raw transcript plus this ledger's provenance, numeric results, and explicit
limitations. Generated summaries are review aids, not substitutes for the raw
serial record. Do not upgrade such a run to a qualification claim after its
complete package has been discarded.

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
