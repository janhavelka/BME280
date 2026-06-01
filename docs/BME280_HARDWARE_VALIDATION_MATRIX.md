# BME280 Hardware Validation Matrix

Last updated: 2026-06-01

This matrix is intentionally conservative. `NOT RUN` means no physical BME280
hardware command was executed and recorded for that item in this repository.
Serial output alone is not enough for environmental accuracy or fault-readiness
claims.

Before changing any row to `PASS` or `FAIL`, record these fields in the HIL
report or attached transcript:

- Firmware `version` output, library version, git commit, and dirty flag.
- HIL runner command and exact arguments.
- Command groups executed, opt-in flags, and any command file path/hash.
- Board model and MCU target: ESP32-S2 and/or ESP32-S3.
- BME280 module or board model.
- Fixture description and power supply/current limit.
- Supply voltage for VDD and VDDIO.
- I2C pull-up values and whether they are on-module or external.
- BME280 address, SDO state, and CSB state.
- SDA/SCL pins and bus speed.
- Reset and interrupt wiring, or `N/A`.
- Serial port and baud rate.
- Full CLI or serial command transcript.
- Artifact manifest path and hashes.
- Environmental references, readings, tolerances/uncertainty, timestamp, and
  stability notes, if plausibility is evaluated.
- Pass/fail result and notes for each test row below.
- Operator notes and sign-off.
- HIL runner artifacts from `tools/run_i2c_hil.py`, if used:
  `serial_transcript.txt`, `summary.md`, timestamped summary,
  `summary.json`, `results.csv`, `command_plan.json`, `environment.txt`,
  `operator_checklist.md`, `hardware_matrix_fragment.md`,
  `failure_analysis.md`, and `manifest.json`.

## Setup Record

Use `unknown` rather than guessing. Leave untested rows as `NOT RUN`.

| Field | Result |
|------|--------|
| Operator | NOT RUN |
| Date/time and timezone | NOT RUN |
| Branch | NOT RUN |
| Git commit hash | NOT RUN |
| Worktree state / dirty flag | NOT RUN |
| Framework | NOT RUN |
| Build target | NOT RUN |
| MCU target | NOT RUN |
| Serial port and baud | NOT RUN |
| HIL runner command | NOT RUN |
| HIL runner arguments | NOT RUN |
| Command groups executed | NOT RUN |
| Opt-in groups used | NOT RUN |
| Command file path / SHA256, if used | NOT RUN |
| HIL log directory | NOT RUN |
| Firmware `version` output | NOT RUN |
| Library version | NOT RUN |
| MCU board model | NOT RUN |
| BME280 module or board model | NOT RUN |
| Chip marking, if visible | NOT RUN |
| Fixture description | NOT RUN |
| VDD / VDDIO | NOT RUN |
| Power supply/current limit | NOT RUN |
| SDA/SCL pins and bus speed | NOT RUN |
| Pull-up values and location | NOT RUN |
| BME280 address | NOT RUN |
| SDO state | NOT RUN |
| CSB state | NOT RUN |
| Reset wiring or `N/A` | NOT RUN |
| Interrupt wiring or `N/A` | NOT RUN |
| Environmental reference instruments | NOT RUN |
| Temperature reference reading / BME280 reading / tolerance | NOT RUN |
| Humidity reference reading / BME280 reading / tolerance | NOT RUN |
| Pressure reference reading / BME280 reading / tolerance | NOT RUN |
| Environmental reading timestamp and stability notes | NOT RUN |
| Logic analyzer capture path, if used | NOT RUN |
| Photo/video evidence path, if used | NOT RUN |
| HIL runner `summary.md` path, if used | NOT RUN |
| HIL runner `summary.json` path, if used | NOT RUN |
| HIL runner `results.csv` path, if used | NOT RUN |
| HIL runner `command_plan.json` path, if used | NOT RUN |
| HIL runner `environment.txt` path, if used | NOT RUN |
| HIL runner `operator_checklist.md` path, if used | NOT RUN |
| HIL runner `hardware_matrix_fragment.md` path, if used | NOT RUN |
| HIL runner `failure_analysis.md` path, if used | NOT RUN |
| HIL runner `manifest.json` path, if used | NOT RUN |
| Artifact manifest path / hash | NOT RUN |
| Exact command transcript path | NOT RUN |
| Runner final verdict | NOT RUN |
| Operator notes / sign-off | NOT RUN |

## Validation Rows

| Area | Target / condition | Status | Evidence |
|------|--------------------|--------|----------|
| Safe default serial run | Default runner groups complete without raw writes, long soak, or physical fault actions | NOT RUN | Pending `tools/run_i2c_hil.py` artifacts and manifest |
| Address 0x76 | SDO tied to GND, CSB tied to VDDIO | NOT RUN | Pending physical board test |
| Address 0x77 | SDO tied to VDDIO, CSB tied to VDDIO | NOT RUN | Pending physical board test |
| Chip ID | Register `0xD0` reads `0x60` | NOT RUN | Pending `chipid` and `reg 0xD0` CLI capture |
| Soft reset | Write `0xB6` to `0xE0`, wait for `im_update` clear | NOT RUN | Pending `reset` and post-reset `status` capture |
| Recover/resync | Dirty-state evidence is recorded before/after recover | NOT RUN | Pending `status`, `recover`, `cfg`, and post-recover `status` capture |
| Forced mode | Forced measurement produces a sample | NOT RUN | Pending `force` and follow-up `read` capture |
| Forced sleep return | `ctrl_meas[1:0]` returns to sleep after forced conversion | NOT RUN | Pending post-force `reg 0xF4` and post-force `status` capture |
| Normal mode | `tick()` polling captures fresh samples across cycles | NOT RUN | Pending repeated `normal on` / `read` / `normal off` captures |
| Normal-mode soak | Opt-in repeated normal-mode reads with timestamps and references | NOT RUN | Pending `--include-normal-soak` artifact package |
| Burst read coherency | Single `0xF7..0xFE` transaction | NOT RUN | Pending logic-analyzer or adapter trace |
| Calibration | `0x88..0xA1` and `0xE1..0xE7` parsed plausibly | NOT RUN | Pending `calib` CLI capture |
| Compensation | Temperature/pressure/humidity plausible for environment | NOT RUN | Pending controlled environment reading |
| Humidity handling | Non-condensing operation after assembly handling | NOT RUN | Pending production hardware procedure |
| Fault mapping | Address NACK, timeout, bus/data errors, recovery | NOT RUN | Pending protected fault-injection bench |
| Shared bus | External lock, timeout policy, scheduled `tick()` | NOT RUN | Pending application integration test |
| Forced stress | Bounded `stress N` completes with `Errors: 0` and reviewed sample ranges | NOT RUN | Pending default `stress 10` or opt-in `--include-soak` transcript |
| Long soak | Repeated normal-mode reads and/or bounded forced stress without hangs | NOT RUN | Pending soak transcript, manifest, and reference notes |

Minimum bring-up command evidence:

```text
version
help
scan
addr 0x76 or addr 0x77
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

Use stress 10 as an automated forced-measurement stress substitute, not a true
counted-read command. There is no `read 10` or `read 20` CLI command.

Default runner example:

```bash
python tools/run_i2c_hil.py --port <PORT> --baud 115200 --address 0x76 --out hil_logs
```

Optional longer soak evidence:

```text
normal on
periodic repeated read commands
normal off
cfg
status
stress 500
```

Normal-mode soak is separate from forced `stress N`. Use
`--include-normal-soak` for a runner-generated normal-mode soak plan, or record
manual repeated `read` commands in normal mode with timestamps and references:

```text
normal on
read
read
read
normal off
cfg
status
```

Fault-path HIL evidence should include wrong-address `addr 0x76`/`addr 0x77`
probe attempts, safe sensor unplug/replug address-NACK capture, safe temporary
SDA/SCL fault capture if the bench setup supports it, reset during measurement,
manual `recover` after unplug/replug, longer normal-mode soak, and plausibility
comparison against local temperature, humidity, and pressure references.

Software-only checks are recorded in CI logs, validation logs, and maintained
docs. They do not prove electrical wiring, timing margins, environmental
accuracy, or sensor assembly handling.

## Evidence Policy

- Use `NOT RUN` for anything not executed.
- Use `unknown` for setup facts the operator could not verify.
- Do not copy CI or host-test results into hardware rows.
- Do not mark environmental rows `PASS` without reference instruments and
  tolerances.
- Do not mark fault rows `PASS` unless the protected bench action was actually
  performed and the recovery result was recorded.
