# BME280 Hardware Validation Matrix

Last updated: 2026-05-31

This matrix is intentionally conservative. `NOT RUN` means no physical BME280
hardware command was executed and recorded for that item in this repository.
Serial output alone is not enough for environmental accuracy or fault-readiness
claims.

Before changing any row to `PASS` or `FAIL`, record these fields in the HIL
report or attached transcript:

- Board model and MCU target: ESP32-S2 and/or ESP32-S3.
- BME280 module or board model.
- Supply voltage for VDD and VDDIO.
- I2C pull-up values and whether they are on-module or external.
- BME280 address, SDO state, and CSB state.
- SDA/SCL pins and bus speed.
- Firmware commit hash and build environment.
- Full CLI or serial command transcript.
- Environmental reference used, if any.
- Pass/fail result and notes for each test row below.
- HIL runner artifacts from `tools/run_i2c_hil.py`, if used:
  `serial_transcript.txt`, `summary.md`, `summary.json`, and
  `operator_checklist.md`.

## Setup Record

Use `unknown` rather than guessing. Leave untested rows as `NOT RUN`.

| Field | Result |
|------|--------|
| Operator | NOT RUN |
| Date/time and timezone | NOT RUN |
| Branch | NOT RUN |
| Commit hash | NOT RUN |
| Worktree state | NOT RUN |
| Framework | NOT RUN |
| Build target | NOT RUN |
| Serial port and baud | NOT RUN |
| HIL log directory | NOT RUN |
| MCU board model | NOT RUN |
| BME280 module or board model | NOT RUN |
| Chip marking, if visible | NOT RUN |
| VDD / VDDIO | NOT RUN |
| SDA/SCL pins and bus speed | NOT RUN |
| Pull-up values and location | NOT RUN |
| BME280 address | NOT RUN |
| SDO state | NOT RUN |
| CSB state | NOT RUN |
| Environmental reference instruments | NOT RUN |
| Logic analyzer capture path, if used | NOT RUN |
| Photo/video evidence path, if used | NOT RUN |
| HIL runner summary path, if used | NOT RUN |
| Operator notes / sign-off | NOT RUN |

## Validation Rows

| Area | Target / condition | Status | Evidence |
|------|--------------------|--------|----------|
| Address 0x76 | SDO tied to GND, CSB tied to VDDIO | NOT RUN | Pending physical board test |
| Address 0x77 | SDO tied to VDDIO, CSB tied to VDDIO | NOT RUN | Pending physical board test |
| Chip ID | Register `0xD0` reads `0x60` | NOT RUN | Pending `id`/`chipid` CLI capture |
| Soft reset | Write `0xB6` to `0xE0`, wait for `im_update` clear | NOT RUN | Pending `reset` CLI capture |
| Forced mode | Forced measurement produces a sample; sleep-return evidence recorded separately with `mode`, `status`, or `reg 0xF4` | NOT RUN | Pending `force`/`read` and post-force mode/status capture |
| Normal mode | `tick()` polling captures fresh samples across cycles | NOT RUN | Pending repeated `normal on` / `read` / `normal off` captures |
| Burst read coherency | Single `0xF7..0xFE` transaction | NOT RUN | Pending logic-analyzer or adapter trace |
| Calibration | `0x88..0xA1` and `0xE1..0xE7` parsed plausibly | NOT RUN | Pending `calib` CLI capture |
| Compensation | Temperature/pressure/humidity plausible for environment | NOT RUN | Pending controlled environment reading |
| Humidity handling | Non-condensing operation after assembly handling | NOT RUN | Pending production hardware procedure |
| Fault mapping | Address NACK, timeout, bus/data errors, recovery | NOT RUN | Pending protected fault-injection bench |
| Shared bus | External lock, timeout policy, scheduled `tick()` | NOT RUN | Pending application integration test |
| Long soak | Repeated normal-mode reads and/or bounded forced stress without hangs | NOT RUN | Pending soak transcript and reference notes |

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
read
normal on
read
normal off
reset
recover
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
stress 500
```

Normal-mode soak is manual evidence, not completed by `--include-soak` alone.
Record repeated `read` commands in normal mode with timestamps and references:

```text
normal on
read
read
read
normal off
cfg
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
