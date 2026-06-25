# BME280 Hardware Validation Matrix

Last updated: 2026-06-25

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
| ESP32-S2 Arduino serial HIL | COM28, PlatformIO Arduino `esp32s2dev`, address `0x76`, chip ID `0x60` | OPERATOR_REVIEW_REQUIRED | Compact ledger: `docs/reports/esp32s2-com28-hil-summary.md`. Local `hil_logs/` artifacts were development-local unless packaged with manifest and hashes. Latest classifier HIL had 0 UNKNOWN, 0 FAIL, 0 TIMEOUT. |
| ESP32-S2 ESP-IDF serial HIL | Native ESP-IDF CLI runtime behavior | NOT RUN | No `idf.py` hardware runtime artifact recorded |
| ESP32-S3 Arduino serial HIL | PlatformIO Arduino on ESP32-S3 hardware | NOT RUN | Build-only evidence is not hardware validation |
| ESP32-S3 ESP-IDF serial HIL | Native ESP-IDF on ESP32-S3 hardware | NOT RUN | No hardware runtime artifact recorded |
| Address 0x77 | SDO tied to VDDIO, CSB tied to VDDIO | NOT RUN | Pending physical board test |
| Calibrated accuracy | Reference temperature, pressure, and humidity instruments | NOT RUN | No controlled reference artifact recorded |
| Fault injection | Protected absence/timeout/bus/data fault fixture | NOT RUN | No protected fault-injection bench artifact recorded |
| Shared-bus contention | Another active device, lock timeout, scheduler evidence | NOT RUN | Pending application integration test |
| Safe default serial run | Default runner groups complete without raw writes, long soak, or physical fault actions | OPERATOR_REVIEW_REQUIRED | COM28 summary records default/extended serial HIL with no FAIL or TIMEOUT; raw artifacts were not committed |
| Address 0x76 | SDO tied to GND, CSB tied to VDDIO | OPERATOR_REVIEW_REQUIRED | COM28 HIL ran at `0x76`; SDO/CSB were inferred from working I2C mode, not independently photographed or probed |
| Chip ID | Register `0xD0` reads `0x60` | PASS | COM28 summary records chip ID `0x60` |
| Soft reset | Write `0xB6` to `0xE0`, wait for `im_update` clear | PASS_WITH_RESET_BUSY_RECOVERED | Reset BUSY/NVM observations are accepted only with immediate recovery evidence; classifier HIL recorded recovered reset rows and 0 UNKNOWN |
| Recover/resync | Dirty-state evidence is recorded before/after recover | PASS | COM28 classifier HIL required `READY`, `dirty=false`, and `Consecutive failures: 0` after reset/recover |
| Forced mode | Forced measurement produces a sample | OPERATOR_REVIEW_REQUIRED | COM28 serial HIL exercised forced samples; environmental plausibility remains operator-reviewed |
| Forced sleep return | `ctrl_meas[1:0]` returns to sleep after forced conversion | OPERATOR_REVIEW_REQUIRED | COM28 serial HIL captured post-force state; raw artifact package not committed |
| Normal mode | `tick()` polling captures fresh samples across cycles | OPERATOR_REVIEW_REQUIRED | COM28 serial HIL exercised normal-mode CLI paths; raw artifact package not committed |
| Normal-mode soak | Opt-in repeated normal-mode reads with timestamps and references | OPERATOR_REVIEW_REQUIRED | COM28 20-hour comprehensive soak recorded 0 FAIL and 0 TIMEOUT; calibrated references were not recorded |
| Burst read coherency | Single `0xF7..0xFE` transaction | NOT RUN | Pending logic-analyzer or adapter trace |
| Calibration | `0x88..0xA1` and `0xE1..0xE7` parsed plausibly | OPERATOR_REVIEW_REQUIRED | COM28 serial HIL exercised calibration CLI output; raw artifact package not committed |
| Compensation | Temperature/pressure/humidity plausible for environment | OPERATOR_REVIEW_REQUIRED | COM28 serial HIL exercised compensated output; no calibrated reference artifact recorded |
| Humidity handling | Non-condensing operation after assembly handling | NOT RUN | Pending production hardware procedure |
| Fault mapping | Address NACK, timeout, bus/data errors, recovery | NOT RUN | Pending protected fault-injection bench |
| Shared bus | External lock, timeout policy, scheduled `tick()` | NOT RUN | Pending application integration test |
| Forced stress | Bounded `stress N` completes with `Errors: 0` and reviewed sample ranges | OPERATOR_REVIEW_REQUIRED | COM28 20-hour comprehensive soak recorded no FAIL or TIMEOUT; sample ranges remain operator-reviewed |
| Long soak | Repeated normal-mode reads and/or bounded forced stress without hangs | OPERATOR_REVIEW_REQUIRED | COM28 20-hour comprehensive soak: 83313 PASS, 51701 OPERATOR_CHECK_REQUIRED, 1206 reset/NVM UNKNOWN, 0 FAIL, 0 TIMEOUT |
| Staged job API | `--include-job-api` job status/init/apply/force/recover/poll evidence | PASS | COM28 reset-classifier HIL included staged job coverage: 0 UNKNOWN, 0 FAIL, 0 TIMEOUT |

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

Production electrical gates are not satisfied until the artifact records VDD in
`1.71..3.6 V`, VDDIO in `1.2..3.6 V`, SDO tied to GND for `0x76` or VDDIO for
`0x77`, CSB tied high before POR, no SDA/SCL/SDO/CSB drive above VDDIO when
VDDIO is off, bus capacitance `<= 400 pF`, verified rise time at the selected
bus speed, and stable burst reads of `0xF7..0xFE`.

Shared-bus production gates are not satisfied until evidence shows at least one
other active bus client, an application-owned lock timeout mapping to
`I2C_TIMEOUT`, `tick()` called inside the sample budget, no hidden BME280
success while another client faults the bus, and no ISR directly calling public
BME280 APIs.

## Evidence Policy

- Use `NOT RUN` for anything not executed.
- Use `unknown` for setup facts the operator could not verify.
- Do not copy CI or host-test results into hardware rows.
- Do not mark environmental rows `PASS` without reference instruments and
  tolerances.
- Do not mark fault rows `PASS` unless the protected bench action was actually
  performed and the recovery result was recorded.
