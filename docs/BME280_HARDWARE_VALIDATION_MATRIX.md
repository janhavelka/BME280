# BME280 Hardware Validation Matrix

Last updated: 2026-05-31

This matrix is intentionally conservative. `NOT RUN` means no physical BME280
hardware command was executed and recorded for that item in this repository.

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

| Area | Target / condition | Status | Evidence |
|------|--------------------|--------|----------|
| Address 0x76 | SDO tied to GND, CSB tied to VDDIO | NOT RUN | Pending physical board test |
| Address 0x77 | SDO tied to VDDIO, CSB tied to VDDIO | NOT RUN | Pending physical board test |
| Chip ID | Register `0xD0` reads `0x60` | NOT RUN | Pending `id`/`chipid` CLI capture |
| Soft reset | Write `0xB6` to `0xE0`, wait for `im_update` clear | NOT RUN | Pending `reset` CLI capture |
| Forced mode | One measurement returns then device sleeps | NOT RUN | Pending `force`/`read` CLI capture |
| Normal mode | `tick()` polling captures fresh samples across cycles | NOT RUN | Pending `normal on` stress capture |
| Burst read coherency | Single `0xF7..0xFE` transaction | NOT RUN | Pending logic-analyzer or adapter trace |
| Calibration | `0x88..0xA1` and `0xE1..0xE7` parsed plausibly | NOT RUN | Pending `calib` CLI capture |
| Compensation | Temperature/pressure/humidity plausible for environment | NOT RUN | Pending controlled environment reading |
| Humidity handling | Non-condensing operation after assembly handling | NOT RUN | Pending production hardware procedure |
| Fault mapping | Address NACK, timeout, bus/data errors, recovery | NOT RUN | Pending hardware/fault-injection bench |
| Shared bus | External lock, timeout policy, scheduled `tick()` | NOT RUN | Pending application integration test |

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

Fault-path HIL evidence should include wrong-address `addr 0x76`/`addr 0x77`
probe attempts, safe sensor unplug/replug address-NACK capture, safe temporary
SDA/SCL fault capture if the bench setup supports it, reset during measurement,
manual `recover` after unplug/replug, longer normal-mode soak, and plausibility
comparison against local temperature, humidity, and pressure references.

Software-only checks are recorded in phase reports and CI logs. They do not
prove electrical wiring, timing margins, environmental accuracy, or sensor
assembly handling.
