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
scan
probe
chipid
cfg
calib
status
read
repeat read 10 times or record stress 10 as the available counted-read equivalent
force
read
normal on
repeat read 20 times; record that true `read 20` is not currently a CLI command
reset
probe
cfg
read
selftest
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
