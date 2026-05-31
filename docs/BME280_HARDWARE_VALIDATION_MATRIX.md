# BME280 Hardware Validation Matrix

Last updated: 2026-05-31

This matrix is intentionally conservative. `NOT RUN` means no physical BME280
hardware command was executed and recorded for that item in this repository.

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

Software-only checks are recorded in phase reports and CI logs. They do not
prove electrical wiring, timing margins, environmental accuracy, or sensor
assembly handling.
