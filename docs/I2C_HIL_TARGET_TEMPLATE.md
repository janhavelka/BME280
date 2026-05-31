# BME280 I2C HIL Target Template

Copy this template into the HIL evidence package for each physical target.

## Target

- Operator:
- Date/time and timezone:
- Branch:
- Commit:
- Worktree state:
- Framework:
- Build target:
- Serial port:
- Baud rate:
- Runner command:
- Firmware `version` output:

## Hardware

- MCU board model:
- MCU target: ESP32-S2 / ESP32-S3 / other:
- BME280 module or sensor board model:
- Chip marking:
- Fixture description:
- VDD:
- VDDIO:
- SDA pin:
- SCL pin:
- I2C speed:
- I2C pull-ups:
- Pull-up location: on-module / external / other:
- BME280 address:
- SDO state:
- CSB state:
- Reset wiring or `N/A`:
- Interrupt wiring or `N/A`:
- Power supply/current limit:

## Reference Instruments

- Temperature reference model:
- Temperature reference calibration status:
- Temperature reference reading:
- BME280 temperature reading:
- Temperature tolerance / uncertainty:
- Temperature pass/fail:
- Humidity reference model:
- Humidity reference calibration status:
- Humidity reference reading:
- BME280 humidity reading:
- Humidity tolerance / uncertainty:
- Humidity pass/fail:
- Pressure reference model:
- Pressure reference calibration status:
- Pressure reference reading:
- BME280 pressure reading:
- Pressure tolerance / uncertainty:
- Pressure pass/fail:
- Site altitude or pressure adjustment notes:
- Reading timestamp:
- Stability notes:

## Artifacts

- `serial_transcript.txt`:
- `summary.md`:
- `summary.json`:
- `operator_checklist.md`:
- Logic analyzer capture, if used:
- Photo/video evidence, if used:

## Manual Checks

- Board/module wiring checked:
- SDO and CSB checked:
- Pull-ups checked:
- Bus speed checked:
- Address selection checked:
- `chipid` or `reg 0xD0` recorded `0x60`:
- Environmental readings plausible against references:
- Unsafe/fault tests run: yes / no
- If fault tests were run, describe fixture protections:
- Recovery after any fault recorded:

## Result

- Hardware run status: NOT RUN / PASS / FAIL / OPERATOR REVIEW REQUIRED
- Blocking issues:
- Operator notes:
- Operator sign-off:
