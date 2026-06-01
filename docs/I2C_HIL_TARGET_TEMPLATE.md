# BME280 I2C HIL Target Template

Copy this template into the HIL evidence package for each physical target.

## Target

- Operator:
- Date/time and timezone:
- Branch:
- Git commit:
- Worktree state / dirty flag:
- Framework:
- Build target:
- Serial port:
- Baud rate:
- Runner command:
- Runner arguments:
- Command groups executed:
- Opt-in flags used:
- Command file path / SHA256, if used:
- Firmware `version` output:
- Library version:
- HIL log directory:

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
- Timestamped summary:
- `summary.json`:
- `results.csv`:
- `command_plan.json`:
- `environment.txt`:
- `operator_checklist.md`:
- `hardware_matrix_fragment.md`:
- `failure_analysis.md`:
- `manifest.json`:
- Artifact manifest path:
- Manifest SHA256 or signed bundle path:
- Exact command transcript path:
- Completed hardware matrix fragment or notes:
- Logic analyzer capture, if used:
- Photo/video evidence, if used:

## Parsed BME280 Evidence

- `chipid` / `reg 0xD0` value:
- Post-force `reg 0xF4` value:
- Post-force `ctrl_meas[1:0]`:
- Post-force `status` measuring / im_update:
- Post-reset `status` measuring / im_update:
- Pre-recover dirty state:
- Post-recover dirty state:
- `cfg` evidence after recover:
- Raw sample validity flags:
- Compensated sample validity flags:
- Burst data capture `0xF7..0xFE`:
- `selftest` pass/fail/skip:
- `stress` Errors count:
- Final consecutive failures:
- Final total failures:

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

- Runner final verdict: NOT RUN / INCOMPLETE / PASS / FAIL / OPERATOR_REVIEW_REQUIRED
- Hardware run status: NOT RUN / PASS / FAIL / OPERATOR_REVIEW_REQUIRED
- Blocking issues:
- Remaining untested rows:
- Operator notes:
- Operator sign-off:
