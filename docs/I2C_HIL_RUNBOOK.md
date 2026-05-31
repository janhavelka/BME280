# BME280 I2C HIL Runbook

Date: 2026-05-31

This runbook describes the serial hardware-in-the-loop procedure for the BME280
bring-up CLIs. It is a procedure and evidence format, not a completed hardware
result. No physical HIL validation was performed while adding this runbook.

## Scope

The runner drives the existing Arduino/PlatformIO and native ESP-IDF diagnostic
CLIs over a serial port. It does not flash firmware, change library APIs, or
prove sensor accuracy by itself. It captures command output, classifies serial
evidence, and leaves environmental plausibility and unsafe fault work to an
operator.

`scan` proves only an I2C ACK from an address. BME280 identity is established
only when `chipid` or `reg 0xD0` records `0x60`. Environmental accuracy requires
reference instruments and recorded limits.

## Required Setup Record

Record these fields before marking a hardware run PASS or FAIL:

- Operator name.
- Date/time and timezone.
- Branch and commit hash.
- Dirty/clean worktree state.
- Framework: Arduino/PlatformIO or ESP-IDF.
- Build target: `esp32s3dev`, `esp32s2dev`, `esp32s3`, or `esp32s2`.
- Serial port and baud rate.
- Firmware version as printed by `version`.
- MCU board model.
- BME280 module or sensor board model.
- Chip marking, if visible.
- Fixture description.
- Supply voltage for VDD and VDDIO.
- I2C pull-up values and whether they are on-module or external.
- BME280 address, SDO state, and CSB state.
- SDA/SCL pins and bus speed.
- Reset wiring or `N/A`.
- Interrupt wiring as `N/A` unless the bench fixture adds one.
- Temperature, humidity, and pressure reference instruments and calibration
  status, if plausibility is evaluated.
- Serial transcript path.
- Logic analyzer capture path, if used.
- Photo or video evidence path, if used.
- Operator notes and sign-off.

## Build And Flash

PlatformIO Arduino builds:

```bash
python -m platformio run -e esp32s3dev
python -m platformio run -e esp32s2dev
```

PlatformIO upload examples:

```bash
python -m platformio run -e esp32s3dev -t upload --upload-port <PORT>
python -m platformio run -e esp32s2dev -t upload --upload-port <PORT>
```

PlatformIO monitor example:

```bash
python -m platformio device monitor --port <PORT> --baud 115200
```

ESP-IDF build examples, when `idf.py` is installed:

```bash
idf.py -C examples/idf/basic set-target esp32s3
idf.py -C examples/idf/basic build
idf.py -C examples/idf/basic set-target esp32s2
idf.py -C examples/idf/basic build
```

ESP-IDF flash/monitor example:

```bash
idf.py -C examples/idf/basic -p <PORT> flash monitor
```

## Runner Commands

Dry-run the plan without opening serial:

```bash
python tools/run_i2c_hil.py --dry-run
```

Run the default serial HIL plan:

```bash
python tools/run_i2c_hil.py --port <PORT> --baud 115200 --address 0x76 --out hil_logs
```

Run with the longer soak command included:

```bash
python tools/run_i2c_hil.py --port <PORT> --baud 115200 --address 0x76 --out hil_logs --include-soak
```

Use `--address 0x77` when SDO is tied to VDDIO. The runner writes artifacts to a
new directory under `hil_logs/`:

- `serial_transcript.txt` - raw serial transcript with command boundaries.
- `summary.md` - auditor-readable result summary.
- `summary.json` - machine-readable equivalent.
- `operator_checklist.md` - manual checks and skipped unsafe/fault work.

Install `pyserial` only for non-dry-run serial execution:

```bash
python -m pip install pyserial
```

## Default Command Sequence

The guarded default sequence below must match `tools/run_i2c_hil.py` for address
`0x76`. With `--address 0x77`, only the `addr` command changes.

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
<!-- HIL_DEFAULT_SEQUENCE_END -->

`read` entries are intentional repeated commands. There is no `read 10` or
`read 20` CLI contract.

## Gated Work

The default run excludes raw writes, long soak, and physical fault injection.

- `--include-soak` adds `stress 500`.
- `--include-destructive` adds a diagnostic `wreg 0xF4 0x00` raw write and must
  be followed by recovery evidence. Raw writes can desynchronize cached driver
  settings and are unsafe for default automation.
- `--include-fault-tests` marks manual checklist items for wrong-address,
  unplug/replug, and safe SDA/SCL fault evidence as intentionally requested.
  The runner does not perform those actions, and the checklist always lists
  them so skipped fault work remains visible.

## Result Rules

- `PASS` means required serial tokens were captured for that command.
- `OPERATOR_CHECK_REQUIRED` means serial output was captured but a human must
  review environmental plausibility or bench evidence.
- `SERIAL_OK_OR_REVIEW` means serial output exists but did not prove all expected
  tokens.
- `FAIL` or `TIMEOUT` means the transcript contains a precise failure token or a
  command exceeded its deadline.
- `SKIPPED_DRY_RUN` means no serial command was sent.
- `SKIPPED_UNSAFE` means a gated destructive command was excluded.

Hardware validation is complete only after the transcript, summary, manual
checklist, and setup record are reviewed together.
