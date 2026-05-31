# BME280 Pre-HIL Readiness Report

Date: 2026-05-31
Branch: `hardening/bme280-industry-readiness`
Starting HEAD: `2c4098ec027cfc3d07ea323b204fdb86d94c8cfa`
Final HEAD: this pass commit; exact hash is returned by Git in the final response.

## Scope

This was a focused pre-HIL polish pass. No public API changes were made. Core
transport ownership, dirty-state diagnostics, and transport error precision were
preserved. Hardware validation was not run.

## Changes Made

- Added minimal FakeBus address discrimination for native tests. The fake bus
  now has a configured device address and returns deterministic
  `I2C_NACK_ADDR` when the driver uses the wrong 7-bit address.
- Added a FakeBus status-read time-advance hook for focused NVM deadline and
  wraparound coverage.
- Added native tests for:
  - `test_begin_address_nack_maps_to_device_not_found`
  - `test_begin_accepts_both_supported_addresses_and_rejects_wrong_address`
  - `test_begin_nvm_wait_handles_monotonic_wraparound`
  - `test_recover_preserves_cached_sample_and_dirty_until_successful_resync`
- Updated the hardware validation matrix with required HIL evidence fields and
  a minimum command evidence plan.
- Linked this pre-HIL report from `README.md` and included it in `Doxyfile`.

## Partial Mutation Modeling Decision

Native tests model transaction-boundary failures: a write transaction either
succeeds and mutates FakeBus registers, or fails before mutation. This matches
what the driver can observe through its transport callbacks.

The requested single-I2C-call partial internal mutation case, where a target
device changes one or more bytes internally before returning a failed
transaction, is not modeled in the native fake transport. Adding that behavior
would make the fake bus less representative of the driver's transaction-level
contract. This remains a HIL or bench fault-injection case, ideally captured
with a real adapter, safe SDA/SCL fault, power/reset interruption, or logic
analyzer trace.

## Local Check Results

Commands were run locally on 2026-05-31.

| Command | Result |
| --- | --- |
| `git status --short` at start | PASS, clean |
| `git branch --show-current` | PASS, `hardening/bme280-industry-readiness` |
| `git log --oneline -5` | PASS, starting HEAD `2c4098e` |
| `python tools/check_core_timing_guard.py` | PASS, `Core timing guard PASSED` |
| `python tools/check_cli_contract.py` | PASS, `CLI contract PASSED` |
| `python tools/check_idf_example_contract.py` | PASS, `IDF example contract PASSED` |
| `python scripts/generate_version.py check` | PASS, `Version.h` up to date |
| Core framework-leakage `rg` scan in `include src` | PASS, no matches |
| `python -m platformio test -e native` | PASS, 88/88 tests, `00:00:03.231` |
| `python -m platformio run -e esp32s3dev` | PASS, `00:00:19.848` |
| `python -m platformio run -e esp32s2dev` | PASS, `00:00:19.566` |
| `python -m platformio pkg pack` | PASS, wrote `BME280-1.5.0.tar.gz` |
| `python tools/check_package_contents.py` | PASS, `Package contents PASSED (BME280-1.5.0.tar.gz)` |
| Package archive cleanup | PASS, `BME280-1.5.0.tar.gz` removed |
| `git diff --check` | PASS |
| `idf.py --version` | FAIL, `idf.py` is not installed locally |

Local ESP-IDF builds were not run because `idf.py` is unavailable on this
machine. HIL was not run.

## HIL Operator Checklist

Record these fields before marking any hardware test PASS or FAIL:

- Board model.
- MCU target: ESP32-S2 and/or ESP32-S3.
- BME280 module or board model.
- Supply voltage for VDD and VDDIO.
- I2C pull-up values and whether pull-ups are on-module or external.
- BME280 address: `0x76` or `0x77`.
- SDO state: GND for `0x76`, VDDIO for `0x77`.
- CSB state: tied to VDDIO for I2C mode.
- SDA/SCL pins.
- Bus speed.
- Firmware commit hash.
- CLI or serial command transcript.
- Environmental reference used, if any.
- Pass/fail result and notes for each test.

Example defaults to verify or override:

- Arduino example defaults: SDA GPIO8, SCL GPIO9, 400 kHz, 50 ms transaction
  timeout.
- ESP-IDF example defaults: SDA GPIO8, SCL GPIO9, 400 kHz, 50 ms transaction
  timeout.
- PlatformIO board targets: `esp32s3dev` and `esp32s2dev`.

## Recommended HIL Command Sequence

Use the actual supported commands below. Commands marked as equivalents are
intentional because the CLI does not implement the exact requested spelling.

```text
version
scan
probe
chipid
cfg
calib
status
read
repeat read 10 times, or run stress 10 and record that it is the available counted-read equivalent
force
read
normal on
repeat read 20 times; record that true read 20 is not currently implemented
reset
probe
cfg
read
selftest
stress 500
```

Supported equivalents:

- `cal` equivalent: `calib` or `calib raw`.
- `forced` equivalent: `force` or `mode forced`.
- `normal` setup: `normal on`; use `normal off` to return to sleep.
- `softreset` equivalent: `reset`.
- `read 10` / `read 20`: no exact counted-read command. Use repeated manual
  `read` commands for normal-mode evidence, and optionally `stress N` for an
  automated forced-measurement stress equivalent.

## Fault-Path HIL Plan

- Wrong-address probe: use `addr 0x76` and `addr 0x77` against the opposite SDO
  wiring and record `probe`/`chipid` status.
- Sensor unplug or address NACK: disconnect only when safe for the board, record
  `probe`, `read`, and `recover` behavior, then reconnect and recover.
- Temporary SDA/SCL fault: perform only with safe bench current limits and record
  whether the transport reports timeout, bus error, or NACK.
- Reset during measurement: start a measurement, reset the sensor or board, then
  record `status`, `recover`, `cfg`, and `read`.
- Recover after unplug/replug: record state before unplug, during failure, after
  reconnect, and after `recover`.
- Longer soak in normal mode: run `normal on`, periodic repeated `read`, and a
  longer `stress` run as separate evidence.
- Plausibility comparison: compare temperature and humidity against a local
  reference, and pressure against local station/reference pressure adjusted for
  site conditions where possible.

## Remaining HIL-Only Risks

- Electrical wiring, pull-up strength, bus capacitance, and brownout behavior.
- True single-transaction partial internal mutation on physical I2C failure.
- Sensor module assembly handling and humidity exposure history.
- Environmental accuracy against known references.
- Long-duration normal-mode operation and application-owned shared-bus locking.

## Readiness Verdict

Ready for HIL: YES, with no software blocker before connecting hardware.

The only pre-connection requirements are operator-owned: select the board and
BME280 module, wire SDO/CSB/SDA/SCL/VDD/VDDIO deliberately, document pull-ups
and bus speed, build firmware from the final commit, and record the transcript.

Merge/release wording must remain conservative: this pass adds software tests
and an HIL procedure. It does not prove hardware operation, field robustness,
humidity accuracy, or local pure ESP-IDF builds.
