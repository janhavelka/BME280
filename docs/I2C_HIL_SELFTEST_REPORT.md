# BME280 I2C HIL Self-Test Report

Date: 2026-05-31
Branch: `hardening/bme280-industry-readiness`
Starting commit: `e4e859c44efe61aba27eb6784a576aed79b82d5c`
Final commit: recorded by Git after this report is committed.

## Scope

This phase added a host-side serial HIL runner, auditor-facing HIL documents,
and a repo-local contract guard. No physical HIL validation was performed.

Hardware run: NOT RUN

The runner is intended for the existing BME280 Arduino and ESP-IDF diagnostic
CLIs. It does not flash firmware. It sends supported serial commands, records
transcripts, writes structured summaries, and separates serial evidence from
operator-required plausibility review.

## Added Or Updated Files

- `tools/run_i2c_hil.py`
- `tools/check_hil_contract.py`
- `docs/I2C_HIL_RUNBOOK.md`
- `docs/I2C_HIL_TARGET_TEMPLATE.md`
- `docs/I2C_HIL_SELFTEST_REPORT.md`
- `.gitignore`
- `README.md`
- `Doxyfile`
- `docs/BME280_HARDWARE_VALIDATION_MATRIX.md`
- `docs/BME280_PRE_HIL_READINESS_REPORT.md`
- `docs/BME280_INDUSTRY_READINESS_REPORT.md`

## Runner Command

Dry-run command used for software validation:

```bash
python tools/run_i2c_hil.py --dry-run
```

Hardware command to run after a physical target and serial port are selected:

```bash
python tools/run_i2c_hil.py --port <PORT> --baud 115200 --address 0x76 --out hil_logs
```

Optional longer soak:

```bash
python tools/run_i2c_hil.py --port <PORT> --baud 115200 --address 0x76 --out hil_logs --include-soak
```

## Default Command Sequence

The guarded default sequence below matches `tools/run_i2c_hil.py` for address
`0x76`.

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

## Semantics

- `scan` is ACK-only evidence and does not prove BME280 identity.
- `probe` is diagnostic reachability evidence, not a replacement for transcript
  evidence of the chip ID.
- `chipid` and `reg 0xD0` are the auditable identity checks and must record
  `0x60`.
- `selftest` is a safe command smoke check. It is not Bosch factory calibration,
  not hardware qualification, and not sensor accuracy proof.
- `read`, `force`, `normal on`, and `stress N` require operator plausibility
  review unless reference instruments and limits are recorded.
- `wreg`, long soak, and fault tests are excluded from the default sequence and
  require explicit opt-in or manual bench action.

## Local Validation Results

Commands were run locally on 2026-05-31.

| Command | Result |
| --- | --- |
| `python -m py_compile tools/run_i2c_hil.py` | PASS, no output |
| `python tools/run_i2c_hil.py --dry-run` | PASS, wrote ignored artifacts under `hil_logs/i2c_20260531_150948`, final verdict `INCOMPLETE`, no physical HIL validation |
| `python tools/check_hil_contract.py` | PASS, `HIL contract PASSED` |
| `python tools/check_core_timing_guard.py` | PASS, `Core timing guard PASSED` |
| `python tools/check_cli_contract.py` | PASS, `CLI contract PASSED` |
| `python tools/check_idf_example_contract.py` | PASS, `IDF example contract PASSED` |
| `python scripts/generate_version.py check` | PASS, `Version.h` up to date |
| `python -m platformio test -e native` | PASS, 88/88 tests, `00:00:05.365` |
| `python -m platformio run -e esp32s3dev` | PASS, `00:00:27.622` |
| `python -m platformio run -e esp32s2dev` | PASS, `00:00:27.357` |
| `python -m platformio pkg pack` | PASS, wrote `BME280-1.5.0.tar.gz` |
| `python tools/check_package_contents.py` | PASS, `Package contents PASSED (BME280-1.5.0.tar.gz)` |
| Package archive cleanup | PASS, `BME280-1.5.0.tar.gz` removed |
| `idf.py --version` | FAIL, `idf.py` is not installed locally |
| `gh --version` | FAIL, `gh` is not installed locally |
| `git diff --check` | PASS, only Git line-ending conversion warnings |

## Validation Boundaries

No physical BME280 hardware was connected, no serial hardware session was run,
and no ESP-IDF local build is claimed unless `idf.py` is available and the exact
commands pass. GitHub Actions status is reported only if `gh` is available in
the local environment.
