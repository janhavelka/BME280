# BME280 GAP06 HIL Calibration Timeout Fix Report

Date: 2026-06-02

## Scope

This pass is limited to HIL-runner and diagnostic evidence handling. It does not
change the core driver, public API, release metadata, library version, or
example CLI behavior.

## Root Cause

The COM16 HIL transcript from `i2c_20260602_094812` showed the cached `calib`
command timing out after the cached calibration header and temperature
coefficients, while the remaining pressure/humidity coefficients appeared at
the next command boundary. Subsequent reruns showed the same boundary symptom
on the initial `cfg` command: the command timed out with partial multi-line
output, then the remaining lines appeared under the following command.

The evidence points to a runner-side serial output-window problem, not a BME280
communication failure:

- I2C scan, address selection, `begin`, `probe`, chip ID, raw calibration,
  forced-mode, normal-mode reads, reset/recover, selftest, stress, and final
  health all produced valid serial evidence in completed runs.
- The original `calib` command did not require the final cached-calibration
  evidence line before classification.
- The runner read serial bytes only when `in_waiting` reported nonzero bytes.
  On this Windows/USB CDC path, multi-line command output could be missed until
  the next command was sent, causing command-boundary leakage and false
  timeouts.

## Code Changes

- `tools/run_i2c_hil.py`
  - Cached `calib` evidence now requires full output tokens:
    `Calibration (Cached)`, `T1=`, `P1=`, `H1=`, and `Plausibility:`.
  - Cached `calib` now uses `Plausibility:` as the completion token and a
    bounded `10.0 s` timeout.
  - `cfg` evidence now requires the final `Hardware config dirty:` line for
    the default, post-recover, and opt-in normal-soak `cfg` captures.
  - `cfg` now uses `Hardware config dirty:` as the completion token and a
    bounded `10.0 s` timeout.
  - `read_available()` now performs a bounded one-byte serial read when
    `in_waiting` reports zero, then drains any remaining bytes. This keeps the
    pyserial timeout in control while avoiding reliance on `in_waiting` as the
    only signal that data is available.
- `tools/check_hil_contract.py`
  - Added contract checks so `calib` and `cfg` cannot regress to partial-output
    evidence or shorter windows.
- `tools/test_run_i2c_hil_parser.py`
  - Added focused runner tests for complete cached calibration output, delayed
    multi-line cached calibration output, truncated calibration timeout,
    complete `cfg` output, and fallback serial reads when `in_waiting` is zero.

## Behavior Before And After

Before:

- `calib` could time out after partial cached calibration output.
- `cfg` could time out after partial settings output.
- The runner could write the next command boundary before all bytes from the
  previous command were read.
- Cached calibration evidence could match before the final plausibility line
  was captured.

After:

- `calib` cannot pass unless the final `Plausibility:` line is captured.
- `cfg` cannot pass unless the final `Hardware config dirty:` line is captured.
- Both commands still use bounded deadlines; missing or incomplete output still
  results in `TIMEOUT` rather than a pass.
- Serial polling no longer depends solely on `in_waiting` before reading.

## Local Checks

The final software check pass after the runner read-loop fix:

| Check | Result |
| --- | --- |
| `python tools/check_core_timing_guard.py` | PASS: `Core timing guard PASSED` |
| `python tools/check_cli_contract.py` | PASS: `CLI contract PASSED` |
| `python tools/check_idf_example_contract.py` | PASS: `IDF example contract PASSED` |
| `python tools/check_hil_contract.py` | PASS: `HIL contract PASSED` |
| `python scripts/generate_version.py check` | PASS: `Version.h` up to date |
| `python tools/test_run_i2c_hil_parser.py` | PASS: 6 tests passed |
| `python -m platformio test -e native` | PASS: 97 test cases, 97 succeeded |
| `python -m platformio run -e esp32s3dev` | PASS |
| `python -m platformio run -e esp32s2dev` | PASS |
| `git diff --check` | PASS; Git printed CRLF normalization warnings for modified Python files |

## HIL Runs

Requested command:

```text
python tools/run_i2c_hil.py --port COM16 --baud 115200 --address 0x76 --out hil_logs --operator Codex --operator-notes "Default HIL rerun after calib timeout/parser fix."
```

Completed rerun after tightening `calib` evidence:

- Run ID: `i2c_20260602_104343`
- Verdict: `FAIL`
- Failure: `identity-calibration / cfg` timed out.
- Finding: cached `calib` passed, but partial `cfg` output crossed into the
  following command boundary.

Completed rerun after tightening `cfg` evidence:

- Run ID: `i2c_20260602_104703`
- Verdict: `FAIL`
- Failure: `identity-calibration / cfg` timed out after `10.051 s`.
- Finding: the remaining `cfg` lines still appeared under the next command
  boundary, which drove the `read_available()` fallback fix.
- Artifacts:
  - Summary: `hil_logs/i2c_20260602_104703/summary.md`
  - Transcript: `hil_logs/i2c_20260602_104703/serial_transcript.txt`
  - CSV: `hil_logs/i2c_20260602_104703/results.csv`
  - JSON: `hil_logs/i2c_20260602_104703/summary.json`
  - Manifest: `hil_logs/i2c_20260602_104703/manifest.json`

Final rerun attempt after the serial read-loop fix:

- Run ID: `i2c_20260602_104924`
- Verdict: no HIL verdict generated.
- Result: pyserial could not open `COM16`; Windows reported that the port does
  not exist.
- Visible serial ports at the time were `COM3`, `COM4`, `COM5`, `COM6`,
  `COM10`, `COM13`, and `COM18`. `COM16` was not enumerated.
- Artifact:
  - Partial transcript only: `hil_logs/i2c_20260602_104924/serial_transcript.txt`

## Remaining HIL Limitations

- The default HIL plan does not currently have a passing post-fix COM16 run.
- The last completed HIL run before the serial read-loop fix still failed at
  the initial `cfg` command.
- The final post-fix HIL attempt could not run because `COM16` was unavailable.
- Environmental plausibility rows still require operator review and reference
  instruments.
- Long forced soak, opt-in normal-mode soak, destructive raw-write diagnostics,
  and manual fault tests were not run.
- Board, MCU target, module, supply, pull-up, pin, bus-speed, and reference
  instrument metadata were not provided and remain unknown in generated HIL
  artifacts.

## Default HIL Pass Status

Default HIL plan now passes: **No, not proven**.

The software-side evidence fix is implemented and local checks pass, but a
passing default HIL run still needs to be captured after `COM16` is available
again or after the operator confirms the correct replacement port.
