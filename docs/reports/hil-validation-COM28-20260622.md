# BME280 HIL Validation And Repository Audit - COM28 - 2026-06-22

Date range: 2026-06-22T20:59:29+02:00 to 2026-06-23T05:02:43+02:00  
Operator: Codex  
Repository: `c:\Users\Honza\Documents\Projects\BME280`  
Branch: `main`  
Commit: `0aa7c21049edd844248968626f170f8b1ea9e15e`  
Worktree: dirty during validation

## Claim Boundary

This report records serial HIL evidence from one ESP32-S2 board on `COM28` using the PlatformIO Arduino example. It does not prove calibrated environmental accuracy, field readiness, destructive fault recovery, ESP-IDF runtime behavior, or multi-board coverage.

Rows marked `OPERATOR_CHECK_REQUIRED` passed command/validator checks but require human review against physical setup and reference instruments. Rows marked `UNKNOWN` were bounded reset/NVM-busy observations and are treated as anomalies requiring review, not silent pass results.

This run predates the `v1.7.0` `--include-job-api` HIL group. Staged-job CLI
hardware coverage is therefore `NOT RUN` in this report unless a later artifact
explicitly records it.

## Setup

| Item | Value |
| --- | --- |
| Board | ESP32-S2 |
| Port | `COM28` |
| Baud | `115200` |
| Framework exercised on hardware | PlatformIO Arduino |
| PlatformIO env flashed | `esp32s2dev` |
| I2C SDA/SCL | GPIO8/GPIO9 |
| I2C bus speed | 400000 Hz |
| BME280 address | `0x76` |
| Inferred SDO state | GND, inferred from `0x76` |
| Inferred CSB state | High/I2C mode, inferred from successful I2C |
| BME280 module/VDD/pullups | Not independently identified |
| Other detected I2C addresses | `0x3C`, `0x6A` |

The scan showed three devices on the bus:

```text
0x3C, 0x6A, 0x76
Scan complete. Found 3 device(s).
```

BME280 identity was verified by reading chip ID `0x60` at address `0x76`.

## Host Tools

| Tool | Result |
| --- | --- |
| Windows | Windows 11 Education 64-bit, version 10.0.26200 |
| Python | 3.12.10 |
| PlatformIO Core | 6.1.18 |
| pyserial | 3.5 |
| Doxygen | 1.13.2 |
| `idf.py` | Not found on PATH; native ESP-IDF build not run |

## Commands Run

Build/flash/reset commands:

```powershell
python -m platformio run -e esp32s2dev
python -m platformio run -e esp32s3dev
python -m platformio run -e esp32s2dev --target upload --upload-port COM28
python $env:USERPROFILE\.platformio\packages\tool-esptoolpy\esptool.py --chip esp32s2 --port COM28 run
```

Functional HIL command:

```powershell
python tools\run_i2c_hil.py --port COM28 --baud 115200 --timeout-s 8 --boot-settle-s 2 --include-config-matrix --include-invalid-inputs --include-benchmarks --include-normal-soak --normal-soak-count 10 --normal-soak-interval-s 1 --include-soak --soak-count 500 --operator Codex --board ESP32-S2 --mcu-target esp32s2 --framework PlatformIO-Arduino --build-target esp32s2dev --sda-pin GPIO8 --scl-pin GPIO9 --bus-speed 400000Hz --sdo-state GND-inferred-from-0x76 --csb-state high-inferred-from-I2C-mode --operator-notes "User reported ESP32S2 board on COM28; BME280 at 0x76 found on I2C scan; no unsafe fault fixture or reference instrument provided."
```

8-hour soak command:

```powershell
python tools\run_i2c_hil.py --port COM28 --baud 115200 --timeout-s 8 --boot-settle-s 2 --soak-duration-s 28800 --soak-cycle-stress-count 500 --soak-cycle-mix-count 140 --soak-reset-interval 100 --operator Codex --board ESP32-S2 --mcu-target esp32s2 --framework PlatformIO-Arduino --build-target esp32s2dev --sda-pin GPIO8 --scl-pin GPIO9 --bus-speed 400000Hz --sdo-state GND-inferred-from-0x76 --csb-state high-inferred-from-I2C-mode --operator-notes=8h_soak_after_functional_HIL_no_unsafe_fault_fixture_or_reference_instrument
```

Local validation commands:

```powershell
python tools\run_i2c_hil.py --parser-self-test
python tools\test_run_i2c_hil_parser.py
python tools\check_hil_contract.py
python tools\check_cli_contract.py
python tools\check_idf_example_contract.py
python tools\check_core_timing_guard.py
python scripts\generate_version.py check
python tools\check_release_metadata.py
python -m py_compile tools\run_i2c_hil.py tools\check_hil_contract.py tools\check_release_metadata.py
python -m platformio test -e native
python -m platformio run -e esp32s2dev
python -m platformio run -e esp32s3dev
python -m platformio pkg pack
python tools\check_package_contents.py
doxygen Doxyfile
```

## Artifact Index

Functional HIL artifacts:

- Serial transcript: `hil_logs\i2c_20260622_205926\serial_transcript.txt`
- Summary: `hil_logs\i2c_20260622_205926\summary.md`
- JSON: `hil_logs\i2c_20260622_205926\summary.json`
- CSV: `hil_logs\i2c_20260622_205926\results.csv`
- Environment: `hil_logs\i2c_20260622_205926\environment.txt`

8-hour soak artifacts:

- Serial transcript: `hil_logs\i2c_20260622_210228\serial_transcript.txt`
- Summary: `hil_logs\i2c_20260622_210228\summary.md`
- JSON: `hil_logs\i2c_20260622_210228\summary.json`
- CSV: `hil_logs\i2c_20260622_210228\results.csv`
- Environment: `hil_logs\i2c_20260622_210228\environment.txt`
- Manifest: `hil_logs\i2c_20260622_210228\manifest.json`

Earlier tooling-adjustment artifact:

- `hil_logs\i2c_20260622_205606`: initial reset classification attempt before `Status: BUSY` was changed from timeout to `UNKNOWN`.

## Functional HIL Summary

Run ID: `i2c_20260622_205926`  
Time: 2026-06-22T21:00:23+02:00  
Final verdict: `OPERATOR_REVIEW_REQUIRED`

| Result | Count |
| --- | ---: |
| `PASS` | 61 |
| `OPERATOR_CHECK_REQUIRED` | 22 |
| `UNKNOWN` | 1 |
| `FAIL` | 0 |
| `TIMEOUT` | 0 |

Functional group coverage:

| Group | Rows | PASS | Operator Review | UNKNOWN | FAIL/TIMEOUT |
| --- | ---: | ---: | ---: | ---: | ---: |
| `provenance` | 2 | 2 | 0 | 0 | 0 |
| `bus-reachability` | 4 | 4 | 0 | 0 | 0 |
| `identity-calibration` | 7 | 7 | 0 | 0 | 0 |
| `forced-mode` | 8 | 5 | 3 | 0 | 0 |
| `normal-mode` | 4 | 2 | 2 | 0 | 0 |
| `reset-recover` | 5 | 4 | 0 | 1 | 0 |
| `stress-health` | 4 | 2 | 2 | 0 | 0 |
| `soak-forced` | 1 | 0 | 1 | 0 | 0 |
| `soak-normal` | 14 | 4 | 10 | 0 | 0 |
| `config-matrix` | 23 | 22 | 1 | 0 | 0 |
| `invalid-input` | 8 | 8 | 0 | 0 | 0 |
| `benchmark` | 4 | 1 | 3 | 0 | 0 |

Key functional observations:

| Area | Evidence |
| --- | --- |
| Identity | `chipid` and `reg 0xD0` returned `0x60`. |
| Calibration | Cached coefficients read and plausibility section emitted; no calibrated reference review was available. |
| Default timing | Measurement 11 ms, standby 125 ms, estimated normal cycle 136 ms. |
| Max oversampling timing | Measurement 114 ms, standby 20 ms, estimated normal cycle 134 ms. |
| Forced mode | Post-force `reg 0xF4 = 0x24`; mode bits were sleep after forced conversion. |
| Normal mode | Repeated reads completed; samples require operator/reference review. |
| Invalid inputs | Unknown command, invalid address, invalid mode, invalid oversampling/filter/standby/register, and incomplete `wreg` were rejected. |
| Benchmarks | `stress 500`: 500 attempts, 0 errors, 6003 ms, 83.29 samples/s. `stress_mix 140`: 140 ok, 0 fail, 311 ms, 450.16 ops/s. |

Representative functional samples from serial output:

| Mode/Phase | Sample |
| --- | --- |
| Initial forced read | 25.79 C, 99738 Pa, 43.87 %RH |
| Forced read after trigger | 25.80 C, 99740 Pa, 43.73 %RH |
| Normal-mode repeated read | 25.82 C, 99743 Pa, 43.93 %RH |
| Max oversampling forced read | 25.88 C, 99730 Pa, 43.68 %RH |

These values were plausible for an indoor bench but were not checked against a calibrated reference instrument.

## 8-Hour Soak Summary

Run ID: `i2c_20260622_210228`  
Start: 2026-06-22T21:02:43+02:00  
End: 2026-06-23T05:02:43+02:00  
Elapsed: 28800.0 s  
Stop reason: deadline reached before next command  
Final verdict: `OPERATOR_REVIEW_REQUIRED`

| Result | Count |
| --- | ---: |
| `PASS` | 11497 |
| `OPERATOR_CHECK_REQUIRED` | 7297 |
| `UNKNOWN` | 35 |
| `FAIL` | 0 |
| `TIMEOUT` | 0 |

Soak workload:

| Item | Count/Range |
| --- | ---: |
| Generated soak command rows | 18795 |
| Runner cycle steps | 21004 |
| Max command cycle | 3402 |
| `stress 500` rows | 3402 |
| `stress_mix 140` rows | 3402 |
| `status` rows | 3436 |
| `drv` rows | 3402 |
| Periodic `reset` rows | 34 |
| Periodic `recover` rows | 34 |
| Normal-mode read rows | 486 |

Soak timing and throughput:

| Command | Rows | Min elapsed s | Avg elapsed s | Max elapsed s |
| --- | ---: | ---: | ---: | ---: |
| `stress 500` | 3402 | 6.343 | 6.369 | 6.391 |
| `stress_mix 140` | 3402 | 0.625 | 0.653 | 0.704 |

Final soak health from the transcript tail:

```text
State: READY
Online: yes
Active I2C address: 0x76
Hardware config dirty: false
Consecutive failures: 0
Total success: 8246537
Total failures: 35
Error code: BUSY
Error detail: 10
Error msg: NVM update in progress
Mode: FORCED
Chip ID: 0x60
```

Final normal-mode sample before soak end:

```text
Temp: 27.23 C, Pressure: 99757.00 Pa, Humidity: 41.75 %
Valid channels: T=1 P=1 H=1
```

## Anomalies

1. `reset` returned bounded `Status: BUSY` / `NVM update in progress`.
   - Functional run: 1 `UNKNOWN` row.
   - 8-hour soak: 34 periodic reset `UNKNOWN` rows plus the initial functional reset row for 35 total in the long run summary.
   - Every occurrence was bounded by the runner timeout and classified `UNKNOWN`, not `PASS`.
   - Follow-up `status`, `recover`, `cfg`, `probe`, and `chipid` rows passed. Final driver state was `READY`, `dirty=false`, `consecutiveFailures=0`.

2. Post-reset functional status briefly showed `dirty=true`.
   - This is expected after incomplete reset/resync evidence.
   - The following `recover` row cleared dirty state (`Dirty: true -> false`) and subsequent `cfg`/`status` showed `Hardware config dirty: false`.

3. Driver health printed `Success rate: 100.0%` with nonzero total failures during the long soak.
   - The raw counters are preserved in the transcript and report (`8246537` successes, `35` failures).
   - The displayed success rate appears rounded and should not be used as the sole health metric.

No `FAIL` or `TIMEOUT` result markers were recorded in the functional or 8-hour soak summaries.

## Repository Audit Findings

### Fixed: Offline typed config setters could mark dirty without I2C

Finding: typed config setters in an `OFFLINE` latch path could report local `BUSY` without touching hardware, but still mark hardware config dirty. That conflated "write may have reached device" with "write was locally suppressed".

Implemented fix:

- Added `mayHaveReachedDeviceAfterDiagnosticWrite()` in `src/BME280.cpp`.
- Used it in typed config paths such as `setMode()`, oversampling setters, filter, and standby.
- Address/data NACK and local/offline BUSY no longer mark dirty unless the write may have reached the device.

Relevant locations:

- `src/BME280.cpp:139`
- `src/BME280.cpp:1289`
- `src/BME280.cpp:1328`
- `src/BME280.cpp:1353`
- `src/BME280.cpp:1377`
- `src/BME280.cpp:1387`
- `test/test_basic.cpp:2127`
- `test/test_basic.cpp:3092`

Verification:

- `python -m platformio test -e native`: 121/121 tests passed.

### Fixed: Stuck measuring waits could remain pending indefinitely

Finding: caller-driven `tick()` and staged forced/config jobs could keep returning pending/in-progress indefinitely if the BME280 status `measuring` bit stayed set and time did not advance as expected or the bit never cleared.

Implemented fix:

- Added `MEASURING_READY_MAX_POLLS = 255`.
- Added `_measurementDeadlineMs`, `_measurementStatusPolls`, and `_jobWaitPolls`.
- `tick()` now returns terminal `TIMEOUT` and clears pending state after deadline or poll cap.
- Staged forced measurement and apply-config wait phases now have bounded deadline and poll-cap exits.

Relevant locations:

- `src/BME280.cpp:18`
- `src/BME280.cpp:344`
- `src/BME280.cpp:386`
- `src/BME280.cpp:851`
- `src/BME280.cpp:897`
- `src/BME280.cpp:1021`
- `include/BME280/BME280.h:736`
- `include/BME280/BME280.h:767`
- `include/BME280/BME280.h:768`
- `test/test_basic.cpp:2348`
- `test/test_basic.cpp:2785`
- `test/test_basic.cpp:3100`
- `test/test_basic.cpp:3106`

Verification:

- `python -m platformio test -e native`: 121/121 tests passed.

### Fixed: HIL runner coverage and reset classification

Implemented:

- Parser self-test mode.
- `--timeout-s` alias and additional serial timing controls.
- Functional coverage options for config matrix, invalid inputs, benchmarks, normal soak, and safe soak.
- 8-hour duration soak support with bounded command deadlines.
- `stress_mix` parser/validator.
- `UNKNOWN` result contract for bounded but incomplete reset/NVM-busy evidence.
- Raw-write safety gate that does not block malformed/incomplete `wreg` validation tests.

Relevant locations:

- `tools/run_i2c_hil.py`
- `tools/test_run_i2c_hil_parser.py`
- `tools/check_hil_contract.py`
- `docs/I2C_HIL_RUNBOOK.md`
- `README.md`

Verification:

- `python tools\run_i2c_hil.py --parser-self-test`: pass.
- `python tools\test_run_i2c_hil_parser.py`: pass, 14 tests.
- `python tools\check_hil_contract.py`: pass.

### Residual: stale successful sample contract after failed refresh

Existing behavior preserves the latest successful sample after later measurement/read failures, with status/timestamps available for callers to distinguish freshness. This can be useful, but the user-facing contract should stay explicit: consumers must check `lastMeasurementStatus`/timestamps before treating cached values as fresh.

Recommendation: keep current behavior only if documented as "latest successful sample" semantics; otherwise add invalidation on failed refresh and update tests/docs accordingly.

### Residual: staged recovery ordering differs from synchronous recover

The staged recovery flow and synchronous recover path do not have identical operation ordering in all phases. No HIL failure was observed from this, but the contract should be decided deliberately so both paths either match or document why they differ.

Recommendation: align staged and synchronous recovery sequencing in a follow-up, or document the distinction with tests proving both paths clear dirty state only after full resync.

## Limitations And Not-Run Items

- No calibrated temperature, pressure, or humidity reference instrument was used.
- No VDD/VDDIO measurements, pull-up values, bus capacitance, or logic-analyzer waveforms were captured.
- No destructive or unsafe fault tests were run: bus short, hot unplug, stuck SDA/SCL fixture, brownout, power rail cycling, or sensor removal.
- No native ESP-IDF hardware run was performed; `idf.py` was not available on PATH.
- ESP32-S3 hardware was not run.
- Multi-task concurrency, ISR misuse, and shared-bus lock behavior were not validated on hardware.
- The bus had other devices at `0x3C` and `0x6A`; no contention was observed, but the bus was not isolated.
- HIL logs are local artifacts under `hil_logs\`; this report references them but does not embed full transcripts.

## Final Verification Status

| Check | Result |
| --- | --- |
| Parser self-test | PASS |
| HIL parser unit tests | PASS, 14 tests |
| HIL contract checker | PASS |
| CLI contract checker | PASS |
| IDF example contract checker | PASS |
| Core timing guard | PASS |
| Version check | PASS |
| Release metadata check | PASS |
| Python compile checks | PASS |
| Native PlatformIO tests | PASS, 121/121 |
| PlatformIO `esp32s2dev` build | PASS |
| PlatformIO `esp32s3dev` build | PASS |
| Package contents check | PASS |
| Doxygen | PASS |
| ESP-IDF native build | NOT RUN, `idf.py` unavailable |

## Conclusion

The ESP32-S2/COM28 PlatformIO-Arduino HIL run found the BME280 at address `0x76`, verified chip ID `0x60`, exercised forced mode, normal mode, config boundaries, invalid input handling, reset/recover, benchmarks, and completed an 8-hour soak with zero `FAIL`/`TIMEOUT` result markers.

The run does not close calibrated accuracy or destructive fault-validation requirements. The main code findings uncovered during audit were fixed and covered by native tests. Remaining work is contract-level cleanup around cached sample freshness, recovery sequencing parity, and native ESP-IDF hardware/build validation.
