# ESP32-S2 COM28 HIL Summary

This is the compact release-facing record for BME280 serial HIL runs on one
ESP32-S2 board using the PlatformIO Arduino `esp32s2dev` example.

## Scope

| Field | Value |
| --- | --- |
| Board | ESP32-S2 |
| Port | `COM28` |
| Framework | PlatformIO Arduino |
| Build target | `esp32s2dev` |
| BME280 address | `0x76` |
| Chip ID | `0x60` |
| SDA / SCL | GPIO8 / GPIO9 |
| Bus speed | 400000 Hz |
| SDO / CSB | SDO inferred GND, CSB inferred high/I2C mode |

## Evidence Summary

| Date | Run | Result |
| --- | --- | --- |
| 2026-06-22 | Functional HIL, pre-`--include-job-api` | 61 PASS, 22 OPERATOR_CHECK_REQUIRED, 1 reset/NVM UNKNOWN, 0 FAIL, 0 TIMEOUT |
| 2026-06-22 to 2026-06-23 | 8-hour soak, pre-`--include-job-api` | 11497 PASS, 7297 OPERATOR_CHECK_REQUIRED, 35 reset/NVM UNKNOWN, 0 FAIL, 0 TIMEOUT |
| 2026-06-23 to 2026-06-24 | 20-hour comprehensive soak, run `i2c_20260623_140356` | 83313 PASS, 51701 OPERATOR_CHECK_REQUIRED, 1206 reset/NVM UNKNOWN, 0 FAIL, 0 TIMEOUT |
| 2026-06-24 | Reset-BUSY classifier 180 s HIL, run `i2c_20260624_225853` | 354 PASS, 127 OPERATOR_CHECK_REQUIRED, 54 PASS_WITH_RESET_BUSY_RECOVERED, 0 UNKNOWN, 0 FAIL, 0 TIMEOUT |
| 2026-06-24 | Reset-BUSY classifier 60 s HIL, run `i2c_20260624_230410` | 132 PASS, 48 OPERATOR_CHECK_REQUIRED, 19 PASS_WITH_RESET_BUSY_RECOVERED, 0 UNKNOWN, 0 FAIL, 0 TIMEOUT |

## Reset/NVM Interpretation

The repeated reset observations were:

```text
Status: BUSY
Message: NVM update in progress
```

This matches the BME280 `status.im_update` behavior while NVM calibration data
are copied after reset. The runner now records this as
`PASS_WITH_RESET_BUSY_RECOVERED` only when immediate follow-up evidence proves:

- `recover` returns `Status: OK`;
- post-reset status has `im_update=0` and `measuring=0`;
- driver state is `READY`;
- hardware config dirty state is `false`.

The post-classifier short HIL runs proved that rule on COM28 with no UNKNOWN,
FAIL, or TIMEOUT rows.

## Claim Boundary

ESP32-S2 Arduino `0x76` serial endurance and staged-job API evidence: PASS for
the tested board and firmware boundary above.

Full production qualification remains incomplete for:

- calibrated temperature, pressure, and humidity accuracy;
- protected electrical/fault injection;
- ESP-IDF runtime HIL;
- ESP32-S3 hardware HIL;
- address `0x77`;
- shared-bus contention under a real application scheduler.

Raw HIL artifacts under `hil_logs/` were development-local and intentionally
not committed. Re-run HIL or package artifacts with manifests and hashes before
using this as formal release evidence.
