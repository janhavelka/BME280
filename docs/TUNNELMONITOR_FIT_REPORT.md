# TunnelMonitor Fit Report

Audit scope: synchronous API classification, visible timeout/error behavior, raw and
fixed-point output availability, and optional ENV absence-vs-fault mapping.
Poll chunking details such as exact job phase sequencing and instruction budgets
belong to the companion prompt.

## API Classification

| API | Classification | Notes for TunnelMonitor |
| --- | --- | --- |
| `begin(const Config&)` | Lifecycle/setup | Synchronous multi-transfer setup. It validates config, reads chip ID, checks NVM readiness once, reads calibration, validates calibration, and applies config. It is not a steady poll path. If NVM is still updating it now returns visible `BUSY` or `TIMEOUT` instead of spinning. |
| `recover()` | Lifecycle/manual recovery | Synchronous tracked chip-ID read plus cached config re-apply. It is application-controlled recovery policy, not ENV health policy. |
| `_applyConfig()` | Lifecycle/setup helper | Synchronous multi-write helper used by setup/recovery/config paths. Not a steady-path candidate. |
| `requestMeasurement()` | Steady-path candidate with caveat | Normal mode only schedules state. Forced mode can perform immediate I2C work and should be wrapped or replaced by a step-driven path when a one-transfer-per-poll owner is required. |
| `tick(uint32_t nowMs)` | Steady-path candidate with caveat | Before a deadline it performs no I2C. At readiness it may synchronously read status and burst data. Use with an owner wrapper that budgets those transfers, or use the step-driven path from the companion work. |
| `_waitForNvmReady()` | Lifecycle/setup wait check | Now performs at most one raw status read per call. It returns `OK`, `BUSY`, `TIMEOUT`, or the detailed transport error. It no longer hides a tight polling loop. |
| Synchronous register/config setters | Convenience/diagnostic/config | Single-register helpers are one transfer, but typed setters can chain several writes. Keep them outside TunnelMonitor's steady owner poll unless explicitly budgeted. |
| `getRawSample()` / `getCompensatedSample()` | Steady data access | No I2C. Raw ADC and fixed-point outputs are first-class. `CompensatedSample::pressurePa` remains integer Pascals. |
| `getMeasurement()` | Convenience-only | No I2C, but converts cached fixed-point data to float and clears the ready flag. |

## Timeout And Error Behavior

- `Config::i2cTimeoutMs` is passed to transport callbacks; adapters decide how to
  enforce it.
- `Config::nvmReadyTimeoutMs` controls the NVM-ready deadline used by synchronous
  setup/reset checks and the staged init path.
- Address NACK during chip-ID presence checks maps to `DEVICE_NOT_FOUND`, which
  adapter authors may treat as optional ENV absence.
- Data NACK, timeout, bus errors, and generic transport faults are preserved as
  their original `I2C_*` or `I2C_ERROR` status codes.
- BME280 has no CRC result path. Calibration integrity is represented by
  `CALIBRATION_INVALID` when cached coefficients fail local validation.

## Integration Notes

TunnelMonitor should treat lifecycle/setup and manual recovery separately from
steady ENV polling. The driver still owns only device-local health counters; it
does not encode TunnelMonitor row/device policy. For optional ENV mapping, use:

- `DEVICE_NOT_FOUND`: candidate optional absence when caused by address NACK.
- `I2C_TIMEOUT`, `I2C_BUS`, `I2C_NACK_DATA`, `I2C_ERROR`: bus/device fault, not
  silent optional absence.
- `CALIBRATION_INVALID`: device responded but returned unusable compensation data.
