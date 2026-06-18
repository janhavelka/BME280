# BME280 I2C Uniformization Prompt

Repository: `BME280`

Absolute path: `C:\Users\Honza\Documents\Projects\BME280`

## Execution Rules

You are working inside this single repository. Implement this prompt directly;
do not repeat the cross-repository audit.

You may spawn subagents for read-only inspection of APIs, tests, I2C
transactions, docs, and diagnostics. Keep final judgment, edits, and
verification in the main agent.

Prefer simple, robust, readable code. Before adding code, inspect whether
existing code can be simplified, reused, tightened, or deleted.

Preserve dirty user changes. Do not commit unless explicitly asked.

## Common Uniformization Target

Apply this shared I2C library contract: injected non-owning transport, `Status` returns, cache-only `getSettings(SettingsSnapshot&) const`, active `probe()`/diagnostics named explicitly, `DriverState` with `state()` and `driverState()`, `isOnline()`, `lastOkMs()`, `lastErrorMs()`, `lastError()`, `consecutiveFailures()`, `totalFailures()`, and `totalSuccess()`.

Keep the common `Err` vocabulary append-only where missing: `OK`, `NOT_INITIALIZED`, `INVALID_CONFIG`, `INVALID_PARAM`, `I2C_ERROR`, `I2C_NACK_ADDR`, `I2C_NACK_DATA`, `I2C_TIMEOUT`, `I2C_BUS`, `DEVICE_NOT_FOUND`, `TIMEOUT`, `BUSY`, and `IN_PROGRESS`. Preserve BME280-specific chip-ID, calibration, compensation, measurement, and dirty-state behavior.

Uniformization is not a new base class or framework. Make only local, source-compatible additions and tests.

## Current State

- Public lifecycle and health are already close to the target shape: `DriverState` at `include\BME280\BME280.h:20`, `begin()` at line 197, `probe()` at line 263, `recover()` at line 272, `getSettings(SettingsSnapshot&)` at line 277, `driverState()` at line 306, `lastOkMs()` through `totalSuccess()` at lines 321-344.
- `SettingsSnapshot` includes read-only health and dirty state at `include\BME280\BME280.h:115-134`.
- Dirty config handling is explicit: public register writes are documented at `include\BME280\BME280.h:549-585`; implementation marks dirty at `src\BME280.cpp:1595-1605` and clears at `src\BME280.cpp:1713-1725`.
- Probe and raw register reads are separated from tracked health in `src\BME280.cpp:401-407`, `src\BME280.cpp:1506`, and `src\BME280.cpp:1529`.
- HIL runner and contract checks exist: `tools\run_i2c_hil.py`, `tools\test_run_i2c_hil_parser.py`, and `tools\check_hil_contract.py`.

## Best Sources To Adapt

- Treat this repository as a source implementation for dirty hardware config state and HIL command classification.
- For active status refresh naming, compare SHT3x `readSettings()` at `C:\Users\Honza\Documents\Projects\SHT3x-main\include\SHT3x\SHT3x.h:340-358`.
- For chunked long I2C work, do not copy SSD1315; BME280 measurement scheduling is already sensor-specific and smaller.

## Implementation Tasks

1. Make no broad core API changes. Preserve current `DriverState`, `driverState()`, `getSettings(SettingsSnapshot&)`, `probe()`, `recover()`, `softReset()`, and dirty-state contracts.
   Preserve existing compatibility aliases; do not remove or rename public APIs to achieve uniform naming.
2. Review README/Doxygen and ensure it clearly distinguishes `getSettings()` as cache-only from `probe()`, `readRegister()`, `readRegisters()`, and `softReset()` as I2C-active operations.
3. Ensure the HIL runner documentation says ACK scan is not identity proof and that chip ID `0x60` read is the identity check.
   Keep the HIL runner aligned with the common minimum contract: `version`, `scan`, `probe`, `settings`, `health`, failure-token classification, and dry-run/parser test support.
4. Keep dirty-state behavior as the reference pattern for other register-mapped sensors. Do not weaken `hardwareConfigDirtyError()` by replacing the root transport error with a generic dirty code.
5. Add any missing parser test coverage for repeated failure tokens if new HIL output formats were added since `tools\test_run_i2c_hil_parser.py`.

## API Changes Required

- None expected. Add only documentation or tests unless a concrete gap is found.

## Simplifications Before Adding Code

- If README duplicates HIL command descriptions from the runner, replace duplication with a short command reference plus a pointer to `tools\run_i2c_hil.py --help`.

## Tests To Add Or Update

- Parser tests for any new HIL output format.
- Native test: `getSettings(SettingsSnapshot&) const` is bus-silent.
- Native tests only if docs reveal a behavior mismatch. Current native suite already covers register access after `end()`, dirty config, and measurement scheduling.

## Commands To Run

- `pio test -e native`
- `pio run -e esp32s3dev`
- `python tools\test_run_i2c_hil_parser.py`
- `python tools\check_hil_contract.py`
- Live HIL only with hardware: `python tools\run_i2c_hil.py --port <PORT> --expect-address 0x76` or `0x77`, adjusted to the actual board wiring.

## Constraints And Non-Goals

- Do not introduce bus ownership, reset pins, logging, Arduino/ESP-IDF headers, or unbounded polling in core.
- Preserve distinct timeout, address NACK/device-not-found, data NACK, bus, chip-ID, calibration, compensation, measurement, and dirty-state statuses. Do not collapse them into generic `I2C_ERROR` or use `DEVICE_NOT_FOUND` for timeout/data/bus failures.
- Do not copy display-style framebuffer/flush abstractions into this sensor driver.

## Risks And Open Questions

- Open: whether the project wants all HIL parser tests to remain stdlib-only. BME280 already uses stdlib `unittest`, which is the low-dependency pattern.
