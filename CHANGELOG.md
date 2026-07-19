# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

No unreleased changes.

## [2.0.0] - 2026-07-19

### Added

- Zero-I2C cancellation, nonzero job identity, public `JobPhase`, conversion
  ambiguity, chip-phase deadline, callback-usage, and exactly-once terminal
  result diagnostics for the fixed-memory staged runner.
- Separate non-reset `startResyncJob()` and explicit
  `startSoftResetJob()` operations; legacy `startRecoveryJob()` is a non-reset
  compatibility alias.
- `ConfigSyncState`, `CalibrationState`, configuration generations,
  `SampleEnvelope`, sample sequences, and zero-I2C device/calibration
  invalidation for safe hotplug/replacement handling.
- Terminal-only `TransportResult` / `TransportErr` with exact write/read byte
  counts and `Err::I2C_SHORT_TRANSFER`.
- Compact `SensorSettings`, zero-I2C `validateSettings()`, cooperative
  `startApplySettingsJob()`, exact Bosch microsecond timing, rounded scheduler
  timing, checked fixed-unit conversion, chip-ID, and exhaustive public enum
  string helpers.
- Typed `BusyReason` details for every `BUSY` result.
- Native tests for each settings-write stage, transport error/count mapping,
  cancellation/timing boundaries, cache provenance, partial and ambiguous
  effects, fixed-layout contracts, and helper boundaries.
- Matching Arduino/ESP-IDF CLI start, poll, cancel, resync, and reset commands,
  plus opt-in HIL parser coverage for zero-budget exactly-once cancellation
  retrieval.
- Exact PlatformIO Core/platform pins and a configured ASan/UBSan native CI
  lane.

### Changed

- **Breaking:** injected I2C callbacks return `TransportResult` instead of
  driver `Status`. A callback is one terminal physical attempt, must not retry
  or recover the bus, and a write-read must be one combined repeated-start
  transaction without an intermediate STOP.
- **Breaking:** `Status::msg` is always library-owned canonical text derived
  from `Err`; custom message pointers passed through the legacy constructor are
  ignored. Typed code and numeric detail are the persistent diagnostic
  contract.
- A running/waiting staged job now exclusively owns hardware-facing access.
  Conflicting calls return typed `BUSY` without I2C, while `tick()` performs no
  I2C.
- `OFFLINE` is observational rather than an admission gate. Retry, recovery,
  reset, and retirement policy remain with the application I2C owner.
- `end()` is an idempotent zero-I2C unbind. Putting the sensor to sleep remains
  an explicit fallible operation.
- `conversionReadyTimeoutMs` is distinct from the per-transfer timeout, and
  cooperative health timing uses the explicit `pollJob()` / `tick()` time.
- Whole-settings apply reuses the existing apply phases. Pre-write failure or
  cancellation restores prior settings; any possible write effect retains the
  desired settings with `RESYNC_REQUIRED`.
- ESP32-S3/S2 Arduino and native ESP-IDF example adapters implement the exact
  terminal transport contract without adapter retries or recovery.

### Fixed

- Raw data, compensated data, `t_fine`, timestamp, sequence, and configuration
  generation now commit atomically only after complete validation; a failed
  refresh preserves the last-good sample byte-for-byte.
- Old-generation samples remain explicitly stale after configuration apply or
  resync and cannot become fresh under new settings.
- Partial configuration writes, diagnostic control-register writes, and
  uncertain reset/config effects preserve the original error and require a
  full verified resync.
- Forced-trigger timeouts/cancellation expose an unknown conversion state; the
  next forced job reconciles `status.measuring` before issuing another trigger.
- Synchronized steady forced sampling no longer rewrites unchanged
  `ctrl_hum`; humidity remains latched through the configuration
  `ctrl_meas` write.
- Cached calibration can be invalidated independently and is committed only
  after complete reads and validation, including erased humidity-block checks
  when humidity is enabled.
- Synchronous and staged recovery keep replacement calibration private until
  configuration also succeeds. A definite identity/calibration change that
  cannot be fully resynchronized leaves calibration invalid and blocks further
  measurement instead of restoring stale operational state.
- Accepted configuration jobs cancel older compatibility measurement tracking,
  and `tick()` performs no I2C while configuration or calibration truth is
  unresolved, preventing old-settings data from receiving a new generation.
- A successful `ctrl_hum` write followed by any failed `ctrl_meas` latch write
  now records partial hardware state even when the second failure is a definite
  address NACK.
- Conversion-grace validation reserves the maximum mutable measurement plus
  standby interval so every composed millisecond deadline remains within the
  signed wrap-safe half range.
- Pressure and humidity compensation replace negative signed shifts and
  unchecked extreme-trim intermediates with checked 64-bit operations;
  overflow returns `COMPENSATION_ERROR` without changing the last-good sample.
- Health timestamps expose validity, and persistent status snapshots cannot
  retain borrowed adapter text.

### Removed

- **Breaking:** duplicate `CalibrationRaw::h1`. Register `0xA1` is already
  available as `CalibrationRaw::tp[25]`; raw calibration now uses exactly two
  bursts rather than a redundant third transaction.
- The inert internal platform-time shim; framework-neutral time is now explicit
  through the injected or caller-provided time context.

## [1.7.0] - 2026-06-23

### Added
- `Config::nvmReadyTimeoutMs` and `SettingsSnapshot::nvmReadyTimeoutMs` for explicit NVM-ready deadline visibility.
- Chunked job APIs for integration owners that need bounded transfer budgets:
  `startInitJob()`, `startForcedMeasurementJob()`, `startApplyConfigJob()`,
  `startRecoveryJob()`, and `pollJob(nowMs, maxInstructions)`.
- `SampleFreshness`, `sampleFreshness()`, `sampleFresh(nowMs, maxAgeMs)`, and
  `SettingsSnapshot::sampleFreshness` so applications can distinguish fresh
  cached samples from stale-but-readable samples after errors or dirty hardware
  config.
- Arduino and native ESP-IDF diagnostic CLI `job` commands for staged init,
  forced measurement, config apply, recovery, single-poll, and status checks.
- HIL runner `--include-job-api`, staged-job parser validators, and
  release-gating exit flags `--require-pass` and `--fail-on-review`.
- TunnelMonitor fit report with synchronous API classification and optional ENV absence-vs-fault mapping notes.
- Native coverage for staged init/apply/forced/recovery jobs, forced-job status
  lifecycle, sample freshness, NVM-ready single-read `BUSY`/wrap-safe timeout
  behavior, chip-ID transport error preservation, calibration-invalid begin
  failures, and I2C timeout propagation.

### Changed
- Synchronous NVM readiness checks now perform one raw status read per call and return visible `BUSY`, `TIMEOUT`, or detailed transport errors instead of hiding a tight polling loop.
- `begin()`, `probe()`, and staged init preserve chip-ID transport faults; only address NACK maps to `DEVICE_NOT_FOUND`.
- Core time fallback is routed through private `PlatformTime` and remains inert; production timestamps and deadlines should come from `Config::nowMs`.
- Staged recovery now verifies chip ID after reset, can complete from `OFFLINE`
  without exposing an intermediate `READY` latch, and reasserts `OFFLINE` on
  failed recovery that started offline.
- Staged recovery now preserves the last cached raw/compensated sample after a
  failed resync and invalidates it only after successful full recovery.
- Staged forced-measurement jobs now expose the same observable
  `lastMeasurementStatus()` lifecycle as `requestMeasurement()`.

## [1.6.0] - 2026-06-02

This is the direct public successor to `v1.5.0`. It is the single release entry
for all changes accumulated since `v1.5.0`.

### Added
- Formal pre-HIL evidence reporting for the default runner sequence, captured
  command arguments, firmware/library/git/worktree metadata, and operator
  sign-off fields.
- Native tests pinning diagnostic raw-write dirty-state behavior, health-session
  counter reset semantics, no-clock NVM polling fallback, and sample-cache
  invalidation after recovery/reset.
- Production shared-bus integration guide covering application-owned I2C,
  locking, timeout, scheduling, recovery, and HIL evidence expectations.
- Focused HIL runner parser tests for complete, delayed, and truncated
  calibration/settings evidence.
- Dirty hardware-configuration diagnostics with `hardwareConfigDirty()`,
  `hardwareConfigDirtyError()`, and matching `SettingsSnapshot` fields.
- `lastMeasurementStatus()` for checking scheduler/capture failures after
  `tick()`.
- `tools/check_package_contents.py` for validating generated PlatformIO
  archives.
- `tools/run_i2c_hil.py`, `tools/check_hil_contract.py`, an HIL runbook, and a
  target evidence template for future serial hardware validation.
- Hardware validation matrix and maintained hardening summary documentation.
- ESP-IDF component metadata for building the framework-neutral core with
  `idf_component_register`.
- ESP-IDF basic example with an application-owned `i2c_master` bus/device and
  transport callbacks.
- Native ESP-IDF CLI preserving Arduino command coverage without Arduino
  compatibility facades.
- ESP-IDF example contract check covering native IDF glue, forbidden Arduino
  compatibility tokens, component metadata, and required command coverage.
- Consolidated ESP-IDF port documentation in `docs/IDF_PORT.md`, including
  component/example structure, adapter contract, validation commands, and
  remaining hardware checks.
- Per-channel validity flags on `Measurement`, `RawSample`, and
  `CompensatedSample`, plus named Bosch skipped-sentinel constants.
- Native golden-vector coverage for calibration parsing, H4/H5 nibble packing,
  raw burst reconstruction, fixed-point compensation, humidity clamps, skipped
  sentinels, and pressure denominator guarding.

### Changed
- Default HIL guidance now records forced-mode sleep-return evidence with
  post-`force` `reg 0xF4`, `status`, and `read` commands before any
  normal-mode sequence.
- Tightened Doxygen and release documentation so the managed synchronous
  lifecycle, release steps, validation commands, and hardware-validation
  boundary use the same language across README, AGENTS, and maintained docs.
- CI now runs the HIL contract checker alongside the CLI and ESP-IDF example
  contract checks without requiring physical hardware.
- Package validation now requires the ESP-IDF example transport files used by
  the packaged native IDF example.
- Package validation now checks the archive matching `library.json` and verifies
  packaged `library.json`, `idf_component.yml`, and `Version.h` agree on the
  release version.
- Diagnostic raw writes to BME280 control/config/reset registers now mark
  `hardwareConfigDirty()` and document `recover()`, `begin()`, or a successful
  `softReset()` as the resync path.
- Health counter documentation now describes counters as current health-session
  totals since the most recent `begin()`, matching existing reset behavior.
- NVM polling documentation now clarifies that the millisecond deadline requires
  an advancing `Config::nowMs`; the framework-neutral fallback is bounded by
  poll count.
- Successful `recover()` now invalidates cached samples after a complete resync
  so pre-recovery data cannot be reused accidentally.
- Doxygen inputs now include the production shared-bus guide while excluding
  local/generated artifacts and source-extraction directories.
- Release documentation now removes prompt-scoped audit, phase, and merge-gate
  reports from tracked docs; durable conclusions remain in the maintained docs
  and release notes.
- HIL runner `cfg` and cached `calib` evidence now waits for final command
  completion tokens with bounded command windows so long multi-line serial
  output is not split across command boundaries.
- HIL serial reads no longer rely only on `in_waiting`; the runner performs a
  bounded fallback read before draining available bytes.
- Core timing guard now rejects Arduino and ESP-IDF framework headers in
  core/public headers and `src/`.
- README and ESP-IDF port documentation now describe the native-IDF
  component/example flow and full Arduino/ESP-IDF CLI parity.
- `library.json` now advertises both Arduino and ESP-IDF framework support.
- Native ESP-IDF example timing now uses an explicitly named `currentMs()`
  helper instead of an Arduino-style `millis()` shim; the IDF example contract
  rejects future `millis()` / `delay()` timing regressions.
- Successful typed configuration changes now invalidate cached samples so old
  values are not reused under a new measurement configuration.
- Diagnostic CLIs now include address selection, chip-ID/register checks,
  reset/recover/selftest/stress commands, and clearer health output for HIL
  evidence.
- Public documentation was consolidated around maintained user-facing docs;
  temporary phase and prompt reports were removed from published Doxygen inputs
  or superseded by maintained docs.
- Old prompt and split ESP-IDF implementation notes were removed after their
  useful content was folded into the maintained README, IDF port note, hardening
  summary, and HIL docs.
- Generated HIL logs and Doxygen HTML output are ignored by git.
- Supporting documentation now has a maintained `docs/README.md` map, clearer
  hardware-evidence policy, and updated wording for the merged hardening work.

### Removed
- Removed the stale ESP-IDF Arduino compatibility shim and documentation that
  described compiling Arduino CLI source into IDF examples.
- Removed superseded industry-readiness intermediate reports after their useful
  content was folded into the hardening summary, README, changelog, HIL
  runbook, and hardware matrix.

### Migration Notes
- Users upgrading from `v1.5.0` must check `temperatureValid`,
  `pressureValid`, and `humidityValid` before using measurement fields.
  Skipped or invalid channels leave numeric fields at zero.
- `Measurement`, `RawSample`, `CompensatedSample`, and `SettingsSnapshot` have
  changed public layout since `v1.5.0`; rebuild dependent firmware and avoid
  assuming binary compatibility across versions.
- `BME280::BME280` instances are intentionally non-copyable and non-movable.
  Keep a single owned instance and pass references or pointers.
- Use typed setters for normal configuration. `writeRegister()` and
  `writeRegisters()` are diagnostic raw access; writes to reset/control/config
  registers mark dirty state and require `recover()`, `begin()`, or a
  successful `softReset()` to resync.
- After successful `recover()` or any `softReset()` attempt, request a fresh
  measurement before using cached sample data.
- Named Bosch skipped-sentinel constants describe skipped raw channels; callers
  should use the validity flags rather than treating all-zero numeric outputs as
  valid readings.
- PlatformIO Arduino builds do not imply local pure ESP-IDF `idf.py` validation.
  Record exact `idf.py` command results before claiming local pure ESP-IDF
  builds.

### Validation Boundary
- Software checks, metadata synchronization, package validation, and HIL
  contract checks were run for this release. No physical BME280 HIL,
  environmental accuracy validation, bench fault validation, or long-soak
  hardware validation is claimed by this release.

## [1.5.0] - 2026-05-14

### Added
- `SettingsSnapshot` struct for reading cached configuration and runtime state without I2C.
- `getSettings(SettingsSnapshot&)` method to populate a settings snapshot.
- `Status::is(Err)` method for type-safe error code comparison.
- `Status::operator bool()` explicit conversion for concise success checks.
- Native tests for forced-mode on-demand behavior, normal-mode fresh-cycle gating, invalid oversampling combinations, and chip-ID mismatch recovery health.
- Native coverage proving latched `OFFLINE` blocks normal I2C operations without touching the bus while `recover()` remains the explicit recovery path.

### Changed
- Doxyfile project metadata now matches `library.json`, and archived prompt
  metadata no longer contains placeholder ownership values.
- Explicit recovery/reset bypass internals now use the shared `ScopedOfflineI2cAllowance` / `_reassertOfflineLatch()` procedure so failed recovery attempts that begin from `OFFLINE` keep the latch asserted.
- Forced mode is now treated as an on-demand policy: `begin()` and `setMode(FORCED)` leave the hardware in sleep until `requestMeasurement()` triggers a conversion.
- Normal-mode `requestMeasurement()` now waits one estimated normal cycle before `tick()` reads data registers, avoiding stale immediate samples.
- README API docs now include measurement, configuration, calibration, status, and oversampling constraints.
- Reference documentation now uses human-readable vendor PDF names and separates compact chip notes from full PDF extractions under `docs/extracted-md/` and `docs/pdf-extracted-md/`.
- Health behavior is now standardized on latched `OFFLINE`: normal public I2C operations return `BUSY` with `Driver is offline; call recover()` and do not touch I2C until `recover()` succeeds.

### Fixed
- Bring-up CLI stress runs now clear stale pending measurement state before
  restarting, and `stress_mix` no longer counts expected sleep-mode measurement
  attempts as failures.
- Stress progress output now leaves the progress percentage uncolored and only
  colorizes the ok/fail counters.
- Invalid oversampling combinations are rejected in `begin()` and typed setters before I2C, instead of failing later during compensation inside `tick()`.
- `recover()` now records a chip-ID mismatch as a health failure instead of leaving the driver marked healthy after a failed recovery.
- Example diagnostic error strings now include granular `I2C_*` status codes.

## [1.4.0] - 2026-04-05

### Added
- Public lifecycle/config introspection helpers: `isInitialized()` and `getConfig()`.
- Public tracked raw-register helpers: `readRegisters()`, `writeRegisters()`, `readRegister()`, and `writeRegister()`.
- `Err::CONVERSION_NOT_READY` alias for cross-library uniformity.

### Changed
- Health tracking now treats `IN_PROGRESS` as non-failure activity and keeps pre-`begin()` validation errors from forcing state transitions.
- Bringup CLI now exposes `reg` / `wreg` register diagnostics for tracked low-level access.

## [1.3.0] - 2026-04-03

### Added
- Granular I2C transport error codes: `I2C_NACK_ADDR`, `I2C_NACK_DATA`, `I2C_TIMEOUT`, and `I2C_BUS`.
- Native tests covering the `millis()` fallback and example transport error mapping.

### Changed
- `examples/common/I2cTransport.h` now uses `TwoWire*` from `Config::i2cUser` and treats per-call `timeoutMs` as advisory.
- README quick start, transport notes, and documentation section now match the example adapter and shipped datasheet/register notes.

### Fixed
- `BME280::_nowMs()` now falls back to `millis()` when `Config::nowMs` is not injected, matching the documented behavior.

## [1.2.2] - 2026-04-03

### Added
- `inProgress()` convenience method on `Status` struct.
- `CommandHandler.h` example helper for serial command parsing (`cmd::readLine`, `cmd::match`, `cmd::parseInt`).
- `HealthDiag.h` example helper with verbose health diagnostics, color-coded output, snapshots, diffs, and `HealthMonitor` class for continuous monitoring.

### Changed
- `I2cScanner.h` upgraded: advanced table-format scan, bus recovery via `recoverBus()`, timeout support, yield() calls, common address hints, `LOG_SERIAL` macro usage.
- `I2cTransport.h` upgraded: `TwoWire*` via user pointer (no global Wire), null pointer checks, 128-byte buffer validation, detailed per-error-code mapping, `ARDUINO_ARCH_ESP32` guards.
- `BusDiag.h` updated to use `i2c_scanner::scan(Wire)` and include `<Wire.h>`.
- `Log.h`: added `LOGV` runtime-verbose macro, ESP32-S3 USB CDC delay in `log_begin()`, fixed include path.
- `BoardConfig.h`: fixed include path for `I2cTransport.h`.
- `main.cpp` example updated to use `bus_diag::scan()`, set `cfg.i2cUser = &Wire`, corrected include paths.

## [1.2.1] - 2026-03-01

### Changed
- Quick start and bringup example now set `Config.nowMs` explicitly for portable timing/health timestamps.
- `docs/IDF_PORT.md` and timing-guard policy now reflect zero direct Arduino timing calls in library core.

### Fixed
- Restored injected time source in core driver (`_nowMs()`); removed direct `millis()` usage from library internals.
- Forced-mode `requestMeasurement()` now tracks an already-running conversion as `IN_PROGRESS` instead of returning `BUSY`, preventing avoidable measurement-cycle timeouts.
- `getRawSample()` and `getCompensatedSample()` now return the latest cached sample even after `getMeasurement()` consumes the ready flag.

## [1.2.0] - 2026-03-01

### Changed
- Updated `docs/IDF_PORT.md` to reflect the actual timing abstraction and portability flow.

### Fixed
- Core timing guard compliance updates in `src/BME280.cpp`.

### Removed
- Stale auxiliary documentation templates not used by the current release flow.

## [1.1.1] - 2026-02-28

### Added
- Unified bringup helper layer under `examples/common/*` (`BusDiag`, `CliShell`, `HealthView`, `TransportAdapter`).
- `docs/IDF_PORT.md` and `docs/UNIFICATION_STANDARD.md` as baseline portability/unification references. `docs/UNIFICATION_STANDARD.md` was later superseded by the maintained README, AGENTS, and IDF/HIL docs.
- CLI/timing contract validation scripts in `tools/`.

### Changed
- `examples/01_basic_bringup_cli` aligned to the shared I2C CLI command/help/reporting scheme.
- CI and native test layout standardized (`test/test_basic.cpp`, workflow/profile normalization).

## [1.1.0] - 2026-02-22

### Added
- `getStandbyTimeMs()` - returns configured standby interval in milliseconds (rounded up)
- `estimateNormalCycleMs()` - returns full normal-mode cycle time (measurement + standby)

### Fixed
- **`tick()` used `millis()` directly** - broke determinism; now uses only the caller-supplied `nowMs` parameter
- **`softReset()` polling used tracked reads** - during POR (~2 ms) the BME280 may NACK, which inflated health-failure counters and could abort reset prematurely; now uses raw reads and tolerates transient I2C errors
- **`setFilter()`/`setStandby()` masked original error** - if config-register write failed and restore-to-original-mode also failed, the restore error was returned instead of the root cause; restore is now best-effort
- **`end()` didn't put device to sleep** - device continued measuring in normal mode after shutdown; now sends best-effort sleep command via raw I2C before clearing state
- **`_compensate()` didn't handle skipped channels** - running compensation on sentinel ADC values when `osrsT/P/H == SKIP` produced garbage; now guards each channel and returns `COMPENSATION_ERROR` if temperature is skipped while P/H are enabled
- **`recover()` didn't re-apply configuration** - after a power glitch, device registers revert to defaults; `recover()` now calls `_applyConfig()` after successful probe

## [1.0.0] - 2026-01-20

### Added
- **First stable release**
- Complete BME280 driver with Bosch compensation formulas (32-bit/64-bit)
- Injected I2C transport architecture (no Wire dependency in library)
- Health monitoring with automatic state tracking (READY/DEGRADED/OFFLINE)
- Configurable oversampling (SKIP, X1, X2, X4, X8, X16) for T/P/H
- Configurable IIR filter coefficient (OFF, X2, X4, X8, X16)
- Configurable standby time for normal mode (0.5ms to 1000ms)
- Support for all measurement modes: Sleep, Forced, Normal
- Non-blocking tick-based architecture for async operations
- Soft reset with proper timeout handling
- Calibration data validation
- Raw and compensated sample access
- Measurement time estimation
- Register-level read/write access for diagnostics
- Basic CLI example (`01_basic_bringup_cli`)
- Comprehensive Doxygen documentation in public headers
- MIT License

## [0.1.0] - 2026-01-19

### Added
- Initial development version
- Production BME280 driver with injected I2C transport
- Health monitoring and tracked transport wrappers
- Basic CLI example (`01_basic_bringup_cli`)
- Doxygen-style documentation in public headers

[Unreleased]: https://github.com/janhavelka/BME280/compare/v2.0.0...HEAD
[2.0.0]: https://github.com/janhavelka/BME280/compare/v1.7.0...v2.0.0
[1.7.0]: https://github.com/janhavelka/BME280/compare/v1.6.0...v1.7.0
[1.6.0]: https://github.com/janhavelka/BME280/compare/v1.5.0...v1.6.0
[1.5.0]: https://github.com/janhavelka/BME280/compare/v1.4.0...v1.5.0
[1.4.0]: https://github.com/janhavelka/BME280/compare/v1.3.0...v1.4.0
[1.3.0]: https://github.com/janhavelka/BME280/compare/v1.2.2...v1.3.0
[1.2.2]: https://github.com/janhavelka/BME280/compare/v1.2.1...v1.2.2
[1.2.1]: https://github.com/janhavelka/BME280/compare/v1.2.0...v1.2.1
[1.2.0]: https://github.com/janhavelka/BME280/compare/v1.1.1...v1.2.0
[1.1.1]: https://github.com/janhavelka/BME280/compare/v1.1.0...v1.1.1
[1.1.0]: https://github.com/janhavelka/BME280/compare/v1.0.0...v1.1.0
[1.0.0]: https://github.com/janhavelka/BME280/releases/tag/v1.0.0
[0.1.0]: https://github.com/janhavelka/BME280/releases/tag/v0.1.0
