# BME280 Industry Hardening Summary

Last updated: 2026-07-19

This document is the maintained summary for the merged industry-readiness work.
It replaces the temporary prompt, phase, merge-gate, and self-test records that
were created while that work was being built.

## Scope

The hardening work prepares the BME280 library for production-style embedded
use on ESP32-S2 and ESP32-S3 with Arduino/PlatformIO and ESP-IDF consumers.

No physical BME280 hardware validation is claimed here. Local ESP-IDF `idf.py`
validation is claimed only when the exact commands are run and recorded.

Release scope: `v1.7.0` is the direct public successor to `v1.6.0`. The
release notes for `v1.7.0` contain the staged recovery closure, sample
freshness API, job CLI/HIL coverage, and release-gating runner flags.

## Active Documentation Set

- `README.md`: public usage, API, build, validation, and limitations.
- `CHANGELOG.md`: release-facing change history.
- `AGENTS.md`: repository engineering rules.
- `CONTRIBUTING.md`: contribution workflow.
- `docs/README.md`: map of maintained docs, source evidence, and local
  artifacts.
- `docs/IDF_PORT.md`: ESP-IDF component/example boundary and validation notes.
- `docs/BME280_Register_Reference.md`: register and bitfield reference.
- `docs/PRODUCTION_SHARED_BUS_GUIDE.md`: production shared-bus integration
  guidance.
- `docs/BME280_HARDWARE_VALIDATION_MATRIX.md`: hardware result ledger.
- `docs/I2C_HIL_RUNBOOK.md`: serial HIL procedure.
- `docs/I2C_HIL_TARGET_TEMPLATE.md`: per-target HIL evidence template.
- `docs/BME280_datasheet.pdf` and extracted markdown: source evidence.

Prompt-scoped audit reports are not shipped as user-facing release
documentation. Their durable conclusions are folded into this summary, the
maintained docs above, and the current release entry in `CHANGELOG.md`.

## What Changed

Core and API contracts:

- The core stays framework-neutral: no Arduino, Wire, ESP-IDF, FreeRTOS, log, or
  delay APIs in `include/` or `src/`.
- I2C remains injected through `Config`; the library does not own pins, buses,
  locks, or timeout policy.
- Driver objects are non-copyable and non-movable.
- Public error reporting keeps distinguishable transport errors when the
  adapter can provide them.
- Transport callbacks return terminal-only `TransportResult`, require exact
  byte counts and one physical attempt, and forbid adapter retry/recovery.
  Driver `Status` messages are canonical library strings derived from typed
  codes; adapter-owned text is never retained.
- Measurement scheduling now requires an injected timebase for the synchronous
  compatibility path. Cooperative `pollJob(nowMs, ...)` and `tick(nowMs)` use
  the explicit timestamp for chip phases and health events in that call;
  timestamp-validity flags distinguish real time from the inert zero fallback.
- `OFFLINE` is an observational health threshold. It no longer overrides an
  explicit application-owned retry, resync, reset, or retirement decision.

Cooperative job ownership:

- Init, forced measurement, config apply, non-reset resync, and explicit soft
  reset have zero-I2C start operations and one fixed-memory state machine.
- An accepted job has a nonzero `jobId`. `JobPollResult` exposes public
  `JobPhase`, chip-phase deadline state, callback use, terminal status, and
  `ConversionState`.
- `pollJob(nowMs, budget)` issues no more than `budget` callbacks. A running or
  waiting job exclusively owns hardware access: fallible conflicting calls
  return `BUSY`, while `tick()` performs no I2C.
- Natural completion/failure is returned by the exact poll that reaches it.
  Zero-I2C cancellation is retained for exactly one later poll; until retrieval,
  terminal-pending state blocks later hardware work.
- Application deadlines remain external and include queue time. An owner calls
  `cancelJob(OWNER_REQUEST)` or `cancelJob(DEADLINE_EXPIRED)` without I2C;
  `phaseDeadlineMs` never replaces or renews the owner deadline.
- `end()` is an idempotent zero-I2C unbind. Sensor sleep, if required, must be a
  separate explicit fallible hardware operation.

Measurement and compensation:

- Calibration parsing, raw burst reconstruction, Bosch-style compensation, and
  humidity clamp behavior have native test coverage.
- Raw and compensated sample structs expose per-channel validity flags.
- Bosch skipped-channel sentinel constants are named in the public command
  table.
- Configuration changes invalidate cached samples or retain them explicitly as
  stale under an older generation so callers do not reuse them as current data.
- Candidate raw data, compensated data, `t_fine`, timestamp, sample sequence,
  and configuration generation commit atomically; failed refreshes preserve the
  previous envelope byte-for-byte.
- Forced conversion knowledge is explicit. Ambiguous trigger failure or
  post-trigger cancellation becomes `UNKNOWN_AFTER_TRIGGER_ERROR`; the next
  staged forced job reconciles `status.measuring` before issuing a new trigger.
  The ambiguous trigger is never replayed, and steady forced sampling does not
  rewrite an unchanged `ctrl_hum` setting.

Configuration, reset, and recovery:

- Multi-register config paths expose `hardwareConfigDirty()` and the original
  dirty-state error when hardware and cache may diverge.
- Diagnostic raw writes to `ctrl_hum`, `ctrl_meas`, `config`, or `reset` mark
  dirty state so service commands cannot silently desynchronize the typed
  config cache.
- Dirty state is cleared only after a complete successful config resync through
  `begin()`, `recover()`, `softReset()`, or the matching staged operation.
- Synchronous `recover()` and staged `startResyncJob()` resynchronize identity,
  NVM readiness, calibration, and config without resetting the sensor. Legacy
  `startRecoveryJob()` is a non-reset alias. `startSoftResetJob()` is the
  separate explicit reset-plus-resync operation.
- Synchronous successful recovery/reset invalidates cached samples. Staged
  config/resync completion advances the configuration generation, preserving
  any last-good sample only as stale diagnostic data.
- `setFilter()` and `setStandby()` avoid config writes while the sensor reports
  `measuring`.
- `SensorSettings` and `startApplySettingsJob()` provide one zero-I2C admission
  point for coherent whole-settings changes while reusing the existing apply
  phases. Pre-write failure restores the prior snapshot; possible partial
  effects retain desired settings with an explicit resync requirement.
- Synchronous reset/recover NVM readiness checks perform one status read and
  return visible `BUSY`, `TIMEOUT`, or the original transport error. Repeated
  bounded NVM polling is available through staged jobs advanced by `pollJob()`.
- `Config::conversionReadyTimeoutMs` is separate from the per-callback
  `i2cTimeoutMs`. All wrap-safe timeout fields reject zero and values above
  `INT32_MAX`.
- Pure helpers cover settings validation, exact Bosch microsecond timing,
  rounded scheduler timing, chip identity, and checked fixed-unit conversion.
  `CalibrationRaw` exposes the complete image through two bursts without a
  duplicate H1 field or read.
- The library exposes no writable calibration NVM, trimming, or factory-
  programming API. It only performs bounded waits for the BME280's internal NVM
  copy after POR/reset and reads the resulting calibration registers.

Examples, ESP-IDF, and CI:

- The Arduino bring-up CLI and native ESP-IDF CLI share the same command
  contract without sharing Arduino source in IDF builds.
- Both CLIs keep matching staged-job diagnostics for status, init, force,
  apply, non-reset resync, explicit reset, cancel, and poll, including job
  identity, phase/deadline, callback use, terminal state, and conversion state.
- The ESP-IDF example uses `app_main`, `driver/i2c_master.h`, `esp_timer`, and
  fixed C command buffers.
- Guard scripts check core framework neutrality, CLI parity, IDF example
  boundaries, HIL runner/docs consistency, generated version state, and package
  contents.
- CI is configured for PlatformIO Arduino builds, native tests, an ASan/UBSan
  native lane, package checks, and ESP-IDF example builds. A configured lane is
  not runtime evidence until the corresponding workflow succeeds.

HIL preparation:

- `docs/BME280_HARDWARE_VALIDATION_MATRIX.md` is the hardware result ledger.
- `docs/I2C_HIL_RUNBOOK.md` describes the serial HIL procedure.
- `docs/I2C_HIL_TARGET_TEMPLATE.md` captures per-target setup and evidence.
- `tools/run_i2c_hil.py` captures grouped serial transcripts, summaries,
  command plans, environment records, CSV/JSON results, and artifact manifests;
  `--include-job-api` adds staged-job HIL coverage, while `--require-pass` and
  `--fail-on-review` provide release-gating exit modes.

## Deterministic Operation Bounds

Zero-I2C operations include staged starts, cancellation, `end()`, and cached
snapshots. Synchronous compatibility operations use fixed transaction shapes,
and every transport callback receives `Config::i2cTimeoutMs`. Cooperative work
is bounded per call by the `uint8_t` callback budget.

NVM readiness permits at most 255 status callbacks. A measuring/idle phase uses
a 255-poll counter and may make one final status callback before reporting its
poll-limit timeout. With no earlier chip-phase deadline or transport error, the
cumulative staged callback caps are 518 for init or non-reset resync, 519 for
explicit soft reset, 516 for config apply, 258 for a known-idle forced job, and
514 for a forced job that first reconciles an ambiguous trigger. These counts do
not define an overall owner deadline or elapsed-time guarantee; the application
owns queueing, poll cadence, bus policy, and its original operation deadline.

## Validation Boundary

Software checks can show that the driver builds, tests pass, examples compile,
and docs/tool contracts are coherent. They do not prove wiring, pull-up values,
bus margin, sensor accuracy, humidity handling, long soak behavior, or physical
fault recovery.

Before any hardware row is changed from `NOT RUN`, record board, sensor module,
wiring, address, pull-ups, voltage rails, command transcript, environmental
reference, result, and operator notes in the hardware validation matrix or an
attached HIL artifact package.

Generated Doxygen output is a local artifact. Publish API docs only from a
release commit where the documented public API matches `library.json` and
`include/BME280/Version.h`.

## Current HIL Baseline

The intended first HIL software baseline is the commit used to build the test
firmware. Record the exact `git rev-parse HEAD` value in the HIL target
template before flashing.

`scan` is only I2C ACK evidence. Use `chipid` or `reg 0xD0` reading `0x60` as
the BME280 identity check.

The CLI does not support counted read commands such as `read 10` or `read 20`.
Use repeated `read` commands for normal-mode evidence and `stress N` only as a
forced-measurement stress substitute.

Default HIL evidence captures `force`, then `reg 0xF4`, `status`, and `read`.
For forced-mode sleep-return evidence, `ctrl_meas` mode bits `[1:0]` must read
`00` after the forced conversion completes.

Normal-mode soak evidence is separate from forced `stress N`; use repeated
`read` commands in normal mode and record reference/timestamp notes before
claiming soak coverage.

## Release Gate

- Merge status: the hardening work is present in the maintained release
  history. Check the current branch and release tag before publishing new
  evidence.
- Release status: version metadata, changelog entries, generated version
  header, and Doxygen project metadata must be aligned on the exact release
  commit before tagging. Publishing still requires the pushed tag and
  successful CI for that commit.
- Hardware status: not complete until `BME280_HARDWARE_VALIDATION_MATRIX.md` or
  an attached HIL artifact package records real board, wiring, transcript,
  reference, fault, and soak evidence. The software release does not claim
  completed hardware validation.
