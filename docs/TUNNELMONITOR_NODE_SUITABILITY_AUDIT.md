# TunnelMonitor-node suitability audit

## BME280 environmental sensor library

Date: 2026-07-18

Audit result: **strong base, focused refactor required before integration**

BME280 v1.7.0 is a substantially better integration candidate than the v1.6.0
revision previously evaluated by TunnelMonitor. It already has framework-neutral
transport injection, fixed memory, strong compensation code, and cooperative
initialization, forced-measurement, configuration, and recovery jobs. Its
`pollJob(nowMs, 1)` path provides the main scheduling primitive that
TunnelMonitor needs.

It should not be integrated unchanged. The cooperative path can corrupt the
last-good cached sample when a new compensation attempt fails, can make an old
sample appear fresh after configuration resync, has no deadline cancellation,
and does not prevent synchronous hardware APIs from running during an active
job. The mandatory library `OFFLINE` latch also conflicts with `I2cTask` as the
sole health and recovery owner.

The recommended path is to refactor the existing staged core, release and pin a
new immutable revision, then replace the direct BME280 protocol in `I2cTask`
with one narrow owner-private adapter. Do not rewrite the Bosch protocol in the
firmware, and do not hide the current gaps in adapter checks.

## Implementation re-audit baseline - 2026-07-19

This implementation pass started from the following exact repository state:

| Repository | Starting revision and worktree |
| --- | --- |
| BME280 | `0aabd587e088117142f2b7e7c3b7b90ee4a778a3`; clean working tree; branch `hardening/tunnelmonitor-suitability-reaudit` created for this work |
| TunnelMonitor-node | `602114ea6c723e31c41f0eb7cd8ac2b56a46d40e`; branch `prompt-44b-sequence`; preserved modified `.vscode/extensions.json` and untracked `docs/reports/i2c_library_latest_branch_audit_revalidation_20260718.md` |

After the repository fetch, the TunnelMonitor remote branch advanced beyond
the audited revision to `3317b5f`. The local checkout was intentionally left at
`602114ea` so this pass continued against the requested immutable baseline and
did not disturb its dirty working tree.

The lead ran these software baseline checks before implementation:

| Check | Baseline result |
| --- | --- |
| Native suite | PASS, 136/136 |
| Core timing, CLI, HIL contract, and ESP-IDF example guards | PASS |
| Release metadata | PASS, version `1.7.0` |
| HIL parser | PASS |
| PlatformIO ESP32-S3 build | PASS, 22,584 bytes RAM and 394,318 bytes flash |
| PlatformIO ESP32-S2 build | PASS, 37,000 bytes RAM and 383,705 bytes flash |
| Doxygen | PASS with Doxygen 1.15 |
| Local ESP-IDF build | NOT RUN; `idf.py` was unavailable |
| New physical HIL | NOT RUN |

The source re-audit retained the following pre-implementation dispositions:

| Findings | 2026-07-19 disposition |
| --- | --- |
| H-01 through H-11 | Confirmed; pending implementation and focused regression tests |
| S-01 | Confirmed; configuration/conversion timing remains coupled to the transport timeout; pending implementation |
| S-02 | Confirmed; cooperative operation and health timestamps still use two time inputs; pending implementation |
| S-03 | Partially confirmed; retain the deliberately narrow coefficient checks and add the scoped erased humidity-block check |
| S-04 | Confirmed; PlatformIO core and build-platform resolution remain broadly specified; pending exact pinning |

This table is a baseline disposition, not a completion claim. Final status must
be recorded only after the corresponding code, tests, documentation, and
release checks have completed.

### Implementation progress

The first core-state chunk resolves H-01, H-02, H-10, and the scoped S-03
humidity check. Raw decoding and compensation now use candidate storage and
commit one `SampleEnvelope` only after complete validation. Typed configuration
and calibration state, configuration generations, sample sequences, and
zero-I2C device-state invalidation prevent uncertain settings or coefficients
from being used for a new measurement. A successful staged resynchronization
preserves the prior sample only as generation-stale diagnostic evidence.

The focused native regressions cover last-good preservation after raw-sentinel
and pressure-divisor failures, zero-I2C `RESYNC_REQUIRED` admission, generation
and sequence provenance, invalidation/reload behavior, and erased humidity
calibration with humidity-enabled and humidity-skipped configurations. The
native suite passed 142/142 after this chunk. Remaining findings stay open
until their later implementation chunks and final full validation.

S-04 is also resolved in build policy: PlatformIO Core is pinned to 6.1.19 in
CI, the ESP32 platform is pinned to the qualified pioarduino 54.03.20 release,
and the native platform is pinned to 1.2.1. With those pins active, the native
suite passed 142/142 and both ESP32-S3 and ESP32-S2 PlatformIO builds passed.
This is build evidence only; it does not add local ESP-IDF or hardware-runtime
evidence.

The ownership and lifecycle chunk resolves H-03, H-05, and H-06. A staged job
now has exclusive hardware access: every synchronous public operation that can
reach the transport returns `BUSY` without a callback while a job is running or
waiting. Health states and counters remain observable, but `OFFLINE` no longer
overrides an explicit owner-directed transaction or recovery policy. `end()` is
now an idempotent, zero-I2C unbind that cancels local work and clears callbacks,
cached device state, samples, and health history; putting the sensor to sleep is
an explicit fallible hardware operation, not hidden teardown behavior. Native
regressions cover the hardware-API admission matrix, passive OFFLINE history,
zero-I2C repeated teardown, and transport rebinding. The native suite passed
143/143, and the core-timing, CLI, HIL-contract, and ESP-IDF-example guards also
passed after this chunk.

The cooperative control/timing chunk has the following implementation
disposition. This is source-and-focused-regression progress, not final release,
ESP-IDF, TunnelMonitor, or hardware validation:

| Finding | 2026-07-19 implementation disposition |
| --- | --- |
| H-03 | Resolved in the library core: a running/waiting staged job exclusively owns hardware-facing entry points; conflicting fallible calls return typed `BUSY`, and `tick()` performs no I2C. |
| H-04 | Resolved in the library core: every accepted job has a nonzero identity and public phase/progress data; zero-I2C cancellation records owner-request or external-deadline reason. Natural terminal results are returned by the completing poll, while cancellation is retained for exactly one poll and blocks later hardware until retrieved. |
| H-05 | Resolved in the library core: `OFFLINE` remains observable but no longer gates an explicit owner-directed operation. |
| H-06 | Resolved in the library core: `end()` is idempotent zero-I2C unbind/teardown. |
| H-07 | Resolved in the library and shipped adapters: callbacks return terminal-only `TransportResult`, success requires exact byte counts, write-read requires one combined repeated-start transaction, and callbacks are contractually one physical attempt with no retry or bus recovery. Definite address/data NACK, timeout, bus, short-transfer, and other failures remain distinct. |
| H-08 | Resolved in the library core: `startResyncJob()` and legacy `startRecoveryJob()` perform non-reset resync; `startSoftResetJob()` is the separate explicit reset operation. |
| H-09 | Resolved in the library core: `ConversionState` represents trigger ambiguity, cancellation preserves it when a trigger may have reached the chip, and the next forced job reconciles status before issuing another trigger. Steady forced sampling no longer rewrites `ctrl_hum`. |
| S-01 | Resolved in configuration: `conversionReadyTimeoutMs` is separate from per-transfer `i2cTimeoutMs`; short wrap-safe timeouts are range-checked. |
| S-02 | Resolved in cooperative timing: `pollJob(nowMs, ...)` and `tick(nowMs)` provide the time context for their chip phases and health updates; synchronous calls use the optional hook, and validity flags identify unavailable timestamps. |

The job result now distinguishes its internal chip-phase deadline from the
application's original end-to-end deadline. TunnelMonitor would still have to
retain the latter across queueing and polling and call
`cancelJob(DEADLINE_EXPIRED)` when it expires. Per-poll callback budgets and
fixed NVM/measuring poll caps bound library work, but do not establish or renew
an owner deadline.

H-11 is resolved in the library core. Persistent `Status` values retain only
typed code and numeric detail; construction, copying, and assignment derive
`msg` from the library-owned exhaustive `toString(Err)`. Transport adapters can
no longer return a driver `Status` or inject borrowed text. Focused tests cover
temporary-message mutation across direct copies and persistent driver, job, and
snapshot fields.

After the H-07/H-11 transport checkpoint, the native suite passed 158/158; the
core-timing, CLI, HIL-contract, and ESP-IDF-example guards passed; Doxygen 1.15
completed without warnings; and the pinned ESP32-S3/S2 Arduino builds passed at
22,696/37,096 bytes RAM and 394,662/383,633 bytes flash respectively. Final
integration review, release qualification, local ESP-IDF validation,
TunnelMonitor adapter validation, and physical HIL remain separate gates. These
are software checks only, not hardware or local ESP-IDF evidence.

TunnelMonitor source changes are not authorized by its current architecture
authority. `docs/guidelines/dependency_policy.md:32-41,114`,
`docs/guidelines/i2c_peripherals.md:477-486`, and
`docs/guidelines/decisions.md:107` in TunnelMonitor-node still defer the BME280
library and retain direct owner-private ENV protocol as the implemented
baseline. A later scoped TunnelMonitor decision must authorize the integration
and select an exact immutable BME280 release before firmware source,
configuration, dependency, or test changes are made. This does not block the
general-purpose library hardening in this repository.

## Audit basis

The audit used these exact revisions:

| Repository | Revision | Notes |
| --- | --- | --- |
| TunnelMonitor-node | `fff99fe17e60b9287ec4d8d3eca5b3230ae44223` | Branch `prompt-44b-sequence`; current direct BME280 and runtime ENV selection path |
| BME280 | `bd393e428acc78229574713105cca7edb1a3cdd2` | `main`, `origin/main`, and tag `v1.7.0` all resolved to this commit during the audit |

Unless stated otherwise, BME280 source references below mean v1.7.0 at the
commit above. TunnelMonitor references mean the revision above.

Primary device facts were checked against `docs/BME280_datasheet.pdf`, including
visual review of its mode, status, register, I2C wiring, and measurement-time
pages. The bundled PDF identifies Bosch document `BST-BME280-DS001-23`.

The existing `docs/TUNNELMONITOR_FIT_REPORT.md` is a useful earlier API
classification, but its scope is narrow and it does not cover the current
staged-state defects found here. It is historical evidence, not the current
integration decision.

This audit changed no firmware or library source, selected no production
dependency, and ran no new physical hardware tests.

## Latest branch revalidation

Revalidated after `git fetch origin --prune --tags` on 2026-07-18:

- GitHub reports `main` as the remote default branch.
- `origin/main@bd393e42` is the newest remote branch tip by commit date. The
  next newest branch is
  `origin/hardening/bme280-industry-gap-closure@c7266fc` from 2026-06-02.
- The local checkout was already `main@bd393e42`, exactly aligned with
  `origin/main` with ahead/behind `0/0`. Only this audit report was untracked.
- The audited revision and final checkout are identical, but the staged job
  runner, synchronous APIs, cache commit path, recovery, transport wrappers,
  tests, and build metadata were re-read at the final HEAD. This was not only a
  commit-label comparison.

| Finding | Recheck against final `main` |
| --- | --- |
| H-01 | Confirmed: staged data read and compensation still mutate the committed raw, compensated, and `tFine` cache before complete validation. |
| H-02 | Confirmed: forced-job admission still ignores dirty hardware configuration, while successful apply clears dirty state without retagging the old sample. |
| H-03 | Confirmed: `_jobActive()` does not gate the synchronous setters, reset, register access, measurement, probe/recovery, or `end()` APIs. |
| H-04 | Confirmed: the staged API still has no zero-I2C cancel/abort operation. |
| H-05 | Confirmed: `offlineThreshold` and `OFFLINE` transport admission remain core policy; health still updates per callback. |
| H-06 | Confirmed: `end()` still performs a best-effort sleep write, discards its status, then clears runtime state. |
| H-07 | Confirmed: callbacks still return general `Status`; exact lengths, terminal-only results, repeated-start, and no-hidden-retry semantics remain unspecified. |
| H-08 | Confirmed: synchronous `recover()` resynchronizes without reset, while the only staged recovery starts with a soft reset. |
| H-09 | Confirmed: a failed forced-trigger write has no explicit unknown-conversion state and a later job can issue another trigger. |
| H-10 | Confirmed: calibration remains cached without an explicit owner-callable invalidation contract for runtime replacement. |
| H-11 | Confirmed: persistent status fields still copy the transport-supplied `const char* msg`. |

Secondary findings S-01 through S-04 also remain current: conversion grace is
still coupled to `i2cTimeoutMs`, job and health timestamps still use two time
inputs, calibration validation remains deliberately narrow, and PlatformIO
tool resolution remains broad.

The native suite was re-run on the final HEAD: 136 of 136 tests passed.

## Decision summary

### Use after a focused refactor

The following are release gates for a TunnelMonitor integration:

1. Make sample refresh atomic. A failed new measurement must leave the exact
   previous raw and compensated sample unchanged.
2. Add explicit hardware-configuration synchronization state. Do not allow a
   measurement while configuration is uncertain, and never reclassify a sample
   captured under old settings as fresh.
3. Give an active staged job exclusive hardware access. While a job is running,
   only poll, cancel, and cached diagnostics may be used.
4. Add zero-I2C job cancellation that preserves the last-good sample and marks
   partially updated hardware state as requiring resync.
5. Remove mandatory `OFFLINE` admission gating from the passive core.
   TunnelMonitor already owns optional-device health, recovery, and retry
   policy.
6. Make `end()`/unbind zero-I2C. If sleep is needed, expose it as an explicit
   cooperative operation with a result.
7. Separate cooperative resync from explicit soft reset. A recovered bus does
   not by itself justify resetting the sensor.
8. Tighten the transport contract to terminal exact-transfer results. The
   library must not retry or recover the bus, and mutating owner callbacks must
   be one physical attempt.
9. Define calibration invalidation for runtime removal/replacement. Do not use
   coefficients from a sensor that may no longer be the installed device.
10. Exact-pin the refactored library, qualify the private adapter in native
    tests, and run TunnelMonitor-specific ESP32-S3 shared-bus HIL on the actual
    module and enclosure.

### Do not solve this with adapter band-aids

These are not acceptable long-term fixes:

- checking `sampleFreshness()` in one adapter call while leaving the cache
  internally half-updated;
- reconstructing the driver object to cancel a timed-out job;
- calling `end()` to clear an operation after its owner deadline;
- setting `offlineThreshold` to a very high value;
- calling synchronous `begin()`, `recover()`, `softReset()`, `tick()`, or
  `requestMeasurement()` from the owner poll;
- automatically soft-resetting the chip after every shared-bus fault;
- retrying an ambiguous forced-trigger write;
- keeping calibration forever without invalidating it after disappearance;
- letting the BME280 library choose between BME280 and SHT3x; or
- exposing BME280 types in TunnelMonitor public contracts.

## TunnelMonitor requirements

The library must fit the existing owner model. The firmware should not weaken
that model to fit a device driver.

| Requirement | Current authority or evidence | Consequence for BME280 |
| --- | --- | --- |
| One I2C owner | `docs/guidelines/i2c_peripherals.md:28-35`; `docs/guidelines/ownership.md:48` | Only `I2cTask` may call transport. The library owns no bus, task, lock, queue, recovery, retry, presence, health, schedule, or logging. |
| Cooperative work | `docs/guidelines/i2c_peripherals.md:100-133` | One normal library callback per owner poll. Blocking convenience APIs are excluded from the TunnelMonitor adapter. |
| Fixed timing | `include/TunnelMonitor/i2c/I2cConfig.h:60-68`; `include/TunnelMonitor/contracts/EnvPowerDisplay.h:50-54` | Per callback timeout is 20 ms. The full `ReadEnv` deadline is 1000 ms from original admission and must not be renewed by library phases. |
| Fixed board facts | `include/TunnelMonitor/BoardPins.h:15-30,80-87`; `include/TunnelMonitor/i2c/I2cConfig.h:9` | ESP32-S3-N16R8 hardware 2.0.0, SDA GPIO8, SCL GPIO9, 400 kHz, BME280 address `0x76`. No ENV power/reset GPIO is defined. |
| Runtime ENV interchangeability | `docs/guidelines/i2c_peripherals.md:427-445`; `docs/guidelines/decisions.md:102` | Every read evaluates SHT3x `0x44`, SHT3x `0x45`, and BME280 `0x76`, then selects the lowest valid address. This remains owner/application policy. |
| Expected-miss probe accounting | `src/i2c/I2cTask.cpp:2372-2392` | The owner probes each candidate before chip protocol. A discovery NACK is optional absence evidence and does not increment global error counters. The BME library must not hide another ACK-only probe. |
| Identity | `src/i2c/I2cTask.cpp:2455-2483` | BME280 identity requires register `0xD0 == 0x60`. Address ACK alone is insufficient. |
| Forced sample | `src/i2c/I2cTask.cpp:2535-2619` | Current settings are humidity, temperature, and pressure x1; forced mode; nonblocking conversion wait; one coherent eight-byte data read. |
| Fixed public result | `include/TunnelMonitor/contracts/EnvPowerDisplay.h:11-21,80-105,115-128` | Output is signed milli-degrees Celsius, signed milli-percent RH, signed Pascals, validity flags, address, kind, and 64-bit completion uptime. Library types remain private. |
| Complete BME result | `docs/guidelines/measurement_data.md:202-212` | Temperature, humidity, and pressure must all be valid for BME280 success. Partial data is a failed/partial measurement, not a successful BME sample. |
| No stale success | `src/output/OutputAutomation.cpp:357-426`; `src/measurement/MeasurementAssembler.cpp:157-210` | A failed refresh must not publish old data with current-valid flags. Fan automation uses validity/freshness and must enter its existing failsafe when ENV temperature is unavailable. |
| Optional absence | `docs/guidelines/i2c_peripherals.md:433-445,494-503` | No supported sensor maps to optional disabled/absent and does not degrade aggregate health. Protocol and transport faults remain distinguishable. |
| Fixed memory | `include/TunnelMonitor/i2c/I2cTask.h:121-149` | No steady heap, dynamic STL, unbounded retry, or unbounded wait. Current BME buffers are fixed 26, 7, and 8-byte arrays. |

The generic candidate probe should stay outside the BME280 library. The
preferred sequence is owner expected-miss probe, then a library identity/init or
sample job. The first library transaction reads chip ID; it is not another
ACK-only discovery probe.

## What already fits

These v1.7.0 properties should be preserved:

- Core headers and source are independent of Arduino, ESP-IDF, FreeRTOS, and
  `Wire`.
- I2C and time are injected through non-owning callbacks
  (`include/BME280/Config.h:11-49,89-123`).
- The core uses fixed storage. No heap allocation or dynamic container was found
  in normal library paths.
- Driver objects are noncopyable and nonmovable, and the destructor performs no
  I2C (`include/BME280/BME280.h:168-190`).
- `Oversampling`, `Mode`, `Filter`, and `Standby` are already typed.
- Cooperative init, forced measurement, config apply, and recovery jobs already
  exist (`include/BME280/BME280.h:227-266`).
- `pollJob()` defaults to one callback, reports `instructionsUsed`, honors the
  supplied budget, and has a bounded 32-phase local guard
  (`src/BME280.cpp:746-760`).
- The core has no internal I2C retry loop. All transport calls pass through two
  small wrappers (`src/BME280.cpp:1734-1754`).
- Chip ID `0x60` is checked explicitly.
- NVM and measuring polling is bounded by time and poll count in the staged
  path.
- Calibration reads and signed H4/H5 unpacking follow the Bosch layout.
- Temperature is computed first to produce `t_fine`; pressure uses widened
  64-bit arithmetic and a divide-by-zero guard; humidity is clamped.
- The raw sample is read coherently in one `0xF7..0xFE` burst.
- Skipped-channel sentinels and per-channel validity are exposed.
- Fixed-point output is available; TunnelMonitor does not need the float API.
- Partial multi-register configuration updates are already tracked with
  `hardwareConfigDirty()` and a root-cause status.
- Wrap-safe 32-bit short-deadline comparison is used.
- Default x1 temperature/pressure/humidity, filter-off, forced-on-demand settings
  match the current TunnelMonitor BME path.

The staged engine is the correct foundation. The audit does not recommend a new
generic driver framework or a second scheduler.

## Hard findings

### H-01: failed compensation can corrupt the last-good cache

Priority: correctness blocker

`_readRawData()` writes directly into `_rawSample` before the new sample is
fully validated (`src/BME280.cpp:2198-2221`). `_compensate()` immediately clears
and then partially rewrites `_compSample` and `_tFine`, but it has later failure
exits for invalid/skipped values, overflow, and pressure division by zero
(`src/BME280.cpp:2224-2330`).

If a previous valid sample exists and the new compensation fails, `_hasSample`
and the old timestamp remain set. `getRawSample()` and
`getCompensatedSample()` only test `_hasSample`
(`src/BME280.cpp:1246-1267`). They can return new raw data plus cleared or
partially updated compensated data under the old timestamp.

`sampleFreshness()` may label the cache stale after the error, but that does not
repair the corrupted last-good sample. An adapter-side freshness check is not a
fix for an internally inconsistent snapshot.

Required refactor:

- Decode the burst into job-local candidate raw data.
- Compensate into job-local candidate fixed-point data and candidate `tFine`.
- Validate all requested channels.
- Commit raw, compensated, timestamp, sequence, validity, and `tFine` together
  only after the complete operation succeeds.
- On any failed refresh, leave the previous sample byte-for-byte unchanged and
  expose the new terminal error separately.

### H-02: configuration uncertainty does not block measurement

Priority: correctness blocker

`startForcedMeasurementJob()` does not check `hardwareConfigDirty`
(`src/BME280.cpp:695-716`). It can trigger a conversion while actual sensor
settings are unknown, then interpret the result using cached settings.

There is a second stale-data path. Successful staged apply clears dirty state
without invalidating or retagging the cached sample
(`src/BME280.cpp:1129-1130`). `sampleFreshness()` then sees clean hardware and a
prior successful measurement status and may classify the old sample as fresh
(`src/BME280.cpp:574-589`).

Required refactor:

- Replace the single dirty boolean as the authoritative gate with a small typed
  state such as `Synchronized`, `UpdateInProgress`, or `ResyncRequired`.
- Reject measurement start with a typed `RESYNC_REQUIRED` result unless settings
  are synchronized.
- Maintain a monotonically increasing configuration generation.
- Tag each committed sample with the generation under which it was captured.
- Invalidate the sample, or keep it explicitly stale under its old generation,
  after any successful settings change or resync.
- Never turn a stale sample fresh only by clearing a dirty flag.

### H-03: staged jobs do not have exclusive hardware ownership

Priority: architecture blocker

`_jobActive()` prevents `tick()` and another staged job from progressing
concurrently (`src/BME280.cpp:344-350,592-626`). It does not reject synchronous
measurement requests, probe/recovery, typed setters, soft reset, `end()`, or raw
register access.

Relevant entry points include:

- `requestMeasurement()` at `src/BME280.cpp:1150`;
- typed configuration starting near `src/BME280.cpp:1317`;
- `softReset()` at `src/BME280.cpp:1588`; and
- raw register access at `src/BME280.cpp:1816`.

Even a single-threaded caller can invoke one of these between two `pollJob()`
calls. That can change hardware and cache state during a measurement, apply, or
recovery job. It also bypasses the owner's one-callback operation budget.

Required refactor:

- Route every hardware-facing public entry point through one operation-state
  gate.
- While a staged job is running or waiting, allow only `pollJob()`, zero-I2C
  cancellation, and cached snapshots.
- Return a typed busy cause. Do not require callers to parse a message string.
- Add tests for every hardware-facing public method during every job kind.

### H-04: there is no safe cancellation at the owner deadline

Priority: integration blocker

The public job API has start, poll, and status calls but no cancel or abort
(`include/BME280/BME280.h:227-266`). TunnelMonitor's 1000 ms deadline begins at
command admission and includes queue wait. Library NVM and conversion deadlines
start later inside job phases.

If the owner deadline expires after a reset or configuration write, it cannot
terminate the library job and record the resulting uncertain chip state.
Stopping calls to `pollJob()` leaves the driver permanently busy. Calling
`end()` is not safe cancellation because it performs I2C and erases state.

Required refactor:

- Add `cancelJob(CancelReason)` with zero I2C side effects.
- Return a stable terminal `CANCELLED` or `DEADLINE_EXPIRED` result.
- If a configuration/reset write succeeded or may have reached the device, set
  synchronization state to `ResyncRequired`.
- Cancelled forced conversion must invalidate only the in-progress candidate;
  preserve the committed last-good sample as stale diagnostic data.
- Record a possible ambiguous trigger state when cancellation follows a trigger
  write.
- Let `I2cTask` retain the original wrap-safe 64-bit deadline. The library must
  not renew it.

### H-05: mandatory library `OFFLINE` policy conflicts with `I2cTask`

Priority: architecture blocker

`offlineThreshold` defaults to five and zero is normalized to one
(`include/BME280/Config.h:121-123`; `src/BME280.cpp:297-300`). Tracked wrappers
refuse normal I2C after the library enters `OFFLINE`
(`src/BME280.cpp:1757-1789`).

TunnelMonitor already owns optional presence, device health, bus health,
deadline, retry, backoff, recovery, hotplug, and aggregate-health policy. A
second latch can reject an owner-directed BME attempt after the bus is healthy
again. It also treats expected runtime absence as a library lifecycle problem.

The counter meaning is weak for whole measurements. Health is updated per I2C
callback, and any successful callback clears consecutive failures
(`src/BME280.cpp:1860-1903`). A logical measurement can repeatedly succeed in
its trigger writes and fail at the final read without accumulating a useful
logical-operation failure count.

Required refactor:

- Make core execution passive: run the requested chip step and return the exact
  result.
- Remove health-based I2C admission gating from the core.
- Keep counters only as observation if they never block an owner request.
- If existing standalone users require managed health, place it in an optional
  compatibility facade. Do not make TunnelMonitor run two health owners.

### H-06: `end()` performs hidden, unreported I2C

Priority: platform lifecycle blocker

The destructor is correctly zero-I2C. `end()` is not. It writes sleep mode
through the callback, discards the result, then clears runtime state
(`src/BME280.cpp:429-454`).

This can access a bus or callback context during teardown, performs work outside
the staged job budget, and cannot report failure. It is also an unsafe escape
from a timed-out active job.

Required refactor:

- Make `end()` or `unbind()` strictly zero-I2C.
- Add an explicit cooperative sleep/shutdown operation only if a consumer needs
  it.
- Return its transport result and let the owner decide whether to wait for it.
- In TunnelMonitor forced mode, a completed measurement already returns the
  BME280 to sleep, so shutdown I2C is normally unnecessary.

### H-07: transport callback semantics are too broad

Priority: integration contract blocker

Both transport callbacks return the general library `Status`
(`include/BME280/Config.h:11-42`). That type includes driver states such as
`IN_PROGRESS` and `BUSY`, although a callback must return one terminal transport
result. `_failJob()` can therefore produce the contradictory combination
`JobState::FAILED` with an `IN_PROGRESS` status
(`src/BME280.cpp:636-660`).

The callback contract also does not state that success means all requested
bytes completed, that write/read is one combined register-pointer transaction,
or that the callback did not perform a hidden retry or bus recovery.

Required refactor:

- Introduce a terminal-only `TransportResult` with transport error code and
  numeric detail.
- Define success as exact completion of the requested write and read lengths.
- Define write/read as one combined transaction with the required repeated
  start behavior supplied by the owner backend.
- Forbid recursion and library/adapter-internal retry.
- Map transport results to driver status inside the library.
- Keep the two callback shapes. The adapter can treat `i2cWrite` as mutating and
  `i2cWriteRead` as read-only without decoding register bytes.

TunnelMonitor's approved owner policy may perform its bounded recovery/read
retry exception for a read-only callback. A mutating BME write, especially the
forced trigger, must be exactly one physical attempt. Current direct optional
steps pass `allowRetry=true` for writes and reads
(`src/i2c/I2cTask.cpp:2372-2619`); the adapter must not preserve that setting for
mutating writes.

### H-08: cooperative recovery always resets the sensor

Priority: required recovery refactor

Synchronous `recover()` reads identity and NVM status, reloads calibration, and
reapplies configuration without issuing a reset
(`src/BME280.cpp:473-525`). `startRecoveryJob()` always begins with a soft-reset
write (`src/BME280.cpp:728-744,1090-1102`).

The only callback-budgeted recovery is therefore more destructive than the
synchronous path. A transient shared-bus fault does not justify a chip reset
merely because TunnelMonitor requires cooperative execution.

Required refactor:

- Add an explicit cooperative `startResyncJob()` that verifies identity/NVM,
  reloads calibration, and reapplies settings without reset.
- Rename or document the current operation as `startSoftResetJob()`.
- Let `I2cTask` choose among bus recovery, BME resync, and explicit BME reset.
- Never perform a device reset as an automatic side effect of a transport
  failure.

### H-09: ambiguous forced-trigger state is not represented

Priority: correctness requirement

The staged forced job writes `ctrl_hum`, then writes forced `ctrl_meas` and
assumes a failed callback did not start conversion
(`src/BME280.cpp:985-1017`). A timeout can occur after the device accepted the
trigger. The sensor may still be converting even though the job reports
failure.

A new staged forced job can immediately repeat the trigger. The synchronous
path behaves differently: it reads the measuring bit first
(`src/BME280.cpp:1179-1198`).

Required refactor:

- Track an explicit conversion state, including
  `UnknownAfterTriggerError`.
- After an ambiguous trigger error, converge by reading `status.measuring`
  before another trigger.
- Never internally replay the same trigger write.
- Once synchronized config has latched `ctrl_hum`, do not rewrite unchanged
  humidity settings before every sample. The normal steady forced path can be
  trigger, nonblocking wait/status, then coherent data burst.

### H-10: calibration lifecycle is incomplete for runtime replacement

Priority: integration requirement

The library reads calibration at initialization and caches it, which is normal
for a fixed device. TunnelMonitor explicitly supports ENV removal and
replacement without reboot. Calibration coefficients are device-specific and
must not survive a device-generation change without validation.

The current direct firmware re-reads both calibration blocks for every BME
candidate measurement (`src/i2c/I2cTask.cpp:2487-2531`). This is inefficient
but handles replacement safely.

Required integration behavior:

- Provide a zero-I2C `invalidateCalibration()` or equivalent device-state
  invalidation.
- Invalidate on owner-observed absence, identity mismatch, reset, or uncertain
  device replacement.
- Require init/resync to reload and validate calibration before the next sample.
- Do not treat an address ACK as proof that the cached coefficients still belong
  to the installed device.

The simplest first integration is to run the staged identity/calibration/config
init for each BME candidate evaluation before its forced sample. This preserves
current hotplug behavior and remains bounded. If later measurements show that
calibration caching is worth the optimization, cache it only with explicit
device-generation invalidation and replacement tests.

### H-11: stored status contains unenforced borrowed text

Priority: required API cleanup for platform use

`Status` carries `const char* msg` and persistent driver fields copy complete
status objects (`include/BME280/Status.h:33-46`). The API says messages must be
static, but a transport adapter can accidentally return a pointer to temporary
storage.

Required refactor:

- Store code and numeric detail in persistent state.
- Derive library-owned text with `toString()`.
- Do not cache transport-provided pointers.
- Use distinct typed causes for `Busy`, `ResyncRequired`, `Cancelled`, and
  terminal transport errors instead of message parsing.

## Important secondary findings

These should be handled in the same major refactor when practical, but they are
not all integration blockers by themselves.

### S-01: configuration and transport timing are mixed

`i2cTimeoutMs` is also used as post-estimate conversion-ready grace time
(`src/BME280.cpp:1013,1027,1191-1209`). Split it into transport timeout and a
chip-specific conversion-ready timeout. TunnelMonitor sets transport timeout to
20 ms and retains the outer 1000 ms operation deadline.

`i2cTimeoutMs` is validated only as nonzero (`src/BME280.cpp:272-274`). Any
interval used with the signed wrap-safe deadline comparison must be constrained
below `INT32_MAX`.

### S-02: cooperative timing uses two time sources

`pollJob()` receives `nowMs`, but health timestamps call `Config::nowMs`. If no
hook exists, the framework-neutral fallback always returns zero
(`src/PlatformTime.h:10-14`; `src/BME280.cpp:2345-2349`).

The cooperative core should use the timestamp passed to `pollJob()` for its
operation state. Standalone synchronous convenience APIs may require an
explicit time hook. Do not silently use an inert clock for meaningful
timestamps.

### S-03: calibration validation should remain narrow but complete

The library rejects `digT1` and `digP1` values of zero or `0xFFFF`
(`src/BME280.cpp:2162-2170`). BME280 provides no calibration CRC, so broad
coefficient plausibility limits would create false rejects.

A narrow erased-block check for an all-zero or all-`0xFF` humidity block is
reasonable when humidity is enabled. Keep the exact transport/protocol error
visible rather than claiming CRC-level integrity.

### S-04: build-tool pinning is incomplete

`platformio.ini` uses broad `platform = espressif32`, and CI installs PlatformIO
without an exact version. The audit resolved Espressif platform 54.3.20,
Arduino 3.2.0, and ESP-IDF libraries 5.4.0 locally, but that resolution can
change.

Exact-pin the qualified toolchain in the library release process and exact-pin
the reviewed BME280 revision in TunnelMonitor.

## Recommended narrow design

Do not add a generic sensor registry, scheduler, task, service manager, or ENV
abstraction to this library. Strengthen the existing BME280 state machine.

### Cooperative core operations

A simple target API is:

```cpp
Status bind(const TransportConfig& transport,
            const SensorSettings& settings);       // zero I2C
Status startInitializeJob();                        // zero I2C request
Status startForcedMeasurementJob();                 // zero I2C request
Status startApplySettingsJob(const SensorSettings& settings);
Status startResyncJob();                            // no soft reset
Status startSoftResetJob();                         // explicit reset
Status startSleepJob();                             // explicit optional I2C
Status cancelJob(CancelReason reason);              // zero I2C
JobPollResult pollJob(uint32_t nowMs,
                       uint8_t maxInstructions = 1);
void unbind();                                      // zero I2C
```

Names may differ. The behavior is the important part:

- bind and start methods perform no transport callback;
- one poll with budget one performs at most one library callback;
- one active job owns all hardware-facing access;
- cancellation is always available and performs no I2C;
- last-good sample commit is atomic;
- partial config and ambiguous trigger state remain visible;
- absence does not erase the transport binding or force an offline latch; and
- resync and reset are different explicit operations.

The existing synchronous APIs may remain as clearly marked standalone
compatibility helpers. They must not be the authoritative platform core, and
TunnelMonitor must not call them from the I2C owner path.

### Useful types

Required or strongly useful:

- `TransportResult { TransportErr code; int32_t detail; }`
- `TransportConfig`
- `SensorSettings`
- `ConfigSyncState { Synchronized, UpdateInProgress, ResyncRequired }`
- `ConversionState`, including an ambiguous-trigger state
- `CancelReason`
- `SampleEnvelope` containing fixed-point sample, validity mask, timestamp,
  sample sequence, and configuration generation
- public coarse `JobPhase` or `JobProgress`
- `Err::RESYNC_REQUIRED`
- `Err::CANCELLED`
- distinct typed busy reasons

Extend the existing `JobKind`, `JobState`, `CompensatedSample`, and
`JobPollResult` where practical. Do not create a parallel job framework.

Reasonable nice-to-have types:

- `enum class I2cAddress : uint8_t { Primary = 0x76, Secondary = 0x77 }`;
- a compact `ChannelMask` in detailed diagnostics;
- separate small operational and large diagnostic snapshots; and
- a zero-I2C `CalibrationState` or device-generation snapshot.

### Useful pure helpers

- `validateSettings()` with no I2C;
- `estimateMeasurementTimeUs(const SensorSettings&)` using the Bosch maximum
  formula;
- existing millisecond estimate as a rounded/safe wrapper;
- checked conversion from `tempC_x100` to milli-degrees Celsius;
- checked conversion from `humidityPct_x1024` to milli-percent RH;
- `isBme280ChipId(uint8_t)`;
- `invalidateCalibration()` / `invalidateDeviceState()` with no I2C; and
- `toString()` for transport, driver, job, phase, sync, and conversion enums.

Lower-priority cleanup:

- `CalibrationRaw::tp` already includes register `0xA1`, while `h1` stores that
  byte again (`include/BME280/BME280.h:107-112`). Remove the duplicate in the
  next breaking layout change.
- `getSettings()` always succeeds and performs no I2C. A plain snapshot return
  would be clearer than a fallible `Status` API.

## Recommended TunnelMonitor integration flow

The BME280 library should remain a private leaf within the existing active
`ReadEnv` envelope.

For each BME candidate trial:

1. `I2cTask` performs its existing generic expected-miss probe at `0x76`.
2. On ACK, the adapter starts the zero-I2C BME init/resync job needed by the
   current calibration policy.
3. Each owner poll calls `pollJob(nowLow32, 1)` once.
4. After synchronized init, the adapter starts the forced-measurement job.
5. The job waits cooperatively, checks readiness, reads `0xF7..0xFE` in one
   transaction, compensates into candidate storage, and atomically commits.
6. Only terminal success with all three valid channels maps to
   `EnvReadResult`.
7. The owner continues evaluating all SHT3x/BME candidates and applies its
   existing deterministic selection and flags.
8. On original deadline expiry, the owner calls `cancelJob()` and terminates the
   current `ReadEnv` without further I2C.
9. On owner-observed disappearance or replacement, the adapter invalidates BME
   calibration/device state before a later attempt.

Fixed-point mapping is straightforward and bounded:

```text
temperatureMilliCelsius     = tempC_x100 * 10
relativeHumidityMilliPercent = humidityPct_x1024 * 1000 / 1024
pressurePascal               = pressurePa
```

Check ranges before converting unsigned library pressure to the signed project
field. Do not use the library float `Measurement` path.

The integration should delete the duplicate BME implementation after the
adapter is proven:

- `Bme280Calibration`, `parseBme280Calibration()`, and `decodeBme280()` in
  `src/i2c/I2cTask.cpp:530-680`;
- BME-specific step enums and buffers in
  `include/TunnelMonitor/i2c/I2cTask.h:88-93,136-139`; and
- BME protocol assumptions in the generic fake backend where library-adapter
  tests replace them.

Keep the generic active-job envelope, deadline, candidate policy, status
projection, and fake owner transport. Do not retain two compensation paths.

## What must stay in TunnelMonitor

The BME280 library owns only BME chip protocol, calibration, compensation, and
fixed chip-operation phases. It must not own:

- the I2C bus, pins, handle, lock, timeout enforcement, bus recovery, or task;
- request queues, priorities, admission deadlines, periodic cadence, or result
  slots;
- read/write retry policy;
- optional-device presence, health, aggregate health, watchdog, or events;
- SHT3x/BME candidate discovery and deterministic selection;
- sensor-changed or multiple-candidate flags;
- fan failsafe, measurement quality, stale threshold, application units, CLI,
  web, storage, or cloud policy; or
- public `EnvReadCommand`, `EnvReadResult`, `EnvStatus`, `PublicStatus`, and
  project error types.

## Validation evidence and gaps

### Results available now

| Check | Result | Scope |
| --- | --- | --- |
| Native test suite | PASS, 136/136 | Exact v1.7.0 commit audited |
| Arduino ESP32-S3 build | PASS | Example build; 22,584 bytes RAM and 394,334 bytes flash |
| Arduino ESP32-S2 build | PASS | Example build; 37,000 bytes RAM and 383,713 bytes flash |
| Core timing, CLI, HIL, IDF example, version, and release metadata guards | PASS | Local scripted checks |
| HIL parser tests | PASS, 22/22 | Parser only |
| Package build and content check | PASS | Local release-package check |
| Doxygen | PASS without warnings | Doxygen 1.15 local generation |
| ESP-IDF CI | PASS in [exact-commit workflow run 28164301403](https://github.com/janhavelka/BME280/actions/runs/28164301403) | Six jobs passed, including ESP32-S2/S3 IDF builds with IDF v5.3.2; local `idf.py` was unavailable |
| Retained BME280 HIL | Useful ESP32-S2 Arduino serial/soak evidence | One COM28 fixture at `0x76`; formal hardware coverage remains incomplete |
| Retained TunnelMonitor ENV stress | Positive current-direct-path evidence | Sensor identity/module/electrical details are not retained well enough to qualify this library adapter |

The library's native tests already cover signed calibration parsing, raw burst
reconstruction, synthetic compensation, humidity clamps, pressure divide by
zero, wraparound timing, dirty state, and staged callback budgets. Those are
strong foundations.

### Limits of current HIL evidence

`docs/reports/esp32s2-com28-hil-summary.md` records one ESP32-S2 Arduino fixture,
address `0x76`, GPIO8/GPIO9, and 400 kHz. It includes a 20-hour comprehensive
soak and later reset-classifier runs with no reported FAIL or TIMEOUT rows.

It does not fully retain:

- the exact v1.7.0 release commit/version used for every run;
- BME280 module/ordering code;
- VDD/VDDIO, CSB/SDO straps, pull-ups, and bus capacitance;
- full raw artifact packages and manifests;
- calibrated temperature, pressure, and humidity references;
- ESP32-S3 hardware runtime;
- ESP-IDF hardware runtime;
- address `0x77` hardware;
- protected address/data NACK, timeout, bus-fault, and hotplug evidence;
- real shared-bus contention; or
- logic-analyzer proof of the coherent data burst.

The hardware matrix correctly leaves these rows incomplete. Do not reinterpret
serial plausibility or build success as physical field qualification.

## Required tests after refactor

### Library-native tests

Add focused regression tests that prove:

- bind/start/cancel/unbind/destructor perform zero callbacks;
- `pollJob(now, 1)` performs at most one callback for every job and phase;
- all hardware-facing synchronous APIs reject while a staged job is active;
- a good cached sample remains byte-for-byte unchanged after raw sentinel,
  overflow, divide-by-zero, humidity, or compensation failure;
- raw, compensated, timestamp, sequence, and config generation commit atomically;
- dirty/resync-required config blocks measurement;
- successful apply/resync cannot make an old-generation sample fresh;
- cancellation before I2C, after each config write, after trigger, during wait,
  and after status read preserves correct sync/conversion state;
- a later job can start after cancellation;
- terminal transport types cannot represent `IN_PROGRESS`;
- exact byte-count completion is required;
- mutating callbacks are never internally retried;
- ambiguous trigger failure converges through status before another trigger;
- resync performs no reset and soft-reset job sends exactly `0xE0,0xB6`;
- absence/offline history never blocks an explicit core operation;
- calibration invalidation requires reload before measurement;
- wrong ID and erased calibration blocks return distinct errors;
- H4/H5 signed nibble decode, skipped sentinels, humidity clamp, and pressure
  divisor guards remain correct; and
- official Bosch compensation vectors and boundary inputs pass under host
  sanitizers where available.

An ASan/UBSan host CI job and differential comparison against Bosch reference
formulas are reasonable hardening improvements, not prerequisites for starting
the refactor.

### TunnelMonitor integration tests

Prove the adapter preserves:

- one library callback per normal owner poll;
- the original 64-bit 1000 ms `ReadEnv` deadline, including queue wait;
- 20 ms per callback clipped to remaining owner time;
- owner expected-miss probe accounting before BME protocol;
- no write retry and only the approved bounded read-only recovery exception;
- full SHT3x `0x44`, SHT3x `0x45`, BME280 `0x76` evaluation;
- last-success trial optimization without changing lowest-address selection;
- multiple-candidate and sensor-changed flags;
- `DEVICE_NOT_FOUND`/absence versus ID, calibration, compensation, timeout, data
  NACK, and bus errors;
- all three BME validity fields required for success;
- no stale numeric fields or validity bits after a failed current read;
- cancellation at the original deadline with no later I2C;
- removal/reappearance and BME-to-SHT/SHT-to-BME replacement without reboot;
- calibration reload after device-generation invalidation;
- fixed integer unit conversion and range checks; and
- continued RTC, FRAM, power, and display scheduling during BME waits/failures.

The existing fake BME register model is permissive and does not model status,
conversion timing, reset, or write ordering. Add a focused library fake with
those state transitions; do not make the production backend into a simulator.

### Physical TunnelMonitor HIL

On hardware revision 2.0.0 with the actual BME280 module:

- record exact firmware/library commits and dirty state;
- record module, VDD/VDDIO, CSB and SDO straps, decoupling, pull-ups, lead length,
  rise time, and bus capacitance;
- confirm `0x76`, chip ID `0x60`, and the exact 400 kHz bus;
- verify repeated forced temperature/humidity/pressure samples and return to
  sleep;
- use a logic analyzer to confirm combined register reads and one coherent
  `0xF7..0xFE` burst;
- unplug/replug BME280 and replace BME280/SHT3x without reboot;
- inject protected NACK, timeout, and bus faults, then prove fresh recovery with
  no stale-valid output;
- run with RTC, FRAM, INA228, and display traffic and prove no starvation;
- verify fan automation enters its existing safe fallback when BME temperature
  becomes invalid; and
- retain transcript, manifest, hashes, and operator notes.

Reference-instrument comparisons are required if environmental accuracy is an
acceptance claim. A reasonable shared-bus soak is useful; there is no need to
repeat extreme endurance testing without a concrete field risk.

## Physical suitability boundary

Several field constraints are hardware/enclosure issues, not missing driver
features:

- The Bosch humidity specification is for non-condensing operation. Tunnel
  condensation or liquid contact must be prevented by enclosure and vent
  design.
- Pressure needs a suitable vent path. The sensor must be protected from liquid,
  contamination, and strong light as specified by Bosch handling guidance.
- The internal temperature is influenced by PCB temperature and self-heating.
  Fan-control acceptance must characterize the installed location rather than
  assume it is free-air ambient temperature.
- CSB must be held high for I2C, SDO must not float, and the selected strap must
  produce address `0x76`.
- Bosch warns against holding interface pins high while VDDIO is off.
- Pull-ups must be sized for the real shared-bus load; the datasheet gives 4.7
  kohm as a normal value, not a universal requirement.

Do not add application offsets, condensation policy, fan policy, or enclosure
logic to the BME280 chip library. Record and qualify them at the board/system
level.

## Recommended implementation order

1. Fix atomic sample candidate/commit behavior and add regression tests.
2. Add config generation/synchronization and ambiguous conversion state.
3. Add one central active-operation gate and zero-I2C cancellation.
4. Make core health observational and make unbind/end zero-I2C.
5. Tighten terminal transport results and stored error representation.
6. Split resync from explicit soft reset and remove unnecessary steady
   `ctrl_hum` writes.
7. Add calibration/device-generation invalidation and public job progress.
8. Pin and release the refactored library revision with all software checks.
9. Implement one private `I2cTask` adapter and delete the direct BME protocol.
10. Run native integration tests, exact production builds, ESP32-S3 shared-bus
    HIL, hotplug/fault tests, and installed-enclosure environmental checks.

## Final assessment

BME280 v1.7.0 is the right codebase to harden. Its staged job engine,
calibration parsing, coherent data burst, fixed-point compensation, fixed
memory, and transport-neutral design are strong and should replace the direct
chip math in TunnelMonitor after the focused refactor.

The remaining gaps are state truth and owner integration, not basic Bosch
protocol support. Fix the core cache commit, configuration generation,
operation exclusivity, cancellation, health gating, and recovery semantics in
the library. Then integrate it as a private passive leaf while `I2cTask` retains
candidate selection, deadlines, retry policy, health, and public result mapping.

After those changes, exact pinning, and real ESP32-S3 shared-bus and enclosure
qualification, the library should be suitable for a platformized
TunnelMonitor. It should not replace the current direct BME280 path before those
gates are complete.
