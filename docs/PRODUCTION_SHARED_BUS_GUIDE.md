# BME280 Production Shared-Bus Guide

This guide describes a production integration shape for applications that share
one I2C bus between a BME280 and other devices. It is architecture guidance, not
hardware validation evidence and not a replacement for the diagnostic examples.

The BME280 core driver is framework-neutral and non-owning. It does not create
or reset an I2C bus, configure pins, own a mutex, call platform logging APIs, or
run a task. The application owns those policies and injects bounded callbacks
through `BME280::Config`.

## Ownership Model

Application-owned resources:

- I2C peripheral, bus/device handles, pins, clock, pull-ups, and timeout policy.
- A bus mutex used by every device on the shared bus.
- A driver-instance mutex or single owner task for each `BME280::BME280`
  instance.
- The monotonic clock used by `Config::nowMs`, `pollJob(nowMs, ...)`, and
  `tick(nowMs)`.
- End-to-end operation deadlines, including time spent queued before a library
  job starts.
- Power/reset GPIOs and any bus recovery/reset procedure.
- Retry, backoff, degraded/offline policy, and field telemetry.

Driver-owned state:

- Cached BME280 configuration and calibration.
- Measurement scheduling state.
- One fixed-memory staged job, including nonzero identity, public phase,
  chip-phase deadline, conversion state, and terminal status.
- Cached raw and compensated samples.
- Health counters and dirty-state diagnostics.

Do not call BME280 public APIs from an ISR. An ISR should notify a task, set an
atomic flag, or enqueue work that a task handles later.

## Recommended Shape

Prefer a single BME280 owner task when the application has multiple tasks:

```text
application tasks
    |
    | enqueue requests / read latest published sample
    v
BME280 owner task
    |
    | owns BME280 instance mutex or is the only caller
    v
BME280::BME280 driver
    |
    | Config::i2cWrite / Config::i2cWriteRead
    v
application I2C bus manager
    |
    | owns shared bus mutex and finite transfer timeout
    v
I2C peripheral shared with other devices
```

This keeps driver state serialized and avoids mixing console, telemetry, and
sensor work into the same call path.

## Transport Boundary

The transport callbacks are terminal adapters for exactly one physical attempt.
They must be small, synchronous, bounded, non-recursive, and must not retry or
recover the bus:

```cpp
struct BusContext {
  I2cHandle bus;
  Mutex* busMutex;
  uint32_t maxTimeoutMs;
};

BME280::TransportResult bmeWrite(uint8_t addr,
                                 const uint8_t* data,
                                 size_t len,
                                 uint32_t timeoutMs,
                                 void* user) {
  auto* ctx = static_cast<BusContext*>(user);
  if (ctx == nullptr || data == nullptr || len == 0) {
    return BME280::TransportResult::Error(BME280::TransportErr::OTHER, -1);
  }

  // Deadline and remainingMs() are application helpers using one monotonic
  // clock. The callback budget includes both lock acquisition and transfer.
  const Deadline deadline =
      Deadline::afterMs(clampTimeout(timeoutMs, ctx->maxTimeoutMs));
  LockGuard lock(*ctx->busMutex, deadline.remainingMs());
  if (!lock.locked()) {
    return BME280::TransportResult::Error(BME280::TransportErr::TIMEOUT,
                                          BUS_LOCK_TIMEOUT);
  }

  // Exactly one mutating transfer; never replay this call in the adapter.
  const uint32_t transferBudgetMs = deadline.remainingMs();
  if (transferBudgetMs == 0) {
    return BME280::TransportResult::Error(BME280::TransportErr::TIMEOUT,
                                          CALLBACK_DEADLINE_EXPIRED);
  }
  I2cResult result =
      i2cTransmit(ctx->bus, addr, data, len, transferBudgetMs);
  return mapI2cResult(result, len, 0);
}

BME280::TransportResult bmeWriteRead(uint8_t addr,
                                     const uint8_t* txData,
                                     size_t txLen,
                                     uint8_t* rxData,
                                     size_t rxLen,
                                     uint32_t timeoutMs,
                                     void* user) {
  auto* ctx = static_cast<BusContext*>(user);
  if (ctx == nullptr || txData == nullptr || rxData == nullptr ||
      txLen == 0 || rxLen == 0) {
    return BME280::TransportResult::Error(BME280::TransportErr::OTHER, -2);
  }

  const Deadline deadline =
      Deadline::afterMs(clampTimeout(timeoutMs, ctx->maxTimeoutMs));
  LockGuard lock(*ctx->busMutex, deadline.remainingMs());
  if (!lock.locked()) {
    return BME280::TransportResult::Error(BME280::TransportErr::TIMEOUT,
                                          BUS_LOCK_TIMEOUT);
  }

  // One combined pointer-write/repeated-START/read transaction, with no STOP
  // between phases and no adapter retry.
  const uint32_t transferBudgetMs = deadline.remainingMs();
  if (transferBudgetMs == 0) {
    return BME280::TransportResult::Error(BME280::TransportErr::TIMEOUT,
                                          CALLBACK_DEADLINE_EXPIRED);
  }
  I2cResult result = i2cTransmitReceive(ctx->bus, addr, txData, txLen,
                                        rxData, rxLen, transferBudgetMs);
  return mapI2cResult(result, txLen, rxLen);
}
```

The callback timeout is one end-to-end budget for lock acquisition plus the
physical transfer. Do not give each phase the original timeout independently;
that can double the documented blocking bound. The callback may lock the shared
bus for the actual I2C transaction, but it does not make the BME280 driver
instance thread-safe by itself. Serialize public driver calls with a driver
mutex or by routing them through one owner task.

Return `TransportResult::Complete()` only with the exact physical byte counts.
An `OK` result with shorter counts becomes `Err::I2C_SHORT_TRANSFER` in the
core. Preserve precise terminal transport errors when the platform proves them:

- definite address NACK: `TransportErr::NACK_ADDRESS`;
- definite data NACK: `TransportErr::NACK_DATA`;
- transfer timeout: `TransportErr::TIMEOUT`;
- bus/arbitration fault: `TransportErr::BUS`;
- phase-ambiguous NACK or another platform failure: `TransportErr::OTHER`.

The driver maps those results to canonical `Status` codes and retains the
adapter numeric diagnostic in `Status::detail`. Only a definite address NACK
can become optional `DEVICE_NOT_FOUND` at an identity/presence boundary.

## Cooperative Driver Initialization

```cpp
static BusContext busCtx = {/* bus, mutex, maxTimeoutMs */};
static BME280::BME280 bme;

BME280::Config makeBme280Config() {
  BME280::Config cfg;
  cfg.i2cWrite = bmeWrite;
  cfg.i2cWriteRead = bmeWriteRead;
  cfg.i2cUser = &busCtx;
  cfg.nowMs = appMonotonicMs;
  cfg.timeUser = nullptr;
  cfg.i2cAddress = 0x76;     // 0x76 for SDO=GND, 0x77 for SDO=VDDIO
  cfg.i2cTimeoutMs = 50;     // finite transport timeout
  cfg.nvmReadyTimeoutMs = 10;
  cfg.conversionReadyTimeoutMs = 20; // chip-ready grace, not bus timeout
  cfg.mode = BME280::Mode::FORCED;
  return cfg;
}
```

For a shared-bus owner, prefer the staged API. Every start performs validation
and fixed-memory state setup but no I2C. Save the nonzero identity immediately:

```cpp
struct ActiveBmeJob {
  bool active = false;
  uint32_t jobId = 0;
  uint64_t ownerDeadlineMs = 0; // chosen and retained by the application
};

ActiveBmeJob activeBme;

BME280::Status startBme280Init(uint64_t ownerDeadlineMs) {
  BME280::Status st = bme.startInitJob(makeBme280Config());
  if (st.inProgress()) {
    activeBme = {true, bme.jobId(), ownerDeadlineMs};
  }
  return st;
}
```

The owner advances the job once per scheduling opportunity and captures the
result from that exact call:

```cpp
void pollBmeJob(uint32_t nowLow32, uint64_t nowMs) {
  BME280::JobPollResult result;

  if (activeBme.active && nowMs >= activeBme.ownerDeadlineMs) {
    BME280::Status cancelled =
        bme.cancelJob(BME280::CancelReason::DEADLINE_EXPIRED); // zero I2C
    if (!cancelled.ok() && cancelled.code != BME280::Err::DEADLINE_EXPIRED) {
      publishBmeFault(cancelled);
      return;
    }
    result = bme.pollJob(nowLow32, 0); // retrieve retained result exactly once
  } else {
    result = bme.pollJob(nowLow32, 1); // at most one transport callback
  }

  if (result.jobId == 0) {
    return;
  }
  if (result.jobId != activeBme.jobId) {
    publishBmeIdentityFault(result.jobId);
    return;
  }

  switch (result.state) {
    case BME280::JobState::RUNNING:
    case BME280::JobState::WAITING:
      return;
    case BME280::JobState::DONE:
      publishBmeJobSuccess(result);
      break;
    case BME280::JobState::FAILED:
    case BME280::JobState::CANCELLED:
    case BME280::JobState::TIMED_OUT:
      publishBmeFault(result.status);
      break;
    default:
      publishBmeIdentityFault(result.jobId);
      return;
  }
  activeBme = {};
}
```

Natural `DONE` and `FAILED` results exist only on the poll that reaches the
terminal transition, so the owner must not infer completion from a later status
read. Cancellation is different: its terminal result is retained until exactly
one `pollJob()` call retrieves it. While that result is pending, later job
starts and fallible synchronous hardware operations return `BUSY` without I2C;
`tick()` also performs no I2C. Cached inspection and zero-I2C `end()` remain
available.

`JobPollResult::phaseDeadlineMs` is an active BME280 chip-phase deadline for
NVM, conversion, or idle readiness. It is not the application deadline and
must not replace, extend, or restart `ownerDeadlineMs`. The owner may log
`JobPhase`, `phaseDeadlineActive`, `phaseDeadlineMs`, `callbacksUsed`, and
`ConversionState` as bounded progress evidence.

For I2C operation, tie CSB high to VDDIO before power-on reset. Do not leave SDO
floating. Do not drive SDA, SCL, SDO, or CSB high while VDDIO is off.

## Measurement Task

After staged initialization completes, a production shared-bus owner can use
the same envelope to start `startForcedMeasurementJob()` and advance it with
`pollJob(now, 1)`. The start is zero-I2C. On terminal success,
`getSampleEnvelope()` returns the atomically committed sample, timestamp,
sample sequence, and configuration generation without touching I2C.

`requestMeasurement()` plus scheduled `tick(nowMs)` remains a synchronous
compatibility path for simpler single-owner applications:

```cpp
void bmeTask() {
  BME280::Status st = initBme280();
  if (!st.ok()) {
    publishBmeFault(st);
    return;
  }

  uint32_t nextRequestMs = appMonotonicMs(nullptr);

  for (;;) {
    const uint32_t now = appMonotonicMs(nullptr);

    if (timeReached(now, nextRequestMs)) {
      BME280::Status req = bme.requestMeasurement();
      if (!req.ok() && !req.inProgress()) {
        publishBmeFault(req);
      }
      nextRequestMs = now + samplePeriodMs;
    }

    bme.tick(now);

    if (bme.measurementReady()) {
      BME280::Measurement m;
      BME280::Status read = bme.getMeasurement(m);
      if (read.ok()) {
        if (m.temperatureValid) {
          publishTemperature(m.temperatureC);
        }
        if (m.pressureValid) {
          publishPressure(m.pressurePa);
        }
        if (m.humidityValid) {
          publishHumidity(m.humidityPct);
        }
      }
    }

    sleepUntilNextSchedulerTick();
  }
}
```

`requestMeasurement()` returns `INVALID_CONFIG` if no `Config::nowMs` hook is
injected. `pollJob(nowMs, ...)` and `tick(nowMs)` use their explicit argument as
the time source for chip phases and health updates during that call; the hook is
used by synchronous calls outside those scopes. All must use the same monotonic
clock. If neither an explicit call time nor the hook is available, health time
values are zero and `lastOkTimeValid()` / `lastErrorTimeValid()` are false.

In forced mode, the device returns to sleep after conversion. A timeout or
cancellation after the trigger can leave `ConversionState` as
`UNKNOWN_AFTER_TRIGGER_ERROR`; the next staged forced job reconciles
`status.measuring` before it may issue one new trigger. The driver never replays
an ambiguous trigger. Once settings are synchronized, steady forced sampling
writes only `ctrl_meas`; `ctrl_hum` is latched during configuration apply and is
not rewritten for every sample. In normal mode, the compatibility scheduler
waits the worst phase interval (two configured conversion bounds plus one standby
interval) before reading so the sample is fresh relative to the request. An
ambiguous existing forced conversion receives one full configured conversion
interval plus readiness grace. These bounds remain wrap-safe across `uint32_t`
rollover.

`hasSample()` means the latest successful raw/compensated sample is cached; it
does not by itself prove freshness for the current request. Use
`sampleFreshness()` or `sampleFresh(nowMs, maxAgeMs)` before publishing data.
`STALE_AFTER_ERROR` means a later refresh failed or is still non-OK, and
`STALE_AFTER_CONFIG_DIRTY` means the hardware configuration may differ from the
driver cache. `STALE_AFTER_CONFIG_CHANGE` means the sample belongs to an older
configuration generation.
Dirty transitions clear pending/unread measurement readiness but preserve the
last `SampleEnvelope` for explicitly stale diagnostics. Terminal status-read,
raw-read, or compensation errors end the request, so later `tick()` calls perform
no I2C until the owner explicitly requests another measurement.

## Owner-Managed Integration Checklist

For an application that already owns a device queue or sensor-candidate policy,
keep one owner-private BME280 adapter and preserve these boundaries:

- advance a staged operation with `pollJob(nowMs, 1)` so one owner poll performs
  at most one transport callback;
- retain the application's original end-to-end deadline across queueing and all
  polls; a library chip-phase deadline must never replace or renew it;
- accept a sample only when every application-required channel validity flag is
  true, and use the checked fixed-point conversion helpers for integer units;
- map only definite address NACK / `DEVICE_NOT_FOUND` to optional absence;
  identity, calibration, timeout, data-NACK, bus, and compensation failures are
  faults;
- evaluate `sampleFreshness()` as well as cached values, and do not publish a
  stale last-good sample as the result of a failed current request;
- on removal or possible replacement, call zero-I2C
  `invalidateDeviceState()` and require a complete init/resync before measuring;
  an ACK alone does not prove cached calibration belongs to the current device;
- keep candidate selection, retries, aggregate health, recovery/backoff,
  application units, and publication policy in the application owner.

These are integration requirements, not hardware-validation claims. Validate
the final adapter with its actual queue, deadline, hotplug, shared-bus, and fault
policy.

## Shared Bus With Other Devices

Every device adapter on the bus should use the same bus manager and finite
timeout policy. Avoid long work while holding the bus mutex. Do not call a
BME280 public API from inside another device callback, and do not call another
driver from inside the BME280 transport callbacks.

If another device needs a bus reset, treat it as an application-level event:

1. Stop new transfers and acquire the bus manager lock.
2. Put affected devices into a known state when possible.
3. Perform the platform-specific bus recovery or peripheral reset outside the
   BME280 core driver.
4. Reinitialize bus handles if the platform requires it.
5. Start `startResyncJob()` from task context and poll it under the original
   application deadline. Use `startSoftResetJob()` only when application policy
   explicitly requires a device reset.
6. Request a fresh sample before publishing BME280 data again.

## Recovery Policy

The driver tracks communication health for the current health session. A
successful `begin()` starts a new session and resets total success/failure
counters. Consecutive failures move the driver through `DEGRADED` and then
`OFFLINE` according to `Config::offlineThreshold`.

`OFFLINE` is an observational failure-threshold state, not an I2C admission
gate. The owner may still perform an explicit probe, retry, resync, or reset; a
successful tracked transaction returns health to `READY`. `probe()` remains a
raw diagnostic and does not itself clear `OFFLINE`.

`startResyncJob()` is the cooperative non-reset path. The legacy
`startRecoveryJob()` is an alias and reports `JobKind::RESYNC`. Synchronous
`recover()` also performs no reset. A successful `recover()` reloads
calibration, reapplies cached configuration, clears dirty state, and invalidates
cached raw and compensated samples. A failed `recover()` leaves pre-existing
cached samples unchanged; publish them only if your application explicitly
accepts stale data and records `sampleFreshness()`.

`startSoftResetJob()` is the separate explicit reset operation. Its first
callback writes `0xB6` to `0xE0`, then performs bounded NVM-copy readiness,
calibration reload, and configuration apply. Do not turn a transport or shared-
bus recovery automatically into a BME280 reset.

For a whole runtime settings change, build a `SensorSettings` value, call the
pure `validateSettings()`, then call zero-I2C `startApplySettingsJob()`. It uses
the same fixed APPLY_CONFIG phases as cached reapply. Before a config write has
any possible effect, failure/cancellation restores the prior cached settings.
After a successful or ambiguous write, the desired settings remain the explicit
resync target and `RESYNC_REQUIRED` blocks measurement until a full apply or
resync succeeds. This makes partial hardware state visible without asking the
application to duplicate the register sequence.

Raw register writes are diagnostics, not normal configuration APIs. Writes that
overlap `ctrl_hum`, `ctrl_meas`, `config`, or `reset` mark
`hardwareConfigDirty()` and require `recover()` or `begin()` before the cached
configuration should be trusted again.

The driver exposes no writable-NVM, calibration-trim, or factory-programming
API. It only waits for the BME280's internal calibration NVM copy to finish
after POR/reset and reads the image registers. Cooperative NVM polling is
bounded by `Config::nvmReadyTimeoutMs` and a fixed 255-status-callback cap.

## Operation Classes and Bounds

| Class | Operations | Bound |
| --- | --- | --- |
| Pure/zero-I2C | settings validation/timing/unit helpers, staged starts, `cancelJob()`, `end()`, cached snapshots | No transport callback |
| Cooperative | `pollJob(nowMs, budget)` | At most `budget` callbacks; zero budget may advance local-only phases |
| Synchronous compatibility | `begin()`, `recover()`, `softReset()`, setters, request/tick, register diagnostics | Fixed transaction shape; each callback receives `i2cTimeoutMs` |

Each measuring/idle readiness phase uses `conversionReadyTimeoutMs` plus a
fixed 255-poll counter and can perform at most one final status callback before
the poll-limit error. With no earlier phase deadline or transport failure, the
staged cumulative callback caps are 518 for init or non-reset resync, 519 for
explicit soft reset, 516 for config apply, 258 for a forced job starting from
known idle, and 514 when a forced job first reconciles an ambiguous conversion.
These are callback-count caps only. They do not define an overall application
deadline or elapsed-time guarantee; the bus-manager timeout, owner scheduling
cadence, queue time, and application deadline remain outside the library.

`readCalibrationRaw()` is a diagnostic/commissioning read rather than a
steady sampling operation. It is still fixed and bounded: one 26-byte burst for
`0x88..0xA1` followed by one 7-byte burst for `0xE1..0xE7`. The H1 byte is
`CalibrationRaw::tp[25]`; there is no redundant third transaction. The library
does not expose calibration or NVM writes, so write endurance, unknown write
completion, and maintenance retry policy do not arise in its public surface.

## HIL Evidence Expectations

Before claiming production hardware readiness for a shared-bus integration,
record evidence beyond host tests:

- exact firmware commit and dirty flag;
- board, MCU target, BME280 module, address strap, CSB state, rails, pull-ups,
  pins, and bus speed;
- `chipid` or `reg 0xD0` showing `0x60`;
- forced-mode sleep-return evidence through post-`force` `reg 0xF4` and
  `status`;
- repeated normal-mode reads if the application uses normal mode;
- reset/recover dirty-state evidence;
- protected fault-injection results if fault recovery is claimed;
- environmental reference readings and tolerances if accuracy is claimed;
- shared-bus contention or coexistence test notes for the other devices on the
  bus.

Use `docs/HARDWARE_VALIDATION.md` for the HIL procedure, evidence schema, and
result ledger. CI, PlatformIO builds, package checks, and Doxygen generation do
not prove physical bus margin, environmental accuracy, or field readiness.

For a shared-bus production claim, add application-level evidence:

- at least one other device active on the same bus during BME280 sampling;
- lock-timeout behavior when the bus manager is intentionally held too long;
- a bounded retry/backoff decision after BME280 address NACK or timeout;
- recovery after a fault injected by another bus client, if that claim is made;
- scheduler evidence that `tick()` still runs under expected application load;
- proof that no ISR directly calls BME280 public APIs.
