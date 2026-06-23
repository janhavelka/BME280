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
- The monotonic clock used by `Config::nowMs` and `tick(nowMs)`.
- Power/reset GPIOs and any bus recovery/reset procedure.
- Retry, backoff, degraded/offline policy, and field telemetry.

Driver-owned state:

- Cached BME280 configuration and calibration.
- Measurement scheduling state.
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

The transport callbacks should be small, synchronous, bounded, and non-recursive:

```cpp
struct BusContext {
  I2cHandle bus;
  Mutex* busMutex;
  uint32_t maxTimeoutMs;
};

BME280::Status bmeWrite(uint8_t addr,
                        const uint8_t* data,
                        size_t len,
                        uint32_t timeoutMs,
                        void* user) {
  auto* ctx = static_cast<BusContext*>(user);
  if (ctx == nullptr || data == nullptr || len == 0) {
    return BME280::Status::Error(BME280::Err::INVALID_PARAM,
                                 "Invalid I2C write");
  }

  const uint32_t boundedTimeout = clampTimeout(timeoutMs, ctx->maxTimeoutMs);
  LockGuard lock(*ctx->busMutex, boundedTimeout);
  if (!lock.locked()) {
    return BME280::Status::Error(BME280::Err::I2C_TIMEOUT,
                                 "I2C bus lock timeout");
  }

  I2cResult result = i2cTransmit(ctx->bus, addr, data, len, boundedTimeout);
  return mapI2cResult(result);
}

BME280::Status bmeWriteRead(uint8_t addr,
                            const uint8_t* txData,
                            size_t txLen,
                            uint8_t* rxData,
                            size_t rxLen,
                            uint32_t timeoutMs,
                            void* user) {
  auto* ctx = static_cast<BusContext*>(user);
  if (ctx == nullptr || txData == nullptr || rxData == nullptr ||
      txLen == 0 || rxLen == 0) {
    return BME280::Status::Error(BME280::Err::INVALID_PARAM,
                                 "Invalid I2C write-read");
  }

  const uint32_t boundedTimeout = clampTimeout(timeoutMs, ctx->maxTimeoutMs);
  LockGuard lock(*ctx->busMutex, boundedTimeout);
  if (!lock.locked()) {
    return BME280::Status::Error(BME280::Err::I2C_TIMEOUT,
                                 "I2C bus lock timeout");
  }

  I2cResult result = i2cTransmitReceive(ctx->bus, addr, txData, txLen,
                                        rxData, rxLen, boundedTimeout);
  return mapI2cResult(result);
}
```

The callback may lock the shared bus for the actual I2C transaction, but it does
not make the BME280 driver instance thread-safe by itself. Serialize public
driver calls with a driver mutex or by routing them through one owner task.

Preserve precise transport errors when possible:

- definite address NACK: `DEVICE_NOT_FOUND` or `I2C_NACK_ADDR` according to the
  adapter context;
- data NACK: `I2C_NACK_DATA`;
- transfer timeout: `I2C_TIMEOUT`;
- bus/arbitration fault: `I2C_BUS`;
- generic platform failure: `I2C_ERROR` or `I2C_BUS` with the raw code in
  `Status::detail`.

## Driver Initialization

```cpp
static BusContext busCtx = {/* bus, mutex, maxTimeoutMs */};
static BME280::BME280 bme;

BME280::Status initBme280() {
  BME280::Config cfg;
  cfg.i2cWrite = bmeWrite;
  cfg.i2cWriteRead = bmeWriteRead;
  cfg.i2cUser = &busCtx;
  cfg.nowMs = appMonotonicMs;
  cfg.timeUser = nullptr;
  cfg.i2cAddress = 0x76;     // 0x76 for SDO=GND, 0x77 for SDO=VDDIO
  cfg.i2cTimeoutMs = 50;     // finite transport timeout
  cfg.mode = BME280::Mode::FORCED;

  return bme.begin(cfg);     // verifies chip ID 0x60 and reads calibration
}
```

For I2C operation, tie CSB high to VDDIO before power-on reset. Do not leave SDO
floating. Do not drive SDA, SCL, SDO, or CSB high while VDDIO is off.

## Measurement Task

Use `requestMeasurement()` and scheduled `tick(nowMs)` instead of hidden waits:

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

`Config::nowMs` and `tick(nowMs)` must use the same monotonic timebase.
`requestMeasurement()` returns `INVALID_CONFIG` if no monotonic clock is
injected. In forced mode, the driver triggers one conversion and the device
returns to sleep. In normal mode, the driver waits one estimated normal cycle
before reading so the sample is fresh relative to the request.

`hasSample()` means the latest successful raw/compensated sample is cached; it
does not by itself prove freshness for the current request. Use
`sampleFreshness()` or `sampleFresh(nowMs, maxAgeMs)` before publishing data.
`STALE_AFTER_ERROR` means a later refresh failed or is still non-OK, and
`STALE_AFTER_CONFIG_DIRTY` means the hardware configuration may differ from the
driver cache.

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
5. Call `recover()` or `begin()` on the BME280 from task context.
6. Request a fresh sample before publishing BME280 data again.

## Recovery Policy

The driver tracks communication health for the current health session. A
successful `begin()` starts a new session and resets total success/failure
counters. Consecutive failures move the driver through `DEGRADED` and then
`OFFLINE` according to `Config::offlineThreshold`.

When `OFFLINE` is latched, normal public I2C APIs return `BUSY` without touching
the bus. `probe()` remains a raw diagnostic and does not clear the latch.
`recover()` is the explicit resync path. A successful `recover()` reloads
calibration, reapplies cached configuration, clears dirty state, and invalidates
cached raw and compensated samples. A failed `recover()` leaves pre-existing
cached samples unchanged; publish them only if your application explicitly
accepts stale data and records `sampleFreshness()`.

Raw register writes are diagnostics, not normal configuration APIs. Writes that
overlap `ctrl_hum`, `ctrl_meas`, `config`, or `reset` mark
`hardwareConfigDirty()` and require `recover()` or `begin()` before the cached
configuration should be trusted again.

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

Use `docs/I2C_HIL_RUNBOOK.md`, `docs/I2C_HIL_TARGET_TEMPLATE.md`, and
`docs/BME280_HARDWARE_VALIDATION_MATRIX.md` for formal hardware evidence. CI,
PlatformIO builds, package checks, and Doxygen generation do not prove physical
bus margin, environmental accuracy, or field readiness.

For a shared-bus production claim, add application-level evidence:

- at least one other device active on the same bus during BME280 sampling;
- lock-timeout behavior when the bus manager is intentionally held too long;
- a bounded retry/backoff decision after BME280 address NACK or timeout;
- recovery after a fault injected by another bus client, if that claim is made;
- scheduler evidence that `tick()` still runs under expected application load;
- proof that no ISR directly calls BME280 public APIs.
