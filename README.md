# BME280 Driver Library

Production-oriented BME280 I2C driver for ESP32 systems using
Arduino/PlatformIO or ESP-IDF.

Validation status: native tests, sanitizers, contract guards, package checks,
Arduino/PlatformIO builds, and native ESP-IDF builds run in CI. Retained
ESP32-S2/Arduino HIL includes a clean-source 61-minute campaign and an expanded
60-minute campaign plus post-correction gate; all completed with zero
classified `FAIL` or `TIMEOUT` rows and a verified final safe state. The driver
implementation and public contracts, excluding generated version metadata, are
byte-identical to the clean HIL commit. These results are functional evidence
for the release scope, not electrical or calibrated hardware qualification.
See `docs/HARDWARE_VALIDATION.md` for provenance, totals, and evidence
boundaries.

## Features

- **Injected I2C transport** - no Wire dependency in library code
- **Framework-neutral core** - Arduino and ESP-IDF integration live behind callbacks/adapters
- **Health monitoring** - observational state tracking (READY/DEGRADED/OFFLINE)
- **Deterministic behavior** - no unbounded loops, no heap allocations
- **Managed synchronous lifecycle** - visible reset/NVM readiness status and tick-driven measurement polling
- **Staged job API** - zero-I2C admission, exclusive hardware ownership, cancellation, identity, public phases, and bounded callback budgets for init, forced measurement, config apply, resync, and explicit soft reset

## Installation

### PlatformIO (recommended)

Add to `platformio.ini`:

```ini
lib_deps = 
  https://github.com/janhavelka/BME280.git#v2.1.0
```

Production consumers should pin an exact tag or commit. Do not use an unpinned
branch for a qualified firmware build.

### Manual

Copy `include/BME280/` and `src/` to your project.

### ESP-IDF Component

The repository root can be used as an ESP-IDF component. Add it to an IDF
project through `EXTRA_COMPONENT_DIRS` or your component manager workflow and
provide application-owned I2C callbacks through `BME280::Config`.

The core component does not configure pins, create I2C buses, log, or include
Arduino or ESP-IDF framework headers. Applications should inject `Config::nowMs`
for synchronous scheduling and timestamped diagnostics. During `pollJob(nowMs,
...)` and `tick(nowMs)`, the supplied argument is the authoritative time for
that call, including health updates made by its transport callbacks.

See `examples/idf/basic` for a native ESP-IDF `i2c_master` adapter and
`app_main` CLI. The ESP-IDF example preserves the Arduino CLI command contract
without including Arduino source or compatibility facades.

## Quick Start

```cpp
#include <Wire.h>
#include "BME280/BME280.h"
#include "examples/common/I2cTransport.h"

BME280::BME280 device;

uint32_t appNowMs(void*) {
  return millis();
}

void setup() {
  Serial.begin(115200);
  if (!transport::initWire(8, 9, 400000, 50)) {
    Serial.println("I2C initialization failed");
    return;
  }
  
  BME280::Config cfg;
  cfg.i2cWrite = transport::wireWrite;
  cfg.i2cWriteRead = transport::wireWriteRead;
  cfg.i2cUser = &Wire;
  cfg.nowMs = appNowMs;
  cfg.i2cAddress = 0x76;
  cfg.i2cTimeoutMs = 50;
  cfg.nvmReadyTimeoutMs = 10;
  cfg.conversionReadyTimeoutMs = 20;
  
  auto status = device.begin(cfg);
  if (!status.ok()) {
    Serial.printf("Init failed: %s\n", status.msg);
    return;
  }
  
  Serial.println("Device initialized!");
}

void loop() {
  device.tick(millis());
  
  // Your code here
}
```

The example adapter returns terminal `TransportResult` values and maps Arduino
`Wire` failures to address NACK, data NACK, timeout, bus, or other transport
causes. Every callback makes exactly one physical attempt; write-read uses a
combined pointer write and repeated START, and success requires exact byte
counts. Adapters must not retry, recover the bus, or return driver-level
`Status` values. The core maps results to canonical `I2C_*` status codes and
retains only numeric adapter detail, never adapter-owned message storage.

Bus timeout ownership in the Arduino example remains in
`transport::initWire()`: its callbacks do not reconfigure `Wire` from the
per-call `timeoutMs` argument. Keep `Config::i2cTimeoutMs` equal to that fixed
Wire timeout, as the quick start does. A production shared-bus callback must
treat `i2cTimeoutMs` as one end-to-end budget covering lock acquisition and the
transfer. The separate `conversionReadyTimeoutMs` is chip-level grace after the
estimated conversion or idle time. Inject `Config::nowMs` for synchronous
scheduling and meaningful timestamps; absent an injected or explicit poll/tick
time, timestamp values are zero and the matching validity flag is false.
`examples/common/I2cTransport.h` is example-only glue; when manually copying only `include/` and
`src/`, provide equivalent `Config::i2cWrite` and `Config::i2cWriteRead` callbacks in
your application.

For optional sensor slots, only a definite `TransportErr::NACK_ADDRESS` during
a chip-ID presence check becomes `DEVICE_NOT_FOUND`. A phase-ambiguous NACK,
data NACK, bus error, transaction timeout, short transfer, or generic I2C fault
remains a transport fault and must not be collapsed into optional absence.

## Hardware Integration Notes

- I2C address is selected by SDO: `0x76` when SDO is tied to GND, `0x77` when SDO is tied to VDDIO. Do not leave SDO floating.
- For I2C operation, tie CSB high to VDDIO before power-on reset. Driving CSB low can select SPI mode and interfere with I2C until POR.
- BME280 has separate VDD and VDDIO rails. Do not drive SDA, SCL, SDO, or CSB high while VDDIO is off.
- Use external SDA/SCL pullups to VDDIO for production hardware. Example internal pullups are bring-up aids, not a production signal-integrity design.
- Place local decoupling close to the sensor VDD/VDDIO/GND pins and keep humidity exposure away from condensation, contamination, and board heat sources.

### Shared Bus Guidance

The library does not own I2C. A production shared-bus application should own
the bus handle, pins, clock, pullups, timeout policy, reset/recovery procedure,
and lock/mutex outside the driver. Serialize every driver call that can touch
I2C, do not call driver APIs from ISRs, and call `tick()` from a task or
scheduler that is not blocked by console input. Transport callbacks must
complete synchronously and must not recursively call into the same
`BME280::BME280` instance.

See `docs/PRODUCTION_SHARED_BUS_GUIDE.md` for a production integration pattern
covering an application-owned bus manager, driver-instance serialization,
finite transfer timeouts, scheduled measurement polling, shared-bus recovery,
and HIL evidence expectations.

### Humidity Handling

Humidity readings are for non-condensing environments. Soldering/reflow, contamination, condensation, or storage outside the operating range can temporarily shift humidity output. Treat the broad example self-test humidity range as a smoke check only; use application-specific plausibility limits and follow Bosch handling/reconditioning guidance for production calibration work.

## Health Monitoring

The driver tracks communication and device-resynchronization health:

```cpp
// Check state
if (device.state() == BME280::DriverState::OFFLINE) {
  Serial.println("Failure threshold reached");
  // OFFLINE is observational. The application still chooses whether and when
  // to probe, retry, resynchronize, reset, or stop using the device.
  BME280::Status st = device.recover();  // Non-reset synchronous resync
  if (!st.ok()) {
    Serial.println(st.msg);
  }
}

// Get current health-session statistics
Serial.printf("Failures: %u consecutive, %lu session total\n",
              device.consecutiveFailures(), device.totalFailures());
```

`begin()` starts a new health session and resets the total success/failure
counters. Successful tracked transfers increment `totalSuccess()`. Transport
failures, plus semantic chip-ID/calibration/NVM/config-readiness failures found
during recovery, reset, or staged resynchronization, increment
`totalFailures()` and may move the driver to `DEGRADED` or `OFFLINE`.
`totalSuccess()` and `totalFailures()` saturate at `UINT32_MAX`;
`consecutiveFailures()` saturates at `UINT8_MAX` and resets to zero on the next
tracked transfer success. The counters do not wrap.

### Driver States

| State | Description |
|-------|-------------|
| `UNINIT` | No successful current session: never initialized, ended, or an admitted `begin()` / init job failed |
| `READY` | Initialized session with zero consecutive tracked failures; does not imply measurement readiness |
| `DEGRADED` | 1+ failures, below offline threshold |
| `OFFLINE` | Failure threshold reached; diagnostic only and does not block an explicit owner-directed operation |

These states describe tracked communication and semantic failures. A successful
tracked transfer can return health to `READY` while configuration remains dirty
or calibration remains invalid. Check the operation's returned `Status`;
`isOnline()` does not guarantee that a measurement can be requested.

## API Reference

### Lifecycle

- `Status begin(const Config& config)` - Reset local runtime state, initialize the driver, verify chip ID `0x60`, request sleep and confirm idle, check NVM readiness once, read calibration, apply config, and start a new health session
- `void tick(uint32_t nowMs)` - Process pending compatibility measurement operations; the supplied time is authoritative for this call and must share the application's monotonic timebase
- `void end()` - Zero-I2C, idempotent unbind that clears callbacks and cached runtime state; it does not put the sensor to sleep
- `bool isInitialized()` - True only while the current initialization session remains valid; an admitted failed `begin()` or init job clears it
- `const Config& getConfig()` - Cached configuration snapshot owned by the driver
- `SensorSettings sensorSettings()` - Compact zero-I2C snapshot of typed chip settings, separate from transport/time/health policy

### Diagnostics

- `Status probe()` - Check device presence and chip ID through raw I2C without health tracking
- `Status recover()` - Synchronous non-reset resync; verifies identity, requests sleep and confirms idle before checking NVM readiness, reloads calibration, reapplies cached config, and invalidates cached samples on success
- `Status invalidateDeviceState()` - Zero-I2C owner boundary for removal, hotplug, or possible device replacement; measurements remain blocked until full initialization/resync reloads identity, calibration, and configuration
- `Status getSettings(SettingsSnapshot& out)` - Populate a snapshot of cached config and runtime state (no I2C)
- `Status lastMeasurementStatus()` - Last measurement request, polling, raw-read, or compensation status retained because `tick()` is void

### Staged I2C Jobs

- `Status startInitJob(const Config& config)` - Reset local runtime state, validate/cache configuration, and start chunked initialization without I2C
- `Status startForcedMeasurementJob()` - Start a chunked forced-mode sample without I2C
- `Status startApplyConfigJob()` - Start a cached-config apply without I2C
- `Status startApplySettingsJob(const SensorSettings& settings)` - Validate and stage a whole typed settings update without I2C, then reuse the bounded apply job
- `Status startResyncJob()` - Start identity verification, bounded sleep/idle and NVM readiness, calibration reload, and config re-apply without resetting the sensor
- `Status startRecoveryJob()` - Compatibility alias for `startResyncJob()`; reports `JobKind::RESYNC` and does not reset
- `Status startSoftResetJob()` - Start an explicit `0xE0 = 0xB6` reset followed by full resynchronization
- `Status cancelJob(CancelReason reason)` - Cancel the active job without I2C; use `OWNER_REQUEST` or `DEADLINE_EXPIRED`
- `JobPollResult pollJob(uint32_t nowMs, uint8_t maxInstructions = 1)` - Advance the active job by at most the supplied callback budget

Every accepted start receives a nonzero `jobId`. `JobPollResult` reports that
identity, `JobKind`, public `JobPhase`, `JobState`, `Status`,
`ConversionState`, optional chip-phase deadline, and `callbacksUsed`
(`instructionsUsed` is a compatibility alias). A natural `DONE` or `FAILED`
result is returned by the single poll that reaches it; the next poll is idle and
a new start may be accepted immediately. A cancellation result is retained for
exactly one `pollJob()` retrieval. Until that retrieval, new jobs and fallible
hardware-facing synchronous APIs return `BUSY` with
`BusyReason::TERMINAL_RESULT_PENDING` and perform no I2C; `tick()` also performs
no I2C, cached reads remain available, and `end()` remains available for
zero-I2C teardown.

Start, cancel, and `end()` perform no transport callback. While a job is
`RUNNING` or `WAITING`, it exclusively owns hardware access: another job start
or a fallible synchronous hardware-facing API returns `BUSY` with
`BusyReason::STAGED_JOB_ACTIVE`; `tick()` performs no I2C. Cached snapshots and
zero-I2C `end()` remain available. The
application must retain its own end-to-end deadline, including queue time; an
expired owner deadline is represented by
`cancelJob(CancelReason::DEADLINE_EXPIRED)`. `phaseDeadlineMs` is only the
driver's current chip-phase deadline and must not replace or renew the owner
deadline.

`begin()` and `startInitJob()` are initialization boundaries, not transactional
reconfiguration calls. Once admitted, each clears the previous local runtime
state before validating the new configuration or touching I2C. If validation,
identity, calibration, or configuration then fails, the instance remains
`UNINIT`; the previous initialized session and cached sample are not restored.
Use typed setters or `startApplySettingsJob()` for runtime settings changes.

### Measurement

- `Status requestMeasurement()` - Start a forced measurement or schedule a fresh normal-mode cycle; returns `IN_PROGRESS` when accepted
- `bool measurementReady()` - True after `tick()` captures and compensates a fresh unread sample
- `Status getMeasurement(Measurement& out)` - Consume a fresh unread floating-point measurement; stale cached data is never returned as success
- `Status getRawSample(RawSample& out)` - Get the latest raw ADC sample plus per-channel validity flags after at least one capture
- `Status getCompensatedSample(CompensatedSample& out)` - Get fixed-point compensated values plus per-channel validity flags after at least one capture
- `Status getSampleEnvelope(SampleEnvelope& out)` - Get the atomically committed raw/fixed-point sample, timestamp, sequence, and config generation
- `SampleFreshness sampleFreshness()` - Classify the cached sample as `NONE`, `FRESH`, `STALE_AFTER_ERROR`, `STALE_AFTER_CONFIG_DIRTY`, or `STALE_AFTER_CONFIG_CHANGE`
- `bool sampleFresh(uint32_t nowMs, uint32_t maxAgeMs)` - True only when a cached sample is fresh and within the caller's age budget
- `Status getCalibration(Calibration& out)` - Return cached calibration coefficients
- `Status readCalibrationRaw(CalibrationRaw& out)` - Read the complete calibration image in exactly two bursts; `tp[25]` is register `0xA1` / `dig_H1`

Forced mode is an on-demand policy: `begin()` and `setMode(FORCED)` keep the hardware in
sleep until `requestMeasurement()` writes the forced-mode trigger. Normal-mode requests
wait the worst phase interval (two maximum configured conversions plus one standby
interval) before reading registers, so the returned sample is fresh relative to the
request even if it arrived just after a conversion started. Ambiguous existing forced
conversions receive one complete configured conversion interval plus readiness grace.
All of these deadlines remain wrap-safe across `uint32_t` rollover.

A terminal status-read, raw-read, or compensation error ends the compatibility
request. Later `tick()` calls perform no I2C until the owner explicitly calls
`requestMeasurement()` again. If configuration becomes dirty, any pending/unread
readiness is cleared immediately while the last `SampleEnvelope` remains available
and is classified stale for diagnostics.

`ConversionState` exposes whether a forced conversion is `IDLE`, known
`IN_PROGRESS`, or `UNKNOWN_AFTER_TRIGGER_ERROR`. A trigger timeout or a
cancellation after the trigger may mean the write reached the chip; the driver
does not replay that write. The next staged forced job first reads
`status.measuring` in `FORCE_RECONCILE_STATUS`, waits if necessary, and issues a
new trigger only after the prior state is known idle. A synchronized steady
forced sample writes only `ctrl_meas`, then checks status and burst-reads data;
`ctrl_hum` is written during configuration apply, not before every sample.

Raw and fixed-point outputs are first-class. `CompensatedSample::pressurePa` is
integer Pascals for control and telemetry paths; `Measurement` is a float
convenience view over cached compensated data.

`tick()` does not return a `Status`. Measurement scheduler and capture results are
retained in `lastMeasurementStatus()` and `SettingsSnapshot::lastMeasurementStatus`.
The value is `IN_PROGRESS` while a request is pending, `OK` after a sample is
captured, and the original transport or compensation error when a polling,
burst-read, or compensation step fails.

Raw and fixed-point cached samples remain readable after later refresh failures
unless a successful resync invalidates them. Use `sampleFreshness()` or
`SettingsSnapshot::sampleFreshness` to distinguish fresh samples from
stale-but-readable samples after errors or uncertain hardware configuration.

### Configuration

- `Status validateSettings(const SensorSettings& settings)` - Pure, zero-I2C validation of enum values and channel dependencies.
  On failure the status is `INVALID_PARAM` and `Status::detail` carries a
  `SettingsValidationReason` naming the first offending field in declaration
  order (`OSRS_T`, `OSRS_P`, `OSRS_H`, `FILTER`, `STANDBY`, `MODE`,
  `SELECTION`). `begin()` and the apply paths propagate the same detail.
- `Status setMode(Mode mode)` - Select `SLEEP`, `FORCED`, or `NORMAL`
- `Status setOversamplingT/P/H(Oversampling osrs)` - Configure temperature, pressure, or humidity oversampling
- `Status setFilter(Filter filter)` - Configure the IIR filter coefficient through a safe sleep/config/restore sequence
- `Status setStandby(Standby standby)` - Configure standby interval for normal mode through a safe sleep/config/restore sequence
- `Status softReset()` - Write `0xB6` to reset register `0xE0`, check NVM readiness once, reload calibration, reapply cached config, and invalidate cached samples
- `Status readChipId/readStatus/readCtrlHum/readCtrlMeas/readConfig(...)` - Read status/config registers
- `Status isMeasuring(bool& measuring)` - Read the measuring bit

Temperature oversampling must be enabled whenever pressure or humidity is enabled because
Bosch compensation requires `t_fine`. At least one measured channel must be enabled.
Invalid combinations are rejected in `begin()` and typed setters before touching I2C.

Configuration validation also requires address `0x76` or `0x77`; nonzero
`i2cTimeoutMs` and `nvmReadyTimeoutMs` values below `INT32_MAX`; and a nonzero
`conversionReadyTimeoutMs` small enough that the maximum measurement plus
standby interval remains wrap-safe. An `offlineThreshold` of zero is normalized
to one in the driver's cached configuration.

`startApplySettingsJob()` stages the desired settings in the same
`APPLY_CONFIG` state machine used by `startApplyConfigJob()`; it does not create
a second configuration engine. A failure or cancellation before any config
write succeeds or may have reached the device restores the prior cached
settings and synchronization state. Once a write has a possible hardware
effect, the desired settings remain the explicit resync target and
`ConfigSyncState::RESYNC_REQUIRED` prevents measurements until a complete
apply/resync succeeds. The last-good sample remains readable only with its
existing freshness/generation provenance.

The staged path is a whole-tuple apply: it always writes `config` (`0xF5`),
`ctrl_hum` (`0xF2`) and `ctrl_meas` (`0xF4`), so it resets the BME280 IIR filter
history even for a mode-only change. The synchronous single-field setters are
selective because they change one field: `setMode()` and `setOversamplingT/P()`
write only `ctrl_meas`, `setOversamplingH()` writes `ctrl_hum` then `ctrl_meas`,
and only `setFilter()`/`setStandby()` touch `config`. Use a setter when filter
memory must be preserved.

Every typed settings change first queues `ctrl_meas` sleep, then confirms that
`status.measuring` is clear before writing any sleep-only register. Humidity
oversampling follows the Bosch latch rule: `setOversamplingH()` then writes
`ctrl_hum` followed by `ctrl_meas`. Filter and standby changes write `config`,
then restore normal mode when required. If the post-sleep status check remains
busy, later writes are skipped and `hardwareConfigDirty()` is set because
hardware mode may no longer match the cache. A final `0xF2..0xF5` readback must
match all driver-owned settings bits before the cache is committed. A mismatch
returns `RESYNC_REQUIRED` without committing the cache, and packs the evidence
into `Status::detail` as `0x00RREEAA` - register address in bits 16-23 (`0xF2`
ctrl_hum, `0xF4` ctrl_meas, `0xF5` config), expected driver-owned bits in 8-15,
actual driver-owned bits in 0-7. The same status is retained in
`hardwareConfigDirtyError()`. Successful typed configuration changes invalidate
cached samples so callers cannot read a sample captured under old settings.

If a multi-register configuration sequence touches hardware and then fails, the
driver sets `hardwareConfigDirty()` and preserves the original error in
`hardwareConfigDirtyError()` and `SettingsSnapshot::hardwareConfigDirtyError`.
The dirty flag is cleared only by a complete successful resync through
`begin()`, `recover()`, `softReset()`, or the equivalent successful staged
init/apply-config/resync/soft-reset job. `startRecoveryJob()` is the legacy name
for non-reset resync; only `softReset()` or `startSoftResetJob()` intentionally
writes the reset register.

### Probe, Begin, Recover, and Reset Diagnostics

`begin()` reads chip ID register `0xD0` once after configuration validation.
Applications should call it after the BME280 has completed POR and is reachable
on I2C. A value other than `0x60` returns `CHIP_ID_MISMATCH` with the observed
ID in `Status::detail`. A definite address NACK maps to `DEVICE_NOT_FOUND`;
timeouts, bus errors, data NACK, and generic I2C errors are preserved when the
transport reports them.

`probe()` is diagnostic-only. It uses raw I2C so it does not update health
counters or clear an `OFFLINE` latch. It maps only definite address NACK to
`DEVICE_NOT_FOUND`; other transport errors and chip-ID mismatch are returned
unchanged.

`begin()` and `recover()` request sleep and confirm the device is idle before
checking status bit `im_update`, so a normal-mode conversion cannot start a new
calibration-image copy between readiness and the calibration bursts.
`softReset()` checks the same bit after reset, when the device is already
quiescent. If a transition or NVM copy is still busy, the synchronous paths
return `BUSY` or `TIMEOUT` instead of hiding a polling loop. Transport errors
from status reads are preserved. Owners that need repeated readiness polling
without exceeding a per-poll callback budget should use `startInitJob()`,
`startResyncJob()`, or `startSoftResetJob()` and advance it with `pollJob()`.

The BME280 exposes factory calibration through read-only image registers after
the chip copies its internal NVM following POR or reset. This library exposes
no writable-NVM, trimming, or factory-programming API. Staged NVM-copy waiting
is bounded by `nvmReadyTimeoutMs` and the fixed 255-status-poll cap.

After a successful reset write, hardware config may be back at defaults. If an
NVM readiness check, calibration reload, validation, or config reapply fails,
`hardwareConfigDirty()` remains set with the root-cause status. A successful
`softReset()` or `recover()` reloads calibration, invalidates cached samples,
and clears dirty state only after the full cached configuration is reapplied.
Runtime reset/recover NVM readiness checks use health-tracked I2C, so a
status-read transport failure can move the driver to `DEGRADED` or `OFFLINE`
according to the configured failure threshold while preserving the root-cause
`Status`.

A failed synchronous `recover()` leaves any pre-existing cached sample
unchanged. Successful synchronous `recover()` and any synchronous `softReset()`
attempt invalidate cached raw and compensated samples. Staged config/resync
completion instead advances the config generation, so a preserved last-good
sample remains explicitly stale under its prior generation.

### Public I2C Transaction Shape

| API | Typical I2C transactions | Notes |
|-----|--------------------------|-------|
| `begin()` | chip ID read, sleep write, idle-status read, one NVM status read, calibration reads, full settings writes, settings readback | Quiesces first so calibration cannot race a normal-mode image copy; staged init owns repeated polling |
| `requestMeasurement()` in forced mode | status read, `ctrl_meas` write | Returns `IN_PROGRESS` when accepted |
| `tick()` after deadline | status read, one `0xF7..0xFE` burst read | Captures coherent pressure, temperature, and humidity ADC bytes |
| `setMode()` / `setOversamplingT/P()` | sleep/desired `ctrl_meas` write, idle-status read, optional normal-mode restore, settings readback | Invalid settings are rejected before I2C; these setters never write `config` and therefore do not reset IIR history |
| `setOversamplingH()` | sleep/desired `ctrl_meas` write, idle-status read, `ctrl_hum` write, `ctrl_meas` latch, settings readback | `ctrl_meas` latches humidity oversampling |
| `setFilter()` / `setStandby()` | sleep write, idle-status read, `config` write, optional normal-mode restore, settings readback | Skips `config` and marks dirty if measuring persists after the sleep request |
| `recover()` | chip-ID read, sleep/idle sequence, one NVM status read, calibration reads, full settings writes and readback | Non-reset compatibility helper; OFFLINE does not block it |
| `softReset()` | reset write, one NVM status read, calibration reads, sleep/idle sequence, full settings writes and readback | Marks dirty if reset succeeds but any later step fails |
| Staged job starts / `cancelJob()` / `end()` | none | Zero-I2C state transitions |
| `pollJob(nowMs, budget)` | at most `budget` callbacks | Zero budget still permits bounded local-only phase transitions |

### Blocking Latency Bounds

All transport callbacks are synchronous and individually bounded by
`Config::i2cTimeoutMs`. The API has three operation classes:

- zero-I2C control and cached inspection, including staged starts,
  `cancelJob()`, `end()`, and snapshots;
- cooperative staged work, where each `pollJob()` call issues no more than its
  `uint8_t` callback budget;
- synchronous compatibility/diagnostic work, whose fixed transaction shapes
  are listed below.

| API | Blocking bound |
|-----|----------------|
| `begin()` | A fixed sleep-first sequence including one idle check, one NVM status read, and verified full settings apply; each transport callback receives `Config::i2cTimeoutMs` |
| `probe()` | One chip-ID register read through raw I2C, bounded by `Config::i2cTimeoutMs` |
| `requestMeasurement()` | Forced mode performs one status read and one mode write; normal mode schedules work and returns |
| `tick()` | Before deadline: no I2C. After deadline: one status read and one `0xF7..0xFE` burst read |
| Typed setters | Sleep/desired write, one idle-status read, only the needed settings writes, and one `0xF2..0xF5` readback |
| `recover()` | Chip-ID read, sleep/idle transition, one NVM status read, calibration reload, and verified full settings resync |
| `softReset()` | Reset write, one NVM status read, calibration reload, and verified settings resync |

Staged readiness is bounded twice: by wrap-safe chip-phase deadlines and by
fixed counters. NVM readiness permits at most 255 status callbacks. Each
measuring/idle wait uses a 255-poll counter and can perform at most one final
status callback before reporting the poll-limit timeout. With no earlier
deadline or transport failure, the resulting worst-case callback counts are
519 for init or non-reset resync, 520 for explicit soft reset, 261 for config
apply, 258 for a forced job starting from known idle, and 514 for a forced job
that must first reconcile an ambiguous prior trigger. These are library
callback caps, not elapsed-time guarantees and not an application owner
deadline; callback duration and owner scheduling cadence remain external.

Sample numeric units are stable: `Measurement` returns degrees Celsius, Pascals,
and percent RH; `CompensatedSample` returns `tempC_x100`, integer Pascals, and
`humidityPct_x1024` (Q22.10). Skipped or invalid channels keep numeric fields at
zero for compatibility, so callers must check `temperatureValid`,
`pressureValid`, and `humidityValid` before using a channel. The raw Bosch
skipped sentinels are exposed as `cmd::RAW_PRESSURE_SKIPPED`,
`cmd::RAW_TEMPERATURE_SKIPPED`, and `cmd::RAW_HUMIDITY_SKIPPED`.
Raw validity follows configured oversampling: the same numeric encoding remains
a valid ADC result when its channel is enabled.

Calibration coefficients are read from `0x88..0xA1` and `0xE1..0xE7` during
`begin()`. `dig_T1` and `dig_P1` are unsigned 16-bit values; the other
temperature/pressure coefficients are signed 16-bit values. `dig_H4` and
`dig_H5` are signed 12-bit humidity coefficients packed across `0xE4`, `0xE5`,
and `0xE6`. Compensation follows the Bosch integer flow: temperature is computed
first to produce `t_fine`; pressure uses the 64-bit path with a divide-by-zero
guard; humidity is clamped to `0..100%RH`.

The IIR filter affects pressure and temperature only; humidity is not filtered
by the BME280 IIR path. Changing filter settings resets the hardware filter
memory, so the next sample seeds the filter again.

### Raw Register Access

- `Status readRegisters(uint8_t startReg, uint8_t* buf, size_t len)` - Read a contiguous tracked register block
- `Status writeRegisters(uint8_t startReg, const uint8_t* buf, size_t len)` - Write a contiguous tracked register block
- `Status readRegister(uint8_t reg, uint8_t& value)` - Read a single tracked register
- `Status writeRegister(uint8_t reg, uint8_t value)` - Write a single tracked register

A multi-register write is encoded as repeated register-address/value pairs,
because the BME280 write protocol does not auto-increment the register pointer
(datasheet section 6; reads do auto-increment). `writeRegisters(0xF2, buf, 3)`
therefore transmits `F2 v0 F3 v1 F4 v2`, not `F2 v0 v1 v2`, and the register
range must not wrap past `0xFF`.

Raw writes are diagnostic tools. Writes that overlap `ctrl_hum` (`0xF2`),
`ctrl_meas` (`0xF4`), `config` (`0xF5`), or `reset` (`0xE0`) are health-tracked
and mark `hardwareConfigDirty()` on success because they bypass the typed config
cache. Transport failures that may have partially reached those registers
preserve the original status in `hardwareConfigDirtyError()`. Call `recover()`,
`begin()`, a successful `softReset()`, or the corresponding staged resync/reset
job to restore synchronization after manual register edits.

### State

- `DriverState state()` - Current driver state
- `bool isOnline()` - True if READY or DEGRADED; does not probe reachability or guarantee synchronized settings, valid calibration, or measurement readiness

### Health

- `uint32_t lastOkMs()` - Timestamp of last success
- `bool lastOkTimeValid()` - Whether `lastOkMs()` came from an explicit or injected time source
- `uint32_t lastErrorMs()` - Timestamp of last failure
- `bool lastErrorTimeValid()` - Whether `lastErrorMs()` came from an explicit or injected time source
- `Status lastError()` - Most recent error
- `uint8_t consecutiveFailures()` - Failures since last success
- `uint32_t totalFailures()` - Tracked failure count in the current health session
- `uint32_t totalSuccess()` - Tracked success count in the current health session

`begin()` starts a new health session and resets the tracked success/failure
counters. Total success/failure counters saturate at `UINT32_MAX`, and
consecutive failures saturate at `UINT8_MAX`; they do not wrap. `IN_PROGRESS`
is treated as non-failure activity. In an initialized session, recovery/reset
and staged resynchronization also record semantic chip-ID, calibration, NVM,
and config-readiness failures even when the immediately preceding transport
callback succeeded. Pre-`begin()` validation and transport setup errors do not
transition the driver into `DEGRADED` or `OFFLINE`.

### Timing

- `uint32_t estimateMeasurementTimeUs(const SensorSettings&)` - Pure exact Bosch maximum-duration formula, without scheduling margin
- `uint32_t estimateMeasurementTimeMs(const SensorSettings&)` - Pure rounded scheduler estimate with the fixed 1 ms library margin
- `Status temperatureX100ToMilliC(...)` - Checked centi-Celsius to signed milli-Celsius conversion
- `Status humidityX1024ToMilliPercent(...)` - Checked Q22.10 percent to signed milli-percent conversion, rejecting input above 100%
- `bool isBme280ChipId(uint8_t)` - Pure identity check for chip ID `0x60`
- `uint32_t estimateMeasurementTimeUs()` - Exact Bosch maximum for current cached settings
- `uint32_t estimateMeasurementTimeMs()` - Max measurement time for current oversampling
- `uint32_t getStandbyTimeMs()` - Configured standby interval in ms
- `uint32_t estimateNormalCycleMs()` - Full normal-mode cycle (measurement + standby)

`Config::nvmReadyTimeoutMs` controls the visible NVM-ready deadline after POR or
reset. The synchronous check performs one status-register transaction per call
and returns `BUSY`, `TIMEOUT`, or the detailed transport error instead of hiding
a tight polling loop.

Measurement timing uses the Bosch oversampling multipliers `SKIP=0`, `X1=1`,
`X2=2`, `X4=4`, `X8=8`, and `X16=16`:

```text
t_meas_us = 1250
          + (temperature enabled ? 2300 * osrs_t : 0)
          + (pressure enabled ? 2300 * osrs_p + 575 : 0)
          + (humidity enabled ? 2300 * osrs_h + 575 : 0)
estimateMeasurementTimeUs = t_meas_us
estimateMeasurementTimeMs = ceil((t_meas_us + 1000 safety margin) / 1000)
estimateNormalCycleMs = estimateMeasurementTimeMs + getStandbyTimeMs()
```

The public timing accessors above report the configured nominal standby. The
driver's internal normal-mode freshness budget uses a larger figure: the
datasheet specifies standby time accuracy as typ +/-5% and max +/-25%, so the
budget reserves the worst-case standby (for example 1250 ms for `MS_1000` and
79 ms for `MS_62_5`). Every other term in that budget is already the Bosch
maximum, so standby was the only untoleranced quantity.

## Examples

- `01_basic_bringup_cli/` - Arduino/PlatformIO bring-up and diagnostic CLI; not a production firmware template
- `idf/basic/` - Native ESP-IDF bring-up and diagnostic CLI using `app_main`, `driver/i2c_master.h`, FreeRTOS timing, fixed command buffers, and the same user-facing CLI workflow as Arduino
- Complete typed settings commands are available in both CLIs. `settings values` lists the accepted named/numeric grammar and channel constraints; `settings validate <mode> <t> <p> <h> <filter> <standby>` performs pure validation; `settings start ...` starts the zero-I2C staged apply; and `settings set ...` runs that bounded job to completion. The individual `mode`, `osrs`, `filter`, and `standby` commands accept the same names and numeric enum codes. These six fields are the complete I2C-mode `SensorSettings` surface. The address is an SDO hardware/config choice, and SPI three-wire mode is intentionally not a typed setting.
- Setting and diagnostic parsers require complete tokens and exact command arity; malformed numbers, trailing text, and unsupported enum codes are rejected before a setting command performs I2C. `stress` and `stress_mix` accept bounded decimal counts from 1 through 100000.
- The native ESP-IDF fixed-buffer frontend drains and rejects an entire overlong
  input line as `Command too long`; it never executes a valid-looking prefix.
- CLI register diagnostics include tracked `reg <addr>`, bounded single-transaction `dump`/`rregs <addr> <1..32>`, and diagnostic `wreg <addr> <val>`. Raw config/reset writes bypass typed helpers, mark dirty state, and require `recover()` or `begin()` after manual edits. Destructive raw writes are not part of default HIL automation.
- Lifecycle/provenance diagnostics include zero-I2C `end`, zero-I2C `invalidate`, and cache-only `freshness [max_age_ms]`. Example-only `xfer_reset`, `xfer_stats`, and `xfer_assert <read> <write> <total>` expose saturating counts of validated transport callback attempts for deterministic HIL checks; they are not library APIs and do not themselves access I2C.
- Other CLI diagnostics include `addr [0x76|0x77]`, `id`/`chipid`, `status`, `calib`, `force`, `normal on/off`, `reset`, `probe`, `recover`, `selftest`, `stress N`, and `stress_mix N`. `selftest` is a safe command smoke check with loose environment-dependent plausibility ranges; it is not factory calibration or hardware qualification.
- No-argument settings queries read the chip registers and show the cached
  internal value so divergence is visible. Mixed stress and self-test restore
  and verify the complete pre-command sensor settings; a restoration failure is
  a reported command failure, not a warning-only result.

### Example Helpers (`examples/common/`)

Not part of the library. These simulate project-level glue and keep examples self-contained:

| File | Purpose |
|------|---------|
| `BoardConfig.h` | Pin definitions and Wire init for supported boards |
| `BuildConfig.h` | Compile-time `LOG_LEVEL` configuration |
| `Log.h` | Serial logging macros (`LOGE`/`LOGW`/`LOGI`) |
| `I2cTransport.h` | Wire-based I2C adapter plus example-only saturating callback counters |
| `I2cScanner.h` | I2C bus scanner with table output |
| `CliStyle.h` | Shared ANSI colors and CLI formatting helpers |
| `HealthView.h` | Compact health status display |

## Behavioral Contracts

1. Threading model: single-threaded by default; not thread-safe.
2. Timing model: `pollJob(nowMs, ...)` and `tick(nowMs)` use the supplied time for chip phases and health events during that call. Synchronous scheduling uses `Config::nowMs`; all sources must share one monotonic timebase.
3. Resource ownership: bus, pins, and timeout policy remain application-owned via `Config`.
4. Memory behavior: no heap allocation in steady-state library operation.
5. Error handling: all fallible APIs return `Status`; no exceptions and no silent failures.
6. Health behavior: `OFFLINE` is an observational threshold state, not an admission gate. Explicit owner-directed operations may touch I2C; a tracked success returns health to `READY`. `probe()` remains raw diagnostic I2C and does not itself clear `OFFLINE`.
7. Measurement scheduling requires `Config::nowMs`. `begin()` does not fail without it, but `requestMeasurement()` returns `INVALID_CONFIG` if no monotonic clock is injected.
8. Multi-register configuration failures and successful diagnostic raw writes to config/control/reset registers set `hardwareConfigDirty()` and expose the dirty-state cause in `hardwareConfigDirtyError()` and `SettingsSnapshot`.
9. Driver instances are not thread-safe and public APIs are not ISR-safe. Shared-bus users must serialize access externally.
10. Typed setters first queue sleep without a status pre-read. If the single post-sleep status read still reports `measuring`, later settings writes are skipped and dirty state is set.
11. `probe()` is diagnostic-only and preserves timeout, bus, data-NACK, and generic I2C errors. `DEVICE_NOT_FOUND` is reserved for definite address NACK.
12. A running staged job exclusively owns hardware access. Cancellation is zero-I2C and its terminal result must be retrieved once before later hardware work.
13. Synchronous reset/resync NVM readiness checks perform one status read and return visible `BUSY`, `TIMEOUT`, or the original transport error. Bounded repeated NVM polling belongs to staged jobs advanced by `pollJob()`.
14. Health timestamp values are meaningful only when `lastOkTimeValid()` / `lastErrorTimeValid()` (or the snapshot flags) are true.

## Migration From 1.x to 2.x

These are the breaking changes introduced at the `2.0.0` boundary; they still
apply when upgrading from any `1.7.x` release.

- Transport callbacks now return terminal-only `TransportResult`, not driver
  `Status`. Return exact physical write/read counts; perform one physical
  attempt; use a combined repeated-start register read; and never retry or
  recover the bus inside a callback. An `OK` result with short counts maps to
  `I2C_SHORT_TRANSFER`.
- `Status::msg` is now always derived from the canonical library
  `toString(Err)` table. Custom adapter/application message pointers passed to
  the legacy constructor are ignored; retain diagnostics in the typed code and
  numeric `detail` field.
- `CalibrationRaw::h1` was removed. Register `0xA1` is already `tp[25]`, and
  `readCalibrationRaw()` now performs two bursts instead of three callbacks.
- Use `SensorSettings` plus `startApplySettingsJob()` for cooperative whole-
  settings updates. The existing individual synchronous setters and
  `startApplyConfigJob()` remain available.
- Rebuild dependent firmware: public result/settings layouts and callback
  signatures changed, so this is a major-version migration.

## Validation

The repository's Arduino example environments exact-pin pioarduino
`platform-espressif32` `55.03.311`, which supplies Arduino-ESP32 `3.3.11` and
ESP-IDF `5.5.5`. This pin keeps repository builds reproducible; consuming
applications continue to select and own their platform version.

```text
python tools/check_core_timing_guard.py
python tools/check_cli_contract.py
python tools/check_hil_contract.py
python tools/check_idf_example_contract.py
python scripts/generate_version.py check
python tools/check_release_metadata.py
python -m py_compile tools/run_i2c_hil.py tools/check_hil_contract.py tools/check_release_metadata.py
python tools/test_run_i2c_hil_parser.py
python tools/run_i2c_hil.py --dry-run --out .pio/hil_dry_runs
python tools/run_i2c_hil.py --dry-run --include-job-api --out .pio/hil_dry_runs
doxygen Doxyfile
```

On Windows, all PlatformIO commands in this repository must use the checked-in
wrapper so the current user's VS Code-managed Core is selected:

```powershell
.\scripts\pio.cmd test -e native
.\scripts\pio.cmd run -e esp32s3dev
.\scripts\pio.cmd run -e esp32s2dev
.\scripts\pio.cmd pkg pack
```

The `native_sanitized` environment is a Linux/CI gate. It may also work with a
local compiler that provides ASan and UBSan runtimes, but common Windows MinGW
packages do not ship the required `libasan`/`libubsan` libraries.

Then validate the generated archive and final diff:

```text
python tools/check_package_contents.py
git diff --check
```

Remove the generated package tarball after local validation unless you are
preparing a release artifact.

The package manifest exports the library, examples, and maintained
documentation while excluding development-only CI, test, and validation tools.

Optional local ESP-IDF checks, when `idf.py` is installed:

```bash
idf.py -C examples/idf/basic set-target esp32s3
idf.py -C examples/idf/basic build
idf.py -C examples/idf/basic set-target esp32s2
idf.py -C examples/idf/basic build
```

Local Doxygen generation requires `doxygen` on `PATH`. CI installs Doxygen and
generates the HTML docs as a smoke check:

```bash
doxygen Doxyfile
```

Generated docs under `docs/doxygen/` are local artifacts and are not committed.

## Documentation

- `CHANGELOG.md` - full release history
- `AGENTS.md` - repository engineering rules for future changes
- `CONTRIBUTING.md` - contribution workflow
- `docs/README.md` - maintained documentation map
- `docs/IDF_PORT.md` - ESP-IDF portability guidance
- `docs/BME280_Register_Reference.md` - register reference and bitfield notes
- `docs/PRODUCTION_SHARED_BUS_GUIDE.md` - production shared-bus integration guidance
- `docs/HARDWARE_VALIDATION.md` - consolidated HIL procedure, evidence schema,
  current status, and qualification boundary
- `docs/BME280_datasheet.pdf` - Bosch datasheet copy used for verification

The `2.1.0` changelog entry records the typed-settings and staged-job expansion,
state/cache integrity fixes, example parity, HIL hardening, and release evidence.
The `v2.1.0` tag identifies the exact released commit; `2.0.0` remains the
baseline for the current major transport contract.

## Known Limitations

- The retained ESP32-S2/Arduino HIL is functional release evidence, not a claim
  of electrical, environmental, or every-target hardware qualification. Those
  activities are outside this release's acceptance scope.
- The shipped examples are diagnostic bring-up CLIs. Production shared-bus firmware should follow `docs/PRODUCTION_SHARED_BUS_GUIDE.md` and add application-owned locking, scheduling, timeout, and recovery policy around the injected transport.
- Generated Doxygen HTML, dry-run HIL plans, derived HIL reports, PlatformIO
  build output, and package tarballs are disposable local artifacts. Retain the
  raw transcript and ledger entry for functional evidence; preserve the full
  manifest-backed artifact set before making a formal qualification claim.

## License

MIT License. See [LICENSE](LICENSE).
