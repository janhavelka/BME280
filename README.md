# BME280 Driver Library

Production-oriented BME280 I2C driver for ESP32 (Arduino/PlatformIO and ESP-IDF component use).

Validation status: host/native tests, guard scripts, and Arduino PlatformIO
builds are run in this repository. Physical BME280 hardware validation and
local pure ESP-IDF `idf.py` builds are not claimed unless a phase report records
the exact commands and hardware setup.

## Features

- **Injected I2C transport** - no Wire dependency in library code
- **Framework-neutral core** - Arduino and ESP-IDF integration live behind callbacks/adapters
- **Health monitoring** - automatic state tracking (READY/DEGRADED/OFFLINE)
- **Deterministic behavior** - no unbounded loops, no heap allocations
- **Managed synchronous lifecycle** - blocking bounded reset/NVM polling and tick-driven measurement polling

## Installation

### PlatformIO (recommended)

Add to `platformio.ini`:

```ini
lib_deps = 
  https://github.com/janhavelka/BME280.git
```

### Manual

Copy `include/BME280/` and `src/` to your project.

### ESP-IDF Component

The repository root can be used as an ESP-IDF component. Add it to an IDF
project through `EXTRA_COMPONENT_DIRS` or your component manager workflow and
provide application-owned I2C callbacks through `BME280::Config`.

The core component does not configure pins, create I2C buses, log, or include
Arduino or ESP-IDF framework headers. Applications should inject `Config::nowMs`
so health timestamps and scheduler timing share the application clock.

See `examples/idf/basic` for a native ESP-IDF v6-style `i2c_master` adapter and
`app_main` CLI. The ESP-IDF example preserves the Arduino CLI command contract
without including Arduino source or compatibility facades.

## Quick Start

```cpp
#include <Wire.h>
#include "BME280/BME280.h"
#include "common/I2cTransport.h"

BME280::BME280 device;

uint32_t appNowMs(void*) {
  return millis();
}

void setup() {
  Serial.begin(115200);
  transport::initWire(8, 9, 400000, 50);
  
  BME280::Config cfg;
  cfg.i2cWrite = transport::wireWrite;
  cfg.i2cWriteRead = transport::wireWriteRead;
  cfg.i2cUser = transport::configUser();
  cfg.nowMs = appNowMs;
  cfg.i2cAddress = 0x76;
  
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

The example adapter maps Arduino `Wire` failures to specific `I2C_*` status codes and keeps
bus timeout ownership in `transport::initWire()`. Inject `Config::nowMs` for real timestamps;
the framework-neutral core fallback is intentionally inert.
`common/I2cTransport.h` is example-only glue; when manually copying only `include/` and
`src/`, provide equivalent `Config::i2cWrite` and `Config::i2cWriteRead` callbacks in
your application.

## Health Monitoring

The driver tracks I2C communication health:

```cpp
// Check state
if (device.state() == BME280::DriverState::OFFLINE) {
  Serial.println("Device offline!");
  device.recover();  // Try to reconnect
}

// Get statistics
Serial.printf("Failures: %u consecutive, %lu total\n",
              device.consecutiveFailures(), device.totalFailures());
```

### Driver States

| State | Description |
|-------|-------------|
| `UNINIT` | `begin()` not called or `end()` called |
| `READY` | Operational, no recent failures |
| `DEGRADED` | 1+ failures, below offline threshold |
| `OFFLINE` | Too many consecutive failures |

## API Reference

### Lifecycle

- `Status begin(const Config& config)` - Initialize driver, verify chip ID `0x60`, wait for NVM copy, read calibration, and apply config
- `void tick(uint32_t nowMs)` - Process pending measurement operations; `nowMs` must use the same monotonic timebase as `Config::nowMs`
- `void end()` - Shutdown driver and best-effort return the sensor to sleep
- `bool isInitialized()` - True after successful `begin()` until `end()`
- `const Config& getConfig()` - Cached configuration snapshot owned by the driver

### Diagnostics

- `Status probe()` - Check device presence and chip ID through raw I2C without health tracking
- `Status recover()` - Attempt recovery from DEGRADED/OFFLINE; waits for NVM, reloads calibration, and reapplies cached config
- `Status getSettings(SettingsSnapshot& out)` - Populate a snapshot of cached config and runtime state (no I2C)
- `Status lastMeasurementStatus()` - Last measurement request, polling, raw-read, or compensation status retained because `tick()` is void

### Measurement

- `Status requestMeasurement()` - Start a forced measurement or schedule a fresh normal-mode cycle; returns `IN_PROGRESS` when accepted
- `bool measurementReady()` - True after `tick()` captures and compensates a sample
- `Status getMeasurement(Measurement& out)` - Get floating-point temperature, pressure, and humidity plus per-channel validity flags
- `Status getRawSample(RawSample& out)` - Get the latest raw ADC sample plus per-channel validity flags after at least one capture
- `Status getCompensatedSample(CompensatedSample& out)` - Get fixed-point compensated values plus per-channel validity flags after at least one capture
- `Status getCalibration(Calibration& out)` - Return cached calibration coefficients
- `Status readCalibrationRaw(CalibrationRaw& out)` - Read calibration register blocks from the device

Forced mode is an on-demand policy: `begin()` and `setMode(FORCED)` keep the hardware in
sleep until `requestMeasurement()` writes the forced-mode trigger. Normal-mode requests
wait one estimated normal cycle before reading registers, so the returned sample is fresh
relative to the request.

`tick()` does not return a `Status`. Measurement scheduler and capture results are
retained in `lastMeasurementStatus()` and `SettingsSnapshot::lastMeasurementStatus`.
The value is `IN_PROGRESS` while a request is pending, `OK` after a sample is
captured, and the original transport or compensation error when a polling,
burst-read, or compensation step fails.

### Configuration

- `Status setMode(Mode mode)` - Select `SLEEP`, `FORCED`, or `NORMAL`
- `Status setOversamplingT/P/H(Oversampling osrs)` - Configure temperature, pressure, or humidity oversampling
- `Status setFilter(Filter filter)` - Configure the IIR filter coefficient through a safe sleep/config/restore sequence
- `Status setStandby(Standby standby)` - Configure standby interval for normal mode through a safe sleep/config/restore sequence
- `Status softReset()` - Write `0xB6` to reset register `0xE0`, wait for NVM copy, reload calibration, and reapply cached config
- `Status readChipId/readStatus/readCtrlHum/readCtrlMeas/readConfig(...)` - Read status/config registers
- `Status isMeasuring(bool& measuring)` - Read the measuring bit

Temperature oversampling must be enabled whenever pressure or humidity is enabled because
Bosch compensation requires `t_fine`. At least one measured channel must be enabled.
Invalid combinations are rejected in `begin()` and typed setters before touching I2C.

Humidity oversampling follows the Bosch latch rule: `setOversamplingH()` writes
`ctrl_hum` first and then writes `ctrl_meas` so the new humidity setting becomes
effective. `setFilter()` and `setStandby()` first verify that the status
`measuring` bit is clear. If the device is already measuring, they return `BUSY`
without writing config registers. Otherwise they switch `ctrl_meas` to sleep,
verify `measuring` is still clear, write `config`, and restore the cached mode.
If the second status check reports busy after the sleep write, the config write
is skipped and `hardwareConfigDirty()` is set because hardware mode may no
longer match the cache. Successful typed configuration changes invalidate cached
samples so callers cannot read a sample captured under old settings.

If a multi-register configuration sequence touches hardware and then fails, the
driver sets `hardwareConfigDirty()` and preserves the original error in
`hardwareConfigDirtyError()` and `SettingsSnapshot::hardwareConfigDirtyError`.
The dirty flag is cleared only by a complete successful resync through
`begin()`, `recover()`, or `softReset()`.

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

`softReset()` writes the Bosch reset command, then polls status bit `im_update`
until NVM copy completes. Polling is bounded by both a 10 ms deadline when a real
`Config::nowMs` hook is injected and a fixed poll limit when no real clock is
available. Transient status-read errors are tolerated during reset/POR if a
later poll succeeds. If NVM never becomes ready, the first useful transport
error is returned; if status reads succeed but `im_update` remains set, the
result is `TIMEOUT`.

After a successful reset write, hardware config may be back at defaults. If NVM
polling, calibration reload, validation, or config reapply fails,
`hardwareConfigDirty()` remains set with the root-cause status. A successful
`softReset()` or `recover()` reloads calibration and clears dirty state only
after the full cached configuration is reapplied. Runtime reset/recover NVM
polling uses health-tracked I2C, so repeated status-read transport failures can
move the driver to `DEGRADED` or `OFFLINE` while preserving the root-cause
`Status`.

### Public I2C Transaction Shape

| API | Typical I2C transactions | Notes |
|-----|--------------------------|-------|
| `begin()` | chip ID read, bounded NVM status polling, calibration reads, status guard, config writes | Does not require `Config::nowMs`; call after device POR/I2C readiness |
| `requestMeasurement()` in forced mode | status read, `ctrl_meas` write | Returns `IN_PROGRESS` when accepted |
| `tick()` after deadline | status read, one `0xF7..0xFE` burst read | Captures coherent pressure, temperature, and humidity ADC bytes |
| `setOversamplingT/P()` | one `ctrl_meas` write | Invalid combinations are rejected before I2C |
| `setOversamplingH()` | `ctrl_hum` write, `ctrl_meas` write | `ctrl_meas` latches humidity oversampling |
| `setFilter()` / `setStandby()` | status read, sleep write, status read, `config` write, restore write | Returns `BUSY` without writes if already measuring; skips `config` and marks dirty if still measuring after sleep write |
| `recover()` | chip ID read, bounded NVM polling, calibration reads, status guard, config resync writes | Allowed while OFFLINE; reloads calibration before config reapply |
| `softReset()` | reset write, bounded NVM polling, calibration reads, status guard, config resync writes | Marks dirty if reset succeeds but any later step fails |

Sample numeric units are stable: `Measurement` returns degrees Celsius, Pascals,
and percent RH; `CompensatedSample` returns `tempC_x100`, integer Pascals, and
`humidityPct_x1024` (Q22.10). Skipped or invalid channels keep numeric fields at
zero for compatibility, so callers must check `temperatureValid`,
`pressureValid`, and `humidityValid` before using a channel. The raw Bosch
skipped sentinels are exposed as `cmd::RAW_PRESSURE_SKIPPED`,
`cmd::RAW_TEMPERATURE_SKIPPED`, and `cmd::RAW_HUMIDITY_SKIPPED`.

Calibration coefficients are read from `0x88..0xA1` and `0xE1..0xE7` during
`begin()`. `dig_T1` and `dig_P1` are unsigned 16-bit values; the other
temperature/pressure coefficients are signed 16-bit values. `dig_H4` and
`dig_H5` are signed 12-bit humidity coefficients packed across `0xE4`, `0xE5`,
and `0xE6`. Compensation follows the Bosch integer flow: temperature is computed
first to produce `t_fine`; pressure uses the 64-bit path with a divide-by-zero
guard; humidity is clamped to `0..100%RH`.

### Raw Register Access

- `Status readRegisters(uint8_t startReg, uint8_t* buf, size_t len)` - Read a contiguous tracked register block
- `Status writeRegisters(uint8_t startReg, const uint8_t* buf, size_t len)` - Write a contiguous tracked register block
- `Status readRegister(uint8_t reg, uint8_t& value)` - Read a single tracked register
- `Status writeRegister(uint8_t reg, uint8_t value)` - Write a single tracked register

Raw writes are diagnostic tools. Writes to BME280 configuration registers can
desynchronize the driver's cached settings from hardware, so call `recover()` or
`begin()` to resync after manual register edits.

### State

- `DriverState state()` - Current driver state
- `bool isOnline()` - True if READY or DEGRADED

### Health

- `uint32_t lastOkMs()` - Timestamp of last success
- `uint32_t lastErrorMs()` - Timestamp of last failure
- `Status lastError()` - Most recent error
- `uint8_t consecutiveFailures()` - Failures since last success
- `uint32_t totalFailures()` - Lifetime failure count
- `uint32_t totalSuccess()` - Lifetime success count

`IN_PROGRESS` is treated as non-failure activity for health tracking. Pre-`begin()` validation and transport setup errors do not transition the driver into `DEGRADED` or `OFFLINE`.

### Timing

- `uint32_t estimateMeasurementTimeMs()` - Max measurement time for current oversampling
- `uint32_t getStandbyTimeMs()` - Configured standby interval in ms
- `uint32_t estimateNormalCycleMs()` - Full normal-mode cycle (measurement + standby)

Measurement timing uses the Bosch oversampling multipliers `SKIP=0`, `X1=1`,
`X2=2`, `X4=4`, `X8=8`, and `X16=16`:

```text
t_meas_us = 1250
          + (temperature enabled ? 2300 * osrs_t : 0)
          + (pressure enabled ? 2300 * osrs_p + 575 : 0)
          + (humidity enabled ? 2300 * osrs_h + 575 : 0)
          + 1000 safety margin
estimateMeasurementTimeMs = ceil(t_meas_us / 1000)
estimateNormalCycleMs = estimateMeasurementTimeMs + getStandbyTimeMs()
```

## Examples

- `01_basic_bringup_cli/` - Interactive CLI for testing
- `idf/basic/` - Native ESP-IDF example using `app_main`, `driver/i2c_master.h`, FreeRTOS timing, fixed command buffers, and the same user-facing CLI workflow as Arduino
- CLI register diagnostics: `reg <addr>` and `wreg <addr> <val>` provide tracked raw register access for bring-up and service work. Raw writes bypass the typed config helpers; use `recover()` or `begin()` to restore cached settings after manual register edits.

### Example Helpers (`examples/common/`)

Not part of the library. These simulate project-level glue and keep examples self-contained:

| File | Purpose |
|------|---------|
| `BoardConfig.h` | Pin definitions and Wire init for supported boards |
| `BuildConfig.h` | Compile-time `LOG_LEVEL` configuration |
| `Log.h` | Serial logging macros (`LOGE`/`LOGW`/`LOGI`/`LOGD`/`LOGT`/`LOGV`) |
| `I2cTransport.h` | Wire-based I2C transport adapter (`wireWrite`, `wireWriteRead`, `initWire`) |
| `I2cScanner.h` | I2C bus scanner with table output and bus recovery |
| `BusDiag.h` | Bus diagnostics wrapper (scan + probe) |
| `CliStyle.h` | Shared ANSI colors and CLI formatting helpers |
| `CliShell.h` | Serial command-line shell with line editing |
| `CommandHandler.h` | Command parsing helpers (`readLine`, `match`, `parseInt`) |
| `HealthView.h` | Compact health status display |
| `HealthDiag.h` | Verbose health diagnostics with color, snapshots, and `HealthMonitor` |
| `TransportAdapter.h` | Transport function pointer adapter |

## Behavioral Contracts

1. Threading model: single-threaded by default; not thread-safe.
2. Timing model: `tick()` is bounded; `tick(nowMs)` and `Config::nowMs` must use the same monotonic timebase.
3. Resource ownership: bus, pins, and timeout policy remain application-owned via `Config`.
4. Memory behavior: no heap allocation in steady-state library operation.
5. Error handling: all fallible APIs return `Status`; no exceptions and no silent failures.
6. Health behavior: `OFFLINE` is latched. Normal public I2C operations return `BUSY` with `Driver is offline; call recover()` without touching the bus until `recover()` succeeds. `probe()` remains raw diagnostic I2C and does not clear the latch.
7. Measurement scheduling requires `Config::nowMs`. `begin()` does not fail without it, but `requestMeasurement()` returns `INVALID_CONFIG` if no monotonic clock is injected.
8. Multi-register configuration failures set `hardwareConfigDirty()` and expose the original dirty-state error in `hardwareConfigDirtyError()` and `SettingsSnapshot`.
9. Driver instances are not thread-safe and public APIs are not ISR-safe. Shared-bus users must serialize access externally.
10. `setFilter()` and `setStandby()` return `BUSY` without config writes when the device initially reports `measuring`; if `measuring` appears after the sleep write, config is skipped and dirty state is set.
11. `probe()` is diagnostic-only and preserves timeout, bus, data-NACK, and generic I2C errors. `DEVICE_NOT_FOUND` is reserved for definite address NACK.
12. Reset and NVM polling are bounded. If status reads never succeed, the first transport error is returned; if successful status reads keep reporting `im_update`, the result is `TIMEOUT`.

## Running Tests

```bash
pio test -e native
python tools/check_cli_contract.py
python tools/check_core_timing_guard.py
python tools/check_idf_example_contract.py
pio run -e esp32s3dev
pio run -e esp32s2dev
idf.py -C examples/idf/basic set-target esp32s3
idf.py -C examples/idf/basic build
idf.py -C examples/idf/basic set-target esp32s2
idf.py -C examples/idf/basic build
```

## Documentation

- `CHANGELOG.md` - full release history
- `docs/IDF_PORT.md` - ESP-IDF portability guidance
- `docs/IDF_PORT_IMPLEMENTATION.md` - implemented IDF component/example notes
- `docs/BME280_Register_Reference.md` - register reference and bitfield notes
- `docs/BME280_datasheet.pdf` - Bosch datasheet copy used for verification

## License

MIT License. See [LICENSE](LICENSE).
