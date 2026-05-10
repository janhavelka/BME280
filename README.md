# BME280 Driver Library

Production-grade BME280 I2C driver for ESP32 (Arduino/PlatformIO).

## Features

- **Injected I2C transport** - no Wire dependency in library code
- **Health monitoring** - automatic state tracking (READY/DEGRADED/OFFLINE)
- **Deterministic behavior** - no unbounded loops, no heap allocations
- **Managed synchronous lifecycle** - blocking I2C ops with tick-based polling for waits

## Installation

### PlatformIO (recommended)

Add to `platformio.ini`:

```ini
lib_deps = 
  https://github.com/janhavelka/BME280.git
```

### Manual

Copy `include/BME280/` and `src/` to your project.

## Quick Start

```cpp
#include <Wire.h>
#include "BME280/BME280.h"
#include "common/I2cTransport.h"

BME280::BME280 device;

void setup() {
  Serial.begin(115200);
  transport::initWire(8, 9, 400000, 50);
  
  BME280::Config cfg;
  cfg.i2cWrite = transport::wireWrite;
  cfg.i2cWriteRead = transport::wireWriteRead;
  cfg.i2cUser = &Wire;
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
bus timeout ownership in `transport::initWire()`. If you do not inject `Config::nowMs`, the
driver falls back to `millis()` on Arduino/native-test builds.
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

- `Status begin(const Config& config)` - Initialize driver
- `void tick(uint32_t nowMs)` - Process pending operations
- `void end()` - Shutdown driver and best-effort return the sensor to sleep
- `bool isInitialized()` - True after successful `begin()` until `end()`
- `const Config& getConfig()` - Cached configuration snapshot owned by the driver

### Diagnostics

- `Status probe()` - Check device presence (no health tracking)
- `Status recover()` - Attempt recovery from DEGRADED/OFFLINE (re-applies config)
- `Status getSettings(SettingsSnapshot& out)` - Populate a snapshot of cached config and runtime state (no I2C)

### Measurement

- `Status requestMeasurement()` - Start a forced measurement or schedule a fresh normal-mode cycle; returns `IN_PROGRESS` when accepted
- `bool measurementReady()` - True after `tick()` captures and compensates a sample
- `Status getMeasurement(Measurement& out)` - Get floating-point temperature, pressure, and humidity
- `Status getRawSample(RawSample& out)` - Get the latest raw ADC sample after at least one capture
- `Status getCompensatedSample(CompensatedSample& out)` - Get fixed-point compensated values after at least one capture
- `Status getCalibration(Calibration& out)` - Return cached calibration coefficients
- `Status readCalibrationRaw(CalibrationRaw& out)` - Read calibration register blocks from the device

Forced mode is an on-demand policy: `begin()` and `setMode(FORCED)` keep the hardware in
sleep until `requestMeasurement()` writes the forced-mode trigger. Normal-mode requests
wait one estimated normal cycle before reading registers, so the returned sample is fresh
relative to the request.

### Configuration

- `Status setMode(Mode mode)` - Select `SLEEP`, `FORCED`, or `NORMAL`
- `Status setOversamplingT/P/H(Oversampling osrs)` - Configure temperature, pressure, or humidity oversampling
- `Status setFilter(Filter filter)` - Configure the IIR filter coefficient
- `Status setStandby(Standby standby)` - Configure standby interval for normal mode
- `Status softReset()` - Write the Bosch reset command, reload calibration, and reapply cached config
- `Status readChipId/readStatus/readCtrlHum/readCtrlMeas/readConfig(...)` - Read status/config registers
- `Status isMeasuring(bool& measuring)` - Read the measuring bit

Temperature oversampling must be enabled whenever pressure or humidity is enabled because
Bosch compensation requires `t_fine`. At least one measured channel must be enabled.
Invalid combinations are rejected in `begin()` and typed setters before touching I2C.

### Raw Register Access

- `Status readRegisters(uint8_t startReg, uint8_t* buf, size_t len)` - Read a contiguous tracked register block
- `Status writeRegisters(uint8_t startReg, const uint8_t* buf, size_t len)` - Write a contiguous tracked register block
- `Status readRegister(uint8_t reg, uint8_t& value)` - Read a single tracked register
- `Status writeRegister(uint8_t reg, uint8_t value)` - Write a single tracked register

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

## Examples

- `01_basic_bringup_cli/` - Interactive CLI for testing
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
2. Timing model: `tick()` is bounded; forced-mode wait and recovery timing stay callback-timebase-driven.
3. Resource ownership: bus, pins, and timeout policy remain application-owned via `Config`.
4. Memory behavior: no heap allocation in steady-state library operation.
5. Error handling: all fallible APIs return `Status`; no exceptions and no silent failures.
6. Health behavior: `OFFLINE` is latched. Normal public I2C operations return `BUSY` with `Driver is offline; call recover()` without touching the bus until `recover()` succeeds.

## Running Tests

```bash
pio test -e native
python tools/check_cli_contract.py
python tools/check_core_timing_guard.py
pio run -e esp32s3dev
pio run -e esp32s2dev
```

## Documentation

- `CHANGELOG.md` - full release history
- `docs/IDF_PORT.md` - ESP-IDF portability guidance
- `docs/BME280_Register_Reference.md` - register reference and bitfield notes
- `docs/BME280_datasheet.pdf` - Bosch datasheet copy used for verification

## License

MIT License. See [LICENSE](LICENSE).
