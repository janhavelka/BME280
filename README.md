# BME280 Driver Library

Production-grade BME280 I2C driver for ESP32 (Arduino/PlatformIO).

## Features

- **Injected I2C transport** - no Wire dependency in library code
- **Health monitoring** - automatic state tracking (READY/DEGRADED/OFFLINE)
- **Deterministic behavior** - no unbounded loops, no heap allocations
- **Non-blocking architecture** - tick-based state machine for async operations

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

// Transport callbacks
BME280::Status i2cWrite(uint8_t addr, const uint8_t* data, size_t len,
                             uint32_t timeoutMs, void* user) {
  Wire.beginTransmission(addr);
  Wire.write(data, len);
  return Wire.endTransmission() == 0 
    ? BME280::Status::Ok() 
    : BME280::Status::Error(BME280::Err::I2C_ERROR, "Write failed");
}

BME280::Status i2cWriteRead(uint8_t addr, const uint8_t* tx, size_t txLen,
                                  uint8_t* rx, size_t rxLen, uint32_t timeoutMs, void* user) {
  Wire.beginTransmission(addr);
  Wire.write(tx, txLen);
  if (Wire.endTransmission(false) != 0) {
    return BME280::Status::Error(BME280::Err::I2C_ERROR, "Write failed");
  }
  if (Wire.requestFrom(addr, rxLen) != rxLen) {
    return BME280::Status::Error(BME280::Err::I2C_ERROR, "Read failed");
  }
  for (size_t i = 0; i < rxLen; i++) {
    rx[i] = Wire.read();
  }
  return BME280::Status::Ok();
}

uint32_t nowMs(void*) {
  return millis();
}

BME280::BME280 device;

void setup() {
  Serial.begin(115200);
  Wire.begin(8, 9);  // SDA, SCL
  
  BME280::Config cfg;
  cfg.i2cWrite = i2cWrite;
  cfg.i2cWriteRead = i2cWriteRead;
  cfg.nowMs = nowMs;
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
- `void end()` - Shutdown driver

### Diagnostics

- `Status probe()` - Check device presence (no health tracking)
- `Status recover()` - Attempt recovery from DEGRADED/OFFLINE (re-applies config)

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

### Timing

- `uint32_t estimateMeasurementTimeMs()` - Max measurement time for current oversampling
- `uint32_t getStandbyTimeMs()` - Configured standby interval in ms
- `uint32_t estimateNormalCycleMs()` - Full normal-mode cycle (measurement + standby)

## Examples

- `01_basic_bringup_cli/` - Interactive CLI for testing

### Example Helpers (`examples/common/`)

Not part of the library. These simulate project-level glue and keep examples self-contained:

| File | Purpose |
|------|---------|
| `BoardConfig.h` | Pin definitions and Wire init for supported boards |
| `BuildConfig.h` | Compile-time `LOG_LEVEL` configuration |
| `Log.h` | Serial logging macros (`LOGE`/`LOGW`/`LOGI`/`LOGD`/`LOGT`/`LOGV`) |
| `I2cTransport.h` | Wire-based I2C transport adapter (`wireWrite`, `wireWriteRead`) |
| `I2cScanner.h` | I2C bus scanner with table output and bus recovery |
| `BusDiag.h` | Bus diagnostics wrapper (scan + probe) |
| `CliShell.h` | Serial command-line shell with line editing |
| `CommandHandler.h` | Command parsing helpers (`readLine`, `match`, `parseInt`) |
| `HealthView.h` | Compact health status display |
| `HealthDiag.h` | Verbose health diagnostics with color, snapshots, and `HealthMonitor` |
| `TransportAdapter.h` | Transport function pointer adapter |

## License

MIT License. See [LICENSE](LICENSE).
