# {DEVICE_NAME} Driver Library

Production-grade {DEVICE_NAME} I2C driver for ESP32 (Arduino/PlatformIO).

## Features

- **Injected I2C transport** — no Wire dependency in library code
- **Health monitoring** — automatic state tracking (READY/DEGRADED/OFFLINE)
- **Deterministic behavior** — no unbounded loops, no heap allocations
- **Non-blocking architecture** — tick-based state machine for async operations

## Installation

### PlatformIO (recommended)

Add to `platformio.ini`:

```ini
lib_deps = 
  https://github.com/your-username/{DEVICE}.git
```

### Manual

Copy `include/{DEVICE}/` and `src/` to your project.

## Quick Start

```cpp
#include <Wire.h>
#include "{DEVICE}/{DEVICE}.h"

// Transport callbacks
{NAMESPACE}::Status i2cWrite(uint8_t addr, const uint8_t* data, size_t len,
                             uint32_t timeoutMs, void* user) {
  Wire.beginTransmission(addr);
  Wire.write(data, len);
  return Wire.endTransmission() == 0 
    ? {NAMESPACE}::Status::Ok() 
    : {NAMESPACE}::Status::Error({NAMESPACE}::Err::I2C_ERROR, "Write failed");
}

{NAMESPACE}::Status i2cWriteRead(uint8_t addr, const uint8_t* tx, size_t txLen,
                                 uint8_t* rx, size_t rxLen, uint32_t timeoutMs, void* user) {
  Wire.beginTransmission(addr);
  Wire.write(tx, txLen);
  if (Wire.endTransmission(false) != 0) {
    return {NAMESPACE}::Status::Error({NAMESPACE}::Err::I2C_ERROR, "Write failed");
  }
  if (Wire.requestFrom(addr, rxLen) != rxLen) {
    return {NAMESPACE}::Status::Error({NAMESPACE}::Err::I2C_ERROR, "Read failed");
  }
  for (size_t i = 0; i < rxLen; i++) {
    rx[i] = Wire.read();
  }
  return {NAMESPACE}::Status::Ok();
}

{NAMESPACE}::{DEVICE} device;

void setup() {
  Serial.begin(115200);
  Wire.begin(8, 9);  // SDA, SCL
  
  {NAMESPACE}::Config cfg;
  cfg.i2cWrite = i2cWrite;
  cfg.i2cWriteRead = i2cWriteRead;
  cfg.i2cAddress = 0x00;  // Your device address
  
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
if (device.state() == {NAMESPACE}::DriverState::OFFLINE) {
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

- `Status begin(const Config& config)` — Initialize driver
- `void tick(uint32_t nowMs)` — Process pending operations
- `void end()` — Shutdown driver

### Diagnostics

- `Status probe()` — Check device presence (no health tracking)
- `Status recover()` — Attempt recovery from DEGRADED/OFFLINE

### State

- `DriverState state()` — Current driver state
- `bool isOnline()` — True if READY or DEGRADED

### Health

- `uint32_t lastOkMs()` — Timestamp of last success
- `uint32_t lastErrorMs()` — Timestamp of last failure
- `Status lastError()` — Most recent error
- `uint8_t consecutiveFailures()` — Failures since last success
- `uint32_t totalFailures()` — Lifetime failure count
- `uint32_t totalSuccess()` — Lifetime success count

## Examples

- `01_basic_bringup_cli/` — Interactive CLI for testing

## License

MIT License. See [LICENSE](LICENSE).
