# BME280 ESP-IDF Port

Last updated: 2026-05-31

This document is the maintained ESP-IDF note for the BME280 library. It replaces
the older separate implementation report.

## Scope

The repository can be used as:

- an Arduino/PlatformIO library;
- an ESP-IDF component from the repository root;
- a native ESP-IDF example under `examples/idf/basic`.

The core driver remains framework-neutral. It does not include Arduino or
ESP-IDF headers, configure pins, create I2C buses, log, or own task timing.

## Current State

- Public API: `include/BME280/`.
- Core implementation: `src/BME280.cpp`.
- ESP-IDF component metadata: `CMakeLists.txt` and `idf_component.yml`.
- Native ESP-IDF example: `examples/idf/basic`.
- Arduino bring-up example: `examples/01_basic_bringup_cli`.
- Example-only shared helpers: `examples/common`.

The ESP-IDF example uses native IDF APIs:

- `extern "C" void app_main(void)`;
- `driver/i2c_master.h`;
- `esp_timer_get_time()` for the injected millisecond clock;
- FreeRTOS delays/tasks;
- fixed C command buffers.

It must not include Arduino source or compatibility facades such as
`Arduino.h`, `Wire.h`, `String`, `Serial`, `TwoWire`, `ArduinoCompat`, or
`IdfArduinoCompat`.

The native CLI keeps the same operator-facing bring-up flow as the Arduino
example, including `scan`, address selection, `chipid`, measurement, reset,
recover, self-test, and stress commands.

## Ownership Boundary

The driver does not own I2C resources. The application or example owns:

- bus and device handles;
- SDA/SCL pins;
- pull-ups and bus speed;
- timeout policy;
- bus locking/mutexes;
- reset or power-control GPIOs;
- task scheduling for `tick()`.

The application passes callbacks through `BME280::Config`:

- `i2cWrite`;
- `i2cWriteRead`;
- `i2cUser`;
- `nowMs`;
- `timeUser`.

`Config::nowMs` should be set in ESP-IDF applications. The core fallback is
intentionally inert and is not a platform clock.

## Transport Contract

The native IDF adapter uses the ESP-IDF v6 I2C master driver:

```cpp
#include <driver/i2c_master.h>
```

Expected callback behavior:

- `addr` is a 7-bit BME280 address, `0x76` or `0x77`.
- `i2cWrite()` maps to `i2c_master_transmit()`.
- `i2cWriteRead()` maps to `i2c_master_transmit_receive()` for register reads.
- The callback completes synchronously before returning.
- `timeoutMs` is clamped or rejected before passing it to ESP-IDF so it cannot
  become an accidental infinite wait.
- Transport callbacks must not recursively call into the same `BME280::BME280`
  instance.

Simple ESP-IDF transfer APIs do not always reveal whether a NACK happened during
address or data phase. Preserve precise errors when the adapter can prove them;
otherwise return `I2C_ERROR` or `I2C_BUS` with the raw `esp_err_t` value in
`Status::detail`.

Current example mapping:

| ESP-IDF result | Library status |
| --- | --- |
| `ESP_OK` | `Err::OK` |
| `ESP_ERR_TIMEOUT` | `Err::I2C_TIMEOUT` |
| `ESP_ERR_INVALID_ARG` | `Err::INVALID_PARAM` |
| `ESP_ERR_INVALID_RESPONSE` | `Err::I2C_ERROR` |
| other failures | `Err::I2C_BUS` |

## Build Checks

Run these from the repository root:

```bash
python tools/check_core_timing_guard.py
python tools/check_cli_contract.py
python tools/check_idf_example_contract.py
python -m platformio test -e native
python -m platformio run -e esp32s3dev
python -m platformio run -e esp32s2dev
```

When `idf.py` is installed, also run:

```bash
idf.py -C examples/idf/basic set-target esp32s3
idf.py -C examples/idf/basic build
idf.py -C examples/idf/basic set-target esp32s2
idf.py -C examples/idf/basic build
```

Do not claim local pure ESP-IDF validation unless those exact builds were run
and recorded.

## Hardware Work Still Required

The ESP-IDF port is not hardware-validated by this document. Before claiming a
hardware result, record:

- board and MCU target;
- BME280 module;
- address and SDO/CSB wiring;
- VDD/VDDIO voltage;
- SDA/SCL pins, pull-ups, and bus speed;
- firmware commit;
- command transcript;
- environmental reference notes;
- fault/recovery and soak results if run.

Use `docs/I2C_HIL_RUNBOOK.md`,
`docs/I2C_HIL_TARGET_TEMPLATE.md`, and
`docs/BME280_HARDWARE_VALIDATION_MATRIX.md` for that evidence.
