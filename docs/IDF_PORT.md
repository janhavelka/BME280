# BME280 ESP-IDF v6.0.1 Port Status

Last updated: 2026-05-19

Scope: implemented core portability changes plus an ESP-IDF component/example. Arduino/PlatformIO support remains intact.

Official ESP-IDF references:
- I2C master driver: https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/i2c.html
- Build system and components: https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/build-system.html
- ESP-IDF v6 peripheral migration notes: https://docs.espressif.com/projects/esp-idf/en/release-v6.0/esp32c3/migration-guides/release-6.x/6.0/peripherals.html

## Current State

- Public API is in `include/BME280/`; implementation is in `src/BME280.cpp`.
- `library.json` and `platformio.ini` remain Arduino/PlatformIO package files.
- Root `CMakeLists.txt` registers the core as an ESP-IDF component.
- The driver already uses injected I2C callbacks from `Config`; library code does not call `Wire` directly.
- `begin()` probes the configured address by reading the chip ID and configures the device from `Config`.
- Supported device addresses are `0x76` and `0x77`; expected chip ID is `0x60`.
- The driver supports sleep, forced, and normal mode, oversampling, filter, standby, raw and compensated pressure/temperature/humidity, soft reset, register access, and health tracking.
- `tick(uint32_t nowMs)` drives non-blocking measurement and reset wait paths.
- Native tests and examples are still organized around the Arduino project layout.

## Arduino Dependencies

- `src/BME280.cpp` no longer includes `<Arduino.h>`.
- `src/PlatformTime.h` is the private fallback timing shim for Arduino, ESP-IDF, and native tests.
- `include/BME280/Config.h` exposes `NowMsFn nowMs`, `void* timeUser`, I2C callbacks, and timeout/configuration fields that are already usable from ESP-IDF.
- `examples/01_basic_bringup_cli/main.cpp` and all `examples/common/*.h`
  helpers are Arduino example glue. They use `Wire`, `Serial`, `String`, GPIO
  helpers, and Arduino timing.
- `platformio.ini` builds Arduino examples and native tests. It is not an IDF project file.
- `library.json` declares only the Arduino framework and must remain the PlatformIO package manifest.
- `include/BME280/Version.h` is generated from `library.json`; do not edit it by hand.

## Portability Status

Implemented:

1. The core driver compiles through a private platform time shim instead of direct Arduino includes.
2. ESP-IDF fallback timing uses `esp_timer_get_time() / 1000`.
3. Root `CMakeLists.txt` provides `idf_component_register`.
4. `examples/idf/basic` provides an ESP-IDF v6 `i2c_master` adapter.
5. Arduino examples remain separate and are not part of the IDF component target.
6. The IDF example maps `esp_err_t` values to library `Status` codes.

Still application-owned:

1. I2C bus/device creation and destruction.
2. SDA/SCL pins, pullups, clock speed, and timeout policy.
3. Hardware smoke testing on target boards.

## Exact Files and APIs to Change

- `src/BME280.cpp`
  - Replace the direct `<Arduino.h>` include with a private platform/timing shim.
  - Keep all register access through existing `readRegs()` and `writeRegs()` helpers.
  - Do not add `Wire`, ESP-IDF bus handles, pins, or bus configuration to the driver class.
- `include/BME280/Config.h`
  - No API break is required for the first IDF port.
  - Keep `i2cWrite`, `i2cWriteRead`, `i2cUser`, `nowMs`, and `timeUser` as the public portability boundary.
  - If the code chooses to require `nowMs` under IDF, document that in the field comment without changing Arduino behavior.
- Private shim `src/PlatformTime.h`
  - Under Arduino, call `millis()`.
  - Under ESP-IDF, call `esp_timer_get_time() / 1000`.
  - Keep this file private; public headers must stay framework-neutral.
- ESP-IDF example files under `examples/idf/basic/main/`
  - Own the I2C bus and device handles.
  - Fill `Config` with adapter callbacks and time callback.
  - Call `tick()` from the application task.
- CMake files
  - Root component `CMakeLists.txt`.
  - Example project files under `examples/idf/basic`.

## Dual Arduino and ESP-IDF Architecture

- Keep the BME280 core as a framework-neutral C++17 component.
- Keep Arduino-only adapters in `examples/common/`; they are not part of the library.
- Add IDF-only adapters under an IDF example or an optional `extras/idf/` style directory.
- The library must never own the ESP-IDF I2C bus, pins, clock speed, pullups, or interrupt routing.
- The application owns bus lifetime:
  - create an `i2c_master_bus_handle_t`;
  - add an `i2c_master_dev_handle_t` for `0x76` or `0x77`;
  - pass an adapter context through `Config.i2cUser`;
  - destroy handles after `driver.end()` and after all driver calls stop.
- Do not call public driver APIs from an ISR. If a future data-ready GPIO is used, notify a task and call the driver there.

## IDF Transport Adapter Contract

Use ESP-IDF v6.0.1's new I2C master driver only:

```cpp
#include <driver/i2c_master.h>
```

The adapter should provide the existing callback shape:

```cpp
Status idfWrite(uint8_t addr,
                const uint8_t* data,
                size_t len,
                uint32_t timeoutMs,
                void* user);

Status idfWriteRead(uint8_t addr,
                    const uint8_t* txData,
                    size_t txLen,
                    uint8_t* rxData,
                    size_t rxLen,
                    uint32_t timeoutMs,
                    void* user);
```

Expected behavior:

- `addr` is a 7-bit address. Reject any address other than the configured BME280 address unless the adapter intentionally supports multiple devices.
- `idfWrite()` calls `i2c_master_transmit()`.
- `idfWriteRead()` calls:
  - `i2c_master_receive()` when `txLen == 0`;
  - `i2c_master_transmit()` when `rxLen == 0`;
  - `i2c_master_transmit_receive()` for register reads.
- `timeoutMs` is passed as the ESP-IDF transfer timeout in milliseconds.
- Clamp or reject `timeoutMs` before passing it to ESP-IDF's signed
  `xfer_timeout_ms`; never allow overflow to become `-1` because `-1` waits
  forever.
- Map `ESP_OK` to `Err::OK`.
- Map `ESP_ERR_TIMEOUT` to `Err::I2C_TIMEOUT`.
- Map `ESP_ERR_INVALID_ARG` to `Err::INVALID_PARAM`.
- Map `ESP_ERR_INVALID_RESPONSE` to an I2C NACK-related status. The simple
  ESP-IDF master APIs do not expose address-vs-data phase, so use
  `Err::I2C_ERROR` with `Status.detail = ESP_ERR_INVALID_RESPONSE` unless a
  custom adapter can prove the NACK phase.
- Map address/data NACKs to `Err::I2C_NACK_ADDR` or `Err::I2C_NACK_DATA` only when the adapter can distinguish them. Otherwise use `Err::I2C_ERROR` and place the raw `esp_err_t` value in `Status.detail`.
- Do not register `i2c_master_register_event_callbacks()` on the handle used by
  these callbacks unless the adapter waits for transfer completion before
  returning. The driver expects callbacks to be complete and blocking.
- Adapter setup failures such as bus creation or device-add failure occur before `begin()` and should be reported by the example/application, not hidden inside the driver.

## CMake and Component Plan

Minimal core component:

```cmake
idf_component_register(
  SRCS "src/BME280.cpp"
  INCLUDE_DIRS "include"
)

target_compile_features(${COMPONENT_LIB} PUBLIC cxx_std_17)
```

If the private timing shim uses ESP-IDF fallback APIs, add private requirements:

```cmake
idf_component_register(
  SRCS "src/BME280.cpp"
  INCLUDE_DIRS "include"
  PRIV_REQUIRES esp_timer
)
```

If an IDF adapter is built into an example component, that example should declare:

```cmake
idf_component_register(
  SRCS "main.cpp" "IdfI2cTransport.cpp"
  INCLUDE_DIRS "."
  REQUIRES BME280 esp_driver_i2c esp_timer
)
```

Do not compile `examples/common/*.h` into ESP-IDF targets.

## IDF and Arduino Example Plan

Arduino examples:

- Keep existing examples using `examples/common/BoardConfig.h` and `I2cTransport.h`.
- Do not replace Arduino `Wire` examples with IDF code.

ESP-IDF examples:

- Add `examples/idf/basic` with a normal ESP-IDF `main` component.
- Configure SDA, SCL, pullups, and bus frequency in the example only.
- Use `i2c_new_master_bus()`, `i2c_master_bus_add_device()`, and the callback adapter.
- Build a `BME280::Config`, set the address, callbacks, timeout, and `nowMs`.
- Call `begin(config)`, then request forced measurements or run normal mode.
- Call `tick()` periodically from the task loop.
- Print results with ESP-IDF logging from the example, not from the library.

## Test and Validation Plan

- Compile the existing Arduino PlatformIO environments as regression checks
  only. Arduino-ESP32 builds do not prove pure ESP-IDF v6.0.1 compatibility.
- Compile native tests to preserve framework-neutral behavior.
- Add an ESP-IDF example build for ESP32-S2 and ESP32-S3.
- Hardware smoke test both valid addresses, `0x76` and `0x77`.
- Verify `begin()` fails cleanly when the bus is absent, the address is wrong, or chip ID is not `0x60`.
- Verify raw burst read from `0xF7..0xFE` and compensation output against known environmental ranges.
- Verify forced mode conversion timing uses deadlines and does not block with `delay()`.
- Verify normal mode polling through `tick()`.
- Verify soft reset waits for NVM copy completion using bounded polling.
- Verify health transitions on injected timeout/NACK and recovery on later success.

## ESP-IDF v6.0.1 Hazards

- Do not include `<driver/i2c.h>` or use legacy APIs such as `i2c_param_config()`, `i2c_driver_install()`, `i2c_cmd_link_create()`, or command-link transactions.
- Use `<driver/i2c_master.h>` and the `esp_driver_i2c` component dependency.
- IDF builds commonly surface warnings that Arduino builds hide. Keep casts explicit for enum, integer-width, and `size_t` conversions.
- Avoid global constructors that create ESP-IDF bus handles before the scheduler/runtime is initialized.
- `esp_timer_get_time()` returns microseconds as `int64_t`; down-convert deliberately for `uint32_t` millisecond wrap-safe logic.
- Do not enable the BME280 `spi3w_en` config bit in an I2C-only port.
- Keep reserved register bits preserved during config writes.

## Ordered Checklist

1. Add a private timing shim and remove direct `<Arduino.h>` use from `src/BME280.cpp`. Done.
2. Add a minimal component `CMakeLists.txt` for the core library. Done.
3. Add an IDF I2C adapter using `<driver/i2c_master.h>` outside the core driver. Done.
4. Add `examples/idf/basic` with bus setup, adapter callbacks, and a polling task. Done.
   Include top-level and `main` CMake files, component path wiring, and
   `extern "C" void app_main(void)`. Done.
5. Build with ESP-IDF v6.0.1 for ESP32-S2 and ESP32-S3. Pending local ESP-IDF environment.
6. Run Arduino and native builds to confirm existing users are unaffected. Pending local validation.
7. Run hardware tests for probe, forced mode, normal mode, reset, compensation, and fault injection. Pending hardware.
8. Update README and changelog for the implemented port. Done.
