# BME280 -- ESP-IDF Migration Prompt

> **Library**: BME280 (Bosch temperature / humidity / pressure sensor)
> **Current version**: 1.1.0 -> **Target**: 2.0.0
> **Namespace**: `BME280`
> **Include path**: `#include "BME280/BME280.h"`
> **Difficulty**: Easy -- `millis()` replacement in .cpp only, I2C already callback-based

---

## Pre-Migration

```bash
git tag v1.1.0   # freeze Arduino-era version
```

---

## Current State -- Arduino Dependencies (exact)

| API | Count | Locations |
|-----|-------|-----------|
| `#include <Arduino.h>` | 1 | `.cpp` only (not in header) |
| `millis()` | 4 | Lines 312, 614, 617, 821 |

I2C is already abstracted via injected callbacks:

```cpp
using I2cWriteFn     = Status (*)(uint8_t addr, const uint8_t* data, size_t len, void* user);
using I2cWriteReadFn = Status (*)(uint8_t addr, const uint8_t* wdata, size_t wlen,
                                   uint8_t* rdata, size_t rlen, void* user);
```

Config is struct-based. Time injected via `tick(uint32_t nowMs)`.

---

## Steps

### 1. Remove `#include <Arduino.h>`

Delete the include from the .cpp file.

### 2. Replace 4x `millis()` with `esp_timer_get_time()`

Add at file scope in the .cpp:

```cpp
#include "esp_timer.h"

static inline uint32_t nowMs() {
    return (uint32_t)(esp_timer_get_time() / 1000);
}
```

Replace all 4 `millis()` call sites (lines 312, 614, 617, 821) with `nowMs()`.

**Important**: Some of these `millis()` calls may be in `begin()` or internal methods, not just `tick()`. Using `esp_timer_get_time()` is correct in all contexts since the ESP-IDF timer runs from boot.

### 3. Add `CMakeLists.txt` (library root)

```cmake
idf_component_register(
    SRCS "src/BME280.cpp"
    INCLUDE_DIRS "include"
    REQUIRES esp_timer
)
```

### 4. Add `idf_component.yml` (library root)

```yaml
version: "2.0.0"
description: "BME280 temperature/humidity/pressure sensor driver"
targets:
  - esp32s2
  - esp32s3
dependencies:
  idf: ">=5.0"
```

### 5. Version bump

- `library.json` -> `2.0.0`
- `Version.h` (if present) -> `2.0.0`

### 6. Replace Arduino example with ESP-IDF example

Create `examples/espidf_basic/main/main.cpp`:

```cpp
#include <cstdio>
#include "BME280/BME280.h"
#include "driver/i2c_master.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static i2c_master_bus_handle_t bus;
static i2c_master_dev_handle_t dev;

static BME280::Status i2cWrite(uint8_t addr, const uint8_t* data, size_t len, void*) {
    esp_err_t err = i2c_master_transmit(dev, data, len, 100);
    return err == ESP_OK ? BME280::Status{BME280::Err::Ok}
                         : BME280::Status{BME280::Err::I2cNack, "transmit failed"};
}

static BME280::Status i2cWriteRead(uint8_t addr,
                                    const uint8_t* wdata, size_t wlen,
                                    uint8_t* rdata, size_t rlen, void*) {
    esp_err_t err = i2c_master_transmit_receive(dev, wdata, wlen, rdata, rlen, 100);
    return err == ESP_OK ? BME280::Status{BME280::Err::Ok}
                         : BME280::Status{BME280::Err::I2cNack, "xfer failed"};
}

extern "C" void app_main() {
    // Init I2C bus (ESP-IDF 5.x new driver)
    i2c_master_bus_config_t busCfg{};
    busCfg.i2c_port = I2C_NUM_0;
    busCfg.sda_io_num = GPIO_NUM_8;
    busCfg.scl_io_num = GPIO_NUM_9;
    busCfg.clk_source = I2C_CLK_SRC_DEFAULT;
    busCfg.glitch_ignore_cnt = 7;
    busCfg.flags.enable_internal_pullup = true;
    i2c_new_master_bus(&busCfg, &bus);

    i2c_device_config_t devCfg{};
    devCfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    devCfg.device_address = 0x76;
    devCfg.scl_speed_hz = 100000;
    i2c_master_bus_add_device(bus, &devCfg, &dev);

    BME280::Config cfg{};
    cfg.i2cAddr = 0x76;
    cfg.i2cWrite = i2cWrite;
    cfg.i2cWriteRead = i2cWriteRead;

    BME280::Sensor sensor;
    auto st = sensor.begin(cfg);
    if (st.err != BME280::Err::Ok) {
        printf("begin() failed: %s\n", st.msg);
    }

    while (true) {
        uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
        sensor.tick(now);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
```

Create `examples/espidf_basic/main/CMakeLists.txt`:

```cmake
idf_component_register(SRCS "main.cpp" INCLUDE_DIRS "." REQUIRES driver esp_timer)
```

Create `examples/espidf_basic/CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.16)
set(EXTRA_COMPONENT_DIRS "../..")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(bme280_espidf_basic)
```

---

## Verification

```bash
cd examples/espidf_basic && idf.py set-target esp32s2 && idf.py build
```

- [ ] `idf.py build` succeeds
- [ ] Zero `#include <Arduino.h>` anywhere
- [ ] Zero `millis()` calls remaining
- [ ] Version bumped to 2.0.0
- [ ] `git tag v2.0.0`
