# BME280 ESP-IDF Portability Status

Last audited: 2026-03-01

## Current Reality
- Primary runtime remains PlatformIO + Arduino.
- Core transport is callback-based (`Config.i2cWrite`, `Config.i2cWriteRead`).
- Optional timing hook is available (`Config.nowMs`, `Config.timeUser`).
- Core measurement/recovery logic uses `_nowMs()` wrapper.
- Arduino timing is used only as fallback in one place:
  - `BME280::_nowMs()` -> `millis()` when `Config.nowMs == nullptr`

## ESP-IDF Adapter Requirements
To run under pure ESP-IDF, provide:
1. I2C write callback.
2. I2C write-read callback.
3. Optional `nowMs(user)` callback.

## Minimal Adapter Pattern
```cpp
static uint32_t idfNowMs(void*) {
  return static_cast<uint32_t>(esp_timer_get_time() / 1000ULL);
}

BME280::Config cfg{};
cfg.i2cWrite = myI2cWrite;
cfg.i2cWriteRead = myI2cWriteRead;
cfg.nowMs = idfNowMs;
```

## Porting Notes
- Keep `tick(nowMs)` driven by application scheduler/task.
- Callback timeout arguments must be honored to preserve recovery semantics.
- Health timestamps (`lastOkMs`, `lastErrorMs`) are sourced from `_nowMs()`.

## Verification Checklist
- `python tools/check_core_timing_guard.py` passes.
- `pio test -e native` passes.
- Example builds pass (`pio run -e esp32s3dev`, `pio run -e esp32s2dev`).
- No new direct Arduino timing calls outside wrapper fallback.
