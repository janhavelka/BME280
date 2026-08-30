# BME280 ESP-IDF Port

This document is the maintained ESP-IDF note for the BME280 library.

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
recover, self-test, and stress commands. Parity also covers the full typed
settings grammar (`settings values|validate|start|set` and the individual
mode/oversampling/filter/standby setters), bounded `dump`/`rregs` reads,
zero-I2C `end`/`invalidate`, cache-only sample `freshness`, and example-only
`xfer_reset`/`xfer_stats`/`xfer_assert` callback-count diagnostics.

## Ownership Boundary

The driver does not own I2C resources. The application or example owns:

- bus and device handles;
- SDA/SCL pins;
- pull-ups and bus speed;
- timeout policy;
- bus locking/mutexes;
- reset or power-control GPIOs;
- task scheduling for `pollJob()` / `tick()` and the end-to-end owner deadline.

The application passes callbacks through `BME280::Config`:

- `i2cWrite`;
- `i2cWriteRead`;
- `i2cUser`;
- `nowMs`;
- `timeUser`.

`Config::nowMs` should be set in ESP-IDF applications. The core fallback is
intentionally inert and is not a platform clock. `i2cTimeoutMs` bounds one
transport callback; `conversionReadyTimeoutMs` separately controls BME280
conversion/idle readiness grace.

## Transport Contract

The native IDF adapter uses the modern ESP-IDF I2C master driver:

```cpp
#include <driver/i2c_master.h>
```

## Porting Notes

- For a production shared-bus owner, use zero-I2C staged starts and advance the
  active operation with `pollJob(nowMs, 1)` from the owning task. The accepted
  job has a nonzero `jobId`; match every progress/terminal result to it.
- Keep the end-to-end deadline in the ESP-IDF owner, including queue time.
  `JobPollResult::phaseDeadlineMs` is only the current chip-phase deadline. On
  expiry call zero-I2C `cancelJob(DEADLINE_EXPIRED)`, then retrieve the retained
  cancellation with exactly one `pollJob()` call.
- Capture natural `DONE` / `FAILED` from the poll that reaches the transition;
  it is not retained. A pending cancellation result blocks later starts and
  hardware-facing synchronous APIs until retrieved.
- Keep `tick(nowMs)` driven by the application scheduler/task when using the
  synchronous compatibility measurement API.
- Callback timeout arguments must be honored to preserve recovery semantics.
- Preserve transport error detail: map address NACK to optional absence only at
  chip-ID presence checks, and keep timeout/bus/data NACK faults distinct.
- `Config::nvmReadyTimeoutMs` controls the visible NVM-ready deadline after POR
  or reset. Init and non-reset resync request sleep and confirm idle before the
  NVM check and calibration reads. Synchronous checks perform one NVM-status
  transaction per call and return `BUSY`, `TIMEOUT`, or the detailed transport
  error; staged jobs poll through `pollJob()` with a fixed
  255-NVM-status-callback cap.
  The library exposes no writable-NVM or factory-programming API.
- Use `startResyncJob()` for non-reset cooperative identity/calibration/config
  resynchronization. `startRecoveryJob()` is its compatibility alias. Use
  `startSoftResetJob()` only for an explicit sensor reset; shared-bus recovery
  does not imply a device reset.
- Use `SensorSettings`, pure `validateSettings()`, and
  `startApplySettingsJob()` for one coherent runtime settings change. Admission
  performs no callback; before any possible config-write effect cancellation
  restores prior settings, while a later cancellation retains the desired
  settings with `RESYNC_REQUIRED` for explicit reconciliation.
- The native and Arduino diagnostic CLIs must keep identical staged-job help and
  behavior for status/init/force/apply/resync/reset/cancel/poll, including
  job identity, phase, callback use, deadline, and conversion-state output.
- Keep typed-setting command parsing strict and exact-arity in both frontends.
  Invalid text or invalid complete tuples must be rejected before any
  convenience cancellation can touch I2C. The callback counters count
  validated adapter callback attempts, saturate at `UINT32_MAX`, and remain
  example/HIL instrumentation rather than core-library state.
- Drain and discard a complete input line after the fixed command buffer
  overflows. Report `Command too long` and do not execute the buffered prefix.
- CLI convenience cancellation must propagate a failed pending measurement or
  mode-restoration status; it must not silently discard that hardware error.
- A terminal `tick()` measurement error must complete the example's pending
  read/stress iteration. Direct `recover` must first release pending
  measurement or staged-job ownership. Mixed stress and self-test must restore
  and verify all pre-command `SensorSettings`, with restoration failure included
  in their deterministic result.
- Convert millisecond CLI job pacing to FreeRTOS ticks with a minimum delay of
  one tick so a configured 1 ms cooperative pause never degenerates into a
  tight loop when `configTICK_RATE_HZ` is below 1000.
- `pollJob(nowMs, ...)` and `tick(nowMs)` make their argument authoritative for
  health events within the call. Other synchronous operations use
  `Config::nowMs` when supplied. If neither source exists, timestamps are zero
  and `lastOkTimeValid()` / `lastErrorTimeValid()` are false.
- `OFFLINE` is observational and does not reject an explicit owner-directed I2C
  operation. ESP-IDF application policy remains responsible for retry,
  backoff, resync, reset, and retirement.
- A staged forced-trigger failure or post-trigger cancellation can expose
  `ConversionState::UNKNOWN_AFTER_TRIGGER_ERROR`. A later forced job reads
  `status.measuring` before issuing another trigger; it does not replay the
  ambiguous write. Synchronized steady sampling does not rewrite `ctrl_hum`.

Expected callback behavior:

- `addr` is a 7-bit BME280 address, `0x76` or `0x77`.
- `i2cWrite()` maps to `i2c_master_transmit()`.
- `i2cWriteRead()` maps to `i2c_master_transmit_receive()` for register reads.
- The callback makes exactly one physical attempt and completes synchronously
  before returning. It performs no retry or bus recovery.
- A write-read is one combined pointer-write/repeated-START/read transaction;
  no STOP is inserted between its phases.
- `TransportResult::OK` reports exact write/read byte counts. Short counts are
  reported by the core as `Err::I2C_SHORT_TRANSFER`.
- `timeoutMs` is clamped or rejected before passing it to ESP-IDF so it cannot
  become an accidental infinite wait.
- Transport callbacks must not recursively call into the same `BME280::BME280`
  instance.

Simple ESP-IDF transfer APIs do not always reveal whether a NACK happened during
address or data phase. Preserve precise errors when the adapter can prove them;
otherwise return `TransportErr::OTHER` with the raw `esp_err_t` value in
`TransportResult::detail`. In particular, the example maps
`ESP_ERR_INVALID_RESPONSE` to `OTHER`, not to address absence, because this API
does not identify the failed phase. The core maps terminal transport results to
canonical driver `Status` values and never stores adapter-owned text.

## Cooperative Bounds

Staged start, `cancelJob()`, and `end()` issue zero callbacks. While a staged
job is running or waiting, it exclusively owns hardware-facing access;
conflicting fallible synchronous calls return `BUSY` without invoking the IDF
adapter, and `tick()` performs no I2C.
Each `pollJob(nowMs, budget)` call invokes the adapter no more than `budget`
times. A zero budget can still traverse bounded local-only phases.

The library uses a 255-poll cap for NVM readiness and a 255-counter cap for each
measuring/idle wait, with at most one final status callback before the latter
reports its poll-limit error. With no earlier deadline or adapter failure, the
cumulative staged callback caps are 519 for init/non-reset resync, 520 for
explicit soft reset, 261 for config apply, 258 for forced measurement from a
known-idle state, and 514 when forced measurement first reconciles an ambiguous
trigger. These are callback caps, not an overall owner deadline or elapsed-time
claim.

Pure `estimateMeasurementTimeUs(const SensorSettings&)` implements the exact
Bosch maximum formula. The millisecond helper adds the library's fixed 1 ms
scheduling margin and rounds up. Checked fixed-point helpers convert
centi-Celsius to signed milli-Celsius and Q22.10 relative humidity to signed
milli-percent without heap, float, I2C, or platform dependencies.

Current example mapping:

| ESP-IDF result | Adapter result | Core status |
| --- | --- | --- |
| `ESP_OK` | `TransportErr::OK` with exact counts | `Err::OK` |
| `ESP_ERR_TIMEOUT` | `TransportErr::TIMEOUT` | `Err::I2C_TIMEOUT` |
| `ESP_ERR_INVALID_ARG` | `TransportErr::OTHER` | `Err::I2C_ERROR` |
| `ESP_ERR_INVALID_RESPONSE` | `TransportErr::OTHER` because the failed phase is unknown | `Err::I2C_ERROR` |
| other controller failures | `TransportErr::BUS` | `Err::I2C_BUS` |

## Build Checks

The full repository validation gate is the **Validation** section of
`README.md`; run it from the repository root. The checks that specifically
cover this port are `tools/check_idf_example_contract.py` (IDF example
structure and CLI parity) and `tools/check_cli_contract.py` (Arduino/IDF
command parity).

When `idf.py` is installed, also run:

```bash
idf.py -C examples/idf/basic set-target esp32s3
idf.py -C examples/idf/basic build
idf.py -C examples/idf/basic set-target esp32s2
idf.py -C examples/idf/basic build
```

The example CMake configuration embeds the repository's short Git commit and
clean/dirty state in the CLI `version` output. Both tracked and untracked files
make the build dirty. If Git metadata is unavailable it reports `unknown`, and
the HIL runner treats that firmware provenance as review-only rather than an
exact-build qualification.

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

Use `docs/HARDWARE_VALIDATION.md` for the procedure, evidence schema, and
result ledger.

For production shared-bus application structure, use
`docs/PRODUCTION_SHARED_BUS_GUIDE.md`. The shipped Arduino and ESP-IDF examples
are diagnostic CLIs, not complete production firmware templates.

`docs/README.md` is the maintained documentation map. Generated Doxygen output,
HIL logs, and build artifacts are local outputs unless a release or validation
package explicitly records them.
