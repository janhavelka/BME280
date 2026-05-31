# BME280 Phase 05 - Examples, ESP-IDF CI, Packaging, and Documentation

Date: 2026-05-31

Branch: `hardening/bme280-industry-readiness`

Commit target: `hardening: improve BME280 examples and CI docs`

## Scope

Phase 05 was limited to examples, ESP-IDF CI, package validation, and public
documentation. Core compensation, timing, reset, and dirty-state behavior were
not reworked.

## Starting State

Required start commands were run:

| Command | Result |
|---------|--------|
| `git status --short` | PASS: clean at phase start |
| `git branch --show-current` | PASS: `hardening/bme280-industry-readiness` |
| `git log --oneline -8` | PASS: latest commit was `3dd72de hardening: improve BME280 reset and fault diagnostics` |

## Subagent Review

| Agent | Result |
|-------|--------|
| `idf-ci-agent` | Found package omission of `Version.h`, ESP-IDF CI version mismatch, missing version/package CI checks, and native `Wire` inheritance. |
| `docs-hardware-agent` | Found missing public wiring notes, shared-bus guidance, hardware validation matrix, latency table, and stale IDF reset wording. |
| `core-contracts-agent` | Found stale cached-sample/recover docs, dirty-state visibility gaps in examples, and the ESP-IDF version mismatch. |
| `bme280-datasheet-agent` | Confirmed core datasheet facts were mostly preserved; requested SDO/VDDIO docs, validity flags in example output, filter caveats, and self-test caveats. |
| `integration-review-agent` | No blocking code/build issues; confirmed new files must be explicitly added and no hardware/local IDF validation was overclaimed. |

## Example and CLI Changes

- Arduino and ESP-IDF diagnostic CLIs now expose `addr [0x76|0x77]`.
- `id` is now accepted as an alias for `chipid`.
- `force` triggers one forced-mode measurement.
- `normal on/off` switches normal mode on or returns to sleep.
- `status` now prints status register bits plus driver state, online flag, and dirty flag.
- `drv`, settings output, state views, probe/recover diffs, and recover output expose dirty-state diagnostics more clearly.
- Raw and compensated cached sample commands now print validity flags and cached sample age.
- Measurement output prints channel validity flags.
- `calib` now includes a simple plausibility line rather than only raw coefficient values.
- `selftest` is labeled as a safe command smoke check with loose, environment-dependent plausibility ranges. It is not presented as factory calibration or hardware qualification.
- ESP-IDF example remains native: `app_main`, `driver/i2c_master.h`, FreeRTOS queue/task timing, fixed command buffers, and no Arduino facade.

## ESP-IDF Component and CI Status

- Root `CMakeLists.txt` remains a component registration for `src/BME280.cpp` and `include`.
- `idf_component.yml` remains version `1.5.0`, targets `esp32s2` and `esp32s3`, and requires `idf >=6.0.1`.
- CI ESP-IDF build job now uses `esp_idf_version: v6.0.1` for both `esp32s3` and `esp32s2`, matching the component metadata.
- CI now checks generated `Version.h` freshness and package contents after `pio pkg pack`.
- Local `idf.py` is not installed, so no local ESP-IDF build pass is claimed.
- Espressif's action documentation says `esp_idf_version` must be a Docker Hub tag, and Docker Hub search showed `espressif/idf:v6.0.1` is available.

## Packaging and Version Status

- `include/BME280/Version.h` is no longer ignored by `.gitignore`, so the public include graph is package-complete.
- `tools/check_package_contents.py` validates that the PlatformIO archive includes public headers, `Version.h`, `src/BME280.cpp`, and `library.json`, and excludes build/internal paths.
- `platformio.ini` now clears `lib_deps` for `[env:native]`, so native tests no longer inherit Arduino `Wire`.
- `python -m platformio project config --json-output` confirmed `env:native` has `lib_deps = []`.
- `python -m platformio pkg pack` generated `BME280-1.5.0.tar.gz`; it was validated and deleted before commit so no package archive is committed.

## Documentation Updates

- README now covers SDO address selection, SDO non-floating requirement, CSB tied high to VDDIO for I2C, VDD/VDDIO cautions, external pullups, decoupling, and humidity handling caveats.
- README now includes shared-bus ownership/locking/scheduling guidance.
- README now includes a blocking latency table for lifecycle, measurement, setters, recover, and reset.
- README documents cached samples after `recover()` and the need to request a fresh sample before control use.
- README documents IIR filter scope: pressure/temperature only, not humidity, and filter memory reset after changes.
- `docs/BME280_HARDWARE_VALIDATION_MATRIX.md` was added with explicit `NOT RUN` hardware rows.
- `docs/IDF_PORT.md` now says `tick()` drives measurement polling only; reset/NVM waits are blocking but bounded.
- `Doxyfile` now narrows docs input to current public docs and includes the ESP-IDF example path.
- Public comments now use `SDO=VDDIO` and reserve `DEVICE_NOT_FOUND` for definite address NACK/device absence.

## Checks Run

| Command | Result |
|---------|--------|
| `python tools/check_core_timing_guard.py` | PASS: `Core timing guard PASSED` |
| `python tools/check_cli_contract.py` | PASS: `CLI contract PASSED` |
| `python tools/check_idf_example_contract.py` | PASS: `IDF example contract PASSED` |
| `python scripts/generate_version.py check` | PASS: `include\BME280\Version.h` up to date |
| `python -m platformio test -e native` | PASS on clean rerun: 84/84 tests passed |
| `python -m platformio run -e esp32s3dev` | PASS: `esp32s3dev SUCCESS` |
| `python -m platformio run -e esp32s2dev` | PASS: `esp32s2dev SUCCESS` |
| `python -m platformio pkg pack` | PASS: wrote `BME280-1.5.0.tar.gz` |
| `python tools/check_package_contents.py` | PASS: package contents validated |
| `python -m platformio project config --json-output` | PASS: verified native `lib_deps` is empty |
| `git diff --check` | PASS: no whitespace errors; Git emitted CRLF working-copy warnings only |
| `idf.py --version` | FAIL/NOT AVAILABLE: `idf.py` is not recognized as a command |

Native test note: the first `python -m platformio test -e native` attempt hit a
local build-cache/SCons error after dependency pruning:
missing `.pio\build\native\src\src\BME280.o`,
missing `.pio\build\native\lib795\libUnity.a`, and missing
`.pio\build\native\.sconsign313.tmp`. After deleting `.pio\build\native`, the
same command passed with 84/84 tests.

## Checks Not Run

- Local ESP-IDF builds were not run because `idf.py` is not installed in this shell.
- No physical BME280 hardware validation was run.
- No logic-analyzer or electrical validation was run.

## Remaining Hardware Validation Tasks

See `docs/BME280_HARDWARE_VALIDATION_MATRIX.md`. Remaining work includes:

- Validate both addresses `0x76` and `0x77` with correct SDO wiring.
- Capture chip ID `0x60`, reset/NVM behavior, forced mode, normal mode, burst reads, and calibration plausibility on real hardware.
- Validate humidity behavior in non-condensing conditions after assembly handling.
- Validate timeout/NACK/bus fault mapping and recovery on a hardware or fault-injection bench.
- Validate a production shared-bus integration with external locking and timeout policy.
