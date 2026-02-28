# Unification Standard - BME280

## Scope
- Primary target: PlatformIO + Arduino on ESP32-S2/S3.
- Production stability, bounded timing, and deterministic behavior are first priority.
- Controlled breaking changes are allowed when they improve clarity, safety, or robustness.

## Public API Contract
- Lifecycle methods required: begin, tick, end, probe, recover.
- Health methods required: state, isOnline, lastError, lastOkMs, lastErrorMs, consecutiveFailures, totalFailures, totalSuccess.
- Status shape: Status { Err code; int32_t detail; const char* msg; }.

## Example Contract
- Required example: examples/01_basic_bringup_cli/.
- Required common headers:
  - examples/common/BoardConfig.h
  - examples/common/BuildConfig.h
  - examples/common/Log.h
  - examples/common/TransportAdapter.h
  - examples/common/BusDiag.h
  - examples/common/CliShell.h
  - examples/common/HealthView.h
- Required CLI commands: help, scan, probe, recover, drv, read, cfg|settings, verbose 0|1, stress [N].

## Portability Contract
- Core logic must not depend directly on Arduino timing APIs.
- Time/delay access must go through configurable hooks.
- Arduino adapters remain default for current runtime behavior.
- Portability work must preserve current safety and stability.

## Quality Gates
- platformio.ini must provide a native test environment.
- CI must build example envs and run native tests.
- Metadata and docs must remain valid and consistent.
