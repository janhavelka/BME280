# BME280 v1.1.1 Release Notes
Date: 2026-02-28

## Highlights
- Unified around the common examples/01_basic_bringup_cli structure and command baseline.
- Standardized CLI help and runtime output style across I2C libraries with functional color usage for actionable states.
- Improved health and lifecycle consistency (begin, tick, end, probe, recover) with clearer state/counter diagnostics.
- Expanded safe stress, stress_mix, and selftest workflows and reporting.
- Added portability-oriented timing abstraction patterns to keep PlatformIO + Arduino stable while easing future ESP-IDF migration.
- Refreshed docs including unification and porting guidance.
- Expanded environmental sensor diagnostics and settings CLI flows with unified reporting format.

## Compatibility and Migration
- This release prioritizes API and CLI consistency across libraries; limited compatibility shims may remain where practical.
- In-repo consumers were updated toward the canonical interfaces and naming style.

## Tag
- v1.1.1

## Suggested GitHub Release Title
- BME280 v1.1.1
