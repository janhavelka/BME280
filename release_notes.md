# BME280 v1.3.0 Release Notes
Date: 2026-04-03

## Highlights
- Added granular `I2C_*` transport status codes for example adapters and diagnostics.
- Restored the documented `Config::nowMs` fallback behavior by falling back to `millis()` when no callback is injected.
- Standardized the example `Wire` adapter around `i2cUser` / `TwoWire*` and manager-owned timeout configuration.
- Expanded native coverage for timing fallback and transport error mapping, and refreshed the README documentation inventory.

## Compatibility
- Public APIs remain backward compatible.
- The `Err` enum is append-only in this release.

## Tag
- `v1.3.0`

## Suggested GitHub Release Title
- `BME280 v1.3.0`
