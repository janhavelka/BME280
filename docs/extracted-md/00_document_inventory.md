# Document Inventory

Compact notes in `docs/extracted-md/` summarize the BME280 Rev. 1.24 driver facts. Raw page-oriented extraction remains in `docs/pdf-extracted-md/` for exact Bosch wording, long compensation code, and figure-heavy package data.

| Source PDF | Raw extract | Role | Revision / date | Pages | Notes |
|---|---|---|---|---:|---|
| `docs/BME280_datasheet.pdf` | `docs/pdf-extracted-md/BME280_datasheet.md` | Primary datasheet | Rev. 1.24, February 2024 | 60 | Electrical limits, pinout, modes, register map, compensation data, I2C/SPI protocol, package and handling notes. |

## Compact Note Set

| File | Purpose |
|---|---|
| `01_chip_overview.md` | Sensor purpose, measurement channels, ranges, and driver-relevant capabilities. |
| `02_pinout_and_signals.md` | Pins, I2C address strap, interface selection, and basic wiring. |
| `03_electrical_and_timing.md` | Supply limits, current, timing, interface speeds, and absolute maximum ratings. |
| `04_protocol_commands_and_transactions.md` | I2C/SPI transaction rules and raw data readout behavior. |
| `05_register_map.md` | Driver-facing registers, bit fields, reset values, and calibration/data regions. |
| `06_modes_interrupts_status_and_faults.md` | Sleep/forced/normal modes, status bits, filtering, and self-test return codes. |
| `07_initialization_reset_and_operational_notes.md` | Startup, reset, recommended setup sequence, readout, compensation, and handling notes. |
| `08_variant_differences_and_open_questions.md` | BMP280 compatibility notes, revision notes, and items that require checking the PDF. |

## Scope Notes

- The compact files intentionally omit legal boilerplate, raw page dumps, and OCR artifacts.
- Values are cited by document and page. Check the raw extract or PDF before using figure-only geometry or package artwork.
- Units are written in ASCII-friendly form (`degC`, `uA`, `+/-`) for portability.
