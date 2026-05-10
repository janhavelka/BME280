# Variants And Open Questions

## BMP280 Compatibility

The BME280 is downward register-compatible with BMP280 for pressure and temperature control/readout, with these driver-visible differences. Source: datasheet, p. 26.

| Area | BME280 fact | Driver impact |
|---|---|---|
| Chip ID | `id` register reads `0x60`. | Do not accept BMP280 IDs unless the library explicitly supports them. |
| `config.t_sb` encodings `110` and `111` | BME280 uses 10 ms and 20 ms, unlike BMP280's longer standby encodings. | Keep BME280-specific standby enum values. |
| Pressure/temp resolution | With filter enabled, output resolution is always 20 bit. | Compensation/readout can use the normal 20-bit assembly. |
| Humidity registers | `ctrl_hum`, humidity calibration, and `0xFD..0xFE` are BME280-specific. | Humidity setup must be applied by writing `ctrl_meas` after `ctrl_hum`. |

## Datasheet Revision Notes

- The source inventory uses Rev. 1.24, February 2024. The revision history says Rev. 1.24 removed "condensation" from chapter 2. Source: datasheet, p. 59.
- Some extracted footer text still mentions Rev. 1.23; rely on the cover/document history when citing the document revision. Source: datasheet, pp. 1, 59-60.

## Open Questions For Implementation

- Exact package drawing dimensions, landing pattern, and tape/reel data are figure-heavy; inspect the PDF visually before using them in CAD output. Source: datasheet, pp. 38, 42-47.
- Compensation formulas are long and type-sensitive; use the source PDF or vendor reference code when implementing fixed-point compensation. Source: datasheet, pp. 25, 49-50.

## Interface Scope Choices

| Interface path | BME280 fact | Source |
|---|---|---|
| I2C | Address is `0x76` with `SDO=GND` or `0x77` with `SDO=VDDIO`; no clock stretching; standard, fast, and high-speed modes are supported. | Datasheet, pp. 32-36 |
| SPI 4-wire | Supported up to 10 MHz; mode 0 and mode 3; read/write bit is bit 7 of the transferred control byte. | Datasheet, pp. 34, 37 |
| SPI 3-wire | Supported after setting `config.spi3w_en` (`0xF5[0]`) to 1. | Datasheet, pp. 30, 34 |
| BMP280 fallback | BME280 `id=0x60`; BMP280 lacks `ctrl_hum`, humidity calibration, and humidity data registers. | Datasheet, pp. 26-31 |
