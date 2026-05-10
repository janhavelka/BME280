# Chip Overview

The BME280 is a combined digital humidity, pressure, and temperature sensor in a 2.5 mm x 2.5 mm x 0.93 mm metal-lid LGA package. It supports I2C and SPI host interfaces, has separate sensor and interface supplies, and can run in sleep, forced, or normal modes. Source: datasheet, pp. 2-3.

## Driver-Relevant Capabilities

| Area | Facts | Source |
|---|---|---|
| Measurement channels | Relative humidity, barometric pressure, and temperature. Temperature is used internally for humidity and pressure compensation and can also be reported. | Datasheet, p. 3 |
| Interfaces | I2C up to 3.4 MHz; SPI 3-wire or 4-wire up to 10 MHz. | Datasheet, pp. 2, 32, 37 |
| Supplies | `VDD` 1.71 V to 3.6 V; `VDDIO` 1.2 V to 3.6 V. | Datasheet, pp. 2, 8 |
| Operating range | -40 degC to +85 degC, 0 to 100 %RH, 300 to 1100 hPa. Humidity range is non-condensing. | Datasheet, pp. 2, 9-10 |
| Low-power behavior | Sleep current is typ. 0.1 uA; after power-up the device enters sleep mode. | Datasheet, pp. 2, 8, 14-15 |
| Compatibility | Pressure and temperature control/readout are downward register-compatible with BMP280, with documented exceptions. | Datasheet, p. 26 |

## Output Data

- Raw pressure and temperature are 20-bit values read from `0xF7..0xFC`; raw humidity is a 16-bit value read from `0xFD..0xFE`. Source: datasheet, pp. 23, 30-31.
- Bosch trims every BME280 at production; compensation coefficients are stored as `dig_T1..dig_T3`, `dig_P1..dig_P9`, and `dig_H1..dig_H6` in `0x88..0xA1` and `0xE1..0xE7`. Source: datasheet, pp. 24, 26-27.
- Temperature compensation produces `t_fine`, a signed 32-bit intermediate reused by pressure and humidity compensation; the integer API returns temperature in 0.01 degC, pressure in Pa, and humidity in 1/1024 %RH. Source: datasheet, pp. 25, 49-50.
