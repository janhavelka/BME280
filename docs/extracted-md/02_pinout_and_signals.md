# Pinout And Signals

## Pin Summary

| Pin | Name | Driver/wiring meaning | Source |
|---:|---|---|---|
| 1 | `GND` | Ground. | Datasheet, p. 38 |
| 2 | `CSB` | SPI chip select, active low. Tie to `VDDIO` for I2C. Internal pull-up is present. | Datasheet, pp. 32-34, 38-39 |
| 3 | `SDI` | SPI data input; I2C data (`SDA`). In I2C it is bidirectional/open-drain to GND and needs an external pull-up. | Datasheet, pp. 33-34, 38-39 |
| 4 | `SCK` | SPI serial clock; I2C clock (`SCL`). | Datasheet, pp. 33-34, 38-39 |
| 5 | `SDO` | SPI data output; in I2C selects address LSB. Tie to GND for `0x76`, to `VDDIO` for `0x77`; do not leave floating. | Datasheet, pp. 32-33, 38-39 |
| 6 | `VDDIO` | Digital interface supply, 1.2 V to 3.6 V. | Datasheet, pp. 8, 38 |
| 7 | `GND` | Ground. | Datasheet, p. 38 |
| 8 | `VDD` | Main analog/digital supply, 1.71 V to 3.6 V. | Datasheet, pp. 8, 38 |

## Interface Selection

- `CSB` high at power-on selects I2C. If `CSB` is pulled low once, SPI is selected and I2C is disabled until the next power-on reset. Source: datasheet, p. 32.
- If `CSB` is driven by a programmable host pin for I2C use, it must already be at `VDDIO` level during BME280 power-on reset. Source: datasheet, p. 32.
- In I2C mode, `SCK` is `SCL`, `SDI` is `SDA`, and `SDO` is the address strap. Source: datasheet, p. 33.
