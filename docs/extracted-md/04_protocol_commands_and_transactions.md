# Protocol Commands And Transactions

The BME280 is a register-mapped slave device. There is no separate command opcode layer beyond register reads and writes. Source: datasheet, pp. 26, 32.

## I2C

| Item | Behavior | Source |
|---|---|---|
| 7-bit address | `0x76` when `SDO` is tied to GND; `0x77` when `SDO` is tied to `VDDIO`. | Datasheet, pp. 32-33 |
| Address write/read bytes | Wire address is `111011X0` for write and `111011X1` for read, where `X` is the `SDO` strap. | Datasheet, pp. 33 |
| Write transaction | Master sends address+write, then pairs of register address and register data; addresses are not auto-incremented for multi-byte writes. | Datasheet, p. 33 |
| Read transaction | Master first writes the register address, then issues STOP or repeated START and address+read; data bytes come from auto-incremented register addresses until NACK/STOP. | Datasheet, p. 33 |
| Clock stretching | Not used by the device. | Datasheet, p. 32 |

## SPI

| Item | Behavior | Source |
|---|---|---|
| Modes | SPI mode 0 (`CPOL=0`, `CPHA=0`) and mode 3 (`CPOL=1`, `CPHA=1`) are supported. | Datasheet, pp. 32, 34 |
| 3-wire enable | Set `config.spi3w_en` (`0xF5[0]`) to 1 to use 3-wire SPI. | Datasheet, pp. 30, 34 |
| Register address | SPI uses 7 address bits; bit 7 is read/write (`1` read, `0` write). Example: full register `0xF7` uses SPI address `0x77`; read byte is `0xF7`, write byte is `0x77`. | Datasheet, p. 34 |
| Multi-byte SPI write | Send control-byte/data pairs while `CSB` remains low; no auto-increment form for writes. | Datasheet, p. 34 |
| Multi-byte SPI read | Send one read control byte; returned data auto-increments register address. | Datasheet, p. 35 |

## Raw Measurement Readout

- Burst-read `0xF7..0xFE` to collect pressure, temperature, and humidity in one transaction. Source: datasheet, p. 23.
- Pressure and temperature are unsigned 20-bit raw values; humidity is unsigned 16-bit raw. Source: datasheet, pp. 23, 30-31.
- The datasheet notes that in I2C mode, even when pressure was not measured, reading the unused registers can be faster than separate temperature/humidity reads. Source: datasheet, p. 23.
