# Register Map

## Main Registers

| Address | Name | Access | Reset | Driver notes | Source |
|---:|---|---|---:|---|---|
| `0x88..0xA1` | `calib00..calib25` | R | individual | First calibration block. | Datasheet, p. 27 |
| `0xD0` | `id` | R | `0x60` | Chip ID value is `0x60`. | Datasheet, p. 27 |
| `0xE0` | `reset` | W | read as `0x00` | Write `0xB6` for soft reset; other values have no effect. | Datasheet, p. 27 |
| `0xE1..0xF0` | `calib26..calib41` | R | individual | Second calibration block. | Datasheet, p. 27 |
| `0xF2` | `ctrl_hum` | R/W | `0x00` | Humidity oversampling. Changes apply only after writing `ctrl_meas`. | Datasheet, pp. 27-28 |
| `0xF3` | `status` | R | `0x00` | `measuring` bit 3, `im_update` bit 0. | Datasheet, p. 28 |
| `0xF4` | `ctrl_meas` | R/W | `0x00` | Temperature oversampling, pressure oversampling, mode. | Datasheet, p. 29 |
| `0xF5` | `config` | R/W | `0x00` | Standby time, IIR filter, SPI 3-wire enable. Writes in normal mode may be ignored. | Datasheet, p. 30 |
| `0xF7..0xF9` | `press` | R | `0x80000`-style split | Raw pressure `up[19:0]`. | Datasheet, pp. 27, 30-31 |
| `0xFA..0xFC` | `temp` | R | `0x80000`-style split | Raw temperature `ut[19:0]`. | Datasheet, pp. 27, 31 |
| `0xFD..0xFE` | `hum` | R | `0x8000`-style split | Raw humidity `uh[15:0]`. | Datasheet, pp. 27, 31 |

## Encoding Tables

| Field | Encoding | Source |
|---|---|---|
| `osrs_h`, `osrs_p`, `osrs_t` | `000` skipped; `001` x1; `010` x2; `011` x4; `100` x8; `101` and others x16. Skipped pressure/temp output is `0x80000`; skipped humidity output is `0x8000`. | Datasheet, pp. 28-29 |
| `mode[1:0]` | `00` sleep; `01` and `10` forced; `11` normal. | Datasheet, p. 29 |
| `t_sb[2:0]` | `000` 0.5 ms, `001` 62.5 ms, `010` 125 ms, `011` 250 ms, `100` 500 ms, `101` 1000 ms, `110` 10 ms, `111` 20 ms. | Datasheet, p. 30 |
| `filter[2:0]` | `000` off; `001` 2; `010` 4; `011` 8; `100` and others 16. | Datasheet, p. 30 |

## Calibration Byte Order

| Coefficient | Storage | Type | Source |
|---|---|---|---|
| `dig_T1` | `0x88` LSB, `0x89` MSB | unsigned 16-bit | Datasheet, p. 24 |
| `dig_T2..dig_T3` | `0x8A..0x8D`, LSB first per word | signed 16-bit | Datasheet, p. 24 |
| `dig_P1` | `0x8E` LSB, `0x8F` MSB | unsigned 16-bit | Datasheet, p. 24 |
| `dig_P2..dig_P9` | `0x90..0x9F`, LSB first per word | signed 16-bit | Datasheet, p. 24 |
| `dig_H1` | `0xA1` | unsigned 8-bit | Datasheet, p. 24 |
| `dig_H2` | `0xE1` LSB, `0xE2` MSB | signed 16-bit | Datasheet, p. 24 |
| `dig_H3` | `0xE3` | unsigned 8-bit | Datasheet, p. 24 |
| `dig_H4` | `0xE4[7:0]` as bits `[11:4]`, `0xE5[3:0]` as bits `[3:0]` | signed 12-bit in 16-bit container | Datasheet, p. 24 |
| `dig_H5` | `0xE5[7:4]` as bits `[3:0]`, `0xE6[7:0]` as bits `[11:4]` | signed 12-bit in 16-bit container | Datasheet, p. 24 |
| `dig_H6` | `0xE7` | signed 8-bit | Datasheet, p. 24 |

## Reserved Bits

Reserved register holes in the `0x00..0xFF` map have no BME280 function listed in the datasheet; keep writes inside documented addresses `0x88..0xA1`, `0xD0`, `0xE0`, `0xE1..0xF0`, and `0xF2..0xFE`. Source: datasheet, p. 26.
