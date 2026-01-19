# BME280 Register & Timing Reference (implementation-ready)

Source: `bst-bme280-ds002.pdf` (Bosch Sensortec BME280 datasheet).

This file is designed so an implementation agent can build a **robust BME280 driver** without reopening the PDF.

---

## 1) Bus addressing (I2C)

7-bit I2C slave address depends on **SDO** pin:
- SDO = GND → **0x76**
- SDO = VDDIO → **0x77**

---

## 2) Core registers (data + control)

| Address | Name | Access | Reset | Notes |
|---|---|---|---|---|
| 0xFE | hum_lsb | R | 0x00 |  |
| 0xFD | hum_msb | R | 0x80 |  |
| 0xFC | temp_xlsb | R | 0x00 |  |
| 0xFB | temp_lsb | R | 0x00 |  |
| 0xFA | temp_msb | R | 0x80 |  |
| 0xF9 | press_xlsb | R | 0x00 |  |
| 0xF8 | press_lsb | R | 0x00 |  |
| 0xF7 | press_msb | R | 0x80 |  |
| 0xF5 | config | R/W | 0x00 | tstandby + IIR filter; only writable in sleep |
| 0xF4 | ctrl_meas | R/W | 0x00 | temp/press oversampling + mode |
| 0xF3 | status | R | 0x00 | bit3 measuring, bit0 im_update |
| 0xF2 | ctrl_hum | R/W | 0x00 | humidity oversampling; must write ctrl_meas after changing |
| 0xE0 | reset | W | 0x00 | write 0xB6 for soft reset (POR sequence) |
| 0xD0 | id | R | 0x60 | chip_id = 0x60 (BME280) |

### Notes on raw data formats
- Pressure raw is 20-bit: `press_msb[7:0]`, `press_lsb[7:0]`, `press_xlsb[7:4]`
- Temperature raw is 20-bit: `temp_msb[7:0]`, `temp_lsb[7:0]`, `temp_xlsb[7:4]`
- Humidity raw is 16-bit: `hum_msb[7:0]`, `hum_lsb[7:0]`

---

## 3) Calibration / trimming registers (must read at init)

| Address | Coefficient | Type |
|---|---|---|
| 0x88 / 0x89 | dig_T1 [7:0] ; [15:8] | unsigned short |
| 0x8A / 0x8B | dig_T2 [7:0] ; [15:8] | signed short |
| 0x8C / 0x8D | dig_T3 [7:0] ; [15:8] | signed short |
| 0x8E / 0x8F | dig_P1 [7:0] ; [15:8] | unsigned short |
| 0x90 / 0x91 | dig_P2 [7:0] ; [15:8] | signed short |
| 0x92 / 0x93 | dig_P3 [7:0] ; [15:8] | signed short |
| 0x94 / 0x95 | dig_P4 [7:0] ; [15:8] | signed short |
| 0x96 / 0x97 | dig_P5 [7:0] ; [15:8] | signed short |
| 0x98 / 0x99 | dig_P6 [7:0] ; [15:8] | signed short |
| 0x9A / 0x9B | dig_P7 [7:0] ; [15:8] | signed short |
| 0x9C / 0x9D | dig_P8 [7:0] ; [15:8] | signed short |
| 0x9E / 0x9F | dig_P9 [7:0] ; [15:8] | signed short |
| 0xA1 | dig_H1 [7:0] | unsigned char |
| 0xE1 / 0xE2 | dig_H2 [7:0] ; [15:8] | signed short |
| 0xE3 | dig_H3 [7:0] | unsigned char |
| 0xE4 / 0xE5[3:0] | dig_H4 [11:4] ; [3:0] | signed short |
| 0xE5[7:4] / 0xE6 | dig_H5 [3:0] ; [11:4] | signed short |
| 0xE7 | dig_H6 | signed char |

### Humidity packing detail (important)
- `dig_H4` uses bytes `0xE4` and low nibble of `0xE5`
- `dig_H5` uses high nibble of `0xE5` and `0xE6`
- `dig_H6` is `0xE7` (signed char)

---

## 4) Bitfields / enums (exact encodings)

### 4.1 ctrl_hum (0xF2)
- Bits `[2:0]` = `osrs_h`

| osrs_h[2:0] | Humidity oversampling |
|---|---|
| 000 | Skipped (output set to 0x8000) |
| 001 | oversampling ×1 |
| 010 | oversampling ×2 |
| 011 | oversampling ×4 |
| 100 | oversampling ×8 |
| 101, others | oversampling ×16 |

> Datasheet rule: after writing `ctrl_hum`, you must write `ctrl_meas` for the change to become effective.

### 4.2 status (0xF3)
- Bit 3: `measuring` (1 while conversion running)
- Bit 0: `im_update` (1 while NVM calibration data is being copied)

### 4.3 ctrl_meas (0xF4)
- Bits `[7:5]` = `osrs_t`
- Bits `[4:2]` = `osrs_p`
- Bits `[1:0]` = `mode`

| osrs_t[2:0] | Temperature oversampling |
|---|---|
| 000 | Skipped (output set to 0x80000) |
| 001 | oversampling ×1 |
| 010 | oversampling ×2 |
| 011 | oversampling ×4 |
| 100 | oversampling ×8 |
| 101, others | oversampling ×16 |

| osrs_p[2:0] | Pressure oversampling |
|---|---|
| 000 | Skipped (output set to 0x80000) |
| 001 | oversampling ×1 |
| 010 | oversampling ×2 |
| 011 | oversampling ×4 |
| 100 | oversampling ×8 |
| 101, others | oversampling ×16 |

| mode[1:0] | Mode |
|---|---|
| 00 | Sleep mode |
| 01 and 10 | Forced mode |
| 11 | Normal mode |

### 4.4 config (0xF5)
- Bits `[7:5]` = `t_sb` (standby time in normal mode)
- Bits `[4:2]` = `filter` (IIR filter)
- Bit `[0]` = `spi3w_en` (3-wire SPI enable; keep 0 for I2C)

| t_sb[2:0] | tstandby [ms] |
|---|---|
| 000 | 0.5 |
| 001 | 62.5 |
| 010 | 125 |
| 011 | 250 |
| 100 | 500 |
| 101 | 1000 |
| 110 | 10 |
| 111 | 20 |

| filter[2:0] | IIR filter coefficient |
|---|---|
| 000 | Filter off |
| 001 | 2 |
| 010 | 4 |
| 011 | 8 |
| 100, others | 16 |

> Datasheet rule: `config` is writable only in **sleep** mode. If you’re in normal mode, switch to sleep → write config → restore mode.

---

## 5) Soft reset and chip ID

### 5.1 Chip ID
- Read `0xD0` → must be **0x60** for BME280.

### 5.2 Soft reset
- Write **0xB6** to register `0xE0` to perform a complete power-on-reset procedure.
- Writing any other value has no effect (per datasheet).

---

## 6) Measurement time (for non-blocking scheduling)

The datasheet provides formulas for active measurement time as a function of oversampling.

Let:
- `T = osrs_t`, `P = osrs_p`, `H = osrs_h` where skipped = 0, x1=1, x2=2, x4=4, x8=8, x16=16.

**Typical active time (ms):**
- `t_meas_typ = 1 + (2*T if T!=0 else 0) + ((2*P + 0.5) if P!=0 else 0) + ((2*H + 0.5) if H!=0 else 0)`

**Maximum active time (ms):**
- `t_meas_max = 1.25 + (2.3*T if T!=0 else 0) + ((2.3*P + 0.575) if P!=0 else 0) + ((2.3*H + 0.575) if H!=0 else 0)`

Practical driver guidance:
- In **forced mode**, write mode=forced, then wait at least `t_meas_max` (deadline-based), then burst-read data registers.
- In **normal mode**, sampling period is approximately `t_meas + tstandby`.

---

## 7) Driver “gotchas” worth encoding as tests
- Changing `ctrl_hum` requires a subsequent write to `ctrl_meas`.
- `config` writes must be done in sleep.
- Always burst-read `0xF7..0xFE` to keep pressure/temp/humidity coherent.
- Verify chip_id before trusting calibration data.


---

## 8) Reserved / not used address ranges (from memory map)
- `0x88..0xA1`: calibration block 0 (T/P + H1)
- `0xA2..0xCF`: not used / reserved (leave untouched)
- `0xD0`: chip ID
- `0xD1..0xDF`: reserved
- `0xE0`: soft reset
- `0xE1..0xE7`: calibration block 1 (humidity)
- `0xE8..0xF1`: reserved
- `0xF2..0xF5`: control/status/config
- `0xF6`: reserved
- `0xF7..0xFE`: measurement data

Driver rule: never write to reserved registers. Reads are allowed but meaningless.
