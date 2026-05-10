# Initialization, Reset, And Operational Notes

## Startup And Reset

- A power-on reset occurs after both `VDD` and `VDDIO` reach their minimum levels. There are no stated limitations on supply ramp order or slope. Source: datasheet, p. 14.
- After power-up, the sensor enters sleep mode. Source: datasheet, pp. 14-15.
- Soft reset is `write 0xB6` to register `0xE0`; the datasheet describes this as a complete power-on-reset procedure. Source: datasheet, p. 27.
- Cycling `VDD` resets the sensor; cycling `VDDIO` alone does not. Source: datasheet, p. 14.
- Do not drive interface pins high when `VDDIO` is off. Source: datasheet, p. 14.

## Typical Driver Sequence

1. Bring `CSB` high for I2C, or low for SPI, according to the intended interface. Source: datasheet, p. 32.
2. Wait for startup timing, then read `id` (`0xD0`) and expect `0x60`. Source: datasheet, pp. 8, 27.
3. Read calibration coefficients from `0x88..0xA1` and `0xE1..0xF0`. Source: datasheet, pp. 24, 27.
4. Program humidity oversampling in `ctrl_hum` (`0xF2`), then write `ctrl_meas` (`0xF4`) so humidity settings take effect. Source: datasheet, pp. 27-29.
5. Program `config` (`0xF5`) while in sleep mode if standby time, filter, or 3-wire SPI mode must change. Source: datasheet, p. 30.
6. Trigger forced mode for one-shot sampling or set normal mode for periodic sampling. Source: datasheet, pp. 15-16, 29.
7. Poll `status.measuring` or wait the computed measurement time before reading `0xF7..0xFE`. Source: datasheet, pp. 28, 51.

## Compensation And Timing Notes

- Raw outputs must be compensated using the device calibration values. Source: datasheet, pp. 23-25.
- Measurement time depends on enabled humidity, temperature, and pressure oversampling; the appendix provides formulas and examples. Source: datasheet, p. 51.
- In normal mode, the output data rate depends on measurement time plus standby time. Source: datasheet, p. 51.

## Handling Notes Relevant To Driver Testing

- The humidity sensor can show temporary offset after soldering or exposure outside operating range; reconditioning guidance is in the package/handling section. Source: datasheet, pp. 9, 45-46.
- During operation the sensor chip is sensitive to light; Bosch recommends avoiding exposure of the sensor chip. Source: datasheet, p. 48.
