# Modes, Interrupts, Status, And Faults

## Power Modes

| Mode | Encoding | Behavior | Source |
|---|---:|---|---|
| Sleep | `00` | Default after power-on reset. No measurements; lowest power; all registers accessible. | Datasheet, pp. 14-15, 29 |
| Forced | `01` or `10` | Performs one measurement using current settings, stores results, then returns to sleep. Recommended for low sampling rate or host-synchronized sampling. | Datasheet, pp. 15-16, 29 |
| Normal | `11` | Repeats measurement periods and inactive standby periods. Recommended when using the IIR filter. | Datasheet, pp. 16, 29 |

## Status Bits

| Register/bit | Meaning | Source |
|---|---|---|
| `status.measuring` (`0xF3[3]`) | Set while conversion is running; cleared when results have transferred to data registers. | Datasheet, p. 28 |
| `status.im_update` (`0xF3[0]`) | Set while NVM data are copied to image registers; cleared when copying is done. Copying occurs after power-on reset and before every conversion. | Datasheet, p. 28 |

## Filtering And Output Behavior

- The IIR filter applies to pressure and temperature, not humidity. It reduces short-term pressure/temperature disturbances and increases pressure/temperature output resolution to 20 bit. Source: datasheet, pp. 17-18.
- Writing the filter setting resets filter memory; the next ADC values pass through unchanged and seed the filter. Source: datasheet, p. 18.
- If temperature or pressure measurement is skipped, the corresponding output register is set to `0x80000`; if humidity is skipped, output is `0x8000`. Source: datasheet, pp. 28-29.

## Self-Test Return Codes

The datasheet self-test appendix defines example API return codes, not hardware status-register bits. Useful driver-facing values include `0` sensor OK, `10` communication error or wrong device, `20` trimming data out of bound, `30` temperature bond wire/MEMS issue, `31` pressure bond wire/MEMS issue, and `40..42` implausible temperature/pressure/humidity. Source: datasheet, p. 54.
