# Electrical And Timing

## Electrical Limits

| Parameter | Value | Source |
|---|---:|---|
| `VDD` operating range | 1.71 V to 3.6 V, max ripple 50 mVpp | Datasheet, p. 8 |
| `VDDIO` operating range | 1.2 V to 3.6 V | Datasheet, p. 8 |
| Sleep current | typ. 0.1 uA, max 0.3 uA | Datasheet, p. 8 |
| Standby current in normal-mode inactive period | typ. 0.2 uA, max 0.5 uA | Datasheet, p. 8 |
| Startup time | 2 ms to first communication after `VDD > 1.58 V` and `VDDIO > 0.65 V` | Datasheet, p. 8 |
| Absolute max supply pin voltage | -0.3 V to 4.25 V | Datasheet, p. 13 |
| Absolute max interface pin voltage | -0.3 V to `VDDIO + 0.3 V` | Datasheet, p. 13 |
| Storage temperature | -45 degC to +85 degC at <=65 %RH | Datasheet, p. 13 |
| ESD | HBM +/-2 kV, CDM +/-500 V, machine model +/-200 V | Datasheet, p. 13 |

## Sensor Performance Anchors

| Channel | Key values | Source |
|---|---|---|
| Humidity | Accuracy tolerance +/-3 %RH at 20..80 %RH and 25 degC; response time 1 s to 63% step; resolution 0.008 %RH; noise 0.02 %RH RMS at highest oversampling. | Datasheet, p. 9 |
| Pressure | Full-accuracy range 300..1100 hPa over 0..65 degC; full accuracy +/-1.0 hPa; relative accuracy +/-0.12 hPa; noise 0.2 Pa with reduced bandwidth/highest oversampling. | Datasheet, pp. 10-11 |
| Temperature | Operating range -40..+85 degC; full accuracy 0..65 degC; accuracy +/-0.5 degC over 0..65 degC; API output resolution 0.01 degC. | Datasheet, pp. 11-12 |

## Digital Interface Timing

| Interface | Limit | Notes | Source |
|---|---:|---|---|
| I2C | Standard, Fast, and High Speed modes are supported. | Device does not perform clock stretching; SCL is high-Z input without drain capability. | Datasheet, pp. 32, 35-36 |
| SPI | 0 to 10 MHz | Applies to 3-wire and 4-wire SPI timing table. | Datasheet, p. 37 |
| I2C bus capacitance | 400 pF max on `SDI`/`SCK` | From interface parameter table. | Datasheet, p. 35 |
| Internal `CSB` pull-up | 70 kOhm min, 120 kOhm typ., 190 kOhm max | Pull-up to `VDDIO`. | Datasheet, p. 35 |

## Measurement Time And Rate

Active measurement time in ms is `1.25 + (2.3 * osrs_t) + (2.3 * osrs_p + 0.575) + (2.3 * osrs_h + 0.575)` for enabled temperature, pressure, and humidity paths. The datasheet example `osrs_t=x1`, `osrs_p=x4`, humidity skipped gives 11.5 ms. Forced-mode rate is host-triggered but saturates if triggers are faster than conversion time; normal-mode rate is `1000 / (measurement_time_ms + standby_time_ms)`. Source: datasheet, p. 51.

Current consumption examples from the cover/spec tables: typ. 1.8 uA at 1 Hz humidity+temperature forced mode, typ. 2.8 uA at 1 Hz pressure+temperature forced mode, and typ. 3.6 uA at 1 Hz humidity+pressure+temperature forced mode. Source: datasheet, pp. 2, 8-10.
