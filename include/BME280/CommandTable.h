/// @file CommandTable.h
/// @brief Register addresses and bit definitions for BME280
#pragma once

#include <cstdint>

namespace BME280 {
namespace cmd {

/// @name Chip identification
/// @{
static constexpr uint8_t REG_CHIP_ID = 0xD0;     ///< Chip-ID register address.
static constexpr uint8_t CHIP_ID_BME280 = 0x60;  ///< Expected BME280 chip-ID value.
/// @}

/// @name Reset
/// @{
static constexpr uint8_t REG_RESET = 0xE0;    ///< Soft-reset register address.
static constexpr uint8_t RESET_VALUE = 0xB6;  ///< Soft-reset command value.
/// @}

/// @name Status and control registers
/// @{
static constexpr uint8_t REG_CTRL_HUM = 0xF2;   ///< Humidity oversampling register.
static constexpr uint8_t REG_STATUS = 0xF3;     ///< Measuring and NVM-copy status flags.
static constexpr uint8_t REG_CTRL_MEAS = 0xF4;  ///< Temperature/pressure oversampling and mode register.
static constexpr uint8_t REG_CONFIG = 0xF5;     ///< Standby, IIR filter, and SPI3W register.
/// @}

/// @name Measurement data registers
/// @{
static constexpr uint8_t REG_PRESS_MSB = 0xF7;  ///< Pressure ADC MSB.
static constexpr uint8_t REG_PRESS_LSB = 0xF8;  ///< Pressure ADC LSB.
static constexpr uint8_t REG_PRESS_XLSB = 0xF9; ///< Pressure ADC XLSB.
static constexpr uint8_t REG_TEMP_MSB = 0xFA;   ///< Temperature ADC MSB.
static constexpr uint8_t REG_TEMP_LSB = 0xFB;   ///< Temperature ADC LSB.
static constexpr uint8_t REG_TEMP_XLSB = 0xFC;  ///< Temperature ADC XLSB.
static constexpr uint8_t REG_HUM_MSB = 0xFD;    ///< Humidity ADC MSB.
static constexpr uint8_t REG_HUM_LSB = 0xFE;    ///< Humidity ADC LSB.

static constexpr uint8_t REG_DATA_START = REG_PRESS_MSB; ///< First register in the 0xF7..0xFE burst.
static constexpr uint8_t DATA_LEN = 8;                   ///< Length of the coherent data-register burst.
static constexpr int32_t RAW_PRESSURE_SKIPPED = 0x80000;    ///< Pressure reset/skipped output.
                                                            ///< Documentation only: the driver
                                                            ///< never compares samples against it.
static constexpr int32_t RAW_TEMPERATURE_SKIPPED = 0x80000; ///< Temperature reset/skipped output.
                                                            ///< Documentation only; a valid ADC
                                                            ///< result for an enabled channel.
static constexpr int32_t RAW_HUMIDITY_SKIPPED = 0x8000;     ///< Humidity reset/skipped output.
                                                            ///< Documentation only; validity comes
                                                            ///< from configured oversampling.
/// @}

/// @name Calibration registers
/// @{
static constexpr uint8_t REG_CALIB_TP_START = 0x88; ///< Start of T/P coefficient block plus H1.
static constexpr uint8_t REG_CALIB_TP_LEN = 26;     ///< Length of 0x88..0xA1 calibration block.
static constexpr uint8_t REG_CALIB_H1 = 0xA1;       ///< H1 calibration byte address.
static constexpr uint8_t REG_CALIB_H_START = 0xE1;  ///< Start of H2..H6 humidity calibration block.
static constexpr uint8_t REG_CALIB_H_LEN = 7;       ///< Length of H2..H6 humidity calibration block.

static constexpr uint8_t REG_DIG_T1_LSB = 0x88; ///< dig_T1 low byte.
static constexpr uint8_t REG_DIG_T1_MSB = 0x89; ///< dig_T1 high byte.
static constexpr uint8_t REG_DIG_T2_LSB = 0x8A; ///< dig_T2 low byte.
static constexpr uint8_t REG_DIG_T2_MSB = 0x8B; ///< dig_T2 high byte.
static constexpr uint8_t REG_DIG_T3_LSB = 0x8C; ///< dig_T3 low byte.
static constexpr uint8_t REG_DIG_T3_MSB = 0x8D; ///< dig_T3 high byte.

static constexpr uint8_t REG_DIG_P1_LSB = 0x8E; ///< dig_P1 low byte.
static constexpr uint8_t REG_DIG_P1_MSB = 0x8F; ///< dig_P1 high byte.
static constexpr uint8_t REG_DIG_P2_LSB = 0x90; ///< dig_P2 low byte.
static constexpr uint8_t REG_DIG_P2_MSB = 0x91; ///< dig_P2 high byte.
static constexpr uint8_t REG_DIG_P3_LSB = 0x92; ///< dig_P3 low byte.
static constexpr uint8_t REG_DIG_P3_MSB = 0x93; ///< dig_P3 high byte.
static constexpr uint8_t REG_DIG_P4_LSB = 0x94; ///< dig_P4 low byte.
static constexpr uint8_t REG_DIG_P4_MSB = 0x95; ///< dig_P4 high byte.
static constexpr uint8_t REG_DIG_P5_LSB = 0x96; ///< dig_P5 low byte.
static constexpr uint8_t REG_DIG_P5_MSB = 0x97; ///< dig_P5 high byte.
static constexpr uint8_t REG_DIG_P6_LSB = 0x98; ///< dig_P6 low byte.
static constexpr uint8_t REG_DIG_P6_MSB = 0x99; ///< dig_P6 high byte.
static constexpr uint8_t REG_DIG_P7_LSB = 0x9A; ///< dig_P7 low byte.
static constexpr uint8_t REG_DIG_P7_MSB = 0x9B; ///< dig_P7 high byte.
static constexpr uint8_t REG_DIG_P8_LSB = 0x9C; ///< dig_P8 low byte.
static constexpr uint8_t REG_DIG_P8_MSB = 0x9D; ///< dig_P8 high byte.
static constexpr uint8_t REG_DIG_P9_LSB = 0x9E; ///< dig_P9 low byte.
static constexpr uint8_t REG_DIG_P9_MSB = 0x9F; ///< dig_P9 high byte.

static constexpr uint8_t REG_DIG_H1 = 0xA1;     ///< dig_H1 byte.
static constexpr uint8_t REG_DIG_H2_LSB = 0xE1; ///< dig_H2 low byte.
static constexpr uint8_t REG_DIG_H2_MSB = 0xE2; ///< dig_H2 high byte.
static constexpr uint8_t REG_DIG_H3 = 0xE3;     ///< dig_H3 byte.
static constexpr uint8_t REG_DIG_H4_MSB = 0xE4; ///< dig_H4 high bits.
static constexpr uint8_t REG_DIG_H4_H5 = 0xE5;  ///< Packed dig_H4 low bits and dig_H5 low bits.
static constexpr uint8_t REG_DIG_H5_MSB = 0xE6; ///< dig_H5 high bits.
static constexpr uint8_t REG_DIG_H5_LSB = REG_DIG_H5_MSB; ///< Historical misnamed alias.
static constexpr uint8_t REG_DIG_H6 = 0xE7;     ///< dig_H6 byte.
/// @}

/// @name Bit masks
/// @{
static constexpr uint8_t MASK_STATUS_MEASURING = 0x08; ///< Status measuring bit mask.
static constexpr uint8_t MASK_STATUS_IM_UPDATE = 0x01; ///< Status NVM-copy bit mask.
static constexpr uint8_t MASK_CTRL_HUM_OSRS_H = 0x07;  ///< Humidity oversampling bit mask.
static constexpr uint8_t MASK_CTRL_MEAS_OSRS_T = 0xE0; ///< Temperature oversampling bit mask.
static constexpr uint8_t MASK_CTRL_MEAS_OSRS_P = 0x1C; ///< Pressure oversampling bit mask.
static constexpr uint8_t MASK_CTRL_MEAS_MODE = 0x03;   ///< Measurement mode bit mask.
static constexpr uint8_t MASK_CONFIG_T_SB = 0xE0;      ///< Standby time bit mask.
static constexpr uint8_t MASK_CONFIG_FILTER = 0x1C;    ///< IIR filter bit mask.
static constexpr uint8_t MASK_CONFIG_SPI3W_EN = 0x01;  ///< 3-wire SPI enable bit mask.
/// @}

/// @name Bit positions
/// @{
static constexpr uint8_t BIT_STATUS_MEASURING = 3; ///< Status measuring bit position.
static constexpr uint8_t BIT_STATUS_IM_UPDATE = 0; ///< Status NVM-copy bit position.
static constexpr uint8_t BIT_CTRL_HUM_OSRS_H = 0;  ///< Humidity oversampling bit position.
static constexpr uint8_t BIT_CTRL_MEAS_OSRS_T = 5; ///< Temperature oversampling bit position.
static constexpr uint8_t BIT_CTRL_MEAS_OSRS_P = 2; ///< Pressure oversampling bit position.
static constexpr uint8_t BIT_CTRL_MEAS_MODE = 0;   ///< Measurement mode bit position.
static constexpr uint8_t BIT_CONFIG_T_SB = 5;      ///< Standby time bit position.
static constexpr uint8_t BIT_CONFIG_FILTER = 2;    ///< IIR filter bit position.
static constexpr uint8_t BIT_CONFIG_SPI3W_EN = 0;  ///< 3-wire SPI enable bit position.
/// @}

} // namespace cmd
} // namespace BME280
