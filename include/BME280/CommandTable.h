/// @file CommandTable.h
/// @brief Register addresses and bit definitions for BME280
#pragma once

#include <cstdint>

namespace BME280 {
namespace cmd {

// ============================================================================
// Chip Identification
// ============================================================================

static constexpr uint8_t REG_CHIP_ID = 0xD0;
static constexpr uint8_t CHIP_ID_BME280 = 0x60;

// ============================================================================
// Reset
// ============================================================================

static constexpr uint8_t REG_RESET = 0xE0;
static constexpr uint8_t RESET_VALUE = 0xB6;

// ============================================================================
// Status and Control Registers
// ============================================================================

static constexpr uint8_t REG_CTRL_HUM = 0xF2;   // Humidity oversampling
static constexpr uint8_t REG_STATUS = 0xF3;     // Measuring + NVM update flags
static constexpr uint8_t REG_CTRL_MEAS = 0xF4;  // Temp/press oversampling + mode
static constexpr uint8_t REG_CONFIG = 0xF5;     // Standby + IIR filter + SPI3W

// ============================================================================
// Measurement Data Registers (burst read 0xF7..0xFE)
// ============================================================================

static constexpr uint8_t REG_PRESS_MSB = 0xF7;
static constexpr uint8_t REG_PRESS_LSB = 0xF8;
static constexpr uint8_t REG_PRESS_XLSB = 0xF9;
static constexpr uint8_t REG_TEMP_MSB = 0xFA;
static constexpr uint8_t REG_TEMP_LSB = 0xFB;
static constexpr uint8_t REG_TEMP_XLSB = 0xFC;
static constexpr uint8_t REG_HUM_MSB = 0xFD;
static constexpr uint8_t REG_HUM_LSB = 0xFE;

static constexpr uint8_t REG_DATA_START = REG_PRESS_MSB;
static constexpr uint8_t DATA_LEN = 8;

// ============================================================================
// Calibration Registers
// ============================================================================

static constexpr uint8_t REG_CALIB_TP_START = 0x88;  // T1..T3, P1..P9 (26 bytes)
static constexpr uint8_t REG_CALIB_TP_LEN = 26;
static constexpr uint8_t REG_CALIB_H1 = 0xA1;        // H1 (1 byte)
static constexpr uint8_t REG_CALIB_H_START = 0xE1;   // H2..H6 (7 bytes)
static constexpr uint8_t REG_CALIB_H_LEN = 7;

// Temperature calibration
static constexpr uint8_t REG_DIG_T1_LSB = 0x88;
static constexpr uint8_t REG_DIG_T1_MSB = 0x89;
static constexpr uint8_t REG_DIG_T2_LSB = 0x8A;
static constexpr uint8_t REG_DIG_T2_MSB = 0x8B;
static constexpr uint8_t REG_DIG_T3_LSB = 0x8C;
static constexpr uint8_t REG_DIG_T3_MSB = 0x8D;

// Pressure calibration
static constexpr uint8_t REG_DIG_P1_LSB = 0x8E;
static constexpr uint8_t REG_DIG_P1_MSB = 0x8F;
static constexpr uint8_t REG_DIG_P2_LSB = 0x90;
static constexpr uint8_t REG_DIG_P2_MSB = 0x91;
static constexpr uint8_t REG_DIG_P3_LSB = 0x92;
static constexpr uint8_t REG_DIG_P3_MSB = 0x93;
static constexpr uint8_t REG_DIG_P4_LSB = 0x94;
static constexpr uint8_t REG_DIG_P4_MSB = 0x95;
static constexpr uint8_t REG_DIG_P5_LSB = 0x96;
static constexpr uint8_t REG_DIG_P5_MSB = 0x97;
static constexpr uint8_t REG_DIG_P6_LSB = 0x98;
static constexpr uint8_t REG_DIG_P6_MSB = 0x99;
static constexpr uint8_t REG_DIG_P7_LSB = 0x9A;
static constexpr uint8_t REG_DIG_P7_MSB = 0x9B;
static constexpr uint8_t REG_DIG_P8_LSB = 0x9C;
static constexpr uint8_t REG_DIG_P8_MSB = 0x9D;
static constexpr uint8_t REG_DIG_P9_LSB = 0x9E;
static constexpr uint8_t REG_DIG_P9_MSB = 0x9F;

// Humidity calibration
static constexpr uint8_t REG_DIG_H1 = 0xA1;
static constexpr uint8_t REG_DIG_H2_LSB = 0xE1;
static constexpr uint8_t REG_DIG_H2_MSB = 0xE2;
static constexpr uint8_t REG_DIG_H3 = 0xE3;
static constexpr uint8_t REG_DIG_H4_MSB = 0xE4;
static constexpr uint8_t REG_DIG_H4_H5 = 0xE5;  // H4 LSB (bits 3:0), H5 MSB (bits 7:4)
static constexpr uint8_t REG_DIG_H5_LSB = 0xE6;
static constexpr uint8_t REG_DIG_H6 = 0xE7;

// ============================================================================
// Bit Masks
// ============================================================================

static constexpr uint8_t MASK_STATUS_MEASURING = 0x08;
static constexpr uint8_t MASK_STATUS_IM_UPDATE = 0x01;

static constexpr uint8_t MASK_CTRL_HUM_OSRS_H = 0x07;

static constexpr uint8_t MASK_CTRL_MEAS_OSRS_T = 0xE0;
static constexpr uint8_t MASK_CTRL_MEAS_OSRS_P = 0x1C;
static constexpr uint8_t MASK_CTRL_MEAS_MODE = 0x03;

static constexpr uint8_t MASK_CONFIG_T_SB = 0xE0;
static constexpr uint8_t MASK_CONFIG_FILTER = 0x1C;
static constexpr uint8_t MASK_CONFIG_SPI3W_EN = 0x01;

// ============================================================================
// Bit Positions
// ============================================================================

static constexpr uint8_t BIT_STATUS_MEASURING = 3;
static constexpr uint8_t BIT_STATUS_IM_UPDATE = 0;

static constexpr uint8_t BIT_CTRL_HUM_OSRS_H = 0;

static constexpr uint8_t BIT_CTRL_MEAS_OSRS_T = 5;
static constexpr uint8_t BIT_CTRL_MEAS_OSRS_P = 2;
static constexpr uint8_t BIT_CTRL_MEAS_MODE = 0;

static constexpr uint8_t BIT_CONFIG_T_SB = 5;
static constexpr uint8_t BIT_CONFIG_FILTER = 2;
static constexpr uint8_t BIT_CONFIG_SPI3W_EN = 0;

} // namespace cmd
} // namespace BME280
