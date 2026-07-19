/// @file test_basic.cpp
/// @brief Native contract tests for BME280 lifecycle and health behavior.

#include <unity.h>
#include <type_traits>

#include "Arduino.h"
#include "Wire.h"

SerialClass Serial;
TwoWire Wire;

#include "BME280/BME280.h"
#include "common/I2cTransport.h"

using namespace BME280;

namespace {

struct FakeBus {
  uint8_t reg[256] = {};
  uint8_t deviceAddress = 0x76;
  uint8_t chipId = cmd::CHIP_ID_BME280;
  uint32_t nowMs = 1000;
  uint32_t writeCalls = 0;
  uint32_t readCalls = 0;
  uint32_t statusReadCalls = 0;
  uint32_t statusReadNowAdvanceMs = 0;
  uint32_t measuringStatusReadsRemaining = 0;
  uint32_t measuringOnStatusReadCall = 0;
  uint32_t imUpdateStatusReadsRemaining = 0;
  bool calibrationReadWhileImUpdate = false;

  int readErrorRemaining = 0;
  int writeErrorRemaining = 0;
  uint32_t failWriteOnCall = 0;
  Status readError = Status::Error(Err::I2C_ERROR, "forced read error", -1);
  Status writeError = Status::Error(Err::I2C_ERROR, "forced write error", -2);
  uint8_t lastWriteReg = 0;
  uint8_t lastWriteValue = 0;
  uint8_t writeRegLog[64] = {};
  uint8_t writeValueLog[64] = {};
  uint8_t writeLogLen = 0;
  bool failReadRegEnabled = false;
  uint8_t failReadReg = 0;
  uint32_t failReadRegRemaining = 0;
  uint8_t lastReadReg = 0;
  size_t lastReadLen = 0;
  uint32_t lastReadTimeoutMs = 0;
  uint32_t lastWriteTimeoutMs = 0;

  FakeBus() {
    loadDefaultCalibration();
  }

  void loadDefaultCalibration() {
    for (size_t i = 0; i < cmd::REG_CALIB_TP_LEN; ++i) {
      reg[static_cast<uint8_t>(cmd::REG_CALIB_TP_START + static_cast<uint8_t>(i))] =
          static_cast<uint8_t>(i + 1);
    }
    reg[cmd::REG_DIG_T1_LSB] = 0x88;
    reg[cmd::REG_DIG_T1_MSB] = 0x01;  // digT1 = 0x0188
    reg[cmd::REG_DIG_P1_LSB] = 0x34;
    reg[cmd::REG_DIG_P1_MSB] = 0x12;  // digP1 = 0x1234
    reg[cmd::REG_DIG_H1] = 0x01;
    reg[cmd::REG_DIG_H2_LSB] = 0x11;
    reg[cmd::REG_DIG_H2_MSB] = 0x22;
    reg[cmd::REG_DIG_H3] = 0x33;
    reg[cmd::REG_DIG_H4_MSB] = 0x44;
    reg[cmd::REG_DIG_H4_H5] = 0x55;
    reg[cmd::REG_DIG_H5_MSB] = 0x66;
    reg[cmd::REG_DIG_H6] = 0x77;
  }
};

Status fakeWrite(uint8_t addr, const uint8_t* data, size_t len, uint32_t timeoutMs,
                 void* user) {
  FakeBus* bus = static_cast<FakeBus*>(user);
  bus->writeCalls++;
  bus->lastWriteTimeoutMs = timeoutMs;
  if (addr != bus->deviceAddress) {
    return Status::Error(Err::I2C_NACK_ADDR, "fake address nack", addr);
  }
  if (data == nullptr || len == 0) {
    return Status::Error(Err::INVALID_PARAM, "invalid fake write args");
  }
  if (bus->failWriteOnCall != 0 && bus->writeCalls == bus->failWriteOnCall) {
    bus->failWriteOnCall = 0;
    return bus->writeError;
  }
  if (bus->writeErrorRemaining > 0) {
    bus->writeErrorRemaining--;
    return bus->writeError;
  }
  const uint8_t startReg = data[0];
  for (size_t i = 1; i < len; ++i) {
    const uint8_t reg = static_cast<uint8_t>(startReg + static_cast<uint8_t>(i - 1));
    bus->reg[reg] = data[i];
    bus->lastWriteReg = reg;
    bus->lastWriteValue = data[i];
    if (bus->writeLogLen < sizeof(bus->writeRegLog)) {
      bus->writeRegLog[bus->writeLogLen] = reg;
      bus->writeValueLog[bus->writeLogLen] = data[i];
      bus->writeLogLen++;
    }
  }
  return Status::Ok();
}

Status fakeWriteRead(uint8_t addr, const uint8_t* txData, size_t txLen, uint8_t* rxData,
                     size_t rxLen, uint32_t timeoutMs, void* user) {
  FakeBus* bus = static_cast<FakeBus*>(user);
  bus->readCalls++;
  bus->lastReadTimeoutMs = timeoutMs;
  if (addr != bus->deviceAddress) {
    return Status::Error(Err::I2C_NACK_ADDR, "fake address nack", addr);
  }
  if (txData == nullptr || txLen == 0 || (rxLen > 0 && rxData == nullptr)) {
    return Status::Error(Err::INVALID_PARAM, "invalid fake write-read args");
  }
  if (bus->readErrorRemaining > 0) {
    bus->readErrorRemaining--;
    return bus->readError;
  }

  const uint8_t reg = txData[0];
  bus->lastReadReg = reg;
  bus->lastReadLen = rxLen;
  if (bus->failReadRegEnabled && reg == bus->failReadReg) {
    bus->failReadRegEnabled = false;
    return bus->readError;
  }
  if (bus->failReadRegRemaining > 0 && reg == bus->failReadReg) {
    bus->failReadRegRemaining--;
    return bus->readError;
  }
  for (size_t i = 0; i < rxLen; ++i) {
    rxData[i] = 0;
  }

  if (reg == cmd::REG_CHIP_ID && rxLen >= 1) {
    rxData[0] = bus->chipId;
  } else if (reg == cmd::REG_CALIB_TP_START && rxLen == cmd::REG_CALIB_TP_LEN) {
    if (bus->imUpdateStatusReadsRemaining > 0 ||
        (bus->reg[cmd::REG_STATUS] & cmd::MASK_STATUS_IM_UPDATE) != 0) {
      bus->calibrationReadWhileImUpdate = true;
    }
    for (size_t i = 0; i < rxLen; ++i) {
      rxData[i] = bus->reg[static_cast<uint8_t>(reg + static_cast<uint8_t>(i))];
    }
  } else if (reg == cmd::REG_CALIB_H1 && rxLen >= 1) {
    rxData[0] = bus->reg[cmd::REG_CALIB_H1];
  } else if (reg == cmd::REG_CALIB_H_START && rxLen == cmd::REG_CALIB_H_LEN) {
    for (size_t i = 0; i < rxLen; ++i) {
      rxData[i] = bus->reg[static_cast<uint8_t>(reg + static_cast<uint8_t>(i))];
    }
  } else if (reg == cmd::REG_STATUS && rxLen >= 1) {
    bus->statusReadCalls++;
    uint8_t status = bus->reg[cmd::REG_STATUS];
    if (bus->measuringStatusReadsRemaining > 0) {
      status |= cmd::MASK_STATUS_MEASURING;
      bus->measuringStatusReadsRemaining--;
    }
    if (bus->measuringOnStatusReadCall != 0 &&
        bus->statusReadCalls == bus->measuringOnStatusReadCall) {
      status |= cmd::MASK_STATUS_MEASURING;
      bus->measuringOnStatusReadCall = 0;
    }
    if (bus->imUpdateStatusReadsRemaining > 0) {
      status |= cmd::MASK_STATUS_IM_UPDATE;
      bus->imUpdateStatusReadsRemaining--;
    }
    bus->nowMs += bus->statusReadNowAdvanceMs;
    rxData[0] = status;
  } else {
    for (size_t i = 0; i < rxLen; ++i) {
      rxData[i] = bus->reg[static_cast<uint8_t>(reg + static_cast<uint8_t>(i))];
    }
  }

  return Status::Ok();
}

uint32_t fakeNowMs(void* user) {
  return static_cast<FakeBus*>(user)->nowMs;
}

Config makeConfig(FakeBus& bus) {
  Config cfg;
  cfg.i2cWrite = fakeWrite;
  cfg.i2cWriteRead = fakeWriteRead;
  cfg.i2cUser = &bus;
  cfg.nowMs = fakeNowMs;
  cfg.timeUser = &bus;
  cfg.i2cTimeoutMs = 10;
  cfg.offlineThreshold = 3;
  cfg.mode = Mode::FORCED;
  return cfg;
}

uint32_t totalBusCalls(const FakeBus& bus) {
  return bus.readCalls + bus.writeCalls;
}

JobPollResult pollWithBudget(BME280::BME280& dev, FakeBus& bus,
                             uint8_t maxInstructions) {
  const uint32_t callsBefore = totalBusCalls(bus);
  JobPollResult result = dev.pollJob(bus.nowMs, maxInstructions);
  const uint32_t callsAfter = totalBusCalls(bus);
  TEST_ASSERT_EQUAL_UINT32(static_cast<uint32_t>(result.instructionsUsed),
                           callsAfter - callsBefore);
  TEST_ASSERT_TRUE(result.instructionsUsed <= maxInstructions);
  return result;
}

JobPollResult pollUntilTerminal(BME280::BME280& dev, FakeBus& bus,
                                uint8_t maxInstructions,
                                uint16_t maxPolls = 64) {
  JobPollResult result{};
  for (uint16_t i = 0; i < maxPolls; ++i) {
    result = pollWithBudget(dev, bus, maxInstructions);
    if (result.state == JobState::DONE || result.state == JobState::FAILED) {
      return result;
    }
  }
  TEST_FAIL_MESSAGE("job did not reach terminal state");
  return result;
}

Config makeConfigNoNowMs(FakeBus& bus) {
  Config cfg = makeConfig(bus);
  cfg.nowMs = nullptr;
  cfg.timeUser = nullptr;
  return cfg;
}

void putLe16(FakeBus& bus, uint8_t reg, uint16_t value) {
  bus.reg[reg] = static_cast<uint8_t>(value & 0xFF);
  bus.reg[static_cast<uint8_t>(reg + 1)] = static_cast<uint8_t>(value >> 8);
}

void putS16(FakeBus& bus, uint8_t reg, int16_t value) {
  putLe16(bus, reg, static_cast<uint16_t>(value));
}

void setH4H5(FakeBus& bus, int16_t h4, int16_t h5) {
  const uint16_t h4Raw = static_cast<uint16_t>(h4) & 0x0FFF;
  const uint16_t h5Raw = static_cast<uint16_t>(h5) & 0x0FFF;
  bus.reg[cmd::REG_DIG_H4_MSB] = static_cast<uint8_t>(h4Raw >> 4);
  bus.reg[cmd::REG_DIG_H4_H5] = static_cast<uint8_t>(((h5Raw & 0x000F) << 4) |
                                                     (h4Raw & 0x000F));
  bus.reg[cmd::REG_DIG_H5_MSB] = static_cast<uint8_t>(h5Raw >> 4);
}

void setBoschSyntheticCalibration(FakeBus& bus) {
  putLe16(bus, cmd::REG_DIG_T1_LSB, 27504);
  putS16(bus, cmd::REG_DIG_T2_LSB, 26435);
  putS16(bus, cmd::REG_DIG_T3_LSB, -1000);

  putLe16(bus, cmd::REG_DIG_P1_LSB, 36477);
  putS16(bus, cmd::REG_DIG_P2_LSB, -10685);
  putS16(bus, cmd::REG_DIG_P3_LSB, 3024);
  putS16(bus, cmd::REG_DIG_P4_LSB, 2855);
  putS16(bus, cmd::REG_DIG_P5_LSB, 140);
  putS16(bus, cmd::REG_DIG_P6_LSB, -7);
  putS16(bus, cmd::REG_DIG_P7_LSB, 15500);
  putS16(bus, cmd::REG_DIG_P8_LSB, -14600);
  putS16(bus, cmd::REG_DIG_P9_LSB, 6000);

  bus.reg[cmd::REG_DIG_H1] = 75;
  putS16(bus, cmd::REG_DIG_H2_LSB, 362);
  bus.reg[cmd::REG_DIG_H3] = 0;
  setH4H5(bus, 325, 50);
  bus.reg[cmd::REG_DIG_H6] = static_cast<uint8_t>(30);
}

void setPressureDenominatorZeroCalibration(FakeBus& bus) {
  putLe16(bus, cmd::REG_DIG_T1_LSB, 32327);
  putS16(bus, cmd::REG_DIG_T2_LSB, 28683);
  putS16(bus, cmd::REG_DIG_T3_LSB, -12560);
  putLe16(bus, cmd::REG_DIG_P1_LSB, 1);
  putS16(bus, cmd::REG_DIG_P2_LSB, 31186);
  putS16(bus, cmd::REG_DIG_P3_LSB, -22662);
  putS16(bus, cmd::REG_DIG_P4_LSB, 0);
  putS16(bus, cmd::REG_DIG_P5_LSB, 0);
  putS16(bus, cmd::REG_DIG_P6_LSB, 0);
  putS16(bus, cmd::REG_DIG_P7_LSB, 0);
  putS16(bus, cmd::REG_DIG_P8_LSB, 0);
  putS16(bus, cmd::REG_DIG_P9_LSB, 0);
}

void setRawBurstBytes(FakeBus& bus,
                      uint8_t pMsb,
                      uint8_t pLsb,
                      uint8_t pXlsb,
                      uint8_t tMsb,
                      uint8_t tLsb,
                      uint8_t tXlsb,
                      uint8_t hMsb,
                      uint8_t hLsb) {
  bus.reg[cmd::REG_PRESS_MSB] = pMsb;
  bus.reg[cmd::REG_PRESS_LSB] = pLsb;
  bus.reg[cmd::REG_PRESS_XLSB] = pXlsb;
  bus.reg[cmd::REG_TEMP_MSB] = tMsb;
  bus.reg[cmd::REG_TEMP_LSB] = tLsb;
  bus.reg[cmd::REG_TEMP_XLSB] = tXlsb;
  bus.reg[cmd::REG_HUM_MSB] = hMsb;
  bus.reg[cmd::REG_HUM_LSB] = hLsb;
}

void setRawSample(FakeBus& bus, int32_t adcP, int32_t adcT, int32_t adcH) {
  setRawBurstBytes(bus,
                   static_cast<uint8_t>((adcP >> 12) & 0xFF),
                   static_cast<uint8_t>((adcP >> 4) & 0xFF),
                   static_cast<uint8_t>((adcP & 0x0F) << 4),
                   static_cast<uint8_t>((adcT >> 12) & 0xFF),
                   static_cast<uint8_t>((adcT >> 4) & 0xFF),
                   static_cast<uint8_t>((adcT & 0x0F) << 4),
                   static_cast<uint8_t>((adcH >> 8) & 0xFF),
                   static_cast<uint8_t>(adcH & 0xFF));
}

void captureForcedSample(BME280::BME280& dev, FakeBus& bus) {
  Status st = dev.requestMeasurement();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::IN_PROGRESS),
                          static_cast<uint8_t>(st.code));
  bus.nowMs += dev.estimateMeasurementTimeMs();
  dev.tick(bus.nowMs);
}

void assertRawSampleEqual(const RawSample& expected, const RawSample& actual) {
  TEST_ASSERT_EQUAL_INT32(expected.adcT, actual.adcT);
  TEST_ASSERT_EQUAL_INT32(expected.adcP, actual.adcP);
  TEST_ASSERT_EQUAL_INT32(expected.adcH, actual.adcH);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(expected.temperatureValid),
                          static_cast<uint8_t>(actual.temperatureValid));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(expected.pressureValid),
                          static_cast<uint8_t>(actual.pressureValid));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(expected.humidityValid),
                          static_cast<uint8_t>(actual.humidityValid));
}

void assertCompensatedSampleEqual(const CompensatedSample& expected,
                                  const CompensatedSample& actual) {
  TEST_ASSERT_EQUAL_INT32(expected.tempC_x100, actual.tempC_x100);
  TEST_ASSERT_EQUAL_UINT32(expected.pressurePa, actual.pressurePa);
  TEST_ASSERT_EQUAL_UINT32(expected.humidityPct_x1024,
                           actual.humidityPct_x1024);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(expected.temperatureValid),
                          static_cast<uint8_t>(actual.temperatureValid));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(expected.pressureValid),
                          static_cast<uint8_t>(actual.pressureValid));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(expected.humidityValid),
                          static_cast<uint8_t>(actual.humidityValid));
}

void assertSampleEnvelopeEqual(const SampleEnvelope& expected,
                               const SampleEnvelope& actual) {
  assertRawSampleEqual(expected.rawSample, actual.rawSample);
  assertCompensatedSampleEqual(expected.compensatedSample,
                               actual.compensatedSample);
  TEST_ASSERT_EQUAL_INT32(expected.tFine, actual.tFine);
  TEST_ASSERT_EQUAL_UINT32(expected.timestampMs, actual.timestampMs);
  TEST_ASSERT_EQUAL_UINT32(expected.sampleSequence, actual.sampleSequence);
  TEST_ASSERT_EQUAL_UINT32(expected.configGeneration, actual.configGeneration);
}

void assertCalibrationEqual(const Calibration& expected,
                            const Calibration& actual) {
  TEST_ASSERT_EQUAL_UINT16(expected.digT1, actual.digT1);
  TEST_ASSERT_EQUAL_INT16(expected.digT2, actual.digT2);
  TEST_ASSERT_EQUAL_INT16(expected.digT3, actual.digT3);
  TEST_ASSERT_EQUAL_UINT16(expected.digP1, actual.digP1);
  TEST_ASSERT_EQUAL_INT16(expected.digP2, actual.digP2);
  TEST_ASSERT_EQUAL_INT16(expected.digP3, actual.digP3);
  TEST_ASSERT_EQUAL_INT16(expected.digP4, actual.digP4);
  TEST_ASSERT_EQUAL_INT16(expected.digP5, actual.digP5);
  TEST_ASSERT_EQUAL_INT16(expected.digP6, actual.digP6);
  TEST_ASSERT_EQUAL_INT16(expected.digP7, actual.digP7);
  TEST_ASSERT_EQUAL_INT16(expected.digP8, actual.digP8);
  TEST_ASSERT_EQUAL_INT16(expected.digP9, actual.digP9);
  TEST_ASSERT_EQUAL_UINT8(expected.digH1, actual.digH1);
  TEST_ASSERT_EQUAL_INT16(expected.digH2, actual.digH2);
  TEST_ASSERT_EQUAL_UINT8(expected.digH3, actual.digH3);
  TEST_ASSERT_EQUAL_INT16(expected.digH4, actual.digH4);
  TEST_ASSERT_EQUAL_INT16(expected.digH5, actual.digH5);
  TEST_ASSERT_EQUAL_INT8(expected.digH6, actual.digH6);
}

}  // namespace

void setUp() {
  setMillis(0);
  Wire._clearEndTransmissionResult();
  Wire._clearRequestFromOverride();
}

void tearDown() {}

void test_status_ok() {
  Status st = Status::Ok();
  TEST_ASSERT_TRUE(st.ok());
  TEST_ASSERT_TRUE(static_cast<bool>(st));
  TEST_ASSERT_TRUE(st.is(Err::OK));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::OK), static_cast<uint8_t>(st.code));
}

void test_status_error() {
  Status st = Status::Error(Err::I2C_ERROR, "Test error", 42);
  TEST_ASSERT_FALSE(st.ok());
  TEST_ASSERT_FALSE(static_cast<bool>(st));
  TEST_ASSERT_TRUE(st.is(Err::I2C_ERROR));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_ERROR), static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_INT32(42, st.detail);
}

void test_status_in_progress() {
  Status st{Err::IN_PROGRESS, 0, "In progress"};
  TEST_ASSERT_FALSE(st.ok());
  TEST_ASSERT_FALSE(static_cast<bool>(st));
  TEST_ASSERT_TRUE(st.is(Err::IN_PROGRESS));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::IN_PROGRESS), static_cast<uint8_t>(st.code));
  TEST_ASSERT_TRUE(st.inProgress());
}

void test_config_defaults() {
  Config cfg;
  TEST_ASSERT_NULL(cfg.i2cWrite);
  TEST_ASSERT_NULL(cfg.i2cWriteRead);
  TEST_ASSERT_EQUAL_HEX8(0x76, cfg.i2cAddress);
  TEST_ASSERT_EQUAL_UINT16(50, cfg.i2cTimeoutMs);
  TEST_ASSERT_EQUAL_UINT32(10u, cfg.nvmReadyTimeoutMs);
  TEST_ASSERT_EQUAL_UINT8(5, cfg.offlineThreshold);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Oversampling::X1), static_cast<uint8_t>(cfg.osrsT));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Oversampling::X1), static_cast<uint8_t>(cfg.osrsP));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Oversampling::X1), static_cast<uint8_t>(cfg.osrsH));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Filter::OFF), static_cast<uint8_t>(cfg.filter));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Standby::MS_125), static_cast<uint8_t>(cfg.standby));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Mode::FORCED), static_cast<uint8_t>(cfg.mode));
}

void test_driver_is_not_copyable_or_movable() {
  static_assert(!std::is_copy_constructible<BME280::BME280>::value,
                "BME280 must not be copy constructible");
  static_assert(!std::is_copy_assignable<BME280::BME280>::value,
                "BME280 must not be copy assignable");
  static_assert(!std::is_move_constructible<BME280::BME280>::value,
                "BME280 must not be move constructible");
  static_assert(!std::is_move_assignable<BME280::BME280>::value,
                "BME280 must not be move assignable");
  TEST_ASSERT_TRUE(true);
}

void test_get_settings_snapshot() {
  FakeBus bus;
  bus.deviceAddress = 0x77;
  BME280::BME280 dev;
  Config cfg = makeConfig(bus);
  cfg.i2cAddress = 0x77;
  cfg.osrsT = Oversampling::X8;
  cfg.osrsP = Oversampling::X4;
  cfg.osrsH = Oversampling::X2;
  cfg.filter = Filter::X8;
  cfg.standby = Standby::MS_500;
  cfg.mode = Mode::NORMAL;
  TEST_ASSERT_TRUE(dev.begin(cfg).ok());

  const uint32_t readsBefore = bus.readCalls;
  const uint32_t writesBefore = bus.writeCalls;
  SettingsSnapshot snap;
  Status st = dev.getSettings(snap);
  TEST_ASSERT_TRUE(st.ok());
  TEST_ASSERT_EQUAL_UINT32(readsBefore, bus.readCalls);
  TEST_ASSERT_EQUAL_UINT32(writesBefore, bus.writeCalls);
  TEST_ASSERT_TRUE(snap.initialized);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::READY),
                          static_cast<uint8_t>(snap.state));
  TEST_ASSERT_EQUAL_HEX8(0x77, snap.i2cAddress);
  TEST_ASSERT_EQUAL_UINT32(10u, snap.i2cTimeoutMs);
  TEST_ASSERT_EQUAL_UINT32(10u, snap.nvmReadyTimeoutMs);
  TEST_ASSERT_EQUAL_UINT8(3u, snap.offlineThreshold);
  TEST_ASSERT_TRUE(snap.hasNowMsHook);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Mode::NORMAL), static_cast<uint8_t>(snap.mode));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Oversampling::X8),
                          static_cast<uint8_t>(snap.osrsT));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Oversampling::X4),
                          static_cast<uint8_t>(snap.osrsP));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Oversampling::X2),
                          static_cast<uint8_t>(snap.osrsH));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Filter::X8),
                          static_cast<uint8_t>(snap.filter));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Standby::MS_500),
                          static_cast<uint8_t>(snap.standby));
  TEST_ASSERT_FALSE(snap.measurementRequested);
  TEST_ASSERT_FALSE(snap.measurementReady);
  TEST_ASSERT_TRUE(snap.lastMeasurementStatus.ok());
  TEST_ASSERT_FALSE(snap.hasSample);
  TEST_ASSERT_FALSE(snap.hardwareConfigDirty);
  TEST_ASSERT_TRUE(snap.hardwareConfigDirtyError.ok());
  TEST_ASSERT_EQUAL_UINT32(0u, snap.measurementStartMs);
  TEST_ASSERT_EQUAL_UINT32(0u, snap.sampleTimestampMs);
  TEST_ASSERT_EQUAL_INT32(0, snap.tFine);
  TEST_ASSERT_EQUAL_INT32(0, snap.rawSample.adcT);
  TEST_ASSERT_EQUAL_UINT32(0u, snap.compSample.pressurePa);
}

void test_begin_rejects_missing_callbacks() {
  BME280::BME280 dev;
  Config cfg;
  Status st = dev.begin(cfg);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_CONFIG), static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::UNINIT),
                          static_cast<uint8_t>(dev.state()));
}

void test_begin_rejects_zero_nvm_timeout() {
  FakeBus bus;
  BME280::BME280 dev;
  Config cfg = makeConfig(bus);
  cfg.nvmReadyTimeoutMs = 0;

  Status st = dev.begin(cfg);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_CONFIG),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_UINT32(0u, totalBusCalls(bus));
  TEST_ASSERT_FALSE(dev.isInitialized());
}

void test_begin_rejects_oversized_nvm_timeout_without_i2c() {
  FakeBus bus;
  BME280::BME280 dev;
  Config cfg = makeConfig(bus);
  cfg.nvmReadyTimeoutMs = 0x80000000UL;

  Status st = dev.begin(cfg);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_CONFIG),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_UINT32(0u, totalBusCalls(bus));
  TEST_ASSERT_FALSE(dev.isInitialized());
}

void test_begin_rejects_invalid_oversampling_combination() {
  FakeBus bus;
  BME280::BME280 dev;
  Config cfg = makeConfig(bus);
  cfg.osrsT = Oversampling::SKIP;
  cfg.osrsP = Oversampling::X1;
  cfg.osrsH = Oversampling::SKIP;

  Status st = dev.begin(cfg);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_CONFIG),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_UINT32(0u, bus.readCalls);
  TEST_ASSERT_EQUAL_UINT32(0u, bus.writeCalls);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::UNINIT),
                          static_cast<uint8_t>(dev.state()));
}

void test_high_bit_enum_values_are_rejected_without_i2c() {
  FakeBus bus;
  BME280::BME280 dev;
  Config cfg = makeConfig(bus);
  cfg.osrsT = static_cast<Oversampling>(8);

  Status st = dev.begin(cfg);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_CONFIG),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_UINT32(0u, totalBusCalls(bus));

  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());
  const uint32_t callsBefore = totalBusCalls(bus);
  st = dev.setOversamplingT(static_cast<Oversampling>(8));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_PARAM),
                          static_cast<uint8_t>(st.code));
  st = dev.setFilter(static_cast<Filter>(8));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_PARAM),
                          static_cast<uint8_t>(st.code));
  st = dev.setStandby(static_cast<Standby>(8));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_PARAM),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_UINT32(callsBefore, totalBusCalls(bus));
}

void test_invalid_begin_after_success_resets_default_runtime() {
  FakeBus bus;
  bus.deviceAddress = 0x77;
  BME280::BME280 dev;
  Config cfg = makeConfig(bus);
  cfg.i2cAddress = 0x77;
  cfg.offlineThreshold = 1;
  cfg.mode = Mode::NORMAL;
  TEST_ASSERT_TRUE(dev.begin(cfg).ok());

  const uint32_t readsBefore = bus.readCalls;
  const uint32_t writesBefore = bus.writeCalls;
  Config bad = makeConfig(bus);
  bad.i2cAddress = 0x75;
  Status st = dev.begin(bad);

  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_CONFIG),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_FALSE(dev.isInitialized());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::UNINIT),
                          static_cast<uint8_t>(dev.state()));
  TEST_ASSERT_EQUAL_UINT32(0u, dev.totalSuccess());
  TEST_ASSERT_EQUAL_UINT32(0u, dev.totalFailures());
  TEST_ASSERT_EQUAL_UINT32(0u, dev.lastOkMs());
  TEST_ASSERT_EQUAL_UINT32(0u, dev.lastErrorMs());
  TEST_ASSERT_EQUAL_UINT32(readsBefore, bus.readCalls);
  TEST_ASSERT_EQUAL_UINT32(writesBefore, bus.writeCalls);

  const Config& stored = dev.getConfig();
  TEST_ASSERT_NULL(stored.i2cWrite);
  TEST_ASSERT_NULL(stored.i2cWriteRead);
  TEST_ASSERT_EQUAL_HEX8(0x76, stored.i2cAddress);
  TEST_ASSERT_EQUAL_UINT32(50u, stored.i2cTimeoutMs);
  TEST_ASSERT_EQUAL_UINT32(10u, stored.nvmReadyTimeoutMs);
  TEST_ASSERT_EQUAL_UINT8(5u, stored.offlineThreshold);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Mode::FORCED),
                          static_cast<uint8_t>(stored.mode));

  SettingsSnapshot snap;
  TEST_ASSERT_TRUE(dev.getSettings(snap).ok());
  TEST_ASSERT_FALSE(snap.initialized);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::UNINIT),
                          static_cast<uint8_t>(snap.state));
  TEST_ASSERT_EQUAL_UINT8(5u, snap.offlineThreshold);
  TEST_ASSERT_FALSE(snap.hasNowMsHook);
  TEST_ASSERT_FALSE(snap.hasSample);
  TEST_ASSERT_EQUAL_UINT16(0u, snap.calibration.digT1);
}

void test_begin_normalizes_offline_threshold_on_stored_copy() {
  FakeBus bus;
  BME280::BME280 dev;
  Config cfg = makeConfig(bus);
  cfg.offlineThreshold = 0;

  Status st = dev.begin(cfg);
  TEST_ASSERT_TRUE(st.ok());
  TEST_ASSERT_EQUAL_UINT8(0u, cfg.offlineThreshold);
  TEST_ASSERT_EQUAL_UINT8(1u, dev.getConfig().offlineThreshold);

  SettingsSnapshot snap;
  TEST_ASSERT_TRUE(dev.getSettings(snap).ok());
  TEST_ASSERT_EQUAL_UINT8(1u, snap.offlineThreshold);
}

void test_begin_success_sets_ready_without_health_counts() {
  FakeBus bus;
  BME280::BME280 dev;
  Status st = dev.begin(makeConfig(bus));
  TEST_ASSERT_TRUE(st.ok());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::READY),
                          static_cast<uint8_t>(dev.state()));
  TEST_ASSERT_TRUE(dev.isOnline());
  TEST_ASSERT_GREATER_THAN_UINT32(0u, bus.readCalls + bus.writeCalls);
  TEST_ASSERT_EQUAL_UINT32(0u, dev.totalSuccess());
  TEST_ASSERT_EQUAL_UINT32(0u, dev.totalFailures());
  TEST_ASSERT_EQUAL_UINT8(0u, dev.consecutiveFailures());
  TEST_ASSERT_EQUAL_UINT32(0u, dev.lastOkMs());
}

void test_begin_passes_configured_i2c_timeout_to_transport() {
  FakeBus bus;
  BME280::BME280 dev;
  Config cfg = makeConfig(bus);
  cfg.i2cTimeoutMs = 20;

  Status st = dev.begin(cfg);
  TEST_ASSERT_TRUE(st.ok());
  TEST_ASSERT_EQUAL_UINT32(20u, bus.lastReadTimeoutMs);
  TEST_ASSERT_EQUAL_UINT32(20u, bus.lastWriteTimeoutMs);
}

void test_begin_starts_new_health_session_and_resets_counters() {
  FakeBus bus;
  BME280::BME280 dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  bus.nowMs = 1234;
  bus.readErrorRemaining = 1;
  bus.readError = Status::Error(Err::I2C_TIMEOUT, "tracked status timeout", -70);
  uint8_t status = 0;
  Status st = dev.readStatus(status);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_TIMEOUT),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_UINT32(1u, dev.totalFailures());
  TEST_ASSERT_EQUAL_UINT8(1u, dev.consecutiveFailures());
  TEST_ASSERT_EQUAL_UINT32(1234u, dev.lastErrorMs());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::DEGRADED),
                          static_cast<uint8_t>(dev.state()));

  bus.nowMs = 1300;
  st = dev.begin(makeConfig(bus));
  TEST_ASSERT_TRUE(st.ok());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::READY),
                          static_cast<uint8_t>(dev.state()));
  TEST_ASSERT_EQUAL_UINT32(0u, dev.totalFailures());
  TEST_ASSERT_EQUAL_UINT32(0u, dev.totalSuccess());
  TEST_ASSERT_EQUAL_UINT8(0u, dev.consecutiveFailures());
  TEST_ASSERT_EQUAL_UINT32(0u, dev.lastErrorMs());
  TEST_ASSERT_EQUAL_UINT32(0u, dev.lastOkMs());
  TEST_ASSERT_TRUE(dev.lastError().ok());
}

void test_begin_returns_busy_when_nvm_update_in_progress_without_calibration_read() {
  FakeBus bus;
  bus.imUpdateStatusReadsRemaining = 2;
  BME280::BME280 dev;

  Status st = dev.begin(makeConfig(bus));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::BUSY),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_UINT32(1u, bus.statusReadCalls);
  TEST_ASSERT_FALSE(bus.calibrationReadWhileImUpdate);
  TEST_ASSERT_FALSE(dev.isInitialized());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::UNINIT),
                          static_cast<uint8_t>(dev.state()));
}

void test_begin_defers_apply_config_when_device_measuring_without_write() {
  FakeBus bus;
  bus.reg[cmd::REG_STATUS] = cmd::MASK_STATUS_MEASURING;
  BME280::BME280 dev;

  Status st = dev.begin(makeConfig(bus));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::BUSY),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_FALSE(dev.isInitialized());
  TEST_ASSERT_EQUAL_UINT32(0u, bus.writeCalls);
  TEST_ASSERT_FALSE(dev.hardwareConfigDirty());
}

void test_begin_nvm_timeout_uses_wrap_safe_deadline_without_tight_poll_loop() {
  FakeBus bus;
  bus.reg[cmd::REG_STATUS] = cmd::MASK_STATUS_IM_UPDATE;
  bus.statusReadNowAdvanceMs = 2;
  bus.nowMs = 0xFFFFFFFEu;
  BME280::BME280 dev;
  Config cfg = makeConfig(bus);
  cfg.nvmReadyTimeoutMs = 1;

  Status st = dev.begin(cfg);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::TIMEOUT),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_INT32(1, st.detail);
  TEST_ASSERT_FALSE(dev.isInitialized());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::UNINIT),
                          static_cast<uint8_t>(dev.state()));
  TEST_ASSERT_FALSE(bus.calibrationReadWhileImUpdate);
  TEST_ASSERT_EQUAL_UINT32(1u, bus.statusReadCalls);
}

void test_begin_rejects_invalid_temperature_calibration() {
  FakeBus bus;
  putLe16(bus, cmd::REG_DIG_T1_LSB, 0);
  BME280::BME280 dev;

  Status st = dev.begin(makeConfig(bus));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::CALIBRATION_INVALID),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_FALSE(dev.isInitialized());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::UNINIT),
                          static_cast<uint8_t>(dev.state()));
}

void test_begin_rejects_invalid_pressure_calibration() {
  FakeBus bus;
  putLe16(bus, cmd::REG_DIG_P1_LSB, 0);
  BME280::BME280 dev;

  Status st = dev.begin(makeConfig(bus));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::CALIBRATION_INVALID),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_FALSE(dev.isInitialized());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::UNINIT),
                          static_cast<uint8_t>(dev.state()));
}

void test_begin_forced_mode_keeps_hardware_sleep_until_requested() {
  FakeBus bus;
  BME280::BME280 dev;
  Config cfg = makeConfig(bus);
  cfg.mode = Mode::FORCED;
  TEST_ASSERT_TRUE(dev.begin(cfg).ok());

  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Mode::SLEEP),
                          bus.reg[cmd::REG_CTRL_MEAS] & cmd::MASK_CTRL_MEAS_MODE);

  Status st = dev.requestMeasurement();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::IN_PROGRESS),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Mode::FORCED),
                          bus.reg[cmd::REG_CTRL_MEAS] & cmd::MASK_CTRL_MEAS_MODE);
}

void test_init_job_budget_one_instruction_per_poll() {
  FakeBus bus;
  BME280::BME280 dev;

  Status st = dev.startInitJob(makeConfig(bus));
  TEST_ASSERT_TRUE(st.inProgress());

  uint32_t instructionsTotal = 0;
  JobPollResult result{};
  for (uint8_t poll = 0; poll < 16; ++poll) {
    result = pollWithBudget(dev, bus, 1);
    instructionsTotal += result.instructionsUsed;
    if (result.state == JobState::DONE || result.state == JobState::FAILED) {
      break;
    }
  }

  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(JobState::DONE),
                          static_cast<uint8_t>(result.state));
  TEST_ASSERT_TRUE(result.status.ok());
  TEST_ASSERT_TRUE(dev.isInitialized());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::READY),
                          static_cast<uint8_t>(dev.state()));
  TEST_ASSERT_EQUAL_UINT32(9u, instructionsTotal);
  TEST_ASSERT_EQUAL_UINT32(9u, totalBusCalls(bus));
  TEST_ASSERT_EQUAL_UINT32(2u, bus.statusReadCalls);
  TEST_ASSERT_FALSE(bus.calibrationReadWhileImUpdate);
}

void test_init_job_nvm_busy_reads_status_one_poll_at_a_time() {
  FakeBus bus;
  bus.imUpdateStatusReadsRemaining = 2;
  BME280::BME280 dev;

  Status st = dev.startInitJob(makeConfig(bus));
  TEST_ASSERT_TRUE(st.inProgress());

  JobPollResult result = pollUntilTerminal(dev, bus, 1);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(JobState::DONE),
                          static_cast<uint8_t>(result.state));
  TEST_ASSERT_TRUE(result.status.ok());
  TEST_ASSERT_EQUAL_UINT32(4u, bus.statusReadCalls);
  TEST_ASSERT_FALSE(bus.calibrationReadWhileImUpdate);
}

void test_init_job_stuck_nvm_no_spin_when_time_static() {
  FakeBus bus;
  bus.reg[cmd::REG_STATUS] = cmd::MASK_STATUS_IM_UPDATE;
  BME280::BME280 dev;

  Status st = dev.startInitJob(makeConfig(bus));
  TEST_ASSERT_TRUE(st.inProgress());

  JobPollResult result = pollWithBudget(dev, bus, 5);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(JobState::WAITING),
                          static_cast<uint8_t>(result.state));
  TEST_ASSERT_EQUAL_UINT32(1u, bus.statusReadCalls);
  TEST_ASSERT_TRUE(result.instructionsUsed <= 2u);

  for (uint16_t i = 1; i < 255; ++i) {
    const uint32_t statusReadsBefore = bus.statusReadCalls;
    result = pollWithBudget(dev, bus, 5);
    TEST_ASSERT_EQUAL_UINT32(statusReadsBefore + 1u, bus.statusReadCalls);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(JobState::WAITING),
                            static_cast<uint8_t>(result.state));
    TEST_ASSERT_EQUAL_UINT8(1u, result.instructionsUsed);
  }

  const uint32_t statusReadsBefore = bus.statusReadCalls;
  result = pollWithBudget(dev, bus, 5);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(JobState::FAILED),
                          static_cast<uint8_t>(result.state));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::TIMEOUT),
                          static_cast<uint8_t>(result.status.code));
  TEST_ASSERT_EQUAL_UINT32(statusReadsBefore, bus.statusReadCalls);
  TEST_ASSERT_FALSE(dev.isInitialized());
}

void test_apply_config_job_waits_for_not_measuring_before_writes() {
  FakeBus bus;
  BME280::BME280 dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  const uint32_t writesBefore = bus.writeCalls;
  bus.measuringStatusReadsRemaining = 2;

  Status st = dev.startApplyConfigJob();
  TEST_ASSERT_TRUE(st.inProgress());

  JobPollResult result = pollWithBudget(dev, bus, 3);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(JobState::WAITING),
                          static_cast<uint8_t>(result.state));
  TEST_ASSERT_EQUAL_UINT32(writesBefore, bus.writeCalls);

  result = pollWithBudget(dev, bus, 3);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(JobState::WAITING),
                          static_cast<uint8_t>(result.state));
  TEST_ASSERT_EQUAL_UINT32(writesBefore, bus.writeCalls);

  result = pollUntilTerminal(dev, bus, 3);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(JobState::DONE),
                          static_cast<uint8_t>(result.state));
  TEST_ASSERT_TRUE(result.status.ok());
  TEST_ASSERT_EQUAL_UINT32(writesBefore + 4u, bus.writeCalls);
}

void test_apply_config_job_checks_not_measuring_after_sleep_write() {
  FakeBus bus;
  BME280::BME280 dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  const uint32_t writesBefore = bus.writeCalls;
  bus.measuringOnStatusReadCall = bus.statusReadCalls + 2u;

  Status st = dev.startApplyConfigJob();
  TEST_ASSERT_TRUE(st.inProgress());

  JobPollResult result = pollWithBudget(dev, bus, 4);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(JobState::WAITING),
                          static_cast<uint8_t>(result.state));
  TEST_ASSERT_EQUAL_UINT32(writesBefore + 1u, bus.writeCalls);
  TEST_ASSERT_EQUAL_HEX8(cmd::REG_CTRL_MEAS, bus.lastWriteReg);
  TEST_ASSERT_TRUE(dev.hardwareConfigDirty());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::BUSY),
                          static_cast<uint8_t>(dev.hardwareConfigDirtyError().code));

  result = pollUntilTerminal(dev, bus, 4);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(JobState::DONE),
                          static_cast<uint8_t>(result.state));
  TEST_ASSERT_TRUE(result.status.ok());
  TEST_ASSERT_FALSE(dev.hardwareConfigDirty());
}

void test_init_job_success_clears_existing_dirty_state() {
  FakeBus bus;
  BME280::BME280 dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  Status st = dev.writeRegister(cmd::REG_CTRL_MEAS, 0);
  TEST_ASSERT_TRUE(st.ok());
  TEST_ASSERT_TRUE(dev.hardwareConfigDirty());

  st = dev.startInitJob(makeConfig(bus));
  TEST_ASSERT_TRUE(st.inProgress());
  const JobPollResult result = pollUntilTerminal(dev, bus, 3);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(JobState::DONE),
                          static_cast<uint8_t>(result.state));
  TEST_ASSERT_TRUE(result.status.ok());
  TEST_ASSERT_FALSE(dev.hardwareConfigDirty());
  TEST_ASSERT_TRUE(dev.hardwareConfigDirtyError().ok());
}

void test_recovery_job_success_clears_existing_dirty_state() {
  FakeBus bus;
  BME280::BME280 dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  Status st = dev.writeRegister(cmd::REG_CONFIG, 0xA0);
  TEST_ASSERT_TRUE(st.ok());
  TEST_ASSERT_TRUE(dev.hardwareConfigDirty());

  st = dev.startRecoveryJob();
  TEST_ASSERT_TRUE(st.inProgress());
  const JobPollResult result = pollUntilTerminal(dev, bus, 3);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(JobState::DONE),
                          static_cast<uint8_t>(result.state));
  TEST_ASSERT_TRUE(result.status.ok());
  TEST_ASSERT_FALSE(dev.hardwareConfigDirty());
  TEST_ASSERT_TRUE(dev.hardwareConfigDirtyError().ok());
}

void test_recovery_job_failure_after_reset_marks_dirty() {
  FakeBus bus;
  BME280::BME280 dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  bus.failReadReg = cmd::REG_STATUS;
  bus.failReadRegRemaining = 1;
  bus.readError = Status::Error(Err::I2C_TIMEOUT, "recovery nvm timeout", -93);

  Status st = dev.startRecoveryJob();
  TEST_ASSERT_TRUE(st.inProgress());
  const JobPollResult result = pollWithBudget(dev, bus, 4);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(JobState::FAILED),
                          static_cast<uint8_t>(result.state));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_TIMEOUT),
                          static_cast<uint8_t>(result.status.code));
  TEST_ASSERT_EQUAL_INT32(-93, result.status.detail);
  TEST_ASSERT_TRUE(dev.hardwareConfigDirty());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_TIMEOUT),
                          static_cast<uint8_t>(dev.hardwareConfigDirtyError().code));
  TEST_ASSERT_EQUAL_INT32(-93, dev.hardwareConfigDirtyError().detail);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_TIMEOUT),
                          static_cast<uint8_t>(dev.lastError().code));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::DEGRADED),
                          static_cast<uint8_t>(dev.state()));
  TEST_ASSERT_EQUAL_UINT32(1u, dev.totalFailures());
}

void test_missing_now_ms_fallback_is_framework_neutral() {
  FakeBus bus;
  BME280::BME280 dev;
  Config cfg = makeConfig(bus);
  cfg.nowMs = nullptr;
  cfg.timeUser = nullptr;
  TEST_ASSERT_TRUE(dev.begin(cfg).ok());

  setMillis(4321);
  Status st = dev.recover();
  TEST_ASSERT_TRUE(st.ok());
  TEST_ASSERT_EQUAL_UINT32(0u, dev.lastOkMs());
}

void test_begin_without_now_ms_uses_framework_neutral_fallback() {
  FakeBus bus;
  BME280::BME280 dev;
  Config cfg = makeConfig(bus);
  cfg.nowMs = nullptr;
  cfg.timeUser = nullptr;

  Status st = dev.begin(cfg);
  TEST_ASSERT_TRUE(st.ok());
  setMillis(4242u);
  st = dev.setMode(Mode::NORMAL);
  TEST_ASSERT_TRUE(st.ok());
  TEST_ASSERT_EQUAL_UINT32(0u, dev.lastOkMs());
}

void test_request_measurement_requires_now_ms_hook() {
  FakeBus bus;
  BME280::BME280 dev;
  Config cfg = makeConfig(bus);
  cfg.nowMs = nullptr;
  cfg.timeUser = nullptr;
  TEST_ASSERT_TRUE(dev.begin(cfg).ok());

  const uint32_t writesBefore = bus.writeCalls;
  Status st = dev.requestMeasurement();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_CONFIG),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_UINT32(writesBefore, bus.writeCalls);
  TEST_ASSERT_FALSE(dev.measurementReady());
}

void test_probe_transport_fault_is_preserved_and_does_not_update_health() {
  FakeBus bus;
  BME280::BME280 dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  const uint32_t beforeSuccess = dev.totalSuccess();
  const uint32_t beforeFailures = dev.totalFailures();
  const DriverState beforeState = dev.state();

  bus.readErrorRemaining = 1;
  bus.readError = Status::Error(Err::I2C_ERROR, "forced probe error", -7);
  Status st = dev.probe();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_ERROR),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_INT32(-7, st.detail);
  TEST_ASSERT_EQUAL_UINT32(beforeSuccess, dev.totalSuccess());
  TEST_ASSERT_EQUAL_UINT32(beforeFailures, dev.totalFailures());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(beforeState),
                          static_cast<uint8_t>(dev.state()));
}

void test_probe_address_nack_maps_to_device_not_found_without_health_update() {
  FakeBus bus;
  BME280::BME280 dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  const uint32_t beforeSuccess = dev.totalSuccess();
  const uint32_t beforeFailures = dev.totalFailures();
  const DriverState beforeState = dev.state();

  bus.readErrorRemaining = 1;
  bus.readError = Status::Error(Err::I2C_NACK_ADDR, "forced probe nack", 2);
  Status st = dev.probe();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::DEVICE_NOT_FOUND),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_INT32(2, st.detail);
  TEST_ASSERT_EQUAL_UINT32(beforeSuccess, dev.totalSuccess());
  TEST_ASSERT_EQUAL_UINT32(beforeFailures, dev.totalFailures());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(beforeState),
                          static_cast<uint8_t>(dev.state()));
}

void test_probe_address_nack_maps_to_device_not_found() {
  FakeBus bus;
  BME280::BME280 dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  bus.readErrorRemaining = 1;
  bus.readError = Status::Error(Err::I2C_NACK_ADDR, "forced address nack", -17);
  Status st = dev.probe();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::DEVICE_NOT_FOUND),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_INT32(-17, st.detail);
  TEST_ASSERT_EQUAL_UINT32(0u, dev.totalFailures());
}

void test_probe_preserves_non_address_transport_errors_without_health_update() {
  FakeBus bus;
  BME280::BME280 dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  const uint32_t beforeFailures = dev.totalFailures();
  const uint32_t beforeSuccess = dev.totalSuccess();
  const DriverState beforeState = dev.state();

  const Err errors[] = {Err::I2C_TIMEOUT, Err::I2C_BUS, Err::I2C_NACK_DATA, Err::I2C_ERROR};
  for (size_t i = 0; i < sizeof(errors) / sizeof(errors[0]); ++i) {
    bus.readErrorRemaining = 1;
    bus.readError = Status::Error(errors[i], "forced probe transport error",
                                  static_cast<int32_t>(-60 - static_cast<int32_t>(i)));
    Status st = dev.probe();
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(errors[i]), static_cast<uint8_t>(st.code));
    TEST_ASSERT_EQUAL_INT32(static_cast<int32_t>(-60 - static_cast<int32_t>(i)), st.detail);
    TEST_ASSERT_EQUAL_UINT32(beforeFailures, dev.totalFailures());
    TEST_ASSERT_EQUAL_UINT32(beforeSuccess, dev.totalSuccess());
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(beforeState),
                            static_cast<uint8_t>(dev.state()));
  }
}

void test_probe_chip_id_mismatch_does_not_update_health() {
  FakeBus bus;
  BME280::BME280 dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  const uint32_t beforeFailures = dev.totalFailures();
  const uint32_t beforeSuccess = dev.totalSuccess();
  bus.chipId = 0x61;

  Status st = dev.probe();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::CHIP_ID_MISMATCH),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_INT32(0x61, st.detail);
  TEST_ASSERT_EQUAL_UINT32(beforeFailures, dev.totalFailures());
  TEST_ASSERT_EQUAL_UINT32(beforeSuccess, dev.totalSuccess());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::READY),
                          static_cast<uint8_t>(dev.state()));
}

void test_begin_accepts_both_supported_addresses_and_rejects_wrong_address() {
  FakeBus bus76;
  BME280::BME280 dev76;
  TEST_ASSERT_TRUE(dev76.begin(makeConfig(bus76)).ok());
  SettingsSnapshot snap{};
  TEST_ASSERT_TRUE(dev76.getSettings(snap).ok());
  TEST_ASSERT_EQUAL_UINT8(0x76, snap.i2cAddress);

  FakeBus bus77;
  bus77.deviceAddress = 0x77;
  Config cfg77 = makeConfig(bus77);
  cfg77.i2cAddress = 0x77;
  BME280::BME280 dev77;
  TEST_ASSERT_TRUE(dev77.begin(cfg77).ok());
  TEST_ASSERT_TRUE(dev77.getSettings(snap).ok());
  TEST_ASSERT_EQUAL_UINT8(0x77, snap.i2cAddress);

  FakeBus wrongAddressBus;
  wrongAddressBus.deviceAddress = 0x77;
  BME280::BME280 wrongAddressDev;
  Status st = wrongAddressDev.begin(makeConfig(wrongAddressBus));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::DEVICE_NOT_FOUND),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_INT32(0x76, st.detail);
  TEST_ASSERT_FALSE(wrongAddressDev.isInitialized());

  FakeBus wrongChipIdBus;
  wrongChipIdBus.deviceAddress = 0x77;
  wrongChipIdBus.chipId = 0x61;
  Config wrongChipCfg = makeConfig(wrongChipIdBus);
  wrongChipCfg.i2cAddress = 0x77;
  BME280::BME280 wrongChipDev;
  st = wrongChipDev.begin(wrongChipCfg);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::CHIP_ID_MISMATCH),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_INT32(0x61, st.detail);
  TEST_ASSERT_FALSE(wrongChipDev.isInitialized());
}

void test_begin_rejects_wrong_chip_id() {
  FakeBus bus;
  bus.chipId = 0x61;
  BME280::BME280 dev;

  Status st = dev.begin(makeConfig(bus));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::CHIP_ID_MISMATCH),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_INT32(0x61, st.detail);
  TEST_ASSERT_FALSE(dev.isInitialized());
}

void test_begin_address_nack_maps_to_device_not_found() {
  FakeBus bus;
  bus.readErrorRemaining = 1;
  bus.readError = Status::Error(Err::I2C_NACK_ADDR, "chip id address nack", -22);
  BME280::BME280 dev;

  Status st = dev.begin(makeConfig(bus));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::DEVICE_NOT_FOUND),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_INT32(-22, st.detail);
  TEST_ASSERT_FALSE(dev.isInitialized());
  TEST_ASSERT_EQUAL_UINT32(0u, dev.totalFailures());
}

void test_begin_address_nack_after_chip_id_maps_to_device_not_found() {
  {
    FakeBus bus;
    bus.failReadReg = cmd::REG_STATUS;
    bus.failReadRegRemaining = 1;
    bus.readError = Status::Error(Err::I2C_NACK_ADDR, "nvm address nack", -23);
    BME280::BME280 dev;

    Status st = dev.begin(makeConfig(bus));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::DEVICE_NOT_FOUND),
                            static_cast<uint8_t>(st.code));
    TEST_ASSERT_EQUAL_INT32(-23, st.detail);
    TEST_ASSERT_FALSE(dev.isInitialized());
  }

  {
    FakeBus bus;
    bus.failReadRegEnabled = true;
    bus.failReadReg = cmd::REG_CALIB_TP_START;
    bus.readError = Status::Error(Err::I2C_NACK_ADDR, "calibration address nack", -24);
    BME280::BME280 dev;

    Status st = dev.begin(makeConfig(bus));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::DEVICE_NOT_FOUND),
                            static_cast<uint8_t>(st.code));
    TEST_ASSERT_EQUAL_INT32(-24, st.detail);
    TEST_ASSERT_FALSE(dev.isInitialized());
  }

  {
    FakeBus bus;
    bus.failWriteOnCall = 1;
    bus.writeError = Status::Error(Err::I2C_NACK_ADDR, "config address nack", -25);
    BME280::BME280 dev;

    Status st = dev.begin(makeConfig(bus));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::DEVICE_NOT_FOUND),
                            static_cast<uint8_t>(st.code));
    TEST_ASSERT_EQUAL_INT32(-25, st.detail);
    TEST_ASSERT_FALSE(dev.isInitialized());
  }
}

void test_begin_preserves_timeout_transport_error() {
  FakeBus bus;
  bus.readErrorRemaining = 1;
  bus.readError = Status::Error(Err::I2C_TIMEOUT, "chip id timeout", -21);
  BME280::BME280 dev;

  Status st = dev.begin(makeConfig(bus));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_TIMEOUT),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_INT32(-21, st.detail);
  TEST_ASSERT_FALSE(dev.isInitialized());
}

void test_begin_preserves_chip_id_bus_and_data_errors() {
  const Err errors[] = {Err::I2C_BUS, Err::I2C_NACK_DATA, Err::I2C_ERROR};
  for (size_t i = 0; i < sizeof(errors) / sizeof(errors[0]); ++i) {
    FakeBus bus;
    bus.readErrorRemaining = 1;
    bus.readError = Status::Error(errors[i], "chip id transport error",
                                  static_cast<int32_t>(-70 - static_cast<int32_t>(i)));
    BME280::BME280 dev;

    Status st = dev.begin(makeConfig(bus));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(errors[i]), static_cast<uint8_t>(st.code));
    TEST_ASSERT_EQUAL_INT32(static_cast<int32_t>(-70 - static_cast<int32_t>(i)), st.detail);
    TEST_ASSERT_FALSE(dev.isInitialized());
  }
}

void test_recover_failure_updates_health_once() {
  FakeBus bus;
  BME280::BME280 dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  bus.readErrorRemaining = 1;
  bus.readError = Status::Error(Err::I2C_ERROR, "forced recover error", -8);
  Status st = dev.recover();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_ERROR), static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_UINT32(1u, dev.totalFailures());
  TEST_ASSERT_EQUAL_UINT8(1u, dev.consecutiveFailures());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::DEGRADED),
                          static_cast<uint8_t>(dev.state()));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_ERROR),
                          static_cast<uint8_t>(dev.lastError().code));
  TEST_ASSERT_EQUAL_UINT32(bus.nowMs, dev.lastErrorMs());
}

void test_recover_success_returns_ready() {
  FakeBus bus;
  BME280::BME280 dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  bus.readErrorRemaining = 1;
  bus.readError = Status::Error(Err::I2C_ERROR, "forced recover error", -9);
  (void)dev.recover();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::DEGRADED),
                          static_cast<uint8_t>(dev.state()));

  const uint32_t successBefore = dev.totalSuccess();
  bus.nowMs = 4321;
  Status st = dev.recover();
  TEST_ASSERT_TRUE(st.ok());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::READY),
                          static_cast<uint8_t>(dev.state()));
  TEST_ASSERT_EQUAL_UINT8(0u, dev.consecutiveFailures());
  TEST_ASSERT_GREATER_THAN_UINT32(successBefore, dev.totalSuccess());
  TEST_ASSERT_EQUAL_UINT32(1u, dev.totalFailures());
  TEST_ASSERT_EQUAL_UINT32(4321u, dev.lastOkMs());
}

void test_recover_chip_id_mismatch_updates_health() {
  FakeBus bus;
  BME280::BME280 dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  bus.chipId = 0x61;
  Status st = dev.recover();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::CHIP_ID_MISMATCH),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_UINT32(1u, dev.totalFailures());
  TEST_ASSERT_EQUAL_UINT8(1u, dev.consecutiveFailures());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::DEGRADED),
                          static_cast<uint8_t>(dev.state()));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::CHIP_ID_MISMATCH),
                          static_cast<uint8_t>(dev.lastError().code));
}

void test_recover_preserves_transport_error_code() {
  FakeBus bus;
  BME280::BME280 dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  bus.readErrorRemaining = 1;
  bus.readError = Status::Error(Err::I2C_NACK_ADDR, "forced recover nack", 7);
  Status st = dev.recover();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_NACK_ADDR),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_NACK_ADDR),
                          static_cast<uint8_t>(dev.lastError().code));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::DEGRADED),
                          static_cast<uint8_t>(dev.state()));
}

void test_set_oversampling_rejects_invalid_compensation_combo_without_write() {
  FakeBus bus;
  BME280::BME280 dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());
  const uint32_t writesBefore = bus.writeCalls;

  Status st = dev.setOversamplingT(Oversampling::SKIP);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_PARAM),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_UINT32(writesBefore, bus.writeCalls);

  Oversampling osrs = Oversampling::SKIP;
  st = dev.getOversamplingT(osrs);
  TEST_ASSERT_TRUE(st.ok());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Oversampling::X1),
                          static_cast<uint8_t>(osrs));
}

void test_humidity_oversampling_writes_ctrl_hum_then_ctrl_meas() {
  FakeBus bus;
  BME280::BME280 dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  const uint8_t writesBefore = bus.writeLogLen;
  Status st = dev.setOversamplingH(Oversampling::X4);
  TEST_ASSERT_TRUE(st.ok());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(writesBefore + 2u), bus.writeLogLen);
  TEST_ASSERT_EQUAL_UINT8(cmd::REG_CTRL_HUM, bus.writeRegLog[writesBefore]);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Oversampling::X4),
                          bus.writeValueLog[writesBefore] & cmd::MASK_CTRL_HUM_OSRS_H);
  TEST_ASSERT_EQUAL_UINT8(cmd::REG_CTRL_MEAS, bus.writeRegLog[writesBefore + 1u]);

  Oversampling osrs = Oversampling::SKIP;
  TEST_ASSERT_TRUE(dev.getOversamplingH(osrs).ok());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Oversampling::X4),
                          static_cast<uint8_t>(osrs));
  TEST_ASSERT_FALSE(dev.hardwareConfigDirty());
}

void test_config_change_in_normal_mode_sleeps_writes_config_and_restores() {
  FakeBus bus;
  BME280::BME280 dev;
  Config cfg = makeConfig(bus);
  cfg.mode = Mode::NORMAL;
  TEST_ASSERT_TRUE(dev.begin(cfg).ok());

  const uint8_t writesBefore = bus.writeLogLen;
  Status st = dev.setFilter(Filter::X4);
  TEST_ASSERT_TRUE(st.ok());

  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(writesBefore + 3u), bus.writeLogLen);
  TEST_ASSERT_EQUAL_UINT8(cmd::REG_CTRL_MEAS, bus.writeRegLog[writesBefore]);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Mode::SLEEP),
                          bus.writeValueLog[writesBefore] & cmd::MASK_CTRL_MEAS_MODE);
  TEST_ASSERT_EQUAL_UINT8(cmd::REG_CONFIG, bus.writeRegLog[writesBefore + 1u]);
  TEST_ASSERT_EQUAL_UINT8(cmd::REG_CTRL_MEAS, bus.writeRegLog[writesBefore + 2u]);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Mode::NORMAL),
                          bus.writeValueLog[writesBefore + 2u] & cmd::MASK_CTRL_MEAS_MODE);

  Mode mode = Mode::SLEEP;
  TEST_ASSERT_TRUE(dev.getMode(mode).ok());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Mode::NORMAL), static_cast<uint8_t>(mode));
  TEST_ASSERT_FALSE(dev.hardwareConfigDirty());
}

void test_config_write_while_measuring_returns_busy_without_write() {
  FakeBus bus;
  BME280::BME280 dev;
  Config cfg = makeConfig(bus);
  cfg.mode = Mode::NORMAL;
  TEST_ASSERT_TRUE(dev.begin(cfg).ok());

  const uint32_t writesBefore = bus.writeCalls;
  const uint8_t writeLogBefore = bus.writeLogLen;
  bus.reg[cmd::REG_STATUS] = cmd::MASK_STATUS_MEASURING;

  Status st = dev.setFilter(Filter::X4);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::BUSY),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_UINT32(writesBefore, bus.writeCalls);
  TEST_ASSERT_EQUAL_UINT8(writeLogBefore, bus.writeLogLen);
  TEST_ASSERT_FALSE(dev.hardwareConfigDirty());
}

void test_config_write_measuring_after_sleep_marks_dirty_without_config_write() {
  FakeBus bus;
  BME280::BME280 dev;
  Config cfg = makeConfig(bus);
  cfg.mode = Mode::NORMAL;
  TEST_ASSERT_TRUE(dev.begin(cfg).ok());

  const uint32_t statusBefore = bus.statusReadCalls;
  const uint8_t writesBefore = bus.writeLogLen;
  bus.measuringOnStatusReadCall = statusBefore + 2u;

  Status st = dev.setFilter(Filter::X4);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::BUSY),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_TRUE(dev.hardwareConfigDirty());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::BUSY),
                          static_cast<uint8_t>(dev.hardwareConfigDirtyError().code));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(writesBefore + 2u), bus.writeLogLen);
  TEST_ASSERT_EQUAL_UINT8(cmd::REG_CTRL_MEAS, bus.writeRegLog[writesBefore]);
  TEST_ASSERT_EQUAL_UINT8(cmd::REG_CTRL_MEAS, bus.writeRegLog[writesBefore + 1u]);
}

void test_config_change_failure_at_sleep_step_marks_dirty() {
  FakeBus bus;
  BME280::BME280 dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  bus.failWriteOnCall = bus.writeCalls + 1u;
  bus.writeError = Status::Error(Err::I2C_TIMEOUT, "sleep write timeout", -41);
  Status st = dev.setFilter(Filter::X2);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_TIMEOUT),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_TRUE(dev.hardwareConfigDirty());
  TEST_ASSERT_EQUAL_INT32(-41, dev.hardwareConfigDirtyError().detail);
}

void test_config_change_failure_at_config_step_marks_dirty_after_restore() {
  FakeBus bus;
  BME280::BME280 dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  bus.failWriteOnCall = bus.writeCalls + 2u;
  bus.writeError = Status::Error(Err::I2C_NACK_DATA, "config write nack", -42);
  Status st = dev.setStandby(Standby::MS_250);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_NACK_DATA),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_TRUE(dev.hardwareConfigDirty());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_NACK_DATA),
                          static_cast<uint8_t>(dev.hardwareConfigDirtyError().code));
  TEST_ASSERT_EQUAL_INT32(-42, dev.hardwareConfigDirtyError().detail);
  TEST_ASSERT_EQUAL_UINT8(cmd::REG_CTRL_MEAS, bus.lastWriteReg);
}

void test_humidity_ctrl_hum_failure_marks_dirty_and_preserves_error() {
  FakeBus bus;
  BME280::BME280 dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  bus.failWriteOnCall = bus.writeCalls + 1u;
  bus.writeError = Status::Error(Err::I2C_BUS, "ctrl_hum bus error", -43);
  Status st = dev.setOversamplingH(Oversampling::X4);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_BUS),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_TRUE(dev.hardwareConfigDirty());
  TEST_ASSERT_EQUAL_INT32(-43, dev.hardwareConfigDirtyError().detail);
}

void test_recover_apply_config_first_write_failure_marks_dirty() {
  FakeBus bus;
  BME280::BME280 dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  bus.failWriteOnCall = bus.writeCalls + 1u;
  bus.writeError = Status::Error(Err::I2C_TIMEOUT, "apply sleep timeout", -44);
  Status st = dev.recover();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_TIMEOUT),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_TRUE(dev.hardwareConfigDirty());
  TEST_ASSERT_EQUAL_INT32(-44, dev.hardwareConfigDirtyError().detail);
}

void test_measurement_time_estimate_uses_oversampling_formula() {
  FakeBus bus;
  BME280::BME280 dev;
  Config cfg = makeConfig(bus);
  cfg.osrsT = Oversampling::X1;
  cfg.osrsP = Oversampling::X1;
  cfg.osrsH = Oversampling::X1;
  cfg.standby = Standby::MS_62_5;
  TEST_ASSERT_TRUE(dev.begin(cfg).ok());
  TEST_ASSERT_EQUAL_UINT32(11u, dev.estimateMeasurementTimeMs());
  TEST_ASSERT_EQUAL_UINT32(63u, dev.getStandbyTimeMs());
  TEST_ASSERT_EQUAL_UINT32(74u, dev.estimateNormalCycleMs());

  TEST_ASSERT_TRUE(dev.setOversamplingT(Oversampling::X16).ok());
  TEST_ASSERT_TRUE(dev.setOversamplingP(Oversampling::X16).ok());
  TEST_ASSERT_TRUE(dev.setOversamplingH(Oversampling::X16).ok());
  TEST_ASSERT_EQUAL_UINT32(114u, dev.estimateMeasurementTimeMs());
}

void test_partial_config_restore_failure_marks_hardware_dirty() {
  FakeBus bus;
  BME280::BME280 dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  const uint32_t failCall = bus.writeCalls + 3u;  // sleep write, config write, restore write
  bus.failWriteOnCall = failCall;
  bus.writeError = Status::Error(Err::I2C_TIMEOUT, "restore failed", -31);

  Status st = dev.setFilter(Filter::X2);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_TIMEOUT),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_TRUE(dev.hardwareConfigDirty());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_TIMEOUT),
                          static_cast<uint8_t>(dev.hardwareConfigDirtyError().code));
  TEST_ASSERT_EQUAL_INT32(-31, dev.hardwareConfigDirtyError().detail);

  SettingsSnapshot snap{};
  TEST_ASSERT_TRUE(dev.getSettings(snap).ok());
  TEST_ASSERT_TRUE(snap.hardwareConfigDirty);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_TIMEOUT),
                          static_cast<uint8_t>(snap.hardwareConfigDirtyError.code));
}

void test_dirty_state_clears_after_successful_recover_resync() {
  FakeBus bus;
  BME280::BME280 dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  bus.failWriteOnCall = bus.writeCalls + 3u;
  bus.writeError = Status::Error(Err::I2C_BUS, "restore bus error", -32);
  Status st = dev.setStandby(Standby::MS_250);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_BUS), static_cast<uint8_t>(st.code));
  TEST_ASSERT_TRUE(dev.hardwareConfigDirty());

  st = dev.recover();
  TEST_ASSERT_TRUE(st.ok());
  TEST_ASSERT_FALSE(dev.hardwareConfigDirty());
  TEST_ASSERT_TRUE(dev.hardwareConfigDirtyError().ok());
}

void test_diagnostic_config_write_marks_dirty_after_success_and_recover_clears() {
  FakeBus bus;
  BME280::BME280 dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  Status st = dev.writeRegister(cmd::REG_CTRL_MEAS, 0);
  TEST_ASSERT_TRUE(st.ok());
  TEST_ASSERT_EQUAL_UINT8(0u, bus.reg[cmd::REG_CTRL_MEAS]);
  TEST_ASSERT_TRUE(dev.hardwareConfigDirty());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_CONFIG),
                          static_cast<uint8_t>(dev.hardwareConfigDirtyError().code));
  TEST_ASSERT_EQUAL_INT32(cmd::REG_CTRL_MEAS, dev.hardwareConfigDirtyError().detail);

  st = dev.recover();
  TEST_ASSERT_TRUE(st.ok());
  TEST_ASSERT_FALSE(dev.hardwareConfigDirty());
  TEST_ASSERT_TRUE(dev.hardwareConfigDirtyError().ok());
}

void test_diagnostic_config_write_invalidates_cached_samples() {
  FakeBus bus;
  setBoschSyntheticCalibration(bus);
  setRawSample(bus, 415148, 519888, 30000);
  BME280::BME280 dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());
  captureForcedSample(dev, bus);
  TEST_ASSERT_TRUE(dev.hasSample());

  Status st = dev.writeRegister(cmd::REG_CTRL_MEAS, 0);
  TEST_ASSERT_TRUE(st.ok());
  TEST_ASSERT_TRUE(dev.hardwareConfigDirty());
  TEST_ASSERT_FALSE(dev.hasSample());
  TEST_ASSERT_FALSE(dev.measurementReady());

  RawSample raw{};
  st = dev.getRawSample(raw);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::MEASUREMENT_NOT_READY),
                          static_cast<uint8_t>(st.code));
}

void test_diagnostic_config_block_write_marks_dirty_when_range_overlaps_config() {
  FakeBus bus;
  BME280::BME280 dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  const uint8_t payload[] = {0x01, 0x02, 0x03, 0x04};
  Status st = dev.writeRegisters(cmd::REG_CTRL_HUM, payload, sizeof(payload));
  TEST_ASSERT_TRUE(st.ok());
  TEST_ASSERT_TRUE(dev.hardwareConfigDirty());
  TEST_ASSERT_EQUAL_INT32(cmd::REG_CTRL_HUM, dev.hardwareConfigDirtyError().detail);
}

void test_diagnostic_non_config_write_does_not_mark_dirty() {
  FakeBus bus;
  BME280::BME280 dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  Status st = dev.writeRegister(cmd::REG_PRESS_MSB, 0xAB);
  TEST_ASSERT_TRUE(st.ok());
  TEST_ASSERT_EQUAL_UINT8(0xAB, bus.reg[cmd::REG_PRESS_MSB]);
  TEST_ASSERT_FALSE(dev.hardwareConfigDirty());
  TEST_ASSERT_TRUE(dev.hardwareConfigDirtyError().ok());
}

void test_diagnostic_config_write_failure_preserves_error_and_marks_dirty() {
  FakeBus bus;
  BME280::BME280 dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  bus.failWriteOnCall = bus.writeCalls + 1u;
  bus.writeError = Status::Error(Err::I2C_TIMEOUT, "diagnostic write timeout", -52);
  Status st = dev.writeRegister(cmd::REG_CONFIG, 0xA0);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_TIMEOUT),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_INT32(-52, st.detail);
  TEST_ASSERT_TRUE(dev.hardwareConfigDirty());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_TIMEOUT),
                          static_cast<uint8_t>(dev.hardwareConfigDirtyError().code));
  TEST_ASSERT_EQUAL_INT32(-52, dev.hardwareConfigDirtyError().detail);
}

void test_diagnostic_config_write_address_nack_does_not_mark_dirty() {
  FakeBus bus;
  BME280::BME280 dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  bus.deviceAddress = 0x77;
  Status st = dev.writeRegister(cmd::REG_CONFIG, 0xA0);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_NACK_ADDR),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_FALSE(dev.hardwareConfigDirty());
  TEST_ASSERT_TRUE(dev.hardwareConfigDirtyError().ok());
}

void test_recover_preserves_cached_sample_until_successful_resync_then_invalidates() {
  FakeBus bus;
  setBoschSyntheticCalibration(bus);
  setRawSample(bus, 415148, 519888, 30000);
  BME280::BME280 dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  captureForcedSample(dev, bus);
  TEST_ASSERT_TRUE(dev.hasSample());
  const uint32_t sampleTimestamp = dev.sampleTimestampMs();
  RawSample rawBefore{};
  CompensatedSample compBefore{};
  TEST_ASSERT_TRUE(dev.getRawSample(rawBefore).ok());
  TEST_ASSERT_TRUE(dev.getCompensatedSample(compBefore).ok());

  bus.failWriteOnCall = bus.writeCalls + 3u;  // sleep write, config write, restore write
  bus.writeError = Status::Error(Err::I2C_BUS, "make dirty before recover", -35);
  Status st = dev.setFilter(Filter::X2);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_BUS),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_TRUE(dev.hardwareConfigDirty());
  TEST_ASSERT_EQUAL_INT32(-35, dev.hardwareConfigDirtyError().detail);
  TEST_ASSERT_TRUE(dev.hasSample());

  bus.failWriteOnCall = bus.writeCalls + 2u;  // recover sleep write succeeds, config write fails
  bus.writeError = Status::Error(Err::I2C_TIMEOUT, "recover resync timeout", -36);
  st = dev.recover();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_TIMEOUT),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_TRUE(dev.hardwareConfigDirty());
  TEST_ASSERT_EQUAL_INT32(-35, dev.hardwareConfigDirtyError().detail);
  TEST_ASSERT_TRUE(dev.hasSample());
  TEST_ASSERT_EQUAL_UINT32(sampleTimestamp, dev.sampleTimestampMs());

  SettingsSnapshot snap{};
  TEST_ASSERT_TRUE(dev.getSettings(snap).ok());
  TEST_ASSERT_TRUE(snap.hardwareConfigDirty);
  TEST_ASSERT_TRUE(snap.hasSample);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Filter::OFF),
                          static_cast<uint8_t>(snap.filter));
  TEST_ASSERT_EQUAL_INT32(rawBefore.adcT, snap.rawSample.adcT);
  TEST_ASSERT_EQUAL_INT32(rawBefore.adcP, snap.rawSample.adcP);
  TEST_ASSERT_EQUAL_INT32(rawBefore.adcH, snap.rawSample.adcH);
  TEST_ASSERT_EQUAL_INT32(compBefore.tempC_x100, snap.compSample.tempC_x100);
  TEST_ASSERT_EQUAL_UINT32(compBefore.pressurePa, snap.compSample.pressurePa);
  TEST_ASSERT_EQUAL_UINT32(compBefore.humidityPct_x1024, snap.compSample.humidityPct_x1024);

  st = dev.recover();
  TEST_ASSERT_TRUE(st.ok());
  TEST_ASSERT_FALSE(dev.hardwareConfigDirty());
  TEST_ASSERT_TRUE(dev.hardwareConfigDirtyError().ok());
  TEST_ASSERT_FALSE(dev.hasSample());
  TEST_ASSERT_FALSE(dev.measurementReady());
  TEST_ASSERT_EQUAL_UINT32(0u, dev.sampleTimestampMs());

  RawSample rawAfter{};
  st = dev.getRawSample(rawAfter);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::MEASUREMENT_NOT_READY),
                          static_cast<uint8_t>(st.code));
}

void test_recovery_job_failure_preserves_cached_sample_until_successful_resync() {
  FakeBus bus;
  setBoschSyntheticCalibration(bus);
  setRawSample(bus, 415148, 519888, 30000);
  BME280::BME280 dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  captureForcedSample(dev, bus);
  TEST_ASSERT_TRUE(dev.hasSample());
  const uint32_t sampleTimestamp = dev.sampleTimestampMs();
  const uint32_t generationBefore = dev.configGeneration();
  SampleEnvelope sampleBefore{};
  TEST_ASSERT_TRUE(dev.getSampleEnvelope(sampleBefore).ok());

  bus.failReadReg = cmd::REG_STATUS;
  bus.failReadRegRemaining = 1;
  bus.readError = Status::Error(Err::I2C_TIMEOUT, "recovery nvm timeout", -126);
  TEST_ASSERT_TRUE(dev.startRecoveryJob().inProgress());
  JobPollResult result = pollWithBudget(dev, bus, 4);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(JobState::FAILED),
                          static_cast<uint8_t>(result.state));
  TEST_ASSERT_TRUE(dev.hardwareConfigDirty());
  TEST_ASSERT_TRUE(dev.hasSample());
  TEST_ASSERT_EQUAL_UINT32(sampleTimestamp, dev.sampleTimestampMs());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(SampleFreshness::STALE_AFTER_CONFIG_DIRTY),
                          static_cast<uint8_t>(dev.sampleFreshness()));

  SampleEnvelope sampleAfterFailure{};
  TEST_ASSERT_TRUE(dev.getSampleEnvelope(sampleAfterFailure).ok());
  assertSampleEnvelopeEqual(sampleBefore, sampleAfterFailure);

  TEST_ASSERT_TRUE(dev.startRecoveryJob().inProgress());
  result = pollUntilTerminal(dev, bus, 4);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(JobState::DONE),
                          static_cast<uint8_t>(result.state));
  TEST_ASSERT_FALSE(dev.hardwareConfigDirty());
  TEST_ASSERT_TRUE(dev.hasSample());
  TEST_ASSERT_TRUE(dev.configGeneration() > generationBefore);
  SampleEnvelope sampleAfterRecovery{};
  TEST_ASSERT_TRUE(dev.getSampleEnvelope(sampleAfterRecovery).ok());
  assertSampleEnvelopeEqual(sampleBefore, sampleAfterRecovery);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(SampleFreshness::STALE_AFTER_CONFIG_CHANGE),
                          static_cast<uint8_t>(dev.sampleFreshness()));
}

void test_invalid_begin_does_not_clear_existing_dirty_state() {
  FakeBus bus;
  BME280::BME280 dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  bus.failWriteOnCall = bus.writeCalls + 3u;
  bus.writeError = Status::Error(Err::I2C_BUS, "restore bus error", -34);
  Status st = dev.setFilter(Filter::X2);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_BUS), static_cast<uint8_t>(st.code));
  TEST_ASSERT_TRUE(dev.hardwareConfigDirty());

  Config bad = makeConfig(bus);
  bad.i2cAddress = 0x75;
  st = dev.begin(bad);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_CONFIG),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_TRUE(dev.hardwareConfigDirty());
  TEST_ASSERT_EQUAL_INT32(-34, dev.hardwareConfigDirtyError().detail);
}

void test_apply_config_partial_failure_marks_dirty_and_preserves_error() {
  FakeBus bus;
  BME280::BME280 dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  bus.failWriteOnCall = bus.writeCalls + 2u;  // ctrl_meas sleep succeeds, config write fails
  bus.writeError = Status::Error(Err::I2C_NACK_DATA, "config write nack", -33);
  Status st = dev.recover();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_NACK_DATA),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_TRUE(dev.hardwareConfigDirty());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_NACK_DATA),
                          static_cast<uint8_t>(dev.hardwareConfigDirtyError().code));
  TEST_ASSERT_EQUAL_INT32(-33, dev.hardwareConfigDirtyError().detail);
}

void test_soft_reset_write_timeout_marks_dirty_and_preserves_error() {
  FakeBus bus;
  BME280::BME280 dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  bus.failWriteOnCall = bus.writeCalls + 1u;
  bus.writeError = Status::Error(Err::I2C_TIMEOUT, "reset write timeout", -80);
  Status st = dev.softReset();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_TIMEOUT),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_INT32(-80, st.detail);
  TEST_ASSERT_TRUE(dev.hardwareConfigDirty());
  TEST_ASSERT_EQUAL_INT32(-80, dev.hardwareConfigDirtyError().detail);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::DEGRADED),
                          static_cast<uint8_t>(dev.state()));
}

void test_soft_reset_write_address_nack_preserves_error_without_dirty_state() {
  FakeBus bus;
  BME280::BME280 dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  bus.failWriteOnCall = bus.writeCalls + 1u;
  bus.writeError = Status::Error(Err::I2C_NACK_ADDR, "reset address nack", -91);
  Status st = dev.softReset();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_NACK_ADDR),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_INT32(-91, st.detail);
  TEST_ASSERT_FALSE(dev.hardwareConfigDirty());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::DEGRADED),
                          static_cast<uint8_t>(dev.state()));
}

void test_soft_reset_nvm_im_update_busy_marks_dirty_and_health_failure() {
  FakeBus bus;
  BME280::BME280 dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  bus.reg[cmd::REG_STATUS] = cmd::MASK_STATUS_IM_UPDATE;
  Status st = dev.softReset();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::BUSY),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_TRUE(dev.hardwareConfigDirty());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::BUSY),
                          static_cast<uint8_t>(dev.hardwareConfigDirtyError().code));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::BUSY),
                          static_cast<uint8_t>(dev.lastError().code));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::DEGRADED),
                          static_cast<uint8_t>(dev.state()));
}

void test_begin_nvm_transport_error_preserved() {
  FakeBus bus;
  bus.failReadReg = cmd::REG_STATUS;
  bus.failReadRegRemaining = 300;
  bus.readError = Status::Error(Err::I2C_TIMEOUT, "nvm status timeout", -81);
  BME280::BME280 dev;

  Status st = dev.begin(makeConfig(bus));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_TIMEOUT),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_INT32(-81, st.detail);
  TEST_ASSERT_FALSE(dev.isInitialized());
  TEST_ASSERT_EQUAL_UINT32(0u, bus.writeCalls);
}

void test_begin_calibration_read_failure_preserves_transport_error() {
  FakeBus bus;
  bus.failReadRegEnabled = true;
  bus.failReadReg = cmd::REG_CALIB_TP_START;
  bus.readError = Status::Error(Err::I2C_NACK_DATA, "begin calibration nack", -89);
  BME280::BME280 dev;

  Status st = dev.begin(makeConfig(bus));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_NACK_DATA),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_INT32(-89, st.detail);
  TEST_ASSERT_FALSE(dev.isInitialized());
}

void test_begin_apply_config_failure_marks_dirty_and_stays_uninitialized() {
  FakeBus bus;
  bus.failWriteOnCall = 1;
  bus.writeError = Status::Error(Err::I2C_BUS, "begin apply bus error", -90);
  BME280::BME280 dev;

  Status st = dev.begin(makeConfig(bus));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_BUS),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_FALSE(dev.isInitialized());
  TEST_ASSERT_TRUE(dev.hardwareConfigDirty());
  TEST_ASSERT_EQUAL_INT32(-90, dev.hardwareConfigDirtyError().detail);
}

void test_soft_reset_nvm_transport_error_preserved_and_marks_dirty() {
  FakeBus bus;
  BME280::BME280 dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  bus.failReadReg = cmd::REG_STATUS;
  bus.failReadRegRemaining = 300;
  bus.readError = Status::Error(Err::I2C_NACK_DATA, "nvm status data nack", -82);
  Status st = dev.softReset();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_NACK_DATA),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_INT32(-82, st.detail);
  TEST_ASSERT_TRUE(dev.hardwareConfigDirty());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_NACK_DATA),
                          static_cast<uint8_t>(dev.hardwareConfigDirtyError().code));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::DEGRADED),
                          static_cast<uint8_t>(dev.state()));
}

void test_soft_reset_nvm_status_transport_error_preserves_detail() {
  FakeBus bus;
  BME280::BME280 dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  bus.reg[cmd::REG_STATUS] = cmd::MASK_STATUS_IM_UPDATE;
  bus.failReadReg = cmd::REG_STATUS;
  bus.failReadRegRemaining = 1;
  bus.readError = Status::Error(Err::I2C_TIMEOUT, "first nvm status timeout", -92);

  Status st = dev.softReset();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_TIMEOUT),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_INT32(-92, st.detail);
  TEST_ASSERT_TRUE(dev.hardwareConfigDirty());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_TIMEOUT),
                          static_cast<uint8_t>(dev.hardwareConfigDirtyError().code));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_TIMEOUT),
                          static_cast<uint8_t>(dev.lastError().code));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::DEGRADED),
                          static_cast<uint8_t>(dev.state()));
}

void test_soft_reset_calibration_read_failure_marks_dirty() {
  FakeBus bus;
  BME280::BME280 dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());
  Calibration before{};
  TEST_ASSERT_TRUE(dev.getCalibration(before).ok());

  setBoschSyntheticCalibration(bus);

  bus.failReadRegEnabled = true;
  bus.failReadReg = cmd::REG_CALIB_H_START;
  bus.readError = Status::Error(Err::I2C_BUS, "reset calibration bus error", -83);
  Status st = dev.softReset();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_BUS),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_TRUE(dev.hardwareConfigDirty());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_BUS),
                          static_cast<uint8_t>(dev.hardwareConfigDirtyError().code));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::DEGRADED),
                          static_cast<uint8_t>(dev.state()));

  Calibration after{};
  TEST_ASSERT_TRUE(dev.getCalibration(after).ok());
  TEST_ASSERT_EQUAL_UINT16(before.digT1, after.digT1);
  TEST_ASSERT_EQUAL_INT16(before.digT2, after.digT2);
  TEST_ASSERT_EQUAL_UINT16(before.digP1, after.digP1);
}

void test_soft_reset_invalid_calibration_marks_dirty_and_records_health_failure() {
  FakeBus bus;
  BME280::BME280 dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  bus.reg[cmd::REG_DIG_T1_LSB] = 0;
  bus.reg[cmd::REG_DIG_T1_MSB] = 0;

  Status st = dev.softReset();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::CALIBRATION_INVALID),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_TRUE(dev.hardwareConfigDirty());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::CALIBRATION_INVALID),
                          static_cast<uint8_t>(dev.hardwareConfigDirtyError().code));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::CALIBRATION_INVALID),
                          static_cast<uint8_t>(dev.lastError().code));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::DEGRADED),
                          static_cast<uint8_t>(dev.state()));
}

void test_soft_reset_apply_config_failure_marks_dirty_and_preserves_error() {
  FakeBus bus;
  BME280::BME280 dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  bus.failWriteOnCall = bus.writeCalls + 2u;  // reset write succeeds, apply sleep write fails
  bus.writeError = Status::Error(Err::I2C_TIMEOUT, "reset apply timeout", -84);
  Status st = dev.softReset();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_TIMEOUT),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_TRUE(dev.hardwareConfigDirty());
  TEST_ASSERT_EQUAL_INT32(-84, dev.hardwareConfigDirtyError().detail);
}

void test_soft_reset_success_reloads_calibration_and_clears_dirty() {
  FakeBus bus;
  BME280::BME280 dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  bus.failWriteOnCall = bus.writeCalls + 3u;
  bus.writeError = Status::Error(Err::I2C_BUS, "make dirty before reset", -85);
  Status st = dev.setFilter(Filter::X2);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_BUS),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_TRUE(dev.hardwareConfigDirty());

  setBoschSyntheticCalibration(bus);
  st = dev.softReset();
  TEST_ASSERT_TRUE(st.ok());
  TEST_ASSERT_FALSE(dev.hardwareConfigDirty());

  Calibration calib{};
  TEST_ASSERT_TRUE(dev.getCalibration(calib).ok());
  TEST_ASSERT_EQUAL_UINT16(27504, calib.digT1);
  TEST_ASSERT_EQUAL_INT16(26435, calib.digT2);
  TEST_ASSERT_EQUAL_UINT16(36477, calib.digP1);
}

void test_soft_reset_success_invalidates_cached_samples() {
  FakeBus bus;
  setBoschSyntheticCalibration(bus);
  setRawSample(bus, 415148, 519888, 30000);
  BME280::BME280 dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  captureForcedSample(dev, bus);
  TEST_ASSERT_TRUE(dev.hasSample());

  Status st = dev.softReset();
  TEST_ASSERT_TRUE(st.ok());
  TEST_ASSERT_FALSE(dev.hasSample());
  TEST_ASSERT_FALSE(dev.measurementReady());
  TEST_ASSERT_EQUAL_UINT32(0u, dev.sampleTimestampMs());

  RawSample raw{};
  st = dev.getRawSample(raw);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::MEASUREMENT_NOT_READY),
                          static_cast<uint8_t>(st.code));
}

void test_recover_returns_busy_when_nvm_update_in_progress_without_calibration_read() {
  FakeBus bus;
  BME280::BME280 dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  setBoschSyntheticCalibration(bus);
  bus.imUpdateStatusReadsRemaining = 2;
  const uint32_t statusReadsBefore = bus.statusReadCalls;
  Status st = dev.recover();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::BUSY),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_FALSE(bus.calibrationReadWhileImUpdate);
  TEST_ASSERT_EQUAL_UINT32(statusReadsBefore + 1u, bus.statusReadCalls);

  Calibration calib{};
  TEST_ASSERT_TRUE(dev.getCalibration(calib).ok());
  TEST_ASSERT_NOT_EQUAL_UINT16(27504, calib.digT1);
}

void test_recover_nvm_transport_error_updates_health_and_preserves_error() {
  FakeBus bus;
  BME280::BME280 dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  bus.failReadReg = cmd::REG_STATUS;
  bus.failReadRegRemaining = 300;
  bus.readError = Status::Error(Err::I2C_TIMEOUT, "recover nvm timeout", -86);
  Status st = dev.recover();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_TIMEOUT),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_INT32(-86, st.detail);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_TIMEOUT),
                          static_cast<uint8_t>(dev.lastError().code));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::DEGRADED),
                          static_cast<uint8_t>(dev.state()));
}

void test_recover_invalid_calibration_records_health_failure() {
  FakeBus bus;
  BME280::BME280 dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  bus.reg[cmd::REG_DIG_T1_LSB] = 0;
  bus.reg[cmd::REG_DIG_T1_MSB] = 0;

  Status st = dev.recover();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::CALIBRATION_INVALID),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::CALIBRATION_INVALID),
                          static_cast<uint8_t>(dev.lastError().code));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::DEGRADED),
                          static_cast<uint8_t>(dev.state()));
}

void test_set_mode_forced_does_not_trigger_conversion() {
  FakeBus bus;
  BME280::BME280 dev;
  Config cfg = makeConfig(bus);
  cfg.mode = Mode::NORMAL;
  TEST_ASSERT_TRUE(dev.begin(cfg).ok());

  Status st = dev.setMode(Mode::FORCED);
  TEST_ASSERT_TRUE(st.ok());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Mode::SLEEP),
                          bus.reg[cmd::REG_CTRL_MEAS] & cmd::MASK_CTRL_MEAS_MODE);
  TEST_ASSERT_FALSE(dev.measurementReady());
}

void test_example_transport_maps_wire_errors_and_keeps_timeout_owned_by_init() {
  Wire._clearEndTransmissionResult();
  Wire._clearRequestFromOverride();

  TEST_ASSERT_TRUE(transport::initWire(8, 9, 400000, 77));
  TEST_ASSERT_EQUAL_UINT32(77u, Wire.getTimeOut());

  const uint8_t byte = 0x55;

  Wire._setEndTransmissionResult(2);
  Status st = transport::wireWrite(0x76, &byte, 1, 123, &Wire);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_NACK_ADDR),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_UINT32(77u, Wire.getTimeOut());

  Wire._setEndTransmissionResult(3);
  st = transport::wireWrite(0x76, &byte, 1, 999, &Wire);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_NACK_DATA),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_UINT32(77u, Wire.getTimeOut());

  Wire._setEndTransmissionResult(4);
  st = transport::wireWrite(0x76, &byte, 1, 999, &Wire);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_BUS),
                          static_cast<uint8_t>(st.code));

  Wire._setEndTransmissionResult(5);
  st = transport::wireWrite(0x76, &byte, 1, 999, &Wire);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_TIMEOUT),
                          static_cast<uint8_t>(st.code));

  Wire._setEndTransmissionResult(1);
  st = transport::wireWrite(0x76, &byte, 1, 999, &Wire);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_PARAM),
                          static_cast<uint8_t>(st.code));
}

void test_example_transport_validates_params_and_handles_write_read() {
  const uint8_t tx = 0x00;
  uint8_t rx = 0;

  Status st = transport::wireWrite(0x76, nullptr, 1, 50, nullptr);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_CONFIG),
                          static_cast<uint8_t>(st.code));

  st = transport::wireWrite(0x76, &tx, 0, 50, &Wire);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_PARAM),
                          static_cast<uint8_t>(st.code));

  st = transport::wireWriteRead(0x76, nullptr, 1, &rx, 1, 50, &Wire);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_PARAM),
                          static_cast<uint8_t>(st.code));

  st = transport::wireWriteRead(0x76, &tx, 1, nullptr, 1, 50, &Wire);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_PARAM),
                          static_cast<uint8_t>(st.code));

  Wire._setEndTransmissionResult(0);
  Wire._setRequestFromResult(1);
  st = transport::wireWriteRead(0x76, &tx, 1, &rx, 1, 50, &Wire);
  TEST_ASSERT_TRUE(st.ok());

  Wire._setRequestFromResult(0);
  st = transport::wireWriteRead(0x76, &tx, 1, &rx, 1, 50, &Wire);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_ERROR),
                          static_cast<uint8_t>(st.code));
}

void test_recover_reaches_offline_when_threshold_is_one() {
  FakeBus bus;
  BME280::BME280 dev;
  Config cfg = makeConfig(bus);
  cfg.offlineThreshold = 1;
  TEST_ASSERT_TRUE(dev.begin(cfg).ok());

  bus.readErrorRemaining = 1;
  bus.readError = Status::Error(Err::I2C_ERROR, "forced timeout", -10);
  Status st = dev.recover();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_ERROR), static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::OFFLINE),
                          static_cast<uint8_t>(dev.state()));
  TEST_ASSERT_FALSE(dev.isOnline());
}

void test_offline_latches_public_register_read_without_i2c() {
  FakeBus bus;
  BME280::BME280 dev;
  Config cfg = makeConfig(bus);
  cfg.offlineThreshold = 1;
  TEST_ASSERT_TRUE(dev.begin(cfg).ok());

  bus.readErrorRemaining = 1;
  bus.readError = Status::Error(Err::I2C_ERROR, "forced offline", -11);
  Status st = dev.recover();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_ERROR), static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::OFFLINE),
                          static_cast<uint8_t>(dev.state()));

  const uint32_t readsBefore = bus.readCalls;
  uint8_t value = 0;
  st = dev.readRegister(cmd::REG_CHIP_ID, value);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::BUSY), static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_STRING("Driver is offline; call recover()", st.msg);
  TEST_ASSERT_EQUAL_UINT32(readsBefore, bus.readCalls);
}

void test_offline_typed_config_setters_do_not_mark_dirty_without_i2c() {
  FakeBus bus;
  BME280::BME280 dev;
  Config cfg = makeConfig(bus);
  cfg.offlineThreshold = 1;
  TEST_ASSERT_TRUE(dev.begin(cfg).ok());

  bus.readErrorRemaining = 1;
  bus.readError = Status::Error(Err::I2C_ERROR, "forced offline", -111);
  Status st = dev.recover();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_ERROR), static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::OFFLINE),
                          static_cast<uint8_t>(dev.state()));
  TEST_ASSERT_FALSE(dev.hardwareConfigDirty());

  const uint32_t callsBefore = totalBusCalls(bus);
  st = dev.setMode(Mode::SLEEP);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::BUSY), static_cast<uint8_t>(st.code));
  st = dev.setOversamplingT(Oversampling::X2);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::BUSY), static_cast<uint8_t>(st.code));
  st = dev.setOversamplingP(Oversampling::X2);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::BUSY), static_cast<uint8_t>(st.code));
  st = dev.setOversamplingH(Oversampling::X2);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::BUSY), static_cast<uint8_t>(st.code));

  TEST_ASSERT_EQUAL_UINT32(callsBefore, totalBusCalls(bus));
  TEST_ASSERT_FALSE(dev.hardwareConfigDirty());
}

void test_probe_works_while_offline_without_clearing_latch() {
  FakeBus bus;
  BME280::BME280 dev;
  Config cfg = makeConfig(bus);
  cfg.offlineThreshold = 1;
  TEST_ASSERT_TRUE(dev.begin(cfg).ok());

  bus.readErrorRemaining = 1;
  bus.readError = Status::Error(Err::I2C_ERROR, "forced offline", -87);
  Status st = dev.recover();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::OFFLINE),
                          static_cast<uint8_t>(dev.state()));

  const uint32_t failuresBefore = dev.totalFailures();
  st = dev.probe();
  TEST_ASSERT_TRUE(st.ok());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::OFFLINE),
                          static_cast<uint8_t>(dev.state()));
  TEST_ASSERT_EQUAL_UINT32(failuresBefore, dev.totalFailures());
}

void test_successful_recover_from_offline_clears_latch_and_allows_i2c() {
  FakeBus bus;
  BME280::BME280 dev;
  Config cfg = makeConfig(bus);
  cfg.offlineThreshold = 1;
  TEST_ASSERT_TRUE(dev.begin(cfg).ok());

  bus.readErrorRemaining = 1;
  bus.readError = Status::Error(Err::I2C_ERROR, "forced offline", -88);
  Status st = dev.recover();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::OFFLINE),
                          static_cast<uint8_t>(dev.state()));

  st = dev.recover();
  TEST_ASSERT_TRUE(st.ok());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::READY),
                          static_cast<uint8_t>(dev.state()));
  TEST_ASSERT_EQUAL_UINT8(0u, dev.consecutiveFailures());

  const uint32_t readsBefore = bus.readCalls;
  uint8_t value = 0;
  st = dev.readRegister(cmd::REG_CHIP_ID, value);
  TEST_ASSERT_TRUE(st.ok());
  TEST_ASSERT_GREATER_THAN_UINT32(readsBefore, bus.readCalls);
}

void test_failed_recover_from_offline_keeps_latch_after_intermediate_success() {
  FakeBus bus;
  BME280::BME280 dev;
  Config cfg = makeConfig(bus);
  cfg.offlineThreshold = 3;
  TEST_ASSERT_TRUE(dev.begin(cfg).ok());

  for (uint8_t i = 0; i < cfg.offlineThreshold; ++i) {
    bus.readErrorRemaining = 1;
    bus.readError = Status::Error(Err::I2C_ERROR, "forced offline", -12);
    (void)dev.recover();
  }
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::OFFLINE),
                          static_cast<uint8_t>(dev.state()));

  bus.writeErrorRemaining = 1;
  bus.writeError = Status::Error(Err::I2C_ERROR, "recover apply failed", -13);
  Status st = dev.recover();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_ERROR),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::OFFLINE),
                          static_cast<uint8_t>(dev.state()));
  TEST_ASSERT_TRUE(dev.consecutiveFailures() >= cfg.offlineThreshold);

  const uint32_t readsBefore = bus.readCalls;
  uint8_t value = 0;
  st = dev.readRegister(cmd::REG_CHIP_ID, value);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::BUSY),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_STRING("Driver is offline; call recover()", st.msg);
  TEST_ASSERT_EQUAL_UINT32(readsBefore, bus.readCalls);
}

void test_recovery_job_from_offline_clears_latch_and_allows_i2c() {
  FakeBus bus;
  BME280::BME280 dev;
  Config cfg = makeConfig(bus);
  cfg.offlineThreshold = 1;
  TEST_ASSERT_TRUE(dev.begin(cfg).ok());

  bus.readErrorRemaining = 1;
  bus.readError = Status::Error(Err::I2C_ERROR, "force staged offline", -120);
  Status st = dev.recover();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::OFFLINE),
                          static_cast<uint8_t>(dev.state()));

  const uint32_t callsBefore = totalBusCalls(bus);
  st = dev.startRecoveryJob();
  TEST_ASSERT_TRUE(st.inProgress());
  const JobPollResult result = pollUntilTerminal(dev, bus, 4);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(JobState::DONE),
                          static_cast<uint8_t>(result.state));
  TEST_ASSERT_TRUE(result.status.ok());
  TEST_ASSERT_GREATER_THAN_UINT32(callsBefore, totalBusCalls(bus));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::READY),
                          static_cast<uint8_t>(dev.state()));
  TEST_ASSERT_EQUAL_UINT8(0u, dev.consecutiveFailures());
  TEST_ASSERT_FALSE(dev.hardwareConfigDirty());
}

void test_recovery_job_from_offline_failure_reasserts_offline_latch() {
  FakeBus bus;
  BME280::BME280 dev;
  Config cfg = makeConfig(bus);
  cfg.offlineThreshold = 1;
  TEST_ASSERT_TRUE(dev.begin(cfg).ok());

  bus.readErrorRemaining = 1;
  bus.readError = Status::Error(Err::I2C_ERROR, "force staged offline", -121);
  (void)dev.recover();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::OFFLINE),
                          static_cast<uint8_t>(dev.state()));

  bus.failReadRegEnabled = true;
  bus.failReadReg = cmd::REG_CHIP_ID;
  bus.readError = Status::Error(Err::I2C_TIMEOUT, "recovery chip timeout", -122);
  Status st = dev.startRecoveryJob();
  TEST_ASSERT_TRUE(st.inProgress());
  const JobPollResult result = pollUntilTerminal(dev, bus, 4);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(JobState::FAILED),
                          static_cast<uint8_t>(result.state));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_TIMEOUT),
                          static_cast<uint8_t>(result.status.code));
  TEST_ASSERT_EQUAL_INT32(-122, result.status.detail);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::OFFLINE),
                          static_cast<uint8_t>(dev.state()));
  TEST_ASSERT_TRUE(dev.consecutiveFailures() >= cfg.offlineThreshold);
}

void test_recovery_job_from_offline_does_not_report_ready_until_complete() {
  FakeBus bus;
  BME280::BME280 dev;
  Config cfg = makeConfig(bus);
  cfg.offlineThreshold = 1;
  TEST_ASSERT_TRUE(dev.begin(cfg).ok());

  bus.readErrorRemaining = 1;
  bus.readError = Status::Error(Err::I2C_ERROR, "force staged offline", -125);
  (void)dev.recover();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::OFFLINE),
                          static_cast<uint8_t>(dev.state()));

  TEST_ASSERT_TRUE(dev.startRecoveryJob().inProgress());
  JobPollResult result = pollWithBudget(dev, bus, 1);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(JobState::RUNNING),
                          static_cast<uint8_t>(result.state));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::OFFLINE),
                          static_cast<uint8_t>(dev.state()));

  result = pollWithBudget(dev, bus, 1);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(JobState::RUNNING),
                          static_cast<uint8_t>(result.state));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::OFFLINE),
                          static_cast<uint8_t>(dev.state()));

  const uint32_t readsBefore = bus.readCalls;
  uint8_t value = 0;
  Status st = dev.readRegister(cmd::REG_CHIP_ID, value);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::BUSY),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_UINT32(readsBefore, bus.readCalls);

  result = pollUntilTerminal(dev, bus, 4);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(JobState::DONE),
                          static_cast<uint8_t>(result.state));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::READY),
                          static_cast<uint8_t>(dev.state()));
}

void test_non_recovery_job_still_blocked_while_offline() {
  FakeBus bus;
  BME280::BME280 dev;
  Config cfg = makeConfig(bus);
  cfg.offlineThreshold = 1;
  TEST_ASSERT_TRUE(dev.begin(cfg).ok());

  bus.readErrorRemaining = 1;
  bus.readError = Status::Error(Err::I2C_ERROR, "force staged offline", -123);
  (void)dev.recover();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::OFFLINE),
                          static_cast<uint8_t>(dev.state()));

  const uint32_t callsBefore = totalBusCalls(bus);
  Status st = dev.startForcedMeasurementJob();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::BUSY),
                          static_cast<uint8_t>(st.code));
  st = dev.startApplyConfigJob();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::BUSY),
                          static_cast<uint8_t>(st.code));
  st = dev.requestMeasurement();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::BUSY),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_UINT32(callsBefore, totalBusCalls(bus));
}

void test_forced_measurement_timing_wraparound_reaches_ready() {
  FakeBus bus;
  BME280::BME280 dev;
  Config cfg = makeConfig(bus);
  cfg.mode = Mode::FORCED;
  TEST_ASSERT_TRUE(dev.begin(cfg).ok());

  bus.nowMs = 0xFFFFFFF8u;
  Status st = dev.requestMeasurement();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::IN_PROGRESS),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_FALSE(dev.measurementReady());

  // Elapsed time across uint32 wrap should still satisfy forced-mode deadline.
  bus.nowMs = 20u;
  dev.tick(bus.nowMs);
  TEST_ASSERT_TRUE(dev.measurementReady());
  TEST_ASSERT_TRUE(dev.hasSample());
  TEST_ASSERT_EQUAL_UINT32(bus.nowMs, dev.sampleTimestampMs());
  TEST_ASSERT_EQUAL_UINT32(0u, dev.sampleAgeMs(bus.nowMs));
  TEST_ASSERT_EQUAL_UINT32(25u, dev.sampleAgeMs(bus.nowMs + 25u));

  Measurement m{};
  st = dev.getMeasurement(m);
  TEST_ASSERT_TRUE(st.ok());
}

void test_sample_freshness_none_before_capture() {
  FakeBus bus;
  BME280::BME280 dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  TEST_ASSERT_FALSE(dev.hasSample());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(SampleFreshness::NONE),
                          static_cast<uint8_t>(dev.sampleFreshness()));
  TEST_ASSERT_FALSE(dev.sampleFresh(bus.nowMs, 1000));

  SettingsSnapshot snap{};
  TEST_ASSERT_TRUE(dev.getSettings(snap).ok());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(SampleFreshness::NONE),
                          static_cast<uint8_t>(snap.sampleFreshness));
}

void test_sample_freshness_fresh_after_successful_capture() {
  FakeBus bus;
  setBoschSyntheticCalibration(bus);
  setRawSample(bus, 415148, 519888, 30000);
  BME280::BME280 dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  captureForcedSample(dev, bus);
  TEST_ASSERT_TRUE(dev.hasSample());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(SampleFreshness::FRESH),
                          static_cast<uint8_t>(dev.sampleFreshness()));
  TEST_ASSERT_TRUE(dev.sampleFresh(bus.nowMs, 0));
  TEST_ASSERT_TRUE(dev.sampleFresh(bus.nowMs + 10u, 10));
  TEST_ASSERT_FALSE(dev.sampleFresh(bus.nowMs + 11u, 10));
}

void test_sample_freshness_stale_after_failed_refresh() {
  FakeBus bus;
  setBoschSyntheticCalibration(bus);
  setRawSample(bus, 415148, 519888, 30000);
  BME280::BME280 dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  captureForcedSample(dev, bus);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(SampleFreshness::FRESH),
                          static_cast<uint8_t>(dev.sampleFreshness()));

  bus.failReadRegEnabled = true;
  bus.failReadReg = cmd::REG_STATUS;
  bus.readError = Status::Error(Err::I2C_TIMEOUT, "refresh status timeout", -70);
  Status st = dev.requestMeasurement();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_TIMEOUT),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_TRUE(dev.hasSample());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(SampleFreshness::STALE_AFTER_ERROR),
                          static_cast<uint8_t>(dev.sampleFreshness()));
  TEST_ASSERT_FALSE(dev.sampleFresh(bus.nowMs, 1000));
}

void test_sample_freshness_stale_when_hardware_config_dirty() {
  FakeBus bus;
  setBoschSyntheticCalibration(bus);
  setRawSample(bus, 415148, 519888, 30000);
  BME280::BME280 dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  captureForcedSample(dev, bus);
  bus.failWriteOnCall = bus.writeCalls + 3u;
  bus.writeError = Status::Error(Err::I2C_BUS, "restore bus error", -71);
  Status st = dev.setFilter(Filter::X2);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_BUS),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_TRUE(dev.hasSample());
  TEST_ASSERT_TRUE(dev.hardwareConfigDirty());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(SampleFreshness::STALE_AFTER_CONFIG_DIRTY),
                          static_cast<uint8_t>(dev.sampleFreshness()));
  TEST_ASSERT_FALSE(dev.sampleFresh(bus.nowMs, 1000));

  SettingsSnapshot snap{};
  TEST_ASSERT_TRUE(dev.getSettings(snap).ok());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(SampleFreshness::STALE_AFTER_CONFIG_DIRTY),
                          static_cast<uint8_t>(snap.sampleFreshness));
}

void test_sample_fresh_uses_wrap_safe_age_check() {
  FakeBus bus;
  setBoschSyntheticCalibration(bus);
  setRawSample(bus, 415148, 519888, 30000);
  bus.nowMs = 0xFFFFFFF0u;
  BME280::BME280 dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  captureForcedSample(dev, bus);
  const uint32_t sampleTs = dev.sampleTimestampMs();
  TEST_ASSERT_TRUE(dev.sampleFresh(sampleTs + 10u, 10));
  TEST_ASSERT_FALSE(dev.sampleFresh(sampleTs + 11u, 10));
}

void test_normal_mode_request_waits_for_fresh_cycle() {
  FakeBus bus;
  BME280::BME280 dev;
  Config cfg = makeConfig(bus);
  cfg.mode = Mode::NORMAL;
  cfg.standby = Standby::MS_125;
  TEST_ASSERT_TRUE(dev.begin(cfg).ok());

  Status st = dev.requestMeasurement();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::IN_PROGRESS),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_FALSE(dev.measurementReady());

  const uint32_t readsAfterRequest = bus.readCalls;
  dev.tick(bus.nowMs);
  TEST_ASSERT_FALSE(dev.measurementReady());
  TEST_ASSERT_EQUAL_UINT32(readsAfterRequest, bus.readCalls);

  bus.nowMs += dev.estimateNormalCycleMs() - 1U;
  dev.tick(bus.nowMs);
  TEST_ASSERT_FALSE(dev.measurementReady());
  TEST_ASSERT_EQUAL_UINT32(readsAfterRequest, bus.readCalls);

  bus.nowMs += 1U;
  dev.tick(bus.nowMs);
  TEST_ASSERT_TRUE(dev.measurementReady());
  TEST_ASSERT_GREATER_THAN_UINT32(readsAfterRequest, bus.readCalls);
}

void test_forced_measurement_request_while_busy_tracks_completion() {
  FakeBus bus;
  BME280::BME280 dev;
  Config cfg = makeConfig(bus);
  cfg.mode = Mode::FORCED;
  TEST_ASSERT_TRUE(dev.begin(cfg).ok());

  // Simulate an in-flight forced conversion (e.g., started by prior config write).
  bus.measuringStatusReadsRemaining = 1;
  Status st = dev.requestMeasurement();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::IN_PROGRESS),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_FALSE(dev.measurementReady());

  bus.nowMs += dev.estimateMeasurementTimeMs();
  dev.tick(bus.nowMs);
  TEST_ASSERT_TRUE(dev.measurementReady());

  Measurement m{};
  st = dev.getMeasurement(m);
  TEST_ASSERT_TRUE(st.ok());
}

void test_tick_raw_read_failure_records_measurement_status() {
  FakeBus bus;
  BME280::BME280 dev;
  Config cfg = makeConfig(bus);
  cfg.mode = Mode::FORCED;
  TEST_ASSERT_TRUE(dev.begin(cfg).ok());

  Status st = dev.requestMeasurement();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::IN_PROGRESS),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::IN_PROGRESS),
                          static_cast<uint8_t>(dev.lastMeasurementStatus().code));

  bus.failReadRegEnabled = true;
  bus.failReadReg = cmd::REG_DATA_START;
  bus.readError = Status::Error(Err::I2C_NACK_DATA, "forced raw burst nack", -55);
  bus.nowMs += dev.estimateMeasurementTimeMs();
  dev.tick(bus.nowMs);

  TEST_ASSERT_FALSE(dev.measurementReady());
  TEST_ASSERT_FALSE(dev.hasSample());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_NACK_DATA),
                          static_cast<uint8_t>(dev.lastMeasurementStatus().code));
  TEST_ASSERT_EQUAL_INT32(-55, dev.lastMeasurementStatus().detail);

  SettingsSnapshot snap{};
  TEST_ASSERT_TRUE(dev.getSettings(snap).ok());
  TEST_ASSERT_TRUE(snap.measurementRequested);
  TEST_ASSERT_FALSE(snap.hasSample);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_NACK_DATA),
                          static_cast<uint8_t>(snap.lastMeasurementStatus.code));
}

void test_tick_stuck_measuring_times_out_without_raw_read() {
  FakeBus bus;
  BME280::BME280 dev;
  Config cfg = makeConfig(bus);
  cfg.mode = Mode::FORCED;
  cfg.i2cTimeoutMs = 10;
  TEST_ASSERT_TRUE(dev.begin(cfg).ok());

  Status st = dev.requestMeasurement();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::IN_PROGRESS),
                          static_cast<uint8_t>(st.code));

  bus.reg[cmd::REG_STATUS] = cmd::MASK_STATUS_MEASURING;
  bus.nowMs += dev.estimateMeasurementTimeMs() + cfg.i2cTimeoutMs + 1U;
  dev.tick(bus.nowMs);

  TEST_ASSERT_FALSE(dev.measurementReady());
  TEST_ASSERT_FALSE(dev.hasSample());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::TIMEOUT),
                          static_cast<uint8_t>(dev.lastMeasurementStatus().code));
  TEST_ASSERT_EQUAL_HEX8(cmd::REG_STATUS, bus.lastReadReg);

  SettingsSnapshot snap{};
  TEST_ASSERT_TRUE(dev.getSettings(snap).ok());
  TEST_ASSERT_FALSE(snap.measurementRequested);
  TEST_ASSERT_FALSE(snap.measurementReady);
}

void test_set_mode_sleep_cancels_pending_measurement_request() {
  FakeBus bus;
  BME280::BME280 dev;
  Config cfg = makeConfig(bus);
  cfg.mode = Mode::FORCED;
  TEST_ASSERT_TRUE(dev.begin(cfg).ok());

  Status st = dev.requestMeasurement();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::IN_PROGRESS),
                          static_cast<uint8_t>(st.code));

  SettingsSnapshot snap{};
  TEST_ASSERT_TRUE(dev.getSettings(snap).ok());
  TEST_ASSERT_TRUE(snap.measurementRequested);
  TEST_ASSERT_FALSE(snap.measurementReady);

  st = dev.setMode(Mode::SLEEP);
  TEST_ASSERT_TRUE(st.ok());

  TEST_ASSERT_TRUE(dev.getSettings(snap).ok());
  TEST_ASSERT_FALSE(snap.measurementRequested);
  TEST_ASSERT_FALSE(snap.measurementReady);

  st = dev.setMode(Mode::FORCED);
  TEST_ASSERT_TRUE(st.ok());
  st = dev.requestMeasurement();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::IN_PROGRESS),
                          static_cast<uint8_t>(st.code));
}

void test_raw_and_compensated_samples_remain_available_after_measurement_read() {
  FakeBus bus;
  BME280::BME280 dev;
  Config cfg = makeConfig(bus);
  cfg.mode = Mode::FORCED;
  TEST_ASSERT_TRUE(dev.begin(cfg).ok());

  RawSample raw{};
  CompensatedSample comp{};

  Status st = dev.getRawSample(raw);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::MEASUREMENT_NOT_READY),
                          static_cast<uint8_t>(st.code));
  st = dev.getCompensatedSample(comp);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::MEASUREMENT_NOT_READY),
                          static_cast<uint8_t>(st.code));

  st = dev.requestMeasurement();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::IN_PROGRESS),
                          static_cast<uint8_t>(st.code));
  bus.nowMs += dev.estimateMeasurementTimeMs();
  dev.tick(bus.nowMs);
  TEST_ASSERT_TRUE(dev.measurementReady());

  Measurement m{};
  st = dev.getMeasurement(m);
  TEST_ASSERT_TRUE(st.ok());
  TEST_ASSERT_FALSE(dev.measurementReady());

  st = dev.getRawSample(raw);
  TEST_ASSERT_TRUE(st.ok());
  st = dev.getCompensatedSample(comp);
  TEST_ASSERT_TRUE(st.ok());
}

void test_calibration_parses_bosch_synthetic_coefficients() {
  FakeBus bus;
  setBoschSyntheticCalibration(bus);
  BME280::BME280 dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  Calibration calib{};
  TEST_ASSERT_TRUE(dev.getCalibration(calib).ok());
  TEST_ASSERT_EQUAL_UINT16(27504, calib.digT1);
  TEST_ASSERT_EQUAL_INT16(26435, calib.digT2);
  TEST_ASSERT_EQUAL_INT16(-1000, calib.digT3);
  TEST_ASSERT_EQUAL_UINT16(36477, calib.digP1);
  TEST_ASSERT_EQUAL_INT16(-10685, calib.digP2);
  TEST_ASSERT_EQUAL_INT16(3024, calib.digP3);
  TEST_ASSERT_EQUAL_INT16(2855, calib.digP4);
  TEST_ASSERT_EQUAL_INT16(140, calib.digP5);
  TEST_ASSERT_EQUAL_INT16(-7, calib.digP6);
  TEST_ASSERT_EQUAL_INT16(15500, calib.digP7);
  TEST_ASSERT_EQUAL_INT16(-14600, calib.digP8);
  TEST_ASSERT_EQUAL_INT16(6000, calib.digP9);
  TEST_ASSERT_EQUAL_UINT8(75, calib.digH1);
  TEST_ASSERT_EQUAL_INT16(362, calib.digH2);
  TEST_ASSERT_EQUAL_UINT8(0, calib.digH3);
  TEST_ASSERT_EQUAL_INT16(325, calib.digH4);
  TEST_ASSERT_EQUAL_INT16(50, calib.digH5);
  TEST_ASSERT_EQUAL_INT8(30, calib.digH6);
}

void test_calibration_parses_signed_boundaries_and_humidity_nibbles() {
  FakeBus bus;
  putLe16(bus, cmd::REG_DIG_T1_LSB, 1);
  putS16(bus, cmd::REG_DIG_T2_LSB, -32768);
  putS16(bus, cmd::REG_DIG_T3_LSB, 32767);
  putLe16(bus, cmd::REG_DIG_P1_LSB, 2);
  putS16(bus, cmd::REG_DIG_P2_LSB, -32768);
  putS16(bus, cmd::REG_DIG_P3_LSB, -1);
  putS16(bus, cmd::REG_DIG_P4_LSB, 0);
  putS16(bus, cmd::REG_DIG_P5_LSB, 32767);
  putS16(bus, cmd::REG_DIG_P6_LSB, -123);
  putS16(bus, cmd::REG_DIG_P7_LSB, 123);
  putS16(bus, cmd::REG_DIG_P8_LSB, -32768);
  putS16(bus, cmd::REG_DIG_P9_LSB, 32767);
  bus.reg[cmd::REG_DIG_H1] = 255;
  putS16(bus, cmd::REG_DIG_H2_LSB, -32768);
  bus.reg[cmd::REG_DIG_H3] = 255;
  setH4H5(bus, 2047, -2048);
  bus.reg[cmd::REG_DIG_H6] = static_cast<uint8_t>(-128);

  BME280::BME280 dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  Calibration calib{};
  TEST_ASSERT_TRUE(dev.getCalibration(calib).ok());
  TEST_ASSERT_EQUAL_INT16(-32768, calib.digT2);
  TEST_ASSERT_EQUAL_INT16(32767, calib.digT3);
  TEST_ASSERT_EQUAL_INT16(-32768, calib.digP2);
  TEST_ASSERT_EQUAL_INT16(-1, calib.digP3);
  TEST_ASSERT_EQUAL_INT16(32767, calib.digP5);
  TEST_ASSERT_EQUAL_INT16(-32768, calib.digP8);
  TEST_ASSERT_EQUAL_UINT8(255, calib.digH1);
  TEST_ASSERT_EQUAL_INT16(-32768, calib.digH2);
  TEST_ASSERT_EQUAL_UINT8(255, calib.digH3);
  TEST_ASSERT_EQUAL_INT16(2047, calib.digH4);
  TEST_ASSERT_EQUAL_INT16(-2048, calib.digH5);
  TEST_ASSERT_EQUAL_INT8(-128, calib.digH6);
}

void test_invalid_pressure_calibration_is_rejected() {
  FakeBus busZero;
  setBoschSyntheticCalibration(busZero);
  putLe16(busZero, cmd::REG_DIG_P1_LSB, 0);
  BME280::BME280 zeroDev;
  Status st = zeroDev.begin(makeConfig(busZero));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::CALIBRATION_INVALID),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_FALSE(zeroDev.isInitialized());

  FakeBus busFfff;
  setBoschSyntheticCalibration(busFfff);
  putLe16(busFfff, cmd::REG_DIG_P1_LSB, 0xFFFF);
  BME280::BME280 ffffDev;
  st = ffffDev.begin(makeConfig(busFfff));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::CALIBRATION_INVALID),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_FALSE(ffffDev.isInitialized());
}

void test_read_calibration_raw_uses_register_bytes_and_preserves_error() {
  FakeBus bus;
  setBoschSyntheticCalibration(bus);
  BME280::BME280 dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  CalibrationRaw raw{};
  TEST_ASSERT_TRUE(dev.readCalibrationRaw(raw).ok());
  TEST_ASSERT_EQUAL_UINT8(bus.reg[cmd::REG_DIG_T1_LSB],
                          raw.tp[cmd::REG_DIG_T1_LSB - cmd::REG_CALIB_TP_START]);
  TEST_ASSERT_EQUAL_UINT8(bus.reg[cmd::REG_DIG_P9_MSB],
                          raw.tp[cmd::REG_DIG_P9_MSB - cmd::REG_CALIB_TP_START]);
  TEST_ASSERT_EQUAL_UINT8(bus.reg[cmd::REG_DIG_H1], raw.h1);
  TEST_ASSERT_EQUAL_UINT8(bus.reg[cmd::REG_DIG_H4_MSB],
                          raw.h[cmd::REG_DIG_H4_MSB - cmd::REG_CALIB_H_START]);
  TEST_ASSERT_EQUAL_UINT8(bus.reg[cmd::REG_DIG_H4_H5],
                          raw.h[cmd::REG_DIG_H4_H5 - cmd::REG_CALIB_H_START]);
  TEST_ASSERT_EQUAL_UINT8(bus.reg[cmd::REG_DIG_H5_MSB],
                          raw.h[cmd::REG_DIG_H5_MSB - cmd::REG_CALIB_H_START]);

  bus.failReadRegEnabled = true;
  bus.failReadReg = cmd::REG_CALIB_H_START;
  bus.readError = Status::Error(Err::I2C_NACK_DATA, "forced h calib nack", -31);
  Status st = dev.readCalibrationRaw(raw);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_NACK_DATA),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_INT32(-31, st.detail);
}

void test_raw_burst_reconstructs_20_and_16_bit_samples() {
  FakeBus bus;
  setBoschSyntheticCalibration(bus);
  setRawBurstBytes(bus, 0xAB, 0xCD, 0xEF, 0x54, 0x32, 0x1F, 0xBE, 0xEF);
  BME280::BME280 dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  captureForcedSample(dev, bus);
  TEST_ASSERT_TRUE(dev.hasSample());
  TEST_ASSERT_EQUAL_UINT8(cmd::REG_DATA_START, bus.lastReadReg);
  TEST_ASSERT_EQUAL_UINT32(cmd::DATA_LEN, bus.lastReadLen);

  RawSample raw{};
  TEST_ASSERT_TRUE(dev.getRawSample(raw).ok());
  TEST_ASSERT_EQUAL_INT32(0x54321, raw.adcT);
  TEST_ASSERT_EQUAL_INT32(0xABCDE, raw.adcP);
  TEST_ASSERT_EQUAL_INT32(0xBEEF, raw.adcH);
  TEST_ASSERT_TRUE(raw.temperatureValid);
  TEST_ASSERT_TRUE(raw.pressureValid);
  TEST_ASSERT_TRUE(raw.humidityValid);
}

void test_compensation_matches_datasheet_derived_synthetic_vector() {
  FakeBus bus;
  setBoschSyntheticCalibration(bus);
  setRawSample(bus, 415148, 519888, 30000);
  BME280::BME280 dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  captureForcedSample(dev, bus);
  TEST_ASSERT_TRUE(dev.measurementReady());

  CompensatedSample comp{};
  TEST_ASSERT_TRUE(dev.getCompensatedSample(comp).ok());
  TEST_ASSERT_EQUAL_INT32(2508, comp.tempC_x100);
  TEST_ASSERT_EQUAL_UINT32(100653, comp.pressurePa);
  TEST_ASSERT_EQUAL_UINT32(51941, comp.humidityPct_x1024);
  TEST_ASSERT_TRUE(comp.temperatureValid);
  TEST_ASSERT_TRUE(comp.pressureValid);
  TEST_ASSERT_TRUE(comp.humidityValid);

  SettingsSnapshot snap{};
  TEST_ASSERT_TRUE(dev.getSettings(snap).ok());
  TEST_ASSERT_EQUAL_INT32(128422, snap.tFine);

  Measurement measurement{};
  TEST_ASSERT_TRUE(dev.getMeasurement(measurement).ok());
  TEST_ASSERT_TRUE(measurement.temperatureValid);
  TEST_ASSERT_TRUE(measurement.pressureValid);
  TEST_ASSERT_TRUE(measurement.humidityValid);
}

void test_humidity_compensation_clamps_low_and_high() {
  FakeBus bus;
  setBoschSyntheticCalibration(bus);
  BME280::BME280 dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  setRawSample(bus, 415148, 519888, 0);
  captureForcedSample(dev, bus);
  CompensatedSample comp{};
  TEST_ASSERT_TRUE(dev.getCompensatedSample(comp).ok());
  TEST_ASSERT_EQUAL_UINT32(0, comp.humidityPct_x1024);
  TEST_ASSERT_TRUE(comp.humidityValid);

  setRawSample(bus, 415148, 519888, 40000);
  captureForcedSample(dev, bus);
  TEST_ASSERT_TRUE(dev.getCompensatedSample(comp).ok());
  TEST_ASSERT_EQUAL_UINT32(102400, comp.humidityPct_x1024);
  TEST_ASSERT_TRUE(comp.humidityValid);
}

void test_skipped_sentinels_are_explicit_validity_flags() {
  FakeBus bus;
  setBoschSyntheticCalibration(bus);
  setRawSample(bus,
               cmd::RAW_PRESSURE_SKIPPED,
               519888,
               cmd::RAW_HUMIDITY_SKIPPED);
  Config cfg = makeConfig(bus);
  cfg.osrsP = Oversampling::SKIP;
  cfg.osrsH = Oversampling::SKIP;
  BME280::BME280 dev;
  TEST_ASSERT_TRUE(dev.begin(cfg).ok());

  captureForcedSample(dev, bus);
  TEST_ASSERT_TRUE(dev.hasSample());

  RawSample raw{};
  TEST_ASSERT_TRUE(dev.getRawSample(raw).ok());
  TEST_ASSERT_EQUAL_INT32(519888, raw.adcT);
  TEST_ASSERT_EQUAL_INT32(cmd::RAW_PRESSURE_SKIPPED, raw.adcP);
  TEST_ASSERT_EQUAL_INT32(cmd::RAW_HUMIDITY_SKIPPED, raw.adcH);
  TEST_ASSERT_TRUE(raw.temperatureValid);
  TEST_ASSERT_FALSE(raw.pressureValid);
  TEST_ASSERT_FALSE(raw.humidityValid);

  CompensatedSample comp{};
  TEST_ASSERT_TRUE(dev.getCompensatedSample(comp).ok());
  TEST_ASSERT_TRUE(comp.temperatureValid);
  TEST_ASSERT_FALSE(comp.pressureValid);
  TEST_ASSERT_FALSE(comp.humidityValid);
  TEST_ASSERT_EQUAL_UINT32(0, comp.pressurePa);
  TEST_ASSERT_EQUAL_UINT32(0, comp.humidityPct_x1024);

  Measurement measurement{};
  TEST_ASSERT_TRUE(dev.getMeasurement(measurement).ok());
  TEST_ASSERT_TRUE(measurement.temperatureValid);
  TEST_ASSERT_FALSE(measurement.pressureValid);
  TEST_ASSERT_FALSE(measurement.humidityValid);
}

void test_forced_measurement_job_budget_and_raw_fixed_outputs() {
  FakeBus bus;
  setBoschSyntheticCalibration(bus);
  setRawSample(bus, 415148, 519888, 30000);
  BME280::BME280 dev;
  Config cfg = makeConfig(bus);
  cfg.mode = Mode::FORCED;
  TEST_ASSERT_TRUE(dev.begin(cfg).ok());

  Status st = dev.startForcedMeasurementJob();
  TEST_ASSERT_TRUE(st.inProgress());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::IN_PROGRESS),
                          static_cast<uint8_t>(dev.lastMeasurementStatus().code));
  TEST_ASSERT_EQUAL_STRING("Measurement job started", dev.lastMeasurementStatus().msg);

  JobPollResult result = pollWithBudget(dev, bus, 1);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(JobState::RUNNING),
                          static_cast<uint8_t>(result.state));
  TEST_ASSERT_EQUAL_HEX8(cmd::REG_CTRL_HUM, bus.lastWriteReg);

  result = pollWithBudget(dev, bus, 1);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(JobState::WAITING),
                          static_cast<uint8_t>(result.state));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Mode::FORCED),
                          bus.reg[cmd::REG_CTRL_MEAS] & cmd::MASK_CTRL_MEAS_MODE);

  const uint32_t callsBeforeDelay = totalBusCalls(bus);
  result = dev.pollJob(bus.nowMs, 1);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(JobState::WAITING),
                          static_cast<uint8_t>(result.state));
  TEST_ASSERT_EQUAL_UINT8(0u, result.instructionsUsed);
  TEST_ASSERT_EQUAL_UINT32(callsBeforeDelay, totalBusCalls(bus));

  bus.nowMs += dev.estimateMeasurementTimeMs();
  result = pollWithBudget(dev, bus, 1);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(JobState::RUNNING),
                          static_cast<uint8_t>(result.state));
  TEST_ASSERT_EQUAL_HEX8(cmd::REG_STATUS, bus.lastReadReg);

  result = pollWithBudget(dev, bus, 1);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(JobState::DONE),
                          static_cast<uint8_t>(result.state));
  TEST_ASSERT_TRUE(result.status.ok());
  TEST_ASSERT_EQUAL_HEX8(cmd::REG_DATA_START, bus.lastReadReg);
  TEST_ASSERT_EQUAL_UINT32(static_cast<uint32_t>(cmd::DATA_LEN),
                           static_cast<uint32_t>(bus.lastReadLen));
  TEST_ASSERT_TRUE(dev.measurementReady());
  TEST_ASSERT_TRUE(dev.lastMeasurementStatus().ok());

  RawSample raw{};
  CompensatedSample comp{};
  st = dev.getRawSample(raw);
  TEST_ASSERT_TRUE(st.ok());
  st = dev.getCompensatedSample(comp);
  TEST_ASSERT_TRUE(st.ok());
  TEST_ASSERT_TRUE(comp.pressureValid);
  TEST_ASSERT_EQUAL_UINT32(100653, comp.pressurePa);
}

void test_forced_measurement_job_reports_in_progress_status() {
  FakeBus bus;
  setBoschSyntheticCalibration(bus);
  setRawSample(bus, 415148, 519888, 30000);
  BME280::BME280 dev;
  Config cfg = makeConfig(bus);
  cfg.mode = Mode::FORCED;
  TEST_ASSERT_TRUE(dev.begin(cfg).ok());

  Status st = dev.startForcedMeasurementJob();
  TEST_ASSERT_TRUE(st.inProgress());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::IN_PROGRESS),
                          static_cast<uint8_t>(dev.lastMeasurementStatus().code));
  TEST_ASSERT_EQUAL_STRING("Measurement job started", dev.lastMeasurementStatus().msg);

  (void)pollWithBudget(dev, bus, 1);
  const JobPollResult result = pollWithBudget(dev, bus, 1);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(JobState::WAITING),
                          static_cast<uint8_t>(result.state));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::IN_PROGRESS),
                          static_cast<uint8_t>(dev.lastMeasurementStatus().code));
  TEST_ASSERT_EQUAL_STRING("Measurement delay active", dev.lastMeasurementStatus().msg);
}

void test_forced_measurement_job_success_clears_last_measurement_status() {
  FakeBus bus;
  setBoschSyntheticCalibration(bus);
  setRawSample(bus, 415148, 519888, 30000);
  BME280::BME280 dev;
  Config cfg = makeConfig(bus);
  cfg.mode = Mode::FORCED;
  TEST_ASSERT_TRUE(dev.begin(cfg).ok());

  TEST_ASSERT_TRUE(dev.startForcedMeasurementJob().inProgress());
  (void)pollWithBudget(dev, bus, 1);
  (void)pollWithBudget(dev, bus, 1);
  bus.nowMs += dev.estimateMeasurementTimeMs();
  const JobPollResult result = pollUntilTerminal(dev, bus, 4);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(JobState::DONE),
                          static_cast<uint8_t>(result.state));
  TEST_ASSERT_TRUE(dev.lastMeasurementStatus().ok());
}

void test_forced_measurement_job_failure_preserves_last_measurement_status() {
  FakeBus bus;
  setBoschSyntheticCalibration(bus);
  setRawSample(bus, 415148, 519888, 30000);
  BME280::BME280 dev;
  Config cfg = makeConfig(bus);
  cfg.mode = Mode::FORCED;
  TEST_ASSERT_TRUE(dev.begin(cfg).ok());

  TEST_ASSERT_TRUE(dev.startForcedMeasurementJob().inProgress());
  (void)pollWithBudget(dev, bus, 1);
  (void)pollWithBudget(dev, bus, 1);

  bus.nowMs += dev.estimateMeasurementTimeMs();
  bus.failReadRegEnabled = true;
  bus.failReadReg = cmd::REG_DATA_START;
  bus.readError = Status::Error(Err::I2C_NACK_DATA, "forced raw nack", -124);
  const JobPollResult result = pollUntilTerminal(dev, bus, 4);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(JobState::FAILED),
                          static_cast<uint8_t>(result.state));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_NACK_DATA),
                          static_cast<uint8_t>(dev.lastMeasurementStatus().code));
  TEST_ASSERT_EQUAL_INT32(-124, dev.lastMeasurementStatus().detail);
}

void test_forced_measurement_job_measuring_status_does_not_read_raw_same_poll() {
  FakeBus bus;
  setBoschSyntheticCalibration(bus);
  setRawSample(bus, 415148, 519888, 30000);
  BME280::BME280 dev;
  Config cfg = makeConfig(bus);
  cfg.mode = Mode::FORCED;
  TEST_ASSERT_TRUE(dev.begin(cfg).ok());

  TEST_ASSERT_TRUE(dev.startForcedMeasurementJob().inProgress());
  (void)pollWithBudget(dev, bus, 1);
  (void)pollWithBudget(dev, bus, 1);

  bus.nowMs += dev.estimateMeasurementTimeMs();
  bus.measuringStatusReadsRemaining = 1;
  const uint32_t readsBefore = bus.readCalls;
  JobPollResult result = pollWithBudget(dev, bus, 4);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(JobState::WAITING),
                          static_cast<uint8_t>(result.state));
  TEST_ASSERT_EQUAL_UINT32(readsBefore + 1u, bus.readCalls);
  TEST_ASSERT_EQUAL_HEX8(cmd::REG_STATUS, bus.lastReadReg);

  result = pollUntilTerminal(dev, bus, 4);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(JobState::DONE),
                          static_cast<uint8_t>(result.state));
  TEST_ASSERT_EQUAL_HEX8(cmd::REG_DATA_START, bus.lastReadReg);
}

void test_forced_measurement_job_status_failure_clears_pending_state() {
  FakeBus bus;
  setBoschSyntheticCalibration(bus);
  setRawSample(bus, 415148, 519888, 30000);
  BME280::BME280 dev;
  Config cfg = makeConfig(bus);
  cfg.mode = Mode::FORCED;
  TEST_ASSERT_TRUE(dev.begin(cfg).ok());

  TEST_ASSERT_TRUE(dev.startForcedMeasurementJob().inProgress());
  (void)pollWithBudget(dev, bus, 1);
  (void)pollWithBudget(dev, bus, 1);

  bus.nowMs += dev.estimateMeasurementTimeMs();
  bus.failReadReg = cmd::REG_STATUS;
  bus.failReadRegRemaining = 1;
  bus.readError = Status::Error(Err::I2C_TIMEOUT, "forced status timeout", -95);
  JobPollResult result = pollWithBudget(dev, bus, 2);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(JobState::FAILED),
                          static_cast<uint8_t>(result.state));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_TIMEOUT),
                          static_cast<uint8_t>(result.status.code));

  SettingsSnapshot snap{};
  TEST_ASSERT_TRUE(dev.getSettings(snap).ok());
  TEST_ASSERT_FALSE(snap.measurementRequested);
  TEST_ASSERT_FALSE(snap.measurementReady);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_TIMEOUT),
                          static_cast<uint8_t>(snap.lastMeasurementStatus.code));

  TEST_ASSERT_TRUE(dev.startForcedMeasurementJob().inProgress());
}

void test_forced_measurement_job_stuck_measuring_times_out_without_raw_read() {
  FakeBus bus;
  setBoschSyntheticCalibration(bus);
  setRawSample(bus, 415148, 519888, 30000);
  BME280::BME280 dev;
  Config cfg = makeConfig(bus);
  cfg.mode = Mode::FORCED;
  cfg.i2cTimeoutMs = 10;
  TEST_ASSERT_TRUE(dev.begin(cfg).ok());

  TEST_ASSERT_TRUE(dev.startForcedMeasurementJob().inProgress());
  (void)pollWithBudget(dev, bus, 1);
  (void)pollWithBudget(dev, bus, 1);

  bus.nowMs += dev.estimateMeasurementTimeMs();
  bus.reg[cmd::REG_STATUS] = cmd::MASK_STATUS_MEASURING;
  JobPollResult result = pollWithBudget(dev, bus, 4);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(JobState::WAITING),
                          static_cast<uint8_t>(result.state));
  TEST_ASSERT_EQUAL_HEX8(cmd::REG_STATUS, bus.lastReadReg);

  bus.nowMs += cfg.i2cTimeoutMs + 1U;
  result = pollWithBudget(dev, bus, 4);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(JobState::FAILED),
                          static_cast<uint8_t>(result.state));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::TIMEOUT),
                          static_cast<uint8_t>(result.status.code));
  TEST_ASSERT_EQUAL_HEX8(cmd::REG_STATUS, bus.lastReadReg);
  TEST_ASSERT_FALSE(dev.measurementReady());

  SettingsSnapshot snap{};
  TEST_ASSERT_TRUE(dev.getSettings(snap).ok());
  TEST_ASSERT_FALSE(snap.measurementRequested);
  TEST_ASSERT_FALSE(snap.measurementReady);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::TIMEOUT),
                          static_cast<uint8_t>(snap.lastMeasurementStatus.code));
}

void test_recovery_job_budget_and_full_sequence() {
  FakeBus bus;
  BME280::BME280 dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  const uint32_t callsBefore = totalBusCalls(bus);
  Status st = dev.startRecoveryJob();
  TEST_ASSERT_TRUE(st.inProgress());

  uint32_t instructionsTotal = 0;
  JobPollResult result{};
  for (uint8_t poll = 0; poll < 16; ++poll) {
    result = pollWithBudget(dev, bus, 1);
    instructionsTotal += result.instructionsUsed;
    if (result.state == JobState::DONE || result.state == JobState::FAILED) {
      break;
    }
  }

  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(JobState::DONE),
                          static_cast<uint8_t>(result.state));
  TEST_ASSERT_TRUE(result.status.ok());
  TEST_ASSERT_EQUAL_UINT32(10u, instructionsTotal);
  TEST_ASSERT_EQUAL_UINT32(callsBefore + 10u, totalBusCalls(bus));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::READY),
                          static_cast<uint8_t>(dev.state()));
}

void test_recovery_job_chip_id_mismatch_records_health_failure() {
  FakeBus bus;
  BME280::BME280 dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  bus.chipId = 0x61;
  TEST_ASSERT_TRUE(dev.startRecoveryJob().inProgress());
  const JobPollResult result = pollUntilTerminal(dev, bus, 4);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(JobState::FAILED),
                          static_cast<uint8_t>(result.state));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::CHIP_ID_MISMATCH),
                          static_cast<uint8_t>(result.status.code));
  TEST_ASSERT_EQUAL_INT32(0x61, result.status.detail);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::CHIP_ID_MISMATCH),
                          static_cast<uint8_t>(dev.lastError().code));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::DEGRADED),
                          static_cast<uint8_t>(dev.state()));
  TEST_ASSERT_TRUE(dev.hardwareConfigDirty());
}

void test_recovery_job_nvm_busy_uses_one_status_read_per_poll() {
  FakeBus bus;
  BME280::BME280 dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());
  bus.reg[cmd::REG_STATUS] = cmd::MASK_STATUS_IM_UPDATE;

  const uint32_t statusReadsBefore = bus.statusReadCalls;
  TEST_ASSERT_TRUE(dev.startRecoveryJob().inProgress());
  JobPollResult result = pollWithBudget(dev, bus, 3);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(JobState::WAITING),
                          static_cast<uint8_t>(result.state));
  TEST_ASSERT_EQUAL_UINT32(statusReadsBefore + 1u, bus.statusReadCalls);

  result = pollWithBudget(dev, bus, 1);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(JobState::WAITING),
                          static_cast<uint8_t>(result.state));
  TEST_ASSERT_EQUAL_UINT32(statusReadsBefore + 2u, bus.statusReadCalls);
}

void test_recovery_job_nvm_timeout_records_health_failure() {
  FakeBus bus;
  BME280::BME280 dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());
  bus.reg[cmd::REG_STATUS] = cmd::MASK_STATUS_IM_UPDATE;
  bus.statusReadNowAdvanceMs = 11;

  Status st = dev.startRecoveryJob();
  TEST_ASSERT_TRUE(st.inProgress());

  JobPollResult result = pollWithBudget(dev, bus, 4);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(JobState::WAITING),
                          static_cast<uint8_t>(result.state));
  result = pollWithBudget(dev, bus, 4);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(JobState::FAILED),
                          static_cast<uint8_t>(result.state));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::TIMEOUT),
                          static_cast<uint8_t>(result.status.code));
  TEST_ASSERT_TRUE(dev.hardwareConfigDirty());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::TIMEOUT),
                          static_cast<uint8_t>(dev.lastError().code));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::DEGRADED),
                          static_cast<uint8_t>(dev.state()));
}

void test_recovery_job_failed_humidity_calibration_keeps_previous_coefficients() {
  FakeBus bus;
  BME280::BME280 dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  Calibration before{};
  TEST_ASSERT_TRUE(dev.getCalibration(before).ok());
  putLe16(bus, cmd::REG_DIG_T1_LSB, 0x4321);
  putLe16(bus, cmd::REG_DIG_P1_LSB, 0x5678);
  bus.failReadRegEnabled = true;
  bus.failReadReg = cmd::REG_CALIB_H_START;
  bus.readError = Status::Error(Err::I2C_BUS, "recovery humidity calibration bus", -94);

  TEST_ASSERT_TRUE(dev.startRecoveryJob().inProgress());
  const JobPollResult result = pollUntilTerminal(dev, bus, 4);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(JobState::FAILED),
                          static_cast<uint8_t>(result.state));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_BUS),
                          static_cast<uint8_t>(result.status.code));

  Calibration after{};
  TEST_ASSERT_TRUE(dev.getCalibration(after).ok());
  TEST_ASSERT_EQUAL_UINT16(before.digT1, after.digT1);
  TEST_ASSERT_EQUAL_UINT16(before.digP1, after.digP1);
  TEST_ASSERT_EQUAL_INT16(before.digT2, after.digT2);
  TEST_ASSERT_TRUE(dev.hardwareConfigDirty());
}

void test_recovery_job_error_stops_without_extra_instructions() {
  FakeBus bus;
  BME280::BME280 dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  bus.writeErrorRemaining = 1;
  bus.writeError = Status::Error(Err::I2C_BUS, "forced reset write error", -55);
  TEST_ASSERT_TRUE(dev.startRecoveryJob().inProgress());

  JobPollResult result = pollWithBudget(dev, bus, 4);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(JobState::FAILED),
                          static_cast<uint8_t>(result.state));
  TEST_ASSERT_EQUAL_UINT8(1u, result.instructionsUsed);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_BUS),
                          static_cast<uint8_t>(result.status.code));
  TEST_ASSERT_EQUAL_INT32(-55, result.status.detail);
}

void test_enabled_raw_sentinel_rejects_compensated_sample() {
  FakeBus bus;
  setBoschSyntheticCalibration(bus);
  setRawSample(bus, cmd::RAW_PRESSURE_SKIPPED, 519888, 30000);
  BME280::BME280 dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  captureForcedSample(dev, bus);
  TEST_ASSERT_FALSE(dev.measurementReady());
  TEST_ASSERT_FALSE(dev.hasSample());

  CompensatedSample comp{};
  Status st = dev.getCompensatedSample(comp);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::MEASUREMENT_NOT_READY),
                          static_cast<uint8_t>(st.code));
}

void test_pressure_compensation_divide_by_zero_guard_blocks_sample() {
  FakeBus bus;
  setPressureDenominatorZeroCalibration(bus);
  setRawSample(bus, 415148, 211674, cmd::RAW_HUMIDITY_SKIPPED);
  Config cfg = makeConfig(bus);
  cfg.osrsH = Oversampling::SKIP;
  BME280::BME280 dev;
  TEST_ASSERT_TRUE(dev.begin(cfg).ok());

  captureForcedSample(dev, bus);
  TEST_ASSERT_FALSE(dev.measurementReady());
  TEST_ASSERT_FALSE(dev.hasSample());
}

void test_config_change_invalidates_cached_samples() {
  FakeBus bus;
  setBoschSyntheticCalibration(bus);
  setRawSample(bus, 415148, 519888, 30000);
  BME280::BME280 dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  captureForcedSample(dev, bus);
  TEST_ASSERT_TRUE(dev.hasSample());
  TEST_ASSERT_TRUE(dev.setOversamplingP(Oversampling::SKIP).ok());
  TEST_ASSERT_FALSE(dev.hasSample());

  RawSample raw{};
  Status st = dev.getRawSample(raw);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::MEASUREMENT_NOT_READY),
                          static_cast<uint8_t>(st.code));
}

void test_raw_sentinel_failure_preserves_committed_sample_envelope() {
  FakeBus bus;
  setBoschSyntheticCalibration(bus);
  setRawSample(bus, 415148, 519888, 30000);
  BME280::BME280 dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  captureForcedSample(dev, bus);
  TEST_ASSERT_TRUE(dev.hasSample());
  TEST_ASSERT_TRUE(dev.lastMeasurementStatus().ok());
  SampleEnvelope before{};
  TEST_ASSERT_TRUE(dev.getSampleEnvelope(before).ok());
  TEST_ASSERT_NOT_EQUAL(0u, before.sampleSequence);
  TEST_ASSERT_NOT_EQUAL(0u, before.configGeneration);

  setRawSample(bus, cmd::RAW_PRESSURE_SKIPPED, 520000, 31000);
  captureForcedSample(dev, bus);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::COMPENSATION_ERROR),
                          static_cast<uint8_t>(dev.lastMeasurementStatus().code));
  TEST_ASSERT_TRUE(dev.hasSample());

  SampleEnvelope after{};
  TEST_ASSERT_TRUE(dev.getSampleEnvelope(after).ok());
  assertSampleEnvelopeEqual(before, after);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(SampleFreshness::STALE_AFTER_ERROR),
                          static_cast<uint8_t>(dev.sampleFreshness()));
}

void test_pressure_divide_by_zero_failure_preserves_committed_sample_envelope() {
  FakeBus bus;
  setPressureDenominatorZeroCalibration(bus);
  setRawSample(bus, 415148, 519888, 30000);
  BME280::BME280 dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  captureForcedSample(dev, bus);
  TEST_ASSERT_TRUE(dev.hasSample());
  TEST_ASSERT_TRUE(dev.lastMeasurementStatus().ok());
  SampleEnvelope before{};
  TEST_ASSERT_TRUE(dev.getSampleEnvelope(before).ok());

  setRawSample(bus, 415148, 211674, 30000);
  captureForcedSample(dev, bus);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::COMPENSATION_ERROR),
                          static_cast<uint8_t>(dev.lastMeasurementStatus().code));
  TEST_ASSERT_TRUE(dev.hasSample());

  SampleEnvelope after{};
  TEST_ASSERT_TRUE(dev.getSampleEnvelope(after).ok());
  assertSampleEnvelopeEqual(before, after);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(SampleFreshness::STALE_AFTER_ERROR),
                          static_cast<uint8_t>(dev.sampleFreshness()));
}

void test_resync_required_blocks_sync_and_staged_measurement_without_i2c() {
  FakeBus bus;
  BME280::BME280 dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(CalibrationState::VALID),
                          static_cast<uint8_t>(dev.calibrationState()));

  bus.failWriteOnCall = bus.writeCalls + 1u;
  bus.writeError = Status::Error(Err::I2C_TIMEOUT, "config sleep timeout", -141);
  const Status configStatus = dev.setFilter(Filter::X2);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_TIMEOUT),
                          static_cast<uint8_t>(configStatus.code));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ConfigSyncState::RESYNC_REQUIRED),
                          static_cast<uint8_t>(dev.configSyncState()));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(CalibrationState::VALID),
                          static_cast<uint8_t>(dev.calibrationState()));

  const uint32_t callsBeforeBlockedRequests = totalBusCalls(bus);
  Status st = dev.requestMeasurement();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::RESYNC_REQUIRED),
                          static_cast<uint8_t>(st.code));
  st = dev.startForcedMeasurementJob();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::RESYNC_REQUIRED),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_UINT32(callsBeforeBlockedRequests, totalBusCalls(bus));
}

void test_staged_apply_advances_generation_without_freshening_old_sample() {
  FakeBus bus;
  setBoschSyntheticCalibration(bus);
  setRawSample(bus, 415148, 519888, 30000);
  BME280::BME280 dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());
  captureForcedSample(dev, bus);

  SampleEnvelope oldSample{};
  TEST_ASSERT_TRUE(dev.getSampleEnvelope(oldSample).ok());
  const uint32_t oldGeneration = dev.configGeneration();
  const uint32_t oldSequence = dev.sampleSequence();
  TEST_ASSERT_NOT_EQUAL(0u, oldGeneration);
  TEST_ASSERT_NOT_EQUAL(0u, oldSequence);

  bus.failWriteOnCall = bus.writeCalls + 1u;
  bus.writeError = Status::Error(Err::I2C_TIMEOUT, "make config dirty", -142);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_TIMEOUT),
                          static_cast<uint8_t>(dev.setFilter(Filter::X2).code));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ConfigSyncState::RESYNC_REQUIRED),
                          static_cast<uint8_t>(dev.configSyncState()));

  TEST_ASSERT_TRUE(dev.startApplyConfigJob().inProgress());
  const JobPollResult result = pollUntilTerminal(dev, bus, 4);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(JobState::DONE),
                          static_cast<uint8_t>(result.state));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ConfigSyncState::SYNCHRONIZED),
                          static_cast<uint8_t>(dev.configSyncState()));
  TEST_ASSERT_TRUE(dev.configGeneration() > oldGeneration);

  SampleEnvelope afterApply{};
  TEST_ASSERT_TRUE(dev.getSampleEnvelope(afterApply).ok());
  assertSampleEnvelopeEqual(oldSample, afterApply);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(SampleFreshness::STALE_AFTER_CONFIG_CHANGE),
                          static_cast<uint8_t>(dev.sampleFreshness()));

  setRawSample(bus, 416000, 520000, 31000);
  captureForcedSample(dev, bus);
  SampleEnvelope newSample{};
  TEST_ASSERT_TRUE(dev.getSampleEnvelope(newSample).ok());
  TEST_ASSERT_EQUAL_UINT32(dev.configGeneration(), newSample.configGeneration);
  TEST_ASSERT_TRUE(newSample.sampleSequence > oldSequence);
  TEST_ASSERT_NOT_EQUAL(0u, newSample.sampleSequence);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(SampleFreshness::FRESH),
                          static_cast<uint8_t>(dev.sampleFreshness()));
}

void test_invalidate_device_state_is_zero_i2c_and_recovery_reloads_state() {
  FakeBus bus;
  setBoschSyntheticCalibration(bus);
  setRawSample(bus, 415148, 519888, 30000);
  BME280::BME280 dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());
  captureForcedSample(dev, bus);

  SampleEnvelope before{};
  TEST_ASSERT_TRUE(dev.getSampleEnvelope(before).ok());
  const uint32_t generationBefore = dev.configGeneration();
  const uint32_t callsBeforeInvalidation = totalBusCalls(bus);
  TEST_ASSERT_TRUE(dev.invalidateDeviceState().ok());
  TEST_ASSERT_EQUAL_UINT32(callsBeforeInvalidation, totalBusCalls(bus));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ConfigSyncState::RESYNC_REQUIRED),
                          static_cast<uint8_t>(dev.configSyncState()));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(CalibrationState::INVALID),
                          static_cast<uint8_t>(dev.calibrationState()));
  TEST_ASSERT_TRUE(dev.hasSample());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(SampleFreshness::STALE_AFTER_CONFIG_DIRTY),
                          static_cast<uint8_t>(dev.sampleFreshness()));

  SampleEnvelope afterInvalidation{};
  TEST_ASSERT_TRUE(dev.getSampleEnvelope(afterInvalidation).ok());
  assertSampleEnvelopeEqual(before, afterInvalidation);

  Calibration invalidCalibration{};
  invalidCalibration.digT1 = 0xCAFE;
  invalidCalibration.digT2 = -101;
  invalidCalibration.digT3 = 102;
  invalidCalibration.digP1 = 0xBEEF;
  invalidCalibration.digP2 = -201;
  invalidCalibration.digP3 = 202;
  invalidCalibration.digP4 = -203;
  invalidCalibration.digP5 = 204;
  invalidCalibration.digP6 = -205;
  invalidCalibration.digP7 = 206;
  invalidCalibration.digP8 = -207;
  invalidCalibration.digP9 = 208;
  invalidCalibration.digH1 = 0xA1;
  invalidCalibration.digH2 = -301;
  invalidCalibration.digH3 = 0xA3;
  invalidCalibration.digH4 = -304;
  invalidCalibration.digH5 = 305;
  invalidCalibration.digH6 = -36;
  const Calibration expectedInvalidOutput = invalidCalibration;
  const Status calibrationStatus = dev.getCalibration(invalidCalibration);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::RESYNC_REQUIRED),
                          static_cast<uint8_t>(calibrationStatus.code));
  assertCalibrationEqual(expectedInvalidOutput, invalidCalibration);

  const uint32_t callsBeforeBlockedRequests = totalBusCalls(bus);
  Status st = dev.requestMeasurement();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::RESYNC_REQUIRED),
                          static_cast<uint8_t>(st.code));
  st = dev.startForcedMeasurementJob();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::RESYNC_REQUIRED),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_UINT32(callsBeforeBlockedRequests, totalBusCalls(bus));

  putLe16(bus, cmd::REG_DIG_T1_LSB, 0x4321);
  putLe16(bus, cmd::REG_DIG_P1_LSB, 0x5678);
  TEST_ASSERT_TRUE(dev.startRecoveryJob().inProgress());
  const JobPollResult result = pollUntilTerminal(dev, bus, 4);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(JobState::DONE),
                          static_cast<uint8_t>(result.state));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ConfigSyncState::SYNCHRONIZED),
                          static_cast<uint8_t>(dev.configSyncState()));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(CalibrationState::VALID),
                          static_cast<uint8_t>(dev.calibrationState()));
  TEST_ASSERT_TRUE(dev.configGeneration() > generationBefore);
  TEST_ASSERT_TRUE(dev.hasSample());

  SampleEnvelope afterRecovery{};
  TEST_ASSERT_TRUE(dev.getSampleEnvelope(afterRecovery).ok());
  assertSampleEnvelopeEqual(before, afterRecovery);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(SampleFreshness::STALE_AFTER_CONFIG_CHANGE),
                          static_cast<uint8_t>(dev.sampleFreshness()));

  Calibration reloaded{};
  TEST_ASSERT_TRUE(dev.getCalibration(reloaded).ok());
  TEST_ASSERT_EQUAL_HEX16(0x4321, reloaded.digT1);
  TEST_ASSERT_EQUAL_HEX16(0x5678, reloaded.digP1);
}

void test_erased_humidity_calibration_is_rejected_unless_humidity_skipped() {
  const uint8_t erasedValues[] = {0x00, 0xFF};
  for (size_t caseIndex = 0; caseIndex < sizeof(erasedValues); ++caseIndex) {
    FakeBus bus;
    for (size_t i = 0; i < cmd::REG_CALIB_H_LEN; ++i) {
      bus.reg[static_cast<uint8_t>(cmd::REG_CALIB_H_START + static_cast<uint8_t>(i))] =
          erasedValues[caseIndex];
    }
    BME280::BME280 dev;
    const Status st = dev.begin(makeConfig(bus));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::CALIBRATION_INVALID),
                            static_cast<uint8_t>(st.code));
    TEST_ASSERT_FALSE(dev.isInitialized());
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(CalibrationState::INVALID),
                            static_cast<uint8_t>(dev.calibrationState()));
  }

  FakeBus skipBus;
  for (size_t i = 0; i < cmd::REG_CALIB_H_LEN; ++i) {
    skipBus.reg[static_cast<uint8_t>(cmd::REG_CALIB_H_START + static_cast<uint8_t>(i))] = 0;
  }
  Config skipConfig = makeConfig(skipBus);
  skipConfig.osrsH = Oversampling::SKIP;
  BME280::BME280 skipDev;
  TEST_ASSERT_TRUE(skipDev.begin(skipConfig).ok());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(CalibrationState::VALID),
                          static_cast<uint8_t>(skipDev.calibrationState()));
  const uint32_t callsBeforeEnable = totalBusCalls(skipBus);
  const Status enableStatus = skipDev.setOversamplingH(Oversampling::X1);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::CALIBRATION_INVALID),
                          static_cast<uint8_t>(enableStatus.code));
  TEST_ASSERT_EQUAL_UINT32(callsBeforeEnable, totalBusCalls(skipBus));
}

void test_register_access_after_end_does_not_touch_bus() {
  FakeBus bus;
  BME280::BME280 dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  const uint32_t writesAfterBegin = bus.writeCalls;
  const uint32_t readsAfterBegin = bus.readCalls;

  dev.end();
  TEST_ASSERT_EQUAL_UINT32(writesAfterBegin + 1u, bus.writeCalls);
  TEST_ASSERT_EQUAL_UINT32(readsAfterBegin, bus.readCalls);

  uint8_t value = 0;
  Status st = dev.readRegister(cmd::REG_CHIP_ID, value);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::NOT_INITIALIZED),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_UINT32(readsAfterBegin, bus.readCalls);

  st = dev.writeRegister(cmd::REG_CTRL_MEAS, 0);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::NOT_INITIALIZED),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_UINT32(writesAfterBegin + 1u, bus.writeCalls);

  st = dev.readRegisters(cmd::REG_CHIP_ID, &value, 1);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::NOT_INITIALIZED),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_UINT32(readsAfterBegin, bus.readCalls);
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_status_ok);
  RUN_TEST(test_status_error);
  RUN_TEST(test_status_in_progress);
  RUN_TEST(test_config_defaults);
  RUN_TEST(test_driver_is_not_copyable_or_movable);
  RUN_TEST(test_get_settings_snapshot);
  RUN_TEST(test_begin_rejects_missing_callbacks);
  RUN_TEST(test_begin_rejects_zero_nvm_timeout);
  RUN_TEST(test_begin_rejects_oversized_nvm_timeout_without_i2c);
  RUN_TEST(test_begin_rejects_invalid_oversampling_combination);
  RUN_TEST(test_high_bit_enum_values_are_rejected_without_i2c);
  RUN_TEST(test_invalid_begin_after_success_resets_default_runtime);
  RUN_TEST(test_begin_normalizes_offline_threshold_on_stored_copy);
  RUN_TEST(test_begin_success_sets_ready_without_health_counts);
  RUN_TEST(test_begin_passes_configured_i2c_timeout_to_transport);
  RUN_TEST(test_begin_starts_new_health_session_and_resets_counters);
  RUN_TEST(test_begin_returns_busy_when_nvm_update_in_progress_without_calibration_read);
  RUN_TEST(test_begin_defers_apply_config_when_device_measuring_without_write);
  RUN_TEST(test_begin_nvm_timeout_uses_wrap_safe_deadline_without_tight_poll_loop);
  RUN_TEST(test_begin_rejects_invalid_temperature_calibration);
  RUN_TEST(test_begin_rejects_invalid_pressure_calibration);
  RUN_TEST(test_begin_forced_mode_keeps_hardware_sleep_until_requested);
  RUN_TEST(test_init_job_budget_one_instruction_per_poll);
  RUN_TEST(test_init_job_nvm_busy_reads_status_one_poll_at_a_time);
  RUN_TEST(test_init_job_stuck_nvm_no_spin_when_time_static);
  RUN_TEST(test_apply_config_job_waits_for_not_measuring_before_writes);
  RUN_TEST(test_apply_config_job_checks_not_measuring_after_sleep_write);
  RUN_TEST(test_init_job_success_clears_existing_dirty_state);
  RUN_TEST(test_recovery_job_success_clears_existing_dirty_state);
  RUN_TEST(test_recovery_job_failure_after_reset_marks_dirty);
  RUN_TEST(test_missing_now_ms_fallback_is_framework_neutral);
  RUN_TEST(test_begin_without_now_ms_uses_framework_neutral_fallback);
  RUN_TEST(test_request_measurement_requires_now_ms_hook);
  RUN_TEST(test_probe_transport_fault_is_preserved_and_does_not_update_health);
  RUN_TEST(test_probe_address_nack_maps_to_device_not_found_without_health_update);
  RUN_TEST(test_probe_address_nack_maps_to_device_not_found);
  RUN_TEST(test_probe_preserves_non_address_transport_errors_without_health_update);
  RUN_TEST(test_probe_chip_id_mismatch_does_not_update_health);
  RUN_TEST(test_begin_accepts_both_supported_addresses_and_rejects_wrong_address);
  RUN_TEST(test_begin_rejects_wrong_chip_id);
  RUN_TEST(test_begin_address_nack_maps_to_device_not_found);
  RUN_TEST(test_begin_address_nack_after_chip_id_maps_to_device_not_found);
  RUN_TEST(test_begin_preserves_timeout_transport_error);
  RUN_TEST(test_begin_preserves_chip_id_bus_and_data_errors);
  RUN_TEST(test_recover_failure_updates_health_once);
  RUN_TEST(test_recover_success_returns_ready);
  RUN_TEST(test_recover_chip_id_mismatch_updates_health);
  RUN_TEST(test_recover_preserves_transport_error_code);
  RUN_TEST(test_set_oversampling_rejects_invalid_compensation_combo_without_write);
  RUN_TEST(test_humidity_oversampling_writes_ctrl_hum_then_ctrl_meas);
  RUN_TEST(test_config_change_in_normal_mode_sleeps_writes_config_and_restores);
  RUN_TEST(test_config_write_while_measuring_returns_busy_without_write);
  RUN_TEST(test_config_write_measuring_after_sleep_marks_dirty_without_config_write);
  RUN_TEST(test_config_change_failure_at_sleep_step_marks_dirty);
  RUN_TEST(test_config_change_failure_at_config_step_marks_dirty_after_restore);
  RUN_TEST(test_humidity_ctrl_hum_failure_marks_dirty_and_preserves_error);
  RUN_TEST(test_recover_apply_config_first_write_failure_marks_dirty);
  RUN_TEST(test_measurement_time_estimate_uses_oversampling_formula);
  RUN_TEST(test_partial_config_restore_failure_marks_hardware_dirty);
  RUN_TEST(test_dirty_state_clears_after_successful_recover_resync);
  RUN_TEST(test_diagnostic_config_write_marks_dirty_after_success_and_recover_clears);
  RUN_TEST(test_diagnostic_config_write_invalidates_cached_samples);
  RUN_TEST(test_diagnostic_config_block_write_marks_dirty_when_range_overlaps_config);
  RUN_TEST(test_diagnostic_non_config_write_does_not_mark_dirty);
  RUN_TEST(test_diagnostic_config_write_failure_preserves_error_and_marks_dirty);
  RUN_TEST(test_diagnostic_config_write_address_nack_does_not_mark_dirty);
  RUN_TEST(test_recover_preserves_cached_sample_until_successful_resync_then_invalidates);
  RUN_TEST(test_recovery_job_failure_preserves_cached_sample_until_successful_resync);
  RUN_TEST(test_invalid_begin_does_not_clear_existing_dirty_state);
  RUN_TEST(test_apply_config_partial_failure_marks_dirty_and_preserves_error);
  RUN_TEST(test_soft_reset_write_timeout_marks_dirty_and_preserves_error);
  RUN_TEST(test_soft_reset_write_address_nack_preserves_error_without_dirty_state);
  RUN_TEST(test_soft_reset_nvm_im_update_busy_marks_dirty_and_health_failure);
  RUN_TEST(test_begin_nvm_transport_error_preserved);
  RUN_TEST(test_begin_calibration_read_failure_preserves_transport_error);
  RUN_TEST(test_begin_apply_config_failure_marks_dirty_and_stays_uninitialized);
  RUN_TEST(test_soft_reset_nvm_transport_error_preserved_and_marks_dirty);
  RUN_TEST(test_soft_reset_nvm_status_transport_error_preserves_detail);
  RUN_TEST(test_soft_reset_calibration_read_failure_marks_dirty);
  RUN_TEST(test_soft_reset_invalid_calibration_marks_dirty_and_records_health_failure);
  RUN_TEST(test_soft_reset_apply_config_failure_marks_dirty_and_preserves_error);
  RUN_TEST(test_soft_reset_success_reloads_calibration_and_clears_dirty);
  RUN_TEST(test_soft_reset_success_invalidates_cached_samples);
  RUN_TEST(test_recover_returns_busy_when_nvm_update_in_progress_without_calibration_read);
  RUN_TEST(test_recover_nvm_transport_error_updates_health_and_preserves_error);
  RUN_TEST(test_recover_invalid_calibration_records_health_failure);
  RUN_TEST(test_set_mode_forced_does_not_trigger_conversion);
  RUN_TEST(test_example_transport_maps_wire_errors_and_keeps_timeout_owned_by_init);
  RUN_TEST(test_example_transport_validates_params_and_handles_write_read);
  RUN_TEST(test_recover_reaches_offline_when_threshold_is_one);
  RUN_TEST(test_offline_latches_public_register_read_without_i2c);
  RUN_TEST(test_offline_typed_config_setters_do_not_mark_dirty_without_i2c);
  RUN_TEST(test_probe_works_while_offline_without_clearing_latch);
  RUN_TEST(test_successful_recover_from_offline_clears_latch_and_allows_i2c);
  RUN_TEST(test_failed_recover_from_offline_keeps_latch_after_intermediate_success);
  RUN_TEST(test_recovery_job_from_offline_clears_latch_and_allows_i2c);
  RUN_TEST(test_recovery_job_from_offline_failure_reasserts_offline_latch);
  RUN_TEST(test_recovery_job_from_offline_does_not_report_ready_until_complete);
  RUN_TEST(test_non_recovery_job_still_blocked_while_offline);
  RUN_TEST(test_forced_measurement_timing_wraparound_reaches_ready);
  RUN_TEST(test_sample_freshness_none_before_capture);
  RUN_TEST(test_sample_freshness_fresh_after_successful_capture);
  RUN_TEST(test_sample_freshness_stale_after_failed_refresh);
  RUN_TEST(test_sample_freshness_stale_when_hardware_config_dirty);
  RUN_TEST(test_sample_fresh_uses_wrap_safe_age_check);
  RUN_TEST(test_normal_mode_request_waits_for_fresh_cycle);
  RUN_TEST(test_forced_measurement_request_while_busy_tracks_completion);
  RUN_TEST(test_tick_raw_read_failure_records_measurement_status);
  RUN_TEST(test_tick_stuck_measuring_times_out_without_raw_read);
  RUN_TEST(test_set_mode_sleep_cancels_pending_measurement_request);
  RUN_TEST(test_raw_and_compensated_samples_remain_available_after_measurement_read);
  RUN_TEST(test_forced_measurement_job_budget_and_raw_fixed_outputs);
  RUN_TEST(test_forced_measurement_job_reports_in_progress_status);
  RUN_TEST(test_forced_measurement_job_success_clears_last_measurement_status);
  RUN_TEST(test_forced_measurement_job_failure_preserves_last_measurement_status);
  RUN_TEST(test_forced_measurement_job_measuring_status_does_not_read_raw_same_poll);
  RUN_TEST(test_forced_measurement_job_status_failure_clears_pending_state);
  RUN_TEST(test_forced_measurement_job_stuck_measuring_times_out_without_raw_read);
  RUN_TEST(test_recovery_job_budget_and_full_sequence);
  RUN_TEST(test_recovery_job_chip_id_mismatch_records_health_failure);
  RUN_TEST(test_recovery_job_nvm_busy_uses_one_status_read_per_poll);
  RUN_TEST(test_recovery_job_nvm_timeout_records_health_failure);
  RUN_TEST(test_recovery_job_failed_humidity_calibration_keeps_previous_coefficients);
  RUN_TEST(test_recovery_job_error_stops_without_extra_instructions);
  RUN_TEST(test_calibration_parses_bosch_synthetic_coefficients);
  RUN_TEST(test_calibration_parses_signed_boundaries_and_humidity_nibbles);
  RUN_TEST(test_invalid_pressure_calibration_is_rejected);
  RUN_TEST(test_read_calibration_raw_uses_register_bytes_and_preserves_error);
  RUN_TEST(test_raw_burst_reconstructs_20_and_16_bit_samples);
  RUN_TEST(test_compensation_matches_datasheet_derived_synthetic_vector);
  RUN_TEST(test_humidity_compensation_clamps_low_and_high);
  RUN_TEST(test_skipped_sentinels_are_explicit_validity_flags);
  RUN_TEST(test_enabled_raw_sentinel_rejects_compensated_sample);
  RUN_TEST(test_pressure_compensation_divide_by_zero_guard_blocks_sample);
  RUN_TEST(test_config_change_invalidates_cached_samples);
  RUN_TEST(test_raw_sentinel_failure_preserves_committed_sample_envelope);
  RUN_TEST(test_pressure_divide_by_zero_failure_preserves_committed_sample_envelope);
  RUN_TEST(test_resync_required_blocks_sync_and_staged_measurement_without_i2c);
  RUN_TEST(test_staged_apply_advances_generation_without_freshening_old_sample);
  RUN_TEST(test_invalidate_device_state_is_zero_i2c_and_recovery_reloads_state);
  RUN_TEST(test_erased_humidity_calibration_is_rejected_unless_humidity_skipped);
  RUN_TEST(test_register_access_after_end_does_not_touch_bus);
  return UNITY_END();
}
