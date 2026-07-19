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
  uint32_t failWriteAfterEffectOnCall = 0;
  TransportResult readError =
      TransportResult::Error(TransportErr::OTHER, -1);
  TransportResult writeError =
      TransportResult::Error(TransportErr::OTHER, -2);
  uint8_t lastWriteReg = 0;
  uint8_t lastWriteValue = 0;
  uint8_t writeRegLog[64] = {};
  uint8_t writeValueLog[64] = {};
  uint8_t writeLogLen = 0;
  bool failReadRegEnabled = false;
  uint8_t failReadReg = 0;
  uint32_t failReadRegRemaining = 0;
  uint8_t lastReadReg = 0;
  size_t lastReadTxLen = 0;
  size_t lastReadLen = 0;
  size_t lastWriteLen = 0;
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

TransportResult fakeWrite(uint8_t addr, const uint8_t* data, size_t len,
                          uint32_t timeoutMs, void* user) {
  FakeBus* bus = static_cast<FakeBus*>(user);
  bus->writeCalls++;
  bus->lastWriteTimeoutMs = timeoutMs;
  bus->lastWriteLen = len;
  if (addr != bus->deviceAddress) {
    return TransportResult::Error(TransportErr::NACK_ADDRESS, addr);
  }
  if (data == nullptr || len == 0) {
    return TransportResult::Error(TransportErr::OTHER, -3);
  }
  if (bus->failWriteOnCall != 0 && bus->writeCalls == bus->failWriteOnCall) {
    bus->failWriteOnCall = 0;
    return bus->writeError;
  }
  if (bus->writeErrorRemaining > 0) {
    bus->writeErrorRemaining--;
    return bus->writeError;
  }
  const bool failAfterEffect =
      bus->failWriteAfterEffectOnCall != 0 &&
      bus->writeCalls == bus->failWriteAfterEffectOnCall;
  if (failAfterEffect) {
    bus->failWriteAfterEffectOnCall = 0;
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
  if (failAfterEffect) {
    return bus->writeError;
  }
  return TransportResult::Complete(len);
}

TransportResult fakeWriteRead(uint8_t addr, const uint8_t* txData, size_t txLen,
                              uint8_t* rxData, size_t rxLen,
                              uint32_t timeoutMs, void* user) {
  FakeBus* bus = static_cast<FakeBus*>(user);
  bus->readCalls++;
  bus->lastReadTimeoutMs = timeoutMs;
  bus->lastReadTxLen = txLen;
  if (addr != bus->deviceAddress) {
    return TransportResult::Error(TransportErr::NACK_ADDRESS, addr);
  }
  if (txData == nullptr || txLen == 0 || (rxLen > 0 && rxData == nullptr)) {
    return TransportResult::Error(TransportErr::OTHER, -4);
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

  return TransportResult::Complete(txLen, rxLen);
}

static_assert(std::is_same<I2cWriteFn, decltype(&fakeWrite)>::value,
              "write callback must return TransportResult");
static_assert(std::is_same<I2cWriteReadFn, decltype(&fakeWriteRead)>::value,
              "write-read callback must return TransportResult");
static_assert(!std::is_constructible<TransportResult, Status>::value,
              "driver Status must not be accepted as a transport result");
static_assert(!std::is_convertible<Status, TransportResult>::value,
              "driver Status must not convert to a transport result");

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
  cfg.conversionReadyTimeoutMs = 20;
  cfg.offlineThreshold = 3;
  cfg.mode = Mode::FORCED;
  return cfg;
}

uint32_t totalBusCalls(const FakeBus& bus) {
  return bus.readCalls + bus.writeCalls;
}

uint32_t countWritesToRegSince(const FakeBus& bus, uint8_t reg,
                               uint8_t startIndex) {
  uint32_t count = 0;
  for (uint8_t i = startIndex; i < bus.writeLogLen; ++i) {
    if (bus.writeRegLog[i] == reg) {
      ++count;
    }
  }
  return count;
}

JobPollResult pollAtWithBudget(BME280::BME280& dev, FakeBus& bus,
                               uint32_t nowMs, uint8_t maxInstructions) {
  const uint32_t callsBefore = totalBusCalls(bus);
  JobPollResult result = dev.pollJob(nowMs, maxInstructions);
  const uint32_t callsAfter = totalBusCalls(bus);
  TEST_ASSERT_EQUAL_UINT32(static_cast<uint32_t>(result.instructionsUsed),
                           callsAfter - callsBefore);
  TEST_ASSERT_EQUAL_UINT8(result.instructionsUsed, result.callbacksUsed);
  TEST_ASSERT_TRUE(result.instructionsUsed <= maxInstructions);
  return result;
}

JobPollResult pollWithBudget(BME280::BME280& dev, FakeBus& bus,
                             uint8_t maxInstructions) {
  return pollAtWithBudget(dev, bus, bus.nowMs, maxInstructions);
}

JobPollResult pollUntilPhase(BME280::BME280& dev, FakeBus& bus,
                             JobPhase target, uint16_t maxPolls = 32) {
  JobPollResult result{};
  for (uint16_t i = 0; i < maxPolls; ++i) {
    if (dev.jobPhase() == target) {
      return result;
    }
    result = pollWithBudget(dev, bus, 1);
    if (result.state == JobState::FAILED || result.state == JobState::DONE ||
        result.state == JobState::CANCELLED || result.state == JobState::TIMED_OUT) {
      break;
    }
  }
  TEST_FAIL_MESSAGE("job did not reach requested phase");
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

void assertBusy(const Status& status) {
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::BUSY),
                          static_cast<uint8_t>(status.code));
}

void assertBusyReason(const Status& status, BusyReason reason) {
  assertBusy(status);
  TEST_ASSERT_EQUAL_INT32(static_cast<int32_t>(reason), status.detail);
}

void assertRepresentativeHardwareApisBusy(BME280::BME280& dev, FakeBus& bus,
                                           const Config& beginConfig) {
  const uint32_t callsBefore = totalBusCalls(bus);
  uint8_t value = 0;
  assertBusy(dev.probe());
  assertBusy(dev.readStatus(value));
  assertBusy(dev.setMode(Mode::SLEEP));
  assertBusy(dev.requestMeasurement());
  assertBusy(dev.begin(beginConfig));
  TEST_ASSERT_EQUAL_UINT32(callsBefore, totalBusCalls(bus));
}

void assertAllHardwareApisBusy(BME280::BME280& dev, FakeBus& bus,
                               const Config& beginConfig) {
  const uint32_t callsBefore = totalBusCalls(bus);
  uint8_t value = 0;
  uint8_t readBuffer[2] = {};
  const uint8_t writeBuffer[2] = {0x12, 0x34};
  bool measuring = false;
  CalibrationRaw calibrationRaw{};

  assertBusy(dev.probe());
  assertBusy(dev.recover());
  assertBusy(dev.requestMeasurement());
  assertBusy(dev.readCalibrationRaw(calibrationRaw));
  assertBusy(dev.setMode(Mode::SLEEP));
  assertBusy(dev.setOversamplingT(Oversampling::X2));
  assertBusy(dev.setOversamplingP(Oversampling::X2));
  assertBusy(dev.setOversamplingH(Oversampling::X2));
  assertBusy(dev.setFilter(Filter::X2));
  assertBusy(dev.setStandby(Standby::MS_250));
  assertBusy(dev.softReset());
  assertBusy(dev.readChipId(value));
  assertBusy(dev.readStatus(value));
  assertBusy(dev.readCtrlHum(value));
  assertBusy(dev.readCtrlMeas(value));
  assertBusy(dev.readConfig(value));
  assertBusy(dev.isMeasuring(measuring));
  assertBusy(dev.readRegisters(cmd::REG_STATUS, readBuffer, sizeof(readBuffer)));
  assertBusy(dev.writeRegisters(cmd::REG_CONFIG, writeBuffer, sizeof(writeBuffer)));
  assertBusy(dev.readRegister(cmd::REG_CHIP_ID, value));
  assertBusy(dev.writeRegister(cmd::REG_CONFIG, 0));
  assertBusy(dev.begin(beginConfig));

  TEST_ASSERT_EQUAL_UINT32(callsBefore, totalBusCalls(bus));
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

void test_transport_result_contract_is_terminal_only() {
  const TransportResult writeDone = TransportResult::Complete(2);
  TEST_ASSERT_TRUE(writeDone.ok());
  TEST_ASSERT_EQUAL_UINT32(2u, static_cast<uint32_t>(writeDone.writeCount));
  TEST_ASSERT_EQUAL_UINT32(0u, static_cast<uint32_t>(writeDone.readCount));

  const TransportResult combinedDone = TransportResult::Complete(1, 8);
  TEST_ASSERT_TRUE(combinedDone.ok());
  TEST_ASSERT_EQUAL_UINT32(1u,
                           static_cast<uint32_t>(combinedDone.writeCount));
  TEST_ASSERT_EQUAL_UINT32(8u,
                           static_cast<uint32_t>(combinedDone.readCount));

  const TransportResult timeout =
      TransportResult::Error(TransportErr::TIMEOUT, -180, 1, 3);
  TEST_ASSERT_FALSE(timeout.ok());
  TEST_ASSERT_EQUAL_STRING("TIMEOUT", toString(timeout.code));
  TEST_ASSERT_EQUAL_INT32(-180, timeout.detail);
}

void test_status_copy_assignment_and_persistent_fields_use_canonical_messages() {
  char borrowed[] = "borrowed message";
  Status source{Err::I2C_BUS, -181, borrowed};
  TEST_ASSERT_EQUAL_PTR(toString(Err::I2C_BUS), source.msg);

  source.msg = borrowed;
  Status copied(source);
  Status assigned;
  assigned = source;
  borrowed[0] = 'X';
  TEST_ASSERT_EQUAL_PTR(toString(Err::I2C_BUS), copied.msg);
  TEST_ASSERT_EQUAL_PTR(toString(Err::I2C_BUS), assigned.msg);
  TEST_ASSERT_EQUAL_STRING("I2C_BUS", copied.msg);
  TEST_ASSERT_EQUAL_STRING("I2C_BUS", assigned.msg);

  FakeBus configBus;
  BME280::BME280 configDev;
  TEST_ASSERT_TRUE(configDev.begin(makeConfig(configBus)).ok());
  configBus.writeError =
      TransportResult{TransportErr::OK, -182, 1, 0};
  configBus.failWriteAfterEffectOnCall = configBus.writeCalls + 1u;
  Status returned = configDev.writeRegister(cmd::REG_CONFIG, 0xA0);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_SHORT_TRANSFER),
                          static_cast<uint8_t>(returned.code));
  char returnedBorrowed[] = "returned borrowed";
  returned.msg = returnedBorrowed;
  SettingsSnapshot configSnapshot{};
  TEST_ASSERT_TRUE(configDev.getSettings(configSnapshot).ok());
  returnedBorrowed[0] = 'X';
  TEST_ASSERT_EQUAL_PTR(toString(Err::I2C_SHORT_TRANSFER),
                        configDev.lastError().msg);
  TEST_ASSERT_EQUAL_PTR(toString(Err::I2C_SHORT_TRANSFER),
                        configDev.hardwareConfigDirtyError().msg);
  TEST_ASSERT_EQUAL_PTR(toString(Err::I2C_SHORT_TRANSFER),
                        configSnapshot.hardwareConfigDirtyError.msg);

  FakeBus jobBus;
  BME280::BME280 jobDev;
  TEST_ASSERT_TRUE(jobDev.begin(makeConfig(jobBus)).ok());
  Status started = jobDev.startForcedMeasurementJob();
  TEST_ASSERT_TRUE(started.inProgress());
  char jobBorrowed[] = "job borrowed";
  started.msg = jobBorrowed;
  const JobPollResult polled = jobDev.pollJob(jobBus.nowMs, 0);
  SettingsSnapshot jobSnapshot{};
  TEST_ASSERT_TRUE(jobDev.getSettings(jobSnapshot).ok());
  jobBorrowed[0] = 'X';
  TEST_ASSERT_EQUAL_PTR(toString(Err::IN_PROGRESS), jobDev.jobStatus().msg);
  TEST_ASSERT_EQUAL_PTR(toString(Err::IN_PROGRESS), polled.status.msg);
  TEST_ASSERT_EQUAL_PTR(toString(Err::IN_PROGRESS),
                        jobDev.lastMeasurementStatus().msg);
  TEST_ASSERT_EQUAL_PTR(toString(Err::IN_PROGRESS),
                        jobSnapshot.lastMeasurementStatus.msg);
}

void test_successful_transport_callbacks_report_exact_counts_once() {
  FakeBus bus;
  BME280::BME280 dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  const uint32_t writesBefore = bus.writeCalls;
  Status st = dev.writeRegister(0xA0, 0x5A);
  TEST_ASSERT_TRUE(st.ok());
  TEST_ASSERT_EQUAL_UINT32(writesBefore + 1u, bus.writeCalls);
  TEST_ASSERT_EQUAL_UINT32(2u, static_cast<uint32_t>(bus.lastWriteLen));

  const uint32_t readsBefore = bus.readCalls;
  uint8_t values[2] = {};
  st = dev.readRegisters(0xA0, values, sizeof(values));
  TEST_ASSERT_TRUE(st.ok());
  TEST_ASSERT_EQUAL_UINT32(readsBefore + 1u, bus.readCalls);
  TEST_ASSERT_EQUAL_UINT32(1u, static_cast<uint32_t>(bus.lastReadTxLen));
  TEST_ASSERT_EQUAL_UINT32(2u, static_cast<uint32_t>(bus.lastReadLen));
  TEST_ASSERT_EQUAL_HEX8(0x5A, values[0]);
}

void test_short_ok_counts_map_to_short_transfer_without_retry() {
  FakeBus bus;
  BME280::BME280 dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  uint32_t callsBefore = bus.writeCalls;
  bus.failWriteOnCall = callsBefore + 1u;
  bus.writeError = TransportResult{TransportErr::OK, -183, 1, 0};
  Status st = dev.writeRegister(0xA0, 0x5A);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_SHORT_TRANSFER),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_INT32(-183, st.detail);
  TEST_ASSERT_EQUAL_UINT32(callsBefore + 1u, bus.writeCalls);

  uint8_t value = 0xA5;
  callsBefore = bus.readCalls;
  bus.failReadRegEnabled = true;
  bus.failReadReg = 0xA0;
  bus.readError = TransportResult{TransportErr::OK, -184, 0, 1};
  st = dev.readRegister(0xA0, value);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_SHORT_TRANSFER),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_INT32(-184, st.detail);
  TEST_ASSERT_EQUAL_UINT32(callsBefore + 1u, bus.readCalls);
  TEST_ASSERT_EQUAL_HEX8(0xA5, value);

  uint8_t values[2] = {0xA5, 0x5A};
  callsBefore = bus.readCalls;
  bus.failReadRegEnabled = true;
  bus.failReadReg = 0xA0;
  bus.readError = TransportResult{TransportErr::OK, -185, 1, 1};
  st = dev.readRegisters(0xA0, values, sizeof(values));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_SHORT_TRANSFER),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_INT32(-185, st.detail);
  TEST_ASSERT_EQUAL_UINT32(callsBefore + 1u, bus.readCalls);
  TEST_ASSERT_EQUAL_HEX8(0xA5, values[0]);
  TEST_ASSERT_EQUAL_HEX8(0x5A, values[1]);
}

void test_every_transport_error_maps_detail_and_nack_remap_is_presence_only() {
  struct MappingCase {
    TransportErr transport;
    Err driver;
  };
  const MappingCase cases[] = {
      {TransportErr::NACK_ADDRESS, Err::I2C_NACK_ADDR},
      {TransportErr::NACK_DATA, Err::I2C_NACK_DATA},
      {TransportErr::TIMEOUT, Err::I2C_TIMEOUT},
      {TransportErr::BUS, Err::I2C_BUS},
      {TransportErr::OTHER, Err::I2C_ERROR},
  };

  for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
    FakeBus bus;
    BME280::BME280 dev;
    TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());
    const int32_t detail = static_cast<int32_t>(-190 -
        static_cast<int32_t>(i));
    const uint32_t readsBefore = bus.readCalls;
    bus.failReadRegEnabled = true;
    bus.failReadReg = 0xA0;
    bus.readError = TransportResult::Error(cases[i].transport, detail);
    uint8_t value = 0;
    const Status st = dev.readRegister(0xA0, value);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(cases[i].driver),
                            static_cast<uint8_t>(st.code));
    TEST_ASSERT_EQUAL_INT32(detail, st.detail);
    TEST_ASSERT_EQUAL_UINT32(readsBefore + 1u, bus.readCalls);
  }

  FakeBus presenceBus;
  BME280::BME280 presenceDev;
  TEST_ASSERT_TRUE(presenceDev.begin(makeConfig(presenceBus)).ok());
  const uint32_t readsBefore = presenceBus.readCalls;
  presenceBus.readErrorRemaining = 1;
  presenceBus.readError =
      TransportResult::Error(TransportErr::NACK_ADDRESS, -199);
  const Status presence = presenceDev.probe();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::DEVICE_NOT_FOUND),
                          static_cast<uint8_t>(presence.code));
  TEST_ASSERT_EQUAL_INT32(-199, presence.detail);
  TEST_ASSERT_EQUAL_UINT32(readsBefore + 1u, presenceBus.readCalls);
}

void test_mutating_short_transfer_marks_dirty_or_trigger_ambiguous() {
  {
    FakeBus bus;
    BME280::BME280 dev;
    TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());
    const uint32_t writesBefore = bus.writeCalls;
    bus.writeError = TransportResult{TransportErr::OK, -200, 1, 0};
    bus.failWriteAfterEffectOnCall = writesBefore + 1u;
    const Status st = dev.writeRegister(cmd::REG_CONFIG, 0xA0);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_SHORT_TRANSFER),
                            static_cast<uint8_t>(st.code));
    TEST_ASSERT_EQUAL_INT32(-200, st.detail);
    TEST_ASSERT_EQUAL_UINT32(writesBefore + 1u, bus.writeCalls);
    TEST_ASSERT_TRUE(dev.hardwareConfigDirty());
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_SHORT_TRANSFER),
                            static_cast<uint8_t>(
                                dev.hardwareConfigDirtyError().code));
  }

  {
    FakeBus bus;
    BME280::BME280 dev;
    TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());
    const uint32_t writesBefore = bus.writeCalls;
    bus.writeError = TransportResult{TransportErr::OK, -201, 1, 0};
    bus.failWriteAfterEffectOnCall = writesBefore + 1u;
    const Status st = dev.requestMeasurement();
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_SHORT_TRANSFER),
                            static_cast<uint8_t>(st.code));
    TEST_ASSERT_EQUAL_INT32(-201, st.detail);
    TEST_ASSERT_EQUAL_UINT32(writesBefore + 1u, bus.writeCalls);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(ConversionState::UNKNOWN_AFTER_TRIGGER_ERROR),
        static_cast<uint8_t>(dev.conversionState()));
  }
}

void test_partial_raw_read_preserves_committed_sample() {
  FakeBus bus;
  setBoschSyntheticCalibration(bus);
  setRawSample(bus, 415148, 519888, 30000);
  BME280::BME280 dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());
  captureForcedSample(dev, bus);
  SampleEnvelope before{};
  TEST_ASSERT_TRUE(dev.getSampleEnvelope(before).ok());

  setRawSample(bus, 415148, 520000, 31000);
  TEST_ASSERT_TRUE(dev.requestMeasurement().inProgress());
  bus.failReadRegEnabled = true;
  bus.failReadReg = cmd::REG_DATA_START;
  bus.readError = TransportResult{
      TransportErr::OK, -202, 1, cmd::DATA_LEN - 1u};
  const uint32_t readsBefore = bus.readCalls;
  bus.nowMs += dev.estimateMeasurementTimeMs();
  dev.tick(bus.nowMs);
  TEST_ASSERT_EQUAL_UINT32(readsBefore + 2u, bus.readCalls);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_SHORT_TRANSFER),
                          static_cast<uint8_t>(
                              dev.lastMeasurementStatus().code));
  TEST_ASSERT_EQUAL_INT32(-202, dev.lastMeasurementStatus().detail);
  TEST_ASSERT_FALSE(dev.hardwareConfigDirty());

  SampleEnvelope after{};
  TEST_ASSERT_TRUE(dev.getSampleEnvelope(after).ok());
  assertSampleEnvelopeEqual(before, after);
}

void test_config_defaults() {
  Config cfg;
  TEST_ASSERT_NULL(cfg.i2cWrite);
  TEST_ASSERT_NULL(cfg.i2cWriteRead);
  TEST_ASSERT_EQUAL_HEX8(0x76, cfg.i2cAddress);
  TEST_ASSERT_EQUAL_UINT16(50, cfg.i2cTimeoutMs);
  TEST_ASSERT_EQUAL_UINT32(10u, cfg.nvmReadyTimeoutMs);
  TEST_ASSERT_EQUAL_UINT32(20u, cfg.conversionReadyTimeoutMs);
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
  TEST_ASSERT_EQUAL_UINT32(20u, snap.conversionReadyTimeoutMs);
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
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ConversionState::IDLE),
                          static_cast<uint8_t>(snap.conversionState));
  TEST_ASSERT_TRUE(snap.lastMeasurementStatus.ok());
  TEST_ASSERT_FALSE(snap.hasSample);
  TEST_ASSERT_FALSE(snap.hardwareConfigDirty);
  TEST_ASSERT_TRUE(snap.hardwareConfigDirtyError.ok());
  TEST_ASSERT_EQUAL_UINT32(0u, snap.measurementStartMs);
  TEST_ASSERT_EQUAL_UINT32(0u, snap.sampleTimestampMs);
  TEST_ASSERT_EQUAL_INT32(0, snap.tFine);
  TEST_ASSERT_EQUAL_INT32(0, snap.rawSample.adcT);
  TEST_ASSERT_EQUAL_UINT32(0u, snap.compSample.pressurePa);
  TEST_ASSERT_FALSE(snap.lastOkTimeValid);
  TEST_ASSERT_FALSE(snap.lastErrorTimeValid);
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
  const uint32_t invalidValues[] = {0x7FFFFFFFUL, 0x80000000UL};
  for (size_t i = 0; i < 2; ++i) {
    FakeBus bus;
    BME280::BME280 dev;
    Config cfg = makeConfig(bus);
    cfg.nvmReadyTimeoutMs = invalidValues[i];

    const Status st = dev.begin(cfg);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_CONFIG),
                            static_cast<uint8_t>(st.code));
    TEST_ASSERT_EQUAL_UINT32(0u, totalBusCalls(bus));
    TEST_ASSERT_FALSE(dev.isInitialized());
  }
}

void test_begin_rejects_invalid_conversion_timeout_without_i2c() {
  const uint32_t invalidValues[] = {0u, 0x7FFFFFFFUL, 0x80000000UL};
  for (size_t i = 0; i < 3; ++i) {
    FakeBus bus;
    BME280::BME280 dev;
    Config cfg = makeConfig(bus);
    cfg.conversionReadyTimeoutMs = invalidValues[i];

    const Status st = dev.begin(cfg);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_CONFIG),
                            static_cast<uint8_t>(st.code));
    TEST_ASSERT_EQUAL_UINT32(0u, totalBusCalls(bus));
    TEST_ASSERT_FALSE(dev.isInitialized());
  }
}

void test_begin_rejects_invalid_i2c_timeout_without_i2c() {
  const uint32_t invalidValues[] = {0u, 0x7FFFFFFFUL, 0x80000000UL};
  for (size_t i = 0; i < 3; ++i) {
    FakeBus bus;
    BME280::BME280 dev;
    Config cfg = makeConfig(bus);
    cfg.i2cTimeoutMs = invalidValues[i];

    const Status st = dev.begin(cfg);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_CONFIG),
                            static_cast<uint8_t>(st.code));
    TEST_ASSERT_EQUAL_UINT32(0u, totalBusCalls(bus));
    TEST_ASSERT_FALSE(dev.isInitialized());
  }
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
  TEST_ASSERT_FALSE(dev.lastOkTimeValid());
  TEST_ASSERT_FALSE(dev.lastErrorTimeValid());
  TEST_ASSERT_EQUAL_UINT32(readsBefore, bus.readCalls);
  TEST_ASSERT_EQUAL_UINT32(writesBefore, bus.writeCalls);

  const Config& stored = dev.getConfig();
  TEST_ASSERT_NULL(stored.i2cWrite);
  TEST_ASSERT_NULL(stored.i2cWriteRead);
  TEST_ASSERT_EQUAL_HEX8(0x76, stored.i2cAddress);
  TEST_ASSERT_EQUAL_UINT32(50u, stored.i2cTimeoutMs);
  TEST_ASSERT_EQUAL_UINT32(10u, stored.nvmReadyTimeoutMs);
  TEST_ASSERT_EQUAL_UINT32(20u, stored.conversionReadyTimeoutMs);
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
  TEST_ASSERT_FALSE(dev.lastOkTimeValid());
  TEST_ASSERT_FALSE(dev.lastErrorTimeValid());
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
  bus.readError = TransportResult::Error(TransportErr::TIMEOUT, -70);
  uint8_t status = 0;
  Status st = dev.readStatus(status);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_TIMEOUT),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_UINT32(1u, dev.totalFailures());
  TEST_ASSERT_EQUAL_UINT8(1u, dev.consecutiveFailures());
  TEST_ASSERT_EQUAL_UINT32(1234u, dev.lastErrorMs());
  TEST_ASSERT_TRUE(dev.lastErrorTimeValid());
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
  TEST_ASSERT_FALSE(dev.lastErrorTimeValid());
  TEST_ASSERT_FALSE(dev.lastOkTimeValid());
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

void test_soft_reset_job_failure_after_reset_marks_dirty() {
  FakeBus bus;
  BME280::BME280 dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  bus.failReadReg = cmd::REG_STATUS;
  bus.failReadRegRemaining = 1;
  bus.readError = TransportResult::Error(TransportErr::TIMEOUT, -93);

  Status st = dev.startSoftResetJob();
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
  TEST_ASSERT_FALSE(dev.lastOkTimeValid());

  bus.readErrorRemaining = 1;
  bus.readError = TransportResult::Error(TransportErr::TIMEOUT, -175);
  uint8_t status = 0;
  st = dev.readStatus(status);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_TIMEOUT),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_UINT32(0u, dev.lastErrorMs());
  TEST_ASSERT_FALSE(dev.lastErrorTimeValid());
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
  TEST_ASSERT_FALSE(dev.lastOkTimeValid());
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
  bus.readError = TransportResult::Error(TransportErr::OTHER, -7);
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
  bus.readError = TransportResult::Error(TransportErr::NACK_ADDRESS, 2);
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
  bus.readError = TransportResult::Error(TransportErr::NACK_ADDRESS, -17);
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

  struct MappingCase {
    TransportErr transport;
    Err driver;
  };
  const MappingCase cases[] = {
      {TransportErr::TIMEOUT, Err::I2C_TIMEOUT},
      {TransportErr::BUS, Err::I2C_BUS},
      {TransportErr::NACK_DATA, Err::I2C_NACK_DATA},
      {TransportErr::OTHER, Err::I2C_ERROR},
  };
  for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
    bus.readErrorRemaining = 1;
    bus.readError = TransportResult::Error(
        cases[i].transport,
        static_cast<int32_t>(-60 - static_cast<int32_t>(i)));
    Status st = dev.probe();
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(cases[i].driver),
                            static_cast<uint8_t>(st.code));
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
  bus.readError = TransportResult::Error(TransportErr::NACK_ADDRESS, -22);
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
    bus.readError = TransportResult::Error(TransportErr::NACK_ADDRESS, -23);
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
    bus.readError = TransportResult::Error(TransportErr::NACK_ADDRESS, -24);
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
    bus.writeError = TransportResult::Error(TransportErr::NACK_ADDRESS, -25);
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
  bus.readError = TransportResult::Error(TransportErr::TIMEOUT, -21);
  BME280::BME280 dev;

  Status st = dev.begin(makeConfig(bus));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_TIMEOUT),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_INT32(-21, st.detail);
  TEST_ASSERT_FALSE(dev.isInitialized());
}

void test_begin_preserves_chip_id_bus_and_data_errors() {
  struct MappingCase {
    TransportErr transport;
    Err driver;
  };
  const MappingCase cases[] = {
      {TransportErr::BUS, Err::I2C_BUS},
      {TransportErr::NACK_DATA, Err::I2C_NACK_DATA},
      {TransportErr::OTHER, Err::I2C_ERROR},
  };
  for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
    FakeBus bus;
    bus.readErrorRemaining = 1;
    bus.readError = TransportResult::Error(
        cases[i].transport,
        static_cast<int32_t>(-70 - static_cast<int32_t>(i)));
    BME280::BME280 dev;

    Status st = dev.begin(makeConfig(bus));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(cases[i].driver),
                            static_cast<uint8_t>(st.code));
    TEST_ASSERT_EQUAL_INT32(static_cast<int32_t>(-70 - static_cast<int32_t>(i)), st.detail);
    TEST_ASSERT_FALSE(dev.isInitialized());
  }
}

void test_recover_failure_updates_health_once() {
  FakeBus bus;
  BME280::BME280 dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  bus.readErrorRemaining = 1;
  bus.readError = TransportResult::Error(TransportErr::OTHER, -8);
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
  bus.readError = TransportResult::Error(TransportErr::OTHER, -9);
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
  bus.readError = TransportResult::Error(TransportErr::NACK_ADDRESS, 7);
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
  bus.writeError = TransportResult::Error(TransportErr::TIMEOUT, -41);
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
  bus.writeError = TransportResult::Error(TransportErr::NACK_DATA, -42);
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
  bus.writeError = TransportResult::Error(TransportErr::BUS, -43);
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
  bus.writeError = TransportResult::Error(TransportErr::TIMEOUT, -44);
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
  bus.writeError = TransportResult::Error(TransportErr::TIMEOUT, -31);

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
  bus.writeError = TransportResult::Error(TransportErr::BUS, -32);
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
  bus.writeError = TransportResult::Error(TransportErr::TIMEOUT, -52);
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
  bus.writeError = TransportResult::Error(TransportErr::BUS, -35);
  Status st = dev.setFilter(Filter::X2);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_BUS),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_TRUE(dev.hardwareConfigDirty());
  TEST_ASSERT_EQUAL_INT32(-35, dev.hardwareConfigDirtyError().detail);
  TEST_ASSERT_TRUE(dev.hasSample());

  bus.failWriteOnCall = bus.writeCalls + 2u;  // recover sleep write succeeds, config write fails
  bus.writeError = TransportResult::Error(TransportErr::TIMEOUT, -36);
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

void test_resync_job_failure_preserves_cached_sample_until_successful_resync() {
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
  bus.readError = TransportResult::Error(TransportErr::TIMEOUT, -126);
  TEST_ASSERT_TRUE(dev.startResyncJob().inProgress());
  JobPollResult result = pollWithBudget(dev, bus, 4);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(JobState::FAILED),
                          static_cast<uint8_t>(result.state));
  TEST_ASSERT_FALSE(dev.hardwareConfigDirty());
  TEST_ASSERT_TRUE(dev.hasSample());
  TEST_ASSERT_EQUAL_UINT32(sampleTimestamp, dev.sampleTimestampMs());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(SampleFreshness::FRESH),
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
  bus.writeError = TransportResult::Error(TransportErr::BUS, -34);
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
  bus.writeError = TransportResult::Error(TransportErr::NACK_DATA, -33);
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
  bus.writeError = TransportResult::Error(TransportErr::TIMEOUT, -80);
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
  bus.writeError = TransportResult::Error(TransportErr::NACK_ADDRESS, -91);
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
  bus.readError = TransportResult::Error(TransportErr::TIMEOUT, -81);
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
  bus.readError = TransportResult::Error(TransportErr::NACK_DATA, -89);
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
  bus.writeError = TransportResult::Error(TransportErr::BUS, -90);
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
  bus.readError = TransportResult::Error(TransportErr::NACK_DATA, -82);
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
  bus.readError = TransportResult::Error(TransportErr::TIMEOUT, -92);

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

  setBoschSyntheticCalibration(bus);

  bus.failReadRegEnabled = true;
  bus.failReadReg = cmd::REG_CALIB_H_START;
  bus.readError = TransportResult::Error(TransportErr::BUS, -83);
  Status st = dev.softReset();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_BUS),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_TRUE(dev.hardwareConfigDirty());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_BUS),
                          static_cast<uint8_t>(dev.hardwareConfigDirtyError().code));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::DEGRADED),
                          static_cast<uint8_t>(dev.state()));

  Calibration after{};
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::RESYNC_REQUIRED),
                          static_cast<uint8_t>(dev.getCalibration(after).code));
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
  bus.writeError = TransportResult::Error(TransportErr::TIMEOUT, -84);
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
  bus.writeError = TransportResult::Error(TransportErr::BUS, -85);
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
  bus.readError = TransportResult::Error(TransportErr::TIMEOUT, -86);
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
  TransportResult st = transport::wireWrite(0x76, &byte, 1, 123, &Wire);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(TransportErr::NACK_ADDRESS),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_INT32(2, st.detail);
  TEST_ASSERT_EQUAL_UINT32(77u, Wire.getTimeOut());

  Wire._setEndTransmissionResult(3);
  st = transport::wireWrite(0x76, &byte, 1, 999, &Wire);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(TransportErr::NACK_DATA),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_INT32(3, st.detail);
  TEST_ASSERT_EQUAL_UINT32(77u, Wire.getTimeOut());

  Wire._setEndTransmissionResult(4);
  st = transport::wireWrite(0x76, &byte, 1, 999, &Wire);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(TransportErr::BUS),
                          static_cast<uint8_t>(st.code));

  Wire._setEndTransmissionResult(5);
  st = transport::wireWrite(0x76, &byte, 1, 999, &Wire);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(TransportErr::TIMEOUT),
                          static_cast<uint8_t>(st.code));

  Wire._setEndTransmissionResult(1);
  st = transport::wireWrite(0x76, &byte, 1, 999, &Wire);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(TransportErr::OTHER),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_INT32(1, st.detail);
}

void test_example_transport_validates_params_and_handles_write_read() {
  const uint8_t tx = 0x00;
  uint8_t rx = 0;

  TransportResult st = transport::wireWrite(0x76, nullptr, 1, 50, nullptr);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(TransportErr::OTHER),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_INT32(-1, st.detail);

  st = transport::wireWrite(0x76, &tx, 0, 50, &Wire);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(TransportErr::OTHER),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_INT32(-2, st.detail);

  st = transport::wireWriteRead(0x76, nullptr, 1, &rx, 1, 50, &Wire);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(TransportErr::OTHER),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_INT32(-2, st.detail);

  st = transport::wireWriteRead(0x76, &tx, 1, nullptr, 1, 50, &Wire);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(TransportErr::OTHER),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_INT32(-2, st.detail);

  Wire._setEndTransmissionResult(0);
  Wire._setRequestFromResult(1);
  st = transport::wireWriteRead(0x76, &tx, 1, &rx, 1, 50, &Wire);
  TEST_ASSERT_TRUE(st.ok());
  TEST_ASSERT_EQUAL_UINT32(1u, static_cast<uint32_t>(st.writeCount));
  TEST_ASSERT_EQUAL_UINT32(1u, static_cast<uint32_t>(st.readCount));

  Wire._setRequestFromResult(0);
  st = transport::wireWriteRead(0x76, &tx, 1, &rx, 1, 50, &Wire);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(TransportErr::OK),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_UINT32(1u, static_cast<uint32_t>(st.writeCount));
  TEST_ASSERT_EQUAL_UINT32(0u, static_cast<uint32_t>(st.readCount));
}

void test_recover_reaches_offline_when_threshold_is_one() {
  FakeBus bus;
  BME280::BME280 dev;
  Config cfg = makeConfig(bus);
  cfg.offlineThreshold = 1;
  TEST_ASSERT_TRUE(dev.begin(cfg).ok());

  bus.readErrorRemaining = 1;
  bus.readError = TransportResult::Error(TransportErr::OTHER, -10);
  Status st = dev.recover();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_ERROR), static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::OFFLINE),
                          static_cast<uint8_t>(dev.state()));
  TEST_ASSERT_FALSE(dev.isOnline());
}

void test_offline_history_does_not_block_public_register_read() {
  FakeBus bus;
  BME280::BME280 dev;
  Config cfg = makeConfig(bus);
  cfg.offlineThreshold = 1;
  TEST_ASSERT_TRUE(dev.begin(cfg).ok());

  bus.readErrorRemaining = 1;
  bus.readError = TransportResult::Error(TransportErr::OTHER, -11);
  Status st = dev.recover();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_ERROR), static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::OFFLINE),
                          static_cast<uint8_t>(dev.state()));

  const uint32_t readsBefore = bus.readCalls;
  const uint32_t successesBefore = dev.totalSuccess();
  const uint32_t failuresBefore = dev.totalFailures();
  uint8_t value = 0;
  st = dev.readRegister(cmd::REG_CHIP_ID, value);
  TEST_ASSERT_TRUE(st.ok());
  TEST_ASSERT_EQUAL_HEX8(cmd::CHIP_ID_BME280, value);
  TEST_ASSERT_EQUAL_UINT32(readsBefore + 1u, bus.readCalls);
  TEST_ASSERT_EQUAL_UINT32(successesBefore + 1u, dev.totalSuccess());
  TEST_ASSERT_EQUAL_UINT32(failuresBefore, dev.totalFailures());
  TEST_ASSERT_EQUAL_UINT8(0u, dev.consecutiveFailures());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::READY),
                          static_cast<uint8_t>(dev.state()));
}

void test_offline_history_does_not_block_typed_config_setters() {
  FakeBus bus;
  BME280::BME280 dev;
  Config cfg = makeConfig(bus);
  cfg.offlineThreshold = 1;
  TEST_ASSERT_TRUE(dev.begin(cfg).ok());

  bus.readErrorRemaining = 1;
  bus.readError = TransportResult::Error(TransportErr::OTHER, -111);
  Status st = dev.recover();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_ERROR), static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::OFFLINE),
                          static_cast<uint8_t>(dev.state()));
  TEST_ASSERT_FALSE(dev.hardwareConfigDirty());

  const uint32_t callsBefore = totalBusCalls(bus);
  const uint32_t successesBefore = dev.totalSuccess();
  const uint32_t failuresBefore = dev.totalFailures();
  st = dev.setMode(Mode::SLEEP);
  TEST_ASSERT_TRUE(st.ok());
  st = dev.setOversamplingT(Oversampling::X2);
  TEST_ASSERT_TRUE(st.ok());
  st = dev.setOversamplingP(Oversampling::X2);
  TEST_ASSERT_TRUE(st.ok());
  st = dev.setOversamplingH(Oversampling::X2);
  TEST_ASSERT_TRUE(st.ok());

  TEST_ASSERT_EQUAL_UINT32(callsBefore + 5u, totalBusCalls(bus));
  TEST_ASSERT_EQUAL_UINT32(successesBefore + 5u, dev.totalSuccess());
  TEST_ASSERT_EQUAL_UINT32(failuresBefore, dev.totalFailures());
  TEST_ASSERT_EQUAL_UINT8(0u, dev.consecutiveFailures());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::READY),
                          static_cast<uint8_t>(dev.state()));
  TEST_ASSERT_FALSE(dev.hardwareConfigDirty());
}

void test_probe_works_while_offline_without_clearing_latch() {
  FakeBus bus;
  BME280::BME280 dev;
  Config cfg = makeConfig(bus);
  cfg.offlineThreshold = 1;
  TEST_ASSERT_TRUE(dev.begin(cfg).ok());

  bus.readErrorRemaining = 1;
  bus.readError = TransportResult::Error(TransportErr::OTHER, -87);
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
  bus.readError = TransportResult::Error(TransportErr::OTHER, -88);
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

void test_failed_recover_from_offline_reports_current_health_observation() {
  FakeBus bus;
  BME280::BME280 dev;
  Config cfg = makeConfig(bus);
  cfg.offlineThreshold = 3;
  TEST_ASSERT_TRUE(dev.begin(cfg).ok());

  for (uint8_t i = 0; i < cfg.offlineThreshold; ++i) {
    bus.readErrorRemaining = 1;
    bus.readError = TransportResult::Error(TransportErr::OTHER, -12);
    (void)dev.recover();
  }
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::OFFLINE),
                          static_cast<uint8_t>(dev.state()));

  bus.writeErrorRemaining = 1;
  bus.writeError = TransportResult::Error(TransportErr::OTHER, -13);
  Status st = dev.recover();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_ERROR),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::DEGRADED),
                          static_cast<uint8_t>(dev.state()));
  TEST_ASSERT_EQUAL_UINT8(1u, dev.consecutiveFailures());
  TEST_ASSERT_EQUAL_UINT32(static_cast<uint32_t>(cfg.offlineThreshold) + 1u,
                           dev.totalFailures());

  const uint32_t readsBefore = bus.readCalls;
  uint8_t value = 0;
  st = dev.readRegister(cmd::REG_CHIP_ID, value);
  TEST_ASSERT_TRUE(st.ok());
  TEST_ASSERT_EQUAL_HEX8(cmd::CHIP_ID_BME280, value);
  TEST_ASSERT_EQUAL_UINT32(readsBefore + 1u, bus.readCalls);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::READY),
                          static_cast<uint8_t>(dev.state()));
  TEST_ASSERT_EQUAL_UINT8(0u, dev.consecutiveFailures());
}

void test_recovery_job_from_offline_clears_latch_and_allows_i2c() {
  FakeBus bus;
  BME280::BME280 dev;
  Config cfg = makeConfig(bus);
  cfg.offlineThreshold = 1;
  TEST_ASSERT_TRUE(dev.begin(cfg).ok());

  bus.readErrorRemaining = 1;
  bus.readError = TransportResult::Error(TransportErr::OTHER, -120);
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
  bus.readError = TransportResult::Error(TransportErr::OTHER, -121);
  (void)dev.recover();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::OFFLINE),
                          static_cast<uint8_t>(dev.state()));

  bus.failReadRegEnabled = true;
  bus.failReadReg = cmd::REG_CHIP_ID;
  bus.readError = TransportResult::Error(TransportErr::TIMEOUT, -122);
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

void test_recovery_job_from_offline_updates_health_before_job_completion() {
  FakeBus bus;
  BME280::BME280 dev;
  Config cfg = makeConfig(bus);
  cfg.offlineThreshold = 1;
  TEST_ASSERT_TRUE(dev.begin(cfg).ok());

  bus.readErrorRemaining = 1;
  bus.readError = TransportResult::Error(TransportErr::OTHER, -125);
  (void)dev.recover();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::OFFLINE),
                          static_cast<uint8_t>(dev.state()));
  TEST_ASSERT_TRUE(dev.consecutiveFailures() >= cfg.offlineThreshold);

  TEST_ASSERT_TRUE(dev.startRecoveryJob().inProgress());
  JobPollResult result = pollWithBudget(dev, bus, 1);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(JobState::RUNNING),
                          static_cast<uint8_t>(result.state));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::READY),
                          static_cast<uint8_t>(dev.state()));

  result = pollWithBudget(dev, bus, 1);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(JobState::RUNNING),
                          static_cast<uint8_t>(result.state));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::READY),
                          static_cast<uint8_t>(dev.state()));
  TEST_ASSERT_EQUAL_UINT8(0u, dev.consecutiveFailures());

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

void test_offline_history_does_not_block_staged_measurement_job() {
  FakeBus bus;
  BME280::BME280 dev;
  Config cfg = makeConfig(bus);
  cfg.offlineThreshold = 1;
  TEST_ASSERT_TRUE(dev.begin(cfg).ok());

  bus.readErrorRemaining = 1;
  bus.readError = TransportResult::Error(TransportErr::OTHER, -123);
  (void)dev.recover();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::OFFLINE),
                          static_cast<uint8_t>(dev.state()));

  const uint32_t callsBefore = totalBusCalls(bus);
  const uint32_t successesBefore = dev.totalSuccess();
  const uint32_t failuresBefore = dev.totalFailures();
  Status st = dev.startForcedMeasurementJob();
  TEST_ASSERT_TRUE(st.inProgress());
  TEST_ASSERT_EQUAL_UINT32(callsBefore, totalBusCalls(bus));

  const JobPollResult result = pollWithBudget(dev, bus, 1);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(JobState::WAITING),
                          static_cast<uint8_t>(result.state));
  TEST_ASSERT_EQUAL_UINT8(1u, result.instructionsUsed);
  TEST_ASSERT_EQUAL_UINT32(callsBefore + 1u, totalBusCalls(bus));
  TEST_ASSERT_EQUAL_UINT32(successesBefore + 1u, dev.totalSuccess());
  TEST_ASSERT_EQUAL_UINT32(failuresBefore, dev.totalFailures());
  TEST_ASSERT_EQUAL_UINT8(0u, dev.consecutiveFailures());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::READY),
                          static_cast<uint8_t>(dev.state()));
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
  bus.readError = TransportResult::Error(TransportErr::TIMEOUT, -70);
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
  bus.writeError = TransportResult::Error(TransportErr::BUS, -71);
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
  bus.readError = TransportResult::Error(TransportErr::NACK_DATA, -55);
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
  cfg.i2cTimeoutMs = 2;
  cfg.conversionReadyTimeoutMs = 10;
  TEST_ASSERT_TRUE(dev.begin(cfg).ok());

  Status st = dev.requestMeasurement();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::IN_PROGRESS),
                          static_cast<uint8_t>(st.code));

  bus.reg[cmd::REG_STATUS] = cmd::MASK_STATUS_MEASURING;
  bus.nowMs += dev.estimateMeasurementTimeMs() +
               cfg.conversionReadyTimeoutMs + 1U;
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
  bus.readError = TransportResult::Error(TransportErr::NACK_DATA, -31);
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
  const uint8_t writeLogStart = bus.writeLogLen;

  Status st = dev.startForcedMeasurementJob();
  TEST_ASSERT_TRUE(st.inProgress());
  const uint32_t jobId = dev.jobId();
  TEST_ASSERT_NOT_EQUAL(0u, jobId);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::IN_PROGRESS),
                          static_cast<uint8_t>(dev.lastMeasurementStatus().code));
  TEST_ASSERT_EQUAL_STRING(toString(Err::IN_PROGRESS),
                           dev.lastMeasurementStatus().msg);

  JobPollResult result = pollWithBudget(dev, bus, 0);
  TEST_ASSERT_EQUAL_UINT32(jobId, result.jobId);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(JobPhase::FORCE_TRIGGER),
                          static_cast<uint8_t>(result.phase));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(JobState::RUNNING),
                          static_cast<uint8_t>(result.state));
  TEST_ASSERT_FALSE(result.phaseDeadlineActive);

  result = pollWithBudget(dev, bus, 1);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(JobState::WAITING),
                          static_cast<uint8_t>(result.state));
  TEST_ASSERT_EQUAL_UINT32(jobId, result.jobId);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(JobKind::FORCED_MEASUREMENT),
                          static_cast<uint8_t>(result.kind));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(JobPhase::FORCE_WAIT_TIME),
                          static_cast<uint8_t>(result.phase));
  TEST_ASSERT_TRUE(result.phaseDeadlineActive);
  TEST_ASSERT_EQUAL_HEX8(cmd::REG_CTRL_MEAS, bus.lastWriteReg);
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
  TEST_ASSERT_EQUAL_UINT32(
      0u, countWritesToRegSince(bus, cmd::REG_CTRL_HUM, writeLogStart));
  TEST_ASSERT_EQUAL_UINT32(
      1u, countWritesToRegSince(bus, cmd::REG_CTRL_MEAS, writeLogStart));

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
  TEST_ASSERT_EQUAL_STRING(toString(Err::IN_PROGRESS),
                           dev.lastMeasurementStatus().msg);

  const JobPollResult result = pollWithBudget(dev, bus, 1);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(JobState::WAITING),
                          static_cast<uint8_t>(result.state));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::IN_PROGRESS),
                          static_cast<uint8_t>(dev.lastMeasurementStatus().code));
  TEST_ASSERT_EQUAL_STRING(toString(Err::IN_PROGRESS),
                           dev.lastMeasurementStatus().msg);
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

  bus.nowMs += dev.estimateMeasurementTimeMs();
  bus.failReadRegEnabled = true;
  bus.failReadReg = cmd::REG_DATA_START;
  bus.readError = TransportResult::Error(TransportErr::NACK_DATA, -124);
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

  bus.nowMs += dev.estimateMeasurementTimeMs();
  bus.failReadReg = cmd::REG_STATUS;
  bus.failReadRegRemaining = 1;
  bus.readError = TransportResult::Error(TransportErr::TIMEOUT, -95);
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
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(ConversionState::UNKNOWN_AFTER_TRIGGER_ERROR),
      static_cast<uint8_t>(snap.conversionState));

  TEST_ASSERT_TRUE(dev.startForcedMeasurementJob().inProgress());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(JobPhase::FORCE_RECONCILE_STATUS),
                          static_cast<uint8_t>(dev.jobPhase()));
}

void test_forced_measurement_job_stuck_measuring_times_out_without_raw_read() {
  FakeBus bus;
  setBoschSyntheticCalibration(bus);
  setRawSample(bus, 415148, 519888, 30000);
  BME280::BME280 dev;
  Config cfg = makeConfig(bus);
  cfg.mode = Mode::FORCED;
  cfg.i2cTimeoutMs = 2;
  cfg.conversionReadyTimeoutMs = 10;
  TEST_ASSERT_TRUE(dev.begin(cfg).ok());

  TEST_ASSERT_TRUE(dev.startForcedMeasurementJob().inProgress());
  (void)pollWithBudget(dev, bus, 1);

  bus.nowMs += dev.estimateMeasurementTimeMs();
  bus.reg[cmd::REG_STATUS] = cmd::MASK_STATUS_MEASURING;
  JobPollResult result = pollWithBudget(dev, bus, 4);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(JobState::WAITING),
                          static_cast<uint8_t>(result.state));
  TEST_ASSERT_EQUAL_HEX8(cmd::REG_STATUS, bus.lastReadReg);

  bus.nowMs += cfg.conversionReadyTimeoutMs + 1U;
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
  TEST_ASSERT_EQUAL_UINT32(9u, instructionsTotal);
  TEST_ASSERT_EQUAL_UINT32(callsBefore + 9u, totalBusCalls(bus));
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
  TEST_ASSERT_FALSE(dev.hardwareConfigDirty());
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
  TEST_ASSERT_FALSE(dev.hardwareConfigDirty());
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
  bus.readError = TransportResult::Error(TransportErr::BUS, -94);

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
  TEST_ASSERT_FALSE(dev.hardwareConfigDirty());
}

void test_recovery_job_error_stops_without_extra_instructions() {
  FakeBus bus;
  BME280::BME280 dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  bus.writeErrorRemaining = 1;
  bus.writeError = TransportResult::Error(TransportErr::BUS, -55);
  TEST_ASSERT_TRUE(dev.startSoftResetJob().inProgress());

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
  bus.writeError = TransportResult::Error(TransportErr::TIMEOUT, -141);
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
  bus.writeError = TransportResult::Error(TransportErr::TIMEOUT, -142);
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
  TEST_ASSERT_TRUE(dev.startResyncJob().inProgress());
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

void test_cancel_result_is_zero_i2c_exactly_once_and_blocks_until_retrieved() {
  const CancelReason reasons[] = {
      CancelReason::OWNER_REQUEST, CancelReason::DEADLINE_EXPIRED};
  const Err expectedErrors[] = {Err::CANCELLED, Err::DEADLINE_EXPIRED};
  const JobState expectedStates[] = {JobState::CANCELLED, JobState::TIMED_OUT};

  for (size_t i = 0; i < 2; ++i) {
    FakeBus bus;
    BME280::BME280 dev;
    const Config cfg = makeConfig(bus);
    TEST_ASSERT_TRUE(dev.begin(cfg).ok());
    const uint32_t callsBeforeStart = totalBusCalls(bus);
    TEST_ASSERT_TRUE(dev.startApplyConfigJob().inProgress());
    const uint32_t cancelledJobId = dev.jobId();
    TEST_ASSERT_NOT_EQUAL(0u, cancelledJobId);
    TEST_ASSERT_EQUAL_UINT32(callsBeforeStart, totalBusCalls(bus));

    const Status cancellation = dev.cancelJob(reasons[i]);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(expectedErrors[i]),
                            static_cast<uint8_t>(cancellation.code));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(expectedStates[i]),
                            static_cast<uint8_t>(dev.jobState()));
    TEST_ASSERT_EQUAL_UINT32(callsBeforeStart, totalBusCalls(bus));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ConfigSyncState::SYNCHRONIZED),
                            static_cast<uint8_t>(dev.configSyncState()));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(CalibrationState::VALID),
                            static_cast<uint8_t>(dev.calibrationState()));

    assertAllHardwareApisBusy(dev, bus, cfg);
    assertBusyReason(dev.invalidateDeviceState(),
                     BusyReason::TERMINAL_RESULT_PENDING);
    assertBusyReason(dev.startInitJob(cfg),
                     BusyReason::TERMINAL_RESULT_PENDING);
    assertBusyReason(dev.startForcedMeasurementJob(),
                     BusyReason::TERMINAL_RESULT_PENDING);
    assertBusyReason(dev.startApplyConfigJob(),
                     BusyReason::TERMINAL_RESULT_PENDING);
    assertBusyReason(dev.startResyncJob(),
                     BusyReason::TERMINAL_RESULT_PENDING);
    assertBusyReason(dev.startRecoveryJob(),
                     BusyReason::TERMINAL_RESULT_PENDING);
    assertBusyReason(dev.startSoftResetJob(),
                     BusyReason::TERMINAL_RESULT_PENDING);
    TEST_ASSERT_EQUAL_UINT32(callsBeforeStart, totalBusCalls(bus));

    JobPollResult terminal = pollWithBudget(dev, bus, 0);
    TEST_ASSERT_EQUAL_UINT32(cancelledJobId, terminal.jobId);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(JobKind::APPLY_CONFIG),
                            static_cast<uint8_t>(terminal.kind));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(JobPhase::APPLY_WAIT_IDLE),
                            static_cast<uint8_t>(terminal.phase));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(expectedStates[i]),
                            static_cast<uint8_t>(terminal.state));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(expectedErrors[i]),
                            static_cast<uint8_t>(terminal.status.code));

    const JobPollResult idle = pollWithBudget(dev, bus, 0);
    TEST_ASSERT_EQUAL_UINT32(0u, idle.jobId);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(JobKind::NONE),
                            static_cast<uint8_t>(idle.kind));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(JobState::IDLE),
                            static_cast<uint8_t>(idle.state));
    TEST_ASSERT_TRUE(dev.startForcedMeasurementJob().inProgress());
    TEST_ASSERT_NOT_EQUAL(cancelledJobId, dev.jobId());
  }
}

void test_cancellation_preserves_last_good_state_across_mutating_phases() {
  {
    FakeBus bus;
    setBoschSyntheticCalibration(bus);
    setRawSample(bus, 415148, 519888, 30000);
    BME280::BME280 dev;
    TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());
    captureForcedSample(dev, bus);
    SampleEnvelope before{};
    TEST_ASSERT_TRUE(dev.getSampleEnvelope(before).ok());

    TEST_ASSERT_TRUE(dev.startApplyConfigJob().inProgress());
    (void)pollUntilPhase(dev, bus, JobPhase::APPLY_WAIT_AFTER_SLEEP);
    const uint32_t callsBeforeCancel = totalBusCalls(bus);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::CANCELLED),
                            static_cast<uint8_t>(
                                dev.cancelJob(CancelReason::OWNER_REQUEST).code));
    TEST_ASSERT_EQUAL_UINT32(callsBeforeCancel, totalBusCalls(bus));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ConfigSyncState::RESYNC_REQUIRED),
                            static_cast<uint8_t>(dev.configSyncState()));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(CalibrationState::VALID),
                            static_cast<uint8_t>(dev.calibrationState()));
    SampleEnvelope after{};
    TEST_ASSERT_TRUE(dev.getSampleEnvelope(after).ok());
    assertSampleEnvelopeEqual(before, after);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(SampleFreshness::STALE_AFTER_CONFIG_DIRTY),
        static_cast<uint8_t>(dev.sampleFreshness()));
    (void)pollWithBudget(dev, bus, 0);
  }

  {
    FakeBus bus;
    setBoschSyntheticCalibration(bus);
    setRawSample(bus, 415148, 519888, 30000);
    BME280::BME280 dev;
    TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());
    captureForcedSample(dev, bus);
    SampleEnvelope before{};
    TEST_ASSERT_TRUE(dev.getSampleEnvelope(before).ok());
    const uint8_t writeLogStart = bus.writeLogLen;

    TEST_ASSERT_TRUE(dev.startSoftResetJob().inProgress());
    const JobPollResult afterReset = pollWithBudget(dev, bus, 1);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(JobPhase::RESYNC_READ_CHIP_ID),
                            static_cast<uint8_t>(afterReset.phase));
    const uint32_t callsBeforeCancel = totalBusCalls(bus);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::DEADLINE_EXPIRED),
        static_cast<uint8_t>(
            dev.cancelJob(CancelReason::DEADLINE_EXPIRED).code));
    TEST_ASSERT_EQUAL_UINT32(callsBeforeCancel, totalBusCalls(bus));
    TEST_ASSERT_EQUAL_UINT32(
        1u, countWritesToRegSince(bus, cmd::REG_RESET, writeLogStart));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ConfigSyncState::RESYNC_REQUIRED),
                            static_cast<uint8_t>(dev.configSyncState()));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(CalibrationState::INVALID),
                            static_cast<uint8_t>(dev.calibrationState()));
    SampleEnvelope after{};
    TEST_ASSERT_TRUE(dev.getSampleEnvelope(after).ok());
    assertSampleEnvelopeEqual(before, after);
    (void)pollWithBudget(dev, bus, 0);
  }

  {
    FakeBus bus;
    setBoschSyntheticCalibration(bus);
    setRawSample(bus, 415148, 519888, 30000);
    BME280::BME280 dev;
    TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());
    captureForcedSample(dev, bus);
    SampleEnvelope before{};
    TEST_ASSERT_TRUE(dev.getSampleEnvelope(before).ok());
    const uint8_t writeLogStart = bus.writeLogLen;

    TEST_ASSERT_TRUE(dev.startForcedMeasurementJob().inProgress());
    const JobPollResult afterTrigger = pollWithBudget(dev, bus, 1);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(JobPhase::FORCE_WAIT_TIME),
                            static_cast<uint8_t>(afterTrigger.phase));
    const uint32_t callsBeforeCancel = totalBusCalls(bus);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::CANCELLED),
                            static_cast<uint8_t>(
                                dev.cancelJob(CancelReason::OWNER_REQUEST).code));
    TEST_ASSERT_EQUAL_UINT32(callsBeforeCancel, totalBusCalls(bus));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ConfigSyncState::SYNCHRONIZED),
                            static_cast<uint8_t>(dev.configSyncState()));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(CalibrationState::VALID),
                            static_cast<uint8_t>(dev.calibrationState()));
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(ConversionState::UNKNOWN_AFTER_TRIGGER_ERROR),
        static_cast<uint8_t>(dev.conversionState()));
    TEST_ASSERT_EQUAL_UINT32(
        0u, countWritesToRegSince(bus, cmd::REG_CTRL_HUM, writeLogStart));
    TEST_ASSERT_EQUAL_UINT32(
        1u, countWritesToRegSince(bus, cmd::REG_CTRL_MEAS, writeLogStart));
    SampleEnvelope after{};
    TEST_ASSERT_TRUE(dev.getSampleEnvelope(after).ok());
    assertSampleEnvelopeEqual(before, after);
    (void)pollWithBudget(dev, bus, 0);
  }
}

void test_resync_never_resets_and_soft_reset_has_exact_payload() {
  FakeBus bus;
  BME280::BME280 dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  uint8_t writeLogStart = bus.writeLogLen;
  uint32_t callsBefore = totalBusCalls(bus);
  TEST_ASSERT_TRUE(dev.startResyncJob().inProgress());
  JobPollResult result = pollUntilTerminal(dev, bus, 1);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(JobState::DONE),
                          static_cast<uint8_t>(result.state));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(JobKind::RESYNC),
                          static_cast<uint8_t>(result.kind));
  TEST_ASSERT_EQUAL_UINT32(callsBefore + 9u, totalBusCalls(bus));
  TEST_ASSERT_EQUAL_UINT32(
      0u, countWritesToRegSince(bus, cmd::REG_RESET, writeLogStart));

  writeLogStart = bus.writeLogLen;
  TEST_ASSERT_TRUE(dev.startRecoveryJob().inProgress());
  result = pollUntilTerminal(dev, bus, 1);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(JobKind::RESYNC),
                          static_cast<uint8_t>(result.kind));
  TEST_ASSERT_EQUAL_UINT32(
      0u, countWritesToRegSince(bus, cmd::REG_RESET, writeLogStart));

  writeLogStart = bus.writeLogLen;
  TEST_ASSERT_TRUE(dev.startSoftResetJob().inProgress());
  result = pollWithBudget(dev, bus, 1);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(JobKind::SOFT_RESET),
                          static_cast<uint8_t>(result.kind));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(JobPhase::RESYNC_READ_CHIP_ID),
                          static_cast<uint8_t>(result.phase));
  TEST_ASSERT_EQUAL_HEX8(cmd::REG_RESET, bus.writeRegLog[writeLogStart]);
  TEST_ASSERT_EQUAL_HEX8(cmd::RESET_VALUE, bus.writeValueLog[writeLogStart]);
  result = pollUntilTerminal(dev, bus, 1);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(JobState::DONE),
                          static_cast<uint8_t>(result.state));
  TEST_ASSERT_EQUAL_UINT32(
      1u, countWritesToRegSince(bus, cmd::REG_RESET, writeLogStart));
}

void test_ambiguous_trigger_reconciles_before_any_new_trigger() {
  FakeBus bus;
  BME280::BME280 dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());
  const uint8_t writeLogStart = bus.writeLogLen;

  bus.failWriteAfterEffectOnCall = bus.writeCalls + 1u;
  bus.writeError = TransportResult::Error(TransportErr::TIMEOUT, -171);
  TEST_ASSERT_TRUE(dev.startForcedMeasurementJob().inProgress());
  JobPollResult result = pollWithBudget(dev, bus, 1);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(JobState::FAILED),
                          static_cast<uint8_t>(result.state));
  TEST_ASSERT_EQUAL_INT32(-171, result.status.detail);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(ConversionState::UNKNOWN_AFTER_TRIGGER_ERROR),
      static_cast<uint8_t>(result.conversionState));
  TEST_ASSERT_EQUAL_UINT32(
      1u, countWritesToRegSince(bus, cmd::REG_CTRL_MEAS, writeLogStart));
  TEST_ASSERT_EQUAL_UINT32(
      0u, countWritesToRegSince(bus, cmd::REG_CTRL_HUM, writeLogStart));

  bus.failReadReg = cmd::REG_STATUS;
  bus.failReadRegRemaining = 1;
  bus.readError = TransportResult::Error(TransportErr::BUS, -172);
  TEST_ASSERT_TRUE(dev.startForcedMeasurementJob().inProgress());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(JobPhase::FORCE_RECONCILE_STATUS),
                          static_cast<uint8_t>(dev.jobPhase()));
  const uint32_t writesBeforeReconcile = bus.writeCalls;
  result = pollWithBudget(dev, bus, 1);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(JobState::FAILED),
                          static_cast<uint8_t>(result.state));
  TEST_ASSERT_EQUAL_UINT32(writesBeforeReconcile, bus.writeCalls);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(ConversionState::UNKNOWN_AFTER_TRIGGER_ERROR),
      static_cast<uint8_t>(dev.conversionState()));

  TEST_ASSERT_TRUE(dev.startForcedMeasurementJob().inProgress());
  result = pollWithBudget(dev, bus, 1);
  TEST_ASSERT_EQUAL_HEX8(cmd::REG_STATUS, bus.lastReadReg);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(JobPhase::FORCE_TRIGGER),
                          static_cast<uint8_t>(result.phase));
  TEST_ASSERT_EQUAL_UINT32(writesBeforeReconcile, bus.writeCalls);
  result = pollWithBudget(dev, bus, 1);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(JobPhase::FORCE_WAIT_TIME),
                          static_cast<uint8_t>(result.phase));
  TEST_ASSERT_EQUAL_UINT32(writesBeforeReconcile + 1u, bus.writeCalls);
  TEST_ASSERT_EQUAL_UINT32(
      2u, countWritesToRegSince(bus, cmd::REG_CTRL_MEAS, writeLogStart));
  TEST_ASSERT_EQUAL_UINT32(
      0u, countWritesToRegSince(bus, cmd::REG_CTRL_HUM, writeLogStart));
}

void test_forced_job_wrap_and_conversion_timeout_are_independent() {
  FakeBus bus;
  BME280::BME280 dev;
  Config cfg = makeConfig(bus);
  cfg.i2cTimeoutMs = 3;
  cfg.conversionReadyTimeoutMs = 7;
  TEST_ASSERT_TRUE(dev.begin(cfg).ok());

  bus.nowMs = 0xFFFFFFF8u;
  TEST_ASSERT_TRUE(dev.startForcedMeasurementJob().inProgress());
  JobPollResult result = pollWithBudget(dev, bus, 1);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(JobState::WAITING),
                          static_cast<uint8_t>(result.state));
  TEST_ASSERT_TRUE(result.phaseDeadlineActive);
  TEST_ASSERT_EQUAL_UINT32(bus.nowMs + dev.estimateMeasurementTimeMs(),
                           result.phaseDeadlineMs);
  TEST_ASSERT_EQUAL_UINT32(cfg.i2cTimeoutMs, bus.lastWriteTimeoutMs);

  bus.reg[cmd::REG_STATUS] = cmd::MASK_STATUS_MEASURING;
  bus.nowMs = result.phaseDeadlineMs;
  result = pollWithBudget(dev, bus, 1);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(JobState::WAITING),
                          static_cast<uint8_t>(result.state));
  TEST_ASSERT_EQUAL_UINT32(bus.nowMs + cfg.conversionReadyTimeoutMs,
                           result.phaseDeadlineMs);
  const uint32_t readyDeadline = result.phaseDeadlineMs;

  bus.nowMs = readyDeadline - 1u;
  result = pollWithBudget(dev, bus, 1);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(JobState::WAITING),
                          static_cast<uint8_t>(result.state));
  bus.nowMs = readyDeadline;
  result = pollWithBudget(dev, bus, 1);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(JobState::FAILED),
                          static_cast<uint8_t>(result.state));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::TIMEOUT),
                          static_cast<uint8_t>(result.status.code));
  TEST_ASSERT_EQUAL_UINT32(cfg.i2cTimeoutMs, bus.lastReadTimeoutMs);
}

void test_poll_and_tick_use_explicit_health_timestamp_context() {
  {
    FakeBus bus;
    BME280::BME280 dev;
    TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());
    bus.nowMs = 9000u;
    TEST_ASSERT_TRUE(dev.startResyncJob().inProgress());
    JobPollResult result = pollAtWithBudget(dev, bus, 0u, 1);
    TEST_ASSERT_EQUAL_UINT8(1u, result.callbacksUsed);
    TEST_ASSERT_TRUE(dev.lastOkTimeValid());
    TEST_ASSERT_EQUAL_UINT32(0u, dev.lastOkMs());

    bus.failReadReg = cmd::REG_STATUS;
    bus.failReadRegRemaining = 1;
    bus.readError = TransportResult::Error(TransportErr::TIMEOUT, -173);
    result = pollAtWithBudget(dev, bus, 1234u, 1);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(JobState::FAILED),
                            static_cast<uint8_t>(result.state));
    TEST_ASSERT_TRUE(dev.lastErrorTimeValid());
    TEST_ASSERT_EQUAL_UINT32(1234u, dev.lastErrorMs());
    TEST_ASSERT_EQUAL_UINT32(9000u, bus.nowMs);
  }

  {
    FakeBus bus;
    BME280::BME280 dev;
    TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());
    bus.nowMs = 0u;
    TEST_ASSERT_TRUE(dev.requestMeasurement().inProgress());
    const uint32_t tickNow = dev.estimateMeasurementTimeMs();
    bus.nowMs = 9000u;
    bus.failReadRegEnabled = true;
    bus.failReadReg = cmd::REG_DATA_START;
    bus.readError = TransportResult::Error(TransportErr::NACK_DATA, -174);
    dev.tick(tickNow);
    TEST_ASSERT_TRUE(dev.lastOkTimeValid());
    TEST_ASSERT_TRUE(dev.lastErrorTimeValid());
    TEST_ASSERT_EQUAL_UINT32(tickNow, dev.lastOkMs());
    TEST_ASSERT_EQUAL_UINT32(tickNow, dev.lastErrorMs());
    TEST_ASSERT_EQUAL_UINT32(9000u, bus.nowMs);
    SettingsSnapshot snapshot{};
    TEST_ASSERT_TRUE(dev.getSettings(snapshot).ok());
    TEST_ASSERT_TRUE(snapshot.lastOkTimeValid);
    TEST_ASSERT_TRUE(snapshot.lastErrorTimeValid);
  }
}

void test_staged_jobs_exclusively_own_hardware_operations() {
  {
    FakeBus bus;
    BME280::BME280 dev;
    const Config cfg = makeConfig(bus);
    const uint32_t callsBeforeStart = totalBusCalls(bus);
    TEST_ASSERT_TRUE(dev.startInitJob(cfg).inProgress());
    TEST_ASSERT_EQUAL_UINT32(callsBeforeStart, totalBusCalls(bus));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(JobKind::INIT),
                            static_cast<uint8_t>(dev.jobKind()));
    assertRepresentativeHardwareApisBusy(dev, bus, cfg);
    SettingsSnapshot settings{};
    TEST_ASSERT_TRUE(dev.getSettings(settings).ok());
    TEST_ASSERT_FALSE(settings.initialized);
    TEST_ASSERT_TRUE(dev.estimateMeasurementTimeMs() > 0u);
    dev.end();
    TEST_ASSERT_EQUAL_UINT32(callsBeforeStart, totalBusCalls(bus));
  }

  {
    FakeBus bus;
    setBoschSyntheticCalibration(bus);
    setRawSample(bus, 415148, 519888, 30000);
    BME280::BME280 dev;
    const Config cfg = makeConfig(bus);
    TEST_ASSERT_TRUE(dev.begin(cfg).ok());
    captureForcedSample(dev, bus);

    const uint32_t callsBeforeStart = totalBusCalls(bus);
    TEST_ASSERT_TRUE(dev.startForcedMeasurementJob().inProgress());
    TEST_ASSERT_EQUAL_UINT32(callsBeforeStart, totalBusCalls(bus));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(JobKind::FORCED_MEASUREMENT),
                            static_cast<uint8_t>(dev.jobKind()));
    assertAllHardwareApisBusy(dev, bus, cfg);

    SettingsSnapshot settings{};
    Calibration calibration{};
    RawSample raw{};
    CompensatedSample compensated{};
    SampleEnvelope envelope{};
    Mode mode = Mode::SLEEP;
    Oversampling osrs = Oversampling::SKIP;
    Filter filter = Filter::X16;
    Standby standby = Standby::MS_1000;
    TEST_ASSERT_TRUE(dev.getSettings(settings).ok());
    TEST_ASSERT_TRUE(dev.getCalibration(calibration).ok());
    TEST_ASSERT_TRUE(dev.getRawSample(raw).ok());
    TEST_ASSERT_TRUE(dev.getCompensatedSample(compensated).ok());
    TEST_ASSERT_TRUE(dev.getSampleEnvelope(envelope).ok());
    TEST_ASSERT_TRUE(dev.getMode(mode).ok());
    TEST_ASSERT_TRUE(dev.getOversamplingT(osrs).ok());
    TEST_ASSERT_TRUE(dev.getOversamplingP(osrs).ok());
    TEST_ASSERT_TRUE(dev.getOversamplingH(osrs).ok());
    TEST_ASSERT_TRUE(dev.getFilter(filter).ok());
    TEST_ASSERT_TRUE(dev.getStandby(standby).ok());
    TEST_ASSERT_TRUE(dev.estimateMeasurementTimeMs() > 0u);
    TEST_ASSERT_TRUE(dev.getStandbyTimeMs() > 0u);
    TEST_ASSERT_TRUE(dev.estimateNormalCycleMs() > 0u);
    TEST_ASSERT_EQUAL_UINT32(callsBeforeStart, totalBusCalls(bus));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(JobState::RUNNING),
                            static_cast<uint8_t>(dev.jobState()));
    dev.end();
    TEST_ASSERT_EQUAL_UINT32(callsBeforeStart, totalBusCalls(bus));
  }

  {
    FakeBus bus;
    BME280::BME280 dev;
    const Config cfg = makeConfig(bus);
    TEST_ASSERT_TRUE(dev.begin(cfg).ok());
    const uint32_t callsBeforeStart = totalBusCalls(bus);
    TEST_ASSERT_TRUE(dev.startApplyConfigJob().inProgress());
    TEST_ASSERT_EQUAL_UINT32(callsBeforeStart, totalBusCalls(bus));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(JobKind::APPLY_CONFIG),
                            static_cast<uint8_t>(dev.jobKind()));
    assertRepresentativeHardwareApisBusy(dev, bus, cfg);
    dev.end();
    TEST_ASSERT_EQUAL_UINT32(callsBeforeStart, totalBusCalls(bus));
  }

  {
    FakeBus bus;
    BME280::BME280 dev;
    const Config cfg = makeConfig(bus);
    TEST_ASSERT_TRUE(dev.begin(cfg).ok());
    const uint32_t callsBeforeStart = totalBusCalls(bus);
    TEST_ASSERT_TRUE(dev.startRecoveryJob().inProgress());
    TEST_ASSERT_EQUAL_UINT32(callsBeforeStart, totalBusCalls(bus));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(JobKind::RESYNC),
                            static_cast<uint8_t>(dev.jobKind()));
    assertRepresentativeHardwareApisBusy(dev, bus, cfg);
    dev.end();
    TEST_ASSERT_EQUAL_UINT32(callsBeforeStart, totalBusCalls(bus));
  }

  {
    FakeBus bus;
    BME280::BME280 dev;
    const Config cfg = makeConfig(bus);
    TEST_ASSERT_TRUE(dev.begin(cfg).ok());
    const uint32_t callsBeforeStart = totalBusCalls(bus);
    TEST_ASSERT_TRUE(dev.startSoftResetJob().inProgress());
    TEST_ASSERT_EQUAL_UINT32(callsBeforeStart, totalBusCalls(bus));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(JobKind::SOFT_RESET),
                            static_cast<uint8_t>(dev.jobKind()));
    assertRepresentativeHardwareApisBusy(dev, bus, cfg);
    dev.end();
    TEST_ASSERT_EQUAL_UINT32(callsBeforeStart, totalBusCalls(bus));
  }
}

void test_end_is_zero_i2c_idempotent_and_rebinds_transport() {
  FakeBus firstBus;
  BME280::BME280 dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(firstBus)).ok());
  TEST_ASSERT_TRUE(dev.startForcedMeasurementJob().inProgress());

  const uint32_t firstWritesBeforeEnd = firstBus.writeCalls;
  const uint32_t firstReadsBeforeEnd = firstBus.readCalls;

  dev.end();
  TEST_ASSERT_EQUAL_UINT32(firstWritesBeforeEnd, firstBus.writeCalls);
  TEST_ASSERT_EQUAL_UINT32(firstReadsBeforeEnd, firstBus.readCalls);
  TEST_ASSERT_FALSE(dev.isInitialized());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::UNINIT),
                          static_cast<uint8_t>(dev.state()));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(JobKind::NONE),
                          static_cast<uint8_t>(dev.jobKind()));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(JobState::IDLE),
                          static_cast<uint8_t>(dev.jobState()));
  TEST_ASSERT_TRUE(dev.getConfig().i2cWrite == nullptr);
  TEST_ASSERT_TRUE(dev.getConfig().i2cWriteRead == nullptr);
  TEST_ASSERT_TRUE(dev.getConfig().i2cUser == nullptr);
  TEST_ASSERT_TRUE(dev.getConfig().nowMs == nullptr);
  TEST_ASSERT_TRUE(dev.getConfig().timeUser == nullptr);

  dev.end();
  TEST_ASSERT_EQUAL_UINT32(firstWritesBeforeEnd, firstBus.writeCalls);
  TEST_ASSERT_EQUAL_UINT32(firstReadsBeforeEnd, firstBus.readCalls);

  uint8_t value = 0;
  Status st = dev.readRegister(cmd::REG_CHIP_ID, value);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::NOT_INITIALIZED),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_UINT32(firstReadsBeforeEnd, firstBus.readCalls);

  st = dev.writeRegister(cmd::REG_CTRL_MEAS, 0);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::NOT_INITIALIZED),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_UINT32(firstWritesBeforeEnd, firstBus.writeCalls);

  st = dev.readRegisters(cmd::REG_CHIP_ID, &value, 1);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::NOT_INITIALIZED),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_UINT32(firstReadsBeforeEnd, firstBus.readCalls);

  FakeBus secondBus;
  secondBus.deviceAddress = 0x77;
  Config secondConfig = makeConfig(secondBus);
  secondConfig.i2cAddress = 0x77;
  TEST_ASSERT_TRUE(dev.begin(secondConfig).ok());
  TEST_ASSERT_TRUE(dev.isInitialized());
  TEST_ASSERT_TRUE(totalBusCalls(secondBus) > 0u);
  TEST_ASSERT_EQUAL_UINT32(firstWritesBeforeEnd, firstBus.writeCalls);
  TEST_ASSERT_EQUAL_UINT32(firstReadsBeforeEnd, firstBus.readCalls);

  const uint32_t secondReadsBefore = secondBus.readCalls;
  TEST_ASSERT_TRUE(dev.readRegister(cmd::REG_CHIP_ID, value).ok());
  TEST_ASSERT_EQUAL_HEX8(cmd::CHIP_ID_BME280, value);
  TEST_ASSERT_EQUAL_UINT32(secondReadsBefore + 1u, secondBus.readCalls);
  TEST_ASSERT_EQUAL_UINT32(firstReadsBeforeEnd, firstBus.readCalls);
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_status_ok);
  RUN_TEST(test_status_error);
  RUN_TEST(test_status_in_progress);
  RUN_TEST(test_transport_result_contract_is_terminal_only);
  RUN_TEST(test_status_copy_assignment_and_persistent_fields_use_canonical_messages);
  RUN_TEST(test_successful_transport_callbacks_report_exact_counts_once);
  RUN_TEST(test_short_ok_counts_map_to_short_transfer_without_retry);
  RUN_TEST(test_every_transport_error_maps_detail_and_nack_remap_is_presence_only);
  RUN_TEST(test_mutating_short_transfer_marks_dirty_or_trigger_ambiguous);
  RUN_TEST(test_partial_raw_read_preserves_committed_sample);
  RUN_TEST(test_config_defaults);
  RUN_TEST(test_driver_is_not_copyable_or_movable);
  RUN_TEST(test_get_settings_snapshot);
  RUN_TEST(test_begin_rejects_missing_callbacks);
  RUN_TEST(test_begin_rejects_zero_nvm_timeout);
  RUN_TEST(test_begin_rejects_oversized_nvm_timeout_without_i2c);
  RUN_TEST(test_begin_rejects_invalid_conversion_timeout_without_i2c);
  RUN_TEST(test_begin_rejects_invalid_i2c_timeout_without_i2c);
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
  RUN_TEST(test_soft_reset_job_failure_after_reset_marks_dirty);
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
  RUN_TEST(test_resync_job_failure_preserves_cached_sample_until_successful_resync);
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
  RUN_TEST(test_offline_history_does_not_block_public_register_read);
  RUN_TEST(test_offline_history_does_not_block_typed_config_setters);
  RUN_TEST(test_probe_works_while_offline_without_clearing_latch);
  RUN_TEST(test_successful_recover_from_offline_clears_latch_and_allows_i2c);
  RUN_TEST(test_failed_recover_from_offline_reports_current_health_observation);
  RUN_TEST(test_recovery_job_from_offline_clears_latch_and_allows_i2c);
  RUN_TEST(test_recovery_job_from_offline_failure_reasserts_offline_latch);
  RUN_TEST(test_recovery_job_from_offline_updates_health_before_job_completion);
  RUN_TEST(test_offline_history_does_not_block_staged_measurement_job);
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
  RUN_TEST(test_cancel_result_is_zero_i2c_exactly_once_and_blocks_until_retrieved);
  RUN_TEST(test_cancellation_preserves_last_good_state_across_mutating_phases);
  RUN_TEST(test_resync_never_resets_and_soft_reset_has_exact_payload);
  RUN_TEST(test_ambiguous_trigger_reconciles_before_any_new_trigger);
  RUN_TEST(test_forced_job_wrap_and_conversion_timeout_are_independent);
  RUN_TEST(test_poll_and_tick_use_explicit_health_timestamp_context);
  RUN_TEST(test_staged_jobs_exclusively_own_hardware_operations);
  RUN_TEST(test_end_is_zero_i2c_idempotent_and_rebinds_transport);
  return UNITY_END();
}
