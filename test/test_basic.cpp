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
  uint8_t chipId = cmd::CHIP_ID_BME280;
  uint32_t nowMs = 1000;
  uint32_t writeCalls = 0;
  uint32_t readCalls = 0;
  uint32_t statusReadCalls = 0;
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
  uint8_t lastReadReg = 0;
  size_t lastReadLen = 0;

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

Status fakeWrite(uint8_t, const uint8_t* data, size_t len, uint32_t, void* user) {
  FakeBus* bus = static_cast<FakeBus*>(user);
  bus->writeCalls++;
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

Status fakeWriteRead(uint8_t, const uint8_t* txData, size_t txLen, uint8_t* rxData,
                     size_t rxLen, uint32_t, void* user) {
  FakeBus* bus = static_cast<FakeBus*>(user);
  bus->readCalls++;
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

  SettingsSnapshot snap;
  Status st = dev.getSettings(snap);
  TEST_ASSERT_TRUE(st.ok());
  TEST_ASSERT_TRUE(snap.initialized);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::READY),
                          static_cast<uint8_t>(snap.state));
  TEST_ASSERT_EQUAL_HEX8(0x77, snap.i2cAddress);
  TEST_ASSERT_EQUAL_UINT32(10u, snap.i2cTimeoutMs);
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

void test_invalid_begin_after_success_resets_default_runtime() {
  FakeBus bus;
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

void test_begin_waits_for_nvm_update_before_reading_calibration() {
  FakeBus bus;
  bus.imUpdateStatusReadsRemaining = 2;
  BME280::BME280 dev;

  Status st = dev.begin(makeConfig(bus));
  TEST_ASSERT_TRUE(st.ok());
  TEST_ASSERT_EQUAL_UINT32(5u, bus.statusReadCalls);
  TEST_ASSERT_FALSE(bus.calibrationReadWhileImUpdate);
  TEST_ASSERT_TRUE(dev.isInitialized());
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

void test_begin_times_out_when_nvm_update_stuck() {
  FakeBus bus;
  bus.reg[cmd::REG_STATUS] = cmd::MASK_STATUS_IM_UPDATE;
  BME280::BME280 dev;

  Status st = dev.begin(makeConfig(bus));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::TIMEOUT),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_FALSE(dev.isInitialized());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::UNINIT),
                          static_cast<uint8_t>(dev.state()));
  TEST_ASSERT_FALSE(bus.calibrationReadWhileImUpdate);
  TEST_ASSERT_GREATER_THAN_UINT32(0u, bus.statusReadCalls);
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

void test_probe_failure_does_not_update_health() {
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
  RUN_TEST(test_begin_rejects_invalid_oversampling_combination);
  RUN_TEST(test_invalid_begin_after_success_resets_default_runtime);
  RUN_TEST(test_begin_normalizes_offline_threshold_on_stored_copy);
  RUN_TEST(test_begin_success_sets_ready_without_health_counts);
  RUN_TEST(test_begin_waits_for_nvm_update_before_reading_calibration);
  RUN_TEST(test_begin_defers_apply_config_when_device_measuring_without_write);
  RUN_TEST(test_begin_times_out_when_nvm_update_stuck);
  RUN_TEST(test_begin_forced_mode_keeps_hardware_sleep_until_requested);
  RUN_TEST(test_missing_now_ms_fallback_is_framework_neutral);
  RUN_TEST(test_begin_without_now_ms_uses_framework_neutral_fallback);
  RUN_TEST(test_request_measurement_requires_now_ms_hook);
  RUN_TEST(test_probe_failure_does_not_update_health);
  RUN_TEST(test_probe_address_nack_maps_to_device_not_found);
  RUN_TEST(test_begin_preserves_timeout_transport_error);
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
  RUN_TEST(test_invalid_begin_does_not_clear_existing_dirty_state);
  RUN_TEST(test_apply_config_partial_failure_marks_dirty_and_preserves_error);
  RUN_TEST(test_set_mode_forced_does_not_trigger_conversion);
  RUN_TEST(test_example_transport_maps_wire_errors_and_keeps_timeout_owned_by_init);
  RUN_TEST(test_example_transport_validates_params_and_handles_write_read);
  RUN_TEST(test_recover_reaches_offline_when_threshold_is_one);
  RUN_TEST(test_offline_latches_public_register_read_without_i2c);
  RUN_TEST(test_failed_recover_from_offline_keeps_latch_after_intermediate_success);
  RUN_TEST(test_forced_measurement_timing_wraparound_reaches_ready);
  RUN_TEST(test_normal_mode_request_waits_for_fresh_cycle);
  RUN_TEST(test_forced_measurement_request_while_busy_tracks_completion);
  RUN_TEST(test_tick_raw_read_failure_records_measurement_status);
  RUN_TEST(test_set_mode_sleep_cancels_pending_measurement_request);
  RUN_TEST(test_raw_and_compensated_samples_remain_available_after_measurement_read);
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
  RUN_TEST(test_register_access_after_end_does_not_touch_bus);
  return UNITY_END();
}
