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
  uint32_t imUpdateStatusReadsRemaining = 0;
  bool calibrationReadWhileImUpdate = false;

  int readErrorRemaining = 0;
  int writeErrorRemaining = 0;
  uint32_t failWriteOnCall = 0;
  Status readError = Status::Error(Err::I2C_ERROR, "forced read error", -1);
  Status writeError = Status::Error(Err::I2C_ERROR, "forced write error", -2);
  uint8_t lastWriteReg = 0;
  uint8_t lastWriteValue = 0;
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
      rxData[i] = static_cast<uint8_t>(i + 1);
    }
    rxData[0] = 0x88;
    rxData[1] = 0x01;  // digT1 = 0x0188
    rxData[6] = 0x34;
    rxData[7] = 0x12;  // digP1 = 0x1234
  } else if (reg == cmd::REG_CALIB_H1 && rxLen >= 1) {
    rxData[0] = 0x01;
  } else if (reg == cmd::REG_CALIB_H_START && rxLen == cmd::REG_CALIB_H_LEN) {
    rxData[0] = 0x11;
    rxData[1] = 0x22;
    rxData[2] = 0x33;
    rxData[3] = 0x44;
    rxData[4] = 0x55;
    rxData[5] = 0x66;
    rxData[6] = 0x77;
  } else if (reg == cmd::REG_STATUS && rxLen >= 1) {
    bus->statusReadCalls++;
    uint8_t status = bus->reg[cmd::REG_STATUS];
    if (bus->measuringStatusReadsRemaining > 0) {
      status |= cmd::MASK_STATUS_MEASURING;
      bus->measuringStatusReadsRemaining--;
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
  TEST_ASSERT_EQUAL_UINT32(3u, bus.statusReadCalls);
  TEST_ASSERT_FALSE(bus.calibrationReadWhileImUpdate);
  TEST_ASSERT_TRUE(dev.isInitialized());
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
  RUN_TEST(test_set_mode_sleep_cancels_pending_measurement_request);
  RUN_TEST(test_raw_and_compensated_samples_remain_available_after_measurement_read);
  RUN_TEST(test_register_access_after_end_does_not_touch_bus);
  return UNITY_END();
}
