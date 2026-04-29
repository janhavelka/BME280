/// @file test_basic.cpp
/// @brief Native contract tests for BME280 lifecycle and health behavior.

#include <unity.h>

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
  uint32_t measuringStatusReadsRemaining = 0;

  int readErrorRemaining = 0;
  int writeErrorRemaining = 0;
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
    if (bus->measuringStatusReadsRemaining > 0) {
      rxData[0] = cmd::MASK_STATUS_MEASURING;
      bus->measuringStatusReadsRemaining--;
    } else {
      rxData[0] = bus->reg[cmd::REG_STATUS];
    }
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
  TEST_ASSERT_EQUAL_UINT32(0u, snap.measurementStartMs);
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

void test_begin_success_sets_ready_and_health() {
  FakeBus bus;
  BME280::BME280 dev;
  Status st = dev.begin(makeConfig(bus));
  TEST_ASSERT_TRUE(st.ok());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::READY),
                          static_cast<uint8_t>(dev.state()));
  TEST_ASSERT_TRUE(dev.isOnline());
  TEST_ASSERT_GREATER_THAN_UINT32(0u, dev.totalSuccess());
  TEST_ASSERT_EQUAL_UINT32(0u, dev.totalFailures());
  TEST_ASSERT_EQUAL_UINT8(0u, dev.consecutiveFailures());
  TEST_ASSERT_EQUAL_UINT32(bus.nowMs, dev.lastOkMs());
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

void test_now_ms_fallback_uses_millis_when_callback_missing() {
  FakeBus bus;
  BME280::BME280 dev;
  Config cfg = makeConfig(bus);
  cfg.nowMs = nullptr;
  cfg.timeUser = nullptr;
  TEST_ASSERT_TRUE(dev.begin(cfg).ok());

  setMillis(4321);
  Status st = dev.recover();
  TEST_ASSERT_TRUE(st.ok());
  TEST_ASSERT_EQUAL_UINT32(4321u, dev.lastOkMs());
}

void test_begin_without_now_ms_uses_millis_fallback() {
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
  TEST_ASSERT_EQUAL_UINT32(4242u, dev.lastOkMs());
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
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::DEVICE_NOT_FOUND),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_UINT32(beforeSuccess, dev.totalSuccess());
  TEST_ASSERT_EQUAL_UINT32(beforeFailures, dev.totalFailures());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(beforeState),
                          static_cast<uint8_t>(dev.state()));
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
  RUN_TEST(test_get_settings_snapshot);
  RUN_TEST(test_begin_rejects_missing_callbacks);
  RUN_TEST(test_begin_rejects_invalid_oversampling_combination);
  RUN_TEST(test_begin_success_sets_ready_and_health);
  RUN_TEST(test_begin_forced_mode_keeps_hardware_sleep_until_requested);
  RUN_TEST(test_now_ms_fallback_uses_millis_when_callback_missing);
  RUN_TEST(test_probe_failure_does_not_update_health);
  RUN_TEST(test_recover_failure_updates_health_once);
  RUN_TEST(test_recover_success_returns_ready);
  RUN_TEST(test_recover_chip_id_mismatch_updates_health);
  RUN_TEST(test_recover_preserves_transport_error_code);
  RUN_TEST(test_set_oversampling_rejects_invalid_compensation_combo_without_write);
  RUN_TEST(test_set_mode_forced_does_not_trigger_conversion);
  RUN_TEST(test_example_transport_maps_wire_errors_and_keeps_timeout_owned_by_init);
  RUN_TEST(test_example_transport_validates_params_and_handles_write_read);
  RUN_TEST(test_recover_reaches_offline_when_threshold_is_one);
  RUN_TEST(test_forced_measurement_timing_wraparound_reaches_ready);
  RUN_TEST(test_normal_mode_request_waits_for_fresh_cycle);
  RUN_TEST(test_forced_measurement_request_while_busy_tracks_completion);
  RUN_TEST(test_raw_and_compensated_samples_remain_available_after_measurement_read);
  RUN_TEST(test_begin_without_now_ms_uses_millis_fallback);
  RUN_TEST(test_register_access_after_end_does_not_touch_bus);
  return UNITY_END();
}
