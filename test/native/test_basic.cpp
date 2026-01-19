/// @file test_basic.cpp
/// @brief Basic unit tests for BME280 driver

#include <cstdio>
#include <cassert>

// Include stubs first
#include "Arduino.h"
#include "Wire.h"

// Stub implementations
SerialClass Serial;
TwoWire Wire;

// Include driver
#include "BME280/Status.h"
#include "BME280/Config.h"

using namespace BME280;

// ============================================================================
// Test Helpers
// ============================================================================

static int testsPassed = 0;
static int testsFailed = 0;

#define TEST(name) void test_##name()
#define RUN_TEST(name) do { \
  printf("Running %s... ", #name); \
  test_##name(); \
  printf("PASSED\n"); \
  testsPassed++; \
} catch (...) { \
  printf("FAILED\n"); \
  testsFailed++; \
}

#define ASSERT_TRUE(x) assert(x)
#define ASSERT_FALSE(x) assert(!(x))
#define ASSERT_EQ(a, b) assert((a) == (b))
#define ASSERT_NE(a, b) assert((a) != (b))

// ============================================================================
// Tests
// ============================================================================

TEST(status_ok) {
  Status st = Status::Ok();
  ASSERT_TRUE(st.ok());
  ASSERT_EQ(st.code, Err::OK);
}

TEST(status_error) {
  Status st = Status::Error(Err::I2C_ERROR, "Test error", 42);
  ASSERT_FALSE(st.ok());
  ASSERT_EQ(st.code, Err::I2C_ERROR);
  ASSERT_EQ(st.detail, 42);
}

TEST(status_in_progress) {
  Status st = Status{Err::IN_PROGRESS, 0, "In progress"};
  ASSERT_FALSE(st.ok());
  ASSERT_EQ(st.code, Err::IN_PROGRESS);
}

TEST(config_defaults) {
  Config cfg;
  ASSERT_EQ(cfg.i2cWrite, nullptr);
  ASSERT_EQ(cfg.i2cWriteRead, nullptr);
  ASSERT_EQ(cfg.i2cAddress, 0x76);
  ASSERT_EQ(cfg.i2cTimeoutMs, 50);
  ASSERT_EQ(cfg.offlineThreshold, 5);
  ASSERT_EQ(static_cast<uint8_t>(cfg.osrsT), static_cast<uint8_t>(Oversampling::X1));
  ASSERT_EQ(static_cast<uint8_t>(cfg.osrsP), static_cast<uint8_t>(Oversampling::X1));
  ASSERT_EQ(static_cast<uint8_t>(cfg.osrsH), static_cast<uint8_t>(Oversampling::X1));
  ASSERT_EQ(static_cast<uint8_t>(cfg.filter), static_cast<uint8_t>(Filter::OFF));
  ASSERT_EQ(static_cast<uint8_t>(cfg.standby), static_cast<uint8_t>(Standby::MS_125));
  ASSERT_EQ(static_cast<uint8_t>(cfg.mode), static_cast<uint8_t>(Mode::FORCED));
}

// ============================================================================
// Main
// ============================================================================

int main() {
  printf("\n=== BME280 Unit Tests ===\n\n");
  
  RUN_TEST(status_ok);
  RUN_TEST(status_error);
  RUN_TEST(status_in_progress);
  RUN_TEST(config_defaults);
  
  printf("\n=== Results: %d passed, %d failed ===\n\n", testsPassed, testsFailed);
  
  return testsFailed > 0 ? 1 : 0;
}
