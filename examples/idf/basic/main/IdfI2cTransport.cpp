#include "IdfI2cTransport.h"

#include <limits>

#include <driver/gpio.h>
#include <esp_err.h>

namespace {

IdfI2cContext gContext;

int timeoutToIdf(uint32_t timeoutMs) {
  constexpr uint32_t MAX_TIMEOUT_MS =
      static_cast<uint32_t>(std::numeric_limits<int>::max());
  if (timeoutMs > MAX_TIMEOUT_MS) {
    return std::numeric_limits<int>::max();
  }
  return static_cast<int>(timeoutMs);
}

BME280::TransportResult mapEspError(esp_err_t err, size_t writeCount,
                                    size_t readCount = 0) {
  switch (err) {
    case ESP_OK:
      return BME280::TransportResult::Complete(writeCount, readCount);
    case ESP_ERR_TIMEOUT:
      return BME280::TransportResult::Error(
          BME280::TransportErr::TIMEOUT, static_cast<int32_t>(err));
    case ESP_ERR_INVALID_ARG:
      return BME280::TransportResult::Error(
          BME280::TransportErr::OTHER, static_cast<int32_t>(err));
    case ESP_ERR_INVALID_RESPONSE:
      // The high-level IDF API reports NACK without proving address vs data
      // phase. Do not invent DEVICE_NOT_FOUND or NACK_DATA.
      return BME280::TransportResult::Error(
          BME280::TransportErr::OTHER, static_cast<int32_t>(err));
    default:
      return BME280::TransportResult::Error(
          BME280::TransportErr::BUS, static_cast<int32_t>(err));
  }
}

BME280::TransportResult validate(uint8_t addr, const void* user) {
  if (user == nullptr) {
    return BME280::TransportResult::Error(BME280::TransportErr::OTHER, -1);
  }
  const IdfI2cContext* ctx = static_cast<const IdfI2cContext*>(user);
  if (ctx->device == nullptr) {
    return BME280::TransportResult::Error(BME280::TransportErr::OTHER, -2);
  }
  if (addr != ctx->address) {
    return BME280::TransportResult::Error(BME280::TransportErr::OTHER, -3);
  }
  return BME280::TransportResult::Complete(0, 0);
}

}  // namespace

IdfI2cContext& bme280IdfI2cContext() {
  return gContext;
}

bool bme280IdfInitI2c(int sda, int scl, uint32_t freqHz, uint8_t address) {
  bme280IdfDeinitI2c();

  i2c_master_bus_config_t busConfig = {};
  busConfig.i2c_port = I2C_NUM_0;
  busConfig.sda_io_num = static_cast<gpio_num_t>(sda);
  busConfig.scl_io_num = static_cast<gpio_num_t>(scl);
  busConfig.clk_source = I2C_CLK_SRC_DEFAULT;
  busConfig.glitch_ignore_cnt = 7;
  // Bring-up aid only; production hardware should use external pullups to VDDIO.
  busConfig.flags.enable_internal_pullup = true;

  esp_err_t err = i2c_new_master_bus(&busConfig, &gContext.bus);
  if (err != ESP_OK) {
    gContext.lastError = err;
    return false;
  }

  i2c_device_config_t devConfig = {};
  devConfig.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  devConfig.device_address = address;
  devConfig.scl_speed_hz = freqHz;

  err = i2c_master_bus_add_device(gContext.bus, &devConfig, &gContext.device);
  if (err != ESP_OK) {
    (void)i2c_del_master_bus(gContext.bus);
    gContext.bus = nullptr;
    gContext.lastError = err;
    return false;
  }

  gContext.address = address;
  gContext.lastError = ESP_OK;
  return true;
}

void bme280IdfDeinitI2c() {
  if (gContext.device != nullptr) {
    (void)i2c_master_bus_rm_device(gContext.device);
    gContext.device = nullptr;
  }
  if (gContext.bus != nullptr) {
    (void)i2c_del_master_bus(gContext.bus);
    gContext.bus = nullptr;
  }
}

BME280::TransportResult idfI2cWrite(uint8_t addr, const uint8_t* data,
                                    size_t len, uint32_t timeoutMs, void* user) {
  const BME280::TransportResult validation = validate(addr, user);
  if (!validation.ok()) {
    return validation;
  }
  if (data == nullptr || len == 0U || len > static_cast<size_t>(std::numeric_limits<int>::max())) {
    return BME280::TransportResult::Error(BME280::TransportErr::OTHER, -4);
  }

  IdfI2cContext* ctx = static_cast<IdfI2cContext*>(user);
  // Exactly one physical attempt. No adapter retry or bus recovery.
  ctx->lastError = i2c_master_transmit(ctx->device, data, static_cast<size_t>(len),
                                       timeoutToIdf(timeoutMs));
  return mapEspError(ctx->lastError, len);
}

BME280::TransportResult idfI2cWriteRead(uint8_t addr,
                                        const uint8_t* txData, size_t txLen,
                                        uint8_t* rxData, size_t rxLen,
                                        uint32_t timeoutMs, void* user) {
  const BME280::TransportResult validation = validate(addr, user);
  if (!validation.ok()) {
    return validation;
  }
  if (txData == nullptr || rxData == nullptr || txLen == 0U || rxLen == 0U ||
      txLen > static_cast<size_t>(std::numeric_limits<int>::max()) ||
      rxLen > static_cast<size_t>(std::numeric_limits<int>::max())) {
    return BME280::TransportResult::Error(BME280::TransportErr::OTHER, -5);
  }

  IdfI2cContext* ctx = static_cast<IdfI2cContext*>(user);
  const int timeout = timeoutToIdf(timeoutMs);
  // One combined write/repeated-START/read transaction. No intermediate STOP,
  // adapter retry, or bus recovery.
  ctx->lastError = i2c_master_transmit_receive(ctx->device, txData, txLen, rxData,
                                               rxLen, timeout);
  return mapEspError(ctx->lastError, txLen, rxLen);
}
