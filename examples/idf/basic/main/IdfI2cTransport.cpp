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

BME280::Status mapEspError(esp_err_t err, const char* message) {
  switch (err) {
    case ESP_OK:
      return BME280::Status::Ok();
    case ESP_ERR_TIMEOUT:
      return BME280::Status::Error(BME280::Err::I2C_TIMEOUT, message,
                                   static_cast<int32_t>(err));
    case ESP_ERR_INVALID_ARG:
      return BME280::Status::Error(BME280::Err::INVALID_PARAM, message,
                                   static_cast<int32_t>(err));
    case ESP_ERR_INVALID_RESPONSE:
      return BME280::Status::Error(BME280::Err::I2C_ERROR, message,
                                   static_cast<int32_t>(err));
    default:
      return BME280::Status::Error(BME280::Err::I2C_BUS, message,
                                   static_cast<int32_t>(err));
  }
}

BME280::Status validate(uint8_t addr, const void* user) {
  if (user == nullptr) {
    return BME280::Status::Error(BME280::Err::INVALID_CONFIG,
                                 "IDF I2C context is null");
  }
  const IdfI2cContext* ctx = static_cast<const IdfI2cContext*>(user);
  if (ctx->device == nullptr) {
    return BME280::Status::Error(BME280::Err::INVALID_CONFIG,
                                 "IDF I2C device handle is null");
  }
  if (addr != ctx->address) {
    return BME280::Status::Error(BME280::Err::INVALID_PARAM,
                                 "Unexpected I2C address");
  }
  return BME280::Status::Ok();
}

}  // namespace

IdfI2cContext& bme280IdfI2cContext() {
  return gContext;
}

bool bme280IdfInitI2c(int sda, int scl, uint32_t freqHz, uint16_t timeoutMs,
                      uint8_t address) {
  (void)timeoutMs;
  bme280IdfDeinitI2c();

  i2c_master_bus_config_t busConfig = {};
  busConfig.i2c_port = I2C_NUM_0;
  busConfig.sda_io_num = static_cast<gpio_num_t>(sda);
  busConfig.scl_io_num = static_cast<gpio_num_t>(scl);
  busConfig.clk_source = I2C_CLK_SRC_DEFAULT;
  busConfig.glitch_ignore_cnt = 7;
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

BME280::Status idfI2cWrite(uint8_t addr, const uint8_t* data, size_t len,
                           uint32_t timeoutMs, void* user) {
  BME280::Status st = validate(addr, user);
  if (!st.ok()) {
    return st;
  }
  if (data == nullptr || len == 0U || len > static_cast<size_t>(std::numeric_limits<int>::max())) {
    return BME280::Status::Error(BME280::Err::INVALID_PARAM,
                                 "Invalid IDF I2C write buffer");
  }

  IdfI2cContext* ctx = static_cast<IdfI2cContext*>(user);
  ctx->lastError = i2c_master_transmit(ctx->device, data, static_cast<size_t>(len),
                                       timeoutToIdf(timeoutMs));
  return mapEspError(ctx->lastError, "IDF I2C write failed");
}

BME280::Status idfI2cWriteRead(uint8_t addr, const uint8_t* txData, size_t txLen,
                               uint8_t* rxData, size_t rxLen,
                               uint32_t timeoutMs, void* user) {
  BME280::Status st = validate(addr, user);
  if (!st.ok()) {
    return st;
  }
  if ((txLen > 0U && txData == nullptr) || (rxLen > 0U && rxData == nullptr) ||
      txLen > static_cast<size_t>(std::numeric_limits<int>::max()) ||
      rxLen > static_cast<size_t>(std::numeric_limits<int>::max())) {
    return BME280::Status::Error(BME280::Err::INVALID_PARAM,
                                 "Invalid IDF I2C write-read buffer");
  }

  IdfI2cContext* ctx = static_cast<IdfI2cContext*>(user);
  const int timeout = timeoutToIdf(timeoutMs);
  if (txLen == 0U) {
    ctx->lastError = i2c_master_receive(ctx->device, rxData, rxLen, timeout);
    return mapEspError(ctx->lastError, "IDF I2C read failed");
  }
  if (rxLen == 0U) {
    ctx->lastError = i2c_master_transmit(ctx->device, txData, txLen, timeout);
    return mapEspError(ctx->lastError, "IDF I2C write phase failed");
  }
  ctx->lastError = i2c_master_transmit_receive(ctx->device, txData, txLen, rxData,
                                               rxLen, timeout);
  return mapEspError(ctx->lastError, "IDF I2C write-read failed");
}
