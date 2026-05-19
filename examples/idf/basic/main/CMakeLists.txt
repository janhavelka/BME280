idf_component_register(
  SRCS "main.cpp" "IdfI2cTransport.cpp"
  INCLUDE_DIRS "." "../../../.."
  REQUIRES BME280 esp_driver_i2c esp_driver_gpio esp_timer esp_rom freertos vfs
)

target_compile_features(${COMPONENT_LIB} PUBLIC cxx_std_17)
