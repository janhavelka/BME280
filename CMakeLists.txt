idf_component_register(
  SRCS "src/BME280.cpp"
  INCLUDE_DIRS "include"
  PRIV_REQUIRES esp_timer
)

target_compile_features(${COMPONENT_LIB} PUBLIC cxx_std_17)
