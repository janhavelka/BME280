# The library is discovered through EXTRA_COMPONENT_DIRS pointing at the
# repository root, so ESP-IDF names the component after that directory. Derive
# the name instead of hard-coding it, otherwise the build only works when the
# checkout happens to be called "BME280" (a clone into another directory name,
# or an extracted release tarball, would fail to resolve the dependency).
get_filename_component(BME280_COMPONENT_NAME "${CMAKE_CURRENT_LIST_DIR}/../../../.." NAME)

idf_component_register(
  SRCS "main.cpp" "IdfI2cTransport.cpp"
  INCLUDE_DIRS "." "../../../.."
  REQUIRES ${BME280_COMPONENT_NAME} esp_driver_i2c esp_driver_gpio esp_timer freertos
)

target_compile_features(${COMPONENT_LIB} PUBLIC cxx_std_17)

# Embed the exact repository identity used by the diagnostic CLI so HIL
# artifacts can reject stale or untraceable firmware. Unknown remains explicit
# when Git is unavailable; dirty includes tracked and untracked files.
set(BME280_REPO_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../../..")
set(BME280_EXAMPLE_GIT_COMMIT "unknown")
set(BME280_EXAMPLE_GIT_STATUS "unknown")
find_package(Git QUIET)
if(GIT_FOUND)
  execute_process(
    COMMAND "${GIT_EXECUTABLE}" rev-parse --short=12 HEAD
    WORKING_DIRECTORY "${BME280_REPO_ROOT}"
    RESULT_VARIABLE BME280_GIT_COMMIT_RESULT
    OUTPUT_VARIABLE BME280_EXAMPLE_GIT_COMMIT
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
  )
  execute_process(
    COMMAND "${GIT_EXECUTABLE}" status --porcelain --untracked-files=normal
    WORKING_DIRECTORY "${BME280_REPO_ROOT}"
    RESULT_VARIABLE BME280_GIT_STATUS_RESULT
    OUTPUT_VARIABLE BME280_GIT_STATUS_OUTPUT
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
  )
  if(BME280_GIT_COMMIT_RESULT EQUAL 0 AND BME280_GIT_STATUS_RESULT EQUAL 0)
    set(BME280_EXAMPLE_GIT_STATUS "clean")
    if(NOT BME280_GIT_STATUS_OUTPUT STREQUAL "")
      set(BME280_EXAMPLE_GIT_STATUS "dirty")
    endif()
  else()
    set(BME280_EXAMPLE_GIT_COMMIT "unknown")
  endif()
endif()

target_compile_definitions(${COMPONENT_LIB} PRIVATE
  BME280_GIT_COMMIT=\"${BME280_EXAMPLE_GIT_COMMIT}\"
  BME280_GIT_STATUS=\"${BME280_EXAMPLE_GIT_STATUS}\"
)
