/**
 * @file main.cpp
 * @brief ESP-IDF entry point for the full BME280 bring-up CLI.
 *
 * The command implementation is shared with the Arduino example so both
 * frameworks expose the same workflow, coloring, help structure, diagnostics,
 * and command set.
 */

#define BME280_EXAMPLE_PLATFORM_IDF 1

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "examples/common/IdfArduinoCompat.h"

IdfConsole Serial;

#include "examples/01_basic_bringup_cli/main.cpp"

extern "C" void app_main(void) {
  setup();
  while (true) {
    loop();
    vTaskDelay(idfExampleDelayTicks(1U));
  }
}
