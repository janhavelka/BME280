#!/usr/bin/env python3
from __future__ import annotations

import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]

IDF_EXAMPLE_MACRO = "BME280_EXAMPLE_PLATFORM_IDF"
CLI_SOURCE_INCLUDE = '#include "examples/01_basic_bringup_cli/main.cpp"'
REQUIRED_COMPONENTS = [
    "BME280",
    "esp_driver_i2c",
    "esp_driver_gpio",
    "esp_timer",
    "esp_rom",
    "freertos",
    "vfs",
]
REQUIRED_FILES = [
    "CMakeLists.txt",
    "idf_component.yml",
    "examples/common/IdfArduinoCompat.h",
    "examples/idf/basic/CMakeLists.txt",
    "examples/idf/basic/main/CMakeLists.txt",
    "examples/idf/basic/main/main.cpp",
    "examples/idf/basic/main/IdfI2cTransport.h",
    "examples/idf/basic/main/IdfI2cTransport.cpp",
]
MANDATORY_COMMANDS = [
    "help",
    "version",
    "scan",
    "begin",
    "read",
    "raw",
    "comp",
    "data",
    "measuring",
    "timing",
    "mode",
    "osrs",
    "filter",
    "standby",
    "cfg",
    "calib",
    "status",
    "chipid",
    "reset",
    "reg",
    "wreg",
    "drv",
    "state",
    "probe",
    "recover",
    "verbose",
    "stress",
    "stress_mix",
    "selftest",
]


def fail(msg: str) -> None:
    print(f"IDF example contract FAILED: {msg}")
    raise SystemExit(1)


def require_token(text: str, token: str, label: str) -> None:
    if token not in text:
        fail(f"{label} missing token '{token}'")


def command_has_dispatch(cli: str, command: str) -> bool:
    patterns = [
        rf'cmd\s*==\s*"{re.escape(command)}"',
        rf'cmd\.startsWith\("{re.escape(command)}\s',
        rf'cmd\.startsWith\("{re.escape(command)}"\)',
    ]
    return any(re.search(pattern, cli) for pattern in patterns)


def main() -> int:
    for rel in REQUIRED_FILES:
        if not (ROOT / rel).exists():
            fail(f"missing {rel}")

    idf_main = (ROOT / "examples" / "idf" / "basic" / "main" / "main.cpp").read_text(
        encoding="utf-8", errors="replace"
    )
    for token in (
        f"#define {IDF_EXAMPLE_MACRO} 1",
        '#include "examples/common/IdfArduinoCompat.h"',
        CLI_SOURCE_INCLUDE,
        'extern "C" void app_main(void)',
        "setup();",
        "loop();",
    ):
        require_token(idf_main, token, "ESP-IDF main")

    cmake = (
        ROOT / "examples" / "idf" / "basic" / "main" / "CMakeLists.txt"
    ).read_text(encoding="utf-8", errors="replace")
    for component in REQUIRED_COMPONENTS:
        if re.search(rf"\b{re.escape(component)}\b", cmake) is None:
            fail(f"ESP-IDF CMake missing required component '{component}'")

    compat = (ROOT / "examples" / "common" / "IdfArduinoCompat.h").read_text(
        encoding="utf-8", errors="replace"
    )
    for token in ("class IdfConsole", "class String", "toInt", "esp_timer_get_time", "esp_rom_delay_us", "fcntl"):
        require_token(compat, token, "IdfArduinoCompat.h")

    transport = (
        (ROOT / "examples" / "idf" / "basic" / "main" / "IdfI2cTransport.cpp")
        .read_text(encoding="utf-8", errors="replace")
        + (ROOT / "examples" / "idf" / "basic" / "main" / "IdfI2cTransport.h")
        .read_text(encoding="utf-8", errors="replace")
    )
    for token in ("driver/i2c_master.h", "i2c_new_master_bus", "i2c_master_transmit", "i2c_master_transmit_receive"):
        require_token(transport, token, "ESP-IDF transport")

    cli = (ROOT / "examples" / "01_basic_bringup_cli" / "main.cpp").read_text(
        encoding="utf-8", errors="replace"
    )
    require_token(cli, f"defined({IDF_EXAMPLE_MACRO})", "shared CLI")
    require_token(cli, "transport::configUser()", "shared CLI")
    for command in MANDATORY_COMMANDS:
        if re.search(rf'printHelpItem\("{re.escape(command)}(?:\b| /| \[| <)', cli) is None:
            fail(f"CLI missing help item '{command}'")
        if not command_has_dispatch(cli, command):
            fail(f"CLI missing dispatch '{command}'")

    for header in (ROOT / "examples" / "common").glob("*.h"):
        text = header.read_text(encoding="utf-8", errors="replace")
        if "#include <Arduino.h>" in text:
            require_token(text, IDF_EXAMPLE_MACRO, header.name)
        if "#include <Wire.h>" in text:
            require_token(text, IDF_EXAMPLE_MACRO, header.name)

    manifest = (ROOT / "idf_component.yml").read_text(encoding="utf-8", errors="replace")
    for token in ("esp32s2", "esp32s3", "idf:"):
        require_token(manifest, token, "idf_component.yml")

    package = (ROOT / "library.json").read_text(encoding="utf-8", errors="replace")
    require_token(package, '"espidf"', "library.json")

    print("IDF example contract PASSED")
    return 0


if __name__ == "__main__":
    sys.exit(main())
