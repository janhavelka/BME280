#!/usr/bin/env python3
from __future__ import annotations

import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
ARDUINO_MAIN = ROOT / "examples" / "01_basic_bringup_cli" / "main.cpp"
IDF_MAIN = ROOT / "examples" / "idf" / "basic" / "main" / "main.cpp"
IDF_TRANSPORT = ROOT / "examples" / "idf" / "basic" / "main" / "IdfI2cTransport.cpp"
IDF_CMAKE = ROOT / "examples" / "idf" / "basic" / "main" / "CMakeLists.txt"
IDF_ROOT = ROOT / "examples" / "idf" / "basic"

FORBIDDEN_IDF_TOKENS = [
    "Arduino.h",
    "Wire.h",
    "IdfArduinoCompat",
    "ArduinoCompat",
    "TwoWire",
    "Serial",
    "examples/01_basic_bringup_cli/main.cpp",
    "setup();",
    "loop();",
]

FORBIDDEN_IDF_PATTERNS = {
    "millis() shim or call": re.compile(r"\bmillis\s*\("),
    "Arduino delay() call": re.compile(r"\bdelay\s*\("),
    "Arduino String type": re.compile(r"\bString\b"),
}

REQUIRED_IDF_TOKENS = [
    'extern "C" void app_main(void)',
    "driver/i2c_master.h",
    "i2c_master_probe",
    "i2c_new_master_bus",
    "i2c_master_transmit",
    "i2c_master_transmit_receive",
    "esp_timer_get_time",
    "vTaskDelay",
    "xTaskCreate",
    "QueueHandle_t",
    "LOG_COLOR_GREEN",
    "LOG_COLOR_YELLOW",
    "LOG_COLOR_RED",
]

MANDATORY_COMMANDS = {
    "?",
    "help",
    "version",
    "ver",
    "addr",
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
    "settings",
    "calib",
    "status",
    "id",
    "chipid",
    "force",
    "normal",
    "reset",
    "reg",
    "wreg",
    "drv",
    "state",
    "probe",
    "recover",
    "job",
    "verbose",
    "stress",
    "stress_mix",
    "selftest",
}


def fail(msg: str) -> None:
    print(f"IDF example contract FAILED: {msg}")
    raise SystemExit(1)


def read(path: pathlib.Path) -> str:
    if not path.exists():
        fail(f"missing file: {path.relative_to(ROOT).as_posix()}")
    return path.read_text(encoding="utf-8", errors="replace")


def help_items(text: str) -> list[str]:
    return re.findall(r'printHelpItem\("([^"]+)"', text)


def aliases_from_help(items: list[str]) -> set[str]:
    aliases: set[str] = set()
    for item in items:
        command_part = item.split(" ", 1)[0]
        for alias in command_part.split("/"):
            alias = alias.strip()
            if alias:
                aliases.add(alias)
    return aliases


def dispatched_commands(text: str) -> set[str]:
    return set(re.findall(r'std::strcmp\(head,\s*"([^"]+)"\)\s*==\s*0', text))


def main() -> int:
    arduino = read(ARDUINO_MAIN)
    idf = read(IDF_MAIN)
    read(IDF_TRANSPORT)
    idf_cmake = read(IDF_CMAKE)
    idf_files = sorted(
        path
        for path in IDF_ROOT.rglob("*")
        if path.is_file() and path.suffix in {".c", ".cc", ".cpp", ".h", ".hpp", ".txt"}
        and not (set(path.relative_to(IDF_ROOT).parts) & {"build", "managed_components", ".pytest_cache"})
    )
    combined_idf = "\n".join(read(path) for path in idf_files)

    for token in FORBIDDEN_IDF_TOKENS:
        if token in combined_idf:
            fail(f"IDF example uses forbidden Arduino/compat token: {token}")

    for label, pattern in FORBIDDEN_IDF_PATTERNS.items():
        if pattern.search(combined_idf):
            fail(f"IDF example uses forbidden Arduino-style timing token: {label}")

    if "driver/i2c.h" in combined_idf:
        fail("IDF example must use driver/i2c_master.h, not legacy driver/i2c.h")

    for token in REQUIRED_IDF_TOKENS:
        if token not in combined_idf:
            fail(f"IDF example missing required native token: {token}")

    for token in (
        "rev-parse --short=12 HEAD",
        "status --porcelain --untracked-files=normal",
        "BME280_GIT_COMMIT",
        "BME280_GIT_STATUS",
        "target_compile_definitions",
    ):
        if token not in idf_cmake:
            fail(f"IDF example CMake missing firmware provenance contract: {token}")

    arduino_help = help_items(arduino)
    idf_help = help_items(idf)
    if arduino_help != idf_help:
        missing = [item for item in arduino_help if item not in idf_help]
        extra = [item for item in idf_help if item not in arduino_help]
        fail(f"help items differ: missing={missing}, extra={extra}")

    idf_commands = dispatched_commands(idf) | aliases_from_help(idf_help)
    missing_commands = sorted(MANDATORY_COMMANDS - idf_commands)
    if missing_commands:
        fail(f"IDF CLI missing mandatory commands: {missing_commands}")

    for token in (
        "Selftest result:",
        "=== Stress Summary ===",
        "Duration:",
        "Rate:",
        "Health delta:",
        "=== stress_mix summary ===",
        "Total:",
        "=== Job Status ===",
        "Job kind:",
        "Job ID:",
        "Job phase:",
        "Job state:",
        "Terminal state:",
        "Conversion state:",
        "Phase deadline active:",
        "Phase deadline ms:",
        "Callbacks used:",
        "Instructions:",
        "Driver:",
    ):
        if token not in idf:
            fail(f"IDF CLI output contract missing token: {token}")

    for token in (
        "startResyncJob",
        "startRecoveryJob",
        "startSoftResetJob",
        "cancelJob",
        "CancelReason::OWNER_REQUEST",
        "CancelReason::DEADLINE_EXPIRED",
        "pollJob(currentMs(), 0U)",
        "BME280::toString(result.status.code)",
    ):
        if token not in idf:
            fail(f"IDF staged-job contract missing token: {token}")

    if not re.search(
        r'std::strcmp\(action, "resync"\) == 0\)\s*\{\s*return device\.startResyncJob\(\);',
        idf,
    ):
        fail("IDF job resync must map to non-reset startResyncJob()")
    if not re.search(
        r'std::strcmp\(action, "reset"\) == 0\)\s*\{\s*return device\.startSoftResetJob\(\);',
        idf,
    ):
        fail("IDF job reset must map to explicit startSoftResetJob()")

    ordered_job_labels = (
        '"Boundary: %s\\n"',
        '"Job ID: %lu\\n"',
        '"Job kind: %s\\n"',
        '"Job phase: %s\\n"',
        '"Job state: %s\\n"',
        '"Terminal state: %s\\n"',
        '"Status: %s (code=%u, detail=%ld)\\n"',
        '"Conversion state: %s\\n"',
        '"Phase deadline active: %s\\n"',
        '"Phase deadline ms: %lu\\n"',
        '"Callbacks used: %u\\n"',
        '"Instructions: %u\\n"',
    )
    for source, label in ((arduino, "Arduino"), (idf, "IDF")):
        positions = [source.find(token) for token in ordered_job_labels]
        if any(position < 0 for position in positions) or positions != sorted(positions):
            fail(f"{label} staged-job output fields are missing or out of contract order")

    manifest = read(ROOT / "idf_component.yml")
    for token in ("esp32s2", "esp32s3", "idf:"):
        if token not in manifest:
            fail(f"idf_component.yml missing '{token}'")

    package = read(ROOT / "library.json")
    if '"espidf"' not in package:
        fail("library.json missing espidf framework")

    print("IDF example contract PASSED")
    return 0


if __name__ == "__main__":
    sys.exit(main())
