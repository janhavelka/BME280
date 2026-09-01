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
    "end",
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
    "invalidate",
    "freshness",
    "dump",
    "rregs",
    "xfer_reset",
    "xfer_stats",
    "xfer_assert",
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


def help_entries(text: str) -> list[tuple[str, str]]:
    return re.findall(
        r'printHelpItem\("([^"]+)",\s*"([^"]+)"\)',
        text,
    )


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

    # The root component's name is the checkout directory's name, so the main
    # component must derive it rather than hard-coding "BME280"; hard-coding
    # breaks any clone or extracted tarball with a different directory name.
    if re.search(
        r"(?:PRIV_REQUIRES|REQUIRES)[^)]*BME280(?!_COMPONENT_NAME)",
        idf_cmake,
        re.DOTALL,
    ):
        fail("IDF main component must not hard-code the root component name")
    for token in (
        'get_filename_component(BME280_ROOT_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../.." ABSOLUTE)',
        'get_filename_component(BME280_COMPONENT_NAME "${BME280_ROOT_DIR}" NAME)',
        "REQUIRES ${BME280_COMPONENT_NAME} esp_driver_i2c esp_driver_gpio esp_timer freertos",
    ):
        if token not in idf_cmake:
            fail(f"IDF main-component dependency contract missing: {token}")

    arduino_help = help_items(arduino)
    idf_help = help_items(idf)
    if arduino_help != idf_help:
        missing = [item for item in arduino_help if item not in idf_help]
        extra = [item for item in idf_help if item not in arduino_help]
        fail(f"help items differ: missing={missing}, extra={extra}")
    if help_entries(arduino) != help_entries(idf):
        fail("Arduino and IDF help descriptions/order differ")

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
        "Restore status:",
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
        "=== Sample Freshness ===",
        "=== Register Block ===",
        "XFER_RESET read=0 write=0 total=0",
        "XFER_ASSERT PASS",
        "XFER_ASSERT FAIL",
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
        "startApplySettingsJob",
        "BME280::validateSettings(settings)",
        "invalidateDeviceState",
        "sampleFresh(now, maxAgeMs)",
        "readRegisters",
        "bme280IdfResetTransferStats",
        "bme280IdfTransferStats",
        "JOB_CLI_MAX_POLLS = 1024U",
        "delayTicksAtLeastOne",
        "BME280::Status cancelPending()",
        "cancelPendingForCommand()",
        "cancelPendingForRecovery()",
        "Command too long",
        "CLI_INPUT_MAX_LEN = 127U",
        "CLI_LINE_LEN = CLI_INPUT_MAX_LEN + 2U",
        'static_assert(CLI_LINE_LEN == 129U, "CLI line buffer contract changed")',
        "STRESS_PROGRESS_UPDATES = 10U",
        "void printStressProgress(",
        "Progress: %lu/%lu",
    ):
        if token not in idf:
            fail(f"IDF staged-job contract missing token: {token}")

    strict_legacy_checks = (
        r'nextToken\(cursor\) != nullptr \|\| !parseI2cAddress\(arg, address\)',
        r'nextToken\(cursor\) != nullptr \|\|\s*\(std::strcmp\(arg, "on"\)',
        r'Usage: calib \[raw\]',
        r'nextToken\(cursor\) != nullptr \|\| !parseU32\(addressToken, addr\)',
        r'!parseU32\(addressToken, addr\) \|\| !parseU32\(valueToken, value\)',
        r'!parseU32\(arg, value\) \|\| value > 1U',
    )
    for pattern in strict_legacy_checks:
        if re.search(pattern, idf) is None:
            fail(f"IDF strict legacy-command contract missing pattern: {pattern}")

    for usage in (
        "help|?", "version|ver", "scan", "begin", "read", "force", "raw",
        "comp", "data", "measuring", "timing", "status", "chipid|id",
        "reset", "drv", "state", "probe", "recover", "selftest",
    ):
        if f'requireNoArguments(cursor, "{usage}")' not in idf:
            fail(f"IDF zero-argument command is not strict: {usage}")

    if idf.count("nextToken(cursor) != nullptr ||") < 8:
        fail("IDF strict optional/count command arity checks regressed")
    if not re.search(
        r'valueTok == nullptr \|\| nextToken\(cursor\) != nullptr.*?'
        r'Usage: osrs t <1\.\.5> \| osrs p\|h <0\.\.5>',
        idf,
        re.DOTALL,
    ):
        fail("IDF osrs command must reject missing or trailing arguments as usage errors")

    if "vTaskDelay(pdMS_TO_TICKS(" in idf:
        fail("IDF CLI contains a delay that can round down to zero ticks")
    if idf.count("vTaskDelay(delayTicksAtLeastOne(1U))") < 2:
        fail("IDF blocking measurement and mixed-stress paths need bounded one-tick delays")

    for helper, output in (
        ("printModeSettings", "Chip mode:"),
        ("printOsrsSettings", "Chip osrs:"),
        ("printFilterSettings", "Chip filter:"),
        ("printStandbySettings", "Chip standby:"),
    ):
        if f"void {helper}()" not in idf or output not in idf:
            fail(f"IDF live chip/internal query helper is incomplete: {helper}")
    if idf.count("printModeSettings();") < 2:
        fail("IDF normal/mode queries do not share the live chip/internal view")
    for call in ("printOsrsSettings();", "printFilterSettings();", "printStandbySettings();"):
        if call not in idf:
            fail(f"IDF setting query does not use its live chip/internal view: {call}")

    measurement_handler = re.search(
        r"void handleMeasurementReady\(\).*?\n}\n\nBME280::Status captureSensorSettings",
        idf,
        re.DOTALL,
    )
    if measurement_handler is None:
        fail("IDF pending-measurement completion handler could not be isolated")
    measurement_text = measurement_handler.group(0)
    for token in (
        "device.lastMeasurementStatus()",
        "driverMeasurementPending()",
        "gPendingRead = false",
        "noteStressError(terminalStatus)",
        "printStatus(terminalStatus)",
        "printStressProgress(",
    ):
        if token not in measurement_text:
            fail(f"IDF terminal async measurement handling is incomplete: {token}")

    stress_mix = re.search(
        r"void runStressMix\(.*?\n}\n\nvoid runSelfTest",
        idf,
        re.DOTALL,
    )
    if stress_mix is None:
        fail("IDF mixed-stress implementation could not be isolated")
    stress_text = stress_mix.group(0)
    for token in (
        '"measure"', '"readStatus"', '"readChipId"', '"readCalRaw"',
        '"setMode"', '"setFilter"', '"setStandby"',
        "cancelPendingForCommand()",
        "BME280::SensorSettings originalSettings",
        "captureSensorSettings(originalSettings)",
        "restoreSensorSettings(originalSettings)",
        '"  Restore status: %s\\n"',
        "printStressProgress(",
    ):
        if token not in stress_text:
            fail(f"IDF mixed-stress parity/state contract is incomplete: {token}")
    restore_pos = stress_text.find("restoreSensorSettings(originalSettings)")
    completion_pos = stress_text.find('"  Health delta:')
    if restore_pos < 0 or completion_pos < 0 or restore_pos > completion_pos:
        fail("IDF mixed-stress restore status must precede its completion marker")

    selftest = re.search(
        r"void runSelfTest\(\).*?\n}\n\nvoid printHelp",
        idf,
        re.DOTALL,
    )
    if selftest is None:
        fail("IDF selftest implementation could not be isolated")
    selftest_text = selftest.group(0)
    for token in (
        "const BME280::Status cancelStatus = cancelPending()",
        "capture baseline settings",
        "captureSensorSettings(baselineSettings)",
        "restoreSensorSettings(baselineSettings)",
        "restore baseline settings",
        "restoreStatus.ok()",
    ):
        if token not in selftest_text:
            fail(f"IDF selftest state-preservation contract is incomplete: {token}")

    restore_helper = re.search(
        r"BME280::Status captureSensorSettings\(.*?\n}\n\nvoid runStressMix",
        idf,
        re.DOTALL,
    )
    if restore_helper is None:
        fail("IDF full-settings restore helper could not be isolated")
    restore_text = restore_helper.group(0)
    for token in (
        "settings.mode = snapshot.mode",
        "settings.osrsT = snapshot.osrsT",
        "settings.osrsP = snapshot.osrsP",
        "settings.osrsH = snapshot.osrsH",
        "settings.filter = snapshot.filter",
        "settings.standby = snapshot.standby",
        "BME280::validateSettings(settings)",
        "device.getSettings(restored)",
        "sensorSettingsMatch(settings, restored)",
        "actual.initialized",
        "!actual.hardwareConfigDirty",
        "actual.mode == expected.mode",
        "actual.osrsT == expected.osrsT",
        "actual.osrsP == expected.osrsP",
        "actual.osrsH == expected.osrsH",
        "actual.filter == expected.filter",
        "actual.standby == expected.standby",
        "BME280::Err::RESYNC_REQUIRED",
    ):
        if token not in restore_text:
            fail(f"IDF restored-settings verification missing token: {token}")

    recover_handler = re.search(
        r'} else if \(std::strcmp\(head, "recover"\) == 0\).*?'
        r'} else if \(std::strcmp\(head, "invalidate"\) == 0\)',
        idf,
        re.DOTALL,
    )
    if recover_handler is None:
        fail("IDF direct recover handler could not be isolated")
    recover_text = recover_handler.group(0)
    cancel_pos = recover_text.find("cancelPendingForRecovery()")
    recover_pos = recover_text.find("device.recover()")
    if cancel_pos < 0 or recover_pos < 0 or cancel_pos > recover_pos:
        fail("IDF direct recover must cancel staged and measurement work first")

    recovery_cancel = re.search(
        r"bool cancelPendingForRecovery\(\).*?\n}\n\nbool driverMeasurementPending",
        idf,
        re.DOTALL,
    )
    if recovery_cancel is None:
        fail("IDF recovery cancellation helper could not be isolated")
    recovery_cancel_text = recovery_cancel.group(0)
    for token in (
        "device.jobState()",
        "device.cancelJob(BME280::CancelReason::OWNER_REQUEST)",
        "device.pollJob(currentMs(), 0U)",
        "return cancelPendingForCommand();",
    ):
        if token not in recovery_cancel_text:
            fail(f"IDF direct recovery ownership contract missing token: {token}")

    input_task = re.search(
        r"void inputTask\(.*?\n}\n\nvoid tickApp",
        idf,
        re.DOTALL,
    )
    if input_task is None:
        fail("IDF CLI input task could not be isolated")
    input_text = input_task.group(0)
    for token in (
        "const bool terminated",
        "len == sizeof(buffer) - 1U",
        "std::fgetc(stdin)",
        "next != '\\n'",
        "next != '\\r'",
        'std::printf("Command too long\\n")',
        "continue;",
        "char buffer[CLI_LINE_LEN]",
    ):
        if token not in input_text:
            fail(f"IDF overlong-input discard contract missing token: {token}")
    if idf.count("printStressProgress(") < 6:
        fail("IDF async and mixed stress paths lost periodic progress output")
    if arduino.count("Progress: %lu/%lu") != 1 or idf.count("Progress: %lu/%lu") != 1:
        fail("Arduino/IDF stress progress output format diverged")

    arduino_selftest = re.search(
        r"void runSelfTest\(\).*?\n}\n\nvoid clearPendingBookkeeping",
        arduino,
        re.DOTALL,
    )
    if arduino_selftest is None:
        fail("Arduino selftest implementation could not be isolated")
    if 'reportSkip("restore baseline settings", "baseline unavailable")' not in arduino_selftest.group(0):
        fail("Arduino selftest must skip restore when no baseline was captured")

    settings_handler = re.search(
        r"void handleSettingsCommand\(.*?\n}\n\nvoid printSampleFreshness",
        idf,
        re.DOTALL,
    )
    if settings_handler is None:
        fail("IDF whole-settings command handler could not be isolated")
    handler_text = settings_handler.group(0)
    validation_pos = handler_text.find("BME280::validateSettings(settings)")
    cancellation_pos = handler_text.find("cancelPendingForCommand()")
    if validation_pos < 0 or cancellation_pos < 0 or validation_pos > cancellation_pos:
        fail("IDF settings validation must happen before pending-work cancellation/I2C")
    if not re.search(
        r"startApplySettingsJob\(settings\);\s*if \(status\.inProgress\(\)\) \{\s*clearPendingBookkeeping\(\);",
        handler_text,
    ):
        fail("IDF accepted settings start must clear cancelled example measurement bookkeeping")

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
