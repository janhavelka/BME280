#!/usr/bin/env python3
from __future__ import annotations

import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]

REQUIRED_COMMON = [
    "BoardConfig.h",
    "BuildConfig.h",
    "Log.h",
    "I2cTransport.h",
    "I2cScanner.h",
    "CliStyle.h",
    "HealthView.h",
]

MANDATORY_COMMANDS = [
    "help",
    "version",
    "scan",
    "addr",
    "begin",
    "end",
    "probe",
    "recover",
    "job",
    "drv",
    "state",
    "read",
    "raw",
    "comp",
    "data",
    "measuring",
    "timing",
    "status",
    "id",
    "chipid",
    "force",
    "normal",
    "mode",
    "osrs",
    "filter",
    "standby",
    "cfg",
    "settings",
    "calib",
    "reset",
    "reg",
    "dump",
    "rregs",
    "wreg",
    "invalidate",
    "freshness",
    "xfer_reset",
    "xfer_stats",
    "xfer_assert",
    "selftest",
    "stress",
    "stress_mix",
    "verbose",
]

HANDLED_COMMANDS = [
    "version",
    "scan",
    "addr",
    "begin",
    "end",
    "probe",
    "recover",
    "job",
    "drv",
    "state",
    "read",
    "raw",
    "comp",
    "data",
    "measuring",
    "timing",
    "status",
    "id",
    "chipid",
    "force",
    "normal",
    "mode",
    "osrs",
    "filter",
    "standby",
    "cfg",
    "settings",
    "calib",
    "reset",
    "reg",
    "dump",
    "rregs",
    "wreg",
    "invalidate",
    "freshness",
    "xfer_reset",
    "xfer_stats",
    "xfer_assert",
    "selftest",
    "stress",
    "stress_mix",
    "verbose",
]


def fail(msg: str) -> None:
    print(f"CLI contract FAILED: {msg}")
    raise SystemExit(1)


def ensure_exists(path: pathlib.Path, label: str) -> None:
    if not path.exists():
        fail(f"missing {label}: {path.as_posix()}")


def ensure_missing(path: pathlib.Path, label: str) -> None:
    if path.exists():
        fail(f"forbidden {label} still present: {path.as_posix()}")


def main() -> int:
    common_dir = ROOT / "examples" / "common"
    bringup_main = ROOT / "examples" / "01_basic_bringup_cli" / "main.cpp"

    ensure_exists(common_dir, "common example directory")
    ensure_exists(bringup_main, "bringup CLI example")

    ensure_missing(ROOT / "examples" / "00_smoke_boot", "deprecated example 00_smoke_boot")
    ensure_missing(
        ROOT / "examples" / "03_feature_walkthrough",
        "deprecated example 03_feature_walkthrough",
    )

    for name in REQUIRED_COMMON:
        ensure_exists(common_dir / name, f"common helper {name}")

    text = bringup_main.read_text(encoding="utf-8", errors="replace")

    for cmd in MANDATORY_COMMANDS:
        if re.search(rf"\b{re.escape(cmd)}\b", text) is None:
            fail(f"mandatory command '{cmd}' missing in {bringup_main.as_posix()}")

    for cmd in HANDLED_COMMANDS:
        pattern = rf'cmd(?:\s*==\s*"{re.escape(cmd)}"|\.startsWith\("{re.escape(cmd)}(?: |")|\s*==\s*"{re.escape(cmd)}")'
        if re.search(pattern, text) is None:
            fail(f"mandatory command '{cmd}' has no visible handler")

    if re.search(r"\bcfg\b", text) is None and re.search(r"\bsettings\b", text) is None:
        fail("either 'cfg' or 'settings' command must be present")

    for token in (
        "startResyncJob",
        "startRecoveryJob",
        "startSoftResetJob",
        "cancelJob",
        "CancelReason::OWNER_REQUEST",
        "CancelReason::DEADLINE_EXPIRED",
        'pollJob(millis(), 0U)',
        '"Boundary: %s\\n"',
        '"Job ID: %lu\\n"',
        '"Job phase: %s\\n"',
        '"Terminal state: %s\\n"',
        '"Conversion state: %s\\n"',
        '"Phase deadline active: %s\\n"',
        '"Phase deadline ms: %lu\\n"',
        '"Callbacks used: %u\\n"',
        "BME280::toString(result.status.code)",
        "startApplySettingsJob",
        "BME280::validateSettings(settings)",
        "invalidateDeviceState",
        "sampleFresh(now, maxAgeMs)",
        "readRegisters",
        "resetTransferStats",
        "transferStats",
        "XFER_ASSERT PASS",
        "XFER_ASSERT FAIL",
        "JOB_CLI_MAX_POLLS = 1024U",
        "=== Lifecycle ===",
        "Action: END",
        "=== Sample Freshness ===",
        "=== Register Block ===",
        "XFER_RESET read=0 write=0 total=0",
        "XFER_STATS read=",
        "ms_0_5",
        "ms_1000",
        "BME280::Status cancelPending()",
        "cancelPendingForCommand()",
        "MAX_STRESS_COUNT = 100000U",
        "parseU32(cmd.substring(7), count)",
        "parseU32(cmd.substring(11), count)",
        "const char* noArgumentUsage(const String& command)",
        "const char* usage = noArgumentUsage(commandHead)",
        'cmd.replace("  ", " ")',
    ):
        if token not in text:
            fail(f"staged-job CLI contract missing token: {token}")

    no_arg_usage = re.search(
        r"const char\* noArgumentUsage\(.*?\n}\n\nbool parseSensorSettings",
        text,
        re.DOTALL,
    )
    if no_arg_usage is None:
        fail("strict no-argument usage mapping could not be isolated")
    no_arg_text = no_arg_usage.group(0)
    for command, usage in (
        ("help", "help|?"), ("?", "help|?"),
        ("version", "version|ver"), ("ver", "version|ver"), ("scan", "scan"),
        ("begin", "begin"), ("end", "end"), ("read", "read"),
        ("force", "force"), ("raw", "raw"), ("comp", "comp"),
        ("data", "data"), ("measuring", "measuring"), ("timing", "timing"),
        ("cfg", "cfg"), ("status", "status"),
        ("chipid", "chipid|id"), ("id", "chipid|id"), ("reset", "reset"),
        ("drv", "drv"), ("state", "state"), ("probe", "probe"),
        ("recover", "recover"), ("invalidate", "invalidate"),
        ("xfer_reset", "xfer_reset"), ("xfer_stats", "xfer_stats"),
        ("selftest", "selftest"),
    ):
        if f'command == "{command}"' not in no_arg_text or f'return "{usage}";' not in no_arg_text:
            fail(f"strict no-argument usage mapping missing: {command} -> {usage}")
    for fragment in (
        'if (cmd.startsWith("normal "))',
        'LOGW("Usage: normal on|off")',
        'if (cmd.startsWith("calib "))',
        'LOGW("Usage: calib [raw]")',
        "if (!splitExactTokens(args, tokens, 2U))",
        'if (cmd == "reg")',
        'if (cmd == "wreg")',
    ):
        if fragment not in text:
            fail(f"strict malformed/trailing-argument handling missing: {fragment}")

    for command, following_command in (("filter", "standby"), ("standby", "status")):
        setter_handler = re.search(
            rf'if \(cmd\.startsWith\("{command} "\)\) \{{(.*?)\n  \}}\n\n  if \(cmd == "{following_command}"\)',
            text,
            re.DOTALL,
        )
        if setter_handler is None:
            fail(f"{command} setter handler could not be isolated")
        if "if (value.indexOf(' ') >= 0)" not in setter_handler.group(1):
            fail(f"{command} setter must reject trailing arguments explicitly")

    settings_handler = re.search(
        r"void handleSettingsCommand\(.*?\n}\n\nvoid printSampleFreshness",
        text,
        re.DOTALL,
    )
    if settings_handler is None:
        fail("whole-settings command handler could not be isolated")
    handler_text = settings_handler.group(0)
    validation_pos = handler_text.find("BME280::validateSettings(settings)")
    cancellation_pos = handler_text.find("cancelPendingForCommand()")
    if validation_pos < 0 or cancellation_pos < 0 or validation_pos > cancellation_pos:
        fail("settings validation must happen before pending-work cancellation/I2C")
    if not re.search(
        r"startApplySettingsJob\(settings\);\s*if \(status\.inProgress\(\)\) \{\s*clearPendingBookkeeping\(\);",
        handler_text,
    ):
        fail("accepted settings start must clear cancelled example measurement bookkeeping")

    if not re.search(r'action == "resync"\)\s*\{\s*return device\.startResyncJob\(\);', text):
        fail("job resync must map to non-reset startResyncJob()")
    if not re.search(r'action == "reset"\)\s*\{\s*return device\.startSoftResetJob\(\);', text):
        fail("job reset must map to explicit startSoftResetJob()")

    stress_mix = re.search(
        r"void runStressMix\(.*?\n}\n\nvoid runSelfTest",
        text,
        re.DOTALL,
    )
    if stress_mix is None:
        fail("stress_mix implementation could not be isolated")
    stress_mix_text = stress_mix.group(0)
    restore_pos = stress_mix_text.find("restoreSensorSettings(originalSettings)")
    restore_label_pos = stress_mix_text.find('"Restore status: %s\\n"')
    completion_pos = stress_mix_text.find('"  Health delta:')
    if min(restore_pos, restore_label_pos, completion_pos) < 0 or not (
        restore_pos < restore_label_pos < completion_pos
    ):
        fail("stress_mix must restore and report status before completion evidence")
    if not re.search(
        r"if \(!restoreStatus\.ok\(\)\) \{.*?LOGE\(.*?printStatus\(restoreStatus\);",
        stress_mix_text,
        re.DOTALL,
    ):
        fail("stress_mix restore failure must emit hard-error status evidence")

    selftest = re.search(
        r"void runSelfTest\(\).*?\n}\n\nvoid clearPendingBookkeeping",
        text,
        re.DOTALL,
    )
    if selftest is None:
        fail("selftest implementation could not be isolated")
    selftest_text = selftest.group(0)
    restore_pos = selftest_text.find("restoreSensorSettings(originalSettings)")
    report_pos = selftest_text.find('reportCheck("restore baseline settings"')
    result_pos = selftest_text.rfind('"Selftest result:')
    if min(restore_pos, report_pos, result_pos) < 0 or not (
        restore_pos < report_pos < result_pos
    ):
        fail("selftest must count and report coherent restoration before its result")
    if "sensorSettingsMatch(settings, restored)" not in text:
        fail("settings restoration must verify the complete restored snapshot")

    pending_handler = re.search(
        r"void handlePendingMeasurement\(\).*?\n}\n\nbool parseU32",
        text,
        re.DOTALL,
    )
    if pending_handler is None:
        fail("pending-measurement handler could not be isolated")
    pending_text = pending_handler.group(0)
    if "device.lastMeasurementStatus()" not in pending_text or not re.search(
        r"!terminalStatus\.ok\(\)\s*&&\s*!terminalStatus\.inProgress\(\).*?"
        r"completePendingMeasurementFailure\(terminalStatus\)",
        pending_text,
        re.DOTALL,
    ):
        fail("pending reads/stress must consume terminal tick failures")

    recover_handler = re.search(
        r'if \(cmd == "recover"\) \{(.*?)\n  \}\n\n  if \(cmd == "invalidate"\)',
        text,
        re.DOTALL,
    )
    if recover_handler is None:
        fail("direct recover handler could not be isolated")
    recover_text = recover_handler.group(1)
    cancel_pos = recover_text.find("cancelPendingForRecovery()")
    recover_pos = recover_text.find("device.recover()")
    if cancel_pos < 0 or recover_pos < 0 or cancel_pos > recover_pos:
        fail("direct recover must cancel pending work first")
    recovery_cancel = re.search(
        r"bool cancelPendingForRecovery\(\).*?\n}\n\nBME280::JobPollResult",
        text,
        re.DOTALL,
    )
    if recovery_cancel is None:
        fail("recovery cancellation helper could not be isolated")
    recovery_cancel_text = recovery_cancel.group(0)
    if "device.cancelJob(BME280::CancelReason::OWNER_REQUEST)" not in recovery_cancel_text:
        fail("direct recover must cancel an active staged job")
    if "return cancelPendingForCommand();" not in recovery_cancel_text:
        fail("direct recover must cancel pending measurement work")

    print("CLI contract PASSED")
    return 0


if __name__ == "__main__":
    sys.exit(main())
