#!/usr/bin/env python3
"""Pin checker rules independently and exercise their real entry points."""

from __future__ import annotations

import contextlib
import io
import pathlib
import tempfile
import unittest
from unittest import mock

import check_cli_contract
import check_core_timing_guard
import check_package_contents
import test_check_package_contents as package_tests


ROOT = pathlib.Path(__file__).resolve().parents[1]

# These are deliberately independent of the production checker lists. Removing
# a rule must fail even while the current repository still satisfies the rules.
EXPECTED_CALLS = {
    "millis", "micros", "delay", "delayMicroseconds", "yield", "vTaskDelay",
    "esp_timer_get_time",
}
EXPECTED_COMMANDS = {
    "help", "version", "scan", "addr", "begin", "end", "probe", "recover",
    "job", "drv", "state", "read", "raw", "comp", "data", "measuring",
    "timing", "status", "id", "chipid", "force", "normal", "mode", "osrs",
    "filter", "standby", "cfg", "settings", "calib", "reset", "reg", "dump",
    "rregs", "wreg", "invalidate", "freshness", "xfer_reset", "xfer_stats",
    "xfer_assert", "selftest", "stress", "stress_mix", "verbose",
}
EXPECTED_COMMON_HEADERS = {
    "BoardConfig.h", "BuildConfig.h", "CliStyle.h", "HealthView.h",
    "I2cScanner.h", "I2cTransport.h", "Log.h",
}


def run_checker(checker, root: pathlib.Path) -> tuple[int, str]:
    output = io.StringIO()
    with mock.patch.object(checker, "ROOT", root), contextlib.redirect_stdout(output):
        try:
            result = checker.main()
        except SystemExit as exc:
            result = int(exc.code)
    return result, output.getvalue()


class CoreTimingRulesTest(unittest.TestCase):
    def test_expected_calls_are_required(self) -> None:
        self.assertLessEqual(EXPECTED_CALLS, set(check_core_timing_guard.FORBIDDEN_CALLS))

    def check_source(self, source: str, expected_error: str | None = None) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            for relative in ("src/fixture.cpp", "include/BME280/Fixture.h"):
                with self.subTest(path=relative):
                    path = root / relative
                    path.parent.mkdir(parents=True, exist_ok=True)
                    path.write_text(source, encoding="utf-8")
                    result, output = run_checker(check_core_timing_guard, root)
                    self.assertEqual(1 if expected_error else 0, result, output)
                    self.assertIn(expected_error or "Core timing guard PASSED", output)
                    path.unlink()

    def test_framework_neutral_source_passes(self) -> None:
        self.check_source('void tick(unsigned nowMs) {}\n// vTaskDelay(1);\n')

    def test_each_platform_call_is_rejected(self) -> None:
        for call in sorted(EXPECTED_CALLS):
            with self.subTest(call=call):
                self.check_source(f"void work() {{ {call}(1); }}\n", f": {call} x1")

    def test_each_framework_include_is_rejected(self) -> None:
        for header in ("Arduino.h", "Wire.h", "driver/i2c_master.h",
                       "esp_timer.h", "freertos/task.h"):
            with self.subTest(header=header):
                self.check_source(f"#include <{header}>\n", "framework include in")


class CliRulesTest(unittest.TestCase):
    def setUp(self) -> None:
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.root = pathlib.Path(directory.name)
        common = self.root / "examples/common"
        common.mkdir(parents=True)
        for header in EXPECTED_COMMON_HEADERS:
            (common / header).write_text("", encoding="utf-8")
        relative = "examples/01_basic_bringup_cli/main.cpp"
        self.source = (ROOT / relative).read_text(encoding="utf-8")
        self.main = self.root / relative
        self.main.parent.mkdir(parents=True)
        self.main.write_text(self.source, encoding="utf-8")

    def test_expected_commands_and_headers_are_required(self) -> None:
        self.assertLessEqual(EXPECTED_COMMANDS, set(check_cli_contract.COMMANDS))
        self.assertLessEqual(EXPECTED_COMMON_HEADERS, set(check_cli_contract.REQUIRED_COMMON))

    def test_complete_cli_passes(self) -> None:
        result, output = run_checker(check_cli_contract, self.root)
        self.assertEqual(0, result, output)
        self.assertIn("CLI contract PASSED", output)

    def test_each_missing_command_handler_is_rejected(self) -> None:
        for command in sorted(EXPECTED_COMMANDS):
            with self.subTest(command=command):
                mutated = self.source
                for comparison in (f'cmd == "{command}"',
                                   f'cmd.startsWith("{command} ")',
                                   f'cmd.startsWith("{command}")'):
                    mutated = mutated.replace(comparison, "false")
                self.assertNotEqual(self.source, mutated, "fixture mutation did not apply")
                self.main.write_text(mutated, encoding="utf-8")
                result, output = run_checker(check_cli_contract, self.root)
                self.assertEqual(1, result, output)
                self.assertIn(f"mandatory command '{command}' has no visible handler", output)


class PackageRulesTest(package_tests.PackageFixture):
    def test_expected_paths_are_required(self) -> None:
        self.assertLessEqual(package_tests.EXPECTED_BASE_REQUIRED_PATHS,
                             set(check_package_contents.BASE_REQUIRED_PATHS))

    def test_each_missing_required_path_is_rejected(self) -> None:
        for path in sorted(package_tests.EXPECTED_BASE_REQUIRED_PATHS):
            with self.subTest(path=path):
                contents = self.archive_contents()
                del contents[path]
                self.write_archive(contents)
                self.assert_checker_rejects(f"missing required files: {path}")


if __name__ == "__main__":
    unittest.main()
