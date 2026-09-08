#!/usr/bin/env python3
"""Pin checker rules independently and exercise their real entry points."""

from __future__ import annotations

import contextlib
import io
import json
import pathlib
import tempfile
import unittest
from unittest import mock

import check_cli_contract
import check_core_timing_guard
import check_hil_contract
import check_idf_example_contract
import check_package_contents
import check_release_metadata
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
EXPECTED_IDF_FORBIDDEN = {
    "Arduino.h", "Wire.h", "IdfArduinoCompat", "ArduinoCompat", "TwoWire",
    "Serial", "examples/01_basic_bringup_cli/main.cpp", "setup();", "loop();",
}
EXPECTED_IDF_PATTERNS = {
    "millis() shim or call": "millis()",
    "Arduino delay() call": "delay(1)",
    "Arduino String type": "String value",
}
EXPECTED_IDF_NATIVE = {
    'extern "C" void app_main(void)', "driver/i2c_master.h", "i2c_master_probe",
    "i2c_new_master_bus", "i2c_master_transmit", "i2c_master_transmit_receive",
    "esp_timer_get_time", "vTaskDelay", "xTaskCreate", "QueueHandle_t",
    "LOG_COLOR_GREEN", "LOG_COLOR_YELLOW", "LOG_COLOR_RED",
}
EXPECTED_IDF_CMAKE_TOKENS = {
    'get_filename_component(BME280_ROOT_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../.." ABSOLUTE)',
    'get_filename_component(BME280_COMPONENT_NAME "${BME280_ROOT_DIR}" NAME)',
    "REQUIRES ${BME280_COMPONENT_NAME} esp_driver_i2c esp_driver_gpio esp_timer freertos",
}


def run_checker(checker, root: pathlib.Path) -> tuple[int, str]:
    output = io.StringIO()
    with (mock.patch.object(checker, "ROOT", root),
          contextlib.redirect_stdout(output), contextlib.redirect_stderr(output)):
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

    def test_string_comment_markers_do_not_hide_platform_calls(self) -> None:
        for body in (
            'const char* url = "https://example.com"; delay(1);',
            'const char* start = "/*"; delay(1); const char* end = "*/";',
        ):
            with self.subTest(body=body):
                self.check_source(
                    f"void work() {{ {body} }}\n",
                    ": delay x1",
                )

    def test_literals_and_comments_with_timing_text_pass(self) -> None:
        self.check_source(
            'const char* text = "https://example.com/delay(1)";\n'
            '// "unclosed quote: delay(1)\n'
            '/* "another unclosed quote: millis() */\n'
            'void work() {}\n'
        )

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


class TextContractTest(unittest.TestCase):
    """Supply changed text at the read boundary without editing repository files."""

    def assert_text_result(self, checker, changes: dict[pathlib.Path, str],
                           expected_error: str | None = None) -> None:
        original_read = checker.read

        def read(path: pathlib.Path) -> str:
            return changes[path] if path in changes else original_read(path)

        with mock.patch.object(checker, "read", side_effect=read):
            result, output = run_checker(checker, ROOT)
        self.assertEqual(1 if expected_error else 0, result, output)
        self.assertIn(expected_error or "PASSED", output)


class IdfRulesTest(TextContractTest):
    def test_expected_rules_are_required(self) -> None:
        checker = check_idf_example_contract
        self.assertLessEqual(EXPECTED_IDF_FORBIDDEN, set(checker.FORBIDDEN_IDF_TOKENS))
        self.assertLessEqual(set(EXPECTED_IDF_PATTERNS), set(checker.FORBIDDEN_IDF_PATTERNS))
        self.assertLessEqual(EXPECTED_IDF_NATIVE, set(checker.REQUIRED_IDF_TOKENS))
        self.assertLessEqual(EXPECTED_IDF_CMAKE_TOKENS, set(checker.REQUIRED_IDF_CMAKE_TOKENS))
        self.assertLessEqual(EXPECTED_COMMANDS | {"?", "ver"}, checker.MANDATORY_COMMANDS)

    def test_native_example_passes(self) -> None:
        self.assert_text_result(check_idf_example_contract, {})

    def test_hard_coded_component_name_is_rejected(self) -> None:
        checker = check_idf_example_contract
        source = checker.IDF_CMAKE.read_text(encoding="utf-8")
        dependency = "REQUIRES ${BME280_COMPONENT_NAME} esp_driver_i2c esp_driver_gpio esp_timer freertos"
        self.assertIn(dependency, source)
        for keyword in ("REQUIRES", "PRIV_REQUIRES"):
            with self.subTest(keyword=keyword):
                changed = source.replace("REQUIRES ${BME280_COMPONENT_NAME}", f"{keyword} BME280")
                # Retain the positive token in a comment so only the negative
                # guard can reject the actual hard-coded dependency above.
                changed += f"\n# {dependency}\n"
                self.assert_text_result(checker, {checker.IDF_CMAKE: changed},
                                        "IDF main component must not hard-code the root component name")

    def test_missing_component_name_derivation_tokens_are_rejected(self) -> None:
        checker = check_idf_example_contract
        source = checker.IDF_CMAKE.read_text(encoding="utf-8")
        for token in sorted(EXPECTED_IDF_CMAKE_TOKENS):
            with self.subTest(token=token):
                self.assertIn(token, source)
                self.assert_text_result(checker, {checker.IDF_CMAKE: source.replace(token, "")},
                                        f"IDF main-component dependency contract missing: {token}")

    def test_arduino_tokens_and_calls_are_rejected(self) -> None:
        checker = check_idf_example_contract
        path = checker.IDF_MAIN
        source = path.read_text(encoding="utf-8")
        for token in sorted(EXPECTED_IDF_FORBIDDEN | set(EXPECTED_IDF_PATTERNS.values())):
            with self.subTest(token=token):
                self.assert_text_result(checker, {path: source + f"\n{token};\n"},
                                        "IDF example uses forbidden Arduino")

    def test_missing_native_tokens_are_rejected(self) -> None:
        checker = check_idf_example_contract
        paths = (checker.IDF_MAIN, checker.IDF_TRANSPORT,
                 checker.IDF_TRANSPORT.with_suffix(".h"))
        originals = {path: path.read_text(encoding="utf-8") for path in paths}
        for token in sorted(EXPECTED_IDF_NATIVE):
            with self.subTest(token=token):
                self.assertIn(token, "\n".join(originals.values()))
                changes = {path: text.replace(token, "removed_native_token")
                           for path, text in originals.items()}
                self.assert_text_result(checker, changes,
                                        f"IDF example missing required native token: {token}")

    def test_each_missing_command_handler_is_rejected(self) -> None:
        checker = check_idf_example_contract
        source = checker.IDF_MAIN.read_text(encoding="utf-8")
        for command in sorted(EXPECTED_COMMANDS | {"?", "ver"}):
            with self.subTest(command=command):
                comparison = f'std::strcmp(head, "{command}") == 0'
                self.assertIn(comparison, source, "fixture mutation did not apply")
                changed = source.replace(comparison, "false")
                self.assert_text_result(checker, {checker.IDF_MAIN: changed},
                                        f"IDF CLI missing mandatory commands: ['{command}']")


class ReleaseMetadataTest(unittest.TestCase):
    def setUp(self) -> None:
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.root = pathlib.Path(directory.name)
        self.contents = {
            "library.json": json.dumps({"version": "2.3.4"}),
            "include/BME280/Version.h": (
                '#define BME280_VERSION_STRING "2.3.4"\n'
                "static constexpr uint16_t VERSION_MAJOR = 2;\n"
                "static constexpr uint16_t VERSION_MINOR = 3;\n"
                "static constexpr uint16_t VERSION_PATCH = 4;\n"
                "static constexpr uint32_t VERSION_CODE = 20304;\n"
                "static constexpr int VERSION_INT = 20304;\n"
            ),
            "idf_component.yml": 'version: "2.3.4"\n',
            "Doxyfile": 'PROJECT_NUMBER         = "2.3.4"\n',
            "CHANGELOG.md": (
                "## [2.3.4]\n"
                "[Unreleased]: https://github.com/janhavelka/BME280/compare/v2.3.4...HEAD\n"
            ),
        }
        for relative, content in self.contents.items():
            path = self.root / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(content, encoding="utf-8")

    def test_consistent_metadata_passes(self) -> None:
        result, output = run_checker(check_release_metadata, self.root)
        self.assertEqual(0, result, output)
        self.assertIn("Release metadata PASSED (2.3.4)", output)

    def test_each_metadata_mismatch_is_rejected(self) -> None:
        cases = [
            ("library.json", '{"version": "2.3"}', "not SemVer X.Y.Z"),
            ("idf_component.yml", 'version: "2.3.5"\n', "idf_component.yml version"),
            ("idf_component.yml", 'version: 2.3.4\n', "idf_component.yml version"),
            ("Doxyfile", 'PROJECT_NUMBER         = "2.3.5"\n', "Doxyfile PROJECT_NUMBER"),
        ]
        for line in self.contents["include/BME280/Version.h"].splitlines(keepends=True):
            cases.append(("include/BME280/Version.h",
                          self.contents["include/BME280/Version.h"].replace(line, ""),
                          "Version.h missing or mismatched token"))
        for line, error in zip(self.contents["CHANGELOG.md"].splitlines(keepends=True),
                               ("missing a 2.3.4 release section", "Unreleased compare link")):
            cases.append(("CHANGELOG.md", self.contents["CHANGELOG.md"].replace(line, ""), error))
        for relative, changed, expected_error in cases:
            with self.subTest(path=relative, changed=changed):
                path = self.root / relative
                path.write_text(changed, encoding="utf-8")
                try:
                    result, output = run_checker(check_release_metadata, self.root)
                    self.assertEqual(1, result, output)
                    self.assertIn(expected_error, output)
                finally:
                    path.write_text(self.contents[relative], encoding="utf-8")


class HilRulesTest(TextContractTest):
    def test_current_runner_and_documentation_pass(self) -> None:
        self.assert_text_result(check_hil_contract, {})

    def test_changed_default_sequence_is_rejected(self) -> None:
        checker = check_hil_contract
        text = checker.VALIDATION.read_text(encoding="utf-8")
        marker = "<!-- HIL_DEFAULT_SEQUENCE_START -->"
        self.assertIn(marker, text)
        self.assert_text_result(checker, {checker.VALIDATION: text.replace(
            marker, marker + "\nunexpected_command", 1
        )}, "documented default sequence differs")

    def test_missing_documentation_requirements_are_rejected(self) -> None:
        checker = check_hil_contract
        text = checker.VALIDATION.read_text(encoding="utf-8")
        for token in ("--require-pass", "--include-destructive --confirm-raw-write BME280_RAW_WRITE",
                      "cannot qualify exact build provenance"):
            with self.subTest(token=token):
                self.assertIn(token, text)
                self.assert_text_result(checker, {checker.VALIDATION: text.replace(token, "")},
                                        f"is missing required text: {token}")

    def test_unsupported_hardware_claims_are_rejected(self) -> None:
        checker = check_hil_contract
        text = checker.README.read_text(encoding="utf-8")
        for claim in ("Hardware run: PASS", "Physical HIL: PASS", "HIL validated: PASS"):
            with self.subTest(claim=claim):
                self.assert_text_result(checker, {checker.README: text + f"\n{claim}\n"},
                                        f"unsupported hardware claim: {claim}")

    def test_missing_final_recovery_is_rejected(self) -> None:
        checker = check_hil_contract
        runner = checker.load_runner()
        cleanup = runner.final_cleanup_commands()
        self.assertIn("recover", [spec.command for spec in cleanup])
        with (mock.patch.object(checker, "load_runner", return_value=runner),
              mock.patch.object(runner, "final_cleanup_commands", return_value=[
                  spec for spec in cleanup if spec.command != "recover"
              ])):
            self.assert_text_result(checker, {}, "final cleanup has an unexpected command shape")


if __name__ == "__main__":
    unittest.main()
