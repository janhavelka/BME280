#!/usr/bin/env python3
"""Focused tests for package archive path and metadata validation."""

from __future__ import annotations

import contextlib
import io
import json
import pathlib
import sys
import tarfile
import tempfile
import unittest
from unittest import mock


ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

import check_package_contents  # noqa: E402


VERSION = "2.1.0"

# Independent expectations: never build fixtures from the checker's rule list.
EXPECTED_BASE_REQUIRED_PATHS = {
    "CHANGELOG.md",
    "LICENSE",
    "README.md",
    "library.json",
    "include/BME280/BME280.h",
    "include/BME280/CommandTable.h",
    "include/BME280/Config.h",
    "include/BME280/Status.h",
    "include/BME280/Version.h",
    "CMakeLists.txt",
    "idf_component.yml",
    "examples/idf/basic/CMakeLists.txt",
    "examples/idf/basic/main/CMakeLists.txt",
    "examples/01_basic_bringup_cli/main.cpp",
    "examples/common/BoardConfig.h",
    "examples/common/BuildConfig.h",
    "examples/common/CliStyle.h",
    "examples/common/HealthView.h",
    "examples/common/I2cScanner.h",
    "examples/common/I2cTransport.h",
    "examples/common/Log.h",
    "docs/README.md",
    "docs/IDF_PORT.md",
    "docs/PRODUCTION_SHARED_BUS_GUIDE.md",
    "docs/HARDWARE_VALIDATION.md",
    "docs/BME280_Register_Reference.md",
    "docs/BME280_datasheet.pdf",
    "src/BME280.cpp",
}


class PackageFixture(unittest.TestCase):
    """Temporary archive shared by package behavior and rule-preservation tests."""

    def setUp(self) -> None:
        self.temp_dir = tempfile.TemporaryDirectory()
        self.root = pathlib.Path(self.temp_dir.name)
        self.idf_main_dir = self.root / "examples" / "idf" / "basic" / "main"
        self.idf_main_dir.mkdir(parents=True)
        (self.root / "library.json").write_text(
            json.dumps({"version": VERSION}), encoding="utf-8"
        )
        (self.idf_main_dir / "CMakeLists.txt").write_text(
            'idf_component_register(SRCS "main.cpp")\n', encoding="utf-8"
        )
        (self.idf_main_dir / "main.cpp").write_text("", encoding="utf-8")

    def tearDown(self) -> None:
        self.temp_dir.cleanup()

    def archive_contents(self) -> dict[str, bytes]:
        paths = set(EXPECTED_BASE_REQUIRED_PATHS)
        paths.add("examples/idf/basic/main/main.cpp")
        contents = {path: b"test\n" for path in paths}
        contents["library.json"] = json.dumps({"version": VERSION}).encode()
        contents["idf_component.yml"] = f'version: "{VERSION}"\n'.encode()
        contents["include/BME280/Version.h"] = (
            f'#define BME280_VERSION_STRING "{VERSION}"\n'.encode()
        )
        return contents

    def write_archive(self, contents: dict[str, bytes], prefix: str = "") -> None:
        archive = self.root / f"BME280-{VERSION}.tar.gz"
        with tarfile.open(archive, "w:gz") as tar:
            for path, data in sorted(contents.items()):
                info = tarfile.TarInfo(f"{prefix}{path}")
                info.size = len(data)
                tar.addfile(info, io.BytesIO(data))

    def run_checker(self) -> tuple[int, str]:
        output = io.StringIO()
        with (
            mock.patch.object(check_package_contents, "ROOT", self.root),
            mock.patch.object(check_package_contents, "IDF_MAIN_DIR", self.idf_main_dir),
            contextlib.redirect_stdout(output),
        ):
            try:
                result = check_package_contents.main()
            except SystemExit as exc:
                result = int(exc.code)
        return result, output.getvalue()

    def assert_checker_passes(self) -> None:
        result, output = self.run_checker()
        self.assertEqual(0, result, output)
        self.assertIn("Package contents PASSED", output)

    def assert_checker_rejects(self, expected: str) -> None:
        result, output = self.run_checker()
        self.assertEqual(1, result, output)
        self.assertIn(expected, output)


class PackageContentsTest(PackageFixture):
    def test_flat_members_pass(self) -> None:
        self.write_archive(self.archive_contents())
        self.assert_checker_passes()

    def test_dot_slash_prefixed_members_pass(self) -> None:
        self.write_archive(self.archive_contents(), "./")
        self.assert_checker_passes()

    def test_package_root_prefixed_members_pass(self) -> None:
        self.write_archive(self.archive_contents(), f"BME280-{VERSION}/")
        self.assert_checker_passes()

    def test_forbidden_paths_are_rejected(self) -> None:
        forbidden_paths = (
            ".pio/build/output.bin",
            ".git/config",
            "tools/__pycache__/checker.pyc",
            "BME280-0.0.0.tar.gz",
            "docs/doxygen/html/index.html",
            "hil_logs/session/summary.json",
            "nested/.pio/build/output.bin",
            "nested/.git/config",
            "__pycache__/checker.pyc",
            "nested/docs/doxygen/html/index.html",
            "nested/hil_logs/session/summary.json",
            "nested/BME280-0.0.0.tar.gz",
        )
        for forbidden_path in forbidden_paths:
            with self.subTest(path=forbidden_path):
                contents = self.archive_contents()
                contents[forbidden_path] = b"forbidden\n"
                self.write_archive(contents)
                self.assert_checker_rejects("forbidden build/internal paths")

    def test_missing_common_headers_are_rejected(self) -> None:
        for path in sorted(EXPECTED_BASE_REQUIRED_PATHS):
            if path.startswith("examples/common/"):
                with self.subTest(path=path):
                    contents = self.archive_contents()
                    del contents[path]
                    self.write_archive(contents)
                    self.assert_checker_rejects(f"missing required files: {path}")

    def test_prefixed_archive_reaches_and_passes_version_validation(self) -> None:
        self.write_archive(self.archive_contents(), f"BME280-{VERSION}/")
        with mock.patch.object(
            check_package_contents,
            "validate_packaged_versions",
            wraps=check_package_contents.validate_packaged_versions,
        ) as validate_versions:
            self.assert_checker_passes()
        validate_versions.assert_called_once()


if __name__ == "__main__":
    unittest.main()
