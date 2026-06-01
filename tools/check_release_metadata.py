#!/usr/bin/env python3
from __future__ import annotations

import json
import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]


def fail(message: str) -> None:
    print(f"Release metadata FAILED: {message}")
    raise SystemExit(1)


def read(path: pathlib.Path) -> str:
    if not path.exists():
        fail(f"missing file: {path.relative_to(ROOT).as_posix()}")
    return path.read_text(encoding="utf-8", errors="replace")


def main() -> int:
    with open(ROOT / "library.json", "r", encoding="utf-8") as handle:
        library = json.load(handle)
    version = str(library.get("version", ""))
    match = re.fullmatch(r"(\d+)\.(\d+)\.(\d+)", version)
    if not match:
        fail(f"library.json version is not SemVer X.Y.Z: {version!r}")

    major, minor, patch = (int(part) for part in match.groups())
    version_code = major * 10000 + minor * 100 + patch

    version_h = read(ROOT / "include" / "BME280" / "Version.h")
    expected_version_tokens = [
        f'#define BME280_VERSION_STRING "{version}"',
        f"static constexpr uint16_t VERSION_MAJOR = {major};",
        f"static constexpr uint16_t VERSION_MINOR = {minor};",
        f"static constexpr uint16_t VERSION_PATCH = {patch};",
        f"static constexpr uint32_t VERSION_CODE = {version_code};",
        f"static constexpr int VERSION_INT = {version_code};",
    ]
    for token in expected_version_tokens:
        if token not in version_h:
            fail(f"Version.h missing or mismatched token: {token}")

    idf_component = read(ROOT / "idf_component.yml")
    if f'version: "{version}"' not in idf_component:
        fail("idf_component.yml version does not match library.json")

    doxyfile = read(ROOT / "Doxyfile")
    if f'PROJECT_NUMBER         = "{version}"' not in doxyfile:
        fail("Doxyfile PROJECT_NUMBER does not match library.json")

    changelog = read(ROOT / "CHANGELOG.md")
    if f"## [{version}]" not in changelog:
        fail(f"CHANGELOG.md is missing a {version} release section")
    if f"[Unreleased]: https://github.com/janhavelka/BME280/compare/v{version}...HEAD" not in changelog:
        fail("CHANGELOG.md Unreleased compare link does not start at current version")

    print(f"Release metadata PASSED ({version})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
