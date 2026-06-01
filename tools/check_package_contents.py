#!/usr/bin/env python3
from __future__ import annotations

import pathlib
import json
import re
import sys
import tarfile

ROOT = pathlib.Path(__file__).resolve().parents[1]

REQUIRED_SUFFIXES = {
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
    "examples/idf/basic/main/IdfI2cTransport.cpp",
    "examples/idf/basic/main/IdfI2cTransport.h",
    "examples/idf/basic/main/main.cpp",
    "src/BME280.cpp",
}

FORBIDDEN_PARTS = {
    ".git",
    ".pio",
    "__pycache__",
    "docs/doxygen",
}


def fail(message: str) -> None:
    print(f"Package contents FAILED: {message}")
    raise SystemExit(1)


def library_version() -> str:
    with open(ROOT / "library.json", "r", encoding="utf-8") as handle:
        data = json.load(handle)
    version = str(data.get("version", ""))
    if not re.match(r"^\d+\.\d+\.\d+$", version):
        fail(f"invalid library.json version: {version!r}")
    return version


def expected_package(version: str) -> pathlib.Path:
    archive = ROOT / f"BME280-{version}.tar.gz"
    if not archive.exists():
        fail(
            f"missing BME280-{version}.tar.gz archive; run "
            "'python -m platformio pkg pack' after updating library.json"
        )
    return archive


def normalize(name: str) -> str:
    return name.replace("\\", "/").lstrip("./")


def read_member(tar: tarfile.TarFile, members: set[str], suffix: str) -> str:
    matches = sorted(name for name in members if name.endswith(suffix))
    if len(matches) != 1:
        fail(f"expected exactly one packaged {suffix}, found {len(matches)}")
    handle = tar.extractfile(matches[0])
    if handle is None:
        fail(f"could not read packaged {suffix}")
    return handle.read().decode("utf-8", errors="replace")


def validate_packaged_versions(tar: tarfile.TarFile, members: set[str], version: str) -> None:
    packaged_library = json.loads(read_member(tar, members, "library.json"))
    packaged_component = read_member(tar, members, "idf_component.yml")
    packaged_version_h = read_member(tar, members, "include/BME280/Version.h")

    if packaged_library.get("version") != version:
        fail(
            "packaged library.json version mismatch: "
            f"{packaged_library.get('version')!r} != {version!r}"
        )
    if f'version: "{version}"' not in packaged_component:
        fail("packaged idf_component.yml version does not match library.json")
    if f'#define BME280_VERSION_STRING "{version}"' not in packaged_version_h:
        fail("packaged Version.h version does not match library.json")


def main() -> int:
    version = library_version()
    archive = expected_package(version)
    with tarfile.open(archive, "r:gz") as tar:
        members = {normalize(member.name) for member in tar.getmembers()}

    missing = []
    for suffix in sorted(REQUIRED_SUFFIXES):
        if not any(name.endswith(suffix) for name in members):
            missing.append(suffix)
    if missing:
        fail("missing required files: " + ", ".join(missing))

    forbidden_hits = []
    for name in members:
        parts = set(name.split("/"))
        if parts & FORBIDDEN_PARTS or any(part in name for part in FORBIDDEN_PARTS):
            forbidden_hits.append(name)
    if forbidden_hits:
        fail("forbidden build/internal paths in archive: " + ", ".join(sorted(forbidden_hits)[:8]))

    with tarfile.open(archive, "r:gz") as tar:
        validate_packaged_versions(tar, members, version)

    print(f"Package contents PASSED ({archive.name})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
