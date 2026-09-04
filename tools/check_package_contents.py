#!/usr/bin/env python3
from __future__ import annotations

import pathlib
import json
import re
import sys
import tarfile

ROOT = pathlib.Path(__file__).resolve().parents[1]
IDF_MAIN_DIR = ROOT / "examples" / "idf" / "basic" / "main"

BASE_REQUIRED_PATHS = {
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

FORBIDDEN_PARTS = {
    ".git",
    ".pio",
    "__pycache__",
    "docs/doxygen",
    "hil_logs",
}

ROOT_LEVEL_NAMES = {
    ".github",
    "docs",
    "examples",
    "include",
    "scripts",
    "src",
    "test",
    "tests",
    "tools",
    "CMakeLists.txt",
    "CHANGELOG.md",
    "Doxyfile",
    "idf_component.yml",
    "library.json",
    "platformio.ini",
    "README.md",
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
    # removeprefix, not lstrip: lstrip("./") strips a character set and would
    # turn ".pio/build" into "pio/build", making FORBIDDEN_PARTS unmatchable.
    return name.replace("\\", "/").removeprefix("./")


def strip_package_root(name: str) -> str:
    normalized = normalize(name)
    parts = normalized.split("/")
    if len(parts) > 1 and parts[0] not in ROOT_LEVEL_NAMES:
        return "/".join(parts[1:])
    return normalized


def package_file_map(tar: tarfile.TarFile) -> tuple[dict[str, str], set[str]]:
    files: dict[str, str] = {}
    all_members: set[str] = set()
    for member in tar.getmembers():
        normalized = normalize(member.name)
        stripped = strip_package_root(normalized)
        all_members.add(normalized)
        all_members.add(stripped)
        if not member.isfile():
            continue
        if stripped in files:
            fail(f"duplicate packaged path after root normalization: {stripped}")
        files[stripped] = normalized
    return files, all_members


def idf_example_required_paths() -> set[str]:
    required = {
        "examples/idf/basic/CMakeLists.txt",
        "examples/idf/basic/main/CMakeLists.txt",
    }
    cmake = (IDF_MAIN_DIR / "CMakeLists.txt").read_text(encoding="utf-8")
    source_names = re.findall(r'"([^"]+\.(?:c|cc|cpp|cxx))"', cmake)
    pending = [IDF_MAIN_DIR / name for name in source_names]
    seen: set[pathlib.Path] = set()

    while pending:
        path = pending.pop()
        if path in seen:
            continue
        seen.add(path)
        if not path.exists():
            fail(f"IDF example references missing local file: {path.relative_to(ROOT).as_posix()}")
        required.add(path.relative_to(ROOT).as_posix())
        text = path.read_text(encoding="utf-8", errors="replace")
        for include in re.findall(r'^\s*#\s*include\s+"([^"]+)"', text, flags=re.MULTILINE):
            include_path = (path.parent / include).resolve()
            try:
                include_path.relative_to(IDF_MAIN_DIR.resolve())
            except ValueError:
                continue
            if include_path.exists():
                pending.append(include_path)

    return required


def required_paths() -> set[str]:
    return set(BASE_REQUIRED_PATHS) | idf_example_required_paths()


def read_member(tar: tarfile.TarFile, files: dict[str, str], path: str) -> str:
    if path not in files:
        fail(f"missing packaged {path}")
    handle = tar.extractfile(files[path])
    if handle is None:
        fail(f"could not read packaged {path}")
    return handle.read().decode("utf-8", errors="replace")


def validate_packaged_versions(tar: tarfile.TarFile, files: dict[str, str], version: str) -> None:
    packaged_library = json.loads(read_member(tar, files, "library.json"))
    packaged_component = read_member(tar, files, "idf_component.yml")
    packaged_version_h = read_member(tar, files, "include/BME280/Version.h")

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
        files, members = package_file_map(tar)

    missing = []
    for path in sorted(required_paths()):
        if path not in files:
            missing.append(path)
    if missing:
        fail("missing required files: " + ", ".join(missing))

    forbidden_hits = []
    for name in members:
        parts = set(name.split("/"))
        nested_archive = re.search(r"(?:^|/)BME280-[^/]+\.tar\.gz$", name)
        if (parts & FORBIDDEN_PARTS or
                any(part in name for part in FORBIDDEN_PARTS) or
                nested_archive):
            forbidden_hits.append(name)
    if forbidden_hits:
        fail("forbidden build/internal paths in archive: " + ", ".join(sorted(forbidden_hits)[:8]))

    with tarfile.open(archive, "r:gz") as tar:
        files, _ = package_file_map(tar)
        validate_packaged_versions(tar, files, version)

    print(f"Package contents PASSED ({archive.name})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
