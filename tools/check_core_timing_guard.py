#!/usr/bin/env python3
from __future__ import annotations

import pathlib
import re
import sys
from typing import Dict

ROOT = pathlib.Path(__file__).resolve().parents[1]
SCAN_DIRS = ("src", "include")
VALID_SUFFIXES = {".c", ".cc", ".cpp", ".h", ".hpp"}

FORBIDDEN_CALLS = {
    "millis": re.compile(r"\bmillis\s*\("),
    "micros": re.compile(r"\bmicros\s*\("),
    "delayMicroseconds": re.compile(r"\bdelayMicroseconds\s*\("),
    "yield": re.compile(r"\byield\s*\("),
}

INCLUDE_ARDUINO_RE = re.compile(r'^\s*#\s*include\s*[<\"]Arduino\.h[>\"]', re.MULTILINE)
BLOCK_COMMENT_RE = re.compile(r"/\*.*?\*/", re.DOTALL)
LINE_COMMENT_RE = re.compile(r"//[^\n]*")
STRING_RE = re.compile(r'"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'')

ALLOWED_CALL_COUNTS: Dict[str, Dict[str, int]] = {}
ALLOWED_INCLUDE_COUNTS: Dict[str, int] = {}


def strip_non_code(text: str) -> str:
    text = BLOCK_COMMENT_RE.sub("", text)
    text = LINE_COMMENT_RE.sub("", text)
    return STRING_RE.sub('""', text)


def collect_sources() -> list[pathlib.Path]:
    files: list[pathlib.Path] = []
    for dirname in SCAN_DIRS:
        root = ROOT / dirname
        if not root.exists():
            continue
        for path in root.rglob("*"):
            if path.is_file() and path.suffix.lower() in VALID_SUFFIXES:
                files.append(path)
    return files


def main() -> int:
    observed_calls: Dict[str, Dict[str, int]] = {}
    observed_includes: Dict[str, int] = {}

    for path in collect_sources():
        rel = path.relative_to(ROOT).as_posix()
        raw = path.read_text(encoding="utf-8", errors="replace")
        code = strip_non_code(raw)

        call_counts: Dict[str, int] = {}
        for call_name, pattern in FORBIDDEN_CALLS.items():
            count = len(pattern.findall(code))
            if count > 0:
                call_counts[call_name] = count
        if call_counts:
            observed_calls[rel] = call_counts

        include_count = len(INCLUDE_ARDUINO_RE.findall(raw))
        if include_count > 0:
            observed_includes[rel] = include_count

    errors: list[str] = []

    for rel, counts in observed_calls.items():
        if rel not in ALLOWED_CALL_COUNTS:
            errors.append(f"forbidden timing calls in unexpected file: {rel} -> {counts}")
            continue
        expected = ALLOWED_CALL_COUNTS[rel]
        for call_name, count in counts.items():
            exp = expected.get(call_name, 0)
            if count != exp:
                errors.append(
                    f"timing call count mismatch in {rel}: {call_name} observed={count}, expected={exp}"
                )

    for rel, expected in ALLOWED_CALL_COUNTS.items():
        observed = observed_calls.get(rel, {})
        for call_name, exp in expected.items():
            obs = observed.get(call_name, 0)
            if obs != exp:
                errors.append(
                    f"timing call count mismatch in {rel}: {call_name} observed={obs}, expected={exp}"
                )
        unexpected_calls = set(observed.keys()) - set(expected.keys())
        if unexpected_calls:
            errors.append(f"unexpected timing call types in {rel}: {sorted(unexpected_calls)}")

    for rel, count in observed_includes.items():
        exp = ALLOWED_INCLUDE_COUNTS.get(rel, 0)
        if count != exp:
            errors.append(
                f"Arduino include count mismatch in {rel}: observed={count}, expected={exp}"
            )

    for rel, exp in ALLOWED_INCLUDE_COUNTS.items():
        obs = observed_includes.get(rel, 0)
        if obs != exp:
            errors.append(
                f"Arduino include count mismatch in {rel}: observed={obs}, expected={exp}"
            )

    if errors:
        print("Core timing guard FAILED:")
        for err in errors:
            print(f"- {err}")
        return 1

    print("Core timing guard PASSED")
    return 0


if __name__ == "__main__":
    sys.exit(main())

