#!/usr/bin/env python3
"""Enforce that the core driver stays framework-neutral.

`src/` and `include/` must not call platform timing APIs and must not include
Arduino, ESP-IDF, or FreeRTOS headers. Timing comes from `Config::nowMs` and
explicit `tick(nowMs)` / `pollJob(nowMs)` arguments; transport comes from the
injected callbacks.
"""
from __future__ import annotations

import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
SCAN_DIRS = ("src", "include")
VALID_SUFFIXES = {".c", ".cc", ".cpp", ".h", ".hpp"}

FORBIDDEN_CALLS = {
    "millis": re.compile(r"\bmillis\s*\("),
    "micros": re.compile(r"\bmicros\s*\("),
    "delay": re.compile(r"\bdelay\s*\("),
    "delayMicroseconds": re.compile(r"\bdelayMicroseconds\s*\("),
    "yield": re.compile(r"\byield\s*\("),
    "vTaskDelay": re.compile(r"\bvTaskDelay\s*\("),
    "esp_timer_get_time": re.compile(r"\besp_timer_get_time\s*\("),
}

FORBIDDEN_INCLUDE_RE = re.compile(
    r'^\s*#\s*include\s*[<"](?:Arduino\.h|Wire\.h|driver/[^>"]+|esp_[^>"]+|freertos/[^>"]+)[>"]',
    re.MULTILINE,
)
BLOCK_COMMENT_RE = re.compile(r"/\*.*?\*/", re.DOTALL)
LINE_COMMENT_RE = re.compile(r"//[^\n]*")
STRING_RE = re.compile(r'"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'')


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
        for path in sorted(root.rglob("*")):
            if path.is_file() and path.suffix.lower() in VALID_SUFFIXES:
                files.append(path)
    return files


def main() -> int:
    errors: list[str] = []

    for path in collect_sources():
        rel = path.relative_to(ROOT).as_posix()
        raw = path.read_text(encoding="utf-8", errors="replace")
        code = strip_non_code(raw)

        for call_name, pattern in FORBIDDEN_CALLS.items():
            hits = len(pattern.findall(code))
            if hits:
                errors.append(f"platform timing call in {rel}: {call_name} x{hits}")

        for match in FORBIDDEN_INCLUDE_RE.findall(raw):
            errors.append(f"framework include in {rel}: {match.strip()}")

    if errors:
        print("Core timing guard FAILED:")
        for err in errors:
            print(f"- {err}")
        return 1

    print("Core timing guard PASSED")
    return 0


if __name__ == "__main__":
    sys.exit(main())
