#!/usr/bin/env python3
"""Validate the BME280 serial HIL runner/docs contract."""

from __future__ import annotations

import importlib.util
import pathlib
import py_compile
import sys
import types


ROOT = pathlib.Path(__file__).resolve().parents[1]
RUNNER = ROOT / "tools" / "run_i2c_hil.py"
RUNBOOK = ROOT / "docs" / "I2C_HIL_RUNBOOK.md"
TEMPLATE = ROOT / "docs" / "I2C_HIL_TARGET_TEMPLATE.md"
REPORT = ROOT / "docs" / "I2C_HIL_SELFTEST_REPORT.md"
MATRIX = ROOT / "docs" / "BME280_HARDWARE_VALIDATION_MATRIX.md"
GITIGNORE = ROOT / ".gitignore"


def fail(message: str) -> None:
    print(f"HIL contract FAILED: {message}", file=sys.stderr)
    raise SystemExit(1)


def read(path: pathlib.Path) -> str:
    if not path.exists():
        fail(f"missing required file: {path.relative_to(ROOT)}")
    return path.read_text(encoding="utf-8")


def load_runner():
    spec = importlib.util.spec_from_file_location("run_i2c_hil_contract", RUNNER)
    if spec is None or spec.loader is None:
        fail("could not load tools/run_i2c_hil.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def extract_sequence(text: str, path: pathlib.Path) -> list[str]:
    start = "<!-- HIL_DEFAULT_SEQUENCE_START -->"
    end = "<!-- HIL_DEFAULT_SEQUENCE_END -->"
    if start not in text or end not in text:
        fail(f"{path.relative_to(ROOT)} is missing HIL default sequence markers")
    block = text.split(start, 1)[1].split(end, 1)[0]
    lines: list[str] = []
    for raw in block.splitlines():
        line = raw.strip()
        if not line or line.startswith("```"):
            continue
        lines.append(line)
    return lines


def assert_contains(text: str, needle: str, path: pathlib.Path) -> None:
    if needle not in text:
        fail(f"{path.relative_to(ROOT)} is missing required text: {needle}")


def main() -> int:
    for path in (RUNNER, RUNBOOK, TEMPLATE, REPORT, MATRIX):
        if not path.exists():
            fail(f"missing required file: {path.relative_to(ROOT)}")

    py_compile.compile(str(RUNNER), doraise=True)
    runner_text = read(RUNNER)
    runbook_text = read(RUNBOOK)
    report_text = read(REPORT)
    matrix_text = read(MATRIX)
    gitignore_text = read(GITIGNORE)

    assert_contains(runner_text, "--dry-run", RUNNER)
    assert_contains(runner_text, "python -m pip install pyserial", RUNNER)
    assert_contains(runner_text, "hil_logs", RUNNER)
    assert_contains(gitignore_text, "hil_logs/", GITIGNORE)

    runner = load_runner()
    args = types.SimpleNamespace(
        address="0x76",
        commands=None,
        include_soak=False,
        include_destructive=False,
        include_fault_tests=False,
        timeout=8.0,
    )
    executable, manual = runner.build_command_sequence(args)
    runner_sequence = [spec.command for spec in executable]

    runbook_sequence = extract_sequence(runbook_text, RUNBOOK)
    report_sequence = extract_sequence(report_text, REPORT)
    if runbook_sequence != runner_sequence:
        fail("runbook command sequence differs from tools/run_i2c_hil.py")
    if report_sequence != runner_sequence:
        fail("selftest report command sequence differs from tools/run_i2c_hil.py")

    default_commands = set(runner_sequence)
    for unsafe in ("wreg", "stress 500"):
        if any(command.startswith(unsafe) for command in runner_sequence):
            fail(f"unsafe or soak command is in default sequence: {unsafe}")
    if not any(item.destructive for item in manual):
        fail("manual checklist does not include destructive raw-write opt-in item")
    if not any(item.command == "stress 500" for item in manual):
        fail("manual checklist does not include skipped soak item")
    if "chipid" not in default_commands:
        fail("default sequence must include chipid identity check")
    if "scan" not in default_commands:
        fail("default sequence must include scan reachability check")

    destructive_args = types.SimpleNamespace(
        address="0x76",
        commands=None,
        include_soak=False,
        include_destructive=True,
        include_fault_tests=False,
        timeout=8.0,
    )
    destructive_sequence = [spec.command for spec in runner.build_command_sequence(destructive_args)[0]]
    if "wreg 0xF4 0x00" not in destructive_sequence:
        fail("--include-destructive sequence does not include expected raw write")
    wreg_index = destructive_sequence.index("wreg 0xF4 0x00")
    if "recover" not in destructive_sequence[wreg_index + 1:]:
        fail("--include-destructive sequence must include recovery after raw write")
    if "cfg" not in destructive_sequence[wreg_index + 1:]:
        fail("--include-destructive sequence must include config evidence after raw write")

    for path, text in ((RUNBOOK, runbook_text), (REPORT, report_text)):
        assert_contains(text, "scan", path)
        assert_contains(text, "ACK", path)
        assert_contains(text, "0x60", path)
        assert_contains(text, "No physical HIL validation was performed", path)

    assert_contains(report_text, "Hardware run: NOT RUN", REPORT)
    assert_contains(read(ROOT / "docs" / "BME280_PRE_HIL_READINESS_REPORT.md"), "superseded by `docs/I2C_HIL_RUNBOOK.md`", ROOT / "docs" / "BME280_PRE_HIL_READINESS_REPORT.md")
    forbidden_report_claims = (
        "Hardware run: PASS",
        "Hardware validation: PASS",
        "Physical HIL validation: PASS",
    )
    for claim in forbidden_report_claims:
        if claim in report_text:
            fail(f"selftest report contains unsupported hardware claim: {claim}")

    assert_contains(matrix_text, "tools/run_i2c_hil.py", MATRIX)
    assert_contains(matrix_text, "stress 10 as an automated forced-measurement stress substitute", MATRIX)

    print("HIL contract PASSED")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
