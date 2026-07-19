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
MATRIX = ROOT / "docs" / "BME280_HARDWARE_VALIDATION_MATRIX.md"
SUMMARY = ROOT / "docs" / "BME280_INDUSTRY_HARDENING_SUMMARY.md"
README = ROOT / "README.md"
DOCS_INDEX = ROOT / "docs" / "README.md"
IDF_PORT = ROOT / "docs" / "IDF_PORT.md"
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


def runner_args(**overrides):
    data = {
        "address": "0x76",
        "commands": None,
        "include_soak": False,
        "soak_count": 500,
        "include_normal_soak": False,
        "normal_soak_count": 5,
        "normal_soak_interval_s": 1.0,
        "include_destructive": False,
        "include_fault_tests": False,
        "include_job_api": False,
        "timeout": 8.0,
    }
    data.update(overrides)
    return types.SimpleNamespace(**data)


def main() -> int:
    for path in (RUNNER, RUNBOOK, TEMPLATE, MATRIX, SUMMARY, README, DOCS_INDEX, IDF_PORT):
        if not path.exists():
            fail(f"missing required file: {path.relative_to(ROOT)}")

    py_compile.compile(str(RUNNER), doraise=True)
    runner_text = read(RUNNER)
    runbook_text = read(RUNBOOK)
    template_text = read(TEMPLATE)
    matrix_text = read(MATRIX)
    summary_text = read(SUMMARY)
    readme_text = read(README)
    docs_index_text = read(DOCS_INDEX)
    idf_port_text = read(IDF_PORT)
    gitignore_text = read(GITIGNORE)

    assert_contains(runner_text, "--dry-run", RUNNER)
    assert_contains(runner_text, "python -m pip install pyserial", RUNNER)
    assert_contains(runner_text, "hil_logs", RUNNER)
    assert_contains(runner_text, "astimezone()", RUNNER)
    assert_contains(gitignore_text, "hil_logs/", GITIGNORE)

    runner = load_runner()
    args = runner_args()
    executable, manual = runner.build_command_sequence(args)
    runner_sequence = [spec.command for spec in executable]

    runbook_sequence = extract_sequence(runbook_text, RUNBOOK)
    if runbook_sequence != runner_sequence:
        fail("runbook command sequence differs from tools/run_i2c_hil.py")

    default_commands = set(runner_sequence)
    cfg_specs = [spec for spec in executable if spec.command == "cfg"]
    if len(cfg_specs) != 2:
        fail("default sequence must include initial and post-recover cfg commands")
    for cfg_spec in cfg_specs:
        for token in ("ctrl_hum", "ctrl_meas", "config", "Hardware config dirty:"):
            if token not in cfg_spec.expected:
                fail(f"cfg evidence must require full output token: {token}")
        if "Hardware config dirty:" not in cfg_spec.completion:
            fail("cfg command must wait for the final dirty-state line")
        if cfg_spec.timeout_s < 10.0:
            fail("cfg command needs an extended bounded timeout window")
    calib_specs = [spec for spec in executable if spec.command == "calib"]
    if len(calib_specs) != 1:
        fail("default sequence must include exactly one cached calib command")
    calib_spec = calib_specs[0]
    for token in ("Calibration (Cached)", "T1=", "P1=", "H1=", "Plausibility:"):
        if token not in calib_spec.expected:
            fail(f"cached calib evidence must require full output token: {token}")
    if "Plausibility:" not in calib_spec.completion:
        fail("cached calib command must wait for the final plausibility line")
    if calib_spec.timeout_s < 10.0:
        fail("cached calib command needs an extended bounded timeout window")
    if "force" not in runner_sequence:
        fail("default sequence must include forced-mode command")
    force_index = runner_sequence.index("force")
    expected_post_force = ["reg 0xF4", "status", "read"]
    if runner_sequence[force_index + 1:force_index + 4] != expected_post_force:
        fail("default sequence must capture post-force reg 0xF4, status, and read evidence")
    for unsafe in ("wreg", "stress 500"):
        if any(command.startswith(unsafe) for command in runner_sequence):
            fail(f"unsafe or soak command is in default sequence: {unsafe}")
    if not any(item.destructive for item in manual):
        fail("manual checklist does not include destructive raw-write opt-in item")
    if not any(item.command == "stress 500" for item in manual):
        fail("manual checklist does not include skipped soak item")
    if not any(item.command == "normal on" and item.group == "soak-normal" for item in manual):
        fail("manual checklist does not include skipped normal-mode soak item")
    if "chipid" not in default_commands:
        fail("default sequence must include chipid identity check")
    if "scan" not in default_commands:
        fail("default sequence must include scan reachability check")
    if runner_sequence.count("read") < 4:
        fail("default sequence must include repeated read evidence")
    if runner_sequence[runner_sequence.index("reset") + 1] != "status":
        fail("default sequence must capture post-reset status/im_update evidence")
    if runner_sequence[runner_sequence.index("recover") + 1] != "cfg":
        fail("default sequence must capture config evidence after recover")
    assert_contains(runner_text, "CTRL_MEAS_RE", RUNNER)
    assert_contains(runner_text, "ctrl_meas mode bits", RUNNER)
    assert_contains(runner_text, "group", RUNNER)
    assert_contains(runner_text, "expected_any", RUNNER)
    assert_contains(runner_text, "validators", RUNNER)
    assert_contains(runner_text, "command_plan.json", RUNNER)
    assert_contains(runner_text, "results.csv", RUNNER)
    assert_contains(runner_text, "environment.txt", RUNNER)
    assert_contains(runner_text, "hardware_matrix_fragment.md", RUNNER)
    assert_contains(runner_text, "failure_analysis.md", RUNNER)
    assert_contains(runner_text, "manifest.json", RUNNER)
    assert_contains(runner_text, "BME280_RAW_WRITE", RUNNER)
    assert_contains(runner_text, "runner_command", RUNNER)
    assert_contains(runner_text, "runner_args", RUNNER)
    assert_contains(runner_text, "summary_md", RUNNER)
    assert_contains(runner_text, "--include-job-api", RUNNER)
    assert_contains(runner_text, "--require-pass", RUNNER)
    assert_contains(runner_text, "--fail-on-review", RUNNER)
    assert_contains(runner_text, "VALIDATOR_JOB_DONE_OR_FAILED", RUNNER)
    assert_contains(runner_text, "VALIDATOR_JOB_INSTRUCTION_BUDGET_RESPECTED", RUNNER)
    assert_contains(runner_text, "VALIDATOR_JOB_CANCEL_TIMED_OUT", RUNNER)
    assert_contains(runner_text, "VALIDATOR_JOB_TIMED_OUT_RETRIEVAL", RUNNER)
    assert_contains(runner_text, "VALIDATOR_JOB_IDLE_NO_RESULT", RUNNER)

    destructive_args = runner_args(include_destructive=True)
    destructive_sequence = [spec.command for spec in runner.build_command_sequence(destructive_args)[0]]
    if "wreg 0xF4 0x00" not in destructive_sequence:
        fail("--include-destructive sequence does not include expected raw write")
    wreg_index = destructive_sequence.index("wreg 0xF4 0x00")
    if "recover" not in destructive_sequence[wreg_index + 1:]:
        fail("--include-destructive sequence must include recovery after raw write")
    if "cfg" not in destructive_sequence[wreg_index + 1:]:
        fail("--include-destructive sequence must include config evidence after raw write")
    if "status" not in destructive_sequence[wreg_index + 1:]:
        fail("--include-destructive sequence must include dirty-state evidence after raw write")

    soak_args = runner_args(include_normal_soak=True, normal_soak_count=3)
    soak_executable = runner.build_command_sequence(soak_args)[0]
    soak_sequence = [spec.command for spec in soak_executable]
    if soak_sequence[-7:] != ["normal on", "read", "read", "read", "normal off", "cfg", "status"]:
        fail("--include-normal-soak sequence must include normal on, repeated reads, normal off, cfg, and status")
    soak_cfg_specs = [spec for spec in soak_executable if spec.command == "cfg" and spec.group == "soak-normal"]
    if len(soak_cfg_specs) != 1:
        fail("--include-normal-soak sequence must include one soak-normal cfg command")
    if "Hardware config dirty:" not in soak_cfg_specs[0].expected:
        fail("--include-normal-soak cfg evidence must require dirty-state output")
    if "Hardware config dirty:" not in soak_cfg_specs[0].completion:
        fail("--include-normal-soak cfg command must wait for dirty-state output")

    job_args = runner_args(include_job_api=True)
    expected_job_sequence = [
        "job status",
        "job start init",
        "job poll 1",
        "job cancel deadline",
        "job poll 0",
        "job poll 0",
        "job init 1",
        "job apply 1",
        "job force 1",
        "raw",
        "comp",
        "job resync 1",
        "cfg",
        "status",
        "job reset 1",
        "job force 3",
        "drv",
    ]
    job_specs = [spec for spec in runner.build_command_sequence(job_args)[0] if spec.group == "job-api"]
    actual_job_sequence = [spec.command for spec in job_specs]
    if actual_job_sequence != expected_job_sequence:
        fail("--include-job-api sequence does not include the expected job-api command order")
    if not any(runner.VALIDATOR_JOB_DONE_OR_FAILED in spec.validators for spec in job_specs):
        fail("job-api commands must validate terminal job state")
    if not any(runner.VALIDATOR_JOB_INSTRUCTION_BUDGET_RESPECTED in spec.validators for spec in job_specs):
        fail("job-api commands must validate instruction budgets")
    if not any(runner.VALIDATOR_JOB_CANCEL_TIMED_OUT in spec.validators for spec in job_specs):
        fail("job-api commands must validate deadline cancellation")
    if not any(runner.VALIDATOR_JOB_TIMED_OUT_RETRIEVAL in spec.validators for spec in job_specs):
        fail("job-api commands must validate zero-budget terminal retrieval")
    if not any(runner.VALIDATOR_JOB_IDLE_NO_RESULT in spec.validators for spec in job_specs):
        fail("job-api commands must validate exactly-once terminal retrieval")
    if actual_job_sequence.index("job resync 1") >= actual_job_sequence.index("job reset 1"):
        fail("job-api sequence must exercise non-reset resync separately before explicit reset")

    fault_args = runner_args(include_fault_tests=True)
    _, fault_manual = runner.build_command_sequence(fault_args)
    if not any("Requested via --include-fault-tests" in item.notes for item in fault_manual):
        fail("--include-fault-tests must be visible in manual checklist notes")

    for path, text in ((RUNBOOK, runbook_text), (SUMMARY, summary_text), (README, readme_text), (IDF_PORT, idf_port_text)):
        assert_contains(text, "scan", path)
    for path, text in ((RUNBOOK, runbook_text), (SUMMARY, summary_text), (MATRIX, matrix_text)):
        assert_contains(text, "ACK", path)
        assert_contains(text, "0x60", path)
    for path, text in ((RUNBOOK, runbook_text), (SUMMARY, summary_text), (README, readme_text)):
        assert_contains(text, "No physical", path)

    for result_token in (
        "PASS",
        "OPERATOR_CHECK_REQUIRED",
        "REVIEW_REQUIRED",
        "SERIAL_OK_OR_REVIEW",
        "UNKNOWN",
        "PASS_WITH_RESET_BUSY_RECOVERED",
        "FAIL",
        "TIMEOUT",
        "SKIPPED_DRY_RUN",
        "SKIPPED_UNSAFE",
    ):
        assert_contains(runbook_text, result_token, RUNBOOK)

    for field in (
        "Operator",
        "Date/time and timezone",
        "Git commit",
        "Worktree state / dirty flag",
        "Runner command",
        "Runner arguments",
        "Firmware `version` output",
        "Library version",
        "MCU board model",
        "MCU target",
        "BME280 module or sensor board model",
        "VDD",
        "VDDIO",
        "SDA pin",
        "SCL pin",
        "I2C speed",
        "I2C pull-ups",
        "Pull-up location",
        "SDO state",
        "CSB state",
        "serial_transcript.txt",
        "command_plan.json",
        "results.csv",
        "environment.txt",
        "hardware_matrix_fragment.md",
        "failure_analysis.md",
        "manifest.json",
        "Exact command transcript path",
        "Runner final verdict",
        "Operator notes",
        "Operator sign-off",
        "Temperature reference reading",
        "BME280 temperature reading",
        "Temperature tolerance / uncertainty",
        "Humidity reference reading",
        "BME280 humidity reading",
        "Pressure reference reading",
        "BME280 pressure reading",
        "Reading timestamp",
    ):
        assert_contains(template_text, field, TEMPLATE)

    for read_cmd in ("normal on", "normal off", "cfg"):
        assert_contains(matrix_text, read_cmd, MATRIX)
    for field in (
        "Firmware `version` output",
        "Library version",
        "Git commit hash",
        "Worktree state / dirty flag",
        "HIL runner command",
        "HIL runner arguments",
        "MCU board model",
        "MCU target",
        "BME280 module or board model",
        "VDD / VDDIO",
        "Pull-up values and location",
        "SDO state",
        "CSB state",
        "SDA/SCL pins and bus speed",
        "Serial port and baud",
        "Command groups",
        "Artifact manifest path",
        "Environmental reference instruments",
        "Exact command transcript path",
        "Runner final verdict",
        "Operator notes / sign-off",
    ):
        assert_contains(matrix_text, field, MATRIX)
    for field in (
        "Library version as printed by `version`",
        "HIL runner command and exact arguments",
        "MCU target",
        "Reference readings, BME280 readings, tolerance or uncertainty",
        "Exact serial command transcript path",
        "Runner final verdict",
        "Manifest path",
    ):
        assert_contains(runbook_text, field, RUNBOOK)
    assert_contains(runbook_text, "reg 0xF4", RUNBOOK)
    assert_contains(runbook_text, "post-`force`", RUNBOOK)
    assert_contains(runbook_text, "--confirm-raw-write BME280_RAW_WRITE", RUNBOOK)
    assert_contains(runbook_text, "--include-normal-soak", RUNBOOK)
    assert_contains(runbook_text, "command file", RUNBOOK)
    assert_contains(runbook_text, "stress Errors=0", RUNBOOK)
    assert_contains(matrix_text, "reg 0xF4", MATRIX)
    assert_contains(matrix_text, "post-force", MATRIX)
    assert_contains(matrix_text, "manifest.json", MATRIX)
    assert_contains(matrix_text, "command_plan.json", MATRIX)
    assert_contains(matrix_text, "normal-mode soak", MATRIX)

    for text, path in ((docs_index_text, DOCS_INDEX), (readme_text, README), (summary_text, SUMMARY)):
        assert_contains(text, "BME280_HARDWARE_VALIDATION_MATRIX.md", path)
        assert_contains(text, "I2C_HIL_RUNBOOK.md", path)

    forbidden_report_claims = (
        "Hardware run: PASS",
        "Hardware validation: PASS",
        "Physical HIL validation: PASS",
    )
    scanned_docs = (runbook_text, summary_text, matrix_text, readme_text, idf_port_text)
    for claim in forbidden_report_claims:
        if any(claim in text for text in scanned_docs):
            fail(f"docs contain unsupported hardware claim: {claim}")

    stale_terms = (
        "phase reports",
        "BME280_PHASE_",
        "BME280_PRE_HIL_READINESS_REPORT",
        "BME280_PROMPTS_",
        "I2C_HIL_SELFTEST_REPORT",
        "IDF_PORT_IMPLEMENTATION.md",
        "CODEX_PROMPT_BME280_DRIVER.md",
    )
    for term in stale_terms:
        if any(term in text for text in scanned_docs):
            fail(f"docs contain stale industry-readiness term: {term}")

    assert_contains(matrix_text, "tools/run_i2c_hil.py", MATRIX)
    assert_contains(matrix_text, "stress 10 as an automated forced-measurement stress substitute", MATRIX)
    assert_contains(summary_text, "merged industry-readiness work", SUMMARY)
    assert_contains(summary_text, "The CLI does not support counted read commands", SUMMARY)

    print("HIL contract PASSED")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
