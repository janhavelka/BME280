#!/usr/bin/env python3
"""Validate the serial HIL runner against its single maintained document."""

from __future__ import annotations

import hashlib
import importlib.util
import pathlib
import py_compile
import sys
import types


ROOT = pathlib.Path(__file__).resolve().parents[1]
RUNNER = ROOT / "tools" / "run_i2c_hil.py"
VALIDATION = ROOT / "docs" / "HARDWARE_VALIDATION.md"
README = ROOT / "README.md"
DOCS_INDEX = ROOT / "docs" / "README.md"
IDF_PORT = ROOT / "docs" / "IDF_PORT.md"
SHARED_BUS = ROOT / "docs" / "PRODUCTION_SHARED_BUS_GUIDE.md"
GITATTRIBUTES = ROOT / ".gitattributes"
GITIGNORE = ROOT / ".gitignore"
RETAINED_TRANSCRIPT = (
    ROOT / "hil_logs" / "i2c_20260803_144215" / "serial_transcript.txt"
)
RETAINED_TRANSCRIPT_SIZE = 5_048_514
RETAINED_TRANSCRIPT_SHA256 = (
    "7b858255bfc5053e18c224f3d14621e18b3db21ed1cfef16d92f727f4ba55415"
)

def fail(message: str) -> None:
    print(f"HIL contract FAILED: {message}", file=sys.stderr)
    raise SystemExit(1)


def read(path: pathlib.Path) -> str:
    if not path.exists():
        fail(f"missing required file: {path.relative_to(ROOT)}")
    return path.read_text(encoding="utf-8")


def assert_contains(text: str, token: str, path: pathlib.Path) -> None:
    if token not in text:
        fail(f"{path.relative_to(ROOT)} is missing required text: {token}")


def assert_all(text: str, tokens: tuple[str, ...], path: pathlib.Path) -> None:
    for token in tokens:
        assert_contains(text, token, path)


def load_runner():
    spec = importlib.util.spec_from_file_location("run_i2c_hil_contract", RUNNER)
    if spec is None or spec.loader is None:
        fail("could not load tools/run_i2c_hil.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def extract_sequence(text: str) -> list[str]:
    start = "<!-- HIL_DEFAULT_SEQUENCE_START -->"
    end = "<!-- HIL_DEFAULT_SEQUENCE_END -->"
    if start not in text or end not in text:
        fail("docs/HARDWARE_VALIDATION.md is missing sequence markers")
    block = text.split(start, 1)[1].split(end, 1)[0]
    return [
        line.strip()
        for line in block.splitlines()
        if line.strip() and not line.strip().startswith("```")
    ]


def runner_args(**overrides):
    values = {
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
        "include_config_matrix": False,
        "include_invalid_inputs": False,
        "include_benchmarks": False,
        "sample_rate_benchmark": False,
        "soak_duration_s": 0.0,
        "soak_cycle_stress_count": 50,
        "soak_cycle_mix_count": 70,
        "soak_cycle_idle_s": 0.0,
        "soak_reset_interval": 20,
        "timeout": 8.0,
    }
    values.update(overrides)
    return types.SimpleNamespace(**values)


def validate_default_plan(runner, validation_text: str) -> None:
    executable, manual = runner.build_command_sequence(runner_args())
    commands = [spec.command for spec in executable]
    if extract_sequence(validation_text) != commands:
        fail("documented default sequence differs from tools/run_i2c_hil.py")

    if commands.count("cfg") != 2 or commands.count("read") < 4:
        fail("default plan lost required config or repeated-read evidence")
    if commands[commands.index("force") + 1:commands.index("force") + 4] != [
        "reg 0xF4", "status", "read"
    ]:
        fail("default plan lost post-force sleep/status evidence")
    if commands[commands.index("reset") + 1] != "status":
        fail("default plan lost post-reset status evidence")
    if commands[commands.index("recover") + 1:commands.index("recover") + 3] != [
        "cfg", "status"
    ]:
        fail("default plan lost post-recover config/status evidence")
    if any(spec.command.startswith("wreg") for spec in executable):
        fail("default plan contains a destructive raw write")
    if not any(item.destructive for item in manual):
        fail("manual checklist lost destructive-work visibility")

    cfg_specs = [spec for spec in executable if spec.command == "cfg"]
    for spec in cfg_specs:
        for token in ("ctrl_hum", "ctrl_meas", "config", "Hardware config dirty:"):
            if token not in spec.expected:
                fail(f"cfg evidence is missing parser token: {token}")
    calib = [spec for spec in executable if spec.command == "calib"]
    if len(calib) != 1 or "Plausibility:" not in calib[0].completion:
        fail("cached calibration evidence no longer reaches plausibility output")


def validate_optional_plans(runner) -> None:
    destructive = [
        spec.command
        for spec in runner.build_command_sequence(
            runner_args(include_destructive=True)
        )[0]
    ]
    raw_index = destructive.index("wreg 0xF4 0x00")
    if not all(command in destructive[raw_index + 1:] for command in ("recover", "cfg", "status")):
        fail("destructive plan does not restore and record configuration")

    normal = [
        spec.command
        for spec in runner.build_command_sequence(
            runner_args(include_normal_soak=True, normal_soak_count=3)
        )[0]
    ]
    if normal[-7:] != ["normal on", "read", "read", "read", "normal off", "cfg", "status"]:
        fail("normal soak plan has an unexpected command shape")

    expected_jobs = [
        "job status",
        "xfer_reset",
        "settings start forced x1 x1 x1 off ms_125",
        "xfer_assert 0 0 0",
        "job cancel owner",
        "job poll 0",
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
    job_specs = [
        spec
        for spec in runner.build_command_sequence(
            runner_args(include_job_api=True)
        )[0]
        if spec.group == "job-api"
    ]
    if [spec.command for spec in job_specs] != expected_jobs:
        fail("staged-job HIL plan has an unexpected command shape")
    required_validators = {
        runner.VALIDATOR_JOB_DONE_OR_FAILED,
        runner.VALIDATOR_JOB_INSTRUCTION_BUDGET_RESPECTED,
        runner.VALIDATOR_JOB_CANCEL_TIMED_OUT,
        runner.VALIDATOR_JOB_TIMED_OUT_RETRIEVAL,
        runner.VALIDATOR_JOB_IDLE_NO_RESULT,
    }
    actual_validators = {
        validator for spec in job_specs for validator in spec.validators
    }
    if not required_validators.issubset(actual_validators):
        fail("staged-job plan lost terminal/budget/cancellation validators")

    _, fault_manual = runner.build_command_sequence(
        runner_args(include_fault_tests=True)
    )
    if not any("Requested via --include-fault-tests" in item.notes for item in fault_manual):
        fail("fault-test opt-in is not visible in the manual checklist")

    config_specs = [
        spec
        for spec in runner.build_command_sequence(
            runner_args(include_config_matrix=True)
        )[0]
        if spec.group == "config-matrix"
    ]
    if [spec.command for spec in config_specs[-2:]] != ["mode forced", "cfg"]:
        fail("configuration matrix does not restore and capture forced policy")
    config_commands = [spec.command for spec in config_specs]
    required_enum_commands = {
        *(f"osrs t {value}" for value in range(1, 6)),
        *(f"osrs p {value}" for value in range(0, 6)),
        *(f"osrs h {value}" for value in range(0, 6)),
        *(f"filter {value}" for value in range(0, 5)),
        *(f"standby {value}" for value in range(0, 8)),
    }
    if not required_enum_commands.issubset(config_commands):
        fail("configuration matrix lost legal enum-value coverage")
    required_setter_profiles = {
        "osrs t x2",
        "osrs p x2",
        "osrs h x2",
        "filter x2",
        "standby 250ms",
        "settings set sleep x1 x1 x1 off ms_125",
        "xfer_assert 2 1 3",
        "xfer_assert 2 2 4",
        "xfer_assert 2 3 5",
        "xfer_assert 2 4 6",
    }
    if not required_setter_profiles.issubset(config_commands):
        fail("configuration matrix lost typed-setter callback profiles")
    if not any(spec.expected_settings for spec in config_specs if spec.command == "cfg"):
        fail("configuration matrix lost exact register readback validators")

    invalid_specs = [
        spec
        for spec in runner.build_command_sequence(
            runner_args(include_invalid_inputs=True)
        )[0]
        if spec.group == "invalid-input"
    ]
    if [spec.command for spec in invalid_specs] != [
        spec.command for spec in runner.INVALID_INPUT_COMMANDS
    ]:
        fail("invalid-input opt-in lost command coverage")

    benchmark_specs = [
        spec
        for spec in runner.build_command_sequence(
            runner_args(include_benchmarks=True)
        )[0]
        if spec.group == "benchmark"
    ]
    if [spec.command for spec in benchmark_specs] != [
        spec.command for spec in runner.BENCHMARK_COMMANDS
    ]:
        fail("benchmark opt-in lost command coverage")

    duration_args = runner_args(soak_duration_s=60.0)
    duration_specs = runner.duration_soak_cycle_commands(duration_args, 1)
    if not {"stress 50", "stress_mix 70", "status", "drv"}.issubset(
        {spec.command for spec in duration_specs}
    ):
        fail("duration-soak cycle lost required safe commands")
    if any(spec.command.startswith("wreg") for spec in duration_specs):
        fail("duration-soak cycle contains a destructive raw write")
    grouped = [
        [spec.command for spec in group]
        for group in runner.duration_soak_command_groups(
            runner_args(soak_reset_interval=20), 140
        )
    ]
    if ["normal on", "read", "normal off"] not in grouped:
        fail("duration soak does not reserve normal-mode cleanup as one group")
    if ["reset", "recover"] not in grouped:
        fail("duration soak does not reserve reset recovery as one group")
    default_args = runner.parse_args(["--dry-run"])
    if default_args.reconnect_attempts != 0:
        fail("reconnect attempts must default to zero")
    reconnect_args = runner.parse_args([
        "--dry-run", "--reconnect-attempts", "3",
    ])
    if reconnect_args.reconnect_attempts != 3:
        fail("explicit reconnect attempt budget was not preserved")
    final_cleanup = runner.final_cleanup_commands()
    if [spec.command for spec in final_cleanup] != [
        "normal off", "recover", "cfg", "status", "drv"
    ]:
        fail("final cleanup has an unexpected command shape")
    if any(spec.group != "final-cleanup" for spec in final_cleanup):
        fail("final cleanup rows lost their dedicated result group")
    if any(spec.timeout_s <= 0.0 or spec.timeout_s > 10.0 for spec in final_cleanup):
        fail("final cleanup has an unbounded per-command timeout")
    cleanup_by_command = {spec.command: spec for spec in final_cleanup}
    if "Hardware config dirty: false" not in cleanup_by_command["cfg"].expected:
        fail("final cleanup cfg does not prove clean hardware configuration")
    if "dirty=false" not in cleanup_by_command["status"].expected:
        fail("final cleanup status does not require clean driver state")
    if cleanup_by_command["status"].validators != (
        runner.VALIDATOR_STATUS_NOT_MEASURING,
        runner.VALIDATOR_STATUS_IM_UPDATE_CLEAR,
    ):
        fail("final cleanup status lost measuring/NVM-clear validators")
    if cleanup_by_command["drv"].validators != (
        runner.VALIDATOR_DRIVER_ZERO_CONSECUTIVE,
    ):
        fail("final cleanup driver row lost zero-consecutive-failures validator")


def validate_documentation(validation_text: str) -> None:
    assert_all(
        validation_text,
        (
            "FUNCTIONAL SERIAL PASS / OPERATOR REVIEW REQUIRED",
            "scan` proves only",
            "`0x60`",
            "--include-job-api",
            "--dry-run --include-job-api",
            "--include-config-matrix --include-invalid-inputs --include-benchmarks",
            "--out hil_logs",
            "--include-normal-soak",
            "--include-destructive --confirm-raw-write BME280_RAW_WRITE",
            "--require-pass",
            "--fail-on-review",
            "Operator; date/time and timezone",
            "Git commit",
            "worktree state / dirty flag",
            "firmware `version` output",
            "firmware-reported library version, commit, and clean",
            "cannot qualify exact build provenance",
            "MCU board model and target",
            "VDD; VDDIO",
            "pull-up values and location",
            "verified SDO and CSB state",
            "Reference instrument models",
            "serial_transcript.txt",
            "command_plan.json",
            "results.csv",
            "environment.txt",
            "operator_checklist.md",
            "hardware_matrix_fragment.md",
            "failure_analysis.md",
            "manifest.json",
            "planning budget",
            "recorded exact elapsed time",
            "--reconnect-attempts N",
            "identity probe",
            "safe command group is replayed",
            "completed soak rows preserved",
            "always prepends its canonical",
        ),
        VALIDATION,
    )
    assert_all(
        validation_text,
        (
            "PASS_WITH_RESET_BUSY_RECOVERED",
            "OPERATOR_CHECK_REQUIRED",
            "REVIEW_REQUIRED",
            "SERIAL_OK_OR_REVIEW",
            "UNKNOWN",
            "FAIL` / `TIMEOUT",
            "SKIPPED_DRY_RUN` / `SKIPPED_UNSAFE",
        ),
        VALIDATION,
    )


def main() -> int:
    for path in (
        RUNNER,
        VALIDATION,
        README,
        DOCS_INDEX,
        IDF_PORT,
        SHARED_BUS,
        GITATTRIBUTES,
        RETAINED_TRANSCRIPT,
    ):
        if not path.exists():
            fail(f"missing required file: {path.relative_to(ROOT)}")
    py_compile.compile(str(RUNNER), doraise=True)
    runner = load_runner()

    validation_text = read(VALIDATION)
    validate_default_plan(runner, validation_text)
    validate_optional_plans(runner)
    validate_documentation(validation_text)

    runner_text = read(RUNNER)
    assert_all(
        runner_text,
        (
            "--dry-run",
            "python -m pip install pyserial",
            "hil_logs",
            "astimezone()",
            "manifest.json",
            "summary.json",
            "results.csv",
            "command_plan.json",
            "reclassify_version_provenance(",
            "dirty host or firmware source cannot qualify exact build provenance",
            "BUSY_REASON_NVM_UPDATE",
            "run_automatic_recovery(",
            "run_final_cleanup(",
            "run_live_plan(",
            "duration_soak_command_groups(",
            "reconnect_serial_in_place(",
            "serial_reconnect_event_result(",
            "REPLAYING SAFE GROUP FROM FIRST ROW",
            "expected_settings",
            "output_is_complete(",
            '"MATCHED_COMPLETION"',
            'evidence["driver_state"]',
            'evidence["hardware_config_dirty"]',
        ),
        RUNNER,
    )
    runner_text = read(RUNNER)
    for removed_helper in ("duration_command_fits(", "row_output(", "output_full"):
        if removed_helper in runner_text:
            fail(f"runner retained obsolete evidence/helper token: {removed_helper}")

    gitignore_text = read(GITIGNORE)
    assert_contains(gitignore_text, "hil_logs/*", GITIGNORE)
    for run_id in (
        "i2c_20260803_144215",
        "i2c_20260804_155442",
        "i2c_20260804_171428",
    ):
        assert_contains(gitignore_text, f"!hil_logs/{run_id}/", GITIGNORE)
        assert_contains(
            gitignore_text,
            f"!hil_logs/{run_id}/serial_transcript.txt",
            GITIGNORE,
        )
    for broad_exception in (
        "!hil_logs/*/",
        "!hil_logs/*/serial_transcript.txt",
    ):
        if broad_exception in gitignore_text:
            fail(f".gitignore retained broad HIL exception: {broad_exception}")
    assert_contains(
        read(GITATTRIBUTES), "hil_logs/*/serial_transcript.txt -text", GITATTRIBUTES
    )
    assert_all(
        read(RETAINED_TRANSCRIPT),
        (
            "commit=dc5df8e6735bd21f52dea3e5ae7dcf54ec2b498d",
            "--- DURATION SOAK END",
            "--- FINAL CLEANUP END",
        ),
        RETAINED_TRANSCRIPT,
    )
    transcript_bytes = RETAINED_TRANSCRIPT.read_bytes()
    if len(transcript_bytes) != RETAINED_TRANSCRIPT_SIZE:
        fail("retained HIL transcript size changed")
    if hashlib.sha256(transcript_bytes).hexdigest() != RETAINED_TRANSCRIPT_SHA256:
        fail("retained HIL transcript SHA-256 changed")
    assert_contains(read(README), "--out .pio/hil_dry_runs", README)

    for path in (README, DOCS_INDEX, IDF_PORT, SHARED_BUS):
        assert_contains(read(path), "HARDWARE_VALIDATION.md", path)
    assert_all(
        read(SHARED_BUS),
        (
            "Owner-Managed Integration Checklist",
            "`pollJob(nowMs, 1)`",
            "original end-to-end deadline",
            "checked fixed-point conversion helpers",
            "definite address NACK",
            "`invalidateDeviceState()`",
            "sampleFreshness()",
            "candidate selection, retries, aggregate health",
        ),
        SHARED_BUS,
    )
    claim_text = "\n".join(
        read(path) for path in (README, DOCS_INDEX, IDF_PORT, SHARED_BUS, VALIDATION)
    )
    for unsupported in ("Hardware run: PASS", "Physical HIL: PASS", "HIL validated: PASS"):
        if unsupported in claim_text:
            fail(f"maintained docs contain unsupported hardware claim: {unsupported}")

    print("HIL contract PASSED")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
