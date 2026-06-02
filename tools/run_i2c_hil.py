#!/usr/bin/env python3
"""Serial HIL runner for the BME280 diagnostic CLI.

The runner drives the existing Arduino/ESP-IDF serial CLI. It does not flash
firmware and it does not prove hardware validity by itself; it captures a
disciplined transcript and classifies only what the serial output supports.
"""

from __future__ import annotations

import argparse
import csv
import dataclasses
import datetime as dt
import hashlib
import importlib.metadata
import json
import os
import pathlib
import platform
import re
import shlex
import subprocess
import sys
import time
from typing import Iterable


ROOT = pathlib.Path(__file__).resolve().parents[1]
DEFAULT_OUT = ROOT / "hil_logs"
INSTALL_HINT = "python -m pip install pyserial"

RESULT_PASS = "PASS"
RESULT_FAIL = "FAIL"
RESULT_SERIAL_REVIEW = "SERIAL_OK_OR_REVIEW"
RESULT_REVIEW = "REVIEW_REQUIRED"
RESULT_OPERATOR = "OPERATOR_CHECK_REQUIRED"
RESULT_SKIPPED_UNSAFE = "SKIPPED_UNSAFE"
RESULT_SKIPPED_DRY_RUN = "SKIPPED_DRY_RUN"
RESULT_TIMEOUT = "TIMEOUT"
RESULT_NOT_IMPLEMENTED = "NOT_IMPLEMENTED"

RAW_WRITE_CONFIRMATION = "BME280_RAW_WRITE"
CLAIM_BOUNDARY = (
    "Serial runner results are evidence inputs only. They do not prove "
    "environmental accuracy, field readiness, protected fault recovery, or "
    "completed physical hardware validation without operator-reviewed matrix "
    "evidence."
)

ANSI_RE = re.compile(r"\x1b\[[0-9;]*m")
FAILURE_PATTERNS = (
    re.compile(r"\bStatus:\s*(?:DEVICE_NOT_FOUND|I2C_TIMEOUT|I2C_NACK_ADDR|"
               r"I2C_NACK_DATA|I2C_BUS|I2C_ERROR|CHIP_ID_MISMATCH|"
               r"CALIBRATION_INVALID|INVALID_CONFIG|INVALID_PARAM|"
               r"NOT_INITIALIZED)\b"),
    re.compile(r"\b(?:FAILED|FAIL)\b"),
    re.compile(r"\[E\]"),
)
CTRL_MEAS_RE = re.compile(r"Reg\s+0xF4\s*=\s*0x([0-9A-Fa-f]{1,2})")
STATUS_RE = re.compile(r"Status:\s*0x([0-9A-Fa-f]{1,2})\s*\(measuring=(\d),\s*im_update=(\d)\)")
STRESS_ERRORS_RE = re.compile(r"\bErrors:\s*(\d+)\b")
SELFTEST_RESULT_RE = re.compile(r"Selftest result:\s*pass=\s*(\d+)\s*fail=\s*(\d+)\s*skip=\s*(\d+)")
CONSECUTIVE_FAILURES_RE = re.compile(r"Consecutive failures:\s*(\d+)")
TOTAL_FAILURES_RE = re.compile(r"Total failures:\s*(\d+)")
CHIP_ID_RE = re.compile(r"Chip ID:\s*0x([0-9A-Fa-f]{1,2})|Reg\s+0xD0\s*=\s*0x([0-9A-Fa-f]{1,2})")

VALIDATOR_CTRL_MEAS_SLEEP = "ctrl_meas_sleep"
VALIDATOR_STATUS_NOT_MEASURING = "status_not_measuring"
VALIDATOR_STATUS_IM_UPDATE_CLEAR = "status_im_update_clear"
VALIDATOR_STRESS_ZERO_ERRORS = "stress_zero_errors"
VALIDATOR_SELFTEST_ZERO_FAIL = "selftest_zero_fail"
VALIDATOR_DRIVER_ZERO_CONSECUTIVE = "driver_zero_consecutive_failures"


@dataclasses.dataclass(frozen=True)
class CommandSpec:
    command: str
    purpose: str
    group: str = "default"
    expected: tuple[str, ...] = ()
    expected_any: tuple[str, ...] = ()
    completion: tuple[str, ...] = ()
    validators: tuple[str, ...] = ()
    failures: tuple[str, ...] = ()
    timeout_s: float = 5.0
    pre_delay_s: float = 0.0
    operator_check: bool = False
    destructive: bool = False
    requires_opt_in: str | None = None
    recovery_command: str | None = None
    notes: str = ""

    def formatted(self, *, address: str) -> "CommandSpec":
        return dataclasses.replace(self, command=self.command.format(address=address))


BASE_COMMANDS: tuple[CommandSpec, ...] = (
    CommandSpec(
        command="version",
        purpose="Capture firmware/library version and embedded git status.",
        group="provenance",
        expected=("Version Info", "BME280 library", "commit"),
        timeout_s=4.0,
        notes="Version output identifies firmware provenance but is not hardware proof.",
    ),
    CommandSpec(
        command="help",
        purpose="Capture the CLI command surface used by this HIL run.",
        group="provenance",
        expected=("BME280 CLI Help", "scan", "probe", "selftest"),
        timeout_s=4.0,
    ),
    CommandSpec(
        command="scan",
        purpose="Scan the I2C bus for acknowledging addresses.",
        group="bus-reachability",
        expected=("device(s)",),
        failures=("No I2C devices found",),
        timeout_s=8.0,
        notes="A scan proves only address ACK; it is not chip identity.",
    ),
    CommandSpec(
        command="addr {address}",
        purpose="Select the documented BME280 diagnostic address and re-run begin().",
        group="bus-reachability",
        expected=("Status: OK",),
        failures=("DEVICE_NOT_FOUND", "I2C_NACK_ADDR", "I2C_TIMEOUT", "I2C_BUS"),
        timeout_s=6.0,
        notes="Use 0x76 for SDO=GND or 0x77 for SDO=VDDIO.",
    ),
    CommandSpec(
        command="begin",
        purpose="Initialize the driver, verify chip ID, read calibration, and apply config.",
        group="bus-reachability",
        expected=("Status: OK",),
        timeout_s=6.0,
    ),
    CommandSpec(
        command="probe",
        purpose="Run raw probe without health side effects.",
        group="bus-reachability",
        expected=("Status: OK",),
        failures=("DEVICE_NOT_FOUND", "I2C_NACK_ADDR", "I2C_TIMEOUT", "I2C_BUS"),
        timeout_s=5.0,
        notes="Probe plus ACK does not prove identity unless chip ID is read separately.",
    ),
    CommandSpec(
        command="chipid",
        purpose="Read BME280 chip ID register 0xD0 and verify documented value 0x60.",
        group="identity-calibration",
        expected=("Chip ID: 0x60",),
        failures=("CHIP_ID_MISMATCH", "I2C_TIMEOUT", "I2C_BUS"),
        timeout_s=5.0,
        notes="This is the identity check for BME280.",
    ),
    CommandSpec(
        command="cfg",
        purpose="Capture chip and cached driver settings.",
        group="identity-calibration",
        expected=("ctrl_hum", "ctrl_meas", "config", "Hardware config dirty:"),
        completion=("Hardware config dirty:",),
        timeout_s=10.0,
    ),
    CommandSpec(
        command="calib",
        purpose="Capture cached calibration coefficients for plausibility review.",
        group="identity-calibration",
        expected=("Calibration (Cached)", "T1=", "P1=", "H1=", "Plausibility:"),
        completion=("Plausibility:",),
        timeout_s=10.0,
        notes="Calibration plausibility still requires auditor/operator review.",
    ),
    CommandSpec(
        command="calib raw",
        purpose="Capture raw calibration register bytes.",
        group="identity-calibration",
        expected=("Calibration (Raw Registers)", "TP:", "H:"),
        timeout_s=5.0,
    ),
    CommandSpec(
        command="status",
        purpose="Read status register and driver dirty-state summary.",
        group="identity-calibration",
        expected=("Status: 0x", "Driver:"),
        validators=(VALIDATOR_STATUS_NOT_MEASURING,),
        timeout_s=5.0,
    ),
    CommandSpec(
        command="timing",
        purpose="Record measurement timing estimates for the current config.",
        group="identity-calibration",
        expected=("Estimated measurement time", "Estimated normal cycle"),
        timeout_s=5.0,
    ),
    CommandSpec(
        command="reg 0xD0",
        purpose="Read chip ID register through generic register command.",
        group="identity-calibration",
        expected=("Reg 0xD0 = 0x60",),
        timeout_s=5.0,
        notes="Duplicates chip-ID evidence through raw register helper.",
    ),
    CommandSpec(
        command="read",
        purpose="Request and capture one compensated measurement.",
        group="forced-mode",
        expected=("Temp:", "Pressure:", "Humidity:"),
        timeout_s=10.0,
        operator_check=True,
        notes="Operator must judge environmental plausibility against references.",
    ),
    CommandSpec(
        command="raw",
        purpose="Capture cached raw ADC sample and validity flags.",
        group="forced-mode",
        expected=("Raw ADC", "Valid channels: T=", "Cached sample age"),
        timeout_s=5.0,
    ),
    CommandSpec(
        command="comp",
        purpose="Capture cached fixed-point compensated sample and validity flags.",
        group="forced-mode",
        expected=("Compensated", "Valid channels: T="),
        timeout_s=5.0,
    ),
    CommandSpec(
        command="data",
        purpose="Burst-read and decode live data registers 0xF7..0xFE.",
        group="forced-mode",
        expected=("Live Data Registers", "0xF7..0xFE"),
        timeout_s=5.0,
    ),
    CommandSpec(
        command="force",
        purpose="Trigger a forced-mode measurement.",
        group="forced-mode",
        expected=("Temp:", "Pressure:", "Humidity:"),
        timeout_s=10.0,
        operator_check=True,
        notes="Operator must judge plausibility; serial confirms only transaction flow.",
    ),
    CommandSpec(
        command="reg 0xF4",
        purpose="Capture ctrl_meas after forced measurement to verify return-to-sleep mode bits.",
        group="forced-mode",
        expected=("Reg 0xF4 = 0x",),
        validators=(VALIDATOR_CTRL_MEAS_SLEEP,),
        timeout_s=5.0,
        notes="For formal forced-mode evidence, ctrl_meas mode bits [1:0] must be 00.",
    ),
    CommandSpec(
        command="status",
        purpose="Capture post-forced-measurement status flags.",
        group="forced-mode",
        expected=("Status: 0x", "measuring=0", "Driver:"),
        validators=(VALIDATOR_STATUS_NOT_MEASURING,),
        timeout_s=5.0,
        notes="Post-force status should show measuring=0 after the forced sample is available.",
    ),
    CommandSpec(
        command="read",
        purpose="Read again after forced trigger to capture sample/cache behavior.",
        group="forced-mode",
        expected=("Temp:", "Pressure:", "Humidity:"),
        timeout_s=10.0,
        operator_check=True,
    ),
    CommandSpec(
        command="normal on",
        purpose="Switch to normal mode.",
        group="normal-mode",
        expected=("Status: OK",),
        timeout_s=5.0,
        recovery_command="normal off",
    ),
    CommandSpec(
        command="read",
        purpose="Capture a normal-mode fresh sample.",
        group="normal-mode",
        expected=("Temp:", "Pressure:", "Humidity:"),
        timeout_s=12.0,
        operator_check=True,
    ),
    CommandSpec(
        command="read",
        purpose="Capture a second normal-mode sample for repeated-read evidence.",
        group="normal-mode",
        expected=("Temp:", "Pressure:", "Humidity:"),
        timeout_s=12.0,
        operator_check=True,
    ),
    CommandSpec(
        command="normal off",
        purpose="Return to sleep mode after normal-mode sample.",
        group="normal-mode",
        expected=("Status: OK",),
        timeout_s=5.0,
    ),
    CommandSpec(
        command="reset",
        purpose="Run BME280 soft reset, NVM wait, calibration reload, and config reapply.",
        group="reset-recover",
        expected=("Status: OK",),
        timeout_s=8.0,
        notes="Soft reset is volatile and expected to be safe for BME280.",
    ),
    CommandSpec(
        command="status",
        purpose="Capture post-reset status, NVM flag, and dirty-state summary.",
        group="reset-recover",
        expected=("Status: 0x", "im_update=0", "Driver:"),
        validators=(VALIDATOR_STATUS_NOT_MEASURING, VALIDATOR_STATUS_IM_UPDATE_CLEAR),
        timeout_s=5.0,
        notes="Post-reset evidence should show bounded NVM copy completed.",
    ),
    CommandSpec(
        command="recover",
        purpose="Run manual recover/resync path.",
        group="reset-recover",
        expected=("Status: OK",),
        timeout_s=8.0,
    ),
    CommandSpec(
        command="cfg",
        purpose="Capture settings after recover/resync.",
        group="reset-recover",
        expected=("ctrl_hum", "ctrl_meas", "config", "Hardware config dirty:"),
        completion=("Hardware config dirty:",),
        timeout_s=10.0,
        notes="Shows readable cached/hardware settings after recovery.",
    ),
    CommandSpec(
        command="status",
        purpose="Capture post-recover dirty-state summary.",
        group="reset-recover",
        expected=("Status: 0x", "Driver:", "dirty=false"),
        validators=(VALIDATOR_STATUS_NOT_MEASURING,),
        timeout_s=5.0,
        notes="Dirty state should be clear after successful recover/resync.",
    ),
    CommandSpec(
        command="selftest",
        purpose="Run existing safe command smoke-test report.",
        group="stress-health",
        expected=("selftest", "PASS"),
        completion=("Selftest result:",),
        validators=(VALIDATOR_SELFTEST_ZERO_FAIL,),
        timeout_s=25.0,
        operator_check=True,
        notes="This is not Bosch factory calibration; review the report.",
    ),
    CommandSpec(
        command="stress 10",
        purpose="Run a short forced-measurement stress loop.",
        group="stress-health",
        expected=("Stress Summary", "Errors:"),
        completion=("Health delta:",),
        validators=(VALIDATOR_STRESS_ZERO_ERRORS,),
        timeout_s=30.0,
        operator_check=True,
        notes="Serial summary needs operator review for sample plausibility.",
    ),
    CommandSpec(
        command="drv",
        purpose="Capture final driver health details.",
        group="stress-health",
        expected=("Driver Health", "Total"),
        validators=(VALIDATOR_DRIVER_ZERO_CONSECUTIVE,),
        timeout_s=5.0,
    ),
    CommandSpec(
        command="state",
        purpose="Capture final compact health state.",
        group="stress-health",
        expected=("state",),
        timeout_s=5.0,
    ),
)


def forced_soak_commands(count: int) -> tuple[CommandSpec, ...]:
    return (
        CommandSpec(
            command=f"stress {count}",
            purpose="Run the requested longer forced-measurement stress loop.",
            group="soak-forced",
            expected=("Stress Summary", "Errors:"),
            completion=("Health delta:",),
            validators=(VALIDATOR_STRESS_ZERO_ERRORS,),
            timeout_s=max(30.0, min(900.0, 0.5 * float(count) + 30.0)),
            operator_check=True,
            requires_opt_in="--include-soak",
            notes=(
                "Longer forced-measurement soak evidence; still requires operator "
                "plausibility review."
            ),
        ),
    )


def normal_soak_commands(count: int, interval_s: float) -> tuple[CommandSpec, ...]:
    reads = tuple(
        CommandSpec(
            command="read",
            purpose=f"Normal-mode soak read {idx + 1} of {count}.",
            group="soak-normal",
            expected=("Temp:", "Pressure:", "Humidity:"),
            timeout_s=12.0,
            pre_delay_s=interval_s if idx > 0 else 0.0,
            operator_check=True,
            requires_opt_in="--include-normal-soak",
            notes="Operator must compare repeated samples against references and timing notes.",
        )
        for idx in range(count)
    )
    return (
        CommandSpec(
            command="normal on",
            purpose="Enter normal mode for opt-in repeated-read soak.",
            group="soak-normal",
            expected=("Status: OK",),
            timeout_s=5.0,
            requires_opt_in="--include-normal-soak",
            recovery_command="normal off",
        ),
        *reads,
        CommandSpec(
            command="normal off",
            purpose="Return to sleep mode after opt-in normal-mode soak.",
            group="soak-normal",
            expected=("Status: OK",),
            timeout_s=5.0,
            requires_opt_in="--include-normal-soak",
        ),
        CommandSpec(
            command="cfg",
            purpose="Capture config after opt-in normal-mode soak.",
            group="soak-normal",
            expected=("ctrl_hum", "ctrl_meas", "config", "Hardware config dirty:"),
            completion=("Hardware config dirty:",),
            timeout_s=10.0,
            requires_opt_in="--include-normal-soak",
        ),
        CommandSpec(
            command="status",
            purpose="Capture status and dirty-state after opt-in normal-mode soak.",
            group="soak-normal",
            expected=("Status: 0x", "Driver:"),
            validators=(VALIDATOR_STATUS_NOT_MEASURING,),
            timeout_s=5.0,
            requires_opt_in="--include-normal-soak",
        ),
    )


DESTRUCTIVE_COMMANDS: tuple[CommandSpec, ...] = (
    CommandSpec(
        command="wreg 0xF4 0x00",
        purpose="Diagnostic raw register write to force ctrl_meas sleep.",
        group="raw-write",
        expected=("Status: OK",),
        timeout_s=5.0,
        destructive=True,
        requires_opt_in="--include-destructive",
        recovery_command="recover",
        notes="Raw writes can desynchronize cached config. Excluded by default.",
    ),
)


MANUAL_FAULT_CHECKS: tuple[CommandSpec, ...] = (
    CommandSpec(
        command="MANUAL: wrong-address probe with addr 0x76/0x77 opposite SDO wiring",
        purpose="Verify definite address NACK behavior.",
        group="manual-fault",
        operator_check=True,
        requires_opt_in="--include-fault-tests",
        notes="Do not hotwire SDO while powered unless the fixture supports it.",
    ),
    CommandSpec(
        command="MANUAL: safe sensor unplug/replug and recover",
        purpose="Capture address-NACK and recovery behavior.",
        group="manual-fault",
        operator_check=True,
        requires_opt_in="--include-fault-tests",
        notes="Only disconnect when safe for the board and bench setup.",
    ),
    CommandSpec(
        command="MANUAL: safe SDA/SCL temporary fault if supported",
        purpose="Capture timeout/bus-fault behavior.",
        group="manual-fault",
        operator_check=True,
        requires_opt_in="--include-fault-tests",
        notes="Use current limits and do not short active outputs unsafely.",
    ),
)


def strip_ansi(text: str) -> str:
    return ANSI_RE.sub("", text)


def git_value(args: list[str]) -> str:
    try:
        return subprocess.check_output(
            ["git", *args], cwd=ROOT, text=True, stderr=subprocess.DEVNULL
        ).strip()
    except Exception:
        return "unavailable"


def worktree_state() -> str:
    status = git_value(["status", "--short"])
    return "dirty" if status else "clean"


def parse_address(value: str) -> str:
    text = value.strip().lower()
    try:
        parsed = int(text, 0)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("address must be 0x76 or 0x77") from exc
    if parsed not in (0x76, 0x77):
        raise argparse.ArgumentTypeError("address must be 0x76 or 0x77")
    return f"0x{parsed:02X}"


def parse_positive_int(value: str) -> int:
    try:
        parsed = int(value, 10)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("value must be a positive integer") from exc
    if parsed <= 0:
        raise argparse.ArgumentTypeError("value must be a positive integer")
    return parsed


def parse_nonnegative_float(value: str) -> float:
    try:
        parsed = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("value must be a non-negative number") from exc
    if parsed < 0.0:
        raise argparse.ArgumentTypeError("value must be a non-negative number")
    return parsed


def load_command_file(path: pathlib.Path, *, timeout_s: float) -> list[CommandSpec]:
    commands: list[CommandSpec] = []
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        commands.append(
            CommandSpec(
                command=line,
                purpose="Operator-supplied command file entry.",
                group="custom-command-file",
                timeout_s=timeout_s,
                notes=f"Loaded from {path}",
            )
        )
    return commands


def stress_count(command: str) -> int | None:
    parts = command.strip().split()
    if len(parts) != 2 or parts[0].lower() != "stress":
        return None
    try:
        return int(parts[1], 10)
    except ValueError:
        return None


def custom_command_safety_errors(commands: list[CommandSpec], args: argparse.Namespace) -> list[str]:
    errors: list[str] = []
    include_destructive = getattr(args, "include_destructive", False)
    include_soak = getattr(args, "include_soak", False)
    for spec in commands:
        command = spec.command.strip().lower()
        if command.startswith("wreg ") and not include_destructive:
            errors.append(f"`{spec.command}` requires --include-destructive")
        count = stress_count(command)
        if count is not None and count > 100 and not include_soak:
            errors.append(f"`{spec.command}` requires --include-soak for counts above 100")
    return errors


def build_command_sequence(args: argparse.Namespace) -> tuple[list[CommandSpec], list[CommandSpec]]:
    if args.commands:
        commands = load_command_file(pathlib.Path(args.commands), timeout_s=args.timeout)
        errors = custom_command_safety_errors(commands, args)
        if errors:
            raise ValueError("Custom command file contains gated commands: " + "; ".join(errors))
        return commands, []

    executable = [cmd.formatted(address=args.address) for cmd in BASE_COMMANDS]
    checklist: list[CommandSpec] = []
    soak_count = getattr(args, "soak_count", 500)
    normal_soak_count = getattr(args, "normal_soak_count", 5)
    normal_soak_interval_s = getattr(args, "normal_soak_interval_s", 1.0)

    if args.include_soak:
        executable.extend(cmd.formatted(address=args.address) for cmd in forced_soak_commands(soak_count))
    else:
        checklist.extend(cmd.formatted(address=args.address) for cmd in forced_soak_commands(soak_count))

    if getattr(args, "include_normal_soak", False):
        executable.extend(
            cmd.formatted(address=args.address)
            for cmd in normal_soak_commands(normal_soak_count, normal_soak_interval_s)
        )
    else:
        checklist.extend(
            cmd.formatted(address=args.address)
            for cmd in normal_soak_commands(normal_soak_count, normal_soak_interval_s)
        )

    if args.include_destructive:
        executable.extend(cmd.formatted(address=args.address) for cmd in DESTRUCTIVE_COMMANDS)
        executable.extend(
            (
                CommandSpec(
                    command="recover",
                    purpose="Resync after opt-in destructive raw-write diagnostic.",
                    group="raw-write",
                    expected=("Status: OK",),
                    timeout_s=8.0,
                    requires_opt_in="--include-destructive",
                    notes="Required evidence after raw register write.",
                ),
                CommandSpec(
                    command="cfg",
                    purpose="Capture settings after destructive-write recovery.",
                    group="raw-write",
                    expected=("ctrl_hum", "ctrl_meas", "config"),
                    timeout_s=5.0,
                    requires_opt_in="--include-destructive",
                    notes="Required evidence that recovery produced a readable configuration.",
                ),
                CommandSpec(
                    command="status",
                    purpose="Capture dirty-state summary after destructive-write recovery.",
                    group="raw-write",
                    expected=("Status: 0x", "Driver:", "dirty=false"),
                    validators=(VALIDATOR_STATUS_NOT_MEASURING,),
                    timeout_s=5.0,
                    requires_opt_in="--include-destructive",
                    notes="Required evidence that recovery cleared dirty state after raw write.",
                ),
            )
        )
    else:
        checklist.extend(cmd.formatted(address=args.address) for cmd in DESTRUCTIVE_COMMANDS)

    fault_items = [cmd.formatted(address=args.address) for cmd in MANUAL_FAULT_CHECKS]
    if args.include_fault_tests:
        checklist.extend(
            dataclasses.replace(
                item,
                notes=f"{item.notes} Requested via --include-fault-tests; execute and sign off manually.",
            )
            for item in fault_items
        )
    else:
        checklist.extend(fault_items)

    return executable, checklist


def make_log_dir(base: pathlib.Path) -> pathlib.Path:
    stamp = dt.datetime.now().strftime("i2c_%Y%m%d_%H%M%S")
    candidate = base / stamp
    if not candidate.exists():
        candidate.mkdir(parents=True)
        return candidate
    for idx in range(1, 1000):
        candidate = base / f"{stamp}_{idx:03d}"
        if not candidate.exists():
            candidate.mkdir(parents=True)
            return candidate
    raise RuntimeError(f"could not create unique log directory under {base}")


def timestamp() -> str:
    return dt.datetime.now().astimezone().isoformat(timespec="seconds")


def expected_tokens_match(spec: CommandSpec, output: str) -> bool:
    if spec.expected and not all(token in output for token in spec.expected):
        return False
    if spec.expected_any and not any(token in output for token in spec.expected_any):
        return False
    return bool(spec.expected or spec.expected_any)


def completion_tokens_match(spec: CommandSpec, output: str) -> bool:
    if spec.completion:
        return all(token in output for token in spec.completion)
    return expected_tokens_match(spec, output)


def extract_parsed_evidence(output: str) -> dict:
    evidence: dict[str, object] = {}
    if match := CTRL_MEAS_RE.search(output):
        ctrl_meas = int(match.group(1), 16)
        evidence["ctrl_meas"] = f"0x{ctrl_meas:02X}"
        evidence["ctrl_meas_mode_bits"] = ctrl_meas & 0x03
    if match := STATUS_RE.search(output):
        evidence["status_register"] = f"0x{int(match.group(1), 16):02X}"
        evidence["measuring"] = int(match.group(2))
        evidence["im_update"] = int(match.group(3))
    if match := STRESS_ERRORS_RE.search(output):
        evidence["stress_errors"] = int(match.group(1))
    if match := SELFTEST_RESULT_RE.search(output):
        evidence["selftest_pass"] = int(match.group(1))
        evidence["selftest_fail"] = int(match.group(2))
        evidence["selftest_skip"] = int(match.group(3))
    if match := CONSECUTIVE_FAILURES_RE.search(output):
        evidence["consecutive_failures"] = int(match.group(1))
    if match := TOTAL_FAILURES_RE.search(output):
        evidence["total_failures"] = int(match.group(1))
    if match := CHIP_ID_RE.search(output):
        chip_id = match.group(1) or match.group(2)
        evidence["chip_id"] = f"0x{int(chip_id, 16):02X}"
    return evidence


def validate_parsed_output(spec: CommandSpec, output: str) -> tuple[str | None, str]:
    evidence = extract_parsed_evidence(output)
    reasons: list[str] = []
    for validator in spec.validators:
        if validator == VALIDATOR_CTRL_MEAS_SLEEP:
            mode_bits = evidence.get("ctrl_meas_mode_bits")
            if mode_bits is None:
                return RESULT_REVIEW, "ctrl_meas register value was not found in serial output."
            if mode_bits != 0:
                return RESULT_FAIL, f"ctrl_meas mode bits are {mode_bits:#04b}, expected sleep mode 0b00."
            reasons.append("ctrl_meas mode bits are 0b00")
        elif validator == VALIDATOR_STATUS_NOT_MEASURING:
            measuring = evidence.get("measuring")
            if measuring is None:
                return RESULT_REVIEW, "status measuring flag was not parsed from serial output."
            if measuring != 0:
                return RESULT_FAIL, "status measuring flag is 1, expected 0."
            reasons.append("status measuring=0")
        elif validator == VALIDATOR_STATUS_IM_UPDATE_CLEAR:
            im_update = evidence.get("im_update")
            if im_update is None:
                return RESULT_REVIEW, "status im_update flag was not parsed from serial output."
            if im_update != 0:
                return RESULT_FAIL, "status im_update flag is 1, expected 0 after bounded NVM copy."
            reasons.append("status im_update=0")
        elif validator == VALIDATOR_STRESS_ZERO_ERRORS:
            errors = evidence.get("stress_errors")
            if errors is None:
                return RESULT_REVIEW, "stress error count was not parsed from serial output."
            if errors != 0:
                return RESULT_FAIL, f"stress reported {errors} error(s)."
            reasons.append("stress Errors=0")
        elif validator == VALIDATOR_SELFTEST_ZERO_FAIL:
            failures = evidence.get("selftest_fail")
            if failures is None:
                return RESULT_REVIEW, "selftest summary fail count was not parsed from serial output."
            if failures != 0:
                return RESULT_FAIL, f"selftest reported fail={failures}."
            reasons.append("selftest fail=0")
        elif validator == VALIDATOR_DRIVER_ZERO_CONSECUTIVE:
            failures = evidence.get("consecutive_failures")
            if failures is None:
                return RESULT_REVIEW, "driver consecutive failure count was not parsed from serial output."
            if failures != 0:
                return RESULT_FAIL, f"driver consecutive failures={failures}, expected 0."
            reasons.append("driver consecutive failures=0")
        else:
            return RESULT_REVIEW, f"unknown runner validator: {validator}"
    if reasons:
        return None, "; ".join(reasons)
    return None, ""


def classify_output(spec: CommandSpec, output: str, completion: str) -> tuple[str, str]:
    clean = strip_ansi(output)
    if completion == RESULT_TIMEOUT:
        return RESULT_TIMEOUT, "Command timed out before serial idle/completion."
    for failure in spec.failures:
        if failure and failure in clean:
            return RESULT_FAIL, f"Matched command failure token: {failure}"
    for pattern in FAILURE_PATTERNS:
        if pattern.search(clean):
            return RESULT_FAIL, f"Matched failure pattern: {pattern.pattern}"
    validator_result, validator_reason = validate_parsed_output(spec, clean)
    if validator_result is not None:
        return validator_result, validator_reason
    matched = expected_tokens_match(spec, clean)
    if spec.operator_check:
        if matched:
            suffix = f" Parsed evidence: {validator_reason}." if validator_reason else ""
            return RESULT_OPERATOR, "Serial output matched expected tokens; operator plausibility review required." + suffix
        return RESULT_REVIEW, "Manual/operator command did not match all expected tokens."
    if matched:
        suffix = f" Parsed evidence: {validator_reason}." if validator_reason else ""
        return RESULT_PASS, "Matched expected serial tokens." + suffix
    if clean.strip():
        return RESULT_SERIAL_REVIEW, "Serial output captured but expected tokens were incomplete."
    return RESULT_REVIEW, "No useful serial output captured."


def output_has_expected(spec: CommandSpec, output: str) -> bool:
    return completion_tokens_match(spec, output)


def output_has_failure(spec: CommandSpec, output: str) -> bool:
    for failure in spec.failures:
        if failure and failure in output:
            return True
    return any(pattern.search(output) for pattern in FAILURE_PATTERNS)


def import_serial():
    try:
        import serial  # type: ignore
    except ModuleNotFoundError as exc:
        print("pyserial is required for non-dry-run serial HIL execution.", file=sys.stderr)
        print(f"Install with: {INSTALL_HINT}", file=sys.stderr)
        raise SystemExit(2) from exc
    return serial


def read_available(ser) -> str:
    waiting = getattr(ser, "in_waiting", 0)
    if waiting:
        data = ser.read(waiting)
    else:
        data = ser.read(1)
        waiting = getattr(ser, "in_waiting", 0)
        if waiting:
            data += ser.read(waiting)
    return data.decode("utf-8", errors="replace")


def run_serial_command(ser, spec: CommandSpec, transcript, *, timeout_s: float) -> dict:
    start = time.monotonic()
    deadline = start + timeout_s
    idle_after_output_s = 0.75
    idle_after_match_s = 0.25
    last_rx = time.monotonic()
    saw_output = False
    matched_expected = False
    matched_failure = False
    chunks: list[str] = []

    boundary = f"\n--- COMMAND {spec.command!r} @ {timestamp()} ---\n"
    transcript.write(boundary)
    transcript.flush()

    if spec.pre_delay_s > 0.0:
        transcript.write(f"PRE_DELAY {spec.pre_delay_s:.3f}s before command\n")
        transcript.flush()
        time.sleep(spec.pre_delay_s)

    ser.write((spec.command + "\n").encode("utf-8"))
    ser.flush()

    while True:
        chunk = read_available(ser)
        if chunk:
            saw_output = True
            chunks.append(chunk)
            transcript.write(chunk)
            transcript.flush()
            last_rx = time.monotonic()
            clean = strip_ansi("".join(chunks))
            matched_expected = output_has_expected(spec, clean)
            matched_failure = output_has_failure(spec, clean)

        now = time.monotonic()
        if saw_output and (matched_expected or matched_failure) and now - last_rx >= idle_after_match_s:
            completion = "MATCHED_FAILURE" if matched_failure else "MATCHED_EXPECTED"
            break
        if saw_output and not spec.expected and now - last_rx >= idle_after_output_s:
            completion = "SERIAL_IDLE_NO_EXPECTED_TOKENS"
            break
        if now >= deadline:
            completion = RESULT_TIMEOUT
            break
        time.sleep(0.05)

    output = "".join(chunks)
    elapsed = time.monotonic() - start
    result, reason = classify_output(spec, output, completion)
    clean_output = strip_ansi(output)
    return {
        "command": spec.command,
        "purpose": spec.purpose,
        "group": spec.group,
        "expected": list(spec.expected),
        "expected_any": list(spec.expected_any),
        "validators": list(spec.validators),
        "serial_result": result,
        "operator_result": RESULT_OPERATOR if spec.operator_check else "",
        "completion": completion,
        "elapsed_s": round(elapsed, 3),
        "pre_delay_s": spec.pre_delay_s,
        "notes": spec.notes,
        "classification_reason": reason,
        "destructive": spec.destructive,
        "requires_opt_in": spec.requires_opt_in,
        "recovery_command": spec.recovery_command,
        "parsed_evidence": extract_parsed_evidence(clean_output),
        "output_excerpt": clean_output[-1000:],
    }


def dry_run_result(spec: CommandSpec) -> dict:
    if spec.destructive:
        result = RESULT_SKIPPED_UNSAFE
    elif spec.command.startswith("MANUAL:"):
        result = RESULT_OPERATOR
    else:
        result = RESULT_SKIPPED_DRY_RUN
    return {
        "command": spec.command,
        "purpose": spec.purpose,
        "group": spec.group,
        "expected": list(spec.expected),
        "expected_any": list(spec.expected_any),
        "validators": list(spec.validators),
        "serial_result": result,
        "operator_result": RESULT_OPERATOR if spec.operator_check else "",
        "completion": "DRY_RUN",
        "elapsed_s": 0.0,
        "pre_delay_s": spec.pre_delay_s,
        "notes": spec.notes,
        "classification_reason": "Dry-run only; no serial command was sent.",
        "destructive": spec.destructive,
        "requires_opt_in": spec.requires_opt_in,
        "recovery_command": spec.recovery_command,
        "parsed_evidence": {},
        "output_excerpt": "",
    }


def final_verdict(results: Iterable[dict], *, dry_run: bool) -> str:
    serial_results = [row["serial_result"] for row in results]
    if dry_run:
        return "INCOMPLETE"
    if any(result in (RESULT_FAIL, RESULT_TIMEOUT) for result in serial_results):
        return "FAIL"
    if any(result in (RESULT_OPERATOR, RESULT_REVIEW, RESULT_SERIAL_REVIEW) for result in serial_results):
        return "OPERATOR_REVIEW_REQUIRED"
    return "PASS"


def safe_cell(value: object) -> str:
    return str(value).replace("|", "\\|")


def artifact_path(summary: dict, key: str) -> str:
    return summary["artifacts"].get(key, "")


def write_summary_md(path: pathlib.Path, summary: dict) -> None:
    lines = [
        "# I2C HIL Summary",
        "",
        f"Run ID: `{summary['run_id']}`",
        f"Date/time: `{summary['datetime']}`",
        f"Runner command: `{summary['runner_command']}`",
        f"Runner arguments: `{json.dumps(summary['runner_args'])}`",
        f"Branch: `{summary['branch']}`",
        f"Commit: `{summary['commit']}`",
        f"Worktree: `{summary['worktree']}`",
        f"Port: `{summary['port']}`",
        f"Baud: `{summary['baud']}`",
        f"I2C address: `{summary['address']}`",
        f"Operator: `{summary['environment']['operator']}`",
        f"Board: `{summary['environment']['board']}`",
        f"MCU target: `{summary['environment']['mcu_target']}`",
        f"BME280 module: `{summary['environment']['module']}`",
        f"Dry run: `{summary['dry_run']}`",
        f"Final verdict: `{summary['final_verdict']}`",
        "",
        f"Claim boundary: {summary['claim_boundary']}",
        "",
        "## Command Results",
        "",
        "| Group | Command | Purpose | Expected/validators | Serial result | Operator result | Completion | Elapsed s | Notes |",
        "| --- | --- | --- | --- | --- | --- | --- | ---: | --- |",
    ]
    for row in summary["results"]:
        expected = ", ".join(row.get("expected", []) + row.get("expected_any", []) + row.get("validators", []))
        lines.append(
            f"| `{safe_cell(row['group'])}` | `{safe_cell(row['command'])}` | {safe_cell(row['purpose'])} | "
            f"{safe_cell(expected)} | `{row['serial_result']}` | "
            f"`{row['operator_result']}` | `{row['completion']}` | "
            f"{row['elapsed_s']:.3f} | {safe_cell(row['notes'])} |"
        )
    lines.extend(
        [
            "",
            "## Artifacts",
            "",
            f"- Serial transcript: `{artifact_path(summary, 'serial_transcript')}`",
            f"- Markdown summary: `{artifact_path(summary, 'summary_md')}`",
            f"- Timestamped Markdown summary: `{artifact_path(summary, 'timestamped_summary_md')}`",
            f"- JSON summary: `{artifact_path(summary, 'summary_json')}`",
            f"- Results CSV: `{artifact_path(summary, 'results_csv')}`",
            f"- Command plan: `{artifact_path(summary, 'command_plan')}`",
            f"- Environment record: `{artifact_path(summary, 'environment_txt')}`",
            f"- Operator checklist: `{artifact_path(summary, 'operator_checklist')}`",
            f"- Hardware matrix fragment: `{artifact_path(summary, 'hardware_matrix_fragment')}`",
            f"- Failure analysis: `{artifact_path(summary, 'failure_analysis')}`",
            f"- Manifest: `{artifact_path(summary, 'manifest')}`",
            "",
            "Hardware validation is complete only after this summary is reviewed with the "
            "captured transcript and required operator evidence.",
            "",
        ]
    )
    path.write_text("\n".join(lines), encoding="utf-8")


def write_results_csv(path: pathlib.Path, results: list[dict]) -> None:
    fieldnames = [
        "group",
        "command",
        "serial_result",
        "operator_result",
        "completion",
        "elapsed_s",
        "pre_delay_s",
        "classification_reason",
        "requires_opt_in",
        "destructive",
        "notes",
    ]
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        for row in results:
            writer.writerow(row)


def write_command_plan(path: pathlib.Path, executable: list[CommandSpec], manual_items: list[CommandSpec]) -> None:
    data = {
        "claim_boundary": CLAIM_BOUNDARY,
        "executable": [dataclasses.asdict(item) for item in executable],
        "manual_or_skipped": [dataclasses.asdict(item) for item in manual_items],
    }
    path.write_text(json.dumps(data, indent=2), encoding="utf-8")


def metadata_value(value: str | None) -> str:
    return value if value else "unknown"


def environment_record(args: argparse.Namespace) -> dict[str, object]:
    return {
        "operator": metadata_value(getattr(args, "operator", "")),
        "board": metadata_value(getattr(args, "board", "")),
        "mcu_target": metadata_value(getattr(args, "mcu_target", "")),
        "framework": metadata_value(getattr(args, "framework", "")),
        "build_target": metadata_value(getattr(args, "build_target", "")),
        "module": metadata_value(getattr(args, "module", "")),
        "vdd": metadata_value(getattr(args, "vdd", "")),
        "vddio": metadata_value(getattr(args, "vddio", "")),
        "pullups": metadata_value(getattr(args, "pullups", "")),
        "pullup_location": metadata_value(getattr(args, "pullup_location", "")),
        "sda_pin": metadata_value(getattr(args, "sda_pin", "")),
        "scl_pin": metadata_value(getattr(args, "scl_pin", "")),
        "bus_speed": metadata_value(getattr(args, "bus_speed", "")),
        "sdo_state": metadata_value(getattr(args, "sdo_state", "")),
        "csb_state": metadata_value(getattr(args, "csb_state", "")),
        "environment_ref": metadata_value(getattr(args, "environment_ref", "")),
        "operator_notes": metadata_value(getattr(args, "operator_notes", "")),
        "command_file": metadata_value(getattr(args, "commands", "")),
        "opt_in_flags": {
            "include_soak": getattr(args, "include_soak", False),
            "soak_count": getattr(args, "soak_count", 500),
            "include_normal_soak": getattr(args, "include_normal_soak", False),
            "normal_soak_count": getattr(args, "normal_soak_count", 5),
            "normal_soak_interval_s": getattr(args, "normal_soak_interval_s", 1.0),
            "include_destructive": getattr(args, "include_destructive", False),
            "include_fault_tests": getattr(args, "include_fault_tests", False),
        },
    }


def write_environment(path: pathlib.Path, summary: dict) -> None:
    lines = [
        "# I2C HIL Environment",
        "",
        f"run_id={summary['run_id']}",
        f"datetime={summary['datetime']}",
        f"branch={summary['branch']}",
        f"commit={summary['commit']}",
        f"worktree={summary['worktree']}",
        f"runner_command={summary['runner_command']}",
        f"runner_args={json.dumps(summary['runner_args'])}",
        f"port={summary['port']}",
        f"baud={summary['baud']}",
        f"address={summary['address']}",
    ]
    for key, value in summary["environment"].items():
        if isinstance(value, dict):
            lines.append(f"{key}={json.dumps(value, sort_keys=True)}")
        else:
            lines.append(f"{key}={value}")
    lines.append("")
    path.write_text("\n".join(lines), encoding="utf-8")


def write_hardware_matrix_fragment(path: pathlib.Path, summary: dict) -> None:
    lines = [
        "# BME280 Hardware Matrix Fragment",
        "",
        "This fragment is generated by `tools/run_i2c_hil.py` for operator review. "
        "It does not update the committed hardware validation matrix by itself.",
        "",
        f"- Run ID: `{summary['run_id']}`",
        f"- Final verdict: `{summary['final_verdict']}`",
        f"- Claim boundary: {summary['claim_boundary']}",
        "",
        "## Setup Fields",
        "",
        "| Field | Value |",
        "| --- | --- |",
        f"| Branch | `{summary['branch']}` |",
        f"| Commit | `{summary['commit']}` |",
        f"| Worktree state / dirty flag | `{summary['worktree']}` |",
        f"| HIL runner command | `{summary['runner_command']}` |",
        f"| HIL runner arguments | `{json.dumps(summary['runner_args'])}` |",
        f"| Serial port and baud | `{summary['port']}` / `{summary['baud']}` |",
        f"| BME280 address | `{summary['address']}` |",
    ]
    for key, value in summary["environment"].items():
        if not isinstance(value, dict):
            lines.append(f"| {safe_cell(key)} | `{safe_cell(value)}` |")
    lines.extend(["", "## Command Evidence", "", "| Group | Command | Result | Parsed evidence |", "| --- | --- | --- | --- |"])
    for row in summary["results"]:
        evidence = json.dumps(row.get("parsed_evidence", {}), sort_keys=True)
        lines.append(
            f"| `{safe_cell(row['group'])}` | `{safe_cell(row['command'])}` | "
            f"`{row['serial_result']}` | `{safe_cell(evidence)}` |"
        )
    lines.append("")
    path.write_text("\n".join(lines), encoding="utf-8")


def write_failure_analysis(path: pathlib.Path, summary: dict) -> None:
    actionable = [
        row
        for row in summary["results"]
        if row["serial_result"] in (RESULT_FAIL, RESULT_TIMEOUT, RESULT_REVIEW, RESULT_SERIAL_REVIEW)
    ]
    lines = [
        "# I2C HIL Failure Analysis",
        "",
        f"Final verdict: `{summary['final_verdict']}`",
        "",
    ]
    if not actionable:
        lines.append("No automated failure or review-required serial classifications were recorded.")
    else:
        lines.extend(["| Group | Command | Result | Reason |", "| --- | --- | --- | --- |"])
        for row in actionable:
            lines.append(
                f"| `{safe_cell(row['group'])}` | `{safe_cell(row['command'])}` | "
                f"`{row['serial_result']}` | {safe_cell(row['classification_reason'])} |"
            )
    lines.extend(["", "Operator-review rows still require setup, reference, and sign-off review."])
    path.write_text("\n".join(lines), encoding="utf-8")


def write_operator_checklist(path: pathlib.Path, manual_items: list[CommandSpec], results: list[dict]) -> None:
    lines = [
        "# Operator Checklist",
        "",
        "- Record board model, BME280 module, wiring, pull-ups, supply voltage, bus speed, and firmware commit.",
        "- Attach photos/video, logic analyzer capture, or meter readings when relevant.",
        "- Mark environmental plausibility checks manually; the runner cannot prove sensor accuracy alone.",
        "- Record `NOT_RUN`, `PASS`, `FAIL`, or `OPERATOR_REVIEW_REQUIRED` for each manual item.",
        "",
        "## Serial Results Requiring Operator Review",
        "",
    ]
    for row in results:
        if row["operator_result"] == RESULT_OPERATOR:
            lines.append(f"- [ ] `{row['group']}` / `{row['command']}`: {row['purpose']}")
    lines.extend(["", "## Skipped Or Manual Items", ""])
    for item in manual_items:
        lines.append(
            f"- [ ] `{item.group}` / `{item.command}`: {item.purpose} "
            f"({item.requires_opt_in or 'manual'}) - result:"
        )
        if item.notes:
            lines.append(f"  Notes: {item.notes}")
    lines.extend(
        [
            "",
            "## Sign-Off",
            "",
            "- Fixture protections for any fault work:",
            "- Environmental reference notes:",
            "- Operator sign-off:",
        ]
    )
    lines.append("")
    path.write_text("\n".join(lines), encoding="utf-8")


def sha256_file(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def package_version(name: str) -> str:
    try:
        return importlib.metadata.version(name)
    except importlib.metadata.PackageNotFoundError:
        return "not installed"


def write_manifest(path: pathlib.Path, summary: dict, artifact_files: dict[str, pathlib.Path]) -> None:
    hashes = {
        name: {
            "path": os.fspath(file_path),
            "sha256": sha256_file(file_path),
        }
        for name, file_path in artifact_files.items()
        if name != "manifest" and file_path.exists()
    }
    manifest = {
        "run_id": summary["run_id"],
        "datetime": summary["datetime"],
        "claim_boundary": summary["claim_boundary"],
        "runner_script": os.fspath(pathlib.Path(__file__).resolve()),
        "runner_script_sha256": sha256_file(pathlib.Path(__file__).resolve()),
        "python": sys.version.replace("\n", " "),
        "platform": platform.platform(),
        "pyserial_version": package_version("pyserial"),
        "branch": summary["branch"],
        "commit": summary["commit"],
        "worktree": summary["worktree"],
        "command_sequence": summary["command_sequence"],
        "opt_in_flags": summary["environment"]["opt_in_flags"],
        "artifacts": hashes,
    }
    path.write_text(json.dumps(manifest, indent=2), encoding="utf-8")


def format_runner_command(argv: list[str]) -> str:
    return "python tools/run_i2c_hil.py" + (f" {shlex.join(argv)}" if argv else "")


def write_transcript_header(
    transcript,
    args: argparse.Namespace,
    log_dir: pathlib.Path,
    *,
    runner_argv: list[str],
) -> None:
    transcript.write("# I2C HIL serial transcript\n")
    transcript.write(f"timestamp={timestamp()}\n")
    transcript.write(f"runner_command={format_runner_command(runner_argv)}\n")
    transcript.write(f"runner_args={json.dumps(runner_argv)}\n")
    transcript.write(f"port={args.port or 'DRY_RUN'}\n")
    transcript.write(f"baud={args.baud}\n")
    transcript.write(f"address={args.address}\n")
    transcript.write(f"log_dir={log_dir}\n")
    transcript.write(f"branch={git_value(['branch', '--show-current'])}\n")
    transcript.write(f"commit={git_value(['rev-parse', 'HEAD'])}\n")
    transcript.write(f"worktree={worktree_state()}\n")
    for key, value in environment_record(args).items():
        if isinstance(value, dict):
            transcript.write(f"{key}={json.dumps(value, sort_keys=True)}\n")
        else:
            transcript.write(f"{key}={value}\n")
    transcript.write("\n")
    transcript.flush()


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run BME280 serial I2C HIL self-test.")
    parser.add_argument("--port", help="Serial port, for example COM5 or /dev/ttyUSB0.")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate.")
    parser.add_argument("--out", default=str(DEFAULT_OUT), help="Base output directory.")
    parser.add_argument("--timeout", type=float, default=8.0, help="Default command timeout in seconds.")
    parser.add_argument("--dry-run", action="store_true", help="Write planned artifacts without opening serial.")
    parser.add_argument("--commands", help="Optional newline-delimited command file.")
    parser.add_argument("--address", type=parse_address, default="0x76", help="BME280 I2C address: 0x76 or 0x77.")
    parser.add_argument("--include-destructive", action="store_true", help="Include raw-write diagnostics.")
    parser.add_argument("--confirm-raw-write", default="", help=f"Required value for raw writes: {RAW_WRITE_CONFIRMATION}.")
    parser.add_argument("--include-soak", action="store_true", help="Include longer forced stress soak command.")
    parser.add_argument("--soak-count", type=parse_positive_int, default=500, help="Forced stress soak count used with --include-soak.")
    parser.add_argument("--include-normal-soak", action="store_true", help="Include opt-in normal-mode repeated-read soak.")
    parser.add_argument("--normal-soak-count", type=parse_positive_int, default=5, help="Normal-mode repeated-read count.")
    parser.add_argument("--normal-soak-interval-s", type=parse_nonnegative_float, default=1.0, help="Delay between opt-in normal-mode soak reads.")
    parser.add_argument("--include-fault-tests", action="store_true", help="Mark manual fault-test checklist items as intentionally requested.")
    parser.add_argument("--operator", default="", help="Operator name for evidence metadata.")
    parser.add_argument("--board", default="", help="MCU board model for evidence metadata.")
    parser.add_argument("--mcu-target", default="", help="MCU target, for example esp32s3 or esp32s2.")
    parser.add_argument("--framework", default="", help="Framework used by the flashed CLI firmware.")
    parser.add_argument("--build-target", default="", help="Build environment or target name.")
    parser.add_argument("--module", default="", help="BME280 module or sensor board model.")
    parser.add_argument("--vdd", default="", help="BME280 VDD rail value.")
    parser.add_argument("--vddio", default="", help="BME280 VDDIO rail value.")
    parser.add_argument("--pullups", default="", help="I2C pull-up values.")
    parser.add_argument("--pullup-location", default="", help="Pull-up location, for example on-module or external.")
    parser.add_argument("--sda-pin", default="", help="SDA pin used by the flashed firmware.")
    parser.add_argument("--scl-pin", default="", help="SCL pin used by the flashed firmware.")
    parser.add_argument("--bus-speed", default="", help="I2C bus speed used by the flashed firmware.")
    parser.add_argument("--sdo-state", default="", help="BME280 SDO strap state.")
    parser.add_argument("--csb-state", default="", help="BME280 CSB strap state.")
    parser.add_argument("--environment-ref", default="", help="Environmental reference instruments or artifact path.")
    parser.add_argument("--operator-notes", default="", help="Short operator note copied into generated artifacts.")
    return parser.parse_args(argv)


def execution_safety_errors(args: argparse.Namespace, executable: list[CommandSpec]) -> list[str]:
    errors: list[str] = []
    has_raw_write = any(spec.destructive or spec.command.strip().lower().startswith("wreg ") for spec in executable)
    if has_raw_write and not args.dry_run and args.confirm_raw_write != RAW_WRITE_CONFIRMATION:
        errors.append(f"raw-write execution requires --confirm-raw-write {RAW_WRITE_CONFIRMATION}")
    return errors


def main(argv: list[str] | None = None) -> int:
    runner_argv = list(sys.argv[1:] if argv is None else argv)
    args = parse_args(runner_argv)
    if not args.dry_run and not args.port:
        print("--port is required unless --dry-run is used.", file=sys.stderr)
        return 2

    try:
        executable, manual_items = build_command_sequence(args)
    except ValueError as exc:
        print(str(exc), file=sys.stderr)
        return 2

    safety_errors = execution_safety_errors(args, executable)
    if safety_errors:
        for error in safety_errors:
            print(error, file=sys.stderr)
        return 2

    out_base = pathlib.Path(args.out)
    if not out_base.is_absolute():
        out_base = ROOT / out_base
    log_dir = make_log_dir(out_base)

    transcript_path = log_dir / "serial_transcript.txt"
    summary_md_path = log_dir / "summary.md"
    timestamped_summary_md_path = log_dir / f"{log_dir.name}_summary.md"
    summary_json_path = log_dir / "summary.json"
    results_csv_path = log_dir / "results.csv"
    command_plan_path = log_dir / "command_plan.json"
    environment_path = log_dir / "environment.txt"
    checklist_path = log_dir / "operator_checklist.md"
    matrix_fragment_path = log_dir / "hardware_matrix_fragment.md"
    failure_analysis_path = log_dir / "failure_analysis.md"
    manifest_path = log_dir / "manifest.json"

    results: list[dict] = []
    with transcript_path.open("w", encoding="utf-8", newline="") as transcript:
        write_transcript_header(transcript, args, log_dir, runner_argv=runner_argv)
        if args.dry_run:
            transcript.write("DRY RUN: no serial port opened and no commands sent.\n")
            for spec in executable:
                transcript.write(f"DRY RUN COMMAND: [{spec.group}] {spec.command} -- {spec.purpose}\n")
                results.append(dry_run_result(spec))
        else:
            serial = import_serial()
            with serial.Serial(args.port, args.baud, timeout=0.1) as ser:
                time.sleep(1.5)
                boot = read_available(ser)
                if boot:
                    transcript.write("--- BOOT/INITIAL OUTPUT ---\n")
                    transcript.write(boot)
                    transcript.write("\n")
                for spec in executable:
                    timeout_s = spec.timeout_s if spec.timeout_s > 0 else args.timeout
                    results.append(run_serial_command(ser, spec, transcript, timeout_s=timeout_s))

    summary = {
        "run_id": log_dir.name,
        "datetime": timestamp(),
        "runner_command": format_runner_command(runner_argv),
        "runner_args": runner_argv,
        "branch": git_value(["branch", "--show-current"]),
        "commit": git_value(["rev-parse", "HEAD"]),
        "worktree": worktree_state(),
        "port": args.port or "DRY_RUN",
        "baud": args.baud,
        "address": args.address,
        "dry_run": args.dry_run,
        "claim_boundary": CLAIM_BOUNDARY,
        "environment": environment_record(args),
        "command_sequence": [spec.command for spec in executable],
        "manual_or_skipped": [dataclasses.asdict(item) for item in manual_items],
        "results": results,
        "final_verdict": final_verdict(results, dry_run=args.dry_run),
        "artifacts": {
            "serial_transcript": os.fspath(transcript_path),
            "summary_md": os.fspath(summary_md_path),
            "timestamped_summary_md": os.fspath(timestamped_summary_md_path),
            "summary_json": os.fspath(summary_json_path),
            "results_csv": os.fspath(results_csv_path),
            "command_plan": os.fspath(command_plan_path),
            "environment_txt": os.fspath(environment_path),
            "operator_checklist": os.fspath(checklist_path),
            "hardware_matrix_fragment": os.fspath(matrix_fragment_path),
            "failure_analysis": os.fspath(failure_analysis_path),
            "manifest": os.fspath(manifest_path),
        },
        "pyserial_install_hint": INSTALL_HINT,
    }
    write_command_plan(command_plan_path, executable, manual_items)
    write_results_csv(results_csv_path, results)
    write_environment(environment_path, summary)
    write_operator_checklist(checklist_path, manual_items, results)
    write_hardware_matrix_fragment(matrix_fragment_path, summary)
    write_failure_analysis(failure_analysis_path, summary)
    write_summary_md(summary_md_path, summary)
    write_summary_md(timestamped_summary_md_path, summary)
    summary_json_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    artifact_files = {
        "serial_transcript": transcript_path,
        "summary_md": summary_md_path,
        "timestamped_summary_md": timestamped_summary_md_path,
        "summary_json": summary_json_path,
        "results_csv": results_csv_path,
        "command_plan": command_plan_path,
        "environment_txt": environment_path,
        "operator_checklist": checklist_path,
        "hardware_matrix_fragment": matrix_fragment_path,
        "failure_analysis": failure_analysis_path,
        "manifest": manifest_path,
    }
    write_manifest(manifest_path, summary, artifact_files)

    print(f"HIL artifacts: {log_dir}")
    print(f"Summary: {summary_md_path}")
    print(f"Timestamped summary: {timestamped_summary_md_path}")
    print(f"Final verdict: {summary['final_verdict']}")
    print(f"Operator checklist: {checklist_path}")
    print(f"Manifest: {manifest_path}")
    if args.dry_run:
        print("Dry run only: no physical HIL validation was performed.")
    return 0 if summary["final_verdict"] != "FAIL" else 1


if __name__ == "__main__":
    raise SystemExit(main())
