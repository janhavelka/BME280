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
RESULT_UNKNOWN = "UNKNOWN"
RESULT_RESET_BUSY_RECOVERED = "PASS_WITH_RESET_BUSY_RECOVERED"
RESULT_NOT_RUN = "NOT_RUN"

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
STRESS_DURATION_RE = re.compile(r"\bDuration:\s*(\d+)\s*ms\b")
STRESS_RATE_RE = re.compile(r"\bRate:\s*([0-9]+(?:\.[0-9]+)?)\s*(samples|ops)/s\b")
STRESS_MIX_TOTAL_RE = re.compile(r"\bTotal:\s*ok=(\d+)\s+fail=(\d+)\b")
SELFTEST_RESULT_RE = re.compile(r"Selftest result:\s*pass=\s*(\d+)\s*fail=\s*(\d+)\s*skip=\s*(\d+)")
CONSECUTIVE_FAILURES_RE = re.compile(r"Consecutive failures:\s*(\d+)")
TOTAL_FAILURES_RE = re.compile(r"Total failures:\s*(\d+)")
CHIP_ID_RE = re.compile(r"Chip ID:\s*0x([0-9A-Fa-f]{1,2})|Reg\s+0xD0\s*=\s*0x([0-9A-Fa-f]{1,2})")
JOB_STATE_RE = re.compile(r"Job state:\s*([A-Z_]+)")
JOB_BOUNDARY_RE = re.compile(r"Boundary:\s*([A-Z_]+)")
JOB_ID_RE = re.compile(r"Job ID:\s*(\d+)")
JOB_KIND_RE = re.compile(r"Job kind:\s*([A-Z_]+)")
JOB_PHASE_RE = re.compile(r"Job phase:\s*([A-Z_]+)")
JOB_TERMINAL_RE = re.compile(r"Terminal state:\s*(true|false)", re.IGNORECASE)
JOB_STATUS_RE = re.compile(r"^Status:\s*([A-Z_]+)\s*\(code=(\d+),\s*detail=(-?\d+)\)", re.MULTILINE)
JOB_CONVERSION_RE = re.compile(r"Conversion state:\s*([A-Z_]+)")
JOB_DEADLINE_ACTIVE_RE = re.compile(r"Phase deadline active:\s*(true|false)", re.IGNORECASE)
JOB_DEADLINE_MS_RE = re.compile(r"Phase deadline ms:\s*(\d+)")
JOB_CALLBACKS_RE = re.compile(r"Callbacks used:\s*(\d+)")
JOB_INSTRUCTIONS_RE = re.compile(r"Instructions:\s*(\d+)")
DRIVER_READY_RE = re.compile(r"(?:Driver:\s*state=READY\b|State:\s*READY\b)")
DIRTY_FALSE_RE = re.compile(r"(?:dirty=false\b|Hardware config dirty:\s*(?:false|no)\b)", re.IGNORECASE)

VALIDATOR_CTRL_MEAS_SLEEP = "ctrl_meas_sleep"
VALIDATOR_STATUS_NOT_MEASURING = "status_not_measuring"
VALIDATOR_STATUS_IM_UPDATE_CLEAR = "status_im_update_clear"
VALIDATOR_STRESS_ZERO_ERRORS = "stress_zero_errors"
VALIDATOR_STRESS_MIX_ZERO_FAIL = "stress_mix_zero_fail"
VALIDATOR_SELFTEST_ZERO_FAIL = "selftest_zero_fail"
VALIDATOR_DRIVER_ZERO_CONSECUTIVE = "driver_zero_consecutive_failures"
VALIDATOR_JOB_DONE_OR_FAILED = "job_done_or_failed"
VALIDATOR_JOB_ZERO_CONSECUTIVE_FAILURES = "job_zero_consecutive_failures"
VALIDATOR_JOB_INSTRUCTION_BUDGET_RESPECTED = "job_instruction_budget_respected"
VALIDATOR_JOB_RESULT_FIELDS = "job_result_fields"
VALIDATOR_JOB_START_RUNNING = "job_start_running"
VALIDATOR_JOB_ONE_CALLBACK = "job_one_callback"
VALIDATOR_JOB_CANCEL_TIMED_OUT = "job_cancel_timed_out"
VALIDATOR_JOB_TIMED_OUT_RETRIEVAL = "job_timed_out_retrieval"
VALIDATOR_JOB_IDLE_NO_RESULT = "job_idle_no_result"
JOB_CLI_DEFAULT_BUDGET = 1


@dataclasses.dataclass(frozen=True)
class CommandSpec:
    command: str
    purpose: str
    group: str = "default"
    expected: tuple[str, ...] = ()
    expected_any: tuple[str, ...] = ()
    unknowns: tuple[str, ...] = ()
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
        expected_any=("Status: OK", "Status: BUSY"),
        unknowns=("Status: BUSY",),
        completion=("Status:",),
        timeout_s=8.0,
        notes=(
            "Soft reset is volatile and expected to be safe for BME280. "
            "A bounded BUSY result means NVM copy was still in progress and "
            "post-reset status/recover evidence must be reviewed."
        ),
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


CONFIG_MATRIX_COMMANDS: tuple[CommandSpec, ...] = (
    CommandSpec(
        command="mode sleep",
        purpose="Enter sleep before safe configuration boundary changes.",
        group="config-matrix",
        expected=("Status: OK",),
        timeout_s=5.0,
        requires_opt_in="--include-config-matrix",
    ),
    CommandSpec(
        command="osrs t 1",
        purpose="Set temperature oversampling to minimum enabled value.",
        group="config-matrix",
        expected=("Status: OK",),
        timeout_s=5.0,
        requires_opt_in="--include-config-matrix",
    ),
    CommandSpec(
        command="osrs p 1",
        purpose="Set pressure oversampling to minimum enabled value.",
        group="config-matrix",
        expected=("Status: OK",),
        timeout_s=5.0,
        requires_opt_in="--include-config-matrix",
    ),
    CommandSpec(
        command="osrs h 1",
        purpose="Set humidity oversampling to minimum enabled value.",
        group="config-matrix",
        expected=("Status: OK",),
        timeout_s=5.0,
        requires_opt_in="--include-config-matrix",
    ),
    CommandSpec(
        command="filter 0",
        purpose="Set IIR filter boundary value OFF.",
        group="config-matrix",
        expected=("Status: OK",),
        timeout_s=5.0,
        requires_opt_in="--include-config-matrix",
    ),
    CommandSpec(
        command="standby 0",
        purpose="Set standby boundary value 0.5 ms.",
        group="config-matrix",
        expected=("Status: OK",),
        timeout_s=5.0,
        requires_opt_in="--include-config-matrix",
    ),
    CommandSpec(
        command="cfg",
        purpose="Capture register/cache settings after minimum boundary configuration.",
        group="config-matrix",
        expected=("ctrl_hum", "ctrl_meas", "config", "Hardware config dirty:"),
        completion=("Hardware config dirty:",),
        timeout_s=10.0,
        requires_opt_in="--include-config-matrix",
    ),
    CommandSpec(
        command="osrs t 5",
        purpose="Set temperature oversampling to maximum BME280 value X16.",
        group="config-matrix",
        expected=("Status: OK",),
        timeout_s=5.0,
        requires_opt_in="--include-config-matrix",
    ),
    CommandSpec(
        command="osrs p 5",
        purpose="Set pressure oversampling to maximum BME280 value X16.",
        group="config-matrix",
        expected=("Status: OK",),
        timeout_s=5.0,
        requires_opt_in="--include-config-matrix",
    ),
    CommandSpec(
        command="osrs h 5",
        purpose="Set humidity oversampling to maximum BME280 value X16.",
        group="config-matrix",
        expected=("Status: OK",),
        timeout_s=5.0,
        requires_opt_in="--include-config-matrix",
    ),
    CommandSpec(
        command="filter 4",
        purpose="Set IIR filter maximum value X16.",
        group="config-matrix",
        expected=("Status: OK",),
        timeout_s=5.0,
        requires_opt_in="--include-config-matrix",
    ),
    CommandSpec(
        command="standby 7",
        purpose="Set standby boundary value 20 ms.",
        group="config-matrix",
        expected=("Status: OK",),
        timeout_s=5.0,
        requires_opt_in="--include-config-matrix",
    ),
    CommandSpec(
        command="timing",
        purpose="Capture estimated measurement timing at maximum oversampling boundary.",
        group="config-matrix",
        expected=("Estimated measurement time", "Estimated normal cycle"),
        timeout_s=5.0,
        requires_opt_in="--include-config-matrix",
    ),
    CommandSpec(
        command="force",
        purpose="Run one forced measurement at maximum oversampling boundary.",
        group="config-matrix",
        expected=("Temp:", "Pressure:", "Humidity:"),
        timeout_s=15.0,
        operator_check=True,
        requires_opt_in="--include-config-matrix",
        notes="Operator must judge environmental plausibility.",
    ),
    CommandSpec(
        command="raw",
        purpose="Capture raw cached ADC after maximum oversampling boundary measurement.",
        group="config-matrix",
        expected=("Raw ADC", "Valid channels: T=", "Cached sample age"),
        timeout_s=5.0,
        requires_opt_in="--include-config-matrix",
    ),
    CommandSpec(
        command="comp",
        purpose="Capture fixed-point compensated sample after maximum oversampling boundary measurement.",
        group="config-matrix",
        expected=("Compensated", "Valid channels: T="),
        timeout_s=5.0,
        requires_opt_in="--include-config-matrix",
    ),
    CommandSpec(
        command="mode sleep",
        purpose="Return to sleep before restoring default example configuration.",
        group="config-matrix",
        expected=("Status: OK",),
        timeout_s=5.0,
        requires_opt_in="--include-config-matrix",
    ),
    CommandSpec(
        command="osrs t 1",
        purpose="Restore default temperature oversampling.",
        group="config-matrix",
        expected=("Status: OK",),
        timeout_s=5.0,
        requires_opt_in="--include-config-matrix",
    ),
    CommandSpec(
        command="osrs p 1",
        purpose="Restore default pressure oversampling.",
        group="config-matrix",
        expected=("Status: OK",),
        timeout_s=5.0,
        requires_opt_in="--include-config-matrix",
    ),
    CommandSpec(
        command="osrs h 1",
        purpose="Restore default humidity oversampling.",
        group="config-matrix",
        expected=("Status: OK",),
        timeout_s=5.0,
        requires_opt_in="--include-config-matrix",
    ),
    CommandSpec(
        command="filter 0",
        purpose="Restore default filter setting.",
        group="config-matrix",
        expected=("Status: OK",),
        timeout_s=5.0,
        requires_opt_in="--include-config-matrix",
    ),
    CommandSpec(
        command="standby 2",
        purpose="Restore default 125 ms standby setting.",
        group="config-matrix",
        expected=("Status: OK",),
        timeout_s=5.0,
        requires_opt_in="--include-config-matrix",
    ),
    CommandSpec(
        command="cfg",
        purpose="Capture restored register/cache settings after boundary matrix.",
        group="config-matrix",
        expected=("ctrl_hum", "ctrl_meas", "config", "Hardware config dirty:"),
        completion=("Hardware config dirty:",),
        timeout_s=10.0,
        requires_opt_in="--include-config-matrix",
    ),
)


INVALID_INPUT_COMMANDS: tuple[CommandSpec, ...] = (
    CommandSpec(
        command="not_a_command",
        purpose="Verify unknown CLI command is visible and bounded.",
        group="invalid-input",
        expected=("Unknown command:",),
        timeout_s=3.0,
        requires_opt_in="--include-invalid-inputs",
    ),
    CommandSpec(
        command="addr 0x75",
        purpose="Verify invalid BME280 address is rejected by the example CLI.",
        group="invalid-input",
        expected=("Usage: addr 0x76|0x77",),
        timeout_s=3.0,
        requires_opt_in="--include-invalid-inputs",
    ),
    CommandSpec(
        command="mode invalid",
        purpose="Verify invalid mode token is rejected by the example CLI.",
        group="invalid-input",
        expected=("Invalid mode:",),
        timeout_s=3.0,
        requires_opt_in="--include-invalid-inputs",
    ),
    CommandSpec(
        command="osrs t 6",
        purpose="Verify oversampling upper-bound validation.",
        group="invalid-input",
        expected=("Invalid oversampling value",),
        timeout_s=3.0,
        requires_opt_in="--include-invalid-inputs",
    ),
    CommandSpec(
        command="filter 5",
        purpose="Verify filter upper-bound validation.",
        group="invalid-input",
        expected=("Invalid filter value",),
        timeout_s=3.0,
        requires_opt_in="--include-invalid-inputs",
    ),
    CommandSpec(
        command="standby 8",
        purpose="Verify standby upper-bound validation.",
        group="invalid-input",
        expected=("Invalid standby value",),
        timeout_s=3.0,
        requires_opt_in="--include-invalid-inputs",
    ),
    CommandSpec(
        command="reg 0x100",
        purpose="Verify register address upper-bound validation.",
        group="invalid-input",
        expected=("Usage: reg <addr>",),
        timeout_s=3.0,
        requires_opt_in="--include-invalid-inputs",
    ),
    CommandSpec(
        command="wreg 0xF4",
        purpose="Verify incomplete raw-write command is rejected without touching hardware.",
        group="invalid-input",
        expected=("Usage: wreg <addr> <val>",),
        timeout_s=3.0,
        requires_opt_in="--include-invalid-inputs",
    ),
)


BENCHMARK_COMMANDS: tuple[CommandSpec, ...] = (
    CommandSpec(
        command="stress 50",
        purpose="Quick forced-measurement sampling benchmark.",
        group="benchmark",
        expected=("Stress Summary", "Errors:", "Rate:"),
        completion=("Health delta:",),
        validators=(VALIDATOR_STRESS_ZERO_ERRORS,),
        timeout_s=60.0,
        operator_check=True,
        requires_opt_in="--include-benchmarks",
    ),
    CommandSpec(
        command="stress 500",
        purpose="Extended forced-measurement sampling benchmark.",
        group="benchmark",
        expected=("Stress Summary", "Errors:", "Rate:"),
        completion=("Health delta:",),
        validators=(VALIDATOR_STRESS_ZERO_ERRORS,),
        timeout_s=300.0,
        operator_check=True,
        requires_opt_in="--include-benchmarks",
    ),
    CommandSpec(
        command="stress_mix 140",
        purpose="Mixed-operation benchmark covering reads, raw calibration, modes, filter, and standby.",
        group="benchmark",
        expected=("stress_mix summary", "Total:", "Health delta:"),
        completion=("Health delta:",),
        validators=(VALIDATOR_STRESS_MIX_ZERO_FAIL,),
        timeout_s=180.0,
        operator_check=True,
        requires_opt_in="--include-benchmarks",
    ),
    CommandSpec(
        command="drv",
        purpose="Capture driver health after benchmark commands.",
        group="benchmark",
        expected=("Driver Health", "Total"),
        validators=(VALIDATOR_DRIVER_ZERO_CONSECUTIVE,),
        timeout_s=5.0,
        requires_opt_in="--include-benchmarks",
    ),
)


JOB_API_COMMANDS: tuple[CommandSpec, ...] = (
    CommandSpec(
        command="job status",
        purpose="Capture staged-job baseline status.",
        group="job-api",
        expected=("Job Status", "Boundary: SNAPSHOT", "Job kind:", "Job state:", "Callbacks used:", "Driver:"),
        validators=(VALIDATOR_JOB_RESULT_FIELDS,),
        timeout_s=5.0,
        requires_opt_in="--include-job-api",
    ),
    CommandSpec(
        command="job start init",
        purpose="Start staged initialization without issuing an I2C callback.",
        group="job-api",
        expected=("Boundary: START", "Job kind: INIT", "Job state: RUNNING", "Status: IN_PROGRESS", "Callbacks used: 0"),
        validators=(VALIDATOR_JOB_RESULT_FIELDS, VALIDATOR_JOB_START_RUNNING),
        timeout_s=5.0,
        requires_opt_in="--include-job-api",
    ),
    CommandSpec(
        command="job poll 1",
        purpose="Advance the staged init job with exactly one transport callback.",
        group="job-api",
        expected=("Boundary: POLL", "Job kind: INIT", "Job state: RUNNING", "Status: IN_PROGRESS", "Callbacks used: 1"),
        validators=(
            VALIDATOR_JOB_RESULT_FIELDS,
            VALIDATOR_JOB_ONE_CALLBACK,
            VALIDATOR_JOB_INSTRUCTION_BUDGET_RESPECTED,
        ),
        timeout_s=5.0,
        requires_opt_in="--include-job-api",
    ),
    CommandSpec(
        command="job cancel deadline",
        purpose="Cancel the active job at an owner deadline without issuing I2C.",
        group="job-api",
        expected=("Boundary: CANCEL", "Job kind: INIT", "Job state: TIMED_OUT", "Status: DEADLINE_EXPIRED", "Callbacks used: 0"),
        validators=(VALIDATOR_JOB_RESULT_FIELDS, VALIDATOR_JOB_CANCEL_TIMED_OUT),
        timeout_s=5.0,
        requires_opt_in="--include-job-api",
    ),
    CommandSpec(
        command="job poll 0",
        purpose="Retrieve the retained timed-out terminal result with zero I2C callbacks.",
        group="job-api",
        expected=("Boundary: POLL", "Job kind: INIT", "Job state: TIMED_OUT", "Status: DEADLINE_EXPIRED", "Callbacks used: 0"),
        validators=(
            VALIDATOR_JOB_RESULT_FIELDS,
            VALIDATOR_JOB_TIMED_OUT_RETRIEVAL,
            VALIDATOR_JOB_INSTRUCTION_BUDGET_RESPECTED,
        ),
        timeout_s=5.0,
        requires_opt_in="--include-job-api",
    ),
    CommandSpec(
        command="job poll 0",
        purpose="Prove the retained cancellation terminal is delivered exactly once.",
        group="job-api",
        expected=("Boundary: POLL", "Job ID: 0", "Job kind: NONE", "Job state: IDLE", "Status: OK", "Callbacks used: 0"),
        validators=(
            VALIDATOR_JOB_RESULT_FIELDS,
            VALIDATOR_JOB_IDLE_NO_RESULT,
            VALIDATOR_JOB_INSTRUCTION_BUDGET_RESPECTED,
        ),
        timeout_s=5.0,
        requires_opt_in="--include-job-api",
    ),
    CommandSpec(
        command="job init 1",
        purpose="Run staged initialization to a natural terminal with one callback per poll.",
        group="job-api",
        expected=("Boundary: POLL", "Job state: DONE", "Status: OK", "Driver: READY"),
        validators=(VALIDATOR_JOB_RESULT_FIELDS, VALIDATOR_JOB_DONE_OR_FAILED, VALIDATOR_JOB_INSTRUCTION_BUDGET_RESPECTED),
        timeout_s=25.0,
        requires_opt_in="--include-job-api",
    ),
    CommandSpec(
        command="job apply 1",
        purpose="Run staged cached-config apply with one callback per poll.",
        group="job-api",
        expected=("Boundary: POLL", "Job state: DONE", "Status: OK"),
        validators=(VALIDATOR_JOB_RESULT_FIELDS, VALIDATOR_JOB_DONE_OR_FAILED, VALIDATOR_JOB_INSTRUCTION_BUDGET_RESPECTED),
        timeout_s=25.0,
        requires_opt_in="--include-job-api",
    ),
    CommandSpec(
        command="job force 1",
        purpose="Run staged forced measurement with one callback per poll.",
        group="job-api",
        expected=("Boundary: POLL", "Job state: DONE", "Status: OK"),
        validators=(VALIDATOR_JOB_RESULT_FIELDS, VALIDATOR_JOB_DONE_OR_FAILED, VALIDATOR_JOB_INSTRUCTION_BUDGET_RESPECTED),
        timeout_s=25.0,
        requires_opt_in="--include-job-api",
    ),
    CommandSpec(
        command="raw",
        purpose="Capture raw sample produced by staged forced-measurement job.",
        group="job-api",
        expected=("Raw ADC", "Valid channels: T=", "Cached sample age"),
        timeout_s=5.0,
        requires_opt_in="--include-job-api",
    ),
    CommandSpec(
        command="comp",
        purpose="Capture compensated sample produced by staged forced-measurement job.",
        group="job-api",
        expected=("Compensated", "Valid channels: T="),
        timeout_s=5.0,
        requires_opt_in="--include-job-api",
    ),
    CommandSpec(
        command="job resync 1",
        purpose="Run explicit non-reset resynchronization with one callback per poll.",
        group="job-api",
        expected=("Boundary: POLL", "Job kind: RESYNC", "Job state: DONE", "Status: OK", "Driver: READY"),
        validators=(
            VALIDATOR_JOB_RESULT_FIELDS,
            VALIDATOR_JOB_DONE_OR_FAILED,
            VALIDATOR_JOB_ZERO_CONSECUTIVE_FAILURES,
            VALIDATOR_JOB_INSTRUCTION_BUDGET_RESPECTED,
        ),
        timeout_s=25.0,
        requires_opt_in="--include-job-api",
    ),
    CommandSpec(
        command="cfg",
        purpose="Capture settings after non-reset staged resynchronization.",
        group="job-api",
        expected=("ctrl_hum", "ctrl_meas", "config", "Hardware config dirty:"),
        completion=("Hardware config dirty:",),
        timeout_s=10.0,
        requires_opt_in="--include-job-api",
    ),
    CommandSpec(
        command="status",
        purpose="Capture status and dirty-state after non-reset staged resynchronization.",
        group="job-api",
        expected=("Status: 0x", "Driver:", "dirty=false"),
        validators=(VALIDATOR_STATUS_NOT_MEASURING,),
        timeout_s=5.0,
        requires_opt_in="--include-job-api",
    ),
    CommandSpec(
        command="job reset 1",
        purpose="Run the separately named explicit soft-reset staged job.",
        group="job-api",
        expected=("Boundary: POLL", "Job kind: SOFT_RESET", "Job state: DONE", "Status: OK", "Driver: READY"),
        validators=(
            VALIDATOR_JOB_RESULT_FIELDS,
            VALIDATOR_JOB_DONE_OR_FAILED,
            VALIDATOR_JOB_ZERO_CONSECUTIVE_FAILURES,
            VALIDATOR_JOB_INSTRUCTION_BUDGET_RESPECTED,
        ),
        timeout_s=25.0,
        requires_opt_in="--include-job-api",
    ),
    CommandSpec(
        command="job force 3",
        purpose="Run staged forced measurement with multi-instruction poll budget.",
        group="job-api",
        expected=("Boundary: POLL", "Job state: DONE", "Status: OK"),
        validators=(VALIDATOR_JOB_RESULT_FIELDS, VALIDATOR_JOB_DONE_OR_FAILED, VALIDATOR_JOB_INSTRUCTION_BUDGET_RESPECTED),
        timeout_s=25.0,
        requires_opt_in="--include-job-api",
    ),
    CommandSpec(
        command="drv",
        purpose="Capture driver health after staged-job API checks.",
        group="job-api",
        expected=("Driver Health", "Total"),
        validators=(VALIDATOR_DRIVER_ZERO_CONSECUTIVE,),
        timeout_s=5.0,
        requires_opt_in="--include-job-api",
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


def parse_nonnegative_int(value: str) -> int:
    try:
        parsed = int(value, 10)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("value must be a non-negative integer") from exc
    if parsed < 0:
        raise argparse.ArgumentTypeError("value must be a non-negative integer")
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


def counted_stress_command(command: str) -> tuple[str, int] | None:
    parts = command.strip().split()
    if len(parts) != 2 or parts[0].lower() not in ("stress", "stress_mix"):
        return None
    try:
        return parts[0].lower(), int(parts[1], 10)
    except ValueError:
        return None


def job_command_budget(command: str) -> int:
    parts = command.strip().split()
    if len(parts) >= 3 and parts[0].lower() == "job":
        try:
            return int(parts[2], 10)
        except ValueError:
            return JOB_CLI_DEFAULT_BUDGET
    return JOB_CLI_DEFAULT_BUDGET


def custom_command_safety_errors(commands: list[CommandSpec], args: argparse.Namespace) -> list[str]:
    errors: list[str] = []
    include_destructive = getattr(args, "include_destructive", False)
    include_soak = getattr(args, "include_soak", False)
    for spec in commands:
        command = spec.command.strip().lower()
        if command.startswith("wreg ") and not include_destructive:
            errors.append(f"`{spec.command}` requires --include-destructive")
        counted = counted_stress_command(command)
        if counted is not None and counted[1] > 100 and not include_soak:
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

    if getattr(args, "include_config_matrix", False):
        executable.extend(cmd.formatted(address=args.address) for cmd in CONFIG_MATRIX_COMMANDS)
    else:
        checklist.extend(cmd.formatted(address=args.address) for cmd in CONFIG_MATRIX_COMMANDS)

    if getattr(args, "include_invalid_inputs", False):
        executable.extend(cmd.formatted(address=args.address) for cmd in INVALID_INPUT_COMMANDS)
    else:
        checklist.extend(cmd.formatted(address=args.address) for cmd in INVALID_INPUT_COMMANDS)

    include_benchmarks = getattr(args, "include_benchmarks", False) or getattr(
        args, "sample_rate_benchmark", False
    )
    if include_benchmarks:
        executable.extend(cmd.formatted(address=args.address) for cmd in BENCHMARK_COMMANDS)
    else:
        checklist.extend(cmd.formatted(address=args.address) for cmd in BENCHMARK_COMMANDS)

    if getattr(args, "include_job_api", False):
        executable.extend(cmd.formatted(address=args.address) for cmd in JOB_API_COMMANDS)
    else:
        checklist.extend(cmd.formatted(address=args.address) for cmd in JOB_API_COMMANDS)

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
    if match := STRESS_DURATION_RE.search(output):
        evidence["duration_ms"] = int(match.group(1))
    if match := STRESS_RATE_RE.search(output):
        evidence["rate"] = float(match.group(1))
        evidence["rate_units"] = f"{match.group(2)}/s"
    if match := STRESS_MIX_TOTAL_RE.search(output):
        evidence["stress_mix_ok"] = int(match.group(1))
        evidence["stress_mix_fail"] = int(match.group(2))
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
    if match := JOB_STATE_RE.search(output):
        evidence["job_state"] = match.group(1)
    if match := JOB_BOUNDARY_RE.search(output):
        evidence["job_boundary"] = match.group(1)
    if match := JOB_ID_RE.search(output):
        evidence["job_id"] = int(match.group(1))
    if match := JOB_KIND_RE.search(output):
        evidence["job_kind"] = match.group(1)
    if match := JOB_PHASE_RE.search(output):
        evidence["job_phase"] = match.group(1)
    if match := JOB_TERMINAL_RE.search(output):
        evidence["job_terminal"] = match.group(1).lower() == "true"
    if match := JOB_STATUS_RE.search(output):
        evidence["job_status"] = match.group(1)
        evidence["job_status_code"] = int(match.group(2))
        evidence["job_status_detail"] = int(match.group(3))
    if match := JOB_CONVERSION_RE.search(output):
        evidence["job_conversion_state"] = match.group(1)
    if match := JOB_DEADLINE_ACTIVE_RE.search(output):
        evidence["job_phase_deadline_active"] = match.group(1).lower() == "true"
    if match := JOB_DEADLINE_MS_RE.search(output):
        evidence["job_phase_deadline_ms"] = int(match.group(1))
    if match := JOB_CALLBACKS_RE.search(output):
        evidence["job_callbacks"] = int(match.group(1))
    if match := JOB_INSTRUCTIONS_RE.search(output):
        evidence["job_instructions"] = int(match.group(1))
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
        elif validator == VALIDATOR_STRESS_MIX_ZERO_FAIL:
            failures = evidence.get("stress_mix_fail")
            if failures is None:
                return RESULT_REVIEW, "stress_mix fail count was not parsed from serial output."
            if failures != 0:
                return RESULT_FAIL, f"stress_mix reported fail={failures}."
            reasons.append("stress_mix fail=0")
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
        elif validator == VALIDATOR_JOB_DONE_OR_FAILED:
            state = evidence.get("job_state")
            if state is None:
                return RESULT_REVIEW, "job state was not parsed from serial output."
            if state not in ("DONE", "FAILED"):
                return RESULT_FAIL, f"job state is {state}, expected DONE or FAILED."
            reasons.append(f"job state={state}")
        elif validator == VALIDATOR_JOB_ZERO_CONSECUTIVE_FAILURES:
            failures = evidence.get("consecutive_failures")
            if failures is None:
                return RESULT_REVIEW, "job consecutive failure count was not parsed from serial output."
            if failures != 0:
                return RESULT_FAIL, f"job consecutive failures={failures}, expected 0."
            reasons.append("job consecutive failures=0")
        elif validator == VALIDATOR_JOB_INSTRUCTION_BUDGET_RESPECTED:
            callbacks = evidence.get("job_callbacks")
            if callbacks is None:
                return RESULT_REVIEW, "job callback count was not parsed from serial output."
            budget = job_command_budget(spec.command)
            if callbacks > budget:
                return RESULT_FAIL, f"job used {callbacks} callback(s), budget was {budget}."
            reasons.append(f"job callbacks {callbacks}<={budget}")
        elif validator == VALIDATOR_JOB_RESULT_FIELDS:
            required = (
                "job_boundary",
                "job_id",
                "job_kind",
                "job_phase",
                "job_state",
                "job_terminal",
                "job_status",
                "job_status_code",
                "job_status_detail",
                "job_conversion_state",
                "job_phase_deadline_active",
                "job_phase_deadline_ms",
                "job_callbacks",
                "job_instructions",
            )
            missing = [name for name in required if name not in evidence]
            if missing:
                return RESULT_REVIEW, f"job result fields were not parsed: {', '.join(missing)}."
            if evidence["job_callbacks"] != evidence["job_instructions"]:
                return RESULT_FAIL, "job callback count differs from compatibility instruction count."
            terminal_states = {"DONE", "FAILED", "CANCELLED", "TIMED_OUT"}
            if evidence["job_terminal"] != (evidence["job_state"] in terminal_states):
                return RESULT_FAIL, "job terminal flag does not match the reported job state."
            if (not evidence["job_phase_deadline_active"] and
                    evidence["job_phase_deadline_ms"] != 0):
                return RESULT_FAIL, "inactive job phase deadline has a nonzero value."
            reasons.append("job result fields complete and internally consistent")
        elif validator == VALIDATOR_JOB_START_RUNNING:
            if (evidence.get("job_boundary") != "START" or
                    evidence.get("job_id", 0) == 0 or
                    evidence.get("job_state") != "RUNNING" or
                    evidence.get("job_status") != "IN_PROGRESS" or
                    evidence.get("job_callbacks") != 0):
                return RESULT_FAIL, "job start boundary was not a zero-callback running result."
            reasons.append("job start is running with zero callbacks")
        elif validator == VALIDATOR_JOB_ONE_CALLBACK:
            if evidence.get("job_boundary") != "POLL" or evidence.get("job_callbacks") != 1:
                return RESULT_FAIL, "job poll did not report exactly one callback."
            reasons.append("job poll used exactly one callback")
        elif validator == VALIDATOR_JOB_CANCEL_TIMED_OUT:
            if (evidence.get("job_boundary") != "CANCEL" or
                    evidence.get("job_id", 0) == 0 or
                    evidence.get("job_state") != "TIMED_OUT" or
                    evidence.get("job_status") != "DEADLINE_EXPIRED" or
                    evidence.get("job_callbacks") != 0):
                return RESULT_FAIL, "deadline cancellation boundary was not a zero-callback timed-out result."
            reasons.append("deadline cancellation is terminal with zero callbacks")
        elif validator == VALIDATOR_JOB_TIMED_OUT_RETRIEVAL:
            if (evidence.get("job_boundary") != "POLL" or
                    evidence.get("job_id", 0) == 0 or
                    evidence.get("job_state") != "TIMED_OUT" or
                    evidence.get("job_status") != "DEADLINE_EXPIRED" or
                    evidence.get("job_callbacks") != 0):
                return RESULT_FAIL, "zero-budget poll did not retrieve the timed-out terminal result."
            reasons.append("zero-budget poll retrieved timed-out terminal")
        elif validator == VALIDATOR_JOB_IDLE_NO_RESULT:
            if (evidence.get("job_boundary") != "POLL" or
                    evidence.get("job_id") != 0 or
                    evidence.get("job_kind") != "NONE" or
                    evidence.get("job_phase") != "NONE" or
                    evidence.get("job_state") != "IDLE" or
                    evidence.get("job_status") != "OK" or
                    evidence.get("job_callbacks") != 0):
                return RESULT_FAIL, "second zero-budget poll did not report an empty idle result."
            reasons.append("cancellation terminal was delivered exactly once")
        else:
            return RESULT_REVIEW, f"unknown runner validator: {validator}"
    if reasons:
        return None, "; ".join(reasons)
    return None, ""


def classify_output(spec: CommandSpec, output: str, completion: str) -> tuple[str, str]:
    clean = strip_ansi(output)
    if completion == RESULT_TIMEOUT:
        return RESULT_TIMEOUT, "Command timed out before serial idle/completion."
    for unknown in spec.unknowns:
        if unknown and unknown in clean:
            return RESULT_UNKNOWN, f"Matched command unknown/incomplete token: {unknown}"
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


def row_output(row: dict) -> str:
    return strip_ansi(str(row.get("output_excerpt", "")))


def row_is_pass(row: dict, command: str | None = None) -> bool:
    if command is not None and row.get("command") != command:
        return False
    return row.get("serial_result") == RESULT_PASS


def reclassify_job_api_correlation(results: list[dict]) -> int:
    """Require one identity and one exact terminal across the cancel fixture."""
    commands = (
        "job start init",
        "job poll 1",
        "job cancel deadline",
        "job poll 0",
        "job poll 0",
    )
    terminal_fields = (
        "job_id",
        "job_kind",
        "job_phase",
        "job_state",
        "job_terminal",
        "job_status",
        "job_status_code",
        "job_status_detail",
        "job_conversion_state",
        "job_phase_deadline_active",
        "job_phase_deadline_ms",
        "job_callbacks",
        "job_instructions",
    )
    failures = 0
    for index in range(0, len(results) - len(commands) + 1):
        rows = results[index:index + len(commands)]
        if tuple(row.get("command") for row in rows) != commands:
            continue
        if any(row.get("group") != "job-api" for row in rows):
            continue
        if any(not row_is_pass(row) for row in rows):
            continue

        evidence = [row.get("parsed_evidence", {}) for row in rows]
        active_ids = [item.get("job_id") for item in evidence[:4]]
        active_kinds = [item.get("job_kind") for item in evidence[:4]]
        failure_reason = ""
        failure_index = 3
        if active_ids[0] in (None, 0) or len(set(active_ids)) != 1:
            failure_reason = (
                "Job API correlation failed: start, poll, cancel, and terminal "
                f"retrieval IDs differ ({active_ids})."
            )
        elif len(set(active_kinds)) != 1:
            failure_reason = (
                "Job API correlation failed: active result kinds differ "
                f"({active_kinds})."
            )
        else:
            cancel_evidence = evidence[2]
            retrieval_evidence = evidence[3]
            mismatched = [
                field for field in terminal_fields
                if cancel_evidence.get(field) != retrieval_evidence.get(field)
            ]
            if mismatched:
                failure_reason = (
                    "Job API correlation failed: retained terminal differs from "
                    f"the cancellation result in {', '.join(mismatched)}."
                )
        if not failure_reason and evidence[4].get("job_id") != 0:
            failure_reason = (
                "Job API correlation failed: the second zero-budget retrieval "
                "did not return the empty job identity."
            )
            failure_index = 4

        if failure_reason:
            row = rows[failure_index]
            row["serial_result"] = RESULT_FAIL
            row["classification_reason"] = failure_reason
            failures += 1
    return failures


def row_is_reset_nvm_busy(row: dict) -> bool:
    clean = row_output(row)
    return (
        row.get("command") == "reset"
        and row.get("serial_result") == RESULT_UNKNOWN
        and "Status: BUSY" in clean
        and "NVM update in progress" in clean
    )


def row_proves_ready_clean_status(row: dict) -> bool:
    if not row_is_pass(row, "status"):
        return False
    evidence = row.get("parsed_evidence", {})
    if evidence.get("im_update") != 0 or evidence.get("measuring") != 0:
        return False
    clean = row_output(row)
    return bool(DRIVER_READY_RE.search(clean) and DIRTY_FALSE_RE.search(clean))


def row_proves_readable_config(row: dict) -> bool:
    if not row_is_pass(row, "cfg"):
        return False
    clean = row_output(row)
    return "ctrl_hum" in clean and "ctrl_meas" in clean and "config" in clean


def row_proves_recover_ok(row: dict) -> bool:
    if not row_is_pass(row, "recover"):
        return False
    return "Status: OK" in row_output(row)


def reset_busy_recovery_proven(results: list[dict], index: int) -> bool:
    row = results[index]
    group = row.get("group")
    if group == "reset-recover":
        if index + 4 >= len(results):
            return False
        post_status, recover, cfg, final_status = results[index + 1:index + 5]
        return (
            row_is_pass(post_status, "status")
            and post_status.get("parsed_evidence", {}).get("im_update") == 0
            and row_proves_recover_ok(recover)
            and row_proves_readable_config(cfg)
            and row_proves_ready_clean_status(final_status)
        )
    if group == "soak-duration":
        if index + 2 >= len(results):
            return False
        recover, status = results[index + 1:index + 3]
        return row_proves_recover_ok(recover) and row_proves_ready_clean_status(status)
    return False


def reclassify_recovered_reset_busy(results: list[dict]) -> int:
    count = 0
    for index, row in enumerate(results):
        if row_is_reset_nvm_busy(row) and reset_busy_recovery_proven(results, index):
            row["serial_result"] = RESULT_RESET_BUSY_RECOVERED
            row["classification_reason"] = (
                "Reset returned BUSY while BME280 NVM copy was in progress; "
                "immediate follow-up recovery/status evidence proved READY, "
                "dirty=false, im_update=0, and measuring=0."
            )
            row["parsed_evidence"] = {
                **row.get("parsed_evidence", {}),
                "reset_busy_recovered": True,
            }
            count += 1
    return count


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


def open_serial_with_retries(serial_module, args: argparse.Namespace):
    attempts = 1 + int(getattr(args, "reconnect_attempts", 0))
    last_exc: Exception | None = None
    for attempt in range(1, attempts + 1):
        try:
            return serial_module.Serial(args.port, args.baud, timeout=0.1)
        except Exception as exc:  # pyserial exposes several platform-specific exceptions
            last_exc = exc
            if attempt >= attempts:
                break
            time.sleep(float(getattr(args, "reconnect_delay_s", 1.0)))
    assert last_exc is not None
    raise last_exc


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


def run_serial_command(
    ser,
    spec: CommandSpec,
    transcript,
    *,
    timeout_s: float,
    idle_after_output_s: float = 0.75,
    idle_after_match_s: float = 0.25,
    command_pacing_s: float = 0.0,
    verbose: bool = False,
) -> dict:
    start = time.monotonic()
    deadline = start + timeout_s
    last_rx = time.monotonic()
    saw_output = False
    matched_expected = False
    matched_failure = False
    chunks: list[str] = []

    boundary = f"\n--- COMMAND {spec.command!r} @ {timestamp()} ---\n"
    transcript.write(boundary)
    transcript.flush()
    if verbose:
        print(boundary, end="")

    if spec.pre_delay_s > 0.0:
        transcript.write(f"PRE_DELAY {spec.pre_delay_s:.3f}s before command\n")
        transcript.flush()
        if verbose:
            print(f"PRE_DELAY {spec.pre_delay_s:.3f}s before command")
        time.sleep(spec.pre_delay_s)
    if command_pacing_s > 0.0:
        transcript.write(f"COMMAND_PACING {command_pacing_s:.3f}s before command\n")
        transcript.flush()
        if verbose:
            print(f"COMMAND_PACING {command_pacing_s:.3f}s before command")
        time.sleep(command_pacing_s)

    ser.write((spec.command + "\n").encode("utf-8"))
    ser.flush()

    while True:
        chunk = read_available(ser)
        if chunk:
            saw_output = True
            chunks.append(chunk)
            transcript.write(chunk)
            transcript.flush()
            if verbose:
                print(chunk, end="", flush=True)
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
    result_line = (
        f"\n--- RESULT {result} completion={completion} "
        f"elapsed_s={elapsed:.3f} reason={reason} ---\n"
    )
    transcript.write(result_line)
    transcript.flush()
    if verbose:
        print(result_line, end="")
    return {
        "command": spec.command,
        "purpose": spec.purpose,
        "group": spec.group,
        "expected": list(spec.expected),
        "expected_any": list(spec.expected_any),
        "unknowns": list(spec.unknowns),
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
        "unknowns": list(spec.unknowns),
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


def session_error_result(message: str) -> dict:
    return {
        "command": "SERIAL_SESSION",
        "purpose": "Open and maintain the serial HIL session.",
        "group": "serial",
        "expected": [],
        "expected_any": [],
        "unknowns": [],
        "validators": [],
        "serial_result": RESULT_FAIL,
        "operator_result": "",
        "completion": "EXCEPTION",
        "elapsed_s": 0.0,
        "pre_delay_s": 0.0,
        "notes": "",
        "classification_reason": message,
        "destructive": False,
        "requires_opt_in": "",
        "recovery_command": "",
        "parsed_evidence": {},
        "output_excerpt": message[-1000:],
    }


def parser_self_test() -> tuple[bool, list[str]]:
    checks: list[tuple[str, bool]] = []
    checks.append(("chip id parser", extract_parsed_evidence("Chip ID: 0x60").get("chip_id") == "0x60"))
    checks.append(
        (
            "ctrl_meas sleep validator",
            classify_output(
                CommandSpec(
                    command="reg 0xF4",
                    purpose="self-test",
                    expected=("Reg 0xF4 = 0x",),
                    validators=(VALIDATOR_CTRL_MEAS_SLEEP,),
                ),
                "Reg 0xF4 = 0x24 (36)\n",
                "MATCHED_EXPECTED",
            )[0]
            == RESULT_PASS,
        )
    )
    checks.append(
        (
            "stress parser",
            extract_parsed_evidence("Errors: 0\nDuration: 1234 ms\nRate: 12.34 samples/s\n").get(
                "stress_errors"
            )
            == 0,
        )
    )
    checks.append(
        (
            "stress_mix validator",
            classify_output(
                CommandSpec(
                    command="stress_mix 7",
                    purpose="self-test",
                    expected=("stress_mix summary", "Total:", "Health delta:"),
                    validators=(VALIDATOR_STRESS_MIX_ZERO_FAIL,),
                ),
                "=== stress_mix summary ===\n  Total: ok=7 fail=0 (100.00%)\n  Health delta: success +7, failures +0\n",
                "MATCHED_EXPECTED",
            )[0]
            == RESULT_PASS,
        )
    )
    checks.append(
        (
            "job done validator",
            classify_output(
                CommandSpec(
                    command="job force 1",
                    purpose="self-test",
                    expected=("Job Status", "Job state: DONE", "Status: OK"),
                    validators=(VALIDATOR_JOB_DONE_OR_FAILED,),
                ),
                (
                    "=== Job Status ===\n"
                    "Job kind: FORCED_MEASUREMENT\n"
                    "Job state: DONE\n"
                    "Status: OK (code=0, detail=0)\n"
                    "Callbacks used: 1\n"
                    "Instructions: 1\n"
                    "Driver: READY\n"
                    "Consecutive failures: 0\n"
                ),
                "MATCHED_EXPECTED",
            )[0]
            == RESULT_PASS,
        )
    )
    checks.append(
        (
            "job consecutive validator",
            classify_output(
                CommandSpec(
                    command="job resync 1",
                    purpose="self-test",
                    expected=("Job Status",),
                    validators=(VALIDATOR_JOB_ZERO_CONSECUTIVE_FAILURES,),
                ),
                (
                    "=== Job Status ===\n"
                    "Job state: DONE\n"
                    "Status: OK (code=0, detail=0)\n"
                    "Callbacks used: 1\n"
                    "Instructions: 1\n"
                    "Driver: READY\n"
                    "Consecutive failures: 2\n"
                ),
                "MATCHED_EXPECTED",
            )[0]
            == RESULT_FAIL,
        )
    )
    checks.append(
        (
            "job instruction budget validator",
            classify_output(
                CommandSpec(
                    command="job force 1",
                    purpose="self-test",
                    expected=("Job Status",),
                    validators=(VALIDATOR_JOB_INSTRUCTION_BUDGET_RESPECTED,),
                ),
                (
                    "=== Job Status ===\n"
                    "Job state: DONE\n"
                    "Status: OK (code=0, detail=0)\n"
                    "Callbacks used: 2\n"
                    "Instructions: 2\n"
                    "Driver: READY\n"
                    "Consecutive failures: 0\n"
                ),
                "MATCHED_EXPECTED",
            )[0]
            == RESULT_FAIL,
        )
    )
    checks.append(
        (
            "require pass exit zero",
            exit_code_for_verdict("PASS", argparse.Namespace(require_pass=True, fail_on_review=False)) == 0,
        )
    )
    checks.append(
        (
            "require pass exit three",
            exit_code_for_verdict("OPERATOR_REVIEW_REQUIRED", argparse.Namespace(require_pass=True, fail_on_review=False)) == 3,
        )
    )
    checks.append(
        (
            "fail on review exit three",
            exit_code_for_verdict("OPERATOR_REVIEW_REQUIRED", argparse.Namespace(require_pass=False, fail_on_review=True)) == 3,
        )
    )
    failures = [name for name, ok in checks if not ok]
    return not failures, failures


def duration_soak_cycle_commands(args: argparse.Namespace, cycle_index: int) -> tuple[CommandSpec, ...]:
    stress_count_value = getattr(args, "soak_cycle_stress_count", 50)
    mix_count_value = getattr(args, "soak_cycle_mix_count", 70)
    specs: list[CommandSpec] = [
        CommandSpec(
            command=f"stress {stress_count_value}",
            purpose=f"Duration soak cycle {cycle_index}: forced measurement batch.",
            group="soak-duration",
            expected=("Stress Summary", "Errors:", "Rate:"),
            completion=("Health delta:",),
            validators=(VALIDATOR_STRESS_ZERO_ERRORS,),
            timeout_s=max(30.0, min(300.0, 0.5 * float(stress_count_value) + 30.0)),
            operator_check=True,
            requires_opt_in="--soak-duration-s",
        ),
        CommandSpec(
            command=f"stress_mix {mix_count_value}",
            purpose=f"Duration soak cycle {cycle_index}: mixed safe operation batch.",
            group="soak-duration",
            expected=("stress_mix summary", "Total:", "Health delta:"),
            completion=("Health delta:",),
            validators=(VALIDATOR_STRESS_MIX_ZERO_FAIL,),
            timeout_s=max(30.0, min(300.0, 0.4 * float(mix_count_value) + 30.0)),
            operator_check=True,
            requires_opt_in="--soak-duration-s",
        ),
        CommandSpec(
            command="status",
            purpose=f"Duration soak cycle {cycle_index}: status flags and dirty state.",
            group="soak-duration",
            expected=("Status: 0x", "Driver:"),
            validators=(VALIDATOR_STATUS_NOT_MEASURING,),
            timeout_s=5.0,
            requires_opt_in="--soak-duration-s",
        ),
        CommandSpec(
            command="drv",
            purpose=f"Duration soak cycle {cycle_index}: driver health counters.",
            group="soak-duration",
            expected=("Driver Health", "Total"),
            validators=(VALIDATOR_DRIVER_ZERO_CONSECUTIVE,),
            timeout_s=5.0,
            requires_opt_in="--soak-duration-s",
        ),
    ]
    if cycle_index % 3 == 0:
        specs.extend(
            (
                CommandSpec(
                    command="probe",
                    purpose=f"Duration soak cycle {cycle_index}: raw no-health-side-effect probe.",
                    group="soak-duration",
                    expected=("Status: OK",),
                    timeout_s=5.0,
                    requires_opt_in="--soak-duration-s",
                ),
                CommandSpec(
                    command="chipid",
                    purpose=f"Duration soak cycle {cycle_index}: identity recheck.",
                    group="soak-duration",
                    expected=("Chip ID: 0x60",),
                    timeout_s=5.0,
                    requires_opt_in="--soak-duration-s",
                ),
            )
        )
    if cycle_index % 5 == 0:
        specs.extend(
            (
                CommandSpec(
                    command="cfg",
                    purpose=f"Duration soak cycle {cycle_index}: config snapshot.",
                    group="soak-duration",
                    expected=("ctrl_hum", "ctrl_meas", "config", "Hardware config dirty:"),
                    completion=("Hardware config dirty:",),
                    timeout_s=10.0,
                    requires_opt_in="--soak-duration-s",
                ),
                CommandSpec(
                    command="timing",
                    purpose=f"Duration soak cycle {cycle_index}: timing snapshot.",
                    group="soak-duration",
                    expected=("Estimated measurement time", "Estimated normal cycle"),
                    timeout_s=5.0,
                    requires_opt_in="--soak-duration-s",
                ),
            )
        )
    if cycle_index % 7 == 0:
        specs.extend(
            (
                CommandSpec(
                    command="normal on",
                    purpose=f"Duration soak cycle {cycle_index}: enter normal mode.",
                    group="soak-duration",
                    expected=("Status: OK",),
                    timeout_s=5.0,
                    requires_opt_in="--soak-duration-s",
                    recovery_command="normal off",
                ),
                CommandSpec(
                    command="read",
                    purpose=f"Duration soak cycle {cycle_index}: normal-mode sample.",
                    group="soak-duration",
                    expected=("Temp:", "Pressure:", "Humidity:"),
                    timeout_s=12.0,
                    pre_delay_s=1.0,
                    operator_check=True,
                    requires_opt_in="--soak-duration-s",
                ),
                CommandSpec(
                    command="normal off",
                    purpose=f"Duration soak cycle {cycle_index}: return to sleep.",
                    group="soak-duration",
                    expected=("Status: OK",),
                    timeout_s=5.0,
                    requires_opt_in="--soak-duration-s",
                ),
            )
        )
    reset_interval = getattr(args, "soak_reset_interval", 20)
    if reset_interval > 0 and cycle_index % reset_interval == 0:
        specs.extend(
            (
                CommandSpec(
                    command="reset",
                    purpose=f"Duration soak cycle {cycle_index}: bounded soft reset.",
                    group="soak-duration",
                    expected_any=("Status: OK", "Status: BUSY"),
                    unknowns=("Status: BUSY",),
                    completion=("Status:",),
                    timeout_s=8.0,
                    requires_opt_in="--soak-duration-s",
                ),
                CommandSpec(
                    command="recover",
                    purpose=f"Duration soak cycle {cycle_index}: recovery/resync after reset.",
                    group="soak-duration",
                    expected=("Status: OK",),
                    timeout_s=8.0,
                    requires_opt_in="--soak-duration-s",
                ),
                CommandSpec(
                    command="status",
                    purpose=f"Duration soak cycle {cycle_index}: post-reset status.",
                    group="soak-duration",
                    expected=("Status: 0x", "Driver:", "dirty=false"),
                    validators=(VALIDATOR_STATUS_NOT_MEASURING, VALIDATOR_STATUS_IM_UPDATE_CLEAR),
                    timeout_s=5.0,
                    requires_opt_in="--soak-duration-s",
                ),
            )
        )
    return tuple(specs)


def run_duration_soak(ser, transcript, args: argparse.Namespace) -> tuple[list[dict], dict]:
    requested_s = float(getattr(args, "soak_duration_s", 0.0))
    if requested_s <= 0.0:
        return [], {
            "requested_duration_s": requested_s,
            "executed": False,
            "start": "",
            "end": "",
            "elapsed_s": 0.0,
            "cycles": 0,
            "stop_reason": "not requested",
        }

    start_wall = timestamp()
    start = time.monotonic()
    deadline = start + requested_s
    cycle = 0
    rows: list[dict] = []
    stop_reason = "deadline reached"
    transcript.write(f"\n--- DURATION SOAK START {start_wall} requested_s={requested_s:.3f} ---\n")
    transcript.flush()
    if getattr(args, "verbose", False):
        print(f"\n--- DURATION SOAK START {start_wall} requested_s={requested_s:.3f} ---")

    while time.monotonic() < deadline:
        cycle += 1
        stop_for_deadline = False
        for spec in duration_soak_cycle_commands(args, cycle):
            remaining_s = deadline - time.monotonic()
            if remaining_s <= 0.0:
                stop_for_deadline = True
                break
            if remaining_s < 1.0:
                stop_reason = "deadline reached before next command"
                stop_for_deadline = True
                break
            timeout_s = min(spec.timeout_s if spec.timeout_s > 0 else args.timeout, remaining_s)
            row = run_serial_command(
                ser,
                spec,
                transcript,
                timeout_s=timeout_s,
                idle_after_output_s=args.idle_timeout_s,
                idle_after_match_s=args.idle_after_match_s,
                command_pacing_s=args.command_pacing_s,
                verbose=args.verbose,
            )
            row["soak_cycle"] = cycle
            rows.append(row)
            if row["serial_result"] in (RESULT_FAIL, RESULT_TIMEOUT):
                stop_reason = f"stopped after {row['serial_result']} from {spec.command}"
                end_wall = timestamp()
                elapsed = time.monotonic() - start
                transcript.write(f"\n--- DURATION SOAK STOP {end_wall} reason={stop_reason} ---\n")
                transcript.flush()
                return rows, {
                    "requested_duration_s": requested_s,
                    "executed": True,
                    "start": start_wall,
                    "end": end_wall,
                    "elapsed_s": round(elapsed, 3),
                    "cycles": cycle,
                    "stop_reason": stop_reason,
                }
        if stop_for_deadline:
            break
        idle_s = float(getattr(args, "soak_cycle_idle_s", 0.0))
        remaining_s = deadline - time.monotonic()
        if idle_s > 0.0 and remaining_s > 0.0:
            time.sleep(min(idle_s, remaining_s))

    end_wall = timestamp()
    elapsed = time.monotonic() - start
    transcript.write(f"\n--- DURATION SOAK END {end_wall} elapsed_s={elapsed:.3f} ---\n")
    transcript.flush()
    if getattr(args, "verbose", False):
        print(f"\n--- DURATION SOAK END {end_wall} elapsed_s={elapsed:.3f} ---")
    return rows, {
        "requested_duration_s": requested_s,
        "executed": True,
        "start": start_wall,
        "end": end_wall,
        "elapsed_s": round(elapsed, 3),
        "cycles": cycle,
        "stop_reason": stop_reason,
    }


def final_verdict(results: Iterable[dict], *, dry_run: bool) -> str:
    serial_results = [row["serial_result"] for row in results]
    if dry_run:
        return "INCOMPLETE"
    if any(result in (RESULT_FAIL, RESULT_TIMEOUT) for result in serial_results):
        return "FAIL"
    if any(result in (RESULT_OPERATOR, RESULT_REVIEW, RESULT_SERIAL_REVIEW, RESULT_UNKNOWN) for result in serial_results):
        return "OPERATOR_REVIEW_REQUIRED"
    return "PASS"


def exit_code_for_verdict(verdict: str, args: argparse.Namespace) -> int:
    if verdict == "FAIL":
        return 1
    review_verdicts = {"OPERATOR_REVIEW_REQUIRED", "INCOMPLETE"}
    if verdict in review_verdicts and (getattr(args, "require_pass", False) or getattr(args, "fail_on_review", False)):
        return 3
    if getattr(args, "require_pass", False) and verdict != "PASS":
        return 3
    return 0


def safe_cell(value: object) -> str:
    return str(value).replace("|", "\\|")


def artifact_path(summary: dict, key: str) -> str:
    return summary["artifacts"].get(key, "")


def write_summary_md(path: pathlib.Path, summary: dict) -> None:
    result_counts: dict[str, int] = {}
    for row in summary["results"]:
        result_counts[row["serial_result"]] = result_counts.get(row["serial_result"], 0) + 1
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
        f"Result counts: `{json.dumps(result_counts, sort_keys=True)}`",
        f"Recovered reset BUSY rows: `{summary.get('reset_busy_recovered_count', 0)}`",
        "",
        f"Claim boundary: {summary['claim_boundary']}",
        "",
        "## Duration Soak",
        "",
        f"Requested duration: `{summary.get('soak', {}).get('requested_duration_s', 0.0)}` s",
        f"Executed: `{summary.get('soak', {}).get('executed', False)}`",
        f"Start: `{summary.get('soak', {}).get('start', '')}`",
        f"End: `{summary.get('soak', {}).get('end', '')}`",
        f"Elapsed: `{summary.get('soak', {}).get('elapsed_s', 0.0)}` s",
        f"Cycles: `{summary.get('soak', {}).get('cycles', 0)}`",
        f"Stop reason: `{summary.get('soak', {}).get('stop_reason', '')}`",
        "",
        "## Command Results",
        "",
        "| Group | Command | Purpose | Expected/validators | Serial result | Operator result | Completion | Elapsed s | Soak cycle | Notes |",
        "| --- | --- | --- | --- | --- | --- | --- | ---: | ---: | --- |",
    ]
    for row in summary["results"]:
        expected = ", ".join(
            row.get("expected", [])
            + row.get("expected_any", [])
            + row.get("unknowns", [])
            + row.get("validators", [])
        )
        lines.append(
            f"| `{safe_cell(row['group'])}` | `{safe_cell(row['command'])}` | {safe_cell(row['purpose'])} | "
            f"{safe_cell(expected)} | `{row['serial_result']}` | "
            f"`{row['operator_result']}` | `{row['completion']}` | "
            f"{row['elapsed_s']:.3f} | {row.get('soak_cycle', '')} | {safe_cell(row['notes'])} |"
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
        "soak_cycle",
        "parsed_evidence",
        "unknowns",
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
        "timing": {
            "timeout_s": getattr(args, "timeout", 8.0),
            "idle_timeout_s": getattr(args, "idle_timeout_s", 0.75),
            "idle_after_match_s": getattr(args, "idle_after_match_s", 0.25),
            "boot_settle_s": getattr(args, "boot_settle_s", 1.5),
            "command_pacing_s": getattr(args, "command_pacing_s", 0.0),
            "reconnect_attempts": getattr(args, "reconnect_attempts", 0),
            "reconnect_delay_s": getattr(args, "reconnect_delay_s", 1.0),
        },
        "opt_in_flags": {
            "include_soak": getattr(args, "include_soak", False),
            "soak_count": getattr(args, "soak_count", 500),
            "include_normal_soak": getattr(args, "include_normal_soak", False),
            "normal_soak_count": getattr(args, "normal_soak_count", 5),
            "normal_soak_interval_s": getattr(args, "normal_soak_interval_s", 1.0),
            "include_config_matrix": getattr(args, "include_config_matrix", False),
            "include_invalid_inputs": getattr(args, "include_invalid_inputs", False),
            "include_benchmarks": getattr(args, "include_benchmarks", False)
            or getattr(args, "sample_rate_benchmark", False),
            "include_job_api": getattr(args, "include_job_api", False),
            "soak_duration_s": getattr(args, "soak_duration_s", 0.0),
            "soak_cycle_stress_count": getattr(args, "soak_cycle_stress_count", 50),
            "soak_cycle_mix_count": getattr(args, "soak_cycle_mix_count", 70),
            "soak_cycle_idle_s": getattr(args, "soak_cycle_idle_s", 0.0),
            "soak_reset_interval": getattr(args, "soak_reset_interval", 20),
            "include_destructive": getattr(args, "include_destructive", False),
            "include_fault_tests": getattr(args, "include_fault_tests", False),
            "require_pass": getattr(args, "require_pass", False),
            "fail_on_review": getattr(args, "fail_on_review", False),
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
    parser.add_argument("--timeout", "--timeout-s", dest="timeout", type=float, default=8.0, help="Default command timeout in seconds.")
    parser.add_argument("--idle-timeout-s", type=parse_nonnegative_float, default=0.75, help="Idle time after output for commands without expected tokens.")
    parser.add_argument("--idle-after-match-s", type=parse_nonnegative_float, default=0.25, help="Idle time after expected/failure tokens before command completion.")
    parser.add_argument("--boot-settle-s", type=parse_nonnegative_float, default=1.5, help="Initial serial settle/read window after opening the port.")
    parser.add_argument("--command-pacing-s", type=parse_nonnegative_float, default=0.0, help="Bounded delay before each serial command.")
    parser.add_argument("--reconnect-attempts", type=parse_nonnegative_int, default=0, help="Serial-open retry attempts after the initial attempt.")
    parser.add_argument("--reconnect-delay-s", type=parse_nonnegative_float, default=1.0, help="Delay between serial-open retry attempts.")
    parser.add_argument("--verbose", action="store_true", help="Echo transcript chunks to stdout while running.")
    parser.add_argument("--parser-self-test", action="store_true", help="Run parser/classifier self-tests and exit.")
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
    parser.add_argument("--include-config-matrix", action="store_true", help="Include safe configuration boundary matrix commands.")
    parser.add_argument("--include-invalid-inputs", action="store_true", help="Include safe CLI invalid-input checks.")
    parser.add_argument("--include-benchmarks", action="store_true", help="Include sample-rate benchmark commands.")
    parser.add_argument("--sample-rate-benchmark", action="store_true", help="Alias for --include-benchmarks.")
    parser.add_argument(
        "--include-job-api",
        action="store_true",
        help="Include bounded staged-job start/poll/cancel/resync/reset checks.",
    )
    parser.add_argument("--soak-duration-s", type=parse_nonnegative_float, default=0.0, help="Run a duration-based safe soak loop after the fixed plan.")
    parser.add_argument("--soak-cycle-stress-count", type=parse_positive_int, default=50, help="Forced stress count per duration-soak cycle.")
    parser.add_argument("--soak-cycle-mix-count", type=parse_positive_int, default=70, help="stress_mix count per duration-soak cycle.")
    parser.add_argument("--soak-cycle-idle-s", type=parse_nonnegative_float, default=0.0, help="Idle delay between duration-soak cycles.")
    parser.add_argument("--soak-reset-interval", type=parse_nonnegative_int, default=20, help="Duration-soak cycle interval for soft reset/recover; 0 disables.")
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
    parser.add_argument("--require-pass", action="store_true", help="Exit nonzero unless the final verdict is PASS.")
    parser.add_argument("--fail-on-review", action="store_true", help="Exit 3 for review/unknown verdicts.")
    return parser.parse_args(argv)


def execution_safety_errors(args: argparse.Namespace, executable: list[CommandSpec]) -> list[str]:
    errors: list[str] = []
    def is_well_formed_raw_write(command: str) -> bool:
        parts = command.strip().split()
        return len(parts) == 3 and parts[0].lower() == "wreg"

    has_raw_write = any(
        spec.destructive or is_well_formed_raw_write(spec.command) for spec in executable
    )
    if has_raw_write and not args.dry_run and args.confirm_raw_write != RAW_WRITE_CONFIRMATION:
        errors.append(f"raw-write execution requires --confirm-raw-write {RAW_WRITE_CONFIRMATION}")
    return errors


def main(argv: list[str] | None = None) -> int:
    runner_argv = list(sys.argv[1:] if argv is None else argv)
    args = parse_args(runner_argv)
    if args.parser_self_test:
        ok, failures = parser_self_test()
        if ok:
            print("HIL parser self-test PASSED")
            return 0
        print("HIL parser self-test FAILED: " + ", ".join(failures), file=sys.stderr)
        return 1
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
    soak_summary = {
        "requested_duration_s": getattr(args, "soak_duration_s", 0.0),
        "executed": False,
        "start": "",
        "end": "",
        "elapsed_s": 0.0,
        "cycles": 0,
        "stop_reason": "not requested",
    }
    with transcript_path.open("w", encoding="utf-8", newline="") as transcript:
        write_transcript_header(transcript, args, log_dir, runner_argv=runner_argv)
        if args.dry_run:
            transcript.write("DRY RUN: no serial port opened and no commands sent.\n")
            for spec in executable:
                transcript.write(f"DRY RUN COMMAND: [{spec.group}] {spec.command} -- {spec.purpose}\n")
                results.append(dry_run_result(spec))
            if args.soak_duration_s > 0.0:
                transcript.write("DRY RUN: representative duration-soak cycle only.\n")
                for spec in duration_soak_cycle_commands(args, 1):
                    transcript.write(f"DRY RUN SOAK COMMAND: [{spec.group}] {spec.command} -- {spec.purpose}\n")
                    results.append(dry_run_result(spec))
                soak_summary = {
                    "requested_duration_s": args.soak_duration_s,
                    "executed": False,
                    "start": "",
                    "end": "",
                    "elapsed_s": 0.0,
                    "cycles": 1,
                    "stop_reason": "dry run representative cycle only",
                }
        else:
            serial = import_serial()
            try:
                with open_serial_with_retries(serial, args) as ser:
                    time.sleep(args.boot_settle_s)
                    boot = read_available(ser)
                    if boot:
                        transcript.write("--- BOOT/INITIAL OUTPUT ---\n")
                        transcript.write(boot)
                        transcript.write("\n")
                        if args.verbose:
                            print("--- BOOT/INITIAL OUTPUT ---")
                            print(boot, end="")
                    for spec in executable:
                        timeout_s = spec.timeout_s if spec.timeout_s > 0 else args.timeout
                        results.append(
                            run_serial_command(
                                ser,
                                spec,
                                transcript,
                                timeout_s=timeout_s,
                                idle_after_output_s=args.idle_timeout_s,
                                idle_after_match_s=args.idle_after_match_s,
                                command_pacing_s=args.command_pacing_s,
                                verbose=args.verbose,
                            )
                        )
                        if results[-1]["serial_result"] in (RESULT_FAIL, RESULT_TIMEOUT):
                            transcript.write("Stopping fixed plan after hard failure.\n")
                            break
                    if not any(row["serial_result"] in (RESULT_FAIL, RESULT_TIMEOUT) for row in results):
                        soak_rows, soak_summary = run_duration_soak(ser, transcript, args)
                        results.extend(soak_rows)
            except Exception as exc:
                message = f"Serial session failed: {exc}"
                transcript.write(message + "\n")
                if args.verbose:
                    print(message, file=sys.stderr)
                results.append(session_error_result(message))

    reset_busy_recovered_count = reclassify_recovered_reset_busy(results)
    job_api_correlation_failures = reclassify_job_api_correlation(results)
    summary = {
        "run_id": log_dir.name,
        "datetime": timestamp(),
        "runner_command": format_runner_command(runner_argv),
        "runner_args": runner_argv,
        "branch": git_value(["branch", "--show-current"]),
        "commit": git_value(["rev-parse", "HEAD"]),
        "worktree": worktree_state(),
        "job_api_correlation_failures": job_api_correlation_failures,
        "port": args.port or "DRY_RUN",
        "baud": args.baud,
        "address": args.address,
        "dry_run": args.dry_run,
        "claim_boundary": CLAIM_BOUNDARY,
        "environment": environment_record(args),
        "command_sequence": [spec.command for spec in executable],
        "manual_or_skipped": [dataclasses.asdict(item) for item in manual_items],
        "results": results,
        "soak": soak_summary,
        "reset_busy_recovered_count": reset_busy_recovered_count,
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
    return exit_code_for_verdict(summary["final_verdict"], args)


if __name__ == "__main__":
    raise SystemExit(main())
