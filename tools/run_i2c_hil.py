#!/usr/bin/env python3
"""Serial HIL runner for the BME280 diagnostic CLI.

The runner drives the existing Arduino/ESP-IDF serial CLI. It does not flash
firmware and it does not prove hardware validity by itself; it captures a
disciplined transcript and classifies only what the serial output supports.
"""

from __future__ import annotations

import argparse
import dataclasses
import datetime as dt
import json
import os
import pathlib
import re
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


ANSI_RE = re.compile(r"\x1b\[[0-9;]*m")
FAILURE_PATTERNS = (
    re.compile(r"\bStatus:\s*(?:DEVICE_NOT_FOUND|I2C_TIMEOUT|I2C_NACK_ADDR|"
               r"I2C_NACK_DATA|I2C_BUS|I2C_ERROR|CHIP_ID_MISMATCH|"
               r"CALIBRATION_INVALID|INVALID_CONFIG|INVALID_PARAM|"
               r"NOT_INITIALIZED)\b"),
    re.compile(r"\b(?:FAILED|FAIL)\b"),
    re.compile(r"\[E\]"),
)


@dataclasses.dataclass(frozen=True)
class CommandSpec:
    command: str
    purpose: str
    expected: tuple[str, ...] = ()
    failures: tuple[str, ...] = ()
    timeout_s: float = 5.0
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
        expected=("Version Info", "BME280 library", "commit"),
        timeout_s=4.0,
        notes="Version output identifies firmware provenance but is not hardware proof.",
    ),
    CommandSpec(
        command="help",
        purpose="Capture the CLI command surface used by this HIL run.",
        expected=("BME280 CLI Help", "scan", "probe", "selftest"),
        timeout_s=4.0,
    ),
    CommandSpec(
        command="scan",
        purpose="Scan the I2C bus for acknowledging addresses.",
        expected=("device(s)",),
        failures=("No I2C devices found",),
        timeout_s=8.0,
        notes="A scan proves only address ACK; it is not chip identity.",
    ),
    CommandSpec(
        command="addr {address}",
        purpose="Select the documented BME280 diagnostic address and re-run begin().",
        expected=("Status: OK",),
        failures=("DEVICE_NOT_FOUND", "I2C_NACK_ADDR", "I2C_TIMEOUT", "I2C_BUS"),
        timeout_s=6.0,
        notes="Use 0x76 for SDO=GND or 0x77 for SDO=VDDIO.",
    ),
    CommandSpec(
        command="begin",
        purpose="Initialize the driver, verify chip ID, read calibration, and apply config.",
        expected=("Status: OK",),
        timeout_s=6.0,
    ),
    CommandSpec(
        command="probe",
        purpose="Run raw probe without health side effects.",
        expected=("Status: OK",),
        failures=("DEVICE_NOT_FOUND", "I2C_NACK_ADDR", "I2C_TIMEOUT", "I2C_BUS"),
        timeout_s=5.0,
        notes="Probe plus ACK does not prove identity unless chip ID is read separately.",
    ),
    CommandSpec(
        command="chipid",
        purpose="Read BME280 chip ID register 0xD0 and verify documented value 0x60.",
        expected=("Chip ID: 0x60",),
        failures=("CHIP_ID_MISMATCH", "I2C_TIMEOUT", "I2C_BUS"),
        timeout_s=5.0,
        notes="This is the identity check for BME280.",
    ),
    CommandSpec(
        command="cfg",
        purpose="Capture chip and cached driver settings.",
        expected=("ctrl_hum", "ctrl_meas", "config"),
        timeout_s=5.0,
    ),
    CommandSpec(
        command="calib",
        purpose="Capture cached calibration coefficients for plausibility review.",
        expected=("Calibration", "T1", "P1"),
        timeout_s=5.0,
        notes="Calibration plausibility still requires auditor/operator review.",
    ),
    CommandSpec(
        command="calib raw",
        purpose="Capture raw calibration register bytes.",
        expected=("Calibration (Raw Registers)", "TP:", "H:"),
        timeout_s=5.0,
    ),
    CommandSpec(
        command="status",
        purpose="Read status register and driver dirty-state summary.",
        expected=("Status: 0x", "Driver:"),
        timeout_s=5.0,
    ),
    CommandSpec(
        command="timing",
        purpose="Record measurement timing estimates for the current config.",
        expected=("Estimated measurement time", "Estimated normal cycle"),
        timeout_s=5.0,
    ),
    CommandSpec(
        command="reg 0xD0",
        purpose="Read chip ID register through generic register command.",
        expected=("Reg 0xD0 = 0x60",),
        timeout_s=5.0,
        notes="Duplicates chip-ID evidence through raw register helper.",
    ),
    CommandSpec(
        command="read",
        purpose="Request and capture one compensated measurement.",
        expected=("Temp:", "Pressure:", "Humidity:"),
        timeout_s=10.0,
        operator_check=True,
        notes="Operator must judge environmental plausibility against references.",
    ),
    CommandSpec(
        command="raw",
        purpose="Capture cached raw ADC sample and validity flags.",
        expected=("Raw ADC", "Valid channels: T=", "Cached sample age"),
        timeout_s=5.0,
    ),
    CommandSpec(
        command="comp",
        purpose="Capture cached fixed-point compensated sample and validity flags.",
        expected=("Compensated", "Valid channels: T="),
        timeout_s=5.0,
    ),
    CommandSpec(
        command="data",
        purpose="Burst-read and decode live data registers 0xF7..0xFE.",
        expected=("Live Data Registers", "0xF7..0xFE"),
        timeout_s=5.0,
    ),
    CommandSpec(
        command="force",
        purpose="Trigger a forced-mode measurement.",
        expected=("Temp:", "Pressure:", "Humidity:"),
        timeout_s=10.0,
        operator_check=True,
        notes="Operator must judge plausibility; serial confirms only transaction flow.",
    ),
    CommandSpec(
        command="read",
        purpose="Read again after forced trigger to capture sample/cache behavior.",
        expected=("Temp:", "Pressure:", "Humidity:"),
        timeout_s=10.0,
        operator_check=True,
    ),
    CommandSpec(
        command="normal on",
        purpose="Switch to normal mode.",
        expected=("Status: OK",),
        timeout_s=5.0,
        recovery_command="normal off",
    ),
    CommandSpec(
        command="read",
        purpose="Capture a normal-mode fresh sample.",
        expected=("Temp:", "Pressure:", "Humidity:"),
        timeout_s=12.0,
        operator_check=True,
    ),
    CommandSpec(
        command="normal off",
        purpose="Return to sleep mode after normal-mode sample.",
        expected=("Status: OK",),
        timeout_s=5.0,
    ),
    CommandSpec(
        command="reset",
        purpose="Run BME280 soft reset, NVM wait, calibration reload, and config reapply.",
        expected=("Status: OK",),
        timeout_s=8.0,
        notes="Soft reset is volatile and expected to be safe for BME280.",
    ),
    CommandSpec(
        command="recover",
        purpose="Run manual recover/resync path.",
        expected=("Status: OK",),
        timeout_s=8.0,
    ),
    CommandSpec(
        command="selftest",
        purpose="Run existing safe command smoke-test report.",
        expected=("selftest", "PASS"),
        timeout_s=25.0,
        operator_check=True,
        notes="This is not Bosch factory calibration; review the report.",
    ),
    CommandSpec(
        command="stress 10",
        purpose="Run a short forced-measurement stress loop.",
        expected=("Stress Summary", "Errors:"),
        timeout_s=30.0,
        operator_check=True,
        notes="Serial summary needs operator review for sample plausibility.",
    ),
    CommandSpec(
        command="drv",
        purpose="Capture final driver health details.",
        expected=("Driver Health", "Total"),
        timeout_s=5.0,
    ),
    CommandSpec(
        command="state",
        purpose="Capture final compact health state.",
        expected=("state",),
        timeout_s=5.0,
    ),
)


SOAK_COMMANDS: tuple[CommandSpec, ...] = (
    CommandSpec(
        command="stress 500",
        purpose="Run the requested longer stress loop.",
        expected=("Stress Summary", "Errors:"),
        timeout_s=180.0,
        operator_check=True,
        requires_opt_in="--include-soak",
        notes="Longer soak evidence; still requires operator plausibility review.",
    ),
)


DESTRUCTIVE_COMMANDS: tuple[CommandSpec, ...] = (
    CommandSpec(
        command="wreg 0xF4 0x00",
        purpose="Diagnostic raw register write to force ctrl_meas sleep.",
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
        operator_check=True,
        requires_opt_in="--include-fault-tests",
        notes="Do not hotwire SDO while powered unless the fixture supports it.",
    ),
    CommandSpec(
        command="MANUAL: safe sensor unplug/replug and recover",
        purpose="Capture address-NACK and recovery behavior.",
        operator_check=True,
        requires_opt_in="--include-fault-tests",
        notes="Only disconnect when safe for the board and bench setup.",
    ),
    CommandSpec(
        command="MANUAL: safe SDA/SCL temporary fault if supported",
        purpose="Capture timeout/bus-fault behavior.",
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
                timeout_s=timeout_s,
                notes=f"Loaded from {path}",
            )
        )
    return commands


def build_command_sequence(args: argparse.Namespace) -> tuple[list[CommandSpec], list[CommandSpec]]:
    if args.commands:
        return load_command_file(pathlib.Path(args.commands), timeout_s=args.timeout), []

    executable = [cmd.formatted(address=args.address) for cmd in BASE_COMMANDS]
    checklist: list[CommandSpec] = []

    if args.include_soak:
        executable.extend(cmd.formatted(address=args.address) for cmd in SOAK_COMMANDS)
    else:
        checklist.extend(cmd.formatted(address=args.address) for cmd in SOAK_COMMANDS)

    if args.include_destructive:
        executable.extend(cmd.formatted(address=args.address) for cmd in DESTRUCTIVE_COMMANDS)
        executable.extend(
            (
                CommandSpec(
                    command="recover",
                    purpose="Resync after opt-in destructive raw-write diagnostic.",
                    expected=("Status: OK",),
                    timeout_s=8.0,
                    requires_opt_in="--include-destructive",
                    notes="Required evidence after raw register write.",
                ),
                CommandSpec(
                    command="cfg",
                    purpose="Capture settings after destructive-write recovery.",
                    expected=("ctrl_hum", "ctrl_meas", "config"),
                    timeout_s=5.0,
                    requires_opt_in="--include-destructive",
                    notes="Required evidence that recovery produced a readable configuration.",
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
    if spec.operator_check:
        if spec.expected and all(token in clean for token in spec.expected):
            return RESULT_OPERATOR, "Serial output matched expected tokens; operator plausibility review required."
        return RESULT_REVIEW, "Manual/operator command did not match all expected tokens."
    if spec.expected and all(token in clean for token in spec.expected):
        return RESULT_PASS, "Matched expected serial tokens."
    if clean.strip():
        return RESULT_SERIAL_REVIEW, "Serial output captured but expected tokens were incomplete."
    return RESULT_REVIEW, "No useful serial output captured."


def output_has_expected(spec: CommandSpec, output: str) -> bool:
    return bool(spec.expected) and all(token in output for token in spec.expected)


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
        return ser.read(waiting).decode("utf-8", errors="replace")
    return ""


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
    return {
        "command": spec.command,
        "purpose": spec.purpose,
        "serial_result": result,
        "operator_result": RESULT_OPERATOR if spec.operator_check else "",
        "completion": completion,
        "elapsed_s": round(elapsed, 3),
        "notes": spec.notes,
        "classification_reason": reason,
        "destructive": spec.destructive,
        "requires_opt_in": spec.requires_opt_in,
        "recovery_command": spec.recovery_command,
        "output_excerpt": strip_ansi(output)[-1000:],
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
        "serial_result": result,
        "operator_result": RESULT_OPERATOR if spec.operator_check else "",
        "completion": "DRY_RUN",
        "elapsed_s": 0.0,
        "notes": spec.notes,
        "classification_reason": "Dry-run only; no serial command was sent.",
        "destructive": spec.destructive,
        "requires_opt_in": spec.requires_opt_in,
        "recovery_command": spec.recovery_command,
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


def write_summary_md(path: pathlib.Path, summary: dict) -> None:
    lines = [
        "# I2C HIL Summary",
        "",
        f"Date/time: `{summary['datetime']}`",
        f"Branch: `{summary['branch']}`",
        f"Commit: `{summary['commit']}`",
        f"Worktree: `{summary['worktree']}`",
        f"Port: `{summary['port']}`",
        f"Baud: `{summary['baud']}`",
        f"I2C address: `{summary['address']}`",
        f"Dry run: `{summary['dry_run']}`",
        f"Final verdict: `{summary['final_verdict']}`",
        "",
        "## Command Results",
        "",
        "| Command | Purpose | Serial result | Operator result | Completion | Elapsed s | Notes |",
        "| --- | --- | --- | --- | --- | ---: | --- |",
    ]
    for row in summary["results"]:
        command = row["command"].replace("|", "\\|")
        purpose = row["purpose"].replace("|", "\\|")
        notes = row["notes"].replace("|", "\\|")
        lines.append(
            f"| `{command}` | {purpose} | `{row['serial_result']}` | "
            f"`{row['operator_result']}` | `{row['completion']}` | "
            f"{row['elapsed_s']:.3f} | {notes} |"
        )
    lines.extend(
        [
            "",
            "## Artifacts",
            "",
            f"- Serial transcript: `{summary['artifacts']['serial_transcript']}`",
            f"- JSON summary: `{summary['artifacts']['summary_json']}`",
            f"- Operator checklist: `{summary['artifacts']['operator_checklist']}`",
            "",
            "Hardware validation is complete only after this summary is reviewed with the "
            "captured transcript and required operator evidence.",
            "",
        ]
    )
    path.write_text("\n".join(lines), encoding="utf-8")


def write_operator_checklist(path: pathlib.Path, manual_items: list[CommandSpec], results: list[dict]) -> None:
    lines = [
        "# Operator Checklist",
        "",
        "- Record board model, BME280 module, wiring, pull-ups, supply voltage, bus speed, and firmware commit.",
        "- Attach photos/video, logic analyzer capture, or meter readings when relevant.",
        "- Mark environmental plausibility checks manually; the runner cannot prove sensor accuracy alone.",
        "",
        "## Serial Results Requiring Operator Review",
        "",
    ]
    for row in results:
        if row["operator_result"] == RESULT_OPERATOR:
            lines.append(f"- `{row['command']}`: {row['purpose']}")
    lines.extend(["", "## Skipped Or Manual Items", ""])
    for item in manual_items:
        lines.append(f"- `{item.command}`: {item.purpose} ({item.requires_opt_in or 'manual'})")
        if item.notes:
            lines.append(f"  Notes: {item.notes}")
    lines.append("")
    path.write_text("\n".join(lines), encoding="utf-8")


def write_transcript_header(transcript, args: argparse.Namespace, log_dir: pathlib.Path) -> None:
    transcript.write("# I2C HIL serial transcript\n")
    transcript.write(f"timestamp={timestamp()}\n")
    transcript.write(f"port={args.port or 'DRY_RUN'}\n")
    transcript.write(f"baud={args.baud}\n")
    transcript.write(f"address={args.address}\n")
    transcript.write(f"log_dir={log_dir}\n")
    transcript.write(f"branch={git_value(['branch', '--show-current'])}\n")
    transcript.write(f"commit={git_value(['rev-parse', 'HEAD'])}\n")
    transcript.write(f"worktree={worktree_state()}\n")
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
    parser.add_argument("--include-soak", action="store_true", help="Include longer stress 500 soak command.")
    parser.add_argument("--include-fault-tests", action="store_true", help="Mark manual fault-test checklist items as intentionally requested.")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv or sys.argv[1:])
    if not args.dry_run and not args.port:
        print("--port is required unless --dry-run is used.", file=sys.stderr)
        return 2

    out_base = pathlib.Path(args.out)
    if not out_base.is_absolute():
        out_base = ROOT / out_base
    log_dir = make_log_dir(out_base)

    executable, manual_items = build_command_sequence(args)

    transcript_path = log_dir / "serial_transcript.txt"
    summary_md_path = log_dir / "summary.md"
    summary_json_path = log_dir / "summary.json"
    checklist_path = log_dir / "operator_checklist.md"

    results: list[dict] = []
    with transcript_path.open("w", encoding="utf-8", newline="") as transcript:
        write_transcript_header(transcript, args, log_dir)
        if args.dry_run:
            transcript.write("DRY RUN: no serial port opened and no commands sent.\n")
            for spec in executable:
                transcript.write(f"DRY RUN COMMAND: {spec.command} -- {spec.purpose}\n")
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
        "datetime": timestamp(),
        "branch": git_value(["branch", "--show-current"]),
        "commit": git_value(["rev-parse", "HEAD"]),
        "worktree": worktree_state(),
        "port": args.port or "DRY_RUN",
        "baud": args.baud,
        "address": args.address,
        "dry_run": args.dry_run,
        "command_sequence": [spec.command for spec in executable],
        "manual_or_skipped": [dataclasses.asdict(item) for item in manual_items],
        "results": results,
        "final_verdict": final_verdict(results, dry_run=args.dry_run),
        "artifacts": {
            "serial_transcript": os.fspath(transcript_path),
            "summary_md": os.fspath(summary_md_path),
            "summary_json": os.fspath(summary_json_path),
            "operator_checklist": os.fspath(checklist_path),
        },
        "pyserial_install_hint": INSTALL_HINT,
    }
    summary_json_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    write_summary_md(summary_md_path, summary)
    write_operator_checklist(checklist_path, manual_items, results)

    print(f"HIL artifacts: {log_dir}")
    print(f"Summary: {summary_md_path}")
    print(f"Final verdict: {summary['final_verdict']}")
    print(f"Operator checklist: {checklist_path}")
    if args.dry_run:
        print("Dry run only: no physical HIL validation was performed.")
    return 0 if summary["final_verdict"] != "FAIL" else 1


if __name__ == "__main__":
    raise SystemExit(main())
