#!/usr/bin/env python3
"""Focused tests for HIL runner serial-output classification."""

from __future__ import annotations

import io
import contextlib
import pathlib
import re
import sys
import tempfile
import unittest
from unittest import mock


ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

import run_i2c_hil  # noqa: E402


def calib_spec() -> run_i2c_hil.CommandSpec:
    for spec in run_i2c_hil.BASE_COMMANDS:
        if spec.command == "calib":
            return spec
    raise AssertionError("default HIL command list is missing calib")


def first_cfg_spec() -> run_i2c_hil.CommandSpec:
    for spec in run_i2c_hil.BASE_COMMANDS:
        if spec.command == "cfg":
            return spec
    raise AssertionError("default HIL command list is missing cfg")


def probe_spec() -> run_i2c_hil.CommandSpec:
    for spec in run_i2c_hil.BASE_COMMANDS:
        if spec.command == "probe":
            return spec
    raise AssertionError("default HIL command list is missing probe")


def job_result_output(
    *,
    boundary: str,
    job_id: int,
    kind: str,
    phase: str,
    state: str,
    status: str,
    status_code: int,
    terminal: bool,
    callbacks: int,
    deadline_active: bool = False,
    deadline_ms: int = 0,
) -> str:
    return (
        "=== Job Status ===\n"
        f"Boundary: {boundary}\n"
        f"Job ID: {job_id}\n"
        f"Job kind: {kind}\n"
        f"Job phase: {phase}\n"
        f"Job state: {state}\n"
        f"Terminal state: {'true' if terminal else 'false'}\n"
        f"Status: {status} (code={status_code}, detail=0)\n"
        "Conversion state: IDLE\n"
        f"Phase deadline active: {'true' if deadline_active else 'false'}\n"
        f"Phase deadline ms: {deadline_ms}\n"
        f"Callbacks used: {callbacks}\n"
        f"Instructions: {callbacks}\n"
        "Driver: READY\n"
        "Hardware config dirty: false\n"
        "Consecutive failures: 0\n"
    )


class HilCalibClassificationTest(unittest.TestCase):
    def test_complete_calib_output_passes(self) -> None:
        spec = calib_spec()
        output = (
            "=== Calibration (Cached) ===\n"
            "  T1=28851 T2=27007 T3=50\n"
            "  P1=35571 P2=-10595 P3=3024 P4=8279 P5=3 P6=-7 P7=11700 P8=-11800 P9=5000\n"
            "  H1=75 H2=373 H3=0 H4=294 H5=50 H6=30\n"
            "  Plausibility: PASS (T1/P1 nonzero, humidity coeffs not all zero)\n"
        )

        result, reason = run_i2c_hil.classify_output(spec, output, "MATCHED_COMPLETION")

        self.assertEqual(run_i2c_hil.RESULT_PASS, result)
        self.assertIn("Matched expected serial tokens", reason)

    def test_delayed_multiline_calib_output_passes_after_final_line(self) -> None:
        spec = calib_spec()
        chunks = [
            "=== Calibration (Cached) ===\n",
            "  T1=28851 T2=27007 T3=50\n",
            "  P1=35571 P2=-10595 P3=3024 P4=8279 P5=3 P6=-7 P7=11700 P8=-11800 P9=5000\n",
            "  H1=75 H2=373 H3=0 H4=294 H5=50 H6=30\n",
            "  Plausibility: PASS (T1/P1 nonzero, humidity coeffs not all zero)\n",
        ]
        partial = "".join(chunks[:-1])
        complete = "".join(chunks)

        self.assertFalse(run_i2c_hil.output_is_complete(spec, partial))
        self.assertTrue(run_i2c_hil.output_is_complete(spec, complete))
        self.assertEqual(
            run_i2c_hil.RESULT_PASS,
            run_i2c_hil.classify_output(spec, complete, "MATCHED_COMPLETION")[0],
        )

    def test_truncated_calib_output_times_out(self) -> None:
        spec = calib_spec()
        output = (
            "=== Calibration (Cached) ===\n"
            "  T1=28851 T2=27007 T3=50\n"
        )

        result, reason = run_i2c_hil.classify_output(spec, output, run_i2c_hil.RESULT_TIMEOUT)

        self.assertEqual(run_i2c_hil.RESULT_TIMEOUT, result)
        self.assertIn("timed out", reason)

    def test_repeated_failure_tokens_fail_once_classified(self) -> None:
        spec = probe_spec()
        output = (
            "Probe\n"
            "Status: I2C_TIMEOUT\n"
            "Status: I2C_TIMEOUT\n"
            "[E] I2C_TIMEOUT\n"
        )

        result, reason = run_i2c_hil.classify_output(spec, output, "MATCHED_COMPLETION")

        self.assertEqual(run_i2c_hil.RESULT_FAIL, result)
        self.assertIn("I2C_TIMEOUT", reason)

    def test_calib_spec_uses_bounded_extended_window(self) -> None:
        spec = calib_spec()

        self.assertEqual(("Plausibility:",), spec.completion)
        self.assertGreaterEqual(spec.timeout_s, 10.0)
        self.assertIn("H1=", spec.expected)

    def test_cfg_waits_for_final_dirty_state_line(self) -> None:
        spec = first_cfg_spec()
        partial = (
            "=== Chip Settings ===\n"
            "  ctrl_hum: 0x01 (osrs_h=1 X1)\n"
            "  ctrl_meas: 0x24 (osrs_t=1 X1, osrs_p=1 X1, mode=0 SLEEP)\n"
            "  config: 0x40 (standby=2 125ms, filter=0 OFF, spi3w_en=0)\n"
        )
        complete = (
            partial
            + "=== Internal Settings ===\n"
            + "  Mode: FORCED\n"
            + "  osrs_t: X1 (1)\n"
            + "  osrs_p: X1 (1)\n"
            + "  osrs_h: X1 (1)\n"
            + "  Filter: OFF (0)\n"
            + "  Standby: 125ms (2)\n"
            + "  Verbose: OFF\n"
            + "  Hardware config dirty: false\n"
        )

        self.assertFalse(run_i2c_hil.output_is_complete(spec, partial))
        self.assertTrue(run_i2c_hil.output_is_complete(spec, complete))
        self.assertEqual(
            run_i2c_hil.RESULT_TIMEOUT,
            run_i2c_hil.classify_output(spec, partial, run_i2c_hil.RESULT_TIMEOUT)[0],
        )

    def test_read_available_uses_bounded_read_when_in_waiting_is_zero(self) -> None:
        class FakeSerial:
            def __init__(self) -> None:
                self.payload = [b"H", b"ello"]
                self.in_waiting = 0

            def read(self, size: int) -> bytes:
                self.assert_read_size = size
                if not self.payload:
                    return b""
                chunk = self.payload.pop(0)
                self.in_waiting = len(self.payload[0]) if self.payload else 0
                return chunk

        fake = FakeSerial()

        self.assertEqual("Hello", run_i2c_hil.read_available(fake))
        self.assertEqual(0, fake.in_waiting)

    def test_stress_mix_zero_fail_validator_passes_and_extracts_rate(self) -> None:
        spec = run_i2c_hil.CommandSpec(
            command="stress_mix 7",
            purpose="mixed stress parser test",
            expected=(
                "stress_mix summary", "Total:", "Health delta:",
                "Restore status: OK",
            ),
            validators=(run_i2c_hil.VALIDATOR_STRESS_MIX_ZERO_FAIL,),
        )
        output = (
            "=== stress_mix summary ===\n"
            "  Total: ok=7 fail=0 (100.00%)\n"
            "  Duration: 140 ms\n"
            "  Rate: 50.00 ops/s\n"
            "  Health delta: success +7, failures +0\n"
            "  Restore status: OK\n"
        )

        result, reason = run_i2c_hil.classify_output(spec, output, "MATCHED_COMPLETION")
        evidence = run_i2c_hil.extract_parsed_evidence(output)

        self.assertEqual(run_i2c_hil.RESULT_PASS, result)
        self.assertIn("stress_mix fail=0 and restore status=OK", reason)
        self.assertEqual(7, evidence["stress_mix_ok"])
        self.assertEqual(0, evidence["stress_mix_fail"])
        self.assertEqual(50.0, evidence["rate"])
        self.assertEqual("ops/s", evidence["rate_units"])
        self.assertEqual("OK", evidence["stress_mix_restore_status"])

    def test_stress_mix_nonzero_fail_validator_fails(self) -> None:
        spec = run_i2c_hil.CommandSpec(
            command="stress_mix 7",
            purpose="mixed stress parser test",
            expected=("stress_mix summary", "Total:", "Health delta:"),
            validators=(run_i2c_hil.VALIDATOR_STRESS_MIX_ZERO_FAIL,),
        )
        output = (
            "=== stress_mix summary ===\n"
            "  Total: ok=6 fail=1 (85.71%)\n"
            "  Health delta: success +6, failures +1\n"
            "  Restore status: OK\n"
        )

        result, reason = run_i2c_hil.classify_output(spec, output, "MATCHED_COMPLETION")

        self.assertEqual(run_i2c_hil.RESULT_FAIL, result)
        self.assertIn("fail=1", reason)

    def test_stress_mix_missing_or_failed_restore_cannot_pass(self) -> None:
        spec = run_i2c_hil.CommandSpec(
            command="stress_mix 7",
            purpose="mixed stress restore parser test",
            expected=("stress_mix summary", "Total:", "Health delta:"),
            validators=(run_i2c_hil.VALIDATOR_STRESS_MIX_ZERO_FAIL,),
        )
        prefix = (
            "=== stress_mix summary ===\n"
            "  Total: ok=7 fail=0 (100.00%)\n"
            "  Health delta: success +7, failures +0\n"
        )

        missing, missing_reason = run_i2c_hil.classify_output(
            spec, prefix, "MATCHED_COMPLETION"
        )
        failed, failed_reason = run_i2c_hil.classify_output(
            spec, prefix + "  Restore status: I2C_TIMEOUT\n", "MATCHED_COMPLETION"
        )

        self.assertEqual(run_i2c_hil.RESULT_REVIEW, missing)
        self.assertIn("restore status was not parsed", missing_reason)
        self.assertEqual(run_i2c_hil.RESULT_FAIL, failed)
        self.assertIn("I2C_TIMEOUT", failed_reason)

    def test_selftest_restore_failure_must_be_folded_into_fail_count(self) -> None:
        spec = next(
            item for item in run_i2c_hil.BASE_COMMANDS
            if item.command == "selftest"
        )
        output = (
            "=== BME280 selftest ===\n"
            "[PASS] command smoke checks\n"
            "Selftest result: pass=12 fail=1 skip=0\n"
        )

        result, reason = run_i2c_hil.classify_output(
            spec, output, "MATCHED_COMPLETION"
        )

        self.assertEqual(run_i2c_hil.RESULT_FAIL, result)
        self.assertIn("selftest reported fail=1", reason)

    def test_reset_busy_is_unknown_not_timeout_or_pass(self) -> None:
        spec = next(item for item in run_i2c_hil.BASE_COMMANDS if item.command == "reset")
        output = (
            "  Status: BUSY (code=11, detail=5)\n"
            "  Message: BUSY\n"
        )

        result, reason = run_i2c_hil.classify_output(spec, output, "MATCHED_COMPLETION")

        self.assertEqual(run_i2c_hil.RESULT_UNKNOWN, result)
        self.assertIn("Status: BUSY", reason)

    def test_recovered_reset_busy_uses_parsed_evidence_beyond_excerpt(self) -> None:
        omitted_tail = "x" * 1200
        results = [
            {
                "command": "reset",
                "group": "soak-duration",
                "serial_result": run_i2c_hil.RESULT_UNKNOWN,
                "classification_reason": "Matched command unknown/incomplete token: Status: BUSY",
                "parsed_evidence": run_i2c_hil.extract_parsed_evidence(
                    "Status: BUSY (code=11, detail=5)\nMessage: BUSY\n"
                ),
                "output_excerpt": "Status: BUSY (code=11, detail=5)\nMessage: BUSY\n",
            },
            {
                "command": "recover",
                "group": "soak-duration",
                "serial_result": run_i2c_hil.RESULT_PASS,
                "parsed_evidence": run_i2c_hil.extract_parsed_evidence(
                    "Status: OK (code=0, detail=0)\n" + omitted_tail
                ),
                "output_excerpt": omitted_tail[-1000:],
            },
            {
                "command": "status",
                "group": "soak-duration",
                "serial_result": run_i2c_hil.RESULT_PASS,
                "parsed_evidence": run_i2c_hil.extract_parsed_evidence(
                    "Status: 0x00 (measuring=0, im_update=0)\n"
                    "Driver: state=READY online=true dirty=false\n"
                    + omitted_tail
                ),
                "output_excerpt": omitted_tail[-1000:],
            },
        ]

        count = run_i2c_hil.reclassify_recovered_reset_busy(results)

        self.assertEqual(1, count)
        self.assertEqual(run_i2c_hil.RESULT_RESET_BUSY_RECOVERED, results[0]["serial_result"])
        self.assertIn("immediate follow-up", results[0]["classification_reason"])
        self.assertNotIn("Status:", results[1]["output_excerpt"])
        self.assertNotIn("Driver:", results[2]["output_excerpt"])

    def test_fixed_reset_recovery_proof_uses_only_parsed_evidence(self) -> None:
        omitted_tail = "y" * 1200

        def row(command: str, output: str, result: str = run_i2c_hil.RESULT_PASS) -> dict:
            return {
                "command": command,
                "group": "reset-recover",
                "serial_result": result,
                "parsed_evidence": run_i2c_hil.extract_parsed_evidence(
                    output + omitted_tail
                ),
                "output_excerpt": omitted_tail[-1000:],
            }

        results = [
            row(
                "reset",
                "Status: BUSY (code=11, detail=5)\nMessage: BUSY\n",
                run_i2c_hil.RESULT_UNKNOWN,
            ),
            row("status", "Status: 0x00 (measuring=0, im_update=0)\n"),
            row("recover", "Status: OK (code=0, detail=0)\n"),
            row(
                "cfg",
                "ctrl_hum: 0x01\nctrl_meas: 0x24\nconfig: 0x40\n",
            ),
            row(
                "status",
                "Status: 0x00 (measuring=0, im_update=0)\n"
                "Driver: state=READY online=true dirty=false\n",
            ),
        ]

        self.assertEqual(1, run_i2c_hil.reclassify_recovered_reset_busy(results))
        self.assertEqual(
            run_i2c_hil.RESULT_RESET_BUSY_RECOVERED,
            results[0]["serial_result"],
        )
        self.assertTrue(all(len(item["output_excerpt"]) == 1000 for item in results))

    def test_reset_busy_stays_unknown_without_clean_ready_status(self) -> None:
        results = [
            {
                "command": "reset",
                "group": "soak-duration",
                "serial_result": run_i2c_hil.RESULT_UNKNOWN,
                "classification_reason": "Matched command unknown/incomplete token: Status: BUSY",
                "parsed_evidence": run_i2c_hil.extract_parsed_evidence(
                    "Status: BUSY (code=11, detail=5)\nMessage: BUSY\n"
                ),
                "output_excerpt": "Status: BUSY (code=11, detail=5)\nMessage: BUSY\n",
            },
            {
                "command": "recover",
                "group": "soak-duration",
                "serial_result": run_i2c_hil.RESULT_PASS,
                "parsed_evidence": run_i2c_hil.extract_parsed_evidence(
                    "Status: OK (code=0, detail=0)\n"
                ),
                "output_excerpt": "Status: OK (code=0, detail=0)\n",
            },
            {
                "command": "status",
                "group": "soak-duration",
                "serial_result": run_i2c_hil.RESULT_PASS,
                "parsed_evidence": run_i2c_hil.extract_parsed_evidence(
                    "Status: 0x01 (measuring=0, im_update=1)\n"
                    "Driver: state=READY online=true dirty=false\n"
                ),
                "output_excerpt": "Status: 0x01 (measuring=0, im_update=1)\nDriver: state=READY online=true dirty=false\n",
            },
        ]

        count = run_i2c_hil.reclassify_recovered_reset_busy(results)

        self.assertEqual(0, count)
        self.assertEqual(run_i2c_hil.RESULT_UNKNOWN, results[0]["serial_result"])

    def test_recovered_reset_busy_does_not_block_pass_verdict(self) -> None:
        results = [
            {"serial_result": run_i2c_hil.RESULT_PASS},
            {"serial_result": run_i2c_hil.RESULT_RESET_BUSY_RECOVERED},
        ]

        self.assertEqual("PASS", run_i2c_hil.final_verdict(results, dry_run=False))

    def test_reset_busy_requires_canonical_nvm_detail(self) -> None:
        row = {
            "command": "reset",
            "serial_result": run_i2c_hil.RESULT_UNKNOWN,
            "parsed_evidence": run_i2c_hil.extract_parsed_evidence(
                "Status: BUSY (code=11, detail=4)\n"
            ),
        }

        self.assertFalse(run_i2c_hil.row_is_reset_nvm_busy(row))

    def test_status_result_parser_accepts_indented_canonical_status(self) -> None:
        evidence = run_i2c_hil.extract_parsed_evidence(
            "  Status: BUSY (code=11, detail=5)\n  Message: BUSY\n"
        )

        self.assertEqual("BUSY", evidence["status_name"])
        self.assertEqual(11, evidence["status_code"])
        self.assertEqual(5, evidence["status_detail"])

    def test_parser_self_test_passes(self) -> None:
        ok, failures = run_i2c_hil.parser_self_test()

        self.assertTrue(ok)
        self.assertEqual([], failures)

    def test_timeout_s_alias_sets_default_timeout(self) -> None:
        args = run_i2c_hil.parse_args(["--dry-run", "--timeout-s", "2.5"])

        self.assertEqual(2.5, args.timeout)

    def test_positive_timeout_parser_rejects_zero_negative_and_nonfinite(self) -> None:
        for value in ("0", "-1", "nan", "inf", "-inf"):
            with self.subTest(value=value), self.assertRaises(
                run_i2c_hil.argparse.ArgumentTypeError
            ):
                run_i2c_hil.parse_positive_float(value)
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            run_i2c_hil.parse_args(["--dry-run", "--timeout", "inf"])

    def test_baud_parser_is_strictly_positive_and_bounded(self) -> None:
        self.assertEqual(115200, run_i2c_hil.parse_baud("115200"))
        self.assertEqual(
            run_i2c_hil.MAX_SERIAL_BAUD,
            run_i2c_hil.parse_baud(str(run_i2c_hil.MAX_SERIAL_BAUD)),
        )
        for value in ("0", "-1", str(run_i2c_hil.MAX_SERIAL_BAUD + 1), "nan"):
            with self.subTest(value=value), self.assertRaises(
                run_i2c_hil.argparse.ArgumentTypeError
            ):
                run_i2c_hil.parse_baud(value)
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            run_i2c_hil.parse_args(["--dry-run", "--baud", "0"])

    def test_hil_counts_are_bounded_before_plan_allocation(self) -> None:
        huge = "9" * 401
        for parser, value in (
            (run_i2c_hil.parse_positive_int, huge),
            (run_i2c_hil.parse_positive_int, "100001"),
            (run_i2c_hil.parse_normal_soak_count, "10001"),
            (run_i2c_hil.parse_reconnect_attempts, "101"),
        ):
            with self.subTest(parser=parser.__name__, value=value), self.assertRaises(
                run_i2c_hil.argparse.ArgumentTypeError
            ):
                parser(value)

    def test_normal_soak_interval_has_a_separate_serial_timeout_window(self) -> None:
        specs = run_i2c_hil.normal_soak_commands(2, 20.0)
        delayed_read = specs[2]

        self.assertEqual(20.0, delayed_read.pre_delay_s)
        self.assertEqual(32.0, delayed_read.timeout_s)

    def test_duration_soak_cycle_has_bounded_safe_mix(self) -> None:
        args = run_i2c_hil.parse_args(["--dry-run", "--soak-duration-s", "10"])

        commands = [spec.command for spec in run_i2c_hil.duration_soak_cycle_commands(args, 1)]

        self.assertIn("stress 50", commands)
        self.assertIn("stress_mix 70", commands)
        self.assertIn("status", commands)
        self.assertIn("drv", commands)
        self.assertNotIn("wreg 0xF4 0x00", commands)

    def test_default_plan_covers_every_settings_query_and_verbose_setter(self) -> None:
        args = run_i2c_hil.parse_args(["--dry-run"])
        executable, _ = run_i2c_hil.build_command_sequence(args)
        specs = {
            spec.command: spec
            for spec in executable
            if spec.group == "config-query"
        }

        self.assertIn("Active I2C address:", specs["addr"].expected)
        self.assertEqual(
            ("Chip mode:", "Internal mode:"), specs["normal"].expected
        )
        self.assertEqual(
            ("Chip mode:", "Internal mode:"), specs["mode"].expected
        )
        self.assertEqual(
            ("Chip osrs:", "Internal osrs:"), specs["osrs"].expected
        )
        self.assertEqual(
            ("Chip filter:", "Internal filter:"), specs["filter"].expected
        )
        self.assertEqual(
            ("Chip standby:", "Internal standby:"), specs["standby"].expected
        )
        self.assertIn("Chip Settings", specs["settings"].expected)
        commands = [spec.command for spec in executable]
        verbose_on = commands.index("verbose 1")
        self.assertEqual("xfer_reset", commands[verbose_on - 1])
        self.assertEqual("verbose 0", commands[verbose_on + 1])
        self.assertEqual("xfer_assert 0 0 0", commands[verbose_on + 2])

    def test_default_plan_exercises_maximum_legal_register_dump(self) -> None:
        args = run_i2c_hil.parse_args(["--dry-run"])
        executable, _ = run_i2c_hil.build_command_sequence(args)
        commands = [spec.command for spec in executable]
        index = commands.index("dump 0x80 32")

        self.assertEqual("xfer_reset", commands[index - 1])
        self.assertEqual("xfer_assert 1 0 1", commands[index + 1])

    def test_include_job_api_adds_job_command_group(self) -> None:
        args = run_i2c_hil.parse_args(["--dry-run", "--include-job-api"])
        executable, _ = run_i2c_hil.build_command_sequence(args)
        job_commands = [spec.command for spec in executable if spec.group == "job-api"]

        self.assertEqual(
            [
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
            ],
            job_commands,
        )

    def test_job_done_or_failed_validator_passes_done(self) -> None:
        spec = run_i2c_hil.CommandSpec(
            command="job force 1",
            purpose="job parser test",
            expected=("Job Status", "Job state: DONE", "Status: OK"),
            validators=(run_i2c_hil.VALIDATOR_JOB_DONE_OR_FAILED,),
        )
        output = (
            "=== Job Status ===\n"
            "Job kind: FORCED_MEASUREMENT\n"
            "Job state: DONE\n"
            "Status: OK (code=0, detail=0)\n"
            "Callbacks used: 1\n"
            "Instructions: 1\n"
            "Driver: READY\n"
            "Consecutive failures: 0\n"
        )

        result, reason = run_i2c_hil.classify_output(spec, output, "MATCHED_COMPLETION")

        self.assertEqual(run_i2c_hil.RESULT_PASS, result)
        self.assertIn("job state=DONE", reason)

    def test_job_zero_consecutive_failures_validator_fails_nonzero(self) -> None:
        spec = run_i2c_hil.CommandSpec(
            command="job resync 1",
            purpose="job parser test",
            expected=("Job Status",),
            validators=(run_i2c_hil.VALIDATOR_JOB_ZERO_CONSECUTIVE_FAILURES,),
        )
        output = (
            "=== Job Status ===\n"
            "Job state: DONE\n"
            "Status: OK (code=0, detail=0)\n"
            "Callbacks used: 1\n"
            "Instructions: 1\n"
            "Driver: READY\n"
            "Consecutive failures: 1\n"
        )

        result, reason = run_i2c_hil.classify_output(spec, output, "MATCHED_COMPLETION")

        self.assertEqual(run_i2c_hil.RESULT_FAIL, result)
        self.assertIn("consecutive failures=1", reason)

    def test_job_instruction_budget_respected_fails_over_budget(self) -> None:
        spec = run_i2c_hil.CommandSpec(
            command="job force 1",
            purpose="job parser test",
            expected=("Job Status",),
            validators=(run_i2c_hil.VALIDATOR_JOB_INSTRUCTION_BUDGET_RESPECTED,),
        )
        output = (
            "=== Job Status ===\n"
            "Job state: DONE\n"
            "Status: OK (code=0, detail=0)\n"
            "Callbacks used: 2\n"
            "Instructions: 2\n"
            "Driver: READY\n"
            "Consecutive failures: 0\n"
        )

        result, reason = run_i2c_hil.classify_output(spec, output, "MATCHED_COMPLETION")

        self.assertEqual(run_i2c_hil.RESULT_FAIL, result)
        self.assertIn("budget was 1", reason)

    def test_job_command_budget_is_verb_aware(self) -> None:
        for command in (
            "job start init",
            "job cancel owner",
            "job cancel deadline",
            "job status",
        ):
            with self.subTest(command=command):
                self.assertEqual(0, run_i2c_hil.job_command_budget(command))

        self.assertEqual(0, run_i2c_hil.job_command_budget("job poll 0"))
        self.assertEqual(1, run_i2c_hil.job_command_budget("job poll 1"))
        self.assertEqual(3, run_i2c_hil.job_command_budget("job force 3"))
        self.assertEqual(
            run_i2c_hil.JOB_CLI_DEFAULT_BUDGET,
            run_i2c_hil.job_command_budget("job force malformed"),
        )

    def test_staged_cancel_zero_budget_exactly_once_fixtures(self) -> None:
        cases = (
            (
                "job start init",
                (run_i2c_hil.VALIDATOR_JOB_RESULT_FIELDS, run_i2c_hil.VALIDATOR_JOB_START_RUNNING),
                job_result_output(
                    boundary="START", job_id=17, kind="INIT", phase="INIT_READ_CHIP_ID",
                    state="RUNNING", status="IN_PROGRESS", status_code=12,
                    terminal=False, callbacks=0,
                ),
            ),
            (
                "job poll 1",
                (
                    run_i2c_hil.VALIDATOR_JOB_RESULT_FIELDS,
                    run_i2c_hil.VALIDATOR_JOB_ONE_CALLBACK,
                    run_i2c_hil.VALIDATOR_JOB_INSTRUCTION_BUDGET_RESPECTED,
                ),
                job_result_output(
                    boundary="POLL", job_id=17, kind="INIT", phase="NVM_POLL",
                    state="RUNNING", status="IN_PROGRESS", status_code=12,
                    terminal=False, callbacks=1, deadline_active=True, deadline_ms=1234,
                ),
            ),
            (
                "job cancel deadline",
                (run_i2c_hil.VALIDATOR_JOB_RESULT_FIELDS, run_i2c_hil.VALIDATOR_JOB_CANCEL_TIMED_OUT),
                job_result_output(
                    boundary="CANCEL", job_id=17, kind="INIT", phase="NVM_POLL",
                    state="TIMED_OUT", status="DEADLINE_EXPIRED", status_code=20,
                    terminal=True, callbacks=0,
                ),
            ),
            (
                "job poll 0",
                (
                    run_i2c_hil.VALIDATOR_JOB_RESULT_FIELDS,
                    run_i2c_hil.VALIDATOR_JOB_TIMED_OUT_RETRIEVAL,
                    run_i2c_hil.VALIDATOR_JOB_INSTRUCTION_BUDGET_RESPECTED,
                ),
                job_result_output(
                    boundary="POLL", job_id=17, kind="INIT", phase="NVM_POLL",
                    state="TIMED_OUT", status="DEADLINE_EXPIRED", status_code=20,
                    terminal=True, callbacks=0,
                ),
            ),
            (
                "job poll 0",
                (
                    run_i2c_hil.VALIDATOR_JOB_RESULT_FIELDS,
                    run_i2c_hil.VALIDATOR_JOB_IDLE_NO_RESULT,
                    run_i2c_hil.VALIDATOR_JOB_INSTRUCTION_BUDGET_RESPECTED,
                ),
                job_result_output(
                    boundary="POLL", job_id=0, kind="NONE", phase="NONE",
                    state="IDLE", status="OK", status_code=0,
                    terminal=False, callbacks=0,
                ),
            ),
        )
        for command, validators, output in cases:
            with self.subTest(command=command, state=run_i2c_hil.extract_parsed_evidence(output).get("job_state")):
                spec = run_i2c_hil.CommandSpec(
                    command=command,
                    purpose="staged cancellation fixture",
                    expected=("Job Status",),
                    validators=validators,
                )
                result, reason = run_i2c_hil.classify_output(spec, output, "MATCHED_COMPLETION")
                self.assertEqual(run_i2c_hil.RESULT_PASS, result, reason)

    def test_staged_cancel_sequence_correlates_identity_and_terminal(self) -> None:
        outputs = (
            job_result_output(
                boundary="START", job_id=17, kind="INIT", phase="INIT_READ_CHIP_ID",
                state="RUNNING", status="IN_PROGRESS", status_code=12,
                terminal=False, callbacks=0,
            ),
            job_result_output(
                boundary="POLL", job_id=17, kind="INIT", phase="NVM_POLL",
                state="RUNNING", status="IN_PROGRESS", status_code=12,
                terminal=False, callbacks=1, deadline_active=True, deadline_ms=1234,
            ),
            job_result_output(
                boundary="CANCEL", job_id=17, kind="INIT", phase="NVM_POLL",
                state="TIMED_OUT", status="DEADLINE_EXPIRED", status_code=20,
                terminal=True, callbacks=0,
            ),
            job_result_output(
                boundary="POLL", job_id=17, kind="INIT", phase="NVM_POLL",
                state="TIMED_OUT", status="DEADLINE_EXPIRED", status_code=20,
                terminal=True, callbacks=0,
            ),
            job_result_output(
                boundary="POLL", job_id=0, kind="NONE", phase="NONE",
                state="IDLE", status="OK", status_code=0,
                terminal=False, callbacks=0,
            ),
        )
        commands = (
            "job start init", "job poll 1", "job cancel deadline",
            "job poll 0", "job poll 0",
        )
        rows = [
            {
                "command": command,
                "group": "job-api",
                "serial_result": run_i2c_hil.RESULT_PASS,
                "classification_reason": "",
                "parsed_evidence": run_i2c_hil.extract_parsed_evidence(output),
            }
            for command, output in zip(commands, outputs)
        ]

        self.assertEqual(0, run_i2c_hil.reclassify_job_api_correlation(rows))
        self.assertTrue(all(row["serial_result"] == run_i2c_hil.RESULT_PASS for row in rows))

    def test_staged_cancel_sequence_rejects_mismatched_identity_and_terminal(self) -> None:
        def rows_with_evidence() -> list[dict]:
            base = (
                ("job start init", "START", 17, "INIT_READ_CHIP_ID", "RUNNING", "IN_PROGRESS", 12, False, 0),
                ("job poll 1", "POLL", 17, "NVM_POLL", "RUNNING", "IN_PROGRESS", 12, False, 1),
                ("job cancel deadline", "CANCEL", 17, "NVM_POLL", "TIMED_OUT", "DEADLINE_EXPIRED", 20, True, 0),
                ("job poll 0", "POLL", 17, "NVM_POLL", "TIMED_OUT", "DEADLINE_EXPIRED", 20, True, 0),
                ("job poll 0", "POLL", 0, "NONE", "IDLE", "OK", 0, False, 0),
            )
            return [
                {
                    "command": command,
                    "group": "job-api",
                    "serial_result": run_i2c_hil.RESULT_PASS,
                    "classification_reason": "",
                    "parsed_evidence": run_i2c_hil.extract_parsed_evidence(
                        job_result_output(
                            boundary=boundary, job_id=job_id,
                            kind="NONE" if job_id == 0 else "INIT", phase=phase,
                            state=state, status=status, status_code=status_code,
                            terminal=terminal, callbacks=callbacks,
                        )
                    ),
                }
                for command, boundary, job_id, phase, state, status,
                status_code, terminal, callbacks in base
            ]

        identity_rows = rows_with_evidence()
        identity_rows[1]["parsed_evidence"]["job_id"] = 18
        self.assertEqual(1, run_i2c_hil.reclassify_job_api_correlation(identity_rows))
        self.assertEqual(run_i2c_hil.RESULT_FAIL, identity_rows[3]["serial_result"])
        self.assertIn("IDs differ", identity_rows[3]["classification_reason"])

        terminal_rows = rows_with_evidence()
        terminal_rows[3]["parsed_evidence"]["job_phase"] = "CALIB_TP"
        self.assertEqual(1, run_i2c_hil.reclassify_job_api_correlation(terminal_rows))
        self.assertEqual(run_i2c_hil.RESULT_FAIL, terminal_rows[3]["serial_result"])
        self.assertIn("job_phase", terminal_rows[3]["classification_reason"])

    def test_require_pass_and_fail_on_review_exit_codes(self) -> None:
        self.assertEqual(
            0,
            run_i2c_hil.exit_code_for_verdict(
                "PASS",
                run_i2c_hil.argparse.Namespace(require_pass=True, fail_on_review=False),
            ),
        )
        self.assertEqual(
            3,
            run_i2c_hil.exit_code_for_verdict(
                "OPERATOR_REVIEW_REQUIRED",
                run_i2c_hil.argparse.Namespace(require_pass=True, fail_on_review=False),
            ),
        )
        self.assertEqual(
            3,
            run_i2c_hil.exit_code_for_verdict(
                "OPERATOR_REVIEW_REQUIRED",
                run_i2c_hil.argparse.Namespace(require_pass=False, fail_on_review=True),
            ),
        )

    def test_malformed_wreg_invalid_input_does_not_require_raw_write_confirmation(self) -> None:
        args = run_i2c_hil.parse_args(["--dry-run", "--include-invalid-inputs"])
        executable, _ = run_i2c_hil.build_command_sequence(args)

        errors = run_i2c_hil.execution_safety_errors(args, executable)

        self.assertEqual([], errors)


class HilEvidenceHardeningTest(unittest.TestCase):
    def test_trailing_argument_checks_require_usage_not_fallback_errors(self) -> None:
        for command, usage in (
            ("mode normal extra", "Usage: mode"),
            ("normal on extra", "Usage: normal on|off"),
            ("calib raw extra", "Usage: calib [raw]"),
        ):
            with self.subTest(command=command):
                spec = next(
                    item for item in run_i2c_hil.INVALID_INPUT_COMMANDS
                    if item.command == command
                )
                self.assertEqual((usage,), spec.expected)
                self.assertEqual((), spec.expected_any)

    def test_transfer_counter_output_is_parsed(self) -> None:
        evidence = run_i2c_hil.extract_parsed_evidence(
            "XFER_STATS read=2 write=4 total=6\n"
        )

        self.assertEqual(2, evidence["xfer_read"])
        self.assertEqual(4, evidence["xfer_write"])
        self.assertEqual(6, evidence["xfer_total"])

    def test_transfer_assert_fail_is_classified_as_failure(self) -> None:
        spec = run_i2c_hil.CommandSpec(
            command="xfer_assert 1 0 1",
            purpose="counter assertion",
            expected=("XFER_ASSERT PASS read=1 write=0 total=1",),
        )

        result, _ = run_i2c_hil.classify_output(
            spec,
            "XFER_ASSERT FAIL expected_read=1 expected_write=0 expected_total=1 "
            "actual_read=2 actual_write=0 actual_total=2\n",
            "MATCHED_COMPLETION",
        )

        self.assertEqual(run_i2c_hil.RESULT_FAIL, result)

    def test_expected_invalid_input_rejection_does_not_hide_other_failures(self) -> None:
        spec = next(
            item for item in run_i2c_hil.INVALID_INPUT_COMMANDS
            if item.expected_error
        )

        result, _ = run_i2c_hil.classify_output(
            spec, "Status: INVALID_PARAM (code=2, detail=0)\n", "MATCHED_COMPLETION"
        )
        self.assertEqual(run_i2c_hil.RESULT_PASS, result)

        for extra in ("[E] unexpected bus fault\n", "Status: I2C_TIMEOUT\n"):
            with self.subTest(extra=extra):
                result, _ = run_i2c_hil.classify_output(
                    spec,
                    "Status: INVALID_PARAM (code=2, detail=0)\n" + extra,
                    "MATCHED_COMPLETION",
                )
                self.assertEqual(run_i2c_hil.RESULT_FAIL, result)

    @staticmethod
    def version_row(output: str) -> dict:
        return {
            "command": "version",
            "group": "identity",
            "serial_result": run_i2c_hil.RESULT_PASS,
            "classification_reason": "Matched expected serial tokens.",
            "parsed_evidence": run_i2c_hil.extract_parsed_evidence(output),
        }

    def test_version_output_extracts_firmware_provenance(self) -> None:
        evidence = run_i2c_hil.extract_parsed_evidence(
            "BME280 library version: 2.0.0\n"
            "BME280 library commit: abc1234 (clean)\n"
        )

        self.assertEqual("2.0.0", evidence["library_version"])
        self.assertEqual("abc1234", evidence["library_commit"])
        self.assertEqual("clean", evidence["library_worktree"])

    def test_full_version_output_yields_stable_reconnect_identity(self) -> None:
        evidence = run_i2c_hil.extract_parsed_evidence(
            "BME280 library version: 2.0.0\n"
            "BME280 library full: 2.0.0 (abc1234, 2026-08-04 12:00:00, dirty)\n"
            "BME280 library build: 2026-08-04 12:00:00\n"
            "BME280 library commit: abc1234 (dirty)\n"
        )
        expected = self.firmware_identity_values()

        self.assertEqual(expected, run_i2c_hil.firmware_identity(evidence))
        self.assertEqual(expected, run_i2c_hil.initial_firmware_identity([{
            "command": "version",
            "group": "provenance",
            "parsed_evidence": evidence,
        }]))

    def test_matching_clean_firmware_provenance_remains_pass(self) -> None:
        rows = [self.version_row(
            "BME280 library version: 2.0.0\n"
            "BME280 library full: 2.0.0 (abc1234, 2026-08-04 12:00:00, clean)\n"
            "BME280 library build: 2026-08-04 12:00:00\n"
            "BME280 library commit: abc1234 (clean)\n"
        )]
        provenance = {"branch": "main", "commit": "abc123456789", "worktree": "clean"}

        count = run_i2c_hil.reclassify_version_provenance(
            rows,
            expected_version="2.0.0",
            start_provenance=provenance,
            end_provenance=provenance,
        )

        self.assertEqual(0, count)
        self.assertEqual(run_i2c_hil.RESULT_PASS, rows[0]["serial_result"])

    def test_stale_firmware_provenance_fails(self) -> None:
        rows = [self.version_row(
            "BME280 library version: 1.7.0\n"
            "BME280 library commit: deadbee (clean)\n"
        )]
        provenance = {"branch": "main", "commit": "abc123456789", "worktree": "clean"}

        count = run_i2c_hil.reclassify_version_provenance(
            rows,
            expected_version="2.0.0",
            start_provenance=provenance,
            end_provenance=provenance,
        )

        self.assertGreaterEqual(count, 2)
        self.assertEqual(run_i2c_hil.RESULT_FAIL, rows[0]["serial_result"])

    def test_dirty_or_changed_repository_requires_review(self) -> None:
        rows = [self.version_row(
            "BME280 library version: 2.0.0\n"
            "BME280 library commit: abc1234 (dirty)\n"
        )]
        start = {"branch": "main", "commit": "abc123456789", "worktree": "dirty"}
        end = {"branch": "review", "commit": "abc123456789", "worktree": "dirty"}

        count = run_i2c_hil.reclassify_version_provenance(
            rows,
            expected_version="2.0.0",
            start_provenance=start,
            end_provenance=end,
        )

        self.assertEqual(4, count)
        self.assertEqual(run_i2c_hil.RESULT_REVIEW, rows[0]["serial_result"])

    def test_missing_full_build_identity_requires_review(self) -> None:
        rows = [self.version_row(
            "BME280 library version: 2.0.0\n"
            "BME280 library commit: abc1234 (clean)\n"
        )]
        provenance = {"branch": "main", "commit": "abc123456", "worktree": "clean"}

        count = run_i2c_hil.reclassify_version_provenance(
            rows,
            expected_version="2.0.0",
            start_provenance=provenance,
            end_provenance=provenance,
        )

        self.assertEqual(2, count)
        self.assertEqual(run_i2c_hil.RESULT_REVIEW, rows[0]["serial_result"])

    def test_config_matrix_restores_forced_policy(self) -> None:
        args = run_i2c_hil.parse_args(["--dry-run", "--include-config-matrix"])
        executable, _ = run_i2c_hil.build_command_sequence(args)
        commands = [spec.command for spec in executable if spec.group == "config-matrix"]

        self.assertEqual(["mode forced", "cfg"], commands[-2:])

    def test_named_setter_profiles_have_exact_cfg_readback(self) -> None:
        specs = list(run_i2c_hil.CONFIG_MATRIX_COMMANDS)
        for command in (
            "mode sleep", "osrs t x2", "osrs p x2", "osrs h x2",
            "filter x2", "standby 250ms",
        ):
            matching = [index for index, spec in enumerate(specs) if spec.command == command]
            self.assertTrue(matching, command)
            profile_index = next(
                index for index in matching
                if index + 2 < len(specs) and specs[index + 1].command.startswith("xfer_assert")
            )
            readback = specs[profile_index + 2]
            self.assertEqual("cfg", readback.command)
            self.assertTrue(readback.expected_settings)

    def test_cfg_parser_and_matrix_cover_all_three_internal_modes(self) -> None:
        evidence = run_i2c_hil.extract_parsed_evidence(
            "=== Internal Settings ===\n  Mode: NORMAL\n"
        )
        self.assertEqual(3, evidence["mode"])

        expected_modes = {
            value
            for spec in run_i2c_hil.CONFIG_MATRIX_COMMANDS
            if spec.command == "cfg"
            for name, value in spec.expected_settings
            if name == "mode"
        }
        self.assertEqual({0, 1, 3}, expected_modes)

    def test_config_matrix_covers_all_legal_enum_values_with_readback(self) -> None:
        specs = list(run_i2c_hil.CONFIG_MATRIX_COMMANDS)
        commands = [spec.command for spec in specs]

        for value in range(1, 6):
            self.assertIn(f"osrs t {value}", commands)
        for channel in ("p", "h"):
            for value in range(0, 6):
                self.assertIn(f"osrs {channel} {value}", commands)
        for value in range(0, 5):
            self.assertIn(f"filter {value}", commands)
        for value in range(0, 8):
            self.assertIn(f"standby {value}", commands)

        for index, spec in enumerate(specs[:-1]):
            if re.fullmatch(r"(?:osrs [tph]|filter|standby) \d+", spec.command):
                # Explicit restoration writes can be followed by another write;
                # every exercise write has a cfg row carrying exact expectations.
                if "Restore" not in spec.purpose and "Select maximum" not in spec.purpose:
                    self.assertEqual("cfg", specs[index + 1].command)
                    self.assertTrue(specs[index + 1].expected_settings)

    def test_cfg_exact_readback_validator_passes_and_detects_mismatch(self) -> None:
        spec = run_i2c_hil.CommandSpec(
            command="cfg",
            purpose="settings parser",
            expected=("ctrl_hum", "ctrl_meas", "config", "Hardware config dirty:"),
            expected_settings=(("osrs_t", 5), ("osrs_p", 3), ("osrs_h", 2),
                               ("filter", 4), ("standby", 7)),
        )
        output = (
            "ctrl_hum: 0x02 (osrs_h=2 X2)\n"
            "ctrl_meas: 0xAC (osrs_t=5 X16, osrs_p=3 X4, mode=0 SLEEP)\n"
            "config: 0xF0 (standby=7 20ms, filter=4 X16, spi3w_en=0)\n"
            "Hardware config dirty: false\n"
        )

        result, reason = run_i2c_hil.classify_output(
            spec, output, "MATCHED_COMPLETION"
        )
        self.assertEqual(run_i2c_hil.RESULT_PASS, result)
        self.assertIn("standby=7", reason)

        mismatch = run_i2c_hil.dataclasses.replace(
            spec, expected_settings=(("filter", 3),)
        )
        result, reason = run_i2c_hil.classify_output(
            mismatch, output, "MATCHED_COMPLETION"
        )
        self.assertEqual(run_i2c_hil.RESULT_FAIL, result)
        self.assertIn("filter=4, expected 3", reason)

    def test_duration_state_changes_are_grouped_with_recovery(self) -> None:
        args = run_i2c_hil.parse_args(
            ["--dry-run", "--soak-reset-interval", "20"]
        )
        groups = run_i2c_hil.duration_soak_command_groups(args, 140)
        commands = [[spec.command for spec in group] for group in groups]

        self.assertIn(["normal on", "read", "normal off"], commands)
        self.assertIn(["reset", "recover"], commands)
        self.assertTrue(all(
            run_i2c_hil.duration_group_timeout_s(group, args) > 0.0
            for group in groups
        ))

    def test_initial_serial_open_retries_are_preserved_in_evidence(self) -> None:
        args = run_i2c_hil.parse_args([
            "--port", "COM10", "--reconnect-attempts", "2",
            "--reconnect-delay-s", "0",
        ])
        opened = mock.Mock()
        serial_module = mock.Mock()
        serial_module.Serial.side_effect = [
            OSError("port busy"), OSError("USB settling"), opened,
        ]
        transcript = io.StringIO()

        with mock.patch.object(
            run_i2c_hil.time, "monotonic",
            side_effect=[0.0, 1.0, 2.0, 3.0, 5.0, 6.0, 9.0],
        ), mock.patch.object(run_i2c_hil.time, "sleep"):
            outcome = run_i2c_hil.open_serial_with_retries(
                serial_module, args, transcript
            )

        self.assertIs(opened, outcome.serial)
        self.assertEqual(3, outcome.attempts)
        self.assertEqual(2, outcome.failed_attempts)
        self.assertEqual(9.0, outcome.downtime_s)
        self.assertEqual(2, len(outcome.rows))
        self.assertTrue(all(
            row["serial_result"] == run_i2c_hil.RESULT_REVIEW
            for row in outcome.rows
        ))
        summary = run_i2c_hil.summarize_serial_open(outcome)
        self.assertEqual(3, summary["attempts"])
        self.assertEqual(2, summary["failed_attempts"])
        self.assertEqual(9.0, summary["downtime_s"])
        self.assertIn("SERIAL OPEN ATTEMPT FAILED", transcript.getvalue())
        self.assertIn("SERIAL OPEN SUCCEEDED attempt=3/3", transcript.getvalue())

    def test_exhausted_initial_serial_open_is_a_hard_failure(self) -> None:
        args = run_i2c_hil.parse_args([
            "--port", "COM10", "--reconnect-attempts", "1",
            "--reconnect-delay-s", "0",
        ])
        serial_module = mock.Mock()
        serial_module.Serial.side_effect = [
            OSError("still busy"), OSError("still unavailable"),
        ]

        with mock.patch.object(
            run_i2c_hil.time, "monotonic",
            side_effect=[0.0, 0.0, 1.0, 1.0, 2.0, 2.0],
        ), mock.patch.object(run_i2c_hil.time, "sleep"):
            outcome = run_i2c_hil.open_serial_with_retries(
                serial_module, args, io.StringIO()
            )

        self.assertIsNone(outcome.serial)
        self.assertEqual(2, outcome.failed_attempts)
        self.assertEqual(
            run_i2c_hil.RESULT_FAIL, outcome.rows[-1]["serial_result"]
        )
        self.assertEqual(
            "INITIAL_SERIAL_OPEN_EXHAUSTED", outcome.rows[-1]["completion"]
        )
        self.assertEqual(
            "FAIL", run_i2c_hil.final_verdict(outcome.rows, dry_run=False)
        )

    @staticmethod
    def firmware_identity_values(suffix: str = "") -> dict[str, str]:
        return {
            "library_version": f"2.0.0{suffix}",
            "library_full": f"2.0.0 (abc1234, 2026-08-04 12:00:00, dirty){suffix}",
            "library_build": f"2026-08-04 12:00:00{suffix}",
            "library_commit": f"abc1234{suffix}",
            "library_worktree": "dirty",
        }

    @classmethod
    def serial_failure_row(
        cls, spec: run_i2c_hil.CommandSpec, message: str = "link lost"
    ) -> dict:
        row = cls.command_row(spec, run_i2c_hil.RESULT_FAIL)
        row.update({
            "operator_result": "",
            "completion": "SERIAL_READ_EXCEPTION",
            "elapsed_s": 0.25,
            "classification_reason": message,
            "serial_exception": message,
            "output_excerpt": "partial output",
        })
        return row

    def test_reconnect_reopens_same_serial_object_and_proves_identity(self) -> None:
        args = run_i2c_hil.parse_args([
            "--dry-run", "--reconnect-attempts", "2", "--reconnect-delay-s", "0",
            "--boot-settle-s", "0",
        ])
        serial = mock.Mock()
        probe_row = {
            "command": "version",
            "group": "serial-reconnect",
            "serial_result": run_i2c_hil.RESULT_PASS,
            "completion": "MATCHED_COMPLETION",
            "classification_reason": "matched",
            "parsed_evidence": self.firmware_identity_values(),
        }
        with mock.patch.object(
            run_i2c_hil, "read_available", return_value=""
        ), mock.patch.object(
            run_i2c_hil, "run_serial_command", return_value=probe_row
        ) as run:
            outcome = run_i2c_hil.reconnect_serial_in_place(
                serial,
                io.StringIO(),
                args,
                max_attempts=2,
                expected_identity=self.firmware_identity_values(),
                probe_timeout_s=5.0,
            )

        self.assertTrue(outcome.recovered)
        self.assertEqual(1, outcome.attempts)
        self.assertEqual([probe_row], outcome.rows)
        self.assertEqual("", outcome.reason)
        serial.close.assert_called_once()
        serial.open.assert_called_once()
        self.assertEqual(run_i2c_hil.SERIAL_READ_TIMEOUT_S, serial.timeout)
        self.assertEqual(run_i2c_hil.SERIAL_WRITE_TIMEOUT_S, serial.write_timeout)
        self.assertEqual("version", run.call_args.args[1].command)
        self.assertEqual(0.0, run.call_args.kwargs["command_pacing_s"])

    def test_zero_reconnect_budget_leaves_serial_handle_untouched(self) -> None:
        args = run_i2c_hil.parse_args([
            "--dry-run", "--reconnect-attempts", "0",
        ])
        serial = mock.Mock()

        outcome = run_i2c_hil.reconnect_serial_in_place(
            serial,
            io.StringIO(),
            args,
            max_attempts=0,
            expected_identity=self.firmware_identity_values(),
            probe_timeout_s=5.0,
        )

        self.assertFalse(outcome.recovered)
        self.assertEqual(0, outcome.attempts)
        self.assertEqual(0.0, outcome.downtime_s)
        self.assertEqual([], outcome.rows)
        self.assertEqual("reconnect attempt budget is zero", outcome.reason)
        serial.close.assert_not_called()
        serial.open.assert_not_called()

    def test_reconnect_rejects_different_full_firmware_identity(self) -> None:
        args = run_i2c_hil.parse_args([
            "--dry-run", "--reconnect-delay-s", "0", "--boot-settle-s", "0",
        ])
        probe_row = {
            "command": "version",
            "group": "serial-reconnect",
            "serial_result": run_i2c_hil.RESULT_PASS,
            "completion": "MATCHED_COMPLETION",
            "classification_reason": "matched labels",
            "parsed_evidence": self.firmware_identity_values("-different"),
        }
        with mock.patch.object(
            run_i2c_hil, "read_available", return_value=""
        ), mock.patch.object(
            run_i2c_hil, "run_serial_command", return_value=probe_row
        ):
            outcome = run_i2c_hil.reconnect_serial_in_place(
                mock.Mock(),
                io.StringIO(),
                args,
                max_attempts=2,
                expected_identity=self.firmware_identity_values(),
                probe_timeout_s=5.0,
            )

        self.assertFalse(outcome.recovered)
        self.assertEqual(1, outcome.attempts)
        self.assertEqual(1, len(outcome.rows))
        self.assertEqual(run_i2c_hil.RESULT_FAIL, outcome.rows[0]["serial_result"])
        self.assertEqual(
            "FIRMWARE_IDENTITY_MISMATCH", outcome.rows[0]["completion"]
        )
        self.assertIn("library_full", outcome.reason)

    def test_reconnect_preserves_failed_probe_before_later_success(self) -> None:
        args = run_i2c_hil.parse_args([
            "--dry-run", "--reconnect-delay-s", "0", "--boot-settle-s", "0",
        ])
        timed_out = {
            "command": "version",
            "group": "serial-reconnect",
            "serial_result": run_i2c_hil.RESULT_TIMEOUT,
            "operator_result": "",
            "completion": run_i2c_hil.RESULT_TIMEOUT,
            "classification_reason": "no response",
            "parsed_evidence": {},
        }
        passed = {
            "command": "version",
            "group": "serial-reconnect",
            "serial_result": run_i2c_hil.RESULT_PASS,
            "operator_result": "",
            "completion": "MATCHED_COMPLETION",
            "classification_reason": "matched",
            "parsed_evidence": self.firmware_identity_values(),
        }
        serial = mock.Mock()
        with mock.patch.object(
            run_i2c_hil, "read_available", return_value=""
        ), mock.patch.object(
            run_i2c_hil, "run_serial_command", side_effect=[timed_out, passed]
        ):
            outcome = run_i2c_hil.reconnect_serial_in_place(
                serial,
                io.StringIO(),
                args,
                max_attempts=2,
                expected_identity=self.firmware_identity_values(),
                probe_timeout_s=5.0,
            )

        self.assertTrue(outcome.recovered)
        self.assertEqual(2, outcome.attempts)
        self.assertEqual(2, len(outcome.rows))
        self.assertEqual(run_i2c_hil.RESULT_REVIEW, outcome.rows[0]["serial_result"])
        self.assertEqual(run_i2c_hil.RESULT_PASS, outcome.rows[1]["serial_result"])
        self.assertEqual(2, serial.open.call_count)

    def test_duration_serial_exception_preserves_partial_failure_evidence(self) -> None:
        args = run_i2c_hil.parse_args([
            "--dry-run", "--soak-duration-s", "10", "--reconnect-attempts", "0"
        ])
        spec = run_i2c_hil.CommandSpec(
            command="stress 1", purpose="exception evidence", timeout_s=1.0
        )
        failure = self.serial_failure_row(spec)
        with mock.patch.object(
            run_i2c_hil, "duration_soak_command_groups", return_value=((spec,),)
        ), mock.patch.object(
            run_i2c_hil, "run_serial_command", return_value=failure
        ):
            rows, summary = run_i2c_hil.run_duration_soak(
                mock.Mock(), io.StringIO(), args
            )

        self.assertEqual(1, len(rows))
        self.assertEqual("stress 1", rows[0]["command"])
        self.assertEqual(run_i2c_hil.RESULT_FAIL, rows[0]["serial_result"])
        self.assertEqual("SERIAL_READ_EXCEPTION", rows[0]["completion"])
        self.assertEqual("partial output", rows[0]["output_excerpt"])
        self.assertTrue(summary["executed"])
        self.assertEqual(0, summary["cycles"])
        self.assertEqual(1, summary["cycles_started"])
        self.assertIn("link lost", summary["stop_reason"])

    def test_duration_too_short_for_any_safe_group_is_a_hard_failure(self) -> None:
        args = run_i2c_hil.parse_args([
            "--dry-run", "--soak-duration-s", "0.001"
        ])

        rows, summary = run_i2c_hil.run_duration_soak(
            mock.Mock(), io.StringIO(), args
        )

        self.assertEqual(1, len(rows))
        self.assertEqual("DURATION_SOAK", rows[0]["command"])
        self.assertEqual(run_i2c_hil.RESULT_FAIL, rows[0]["serial_result"])
        self.assertEqual("NO_SAFE_GROUP_FIT", rows[0]["completion"])
        self.assertEqual(0, summary["cycles"])
        self.assertEqual("FAIL", run_i2c_hil.final_verdict(rows, dry_run=False))

    def test_duration_partial_safe_groups_without_a_full_cycle_fail(self) -> None:
        args = run_i2c_hil.parse_args([
            "--dry-run", "--soak-duration-s", "3"
        ])
        first = run_i2c_hil.CommandSpec(
            command="stress 1", purpose="first safe group", timeout_s=1.0
        )
        second = run_i2c_hil.CommandSpec(
            command="stress_mix 1", purpose="second safe group", timeout_s=1.0
        )
        with mock.patch.object(
            run_i2c_hil, "duration_soak_command_groups",
            return_value=((first,), (second,)),
        ), mock.patch.object(
            run_i2c_hil, "run_serial_command",
            return_value=self.command_row(first),
        ), mock.patch.object(
            run_i2c_hil.time, "monotonic",
            side_effect=[0.0, 0.0, 0.0, 0.0, 2.5, 2.5, 2.5],
        ):
            rows, summary = run_i2c_hil.run_duration_soak(
                mock.Mock(), io.StringIO(), args
            )

        self.assertEqual(["stress 1", "DURATION_SOAK"], [
            row["command"] for row in rows
        ])
        self.assertEqual("NO_COMPLETE_SOAK_CYCLE", rows[-1]["completion"])
        self.assertIn("no full duration-soak cycle", rows[-1]["classification_reason"])
        self.assertEqual(0, summary["cycles"])
        self.assertEqual("FAIL", run_i2c_hil.final_verdict(rows, dry_run=False))

    def test_recovered_duration_link_replays_safe_group_from_start(self) -> None:
        args = run_i2c_hil.parse_args([
            "--dry-run", "--soak-duration-s", "10", "--reconnect-attempts", "1"
        ])
        spec = run_i2c_hil.CommandSpec(
            command="stress 1", purpose="replay", timeout_s=1.0
        )
        failure = self.serial_failure_row(spec, "transient link")
        hard_row = self.command_row(spec, run_i2c_hil.RESULT_FAIL)
        probe_row = self.command_row(run_i2c_hil.CommandSpec(
            command="version", purpose="identity", group="serial-reconnect"
        ))
        with mock.patch.object(
            run_i2c_hil, "duration_soak_command_groups", return_value=((spec,),)
        ), mock.patch.object(
            run_i2c_hil, "run_serial_command",
            side_effect=[failure, hard_row],
        ) as run, mock.patch.object(
            run_i2c_hil, "reconnect_serial_in_place",
            return_value=run_i2c_hil.ReconnectOutcome(
                True, 1, 2.0, [probe_row]
            ),
        ):
            rows, summary = run_i2c_hil.run_duration_soak(
                object(), io.StringIO(), args
            )

        self.assertEqual(2, run.call_count)
        self.assertEqual(
            ["stress 1", "version", "SERIAL_RECONNECT", "stress 1"],
            [row["command"] for row in rows],
        )
        self.assertEqual(run_i2c_hil.RESULT_REVIEW, rows[0]["serial_result"])
        self.assertEqual(
            "RECONNECTED_AND_REPLAYING_GROUP", rows[2]["completion"]
        )
        self.assertTrue(rows[2]["parsed_evidence"]["safe_group_replayed"])
        self.assertEqual(1, summary["reconnect_count"])
        self.assertEqual(1, summary["reconnect_attempts_used"])
        self.assertEqual(2.0, summary["reconnect_downtime_s"])
        self.assertGreaterEqual(summary["deadline_extension_s"], 2.0)

    def test_reconnect_never_downgrades_proven_partial_device_failure(self) -> None:
        args = run_i2c_hil.parse_args([
            "--dry-run", "--soak-duration-s", "10", "--reconnect-attempts", "1"
        ])
        spec = run_i2c_hil.CommandSpec(
            command="normal on", purpose="retain failure", timeout_s=1.0,
            recovery_command="normal off",
        )
        recovery = run_i2c_hil.CommandSpec(
            command="normal off", purpose="restore safe state", timeout_s=1.0,
        )
        failure = self.serial_failure_row(spec, "transient link")
        failure.update({
            "partial_output_result": run_i2c_hil.RESULT_FAIL,
            "partial_output_reason": "stress errors=1, expected zero",
            "parsed_evidence": {"stress_errors": 1},
            "output_excerpt": "=== Stress Summary ===\nErrors: 1\n",
        })
        def fake_command(_ser, actual_spec, _transcript, **_kwargs):
            if actual_spec.command == "normal on":
                return failure
            return self.command_row(actual_spec)

        with mock.patch.object(
            run_i2c_hil, "duration_soak_command_groups",
            return_value=((spec, recovery),),
        ), mock.patch.object(
            run_i2c_hil, "run_serial_command",
            side_effect=fake_command,
        ) as run, mock.patch.object(
            run_i2c_hil, "reconnect_serial_in_place",
            return_value=run_i2c_hil.ReconnectOutcome(True, 1, 0.1, []),
        ):
            rows, summary = run_i2c_hil.run_duration_soak(
                object(), io.StringIO(), args
            )

        self.assertEqual(run_i2c_hil.RESULT_FAIL, rows[0]["serial_result"])
        self.assertIn("hard failure is retained", rows[0]["classification_reason"])
        self.assertIn("will not be replayed", rows[0]["classification_reason"])
        self.assertEqual(["normal on", "normal off"], [
            call.args[1].command for call in run.call_args_list
        ])
        self.assertEqual(
            "RECONNECTED_FOR_EVIDENCE_NO_REPLAY", rows[1]["completion"]
        )
        self.assertFalse(rows[1]["parsed_evidence"]["safe_group_replayed"])
        self.assertEqual("automatic-recovery", rows[-1]["group"])
        self.assertIn("proven partial FAIL", summary["stop_reason"])
        self.assertEqual(0.0, summary["replay_extension_s"])
        self.assertEqual(0.0, summary["deadline_extension_s"])
        self.assertEqual("FAIL", run_i2c_hil.final_verdict(rows, dry_run=False))

    def test_replay_accounting_excludes_only_actual_discarded_elapsed(self) -> None:
        args = run_i2c_hil.parse_args([
            "--dry-run", "--soak-duration-s", "10",
            "--reconnect-attempts", "1",
        ])
        spec = run_i2c_hil.CommandSpec(
            command="stress 1", purpose="replay accounting", timeout_s=1.0
        )
        interrupted = self.serial_failure_row(spec, "transient link")
        replay_failure = self.command_row(spec, run_i2c_hil.RESULT_FAIL)
        with mock.patch.object(
            run_i2c_hil, "duration_soak_command_groups", return_value=((spec,),)
        ), mock.patch.object(
            run_i2c_hil, "run_serial_command",
            side_effect=[interrupted, replay_failure],
        ), mock.patch.object(
            run_i2c_hil, "reconnect_serial_in_place",
            return_value=run_i2c_hil.ReconnectOutcome(True, 1, 2.0, []),
        ), mock.patch.object(
            run_i2c_hil.time, "monotonic",
            side_effect=[0.0, 0.0, 0.0, 0.0, 5.0, 16.5, 16.5, 17.0, 17.0],
        ):
            _, summary = run_i2c_hil.run_duration_soak(
                object(), io.StringIO(), args
            )

        self.assertEqual(5.0, summary["replay_extension_s"])
        self.assertEqual(7.5, summary["deadline_extension_s"])
        self.assertEqual(10.0, summary["active_elapsed_s"])

    def test_duration_automatic_recovery_exception_keeps_completed_rows(self) -> None:
        args = run_i2c_hil.parse_args([
            "--dry-run", "--soak-duration-s", "10"
        ])
        start = run_i2c_hil.CommandSpec(
            command="normal on", purpose="state change", timeout_s=1.0,
            recovery_command="normal off",
        )
        recovery = run_i2c_hil.CommandSpec(
            command="normal off", purpose="restore", timeout_s=1.0,
        )
        root_failure = self.command_row(start, run_i2c_hil.RESULT_FAIL)
        with mock.patch.object(
            run_i2c_hil,
            "duration_soak_command_groups",
            return_value=((start, recovery),),
        ), mock.patch.object(
            run_i2c_hil,
            "run_serial_command",
            side_effect=[root_failure, OSError("cleanup transcript failure")],
        ):
            rows, summary = run_i2c_hil.run_duration_soak(
                object(), io.StringIO(), args
            )

        self.assertEqual(["normal on", "normal off"], [r["command"] for r in rows])
        self.assertEqual(run_i2c_hil.RESULT_FAIL, rows[0]["serial_result"])
        self.assertEqual("automatic-recovery", rows[1]["group"])
        self.assertEqual(run_i2c_hil.RESULT_FAIL, rows[1]["serial_result"])
        self.assertIn("cleanup transcript failure", rows[1]["classification_reason"])
        self.assertEqual(0, summary["cycles"])

    def test_precommand_delay_cannot_send_after_deadline(self) -> None:
        for pre_delay_s, pacing_s in ((2.0, 0.0), (0.0, 2.0)):
            with self.subTest(pre_delay_s=pre_delay_s, pacing_s=pacing_s):
                serial = mock.Mock()
                spec = run_i2c_hil.CommandSpec(
                    command="read",
                    purpose="deadline",
                    pre_delay_s=pre_delay_s,
                    timeout_s=1.0,
                )

                row = run_i2c_hil.run_serial_command(
                    serial,
                    spec,
                    io.StringIO(),
                    timeout_s=1.0,
                    command_pacing_s=pacing_s,
                )

                self.assertEqual(
                    run_i2c_hil.RESULT_TIMEOUT, row["serial_result"]
                )
                self.assertFalse(row["command_sent"])
                serial.write.assert_not_called()

    def test_reset_waits_for_expected_any_completion_after_partial_output(self) -> None:
        class ChunkedSerial:
            def __init__(self) -> None:
                self.chunks = [
                    b"Reset requested; waiting for result\n",
                    b"Status: OK (code=0, detail=0)\n",
                ]

            @property
            def in_waiting(self) -> int:
                return len(self.chunks[0]) if self.chunks else 0

            def write(self, payload: bytes) -> int:
                return len(payload)

            def read(self, _size: int) -> bytes:
                return self.chunks.pop(0) if self.chunks else b""

        spec = next(
            item for item in run_i2c_hil.BASE_COMMANDS
            if item.command == "reset"
        )
        row = run_i2c_hil.run_serial_command(
            ChunkedSerial(),
            spec,
            io.StringIO(),
            timeout_s=1.0,
            idle_after_output_s=0.0,
            idle_after_match_s=0.0,
        )

        self.assertEqual(run_i2c_hil.RESULT_PASS, row["serial_result"])
        self.assertEqual("MATCHED_COMPLETION", row["completion"])
        self.assertIn("Status: OK", row["output_excerpt"])

    def test_serial_read_exception_preserves_partial_output_and_send_state(self) -> None:
        class PartialSerial:
            def __init__(self) -> None:
                self.waiting_reads = 0

            @property
            def in_waiting(self) -> int:
                self.waiting_reads += 1
                if self.waiting_reads == 1:
                    return len(b"partial output\n")
                raise OSError("ClearCommError failed")

            def write(self, payload: bytes) -> int:
                return len(payload)

            def read(self, _size: int) -> bytes:
                return b"partial output\n"

        row = run_i2c_hil.run_serial_command(
            PartialSerial(),
            run_i2c_hil.CommandSpec(
                command="stress 1", purpose="partial serial evidence",
                expected=("Stress Summary",), timeout_s=1.0,
            ),
            io.StringIO(),
            timeout_s=1.0,
        )

        self.assertEqual(run_i2c_hil.RESULT_FAIL, row["serial_result"])
        self.assertTrue(row["command_sent"])
        self.assertIn("partial output", row["output_excerpt"])
        self.assertIn("ClearCommError failed", row["serial_exception"])

    def test_nonnegative_float_parser_rejects_nonfinite_values(self) -> None:
        for value in ("nan", "inf", "+inf", "-inf"):
            with self.subTest(value=value), self.assertRaises(
                run_i2c_hil.argparse.ArgumentTypeError
            ):
                run_i2c_hil.parse_nonnegative_float(value)

    def test_declared_recovery_runs_without_pacing(self) -> None:
        args = run_i2c_hil.parse_args(["--dry-run", "--command-pacing-s", "9"])
        recovery = run_i2c_hil.CommandSpec(
            command="normal off", purpose="restore", timeout_s=5.0
        )
        fake_row = {
            "command": "normal off",
            "serial_result": run_i2c_hil.RESULT_PASS,
        }
        with mock.patch.object(
            run_i2c_hil, "run_serial_command", return_value=fake_row
        ) as run:
            row = run_i2c_hil.run_automatic_recovery(
                object(), io.StringIO(), args, recovery, "read"
            )

        cleanup_spec = run.call_args.args[1]
        self.assertEqual("automatic-recovery", cleanup_spec.group)
        self.assertEqual(0.0, run.call_args.kwargs["command_pacing_s"])
        self.assertTrue(row["automatic_recovery"])

    @staticmethod
    def command_row(
        spec: run_i2c_hil.CommandSpec, result: str = run_i2c_hil.RESULT_PASS
    ) -> dict:
        return {
            "command": spec.command,
            "purpose": spec.purpose,
            "group": spec.group,
            "serial_result": result,
            "command_sent": True,
            "parsed_evidence": {},
        }

    def test_normal_duration_stop_emits_ordered_final_cleanup(self) -> None:
        args = run_i2c_hil.parse_args([
            "--dry-run", "--soak-duration-s", "1", "--command-pacing-s", "9"
        ])
        fixed = run_i2c_hil.CommandSpec(command="version", purpose="fixed")
        soak_row = {
            "command": "stress 50",
            "group": "soak-duration",
            "serial_result": run_i2c_hil.RESULT_PASS,
        }
        soak_summary = {
            **run_i2c_hil.empty_soak_summary(args),
            "executed": True,
            "cycles": 3,
            "stop_reason": "deadline window too short for next safe group",
        }

        def fake_command(_ser, spec, _transcript, **_kwargs):
            return self.command_row(spec)

        with mock.patch.object(
            run_i2c_hil, "run_serial_command", side_effect=fake_command
        ) as run, mock.patch.object(
            run_i2c_hil, "run_duration_soak",
            return_value=([soak_row], soak_summary),
        ):
            rows, actual_soak = run_i2c_hil.run_live_plan(
                object(), io.StringIO(), args, [fixed]
            )

        cleanup = [row for row in rows if row.get("group") == "final-cleanup"]
        self.assertEqual(
            ["normal off", "recover", "cfg", "status", "drv"],
            [row["command"] for row in cleanup],
        )
        self.assertEqual(soak_summary, actual_soak)
        cleanup_calls = [
            call for call in run.call_args_list if call.args[1].group == "final-cleanup"
        ]
        self.assertEqual(5, len(cleanup_calls))
        self.assertTrue(all(
            call.kwargs["command_pacing_s"] == 0.0 for call in cleanup_calls
        ))

    def test_successful_fixed_plan_always_emits_final_cleanup(self) -> None:
        args = run_i2c_hil.parse_args(["--dry-run"])
        fixed = run_i2c_hil.CommandSpec(command="version", purpose="fixed")

        def fake_command(_ser, spec, _transcript, **_kwargs):
            return self.command_row(spec)

        with mock.patch.object(
            run_i2c_hil, "run_serial_command", side_effect=fake_command
        ), mock.patch.object(
            run_i2c_hil, "run_duration_soak",
            return_value=([], run_i2c_hil.empty_soak_summary(args)),
        ):
            rows, _ = run_i2c_hil.run_live_plan(
                object(), io.StringIO(), args, [fixed]
            )

        cleanup = [row for row in rows if row.get("group") == "final-cleanup"]
        self.assertEqual(
            ["normal off", "recover", "cfg", "status", "drv"],
            [row["command"] for row in cleanup],
        )
        self.assertTrue(all(
            row["cleanup_trigger"] == "completed fixed command plan"
            for row in cleanup
        ))

    def test_final_cleanup_pass_requires_safe_state_evidence(self) -> None:
        specs = {
            spec.command: spec for spec in run_i2c_hil.final_cleanup_commands()
        }

        self.assertIn("Hardware config dirty: false", specs["cfg"].expected)
        self.assertIn("dirty=false", specs["status"].expected)
        self.assertEqual(
            (
                run_i2c_hil.VALIDATOR_STATUS_NOT_MEASURING,
                run_i2c_hil.VALIDATOR_STATUS_IM_UPDATE_CLEAR,
            ),
            specs["status"].validators,
        )
        self.assertEqual(
            (run_i2c_hil.VALIDATOR_DRIVER_ZERO_CONSECUTIVE,),
            specs["drv"].validators,
        )

    def test_fixed_hard_stop_cleanup_continues_and_keeps_root_verdict(self) -> None:
        args = run_i2c_hil.parse_args([
            "--dry-run", "--command-pacing-s", "9"
        ])
        fixed = run_i2c_hil.CommandSpec(command="probe", purpose="fixed")

        def fake_command(_ser, spec, _transcript, **_kwargs):
            if spec.group != "final-cleanup":
                return self.command_row(spec, run_i2c_hil.RESULT_FAIL)
            if spec.command == "recover":
                raise OSError("link unavailable")
            return self.command_row(spec)

        with mock.patch.object(
            run_i2c_hil, "run_serial_command", side_effect=fake_command
        ) as run, mock.patch.object(
            run_i2c_hil, "run_duration_soak"
        ) as soak:
            rows, _ = run_i2c_hil.run_live_plan(
                object(), io.StringIO(), args, [fixed]
            )

        soak.assert_not_called()
        cleanup = [row for row in rows if row.get("group") == "final-cleanup"]
        self.assertEqual(5, len(cleanup))
        self.assertEqual(
            ["normal off", "recover", "cfg", "status", "drv"],
            [row["command"] for row in cleanup],
        )
        self.assertEqual(run_i2c_hil.RESULT_FAIL, cleanup[1]["serial_result"])
        self.assertEqual(
            "FAIL", run_i2c_hil.final_verdict(rows, dry_run=False)
        )
        cleanup_calls = [
            call for call in run.call_args_list if call.args[1].group == "final-cleanup"
        ]
        self.assertEqual(5, len(cleanup_calls))
        self.assertTrue(all(
            call.kwargs["command_pacing_s"] == 0.0 for call in cleanup_calls
        ))

    def test_final_cleanup_failure_prevents_clean_run_pass_verdict(self) -> None:
        rows = [
            {"group": "default", "serial_result": run_i2c_hil.RESULT_PASS},
            {"group": "final-cleanup", "serial_result": run_i2c_hil.RESULT_FAIL},
        ]

        self.assertEqual("FAIL", run_i2c_hil.final_verdict(rows, dry_run=False))

    def test_recovery_rows_do_not_erase_root_failure(self) -> None:
        rows = [{"serial_result": run_i2c_hil.RESULT_FAIL}]
        rows.extend({"serial_result": run_i2c_hil.RESULT_PASS} for _ in range(5))

        self.assertEqual("FAIL", run_i2c_hil.final_verdict(rows, dry_run=False))

    def test_custom_plan_gets_version_provenance_row(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / "commands.txt"
            path.write_text("drv\n", encoding="utf-8")
            args = run_i2c_hil.parse_args(
                ["--dry-run", "--commands", str(path)]
            )
            executable, _ = run_i2c_hil.build_command_sequence(args)

        self.assertEqual(["version", "drv"], [spec.command for spec in executable])

    def test_custom_version_never_replaces_canonical_provenance_probe(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / "commands.txt"
            path.write_text("version\ndrv\n", encoding="utf-8")
            args = run_i2c_hil.parse_args(
                ["--dry-run", "--commands", str(path)]
            )
            executable, _ = run_i2c_hil.build_command_sequence(args)

        self.assertEqual("version", executable[0].command)
        self.assertEqual("provenance", executable[0].group)
        self.assertEqual("custom-command-file", executable[1].group)

    def test_missing_custom_plan_is_a_clean_configuration_error(self) -> None:
        missing = pathlib.Path(tempfile.gettempdir()) / "bme280-missing-command-plan.txt"
        if missing.exists():
            missing.unlink()
        stderr = io.StringIO()

        with contextlib.redirect_stderr(stderr):
            result = run_i2c_hil.main(["--dry-run", "--commands", str(missing)])

        self.assertEqual(2, result)
        self.assertIn("Cannot load command plan:", stderr.getvalue())
        self.assertNotIn("Traceback", stderr.getvalue())

    def test_unreadable_custom_plan_is_a_clean_configuration_error(self) -> None:
        stderr = io.StringIO()
        with mock.patch.object(
            pathlib.Path, "read_text", side_effect=PermissionError("access denied")
        ), contextlib.redirect_stderr(stderr):
            result = run_i2c_hil.main(["--dry-run", "--commands", "blocked.txt"])

        self.assertEqual(2, result)
        self.assertIn("access denied", stderr.getvalue())
        self.assertNotIn("Traceback", stderr.getvalue())

    def test_invalid_utf8_custom_plan_is_a_clean_configuration_error(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / "commands.txt"
            path.write_bytes(b"drv\n\xff\n")
            stderr = io.StringIO()
            with contextlib.redirect_stderr(stderr):
                result = run_i2c_hil.main(["--dry-run", "--commands", str(path)])

        self.assertEqual(2, result)
        self.assertIn("Cannot load command plan:", stderr.getvalue())
        self.assertNotIn("Traceback", stderr.getvalue())

    def test_tab_separated_custom_raw_write_is_fully_gated_and_marked(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / "commands.txt"
            path.write_text("wreg\t0xF4\t0x00\n", encoding="utf-8")
            blocked = run_i2c_hil.parse_args(
                ["--dry-run", "--commands", str(path)]
            )
            with self.assertRaises(ValueError):
                run_i2c_hil.build_command_sequence(blocked)

            enabled = run_i2c_hil.parse_args([
                "--port", "COM10", "--commands", str(path),
                "--include-destructive",
            ])
            executable, _ = run_i2c_hil.build_command_sequence(enabled)

        raw = executable[1]
        self.assertTrue(raw.destructive)
        self.assertEqual("recover", raw.recovery_command)
        self.assertTrue(run_i2c_hil.execution_safety_errors(enabled, executable))

    def test_generated_log_directory_is_excluded_from_end_worktree_state(self) -> None:
        ignored = run_i2c_hil.ROOT / "hil_logs" / "i2c_test"
        with mock.patch.object(
            run_i2c_hil, "git_value", return_value=""
        ) as git:
            state = run_i2c_hil.worktree_state(ignored)

        self.assertEqual("clean", state)
        self.assertIn(
            ":(exclude,glob)hil_logs/i2c_test/**", git.call_args.args[0]
        )

    def test_unknown_rows_are_written_to_failure_analysis(self) -> None:
        summary = {
            "final_verdict": "OPERATOR_REVIEW_REQUIRED",
            "results": [{
                "group": "reset",
                "command": "reset",
                "serial_result": run_i2c_hil.RESULT_UNKNOWN,
                "classification_reason": "NVM state remained bounded but unknown",
            }],
        }
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / "failure_analysis.md"
            run_i2c_hil.write_failure_analysis(path, summary)
            text = path.read_text(encoding="utf-8")

        self.assertIn("`UNKNOWN`", text)
        self.assertIn("NVM state remained bounded but unknown", text)


if __name__ == "__main__":
    unittest.main()
