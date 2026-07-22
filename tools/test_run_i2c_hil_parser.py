#!/usr/bin/env python3
"""Focused tests for HIL runner serial-output classification."""

from __future__ import annotations

import io
import pathlib
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

        result, reason = run_i2c_hil.classify_output(spec, output, "MATCHED_EXPECTED")

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

        self.assertFalse(run_i2c_hil.output_has_expected(spec, partial))
        self.assertTrue(run_i2c_hil.output_has_expected(spec, complete))
        self.assertEqual(
            run_i2c_hil.RESULT_PASS,
            run_i2c_hil.classify_output(spec, complete, "MATCHED_EXPECTED")[0],
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

        result, reason = run_i2c_hil.classify_output(spec, output, "MATCHED_EXPECTED")

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

        self.assertFalse(run_i2c_hil.output_has_expected(spec, partial))
        self.assertTrue(run_i2c_hil.output_has_expected(spec, complete))
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
            expected=("stress_mix summary", "Total:", "Health delta:"),
            validators=(run_i2c_hil.VALIDATOR_STRESS_MIX_ZERO_FAIL,),
        )
        output = (
            "=== stress_mix summary ===\n"
            "  Total: ok=7 fail=0 (100.00%)\n"
            "  Duration: 140 ms\n"
            "  Rate: 50.00 ops/s\n"
            "  Health delta: success +7, failures +0\n"
        )

        result, reason = run_i2c_hil.classify_output(spec, output, "MATCHED_EXPECTED")
        evidence = run_i2c_hil.extract_parsed_evidence(output)

        self.assertEqual(run_i2c_hil.RESULT_PASS, result)
        self.assertIn("stress_mix fail=0", reason)
        self.assertEqual(7, evidence["stress_mix_ok"])
        self.assertEqual(0, evidence["stress_mix_fail"])
        self.assertEqual(50.0, evidence["rate"])
        self.assertEqual("ops/s", evidence["rate_units"])

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
        )

        result, reason = run_i2c_hil.classify_output(spec, output, "MATCHED_EXPECTED")

        self.assertEqual(run_i2c_hil.RESULT_FAIL, result)
        self.assertIn("fail=1", reason)

    def test_reset_busy_is_unknown_not_timeout_or_pass(self) -> None:
        spec = next(item for item in run_i2c_hil.BASE_COMMANDS if item.command == "reset")
        output = (
            "  Status: BUSY (code=11, detail=5)\n"
            "  Message: BUSY\n"
        )

        result, reason = run_i2c_hil.classify_output(spec, output, "MATCHED_EXPECTED")

        self.assertEqual(run_i2c_hil.RESULT_UNKNOWN, result)
        self.assertIn("Status: BUSY", reason)

    def test_recovered_reset_busy_reclassifies_to_controlled_pass(self) -> None:
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
                "parsed_evidence": {},
                "output_excerpt": "Status: OK (code=0, detail=0)\n",
            },
            {
                "command": "status",
                "group": "soak-duration",
                "serial_result": run_i2c_hil.RESULT_PASS,
                "parsed_evidence": {"measuring": 0, "im_update": 0},
                "output_excerpt": "Status: 0x00 (measuring=0, im_update=0)\nDriver: state=READY online=true dirty=false\n",
            },
        ]

        count = run_i2c_hil.reclassify_recovered_reset_busy(results)

        self.assertEqual(1, count)
        self.assertEqual(run_i2c_hil.RESULT_RESET_BUSY_RECOVERED, results[0]["serial_result"])
        self.assertIn("immediate follow-up", results[0]["classification_reason"])

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
                "parsed_evidence": {},
                "output_excerpt": "Status: OK (code=0, detail=0)\n",
            },
            {
                "command": "status",
                "group": "soak-duration",
                "serial_result": run_i2c_hil.RESULT_PASS,
                "parsed_evidence": {"measuring": 0, "im_update": 1},
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

    def test_duration_soak_cycle_has_bounded_safe_mix(self) -> None:
        args = run_i2c_hil.parse_args(["--dry-run", "--soak-duration-s", "10"])

        commands = [spec.command for spec in run_i2c_hil.duration_soak_cycle_commands(args, 1)]

        self.assertIn("stress 50", commands)
        self.assertIn("stress_mix 70", commands)
        self.assertIn("status", commands)
        self.assertIn("drv", commands)
        self.assertNotIn("wreg 0xF4 0x00", commands)

    def test_include_job_api_adds_job_command_group(self) -> None:
        args = run_i2c_hil.parse_args(["--dry-run", "--include-job-api"])
        executable, _ = run_i2c_hil.build_command_sequence(args)
        job_commands = [spec.command for spec in executable if spec.group == "job-api"]

        self.assertEqual(
            [
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

        result, reason = run_i2c_hil.classify_output(spec, output, "MATCHED_EXPECTED")

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

        result, reason = run_i2c_hil.classify_output(spec, output, "MATCHED_EXPECTED")

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

        result, reason = run_i2c_hil.classify_output(spec, output, "MATCHED_EXPECTED")

        self.assertEqual(run_i2c_hil.RESULT_FAIL, result)
        self.assertIn("budget was 1", reason)

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
                result, reason = run_i2c_hil.classify_output(spec, output, "MATCHED_EXPECTED")
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

    def test_matching_clean_firmware_provenance_remains_pass(self) -> None:
        rows = [self.version_row(
            "BME280 library version: 2.0.0\n"
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

        self.assertEqual(2, count)
        self.assertEqual(run_i2c_hil.RESULT_REVIEW, rows[0]["serial_result"])

    def test_duration_row_requires_its_full_timeout_window(self) -> None:
        args = run_i2c_hil.parse_args(["--dry-run"])
        spec = run_i2c_hil.CommandSpec(
            command="stress 50", purpose="soak", timeout_s=55.0
        )

        self.assertFalse(run_i2c_hil.duration_command_fits(54.999, spec, args))
        self.assertTrue(run_i2c_hil.duration_command_fits(55.0, spec, args))

    def test_config_matrix_restores_forced_policy(self) -> None:
        args = run_i2c_hil.parse_args(["--dry-run", "--include-config-matrix"])
        executable, _ = run_i2c_hil.build_command_sequence(args)
        commands = [spec.command for spec in executable if spec.group == "config-matrix"]

        self.assertEqual(["mode forced", "cfg"], commands[-2:])

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
            if (spec.command.startswith("osrs ") or
                    spec.command.startswith("filter ") or
                    spec.command.startswith("standby ")):
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
            spec, output, "MATCHED_EXPECTED"
        )
        self.assertEqual(run_i2c_hil.RESULT_PASS, result)
        self.assertIn("standby=7", reason)

        mismatch = run_i2c_hil.dataclasses.replace(
            spec, expected_settings=(("filter", 3),)
        )
        result, reason = run_i2c_hil.classify_output(
            mismatch, output, "MATCHED_EXPECTED"
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

    def test_final_cleanup_failure_does_not_override_clean_run_verdict(self) -> None:
        rows = [
            {"group": "default", "serial_result": run_i2c_hil.RESULT_PASS},
            {"group": "final-cleanup", "serial_result": run_i2c_hil.RESULT_FAIL},
        ]

        self.assertEqual("PASS", run_i2c_hil.final_verdict(rows, dry_run=False))

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
