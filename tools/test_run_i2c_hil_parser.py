#!/usr/bin/env python3
"""Focused tests for HIL runner serial-output classification."""

from __future__ import annotations

import pathlib
import sys
import unittest


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
            "  Status: BUSY (code=11, detail=10)\n"
            "  Message: NVM update in progress\n"
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
                "parsed_evidence": {},
                "output_excerpt": "Status: BUSY (code=11, detail=10)\nMessage: NVM update in progress\n",
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
                "parsed_evidence": {},
                "output_excerpt": "Status: BUSY (code=11, detail=10)\nMessage: NVM update in progress\n",
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


if __name__ == "__main__":
    unittest.main()
