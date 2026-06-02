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


if __name__ == "__main__":
    unittest.main()
