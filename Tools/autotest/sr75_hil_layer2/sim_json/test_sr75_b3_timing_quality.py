#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Tests for the SR-75 Layer 2J-B3C-C3 scoring-window timing-quality gate."""

import unittest

from sr75_sim_json_responder import (
    B3TimingGateState,
    B3TimingQualityGate,
    TimeDiscontinuityError,
    b3_time_discontinuity_offenders,
)


class TestB3TimeDiscontinuityOffenders(unittest.TestCase):
    def test_clean_deltas_have_no_offenders(self):
        result = b3_time_discontinuity_offenders(host_dt=0.02, source_simulation_dt=0.02, outgoing_simulation_dt=0.02)
        self.assertEqual(result, ())

    def test_host_dt_non_positive(self):
        offenders = b3_time_discontinuity_offenders(host_dt=0.0, source_simulation_dt=0.02, outgoing_simulation_dt=0.02)
        self.assertTrue(any("host_dt" in o for o in offenders))

    def test_source_simulation_dt_non_positive(self):
        offenders = b3_time_discontinuity_offenders(host_dt=0.02, source_simulation_dt=0.0, outgoing_simulation_dt=0.02)
        self.assertTrue(any(o.startswith("source_simulation_dt=0") and o.endswith("<=0") for o in offenders))

    def test_source_simulation_dt_too_large(self):
        offenders = b3_time_discontinuity_offenders(host_dt=0.02, source_simulation_dt=0.2, outgoing_simulation_dt=0.02)
        self.assertTrue(any("source_simulation_dt=0.200000000>0.05" in o for o in offenders))

    def test_outgoing_simulation_dt_non_positive(self):
        offenders = b3_time_discontinuity_offenders(host_dt=0.02, source_simulation_dt=0.02, outgoing_simulation_dt=-0.01)
        self.assertTrue(any(o.startswith("outgoing_simulation_dt") and o.endswith("<=0") for o in offenders))

    def test_outgoing_simulation_dt_too_large(self):
        offenders = b3_time_discontinuity_offenders(host_dt=0.02, source_simulation_dt=0.02, outgoing_simulation_dt=0.2)
        self.assertTrue(any("outgoing_simulation_dt=0.200000000>0.05" in o for o in offenders))

    def test_ratio_too_large(self):
        offenders = b3_time_discontinuity_offenders(host_dt=0.01, source_simulation_dt=0.04, outgoing_simulation_dt=0.02)
        self.assertTrue(any("source_simulation_dt/host_dt" in o for o in offenders))

    def test_multiple_offenders_reported_together(self):
        offenders = b3_time_discontinuity_offenders(host_dt=0.0, source_simulation_dt=0.0, outgoing_simulation_dt=0.0)
        self.assertEqual(len(offenders), 3)


class TestB3TimingQualityGate(unittest.TestCase):
    def test_disabled_gate_never_checks(self):
        gate = B3TimingQualityGate(False)
        self.assertEqual(gate.state, B3TimingGateState.INACTIVE)
        result = gate.check(1, host_time=1.0, source_simulation_timestamp=100.0, outgoing_simulation_timestamp=0.0)
        self.assertIsNone(result)
        result2 = gate.check(2, host_time=1.02, source_simulation_timestamp=100.5, outgoing_simulation_timestamp=0.5)
        self.assertIsNone(result2)

    def test_first_generic_row_has_nothing_to_compare(self):
        gate = B3TimingQualityGate(True)
        self.assertEqual(gate.state, B3TimingGateState.ACTIVE)
        result = gate.check(1, host_time=1.0, source_simulation_timestamp=4.4, outgoing_simulation_timestamp=0.0)
        self.assertIsNone(result)
        self.assertEqual(gate.baseline_label_for_request(1), "TIMING_BASELINE")

    def test_clean_consecutive_rows_pass_and_return_deltas(self):
        gate = B3TimingQualityGate(True)
        gate.check(1, host_time=1.00, source_simulation_timestamp=4.40, outgoing_simulation_timestamp=0.00)
        host_dt, source_dt, outgoing_dt, ratio = gate.check(
            2, host_time=1.02, source_simulation_timestamp=4.42, outgoing_simulation_timestamp=0.02
        )
        self.assertAlmostEqual(host_dt, 0.02, places=9)
        self.assertAlmostEqual(source_dt, 0.02, places=9)
        self.assertAlmostEqual(outgoing_dt, 0.02, places=9)
        self.assertAlmostEqual(ratio, 1.0, places=6)

    def test_held_row_followed_by_real_large_gap_becomes_baseline(self):
        gate = B3TimingQualityGate(True)
        gate.on_startup_sync_release(
            final_held_outgoing_timestamp=3.195384064,
            release_host_time=10.0,
            release_count=1,
        )
        self.assertEqual(gate.state, B3TimingGateState.WAIT_FIRST_REAL_ROW)

        result = gate.check(
            128,
            host_time=13.49,
            source_simulation_timestamp=3.383198,
            outgoing_simulation_timestamp=6.390770128,
        )

        self.assertIsNone(result)
        self.assertEqual(gate.state, B3TimingGateState.ACTIVE)
        self.assertEqual(gate.baseline_label_for_request(128), "TIMING_BASELINE")

    def test_second_consecutive_real_row_dt_0_02_passes(self):
        gate = B3TimingQualityGate(True)
        gate.on_startup_sync_release(final_held_outgoing_timestamp=3.0, release_host_time=10.0, release_count=1)
        gate.check(128, host_time=10.000, source_simulation_timestamp=4.40, outgoing_simulation_timestamp=3.000001)
        host_dt, source_dt, outgoing_dt, ratio = gate.check(
            129,
            host_time=10.020,
            source_simulation_timestamp=4.42,
            outgoing_simulation_timestamp=3.020001,
        )
        self.assertAlmostEqual(host_dt, 0.02, places=9)
        self.assertAlmostEqual(source_dt, 0.02, places=9)
        self.assertAlmostEqual(outgoing_dt, 0.02, places=9)
        self.assertAlmostEqual(ratio, 1.0, places=6)

    def test_second_consecutive_real_row_dt_3_2_aborts(self):
        gate = B3TimingQualityGate(True)
        gate.on_startup_sync_release(final_held_outgoing_timestamp=3.0, release_host_time=10.0, release_count=1)
        gate.check(128, host_time=10.000, source_simulation_timestamp=4.40, outgoing_simulation_timestamp=3.000001)
        with self.assertRaises(TimeDiscontinuityError) as ctx:
            gate.check(129, host_time=10.020, source_simulation_timestamp=7.60, outgoing_simulation_timestamp=6.200001)
        exc = ctx.exception
        self.assertTrue(any("source_simulation_dt=3.200000000>0.05" in o for o in exc.offenders))
        self.assertTrue(any("outgoing_simulation_dt=3.200000000>0.05" in o for o in exc.offenders))

    def test_timestamp_reversal_at_release_aborts(self):
        gate = B3TimingQualityGate(True)
        gate.on_startup_sync_release(final_held_outgoing_timestamp=3.2, release_host_time=10.0, release_count=1)
        with self.assertRaises(TimeDiscontinuityError) as ctx:
            gate.check(128, host_time=10.1, source_simulation_timestamp=4.40, outgoing_simulation_timestamp=3.1)
        self.assertTrue(any("release_outgoing_timestamp" in o for o in ctx.exception.offenders))

    def test_multiple_held_rows_before_release_do_not_create_timing_checks(self):
        gate = B3TimingQualityGate(True)
        for index in range(10):
            self.assertIsNone(gate.previous)
            self.assertNotEqual(gate.baseline_label_for_request(index), "TIMING_BASELINE")
        gate.on_startup_sync_release(final_held_outgoing_timestamp=2.0, release_host_time=3.0, release_count=1)
        result = gate.check(64, host_time=6.5, source_simulation_timestamp=7.0, outgoing_simulation_timestamp=5.2)
        self.assertIsNone(result)
        self.assertEqual(gate.baseline_label_for_request(64), "TIMING_BASELINE")

    def test_timing_gate_disabled_release_is_generic_noop(self):
        gate = B3TimingQualityGate(False)
        gate.on_startup_sync_release(final_held_outgoing_timestamp=3.2, release_host_time=10.0, release_count=2)
        self.assertEqual(gate.state, B3TimingGateState.INACTIVE)
        self.assertIsNone(
            gate.check(1, host_time=1.0, source_simulation_timestamp=10.0, outgoing_simulation_timestamp=0.0)
        )
        self.assertIsNone(
            gate.check(2, host_time=1.0, source_simulation_timestamp=7.0, outgoing_simulation_timestamp=-1.0)
        )

    def test_scoring_rows_remain_protected_after_baseline(self):
        gate = B3TimingQualityGate(True)
        gate.on_startup_sync_release(final_held_outgoing_timestamp=3.0, release_host_time=10.0, release_count=1)
        gate.check(128, host_time=10.000, source_simulation_timestamp=4.40, outgoing_simulation_timestamp=3.000001)
        gate.check(129, host_time=10.020, source_simulation_timestamp=4.42, outgoing_simulation_timestamp=3.020001)
        with self.assertRaises(TimeDiscontinuityError):
            gate.check(130, host_time=10.040, source_simulation_timestamp=4.80, outgoing_simulation_timestamp=3.400001)

    def test_realtime_burst_catchup_aborts(self):
        gate = B3TimingQualityGate(True)
        gate.check(1, host_time=1.000, source_simulation_timestamp=4.40, outgoing_simulation_timestamp=0.00)
        with self.assertRaises(TimeDiscontinuityError) as ctx:
            gate.check(2, host_time=1.017, source_simulation_timestamp=7.60, outgoing_simulation_timestamp=3.20)
        exc = ctx.exception
        self.assertTrue(any("source_simulation_dt" in o for o in exc.offenders))
        self.assertEqual(exc.previous["request_count"], 1)
        self.assertEqual(exc.current["request_count"], 2)

    def test_gate_stops_tracking_after_abort_state_is_retained(self):
        gate = B3TimingQualityGate(True)
        gate.check(1, host_time=1.0, source_simulation_timestamp=4.40, outgoing_simulation_timestamp=0.00)
        with self.assertRaises(TimeDiscontinuityError):
            gate.check(2, host_time=1.02, source_simulation_timestamp=4.42, outgoing_simulation_timestamp=0.06)
        # Gate keeps the offending row as its new "previous" so a subsequent clean pair still works.
        result = gate.check(3, host_time=1.04, source_simulation_timestamp=4.44, outgoing_simulation_timestamp=0.08)
        self.assertIsNotNone(result)


if __name__ == "__main__":
    unittest.main()
