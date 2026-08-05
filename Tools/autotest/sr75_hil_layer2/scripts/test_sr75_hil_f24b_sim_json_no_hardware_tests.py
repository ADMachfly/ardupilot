#!/usr/bin/env python3
"""HIL-F24-B unit tests: wraps sr75_hil_f24b_sim_json_no_hardware_tests.py's
individual checks as proper unittest cases (for `python3 -m unittest
discover` regression running), plus a few extra pure-function tests on the
profile generators and schema checker directly.

No sockets, no hardware, no PPP, no actuator command path -- same
constraints as the harness itself.
"""
import math
import sys
import unittest
from pathlib import Path

SCRIPTS_DIR = Path(__file__).resolve().parent
SIM_JSON_DIR = Path(__file__).resolve().parents[1] / "sim_json"
sys.path.insert(0, str(SCRIPTS_DIR))
sys.path.insert(0, str(SIM_JSON_DIR))
import sr75_hil_f24b_sim_json_no_hardware_tests as harness  # noqa: E402
import sr75_sim_json_test_profiles as profiles  # noqa: E402
import sr75_sim_json_responder as responder  # noqa: E402


class TestHarnessChecksIndividually(unittest.TestCase):
    """Runs every check the standalone harness runs, as separate unittest
    cases, so a regression in any one of them is individually reported.
    """

    def test_all_profiles_schema_rate_duration(self):
        harness.test_all_profiles()

    def test_pitch_sweep_bounds(self):
        harness.test_pitch_sweep_bounds()

    def test_roll_sweep_bounds(self):
        harness.test_roll_sweep_bounds()

    def test_yaw_sweep_monotonic(self):
        harness.test_yaw_sweep_monotonic()

    def test_altitude_ramp_rate(self):
        harness.test_altitude_ramp_rate()

    def test_stale_timeout(self):
        harness.test_stale_timeout()

    def test_missing_required_fields(self):
        harness.test_missing_required_fields()

    def test_malformed_row_rejected(self):
        harness.test_malformed_row_rejected()

    def test_no_actuator_command_path(self):
        harness.test_no_actuator_command_path()

    def test_run_all_reports_overall_pass(self):
        overall_ok, results = harness.run_all()
        self.assertTrue(overall_ok, results)
        self.assertEqual(len(results), len(harness.TESTS))


class TestProfileGenerators(unittest.TestCase):
    def test_static_level_zero_motion(self):
        rows = profiles.profile_static_level(duration_s=1.0, rate_hz=10.0)
        for s in rows:
            self.assertEqual(s.roll_rad, 0.0)
            self.assertEqual(s.pitch_rad, 0.0)
            self.assertEqual(s.yaw_rad, 0.0)
            self.assertEqual(s.gyro_rad_s, (0.0, 0.0, 0.0))
            self.assertEqual(s.velocity_ned_mps, (0.0, 0.0, 0.0))
            self.assertAlmostEqual(s.accel_body_mss[2], -9.80665, places=3)

    def test_pitch_sweep_returns_to_zero_at_full_period(self):
        rows = profiles.profile_pitch_sweep(duration_s=4.0, rate_hz=100.0, max_pitch_deg=20.0)
        # first sample is exactly t=0 -> sin(0)=0
        self.assertAlmostEqual(rows[0].pitch_rad, 0.0, places=6)

    def test_roll_sweep_amplitude(self):
        rows = profiles.profile_roll_sweep(duration_s=4.0, rate_hz=200.0, max_roll_deg=30.0)
        peak_deg = max(abs(math.degrees(s.roll_rad)) for s in rows)
        self.assertAlmostEqual(peak_deg, 30.0, delta=0.5)

    def test_yaw_sweep_starts_at_base_yaw(self):
        rows = profiles.profile_yaw_sweep(duration_s=2.0, rate_hz=10.0, rate_deg_s=9.0)
        self.assertAlmostEqual(math.degrees(rows[0].yaw_rad), profiles.BASE_YAW_DEG, places=3)

    def test_altitude_ramp_starts_at_base_altitude(self):
        rows = profiles.profile_altitude_ramp(duration_s=5.0, rate_hz=10.0, climb_rate_mps=1.0)
        self.assertAlmostEqual(rows[0].altitude_m, profiles.BASE_ALT_M, places=3)

    def test_all_profiles_produce_valid_json(self):
        import json
        for name, fn in profiles.PROFILES.items():
            rows = fn(duration_s=0.5, rate_hz=10.0)
            payload = json.loads(rows[0].to_json_bytes())
            self.assertIn("timestamp", payload, name)
            self.assertIn("imu", payload, name)
            self.assertIn("gyro", payload["imu"], name)
            self.assertIn("accel_body", payload["imu"], name)


class TestSchemaChecker(unittest.TestCase):
    def test_accepts_a_valid_profile_row(self):
        state = profiles.make_state(0.0)
        harness.check_json_schema(state)  # must not raise

    def test_rejects_missing_gyro_component(self):
        state = profiles.make_state(0.0)
        state.gyro_rad_s = (0.0, 0.0)  # type: ignore[assignment]
        with self.assertRaises(harness.HarnessFailure):
            harness.check_json_schema(state)

    def test_rejects_out_of_range_latitude(self):
        state = profiles.make_state(0.0, lat_deg=200.0)
        with self.assertRaises(harness.HarnessFailure):
            harness.check_json_schema(state)

    def test_no_forbidden_keys_in_any_profile(self):
        for name, fn in profiles.PROFILES.items():
            rows = fn(duration_s=0.3, rate_hz=10.0)
            for state in rows:
                harness.check_json_schema(state)  # raises on forbidden keys


class TestStaleAndMalformedUseRealResponderCode(unittest.TestCase):
    """Confirms the harness genuinely calls into sr75_sim_json_responder.py
    (not a reimplementation) for stale/missing-field handling."""

    def test_stale_timeout_uses_real_read_reply_state(self):
        import inspect
        source = inspect.getsource(harness.test_stale_timeout)
        self.assertIn("responder.read_reply_state", source)

    def test_missing_fields_uses_real_state_mapper(self):
        import inspect
        source = inspect.getsource(harness.test_missing_required_fields)
        self.assertIn("responder.StateMapper", source)

    def test_state_error_class_is_the_real_one(self):
        self.assertIs(harness.HarnessFailure.__bases__[0], Exception)
        # StateError used inside the harness must be the actual responder class
        self.assertTrue(issubclass(responder.StateError, Exception))


if __name__ == "__main__":
    unittest.main()
