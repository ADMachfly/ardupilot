#!/usr/bin/env python3
"""HIL-F24-F unit tests for sr75_hil_f24f_dynamic_profile_validation.py's
pure coherence-check functions. No hardware, no sockets.

Confirms both that the real profiles pass, and -- more importantly -- that
each check actually detects a deliberately broken row (so these checks
have real power, not just a tautological pass on already-correct data).
"""
import copy
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "sim_json"))
import sr75_hil_f24f_dynamic_profile_validation as f24f  # noqa: E402
import sr75_sim_json_test_profiles as profiles  # noqa: E402


class TestQuaternionEulerConsistency(unittest.TestCase):
    def test_all_profiles_pass(self):
        for name, fn in profiles.PROFILES_F24F.items():
            f24f.check_quaternion_euler_consistency(name, fn())

    def test_detects_corrupted_quaternion(self):
        rows = profiles.profile_roll_sweep_multileg()
        rows = copy.deepcopy(rows)
        w, x, y, z = rows[10].quaternion
        rows[10].quaternion = (w, x + 0.5, y, z)  # break the norm/consistency
        with self.assertRaises(f24f.HarnessFailure):
            f24f.check_quaternion_euler_consistency("broken", rows)


class TestGyroMatchesAttitudeRate(unittest.TestCase):
    def test_all_profiles_pass(self):
        for name, fn in profiles.PROFILES_F24F.items():
            f24f.check_gyro_matches_attitude_rate(name, fn())

    def test_detects_wrong_gyro_sign(self):
        rows = profiles.profile_roll_sweep_multileg()
        rows = copy.deepcopy(rows)
        for state in rows:
            p, q, r = state.gyro_rad_s
            state.gyro_rad_s = (-p, q, r)  # flip roll-rate sign vs actual attitude change
        with self.assertRaises(f24f.HarnessFailure):
            f24f.check_gyro_matches_attitude_rate("broken", rows)


class TestNedConsistency(unittest.TestCase):
    def test_all_profiles_pass(self):
        for name, fn in profiles.PROFILES_F24F.items():
            f24f.check_ned_consistency(name, fn())

    def test_detects_altitude_not_matching_vd(self):
        rows = profiles.profile_altitude_ramp_multileg()
        rows = copy.deepcopy(rows)
        rows[5].altitude_m += 50.0  # inconsistent with vd-integral
        with self.assertRaises(f24f.HarnessFailure):
            f24f.check_ned_consistency("broken", rows)

    def test_detects_latlon_drift_with_zero_horizontal_velocity(self):
        rows = profiles.profile_static_level()
        rows = copy.deepcopy(rows)
        rows[3].latitude_deg += 0.001
        with self.assertRaises(f24f.HarnessFailure):
            f24f.check_ned_consistency("broken", rows)


class TestTimestampsMonotonicAtRate(unittest.TestCase):
    def test_all_profiles_pass(self):
        for name, fn in profiles.PROFILES_F24F.items():
            f24f.check_timestamps_monotonic_at_rate(name, fn(), expected_rate_hz=50.0)

    def test_detects_non_monotonic_timestamp(self):
        rows = profiles.profile_static_level()
        rows = copy.deepcopy(rows)
        rows[4].timestamp_s = rows[3].timestamp_s - 0.01
        with self.assertRaises(f24f.HarnessFailure):
            f24f.check_timestamps_monotonic_at_rate("broken", rows, expected_rate_hz=50.0)

    def test_detects_wrong_rate(self):
        rows = profiles.profile_static_level(rate_hz=50.0)
        with self.assertRaises(f24f.HarnessFailure):
            f24f.check_timestamps_monotonic_at_rate("broken", rows, expected_rate_hz=100.0)


class TestAirspeedSane(unittest.TestCase):
    def test_all_profiles_pass(self):
        for name, fn in profiles.PROFILES_F24F.items():
            f24f.check_airspeed_sane(name, fn())

    def test_detects_negative_airspeed(self):
        rows = profiles.profile_static_level()
        rows = copy.deepcopy(rows)
        rows[0].airspeed_mps = -1.0
        with self.assertRaises(f24f.HarnessFailure):
            f24f.check_airspeed_sane("broken", rows)


class TestRunAll(unittest.TestCase):
    def test_run_all_is_overall_pass(self):
        overall_ok, results = f24f.run_all()
        self.assertTrue(overall_ok, results)
        for name, (ok, detail) in results.items():
            self.assertTrue(ok, f"{name}: {detail}")

    def test_run_all_covers_every_required_item2_case(self):
        names = {name for name, _ in f24f.TESTS}
        required = {
            "static_level", "roll_sweep_multileg", "pitch_sweep_multileg",
            "yaw_sweep_multileg", "altitude_ramp_multileg", "combined_gentle",
            "stale_data_stop", "malformed_missing_fields",
        }
        self.assertTrue(required.issubset(names), names)


if __name__ == "__main__":
    unittest.main()
