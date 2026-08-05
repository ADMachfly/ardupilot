#!/usr/bin/env python3
"""HIL-F24-F unit tests for the new multi-leg profile generators added to
sr75_sim_json_test_profiles.py (roll/pitch/yaw/altitude multileg,
combined_gentle). No hardware, no sockets.
"""
import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import sr75_sim_json_test_profiles as profiles  # noqa: E402


class TestPiecewiseLinear(unittest.TestCase):
    def test_value_at_waypoints_matches_exactly(self):
        waypoints = [(0.0, 0.0), (4.0, 10.0), (8.0, -10.0), (12.0, 0.0)]
        for t, expected in waypoints:
            value, _ = profiles._piecewise_linear(t, waypoints)
            self.assertAlmostEqual(value, expected, places=9)

    def test_rate_is_constant_within_a_leg(self):
        waypoints = [(0.0, 0.0), (4.0, 8.0)]
        _, rate_a = profiles._piecewise_linear(1.0, waypoints)
        _, rate_b = profiles._piecewise_linear(3.0, waypoints)
        self.assertAlmostEqual(rate_a, 2.0, places=9)
        self.assertAlmostEqual(rate_b, 2.0, places=9)

    def test_clamped_before_first_and_after_last_waypoint(self):
        waypoints = [(0.0, 5.0), (4.0, 9.0)]
        value_before, _ = profiles._piecewise_linear(-1.0, waypoints)
        value_after, _ = profiles._piecewise_linear(10.0, waypoints)
        self.assertAlmostEqual(value_before, 5.0, places=9)
        self.assertAlmostEqual(value_after, 9.0, places=9)  # clamped to the last waypoint's value


class TestRollSweepMultileg(unittest.TestCase):
    def test_default_legs_reach_plus_and_minus_15_deg(self):
        rows = profiles.profile_roll_sweep_multileg()
        peak = max(math.degrees(s.roll_rad) for s in rows)
        trough = min(math.degrees(s.roll_rad) for s in rows)
        self.assertGreater(peak, 14.9)
        self.assertLess(trough, -14.9)

    def test_starts_and_ends_near_zero(self):
        rows = profiles.profile_roll_sweep_multileg()
        self.assertAlmostEqual(math.degrees(rows[0].roll_rad), 0.0, places=6)
        self.assertLess(abs(math.degrees(rows[-1].roll_rad)), 1.0)

    def test_pitch_yaw_held_level(self):
        rows = profiles.profile_roll_sweep_multileg()
        for s in rows:
            self.assertEqual(s.pitch_rad, 0.0)
            self.assertEqual(s.gyro_rad_s[1], 0.0)
            self.assertEqual(s.gyro_rad_s[2], 0.0)

    def test_custom_leg_values(self):
        rows = profiles.profile_roll_sweep_multileg(leg_values_deg=(0.0, 20.0, 0.0))
        peak = max(math.degrees(s.roll_rad) for s in rows)
        self.assertGreater(peak, 19.5)


class TestPitchSweepMultileg(unittest.TestCase):
    def test_default_legs_reach_plus_15_minus_10_deg(self):
        rows = profiles.profile_pitch_sweep_multileg()
        peak = max(math.degrees(s.pitch_rad) for s in rows)
        trough = min(math.degrees(s.pitch_rad) for s in rows)
        self.assertGreater(peak, 14.9)
        self.assertLess(trough, -9.9)

    def test_roll_yaw_held_level(self):
        rows = profiles.profile_pitch_sweep_multileg()
        for s in rows:
            self.assertEqual(s.roll_rad, 0.0)
            self.assertEqual(s.gyro_rad_s[0], 0.0)
            self.assertEqual(s.gyro_rad_s[2], 0.0)


class TestYawSweepMultileg(unittest.TestCase):
    def test_default_legs_go_315_to_270_to_315(self):
        rows = profiles.profile_yaw_sweep_multileg()
        self.assertAlmostEqual(math.degrees(rows[0].yaw_rad), 315.0, places=4)
        trough = min(math.degrees(s.yaw_rad) for s in rows)
        self.assertLess(trough, 270.9)

    def test_roll_pitch_held_level(self):
        rows = profiles.profile_yaw_sweep_multileg()
        for s in rows:
            self.assertEqual(s.roll_rad, 0.0)
            self.assertEqual(s.pitch_rad, 0.0)


class TestAltitudeRampMultileg(unittest.TestCase):
    def test_default_legs_go_3000_3020_2980_3000(self):
        rows = profiles.profile_altitude_ramp_multileg()
        self.assertAlmostEqual(rows[0].altitude_m, 3000.0, places=4)
        peak = max(s.altitude_m for s in rows)
        trough = min(s.altitude_m for s in rows)
        self.assertGreater(peak, 3019.5)
        self.assertLess(trough, 2980.5)

    def test_vd_matches_ned_down_positive_convention(self):
        rows = profiles.profile_altitude_ramp_multileg()
        climbing_row = next(s for s in rows if 0.0 < s.timestamp_s < 5.0)
        self.assertLess(climbing_row.velocity_ned_mps[2], 0.0)  # climbing => vd < 0

    def test_level_attitude_throughout(self):
        rows = profiles.profile_altitude_ramp_multileg()
        for s in rows:
            self.assertEqual(s.roll_rad, 0.0)
            self.assertEqual(s.pitch_rad, 0.0)


class TestCombinedGentle(unittest.TestCase):
    def test_amplitudes_stay_small_and_gentle(self):
        rows = profiles.profile_combined_gentle()
        peak_roll = max(abs(math.degrees(s.roll_rad)) for s in rows)
        peak_pitch = max(abs(math.degrees(s.pitch_rad)) for s in rows)
        self.assertLess(peak_roll, 6.0)
        self.assertLess(peak_pitch, 4.0)

    def test_altitude_stays_within_small_band(self):
        rows = profiles.profile_combined_gentle()
        peak = max(s.altitude_m for s in rows)
        self.assertLess(peak - profiles.BASE_ALT_M, 11.0)

    def test_airspeed_is_nonzero_constant(self):
        rows = profiles.profile_combined_gentle(airspeed_mps=42.0)
        self.assertTrue(all(s.airspeed_mps == 42.0 for s in rows))

    def test_all_rows_pass_validate(self):
        rows = profiles.profile_combined_gentle()
        for s in rows:
            s.validate()


class TestProfilesF24FRegistry(unittest.TestCase):
    def test_registry_contains_all_six_profiles(self):
        expected = {
            "static_level", "roll_sweep_multileg", "pitch_sweep_multileg",
            "yaw_sweep_multileg", "altitude_ramp_multileg", "combined_gentle",
        }
        self.assertEqual(set(profiles.PROFILES_F24F), expected)

    def test_original_profiles_registry_unchanged(self):
        # HIL-F24-F must not alter HIL-F24-B's existing generic profile set.
        expected = {"static_level", "pitch_sweep", "roll_sweep", "yaw_sweep", "altitude_ramp"}
        self.assertEqual(set(profiles.PROFILES), expected)


if __name__ == "__main__":
    unittest.main()
