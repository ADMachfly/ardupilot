#!/usr/bin/env python3
"""HIL-F24-D unit tests for sr75_hil_f24d_static_state_feed.py.

No hardware, no sockets. Verifies build_static_row() against the REAL
StateMapper.state_from_csv() from sr75_sim_json_responder.py (not a
reimplementation) so the CSV schema is proven compatible, not assumed.
"""
import math
import sys
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import sr75_hil_f24d_static_state_feed as feed  # noqa: E402
import sr75_sim_json_responder as responder  # noqa: E402
import sr75_sim_json_test_profiles as profiles  # noqa: E402


class TestBuildStaticRow(unittest.TestCase):
    def test_defaults_match_base_constants(self):
        row = feed.build_static_row(0.0)
        self.assertEqual(float(row["lat_deg"]), profiles.BASE_LAT)
        self.assertEqual(float(row["lon_deg"]), profiles.BASE_LON)
        self.assertEqual(float(row["alt_m"]), profiles.BASE_ALT_M)
        self.assertAlmostEqual(math.degrees(float(row["yaw_rad"])), profiles.BASE_YAW_DEG, places=5)

    def test_level_and_motionless(self):
        row = feed.build_static_row(1.234)
        self.assertEqual(float(row["roll_rad"]), 0.0)
        self.assertEqual(float(row["pitch_rad"]), 0.0)
        for key in ("vn_mps", "ve_mps", "vd_mps", "p_rad_s", "q_rad_s", "r_rad_s"):
            self.assertEqual(float(row[key]), 0.0)

    def test_airspeed_override(self):
        row = feed.build_static_row(0.0, airspeed_mps=5.0)
        self.assertEqual(float(row["airspeed_mps"]), 5.0)

    def test_time_advances(self):
        row_a = feed.build_static_row(0.0)
        row_b = feed.build_static_row(2.0)
        self.assertLess(float(row_a["time_s"]), float(row_b["time_s"]))

    def test_row_accepted_by_real_state_mapper(self):
        """Proves the CSV schema this script writes is exactly what the
        real, already-validated StateMapper.state_from_csv() expects --
        not just an assumption about column names."""
        row = feed.build_static_row(3.0, yaw_deg=315.091444, airspeed_mps=0.0)
        mapper = responder.StateMapper(strict=True)
        state = mapper.state_from_csv(row, "sig", time.monotonic())
        self.assertAlmostEqual(state.latitude_deg, profiles.BASE_LAT, places=6)
        self.assertAlmostEqual(state.longitude_deg, profiles.BASE_LON, places=6)
        self.assertAlmostEqual(state.altitude_m, profiles.BASE_ALT_M, places=2)
        # StateMapper.state_from_csv() normalizes yaw (e.g. to a signed
        # range) -- compare modulo 360 rather than assuming it preserves
        # the original 0..360 convention.
        self.assertAlmostEqual(math.degrees(state.yaw_rad) % 360, 315.091444 % 360, places=4)
        self.assertEqual(state.roll_rad, 0.0)
        self.assertEqual(state.pitch_rad, 0.0)
        self.assertEqual(state.velocity_ned_mps, (0.0, 0.0, 0.0))
        self.assertEqual(state.gyro_rad_s, (0.0, 0.0, 0.0))
        self.assertAlmostEqual(state.accel_body_mss[2], -responder.GRAVITY_MSS, places=3)


if __name__ == "__main__":
    unittest.main()
