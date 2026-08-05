#!/usr/bin/env python3
"""HIL-F23-F2B2A/F2B3 unit tests for the new GPS_RAW_INT/VFR_HUD altitude
CSV logging fields in sr75_jsbsim_pixhawk_hil_bridge.py's make_row().

No hardware, no MAVLink connection -- fake message objects only.
"""
import types
import unittest

import sr75_jsbsim_pixhawk_hil_bridge as bridge


class TestAltitudeLoggingFields(unittest.TestCase):
    def test_gps_raw_int_and_vfr_hud_alt_present_and_converted(self):
        gps_raw = types.SimpleNamespace(alt=3000350, fix_type=3, lat=325378147, lon=743661871)
        vfr_hud = types.SimpleNamespace(alt=3001.2, airspeed=0.0, groundspeed=0.0, throttle=0)
        latest = {"GPS_RAW_INT": gps_raw, "VFR_HUD": vfr_hud}
        row = bridge.make_row(0.0, latest)
        self.assertAlmostEqual(row["gps_raw_alt_m"], 3000.35, places=3)
        self.assertEqual(row["vfr_alt_m"], 3001.2)
        self.assertEqual(row["gps_raw_fix_type"], 3)
        self.assertAlmostEqual(row["gps_raw_lat_deg"], 32.5378147, places=7)
        self.assertAlmostEqual(row["gps_raw_lon_deg"], 74.3661871, places=7)

    def test_fields_blank_when_messages_absent(self):
        row = bridge.make_row(0.0, {})
        self.assertEqual(row["gps_raw_alt_m"], "")
        self.assertEqual(row["vfr_alt_m"], "")
        self.assertEqual(row["gps_raw_fix_type"], "")
        self.assertEqual(row["gps_raw_lat_deg"], "")
        self.assertEqual(row["gps_raw_lon_deg"], "")

    def test_message_rates_requests_gps_raw_int(self):
        self.assertIn("GPS_RAW_INT", bridge.MESSAGE_RATES_HZ)
        self.assertGreater(bridge.MESSAGE_RATES_HZ["GPS_RAW_INT"], 0)

    def test_fieldnames_include_new_altitude_columns(self):
        fieldnames = list(bridge.make_row(0.0, {}).keys())
        for column in ("gps_raw_alt_m", "vfr_alt_m", "gps_raw_fix_type", "gps_raw_lat_deg", "gps_raw_lon_deg"):
            self.assertIn(column, fieldnames)


if __name__ == "__main__":
    unittest.main()
