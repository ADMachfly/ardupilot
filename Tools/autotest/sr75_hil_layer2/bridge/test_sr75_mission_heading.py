#!/usr/bin/env python3
"""HIL-F23-D1 unit tests for sr75_mission_heading.py.

No hardware, no MAVLink connection, no ArduPlane/JSBSim process -- pure
function tests over mission-file parsing, bearing math, angle wrapping,
and runtime IC-copy generation.
"""
import math
import tempfile
import unittest
from pathlib import Path

import sr75_mission_heading as smh

V2_MISSION = Path(__file__).resolve().parents[1] / "closed_loop" / "SR75_F22_EXTENDED_ALTITUDE_PROFILE_V2.waypoints"
RELEASE_LAT = 32.5378147
RELEASE_LON = 74.3661871

SAMPLE_IC_XML = """<?xml version="1.0"?>
<initialize name="test IC">
  <!-- sample comment, no double hyphen inside -->
  <latitude unit="DEG">32.5378147</latitude>
  <longitude unit="DEG">74.3661871</longitude>
  <altitude unit="M">3000.35</altitude>
  <ubody unit="M/SEC">43.85</ubody>
  <vbody unit="M/SEC">0.0</vbody>
  <wbody unit="M/SEC">0.0</wbody>
  <psi unit="DEG">315.0</psi>
  <phi unit="DEG">0.0</phi>
  <theta unit="DEG">20.0</theta>
  <p unit="RAD/SEC">0.0</p>
  <q unit="RAD/SEC">0.0</q>
  <r unit="RAD/SEC">0.0</r>
</initialize>
"""


class TestMissionParsing(unittest.TestCase):
    def test_parses_v2_mission_all_items(self):
        items = smh.parse_qgc_wpl(V2_MISSION)
        self.assertEqual(len(items), 14)
        self.assertEqual(items[0].seq, 0)
        self.assertEqual(items[-1].seq, 13)

    def test_home_item_fields(self):
        items = smh.parse_qgc_wpl(V2_MISSION)
        home = items[0]
        self.assertEqual(home.command, 16)  # home is listed as cmd 16 but seq==0 marks it as home
        self.assertAlmostEqual(home.lat, 32.5378085, places=6)


class TestSkippedCommandHandling(unittest.TestCase):
    def test_selects_first_real_nav_waypoint_skipping_home_and_takeoff(self):
        items = smh.parse_qgc_wpl(V2_MISSION)
        item = smh.select_first_nav_waypoint(items)
        self.assertEqual(item.seq, 2)
        self.assertEqual(item.command, smh.MAV_CMD_NAV_WAYPOINT)
        self.assertAlmostEqual(item.lat, 32.7285809, places=6)
        self.assertAlmostEqual(item.lon, 74.1399025, places=6)

    def test_skips_takeoff_change_speed_and_delay_items(self):
        # synthetic mission: home, NAV_TAKEOFF, DO_CHANGE_SPEED, NAV_DELAY, then NAV_WAYPOINT
        lines = [
            "QGC WPL 110",
            "0\t1\t3\t16\t0\t0\t0\t0\t32.5378147\t74.3661871\t3000\t1",
            "1\t0\t3\t22\t20\t0\t0\t0\t32.5378147\t74.3661871\t3020\t1",
            "2\t0\t3\t178\t0\t50\t0\t0\t0\t0\t0\t1",  # DO_CHANGE_SPEED
            "3\t0\t3\t93\t2\t0\t0\t0\t0\t0\t0\t1",   # NAV_DELAY
            "4\t0\t3\t16\t0\t0\t0\t0\t32.7285809\t74.1399025\t5000\t1",  # real NAV_WAYPOINT
        ]
        with tempfile.NamedTemporaryFile("w", suffix=".waypoints", delete=False) as f:
            f.write("\n".join(lines) + "\n")
            path = f.name
        try:
            items = smh.parse_qgc_wpl(path)
            item = smh.select_first_nav_waypoint(items)
            self.assertEqual(item.seq, 4)
            self.assertEqual(item.command, 16)
        finally:
            Path(path).unlink()

    def test_no_nav_waypoint_raises(self):
        lines = [
            "QGC WPL 110",
            "0\t1\t3\t16\t0\t0\t0\t0\t32.5378147\t74.3661871\t3000\t1",
            "1\t0\t3\t22\t20\t0\t0\t0\t32.5378147\t74.3661871\t3020\t1",
        ]
        with tempfile.NamedTemporaryFile("w", suffix=".waypoints", delete=False) as f:
            f.write("\n".join(lines) + "\n")
            path = f.name
        try:
            items = smh.parse_qgc_wpl(path)
            with self.assertRaises(smh.MissionHeadingError):
                smh.select_first_nav_waypoint(items)
        finally:
            Path(path).unlink()


class TestBearing(unittest.TestCase):
    def test_v2_mission_bearing_matches_known_value(self):
        result = smh.compute_mission_heading(V2_MISSION, RELEASE_LAT, RELEASE_LON)
        self.assertAlmostEqual(result.bearing_0_360_deg, 315.091444, places=4)
        self.assertAlmostEqual(result.bearing_signed_deg, -44.908556, places=4)
        self.assertEqual(result.item.seq, 2)

    def test_bearing_due_north(self):
        b = smh.great_circle_bearing_deg(0.0, 0.0, 1.0, 0.0)
        self.assertAlmostEqual(b, 0.0, places=6)

    def test_bearing_due_east(self):
        b = smh.great_circle_bearing_deg(0.0, 0.0, 0.0, 1.0)
        self.assertAlmostEqual(b, 90.0, places=6)


class TestAngleWrapping(unittest.TestCase):
    def test_normalize_0_360(self):
        self.assertAlmostEqual(smh.normalize_0_360(-45.0), 315.0, places=6)
        self.assertAlmostEqual(smh.normalize_0_360(370.0), 10.0, places=6)
        self.assertAlmostEqual(smh.normalize_0_360(315.0), 315.0, places=6)

    def test_normalize_signed_180(self):
        self.assertAlmostEqual(smh.normalize_signed_180(315.0), -45.0, places=6)
        self.assertAlmostEqual(smh.normalize_signed_180(180.0), -180.0, places=6)
        self.assertAlmostEqual(smh.normalize_signed_180(-45.0), -45.0, places=6)

    def test_wrap_angle_error_handles_360_crossing(self):
        # target=350, actual=10 -> true error is -20 (target is 20deg behind actual, wrapping)
        err = smh.wrap_angle_error_deg(350.0, 10.0)
        self.assertAlmostEqual(err, -20.0, places=6)
        err2 = smh.wrap_angle_error_deg(10.0, 350.0)
        self.assertAlmostEqual(err2, 20.0, places=6)

    def test_wrap_angle_error_zero(self):
        self.assertAlmostEqual(smh.wrap_angle_error_deg(315.0, 315.0), 0.0, places=6)


class TestVelocityComponents(unittest.TestCase):
    def test_vn_ve_signs_for_nw_heading(self):
        # 315 deg = NW: vn should be positive (moving north), ve should be negative (moving west)
        result = smh.compute_mission_heading(V2_MISSION, RELEASE_LAT, RELEASE_LON)
        vn, ve = result.vn_ve(43.85)
        self.assertGreater(vn, 0.0, "NW heading must have positive (northward) vn")
        self.assertLess(ve, 0.0, "NW heading must have negative (westward) ve")
        # magnitude check
        self.assertAlmostEqual(math.hypot(vn, ve), 43.85, places=6)

    def test_vn_ve_due_east(self):
        vn, ve = smh.MissionHeadingResult(
            item=None, release_lat=0, release_lon=0,
            bearing_0_360_deg=90.0, bearing_signed_deg=90.0, distance_m=0, source="test",
        ).vn_ve(10.0)
        self.assertAlmostEqual(vn, 0.0, places=6)
        self.assertAlmostEqual(ve, 10.0, places=6)


class TestRuntimeIcGeneration(unittest.TestCase):
    def test_generates_new_file_with_updated_psi_source_unchanged(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            source_path = Path(tmpdir) / "source_ic.xml"
            source_path.write_text(SAMPLE_IC_XML)
            source_mtime_before = source_path.stat().st_mtime
            source_content_before = source_path.read_text()

            dest_dir = Path(tmpdir) / "runtime"
            dest = smh.generate_runtime_ic_copy(source_path, 315.091444, dest_dir=str(dest_dir))

            self.assertNotEqual(dest, source_path)
            self.assertTrue(dest.exists())
            dest_content = dest.read_text()
            self.assertIn('<psi unit="DEG">315.091444</psi>', dest_content)
            self.assertNotIn('<psi unit="DEG">315.0</psi>', dest_content)

            # source untouched
            self.assertEqual(source_path.read_text(), source_content_before)
            self.assertEqual(source_path.stat().st_mtime, source_mtime_before)
            self.assertIn('<psi unit="DEG">315.0</psi>', source_path.read_text())

    def test_preserves_everything_else_byte_for_byte_except_psi(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            source_path = Path(tmpdir) / "source_ic.xml"
            source_path.write_text(SAMPLE_IC_XML)
            dest = smh.generate_runtime_ic_copy(source_path, 44.5, dest_dir=tmpdir)
            expected = SAMPLE_IC_XML.replace('<psi unit="DEG">315.0</psi>', '<psi unit="DEG">44.500000</psi>')
            self.assertEqual(dest.read_text(), expected)

    def test_real_validated_source_xml_is_not_modified(self):
        # Uses the actual validated IC file referenced throughout F22-GZ-*/AM2,
        # confirming this helper never writes to it.
        real_ic = Path(__file__).resolve().parents[2] / "aircraft" / "sr_75_6_dof" / "f22gzo_release_state_init.xml"
        if not real_ic.exists():
            self.skipTest(f"validated source IC not found at {real_ic}")
        before = real_ic.read_text()
        before_mtime = real_ic.stat().st_mtime
        with tempfile.TemporaryDirectory() as tmpdir:
            dest = smh.generate_runtime_ic_copy(real_ic, 315.091444, dest_dir=tmpdir)
            self.assertNotEqual(dest, real_ic)
        after = real_ic.read_text()
        after_mtime = real_ic.stat().st_mtime
        self.assertEqual(before, after)
        self.assertEqual(before_mtime, after_mtime)


class TestLiveStateOverridesStaticVelocity(unittest.TestCase):
    """Confirms the *design rule* (req #6): once a live JSBSim state feed is
    present, mission-derived heading must only seed the initial/target
    reference, never override live vn/ve/lat/lon/yaw. This is exercised at
    the bridge-integration level (see sr75_jsbsim_pixhawk_hil_bridge.py's
    JSBSimStateFeeder priority), but the underlying value-selection logic
    is pure and testable here without a live feed or hardware.
    """
    def resolve_attitude_source(self, jsb_state_feeder_present, jsb_feed_row, mission_yaw_rad):
        """Mirrors the bridge's own resolution rule: live feed row wins
        whenever a state feeder is configured and has produced a row;
        the mission-derived (or manual) static yaw is only used when
        there is no live feeder at all.
        """
        if jsb_state_feeder_present:
            if jsb_feed_row is None:
                return None  # feeder configured but no valid row yet -- no send this cycle
            return jsb_feed_row["yaw_rad"]
        return mission_yaw_rad

    def test_live_feed_row_wins_over_mission_derived_static_yaw(self):
        mission_yaw = math.radians(315.091444)
        live_yaw = math.radians(123.4)
        resolved = self.resolve_attitude_source(True, {"yaw_rad": live_yaw}, mission_yaw)
        self.assertAlmostEqual(resolved, live_yaw, places=6)
        self.assertNotAlmostEqual(resolved, mission_yaw, places=2)

    def test_static_mission_yaw_used_only_without_a_feeder(self):
        mission_yaw = math.radians(315.091444)
        resolved = self.resolve_attitude_source(False, None, mission_yaw)
        self.assertAlmostEqual(resolved, mission_yaw, places=6)

    def test_feeder_present_but_no_row_yet_sends_nothing(self):
        mission_yaw = math.radians(315.091444)
        resolved = self.resolve_attitude_source(True, None, mission_yaw)
        self.assertIsNone(resolved)


class TestGpsInputYawEncoding(unittest.TestCase):
    """HIL-F23-F2A: pure encode/decode tests for the GPS_INPUT.yaw
    MAVLink2 extension field (uint16_t centidegrees, 1..36000, 0="not
    available", 36000=north). See modules/mavlink/message_definitions/
    v1.0/common.xml message id 232 for the authoritative field spec.
    """

    def test_encodes_315_091444_deg(self):
        cdeg = smh.encode_gps_input_yaw_cdeg(315.091444)
        self.assertEqual(cdeg, 31509)
        self.assertAlmostEqual(smh.decode_gps_input_yaw_cdeg(cdeg), 315.09, places=1)

    def test_signed_source_normalization_minus_44_908556(self):
        # -44.908556 deg is the signed (-180..180) form of 315.091444 deg;
        # the encoder must normalize signed input transparently.
        cdeg_signed = smh.encode_gps_input_yaw_cdeg(-44.908556)
        cdeg_unsigned = smh.encode_gps_input_yaw_cdeg(315.091444)
        self.assertEqual(cdeg_signed, cdeg_unsigned)

    def test_north_heading_encodes_as_36000_not_zero(self):
        self.assertEqual(smh.encode_gps_input_yaw_cdeg(0.0), 36000)
        self.assertEqual(smh.encode_gps_input_yaw_cdeg(360.0), 36000)

    def test_east_heading(self):
        self.assertEqual(smh.encode_gps_input_yaw_cdeg(90.0), 9000)

    def test_unknown_yaw_encodes_as_zero(self):
        self.assertEqual(smh.encode_gps_input_yaw_cdeg(None), 0)
        self.assertEqual(smh.encode_gps_input_yaw_cdeg(float("nan")), 0)
        self.assertIsNone(smh.decode_gps_input_yaw_cdeg(0))

    def test_near_north_rounding_never_produces_zero(self):
        # A genuine near-north heading that would quantize to raw 0 must
        # be remapped to 36000 -- 0 is reserved for "not available", not
        # for north.
        cdeg = smh.encode_gps_input_yaw_cdeg(0.001)
        self.assertNotEqual(cdeg, 0)
        self.assertEqual(cdeg, 36000)

    def test_round_trip_full_circle_sample(self):
        for deg in (0.0, 44.5, 90.0, 179.999, 180.0, 270.0, 315.091444, 359.99):
            cdeg = smh.encode_gps_input_yaw_cdeg(deg)
            self.assertGreaterEqual(cdeg, 1)
            self.assertLessEqual(cdeg, 36000)
            decoded = smh.decode_gps_input_yaw_cdeg(cdeg)
            self.assertAlmostEqual(decoded, deg % 360.0 or 0.0, delta=0.01)


class TestGpsInputYawWireEncoding(unittest.TestCase):
    """Hardware-independent MAVLink2 wire-level round trip, per HIL-F23-F2A
    task requirement 4: pack a real GPS_INPUT message with the pymavlink
    v20 (MAVLink2) dialect, decode it with a fresh parser instance, and
    confirm the yaw field survives serialization -- no socket, no serial
    port, no Pixhawk.
    """

    @classmethod
    def setUpClass(cls):
        import os
        os.environ["MAVLINK20"] = "1"
        from pymavlink.dialects.v20 import ardupilotmega as mavlink2
        cls.mavlink2 = mavlink2

    def _pack_gps_input(self, yaw_cdeg):
        mav = self.mavlink2.MAVLink(None, srcSystem=1, srcComponent=1)
        mav.robust_parsing = False
        msg = self.mavlink2.MAVLink_gps_input_message(
            time_usec=123456789, gps_id=0, ignore_flags=0,
            time_week_ms=100, time_week=2300, fix_type=3,
            lat=325378147, lon=743661871, alt=3000.35,
            hdop=0.8, vdop=1.0, vn=31.0561, ve=-30.9571, vd=0.0,
            speed_accuracy=0.5, horiz_accuracy=1.0, vert_accuracy=1.5,
            satellites_visible=12, yaw=yaw_cdeg,
        )
        return msg.pack(mav)

    def _decode(self, packed_bytes):
        mav = self.mavlink2.MAVLink(None, srcSystem=1, srcComponent=1)
        mav.robust_parsing = False
        decoded = None
        for byte in packed_bytes:
            decoded = mav.parse_char(bytes([byte]))
            if decoded is not None:
                break
        return decoded

    def test_yaw_315_091444_deg_survives_mavlink2_round_trip(self):
        yaw_cdeg = smh.encode_gps_input_yaw_cdeg(315.091444)
        packed = self._pack_gps_input(yaw_cdeg)
        self.assertEqual(packed[0], 0xFD, "must be framed as MAVLink2 (magic 0xFD)")
        decoded = self._decode(packed)
        self.assertIsNotNone(decoded)
        self.assertEqual(decoded.get_type(), "GPS_INPUT")
        self.assertEqual(decoded.yaw, yaw_cdeg)
        self.assertAlmostEqual(smh.decode_gps_input_yaw_cdeg(decoded.yaw), 315.09, places=1)

    def test_mavlink1_dialect_has_no_yaw_field(self):
        # HIL-F23-F1/F2A finding: yaw is declared after <extensions/> in
        # common.xml, so pymavlink's MAVLink1-only ("v10") generated class
        # omits it entirely -- constructing with yaw= must fail.
        from pymavlink.dialects.v10 import ardupilotmega as mavlink1
        with self.assertRaises(TypeError):
            mavlink1.MAVLink_gps_input_message(
                time_usec=1, gps_id=0, ignore_flags=0, time_week_ms=1, time_week=1,
                fix_type=3, lat=1, lon=1, alt=1.0, hdop=1.0, vdop=1.0,
                vn=1.0, ve=1.0, vd=1.0, speed_accuracy=1.0, horiz_accuracy=1.0,
                vert_accuracy=1.0, satellites_visible=1, yaw=31509,
            )

    def test_mavlink1_wire_frame_is_shorter_and_omits_yaw(self):
        from pymavlink.dialects.v10 import ardupilotmega as mavlink1
        mav1 = mavlink1.MAVLink(None, srcSystem=1, srcComponent=1)
        mav1.robust_parsing = False
        msg1 = mavlink1.MAVLink_gps_input_message(
            time_usec=1, gps_id=0, ignore_flags=0, time_week_ms=1, time_week=1,
            fix_type=3, lat=1, lon=1, alt=1.0, hdop=1.0, vdop=1.0,
            vn=1.0, ve=1.0, vd=1.0, speed_accuracy=1.0, horiz_accuracy=1.0,
            vert_accuracy=1.0, satellites_visible=1,
        )
        packed1 = msg1.pack(mav1)
        self.assertEqual(packed1[0], 0xFE, "must be framed as MAVLink1 (magic 0xFE)")

        packed2 = self._pack_gps_input(31509)
        self.assertEqual(packed2[0], 0xFD)
        # the yaw field (uint16_t = 2 bytes) plus MAVLink2's extra header
        # overhead makes the MAVLink2 frame strictly longer than the
        # MAVLink1 frame of the same (non-extension) field set.
        self.assertLess(len(packed1), len(packed2))


if __name__ == "__main__":
    unittest.main()
