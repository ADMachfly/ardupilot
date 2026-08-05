#!/usr/bin/env python3
"""HIL-F23-F2A unit tests for GPS_INPUT.yaw wiring in
sr75_jsbsim_pixhawk_hil_bridge.py.

No hardware, no MAVLink connection, no serial port, no Pixhawk -- the
fake `master.mav.gps_input_send` below is a plain Python function with a
matching signature, used only to record what GPSInputInjector.send()
would transmit.
"""
import csv
import inspect
import io
import math
import tempfile
import time
import types
import unittest
from pathlib import Path

import sr75_jsbsim_pixhawk_hil_bridge as bridge


class _FakeMav:
    """Records gps_input_send() calls. has_yaw=False mimics the pymavlink
    v10 (MAVLink1-only) dialect, which has no yaw parameter at all --
    matching what GPSInputInjector must detect via inspect.signature(),
    not a hardcoded assumption.
    """

    def __init__(self, has_yaw=True):
        self.calls = []
        if has_yaw:
            def gps_input_send(time_usec, gps_id, ignore_flags, time_week_ms, time_week,
                                fix_type, lat, lon, alt, hdop, vdop, vn, ve, vd,
                                speed_accuracy, horiz_accuracy, vert_accuracy,
                                satellites_visible, yaw=0):
                self.calls.append(dict(locals()))
        else:
            def gps_input_send(time_usec, gps_id, ignore_flags, time_week_ms, time_week,
                                fix_type, lat, lon, alt, hdop, vdop, vn, ve, vd,
                                speed_accuracy, horiz_accuracy, vert_accuracy,
                                satellites_visible):
                self.calls.append(dict(locals()))
        self.gps_input_send = gps_input_send


def _make_master(has_yaw=True):
    master = types.SimpleNamespace()
    master.mav = _FakeMav(has_yaw=has_yaw)
    return master


def _live_row(lat=32.7285809, lon=74.1399025, alt=5000.0, vn=1.0, ve=1.0, vd=0.0, yaw_deg=None):
    row = bridge.blank_jsbsim_row()
    row.update({
        "jsb_time_s": "1.0",
        "jsb_lat_deg": lat,
        "jsb_lon_deg": lon,
        "jsb_alt_m": alt,
        "jsb_vn_mps": vn,
        "jsb_ve_mps": ve,
        "jsb_vd_mps": vd,
        "gps_input_yaw_deg": yaw_deg,
    })
    return row


class TestYawExtensionDetection(unittest.TestCase):
    """HIL-F23-F2A root cause: the bridge used to hardcode 'GPS_INPUT yaw
    extension available: no' regardless of reality. Detection must be
    real (inspect.signature), not hardcoded.
    """

    def test_detects_yaw_available_when_dialect_supports_it(self):
        master = _make_master(has_yaw=True)
        injector = bridge.GPSInputInjector(master, 5.0, False, 0, 0, False)
        self.assertTrue(injector.gps_input_yaw_param_available)

    def test_detects_yaw_unavailable_when_dialect_lacks_it(self):
        master = _make_master(has_yaw=False)
        injector = bridge.GPSInputInjector(master, 5.0, False, 0, 0, False)
        self.assertFalse(injector.gps_input_yaw_param_available)


class TestGpsInputSendYawDefaultDisabled(unittest.TestCase):
    """Default behaviour (send_yaw=False, i.e. no --gps-input-send-yaw)
    must be byte-for-byte identical to every previously-validated bridge
    command: yaw always encodes as 0 ("not available"), even when a
    real heading is present in the row.
    """

    def test_default_send_yaw_false_transmits_zero_even_with_real_heading(self):
        master = _make_master(has_yaw=True)
        injector = bridge.GPSInputInjector(master, 5.0, False, 0, 0, False)  # send_yaw defaults False
        ok = injector.send(_live_row(yaw_deg=315.091444))
        self.assertTrue(ok)
        self.assertEqual(len(master.mav.calls), 1)
        self.assertEqual(master.mav.calls[0]["yaw"], 0)


class TestGpsInputSendYawEnabled(unittest.TestCase):
    def test_send_yaw_true_encodes_mission_derived_heading(self):
        master = _make_master(has_yaw=True)
        injector = bridge.GPSInputInjector(master, 5.0, False, 0, 0, False, send_yaw=True)
        ok = injector.send(_live_row(yaw_deg=315.091444))
        self.assertTrue(ok)
        self.assertEqual(master.mav.calls[0]["yaw"], 31509)

    def test_send_yaw_true_but_row_has_no_yaw_key_sends_unavailable(self):
        master = _make_master(has_yaw=True)
        injector = bridge.GPSInputInjector(master, 5.0, False, 0, 0, False, send_yaw=True)
        row = _live_row(yaw_deg=None)
        ok = injector.send(row)
        self.assertTrue(ok)
        self.assertEqual(master.mav.calls[0]["yaw"], 0)

    def test_send_yaw_true_but_dialect_lacks_yaw_param_omits_kwarg_safely(self):
        # Must not raise TypeError even when send_yaw=True against a
        # MAVLink1-only dialect that has no yaw parameter at all.
        master = _make_master(has_yaw=False)
        injector = bridge.GPSInputInjector(master, 5.0, False, 0, 0, False, send_yaw=True)
        ok = injector.send(_live_row(yaw_deg=315.091444))
        self.assertTrue(ok)
        self.assertNotIn("yaw", master.mav.calls[0])


class TestStaticRowAndLiveFeedRowCarryYaw(unittest.TestCase):
    def test_static_row_uses_att_yaw_deg_when_send_yaw_enabled(self):
        args = types.SimpleNamespace(
            gps_input_static_lat=32.5378147, gps_input_static_lon=74.3661871,
            gps_input_static_alt_m=3000.35, gps_input_static_vn=0.0,
            gps_input_static_ve=0.0, gps_input_static_vd=0.0,
            att_yaw_deg=315.091444, gps_input_send_yaw=True,
        )
        row = bridge.static_gps_input_row(args)
        self.assertEqual(row["gps_input_yaw_deg"], 315.091444)

    def test_static_row_yaw_blank_when_send_yaw_disabled(self):
        # HIL-F23-F2B0A: schema-stable blank ("", not a numeric candidate
        # that was never sent) whenever --gps-input-send-yaw is off.
        args = types.SimpleNamespace(
            gps_input_static_lat=32.5378147, gps_input_static_lon=74.3661871,
            gps_input_static_alt_m=3000.35, gps_input_static_vn=0.0,
            gps_input_static_ve=0.0, gps_input_static_vd=0.0,
            att_yaw_deg=315.091444, gps_input_send_yaw=False,
        )
        row = bridge.static_gps_input_row(args)
        self.assertEqual(row["gps_input_yaw_deg"], "")

    def test_live_feed_row_converts_yaw_rad_to_deg_when_send_yaw_enabled(self):
        import math
        feeder = bridge.JSBSimStateFeeder("/tmp/does-not-need-to-exist-for-this-test.csv")
        feed_row = {
            "jsb_feed_time_s": 1.0, "jsb_feed_lat_deg": 32.7, "jsb_feed_lon_deg": 74.1,
            "jsb_feed_alt_m": 5000.0, "jsb_feed_vn_mps": 1.0, "jsb_feed_ve_mps": 1.0,
            "jsb_feed_vd_mps": 0.0, "jsb_feed_airspeed_mps": 40.0,
            "jsb_feed_roll_rad": 0.0, "jsb_feed_pitch_rad": 0.0,
            "jsb_feed_yaw_rad": math.radians(315.091444),
        }
        row = feeder._jsb_row_from_feed(feed_row, send_yaw=True)
        self.assertAlmostEqual(row["gps_input_yaw_deg"], 315.091444, places=5)

    def test_live_feed_row_yaw_blank_when_send_yaw_disabled(self):
        import math
        feeder = bridge.JSBSimStateFeeder("/tmp/does-not-need-to-exist-for-this-test.csv")
        feed_row = {
            "jsb_feed_time_s": 1.0, "jsb_feed_lat_deg": 32.7, "jsb_feed_lon_deg": 74.1,
            "jsb_feed_alt_m": 5000.0, "jsb_feed_vn_mps": 1.0, "jsb_feed_ve_mps": 1.0,
            "jsb_feed_vd_mps": 0.0, "jsb_feed_airspeed_mps": 40.0,
            "jsb_feed_roll_rad": 0.0, "jsb_feed_pitch_rad": 0.0,
            "jsb_feed_yaw_rad": math.radians(315.091444),
        }
        row = feeder._jsb_row_from_feed(feed_row)  # send_yaw defaults False
        self.assertEqual(row["gps_input_yaw_deg"], "")


class TestStaleJsbFeedStopsYawTransmission(unittest.TestCase):
    """HIL-F23-F2A test requirement: stale JSBSim state must stop yaw
    transmission -- yaw travels inside the same GPS_INPUT packet as
    lat/lon/vn/ve, so this reproduces the exact gating condition from
    sr75_jsbsim_pixhawk_hil_bridge.main()'s live-feed loop (the
    `if jsb_state_feeder.is_stale(): jsb_feed_send_row = None` block)
    against a real temp CSV file that stops advancing.
    """

    def test_stale_feed_row_is_none_so_gps_input_send_is_never_called(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            csv_path = Path(tmpdir) / "state.csv"
            csv_path.write_text(
                "Time,/fdm/jsbsim/simulation/sim-time-sec,/fdm/jsbsim/position/lat-gc-deg,"
                "/fdm/jsbsim/position/long-gc-deg,/fdm/jsbsim/position/h-sl-ft,"
                "/fdm/jsbsim/velocities/v-north-fps,/fdm/jsbsim/velocities/v-east-fps,"
                "/fdm/jsbsim/velocities/v-down-fps,/fdm/jsbsim/velocities/vt-fps,"
                "/fdm/jsbsim/attitude/phi-deg,/fdm/jsbsim/attitude/theta-deg,"
                "/fdm/jsbsim/attitude/psi-deg\n"
                "0.0,0.016666,32.7285809,74.1399025,16404,95.0,-95.0,0.0,140.0,0.0,20.0,315.091444\n"
            )
            args = types.SimpleNamespace(
                gps_input_static_vn=0.0, gps_input_static_ve=0.0, gps_input_static_vd=0.0,
                gps_input_send_yaw=True,
            )
            feeder = bridge.JSBSimStateFeeder(str(csv_path), stale_timeout_s=0.05)

            master = _make_master(has_yaw=True)
            injector = bridge.GPSInputInjector(master, 5.0, False, 0, 0, False, send_yaw=True)

            # first read: fresh row, not stale, GPS_INPUT (with yaw) sends fine.
            jsb_feed_send_row = feeder.read_latest(args)
            feeder.maybe_print_stale()
            if feeder.is_stale():
                jsb_feed_send_row = None
            self.assertIsNotNone(jsb_feed_send_row)
            self.assertTrue(injector.send(jsb_feed_send_row))
            self.assertEqual(master.mav.calls[-1]["yaw"], 31509)

            # CSV never advances again -- wait past stale_timeout_s, then
            # replicate main()'s exact gating condition.
            time.sleep(0.15)
            jsb_feed_send_row = feeder.read_latest(args)
            feeder.maybe_print_stale()
            if feeder.is_stale():
                jsb_feed_send_row = None
            self.assertIsNone(jsb_feed_send_row, "stale JSBSim state must suppress the entire GPS_INPUT "
                                                   "packet, including yaw, not just other fields")
            # confirm the injector was genuinely never called again (still 1 call from before).
            self.assertEqual(len(master.mav.calls), 1)


def _yaw_enabled_args(send_yaw):
    return types.SimpleNamespace(
        gps_input_static_lat=32.5378147, gps_input_static_lon=74.3661871,
        gps_input_static_alt_m=3000.35, gps_input_static_vn=0.0,
        gps_input_static_ve=0.0, gps_input_static_vd=0.0,
        att_yaw_deg=315.091444, gps_input_send_yaw=send_yaw,
    )


def _sample_feed_row():
    return {
        "jsb_feed_time_s": 1.0, "jsb_feed_lat_deg": 32.7, "jsb_feed_lon_deg": 74.1,
        "jsb_feed_alt_m": 5000.0, "jsb_feed_vn_mps": 1.0, "jsb_feed_ve_mps": 1.0,
        "jsb_feed_vd_mps": 0.0, "jsb_feed_airspeed_mps": 40.0,
        "jsb_feed_roll_rad": 0.0, "jsb_feed_pitch_rad": 0.0,
        "jsb_feed_yaw_rad": math.radians(315.091444),
    }


class TestCsvDictWriterRegressionGpsInputYaw(unittest.TestCase):
    """HIL-F23-F2B0A requirements 4 and 5: reproduces the exact hardware
    crash end-to-end -- GPS_INPUT yaw enabled, a real row built the way
    the live loop builds it, written through a REAL csv.DictWriter using
    the bridge's own frozen fieldnames list. Must not raise ValueError.
    """

    def _fieldnames(self):
        return list(bridge.make_row(time.time(), {}).keys())

    def test_send_yaw_enabled_row_writes_without_valueerror_and_shows_yaw(self):
        fieldnames = self._fieldnames()
        self.assertIn("gps_input_yaw_deg", fieldnames)

        row = bridge.make_row(
            time.time(), {}, jsbsim_row=bridge.static_gps_input_row(_yaw_enabled_args(True)),
        )
        buf = io.StringIO()
        writer = csv.DictWriter(buf, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerow(row)  # must not raise ValueError

        self.assertEqual(row["gps_input_yaw_deg"], 315.091444)
        self.assertIn("315.091444", buf.getvalue())

    def test_send_yaw_disabled_row_writes_blank_yaw_without_valueerror(self):
        fieldnames = self._fieldnames()
        row = bridge.make_row(
            time.time(), {}, jsbsim_row=bridge.static_gps_input_row(_yaw_enabled_args(False)),
        )
        buf = io.StringIO()
        writer = csv.DictWriter(buf, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerow(row)  # must not raise ValueError

        self.assertEqual(row["gps_input_yaw_deg"], "")

    def test_live_feed_row_with_yaw_writes_without_valueerror(self):
        fieldnames = self._fieldnames()
        feeder = bridge.JSBSimStateFeeder("/tmp/does-not-need-to-exist-for-this-test.csv")
        row = bridge.make_row(
            time.time(), {},
            jsbsim_row=feeder._jsb_row_from_feed(_sample_feed_row(), send_yaw=True),
        )
        buf = io.StringIO()
        writer = csv.DictWriter(buf, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerow(row)  # must not raise ValueError
        self.assertAlmostEqual(row["gps_input_yaw_deg"], 315.091444, places=5)


class TestCsvSchemaConsistency(unittest.TestCase):
    """HIL-F23-F2B0A requirement 6: every key any row builder can produce
    must already be present in blank_jsbsim_row() / the frozen CSV
    fieldnames snapshot -- the general regression guard behind the
    specific gps_input_yaw_deg fix above.
    """

    def test_gps_input_yaw_deg_present_in_blank_jsbsim_row(self):
        self.assertIn("gps_input_yaw_deg", bridge.blank_jsbsim_row())

    def test_static_row_keys_never_exceed_blank_jsbsim_row_keys(self):
        expected = set(bridge.blank_jsbsim_row().keys())
        for send_yaw in (True, False):
            row = bridge.static_gps_input_row(_yaw_enabled_args(send_yaw))
            self.assertEqual(set(row.keys()), expected)

    def test_live_feed_row_keys_never_exceed_blank_jsbsim_row_keys(self):
        expected = set(bridge.blank_jsbsim_row().keys())
        feeder = bridge.JSBSimStateFeeder("/tmp/does-not-need-to-exist-for-this-test.csv")
        for send_yaw in (True, False):
            row = feeder._jsb_row_from_feed(_sample_feed_row(), send_yaw=send_yaw)
            self.assertEqual(set(row.keys()), expected)

    def test_frozen_fieldnames_accept_every_row_builder_combination(self):
        # Reproduces main()'s exact fieldnames-freezing pattern, then
        # exercises every jsbsim_row shape the live loop can actually
        # produce through a real csv.DictWriter.
        start_time = time.time()
        fieldnames = list(bridge.make_row(start_time, {}).keys())
        buf = io.StringIO()
        writer = csv.DictWriter(buf, fieldnames=fieldnames)
        writer.writeheader()

        feeder = bridge.JSBSimStateFeeder("/tmp/does-not-need-to-exist-for-this-test.csv")
        candidate_jsbsim_rows = [
            None,
            bridge.blank_jsbsim_row(),
            bridge.static_gps_input_row(_yaw_enabled_args(True)),
            bridge.static_gps_input_row(_yaw_enabled_args(False)),
            feeder._jsb_row_from_feed(_sample_feed_row(), send_yaw=True),
            feeder._jsb_row_from_feed(_sample_feed_row(), send_yaw=False),
        ]
        for jsbsim_row in candidate_jsbsim_rows:
            row = bridge.make_row(start_time, {}, jsbsim_row=jsbsim_row)
            try:
                writer.writerow(row)
            except ValueError as exc:
                self.fail(f"csv.DictWriter rejected a real row shape ({jsbsim_row}): {exc}")


if __name__ == "__main__":
    unittest.main()
