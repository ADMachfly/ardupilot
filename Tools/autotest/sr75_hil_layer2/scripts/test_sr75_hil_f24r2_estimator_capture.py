#!/usr/bin/env python3
"""HIL-F24-R2 regression tests: read-only estimator/truth capture.

Covers:
  - mavlink_message_to_row() against synthetic (constructed, not
    device-read) pymavlink message objects, for every recorded message
    type;
  - TruthTailer against a synthetic state.csv and against the REAL
    HIL-F24-R1 JSBSim feeder (where JSBSim is installed);
  - a full end-to-end run of main() against a PTY-based fake Pixhawk
    (a real pseudo-terminal "serial" connection fed synthetic MAVLink
    bytes -- not a real device) combined with the real R1 feeder,
    producing genuine jsbsim_truth.csv/pixhawk_estimator.csv files that
    are then fed through the real comparison pipeline.

No real hardware, no real serial device, no PARAM_SET/arm/mode/mission/
RC-override/actuator command is ever sent by these tests.
"""
import csv
import json
import math
import os
import pty
import shutil
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
CAPTURE_SCRIPT = HERE / "sr75_hil_f24r2_estimator_capture.py"
COMPARISON_SCRIPT = HERE / "sr75_hil_f24r2_estimator_comparison.py"
SIM_JSON_DIR = HERE.parent / "sim_json"
FEEDER_SCRIPT = SIM_JSON_DIR / "sr75_hil_f24r1_dynamic_jsbsim_feed.py"
sys.path.insert(0, str(SIM_JSON_DIR))

import sr75_hil_f24r2_estimator_capture as cap  # noqa: E402
import sr75_hil_f24d_static_state_feed as feed  # noqa: E402

JSBSIM_AVAILABLE = shutil.which("JSBSim") is not None

import pymavlink.dialects.v20.ardupilotmega as mavlink2  # noqa: E402
from pymavlink import mavutil  # noqa: E402


class TestMavlinkMessageToRow(unittest.TestCase):
    def test_heartbeat(self):
        msg = mavlink2.MAVLink_heartbeat_message(
            type=1, autopilot=3, base_mode=81, custom_mode=0, system_status=3, mavlink_version=3,
        )
        row = cap.mavlink_message_to_row(msg, 1.0)
        self.assertEqual(row["msg_type"], "HEARTBEAT")
        self.assertEqual(row["hb_armed"], 0.0)
        self.assertEqual(row["hb_mode"], "MANUAL")
        self.assertEqual(row["hb_mode_is_manual"], 1.0)

    def test_heartbeat_armed(self):
        armed_base_mode = 81 | mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        msg = mavlink2.MAVLink_heartbeat_message(
            type=1, autopilot=3, base_mode=armed_base_mode, custom_mode=0, system_status=3, mavlink_version=3,
        )
        row = cap.mavlink_message_to_row(msg, 1.0)
        self.assertEqual(row["hb_armed"], 1.0)

    def test_global_position_int_unit_conversions(self):
        msg = mavlink2.MAVLink_global_position_int_message(
            time_boot_ms=0, lat=325378085, lon=743661944, alt=170, relative_alt=170,
            vx=10, vy=-5, vz=1, hdg=18000,
        )
        row = cap.mavlink_message_to_row(msg, 2.0)
        self.assertAlmostEqual(row["gpi_lat_deg"], 32.5378085, places=6)
        self.assertAlmostEqual(row["gpi_lon_deg"], 74.3661944, places=6)
        self.assertAlmostEqual(row["gpi_alt_m"], 0.17, places=6)
        self.assertAlmostEqual(row["gpi_vn_mps"], 0.1, places=6)
        self.assertAlmostEqual(row["gpi_ve_mps"], -0.05, places=6)
        self.assertAlmostEqual(row["gpi_hdg_deg"], 180.0, places=6)

    def test_global_position_int_hdg_unknown_sentinel_left_none(self):
        msg = mavlink2.MAVLink_global_position_int_message(
            time_boot_ms=0, lat=0, lon=0, alt=0, relative_alt=0, vx=0, vy=0, vz=0, hdg=65535,
        )
        row = cap.mavlink_message_to_row(msg, 1.0)
        self.assertIsNone(row["gpi_hdg_deg"])

    def test_attitude_radians_to_degrees(self):
        msg = mavlink2.MAVLink_attitude_message(
            time_boot_ms=0, roll=0.1, pitch=-0.05, yaw=1.2, rollspeed=0.01, pitchspeed=-0.02, yawspeed=0.0,
        )
        row = cap.mavlink_message_to_row(msg, 1.0)
        self.assertAlmostEqual(row["att_roll_deg"], math.degrees(0.1), places=6)
        self.assertAlmostEqual(row["att_pitch_deg"], math.degrees(-0.05), places=6)
        self.assertAlmostEqual(row["att_yaw_deg"], math.degrees(1.2), places=6)

    def test_gps_raw_int(self):
        msg = mavlink2.MAVLink_gps_raw_int_message(
            time_usec=0, fix_type=3, lat=325378085, lon=743661944, alt=170,
            eph=100, epv=100, vel=0, cog=0, satellites_visible=12,
        )
        row = cap.mavlink_message_to_row(msg, 1.0)
        self.assertEqual(row["gri_fix_type"], 3.0)
        self.assertEqual(row["gri_satellites_visible"], 12.0)

    def test_vfr_hud(self):
        msg = mavlink2.MAVLink_vfr_hud_message(
            airspeed=1.5, groundspeed=1.2, heading=90, throttle=0, alt=0.17, climb=0.0,
        )
        row = cap.mavlink_message_to_row(msg, 1.0)
        self.assertEqual(row["vfr_airspeed_mps"], 1.5)

    def test_ekf_status_report(self):
        msg = mavlink2.MAVLink_ekf_status_report_message(
            flags=31, velocity_variance=0.01, pos_horiz_variance=0.02,
            pos_vert_variance=0.01, compass_variance=0.001, terrain_alt_variance=0.0, airspeed_variance=0.0,
        )
        row = cap.mavlink_message_to_row(msg, 1.0)
        self.assertEqual(row["ekf_flags"], 31.0)

    def test_sys_status(self):
        msg = mavlink2.MAVLink_sys_status_message(
            onboard_control_sensors_present=0, onboard_control_sensors_enabled=0,
            onboard_control_sensors_health=0xFFFFFFFF, load=100, voltage_battery=12000, current_battery=-1,
            battery_remaining=80, drop_rate_comm=0, errors_comm=0,
            errors_count1=0, errors_count2=0, errors_count3=0, errors_count4=0,
        )
        row = cap.mavlink_message_to_row(msg, 1.0)
        self.assertEqual(row["sys_battery_remaining"], 80.0)

    def test_statustext(self):
        msg = mavlink2.MAVLink_statustext_message(severity=6, text=b"ArduPlane V4.5.0", id=0, chunk_seq=0)
        row = cap.mavlink_message_to_row(msg, 1.0)
        self.assertEqual(row["statustext_text"], "ArduPlane V4.5.0")

    def test_unrecognized_message_type_returns_none(self):
        msg = mavlink2.MAVLink_ping_message(time_usec=0, seq=0, target_system=0, target_component=0)
        self.assertIsNone(cap.mavlink_message_to_row(msg, 1.0))


class FakeUdpSocket:
    def __init__(self, *args):
        self.args = args
        self.sent = []
        self.closed = False
        self.recv_called = False

    def sendto(self, packet, endpoint):
        self.sent.append((bytes(packet), endpoint))
        return len(packet)

    def recvfrom(self, _size):
        self.recv_called = True
        raise AssertionError("Mission Planner fanout must never read UDP commands")

    def close(self):
        self.closed = True


class TestMavlinkUdpForwarder(unittest.TestCase):
    def _message(self):
        mav = mavutil.mavlink.MAVLink(None, srcSystem=1, srcComponent=1)
        msg = mavlink2.MAVLink_heartbeat_message(
            type=1, autopilot=3, base_mode=81, custom_mode=0, system_status=3, mavlink_version=3,
        )
        expected = bytes(msg.pack(mav))
        return msg, expected

    def test_packets_are_copied_to_udp(self):
        fake_socket = FakeUdpSocket()
        forwarder = cap.MavlinkUdpForwarder("192.168.1.20", 14550, socket_factory=lambda *_args: fake_socket)
        msg, expected = self._message()
        self.assertTrue(forwarder.forward_message(msg))
        self.assertEqual(fake_socket.sent, [(expected, ("192.168.1.20", 14550))])
        self.assertEqual(forwarder.packet_count, 1)
        self.assertEqual(forwarder.byte_count, len(expected))

    def test_forwarder_has_no_udp_command_read_path(self):
        fake_socket = FakeUdpSocket()
        forwarder = cap.MavlinkUdpForwarder("127.0.0.1", 14550, socket_factory=lambda *_args: fake_socket)
        msg, _expected = self._message()
        forwarder.forward_message(msg)
        self.assertFalse(fake_socket.recv_called)

    def test_clean_shutdown_closes_udp_socket(self):
        fake_socket = FakeUdpSocket()
        forwarder = cap.MavlinkUdpForwarder("127.0.0.1", 14550, socket_factory=lambda *_args: fake_socket)
        forwarder.close()
        self.assertTrue(fake_socket.closed)


class TestTruthTailer(unittest.TestCase):
    def test_reports_new_row_once(self):
        tmpdir = Path(tempfile.mkdtemp(prefix="hil_f24r2_tailer_"))
        path = str(tmpdir / "state.csv")
        feed.atomic_write_csv_snapshot(path, feed.CSV_FIELDS, feed.build_static_row(0.0))
        tailer = cap.TruthTailer(path)
        row1 = tailer.poll()
        self.assertIsNotNone(row1)
        self.assertEqual(row1["lat_deg"], feed.build_static_row(0.0)["lat_deg"])
        row2 = tailer.poll()
        self.assertIsNone(row2)  # unchanged since last poll

    def test_reports_each_distinct_new_row(self):
        tmpdir = Path(tempfile.mkdtemp(prefix="hil_f24r2_tailer2_"))
        path = str(tmpdir / "state.csv")
        tailer = cap.TruthTailer(path)
        feed.atomic_write_csv_snapshot(path, feed.CSV_FIELDS, feed.build_static_row(0.0))
        self.assertIsNotNone(tailer.poll())
        feed.atomic_write_csv_snapshot(path, feed.CSV_FIELDS, feed.build_static_row(0.02))
        row = tailer.poll()
        self.assertIsNotNone(row)
        self.assertEqual(row["time_s"], feed.build_static_row(0.02)["time_s"])

    def test_missing_file_returns_none_not_a_crash(self):
        tailer = cap.TruthTailer("/tmp/hil_f24r2_definitely_missing_state.csv")
        self.assertIsNone(tailer.poll())

    @unittest.skipUnless(JSBSIM_AVAILABLE, "JSBSim binary not found on PATH")
    def test_against_real_r1_feeder(self):
        tmpdir = Path(tempfile.mkdtemp(prefix="hil_f24r2_tailer_real_"))
        state_csv = tmpdir / "state.csv"
        proc = subprocess.Popen(
            [sys.executable, str(FEEDER_SCRIPT), "--output", str(state_csv), "--duration-s", "4"],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
        )
        try:
            deadline = time.monotonic() + 10.0
            while not state_csv.exists() and time.monotonic() < deadline:
                time.sleep(0.05)
            self.assertTrue(state_csv.exists())
            tailer = cap.TruthTailer(str(state_csv))
            rows = []
            start = time.monotonic()
            while time.monotonic() - start < 2.0:
                row = tailer.poll()
                if row is not None:
                    rows.append(row)
                time.sleep(0.005)
        finally:
            proc.terminate()
            try:
                proc.communicate(timeout=5.0)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.communicate()
        self.assertGreater(len(rows), 50)  # ~50Hz for 2s
        for row in rows:
            self.assertTrue(math.isfinite(float(row["alt_m"])))


class FakeMavlinkPixhawk:
    """HIL-F24-R2 test helper: a real pseudo-terminal presented as a
    "serial" device (openable by path via mavutil.mavlink_connection,
    exactly like a real /dev/ttyACM0), fed synthetic MAVLink messages
    from a background thread. Not a real device, not real hardware."""

    def __init__(self):
        self.master_fd, self.slave_fd = pty.openpty()
        self.device_path = os.ttyname(self.slave_fd)
        self._mav = mavutil.mavlink.MAVLink(None, srcSystem=1, srcComponent=1)
        self._stop = threading.Event()
        self._thread = None

    def _send(self, msg):
        os.write(self.master_fd, msg.pack(self._mav))

    def _loop(self):
        t = 0.0
        while not self._stop.is_set():
            self._send(mavlink2.MAVLink_heartbeat_message(
                type=1, autopilot=3, base_mode=81, custom_mode=0, system_status=3, mavlink_version=3,
            ))
            self._send(mavlink2.MAVLink_global_position_int_message(
                time_boot_ms=int(t * 1000), lat=325378085, lon=743661944, alt=170,
                relative_alt=170, vx=0, vy=0, vz=0, hdg=int(315.0 * 100),
            ))
            self._send(mavlink2.MAVLink_attitude_message(
                time_boot_ms=int(t * 1000), roll=0.0, pitch=-0.0316, yaw=5.4977871,
                rollspeed=0.0, pitchspeed=0.0, yawspeed=0.0,
            ))
            self._send(mavlink2.MAVLink_vfr_hud_message(
                airspeed=0.05, groundspeed=0.0, heading=315, throttle=0, alt=0.17, climb=0.0,
            ))
            self._send(mavlink2.MAVLink_ekf_status_report_message(
                flags=31, velocity_variance=0.01, pos_horiz_variance=0.02,
                pos_vert_variance=0.01, compass_variance=0.001, terrain_alt_variance=0.0, airspeed_variance=0.0,
            ))
            t += 0.1
            time.sleep(0.1)

    def start(self):
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        for fd in (self.master_fd, self.slave_fd):
            try:
                os.close(fd)
            except OSError:
                pass


class TestCaptureMainEndToEnd(unittest.TestCase):
    """Task 9: host-only end-to-end test using synthetic MAVLink (PTY fake
    Pixhawk) and, where JSBSim is installed, the real R1 feeder."""

    def test_capture_against_fake_pixhawk_and_synthetic_truth(self):
        fake = FakeMavlinkPixhawk()
        fake.start()
        tmpdir = Path(tempfile.mkdtemp(prefix="hil_f24r2_capture_e2e_"))
        state_csv = tmpdir / "state.csv"
        feed.atomic_write_csv_snapshot(str(state_csv), feed.CSV_FIELDS, feed.build_static_row(0.0))
        truth_csv = tmpdir / "jsbsim_truth.csv"
        pixhawk_csv = tmpdir / "pixhawk_estimator.csv"
        try:
            proc = subprocess.run(
                [sys.executable, str(CAPTURE_SCRIPT),
                 "--pixhawk", fake.device_path, "--baud", "115200",
                 "--state-file", str(state_csv), "--duration-s", "3",
                 "--jsbsim-truth-csv", str(truth_csv), "--pixhawk-estimator-csv", str(pixhawk_csv)],
                capture_output=True, text=True, timeout=30,
            )
        finally:
            fake.stop()

        self.assertEqual(proc.returncode, 0, proc.stdout + proc.stderr)
        self.assertIn("ESTIMATOR_CAPTURE_SUMMARY", proc.stdout)
        self.assertTrue(truth_csv.exists())
        self.assertTrue(pixhawk_csv.exists())

        with open(pixhawk_csv, newline="") as f:
            rows = list(csv.DictReader(f))
        msg_types = {row["msg_type"] for row in rows}
        self.assertIn("HEARTBEAT", msg_types)
        self.assertIn("GLOBAL_POSITION_INT", msg_types)
        self.assertIn("ATTITUDE", msg_types)
        self.assertIn("VFR_HUD", msg_types)
        self.assertIn("EKF_STATUS_REPORT", msg_types)
        self.assertGreater(len(rows), 10)

    @unittest.skipUnless(JSBSIM_AVAILABLE, "JSBSim binary not found on PATH")
    def test_full_pipeline_real_r1_feeder_plus_fake_pixhawk_plus_comparison(self):
        """The strongest available host-only proof: real JSBSim (via the
        real R1 feeder) for truth, a PTY fake Pixhawk for the estimator
        stream, the real capture script wiring both together, and the
        real comparison script scoring the result -- with zero real
        hardware anywhere in the chain."""
        tmpdir = Path(tempfile.mkdtemp(prefix="hil_f24r2_full_pipeline_"))
        state_csv = tmpdir / "state.csv"
        truth_csv = tmpdir / "jsbsim_truth.csv"
        pixhawk_csv = tmpdir / "pixhawk_estimator.csv"
        comparison_csv = tmpdir / "estimator_comparison.csv"
        summary_json = tmpdir / "estimator_summary.json"
        summary_md = tmpdir / "estimator_summary.md"

        feeder_proc = subprocess.Popen(
            [sys.executable, str(FEEDER_SCRIPT), "--output", str(state_csv), "--duration-s", "10"],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
        )
        fake = FakeMavlinkPixhawk()
        try:
            deadline = time.monotonic() + 10.0
            while not state_csv.exists() and time.monotonic() < deadline:
                time.sleep(0.05)
            self.assertTrue(state_csv.exists())

            fake.start()
            capture_proc = subprocess.run(
                [sys.executable, str(CAPTURE_SCRIPT),
                 "--pixhawk", fake.device_path, "--baud", "115200",
                 "--state-file", str(state_csv), "--duration-s", "5",
                 "--jsbsim-truth-csv", str(truth_csv), "--pixhawk-estimator-csv", str(pixhawk_csv)],
                capture_output=True, text=True, timeout=30,
            )
        finally:
            fake.stop()
            feeder_proc.terminate()
            try:
                feeder_proc.communicate(timeout=5.0)
            except subprocess.TimeoutExpired:
                feeder_proc.kill()
                feeder_proc.communicate()

        self.assertEqual(capture_proc.returncode, 0, capture_proc.stdout + capture_proc.stderr)

        comparison_proc = subprocess.run(
            [sys.executable, str(COMPARISON_SCRIPT),
             "--truth-csv", str(truth_csv), "--pixhawk-csv", str(pixhawk_csv),
             "--comparison-csv", str(comparison_csv),
             "--summary-json", str(summary_json), "--summary-md", str(summary_md),
             "--settle-s", "0"],  # short capture window, don't exclude everything
            capture_output=True, text=True,
        )
        self.assertIn("ESTIMATOR_COMPARISON_RESULT", comparison_proc.stdout, comparison_proc.stdout + comparison_proc.stderr)
        self.assertTrue(comparison_csv.exists())
        self.assertTrue(summary_json.exists())
        # The fake Pixhawk reports a fixed, physically-plausible ground
        # state matching the R1 IC closely -- this should score well.
        # Not asserting overall_pass here (a synthetic fake estimator
        # stream is not a claim about real hardware); asserting the
        # pipeline ran to completion and produced sane, finite output.
        with open(summary_json) as f:
            summary = json.load(f)
        self.assertGreater(summary["sample_counts"]["GLOBAL_POSITION_INT"], 0)
        self.assertIsNotNone(summary["metrics"]["horizontal_error_m"]["rms"])


if __name__ == "__main__":
    unittest.main()
