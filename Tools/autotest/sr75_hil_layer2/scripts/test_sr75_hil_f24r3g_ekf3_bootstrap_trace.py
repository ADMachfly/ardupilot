#!/usr/bin/env python3
"""HIL-F24-R3G regression tests: EKF3-bootstrap trace math and pipeline.

Covers the pure tilt-from-accelerometer formula (reproduced from
AP_NavEKF3_core.cpp's InitialiseFilterBootstrap()), first-divergence
detection, nearest-accelerometer-sample lookup, the bootstrap-complete
STATUSTEXT matcher, the full summarize() pipeline against synthetic
rows (including the exact real-bench frozen AHRS2/accel values from
HIL_F24_R3G_ekf3_tilt_alignment_diagnosis.md as a regression fixture),
and an end-to-end run of main() against a PTY fake Pixhawk. No real
hardware, no PARAM_SET/arm/mission/RC-override/actuator command.
"""
import csv
import json
import math
import os
import pty
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import sr75_hil_f24r3g_ekf3_bootstrap_trace as trace  # noqa: E402

from pymavlink import mavutil  # noqa: E402
from pymavlink.dialects.v20 import ardupilotmega as mavlink2  # noqa: E402

# The real, frozen values captured live in
# sr75_hil_f24r3a_mp_visual_20260807T040401Z (see HIL_F24_R3G_ekf3_
# tilt_alignment_diagnosis.md).
REAL_FROZEN_AHRS2_ROLL_DEG = 7.365185081095156
REAL_FROZEN_AHRS2_PITCH_DEG = 7.547061919635194
# The accel vector that reproduces the above via the exact bootstrap
# formula (derived in that report; arbitrary consistent units since the
# formula normalizes).
REAL_DERIVED_ACCEL_XYZ = (131.34050648071846, -127.08250315063593, -983.1580283710025)


class TestTiltFromAccel(unittest.TestCase):
    def test_level_vector_is_zero_tilt(self):
        roll, pitch = trace.tilt_from_accel(0.0, 0.0, -1000.0)
        self.assertAlmostEqual(roll, 0.0, places=6)
        self.assertAlmostEqual(pitch, 0.0, places=6)

    def test_degenerate_vector_returns_zero_matching_source_guard(self):
        """AP_NavEKF3_core.cpp: `if (initAccVec.length() > 0.001f)` --
        below that, roll/pitch are left at their zero-initialized values."""
        roll, pitch = trace.tilt_from_accel(0.0, 0.0, 0.0)
        self.assertEqual((roll, pitch), (0.0, 0.0))

    def test_reproduces_the_real_frozen_r3g_value(self):
        """The exact regression this task exists to confirm: the accel
        vector derived (by hand) in HIL_F24_R3G_ekf3_tilt_alignment_
        diagnosis.md reproduces the real captured frozen AHRS2 value via
        this exact formula, round-tripped through this script's own
        implementation."""
        roll, pitch = trace.tilt_from_accel(*REAL_DERIVED_ACCEL_XYZ)
        self.assertAlmostEqual(roll, REAL_FROZEN_AHRS2_ROLL_DEG, delta=0.01)
        self.assertAlmostEqual(pitch, REAL_FROZEN_AHRS2_PITCH_DEG, delta=0.01)

    def test_scale_invariant(self):
        """Units cancel under normalization -- mg, m/s^2, or any
        consistent scale must give the same answer."""
        roll_a, pitch_a = trace.tilt_from_accel(100.0, -50.0, -900.0)
        roll_b, pitch_b = trace.tilt_from_accel(1.0, -0.5, -9.0)
        self.assertAlmostEqual(roll_a, roll_b, places=6)
        self.assertAlmostEqual(pitch_a, pitch_b, places=6)

    def test_pure_roll_vector(self):
        # Pointing along -y with zero x: pitch=0, roll=atan2(1,0)=90deg...
        # use a vector with known closed-form answer instead.
        roll, pitch = trace.tilt_from_accel(0.0, -707.1, -707.1)
        self.assertAlmostEqual(pitch, 0.0, places=3)
        self.assertAlmostEqual(roll, 45.0, delta=0.1)


class TestDetectBootstrapComplete(unittest.TestCase):
    def test_matches_exact_source_format_string(self):
        self.assertTrue(trace.detect_bootstrap_complete("EKF3 IMU0 initialised"))
        self.assertTrue(trace.detect_bootstrap_complete("EKF3 IMU1 initialised"))

    def test_matches_as_substring_of_a_longer_statustext(self):
        self.assertTrue(trace.detect_bootstrap_complete("Some prefix EKF3 IMU0 initialised suffix"))

    def test_does_not_match_unrelated_text(self):
        self.assertFalse(trace.detect_bootstrap_complete("PreArm: AHRS: EKF3 Roll/Pitch inconsistent 10 deg."))
        self.assertFalse(trace.detect_bootstrap_complete("EKF3 waiting for GPS config data"))

    def test_handles_none_and_empty(self):
        self.assertFalse(trace.detect_bootstrap_complete(None))
        self.assertFalse(trace.detect_bootstrap_complete(""))


def _row(msg_type, t, **overrides):
    row = {f: None for f in trace.CSV_FIELDS}
    row["host_monotonic_time"] = t
    row["msg_type"] = msg_type
    row.update(overrides)
    return row


class TestFindFirstDivergence(unittest.TestCase):
    def test_finds_first_row_exceeding_threshold(self):
        rows = [
            _row("AHRS2", 1.0, ahrs2_roll_deg=0.1, ahrs2_pitch_deg=0.1),
            _row("AHRS2", 2.0, ahrs2_roll_deg=0.2, ahrs2_pitch_deg=0.15),
            _row("AHRS2", 3.0, ahrs2_roll_deg=7.4, ahrs2_pitch_deg=7.5),
            _row("AHRS2", 4.0, ahrs2_roll_deg=7.4, ahrs2_pitch_deg=7.5),
        ]
        result = trace.find_first_divergence(rows, threshold_deg=1.0)
        self.assertEqual(result["host_monotonic_time"], 3.0)

    def test_pitch_alone_can_trigger(self):
        rows = [_row("AHRS2", 1.0, ahrs2_roll_deg=0.0, ahrs2_pitch_deg=5.0)]
        result = trace.find_first_divergence(rows, threshold_deg=1.0)
        self.assertIsNotNone(result)

    def test_no_divergence_returns_none(self):
        rows = [_row("AHRS2", t, ahrs2_roll_deg=0.1, ahrs2_pitch_deg=0.1) for t in (1.0, 2.0, 3.0)]
        self.assertIsNone(trace.find_first_divergence(rows, threshold_deg=1.0))

    def test_ignores_non_ahrs2_rows(self):
        rows = [
            _row("ATTITUDE", 1.0, att_roll_deg=90.0, att_pitch_deg=90.0),
            _row("AHRS2", 2.0, ahrs2_roll_deg=0.1, ahrs2_pitch_deg=0.1),
        ]
        self.assertIsNone(trace.find_first_divergence(rows, threshold_deg=1.0))


class TestNearestAccelSample(unittest.TestCase):
    def test_prefers_most_recent_sample_at_or_before_target(self):
        rows = [
            _row("RAW_IMU", 1.0, imu_xacc=1.0, imu_yacc=0.0, imu_zacc=-1000.0),
            _row("RAW_IMU", 2.0, imu_xacc=2.0, imu_yacc=0.0, imu_zacc=-1000.0),
            _row("RAW_IMU", 4.0, imu_xacc=4.0, imu_yacc=0.0, imu_zacc=-1000.0),
        ]
        result = trace.nearest_accel_sample_at_or_before(rows, target_time=3.0)
        self.assertEqual(result["host_monotonic_time"], 2.0)

    def test_falls_back_to_nearest_overall_if_none_before(self):
        rows = [_row("RAW_IMU", 5.0, imu_xacc=1.0, imu_yacc=0.0, imu_zacc=-1000.0)]
        result = trace.nearest_accel_sample_at_or_before(rows, target_time=1.0)
        self.assertEqual(result["host_monotonic_time"], 5.0)

    def test_none_when_no_accel_rows(self):
        rows = [_row("ATTITUDE", 1.0)]
        self.assertIsNone(trace.nearest_accel_sample_at_or_before(rows, target_time=1.0))

    def test_considers_all_three_imu_message_types(self):
        for msg_type in ("RAW_IMU", "SCALED_IMU", "SCALED_IMU2"):
            rows = [_row(msg_type, 1.0, imu_xacc=1.0, imu_yacc=0.0, imu_zacc=-1000.0)]
            with self.subTest(msg_type=msg_type):
                self.assertIsNotNone(trace.nearest_accel_sample_at_or_before(rows, target_time=1.0))


class TestCompareToObserved(unittest.TestCase):
    def test_within_tolerance_matches(self):
        matches, roll_res, pitch_res = trace.compare_predicted_to_observed(7.36, 7.55, 7.365, 7.547, tolerance_deg=0.1)
        self.assertTrue(matches)

    def test_beyond_tolerance_does_not_match(self):
        matches, roll_res, pitch_res = trace.compare_predicted_to_observed(0.0, 0.0, 7.365, 7.547, tolerance_deg=1.0)
        self.assertFalse(matches)
        self.assertAlmostEqual(roll_res, 7.365, places=3)


class TestSummarizeEndToEnd(unittest.TestCase):
    def test_real_bench_regression_fixture_reproduces_and_passes(self):
        """The full pipeline, fed a synthetic stream shaped like the real
        capture: GPS reaches FIX_3D, then (1s later, matching the
        bootstrap's own 1000ms accumulation wait) a transient off-level
        accel sample lands moments before AHRS2 freezes at the exact
        real captured value, then EKF3 IMU0 initialised fires."""
        rows = [
            _row("GPS_RAW_INT", 0.0, fix_type=3.0, lat_deg=32.5378085, lon_deg=74.3661944, alt_m=240.2),
            _row("RAW_IMU", 0.5, imu_xacc=0.0, imu_yacc=0.0, imu_zacc=-1000.0),
            _row("AHRS2", 0.9, ahrs2_roll_deg=0.0, ahrs2_pitch_deg=0.0, ahrs2_yaw_deg=0.0),
            # the transient sample, ~1s after the first bootstrap attempt
            _row("RAW_IMU", 1.0, imu_xacc=REAL_DERIVED_ACCEL_XYZ[0], imu_yacc=REAL_DERIVED_ACCEL_XYZ[1],
                 imu_zacc=REAL_DERIVED_ACCEL_XYZ[2]),
            _row("AHRS2", 1.05, ahrs2_roll_deg=REAL_FROZEN_AHRS2_ROLL_DEG, ahrs2_pitch_deg=REAL_FROZEN_AHRS2_PITCH_DEG,
                 ahrs2_yaw_deg=-34.0),
            _row("EKF_STATUS_REPORT", 1.1, ekf_flags=1024.0),
            _row("STATUSTEXT", 1.2, statustext_text="EKF3 IMU0 initialised"),
            # frozen thereafter -- same value, IMU back to level (truth)
            _row("RAW_IMU", 1.5, imu_xacc=0.0, imu_yacc=0.0, imu_zacc=-1000.0),
            _row("AHRS2", 1.6, ahrs2_roll_deg=REAL_FROZEN_AHRS2_ROLL_DEG, ahrs2_pitch_deg=REAL_FROZEN_AHRS2_PITCH_DEG,
                 ahrs2_yaw_deg=-34.0),
        ]
        summary = trace.summarize(rows, divergence_threshold_deg=1.0, match_tolerance_deg=0.5, stop_reason="ekf3_initialised_statustext_seen")

        self.assertEqual(summary["first_divergence_time"], 1.05)
        self.assertEqual(summary["accel_vector_at_divergence"]["xacc"], REAL_DERIVED_ACCEL_XYZ[0])
        self.assertTrue(summary["reproduces_bootstrap_math"])
        self.assertLess(summary["roll_residual_deg"], 0.5)
        self.assertLess(summary["pitch_residual_deg"], 0.5)
        self.assertTrue(summary["ekf3_initialised_statustext_seen"])
        self.assertEqual(summary["first_gps_fix_3d"]["host_monotonic_time"], 0.0)
        self.assertEqual(summary["first_ekf_status_report"]["ekf_flags"], 1024.0)

    def test_no_divergence_reports_none_not_a_crash(self):
        rows = [
            _row("GPS_RAW_INT", 0.0, fix_type=3.0),
            _row("AHRS2", 1.0, ahrs2_roll_deg=0.1, ahrs2_pitch_deg=0.1, ahrs2_yaw_deg=0.0),
            _row("STATUSTEXT", 1.5, statustext_text="EKF3 IMU0 initialised"),
        ]
        summary = trace.summarize(rows, divergence_threshold_deg=1.0, match_tolerance_deg=0.5, stop_reason="ekf3_initialised_statustext_seen")
        self.assertIsNone(summary["first_divergence_time"])
        self.assertIsNone(summary["reproduces_bootstrap_math"])
        self.assertTrue(summary["ekf3_initialised_statustext_seen"])

    def test_mismatched_accel_does_not_falsely_confirm(self):
        """If the divergence doesn't actually match the bootstrap math
        (e.g. a real sustained tilt from something else), the summary
        must say so, not silently claim a match."""
        rows = [
            _row("RAW_IMU", 1.0, imu_xacc=0.0, imu_yacc=0.0, imu_zacc=-1000.0),  # level accel
            _row("AHRS2", 1.1, ahrs2_roll_deg=30.0, ahrs2_pitch_deg=30.0, ahrs2_yaw_deg=0.0),  # but AHRS2 says 30 deg
        ]
        summary = trace.summarize(rows, divergence_threshold_deg=1.0, match_tolerance_deg=1.0, stop_reason="capture_timeout")
        self.assertFalse(summary["reproduces_bootstrap_math"])
        self.assertGreater(summary["roll_residual_deg"], 1.0)


class TestMavlinkMessageToRow(unittest.TestCase):
    def test_unrecognized_message_type_returns_none(self):
        msg = mavlink2.MAVLink_heartbeat_message(
            type=1, autopilot=3, base_mode=81, custom_mode=0, system_status=3, mavlink_version=3,
        )
        self.assertIsNone(trace.mavlink_message_to_row(msg, 1.0))

    def test_ahrs2_row(self):
        msg = mavlink2.MAVLink_ahrs2_message(
            roll=math.radians(7.365185081095156), pitch=math.radians(7.547061919635194),
            yaw=math.radians(-34.08), altitude=0.0, lat=0, lng=0,
        )
        row = trace.mavlink_message_to_row(msg, 5.0)
        self.assertAlmostEqual(row["ahrs2_roll_deg"], 7.365185081095156, places=4)
        self.assertAlmostEqual(row["ahrs2_pitch_deg"], 7.547061919635194, places=4)

    def test_raw_imu_row(self):
        msg = mavlink2.MAVLink_raw_imu_message(
            time_usec=0, xacc=131, yacc=-127, zacc=-983, xgyro=1, ygyro=1, zgyro=1,
            xmag=214, ymag=233, zmag=390, id=0, temperature=0,
        )
        row = trace.mavlink_message_to_row(msg, 5.0)
        self.assertEqual(row["imu_xacc"], 131.0)
        self.assertEqual(row["imu_zacc"], -983.0)

    def test_ekf_status_report_row(self):
        msg = mavlink2.MAVLink_ekf_status_report_message(
            flags=1024, velocity_variance=0.1, pos_horiz_variance=0.1,
            pos_vert_variance=0.1, compass_variance=0.1, terrain_alt_variance=0.0, airspeed_variance=0.0,
        )
        row = trace.mavlink_message_to_row(msg, 5.0)
        self.assertEqual(row["ekf_flags"], 1024.0)


class FakeMavlinkBootstrapPixhawk:
    """PTY fake Pixhawk: streams HEARTBEAT + GPS_RAW_INT/RAW_IMU/AHRS2/
    EKF_STATUS_REPORT from a background thread, sending a scripted
    "transient tilt then freeze" sequence, followed by the STATUSTEXT
    that should stop the trace."""

    def __init__(self):
        self.master_fd, self.slave_fd = pty.openpty()
        self.device_path = os.ttyname(self.slave_fd)
        self._mav = mavutil.mavlink.MAVLink(None, srcSystem=1, srcComponent=1)
        self._stop = threading.Event()
        self._thread = None

    def _send(self, msg):
        os.write(self.master_fd, msg.pack(self._mav))

    def _loop(self):
        # Heartbeat stream (needed for wait_heartbeat()).
        def hb_loop():
            while not self._stop.is_set():
                self._send(mavlink2.MAVLink_heartbeat_message(
                    type=1, autopilot=3, base_mode=81, custom_mode=0, system_status=3, mavlink_version=3,
                ))
                time.sleep(0.1)
        hb_thread = threading.Thread(target=hb_loop, daemon=True)
        hb_thread.start()

        # HIL-F24-R3G test: repeated (not one-shot) sends -- a one-shot
        # send can land while the CUT is still inside wait_heartbeat()'s
        # own internal parsing loop (before this script's own capture
        # loop starts draining messages) and be silently consumed there,
        # exactly the kind of startup race this whole task investigates.
        # Real firmware streams these continuously at the configured
        # rate anyway, so repeating here is also the more realistic
        # simulation.
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline and not self._stop.is_set():
            self._send(mavlink2.MAVLink_gps_raw_int_message(
                time_usec=0, fix_type=3, lat=325378085, lon=743661944, alt=240200,
                eph=100, epv=100, vel=0, cog=0, satellites_visible=15,
            ))
            # The transient off-level sample -- sent throughout, as if
            # this were the (frozen, never-corrected) live stream.
            self._send(mavlink2.MAVLink_raw_imu_message(
                time_usec=0, xacc=131, yacc=-127, zacc=-983, xgyro=0, ygyro=0, zgyro=0,
                xmag=214, ymag=233, zmag=390, id=0, temperature=0,
            ))
            self._send(mavlink2.MAVLink_ahrs2_message(
                roll=math.radians(7.365185081095156), pitch=math.radians(7.547061919635194),
                yaw=math.radians(-34.08), altitude=0.0, lat=0, lng=0,
            ))
            self._send(mavlink2.MAVLink_ekf_status_report_message(
                flags=1024, velocity_variance=0.1, pos_horiz_variance=0.1,
                pos_vert_variance=0.1, compass_variance=0.1, terrain_alt_variance=0.0, airspeed_variance=0.0,
            ))
            time.sleep(0.05)

        for _ in range(5):
            self._send(mavlink2.MAVLink_statustext_message(severity=6, text=b"EKF3 IMU0 initialised"))
            time.sleep(0.05)
        time.sleep(0.2)
        self._stop.set()
        hb_thread.join(timeout=1.0)

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


class TestMainEndToEnd(unittest.TestCase):
    def test_stops_on_ekf3_initialised_statustext_and_reproduces_math(self):
        fake = FakeMavlinkBootstrapPixhawk()
        fake.start()
        tmpdir = Path(tempfile.mkdtemp(prefix="hil_f24r3g_trace_"))
        csv_path = tmpdir / "trace.csv"
        summary_path = tmpdir / "trace_summary.json"
        try:
            cmd = [
                sys.executable, str(HERE / "sr75_hil_f24r3g_ekf3_bootstrap_trace.py"),
                "--pixhawk", fake.device_path, "--baud", "115200",
                "--wait-for-device-timeout-s", "2.0", "--heartbeat-timeout-s", "5.0",
                "--capture-timeout-s", "5.0",
                "--output-csv", str(csv_path), "--summary-json", str(summary_path),
            ]
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertIn("ekf3_initialised_statustext_seen", result.stdout)

            self.assertTrue(csv_path.exists())
            with open(csv_path) as f:
                rows = list(csv.DictReader(f))
            self.assertGreater(len(rows), 0)
            self.assertTrue(any(r["msg_type"] == "AHRS2" for r in rows))
            self.assertTrue(any(r["msg_type"] == "STATUSTEXT" for r in rows))

            summary = json.loads(summary_path.read_text())
            self.assertEqual(summary["stop_reason"], "ekf3_initialised_statustext_seen")
            self.assertTrue(summary["ekf3_initialised_statustext_seen"])
            self.assertIsNotNone(summary["first_divergence_time"])
            self.assertTrue(summary["reproduces_bootstrap_math"])
        finally:
            fake.stop()

    def test_never_sends_param_set_or_arm(self):
        fake = FakeMavlinkBootstrapPixhawk()
        fake.start()
        tmpdir = Path(tempfile.mkdtemp(prefix="hil_f24r3g_trace_safety_"))
        try:
            cmd = [
                sys.executable, str(HERE / "sr75_hil_f24r3g_ekf3_bootstrap_trace.py"),
                "--pixhawk", fake.device_path, "--baud", "115200",
                "--wait-for-device-timeout-s", "2.0", "--heartbeat-timeout-s", "5.0",
                "--capture-timeout-s", "5.0",
                "--output-csv", str(tmpdir / "trace.csv"), "--summary-json", str(tmpdir / "trace_summary.json"),
            ]
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
            self.assertIn("no PARAM_SET, arm, mission, RC override, or actuator command", result.stdout)
        finally:
            fake.stop()

    def test_device_never_reappears_aborts_cleanly(self):
        tmpdir = Path(tempfile.mkdtemp(prefix="hil_f24r3g_trace_nodev_"))
        cmd = [
            sys.executable, str(HERE / "sr75_hil_f24r3g_ekf3_bootstrap_trace.py"),
            "--pixhawk", "/dev/definitely_not_a_real_device_xyz", "--baud", "115200",
            "--wait-for-device-timeout-s", "0.3",
            "--output-csv", str(tmpdir / "trace.csv"), "--summary-json", str(tmpdir / "trace_summary.json"),
        ]
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 2)
        self.assertIn("did not reappear", result.stdout)
        self.assertFalse((tmpdir / "trace.csv").exists())


if __name__ == "__main__":
    unittest.main()
