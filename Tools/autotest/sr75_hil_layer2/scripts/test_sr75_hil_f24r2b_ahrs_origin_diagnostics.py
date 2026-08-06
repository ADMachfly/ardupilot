#!/usr/bin/env python3
"""HIL-F24-R2B regression tests for sr75_hil_f24r2b_ahrs_origin_diagnostics.py.

Covers the pure STATUSTEXT-health-scan helper and a full end-to-end run
against a PTY fake Pixhawk (the same technique proven in
test_sr75_hil_f24r2_estimator_capture.py and
test_sr75_hil_gps_ekf_readonly_audit.py) that answers PARAM_REQUEST_READ,
MAV_CMD_GET_HOME_POSITION, and MAV_CMD_REQUEST_MESSAGE(GPS_GLOBAL_ORIGIN)
-- confirming the real MAVLink request/parse/compare code path locates a
poisoned EKF origin (the real-world regression this task diagnosed: a
GPS_GLOBAL_ORIGIN stuck at ArduPilot's compiled-in default SITL home,
~11,231 km from the bench's actual injected truth position, while
HOME_POSITION correctly tracks truth). No real hardware, no PARAM_SET/
arm/mission/RC-override/actuator command is ever sent by the fake or the
script under test.
"""
import os
import pty
import subprocess
import sys
import threading
import time
import unittest
from pathlib import Path

from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink2

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import sr75_hil_f24r2b_ahrs_origin_diagnostics as diag  # noqa: E402

# The real onboard values captured live from the bench during this task.
DEFAULT_PARAMS = {
    "GPS1_TYPE": 100.0, "GPS2_TYPE": 0.0, "GPS_AUTO_CONFIG": 1.0, "GPS_AUTO_SWITCH": 1.0,
    "EK3_ENABLE": 1.0, "AHRS_EKF_TYPE": 3.0, "EK3_SRC1_POSXY": 3.0, "EK3_SRC1_POSZ": 1.0,
    "EK3_SRC1_VELXY": 3.0, "EK3_SRC1_VELZ": 3.0, "EK3_SRC1_YAW": 1.0,
    "SERIAL1_PROTOCOL": 48.0, "SERIAL1_BAUD": 921.0,
}
# Real captured values: HOME_POSITION correctly tracks truth; GPS_GLOBAL_
# ORIGIN is stuck at ArduPilot's compiled-in default SITL home (Canberra CMAC).
REAL_HOME_DEG = (32.5378085, 74.3661943, 0.16)
REAL_POISONED_ORIGIN_DEG = (-35.3632621, 149.1652374, 584.0)
REAL_TRUTH_DEG = (32.5378085, 74.3661944, 240.201118)


class TestStatustextHealthScan(unittest.TestCase):
    def test_dcm_active_matched(self):
        hits = diag.scan_statustext_for_ahrs_health(["AHRS: DCM active"])
        self.assertEqual(len(hits), 1)
        self.assertEqual(hits[0][0], "ahrs: dcm active")

    def test_not_using_configured_ahrs_type_matched(self):
        hits = diag.scan_statustext_for_ahrs_health(["PreArm: AHRS: not using configured AHRS type"])
        self.assertEqual(len(hits), 1)

    def test_main_loop_slow_matched(self):
        hits = diag.scan_statustext_for_ahrs_health(["PreArm: Main loop slow (36Hz < 50Hz)"])
        self.assertEqual(len(hits), 1)

    def test_gps_and_ahrs_differ_matched(self):
        hits = diag.scan_statustext_for_ahrs_health(["PreArm: GPS and AHRS differ by 2525060.m"])
        self.assertEqual(len(hits), 1)

    def test_benign_text_not_matched(self):
        hits = diag.scan_statustext_for_ahrs_health(["ArduPlane V4.5.0", "PPP[0]: reconnecting"])
        self.assertEqual(hits, [])

    def test_case_insensitive(self):
        hits = diag.scan_statustext_for_ahrs_health(["ahrs: dcm ACTIVE"])
        self.assertEqual(len(hits), 1)


class FakeMavlinkOriginPixhawk:
    """PTY fake Pixhawk answering PARAM_REQUEST_READ (from a fixed dict),
    MAV_CMD_GET_HOME_POSITION (with a fixed HOME_POSITION reply), and
    MAV_CMD_REQUEST_MESSAGE(GPS_GLOBAL_ORIGIN) (with a fixed GPS_GLOBAL_
    ORIGIN reply). Optionally emits one STATUSTEXT after a short delay."""

    def __init__(self, param_values, home_deg, origin_deg, statustext=None):
        self.master_fd, self.slave_fd = pty.openpty()
        self.device_path = os.ttyname(self.slave_fd)
        self._mav = mavutil.mavlink.MAVLink(None, srcSystem=1, srcComponent=1)
        self.param_values = dict(param_values)
        self.home_deg = home_deg
        self.origin_deg = origin_deg
        self.statustext = statustext
        self._stop = threading.Event()
        self._hb_thread = None
        self._reader_thread = None
        self._statustext_thread = None

    def _send(self, msg):
        os.write(self.master_fd, msg.pack(self._mav))

    def _heartbeat_loop(self):
        while not self._stop.is_set():
            self._send(mavlink2.MAVLink_heartbeat_message(
                type=1, autopilot=3, base_mode=81, custom_mode=0, system_status=3, mavlink_version=3,
            ))
            time.sleep(0.1)

    def _statustext_loop(self):
        if self.statustext is None:
            return
        # Repeats (not one-shot) so it is still being sent whenever the
        # script under test's listen window actually opens, regardless of
        # how long the preceding param-read retries take.
        while not self._stop.is_set():
            self._send(mavlink2.MAVLink_statustext_message(severity=6, text=self.statustext.encode("utf-8")))
            time.sleep(0.3)

    def _reader_loop(self):
        parser = mavutil.mavlink.MAVLink(None, srcSystem=1, srcComponent=1)
        while not self._stop.is_set():
            try:
                chunk = os.read(self.master_fd, 4096)
            except OSError:
                break
            for i in range(len(chunk)):
                try:
                    msg = parser.parse_char(chunk[i:i + 1])
                except mavutil.mavlink.MAVError:
                    continue
                if msg is None:
                    continue
                mtype = msg.get_type()
                if mtype == "PARAM_REQUEST_READ":
                    name = msg.param_id
                    if isinstance(name, bytes):
                        name = name.decode("utf-8")
                    name = name.rstrip("\x00")
                    if name in self.param_values:
                        self._send(mavlink2.MAVLink_param_value_message(
                            param_id=name.encode("utf-8"), param_value=float(self.param_values[name]),
                            param_type=9, param_count=1, param_index=0,
                        ))
                elif mtype == "COMMAND_LONG" and msg.command == mavutil.mavlink.MAV_CMD_GET_HOME_POSITION:
                    lat, lon, alt = self.home_deg
                    self._send(mavlink2.MAVLink_home_position_message(
                        latitude=int(lat * 1e7), longitude=int(lon * 1e7), altitude=int(alt * 1e3),
                        x=0.0, y=0.0, z=0.0, q=[1.0, 0.0, 0.0, 0.0], approach_x=0.0, approach_y=0.0, approach_z=0.0,
                    ))
                elif (mtype == "COMMAND_LONG" and msg.command == mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE
                      and int(msg.param1) == mavutil.mavlink.MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN):
                    lat, lon, alt = self.origin_deg
                    self._send(mavlink2.MAVLink_gps_global_origin_message(
                        latitude=int(lat * 1e7), longitude=int(lon * 1e7), altitude=int(alt * 1e3),
                    ))

    def start(self):
        self._hb_thread = threading.Thread(target=self._heartbeat_loop, daemon=True)
        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._statustext_thread = threading.Thread(target=self._statustext_loop, daemon=True)
        self._hb_thread.start()
        self._reader_thread.start()
        self._statustext_thread.start()

    def stop(self):
        self._stop.set()
        for t in (self._hb_thread, self._reader_thread, self._statustext_thread):
            if t is not None:
                t.join(timeout=2.0)
        for fd in (self.master_fd, self.slave_fd):
            try:
                os.close(fd)
            except OSError:
                pass


class TestOriginDiagnosticEndToEnd(unittest.TestCase):
    def _run(self, home_deg, origin_deg, statustext=None, truth_deg=REAL_TRUTH_DEG, summary_json=None):
        fake = FakeMavlinkOriginPixhawk(DEFAULT_PARAMS, home_deg, origin_deg, statustext=statustext)
        fake.start()
        try:
            cmd = [
                sys.executable, str(HERE / "sr75_hil_f24r2b_ahrs_origin_diagnostics.py"),
                "--pixhawk", fake.device_path, "--baud", "115200", "--listen-s", "1.0",
                "--param-timeout-s", "0.5", "--param-retries", "1",
                "--truth-lat", str(truth_deg[0]), "--truth-lon", str(truth_deg[1]), "--truth-alt-m", str(truth_deg[2]),
            ]
            if summary_json is not None:
                cmd += ["--summary-json", str(summary_json)]
            return subprocess.run(cmd, capture_output=True, text=True, timeout=30)
        finally:
            fake.stop()

    def test_healthy_bench_home_and_origin_both_match_truth(self):
        result = self._run(home_deg=REAL_TRUTH_DEG, origin_deg=REAL_TRUTH_DEG)
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertIn("OK: no GPS/EKF param blockers", result.stdout)

    def test_real_bench_poisoned_origin_detected(self):
        """Regression fixture: the exact real values captured live from the
        bench during this diagnosis -- HOME_POSITION correctly tracks
        truth (~0.01m), GPS_GLOBAL_ORIGIN is stuck ~11,231 km away at
        ArduPilot's compiled-in default SITL home. Must be flagged."""
        result = self._run(home_deg=REAL_HOME_DEG, origin_deg=REAL_POISONED_ORIGIN_DEG)
        self.assertEqual(result.returncode, 1, result.stdout)
        self.assertIn("WARN: HOME/origin vs. truth horizontal mismatch", result.stdout)
        # Matches the real captured magnitude (~11,231 km) within 1%.
        self.assertIn("GPS_GLOBAL_ORIGIN vs. truth: horizontal=112", result.stdout)

    def test_ahrs_health_statustext_flagged(self):
        result = self._run(home_deg=REAL_TRUTH_DEG, origin_deg=REAL_TRUTH_DEG, statustext="AHRS: DCM active")
        self.assertEqual(result.returncode, 1, result.stdout)
        self.assertIn("MATCH (ahrs: dcm active)", result.stdout)

    def test_never_sends_param_set_or_arm(self):
        result = self._run(home_deg=REAL_TRUTH_DEG, origin_deg=REAL_TRUTH_DEG)
        self.assertIn(
            "only PARAM_REQUEST_READ, MAV_CMD_GET_HOME_POSITION, and "
            "MAV_CMD_REQUEST_MESSAGE(GPS_GLOBAL_ORIGIN) were sent (all read-only); "
            "no PARAM_SET, arm, mission, RC override, or actuator command.",
            result.stdout,
        )


class TestSummaryJsonOutput(unittest.TestCase):
    """HIL-F24-R2C reuses this diagnostic's --summary-json output as its
    machine-readable relatch gate -- these tests pin down the exact field
    names/shapes the orchestrator's evaluate_origin_relatch_summary()
    depends on."""

    def _run_with_summary(self, home_deg, origin_deg, tmp_path, truth_deg=REAL_TRUTH_DEG):
        summary_path = tmp_path / "origin_relatch_summary.json"
        fake = FakeMavlinkOriginPixhawk(DEFAULT_PARAMS, home_deg, origin_deg)
        fake.start()
        try:
            cmd = [
                sys.executable, str(HERE / "sr75_hil_f24r2b_ahrs_origin_diagnostics.py"),
                "--pixhawk", fake.device_path, "--baud", "115200", "--listen-s", "1.0",
                "--param-timeout-s", "0.5", "--param-retries", "1",
                "--truth-lat", str(truth_deg[0]), "--truth-lon", str(truth_deg[1]), "--truth-alt-m", str(truth_deg[2]),
                "--summary-json", str(summary_path),
            ]
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
            return result, summary_path
        finally:
            fake.stop()

    def test_summary_json_written_for_healthy_case(self):
        import json
        import tempfile
        with tempfile.TemporaryDirectory() as tmpdir:
            result, summary_path = self._run_with_summary(REAL_TRUTH_DEG, REAL_TRUTH_DEG, Path(tmpdir))
            self.assertEqual(result.returncode, 0, result.stdout)
            self.assertTrue(summary_path.exists())
            summary = json.loads(summary_path.read_text())
            self.assertTrue(summary["ok"])
            self.assertAlmostEqual(summary["gps_global_origin_mismatch_m"], 0.0, delta=1.0)
            self.assertAlmostEqual(summary["home_mismatch_m"], 0.0, delta=1.0)
            self.assertEqual(summary["param_blockers"], [])
            self.assertEqual(summary["ahrs_health_events"], [])

    def test_summary_json_matches_real_poisoned_origin_fixture(self):
        """Regression fixture: the exact real captured values -- confirms
        the JSON output (not just stdout) carries the ~11,231 km
        GPS_GLOBAL_ORIGIN mismatch HIL-F24-R2C's orchestrator gate reads."""
        import json
        import tempfile
        with tempfile.TemporaryDirectory() as tmpdir:
            result, summary_path = self._run_with_summary(REAL_HOME_DEG, REAL_POISONED_ORIGIN_DEG, Path(tmpdir))
            self.assertEqual(result.returncode, 1, result.stdout)
            summary = json.loads(summary_path.read_text())
            self.assertFalse(summary["ok"])
            self.assertGreater(summary["gps_global_origin_mismatch_m"], 11_000_000.0)
            self.assertLess(summary["home_mismatch_m"], 50.0)

    def test_no_summary_json_written_when_flag_omitted(self):
        """Backward compatibility: omitting --summary-json must not write
        any file (existing R2B behavior/tests unaffected)."""
        import tempfile
        with tempfile.TemporaryDirectory() as tmpdir:
            fake = FakeMavlinkOriginPixhawk(DEFAULT_PARAMS, REAL_TRUTH_DEG, REAL_TRUTH_DEG)
            fake.start()
            try:
                cmd = [
                    sys.executable, str(HERE / "sr75_hil_f24r2b_ahrs_origin_diagnostics.py"),
                    "--pixhawk", fake.device_path, "--baud", "115200", "--listen-s", "1.0",
                    "--param-timeout-s", "0.5", "--param-retries", "1",
                ]
                subprocess.run(cmd, capture_output=True, text=True, timeout=30)
            finally:
                fake.stop()
            self.assertEqual(list(Path(tmpdir).iterdir()), [])


if __name__ == "__main__":
    unittest.main()
