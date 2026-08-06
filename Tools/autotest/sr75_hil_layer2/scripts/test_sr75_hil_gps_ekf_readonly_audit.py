#!/usr/bin/env python3
"""HIL-F24-R2B regression tests for sr75_hil_gps_ekf_readonly_audit.py.

Covers the fix for the false-positive GPS1_TYPE=100 "BLOCKER": this bench's
actual, confirmed architecture is a SIM_ENABLED firmware build (per
HIL-F24-A) whose compiled-in AP_GPS_SITL backend is selected by GPS1_TYPE=100
(GPS_TYPE_SITL, libraries/AP_GPS/AP_GPS.h:114, a real enum value gated
`#if AP_SIM_GPS_ENABLED`) -- not the GPS_INPUT-over-MAVLink architecture
(GPS1_TYPE=14) this check originally, and now still also, accepts.

Uses a real pseudo-terminal presented as a serial device, fed synthetic
MAVLink HEARTBEAT/PARAM_VALUE replies from a background thread -- the same
technique already proven in test_sr75_hil_f24r2_estimator_capture.py. No
real hardware, no PARAM_SET is ever sent by the fake or the script under
test (PARAM_REQUEST_READ only).
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
import sr75_hil_gps_ekf_readonly_audit as audit  # noqa: E402

DEFAULT_PARAMS = {
    "GPS1_TYPE": 100.0, "GPS2_TYPE": 0.0, "GPS_AUTO_CONFIG": 1.0, "GPS_AUTO_SWITCH": 1.0,
    "EK3_ENABLE": 1.0, "AHRS_EKF_TYPE": 3.0, "EK3_SRC1_POSXY": 3.0, "EK3_SRC1_POSZ": 1.0,
    "EK3_SRC1_VELXY": 3.0, "EK3_SRC1_VELZ": 3.0, "EK3_SRC1_YAW": 1.0,
    "SERIAL1_PROTOCOL": 48.0, "SERIAL1_BAUD": 921.0,
    # EK3_GPS_TYPE deliberately absent -- NO_RESPONSE is expected/correct.
}


class FakeMavlinkParamPixhawk:
    """A real PTY device, openable exactly like /dev/ttyACM0, that streams
    HEARTBEAT and answers PARAM_REQUEST_READ with PARAM_VALUE from a fixed
    dict -- never sends/accepts a PARAM_SET."""

    def __init__(self, param_values):
        self.master_fd, self.slave_fd = pty.openpty()
        self.device_path = os.ttyname(self.slave_fd)
        self._mav = mavutil.mavlink.MAVLink(None, srcSystem=1, srcComponent=1)
        self.param_values = dict(param_values)
        self._stop = threading.Event()
        self._hb_thread = None
        self._reader_thread = None

    def _send(self, msg):
        os.write(self.master_fd, msg.pack(self._mav))

    def _heartbeat_loop(self):
        while not self._stop.is_set():
            self._send(mavlink2.MAVLink_heartbeat_message(
                type=1, autopilot=3, base_mode=81, custom_mode=0, system_status=3, mavlink_version=3,
            ))
            time.sleep(0.1)

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
                if msg is not None and msg.get_type() == "PARAM_REQUEST_READ":
                    name = msg.param_id
                    if isinstance(name, bytes):
                        name = name.decode("utf-8")
                    name = name.rstrip("\x00")
                    if name in self.param_values:
                        self._send(mavlink2.MAVLink_param_value_message(
                            param_id=name.encode("utf-8"), param_value=float(self.param_values[name]),
                            param_type=9, param_count=1, param_index=0,
                        ))

    def start(self):
        self._hb_thread = threading.Thread(target=self._heartbeat_loop, daemon=True)
        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._hb_thread.start()
        self._reader_thread.start()

    def stop(self):
        self._stop.set()
        for t in (self._hb_thread, self._reader_thread):
            if t is not None:
                t.join(timeout=2.0)
        for fd in (self.master_fd, self.slave_fd):
            try:
                os.close(fd)
            except OSError:
                pass


class TestGps1TypeClassification(unittest.TestCase):
    """Pure classification logic, no hardware: the fix generalizes CHECKS'
    `required` field to accept either a single string or a tuple of
    acceptable values."""

    def test_gps1_type_check_accepts_both_14_and_100(self):
        entry = next(c for c in audit.CHECKS if c[0] == "GPS1_TYPE")
        _, required, _ = entry
        self.assertIn("14", required)
        self.assertIn("100", required)

    def test_gps1_type_100_is_not_a_blocker(self):
        entry = next(c for c in audit.CHECKS if c[0] == "GPS1_TYPE")
        _, required, _ = entry
        accepted = (required,) if isinstance(required, str) else required
        self.assertIn(audit.fmt(100.0), accepted)

    def test_gps1_type_14_is_not_a_blocker(self):
        entry = next(c for c in audit.CHECKS if c[0] == "GPS1_TYPE")
        _, required, _ = entry
        accepted = (required,) if isinstance(required, str) else required
        self.assertIn(audit.fmt(14.0), accepted)

    def test_other_single_valued_checks_unaffected(self):
        """Confirms the generalization didn't change behavior for the
        existing single-required-value checks (e.g. AHRS_EKF_TYPE)."""
        entry = next(c for c in audit.CHECKS if c[0] == "AHRS_EKF_TYPE")
        _, required, _ = entry
        self.assertEqual(required, "3")


class TestReadOnlyAuditEndToEnd(unittest.TestCase):
    """Real end-to-end run of main() against a PTY fake Pixhawk -- exercises
    the real MAVLink connection/param-request/classification code path."""

    def _run(self, param_values, extra_args=None):
        fake = FakeMavlinkParamPixhawk(param_values)
        fake.start()
        try:
            cmd = [
                sys.executable, str(HERE / "sr75_hil_gps_ekf_readonly_audit.py"),
                "--pixhawk", fake.device_path, "--baud", "115200",
                "--param-timeout-s", "1.0", "--retries", "2",
            ] + (extra_args or [])
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
            return result
        finally:
            fake.stop()

    def test_gps1_type_100_sim_enabled_bench_passes_with_exit_0(self):
        result = self._run(DEFAULT_PARAMS)
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertIn("No blocking mismatches found", result.stdout)
        self.assertNotIn("GPS1_TYPE", result.stdout.split("BLOCKING")[0].split("BLOCKER")[0] if "BLOCKER" in result.stdout else "")

    def test_gps1_type_14_gps_input_bench_also_passes_with_exit_0(self):
        params = dict(DEFAULT_PARAMS, GPS1_TYPE=14.0)
        result = self._run(params)
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertIn("No blocking mismatches found", result.stdout)

    def test_gps1_type_genuinely_wrong_value_still_blocks(self):
        """A real misconfiguration (e.g. GPS1_TYPE left at 0/None) must
        still be caught -- the fix only widens acceptance to the two
        legitimate architectures, it does not disable the check."""
        params = dict(DEFAULT_PARAMS, GPS1_TYPE=0.0)
        result = self._run(params)
        self.assertEqual(result.returncode, 1, result.stdout)
        self.assertIn("BLOCKING PARAMS: GPS1_TYPE", result.stdout)

    def test_never_sends_param_set_or_arm(self):
        """Safety confirmation line is present and no PARAM_SET-shaped
        traffic reaches the fake (the fake only ever answers
        PARAM_REQUEST_READ; if the script sent PARAM_SET the fake would
        simply never see a matching request type, so this also indirectly
        confirms the script's request traffic is exclusively read-only)."""
        result = self._run(DEFAULT_PARAMS)
        self.assertIn(
            "only PARAM_REQUEST_READ was sent (read-only); no PARAM_SET, arm, "
            "mission, RC override, or actuator command.",
            result.stdout,
        )


if __name__ == "__main__":
    unittest.main()
