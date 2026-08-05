#!/usr/bin/env python3
"""HIL-F23-F2B2A unit tests for sr75_hil_f2b2a_gps_altitude_ramp.py.

No hardware, no MAVLink connection, no JSBSim -- pure function tests over
the altitude profile, the synthetic CSV row/writer, bridge command
building, phase classification, and phase-summary post-processing. One
test exercises the real (unmodified) bridge JSBSimStateFeeder against a
short synthetic profile file to prove wire-format compatibility.
"""
import csv
import io
import sys
import tempfile
import time
import unittest
from pathlib import Path

import sr75_hil_f2b2a_gps_altitude_ramp as ramp

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "bridge"))
import sr75_jsbsim_pixhawk_hil_bridge as bridge  # noqa: E402


class TestAltitudeProfile(unittest.TestCase):
    def test_hold1_phase(self):
        for t in (0.0, 5.0, 9.99):
            alt, vd, phase = ramp.compute_altitude_profile(t)
            self.assertEqual(alt, 3000.0)
            self.assertEqual(vd, 0.0)
            self.assertEqual(phase, "hold1")

    def test_climb_phase_rate_and_vd_sign(self):
        alt0, vd0, phase0 = ramp.compute_altitude_profile(10.0)
        self.assertEqual(alt0, 3000.0)
        self.assertEqual(vd0, -1.0, "climb must use vd=-1 m/s (NED down-positive convention)")
        self.assertEqual(phase0, "climb")
        alt10, _, _ = ramp.compute_altitude_profile(20.0)
        self.assertEqual(alt10, 3010.0)
        alt_end, _, _ = ramp.compute_altitude_profile(29.99)
        self.assertAlmostEqual(alt_end, 3019.99, places=2)

    def test_hold2_phase(self):
        for t in (30.0, 35.0, 39.99):
            alt, vd, phase = ramp.compute_altitude_profile(t)
            self.assertEqual(alt, 3020.0)
            self.assertEqual(vd, 0.0)
            self.assertEqual(phase, "hold2")

    def test_descend_phase_rate_and_vd_sign(self):
        alt0, vd0, phase0 = ramp.compute_altitude_profile(40.0)
        self.assertEqual(alt0, 3020.0)
        self.assertEqual(vd0, 1.0, "descend must use vd=+1 m/s (NED down-positive convention)")
        self.assertEqual(phase0, "descend")
        alt_end, _, _ = ramp.compute_altitude_profile(59.99)
        self.assertAlmostEqual(alt_end, 3000.01, places=2)

    def test_boundary_and_clamping(self):
        alt, vd, phase = ramp.compute_altitude_profile(60.0)
        self.assertEqual((alt, vd, phase), (3000.0, 0.0, "done"))
        alt, vd, phase = ramp.compute_altitude_profile(1000.0)
        self.assertEqual((alt, vd, phase), (3000.0, 0.0, "done"))
        alt, vd, phase = ramp.compute_altitude_profile(-5.0)
        self.assertEqual((alt, vd, phase), (3000.0, 0.0, "hold1"))

    def test_total_duration_is_60s(self):
        self.assertEqual(ramp.PROFILE_DURATION_S, 60.0)

    def test_profile_returns_to_base_altitude(self):
        alt_start, _, _ = ramp.compute_altitude_profile(0.0)
        alt_end, _, _ = ramp.compute_altitude_profile(60.0)
        self.assertEqual(alt_start, alt_end)


class TestBuildProfileRow(unittest.TestCase):
    def test_row_matches_header_length(self):
        row, phase = ramp.build_profile_row(15.0)
        self.assertEqual(len(row), len(ramp.PROFILE_HEADER))
        self.assertEqual(phase, "climb")

    def test_fixed_lat_lon_yaw_used(self):
        row, _phase = ramp.build_profile_row(0.0)
        idx = {name: i for i, name in enumerate(ramp.PROFILE_HEADER)}
        self.assertEqual(float(row[idx["jsb_lat_deg"]]), ramp.FIXED_LAT)
        self.assertEqual(float(row[idx["jsb_lon_deg"]]), ramp.FIXED_LON)
        import math
        self.assertAlmostEqual(float(row[idx["jsb_yaw_rad"]]), math.radians(ramp.FIXED_YAW_DEG), places=5)

    def test_vn_ve_always_zero(self):
        idx = {name: i for i, name in enumerate(ramp.PROFILE_HEADER)}
        for t in (0.0, 15.0, 35.0, 50.0):
            row, _phase = ramp.build_profile_row(t)
            self.assertEqual(float(row[idx["jsb_vn_mps"]]), 0.0)
            self.assertEqual(float(row[idx["jsb_ve_mps"]]), 0.0)

    def test_vd_matches_profile(self):
        idx = {name: i for i, name in enumerate(ramp.PROFILE_HEADER)}
        row, _ = ramp.build_profile_row(15.0)
        self.assertEqual(float(row[idx["jsb_vd_mps"]]), -1.0)
        row, _ = ramp.build_profile_row(50.0)
        self.assertEqual(float(row[idx["jsb_vd_mps"]]), 1.0)


class TestWriteProfileCsvRealtime(unittest.TestCase):
    def test_writes_header_and_rows(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "profile.csv"
            rows_written = ramp.write_profile_csv_realtime(path, duration_s=0.3, rate_hz=20.0)
            self.assertGreater(rows_written, 0)
            with open(path, newline="") as f:
                reader = list(csv.reader(f))
            self.assertEqual(reader[0], ramp.PROFILE_HEADER)
            self.assertEqual(len(reader) - 1, rows_written)

    def test_alive_check_stops_writer_immediately(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "profile.csv"
            start = time.time()
            rows_written = ramp.write_profile_csv_realtime(
                path, duration_s=60.0, rate_hz=20.0, alive_check=lambda: "bridge exited unexpectedly rc=1",
            )
            elapsed = time.time() - start
            self.assertEqual(rows_written, 0)
            self.assertLess(elapsed, 1.0)

    def test_output_is_readable_by_real_bridge_feeder(self):
        # Proves the synthetic CSV is wire-compatible with the existing,
        # unmodified JSBSimStateFeeder/_read_raw_latest() -- no bridge
        # parsing changes were needed for this test mode.
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "profile.csv"
            ramp.write_profile_csv_realtime(path, duration_s=0.2, rate_hz=20.0)
            feeder = bridge.JSBSimStateFeeder(str(path))
            raw = feeder._read_raw_latest()
            self.assertIsNotNone(raw)
            self.assertAlmostEqual(float(raw["jsb_feed_lat_deg"]), ramp.FIXED_LAT, places=6)
            self.assertAlmostEqual(float(raw["jsb_feed_lon_deg"]), ramp.FIXED_LON, places=6)
            self.assertAlmostEqual(float(raw["jsb_feed_alt_m"]), 3000.0, places=1)


class TestBuildBridgeCmdForRampTest(unittest.TestCase):
    def _cmd(self, mp_out="udp:192.168.64.1:14550"):
        return ramp.build_bridge_cmd_for_ramp_test(
            "/dev/ttyACM0", 115200, "/tmp/profile.csv", mp_out, "/tmp/log.csv",
        )

    def test_never_includes_gps_input_send_yaw(self):
        self.assertNotIn("--gps-input-send-yaw", self._cmd())

    def test_never_includes_attitude_or_airspeed_inject(self):
        cmd = self._cmd()
        self.assertNotIn("--attitude-inject", cmd)
        self.assertNotIn("--airspeed-inject", cmd)

    def test_never_includes_allow_actuator_output(self):
        self.assertNotIn("--allow-actuator-output", self._cmd())

    def test_includes_gps_input_inject(self):
        self.assertIn("--gps-input-inject", self._cmd())

    def test_mp_out_propagates(self):
        cmd = self._cmd(mp_out="udp:10.0.0.5:14550")
        self.assertIn("--mp-out", cmd)
        self.assertEqual(cmd[cmd.index("--mp-out") + 1], "udp:10.0.0.5:14550")

    def test_mp_out_omitted_when_empty(self):
        self.assertNotIn("--mp-out", self._cmd(mp_out=""))


class TestClassifyPhaseFromCommanded(unittest.TestCase):
    def test_climb(self):
        self.assertEqual(ramp.classify_phase_from_commanded(3005.0, -1.0), "climb")

    def test_descend(self):
        self.assertEqual(ramp.classify_phase_from_commanded(3015.0, 1.0), "descend")

    def test_hold_low(self):
        self.assertEqual(ramp.classify_phase_from_commanded(3000.0, 0.0), "hold1")

    def test_hold_high(self):
        self.assertEqual(ramp.classify_phase_from_commanded(3020.0, 0.0), "hold2")

    def test_missing_values_return_none(self):
        self.assertIsNone(ramp.classify_phase_from_commanded(None, 0.0))
        self.assertIsNone(ramp.classify_phase_from_commanded(3000.0, None))


class TestComputePhaseSummary(unittest.TestCase):
    FIELDNAMES = [
        "time_s", "gps_tx_alt_m", "gps_tx_vd_mps", "alt_m",
        "gps_raw_alt_m", "vfr_alt_m", "local_ned_z_m", "ekf_flags",
    ]

    def _write_log(self, tmpdir, rows):
        path = Path(tmpdir) / "bridge_log.csv"
        with open(path, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=self.FIELDNAMES)
            w.writeheader()
            w.writerows(rows)
        return path

    def test_steady_state_error_computed_for_hold_phases(self):
        rows = [
            {"time_s": "0.0", "gps_tx_alt_m": "3000.0", "gps_tx_vd_mps": "0.0",
             "alt_m": "3001.5", "gps_raw_alt_m": "3000.5", "vfr_alt_m": "3001.5",
             "local_ned_z_m": "-3000.0", "ekf_flags": "831"},
            {"time_s": "1.0", "gps_tx_alt_m": "3000.0", "gps_tx_vd_mps": "0.0",
             "alt_m": "3002.5", "gps_raw_alt_m": "3000.4", "vfr_alt_m": "3002.5",
             "local_ned_z_m": "-3001.0", "ekf_flags": "831"},
        ]
        with tempfile.TemporaryDirectory() as tmpdir:
            path = self._write_log(tmpdir, rows)
            summary = ramp.compute_phase_summary(path)
            hold1 = summary["per_phase"]["hold1"]
            self.assertEqual(hold1["n_samples"], 2)
            self.assertAlmostEqual(hold1["steady_state_error_m"], 2.0, places=3)

    def test_ramp_tracking_delay_detected(self):
        rows = [
            {"time_s": "10.0", "gps_tx_alt_m": "3000.0", "gps_tx_vd_mps": "-1.0",
             "alt_m": "3010.0", "gps_raw_alt_m": "3000.0", "vfr_alt_m": "3010.0",
             "local_ned_z_m": "-3000.0", "ekf_flags": "831"},  # 10m lag, outside tolerance
            {"time_s": "11.0", "gps_tx_alt_m": "3001.0", "gps_tx_vd_mps": "-1.0",
             "alt_m": "3002.5", "gps_raw_alt_m": "3001.0", "vfr_alt_m": "3002.5",
             "local_ned_z_m": "-3001.0", "ekf_flags": "831"},  # 1.5m lag, within tolerance
        ]
        with tempfile.TemporaryDirectory() as tmpdir:
            path = self._write_log(tmpdir, rows)
            summary = ramp.compute_phase_summary(path, tolerance_m=3.0)
            climb = summary["per_phase"]["climb"]
            self.assertAlmostEqual(climb["ramp_tracking_delay_s"], 1.0, places=3)

    def test_ekf_flags_change_detected_as_discontinuity(self):
        rows = [
            {"time_s": "0.0", "gps_tx_alt_m": "3000.0", "gps_tx_vd_mps": "0.0",
             "alt_m": "3000.0", "gps_raw_alt_m": "3000.0", "vfr_alt_m": "3000.0",
             "local_ned_z_m": "-3000.0", "ekf_flags": "831"},
            {"time_s": "1.0", "gps_tx_alt_m": "3000.0", "gps_tx_vd_mps": "0.0",
             "alt_m": "3000.0", "gps_raw_alt_m": "3000.0", "vfr_alt_m": "3000.0",
             "local_ned_z_m": "-3000.0", "ekf_flags": "167"},  # matches the observed hardware flag toggle
        ]
        with tempfile.TemporaryDirectory() as tmpdir:
            path = self._write_log(tmpdir, rows)
            summary = ramp.compute_phase_summary(path)
            types_found = {d["type"] for d in summary["discontinuities"]}
            self.assertIn("ekf_flags_change", types_found)

    def test_local_ned_z_jump_detected_as_discontinuity(self):
        rows = [
            {"time_s": "0.0", "gps_tx_alt_m": "3000.0", "gps_tx_vd_mps": "0.0",
             "alt_m": "3000.0", "gps_raw_alt_m": "3000.0", "vfr_alt_m": "3000.0",
             "local_ned_z_m": "-3000.0", "ekf_flags": "831"},
            {"time_s": "1.0", "gps_tx_alt_m": "3000.0", "gps_tx_vd_mps": "0.0",
             "alt_m": "3000.0", "gps_raw_alt_m": "3000.0", "vfr_alt_m": "3000.0",
             "local_ned_z_m": "-3420.0", "ekf_flags": "831"},  # >400m jump, matches observed divergence
        ]
        with tempfile.TemporaryDirectory() as tmpdir:
            path = self._write_log(tmpdir, rows)
            summary = ramp.compute_phase_summary(path, reset_ned_z_jump_m=5.0)
            types_found = {d["type"] for d in summary["discontinuities"]}
            self.assertIn("local_ned_z_jump", types_found)

    def test_no_discontinuities_when_stable(self):
        rows = [
            {"time_s": str(float(i)), "gps_tx_alt_m": "3000.0", "gps_tx_vd_mps": "0.0",
             "alt_m": "3001.0", "gps_raw_alt_m": "3000.0", "vfr_alt_m": "3001.0",
             "local_ned_z_m": f"{-3000.0 - 0.01 * i:.2f}", "ekf_flags": "831"}
            for i in range(5)
        ]
        with tempfile.TemporaryDirectory() as tmpdir:
            path = self._write_log(tmpdir, rows)
            summary = ramp.compute_phase_summary(path)
            self.assertEqual(summary["discontinuities"], [])

    def test_overall_max_error(self):
        rows = [
            {"time_s": "0.0", "gps_tx_alt_m": "3000.0", "gps_tx_vd_mps": "0.0",
             "alt_m": "3000.5", "gps_raw_alt_m": "3000.0", "vfr_alt_m": "3000.5",
             "local_ned_z_m": "-3000.0", "ekf_flags": "831"},
            {"time_s": "1.0", "gps_tx_alt_m": "3020.0", "gps_tx_vd_mps": "0.0",
             "alt_m": "3025.0", "gps_raw_alt_m": "3020.0", "vfr_alt_m": "3025.0",
             "local_ned_z_m": "-3020.0", "ekf_flags": "831"},
        ]
        with tempfile.TemporaryDirectory() as tmpdir:
            path = self._write_log(tmpdir, rows)
            summary = ramp.compute_phase_summary(path)
            self.assertAlmostEqual(summary["overall_max_error_m"], 5.0, places=3)


class TestCliDryRun(unittest.TestCase):
    def test_dry_run_prints_plan_and_never_touches_hardware(self):
        argv = ["--dry-run", "--run-dir", tempfile.mkdtemp()]
        old_argv = sys.argv
        sys.argv = ["sr75_hil_f2b2a_gps_altitude_ramp.py"] + argv
        buf = io.StringIO()
        try:
            import contextlib
            with contextlib.redirect_stdout(buf):
                rc = ramp.main()
        finally:
            sys.argv = old_argv
        self.assertEqual(rc, 0)
        output = buf.getvalue()
        self.assertIn("GPS_INPUT yaw transmission: DISABLED", output)
        self.assertIn("Attitude/airspeed display injection: DISABLED", output)
        self.assertIn("hold 3000m for 10s", output)
        self.assertIn("DRY RUN", output)

    def test_gps_input_send_yaw_never_appears_in_dry_run_output(self):
        argv = ["--dry-run", "--run-dir", tempfile.mkdtemp()]
        old_argv = sys.argv
        sys.argv = ["sr75_hil_f2b2a_gps_altitude_ramp.py"] + argv
        buf = io.StringIO()
        try:
            import contextlib
            with contextlib.redirect_stdout(buf):
                ramp.main()
        finally:
            sys.argv = old_argv
        self.assertNotIn("--gps-input-send-yaw", buf.getvalue())


if __name__ == "__main__":
    unittest.main()
