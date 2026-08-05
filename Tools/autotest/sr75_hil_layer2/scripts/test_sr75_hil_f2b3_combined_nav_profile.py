#!/usr/bin/env python3
"""HIL-F23-F2B3 unit tests for sr75_hil_f2b3_combined_nav_profile.py.

No hardware, no MAVLink connection, no JSBSim -- pure function tests over
the combined nav profile (heading/groundspeed/vd per phase, lat/lon
integration, yaw-rate limiting), the synthetic CSV writer, bridge command
building, preflight evaluation logic, and phase-summary/PASS-criteria
post-processing. One test exercises the real (unmodified) bridge
JSBSimStateFeeder against a short synthetic profile file to prove
wire-format compatibility.
"""
import csv
import io
import math
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path

import sr75_hil_f2b3_combined_nav_profile as nav

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "bridge"))
import sr75_jsbsim_pixhawk_hil_bridge as bridge  # noqa: E402
import sr75_mission_heading as smh  # noqa: E402


class TestPhaseBoundaries(unittest.TestCase):
    def test_phase_a_stationary_hold(self):
        for t in (0.0, 5.0, 9.99):
            heading, gs, vd, phase = nav.compute_heading_speed_vd(t)
            self.assertEqual(phase, "A")
            self.assertEqual(gs, 0.0)
            self.assertEqual(vd, 0.0)
            self.assertEqual(heading, nav.FIXED_YAW_DEG)

    def test_phase_b_northwest_cruise(self):
        for t in (10.0, 20.0, 29.99):
            heading, gs, vd, phase = nav.compute_heading_speed_vd(t)
            self.assertEqual(phase, "B")
            self.assertEqual(gs, 10.0)
            self.assertEqual(vd, 0.0)
            self.assertEqual(heading, nav.FIXED_YAW_DEG)

    def test_phase_c_turn_and_climb(self):
        heading0, gs0, vd0, phase0 = nav.compute_heading_speed_vd(30.0)
        self.assertEqual(phase0, "C")
        self.assertEqual(gs0, 10.0)
        self.assertEqual(vd0, -1.0, "climb must use vd=-1 m/s (NED down-positive)")
        self.assertAlmostEqual(heading0, nav.FIXED_YAW_DEG, places=4)
        heading_end, _, _, phase_end = nav.compute_heading_speed_vd(49.99)
        self.assertEqual(phase_end, "C")
        self.assertAlmostEqual(heading_end, 270.0, delta=0.1)

    def test_phase_d_west_cruise(self):
        for t in (50.0, 60.0, 69.99):
            heading, gs, vd, phase = nav.compute_heading_speed_vd(t)
            self.assertEqual(phase, "D")
            self.assertEqual(gs, 10.0)
            self.assertEqual(vd, 0.0)
            self.assertEqual(heading, 270.0)

    def test_phase_e_turn_and_descend(self):
        heading0, gs0, vd0, phase0 = nav.compute_heading_speed_vd(70.0)
        self.assertEqual(phase0, "E")
        self.assertEqual(gs0, 10.0)
        self.assertEqual(vd0, 1.0, "descend must use vd=+1 m/s (NED down-positive)")
        self.assertEqual(heading0, 270.0)
        heading_end, _, _, phase_end = nav.compute_heading_speed_vd(89.99)
        self.assertEqual(phase_end, "E")
        self.assertAlmostEqual(heading_end, nav.FIXED_YAW_DEG, delta=0.1)

    def test_phase_f_final_hold(self):
        for t in (90.0, 95.0, 99.99, 100.0):
            heading, gs, vd, phase = nav.compute_heading_speed_vd(t)
            self.assertEqual(phase, "F")
            self.assertEqual(gs, 0.0)
            self.assertEqual(vd, 0.0)
            self.assertEqual(heading, nav.FIXED_YAW_DEG)

    def test_clamping_before_and_after_profile(self):
        heading, gs, vd, phase = nav.compute_heading_speed_vd(-5.0)
        self.assertEqual(phase, "A")
        heading, gs, vd, phase = nav.compute_heading_speed_vd(1000.0)
        self.assertEqual(phase, "F")
        self.assertEqual(heading, nav.FIXED_YAW_DEG)

    def test_total_duration_is_100s(self):
        self.assertEqual(nav.PROFILE_DURATION_S, 100.0)


class TestYawRateLimit(unittest.TestCase):
    def test_phase_c_rate_is_about_2_25_deg_s(self):
        self.assertAlmostEqual(nav.YAW_RATE_C_DEG_S, 2.25, delta=0.01)

    def test_phase_e_rate_is_about_2_25_deg_s(self):
        self.assertAlmostEqual(nav.YAW_RATE_E_DEG_S, 2.25, delta=0.01)

    def test_phase_c_heading_is_monotonic_decreasing(self):
        prev = None
        for t in [30.0 + i for i in range(21)]:
            heading, _, _, _ = nav.compute_heading_speed_vd(t)
            if prev is not None:
                self.assertLessEqual(heading, prev + 1e-9)
            prev = heading

    def test_phase_e_heading_is_monotonic_increasing(self):
        prev = None
        for t in [70.0 + i for i in range(21)]:
            heading, _, _, _ = nav.compute_heading_speed_vd(t)
            if prev is not None:
                self.assertGreaterEqual(heading, prev - 1e-9)
            prev = heading

    def test_no_single_step_exceeds_rate_limit(self):
        dt = 0.5
        for t0 in [30.0 + i * dt for i in range(int(20 / dt))]:
            h0, _, _, _ = nav.compute_heading_speed_vd(t0)
            h1, _, _, _ = nav.compute_heading_speed_vd(t0 + dt)
            self.assertLessEqual(abs(h1 - h0) / dt, 2.3)  # allow tiny margin over 2.25


class TestVnVeSignsAndMagnitudes(unittest.TestCase):
    def test_stationary_phase_zero_velocity(self):
        vn, ve = nav.groundspeed_to_vn_ve(0.0, nav.FIXED_YAW_DEG)
        self.assertEqual(vn, 0.0)
        self.assertEqual(ve, 0.0)

    def test_northwest_heading_vn_positive_ve_negative(self):
        vn, ve = nav.groundspeed_to_vn_ve(10.0, nav.FIXED_YAW_DEG)
        self.assertGreater(vn, 0.0)
        self.assertLess(ve, 0.0)
        self.assertAlmostEqual(math.hypot(vn, ve), 10.0, places=6)

    def test_due_west_heading_vn_zero_ve_negative(self):
        vn, ve = nav.groundspeed_to_vn_ve(10.0, 270.0)
        self.assertAlmostEqual(vn, 0.0, places=6)
        self.assertAlmostEqual(ve, -10.0, places=6)

    def test_magnitude_always_matches_groundspeed(self):
        for heading in (0.0, 45.0, 90.0, 180.0, 270.0, 315.091444, 359.0):
            vn, ve = nav.groundspeed_to_vn_ve(10.0, heading)
            self.assertAlmostEqual(math.hypot(vn, ve), 10.0, places=6)


class TestAltitudeVdConsistency(unittest.TestCase):
    def test_climb_increases_altitude_by_20m(self):
        traj = nav.integrate_trajectory(dt_s=0.5, duration_s=100.0)
        alt_before_c = next(r["alt_m"] for r in traj if r["t_s"] >= 29.5)
        alt_after_c = next(r["alt_m"] for r in traj if r["t_s"] >= 49.5)
        self.assertAlmostEqual(alt_after_c - alt_before_c, 20.0, delta=0.5)

    def test_descend_decreases_altitude_by_20m(self):
        traj = nav.integrate_trajectory(dt_s=0.5, duration_s=100.0)
        alt_before_e = next(r["alt_m"] for r in traj if r["t_s"] >= 69.5)
        alt_after_e = next(r["alt_m"] for r in traj if r["t_s"] >= 89.5)
        self.assertAlmostEqual(alt_before_e - alt_after_e, 20.0, delta=0.5)

    def test_final_altitude_returns_to_3000m(self):
        traj = nav.integrate_trajectory(dt_s=0.5, duration_s=100.0)
        self.assertAlmostEqual(traj[-1]["alt_m"], 3000.0, places=3)

    def test_peak_altitude_is_3020m(self):
        traj = nav.integrate_trajectory(dt_s=0.5, duration_s=100.0)
        peak = max(r["alt_m"] for r in traj)
        self.assertAlmostEqual(peak, 3020.0, delta=0.5)


class TestLatLonIntegration(unittest.TestCase):
    def test_position_unchanged_during_stationary_phase_a(self):
        traj = nav.integrate_trajectory(dt_s=0.5, duration_s=10.0)
        self.assertAlmostEqual(traj[0]["lat_deg"], nav.FIXED_LAT, places=9)
        self.assertAlmostEqual(traj[-1]["lat_deg"], nav.FIXED_LAT, places=9)
        self.assertAlmostEqual(traj[-1]["lon_deg"], nav.FIXED_LON, places=9)

    def test_position_moves_during_phase_b(self):
        traj = nav.integrate_trajectory(dt_s=0.5, duration_s=30.0)
        row_10 = next(r for r in traj if r["t_s"] >= 10.0)
        row_30 = traj[-1]
        distance_m = smh.great_circle_distance_m(row_10["lat_deg"], row_10["lon_deg"], row_30["lat_deg"], row_30["lon_deg"])
        self.assertAlmostEqual(distance_m, 200.0, delta=2.0)  # 10 m/s * 20s

    def test_northwest_motion_increases_lat_decreases_lon(self):
        traj = nav.integrate_trajectory(dt_s=0.5, duration_s=30.0)
        row_10 = next(r for r in traj if r["t_s"] >= 10.0)
        row_30 = traj[-1]
        self.assertGreater(row_30["lat_deg"], row_10["lat_deg"])
        self.assertLess(row_30["lon_deg"], row_10["lon_deg"])

    def test_final_position_differs_from_start(self):
        # the trajectory returns to the start altitude/heading, but NOT
        # to the start lat/lon (net horizontal displacement is expected).
        traj = nav.integrate_trajectory(dt_s=0.5, duration_s=100.0)
        distance_m = smh.great_circle_distance_m(nav.FIXED_LAT, nav.FIXED_LON, traj[-1]["lat_deg"], traj[-1]["lon_deg"])
        self.assertGreater(distance_m, 100.0)

    def test_final_heading_and_altitude_return_to_start(self):
        traj = nav.integrate_trajectory(dt_s=0.5, duration_s=100.0)
        self.assertAlmostEqual(traj[-1]["heading_deg"], nav.FIXED_YAW_DEG, places=4)
        self.assertAlmostEqual(traj[-1]["alt_m"], nav.BASE_ALT_M, places=3)


class TestGpsYawEncoding(unittest.TestCase):
    def test_trajectory_row_encodes_yaw_correctly(self):
        row = {"t_s": 0.0, "lat_deg": nav.FIXED_LAT, "lon_deg": nav.FIXED_LON, "alt_m": 3000.0,
               "vn_mps": 0.0, "ve_mps": 0.0, "vd_mps": 0.0, "heading_deg": 315.091444}
        csv_row = nav.trajectory_row_to_csv_row(row)
        idx = {name: i for i, name in enumerate(nav.PROFILE_HEADER)}
        yaw_rad = float(csv_row[idx["jsb_yaw_rad"]])
        self.assertAlmostEqual(math.degrees(yaw_rad), 315.091444, places=4)

    def test_encode_gps_input_yaw_cdeg_matches_mission_heading_module(self):
        # F2B3 reuses (does not reimplement) the already-validated
        # F23-F2A encoder -- confirm it is the same function/behaviour.
        cdeg = smh.encode_gps_input_yaw_cdeg(315.091444)
        self.assertEqual(cdeg, 31509)

    def test_phase_c_end_heading_encodes_near_270(self):
        heading, _, _, _ = nav.compute_heading_speed_vd(49.99)
        cdeg = smh.encode_gps_input_yaw_cdeg(heading)
        self.assertAlmostEqual(cdeg / 100.0, 270.0, delta=0.1)


class TestWriteProfileCsvRealtime(unittest.TestCase):
    def test_writes_header_and_rows(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "profile.csv"
            trajectory = nav.integrate_trajectory(dt_s=0.05, duration_s=0.3)
            rows_written = nav.write_profile_csv_realtime(path, trajectory=trajectory, rate_hz=20.0)
            self.assertEqual(rows_written, len(trajectory))
            with open(path, newline="") as f:
                reader = list(csv.reader(f))
            self.assertEqual(reader[0], nav.PROFILE_HEADER)
            self.assertEqual(len(reader) - 1, rows_written)

    def test_alive_check_stops_writer_immediately(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "profile.csv"
            trajectory = nav.integrate_trajectory(dt_s=0.1, duration_s=100.0)
            start = time.time()
            rows_written = nav.write_profile_csv_realtime(
                path, trajectory=trajectory, rate_hz=20.0,
                alive_check=lambda: "bridge exited unexpectedly rc=1",
            )
            elapsed = time.time() - start
            self.assertEqual(rows_written, 0)
            self.assertLess(elapsed, 1.0)

    def test_output_is_readable_by_real_bridge_feeder(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "profile.csv"
            trajectory = nav.integrate_trajectory(dt_s=0.05, duration_s=0.2)
            nav.write_profile_csv_realtime(path, trajectory=trajectory, rate_hz=20.0)
            feeder = bridge.JSBSimStateFeeder(str(path))
            raw = feeder._read_raw_latest()
            self.assertIsNotNone(raw)
            self.assertAlmostEqual(float(raw["jsb_feed_lat_deg"]), nav.FIXED_LAT, places=5)


class TestBuildBridgeCmdForNavProfile(unittest.TestCase):
    def _cmd(self, mp_out="udp:192.168.64.1:14550"):
        return nav.build_bridge_cmd_for_nav_profile(
            "/dev/ttyACM0", 115200, "/tmp/profile.csv", mp_out, "/tmp/log.csv",
        )

    def test_includes_gps_input_inject_and_send_yaw(self):
        cmd = self._cmd()
        self.assertIn("--gps-input-inject", cmd)
        self.assertIn("--gps-input-send-yaw", cmd)

    def test_never_includes_attitude_or_airspeed_inject(self):
        cmd = self._cmd()
        self.assertNotIn("--attitude-inject", cmd)
        self.assertNotIn("--airspeed-inject", cmd)

    def test_never_includes_allow_actuator_output(self):
        self.assertNotIn("--allow-actuator-output", self._cmd())

    def test_mp_out_propagates_and_can_be_disabled(self):
        cmd = self._cmd(mp_out="udp:10.0.0.5:14550")
        self.assertEqual(cmd[cmd.index("--mp-out") + 1], "udp:10.0.0.5:14550")
        self.assertNotIn("--mp-out", self._cmd(mp_out=""))


class TestPreflightEvaluation(unittest.TestCase):
    def _good_params(self):
        return {"GPS1_TYPE": 14, "EK3_SRC1_POSXY": 3, "EK3_SRC1_VELXY": 3,
                "EK3_SRC1_VELZ": 3, "EK3_SRC1_POSZ": 3, "EK3_SRC1_YAW": 2}

    def test_all_correct_passes(self):
        ok, failures = nav.evaluate_combined_preflight(False, "MANUAL", self._good_params())
        self.assertTrue(ok)
        self.assertEqual(failures, [])

    def test_armed_fails(self):
        ok, failures = nav.evaluate_combined_preflight(True, "MANUAL", self._good_params())
        self.assertFalse(ok)
        self.assertTrue(any("ARMED" in f for f in failures))

    def test_wrong_mode_fails(self):
        ok, failures = nav.evaluate_combined_preflight(False, "AUTO", self._good_params())
        self.assertFalse(ok)
        self.assertTrue(any("mode=AUTO" in f for f in failures))

    def test_each_required_param_individually_gates(self):
        for name in ("GPS1_TYPE", "EK3_SRC1_POSXY", "EK3_SRC1_VELXY", "EK3_SRC1_VELZ",
                     "EK3_SRC1_POSZ", "EK3_SRC1_YAW"):
            params = self._good_params()
            params[name] = 0
            ok, failures = nav.evaluate_combined_preflight(False, "MANUAL", params)
            self.assertFalse(ok, f"{name}=0 should have failed preflight")
            self.assertTrue(any(name in f for f in failures))

    def test_missing_param_response_fails(self):
        params = self._good_params()
        params["EK3_SRC1_YAW"] = None
        ok, failures = nav.evaluate_combined_preflight(False, "MANUAL", params)
        self.assertFalse(ok)
        self.assertTrue(any("EK3_SRC1_YAW did not respond" in f for f in failures))


class TestPhaseSummaryAndPassCriteria(unittest.TestCase):
    FIELDNAMES = [
        "time_s", "jsb_feed_time_s", "mode", "armed",
        "gps_tx_lat_deg", "gps_tx_lon_deg", "gps_tx_alt_m",
        "gps_tx_vn_mps", "gps_tx_ve_mps", "gps_tx_vd_mps", "gps_tx_yaw_cdeg",
        "lat_deg", "lon_deg", "alt_m", "gps_raw_lat_deg", "gps_raw_lon_deg", "gps_raw_alt_m",
        "gps_raw_fix_type", "groundspeed_mps", "ahrs2_yaw_rad", "ekf_flags", "local_ned_z_m",
        "servo7_pwm", "servo8_pwm", "rc7_pwm", "rc8_pwm",
    ]

    def _write_log(self, tmpdir, rows):
        path = Path(tmpdir) / "bridge_log.csv"
        with open(path, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=self.FIELDNAMES)
            w.writeheader()
            w.writerows(rows)
        return path

    def _good_row(self, t, commanded_t=None, **overrides):
        # HIL-F23-F2B3A: jsb_feed_time_s (the COMMANDED profile time) is
        # the field phase classification now keys on; it defaults to t
        # (the bridge's own wall-clock time_s) unless a test explicitly
        # wants to exercise the offset between the two.
        if commanded_t is None:
            commanded_t = t
        base = {
            "time_s": str(t), "jsb_feed_time_s": str(commanded_t), "mode": "MANUAL", "armed": "0",
            "gps_tx_lat_deg": f"{nav.FIXED_LAT:.7f}", "gps_tx_lon_deg": f"{nav.FIXED_LON:.7f}",
            "gps_tx_alt_m": "3000.0", "gps_tx_vn_mps": "0.0", "gps_tx_ve_mps": "0.0",
            "gps_tx_vd_mps": "0.0", "gps_tx_yaw_cdeg": "31509",
            "lat_deg": f"{nav.FIXED_LAT:.7f}", "lon_deg": f"{nav.FIXED_LON:.7f}", "alt_m": "3000.5",
            "gps_raw_lat_deg": f"{nav.FIXED_LAT:.7f}", "gps_raw_lon_deg": f"{nav.FIXED_LON:.7f}", "gps_raw_alt_m": "3000.2",
            "gps_raw_fix_type": "3",
            "groundspeed_mps": "0.1", "ahrs2_yaw_rad": str(math.radians(315.0)),
            "ekf_flags": "831", "local_ned_z_m": "-3000.0",
            "servo7_pwm": "0", "servo8_pwm": "0", "rc7_pwm": "0", "rc8_pwm": "0",
        }
        base.update(overrides)
        return base

    def test_classify_phase_from_time_boundaries(self):
        # HIL-F23-F2B3A item 2: strict ranges A[0,10) B[10,30) C[30,50)
        # D[50,70) E[70,90) F[90,100].
        self.assertEqual(nav.classify_phase_from_time(0.0), "A")
        self.assertEqual(nav.classify_phase_from_time(9.999), "A")
        self.assertEqual(nav.classify_phase_from_time(10.0), "B")
        self.assertEqual(nav.classify_phase_from_time(29.999), "B")
        self.assertEqual(nav.classify_phase_from_time(30.0), "C")
        self.assertEqual(nav.classify_phase_from_time(49.999), "C")
        self.assertEqual(nav.classify_phase_from_time(50.0), "D")
        self.assertEqual(nav.classify_phase_from_time(69.999), "D")
        self.assertEqual(nav.classify_phase_from_time(70.0), "E")
        self.assertEqual(nav.classify_phase_from_time(89.999), "E")
        self.assertEqual(nav.classify_phase_from_time(90.0), "F")
        self.assertEqual(nav.classify_phase_from_time(100.0), "F")

    def test_classify_phase_from_time_excludes_invalid(self):
        self.assertIsNone(nav.classify_phase_from_time(None))
        self.assertIsNone(nav.classify_phase_from_time(-0.001))
        self.assertIsNone(nav.classify_phase_from_time(100.001))

    def test_classify_phase_from_commanded_reproduces_the_reported_bug(self):
        # HIL-F23-F2B3A root cause, kept as a permanent regression proof:
        # the OLD value-based classifier cannot distinguish a stationary
        # hold (A/F: alt=3000, vd=0) from cruising through the same
        # altitude (B: alt=3000, vd=0) -- all three collapse to "A".
        self.assertEqual(nav.classify_phase_from_commanded(3000.0, 0.0), "A")  # meant to be A
        self.assertEqual(nav.classify_phase_from_commanded(3000.0, 0.0), "A")  # meant to be B -- BUG: same as A
        self.assertEqual(nav.classify_phase_from_commanded(3000.0, 0.0), "A")  # meant to be F -- BUG: same as A
        self.assertEqual(nav.classify_phase_from_commanded(3020.0, 0.0), "D")
        self.assertEqual(nav.classify_phase_from_commanded(3010.0, -1.0), "C")
        self.assertEqual(nav.classify_phase_from_commanded(3010.0, 1.0), "E")
        self.assertIsNone(nav.classify_phase_from_commanded(None, 0.0))

    def test_full_profile_phase_counts_match_expected(self):
        # HIL-F23-F2B3A item 11: regression proof of the fix using a full
        # ~100-row synthetic dataset (one row per second, matching the
        # bridge's real ~1Hz logging cadence) spanning the whole profile.
        with tempfile.TemporaryDirectory() as tmpdir:
            rows = []
            for i in range(100):
                t = float(i)
                heading, gs, vd, phase = nav.compute_heading_speed_vd(t)
                alt = nav.BASE_ALT_M if phase in ("A", "B", "F") else (
                    nav.PEAK_ALT_M if phase == "D" else 3010.0)
                rows.append(self._good_row(t, commanded_t=t, gps_tx_alt_m=str(alt), gps_tx_vd_mps=str(vd)))
            path = self._write_log(tmpdir, rows)
            summary = nav.compute_phase_summary(path)
            counts = {p: summary["per_phase"][p]["n_samples"] for p in nav.PHASES_IN_ORDER}
            self.assertEqual(counts, {"A": 10, "B": 20, "C": 20, "D": 20, "E": 20, "F": 10})
            self.assertEqual(sum(counts.values()), 100)
            self.assertEqual(summary["n_samples"], 100)

    def test_rows_without_commanded_time_are_excluded_and_counted(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            rows = [
                self._good_row(0.0, commanded_t=""),  # no producer row yet -- must be excluded
                self._good_row(1.0, commanded_t=5.0),
            ]
            path = self._write_log(tmpdir, rows)
            summary = nav.compute_phase_summary(path)
            self.assertEqual(summary["n_samples"], 1)
            self.assertEqual(summary["excluded_no_commanded_time"], 1)

    def test_per_phase_first_last_timestamp_reported(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            rows = [
                self._good_row(0.0, commanded_t=1.0),
                self._good_row(1.0, commanded_t=2.0),
                self._good_row(2.0, commanded_t=8.0),
            ]
            path = self._write_log(tmpdir, rows)
            summary = nav.compute_phase_summary(path)
            self.assertEqual(summary["per_phase"]["A"]["first_t"], 1.0)
            self.assertEqual(summary["per_phase"]["A"]["last_t"], 8.0)
            self.assertEqual(summary["per_phase"]["A"]["n_samples"], 3)

    def test_discontinuity_records_include_phase_and_boundary_flag(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            rows = [
                self._good_row(0.0, commanded_t=35.0, local_ned_z_m="-3000.0"),
                self._good_row(1.0, commanded_t=36.0, local_ned_z_m="-3010.0"),  # 10m jump, mid-phase-C
            ]
            path = self._write_log(tmpdir, rows)
            summary = nav.compute_phase_summary(path)
            self.assertEqual(len(summary["discontinuities"]), 1)
            d = summary["discontinuities"][0]
            self.assertEqual(d["phase"], "C")
            self.assertEqual(d["commanded_t"], 36.0)
            self.assertFalse(d["near_phase_boundary"])  # 36.0 is 6s from the nearest boundary (30 or 50)

    def test_horizontal_and_altitude_error_computed(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            rows = [self._good_row(0.0, alt_m="3002.0", gps_raw_alt_m="3000.5")]
            path = self._write_log(tmpdir, rows)
            summary = nav.compute_phase_summary(path)
            self.assertAlmostEqual(summary["per_phase"]["A"]["alt_err_global_m"]["max"], 2.0, places=2)

    def test_groundspeed_error_computed(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            # commanded groundspeed = hypot(7.071, 7.071) ~= 10.0 m/s;
            # actual telemetry reports 8.0 m/s -> expected error ~= 2.0 m/s.
            rows = [self._good_row(
                0.0, gps_tx_vn_mps="7.071", gps_tx_ve_mps="-7.071", groundspeed_mps="8.0",
            )]
            path = self._write_log(tmpdir, rows)
            summary = nav.compute_phase_summary(path)
            self.assertAlmostEqual(summary["per_phase"]["A"]["gs_err_mps"]["max"], 2.0, delta=0.05)

    def test_groundspeed_error_near_zero_when_matching(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            rows = [self._good_row(
                0.0, gps_tx_vn_mps="7.071", gps_tx_ve_mps="-7.071", groundspeed_mps="10.0",
            )]
            path = self._write_log(tmpdir, rows)
            summary = nav.compute_phase_summary(path)
            self.assertAlmostEqual(summary["per_phase"]["A"]["gs_err_mps"]["max"], 0.0, delta=0.05)

    def test_wrapped_yaw_error_computed(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            # commanded 315.09 cdeg-encoded, actual ahrs2 yaw slightly off
            rows = [self._good_row(0.0, gps_tx_yaw_cdeg="31509", ahrs2_yaw_rad=str(math.radians(313.0)))]
            path = self._write_log(tmpdir, rows)
            summary = nav.compute_phase_summary(path)
            self.assertAlmostEqual(summary["per_phase"]["A"]["yaw_err_deg"]["max"], 2.09, delta=0.05)

    def test_ekf_flags_change_detected(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            rows = [
                self._good_row(0.0, ekf_flags="831"),
                self._good_row(1.0, ekf_flags="167"),  # matches observed hardware flag toggle
            ]
            path = self._write_log(tmpdir, rows)
            summary = nav.compute_phase_summary(path)
            types_found = {d["type"] for d in summary["discontinuities"]}
            self.assertIn("ekf_flags_change", types_found)

    def test_local_ned_jump_detected(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            rows = [
                self._good_row(0.0, local_ned_z_m="-3000.0"),
                self._good_row(1.0, local_ned_z_m="-3420.0"),  # >400m jump, matches observed divergence
            ]
            path = self._write_log(tmpdir, rows)
            summary = nav.compute_phase_summary(path)
            types_found = {d["type"] for d in summary["discontinuities"]}
            self.assertIn("local_ned_z_jump", types_found)

    def test_stale_gap_detected(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            rows = [self._good_row(0.0), self._good_row(5.0)]  # 5s gap > 2s default threshold
            path = self._write_log(tmpdir, rows)
            summary = nav.compute_phase_summary(path)
            self.assertEqual(len(summary["stale_gaps"]), 1)

    def test_non_manual_and_armed_and_ch_nonzero_detected(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            rows = [
                self._good_row(0.0, mode="AUTO"),
                self._good_row(1.0, armed="1"),
                self._good_row(2.0, servo7_pwm="1500"),
            ]
            path = self._write_log(tmpdir, rows)
            summary = nav.compute_phase_summary(path)
            self.assertEqual(summary["non_manual_samples"], 1)
            self.assertEqual(summary["armed_samples"], 1)
            self.assertEqual(summary["ch7_ch8_nonzero_samples"], 1)

    def test_evaluate_pass_criteria_all_pass(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            # commanded_t placed inside each named phase's real time range
            # (HIL-F23-F2B3A: classification is time-based now, not
            # value-based) -- alt/vd stay thematically consistent with
            # what's actually commanded in that phase.
            rows = []
            for commanded_t, alt, vd in ((5.0, 3000.0, 0.0), (20.0, 3000.0, 0.0),
                                          (60.0, 3020.0, 0.0), (95.0, 3000.0, 0.0)):
                rows.append(self._good_row(
                    float(len(rows)), commanded_t=commanded_t,  # closely-spaced time_s avoids a spurious stale-gap
                    gps_tx_alt_m=str(alt), gps_tx_vd_mps=str(vd),
                    alt_m=str(alt + 0.5), gps_raw_alt_m=str(alt + 0.2),
                    ahrs2_yaw_rad=str(math.radians(315.0)),
                ))
            path = self._write_log(tmpdir, rows)
            summary = nav.compute_phase_summary(path)
            results = nav.evaluate_pass_criteria(summary)
            self.assertTrue(results["overall"][0], results)

    def test_evaluate_pass_criteria_fails_on_large_altitude_error(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            rows = [self._good_row(0.0, gps_tx_alt_m="3000.0", gps_tx_vd_mps="0.0", alt_m="3020.0")]
            path = self._write_log(tmpdir, rows)
            summary = nav.compute_phase_summary(path)
            results = nav.evaluate_pass_criteria(summary)
            self.assertFalse(results["altitude_error_settled"][0])
            self.assertFalse(results["overall"][0])

    def test_evaluate_pass_criteria_fails_on_armed(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            rows = [self._good_row(0.0, armed="1")]
            path = self._write_log(tmpdir, rows)
            summary = nav.compute_phase_summary(path)
            results = nav.evaluate_pass_criteria(summary)
            self.assertFalse(results["manual_disarmed_throughout"][0])
            self.assertFalse(results["overall"][0])


class TestCliDryRun(unittest.TestCase):
    def test_dry_run_prints_plan_and_never_touches_hardware(self):
        argv = ["--dry-run", "--run-dir", tempfile.mkdtemp()]
        old_argv = sys.argv
        sys.argv = ["sr75_hil_f2b3_combined_nav_profile.py"] + argv
        buf = io.StringIO()
        try:
            import contextlib
            with contextlib.redirect_stdout(buf):
                rc = nav.main()
        finally:
            sys.argv = old_argv
        self.assertEqual(rc, 0)
        output = buf.getvalue()
        self.assertIn("DRY RUN", output)
        self.assertIn("--gps-input-inject --gps-input-send-yaw", output)
        self.assertIn("Attitude/airspeed display injection: DISABLED", output)

    def test_dry_run_mentions_preconditioning(self):
        argv = ["--dry-run", "--run-dir", tempfile.mkdtemp()]
        old_argv = sys.argv
        sys.argv = ["sr75_hil_f2b3_combined_nav_profile.py"] + argv
        buf = io.StringIO()
        try:
            import contextlib
            with contextlib.redirect_stdout(buf):
                nav.main()
        finally:
            sys.argv = old_argv
        output = buf.getvalue()
        self.assertIn("PRECONDITIONING", output)
        self.assertIn("consecutive", output)


class TestPreconditionTrajectory(unittest.TestCase):
    """HIL-F23-F2B3B item 1/9: precondition trajectory tests."""

    def test_heading_and_groundspeed_match_task_spec(self):
        self.assertEqual(nav.PRECONDITION_HEADING_DEG, nav.FIXED_YAW_DEG)
        self.assertEqual(nav.PRECONDITION_GROUNDSPEED_MPS, 10.0)

    def test_vn_ve_match_task_approximate_values(self):
        vn, ve = nav.groundspeed_to_vn_ve(nav.PRECONDITION_GROUNDSPEED_MPS, nav.PRECONDITION_HEADING_DEG)
        self.assertAlmostEqual(vn, 7.08, places=2)
        self.assertAlmostEqual(ve, -7.06, places=2)

    def test_altitude_constant_and_vd_zero(self):
        traj = nav.generate_precondition_trajectory(dt_s=1.0, max_duration_s=5.0)
        for row in traj:
            self.assertEqual(row["alt_m"], nav.BASE_ALT_M)
            self.assertEqual(row["vd_mps"], 0.0)

    def test_heading_constant(self):
        traj = nav.generate_precondition_trajectory(dt_s=1.0, max_duration_s=5.0)
        for row in traj:
            self.assertEqual(row["heading_deg"], nav.PRECONDITION_HEADING_DEG)

    def test_position_moves_continuously(self):
        traj = nav.generate_precondition_trajectory(dt_s=1.0, max_duration_s=5.0,
                                                      start_lat=nav.FIXED_LAT, start_lon=nav.FIXED_LON)
        self.assertEqual(traj[0]["lat_deg"], nav.FIXED_LAT)
        self.assertEqual(traj[0]["lon_deg"], nav.FIXED_LON)
        for i in range(1, len(traj)):
            self.assertGreater(traj[i]["lat_deg"], traj[i - 1]["lat_deg"])
            self.assertLess(traj[i]["lon_deg"], traj[i - 1]["lon_deg"])

    def test_default_max_duration_is_15s(self):
        self.assertEqual(nav.PRECONDITION_MAX_DURATION_S, 15.0)

    def test_t_s_is_negative_throughout_and_approaches_zero(self):
        # HIL-F23-F2B3B item 2: precondition rows must never advance the
        # A-F profile timer -- t_s stays strictly negative.
        traj = nav.generate_precondition_trajectory(dt_s=0.5, max_duration_s=15.0)
        for row in traj:
            self.assertLess(row["t_s"], 0.0)
        self.assertAlmostEqual(traj[0]["t_s"], -15.0, places=3)
        self.assertLess(traj[-1]["t_s"], 0.0)
        self.assertGreater(traj[-1]["t_s"], -1.0)  # approaches (but never reaches) 0.0


class TestTimerNotAdvancingDuringPrecondition(unittest.TestCase):
    """HIL-F23-F2B3B item 2: classify_phase_from_time() must exclude
    every precondition row (t_s<0) from the A-F profile entirely.
    """

    def test_all_precondition_rows_classify_as_none(self):
        traj = nav.generate_precondition_trajectory(dt_s=0.5, max_duration_s=15.0)
        for row in traj:
            self.assertIsNone(nav.classify_phase_from_time(row["t_s"]))

    def test_negative_time_never_counted_in_phase_summary(self):
        # a compute_phase_summary() row with a negative jsb_feed_time_s
        # must be excluded (item 3), not misclassified as phase A.
        with tempfile.TemporaryDirectory() as tmpdir:
            fieldnames = [
                "time_s", "jsb_feed_time_s", "mode", "armed",
                "gps_tx_lat_deg", "gps_tx_lon_deg", "gps_tx_alt_m",
                "gps_tx_vn_mps", "gps_tx_ve_mps", "gps_tx_vd_mps", "gps_tx_yaw_cdeg",
                "lat_deg", "lon_deg", "alt_m", "gps_raw_lat_deg", "gps_raw_lon_deg",
                "gps_raw_alt_m", "gps_raw_fix_type", "groundspeed_mps", "ahrs2_yaw_rad",
                "ekf_flags", "local_ned_z_m", "servo7_pwm", "servo8_pwm", "rc7_pwm", "rc8_pwm",
            ]
            row = {name: "" for name in fieldnames}
            row.update({
                "time_s": "0.0", "jsb_feed_time_s": "-5.0", "mode": "MANUAL", "armed": "0",
                "gps_tx_lat_deg": "32.5378147", "gps_tx_lon_deg": "74.3661871",
                "gps_tx_alt_m": "3000.0", "gps_tx_vd_mps": "0.0",
            })
            path = Path(tmpdir) / "bridge_log.csv"
            with open(path, "w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=fieldnames)
                w.writeheader()
                w.writerow(row)
            summary = nav.compute_phase_summary(path)
            self.assertEqual(summary["n_samples"], 0)
            self.assertEqual(summary["per_phase"]["A"]["n_samples"], 0)


class TestPreconditionGate(unittest.TestCase):
    """HIL-F23-F2B3B items 3-5, 7: the 3-consecutive-sample gate,
    ahrs2_yaw_rad usage, EKF flag requirement, timeout abort.
    """

    def _sample(self, ahrs2_yaw_deg=315.091444, commanded_yaw_deg=315.091444, ekf_flags=25, gps_fix_type=3):
        return {
            "ahrs2_yaw_deg": ahrs2_yaw_deg, "commanded_yaw_deg": commanded_yaw_deg,
            "ekf_flags": ekf_flags, "gps_fix_type": gps_fix_type,
        }

    def test_three_consecutive_good_samples_pass(self):
        samples = [self._sample() for _ in range(3)]
        passed, _reason, consecutive = nav.evaluate_precondition_gate_samples(samples)
        self.assertTrue(passed)
        self.assertEqual(consecutive, 3)

    def test_two_consecutive_good_samples_do_not_pass(self):
        samples = [self._sample() for _ in range(2)]
        passed, _reason, consecutive = nav.evaluate_precondition_gate_samples(samples)
        self.assertFalse(passed)
        self.assertEqual(consecutive, 2)

    def test_streak_resets_on_a_bad_sample(self):
        samples = [self._sample(), self._sample(), self._sample(ahrs2_yaw_deg=100.0), self._sample()]
        passed, _reason, consecutive = nav.evaluate_precondition_gate_samples(samples)
        self.assertFalse(passed)
        self.assertEqual(consecutive, 1)  # only the last sample counts after the reset

    def test_yaw_error_exactly_at_tolerance_passes(self):
        samples = [self._sample(ahrs2_yaw_deg=315.091444 - 2.0) for _ in range(3)]
        passed, _reason, _consecutive = nav.evaluate_precondition_gate_samples(samples)
        self.assertTrue(passed)

    def test_yaw_error_just_over_tolerance_fails(self):
        samples = [self._sample(ahrs2_yaw_deg=315.091444 - 2.1) for _ in range(3)]
        passed, _reason, _consecutive = nav.evaluate_precondition_gate_samples(samples)
        self.assertFalse(passed)

    def test_ekf_flag_requirement_pos_horiz_rel(self):
        # EKF_POS_HORIZ_REL(8) alone is sufficient.
        samples = [self._sample(ekf_flags=8) for _ in range(3)]
        passed, _reason, _consecutive = nav.evaluate_precondition_gate_samples(samples)
        self.assertTrue(passed)

    def test_ekf_flag_requirement_pos_horiz_abs(self):
        # EKF_POS_HORIZ_ABS(16) alone is sufficient.
        samples = [self._sample(ekf_flags=16) for _ in range(3)]
        passed, _reason, _consecutive = nav.evaluate_precondition_gate_samples(samples)
        self.assertTrue(passed)

    def test_ekf_flag_requirement_fails_without_pos_horiz_bits(self):
        # the exact observed hardware pattern: 167 = ATTITUDE+VEL_HORIZ+
        # VEL_VERT+POS_VERT_ABS+CONST_POS_MODE, with NEITHER
        # POS_HORIZ_REL(8) NOR POS_HORIZ_ABS(16) set.
        samples = [self._sample(ekf_flags=167) for _ in range(3)]
        passed, _reason, _consecutive = nav.evaluate_precondition_gate_samples(samples)
        self.assertFalse(passed)

    def test_gps_fix_requirement(self):
        samples = [self._sample(gps_fix_type=2) for _ in range(3)]
        passed, _reason, _consecutive = nav.evaluate_precondition_gate_samples(samples)
        self.assertFalse(passed)

    def test_empty_samples_never_pass(self):
        passed, reason, consecutive = nav.evaluate_precondition_gate_samples([])
        self.assertFalse(passed)
        self.assertEqual(consecutive, 0)
        self.assertIn("no samples", reason)

    def test_read_bridge_log_tail_samples_uses_ahrs2_yaw_rad(self):
        # HIL-F23-F2B3B item 7: estimator yaw comparisons must use
        # ahrs2_yaw_rad, not att_yaw_rad/yaw_rad (raw ATTITUDE).
        with tempfile.TemporaryDirectory() as tmpdir:
            fieldnames = ["time_s", "ahrs2_yaw_rad", "yaw_rad", "att_yaw_rad", "ekf_flags",
                          "gps_raw_fix_type", "gps_tx_yaw_cdeg"]
            path = Path(tmpdir) / "bridge_log.csv"
            with open(path, "w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=fieldnames)
                w.writeheader()
                w.writerow({
                    "time_s": "1.0",
                    "ahrs2_yaw_rad": str(math.radians(315.091444)),  # correct estimator column
                    "yaw_rad": str(math.radians(0.0)),               # raw ATTITUDE -- must NOT be used
                    "att_yaw_rad": str(math.radians(0.0)),           # display override -- must NOT be used
                    "ekf_flags": "25", "gps_raw_fix_type": "3", "gps_tx_yaw_cdeg": "31509",
                })
            samples = nav.read_bridge_log_tail_samples(path)
            self.assertEqual(len(samples), 1)
            self.assertAlmostEqual(samples[0]["ahrs2_yaw_deg"], 315.091444, places=4)

    def test_missing_bridge_log_returns_empty_samples(self):
        self.assertEqual(nav.read_bridge_log_tail_samples("/tmp/does-not-exist-f2b3b.csv"), [])


class TestRunPreconditionAndProfile(unittest.TestCase):
    """HIL-F23-F2B3B items 4-6: end-to-end (no hardware) tests of the
    precondition -> gate -> profile orchestration.
    """

    FIELDNAMES = ["time_s", "ahrs2_yaw_rad", "ekf_flags", "gps_raw_fix_type", "gps_tx_yaw_cdeg"]

    def test_timeout_aborts_with_no_profile_rows_written(self):
        # HIL-F23-F2B3B item 5: abort if the gate is not met within the cap.
        with tempfile.TemporaryDirectory() as tmpdir:
            profile_csv = Path(tmpdir) / "profile.csv"
            bridge_log_csv = Path(tmpdir) / "never_created_bridge_log.csv"
            result = nav.run_precondition_and_profile(
                profile_csv, bridge_log_csv, trajectory_dt_s=0.2,
                precondition_max_duration_s=0.6, profile_duration_s=100.0, gate_poll_interval_s=0.2,
            )
            self.assertFalse(result["gate_passed"])
            with open(profile_csv, newline="") as f:
                rows = list(csv.reader(f))
            data_rows = rows[1:]
            self.assertTrue(all(float(r[0]) < 0 for r in data_rows), "no A-F profile row (t_s>=0) may be written on timeout")

    def test_gate_pass_transitions_immediately_to_profile_with_continuous_position(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            profile_csv = Path(tmpdir) / "profile.csv"
            bridge_log_csv = Path(tmpdir) / "bridge_log.csv"
            # a bridge log that is ALREADY converged from the first sample.
            with open(bridge_log_csv, "w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=self.FIELDNAMES)
                w.writeheader()
                for t in range(4):
                    w.writerow({
                        "time_s": str(t), "ahrs2_yaw_rad": str(math.radians(315.091444)),
                        "ekf_flags": "25", "gps_raw_fix_type": "3", "gps_tx_yaw_cdeg": "31509",
                    })
            result = nav.run_precondition_and_profile(
                profile_csv, bridge_log_csv, trajectory_dt_s=0.1,
                precondition_max_duration_s=15.0, profile_duration_s=1.0, gate_poll_interval_s=0.1,
            )
            self.assertTrue(result["gate_passed"])
            with open(profile_csv, newline="") as f:
                rows = list(csv.reader(f))
            data_rows = rows[1:]
            self.assertTrue(any(float(r[0]) < 0 for r in data_rows), "must contain precondition rows")
            self.assertTrue(any(float(r[0]) >= 0 for r in data_rows), "must contain A-F profile rows once gate passes")
            # position continuity: the first profile row's lat/lon must
            # equal the precondition's final lat/lon (item 1/6), not the
            # original FIXED_LAT/FIXED_LON.
            first_profile_row = next(r for r in data_rows if float(r[0]) >= 0)
            last_precondition_row = [r for r in data_rows if float(r[0]) < 0][-1]
            self.assertAlmostEqual(float(first_profile_row[2]), float(last_precondition_row[2]), places=5)
            self.assertAlmostEqual(float(first_profile_row[3]), float(last_precondition_row[3]), places=5)

    def test_position_actually_advances_when_precondition_runs_for_a_measurable_time(self):
        # Distinct from the "immediate pass" test above: here the bridge
        # log converges only AFTER a short delay (written by a background
        # thread, mimicking a real EKF taking a moment to lock), so a
        # measurable number of precondition rows are written and the
        # hand-off position must have genuinely moved from FIXED_LAT/LON.
        with tempfile.TemporaryDirectory() as tmpdir:
            profile_csv = Path(tmpdir) / "profile.csv"
            bridge_log_csv = Path(tmpdir) / "bridge_log.csv"

            stop_event = threading.Event()

            def delayed_converge():
                with open(bridge_log_csv, "w", newline="") as f:
                    w = csv.DictWriter(f, fieldnames=self.FIELDNAMES)
                    w.writeheader()
                    f.flush()
                t = 0.0
                while not stop_event.is_set():
                    converged = t >= 2.0  # unconverged for the first 2s, then converges and stays converged
                    try:
                        with open(bridge_log_csv, "a", newline="") as f2:
                            csv.DictWriter(f2, fieldnames=self.FIELDNAMES).writerow({
                                "time_s": str(t),
                                "ahrs2_yaw_rad": str(math.radians(315.091444)) if converged else "0.0",
                                "ekf_flags": "25" if converged else "167",
                                "gps_raw_fix_type": "3", "gps_tx_yaw_cdeg": "31509",
                            })
                    except OSError:
                        return  # tmpdir torn down after the test already finished
                    t += 0.3
                    stop_event.wait(0.3)

            writer_thread = threading.Thread(target=delayed_converge, daemon=True)
            writer_thread.start()
            try:
                result = nav.run_precondition_and_profile(
                    profile_csv, bridge_log_csv, trajectory_dt_s=0.1,
                    precondition_max_duration_s=15.0, profile_duration_s=0.5, gate_poll_interval_s=0.3,
                )
            finally:
                stop_event.set()
                writer_thread.join(timeout=2.0)
            self.assertTrue(result["gate_passed"])
            self.assertGreater(result["precondition_elapsed_s"], 1.0)
            self.assertNotAlmostEqual(result["profile_start_lat"], nav.FIXED_LAT, places=5)

    def test_alive_check_aborts_precondition_immediately(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            profile_csv = Path(tmpdir) / "profile.csv"
            bridge_log_csv = Path(tmpdir) / "never_created.csv"
            start = time.time()
            result = nav.run_precondition_and_profile(
                profile_csv, bridge_log_csv, trajectory_dt_s=0.1,
                precondition_max_duration_s=15.0, profile_duration_s=100.0,
                alive_check=lambda: "bridge exited unexpectedly rc=1",
            )
            elapsed = time.time() - start
            self.assertFalse(result["gate_passed"])
            self.assertIn("aborted", result["gate_reason"])
            self.assertLess(elapsed, 1.0)


class TestNoActuatorAttitudeAirspeedDuringPrecondition(unittest.TestCase):
    """HIL-F23-F2B3B item 9: preconditioning must not change the bridge
    command's safety posture -- no actuator output, attitude injection,
    or airspeed injection are ever introduced.
    """

    def test_bridge_cmd_unaffected_by_preconditioning(self):
        cmd = nav.build_bridge_cmd_for_nav_profile(
            "/dev/ttyACM0", 115200, "/tmp/profile.csv", "udp:192.168.64.1:14550", "/tmp/log.csv",
        )
        for forbidden in ("--allow-actuator-output", "--attitude-inject", "--airspeed-inject"):
            self.assertNotIn(forbidden, cmd)
        self.assertIn("--gps-input-inject", cmd)
        self.assertIn("--gps-input-send-yaw", cmd)

    def test_precondition_trajectory_never_produces_actuator_fields(self):
        # the CSV row schema has no actuator/servo columns at all -- only
        # navigation state (GPS_INPUT-relevant fields).
        traj = nav.generate_precondition_trajectory(dt_s=1.0, max_duration_s=2.0)
        row = nav.trajectory_row_to_csv_row(traj[0])
        self.assertEqual(len(row), len(nav.PROFILE_HEADER))


class TestBuildStartupReport(unittest.TestCase):
    """HIL-F23-F2B3C item 7: pure tests for the startup-report milestones."""

    def _sample(self, t, ahrs2_yaw_deg=None, ekf_flags=None, gps_fix_type=None, commanded_yaw_deg=315.091444):
        return {
            "t": t, "ahrs2_yaw_deg": ahrs2_yaw_deg, "ekf_flags": ekf_flags,
            "gps_fix_type": gps_fix_type, "commanded_yaw_deg": commanded_yaw_deg,
        }

    def test_all_milestones_none_when_no_samples(self):
        report = nav.build_startup_report([])
        for value in report.values():
            self.assertIsNone(value)

    def test_first_gps_raw_int_found(self):
        samples = [self._sample(0.0, gps_fix_type=None), self._sample(1.0, gps_fix_type=3)]
        report = nav.build_startup_report(samples)
        self.assertEqual(report["first_gps_raw_int"]["t"], 1.0)

    def test_first_ahrs2_found(self):
        samples = [self._sample(0.0, ahrs2_yaw_deg=None), self._sample(1.0, ahrs2_yaw_deg=3.5)]
        report = nav.build_startup_report(samples)
        self.assertEqual(report["first_ahrs2"]["t"], 1.0)

    def test_first_pos_horiz_valid_requires_the_right_bits(self):
        samples = [
            self._sample(0.0, ekf_flags=167),   # CONST_POS_MODE, no POS_HORIZ bits
            self._sample(1.0, ekf_flags=25),    # 1+8+16 -- POS_HORIZ_REL and ABS set
        ]
        report = nav.build_startup_report(samples)
        self.assertEqual(report["first_pos_horiz_valid"]["t"], 1.0)

    def test_first_yaw_converged_is_single_sample_not_streak(self):
        # unlike the PASS gate, build_startup_report's "first convergence"
        # milestone fires on the FIRST good sample, no 3-consecutive
        # requirement -- useful diagnostic signal even if it later reverts.
        samples = [
            self._sample(0.0, ahrs2_yaw_deg=100.0),
            self._sample(1.0, ahrs2_yaw_deg=315.09),  # converged once...
            self._sample(2.0, ahrs2_yaw_deg=100.0),   # ...then reverted
        ]
        report = nav.build_startup_report(samples)
        self.assertEqual(report["first_yaw_converged"]["t"], 1.0)

    def test_matches_observed_hardware_pattern_167_then_33599(self):
        # HIL-F23-F2B3C: the exact reported flag transition.
        samples = [
            self._sample(0.0, ahrs2_yaw_deg=3.5, ekf_flags=167, gps_fix_type=3),
            self._sample(1.0, ahrs2_yaw_deg=3.5, ekf_flags=167, gps_fix_type=3),
            self._sample(2.0, ahrs2_yaw_deg=315.091444, ekf_flags=33599, gps_fix_type=3),
        ]
        report = nav.build_startup_report(samples)
        self.assertEqual(report["first_gps_raw_int"]["t"], 0.0)
        self.assertEqual(report["first_ahrs2"]["t"], 0.0)
        self.assertEqual(report["first_pos_horiz_valid"]["t"], 2.0)
        self.assertEqual(report["first_yaw_converged"]["t"], 2.0)


class TestPreconditionOnlyMode(unittest.TestCase):
    """HIL-F23-F2B3C item 6: --precondition-only mode."""

    def test_default_duration_is_20s(self):
        self.assertEqual(nav.PRECONDITION_ONLY_DEFAULT_DURATION_S, 20.0)

    def test_cli_flag_and_default_duration_parse(self):
        args = nav.build_arg_parser().parse_args(["--precondition-only"])
        self.assertTrue(args.precondition_only)
        self.assertEqual(args.precondition_only_duration_s, 20.0)

    def test_cli_duration_override_parses(self):
        args = nav.build_arg_parser().parse_args(["--precondition-only", "--precondition-only-duration-s", "8"])
        self.assertEqual(args.precondition_only_duration_s, 8.0)

    def test_runs_for_full_duration_even_if_gate_passes_immediately(self):
        # unlike run_precondition_and_profile(), an early gate pass must
        # NOT cut the observation window short -- this mode exists to
        # watch the EKF for the whole requested window.
        with tempfile.TemporaryDirectory() as tmpdir:
            profile_csv = Path(tmpdir) / "profile.csv"
            bridge_log_csv = Path(tmpdir) / "bridge_log.csv"
            fieldnames = ["time_s", "ahrs2_yaw_rad", "ekf_flags", "gps_raw_fix_type", "gps_tx_yaw_cdeg"]
            with open(bridge_log_csv, "w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=fieldnames)
                w.writeheader()
                for t in range(4):
                    w.writerow({
                        "time_s": str(t), "ahrs2_yaw_rad": str(math.radians(315.091444)),
                        "ekf_flags": "25", "gps_raw_fix_type": "3", "gps_tx_yaw_cdeg": "31509",
                    })
            start = time.time()
            result = nav.run_precondition_only(
                profile_csv, bridge_log_csv, duration_s=1.0, trajectory_dt_s=0.2, gate_poll_interval_s=0.2,
            )
            elapsed = time.time() - start
            self.assertTrue(result["gate_passed"])
            self.assertIsNotNone(result["first_gate_pass_elapsed_s"])
            self.assertGreaterEqual(elapsed, 0.7)  # ran (close to) the full window, not cut short

    def test_never_writes_a_profile_row(self):
        # no A-F profile concept applies to this mode at all -- every
        # written row is a precondition row (re-anchored to count up from
        # 0, see run_precondition_only()'s docstring), and there is no
        # separate "A-F trajectory" segment appended afterward.
        with tempfile.TemporaryDirectory() as tmpdir:
            profile_csv = Path(tmpdir) / "profile.csv"
            bridge_log_csv = Path(tmpdir) / "never_created.csv"
            result = nav.run_precondition_only(profile_csv, bridge_log_csv, duration_s=0.5, trajectory_dt_s=0.2)
            self.assertFalse(result["gate_passed"])
            with open(profile_csv, newline="") as f:
                rows = list(csv.reader(f))
            self.assertGreater(len(rows) - 1, 0)

    def test_alive_check_aborts_immediately(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            profile_csv = Path(tmpdir) / "profile.csv"
            bridge_log_csv = Path(tmpdir) / "never_created.csv"
            start = time.time()
            result = nav.run_precondition_only(
                profile_csv, bridge_log_csv, duration_s=20.0, trajectory_dt_s=0.1,
                alive_check=lambda: "bridge exited unexpectedly rc=1",
            )
            elapsed = time.time() - start
            self.assertFalse(result["gate_passed"])
            self.assertIn("aborted", result["gate_reason"])
            self.assertLess(elapsed, 1.0)

    def test_no_actuator_attitude_airspeed_in_precondition_only_mode(self):
        # same bridge command builder as the combined run -- precondition-
        # only mode does not introduce any new bridge flags.
        cmd = nav.build_bridge_cmd_for_nav_profile(
            "/dev/ttyACM0", 115200, "/tmp/profile.csv", "udp:192.168.64.1:14550", "/tmp/log.csv",
        )
        for forbidden in ("--allow-actuator-output", "--attitude-inject", "--airspeed-inject"):
            self.assertNotIn(forbidden, cmd)


class TestPreflightEkfFlagsSnapshot(unittest.TestCase):
    """HIL-F23-F2B3C item 7: preflight_checks_combined() must capture an
    EKF_STATUS_REPORT.flags snapshot BEFORE any GPS_INPUT is sent.
    """

    def test_evaluate_combined_preflight_unaffected_by_new_details_key(self):
        # the new ekf_flags_before_gps_input detail must not perturb the
        # existing pass/fail param-check logic.
        params = {"GPS1_TYPE": 14, "EK3_SRC1_POSXY": 3, "EK3_SRC1_VELXY": 3,
                  "EK3_SRC1_VELZ": 3, "EK3_SRC1_POSZ": 3, "EK3_SRC1_YAW": 2}
        ok, failures = nav.evaluate_combined_preflight(False, "MANUAL", params)
        self.assertTrue(ok)
        self.assertEqual(failures, [])


if __name__ == "__main__":
    unittest.main()
