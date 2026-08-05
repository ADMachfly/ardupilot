#!/usr/bin/env python3
"""HIL-F24-G unit tests for sr75_hil_f24g_jsbsim_pipeline_validation.py's
pure conversion/coherence-check functions.

No JSBSim binary is invoked by these tests (run_jsbsim() -- the one thin
I/O wrapper that spawns JSBSim -- is intentionally not exercised here).
Instead, synthetic raw-row dicts are built matching the EXACT column
schema the real SR75_hil_f24g_pipeline_validation.xml JSBSim script
produces, and fed through the REAL convert_rows()/StateMapper.state_from_
csv() conversion path -- so these tests still exercise the real
conversion code, just without needing JSBSim installed to run them fast.
"""
import copy
import math
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "sim_json"))
import sr75_hil_f24g_jsbsim_pipeline_validation as f24g  # noqa: E402
import sr75_sim_json_responder as responder  # noqa: E402

FT_TO_M = 0.3048


def make_raw_row(
    t_s, lat_deg=32.5378085, lon_deg=74.3661944, alt_ft=9842.6, alt_agl_ft=9842.6,
    phi_deg=0.0, theta_deg=-2.0, psi_deg=315.0, p_rad_s=0.0, q_rad_s=0.0, r_rad_s=0.0,
    vn_fps=0.0, ve_fps=0.0, vd_fps=0.0, vt_fps=226.0, vc_kts=134.0,
    elevator=-0.42, aileron=0.0, rudder=0.0, throttle=0.55, rato=0.0,
    rato_thrust_lbs=0.0,
):
    roll_rad = math.radians(phi_deg)
    pitch_rad = math.radians(theta_deg)
    yaw_rad = math.radians(psi_deg)
    ax, ay, az = responder.gravity_body_mss(roll_rad, pitch_rad, yaw_rad)
    q1, q2, q3, q4 = responder.euler_to_quaternion(roll_rad, pitch_rad, yaw_rad)
    return {
        "Time": f"{t_s:.6f}",
        "/fdm/jsbsim/simulation/sim-time-sec": f"{t_s:.6f}",
        "/fdm/jsbsim/position/lat-gc-deg": f"{lat_deg:.7f}",
        "/fdm/jsbsim/position/long-gc-deg": f"{lon_deg:.7f}",
        "/fdm/jsbsim/position/h-sl-ft": f"{alt_ft:.2f}",
        "/fdm/jsbsim/position/h-agl-ft": f"{alt_agl_ft:.2f}",
        "/fdm/jsbsim/velocities/v-north-fps": f"{vn_fps:.4f}",
        "/fdm/jsbsim/velocities/v-east-fps": f"{ve_fps:.4f}",
        "/fdm/jsbsim/velocities/v-down-fps": f"{vd_fps:.4f}",
        "/fdm/jsbsim/velocities/vc-kts": f"{vc_kts:.4f}",
        "/fdm/jsbsim/velocities/vt-fps": f"{vt_fps:.4f}",
        "/fdm/jsbsim/attitude/phi-deg": f"{phi_deg:.6f}",
        "/fdm/jsbsim/attitude/theta-rad": f"{pitch_rad:.9f}",
        "/fdm/jsbsim/attitude/theta-deg": f"{theta_deg:.6f}",
        "/fdm/jsbsim/attitude/psi-deg": f"{psi_deg:.6f}",
        "/fdm/jsbsim/aero/alpha-rad": "0.0",
        "/fdm/jsbsim/aero/beta-rad": "0.0",
        "/fdm/jsbsim/fcs/elevator-cmd-norm": f"{elevator:.4f}",
        "/fdm/jsbsim/fcs/aileron-cmd-norm": f"{aileron:.4f}",
        "/fdm/jsbsim/fcs/rudder-cmd-norm": f"{rudder:.4f}",
        "/fdm/jsbsim/fcs/turbojet-throttle-cmd-norm": f"{throttle:.4f}",
        "/fdm/jsbsim/fcs/rato-throttle-cmd-norm": f"{rato:.4f}",
        "/fdm/jsbsim/fcs/left-elevon-pos-rad": "0.0",
        "/fdm/jsbsim/fcs/right-elevon-pos-rad": "0.0",
        "/fdm/jsbsim/propulsion/engine/thrust-lbs": "500.0",
        "/fdm/jsbsim/propulsion/engine[1]/thrust-lbs": "500.0",
        "/fdm/jsbsim/propulsion/engine[2]/thrust-lbs": f"{rato_thrust_lbs:.2f}",
        "/fdm/jsbsim/propulsion/engine/set-running": "1",
        "/fdm/jsbsim/propulsion/engine[2]/set-running": "0",
        "p_rad_s": f"{p_rad_s:.9f}",
        "q_rad_s": f"{q_rad_s:.9f}",
        "r_rad_s": f"{r_rad_s:.9f}",
        "accel_body_x_mss": f"{ax:.9f}",
        "accel_body_y_mss": f"{ay:.9f}",
        "accel_body_z_mss": f"{az:.9f}",
        "q1": f"{q1:.9f}",
        "q2": f"{q2:.9f}",
        "q3": f"{q3:.9f}",
        "q4": f"{q4:.9f}",
    }


def make_series(n=20, dt=0.02, **kwargs):
    return [make_raw_row(i * dt, **kwargs) for i in range(n)]


class TestConvertRows(unittest.TestCase):
    def test_converts_via_real_state_mapper(self):
        rows = make_series(5)
        states = f24g.convert_rows(rows)
        self.assertEqual(len(states), 5)
        self.assertIsInstance(states[0], responder.SimState)

    def test_pitch_taken_directly_in_radians(self):
        rows = make_series(1, theta_deg=10.0)
        states = f24g.convert_rows(rows)
        self.assertAlmostEqual(states[0].pitch_rad, math.radians(10.0), places=9)


class TestCoherenceChecksPass(unittest.TestCase):
    def setUp(self):
        # A gentle, smoothly varying series (mimics the real capture)
        # small enough to satisfy every check without any injected fault.
        self.rows = [
            make_raw_row(
                i * 0.02, phi_deg=i * 0.01, theta_deg=-2.0 + i * 0.01, psi_deg=315.0 + i * 0.005,
                p_rad_s=0.01, q_rad_s=0.01, r_rad_s=0.005,
            )
            for i in range(50)
        ]
        self.states = f24g.convert_rows(self.rows)

    def test_monotonic_timestamps(self):
        f24g.check_monotonic_timestamps(self.states)

    def test_quaternion_euler_consistency(self):
        f24g.check_quaternion_euler_consistency(self.states)

    def test_gyro_passthrough_zero_error(self):
        err = f24g.check_gyro_passthrough(self.states, self.rows)
        self.assertAlmostEqual(err, 0.0, places=9)

    def test_gravity_consistent_accel_zero_error(self):
        err = f24g.check_gravity_consistent_accel(self.states, self.rows)
        self.assertAlmostEqual(err, 0.0, places=9)

    def test_ned_velocity_conversion_zero_error(self):
        err = f24g.check_ned_velocity_conversion(self.states, self.rows)
        self.assertLess(err, 1e-6)

    def test_airspeed_conversion_uses_true_airspeed(self):
        err = f24g.check_airspeed_conversion(self.states, self.rows)
        self.assertLess(err, 1e-6)
        expected = float(self.rows[0]["/fdm/jsbsim/velocities/vt-fps"]) * FT_TO_M
        self.assertAlmostEqual(self.states[0].airspeed_mps, expected, places=6)

    def test_altitude_conversion_uses_asl_not_agl(self):
        err = f24g.check_altitude_conversion(self.states, self.rows)
        self.assertLess(err, 1e-6)
        row = make_raw_row(0.0, alt_ft=9842.6, alt_agl_ft=500.0)
        state = f24g.convert_rows([row])[0]
        self.assertAlmostEqual(state.altitude_m, 9842.6 * FT_TO_M, places=4)
        self.assertNotAlmostEqual(state.altitude_m, 500.0 * FT_TO_M, places=1)

    def test_position_passthrough(self):
        err = f24g.check_position_passthrough(self.states, self.rows)
        self.assertAlmostEqual(err, 0.0, places=9)

    def test_attitude_conversion(self):
        roll_err, pitch_err, yaw_err = f24g.check_attitude_conversion(self.states, self.rows)
        self.assertLess(roll_err, 1e-6)
        self.assertLess(pitch_err, 1e-6)
        self.assertLess(yaw_err, 1e-6)


class TestCoherenceChecksDetectFaults(unittest.TestCase):
    """Mirrors HIL-F24-F's philosophy: each check must actually detect a
    deliberately broken row, not just tautologically pass real data."""

    def setUp(self):
        self.rows = make_series(
            20, phi_deg=1.0, theta_deg=1.0, psi_deg=320.0, p_rad_s=0.01, q_rad_s=0.01, r_rad_s=0.01,
        )
        self.states = f24g.convert_rows(self.rows)

    def test_detects_non_monotonic_timestamp(self):
        broken = copy.deepcopy(self.states)
        broken[5].timestamp_s = broken[4].timestamp_s - 0.01
        with self.assertRaises(f24g.PipelineValidationError):
            f24g.check_monotonic_timestamps(broken)

    def test_detects_corrupted_quaternion(self):
        broken = copy.deepcopy(self.states)
        w, x, y, z = broken[5].quaternion
        broken[5].quaternion = (w, x + 0.5, y, z)
        with self.assertRaises(f24g.PipelineValidationError):
            f24g.check_quaternion_euler_consistency(broken)

    def test_detects_gyro_mismatch(self):
        broken = copy.deepcopy(self.states)
        p, q, r = broken[5].gyro_rad_s
        broken[5].gyro_rad_s = (p + 1.0, q, r)
        with self.assertRaises(f24g.PipelineValidationError):
            f24g.check_gyro_passthrough(broken, self.rows)

    def test_detects_accel_mismatch(self):
        broken = copy.deepcopy(self.states)
        ax, ay, az = broken[5].accel_body_mss
        broken[5].accel_body_mss = (ax + 5.0, ay, az)
        with self.assertRaises(f24g.PipelineValidationError):
            f24g.check_gravity_consistent_accel(broken, self.rows)

    def test_detects_ned_velocity_mismatch(self):
        broken = copy.deepcopy(self.states)
        vn, ve, vd = broken[5].velocity_ned_mps
        broken[5].velocity_ned_mps = (vn + 10.0, ve, vd)
        with self.assertRaises(f24g.PipelineValidationError):
            f24g.check_ned_velocity_conversion(broken, self.rows)

    def test_detects_airspeed_mismatch(self):
        broken = copy.deepcopy(self.states)
        broken[5].airspeed_mps += 20.0
        with self.assertRaises(f24g.PipelineValidationError):
            f24g.check_airspeed_conversion(broken, self.rows)

    def test_detects_altitude_mismatch(self):
        broken = copy.deepcopy(self.states)
        broken[5].altitude_m += 100.0
        with self.assertRaises(f24g.PipelineValidationError):
            f24g.check_altitude_conversion(broken, self.rows)

    def test_detects_position_mismatch(self):
        broken = copy.deepcopy(self.states)
        broken[5].latitude_deg += 0.01
        with self.assertRaises(f24g.PipelineValidationError):
            f24g.check_position_passthrough(broken, self.rows)

    def test_detects_attitude_mismatch(self):
        broken = copy.deepcopy(self.states)
        broken[5].roll_rad += math.radians(5.0)
        with self.assertRaises(f24g.PipelineValidationError):
            f24g.check_attitude_conversion(broken, self.rows)


class TestFrameSignAudit(unittest.TestCase):
    def test_passes_on_physically_consistent_series(self):
        rows = []
        for i in range(1100):
            t = i * 0.02
            if t < 16.0:
                phi, theta, psi = 0.0, -2.0, 315.0
            else:
                phi = min(12.0, (t - 16.0) * 3.0)
                theta = -2.0 + min(10.0, (t - 16.0) * 2.0)
                psi = 315.0 + min(6.0, (t - 16.0) * 1.5)
            rows.append(make_raw_row(
                t, phi_deg=phi, theta_deg=theta, psi_deg=psi, p_rad_s=0.05, q_rad_s=0.03, r_rad_s=0.02,
            ))
        audit = f24g.check_no_frame_or_sign_inversion(rows)
        self.assertIn("negative", audit["level_flight_accel_body_z_mss"])

    def test_detects_inverted_roll_rate_sign(self):
        rows = []
        for i in range(1100):
            t = i * 0.02
            phi = 0.0 if t < 16.0 else min(12.0, (t - 16.0) * 3.0)
            # p_rad_s deliberately inverted relative to actual d(phi)/dt
            rows.append(make_raw_row(t, phi_deg=phi, p_rad_s=-0.05, q_rad_s=0.0, r_rad_s=0.0))
        with self.assertRaises(AssertionError):
            f24g.check_no_frame_or_sign_inversion(rows)


class TestPhaseTransitionContinuity(unittest.TestCase):
    def test_smooth_series_has_no_discontinuity(self):
        rows = make_series(4000, phi_deg=0.0, theta_deg=-2.0)
        for i, row in enumerate(rows):
            row["/fdm/jsbsim/attitude/phi-deg"] = f"{(i * 0.02) * 0.5:.6f}"
        states = f24g.convert_rows(rows)
        f24g.check_phase_transition_continuity(states)

    def test_detects_artificial_jump_at_boundary(self):
        rows = make_series(4000)
        states = f24g.convert_rows(rows)
        # Simulate a spliced/discontinuous pipeline artifact right at a
        # known phase-boundary timestamp (16.0 s -> index 800).
        idx = next(i for i, s in enumerate(states) if abs(s.timestamp_s - 16.0) < 0.02)
        states[idx].roll_rad += math.radians(45.0)
        with self.assertRaises(f24g.PipelineValidationError):
            f24g.check_phase_transition_continuity(states)


class TestRegressionCaptures(unittest.TestCase):
    def test_save_and_reload_round_trips(self):
        rows = make_series(200)
        states = f24g.convert_rows(rows)
        with tempfile.TemporaryDirectory() as tmpdir:
            out_dir = Path(tmpdir)
            paths = f24g.save_regression_captures(states, rows, out_dir)
            self.assertEqual(set(paths), set(f24g.PHASE_WINDOWS))
            for path in paths.values():
                self.assertTrue(path.exists())
            static_capture = f24g.load_regression_capture(paths["static_release"])
            self.assertGreater(static_capture["n_sampled"], 0)

    def test_compare_regression_capture_matches_fresh_conversion(self):
        rows = make_series(200)
        states = f24g.convert_rows(rows)
        with tempfile.TemporaryDirectory() as tmpdir:
            out_dir = Path(tmpdir)
            paths = f24g.save_regression_captures(states, rows, out_dir)
            for path in paths.values():
                self.assertTrue(f24g.compare_regression_capture(path, states))

    def test_compare_detects_drift(self):
        rows = make_series(200)
        states = f24g.convert_rows(rows)
        with tempfile.TemporaryDirectory() as tmpdir:
            out_dir = Path(tmpdir)
            paths = f24g.save_regression_captures(states, rows, out_dir)
            drifted = copy.deepcopy(states)
            for s in drifted:
                s.roll_rad += math.radians(1.0)
            self.assertFalse(f24g.compare_regression_capture(paths["static_release"], drifted))


class TestF24FAcceptanceIntegration(unittest.TestCase):
    def test_self_consistent_records_pass(self):
        rows = make_series(200, phi_deg=1.0, theta_deg=-1.0, psi_deg=316.0)
        states = f24g.convert_rows(rows)
        ok, report = f24g.run_f24f_acceptance_check(states)
        self.assertTrue(ok, report["violations"])

    def test_records_shape_matches_csv_fields(self):
        import sr75_hil_f24f_hardware_acceptance_analyzer as analyzer

        rows = make_series(10)
        states = f24g.convert_rows(rows)
        records = f24g.build_f24f_records(states)
        for field in analyzer.CSV_FIELDS:
            self.assertIn(field, records[0])


class TestPipelineHealth(unittest.TestCase):
    def test_health_stats_on_clean_series(self):
        rows = make_series(500)
        states = f24g.convert_rows(rows)
        health = f24g.compute_pipeline_health(states, rows)
        self.assertEqual(health["n_rows"], 500)
        self.assertAlmostEqual(health["packet_rate_hz"], 50.0, delta=0.5)
        self.assertAlmostEqual(health["max_stale_gap_s"], 0.02, delta=1e-6)
        self.assertEqual(health["malformed_packet_count"], 0)
        self.assertEqual(health["missing_required_field_count"], 0)


if __name__ == "__main__":
    unittest.main()
