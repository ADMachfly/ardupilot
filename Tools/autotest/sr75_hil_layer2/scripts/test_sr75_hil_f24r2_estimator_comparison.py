#!/usr/bin/env python3
"""HIL-F24-R2 regression tests: estimator-comparison math and pipeline.

Covers: horizontal-distance/yaw-wrap helpers, nearest-neighbour host-
monotonic alignment (never wall-clock), the full align_and_compare() +
compute_summary() pipeline against synthetic truth/Pixhawk data (including
a deliberately injected discontinuity and EKF-unhealthy event), and an
end-to-end run of main() against real files on disk. No hardware, no
MAVLink connection, no JSBSim process.
"""
import csv
import json
import math
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
COMPARISON_SCRIPT = HERE / "sr75_hil_f24r2_estimator_comparison.py"

import sr75_hil_f24r2_estimator_comparison as cmp  # noqa: E402

TRUTH_FIELDS = [
    "host_monotonic_time", "time_s", "lat_deg", "lon_deg", "alt_m",
    "roll_rad", "pitch_rad", "yaw_rad", "vn_mps", "ve_mps", "vd_mps",
    "p_rad_s", "q_rad_s", "r_rad_s",
    "accel_body_x_mss", "accel_body_y_mss", "accel_body_z_mss", "airspeed_mps",
]
PIXHAWK_FIELDS = [
    "host_monotonic_time", "host_wall_time", "msg_type",
    "hb_armed", "hb_mode", "hb_mode_is_manual", "hb_system_status",
    "gpi_lat_deg", "gpi_lon_deg", "gpi_alt_m", "gpi_relative_alt_m",
    "gpi_vn_mps", "gpi_ve_mps", "gpi_vd_mps", "gpi_hdg_deg",
    "lpn_x_m", "lpn_y_m", "lpn_z_m", "lpn_vx_mps", "lpn_vy_mps", "lpn_vz_mps",
    "att_roll_deg", "att_pitch_deg", "att_yaw_deg", "att_p_deg_s", "att_q_deg_s", "att_r_deg_s",
    "gri_fix_type", "gri_lat_deg", "gri_lon_deg", "gri_alt_m", "gri_satellites_visible",
    "vfr_airspeed_mps", "vfr_groundspeed_mps", "vfr_heading_deg", "vfr_alt_m", "vfr_climb_mps",
    "ekf_flags", "ekf_velocity_variance", "ekf_pos_horiz_variance",
    "ekf_pos_vert_variance", "ekf_compass_variance",
    "sys_onboard_control_sensors_health", "sys_battery_remaining",
    "statustext_severity", "statustext_text",
]

BASE_LAT, BASE_LON, BASE_ALT = 32.5378085, 74.3661944, 0.170451
BASE_YAW_RAD = 5.4977871  # ~315 deg


def truth_row(t, **overrides):
    row = {f: "" for f in TRUTH_FIELDS}
    row.update({
        "host_monotonic_time": t, "time_s": t, "lat_deg": BASE_LAT, "lon_deg": BASE_LON, "alt_m": BASE_ALT,
        "roll_rad": 0.0, "pitch_rad": -0.0316, "yaw_rad": BASE_YAW_RAD,
        "vn_mps": 0.0, "ve_mps": 0.0, "vd_mps": 0.0, "p_rad_s": 0.0, "q_rad_s": 0.0, "r_rad_s": 0.0,
        "accel_body_x_mss": -0.309, "accel_body_y_mss": 0.0, "accel_body_z_mss": -9.79, "airspeed_mps": 0.05,
    })
    row.update(overrides)
    return row


def gpi_row(t, **overrides):
    row = {f: "" for f in PIXHAWK_FIELDS}
    row.update({
        "host_monotonic_time": t, "msg_type": "GLOBAL_POSITION_INT",
        "gpi_lat_deg": BASE_LAT, "gpi_lon_deg": BASE_LON, "gpi_alt_m": BASE_ALT,
        "gpi_vn_mps": 0.0, "gpi_ve_mps": 0.0, "gpi_vd_mps": 0.0,
    })
    row.update(overrides)
    return row


def att_row(t, **overrides):
    row = {f: "" for f in PIXHAWK_FIELDS}
    row.update({
        "host_monotonic_time": t, "msg_type": "ATTITUDE",
        "att_roll_deg": 0.0, "att_pitch_deg": math.degrees(-0.0316), "att_yaw_deg": math.degrees(BASE_YAW_RAD),
        "att_p_deg_s": 0.0, "att_q_deg_s": 0.0, "att_r_deg_s": 0.0,
    })
    row.update(overrides)
    return row


def vfr_row(t, **overrides):
    row = {f: "" for f in PIXHAWK_FIELDS}
    row.update({"host_monotonic_time": t, "msg_type": "VFR_HUD", "vfr_airspeed_mps": 0.05})
    row.update(overrides)
    return row


def ekf_row(t, flags, **overrides):
    row = {f: "" for f in PIXHAWK_FIELDS}
    row.update({"host_monotonic_time": t, "msg_type": "EKF_STATUS_REPORT", "ekf_flags": float(flags)})
    row.update(overrides)
    return row


def statustext_row(t, text, **overrides):
    row = {f: "" for f in PIXHAWK_FIELDS}
    row.update({"host_monotonic_time": t, "msg_type": "STATUSTEXT", "statustext_text": text})
    row.update(overrides)
    return row


def write_csv(path, fields, rows):
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


class TestHorizontalDistance(unittest.TestCase):
    def test_zero_distance_for_identical_points(self):
        self.assertAlmostEqual(cmp.horizontal_distance_m(32.5, 74.3, 32.5, 74.3), 0.0, places=6)

    def test_one_degree_latitude_is_about_111km(self):
        d = cmp.horizontal_distance_m(0.0, 0.0, 1.0, 0.0)
        self.assertAlmostEqual(d, 111195.0, delta=200.0)

    def test_small_offset_matches_local_flat_earth_approximation(self):
        # ~1.11m per 1e-5 deg latitude at the equator-ish scale used here.
        d = cmp.horizontal_distance_m(32.5378085, 74.3661944, 32.5378095, 74.3661944)
        self.assertAlmostEqual(d, 0.111, delta=0.02)


class TestYawErrorWrap(unittest.TestCase):
    def test_no_wrap_needed(self):
        self.assertAlmostEqual(cmp.yaw_error_deg(10.0, 5.0), 5.0, places=6)

    def test_wraps_at_positive_180(self):
        self.assertAlmostEqual(cmp.yaw_error_deg(179.0, -179.0), -2.0, places=6)

    def test_wraps_at_negative_180(self):
        self.assertAlmostEqual(cmp.yaw_error_deg(-179.0, 179.0), 2.0, places=6)

    def test_wraps_across_360_style_values(self):
        self.assertAlmostEqual(cmp.yaw_error_deg(1.0, 359.0), 2.0, places=6)


class TestNearestWithinTolerance(unittest.TestCase):
    def test_finds_closest_within_tolerance(self):
        rows = [{"host_monotonic_time": t} for t in (0.0, 1.0, 2.0, 3.0)]
        match = cmp.nearest_within_tolerance(2.05, rows, tolerance_s=0.2)
        self.assertEqual(match["host_monotonic_time"], 2.0)

    def test_returns_none_outside_tolerance(self):
        rows = [{"host_monotonic_time": t} for t in (0.0, 5.0)]
        self.assertIsNone(cmp.nearest_within_tolerance(2.5, rows, tolerance_s=0.1))

    def test_empty_list_returns_none(self):
        self.assertIsNone(cmp.nearest_within_tolerance(1.0, [], tolerance_s=1.0))


class TestAlignAndCompare(unittest.TestCase):
    def test_zero_error_when_pixhawk_matches_truth_exactly(self):
        tmpdir = Path(tempfile.mkdtemp(prefix="hil_f24r2_align_"))
        truth_csv = tmpdir / "truth.csv"
        pixhawk_csv = tmpdir / "pixhawk.csv"
        write_csv(truth_csv, TRUTH_FIELDS, [truth_row(t * 0.02) for t in range(500)])
        rows = []
        for i in range(50):
            t = i * 0.2
            rows.append(gpi_row(t))
            rows.append(att_row(t + 0.005))
            rows.append(vfr_row(t + 0.01))
        write_csv(pixhawk_csv, PIXHAWK_FIELDS, rows)

        truth_rows = cmp.load_truth_csv(truth_csv)
        pixhawk_by_type = cmp.load_pixhawk_csv(pixhawk_csv)
        comparison_rows, coverage = cmp.align_and_compare(truth_rows, pixhawk_by_type, alignment_tolerance_s=0.1)

        self.assertGreater(coverage, 0.99)
        for row in comparison_rows:
            self.assertAlmostEqual(row["horizontal_error_m"], 0.0, delta=1e-6)
            self.assertAlmostEqual(row["altitude_error_m"], 0.0, delta=1e-6)
            self.assertAlmostEqual(row["roll_error_deg"], 0.0, delta=1e-6)
            self.assertAlmostEqual(row["yaw_error_deg"], 0.0, delta=1e-6)
            self.assertAlmostEqual(row["airspeed_error_mps"], 0.0, delta=1e-6)

    def test_known_offsets_reported_correctly(self):
        tmpdir = Path(tempfile.mkdtemp(prefix="hil_f24r2_offset_"))
        truth_csv = tmpdir / "truth.csv"
        pixhawk_csv = tmpdir / "pixhawk.csv"
        write_csv(truth_csv, TRUTH_FIELDS, [truth_row(t * 0.02) for t in range(500)])
        rows = [
            gpi_row(1.0, gpi_alt_m=BASE_ALT + 2.5, gpi_vn_mps=0.3),
            att_row(1.005, att_yaw_deg=math.degrees(BASE_YAW_RAD) + 3.0),
        ]
        write_csv(pixhawk_csv, PIXHAWK_FIELDS, rows)

        truth_rows = cmp.load_truth_csv(truth_csv)
        pixhawk_by_type = cmp.load_pixhawk_csv(pixhawk_csv)
        comparison_rows, coverage = cmp.align_and_compare(truth_rows, pixhawk_by_type, alignment_tolerance_s=0.1)

        self.assertEqual(len(comparison_rows), 1)
        self.assertAlmostEqual(comparison_rows[0]["altitude_error_m"], 2.5, places=3)
        self.assertAlmostEqual(comparison_rows[0]["vn_error_mps"], 0.3, places=3)
        self.assertAlmostEqual(comparison_rows[0]["yaw_error_deg"], 3.0, places=3)

    def test_unaligned_pixhawk_sample_counted_but_no_error(self):
        tmpdir = Path(tempfile.mkdtemp(prefix="hil_f24r2_unaligned_"))
        truth_csv = tmpdir / "truth.csv"
        pixhawk_csv = tmpdir / "pixhawk.csv"
        write_csv(truth_csv, TRUTH_FIELDS, [truth_row(t) for t in (0.0, 0.02)])
        write_csv(pixhawk_csv, PIXHAWK_FIELDS, [gpi_row(50.0)])  # far outside tolerance

        truth_rows = cmp.load_truth_csv(truth_csv)
        pixhawk_by_type = cmp.load_pixhawk_csv(pixhawk_csv)
        comparison_rows, coverage = cmp.align_and_compare(truth_rows, pixhawk_by_type, alignment_tolerance_s=0.1)

        self.assertEqual(coverage, 0.0)
        self.assertEqual(len(comparison_rows), 1)
        self.assertIsNone(comparison_rows[0]["horizontal_error_m"])

    def test_real_bench_ahrs_backend_switch_produces_large_finite_error_not_nan_or_clipped(self):
        """HIL-F24-R2A: regression fixture from the real hardware capture
        sr75_hil_f24r2_estimator_20260806T050616Z (t=3273.14) -- confirmed
        (see the diagnosis report) to be a real Pixhawk AHRS backend
        transition (DCM's own stale dead-reckoned position, reported for
        ~1.5s while re-converging), not a bug in this comparison script.
        This locks in that align_and_compare()/horizontal_distance_m()
        report such a real, large divergence as a large *finite* number --
        never NaN/Inf, never silently clipped/wrapped by any datum or
        scaling logic -- so a genuine large excursion is never hidden."""
        tmpdir = Path(tempfile.mkdtemp(prefix="hil_f24r2_real_divergence_"))
        truth_csv = tmpdir / "truth.csv"
        pixhawk_csv = tmpdir / "pixhawk.csv"
        # Truth: stationary bench position, exactly as fed by SIM_JSON.
        write_csv(truth_csv, TRUTH_FIELDS, [truth_row(t * 0.02) for t in range(50)])
        # GLOBAL_POSITION_INT as literally captured at t=3273.143953256.
        rows = [gpi_row(0.5, gpi_lat_deg=-1.8481855999999999, gpi_lon_deg=113.47457419999999, gpi_alt_m=0.17)]
        write_csv(pixhawk_csv, PIXHAWK_FIELDS, rows)

        truth_rows = cmp.load_truth_csv(truth_csv)
        pixhawk_by_type = cmp.load_pixhawk_csv(pixhawk_csv)
        comparison_rows, coverage = cmp.align_and_compare(truth_rows, pixhawk_by_type, alignment_tolerance_s=0.1)

        self.assertEqual(len(comparison_rows), 1)
        error_m = comparison_rows[0]["horizontal_error_m"]
        self.assertTrue(math.isfinite(error_m))
        # Matches the real captured incident (~5.6 million metres) within 1%.
        self.assertAlmostEqual(error_m, 5675035.589336136, delta=5675035.589336136 * 0.01)


class TestAttitudeJumpDetection(unittest.TestCase):
    def test_no_jump_for_smooth_attitude(self):
        rows = [att_row(t, att_roll_deg=0.1 * t) for t in range(10)]
        jumps = cmp.detect_attitude_jumps(rows, max_jump_deg=10.0)
        self.assertEqual(jumps, [])

    def test_detects_large_roll_jump(self):
        rows = [att_row(0.0, att_roll_deg=0.0), att_row(0.1, att_roll_deg=25.0)]
        jumps = cmp.detect_attitude_jumps(rows, max_jump_deg=10.0)
        self.assertEqual(len(jumps), 1)

    def test_yaw_wrap_not_mistaken_for_jump(self):
        rows = [att_row(0.0, att_yaw_deg=179.0), att_row(0.1, att_yaw_deg=-179.0)]  # true delta = 2 deg
        jumps = cmp.detect_attitude_jumps(rows, max_jump_deg=10.0)
        self.assertEqual(jumps, [])

    def test_real_bench_ahrs_backend_switch_spike_flagged_as_in_and_out_jumps(self):
        """HIL-F24-R2A: regression fixture from the real hardware capture
        sr75_hil_f24r2_estimator_20260806T050616Z (t=3293.10-3293.40). A
        single ATTITUDE sample momentarily reports the DCM backend's
        estimate (confirmed by the bracketing "AHRS: DCM active"/"AHRS:
        EKF3 active" STATUSTEXT pair) before the next sample reverts to
        the EKF3 baseline -- one real discontinuity necessarily produces
        TWO flagged transitions (in, then out); this is correct/expected
        detector behavior, not a double-counting bug, and this test locks
        that down using the exact real captured values."""
        rows = [
            att_row(3293.095585, att_roll_deg=-0.12734176674124006, att_pitch_deg=-2.0421151443729766,
                    att_yaw_deg=-49.46203747850877),
            att_row(3293.196462, att_roll_deg=0.10118915917224634, att_pitch_deg=-1.7296699550597416,
                    att_yaw_deg=-34.38016016842021),  # single-sample DCM-backend spike
            att_row(3293.296995, att_roll_deg=-0.12365153025851588, att_pitch_deg=-2.0366702029440074,
                    att_yaw_deg=-49.45601325166082),  # reverts to EKF3 baseline on the very next sample
        ]
        jumps = cmp.detect_attitude_jumps(rows, max_jump_deg=10.0)
        self.assertEqual(len(jumps), 2)
        self.assertAlmostEqual(jumps[0]["host_monotonic_time"], 3293.196462, places=6)
        self.assertAlmostEqual(jumps[1]["host_monotonic_time"], 3293.296995, places=6)
        # in-jump and out-jump are equal and opposite (a spike-and-revert), not two independent events
        # (small residual is the real baseline's own slight drift between the bracketing samples)
        self.assertAlmostEqual(jumps[0]["yaw_jump_deg"], -jumps[1]["yaw_jump_deg"], delta=0.1)
        self.assertGreater(abs(jumps[0]["yaw_jump_deg"]), 10.0)


class TestHealthEventScanning(unittest.TestCase):
    def test_healthy_flags_no_event(self):
        rows = {"EKF_STATUS_REPORT": [{"host_monotonic_time": 1.0, "ekf_flags": 31.0}]}
        events, gps_glitch_events = cmp.scan_ekf_and_statustext_health(rows)
        self.assertEqual(events, [])
        self.assertEqual(gps_glitch_events, [])

    def test_unhealthy_flags_detected(self):
        rows = {"EKF_STATUS_REPORT": [{"host_monotonic_time": 1.0, "ekf_flags": float(cmp.EKF_UNINITIALIZED)}]}
        events, gps_glitch_events = cmp.scan_ekf_and_statustext_health(rows)
        self.assertEqual(len(events), 1)
        self.assertEqual(events[0]["kind"], "ekf_unhealthy_flags")
        self.assertEqual(gps_glitch_events, [])

    def test_all_healthy_status_bits_together_not_misclassified_as_fault(self):
        """HIL-F24-R2A: per the MAVLink EKF_STATUS_FLAGS enum, bits 1, 2, 4,
        8, 16, 32, 64, 256, 512 are each documented "Set if ... is good" --
        positive/healthy indicators, not faults. Setting every one of them
        simultaneously (a maximally healthy report, no fault bit set) must
        never be classified as unhealthy -- confirms EKF_UNHEALTHY_FLAGS
        contains none of these bits."""
        all_healthy_bits = 1 | 2 | 4 | 8 | 16 | 32 | 64 | 256 | 512
        self.assertEqual(all_healthy_bits & cmp.EKF_UNHEALTHY_FLAGS, 0)
        rows = {"EKF_STATUS_REPORT": [{"host_monotonic_time": 1.0, "ekf_flags": float(all_healthy_bits)}]}
        events, gps_glitch_events = cmp.scan_ekf_and_statustext_health(rows)
        self.assertEqual(events, [])
        self.assertEqual(gps_glitch_events, [])

    def test_real_bench_glitch_flags_32807_and_33599_tracked_as_advisory_not_unhealthy(self):
        """HIL-F24-R3L (supersedes the original HIL-F24-R2A framing of this
        exact fixture): flags 32807/33599 both come from real hardware
        captures (sr75_hil_f24r2_estimator_20260806T050616Z and
        sr75_hil_f24r3a_mp_visual_20260807T080924Z respectively) and both
        set only one fault-shaped bit, 32768 (EKF_GPS_GLITCHING). R2A
        originally classified this as a hard fault because in that capture
        it coincided with a genuine EKF3<->DCM backend flap, a ~2.5
        million metre position divergence, and 5 attitude jumps -- but the
        R3L capture shows the exact same bit (flags=33599) on an otherwise
        fully healthy run: zero backward-timestamp rejects, GPS fix stays
        3, no attitude jump, and every RMS metric passes. Tracing
        AP_NavEKF3_Outputs.cpp's send_status_report() to AP_NavEKF3_
        Control.cpp's filterStatus.flags.gps_glitching assignment shows it
        is a self-clearing 1s-fail/10s-pass innovation-ratio advisory
        (AP_NavEKF3_VehicleStatus.cpp), not a reset/re-initialisation
        signal -- so it is now tracked separately as an advisory, and does
        NOT by itself make scan_ekf_and_statustext_health's `events`
        (the list no_ekf_or_statustext_reset_events gates on). R2A's own
        scenario remains independently caught: its ~2.5 million metre
        divergence and 5 attitude jumps still fail horizontal_error_rms
        and no_large_attitude_jump regardless of this bit (see
        TestR3aEkfWarmupGate's sibling coverage and HIL_F24_R3L report)."""
        for real_flags in (32807.0, 33599.0):
            with self.subTest(flags=real_flags):
                self.assertTrue(int(real_flags) & cmp.EKF_GPS_GLITCHING)
                self.assertEqual(int(real_flags) & cmp.EKF_UNHEALTHY_FLAGS, 0)
                rows = {"EKF_STATUS_REPORT": [{"host_monotonic_time": 1.0, "ekf_flags": real_flags}]}
                events, gps_glitch_events = cmp.scan_ekf_and_statustext_health(rows)
                self.assertEqual(events, [])
                self.assertEqual(len(gps_glitch_events), 1)
                self.assertEqual(gps_glitch_events[0]["kind"], "ekf_gps_glitching_advisory")
                self.assertEqual(gps_glitch_events[0]["flags"], int(real_flags))

    def test_reset_statustext_detected(self):
        rows = {"STATUSTEXT": [{"host_monotonic_time": 1.0, "statustext_text": "EKF3 IMU0 reset"}]}
        events, _ = cmp.scan_ekf_and_statustext_health(rows)
        self.assertEqual(len(events), 1)
        self.assertEqual(events[0]["kind"], "statustext_reset_candidate")

    def test_benign_statustext_no_event(self):
        rows = {"STATUSTEXT": [{"host_monotonic_time": 1.0, "statustext_text": "ArduPlane V4.5.0"}]}
        events, gps_glitch_events = cmp.scan_ekf_and_statustext_health(rows)
        self.assertEqual(events, [])
        self.assertEqual(gps_glitch_events, [])

    def test_armed_during_run_detected(self):
        rows = {"HEARTBEAT": [{"host_monotonic_time": 1.0, "hb_armed": 1.0, "hb_mode_is_manual": 1.0}]}
        events, _ = cmp.scan_ekf_and_statustext_health(rows)
        self.assertEqual(events[0]["kind"], "armed_during_run")


class TestR3aEkfWarmupGate(unittest.TestCase):
    HEALTHY_FLAGS = cmp.EKF_ATTITUDE | cmp.EKF_POS_HORIZ_REL

    def _summary_with_health_rows(self, health_rows):
        tmpdir = Path(tempfile.mkdtemp(prefix="hil_f24r3e_summary_"))
        truth_csv = tmpdir / "truth.csv"
        pixhawk_csv = tmpdir / "pixhawk.csv"
        write_csv(truth_csv, TRUTH_FIELDS, [truth_row(i * 0.02) for i in range(1600)])
        rows = []
        for i in range(150):
            t = i * 0.2
            rows.append(gpi_row(t))
            rows.append(att_row(t + 0.005))
            rows.append(vfr_row(t + 0.01))
        rows.extend(health_rows)
        write_csv(pixhawk_csv, PIXHAWK_FIELDS, rows)

        truth_rows = cmp.load_truth_csv(truth_csv)
        pixhawk_by_type = cmp.load_pixhawk_csv(pixhawk_csv)
        comparison_rows, coverage = cmp.align_and_compare(truth_rows, pixhawk_by_type, alignment_tolerance_s=0.1)
        return cmp.compute_summary(
            truth_rows, pixhawk_by_type, comparison_rows, coverage,
            alignment_tolerance_s=0.1, settle_s=20.0, thresholds=dict(cmp.DEFAULT_THRESHOLDS),
            health_ignore_before_s=15.0, require_ekf_ready_after_s=20.0,
        )

    def test_warmup_uninitialized_flags_before_15s_are_ignored_for_r3a(self):
        summary = self._summary_with_health_rows([
            ekf_row(2.0, cmp.EKF_UNINITIALIZED),
            ekf_row(19.0, self.HEALTHY_FLAGS),
            ekf_row(20.0, self.HEALTHY_FLAGS),
        ])
        self.assertTrue(summary["checks"]["ekf_ready_before_scoring"], summary)
        self.assertTrue(summary["checks"]["no_ekf_or_statustext_reset_events"], summary)
        self.assertEqual(summary["health_ignore_before_s"], 15.0)
        self.assertEqual(summary["require_ekf_ready_after_s"], 20.0)

    def test_no_scoring_while_ekf_uninitialized_at_required_start(self):
        summary = self._summary_with_health_rows([
            ekf_row(2.0, cmp.EKF_UNINITIALIZED),
            ekf_row(20.0, cmp.EKF_UNINITIALIZED),
            ekf_row(21.0, self.HEALTHY_FLAGS),
        ])
        self.assertFalse(summary["checks"]["ekf_ready_before_scoring"], summary)
        self.assertIn("EKF_UNINITIALIZED", summary["ekf_ready_reason"])

    def test_reset_or_alignment_statustext_after_warmup_fails_health_gate(self):
        summary = self._summary_with_health_rows([
            statustext_row(16.0, "EKF3 IMU0 reset"),
            ekf_row(20.0, self.HEALTHY_FLAGS),
        ])
        self.assertTrue(summary["checks"]["ekf_ready_before_scoring"], summary)
        self.assertFalse(summary["checks"]["no_ekf_or_statustext_reset_events"], summary)

    def test_default_r2_health_scan_still_flags_uninitialized_before_settle(self):
        rows = {"EKF_STATUS_REPORT": [ekf_row(2.0, cmp.EKF_UNINITIALIZED)]}
        events, _ = cmp.scan_ekf_and_statustext_health(rows)
        self.assertEqual(len(events), 1)
        self.assertEqual(events[0]["kind"], "ekf_unhealthy_flags")


class TestComputeSummaryEndToEnd(unittest.TestCase):
    def test_well_behaved_run_passes_all_checks(self):
        tmpdir = Path(tempfile.mkdtemp(prefix="hil_f24r2_summary_pass_"))
        truth_csv = tmpdir / "truth.csv"
        pixhawk_csv = tmpdir / "pixhawk.csv"
        write_csv(truth_csv, TRUTH_FIELDS, [truth_row(i * 0.02) for i in range(1600)])  # 20s @ 50Hz
        rows = []
        for i in range(150):  # 30s @ 5Hz
            t = i * 0.2
            rows.append(gpi_row(t, gpi_alt_m=BASE_ALT + 0.05, gpi_vn_mps=0.02))
            rows.append(att_row(t + 0.005, att_roll_deg=0.1, att_yaw_deg=math.degrees(BASE_YAW_RAD) + 0.2))
            rows.append(vfr_row(t + 0.01, vfr_airspeed_mps=0.08))
        write_csv(pixhawk_csv, PIXHAWK_FIELDS, rows)

        truth_rows = cmp.load_truth_csv(truth_csv)
        pixhawk_by_type = cmp.load_pixhawk_csv(pixhawk_csv)
        comparison_rows, coverage = cmp.align_and_compare(truth_rows, pixhawk_by_type, alignment_tolerance_s=0.1)
        summary = cmp.compute_summary(
            truth_rows, pixhawk_by_type, comparison_rows, coverage,
            alignment_tolerance_s=0.1, settle_s=5.0, thresholds=dict(cmp.DEFAULT_THRESHOLDS),
        )
        self.assertTrue(summary["overall_pass"], summary)
        self.assertIsNone(summary["first_failure"])
        # First 5s (25 GPI samples at 5Hz) excluded from steady-state scoring.
        self.assertLess(summary["metrics"]["altitude_error_m"]["count"], len(rows) // 3)

    def test_large_error_fails_relevant_check_only(self):
        tmpdir = Path(tempfile.mkdtemp(prefix="hil_f24r2_summary_fail_"))
        truth_csv = tmpdir / "truth.csv"
        pixhawk_csv = tmpdir / "pixhawk.csv"
        write_csv(truth_csv, TRUTH_FIELDS, [truth_row(i * 0.02) for i in range(1600)])
        rows = [gpi_row(t * 0.2 + 6.0, gpi_alt_m=BASE_ALT + 50.0) for t in range(50)]  # 50m alt error, past settle
        write_csv(pixhawk_csv, PIXHAWK_FIELDS, rows)

        truth_rows = cmp.load_truth_csv(truth_csv)
        pixhawk_by_type = cmp.load_pixhawk_csv(pixhawk_csv)
        comparison_rows, coverage = cmp.align_and_compare(truth_rows, pixhawk_by_type, alignment_tolerance_s=0.1)
        summary = cmp.compute_summary(
            truth_rows, pixhawk_by_type, comparison_rows, coverage,
            alignment_tolerance_s=0.1, settle_s=5.0, thresholds=dict(cmp.DEFAULT_THRESHOLDS),
        )
        self.assertFalse(summary["overall_pass"])
        self.assertFalse(summary["checks"]["altitude_error_rms"])

    def test_nonfinite_truth_field_raises(self):
        tmpdir = Path(tempfile.mkdtemp(prefix="hil_f24r2_nonfinite_"))
        truth_csv = tmpdir / "truth.csv"
        write_csv(truth_csv, TRUTH_FIELDS, [truth_row(0.0, alt_m="nan")])
        with self.assertRaises(ValueError):
            cmp.load_truth_csv(truth_csv)

    def test_r3l_momentary_gps_glitch_flag_does_not_fail_an_otherwise_healthy_run(self):
        """HIL-F24-R3L regression: reproduces sr75_hil_f24r3a_mp_visual_
        20260807T080924Z, where EKF_STATUS_REPORT briefly reports
        flags=33599 (=831 healthy bits | EKF_GPS_GLITCHING) mid-run, with
        every other check (GPS fix, attitude jumps, position/altitude/
        velocity/airspeed/roll/pitch/yaw RMS) passing. This must PASS
        overall -- the momentary advisory bit alone is not a reset event
        -- while still surfacing it in gps_glitch_events for visibility."""
        tmpdir = Path(tempfile.mkdtemp(prefix="hil_f24r2_summary_gpsglitch_"))
        truth_csv = tmpdir / "truth.csv"
        pixhawk_csv = tmpdir / "pixhawk.csv"
        write_csv(truth_csv, TRUTH_FIELDS, [truth_row(i * 0.02) for i in range(1600)])  # 20s @ 50Hz
        rows = []
        for i in range(150):  # 30s @ 5Hz
            t = i * 0.2
            rows.append(gpi_row(t, gpi_alt_m=BASE_ALT + 0.05, gpi_vn_mps=0.02))
            rows.append(att_row(t + 0.005, att_roll_deg=0.1, att_yaw_deg=math.degrees(BASE_YAW_RAD) + 0.2))
            rows.append(vfr_row(t + 0.01, vfr_airspeed_mps=0.08))
        # The real captured flag value, mid-run, alongside otherwise
        # unchanged healthy telemetry -- not accompanied by any attitude
        # jump, position divergence, or backend-flap STATUSTEXT.
        rows.append(ekf_row(15.0, 33599))
        write_csv(pixhawk_csv, PIXHAWK_FIELDS, rows)

        truth_rows = cmp.load_truth_csv(truth_csv)
        pixhawk_by_type = cmp.load_pixhawk_csv(pixhawk_csv)
        comparison_rows, coverage = cmp.align_and_compare(truth_rows, pixhawk_by_type, alignment_tolerance_s=0.1)
        summary = cmp.compute_summary(
            truth_rows, pixhawk_by_type, comparison_rows, coverage,
            alignment_tolerance_s=0.1, settle_s=5.0, thresholds=dict(cmp.DEFAULT_THRESHOLDS),
        )
        self.assertTrue(summary["checks"]["no_ekf_or_statustext_reset_events"], summary)
        self.assertTrue(summary["overall_pass"], summary)
        self.assertIsNone(summary["first_failure"])
        self.assertEqual(len(summary["gps_glitch_events"]), 1)
        self.assertEqual(summary["gps_glitch_events"][0]["flags"], 33599)


class TestMainEndToEnd(unittest.TestCase):
    def test_cli_run_produces_all_output_files_and_correct_exit_code(self):
        tmpdir = Path(tempfile.mkdtemp(prefix="hil_f24r2_cli_"))
        truth_csv = tmpdir / "truth.csv"
        pixhawk_csv = tmpdir / "pixhawk.csv"
        write_csv(truth_csv, TRUTH_FIELDS, [truth_row(i * 0.02) for i in range(1600)])
        rows = []
        for i in range(150):
            t = i * 0.2
            rows.append(gpi_row(t))
            rows.append(att_row(t + 0.005))
            rows.append(vfr_row(t + 0.01))
        write_csv(pixhawk_csv, PIXHAWK_FIELDS, rows)

        comparison_csv = tmpdir / "estimator_comparison.csv"
        summary_json = tmpdir / "estimator_summary.json"
        summary_md = tmpdir / "estimator_summary.md"
        proc = subprocess.run(
            [sys.executable, str(COMPARISON_SCRIPT),
             "--truth-csv", str(truth_csv), "--pixhawk-csv", str(pixhawk_csv),
             "--comparison-csv", str(comparison_csv),
             "--summary-json", str(summary_json), "--summary-md", str(summary_md)],
            capture_output=True, text=True,
        )
        self.assertEqual(proc.returncode, 0, proc.stdout + proc.stderr)
        self.assertTrue(comparison_csv.exists())
        self.assertTrue(summary_json.exists())
        self.assertTrue(summary_md.exists())
        with open(summary_json) as f:
            summary = json.load(f)
        self.assertTrue(summary["overall_pass"])


if __name__ == "__main__":
    unittest.main()
