#!/usr/bin/env python3
"""HIL-F24-R2: offline comparison of Pixhawk estimator output against
live SR-75 JSBSim truth, both captured (as two separate time-series CSVs)
by sr75_hil_f24r2_estimator_capture.py over the HIL-F24-R1 dynamic
PPP/SIM_JSON path.

Reads jsbsim_truth.csv and pixhawk_estimator.csv, aligns Pixhawk samples
to the nearest truth sample by HOST MONOTONIC timestamp only (never
wall-clock -- see align_pixhawk_stream()), computes per-sample errors,
and writes estimator_comparison.csv (every aligned sample) plus
estimator_summary.json/.md (aggregate max/mean/RMS/p95 per metric,
alignment coverage, sample-count/update-rate/finite-value checks,
discontinuity/EKF-health scanning, and PASS/FAIL against documented
thresholds -- excluding an initial settling period from steady-state
scoring, per task 7).

This script only ever reads CSV files already written to disk by the
capture process; it opens no serial/MAVLink connection and sends nothing
to any Pixhawk. It never tunes, adjusts, or otherwise touches the
estimator or any parameter -- per this task's explicit instruction, a
threshold failure is reported (first causal mismatch), not corrected.
"""
import argparse
import bisect
import csv
import json
import math
import os
import sys
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
SIM_JSON_DIR = Path(__file__).resolve().parents[1] / "sim_json"
sys.path.insert(0, str(SIM_JSON_DIR))
import sr75_hil_f24d_static_state_feed as feed  # noqa: E402 -- reuses _percentile()

EARTH_RADIUS_M = 6371000.0

# HIL-F24-R2 task 5: EKF_STATUS_REPORT.flags bits that indicate a
# degraded/unhealthy estimator (from the MAVLink EKF_STATUS_FLAGS enum).
EKF_ATTITUDE = 1
EKF_POS_HORIZ_REL = 8
EKF_POS_HORIZ_ABS = 16
EKF_CONST_POS_MODE = 128
EKF_UNINITIALIZED = 1024
EKF_GPS_GLITCHING = 32768
# HIL-F24-R3L: EKF_GPS_GLITCHING is deliberately NOT in EKF_UNHEALTHY_FLAGS.
# Unlike CONST_POS_MODE ("in constant position mode" -- EKF3 has fallen back
# to dead-reckoning) and UNINITIALIZED ("has never been healthy"), it is not
# a reset/fault condition: AP_NavEKF3_core.cpp's send_status_report() packs
# it straight from filterStatus.flags.gps_glitching
# (AP_NavEKF3_Control.cpp), which is set purely from a 1s-fail/10s-pass
# hysteresis on normalised GPS position/velocity innovations
# (AP_NavEKF3_VehicleStatus.cpp) -- a self-clearing quality advisory that
# can trip on a momentary innovation spike (e.g. a trajectory's own
# kinematic discontinuity) while every position/velocity/attitude validity
# bit remains set the whole time. Treating it as a "reset" event produced a
# false FAIL on an otherwise fully-healthy run (flags=831+32768, i.e. every
# healthy bit still set) -- see HIL_F24_R3L report. It is still surfaced,
# just not gated on, via gps_glitch_events below.
EKF_UNHEALTHY_FLAGS = EKF_CONST_POS_MODE | EKF_UNINITIALIZED
EKF_VALID_POSITION_FLAGS = EKF_POS_HORIZ_REL | EKF_POS_HORIZ_ABS

# Substrings scanned in STATUSTEXT for a reset/lane-switch event. Matched
# case-insensitively; deliberately broad (some false positives are safer
# than missing a real reset) since a match only flags a candidate event
# for the summary's discontinuity list, it does not itself fail the run.
RESET_STATUSTEXT_SUBSTRINGS = (
    "ekf3 imu", "ekf2 imu", "lane switch", "primary changed", "ekf yaw reset",
    "ekf reset", "gps glitch", "inertial nav", "compass switched",
)

DEFAULT_ALIGNMENT_TOLERANCE_S = 0.1
DEFAULT_SETTLE_S = 5.0
DEFAULT_MIN_SAMPLES = 20
DEFAULT_MAX_MEDIAN_INTERVAL_S = 1.0  # message update-rate sanity bound
DEFAULT_ATTITUDE_JUMP_DEG = 10.0

DEFAULT_THRESHOLDS = {
    "alignment_coverage_min": 0.95,
    "horizontal_error_rms_max_m": 5.0,
    "altitude_error_rms_max_m": 3.0,
    "roll_error_rms_max_deg": 2.0,
    "pitch_error_rms_max_deg": 2.0,
    "yaw_error_rms_max_deg": 5.0,
    "vn_error_rms_max_mps": 1.5,
    "ve_error_rms_max_mps": 1.5,
    "vd_error_rms_max_mps": 1.5,
    "airspeed_error_rms_max_mps": 2.0,
    "max_attitude_jump_deg": DEFAULT_ATTITUDE_JUMP_DEG,
}


def horizontal_distance_m(lat1_deg, lon1_deg, lat2_deg, lon2_deg):
    """Equirectangular approximation -- accurate to well under 1% error
    over the few-hundred-metre scales an R1 ground-static (or any
    bench-scale) test can plausibly produce; a full great-circle formula
    is unnecessary complexity at this scale."""
    lat1, lon1, lat2, lon2 = (math.radians(v) for v in (lat1_deg, lon1_deg, lat2_deg, lon2_deg))
    mean_lat = (lat1 + lat2) / 2.0
    x = (lon2 - lon1) * math.cos(mean_lat) * EARTH_RADIUS_M
    y = (lat2 - lat1) * EARTH_RADIUS_M
    return math.hypot(x, y)


def yaw_error_deg(a_deg, b_deg):
    """Signed shortest-path a-b, correctly handling wraparound at
    +-180/360 -- e.g. yaw_error_deg(179, -179) == -2, not 358."""
    return (a_deg - b_deg + 180.0) % 360.0 - 180.0


def rms(values):
    values = [v for v in values if v is not None and math.isfinite(v)]
    if not values:
        return float("nan")
    return math.sqrt(sum(v * v for v in values) / len(values))


def _stats(values):
    values = [v for v in values if v is not None and math.isfinite(v)]
    if not values:
        return {"count": 0, "max": None, "mean": None, "rms": None, "p95": None}
    sorted_values = sorted(abs(v) for v in values)
    return {
        "count": len(values),
        "max": max(sorted_values),
        "mean": sum(sorted_values) / len(sorted_values),
        "rms": rms(values),
        "p95": feed._percentile(sorted_values, 95),
    }


def load_truth_csv(path):
    """Returns a list of dicts sorted by host_monotonic_time (float),
    all remaining fields parsed as float. Raises ValueError on a
    non-finite or unparsable field -- HIL-F24-R2 task 7's finite-value
    check applies to truth as well as to the Pixhawk stream."""
    rows = []
    with open(path, newline="", encoding="utf-8") as f:
        for raw in csv.DictReader(f):
            parsed = {}
            for key, value in raw.items():
                if value == "" or value is None:
                    parsed[key] = None
                    continue
                fvalue = float(value)
                if not math.isfinite(fvalue):
                    raise ValueError(f"non-finite truth field {key}={value!r}")
                parsed[key] = fvalue
            rows.append(parsed)
    rows.sort(key=lambda r: r["host_monotonic_time"])
    return rows


def load_pixhawk_csv(path):
    """Returns {msg_type: [rows...]}, each list sorted by
    host_monotonic_time. Non-numeric/blank fields are kept as None (not
    every column applies to every msg_type) rather than raising, but a
    field that IS present and non-empty must still be finite -- a
    non-finite Pixhawk field is a real anomaly worth failing on."""
    by_type = {}
    with open(path, newline="", encoding="utf-8") as f:
        for raw in csv.DictReader(f):
            msg_type = raw["msg_type"]
            row = {"host_monotonic_time": float(raw["host_monotonic_time"])}
            for key, value in raw.items():
                if key in ("host_monotonic_time", "host_wall_time", "msg_type"):
                    continue
                if value == "" or value is None:
                    row[key] = None
                    continue
                if key.endswith("_text") or key.endswith("_mode"):
                    row[key] = value
                    continue
                fvalue = float(value)
                if not math.isfinite(fvalue):
                    raise ValueError(f"non-finite pixhawk field {msg_type}.{key}={value!r}")
                row[key] = fvalue
            by_type.setdefault(msg_type, []).append(row)
    for rows in by_type.values():
        rows.sort(key=lambda r: r["host_monotonic_time"])
    return by_type


def nearest_within_tolerance(target_time, sorted_rows, tolerance_s):
    """HIL-F24-R2 task 3: nearest-neighbour alignment by host monotonic
    timestamp only, bounded by tolerance_s (the documented maximum
    alignment tolerance -- see DEFAULT_ALIGNMENT_TOLERANCE_S). Returns
    the matching row dict, or None if nothing is within tolerance.
    O(log n) via bisect since sorted_rows is time-sorted."""
    if not sorted_rows:
        return None
    times = [r["host_monotonic_time"] for r in sorted_rows]
    idx = bisect.bisect_left(times, target_time)
    best_row = None
    best_dt = None
    for i in (idx - 1, idx):
        if 0 <= i < len(sorted_rows):
            dt = abs(times[i] - target_time)
            if dt <= tolerance_s and (best_dt is None or dt < best_dt):
                best_dt, best_row = dt, sorted_rows[i]
    return best_row


def align_and_compare(truth_rows, pixhawk_by_type, alignment_tolerance_s):
    """HIL-F24-R2 task 3/4: GLOBAL_POSITION_INT is the primary comparison
    tick (it carries lat/lon/alt/NED velocity); for each such sample,
    the nearest ATTITUDE (roll/pitch/yaw/p/q/r) and VFR_HUD (airspeed)
    samples within tolerance are folded into the same comparison row,
    and the nearest truth row within tolerance supplies the reference
    values. A GLOBAL_POSITION_INT sample with no truth match within
    tolerance is still counted (for alignment-coverage denominator) but
    contributes no error values.

    Returns (comparison_rows, coverage) where coverage = fraction of
    GLOBAL_POSITION_INT samples that found a truth match.
    """
    gpi_rows = pixhawk_by_type.get("GLOBAL_POSITION_INT", [])
    att_rows = pixhawk_by_type.get("ATTITUDE", [])
    vfr_rows = pixhawk_by_type.get("VFR_HUD", [])

    comparison_rows = []
    aligned_count = 0
    for gpi in gpi_rows:
        t = gpi["host_monotonic_time"]
        truth = nearest_within_tolerance(t, truth_rows, alignment_tolerance_s)
        att = nearest_within_tolerance(t, att_rows, alignment_tolerance_s)
        vfr = nearest_within_tolerance(t, vfr_rows, alignment_tolerance_s)

        row = {
            "host_monotonic_time": t,
            "truth_host_monotonic_time": truth["host_monotonic_time"] if truth else None,
            "alignment_dt_s": (t - truth["host_monotonic_time"]) if truth else None,
            "horizontal_error_m": None, "altitude_error_m": None,
            "vn_error_mps": None, "ve_error_mps": None, "vd_error_mps": None,
            "roll_error_deg": None, "pitch_error_deg": None, "yaw_error_deg": None,
            "p_error_deg_s": None, "q_error_deg_s": None, "r_error_deg_s": None,
            "airspeed_error_mps": None,
        }

        if truth is not None:
            aligned_count += 1
            if gpi.get("gpi_lat_deg") is not None and gpi.get("gpi_lon_deg") is not None:
                row["horizontal_error_m"] = horizontal_distance_m(
                    gpi["gpi_lat_deg"], gpi["gpi_lon_deg"], truth["lat_deg"], truth["lon_deg"],
                )
            if gpi.get("gpi_alt_m") is not None:
                row["altitude_error_m"] = gpi["gpi_alt_m"] - truth["alt_m"]
            if gpi.get("gpi_vn_mps") is not None:
                row["vn_error_mps"] = gpi["gpi_vn_mps"] - truth["vn_mps"]
            if gpi.get("gpi_ve_mps") is not None:
                row["ve_error_mps"] = gpi["gpi_ve_mps"] - truth["ve_mps"]
            if gpi.get("gpi_vd_mps") is not None:
                row["vd_error_mps"] = gpi["gpi_vd_mps"] - truth["vd_mps"]
            if att is not None:
                row["roll_error_deg"] = att["att_roll_deg"] - math.degrees(truth["roll_rad"])
                row["pitch_error_deg"] = att["att_pitch_deg"] - math.degrees(truth["pitch_rad"])
                row["yaw_error_deg"] = yaw_error_deg(att["att_yaw_deg"], math.degrees(truth["yaw_rad"]))
                row["p_error_deg_s"] = att["att_p_deg_s"] - math.degrees(truth["p_rad_s"])
                row["q_error_deg_s"] = att["att_q_deg_s"] - math.degrees(truth["q_rad_s"])
                row["r_error_deg_s"] = att["att_r_deg_s"] - math.degrees(truth["r_rad_s"])
            if vfr is not None:
                row["airspeed_error_mps"] = vfr["vfr_airspeed_mps"] - truth["airspeed_mps"]

        comparison_rows.append(row)

    coverage = (aligned_count / len(gpi_rows)) if gpi_rows else 0.0
    return comparison_rows, coverage


def _median_interval_s(sorted_rows):
    if len(sorted_rows) < 2:
        return None
    times = [r["host_monotonic_time"] for r in sorted_rows]
    intervals = sorted(t2 - t1 for t1, t2 in zip(times, times[1:]))
    n = len(intervals)
    return intervals[n // 2] if n % 2 else (intervals[n // 2 - 1] + intervals[n // 2]) / 2.0


def detect_attitude_jumps(att_rows, max_jump_deg):
    """HIL-F24-R2 task 7: flags a consecutive-sample attitude jump larger
    than max_jump_deg on roll, pitch, or yaw (yaw via the wrap-aware
    helper) as a candidate discontinuity."""
    jumps = []
    for prev, cur in zip(att_rows, att_rows[1:]):
        roll_jump = cur["att_roll_deg"] - prev["att_roll_deg"]
        pitch_jump = cur["att_pitch_deg"] - prev["att_pitch_deg"]
        yaw_jump = yaw_error_deg(cur["att_yaw_deg"], prev["att_yaw_deg"])
        biggest = max(abs(roll_jump), abs(pitch_jump), abs(yaw_jump))
        if biggest > max_jump_deg:
            jumps.append({
                "host_monotonic_time": cur["host_monotonic_time"],
                "roll_jump_deg": roll_jump, "pitch_jump_deg": pitch_jump, "yaw_jump_deg": yaw_jump,
            })
    return jumps


def scan_ekf_and_statustext_health(pixhawk_by_type, ekf_start_time=None, statustext_start_time=None):
    """HIL-F24-R2 task 6/7: scans EKF_STATUS_REPORT.flags for the
    unhealthy bits and STATUSTEXT for reset/lane-switch keywords. Also
    scans HEARTBEAT for any transition to armed or away from MANUAL
    during the run (defense in depth alongside the pre/post precheck).

    HIL-F24-R3L: returns (events, gps_glitch_events) -- `events` is what
    gates no_ekf_or_statustext_reset_events (true reset/fault conditions
    only); `gps_glitch_events` is EKF_GPS_GLITCHING occurrences reported
    separately for visibility, since that bit is a self-clearing quality
    advisory, not a reset (see EKF_UNHEALTHY_FLAGS's comment above)."""
    events = []
    gps_glitch_events = []
    for row in pixhawk_by_type.get("EKF_STATUS_REPORT", []):
        if ekf_start_time is not None and row["host_monotonic_time"] < ekf_start_time:
            continue
        flags = row.get("ekf_flags")
        if flags is None:
            continue
        flags = int(flags)
        if flags & EKF_UNHEALTHY_FLAGS:
            events.append({
                "host_monotonic_time": row["host_monotonic_time"],
                "kind": "ekf_unhealthy_flags", "flags": flags,
            })
        if flags & EKF_GPS_GLITCHING:
            gps_glitch_events.append({
                "host_monotonic_time": row["host_monotonic_time"],
                "kind": "ekf_gps_glitching_advisory", "flags": flags,
            })
    for row in pixhawk_by_type.get("STATUSTEXT", []):
        if statustext_start_time is not None and row["host_monotonic_time"] < statustext_start_time:
            continue
        text = (row.get("statustext_text") or "")
        lowered = text.lower()
        if any(sub in lowered for sub in RESET_STATUSTEXT_SUBSTRINGS):
            events.append({
                "host_monotonic_time": row["host_monotonic_time"],
                "kind": "statustext_reset_candidate", "text": text,
            })
    hb_rows = pixhawk_by_type.get("HEARTBEAT", [])
    for row in hb_rows:
        if row.get("hb_armed") not in (None, 0.0):
            events.append({"host_monotonic_time": row["host_monotonic_time"], "kind": "armed_during_run"})
        if row.get("hb_mode_is_manual") == 0.0:
            events.append({
                "host_monotonic_time": row["host_monotonic_time"], "kind": "mode_not_manual_during_run",
            })
    events.sort(key=lambda e: e["host_monotonic_time"])
    gps_glitch_events.sort(key=lambda e: e["host_monotonic_time"])
    return events, gps_glitch_events


def ekf_ready_at_or_after(pixhawk_by_type, start_time):
    for row in pixhawk_by_type.get("EKF_STATUS_REPORT", []):
        if row["host_monotonic_time"] < start_time:
            continue
        flags = row.get("ekf_flags")
        if flags is None:
            return False, "first EKF_STATUS_REPORT after scoring start has no flags"
        flags = int(flags)
        if flags & EKF_UNINITIALIZED:
            return False, f"EKF_UNINITIALIZED still set at scoring start flags={flags}"
        if not (flags & EKF_ATTITUDE):
            return False, f"EKF attitude flag missing at scoring start flags={flags}"
        if not (flags & EKF_VALID_POSITION_FLAGS):
            return False, f"EKF position flag missing at scoring start flags={flags}"
        return True, f"EKF ready at scoring start flags={flags}"
    return False, "no EKF_STATUS_REPORT at or after scoring start"


def compute_summary(truth_rows, pixhawk_by_type, comparison_rows, coverage,
                    alignment_tolerance_s, settle_s, thresholds,
                    health_ignore_before_s=0.0, require_ekf_ready_after_s=None):
    """HIL-F24-R2 tasks 6/7: aggregate max/mean/RMS/p95 per metric
    (excluding the initial settle_s seconds from steady-state scoring),
    sample-count/update-rate/alignment-coverage checks, discontinuity and
    EKF-health scanning, and PASS/FAIL against `thresholds`."""
    run_start = comparison_rows[0]["host_monotonic_time"] if comparison_rows else None
    steady_state_rows = [
        r for r in comparison_rows
        if run_start is not None and (r["host_monotonic_time"] - run_start) >= settle_s
    ]

    metrics = {}
    for field in (
        "horizontal_error_m", "altitude_error_m", "vn_error_mps", "ve_error_mps", "vd_error_mps",
        "roll_error_deg", "pitch_error_deg", "yaw_error_deg",
        "p_error_deg_s", "q_error_deg_s", "r_error_deg_s", "airspeed_error_mps",
    ):
        metrics[field] = _stats([r[field] for r in steady_state_rows])

    att_rows = pixhawk_by_type.get("ATTITUDE", [])
    gpi_rows = pixhawk_by_type.get("GLOBAL_POSITION_INT", [])
    vfr_rows = pixhawk_by_type.get("VFR_HUD", [])
    attitude_jumps = detect_attitude_jumps(att_rows, thresholds["max_attitude_jump_deg"])
    health_start_time = (run_start + health_ignore_before_s) if run_start is not None else None
    scoring_start_time = (
        run_start + require_ekf_ready_after_s
        if run_start is not None and require_ekf_ready_after_s is not None
        else health_start_time
    )
    health_events, gps_glitch_events = scan_ekf_and_statustext_health(
        pixhawk_by_type,
        ekf_start_time=scoring_start_time,
        statustext_start_time=health_start_time,
    )
    ekf_ready_ok = True
    ekf_ready_reason = "not required"
    if run_start is not None and require_ekf_ready_after_s is not None:
        ekf_ready_ok, ekf_ready_reason = ekf_ready_at_or_after(
            pixhawk_by_type, run_start + require_ekf_ready_after_s,
        )

    sample_counts = {
        "truth_rows": len(truth_rows),
        "GLOBAL_POSITION_INT": len(gpi_rows),
        "ATTITUDE": len(att_rows),
        "VFR_HUD": len(vfr_rows),
        "steady_state_comparison_rows": len(steady_state_rows),
    }
    update_rates_s = {
        "GLOBAL_POSITION_INT_median_interval_s": _median_interval_s(gpi_rows),
        "ATTITUDE_median_interval_s": _median_interval_s(att_rows),
        "VFR_HUD_median_interval_s": _median_interval_s(vfr_rows),
    }

    checks = {}
    checks["finite_values"] = True  # load_*_csv() already raises on any non-finite field
    checks["monotonic_truth_timestamps"] = all(
        a["host_monotonic_time"] <= b["host_monotonic_time"] for a, b in zip(truth_rows, truth_rows[1:])
    )
    checks["monotonic_pixhawk_timestamps"] = all(
        all(a["host_monotonic_time"] <= b["host_monotonic_time"] for a, b in zip(rows, rows[1:]))
        for rows in pixhawk_by_type.values()
    )
    checks["sample_count_minimum"] = (
        sample_counts["GLOBAL_POSITION_INT"] >= DEFAULT_MIN_SAMPLES
        and sample_counts["ATTITUDE"] >= DEFAULT_MIN_SAMPLES
    )
    checks["update_rate_sane"] = all(
        v is not None and v <= DEFAULT_MAX_MEDIAN_INTERVAL_S for v in update_rates_s.values()
    )
    checks["alignment_coverage"] = coverage >= thresholds["alignment_coverage_min"]
    checks["no_ekf_or_statustext_reset_events"] = len(health_events) == 0
    checks["ekf_ready_before_scoring"] = ekf_ready_ok
    checks["no_large_attitude_jump"] = len(attitude_jumps) == 0
    checks["horizontal_error_rms"] = (
        metrics["horizontal_error_m"]["rms"] is not None
        and metrics["horizontal_error_m"]["rms"] <= thresholds["horizontal_error_rms_max_m"]
    )
    checks["altitude_error_rms"] = (
        metrics["altitude_error_m"]["rms"] is not None
        and metrics["altitude_error_m"]["rms"] <= thresholds["altitude_error_rms_max_m"]
    )
    checks["roll_error_rms"] = (
        metrics["roll_error_deg"]["rms"] is not None
        and metrics["roll_error_deg"]["rms"] <= thresholds["roll_error_rms_max_deg"]
    )
    checks["pitch_error_rms"] = (
        metrics["pitch_error_deg"]["rms"] is not None
        and metrics["pitch_error_deg"]["rms"] <= thresholds["pitch_error_rms_max_deg"]
    )
    checks["yaw_error_rms"] = (
        metrics["yaw_error_deg"]["rms"] is not None
        and metrics["yaw_error_deg"]["rms"] <= thresholds["yaw_error_rms_max_deg"]
    )
    checks["vn_error_rms"] = (
        metrics["vn_error_mps"]["rms"] is not None
        and metrics["vn_error_mps"]["rms"] <= thresholds["vn_error_rms_max_mps"]
    )
    checks["ve_error_rms"] = (
        metrics["ve_error_mps"]["rms"] is not None
        and metrics["ve_error_mps"]["rms"] <= thresholds["ve_error_rms_max_mps"]
    )
    checks["vd_error_rms"] = (
        metrics["vd_error_mps"]["rms"] is not None
        and metrics["vd_error_mps"]["rms"] <= thresholds["vd_error_rms_max_mps"]
    )
    checks["airspeed_error_rms"] = (
        metrics["airspeed_error_mps"]["rms"] is not None
        and metrics["airspeed_error_mps"]["rms"] <= thresholds["airspeed_error_rms_max_mps"]
    )

    overall_pass = all(checks.values())
    first_failure = next((name for name, ok in checks.items() if not ok), None)

    return {
        "alignment_tolerance_s": alignment_tolerance_s,
        "settle_s": settle_s,
        "health_ignore_before_s": health_ignore_before_s,
        "require_ekf_ready_after_s": require_ekf_ready_after_s,
        "ekf_ready_reason": ekf_ready_reason,
        "thresholds": thresholds,
        "sample_counts": sample_counts,
        "update_rates_s": update_rates_s,
        "alignment_coverage": coverage,
        "metrics": metrics,
        "attitude_jumps": attitude_jumps,
        "health_events": health_events,
        "gps_glitch_events": gps_glitch_events,
        "checks": checks,
        "overall_pass": overall_pass,
        "first_failure": first_failure,
    }


def write_comparison_csv(path, comparison_rows):
    fieldnames = [
        "host_monotonic_time", "truth_host_monotonic_time", "alignment_dt_s",
        "horizontal_error_m", "altitude_error_m", "vn_error_mps", "ve_error_mps", "vd_error_mps",
        "roll_error_deg", "pitch_error_deg", "yaw_error_deg",
        "p_error_deg_s", "q_error_deg_s", "r_error_deg_s", "airspeed_error_mps",
    ]
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in comparison_rows:
            writer.writerow({k: ("" if row.get(k) is None else row[k]) for k in fieldnames})


def write_summary_json(path, summary):
    with open(path, "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2, sort_keys=True)
        f.write("\n")


def write_summary_markdown(path, summary):
    lines = ["# HIL-F24-R2 estimator comparison summary", ""]
    lines.append(f"Overall: **{'PASS' if summary['overall_pass'] else 'FAIL'}**")
    if not summary["overall_pass"]:
        lines.append(f"First failing check: `{summary['first_failure']}`")
    lines.append("")
    lines.append(f"Alignment tolerance: {summary['alignment_tolerance_s']*1000:.0f} ms; "
                 f"settle period excluded from scoring: {summary['settle_s']:.1f} s")
    lines.append(f"Alignment coverage: {summary['alignment_coverage']*100:.1f}%")
    lines.append("")
    lines.append("## Checks")
    for name, ok in sorted(summary["checks"].items()):
        lines.append(f"- [{'x' if ok else ' '}] {name}")
    lines.append("")
    lines.append("## Metrics (steady-state, post-settle)")
    lines.append("| metric | count | max | mean | rms | p95 |")
    lines.append("|---|---|---|---|---|---|")
    for name, stats in sorted(summary["metrics"].items()):
        def fmt(v):
            return "n/a" if v is None else f"{v:.4f}"
        lines.append(f"| {name} | {stats['count']} | {fmt(stats['max'])} | {fmt(stats['mean'])} | "
                     f"{fmt(stats['rms'])} | {fmt(stats['p95'])} |")
    lines.append("")
    if summary["attitude_jumps"]:
        lines.append(f"## Attitude jumps ({len(summary['attitude_jumps'])})")
        for jump in summary["attitude_jumps"][:20]:
            lines.append(f"- t={jump['host_monotonic_time']:.3f} roll={jump['roll_jump_deg']:.2f} "
                         f"pitch={jump['pitch_jump_deg']:.2f} yaw={jump['yaw_jump_deg']:.2f}")
        lines.append("")
    if summary["health_events"]:
        lines.append(f"## Health/reset events ({len(summary['health_events'])})")
        for event in summary["health_events"][:20]:
            lines.append(f"- t={event['host_monotonic_time']:.3f} {event}")
        lines.append("")
    if summary.get("gps_glitch_events"):
        lines.append(
            f"## GPS-glitch advisory events ({len(summary['gps_glitch_events'])}) "
            "-- informational only, not gated on (see EKF_UNHEALTHY_FLAGS)"
        )
        for event in summary["gps_glitch_events"][:20]:
            lines.append(f"- t={event['host_monotonic_time']:.3f} {event}")
        lines.append("")
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")


def build_arg_parser():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--truth-csv", required=True)
    parser.add_argument("--pixhawk-csv", required=True)
    parser.add_argument("--comparison-csv", required=True)
    parser.add_argument("--summary-json", required=True)
    parser.add_argument("--summary-md", required=True)
    parser.add_argument("--alignment-tolerance-s", type=float, default=DEFAULT_ALIGNMENT_TOLERANCE_S)
    parser.add_argument("--settle-s", type=float, default=DEFAULT_SETTLE_S)
    parser.add_argument("--health-ignore-before-s", type=float, default=0.0)
    parser.add_argument("--require-ekf-ready-after-s", type=float, default=None)
    return parser


def main():
    args = build_arg_parser().parse_args()
    truth_rows = load_truth_csv(args.truth_csv)
    pixhawk_by_type = load_pixhawk_csv(args.pixhawk_csv)
    comparison_rows, coverage = align_and_compare(truth_rows, pixhawk_by_type, args.alignment_tolerance_s)
    summary = compute_summary(
        truth_rows, pixhawk_by_type, comparison_rows, coverage,
        args.alignment_tolerance_s, args.settle_s, dict(DEFAULT_THRESHOLDS),
        health_ignore_before_s=args.health_ignore_before_s,
        require_ekf_ready_after_s=args.require_ekf_ready_after_s,
    )
    write_comparison_csv(args.comparison_csv, comparison_rows)
    write_summary_json(args.summary_json, summary)
    write_summary_markdown(args.summary_md, summary)
    print(f"ESTIMATOR_COMPARISON_RESULT overall_pass={summary['overall_pass']} "
          f"first_failure={summary['first_failure']} "
          f"alignment_coverage={summary['alignment_coverage']:.3f}")
    return 0 if summary["overall_pass"] else 1


if __name__ == "__main__":
    sys.exit(main())
