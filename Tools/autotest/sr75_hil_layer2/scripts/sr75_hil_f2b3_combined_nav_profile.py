#!/usr/bin/env python3
"""HIL-F23-F2B3: controlled combined GPS navigation-state profile.

Combines position, velocity, altitude, and yaw into ONE coherent
GPS_INPUT stream (position/velocity/altitude fusion validated
individually in F23-F2B2/F2B2A/F2B0-family tasks; this is the first test
that exercises all four together, at deliberately low dynamics -- the
observed problem with a high-dynamic JSBSim dive was that it was
incoherent with the physical IMU, not that GPS fusion itself is broken).

Trajectory (all times are elapsed seconds from test start):

  Phase A  0-10s   stationary hold      : vn=ve=vd=0, yaw=315.091444 deg, alt=3000m
  Phase B  10-30s  fly NW at 10 m/s     : heading=315.091444 deg (fixed), alt hold 3000m
  Phase C  30-50s  coordinated turn     : heading 315.091444 -> 270 deg (~2.25 deg/s),
                                           groundspeed=10 m/s, climb +1 m/s (vd=-1)
  Phase D  50-70s  fly west at 10 m/s   : heading=270 deg (fixed), alt hold 3020m
  Phase E  70-90s  coordinated turn     : heading 270 -> 315.091444 deg (~2.25 deg/s),
                                           groundspeed=10 m/s, descend -1 m/s (vd=+1)
  Phase F  90-100s final stationary hold: vn=ve=vd=0, yaw=315.091444 deg, alt=3000m

lat/lon are integrated forward (local-tangent-plane Euler integration)
from vn/ve; vn/ve are derived from groundspeed+heading each step. No
JSBSim, no aircraft physics -- a lightweight pure-Python generator writes
rows, in real time, to a CSV file in the exact schema the bridge's
existing (already-validated) JSBSimStateFeeder/JSB_FEED_ALIASES already
parses (proven byte-compatible in F23-F2B2A; unchanged here).

GPS_INPUT ONLY: --gps-input-inject and --gps-input-send-yaw are enabled;
--attitude-inject, --airspeed-inject, and --allow-actuator-output are
never passed.

SR-75 LAYER 2 HIL BENCH SAFETY:
No live engine, no fuel pump, no live RATO ignition, no live ejection,
no arming, no AUTO, no actuator output, no CH7/CH8 activity, no
parameter writes (preflight is entirely PARAM_REQUEST_READ).
"""
import argparse
import csv
import math
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[4]
SCRIPTS_DIR = Path(__file__).resolve().parent
BRIDGE_DIR = REPO / "Tools/autotest/sr75_hil_layer2/bridge"
sys.path.insert(0, str(SCRIPTS_DIR))
sys.path.insert(0, str(BRIDGE_DIR))
import sr75_hil_f23e1_live_feed_launch as e1_launch  # noqa: E402
import sr75_mission_heading as smh  # noqa: E402

BRIDGE_SCRIPT = BRIDGE_DIR / "sr75_jsbsim_pixhawk_hil_bridge.py"
BRIDGE_CONSOLE_LOG_NAME = "bridge_console.log"
DEFAULT_MP_OUT = "udp:192.168.64.1:14550"

# Fixed test-point position/heading/altitude (task spec; same validated
# release lat/lon and mission-derived bearing used throughout this task
# family).
FIXED_LAT = 32.5378147
FIXED_LON = 74.3661871
FIXED_YAW_DEG = 315.091444
BASE_ALT_M = 3000.0
CLIMB_RATE_MPS = 1.0
PEAK_ALT_M = BASE_ALT_M + CLIMB_RATE_MPS * 20.0  # 3020.0 (matches phase C's 20s climb)
GROUNDSPEED_MPS = 10.0
HEADING_MID_DEG = 270.0
EARTH_RADIUS_M = 6371000.0

PHASE_A_S = 10.0
PHASE_B_S = 20.0
PHASE_C_S = 20.0
PHASE_D_S = 20.0
PHASE_E_S = 20.0
PHASE_F_S = 10.0

T_A_END = PHASE_A_S                    # 10.0
T_B_END = T_A_END + PHASE_B_S          # 30.0
T_C_END = T_B_END + PHASE_C_S          # 50.0
T_D_END = T_C_END + PHASE_D_S          # 70.0
T_E_END = T_D_END + PHASE_E_S          # 90.0
T_F_END = T_E_END + PHASE_F_S          # 100.0
PROFILE_DURATION_S = T_F_END           # 100.0

# Turn rate for phases C/E: linear interpolation between the exact named
# heading endpoints (315.091444 <-> 270.0) over 20s each, which works out
# to ~2.2546 deg/s -- matches the task's "about 2.25 deg/s" spec while
# keeping phase boundaries exact (no residual offset).
YAW_RATE_C_DEG_S = (FIXED_YAW_DEG - HEADING_MID_DEG) / PHASE_C_S
YAW_RATE_E_DEG_S = (FIXED_YAW_DEG - HEADING_MID_DEG) / PHASE_E_S

PHASES_IN_ORDER = ("A", "B", "C", "D", "E", "F")
HOLD_ALT_PHASES = ("A", "B", "D", "F")   # constant-altitude phases (B holds 3000m while moving)
HOLD_YAW_PHASES = ("A", "B", "D", "F")   # constant-heading phases (turns happen in C/E only)

# HIL-F23-F2B3B: moving GPS-yaw preconditioning, run BEFORE the A-F
# profile timer starts. F23-F2B3A's retained-log audit found AHRS2 yaw
# never converged toward the transmitted GPS yaw during F23-F2B3's run
# (48-124 deg error throughout) and EKF_STATUS_REPORT.flags stayed stuck
# at 167 (EKF_CONST_POS_MODE set, EKF_POS_HORIZ_REL/ABS never set) for
# the entire 100s -- i.e. EK3 never trusted GPS position. Phase A's
# stationary hold (vn=ve=0) gives GPS-yaw fusion nothing to correlate
# against; preconditioning flies a straight, moving leg first so the EKF
# has a chance to align yaw and gain horizontal-position confidence
# before the scored profile (and its own initial stationary hold) begins.
PRECONDITION_MAX_DURATION_S = 15.0
# HIL-F23-F2B3C item 6: --precondition-only default observation window --
# intentionally longer than the 15s combined-run cap, since this mode
# exists purely to WATCH EKF convergence (or its absence) rather than to
# gate entry into a following profile.
PRECONDITION_ONLY_DEFAULT_DURATION_S = 20.0
PRECONDITION_HEADING_DEG = FIXED_YAW_DEG      # 315.091444
PRECONDITION_GROUNDSPEED_MPS = GROUNDSPEED_MPS  # 10.0 -> vn~=+7.08, ve~=-7.06

# EKF_STATUS_FLAGS bits (modules/mavlink/message_definitions/v1.0/
# ardupilotmega.xml) needed for the preconditioning gate.
EKF_POS_HORIZ_REL_BIT = 8
EKF_POS_HORIZ_ABS_BIT = 16
EKF_POS_HORIZ_VALID_MASK = EKF_POS_HORIZ_REL_BIT | EKF_POS_HORIZ_ABS_BIT

YAW_LOCK_TOLERANCE_DEG = 2.0
YAW_LOCK_CONSECUTIVE_REQUIRED = 3

PROFILE_HEADER = [
    "Time", "jsb_time_s", "jsb_lat_deg", "jsb_lon_deg", "jsb_alt_m",
    "jsb_vn_mps", "jsb_ve_mps", "jsb_vd_mps", "jsb_airspeed_mps",
    "jsb_roll_rad", "jsb_pitch_rad", "jsb_yaw_rad",
]


def compute_heading_speed_vd(t_s: float):
    """Pure, closed-form per-phase (heading_deg, groundspeed_mps, vd_mps,
    phase). Clamps to phase A for t<0 and phase F (final hold) for
    t>=PROFILE_DURATION_S.
    """
    if t_s < 0.0:
        return FIXED_YAW_DEG, 0.0, 0.0, "A"
    if t_s < T_A_END:
        return FIXED_YAW_DEG, 0.0, 0.0, "A"
    if t_s < T_B_END:
        return FIXED_YAW_DEG, GROUNDSPEED_MPS, 0.0, "B"
    if t_s < T_C_END:
        heading = FIXED_YAW_DEG - YAW_RATE_C_DEG_S * (t_s - T_B_END)
        return smh.normalize_0_360(heading), GROUNDSPEED_MPS, -CLIMB_RATE_MPS, "C"
    if t_s < T_D_END:
        return HEADING_MID_DEG, GROUNDSPEED_MPS, 0.0, "D"
    if t_s < T_E_END:
        heading = HEADING_MID_DEG + YAW_RATE_E_DEG_S * (t_s - T_D_END)
        return smh.normalize_0_360(heading), GROUNDSPEED_MPS, CLIMB_RATE_MPS, "E"
    if t_s < T_F_END:
        return FIXED_YAW_DEG, 0.0, 0.0, "F"
    return FIXED_YAW_DEG, 0.0, 0.0, "F"


def groundspeed_to_vn_ve(groundspeed_mps: float, heading_deg: float):
    """North/east velocity components for a heading (0=N, 90=E, clockwise
    -- same convention as sr75_mission_heading.MissionHeadingResult.vn_ve()).
    """
    rad = math.radians(heading_deg)
    return groundspeed_mps * math.cos(rad), groundspeed_mps * math.sin(rad)


def integrate_trajectory(dt_s=0.1, duration_s=PROFILE_DURATION_S,
                          start_lat=FIXED_LAT, start_lon=FIXED_LON, start_alt=BASE_ALT_M):
    """Pure function: precompute the full trajectory table via forward-
    Euler, locally-accurate lat/lon integration from the closed-form
    heading/groundspeed/vd profile (local tangent-plane approximation:
    dlat = vn*dt/R, dlon = ve*dt/(R*cos(lat)) -- valid at these small
    speeds/short durations/mid-latitude). Returns a list of row dicts,
    each with t_s/lat_deg/lon_deg/alt_m/vn_mps/ve_mps/vd_mps/
    heading_deg/phase.
    """
    rows = []
    lat = start_lat
    lon = start_lon
    alt = start_alt
    n_steps = int(round(duration_s / dt_s)) + 1
    for i in range(n_steps):
        t_s = round(i * dt_s, 6)
        heading_deg, gs_mps, vd_mps, phase = compute_heading_speed_vd(t_s)
        vn, ve = groundspeed_to_vn_ve(gs_mps, heading_deg)
        rows.append({
            "t_s": t_s, "lat_deg": lat, "lon_deg": lon, "alt_m": alt,
            "vn_mps": vn, "ve_mps": ve, "vd_mps": vd_mps,
            "heading_deg": heading_deg, "phase": phase,
        })
        lat_rad = math.radians(lat)
        dlat_deg = math.degrees((vn * dt_s) / EARTH_RADIUS_M)
        cos_lat = math.cos(lat_rad)
        dlon_deg = math.degrees((ve * dt_s) / (EARTH_RADIUS_M * cos_lat)) if abs(cos_lat) > 1e-9 else 0.0
        lat = lat + dlat_deg
        lon = lon + dlon_deg
        alt = alt - vd_mps * dt_s
    return rows


def trajectory_row_to_csv_row(row):
    """Convert one integrate_trajectory() row into a PROFILE_HEADER-
    matching CSV data row. roll/pitch/airspeed are fixed at 0.0 (not
    used by this test -- attitude/airspeed injection stay disabled)."""
    yaw_rad = math.radians(row["heading_deg"])
    t_s = row["t_s"]
    return [
        f"{t_s:.3f}", f"{t_s:.3f}", f"{row['lat_deg']:.8f}", f"{row['lon_deg']:.8f}", f"{row['alt_m']:.4f}",
        f"{row['vn_mps']:.4f}", f"{row['ve_mps']:.4f}", f"{row['vd_mps']:.4f}", "0.0",
        "0.0", "0.0", f"{yaw_rad:.6f}",
    ]


def write_profile_csv_realtime(csv_path, trajectory=None, rate_hz=10.0,
                                alive_check=None, sleep_fn=time.sleep, time_fn=time.time):
    """Append rows to csv_path in real (wall-clock) time, one per
    trajectory sample, anchored to a single start timestamp (not
    cumulative per-iteration sleeps) so scheduling jitter does not drift
    the profile over its ~100s duration. Returns rows written.

    alive_check: optional zero-arg callable returning an abort-ready
    diagnostic string (or None) -- if it ever returns non-None, writing
    stops immediately.
    """
    if trajectory is None:
        trajectory = integrate_trajectory(dt_s=1.0 / max(rate_hz, 0.1))
    csv_path = Path(csv_path)
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    rows_written = 0
    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(PROFILE_HEADER)
        f.flush()
        start_wall = time_fn()
        for row in trajectory:
            if alive_check is not None:
                abort_message = alive_check()
                if abort_message is not None:
                    return rows_written
            target_wall = start_wall + row["t_s"]
            now = time_fn()
            if target_wall > now:
                sleep_fn(target_wall - now)
            writer.writerow(trajectory_row_to_csv_row(row))
            f.flush()
            rows_written += 1
    return rows_written


def generate_precondition_trajectory(dt_s=0.1, max_duration_s=PRECONDITION_MAX_DURATION_S,
                                      start_lat=FIXED_LAT, start_lon=FIXED_LON, start_alt=BASE_ALT_M):
    """Pure function: precompute the FULL (worst-case, uncut) preconditioning
    trajectory table -- moving at PRECONDITION_GROUNDSPEED_MPS on the fixed
    PRECONDITION_HEADING_DEG heading, constant altitude (vd=0), lat/lon
    integrated continuously (same local-tangent-plane Euler integration
    as integrate_trajectory()), for up to max_duration_s.

    t_s counts from -max_duration_s up to (but not including) 0.0, so
    classify_phase_from_time() always returns None for these rows --
    HIL-F23-F2B3B item 2, "do not advance the A-F profile timer during
    preconditioning." The real run may cut this short (gate passes
    early); this table is the upper bound, truncated live by the caller.
    """
    vn, ve = groundspeed_to_vn_ve(PRECONDITION_GROUNDSPEED_MPS, PRECONDITION_HEADING_DEG)
    rows = []
    lat = start_lat
    lon = start_lon
    n_steps = int(round(max_duration_s / dt_s))
    for i in range(n_steps):
        elapsed = i * dt_s
        t_s = round(elapsed - max_duration_s, 6)
        rows.append({
            "t_s": t_s, "lat_deg": lat, "lon_deg": lon, "alt_m": start_alt,
            "vn_mps": vn, "ve_mps": ve, "vd_mps": 0.0,
            "heading_deg": PRECONDITION_HEADING_DEG, "phase": "PRECONDITION",
        })
        lat_rad = math.radians(lat)
        dlat_deg = math.degrees((vn * dt_s) / EARTH_RADIUS_M)
        cos_lat = math.cos(lat_rad)
        dlon_deg = math.degrees((ve * dt_s) / (EARTH_RADIUS_M * cos_lat)) if abs(cos_lat) > 1e-9 else 0.0
        lat = lat + dlat_deg
        lon = lon + dlon_deg
    return rows


def read_bridge_log_tail_samples(bridge_log_csv, max_samples=None):
    """Read the bridge's own CSV log (if it exists yet) and return an
    ordered (oldest-first) list of {"t", "ahrs2_yaw_deg", "ekf_flags",
    "gps_fix_type", "commanded_yaw_deg"} sample dicts -- used by the
    preconditioning gate. Returns [] if the file doesn't exist or has no
    data rows yet. HIL-F23-F2B3B item 7: AHRS2 yaw is read from
    ahrs2_yaw_rad exclusively (the confirmed real estimator-yaw column;
    see also compute_phase_summary(), which already used this column).
    """
    path = Path(bridge_log_csv)
    if not path.exists():
        return []
    try:
        with open(path, newline="") as f:
            rows = list(csv.DictReader(f))
    except OSError:
        return []
    samples = []
    for row in rows:
        ahrs2_yaw_rad = _f(row, "ahrs2_yaw_rad")
        commanded_yaw_cdeg = _f(row, "gps_tx_yaw_cdeg")
        samples.append({
            "t": _f(row, "time_s"),
            "ahrs2_yaw_deg": math.degrees(ahrs2_yaw_rad) if ahrs2_yaw_rad is not None else None,
            "ekf_flags": _f(row, "ekf_flags"),
            "gps_fix_type": _f(row, "gps_raw_fix_type"),
            "commanded_yaw_deg": (
                smh.decode_gps_input_yaw_cdeg(int(commanded_yaw_cdeg)) if commanded_yaw_cdeg is not None else None
            ),
        })
    if max_samples is not None:
        samples = samples[-max_samples:]
    return samples


def evaluate_precondition_gate_samples(samples):
    """Pure function: HIL-F23-F2B3B item 4's preconditioning PASS gate.

    A sample counts toward the "consecutive good" streak only if ALL of
    the following hold TOGETHER in that same sample (not independently
    across different samples):
      - wrapped AHRS2 yaw error (ahrs2_yaw_rad vs the transmitted GPS
        yaw for that same row) <= YAW_LOCK_TOLERANCE_DEG (2 deg)
      - EKF horizontal position valid: ekf_flags has EKF_POS_HORIZ_REL
        or EKF_POS_HORIZ_ABS set
      - GPS fix_type == 3

    The gate passes once >= YAW_LOCK_CONSECUTIVE_REQUIRED (3) consecutive
    samples (ending at the most recent sample) all satisfy this.

    Returns (passed: bool, reason: str, consecutive_count: int).
    """
    consecutive = 0
    reason = "no samples observed yet"
    for s in samples:
        yaw_err_deg = None
        if s.get("ahrs2_yaw_deg") is not None and s.get("commanded_yaw_deg") is not None:
            yaw_err_deg = abs(smh.wrap_angle_error_deg(s["commanded_yaw_deg"], s["ahrs2_yaw_deg"]))
        ekf_flags = s.get("ekf_flags")
        ekf_ok = ekf_flags is not None and (int(ekf_flags) & EKF_POS_HORIZ_VALID_MASK) != 0
        fix_ok = s.get("gps_fix_type") == 3

        yaw_ok = yaw_err_deg is not None and yaw_err_deg <= YAW_LOCK_TOLERANCE_DEG
        if yaw_ok and ekf_ok and fix_ok:
            consecutive += 1
            reason = f"consecutive good samples={consecutive} (yaw_err_deg={yaw_err_deg:.3f})"
        else:
            consecutive = 0
            failed = []
            if not yaw_ok:
                failed.append(f"yaw_err_deg={yaw_err_deg}")
            if not ekf_ok:
                failed.append(f"ekf_flags={ekf_flags} (POS_HORIZ_REL/ABS not set)")
            if not fix_ok:
                failed.append(f"gps_fix_type={s.get('gps_fix_type')} (!=3)")
            reason = "; ".join(failed)
    passed = consecutive >= YAW_LOCK_CONSECUTIVE_REQUIRED
    return passed, reason, consecutive


def print_precondition_status(elapsed_s, sample, consecutive, reason):
    """HIL-F23-F2B3B item 8 logging: precondition elapsed time,
    transmitted yaw, AHRS2 yaw, wrapped error, EKF flags, gate state/reason.
    """
    commanded = sample.get("commanded_yaw_deg") if sample else None
    ahrs2 = sample.get("ahrs2_yaw_deg") if sample else None
    ekf_flags = sample.get("ekf_flags") if sample else None
    err = abs(smh.wrap_angle_error_deg(commanded, ahrs2)) if (commanded is not None and ahrs2 is not None) else None
    print(
        f"PRECONDITION t={elapsed_s:.1f}s tx_yaw_deg={commanded} ahrs2_yaw_deg={ahrs2} "
        f"wrapped_err_deg={err} ekf_flags={ekf_flags} consecutive_good={consecutive} "
        f"gate_state={'PASS' if consecutive >= YAW_LOCK_CONSECUTIVE_REQUIRED else 'WAITING'} reason={reason}"
    )


def run_precondition_and_profile(profile_csv, bridge_log_csv, trajectory_dt_s=0.1,
                                  precondition_max_duration_s=PRECONDITION_MAX_DURATION_S,
                                  profile_duration_s=PROFILE_DURATION_S,
                                  gate_poll_interval_s=1.0, alive_check=None,
                                  sleep_fn=time.sleep, time_fn=time.time,
                                  start_lat=FIXED_LAT, start_lon=FIXED_LON):
    """HIL-F23-F2B3B: writes moving preconditioning rows (t_s<0) to
    profile_csv in real time while polling bridge_log_csv for the
    yaw-lock gate (evaluate_precondition_gate_samples()). On gate PASS,
    immediately continues writing the real A-F profile (t_s>=0) to the
    SAME file, inheriting position from wherever preconditioning ended
    (item 1: "lat/lon integrated continuously"). On timeout (item 5), the
    whole run aborts -- no A-F profile rows are written at all.

    Returns a dict: gate_passed, gate_reason, precondition_elapsed_s,
    rows_written, profile_start_lat, profile_start_lon.
    """
    profile_csv = Path(profile_csv)
    profile_csv.parent.mkdir(parents=True, exist_ok=True)

    precondition_trajectory = generate_precondition_trajectory(
        dt_s=trajectory_dt_s, max_duration_s=precondition_max_duration_s,
        start_lat=start_lat, start_lon=start_lon,
    )

    rows_written = 0
    gate_passed = False
    gate_reason = "not evaluated"
    consecutive = 0
    last_row = None

    with open(profile_csv, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(PROFILE_HEADER)
        f.flush()
        start_wall = time_fn()
        next_gate_check_wall = start_wall

        for row in precondition_trajectory:
            if alive_check is not None:
                abort_message = alive_check()
                if abort_message is not None:
                    return {
                        "gate_passed": False, "gate_reason": f"aborted: {abort_message}",
                        "precondition_elapsed_s": time_fn() - start_wall, "rows_written": rows_written,
                        "profile_start_lat": last_row["lat_deg"] if last_row else start_lat,
                        "profile_start_lon": last_row["lon_deg"] if last_row else start_lon,
                    }
            # pacing anchor: t_s is negative (elapsed - max_duration), so
            # add max_duration back to get elapsed-since-precondition-start.
            target_wall = start_wall + (row["t_s"] + precondition_max_duration_s)
            now = time_fn()
            if target_wall > now:
                sleep_fn(target_wall - now)
            writer.writerow(trajectory_row_to_csv_row(row))
            f.flush()
            rows_written += 1
            last_row = row

            now = time_fn()
            if now >= next_gate_check_wall:
                samples = read_bridge_log_tail_samples(bridge_log_csv)
                gate_passed, gate_reason, consecutive = evaluate_precondition_gate_samples(samples)
                print_precondition_status(now - start_wall, samples[-1] if samples else {}, consecutive, gate_reason)
                next_gate_check_wall = now + gate_poll_interval_s
                if gate_passed:
                    break

            if (time_fn() - start_wall) >= precondition_max_duration_s:
                break

        precondition_elapsed_s = time_fn() - start_wall
        profile_start_lat = last_row["lat_deg"] if last_row else start_lat
        profile_start_lon = last_row["lon_deg"] if last_row else start_lon

        if not gate_passed:
            return {
                "gate_passed": False, "gate_reason": gate_reason,
                "precondition_elapsed_s": precondition_elapsed_s, "rows_written": rows_written,
                "profile_start_lat": profile_start_lat, "profile_start_lon": profile_start_lon,
            }

        # gate PASS -- immediately continue with the real A-F profile,
        # inheriting position from wherever preconditioning ended, item 6.
        real_trajectory = integrate_trajectory(
            dt_s=trajectory_dt_s, duration_s=profile_duration_s,
            start_lat=profile_start_lat, start_lon=profile_start_lon, start_alt=BASE_ALT_M,
        )
        profile_start_wall = time_fn()
        for row in real_trajectory:
            if alive_check is not None:
                abort_message = alive_check()
                if abort_message is not None:
                    break
            target_wall = profile_start_wall + row["t_s"]
            now = time_fn()
            if target_wall > now:
                sleep_fn(target_wall - now)
            writer.writerow(trajectory_row_to_csv_row(row))
            f.flush()
            rows_written += 1

    return {
        "gate_passed": True, "gate_reason": gate_reason,
        "precondition_elapsed_s": precondition_elapsed_s, "rows_written": rows_written,
        "profile_start_lat": profile_start_lat, "profile_start_lon": profile_start_lon,
    }


def run_precondition_only(profile_csv, bridge_log_csv, duration_s=PRECONDITION_ONLY_DEFAULT_DURATION_S,
                           trajectory_dt_s=0.1, gate_poll_interval_s=1.0, alive_check=None,
                           sleep_fn=time.sleep, time_fn=time.time,
                           start_lat=FIXED_LAT, start_lon=FIXED_LON):
    """HIL-F23-F2B3C item 6: --precondition-only mode. Flies the same
    moving GPS-yaw preconditioning leg (heading=315.091444 deg,
    groundspeed=10 m/s, GPS yaw enabled) for the FULL requested duration
    (default 20s) -- unlike run_precondition_and_profile(), this never
    transitions into the A-F profile regardless of gate outcome, and
    does NOT stop early just because the gate passed (it is a diagnostic
    observation window, not a precondition-then-proceed gate). The same
    gate (evaluate_precondition_gate_samples()) and the same per-sample
    logging (print_precondition_status()) are reused unchanged.

    Returns a dict: gate_passed (whether the gate was met at any point
    during the window), gate_reason, first_gate_pass_elapsed_s (None if
    never met), elapsed_s, rows_written, startup_report (see
    build_startup_report()).
    """
    profile_csv = Path(profile_csv)
    profile_csv.parent.mkdir(parents=True, exist_ok=True)

    trajectory = generate_precondition_trajectory(
        dt_s=trajectory_dt_s, max_duration_s=duration_s, start_lat=start_lat, start_lon=start_lon,
    )
    # re-anchor t_s to count 0..duration_s (not negative) -- there is no
    # A-F profile after this to protect from an advancing timer, so the
    # negative-time convention isn't needed; kept easy to read in logs.
    for row in trajectory:
        row["t_s"] = round(row["t_s"] + duration_s, 6)

    rows_written = 0
    gate_passed = False
    gate_reason = "not evaluated"
    first_gate_pass_elapsed_s = None

    with open(profile_csv, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(PROFILE_HEADER)
        f.flush()
        start_wall = time_fn()
        next_gate_check_wall = start_wall

        for row in trajectory:
            if alive_check is not None:
                abort_message = alive_check()
                if abort_message is not None:
                    gate_reason = f"aborted: {abort_message}"
                    break
            target_wall = start_wall + row["t_s"]
            now = time_fn()
            if target_wall > now:
                sleep_fn(target_wall - now)
            writer.writerow(trajectory_row_to_csv_row(row))
            f.flush()
            rows_written += 1

            now = time_fn()
            if now >= next_gate_check_wall:
                samples = read_bridge_log_tail_samples(bridge_log_csv)
                passed_now, gate_reason, consecutive = evaluate_precondition_gate_samples(samples)
                print_precondition_status(now - start_wall, samples[-1] if samples else {}, consecutive, gate_reason)
                next_gate_check_wall = now + gate_poll_interval_s
                if passed_now and not gate_passed:
                    gate_passed = True
                    first_gate_pass_elapsed_s = now - start_wall

        elapsed_s = time_fn() - start_wall
        final_samples = read_bridge_log_tail_samples(bridge_log_csv)

    return {
        "gate_passed": gate_passed, "gate_reason": gate_reason,
        "first_gate_pass_elapsed_s": first_gate_pass_elapsed_s,
        "elapsed_s": elapsed_s, "rows_written": rows_written,
        "startup_report": build_startup_report(final_samples),
    }


def build_startup_report(samples):
    """HIL-F23-F2B3C item 7 (pure function): given the ordered bridge-log
    samples observed so far, find the elapsed-sample-index of the FIRST
    occurrence of each required startup milestone: first GPS_RAW_INT
    (fix_type present), first AHRS2 (yaw present), first horizontal-
    position-valid EKF flag (POS_HORIZ_REL or POS_HORIZ_ABS), first yaw
    convergence (single sample, wrapped error <= YAW_LOCK_TOLERANCE_DEG --
    NOT requiring the 3-consecutive streak the PASS gate itself needs).
    Returns a dict of {milestone: sample_dict_or_None}.
    """
    report = {
        "first_gps_raw_int": None,
        "first_ahrs2": None,
        "first_pos_horiz_valid": None,
        "first_yaw_converged": None,
    }
    for s in samples:
        if report["first_gps_raw_int"] is None and s.get("gps_fix_type") is not None:
            report["first_gps_raw_int"] = s
        if report["first_ahrs2"] is None and s.get("ahrs2_yaw_deg") is not None:
            report["first_ahrs2"] = s
        ekf_flags = s.get("ekf_flags")
        if report["first_pos_horiz_valid"] is None and ekf_flags is not None and (int(ekf_flags) & EKF_POS_HORIZ_VALID_MASK) != 0:
            report["first_pos_horiz_valid"] = s
        if report["first_yaw_converged"] is None and s.get("ahrs2_yaw_deg") is not None and s.get("commanded_yaw_deg") is not None:
            err = abs(smh.wrap_angle_error_deg(s["commanded_yaw_deg"], s["ahrs2_yaw_deg"]))
            if err <= YAW_LOCK_TOLERANCE_DEG:
                report["first_yaw_converged"] = s
    return report


def print_startup_report(report, preflight_details):
    print("=== HIL-F23-F2B3C startup report ===")
    print("EK3 source parameters (read-only, from preflight):")
    for name, _expected in COMBINED_PREFLIGHT_PARAM_CHECKS:
        print(f"  {name} = {preflight_details.get(name)}")
    print(f"EKF_STATUS_REPORT.flags before GPS_INPUT = {preflight_details.get('ekf_flags_before_gps_input')}")
    for label, key in (
        ("first GPS_RAW_INT", "first_gps_raw_int"),
        ("first AHRS2", "first_ahrs2"),
        ("first horizontal-position-valid EKF flag", "first_pos_horiz_valid"),
        ("first yaw convergence (<=2 deg, single sample)", "first_yaw_converged"),
    ):
        sample = report.get(key)
        if sample is None:
            print(f"  {label}: NEVER OBSERVED")
        else:
            print(f"  {label}: t={sample.get('t')} ahrs2_yaw_deg={sample.get('ahrs2_yaw_deg')} "
                  f"ekf_flags={sample.get('ekf_flags')} gps_fix_type={sample.get('gps_fix_type')}")


def build_bridge_cmd_for_nav_profile(pixhawk, baud, state_csv, mp_out, bridge_log_csv,
                                      bridge_script=BRIDGE_SCRIPT, python_exe=None):
    """Pure command-list builder. GPS_INPUT only: --gps-input-inject and
    --gps-input-send-yaw are enabled; --attitude-inject, --airspeed-
    inject, and --allow-actuator-output are never included.
    """
    cmd = [
        python_exe or sys.executable, str(bridge_script),
        "--pixhawk", str(pixhawk), "--baud", str(baud),
        "--jsbsim-state-input", str(state_csv),
        "--gps-input-inject", "--gps-input-send-yaw",
        "--log", str(bridge_log_csv),
    ]
    if mp_out:
        cmd += ["--mp-out", str(mp_out)]
    return cmd


# ---------------------------------------------------------------------------
# Preflight: heartbeat + armed/mode + full EK3_SRC1_*/GPS1_TYPE parameter audit
# ---------------------------------------------------------------------------

COMBINED_PREFLIGHT_PARAM_CHECKS = (
    ("GPS1_TYPE", 14),
    ("EK3_SRC1_POSXY", 3),
    ("EK3_SRC1_VELXY", 3),
    ("EK3_SRC1_VELZ", 3),
    ("EK3_SRC1_POSZ", 3),
    ("EK3_SRC1_YAW", 2),
)


def read_params_readonly(master, names, timeout_s=4.0, retries=3):
    """Read-only PARAM_REQUEST_READ helper (mirrors the established
    pattern in sr75_hil_gps_ekf_readonly_audit.py). Returns
    {name: value_or_None}. Never sends PARAM_SET.
    """
    results = {}
    remaining = list(names)
    for _attempt in range(retries):
        if not remaining:
            break
        for name in remaining:
            master.mav.param_request_read_send(
                master.target_system, master.target_component, name.encode("utf-8"), -1,
            )
        deadline = time.time() + timeout_s
        still_missing = set(remaining)
        while time.time() < deadline and still_missing:
            msg = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
            if msg is None:
                continue
            pid = msg.param_id.rstrip("\x00") if isinstance(msg.param_id, str) else msg.param_id
            if pid in still_missing:
                results[pid] = msg.param_value
                still_missing.discard(pid)
        remaining = list(still_missing)
    for name in remaining:
        results[name] = None
    return results


def evaluate_combined_preflight(armed, mode, param_values):
    """Pure function: evaluate the task's 9 preflight conditions against
    already-collected heartbeat/param values. Returns (ok, failures_list).
    Separated from preflight_checks_combined() so the pass/fail logic
    itself is unit-testable without a MAVLink connection.
    """
    failures = []
    if armed:
        failures.append("vehicle reports ARMED")
    if mode != "MANUAL":
        failures.append(f"mode={mode}, expected MANUAL")
    for name, expected in COMBINED_PREFLIGHT_PARAM_CHECKS:
        value = param_values.get(name)
        if value is None:
            failures.append(f"{name} did not respond to PARAM_REQUEST_READ")
        elif int(value) != expected:
            failures.append(f"{name}={int(value)}, expected {expected}")
    return (len(failures) == 0), failures


def preflight_checks_combined(pixhawk_device, baud, param_timeout_s=4.0, param_retries=3):
    """Read-only heartbeat + armed/mode + full EK3_SRC1_*/GPS1_TYPE
    parameter audit. Returns (ok, message, details_dict). Never sends
    PARAM_SET, arm, mission, RC override, or actuator commands. Aborts
    (returns ok=False) if ANY of the 9 required conditions differ.
    """
    from pymavlink import mavutil

    print(f"Preflight: connecting to {pixhawk_device} at {baud} baud (read-only)...")
    master = mavutil.mavlink_connection(pixhawk_device, baud=baud, autoreconnect=True)
    hb = master.wait_heartbeat(timeout=30)
    if hb is None:
        master.close()
        return False, "no heartbeat received within 30s", {}

    armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    mode = mavutil.mode_string_v10(hb)
    print(f"Preflight heartbeat OK: type={hb.type} autopilot={hb.autopilot} armed={int(armed)} mode={mode}")

    names = [name for name, _expected in COMBINED_PREFLIGHT_PARAM_CHECKS]
    param_values = read_params_readonly(master, names, timeout_s=param_timeout_s, retries=param_retries)
    for name, value in param_values.items():
        print(f"Preflight {name} = {value}")

    # HIL-F23-F2B3C item 7: read-only EKF_STATUS_REPORT.flags snapshot
    # BEFORE any GPS_INPUT has been sent, so the startup report can show
    # the true "before" state (whatever compass/baro-only fusion mode the
    # EKF was already in) alongside the "after preconditioning" state.
    ekf_flags_before_gps_input = None
    deadline = time.time() + 3.0
    while time.time() < deadline:
        msg = master.recv_match(type="EKF_STATUS_REPORT", blocking=True, timeout=0.5)
        if msg is not None:
            ekf_flags_before_gps_input = msg.flags
            break
    print(f"Preflight EKF_STATUS_REPORT.flags (before GPS_INPUT) = {ekf_flags_before_gps_input}")
    master.close()

    ok, failures = evaluate_combined_preflight(armed, mode, param_values)
    details = {
        "armed": int(armed), "mode": mode, "ekf_flags_before_gps_input": ekf_flags_before_gps_input,
        **param_values,
    }
    if not ok:
        return False, "; ".join(failures), details
    return True, "all preflight checks passed", details


# ---------------------------------------------------------------------------
# Post-run phase summary + PASS criteria
# ---------------------------------------------------------------------------

def _f(row, key):
    v = row.get(key, "")
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def compute_phase_summary(bridge_log_csv, stale_gap_threshold_s=2.0, ned_z_jump_threshold_m=5.0):
    """Post-process the bridge's own CSV log into the required per-phase
    summary.

    HIL-F23-F2B3A root-cause fix: phase is now classified strictly from
    the COMMANDED PROFILE TIME (the bridge's own jsb_feed_time_s column,
    i.e. the elapsed time the profile writer itself stamped into the
    synthetic state CSV and that the feeder logged verbatim) via
    classify_phase_from_time(), using the exact ranges A[0,10) B[10,30)
    C[30,50) D[50,70) E[70,90) F[90,100].

    The previous approach (classify_phase_from_commanded(), value-based
    on gps_tx_alt_m/gps_tx_vd_mps) could not distinguish a stationary
    hold (A, F: alt=3000m, vd=0) from cruising through the same altitude
    (B: alt=3000m, vd=0) -- both collapsed into one "A" bucket, so B and
    F vanished entirely and A absorbed A+B+F's samples (confirmed against
    a retained hardware log: old classifier produced A=39 B=0 C=19 D=20
    E=20 F=0, exactly the reported symptom; the fix below recovers
    A=9 B=20 C=19 D=20 E=20 F=10 against that same file).

    bridge_log_csv's own wall-clock time_s is deliberately NOT used for
    classification -- it is offset from the profile writer's own t_s by
    the bridge/preflight startup delay (observed ~1-2s in that same
    retained log) plus drift, which would misclassify rows near every
    phase boundary.

    Pure function, no hardware/process access.
    """
    with open(bridge_log_csv, newline="") as f:
        rows = list(csv.DictReader(f))

    samples = []
    prev_t = None
    stale_gaps = []
    excluded_no_commanded_time = 0
    for row in rows:
        t = _f(row, "time_s")
        commanded_t = _f(row, "jsb_feed_time_s")
        commanded_lat = _f(row, "gps_tx_lat_deg")
        commanded_lon = _f(row, "gps_tx_lon_deg")
        commanded_alt = _f(row, "gps_tx_alt_m")
        commanded_vn = _f(row, "gps_tx_vn_mps")
        commanded_ve = _f(row, "gps_tx_ve_mps")
        commanded_vd = _f(row, "gps_tx_vd_mps")
        commanded_yaw_cdeg = _f(row, "gps_tx_yaw_cdeg")
        # HIL-F23-F2B3A item 3: exclude rows without a valid commanded/
        # profile time -- these precede the first live producer row and
        # cannot be attributed to any phase.
        if commanded_t is None:
            excluded_no_commanded_time += 1
            continue
        if t is None or commanded_alt is None or commanded_vd is None:
            continue
        if prev_t is not None and (t - prev_t) > stale_gap_threshold_s:
            stale_gaps.append({"t": t, "gap_s": t - prev_t})
        prev_t = t

        phase = classify_phase_from_time(commanded_t)
        if phase is None:
            continue
        commanded_gs = math.hypot(commanded_vn, commanded_ve) if (commanded_vn is not None and commanded_ve is not None) else None
        commanded_yaw_deg = smh.decode_gps_input_yaw_cdeg(int(commanded_yaw_cdeg)) if commanded_yaw_cdeg is not None else None

        global_lat = _f(row, "lat_deg")
        global_lon = _f(row, "lon_deg")
        global_alt = _f(row, "alt_m")
        gps_raw_lat = _f(row, "gps_raw_lat_deg")
        gps_raw_lon = _f(row, "gps_raw_lon_deg")
        gps_raw_alt = _f(row, "gps_raw_alt_m")
        groundspeed = _f(row, "groundspeed_mps")
        ahrs2_yaw_rad = _f(row, "ahrs2_yaw_rad")
        ahrs2_yaw_deg = math.degrees(ahrs2_yaw_rad) if ahrs2_yaw_rad is not None else None

        horiz_err_global_m = None
        if None not in (commanded_lat, commanded_lon, global_lat, global_lon):
            horiz_err_global_m = smh.great_circle_distance_m(commanded_lat, commanded_lon, global_lat, global_lon)
        horiz_err_gps_raw_m = None
        if None not in (commanded_lat, commanded_lon, gps_raw_lat, gps_raw_lon):
            horiz_err_gps_raw_m = smh.great_circle_distance_m(commanded_lat, commanded_lon, gps_raw_lat, gps_raw_lon)

        alt_err_global_m = abs(global_alt - commanded_alt) if global_alt is not None else None
        alt_err_gps_raw_m = abs(gps_raw_alt - commanded_alt) if gps_raw_alt is not None else None
        gs_err_mps = abs(groundspeed - commanded_gs) if (groundspeed is not None and commanded_gs is not None) else None
        yaw_err_deg = None
        if commanded_yaw_deg is not None and ahrs2_yaw_deg is not None:
            yaw_err_deg = smh.wrap_angle_error_deg(commanded_yaw_deg, ahrs2_yaw_deg)

        samples.append({
            "t": t, "commanded_t": commanded_t, "phase": phase,
            "mode": row.get("mode", ""), "armed": row.get("armed", ""),
            "servo7_pwm": _f(row, "servo7_pwm"), "servo8_pwm": _f(row, "servo8_pwm"),
            "rc7_pwm": _f(row, "rc7_pwm"), "rc8_pwm": _f(row, "rc8_pwm"),
            "horiz_err_global_m": horiz_err_global_m,
            "horiz_err_gps_raw_m": horiz_err_gps_raw_m,
            "alt_err_global_m": alt_err_global_m,
            "alt_err_gps_raw_m": alt_err_gps_raw_m,
            "gs_err_mps": gs_err_mps,
            "yaw_err_deg": abs(yaw_err_deg) if yaw_err_deg is not None else None,
            "ekf_flags": _f(row, "ekf_flags"),
            "local_ned_z_m": _f(row, "local_ned_z_m"),
            "gps_raw_fix_type": _f(row, "gps_raw_fix_type"),
        })

    per_phase = {}
    for phase in PHASES_IN_ORDER:
        phase_samples = [s for s in samples if s["phase"] == phase]

        def _stats(key):
            values = [s[key] for s in phase_samples if s[key] is not None]
            if not values:
                return {"n": 0, "max": None, "mean": None}
            return {"n": len(values), "max": max(values), "mean": sum(values) / len(values)}

        commanded_times = [s["commanded_t"] for s in phase_samples]
        per_phase[phase] = {
            "n_samples": len(phase_samples),
            "first_t": min(commanded_times) if commanded_times else None,
            "last_t": max(commanded_times) if commanded_times else None,
            "horiz_err_global_m": _stats("horiz_err_global_m"),
            "horiz_err_gps_raw_m": _stats("horiz_err_gps_raw_m"),
            "alt_err_global_m": _stats("alt_err_global_m"),
            "alt_err_gps_raw_m": _stats("alt_err_gps_raw_m"),
            "gs_err_mps": _stats("gs_err_mps"),
            "yaw_err_deg": _stats("yaw_err_deg"),
        }

    discontinuities = []
    phase_boundary_times = (0.0, T_A_END, T_B_END, T_C_END, T_D_END, T_E_END, T_F_END)

    def _nearest_boundary_distance(t):
        if t is None:
            return None
        return min(abs(t - b) for b in phase_boundary_times)

    prev_flags = None
    prev_ned_z = None
    for s in samples:
        if prev_flags is not None and s["ekf_flags"] is not None and s["ekf_flags"] != prev_flags:
            discontinuities.append({
                "t": s["t"], "commanded_t": s["commanded_t"], "phase": s["phase"],
                "type": "ekf_flags_change", "from": prev_flags, "to": s["ekf_flags"],
                "near_phase_boundary": (_nearest_boundary_distance(s["commanded_t"]) or 999) <= 2.0,
            })
        if prev_ned_z is not None and s["local_ned_z_m"] is not None:
            jump = abs(s["local_ned_z_m"] - prev_ned_z)
            if jump >= ned_z_jump_threshold_m:
                discontinuities.append({
                    "t": s["t"], "commanded_t": s["commanded_t"], "phase": s["phase"],
                    "type": "local_ned_z_jump", "from": prev_ned_z, "to": s["local_ned_z_m"], "jump_m": jump,
                    "near_phase_boundary": (_nearest_boundary_distance(s["commanded_t"]) or 999) <= 2.0,
                })
        if s["ekf_flags"] is not None:
            prev_flags = s["ekf_flags"]
        if s["local_ned_z_m"] is not None:
            prev_ned_z = s["local_ned_z_m"]

    non_manual = [s for s in samples if s["mode"] not in ("", "MANUAL")]
    armed_rows = [s for s in samples if s["armed"] not in ("", "0", 0)]
    ch_nonzero = [
        s for s in samples
        if any(
            (s[k] is not None and s[k] not in (0, 0.0))
            for k in ("servo7_pwm", "servo8_pwm", "rc7_pwm", "rc8_pwm")
        )
    ]

    # HIL-F23-F2B3A item 9: GPS-health audit fields -- distinct ekf_flags
    # values seen and any non-3D-fix GPS_RAW_INT samples, so "Unhealthy
    # GPS Signal" can be correlated against actual transmitted/received
    # state rather than guessed at.
    distinct_ekf_flags = sorted({int(s["ekf_flags"]) for s in samples if s["ekf_flags"] is not None})
    non_3d_fix_samples = [s for s in samples if s["gps_raw_fix_type"] is not None and s["gps_raw_fix_type"] != 3]

    return {
        "n_samples": len(samples),
        "excluded_no_commanded_time": excluded_no_commanded_time,
        "per_phase": per_phase,
        "discontinuities": discontinuities,
        "stale_gaps": stale_gaps,
        "distinct_ekf_flags": distinct_ekf_flags,
        "non_3d_fix_samples": len(non_3d_fix_samples),
        "non_manual_samples": len(non_manual),
        "armed_samples": len(armed_rows),
        "ch7_ch8_nonzero_samples": len(ch_nonzero),
    }


def classify_phase_from_time(commanded_t_s, vd_eps=0.1):
    """Classify which of A-F a logged row belongs to, strictly from the
    COMMANDED PROFILE TIME (bridge's jsb_feed_time_s column): A[0,10)
    B[10,30) C[30,50) D[50,70) E[70,90) F[90,100]. Returns None for a
    missing or out-of-range time (before 0 or after the profile's 100s
    duration) -- HIL-F23-F2B3A item 3, "exclude rows without valid
    commanded/profile time."

    HIL-F23-F2B3A root cause: the original classify_phase_from_commanded()
    (still defined below, kept only for the regression test that proves
    the old bug) used COMMANDED VALUES (gps_tx_alt_m/gps_tx_vd_mps)
    instead of commanded TIME. Those values cannot distinguish a
    stationary hold (A, F: alt=3000m, vd=0) from cruising through the
    same altitude (B: alt=3000m, vd=0) -- all three collapsed into one
    "A" bucket, so B and F vanished and A absorbed their samples. This
    was confirmed against a retained hardware log (100 rows): the old
    classifier reproduced the exact reported symptom (A=39 B=0 C=19
    D=20 E=20 F=0); this time-based classifier recovers A=9 B=20 C=19
    D=20 E=20 F=10 against that same file (see
    test_sr75_hil_f2b3_combined_nav_profile.py's regression tests).
    """
    if commanded_t_s is None:
        return None
    if commanded_t_s < 0.0:
        return None
    if commanded_t_s < T_A_END:
        return "A"
    if commanded_t_s < T_B_END:
        return "B"
    if commanded_t_s < T_C_END:
        return "C"
    if commanded_t_s < T_D_END:
        return "D"
    if commanded_t_s < T_E_END:
        return "E"
    if commanded_t_s <= T_F_END:
        return "F"
    return None


def classify_phase_from_commanded(commanded_alt_m, commanded_vd_mps, vd_eps=0.1):
    """HIL-F23-F2B3A: RETAINED ONLY as the historical/buggy classifier
    for the regression test that proves the bug it caused (see
    classify_phase_from_time(), which compute_phase_summary() now uses
    instead). Do not use this for real classification -- it cannot
    distinguish a stationary hold (A, F) from cruising through the same
    altitude (B); see classify_phase_from_time()'s docstring for the
    full root-cause explanation.
    """
    if commanded_alt_m is None or commanded_vd_mps is None:
        return None
    if commanded_vd_mps <= -vd_eps:
        return "C"
    if commanded_vd_mps >= vd_eps:
        return "E"
    midpoint = (BASE_ALT_M + PEAK_ALT_M) / 2.0
    return "D" if commanded_alt_m >= midpoint else "A"


PASS_THRESHOLDS = {
    "horizontal_error_max_m": 15.0,
    "altitude_error_settled_max_m": 2.0,
    "groundspeed_error_max_mps": 2.0,
    "yaw_error_settled_max_deg": 2.0,
}


def evaluate_pass_criteria(summary, thresholds=None):
    """Pure function: task item 7's PASS criteria, evaluated against a
    compute_phase_summary() result. Returns {criterion: (ok, detail)}
    plus "overall".
    """
    thresholds = thresholds or PASS_THRESHOLDS
    results = {}

    all_horiz = [summary["per_phase"][p]["horiz_err_global_m"]["max"] for p in PHASES_IN_ORDER]
    all_horiz = [v for v in all_horiz if v is not None]
    max_horiz = max(all_horiz) if all_horiz else None
    results["horizontal_position_bounded"] = (
        max_horiz is not None and max_horiz <= thresholds["horizontal_error_max_m"],
        f"max_horiz_error_m={max_horiz}",
    )

    settled_alt_errors = [
        summary["per_phase"][p]["alt_err_global_m"]["max"]
        for p in HOLD_ALT_PHASES if summary["per_phase"][p]["alt_err_global_m"]["max"] is not None
    ]
    max_settled_alt_err = max(settled_alt_errors) if settled_alt_errors else None
    results["altitude_error_settled"] = (
        max_settled_alt_err is not None and max_settled_alt_err <= thresholds["altitude_error_settled_max_m"],
        f"max_settled_alt_err_m={max_settled_alt_err}",
    )

    all_gs_errors = [summary["per_phase"][p]["gs_err_mps"]["max"] for p in PHASES_IN_ORDER]
    all_gs_errors = [v for v in all_gs_errors if v is not None]
    max_gs_err = max(all_gs_errors) if all_gs_errors else None
    results["groundspeed_error"] = (
        max_gs_err is not None and max_gs_err <= thresholds["groundspeed_error_max_mps"],
        f"max_gs_err_mps={max_gs_err}",
    )

    settled_yaw_errors = [
        summary["per_phase"][p]["yaw_err_deg"]["max"]
        for p in HOLD_YAW_PHASES if summary["per_phase"][p]["yaw_err_deg"]["max"] is not None
    ]
    max_settled_yaw_err = max(settled_yaw_errors) if settled_yaw_errors else None
    results["yaw_error_settled"] = (
        max_settled_yaw_err is not None and max_settled_yaw_err <= thresholds["yaw_error_settled_max_deg"],
        f"max_settled_yaw_err_deg={max_settled_yaw_err}",
    )

    ekf_changes = [d for d in summary["discontinuities"] if d["type"] == "ekf_flags_change"]
    results["ekf_flag_stability"] = (len(ekf_changes) == 0, f"ekf_flags_changes={len(ekf_changes)}")

    ned_jumps = [d for d in summary["discontinuities"] if d["type"] == "local_ned_z_jump"]
    results["no_local_ned_discontinuity"] = (len(ned_jumps) == 0, f"local_ned_z_jumps={len(ned_jumps)}")

    results["no_stale_retransmission"] = (len(summary["stale_gaps"]) == 0, f"stale_gaps={len(summary['stale_gaps'])}")

    results["manual_disarmed_throughout"] = (
        summary["non_manual_samples"] == 0 and summary["armed_samples"] == 0,
        f"non_manual_samples={summary['non_manual_samples']} armed_samples={summary['armed_samples']}",
    )

    results["ch7_ch8_zero_no_actuator_output"] = (
        summary["ch7_ch8_nonzero_samples"] == 0,
        f"ch7_ch8_nonzero_samples={summary['ch7_ch8_nonzero_samples']}",
    )

    overall = all(ok for ok, _detail in results.values())
    results["overall"] = (overall, "PASS" if overall else "FAIL")
    return results


def print_phase_summary(summary):
    print("=== HIL-F23-F2B3 per-phase combined nav summary ===")
    print(f"samples={summary['n_samples']} excluded_no_commanded_time={summary['excluded_no_commanded_time']}")
    # HIL-F23-F2B3A item 4: per-phase first/last commanded-profile-time
    # timestamp and sample count, printed explicitly so a misclassification
    # (like the one this task fixed) is immediately visible again if it recurs.
    for phase in PHASES_IN_ORDER:
        entry = summary["per_phase"][phase]
        print(f"  phase={phase}: n={entry['n_samples']} first_t={entry['first_t']} last_t={entry['last_t']}")
    print("")
    for phase in PHASES_IN_ORDER:
        entry = summary["per_phase"][phase]
        # HIL-F23-F2B3A item 6: GPS_RAW transport error, GLOBAL_POSITION/
        # EKF error, and AHRS2 yaw error printed as separate lines.
        print(f"  phase={phase} n={entry['n_samples']} "
              f"horiz_err_gps_raw_max_m={entry['horiz_err_gps_raw_m']['max']} "
              f"alt_err_gps_raw_max_m={entry['alt_err_gps_raw_m']['max']}  (GPS_RAW transport)")
        print(f"  phase={phase} n={entry['n_samples']} "
              f"horiz_err_global_max_m={entry['horiz_err_global_m']['max']} "
              f"alt_err_global_max_m={entry['alt_err_global_m']['max']} "
              f"gs_err_max_mps={entry['gs_err_mps']['max']}  (GLOBAL_POSITION/EKF)")
        print(f"  phase={phase} n={entry['n_samples']} "
              f"yaw_err_max_deg={entry['yaw_err_deg']['max']}  (AHRS2 yaw vs transmitted GPS yaw)")
    print(f"discontinuities={len(summary['discontinuities'])} stale_gaps={len(summary['stale_gaps'])} "
          f"non_manual_samples={summary['non_manual_samples']} armed_samples={summary['armed_samples']} "
          f"ch7_ch8_nonzero_samples={summary['ch7_ch8_nonzero_samples']}")
    print(f"distinct_ekf_flags={summary['distinct_ekf_flags']} non_3d_fix_samples={summary['non_3d_fix_samples']}")
    if summary["discontinuities"]:
        print("DISCONTINUITIES:")
        for d in summary["discontinuities"]:
            print(f"  t={d['t']} commanded_t={d['commanded_t']} phase={d['phase']} type={d['type']} "
                  f"from={d['from']} to={d['to']} near_phase_boundary={d['near_phase_boundary']}")


def print_pass_criteria(results):
    print("=== HIL-F23-F2B3 PASS criteria ===")
    for name, (ok, detail) in results.items():
        if name == "overall":
            continue
        print(f"  {'PASS' if ok else 'FAIL'}: {name} ({detail})")
    overall_ok, overall_detail = results["overall"]
    print(f"OVERALL: {overall_detail}")


def build_arg_parser():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--pixhawk", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--run-dir", default="/tmp/sr75_hil_f2b3")
    parser.add_argument("--mp-out", default=DEFAULT_MP_OUT,
                         help="Mission Planner MAVLink output, forwarded unchanged to the bridge's --mp-out. "
                              "Pass an empty string to disable forwarding.")
    parser.add_argument("--profile-rate-hz", type=float, default=10.0,
                         help="Rate at which the synthetic nav-profile CSV rows are appended/integrated")
    parser.add_argument("--duration-s", type=float, default=PROFILE_DURATION_S,
                         help="Override the observation window (defaults to the full 100s profile)")
    parser.add_argument("--readiness-timeout-s", type=float, default=10.0)
    parser.add_argument("--precondition-max-duration-s", type=float, default=PRECONDITION_MAX_DURATION_S,
                         help="Max time to wait for the moving GPS-yaw-lock gate before aborting (task cap: 15s)")
    parser.add_argument("--dry-run", action="store_true",
                         help="Print the trajectory plan and exact commands; touch no hardware, start no subprocess.")
    parser.add_argument("--producer-only", action="store_true",
                         help="Run the synthetic nav-profile writer alone (no Pixhawk, no bridge) and print a "
                              "summary of the generated CSV. Zero hardware involvement.")
    parser.add_argument("--precondition-only", action="store_true",
                         help="HIL-F23-F2B3C: run ONLY the moving GPS-yaw preconditioning leg against real "
                              "hardware (heartbeat/param preflight + bridge + precondition), for the full "
                              "--precondition-only-duration-s window, then print the gate/startup report and "
                              "stop. The A-F profile is NEVER started in this mode, regardless of gate outcome.")
    parser.add_argument("--precondition-only-duration-s", type=float, default=PRECONDITION_ONLY_DEFAULT_DURATION_S,
                         help="Observation window for --precondition-only (default 20s)")
    parser.add_argument("--keep-artifacts", action="store_true",
                         help="Do not delete the generated profile CSV/console log after a successful run")
    return parser


def main():
    args = build_arg_parser().parse_args()
    run_dir = Path(args.run_dir)
    run_dir.mkdir(parents=True, exist_ok=True)
    profile_csv = run_dir / "sr75_hil_f2b3_nav_profile.csv"
    run_succeeded = False

    print("=== HIL-F23-F2B3: controlled combined GPS navigation-state profile ===")
    print(f"start lat={FIXED_LAT:.7f} lon={FIXED_LON:.7f} alt={BASE_ALT_M:.0f}m yaw={FIXED_YAW_DEG:.6f} deg")
    print(f"phases: A hold {PHASE_A_S:.0f}s -> B NW@10m/s {PHASE_B_S:.0f}s -> "
          f"C turn 315->270 + climb {PHASE_C_S:.0f}s -> D west@10m/s {PHASE_D_S:.0f}s -> "
          f"E turn 270->315 + descend {PHASE_E_S:.0f}s -> F hold {PHASE_F_S:.0f}s "
          f"(total {PROFILE_DURATION_S:.0f}s)")
    print(f"profile CSV: {profile_csv}")
    print(f"Mission Planner forwarding (--mp-out): {args.mp_out or '(disabled)'}")
    print("GPS_INPUT: position/velocity/altitude/yaw (--gps-input-inject --gps-input-send-yaw)")
    print("Attitude/airspeed display injection: DISABLED (GPS_INPUT only, per task spec)")
    print(f"PRECONDITIONING (before A-F timer starts): moving leg heading={PRECONDITION_HEADING_DEG:.6f} deg "
          f"groundspeed={PRECONDITION_GROUNDSPEED_MPS:.1f} m/s, up to {args.precondition_max_duration_s:.0f}s; "
          f"gate: wrapped AHRS2 yaw error <= {YAW_LOCK_TOLERANCE_DEG:.0f} deg for "
          f"{YAW_LOCK_CONSECUTIVE_REQUIRED} consecutive samples AND EKF_POS_HORIZ_REL/ABS set AND GPS fix=3; "
          f"abort if not met within the cap")

    if args.dry_run:
        print("")
        print("DRY RUN: stopping before any hardware connection or subprocess launch.")
        return 0

    try:
        if args.producer_only:
            print("")
            print("PRODUCER-ONLY: writing the synthetic nav profile alone, no Pixhawk, no bridge.")
            trajectory = integrate_trajectory(dt_s=1.0 / max(args.profile_rate_hz, 0.1), duration_s=args.duration_s)
            rows_written = write_profile_csv_realtime(profile_csv, trajectory=trajectory, rate_hz=args.profile_rate_hz)
            print(f"rows written: {rows_written}")
            if trajectory:
                last = trajectory[-1]
                print(f"final state: t={last['t_s']:.1f}s lat={last['lat_deg']:.7f} lon={last['lon_deg']:.7f} "
                      f"alt={last['alt_m']:.2f} heading={last['heading_deg']:.3f} phase={last['phase']}")
            run_succeeded = rows_written > 0
            return 0 if run_succeeded else 2

        # preflight: heartbeat + armed/mode + full EK3_SRC1_*/GPS1_TYPE audit
        # (+ EKF_STATUS_REPORT.flags snapshot before any GPS_INPUT, item 7)
        ok, message, preflight_details = preflight_checks_combined(args.pixhawk, args.baud)
        if not ok:
            print(f"ABORT: preflight failed: {message}")
            return 2

        if args.precondition_only:
            print("")
            print(f"PRECONDITION-ONLY: moving GPS-yaw preconditioning leg only, "
                  f"{args.precondition_only_duration_s:.0f}s window. The A-F profile will NOT be started.")
            bridge_log_csv = run_dir / f"sr75_hil_f2b3_bridge_log_{time.strftime('%Y%m%d_%H%M%S')}.csv"
            bridge_console_log = run_dir / BRIDGE_CONSOLE_LOG_NAME
            bridge_cmd = build_bridge_cmd_for_nav_profile(
                args.pixhawk, args.baud, profile_csv, args.mp_out, bridge_log_csv,
            )
            print(f"Starting bridge (no-actuator-output, the default): {' '.join(bridge_cmd)}")
            bridge_stdout = open(bridge_console_log, "w")
            bridge_proc = subprocess.Popen(bridge_cmd, stdout=bridge_stdout, stderr=subprocess.STDOUT, start_new_session=True)
            try:
                time.sleep(2.0)
                early_abort = e1_launch.check_process_alive(bridge_proc, "bridge", bridge_console_log)
                if early_abort is not None:
                    print(f"ABORT: {early_abort}")
                    return 2

                def alive_check():
                    return e1_launch.check_process_alive(bridge_proc, "bridge", bridge_console_log)

                result = run_precondition_only(
                    profile_csv, bridge_log_csv, duration_s=args.precondition_only_duration_s,
                    alive_check=alive_check,
                )
            finally:
                if bridge_proc.poll() is None:
                    try:
                        os.killpg(bridge_proc.pid, signal.SIGTERM)
                    except ProcessLookupError:
                        pass
                    time.sleep(1.0)
                    if bridge_proc.poll() is None:
                        try:
                            os.killpg(bridge_proc.pid, signal.SIGKILL)
                        except ProcessLookupError:
                            pass
                bridge_stdout.close()

            print(f"gate_passed={result['gate_passed']} first_gate_pass_elapsed_s={result['first_gate_pass_elapsed_s']} "
                  f"elapsed_s={result['elapsed_s']:.1f} rows_written={result['rows_written']} "
                  f"reason={result['gate_reason']}")
            print_startup_report(result["startup_report"], preflight_details)
            run_succeeded = result["gate_passed"]
            return 0 if run_succeeded else 1

        bridge_log_csv = run_dir / f"sr75_hil_f2b3_bridge_log_{time.strftime('%Y%m%d_%H%M%S')}.csv"
        bridge_console_log = run_dir / BRIDGE_CONSOLE_LOG_NAME
        bridge_cmd = build_bridge_cmd_for_nav_profile(
            args.pixhawk, args.baud, profile_csv, args.mp_out, bridge_log_csv,
        )
        print(f"Starting bridge (no-actuator-output, the default): {' '.join(bridge_cmd)}")
        bridge_stdout = open(bridge_console_log, "w")
        bridge_proc = subprocess.Popen(bridge_cmd, stdout=bridge_stdout, stderr=subprocess.STDOUT, start_new_session=True)

        try:
            time.sleep(2.0)
            early_abort = e1_launch.check_process_alive(bridge_proc, "bridge", bridge_console_log)
            if early_abort is not None:
                print(f"ABORT: {early_abort}")
                return 2

            def alive_check():
                return e1_launch.check_process_alive(bridge_proc, "bridge", bridge_console_log)

            import threading
            writer_result = {}

            def _run_writer():
                writer_result.update(run_precondition_and_profile(
                    profile_csv, bridge_log_csv,
                    trajectory_dt_s=1.0 / max(args.profile_rate_hz, 0.1),
                    precondition_max_duration_s=args.precondition_max_duration_s,
                    profile_duration_s=args.duration_s,
                    alive_check=alive_check,
                ))

            writer_thread = threading.Thread(target=_run_writer, daemon=True)
            writer_thread.start()

            ready_ok, ready_message = e1_launch.wait_for_producer_ready(
                profile_csv, timeout_s=args.readiness_timeout_s, extra_alive_check=alive_check,
            )
            print(ready_message)
            if not ready_ok:
                print(f"ABORT: {ready_message}")
                return 2

            print(f"Preconditioning + observation window: up to "
                  f"{args.precondition_max_duration_s:.0f}s precondition + {args.duration_s:.0f}s profile")
            max_wait_s = args.precondition_max_duration_s + args.duration_s + 15.0
            obs_deadline = time.time() + max_wait_s
            while time.time() < obs_deadline and writer_thread.is_alive():
                abort_message = alive_check()
                if abort_message is not None:
                    print(f"ABORT: {abort_message}")
                    return 2
                time.sleep(1.0)
            writer_thread.join(timeout=5.0)
        finally:
            if bridge_proc.poll() is None:
                try:
                    os.killpg(bridge_proc.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
                time.sleep(1.0)
                if bridge_proc.poll() is None:
                    try:
                        os.killpg(bridge_proc.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
            bridge_stdout.close()

        # HIL-F23-F2B3B item 5: abort if the preconditioning gate never
        # passed within the cap -- no A-F profile was written in that
        # case, so there is nothing meaningful to summarize.
        if not writer_result.get("gate_passed", False):
            print(
                f"ABORT: preconditioning gate did not pass within "
                f"{args.precondition_max_duration_s:.0f}s: {writer_result.get('gate_reason')}"
            )
            print(f"precondition_elapsed_s={writer_result.get('precondition_elapsed_s')}")
            return 2

        print(
            f"Preconditioning PASS after {writer_result.get('precondition_elapsed_s', 0.0):.1f}s: "
            f"{writer_result.get('gate_reason')}"
        )

        if bridge_log_csv.exists():
            summary = compute_phase_summary(bridge_log_csv)
            print_phase_summary(summary)
            results = evaluate_pass_criteria(summary)
            print_pass_criteria(results)
            run_succeeded = results["overall"][0]
        else:
            print(f"WARNING: bridge log CSV not found at {bridge_log_csv}, cannot compute phase summary")

        return 0 if run_succeeded else 1
    finally:
        removed = e1_launch.cleanup_artifacts([profile_csv], args.keep_artifacts, run_succeeded)
        for p in removed:
            print(f"cleaned up temporary file: {p}")
        if not run_succeeded:
            print("Run did not succeed / did not PASS -- profile CSV and console log were retained for debugging.")


if __name__ == "__main__":
    sys.exit(main())
