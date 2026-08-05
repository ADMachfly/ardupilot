#!/usr/bin/env python3
"""HIL-F23-F2B2A: controlled, static-horizontal GPS altitude-ramp test.

F23-F2B2 observed that during an open-loop JSBSim descent (GPS vertical
speed reaching ~124 m/s), EK3_SRC1_POSZ=GPS altitude fusion diverged by
>400m from JSBSim, EKF_STATUS_REPORT flags toggled (167/831), and
LOCAL_POSITION_NED.z showed a reset/jump -- while GPS_INPUT transport
itself stayed valid. That confounds two questions: is GPS-altitude fusion
itself broken, or was it simply never designed for a ~124 m/s vertical
rate? This script isolates the estimator question with a slow, static
alt-only profile with NO JSBSim, NO aircraft physics, and NO horizontal
motion at all:

  - fixed lat=32.5378147 lon=74.3661871, vn=0, ve=0 throughout
  - fixed yaw=315.091444 deg (transmitted only in the sense that a
    synthetic attitude/yaw value must exist to satisfy the bridge's
    existing state-feed validation -- GPS_INPUT yaw transmission itself
    stays OFF, see --gps-input-send-yaw is never passed)
  - altitude: hold 3000m (10s) -> climb +1 m/s (20s) -> hold 3020m (10s)
    -> descend -1 m/s (20s) -> back to 3000m. vd = -1/0/+1 m/s per phase
    (NED down-positive convention: climbing is vd<0).

No JSBSim subprocess is involved -- a lightweight pure-Python generator
appends rows, in real time, to a CSV file in the exact schema the
bridge's existing (already-validated) JSBSimStateFeeder/JSB_FEED_ALIASES
machinery already parses, so this reuses the identical, already-tested
GPS_INPUT/staleness/producer-readiness pipeline built for F23-E1/E2A/E2B
with zero changes to that consumption path.

SR-75 LAYER 2 HIL BENCH SAFETY:
No live engine, no fuel pump, no live RATO ignition, no live ejection,
no arming, no AUTO, no actuator output, no CH7/CH8 activity.
GPS_INPUT yaw transmission (--gps-input-send-yaw) is intentionally never
enabled by this script. Attitude/airspeed display injection is also not
used -- this test targets GPS_INPUT altitude/velocity fusion only.
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

BRIDGE_SCRIPT = BRIDGE_DIR / "sr75_jsbsim_pixhawk_hil_bridge.py"
BRIDGE_CONSOLE_LOG_NAME = "bridge_console.log"

# Fixed test-point position/heading (task spec; same validated release
# lat/lon and mission-derived bearing used throughout this task family).
FIXED_LAT = 32.5378147
FIXED_LON = 74.3661871
FIXED_YAW_DEG = 315.091444
DEFAULT_MP_OUT = "udp:192.168.64.1:14550"

# Altitude profile (task spec): hold / climb / hold / descend, seconds
# and metres, vd in NED down-positive convention (climb => vd negative).
PHASE_HOLD1_S = 10.0
PHASE_CLIMB_S = 20.0
PHASE_HOLD2_S = 10.0
PHASE_DESCEND_S = 20.0
PROFILE_DURATION_S = PHASE_HOLD1_S + PHASE_CLIMB_S + PHASE_HOLD2_S + PHASE_DESCEND_S  # 60.0
BASE_ALT_M = 3000.0
CLIMB_RATE_MPS = 1.0
PEAK_ALT_M = BASE_ALT_M + CLIMB_RATE_MPS * PHASE_CLIMB_S  # 3020.0

T_HOLD1_END = PHASE_HOLD1_S
T_CLIMB_END = T_HOLD1_END + PHASE_CLIMB_S
T_HOLD2_END = T_CLIMB_END + PHASE_HOLD2_S
T_DESCEND_END = T_HOLD2_END + PHASE_DESCEND_S  # == PROFILE_DURATION_S

PROFILE_HEADER = [
    "Time", "jsb_time_s", "jsb_lat_deg", "jsb_lon_deg", "jsb_alt_m",
    "jsb_vn_mps", "jsb_ve_mps", "jsb_vd_mps", "jsb_airspeed_mps",
    "jsb_roll_rad", "jsb_pitch_rad", "jsb_yaw_rad",
]


def compute_altitude_profile(t_s: float):
    """Pure function: commanded (alt_m, vd_mps, phase_name) at elapsed
    time t_s, per the task's hold/climb/hold/descend profile. Clamps to
    the hold1 state for t<0 and the descend-end (hold at 3000m) state for
    t>=PROFILE_DURATION_S.
    """
    if t_s < 0.0:
        return BASE_ALT_M, 0.0, "hold1"
    if t_s < T_HOLD1_END:
        return BASE_ALT_M, 0.0, "hold1"
    if t_s < T_CLIMB_END:
        alt = BASE_ALT_M + CLIMB_RATE_MPS * (t_s - T_HOLD1_END)
        return alt, -CLIMB_RATE_MPS, "climb"
    if t_s < T_HOLD2_END:
        return PEAK_ALT_M, 0.0, "hold2"
    if t_s < T_DESCEND_END:
        alt = PEAK_ALT_M - CLIMB_RATE_MPS * (t_s - T_HOLD2_END)
        return alt, CLIMB_RATE_MPS, "descend"
    return BASE_ALT_M, 0.0, "done"


def build_profile_row(t_s: float, lat=FIXED_LAT, lon=FIXED_LON, yaw_deg=FIXED_YAW_DEG):
    """Pure function: one CSV data row (list, matching PROFILE_HEADER)
    for elapsed time t_s. All velocity/attitude fields the bridge's
    feeder validation requires are supplied as fixed/zero (vn=ve=0,
    roll=pitch=0, airspeed=0) except the commanded altitude/vd.
    """
    alt_m, vd_mps, phase = compute_altitude_profile(t_s)
    yaw_rad = math.radians(yaw_deg)
    return [
        f"{t_s:.3f}", f"{t_s:.3f}", f"{lat:.7f}", f"{lon:.7f}", f"{alt_m:.4f}",
        "0.0", "0.0", f"{vd_mps:.4f}", "0.0",
        "0.0", "0.0", f"{yaw_rad:.6f}",
    ], phase


def write_profile_csv_realtime(csv_path, duration_s=PROFILE_DURATION_S, rate_hz=10.0,
                                lat=FIXED_LAT, lon=FIXED_LON, yaw_deg=FIXED_YAW_DEG,
                                alive_check=None, sleep_fn=time.sleep):
    """Append rows to csv_path in real (wall-clock) time at rate_hz,
    covering [0, duration_s]. Writes the header first. No JSBSim, no
    aircraft physics -- pure scripted profile. Returns the number of data
    rows written.

    alive_check: optional zero-arg callable returning an abort-ready
    diagnostic string (or None) -- if it ever returns non-None, writing
    stops immediately (mirrors the bridge/JSBSim supervision pattern used
    throughout this task family).
    """
    csv_path = Path(csv_path)
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    dt = 1.0 / max(rate_hz, 0.1)
    rows_written = 0
    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(PROFILE_HEADER)
        f.flush()
        start_wall = time.time()
        t_s = 0.0
        while t_s <= duration_s:
            if alive_check is not None:
                abort_message = alive_check()
                if abort_message is not None:
                    return rows_written
            row, _phase = build_profile_row(t_s, lat=lat, lon=lon, yaw_deg=yaw_deg)
            writer.writerow(row)
            f.flush()
            rows_written += 1
            t_s = time.time() - start_wall
            sleep_fn(dt)
    return rows_written


def build_bridge_cmd_for_ramp_test(pixhawk, baud, state_csv, mp_out, bridge_log_csv,
                                    bridge_script=BRIDGE_SCRIPT, python_exe=None):
    """Pure command-list builder. Deliberately does NOT include
    --gps-input-send-yaw, --attitude-inject, or --airspeed-inject (task
    items 7/10: no attitude display sweep, yaw transmission stays
    disabled) and NEVER --allow-actuator-output.
    """
    cmd = [
        python_exe or sys.executable, str(bridge_script),
        "--pixhawk", str(pixhawk), "--baud", str(baud),
        "--jsbsim-state-input", str(state_csv),
        "--gps-input-inject",
        "--log", str(bridge_log_csv),
    ]
    if mp_out:
        cmd += ["--mp-out", str(mp_out)]
    return cmd


PHASES_IN_ORDER = ("hold1", "climb", "hold2", "descend")


def classify_phase_from_commanded(commanded_alt_m, commanded_vd_mps, vd_eps=0.1):
    """Classify which profile phase a logged row belongs to using only
    the COMMANDED altitude/vd values actually transmitted for that row
    (self-consistent with the bridge's own log, no wall-clock
    synchronization assumptions). Returns None if either input is
    missing.
    """
    if commanded_alt_m is None or commanded_vd_mps is None:
        return None
    if commanded_vd_mps <= -vd_eps:
        return "climb"
    if commanded_vd_mps >= vd_eps:
        return "descend"
    midpoint = (BASE_ALT_M + PEAK_ALT_M) / 2.0
    return "hold2" if commanded_alt_m >= midpoint else "hold1"


def compute_phase_summary(bridge_log_csv, tolerance_m=3.0, reset_ned_z_jump_m=5.0):
    """Post-process the bridge's own CSV log into the required per-phase
    summary: steady-state altitude error (hold phases), ramp tracking
    delay (climb/descend), maximum error overall, and discontinuity/reset
    detection (ekf_flags value changes, large consecutive local_ned_z_m
    jumps). Pure function, no hardware/process access.
    """
    with open(bridge_log_csv, newline="") as f:
        rows = list(csv.DictReader(f))

    def f_(row, key):
        v = row.get(key, "")
        try:
            return float(v)
        except (TypeError, ValueError):
            return None

    samples = []
    for row in rows:
        # HIL-F23-F2B2A: phase is classified from the COMMANDED
        # alt/vd values actually logged for this row, not from wall-clock
        # time -- the bridge's own time_s (elapsed since bridge start) is
        # NOT synchronized to the profile writer's t_s (elapsed since the
        # profile writer thread started, ~2s+ later), so back-calculating
        # phase from bridge time_s would misclassify rows near every
        # phase boundary.
        t = f_(row, "time_s")
        commanded_alt = f_(row, "gps_tx_alt_m")
        commanded_vd = f_(row, "gps_tx_vd_mps")
        if t is None or commanded_alt is None:
            continue
        phase = classify_phase_from_commanded(commanded_alt, commanded_vd)
        if phase is None:
            continue
        samples.append({
            "t": t,
            "phase": phase,
            "commanded_alt_m": commanded_alt,
            "commanded_vd_mps": commanded_vd,
            "global_alt_m": f_(row, "alt_m"),
            "gps_raw_alt_m": f_(row, "gps_raw_alt_m"),
            "vfr_alt_m": f_(row, "vfr_alt_m"),
            "local_ned_z_m": f_(row, "local_ned_z_m"),
            "ekf_flags": f_(row, "ekf_flags"),
        })

    per_phase = {}
    for phase in PHASES_IN_ORDER:
        phase_samples = [s for s in samples if s["phase"] == phase]
        errors = [
            abs(s["global_alt_m"] - s["commanded_alt_m"])
            for s in phase_samples if s["global_alt_m"] is not None
        ]
        entry = {
            "n_samples": len(phase_samples),
            "max_error_m": max(errors) if errors else None,
            "mean_error_m": (sum(errors) / len(errors)) if errors else None,
        }
        if phase in ("hold1", "hold2"):
            entry["steady_state_error_m"] = entry["mean_error_m"]
        if phase in ("climb", "descend") and phase_samples:
            # ramp tracking delay: first time the tracked altitude enters
            # +/-tolerance_m of the commanded altitude, relative to phase start.
            phase_start_t = phase_samples[0]["t"]
            delay_s = None
            for s in phase_samples:
                if s["global_alt_m"] is None:
                    continue
                if abs(s["global_alt_m"] - s["commanded_alt_m"]) <= tolerance_m:
                    delay_s = s["t"] - phase_start_t
                    break
            entry["ramp_tracking_delay_s"] = delay_s
        per_phase[phase] = entry

    overall_errors = [
        abs(s["global_alt_m"] - s["commanded_alt_m"])
        for s in samples if s["global_alt_m"] is not None
    ]

    discontinuities = []
    prev_flags = None
    prev_ned_z = None
    prev_t = None
    for s in samples:
        if prev_flags is not None and s["ekf_flags"] is not None and s["ekf_flags"] != prev_flags:
            discontinuities.append({
                "t": s["t"], "type": "ekf_flags_change",
                "from": prev_flags, "to": s["ekf_flags"],
            })
        if prev_ned_z is not None and s["local_ned_z_m"] is not None:
            jump = abs(s["local_ned_z_m"] - prev_ned_z)
            if jump >= reset_ned_z_jump_m:
                discontinuities.append({
                    "t": s["t"], "type": "local_ned_z_jump",
                    "from": prev_ned_z, "to": s["local_ned_z_m"], "jump_m": jump,
                })
        if s["ekf_flags"] is not None:
            prev_flags = s["ekf_flags"]
        if s["local_ned_z_m"] is not None:
            prev_ned_z = s["local_ned_z_m"]
        prev_t = s["t"]

    return {
        "n_samples": len(samples),
        "per_phase": per_phase,
        "overall_max_error_m": max(overall_errors) if overall_errors else None,
        "overall_mean_error_m": (sum(overall_errors) / len(overall_errors)) if overall_errors else None,
        "discontinuities": discontinuities,
        "duration_s": prev_t,
    }


def print_phase_summary(summary):
    print("=== HIL-F23-F2B2A per-phase altitude summary ===")
    print(f"samples={summary['n_samples']} duration_s={summary['duration_s']}")
    for phase in PHASES_IN_ORDER:
        entry = summary["per_phase"].get(phase, {})
        print(f"  phase={phase}: n={entry.get('n_samples')} "
              f"max_error_m={entry.get('max_error_m')} mean_error_m={entry.get('mean_error_m')} "
              f"steady_state_error_m={entry.get('steady_state_error_m')} "
              f"ramp_tracking_delay_s={entry.get('ramp_tracking_delay_s')}")
    print(f"overall_max_error_m={summary['overall_max_error_m']} "
          f"overall_mean_error_m={summary['overall_mean_error_m']}")
    if summary["discontinuities"]:
        print(f"DISCONTINUITIES DETECTED: {len(summary['discontinuities'])}")
        for d in summary["discontinuities"]:
            print(f"  t={d['t']:.2f} {d['type']} from={d['from']} to={d['to']}")
    else:
        print("No discontinuities/resets detected (ekf_flags stable, no large local_ned_z_m jumps)")


def build_arg_parser():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--pixhawk", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--run-dir", default="/tmp/sr75_hil_f2b2a")
    parser.add_argument("--mp-out", default=DEFAULT_MP_OUT,
                         help="Mission Planner MAVLink output, forwarded unchanged to the bridge's --mp-out. "
                              "Pass an empty string to disable forwarding.")
    parser.add_argument("--profile-rate-hz", type=float, default=10.0,
                         help="Rate at which the synthetic altitude-profile CSV rows are appended")
    parser.add_argument("--readiness-timeout-s", type=float, default=10.0)
    parser.add_argument("--dry-run", action="store_true",
                         help="Print the profile plan and exact commands; touch no hardware, start no subprocess.")
    parser.add_argument("--producer-only", action="store_true",
                         help="Run the synthetic altitude-profile writer alone (no Pixhawk, no bridge) and "
                              "print a summary of the generated CSV. Zero hardware involvement.")
    parser.add_argument("--keep-artifacts", action="store_true",
                         help="Do not delete the generated profile CSV/console log after a successful run")
    return parser


def main():
    args = build_arg_parser().parse_args()
    run_dir = Path(args.run_dir)
    run_dir.mkdir(parents=True, exist_ok=True)
    profile_csv = run_dir / "sr75_hil_f2b2a_altitude_profile.csv"
    run_succeeded = False

    print("=== HIL-F23-F2B2A: controlled GPS altitude ramp ===")
    print(f"fixed lat={FIXED_LAT:.7f} lon={FIXED_LON:.7f} yaw={FIXED_YAW_DEG:.6f} deg (yaw transmission DISABLED)")
    print(f"profile: hold {BASE_ALT_M:.0f}m for {PHASE_HOLD1_S:.0f}s -> climb +{CLIMB_RATE_MPS:.0f}m/s for "
          f"{PHASE_CLIMB_S:.0f}s -> hold {PEAK_ALT_M:.0f}m for {PHASE_HOLD2_S:.0f}s -> descend "
          f"-{CLIMB_RATE_MPS:.0f}m/s for {PHASE_DESCEND_S:.0f}s (total {PROFILE_DURATION_S:.0f}s)")
    print(f"profile CSV: {profile_csv}")
    print(f"Mission Planner forwarding (--mp-out): {args.mp_out or '(disabled)'}")
    print("GPS_INPUT yaw transmission: DISABLED (fixed by design for this test)")
    print("Attitude/airspeed display injection: DISABLED (not required for this test)")

    if args.dry_run:
        print("")
        print("DRY RUN: stopping before any hardware connection or subprocess launch.")
        return 0

    try:
        if args.producer_only:
            print("")
            print("PRODUCER-ONLY: writing the synthetic altitude profile alone, no Pixhawk, no bridge.")
            rows_written = write_profile_csv_realtime(
                profile_csv, duration_s=PROFILE_DURATION_S, rate_hz=args.profile_rate_hz,
            )
            print(f"rows written: {rows_written}")
            run_succeeded = rows_written > 0
            return 0 if run_succeeded else 2

        # d/e: read-only preflight (heartbeat + GPS1_TYPE==14)
        ok, message = e1_launch.preflight_checks(args.pixhawk, args.baud)
        if not ok:
            print(f"ABORT: preflight failed: {message}")
            return 2

        # f: bridge, no-actuator-output (the only mode)
        bridge_log_csv = run_dir / f"sr75_hil_f2b2a_bridge_log_{time.strftime('%Y%m%d_%H%M%S')}.csv"
        bridge_console_log = run_dir / BRIDGE_CONSOLE_LOG_NAME
        bridge_cmd = build_bridge_cmd_for_ramp_test(
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

            # g: start the synthetic profile writer LAST (mirrors "start
            # JSBSim last" from the established a-h order), in a
            # background thread so main() can still supervise the bridge.
            import threading
            writer_result = {}

            def _run_writer():
                writer_result["rows"] = write_profile_csv_realtime(
                    profile_csv, duration_s=PROFILE_DURATION_S, rate_hz=args.profile_rate_hz,
                    alive_check=alive_check,
                )

            writer_thread = threading.Thread(target=_run_writer, daemon=True)
            writer_thread.start()

            # h: producer-readiness gate, then observe for the full profile duration.
            ready_ok, ready_message = e1_launch.wait_for_producer_ready(
                profile_csv, timeout_s=args.readiness_timeout_s, extra_alive_check=alive_check,
            )
            print(ready_message)
            if not ready_ok:
                print(f"ABORT: {ready_message}")
                return 2

            print(f"Observation window: {PROFILE_DURATION_S:.0f}s (matches the profile duration)")
            obs_deadline = time.time() + PROFILE_DURATION_S + 5.0
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

        if bridge_log_csv.exists():
            summary = compute_phase_summary(bridge_log_csv)
            print_phase_summary(summary)
        else:
            print(f"WARNING: bridge log CSV not found at {bridge_log_csv}, cannot compute phase summary")

        run_succeeded = True
        return 0
    finally:
        removed = e1_launch.cleanup_artifacts([profile_csv], args.keep_artifacts, run_succeeded)
        for p in removed:
            print(f"cleaned up temporary file: {p}")
        if not run_succeeded:
            print("Run did not succeed -- profile CSV and console log were retained for debugging.")


if __name__ == "__main__":
    sys.exit(main())
