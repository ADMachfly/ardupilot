#!/usr/bin/env python3
"""HIL-F24-R2B: read-only pre-run AHRS/EKF-origin diagnostic.

Connects read-only to the Pixhawk and, before (or independently of) a full
HIL-F24-R2 bench run, reports:

  - GPS1_TYPE/EK3_*/AHRS_EKF_TYPE (reusing sr75_hil_gps_ekf_readonly_
    audit.py's CHECKS unmodified).
  - HOME_POSITION and GPS_GLOBAL_ORIGIN (requested via the read-only
    MAV_CMD_GET_HOME_POSITION / MAV_CMD_REQUEST_MESSAGE commands), compared
    against a given JSBSim initial lat/lon/alt (--truth-lat/--truth-lon/
    --truth-alt-m, defaulting to HIL-F24-R1's runscript IC) using the same
    horizontal_distance_m() helper the R2 comparison script itself uses.
  - Any STATUSTEXT observed in a bounded listen window matching known
    AHRS-backend-health substrings (e.g. "AHRS: DCM active", "not using
    configured AHRS type", "Main loop slow", "GPS and AHRS differ").

This is a standalone diagnostic, not wired into the R2 orchestrator or its
PASS/FAIL thresholds -- it never changes DEFAULT_THRESHOLDS or any other
provisional threshold, and it never sends PARAM_SET, arm/disarm, mode
change, mission item, RC override, or actuator command. It exits 1 if a
GPS1_TYPE/EK3 param check blocks, the origin/home diverges from the given
truth beyond ORIGIN_MISMATCH_WARN_M, or a health-relevant STATUSTEXT is
observed during the listen window -- purely informational for the
operator, never automatically gating anything.
"""
import argparse
import json
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import sr75_hil_gps_ekf_readonly_audit as gps_ekf_audit  # noqa: E402
import sr75_hil_f24r2_estimator_comparison as cmp  # noqa: E402

# HIL-F24-R1's SR75_hil_f24r1_dynamic_state_feed.xml ground IC (accel_ground_init).
DEFAULT_TRUTH_LAT_DEG = 32.5378085
DEFAULT_TRUTH_LON_DEG = 74.3661944
DEFAULT_TRUTH_ALT_M = 240.201118

# Diagnostic-only tolerance -- NOT one of sr75_hil_f24r2_estimator_
# comparison.py's DEFAULT_THRESHOLDS, and never fed into that script.
ORIGIN_MISMATCH_WARN_M = 50.0

AHRS_HEALTH_STATUSTEXT_SUBSTRINGS = (
    "ahrs: dcm active",
    "not using configured ahrs type",
    "main loop slow",
    "gps and ahrs differ",
    "waiting for gps config data",
    "ekf3 waiting for gps",
)


def scan_statustext_for_ahrs_health(texts):
    """Pure: returns the subset of (already lower-cased) STATUSTEXT strings
    that match a known AHRS-backend-health substring."""
    hits = []
    for text in texts:
        lowered = text.lower()
        for sub in AHRS_HEALTH_STATUSTEXT_SUBSTRINGS:
            if sub in lowered:
                hits.append((sub, text))
                break
    return hits


def request_home_and_origin(master):
    """Read-only requests for HOME_POSITION and GPS_GLOBAL_ORIGIN --
    MAV_CMD_GET_HOME_POSITION and MAV_CMD_REQUEST_MESSAGE(GPS_GLOBAL_
    ORIGIN) only. Never PARAM_SET/arm/mode/mission/RC-override/actuator."""
    from pymavlink import mavutil
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_GET_HOME_POSITION, 0, 0, 0, 0, 0, 0, 0, 0,
    )
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0,
        mavutil.mavlink.MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN, 0, 0, 0, 0, 0, 0,
    )


def collect(master, listen_s, log):
    """Drains MAVLink traffic for listen_s seconds, collecting HOME_
    POSITION, GPS_GLOBAL_ORIGIN, and every STATUSTEXT seen. Read-only."""
    home = None
    origin = None
    statustexts = []
    deadline = time.monotonic() + listen_s
    while time.monotonic() < deadline:
        msg = master.recv_match(
            type=["HOME_POSITION", "GPS_GLOBAL_ORIGIN", "STATUSTEXT"], blocking=True, timeout=0.5,
        )
        if msg is None:
            continue
        msg_type = msg.get_type()
        if msg_type == "HOME_POSITION" and home is None:
            home = (msg.latitude * 1.0e-7, msg.longitude * 1.0e-7, msg.altitude * 1.0e-3)
            log(f"HOME_POSITION: lat={home[0]} lon={home[1]} alt={home[2]}")
        elif msg_type == "GPS_GLOBAL_ORIGIN" and origin is None:
            origin = (msg.latitude * 1.0e-7, msg.longitude * 1.0e-7, msg.altitude * 1.0e-3)
            log(f"GPS_GLOBAL_ORIGIN: lat={origin[0]} lon={origin[1]} alt={origin[2]}")
        elif msg_type == "STATUSTEXT":
            statustexts.append(msg.text)
            log(f"STATUSTEXT: {msg.text}")
    return home, origin, statustexts


def build_arg_parser():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--pixhawk", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--listen-s", type=float, default=5.0)
    parser.add_argument("--param-timeout-s", type=float, default=4.0)
    parser.add_argument("--param-retries", type=int, default=3)
    parser.add_argument("--truth-lat", type=float, default=DEFAULT_TRUTH_LAT_DEG)
    parser.add_argument("--truth-lon", type=float, default=DEFAULT_TRUTH_LON_DEG)
    parser.add_argument("--truth-alt-m", type=float, default=DEFAULT_TRUTH_ALT_M)
    parser.add_argument(
        "--summary-json", default=None,
        help="Optional: write a machine-readable summary (home/origin/mismatch/ok) to this path.",
    )
    return parser


def main():
    from pymavlink import mavutil

    args = build_arg_parser().parse_args()
    print(f"=== HIL-F24-R2B AHRS/EKF-origin diagnostic (read-only; --pixhawk {args.pixhawk}) ===")

    master = mavutil.mavlink_connection(args.pixhawk, baud=args.baud, autoreconnect=True)
    hb = master.wait_heartbeat(timeout=30)
    if hb is None:
        print("ABORT: no heartbeat received within 30s")
        return 2
    print(f"Heartbeat OK: type={hb.type} autopilot={hb.autopilot}")

    print()
    print("=== GPS1_TYPE / EK3_* / AHRS_EKF_TYPE (reused from sr75_hil_gps_ekf_readonly_audit.py) ===")
    names = [c[0] for c in gps_ekf_audit.CHECKS]
    onboard = gps_ekf_audit.read_params(
        master, names, timeout_s=args.param_timeout_s, retries=args.param_retries, log=print,
    )
    param_blockers = []
    for name, required, note in gps_ekf_audit.CHECKS:
        value = onboard.get(name)
        accepted = (required,) if isinstance(required, str) else required
        if value is not None and required is not None and gps_ekf_audit.fmt(value) not in accepted:
            param_blockers.append(name)

    print()
    print("=== HOME_POSITION / GPS_GLOBAL_ORIGIN vs. JSBSim initial lat/lon/alt ===")
    request_home_and_origin(master)
    home, origin, statustexts = collect(master, args.listen_s, log=print)

    truth = (args.truth_lat, args.truth_lon, args.truth_alt_m)
    print(f"Truth IC: lat={truth[0]} lon={truth[1]} alt={truth[2]}")

    # Tracked separately (not just a running max) so callers -- notably
    # HIL-F24-R2C's relatch gate, which cares specifically about
    # GPS_GLOBAL_ORIGIN -- can gate on the exact value that matters to
    # them rather than an ambiguous combined figure.
    mismatches_m = {"HOME_POSITION": None, "GPS_GLOBAL_ORIGIN": None}
    origin_mismatch_m = None
    for label, value in (("HOME_POSITION", home), ("GPS_GLOBAL_ORIGIN", origin)):
        if value is None:
            print(f"{label}: NOT RECEIVED within {args.listen_s}s listen window")
            continue
        dist_m = cmp.horizontal_distance_m(value[0], value[1], truth[0], truth[1])
        alt_diff_m = value[2] - truth[2]
        print(f"{label} vs. truth: horizontal={dist_m:.2f} m, altitude_diff={alt_diff_m:.2f} m")
        mismatches_m[label] = dist_m
        if origin_mismatch_m is None or dist_m > origin_mismatch_m:
            origin_mismatch_m = dist_m
    home_mismatch_m = mismatches_m["HOME_POSITION"]
    gps_global_origin_mismatch_m = mismatches_m["GPS_GLOBAL_ORIGIN"]

    print()
    print("=== AHRS-backend-health STATUSTEXT scan ===")
    hits = scan_statustext_for_ahrs_health(statustexts)
    if hits:
        for sub, text in hits:
            print(f"MATCH ({sub}): {text}")
    else:
        print(f"No AHRS-backend-health STATUSTEXT observed in {args.listen_s}s.")

    print()
    ok = True
    if param_blockers:
        print(f"WARN: GPS/EKF param blockers: {', '.join(param_blockers)}")
        ok = False
    if origin is None:
        print("WARN: GPS_GLOBAL_ORIGIN was not received; HOME_POSITION alone is not an EKF-origin relatch")
        ok = False
    if origin_mismatch_m is not None and origin_mismatch_m > ORIGIN_MISMATCH_WARN_M:
        print(f"WARN: HOME/origin vs. truth horizontal mismatch {origin_mismatch_m:.2f} m "
              f"exceeds diagnostic-only tolerance {ORIGIN_MISMATCH_WARN_M} m")
        ok = False
    if hits:
        print(f"WARN: {len(hits)} AHRS-backend-health STATUSTEXT event(s) observed")
        ok = False
    if ok:
        print("OK: no GPS/EKF param blockers, origin/home within tolerance, no AHRS-health STATUSTEXT observed.")

    print()
    print("Safety confirmation: only PARAM_REQUEST_READ, MAV_CMD_GET_HOME_POSITION, "
          "and MAV_CMD_REQUEST_MESSAGE(GPS_GLOBAL_ORIGIN) were sent (all read-only); "
          "no PARAM_SET, arm, mission, RC override, or actuator command.")

    if args.summary_json:
        summary = {
            "ok": ok,
            "truth_lat_deg": truth[0], "truth_lon_deg": truth[1], "truth_alt_m": truth[2],
            "home_position_deg": None if home is None else list(home),
            "gps_global_origin_deg": None if origin is None else list(origin),
            "home_mismatch_m": home_mismatch_m,
            "gps_global_origin_mismatch_m": gps_global_origin_mismatch_m,
            "origin_mismatch_warn_m": ORIGIN_MISMATCH_WARN_M,
            "param_blockers": param_blockers,
            "ahrs_health_events": [{"substring": sub, "text": text} for sub, text in hits],
        }
        with open(args.summary_json, "w", encoding="utf-8") as f:
            json.dump(summary, f, indent=2, sort_keys=True)
            f.write("\n")
        print(f"Summary written to {args.summary_json}")

    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
