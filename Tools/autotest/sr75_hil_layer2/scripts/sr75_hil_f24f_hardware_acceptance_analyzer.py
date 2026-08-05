#!/usr/bin/env python3
"""HIL-F24-F item 4/5: offline acceptance analyzer for FUTURE fmuv3
Simulation-on-Hardware bench logs (CSV or MAVLink).

No hardware is touched by this script. It only ever reads an already-
captured log file and a commanded profile (generated locally by
sr75_sim_json_test_profiles.py) and reports pass/fail against the
per-category thresholds defined in THRESHOLDS_BY_CATEGORY.

Two log input formats are supported, both producing the SAME internal
"record" dict shape so the evaluation logic in evaluate_acceptance() is a
single, pure, fully-unit-testable code path regardless of source:

  --csv PATH       a CSV file with the CSV_FIELDS schema defined below.
                   This is the schema the HIL-F24-F hardware orchestrator
                   (sr75_hil_f24f_hardware_orchestrator.py) writes once a
                   real bench session exists.
  --mavlink PATH   a MAVLink log (.tlog/.bin/etc, anything pymavlink's
                   mavutil.mavlink_connection() accepts), read via the
                   standard read-only recv_match() pattern already used
                   throughout this task family (sr75_hil_f24c_*.py).

No PARAM_SET, arm, mission, RC override, or actuator command is ever sent
-- this script only ever reads a log/file that already exists.
"""
import argparse
import csv
import math
import sys
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

SCRIPTS_DIR = Path(__file__).resolve().parent
SIM_JSON_DIR = SCRIPTS_DIR.parent / "sim_json"
sys.path.insert(0, str(SIM_JSON_DIR))
import sr75_sim_json_test_profiles as profiles  # noqa: E402

EARTH_RADIUS_M = 6371000.0

# The exact schema a future hardware session's CSV log must use. Every
# field is a plain float/int/str; booleans are written as 0/1.
CSV_FIELDS = [
    "t_s",
    "ahrs2_roll_deg", "ahrs2_pitch_deg", "ahrs2_yaw_deg",
    "gps_lat_deg", "gps_lon_deg", "gps_alt_m",
    "airspeed_mps",
    "ekf_flags", "ekf_healthy",
    "cpu_load_pct",
    "armed", "mode",
    "ch7_pwm", "ch8_pwm",
    "boot_time_s",
]

# Required EKF_STATUS_REPORT flag bits (matches ArduPilot's own
# EKF_STATUS_FLAGS: attitude, velocity horiz/vert, pos horiz rel/abs, pos
# vert abs) -- all must be set for the estimator to be considered healthy.
EKF_REQUIRED_HEALTHY_MASK = 0x01 | 0x02 | 0x04 | 0x08 | 0x10 | 0x20

CH_PWM_QUIET_MAX = 50  # matches sr75_hil_f24c_preflash_precheck.py's CH_PWM_QUIET_MAX


# --- Per-profile-category acceptance thresholds (HIL-F24-F item 5) --------
#
# Static: the vehicle is motionless and level -- tightest tolerances, since
# there is no dynamic tracking lag to account for.
#
# Individual sweep: one axis moves at a time -- loosened attitude/rate
# tolerance versus static to allow for normal EKF tracking lag and servo/
# aero response, but position/altitude/airspeed stay tight since only one
# axis is actually being exercised.
#
# Combined gentle: multiple axes move simultaneously -- the loosest
# tolerances, since simultaneous multi-axis excitation is expected to
# produce somewhat larger transient tracking error than a single-axis
# sweep, even though each individual axis's commanded amplitude is small
# ("gentle") by design.
THRESHOLDS_STATIC = {
    "attitude_error_deg": 1.0,
    "position_error_m": 2.0,
    "altitude_error_m": 1.5,
    "airspeed_error_mps": 1.0,
    "cpu_load_max_pct": 90.0,
    "stale_gap_max_s": 0.5,
    "ch_pwm_quiet_max": CH_PWM_QUIET_MAX,
}
THRESHOLDS_SWEEP = {
    "attitude_error_deg": 3.0,
    "position_error_m": 3.0,
    "altitude_error_m": 3.0,
    "airspeed_error_mps": 1.5,
    "cpu_load_max_pct": 90.0,
    "stale_gap_max_s": 0.5,
    "ch_pwm_quiet_max": CH_PWM_QUIET_MAX,
}
THRESHOLDS_COMBINED = {
    "attitude_error_deg": 4.0,
    "position_error_m": 4.0,
    "altitude_error_m": 4.0,
    "airspeed_error_mps": 2.0,
    "cpu_load_max_pct": 95.0,
    "stale_gap_max_s": 0.75,
    "ch_pwm_quiet_max": CH_PWM_QUIET_MAX,
}

THRESHOLDS_BY_CATEGORY = {
    "static": THRESHOLDS_STATIC,
    "sweep": THRESHOLDS_SWEEP,
    "combined": THRESHOLDS_COMBINED,
}

# Which profile name (from sr75_sim_json_test_profiles.PROFILES_F24F) maps
# to which threshold category.
PROFILE_CATEGORY = {
    "static_level": "static",
    "roll_sweep_multileg": "sweep",
    "pitch_sweep_multileg": "sweep",
    "yaw_sweep_multileg": "sweep",
    "altitude_ramp_multileg": "sweep",
    "combined_gentle": "combined",
}


class AnalyzerError(Exception):
    pass


def haversine_m(lat1_deg: float, lon1_deg: float, lat2_deg: float, lon2_deg: float) -> float:
    lat1, lon1, lat2, lon2 = map(math.radians, (lat1_deg, lon1_deg, lat2_deg, lon2_deg))
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    return EARTH_RADIUS_M * 2.0 * math.asin(min(1.0, math.sqrt(a)))


def load_csv_records(path: str) -> List[Dict]:
    records = []
    with open(path, newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            record = {}
            for field in CSV_FIELDS:
                if field not in row or row[field] == "":
                    raise AnalyzerError(f"CSV row missing required field: {field}")
                value = row[field]
                if field in ("mode",):
                    record[field] = value
                elif field in ("armed", "ekf_healthy"):
                    record[field] = bool(int(value))
                elif field in ("ekf_flags", "ch7_pwm", "ch8_pwm"):
                    record[field] = int(value)
                else:
                    record[field] = float(value)
            records.append(record)
    return records


def records_from_mavlink_messages(messages) -> List[Dict]:
    """Pure function: fuses a stream of MAVLink message objects (each with
    .get_type() and pymavlink-style attribute access) into the same record
    shape load_csv_records() produces, emitting one merged record per
    HEARTBEAT (the natural ~1 Hz clock every ArduPilot log already has).
    Last-known-value fusion -- exactly the same pattern already used for
    live reads in sr75_hil_f24c_preflash_precheck.py, just applied to a
    historical stream instead of a live connection.
    """
    latest = {
        "ahrs2_roll_deg": 0.0, "ahrs2_pitch_deg": 0.0, "ahrs2_yaw_deg": 0.0,
        "gps_lat_deg": 0.0, "gps_lon_deg": 0.0, "gps_alt_m": 0.0,
        "airspeed_mps": 0.0,
        "ekf_flags": 0, "ekf_healthy": False,
        "cpu_load_pct": 0.0,
        "armed": False, "mode": "UNKNOWN",
        "ch7_pwm": 0, "ch8_pwm": 0,
    }
    records = []
    t_s = 0.0
    for msg in messages:
        mtype = msg.get_type()
        if mtype == "AHRS2":
            latest["ahrs2_roll_deg"] = math.degrees(msg.roll)
            latest["ahrs2_pitch_deg"] = math.degrees(msg.pitch)
            latest["ahrs2_yaw_deg"] = math.degrees(msg.yaw)
        elif mtype == "GLOBAL_POSITION_INT":
            latest["gps_lat_deg"] = msg.lat / 1e7
            latest["gps_lon_deg"] = msg.lon / 1e7
            latest["gps_alt_m"] = msg.alt / 1000.0
        elif mtype == "VFR_HUD":
            latest["airspeed_mps"] = msg.airspeed
        elif mtype == "EKF_STATUS_REPORT":
            latest["ekf_flags"] = msg.flags
            latest["ekf_healthy"] = (msg.flags & EKF_REQUIRED_HEALTHY_MASK) == EKF_REQUIRED_HEALTHY_MASK
        elif mtype == "SYS_STATUS":
            latest["cpu_load_pct"] = msg.load / 10.0
        elif mtype == "SERVO_OUTPUT_RAW":
            latest["ch7_pwm"] = msg.servo7_raw
            latest["ch8_pwm"] = msg.servo8_raw
        elif mtype == "HEARTBEAT":
            from pymavlink import mavutil
            armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            mode = mavutil.mode_string_v10(msg)
            boot_time_s = getattr(msg, "_timestamp", t_s) or t_s
            t_s = boot_time_s
            record = dict(latest)
            record["t_s"] = t_s
            record["armed"] = armed
            record["mode"] = mode
            record["boot_time_s"] = getattr(msg, "time_boot_ms", 0) / 1000.0
            records.append(record)
    return records


def load_mavlink_log_records(path: str, dialect: str = "ardupilotmega") -> List[Dict]:
    """Thin I/O wrapper: opens a MAVLink log read-only (never write=True,
    never a live serial/UDP endpoint) and feeds every message through
    records_from_mavlink_messages()."""
    from pymavlink import mavutil

    conn = mavutil.mavlink_connection(path, dialect=dialect)

    def _iter_messages():
        while True:
            msg = conn.recv_match(blocking=False)
            if msg is None:
                break
            if msg.get_type() != "BAD_DATA":
                yield msg

    try:
        return records_from_mavlink_messages(_iter_messages())
    finally:
        conn.close()


def _nearest(records: Sequence[Dict], t_s: float) -> Dict:
    return min(records, key=lambda r: abs(r["t_s"] - t_s))


def build_paired_samples(commanded_rows, records: Sequence[Dict]) -> List[Tuple]:
    """Nearest-timestamp-match each commanded row to one telemetry record.
    Both sequences are assumed to already share a comparable time base
    (t_s == 0 at the start of the profile/recording).

    Guards against a silent-clamping trap: if the telemetry span is much
    shorter than the commanded span, "nearest timestamp" degenerates into
    matching every late commanded row to the same final record, which
    would misreport a duration/rate mismatch as a tracking-error
    violation. That case is raised explicitly instead.
    """
    if not records:
        raise AnalyzerError("no telemetry records to match against commanded rows")
    commanded_span = commanded_rows[-1].timestamp_s - commanded_rows[0].timestamp_s
    record_span = records[-1]["t_s"] - records[0]["t_s"]
    if commanded_span > 0.0 and record_span < 0.5 * commanded_span:
        raise AnalyzerError(
            f"telemetry span ({record_span:.2f}s) is far shorter than the commanded profile span "
            f"({commanded_span:.2f}s) -- check --profile/duration match the recorded run"
        )
    return [(state, _nearest(records, state.timestamp_s)) for state in commanded_rows]


def evaluate_acceptance(paired_samples: List[Tuple], records: Sequence[Dict], thresholds: Dict) -> Tuple[bool, Dict]:
    """Pure function: item 4/5's core. Takes already time-matched
    (commanded_state, telemetry_record) pairs plus the full ordered
    telemetry record list (for gap/reboot/CPU checks that need the whole
    sequence, not per-sample pairing), and returns (ok, report)."""
    violations = []

    attitude_errors_deg = []
    position_errors_m = []
    altitude_errors_m = []
    airspeed_errors_mps = []
    for commanded, actual in paired_samples:
        roll_err = abs(actual["ahrs2_roll_deg"] - math.degrees(commanded.roll_rad))
        pitch_err = abs(actual["ahrs2_pitch_deg"] - math.degrees(commanded.pitch_rad))
        yaw_err = abs(actual["ahrs2_yaw_deg"] - math.degrees(commanded.yaw_rad))
        attitude_errors_deg.extend([roll_err, pitch_err, yaw_err])
        position_errors_m.append(haversine_m(
            commanded.latitude_deg, commanded.longitude_deg, actual["gps_lat_deg"], actual["gps_lon_deg"],
        ))
        altitude_errors_m.append(abs(actual["gps_alt_m"] - commanded.altitude_m))
        airspeed_errors_mps.append(abs(actual["airspeed_mps"] - commanded.airspeed_mps))

    max_attitude_err = max(attitude_errors_deg) if attitude_errors_deg else 0.0
    max_position_err = max(position_errors_m) if position_errors_m else 0.0
    max_altitude_err = max(altitude_errors_m) if altitude_errors_m else 0.0
    max_airspeed_err = max(airspeed_errors_mps) if airspeed_errors_mps else 0.0

    if max_attitude_err > thresholds["attitude_error_deg"]:
        violations.append(f"attitude error {max_attitude_err:.3f} deg > {thresholds['attitude_error_deg']} deg")
    if max_position_err > thresholds["position_error_m"]:
        violations.append(f"position error {max_position_err:.3f} m > {thresholds['position_error_m']} m")
    if max_altitude_err > thresholds["altitude_error_m"]:
        violations.append(f"altitude error {max_altitude_err:.3f} m > {thresholds['altitude_error_m']} m")
    if max_airspeed_err > thresholds["airspeed_error_mps"]:
        violations.append(f"airspeed error {max_airspeed_err:.3f} m/s > {thresholds['airspeed_error_mps']} m/s")

    unhealthy_count = sum(1 for r in records if not r["ekf_healthy"])
    if unhealthy_count:
        violations.append(f"EKF unhealthy in {unhealthy_count}/{len(records)} record(s)")

    max_cpu = max((r["cpu_load_pct"] for r in records), default=0.0)
    if max_cpu > thresholds["cpu_load_max_pct"]:
        violations.append(f"CPU load {max_cpu:.1f}% > {thresholds['cpu_load_max_pct']}%")

    max_gap = 0.0
    reboot_detected = False
    for i in range(len(records) - 1):
        gap = records[i + 1]["t_s"] - records[i]["t_s"]
        max_gap = max(max_gap, gap)
        if records[i + 1]["boot_time_s"] < records[i]["boot_time_s"]:
            reboot_detected = True
    if max_gap > thresholds["stale_gap_max_s"]:
        violations.append(f"stale gap {max_gap:.3f} s > {thresholds['stale_gap_max_s']} s")
    if reboot_detected:
        violations.append("reboot detected (boot_time_s went backwards mid-recording)")

    armed_count = sum(1 for r in records if r["armed"])
    non_manual_count = sum(1 for r in records if r["mode"] != "MANUAL")
    if armed_count:
        violations.append(f"vehicle reported ARMED in {armed_count}/{len(records)} record(s)")
    if non_manual_count:
        violations.append(f"vehicle reported non-MANUAL mode in {non_manual_count}/{len(records)} record(s)")

    quiet_max = thresholds["ch_pwm_quiet_max"]
    ch_activity = [r for r in records if r["ch7_pwm"] > quiet_max or r["ch8_pwm"] > quiet_max]
    if ch_activity:
        violations.append(f"CH7/CH8 showed load activity (>{quiet_max} PWM) in {len(ch_activity)} record(s)")

    report = {
        "max_attitude_error_deg": max_attitude_err,
        "max_position_error_m": max_position_err,
        "max_altitude_error_m": max_altitude_err,
        "max_airspeed_error_mps": max_airspeed_err,
        "ekf_unhealthy_count": unhealthy_count,
        "max_cpu_load_pct": max_cpu,
        "max_stale_gap_s": max_gap,
        "reboot_detected": reboot_detected,
        "armed_count": armed_count,
        "non_manual_count": non_manual_count,
        "ch_activity_count": len(ch_activity),
        "violations": violations,
    }
    return (len(violations) == 0), report


def analyze(profile_name: str, records: Sequence[Dict], category_override: Optional[str] = None) -> Tuple[bool, Dict]:
    if profile_name not in profiles.PROFILES_F24F:
        raise AnalyzerError(f"unknown profile: {profile_name} (expected one of {list(profiles.PROFILES_F24F)})")
    category = category_override or PROFILE_CATEGORY[profile_name]
    thresholds = THRESHOLDS_BY_CATEGORY[category]
    commanded_rows = profiles.PROFILES_F24F[profile_name]()
    paired = build_paired_samples(commanded_rows, records)
    return evaluate_acceptance(paired, records, thresholds)


def build_arg_parser():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--profile", required=True, choices=list(profiles.PROFILES_F24F))
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--csv", help="Path to a CSV log using the CSV_FIELDS schema")
    source.add_argument("--mavlink", help="Path to a MAVLink log (.tlog/.bin/etc), read-only")
    parser.add_argument(
        "--category", choices=list(THRESHOLDS_BY_CATEGORY), default=None,
        help="Override the profile's default threshold category",
    )
    return parser


def main():
    args = build_arg_parser().parse_args()
    print("=== HIL-F24-F hardware acceptance analyzer (offline, read-only) ===")
    if args.csv:
        records = load_csv_records(args.csv)
        print(f"Loaded {len(records)} records from CSV: {args.csv}")
    else:
        records = load_mavlink_log_records(args.mavlink)
        print(f"Loaded {len(records)} records from MAVLink log: {args.mavlink}")

    ok, report = analyze(args.profile, records, category_override=args.category)
    for key, value in report.items():
        if key != "violations":
            print(f"  {key}: {value}")
    if ok:
        print("PASS: no acceptance-threshold violations")
    else:
        print("FAIL:")
        for v in report["violations"]:
            print(f"  - {v}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
