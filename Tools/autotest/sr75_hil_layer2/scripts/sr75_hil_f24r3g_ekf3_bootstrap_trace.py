#!/usr/bin/env python3
"""HIL-F24-R3G: read-only EKF3-bootstrap trace, from device reattach
(after a manual Pixhawk power-cycle) until the first "EKF3 IMU%u
initialised" STATUSTEXT (libraries/AP_NavEKF3/AP_NavEKF3_core.cpp's
InitialiseFilterBootstrap(), GCS_SEND_TEXT at the end of that function).

Purpose (see Tools/autotest/sr75_hil_layer2/reports/HIL_F24_R3G_ekf3_
tilt_alignment_diagnosis.md): prove or disprove that AHRS2 (EKF3's own
"secondary" attitude -- see that report's item 2) freezes at ~+7.4 deg
roll / +7.5 deg pitch because InitialiseFilterBootstrap() latches a
single, un-averaged, transient accelerometer sample during startup,
rather than a real, sustained board tilt.

Captures, all timestamped with this process's own time.monotonic():
GPS_RAW_INT, RAW_IMU, SCALED_IMU, SCALED_IMU2, ATTITUDE, AHRS2,
EKF_STATUS_REPORT, and every STATUSTEXT. Stops on the first STATUSTEXT
matching "EKF3 IMU<N> initialised", or --capture-timeout-s, whichever
comes first.

This script NEVER sends PARAM_SET, arm/disarm, mode change, mission
item, RC override, or actuator command. The only messages it ever sends
are read-only MAV_CMD_SET_MESSAGE_INTERVAL stream-rate requests (the
same established pattern sr75_hil_f24r2_estimator_capture.py and
sr75_hil_f24c_preflash_precheck.py already use). It never triggers, nor
waits to be told about, a reboot -- the operator performs the manual
power-cycle; this script only waits for the USB device path to
reappear, then connects and listens over USB (independent of PPP/
SIM_JSON transport, so it can capture the very earliest post-boot
telemetry, before PPP has necessarily reconnected).
"""
import argparse
import csv
import json
import math
import re
import sys
import time
from pathlib import Path
from typing import List, Optional

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import sr75_hil_f24f_hardware_orchestrator as orch  # noqa: E402 -- reuses wait_for_devices()

# HIL-F24-R3G task: message types this trace needs, all requested
# read-only via MAV_CMD_SET_MESSAGE_INTERVAL. Fast rates (up to 50Hz) --
# the window of interest (GPS reaching FIX_3D through EKF3's bootstrap
# tilt sample, ~1s later) is only a few seconds long.
RATE_REQUESTED_MESSAGES_HZ = {
    "GPS_RAW_INT": 10.0,
    "RAW_IMU": 50.0,
    "SCALED_IMU": 50.0,
    "SCALED_IMU2": 50.0,
    "ATTITUDE": 50.0,
    "AHRS2": 50.0,
    "EKF_STATUS_REPORT": 10.0,
}

CSV_FIELDS = [
    "host_monotonic_time", "msg_type",
    "fix_type", "satellites_visible", "lat_deg", "lon_deg", "alt_m",
    "att_roll_deg", "att_pitch_deg", "att_yaw_deg",
    "ahrs2_roll_deg", "ahrs2_pitch_deg", "ahrs2_yaw_deg",
    "ekf_flags",
    "imu_xacc", "imu_yacc", "imu_zacc", "imu_xgyro", "imu_ygyro", "imu_zgyro",
    "statustext_text",
]

# Matches the exact GCS_SEND_TEXT format string in AP_NavEKF3_core.cpp's
# InitialiseFilterBootstrap(): 'GCS_SEND_TEXT(MAV_SEVERITY_INFO, "EKF3
# IMU%u initialised",(unsigned)imu_index);'
EKF3_INITIALISED_RE = re.compile(r"EKF3 IMU\d+ initialised")

# HIL-F24-R3G task: "diverged from level" -- matches the report's own
# framing (primary AHRS/IMU read level; the frozen value is ~7.4/7.5 deg,
# far above any plausible sensor noise floor).
DEFAULT_DIVERGENCE_THRESHOLD_DEG = 1.0


def request_message_intervals(master, rates_hz):
    """Read-only stream-rate request per message type -- MAV_CMD_SET_
    MESSAGE_INTERVAL, never PARAM_SET/arm/mode/mission/RC-override/
    actuator. Mirrors sr75_hil_f24r2_estimator_capture.py's identical
    helper."""
    from pymavlink import mavutil
    for name, hz in rates_hz.items():
        msg_id = getattr(mavutil.mavlink, f"MAVLINK_MSG_ID_{name}", None)
        if msg_id is None:
            continue
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            msg_id, int(1e6 / hz), 0, 0, 0, 0, 0,
        )


def mavlink_message_to_row(msg, host_monotonic_time):
    """Pure function -- no I/O, directly unit-testable against
    synthetic/constructed message objects. Returns a CSV_FIELDS-shaped
    row, or None for a message type this trace does not capture."""
    msg_type = msg.get_type()
    row = {field: None for field in CSV_FIELDS}
    row["host_monotonic_time"] = host_monotonic_time
    row["msg_type"] = msg_type

    if msg_type == "GPS_RAW_INT":
        row["fix_type"] = float(msg.fix_type)
        row["satellites_visible"] = float(msg.satellites_visible)
        row["lat_deg"] = msg.lat * 1.0e-7
        row["lon_deg"] = msg.lon * 1.0e-7
        row["alt_m"] = msg.alt * 1.0e-3
    elif msg_type == "ATTITUDE":
        row["att_roll_deg"] = math.degrees(msg.roll)
        row["att_pitch_deg"] = math.degrees(msg.pitch)
        row["att_yaw_deg"] = math.degrees(msg.yaw)
    elif msg_type == "AHRS2":
        row["ahrs2_roll_deg"] = math.degrees(msg.roll)
        row["ahrs2_pitch_deg"] = math.degrees(msg.pitch)
        row["ahrs2_yaw_deg"] = math.degrees(msg.yaw)
    elif msg_type == "EKF_STATUS_REPORT":
        row["ekf_flags"] = float(msg.flags)
    elif msg_type in ("RAW_IMU", "SCALED_IMU", "SCALED_IMU2"):
        row["imu_xacc"] = float(msg.xacc)
        row["imu_yacc"] = float(msg.yacc)
        row["imu_zacc"] = float(msg.zacc)
        row["imu_xgyro"] = float(msg.xgyro)
        row["imu_ygyro"] = float(msg.ygyro)
        row["imu_zgyro"] = float(msg.zgyro)
    elif msg_type == "STATUSTEXT":
        row["statustext_text"] = msg.text
    else:
        return None
    return row


def detect_bootstrap_complete(text: str) -> bool:
    """Pure: True if `text` is (or contains) the exact "EKF3 IMU<N>
    initialised" STATUSTEXT InitialiseFilterBootstrap() sends once
    statesInitialised is latched true (AP_NavEKF3_core.cpp)."""
    return bool(EKF3_INITIALISED_RE.search(text or ""))


def tilt_from_accel(xacc: float, yacc: float, zacc: float) -> Optional[tuple]:
    """Pure: reproduces AP_NavEKF3_core.cpp's InitialiseFilterBootstrap()
    tilt-from-accelerometer formula exactly:
        initAccVec.normalize()
        pitch = asin(initAccVec.x)
        roll  = atan2(-initAccVec.y, -initAccVec.z)
    Units cancel out under normalization, so raw mg (or m/s^2, or any
    consistent unit) values work directly -- no g-conversion needed.
    Returns (roll_deg, pitch_deg), or None if the vector is degenerate
    (matches the guard at AP_NavEKF3_core.cpp's `if (initAccVec.length()
    > 0.001f)`, below which the source leaves roll=pitch=0)."""
    length = math.sqrt(xacc * xacc + yacc * yacc + zacc * zacc)
    if length <= 0.001:
        return (0.0, 0.0)
    x, y, z = xacc / length, yacc / length, zacc / length
    pitch = math.asin(max(-1.0, min(1.0, x)))
    roll = math.atan2(-y, -z)
    return (math.degrees(roll), math.degrees(pitch))


def find_first_divergence(rows: List[dict], threshold_deg: float) -> Optional[dict]:
    """Pure: first AHRS2 row whose |roll| or |pitch| exceeds
    threshold_deg -- i.e. the first sample no longer consistent with a
    level board. None if AHRS2 never diverges in the given rows."""
    for row in rows:
        if row["msg_type"] != "AHRS2":
            continue
        roll = row["ahrs2_roll_deg"]
        pitch = row["ahrs2_pitch_deg"]
        if roll is None or pitch is None:
            continue
        if abs(roll) > threshold_deg or abs(pitch) > threshold_deg:
            return row
    return None


def nearest_accel_sample_at_or_before(rows: List[dict], target_time: float, imu_msg_type: str = None) -> Optional[dict]:
    """Pure: the most recent RAW_IMU/SCALED_IMU/SCALED_IMU2 sample at or
    before target_time (the accelerometer reading InitialiseFilterBootstrap()
    would have read at that instant) -- falls back to the nearest sample
    overall (even if after target_time) if none exists before it, so a
    result is still returned when the capture began exactly at the
    divergence sample itself. If imu_msg_type is given, only that message
    type is considered (e.g. to isolate a specific IMU instance)."""
    imu_types = (imu_msg_type,) if imu_msg_type else ("RAW_IMU", "SCALED_IMU", "SCALED_IMU2")
    candidates = [r for r in rows if r["msg_type"] in imu_types and r["imu_xacc"] is not None]
    if not candidates:
        return None
    before = [r for r in candidates if r["host_monotonic_time"] <= target_time]
    if before:
        return max(before, key=lambda r: r["host_monotonic_time"])
    return min(candidates, key=lambda r: abs(r["host_monotonic_time"] - target_time))


def compare_predicted_to_observed(
    predicted_roll_deg: float, predicted_pitch_deg: float,
    observed_roll_deg: float, observed_pitch_deg: float,
    tolerance_deg: float,
) -> tuple:
    """Pure: whether the accel-vector-derived tilt matches the observed
    (frozen) AHRS2/EKF3 attitude within tolerance_deg on both axes.
    Returns (matches: bool, roll_residual_deg, pitch_residual_deg)."""
    roll_residual = abs(predicted_roll_deg - observed_roll_deg)
    pitch_residual = abs(predicted_pitch_deg - observed_pitch_deg)
    matches = roll_residual <= tolerance_deg and pitch_residual <= tolerance_deg
    return matches, roll_residual, pitch_residual


def summarize(rows: List[dict], divergence_threshold_deg: float, match_tolerance_deg: float, stop_reason: str) -> dict:
    """Pure: builds the full HIL-F24-R3G answer set (task's 6 return
    items 1-4) from a completed capture's rows."""
    gps_raw = [r for r in rows if r["msg_type"] == "GPS_RAW_INT"]
    first_fix_3d = next((r for r in gps_raw if r["fix_type"] is not None and r["fix_type"] >= 3), None)

    ekf_reports = [r for r in rows if r["msg_type"] == "EKF_STATUS_REPORT"]
    first_ekf_report = ekf_reports[0] if ekf_reports else None

    bootstrap_events = [
        r for r in rows
        if r["msg_type"] == "STATUSTEXT" and detect_bootstrap_complete(r["statustext_text"])
    ]
    bootstrap_complete_row = bootstrap_events[0] if bootstrap_events else None

    divergence_row = find_first_divergence(rows, divergence_threshold_deg)

    summary = {
        "stop_reason": stop_reason,
        "divergence_threshold_deg": divergence_threshold_deg,
        "match_tolerance_deg": match_tolerance_deg,
        "first_gps_fix_3d": None if first_fix_3d is None else {
            "host_monotonic_time": first_fix_3d["host_monotonic_time"],
            "lat_deg": first_fix_3d["lat_deg"], "lon_deg": first_fix_3d["lon_deg"], "alt_m": first_fix_3d["alt_m"],
        },
        "first_ekf_status_report": None if first_ekf_report is None else {
            "host_monotonic_time": first_ekf_report["host_monotonic_time"],
            "ekf_flags": first_ekf_report["ekf_flags"],
        },
        "ekf3_initialised_statustext_seen": bootstrap_complete_row is not None,
        "ekf3_initialised_statustext": None if bootstrap_complete_row is None else {
            "host_monotonic_time": bootstrap_complete_row["host_monotonic_time"],
            "text": bootstrap_complete_row["statustext_text"],
        },
        "first_divergence_time": None if divergence_row is None else divergence_row["host_monotonic_time"],
        "observed_ahrs2_at_divergence": None if divergence_row is None else {
            "roll_deg": divergence_row["ahrs2_roll_deg"], "pitch_deg": divergence_row["ahrs2_pitch_deg"],
        },
        "accel_vector_at_divergence": None,
        "predicted_tilt_from_accel": None,
        "reproduces_bootstrap_math": None,
        "roll_residual_deg": None,
        "pitch_residual_deg": None,
    }

    if divergence_row is not None:
        accel_row = nearest_accel_sample_at_or_before(rows, divergence_row["host_monotonic_time"])
        if accel_row is not None:
            summary["accel_vector_at_divergence"] = {
                "host_monotonic_time": accel_row["host_monotonic_time"],
                "msg_type": accel_row["msg_type"],
                "xacc": accel_row["imu_xacc"], "yacc": accel_row["imu_yacc"], "zacc": accel_row["imu_zacc"],
            }
            predicted = tilt_from_accel(accel_row["imu_xacc"], accel_row["imu_yacc"], accel_row["imu_zacc"])
            if predicted is not None:
                predicted_roll, predicted_pitch = predicted
                summary["predicted_tilt_from_accel"] = {"roll_deg": predicted_roll, "pitch_deg": predicted_pitch}
                matches, roll_res, pitch_res = compare_predicted_to_observed(
                    predicted_roll, predicted_pitch,
                    divergence_row["ahrs2_roll_deg"], divergence_row["ahrs2_pitch_deg"],
                    match_tolerance_deg,
                )
                summary["reproduces_bootstrap_math"] = matches
                summary["roll_residual_deg"] = roll_res
                summary["pitch_residual_deg"] = pitch_res

    return summary


def build_arg_parser():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--pixhawk", default="/dev/ttyACM0", help="Pixhawk USB MAVLink device (read-only)")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument(
        "--wait-for-device-timeout-s", type=float, default=120.0,
        help="Max wait for --pixhawk to reappear (covers the manual power-cycle gap)",
    )
    parser.add_argument("--heartbeat-timeout-s", type=float, default=60.0)
    parser.add_argument(
        "--capture-timeout-s", type=float, default=90.0,
        help="Max capture duration if the 'EKF3 IMU<N> initialised' STATUSTEXT is never observed",
    )
    parser.add_argument("--poll-interval-s", type=float, default=0.02)
    parser.add_argument("--divergence-threshold-deg", type=float, default=DEFAULT_DIVERGENCE_THRESHOLD_DEG)
    parser.add_argument("--match-tolerance-deg", type=float, default=2.0)
    parser.add_argument("--output-csv", required=True)
    parser.add_argument("--summary-json", required=True)
    return parser


def main():
    args = build_arg_parser().parse_args()

    print(f"=== HIL-F24-R3G EKF3 bootstrap trace (read-only; --pixhawk {args.pixhawk}) ===")
    print(f"Waiting up to {args.wait_for_device_timeout_s}s for {args.pixhawk} to reappear...")
    if not orch.wait_for_devices([args.pixhawk], timeout_s=args.wait_for_device_timeout_s):
        print(f"ABORT: {args.pixhawk} did not reappear within {args.wait_for_device_timeout_s}s")
        return 2
    print("Device present. Connecting...")

    from pymavlink import mavutil
    master = mavutil.mavlink_connection(args.pixhawk, baud=args.baud, autoreconnect=True)
    hb = master.wait_heartbeat(timeout=args.heartbeat_timeout_s)
    if hb is None:
        print(f"ABORT: no heartbeat within {args.heartbeat_timeout_s}s")
        return 2
    print(f"Heartbeat OK: type={hb.type} autopilot={hb.autopilot}")
    request_message_intervals(master, RATE_REQUESTED_MESSAGES_HZ)

    rows = []
    stop_reason = "capture_timeout"
    start = time.monotonic()
    with open(args.output_csv, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=CSV_FIELDS)
        writer.writeheader()
        while time.monotonic() - start < args.capture_timeout_s:
            iter_start = time.monotonic()
            while True:
                msg = master.recv_match(blocking=False)
                if msg is None:
                    break
                row = mavlink_message_to_row(msg, time.monotonic())
                if row is None:
                    continue
                rows.append(row)
                writer.writerow(row)
                if row["msg_type"] == "STATUSTEXT" and detect_bootstrap_complete(row["statustext_text"]):
                    stop_reason = "ekf3_initialised_statustext_seen"
            if stop_reason == "ekf3_initialised_statustext_seen":
                break
            sleep_s = args.poll_interval_s - (time.monotonic() - iter_start)
            if sleep_s > 0:
                time.sleep(sleep_s)
    master.close()

    print(f"Capture stopped: {stop_reason} ({len(rows)} rows over {time.monotonic() - start:.1f}s)")

    summary = summarize(rows, args.divergence_threshold_deg, args.match_tolerance_deg, stop_reason)
    with open(args.summary_json, "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2, sort_keys=True)
        f.write("\n")

    print(
        "TRACE_RESULT "
        f"first_divergence_time={summary['first_divergence_time']} "
        f"reproduces_bootstrap_math={summary['reproduces_bootstrap_math']} "
        f"roll_residual_deg={summary['roll_residual_deg']} "
        f"pitch_residual_deg={summary['pitch_residual_deg']}"
    )
    print(
        "Safety confirmation: only PARAM_REQUEST_READ-equivalent MAV_CMD_SET_MESSAGE_INTERVAL "
        "requests were sent (all read-only); no PARAM_SET, arm, mission, RC override, or actuator command."
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
