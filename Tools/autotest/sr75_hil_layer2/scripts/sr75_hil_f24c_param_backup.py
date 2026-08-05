#!/usr/bin/env python3
"""HIL-F24-C item 2: read-only full parameter + firmware-identity backup,
to be run against the bench Pixhawk BEFORE flashing fmuv3-SimOnHardWare
(and again afterward, to snapshot the new firmware's parameter set).

Only these MAVLink messages are ever sent:
  - PARAM_REQUEST_LIST (one request, standard read-only full-dump request)
  - MAV_CMD_REQUEST_MESSAGE for AUTOPILOT_VERSION (read-only)

This script NEVER sends PARAM_SET, arm/disarm, mission upload/start,
RC_CHANNELS_OVERRIDE, or any actuator/servo command. It never starts PPP
or SIM_JSON.

Writes two timestamped files under Tools/autotest/sr75_hil_layer2/backups/:
  sr75_hil_f24c_params_<UTC-timestamp>.parm     (every fetched parameter,
                                                  NAME<space>VALUE per line)
  sr75_hil_f24c_version_<UTC-timestamp>.txt     (heartbeat/AUTOPILOT_VERSION/
                                                  banner STATUSTEXT capture)

and prints the GPS/EKF-source, serial, servo-function, and RATO subsets
explicitly, per HIL-F24-C item 2's categorization.
"""
import argparse
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

REPO = Path(__file__).resolve().parents[4]
DEFAULT_BACKUP_DIR = REPO / "Tools/autotest/sr75_hil_layer2/backups"

GPS_EKF_PARAM_PREFIXES = ("GPS1_TYPE", "GPS2_TYPE", "AHRS_EKF_TYPE", "EK3_ENABLE", "EK3_SRC1_", "EK3_SRC2_")
SERIAL_PARAM_PREFIXES = ("SERIAL1_", "SERIAL0_", "NET_ENABLE", "NET_OPTIONS", "BRD_SER1_RTSCTS")
SERVO_PARAM_PREFIXES = ("SERVO7_FUNCTION", "SERVO8_FUNCTION")
RATO_PARAM_PREFIXES = ("RATO_",)
ARSPD_PARAM_PREFIXES = ("ARSPD_TYPE",)


def matches_any(name, prefixes):
    return any(name == p or name.startswith(p) for p in prefixes)


def fetch_all_params(master, timeout_s=60.0, quiet_s=3.0):
    """Read-only PARAM_REQUEST_LIST + collect PARAM_VALUE until either the
    reported param_count is reached or quiet_s elapses with no new params.
    """
    master.mav.param_request_list_send(master.target_system, master.target_component)
    params = {}
    expected_count = None
    deadline = time.time() + timeout_s
    last_new_param = time.time()
    while time.time() < deadline:
        msg = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
        if msg is None:
            if time.time() - last_new_param > quiet_s and (expected_count is None or len(params) > 0):
                break
            continue
        pid = msg.param_id.rstrip("\x00") if isinstance(msg.param_id, str) else msg.param_id
        if pid not in params:
            last_new_param = time.time()
        params[pid] = msg.param_value
        expected_count = msg.param_count
        if expected_count and len(params) >= expected_count:
            break
    return params, expected_count


def capture_version_info(master, statustext_window_s=3.0):
    """Read-only AUTOPILOT_VERSION request + a short STATUSTEXT capture
    window (the connect-time banner, including any custom firmware
    string set via AP_CUSTOM_FIRMWARE_STRING)."""
    from pymavlink import mavutil

    lines = []
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0,
        mavutil.mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION, 0, 0, 0, 0, 0, 0,
    )
    deadline = time.time() + statustext_window_s
    while time.time() < deadline:
        msg = master.recv_match(type=["AUTOPILOT_VERSION", "STATUSTEXT", "HEARTBEAT"], blocking=True, timeout=0.5)
        if msg is None:
            continue
        if msg.get_type() == "AUTOPILOT_VERSION":
            lines.append(f"AUTOPILOT_VERSION: flight_sw_version={msg.flight_sw_version} "
                         f"board_version={msg.board_version} flight_custom_version={bytes(msg.flight_custom_version).hex()}")
        elif msg.get_type() == "STATUSTEXT":
            lines.append(f"STATUSTEXT[{msg.severity}]: {msg.text}")
        elif msg.get_type() == "HEARTBEAT":
            lines.append(f"HEARTBEAT: type={msg.type} autopilot={msg.autopilot} base_mode={msg.base_mode} "
                          f"custom_mode={msg.custom_mode} system_status={msg.system_status}")
    return lines


def build_arg_parser():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--pixhawk", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--backup-dir", default=str(DEFAULT_BACKUP_DIR))
    parser.add_argument("--timeout-s", type=float, default=60.0)
    parser.add_argument("--label", default=None, help="Optional label appended to the timestamped filenames, "
                                                        "e.g. 'pre-flash' or 'post-flash'")
    return parser


def main():
    args = build_arg_parser().parse_args()
    from pymavlink import mavutil

    backup_dir = Path(args.backup_dir)
    backup_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    suffix = f"_{args.label}" if args.label else ""
    params_path = backup_dir / f"sr75_hil_f24c_params_{stamp}{suffix}.parm"
    version_path = backup_dir / f"sr75_hil_f24c_version_{stamp}{suffix}.txt"

    print(f"Connecting to {args.pixhawk} at {args.baud} baud (read-only)...")
    master = mavutil.mavlink_connection(args.pixhawk, baud=args.baud, autoreconnect=True)
    hb = master.wait_heartbeat(timeout=30)
    if hb is None:
        print("ERROR: no heartbeat received within 30s", file=sys.stderr)
        return 2
    armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    print(f"Heartbeat OK: type={hb.type} autopilot={hb.autopilot} armed={int(armed)} "
          f"mode={mavutil.mode_string_v10(hb)}")

    print("Fetching full parameter set (PARAM_REQUEST_LIST, read-only)...")
    params, expected_count = fetch_all_params(master, timeout_s=args.timeout_s)
    print(f"Fetched {len(params)} parameters (reported count={expected_count})")

    print("Capturing AUTOPILOT_VERSION / boot banner (read-only)...")
    version_lines = capture_version_info(master)
    master.close()

    with open(params_path, "w") as f:
        for name in sorted(params):
            f.write(f"{name}\t{params[name]}\n")
    print(f"Full parameter backup written: {params_path}")

    with open(version_path, "w") as f:
        f.write("\n".join(version_lines) + "\n")
    print(f"Firmware/version capture written: {version_path}")

    for label, prefixes in (
        ("GPS/EKF source", GPS_EKF_PARAM_PREFIXES),
        ("Airspeed", ARSPD_PARAM_PREFIXES),
        ("Serial/PPP", SERIAL_PARAM_PREFIXES),
        ("Servo function", SERVO_PARAM_PREFIXES),
        ("RATO", RATO_PARAM_PREFIXES),
    ):
        subset = {n: v for n, v in params.items() if matches_any(n, prefixes)}
        print(f"--- {label} ({len(subset)}) ---")
        for name in sorted(subset):
            print(f"  {name} = {subset[name]}")

    print("Safety confirmation: only PARAM_REQUEST_LIST and a read-only AUTOPILOT_VERSION "
          "request were sent; no PARAM_SET, arm, mission, RC override, or actuator command.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
