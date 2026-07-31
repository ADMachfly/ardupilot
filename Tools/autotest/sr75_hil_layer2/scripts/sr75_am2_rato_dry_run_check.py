#!/usr/bin/env python3
"""SR-75 HIL-F23-B: AM2 RATO resume param dry-run / CH7-CH8 PWM readback check.

Two independent modes, both safe by construction:

1. Param-file validation (always runs, no hardware needed):
   parses --param-file and confirms the AM2 RATO resume set is present with
   the exact values validated in SITL (F22-GZ-AH..AM2), and flags anything
   that looks unsafe to load on a bench (e.g. ARMING_CHECK/ARMING_REQUIRE
   disabled -- those are fine for unattended SITL automation but must NOT be
   carried onto real hardware).

2. Optional live CH7/CH8 PWM readback (only if --pixhawk is given):
   connects to a real Pixhawk, waits for heartbeat, requests
   SERVO_OUTPUT_RAW, and PASSIVELY logs servo7/servo8 PWM for
   --readback-duration-s seconds. This script:
     - never arms the vehicle
     - never sends PARAM_SET, RC_CHANNELS_OVERRIDE, or any actuator command
     - never uploads or starts a mission
     - never enables HIL/GPS_INPUT injection
   It is read-only monitoring, matching the same "no actuator output" safety
   posture already used in bridge/sr75_jsbsim_pixhawk_hil_bridge.py.

SR-75 LAYER 2 HIL BENCH SAFETY (unchanged from bridge/README.md):
No live engine, no fuel pump, no live RATO ignition, no live ejection.
CH7/CH8 must be wired to dummy loads, LEDs, a PWM logger, or left
disconnected only.
"""
import argparse
import csv
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[4]
DEFAULT_PARAM_FILE = REPO / "Tools/autotest/sr75_hil_layer2/SR75_LAYER2_AM2_HIL_SAFE.param"

# The exact AM2-validated RATO resume set (see
# sr75_f22gzam2_extended_altitude_profile_v2.py). Value is None where any
# value is acceptable (not checked), otherwise the expected value.
REQUIRED_RATO_PARAMS = {
    "RATO_ENABLE": "1",
    "RATO_RESUME": "1",
    "RATO_RES_BURN": "0.768",
    "RATO_IGN_CH": "7",
    "RATO_EJ_CH": "8",
    "RATO_BURN_S": "3.0",
    "RATO_PITCH": "20.0",
    "RATO_REL_MODE": "1",
    "RATO_EJECT_S": "10.0",
}
REQUIRED_SAFETY_PARAMS = {
    "FS_SHORT_ACTN": "0",
    "FS_LONG_ACTN": "0",
}
# Params that are fine in a SITL automation harness but must NOT appear
# (or must not be set to a bypassing value) in a bench/HIL param file.
UNSAFE_ON_BENCH = {
    "ARMING_CHECK": "0",
    "ARMING_REQUIRE": "0",
}


def parse_param_file(path):
    """Return {name: value_string} preserving the file's own formatting."""
    params = {}
    with open(path) as f:
        for line_no, raw_line in enumerate(f, start=1):
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) != 2:
                print(f"WARNING: {path}:{line_no}: unexpected line format: {raw_line!r}")
                continue
            name, value = parts
            if name in params:
                print(f"WARNING: {path}:{line_no}: duplicate param {name} (was {params[name]}, now {value})")
            params[name] = value
    return params


def validate_params(params, path):
    ok = True
    print(f"=== Param file validation: {path} ===")
    print(f"parsed {len(params)} parameters")

    for name, expected in REQUIRED_RATO_PARAMS.items():
        actual = params.get(name)
        if actual is None:
            print(f"FAIL: missing required RATO param {name}")
            ok = False
        elif actual != expected:
            print(f"FAIL: {name}={actual}, expected {expected} (AM2-validated value)")
            ok = False
        else:
            print(f"OK:   {name}={actual}")

    for name, expected in REQUIRED_SAFETY_PARAMS.items():
        actual = params.get(name)
        if actual is None:
            print(f"FAIL: missing required safety param {name}")
            ok = False
        elif actual != expected:
            print(f"FAIL: {name}={actual}, expected {expected} (bench-safe value)")
            ok = False
        else:
            print(f"OK:   {name}={actual}")

    for name, unsafe_value in UNSAFE_ON_BENCH.items():
        actual = params.get(name)
        if actual == unsafe_value:
            print(
                f"FAIL: {name}={actual} bypasses a real-hardware safety check -- "
                "this is acceptable in SITL automation params but must NOT be loaded on a bench Pixhawk"
            )
            ok = False
        elif actual is not None:
            print(f"OK:   {name}={actual} (not a bypass value)")
        else:
            print(f"OK:   {name} not set (ArduPilot default, safety check remains active)")

    print(f"=== Param file validation: {'PASS' if ok else 'FAIL'} ===")
    return ok


def run_readback(args):
    """Connect to a real Pixhawk and passively log SERVO_OUTPUT_RAW CH7/CH8.

    Never arms. Never sends PARAM_SET, RC override, mission, or actuator
    commands. Read-only.
    """
    from pymavlink import mavutil

    print()
    print("=== CH7/CH8 PWM readback (live hardware) ===")
    print(f"Connecting to {args.pixhawk} at {args.baud} baud (read-only, no actuator output)...")
    master = mavutil.mavlink_connection(args.pixhawk, baud=args.baud, autoreconnect=True)

    print("Waiting for heartbeat...")
    hb = master.wait_heartbeat(timeout=30)
    if hb is None:
        print("FAIL: no heartbeat received within 30s")
        return False
    armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    print(f"Heartbeat OK: system={master.target_system} component={master.target_component} armed={int(armed)}")
    if armed:
        print(
            "WARNING: vehicle reports ARMED. This script will not send any command to "
            "disarm or arm it -- if this is unexpected, disarm via your normal GCS/RC "
            "safety procedure before proceeding."
        )

    message_id = mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW
    interval_us = int(1_000_000 / args.rate_hz)
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0, message_id, interval_us, 0, 0, 0, 0, 0,
    )
    print(f"Requested SERVO_OUTPUT_RAW at {args.rate_hz:g} Hz")

    rows = []
    deadline = time.time() + args.readback_duration_s
    print(f"Passively logging CH7/CH8 PWM for {args.readback_duration_s:.0f}s (no commands will be sent)...")
    while time.time() < deadline:
        msg = master.recv_match(type="SERVO_OUTPUT_RAW", blocking=True, timeout=1.0)
        if msg is None:
            continue
        row = {
            "t": time.time(),
            "servo7_raw": msg.servo7_raw,
            "servo8_raw": msg.servo8_raw,
        }
        rows.append(row)
        print(f"t={row['t']:.3f} CH7(IGN)={row['servo7_raw']} CH8(EJ)={row['servo8_raw']}")

    if args.log_csv:
        with open(args.log_csv, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=["t", "servo7_raw", "servo8_raw"])
            writer.writeheader()
            writer.writerows(rows)
        print(f"Logged {len(rows)} samples to {args.log_csv}")

    if not rows:
        print("FAIL: no SERVO_OUTPUT_RAW samples received")
        return False
    print(f"=== CH7/CH8 readback: PASS ({len(rows)} samples) ===")
    return True


def build_arg_parser():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--param-file", default=str(DEFAULT_PARAM_FILE), help="Param file to validate")
    parser.add_argument(
        "--pixhawk", default=None,
        help="Optional Pixhawk serial device (e.g. /dev/ttyACM0) for live CH7/CH8 PWM readback. "
             "If omitted (the default), only the param file is validated -- no hardware is touched.",
    )
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--rate-hz", type=float, default=5.0, help="SERVO_OUTPUT_RAW request rate")
    parser.add_argument("--readback-duration-s", type=float, default=10.0)
    parser.add_argument("--log-csv", default=None, help="Optional CSV path for the readback log")
    return parser


def main():
    args = build_arg_parser().parse_args()
    path = Path(args.param_file)
    if not path.exists():
        print(f"ERROR: param file not found: {path}", file=sys.stderr)
        return 2

    params = parse_param_file(path)
    file_ok = validate_params(params, path)

    if args.pixhawk is None:
        print()
        print("No --pixhawk given: file-validation-only dry run. No hardware was touched.")
        return 0 if file_ok else 1

    if not file_ok:
        print()
        print("Param file validation FAILED -- refusing to proceed to live readback.")
        return 1

    readback_ok = run_readback(args)
    return 0 if readback_ok else 1


if __name__ == "__main__":
    sys.exit(main())
