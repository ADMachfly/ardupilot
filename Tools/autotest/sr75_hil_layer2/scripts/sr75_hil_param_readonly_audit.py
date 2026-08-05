#!/usr/bin/env python3
"""SR-75 HIL-F23-E: read-only onboard parameter audit vs the AM2 HIL-safe
param file.

Connects to a live Pixhawk and compares its CURRENT onboard parameter
values against Tools/autotest/sr75_hil_layer2/SR75_LAYER2_AM2_HIL_SAFE.param
(created in HIL-F23-B), without ever writing anything to the vehicle.

Only these MAVLink messages are ever sent:
  - the connection's own outbound heartbeat (via mavutil's normal recv loop;
    this script does not explicitly send one)
  - PARAM_REQUEST_READ (one per parameter of interest, by name)
  - MAV_CMD_SET_MESSAGE_INTERVAL for SERVO_OUTPUT_RAW (a stream-rate
    request, not an actuator command) for the optional idle CH7/CH8 snapshot

This script NEVER sends: PARAM_SET, COMMAND_LONG arm/disarm, mission
upload/start, RC_CHANNELS_OVERRIDE, or any actuator/servo command.

SR-75 LAYER 2 HIL BENCH SAFETY:
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

# Parameters this audit checks on the live board, grouped for the report.
RATO_PARAMS = [
    "RATO_ENABLE", "RATO_RESUME", "RATO_RES_BURN", "RATO_IGN_CH",
    "RATO_EJ_CH", "RATO_BURN_S", "RATO_PITCH", "RATO_REL_MODE", "RATO_EJECT_S",
]
SAFETY_PARAMS = ["FS_SHORT_ACTN", "FS_LONG_ACTN", "ARMING_CHECK", "ARMING_REQUIRE"]
# SERIAL/TELEM params relevant to the link investigated in HIL-F23-C/D
# (TELEM1 PPP vs MAVLink-rollback state).
SERIAL_PARAMS = [
    "SERIAL1_PROTOCOL", "SERIAL1_BAUD", "SERIAL1_OPTIONS",
    "BRD_SER1_RTSCTS", "NET_ENABLE",
]
SERVO_PARAMS = ["SERVO7_FUNCTION", "SERVO8_FUNCTION"]

ALL_PARAMS = RATO_PARAMS + SAFETY_PARAMS + SERIAL_PARAMS + SERVO_PARAMS


def parse_param_file(path):
    params = {}
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) == 2:
                params[parts[0]] = parts[1]
    return params


def fmt_value(value):
    if value is None:
        return "(not received)"
    if isinstance(value, float) and value == int(value):
        return str(int(value))
    return str(value)


def values_match(onboard, expected_str):
    if onboard is None or expected_str is None:
        return None  # unknown, not a mismatch call
    try:
        return abs(float(onboard) - float(expected_str)) < 1e-6
    except (TypeError, ValueError):
        return str(onboard) == expected_str


def read_params(master, names, timeout_s, log_lines):
    """PARAM_REQUEST_READ each name individually; read-only."""
    results = {}
    for name in names:
        master.mav.param_request_read_send(
            master.target_system, master.target_component,
            name.encode("utf-8"), -1,
        )
    deadline = time.time() + timeout_s
    remaining = set(names)
    while time.time() < deadline and remaining:
        msg = master.recv_match(type=["PARAM_VALUE", "STATUSTEXT"], blocking=True, timeout=0.5)
        if msg is None:
            continue
        if msg.get_type() == "STATUSTEXT":
            line = f"STATUSTEXT: {msg.text}"
            print(line)
            log_lines.append(line)
            continue
        pid = msg.param_id.rstrip("\x00") if isinstance(msg.param_id, str) else msg.param_id
        if pid in remaining:
            results[pid] = msg.param_value
            remaining.discard(pid)
            line = f"PARAM_VALUE {pid} = {msg.param_value}"
            print(line)
            log_lines.append(line)
    for missing in remaining:
        results[missing] = None
        line = f"PARAM_VALUE {missing} = (no response within {timeout_s:.0f}s)"
        print(line)
        log_lines.append(line)
    return results


def idle_servo_snapshot(master, duration_s, log_lines):
    """Passive SERVO_OUTPUT_RAW snapshot for CH7/CH8. Read-only."""
    from pymavlink import mavutil

    message_id = mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0, message_id, int(1_000_000 / 5), 0, 0, 0, 0, 0,
    )
    line = "Requested SERVO_OUTPUT_RAW at 5 Hz (stream-rate request, not an actuator command)"
    print(line)
    log_lines.append(line)

    samples = []
    deadline = time.time() + duration_s
    while time.time() < deadline:
        msg = master.recv_match(type="SERVO_OUTPUT_RAW", blocking=True, timeout=1.0)
        if msg is None:
            continue
        samples.append((time.time(), msg.servo7_raw, msg.servo8_raw))
        line = f"SERVO_OUTPUT_RAW CH7(IGN)={msg.servo7_raw} CH8(EJ)={msg.servo8_raw}"
        print(line)
        log_lines.append(line)
    return samples


def build_arg_parser():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--pixhawk", required=True, help="Pixhawk serial device, e.g. /dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--param-file", default=str(DEFAULT_PARAM_FILE))
    parser.add_argument("--param-timeout-s", type=float, default=15.0)
    parser.add_argument("--idle-servo-duration-s", type=float, default=5.0,
                         help="0 to skip the idle CH7/CH8 snapshot")
    parser.add_argument("--log", default=None, help="Text log path")
    parser.add_argument("--csv", default=None, help="CSV comparison-table path")
    return parser


def main():
    from pymavlink import mavutil

    args = build_arg_parser().parse_args()
    log_lines = []

    def log(line):
        print(line)
        log_lines.append(line)

    file_params = parse_param_file(Path(args.param_file))
    log(f"Loaded {len(file_params)} params from {args.param_file}")

    log(f"Connecting to {args.pixhawk} at {args.baud} baud (read-only)...")
    master = mavutil.mavlink_connection(args.pixhawk, baud=args.baud, autoreconnect=True)

    log("Waiting for heartbeat...")
    hb = master.wait_heartbeat(timeout=30)
    if hb is None:
        log("FAIL: no heartbeat received within 30s")
        _write_log(args.log, log_lines)
        return 2
    armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    mode = mavutil.mode_string_v10(hb)
    log(
        f"HEARTBEAT: type={hb.type} autopilot={hb.autopilot} base_mode={hb.base_mode} "
        f"custom_mode={hb.custom_mode} system_status={hb.system_status} "
        f"armed={int(armed)} mode={mode}"
    )
    if armed:
        log(
            "WARNING: vehicle reports ARMED. This script will not send any arm/disarm "
            "command -- verify safety state manually before proceeding."
        )

    log("")
    log("=== Reading RATO/safety/serial/servo params (PARAM_REQUEST_READ only) ===")
    onboard = read_params(master, ALL_PARAMS, args.param_timeout_s, log_lines)

    log("")
    log("=== Onboard vs SR75_LAYER2_AM2_HIL_SAFE.param comparison ===")
    rows = []
    mismatches = []
    for name in ALL_PARAMS:
        onboard_val = onboard.get(name)
        expected_val = file_params.get(name)
        match = values_match(onboard_val, expected_val)
        if expected_val is None:
            status = "NOT_IN_FILE"
        elif onboard_val is None:
            status = "NO_RESPONSE"
        elif match:
            status = "MATCH"
        else:
            status = "MISMATCH"
            mismatches.append(name)
        line = f"{name:20s} onboard={fmt_value(onboard_val):>10s}  expected={fmt_value(expected_val):>10s}  {status}"
        log(line)
        rows.append({
            "param": name,
            "onboard": fmt_value(onboard_val),
            "expected_file": fmt_value(expected_val),
            "status": status,
        })

    log("")
    if mismatches:
        log(f"MISMATCHES: {', '.join(mismatches)}")
    else:
        log("No mismatches among params present in both onboard and file (see NOT_IN_FILE/NO_RESPONSE rows above for gaps).")

    servo_samples = []
    if args.idle_servo_duration_s > 0:
        log("")
        log(f"=== Idle CH7/CH8 readback ({args.idle_servo_duration_s:.0f}s, passive) ===")
        servo_samples = idle_servo_snapshot(master, args.idle_servo_duration_s, log_lines)
        if servo_samples:
            last_t, last7, last8 = servo_samples[-1]
            log(f"Last idle sample: CH7(IGN)={last7} CH8(EJ)={last8}")
        else:
            log("No SERVO_OUTPUT_RAW samples received during idle snapshot window")

    log("")
    log("Safety confirmation: no PARAM_SET, no arm/disarm command, no mission "
        "upload/start, no RC override, no actuator command were sent by this script.")

    _write_log(args.log, log_lines)
    if args.csv:
        with open(args.csv, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=["param", "onboard", "expected_file", "status"])
            writer.writeheader()
            writer.writerows(rows)
        print(f"Comparison table written to {args.csv}")

    return 1 if mismatches else 0


def _write_log(path, lines):
    if not path:
        return
    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")
    print(f"Log written to {path}")


if __name__ == "__main__":
    sys.exit(main())
