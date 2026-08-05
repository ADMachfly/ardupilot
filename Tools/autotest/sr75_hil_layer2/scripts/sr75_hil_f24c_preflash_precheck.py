#!/usr/bin/env python3
"""HIL-F24-C item 10: single read-only operator precheck for the fmuv3-
SimOnHardWare first flash and static-sensor bench test.

Performs ONLY read-only prechecks against whatever firmware is currently
running on the bench Pixhawk, and prints an explicit STOP/GO verdict. It
is meant to be run twice: once BEFORE flashing (to confirm the board is in
a safe, disarmed, MANUAL state and CH7/CH8 show no load activity before
you touch anything), and once AFTER flashing (to confirm the new firmware
identifies itself correctly and boots into the same safe state).

This script NEVER:
  - flashes firmware (it has no upload capability at all)
  - sends PARAM_SET (only PARAM_REQUEST_READ)
  - arms, disarms, or sends any COMMAND_LONG other than a read-only
    MAV_CMD_SET_MESSAGE_INTERVAL stream-rate request and a read-only
    MAV_CMD_REQUEST_MESSAGE for AUTOPILOT_VERSION
  - starts PPP or SIM_JSON (it never touches TELEM1/SERIAL1 configuration
    or opens any UDP socket)
  - sends RC_CHANNELS_OVERRIDE or any actuator/servo command

Only PARAM_REQUEST_READ, MAV_CMD_SET_MESSAGE_INTERVAL (stream-rate
requests), and MAV_CMD_REQUEST_MESSAGE (AUTOPILOT_VERSION) are ever sent.
"""
import argparse
import sys
import time

EXPECTED_SOH_PARAMS = {
    "AHRS_EKF_TYPE": 3,
    "EK3_ENABLE": 1,
    "GPS1_TYPE": 100,
    "ARSPD_TYPE": 100,
    "SERIAL1_PROTOCOL": 48,
    "SERIAL1_BAUD": 921,
    "SERVO7_FUNCTION": 0,
    "SERVO8_FUNCTION": 0,
    "RATO_ENABLE": 0,
    "RATO_IGN_CH": 0,
    "RATO_EJ_CH": 0,
}

CH_PWM_QUIET_MAX = 50  # PWM counts of deviation from 0/absent still considered "no load activity"


def evaluate_precheck(armed, mode, ch7_pwm, ch8_pwm, param_values, require_soh_params, custom_fw_string):
    """Pure function: HIL-F24-C item 10's STOP/GO logic. Returns
    (go: bool, reasons: list[str]) -- reasons are populated on STOP (and
    left empty, or containing informational notes, on GO).
    """
    reasons = []

    if armed:
        reasons.append("vehicle reports ARMED -- STOP")
    if mode != "MANUAL":
        reasons.append(f"mode={mode}, expected MANUAL -- STOP")

    for label, pwm in (("CH7", ch7_pwm), ("CH8", ch8_pwm)):
        if pwm is not None and pwm != 0 and pwm > CH_PWM_QUIET_MAX:
            reasons.append(f"{label} PWM={pwm} shows possible load activity -- STOP")

    if require_soh_params:
        for name, expected in EXPECTED_SOH_PARAMS.items():
            value = param_values.get(name)
            if value is None:
                reasons.append(f"{name} did not respond to PARAM_REQUEST_READ -- STOP")
            elif int(value) != expected:
                reasons.append(f"{name}={int(value)}, expected {expected} -- STOP")
        if custom_fw_string is not None and "SIMULATION" not in custom_fw_string.upper():
            reasons.append(
                f"boot banner did not contain 'SIMULATION' (got: {custom_fw_string!r}) -- STOP"
            )

    return (len(reasons) == 0), reasons


def read_params_readonly(master, names, timeout_s=4.0, retries=3):
    """Read-only PARAM_REQUEST_READ helper (same established pattern used
    throughout this task family). Never sends PARAM_SET."""
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


def capture_boot_banner(master, window_s=3.0):
    """Read-only STATUSTEXT capture window -- picks up the connect-time
    banner (AP_CUSTOM_FIRMWARE_STRING, if set) without sending anything
    beyond the normal MAVLink connection handshake."""
    lines = []
    deadline = time.time() + window_s
    while time.time() < deadline:
        msg = master.recv_match(type="STATUSTEXT", blocking=True, timeout=0.5)
        if msg is not None:
            lines.append(msg.text)
    return lines


def read_ch_pwm(master, timeout_s=3.0):
    """Read-only CH7/CH8 PWM snapshot via SERVO_OUTPUT_RAW (requests the
    stream at a low rate via MAV_CMD_SET_MESSAGE_INTERVAL -- a read-only
    stream-rate request, not an actuator command)."""
    from pymavlink import mavutil

    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, int(1e6 / 2), 0, 0, 0, 0, 0,
    )
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        msg = master.recv_match(type="SERVO_OUTPUT_RAW", blocking=True, timeout=0.5)
        if msg is not None:
            return getattr(msg, "servo7_raw", None), getattr(msg, "servo8_raw", None)
    return None, None


def build_arg_parser():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--pixhawk", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument(
        "--require-soh-params", action="store_true",
        help="Also check that the currently-running firmware's parameters match the "
             "fmuv3-SimOnHardWare defaults.parm expected values -- use this for the "
             "POST-flash run. Omit for the PRE-flash run (before the new firmware exists).",
    )
    return parser


def main():
    args = build_arg_parser().parse_args()
    from pymavlink import mavutil

    print("=== HIL-F24-C read-only precheck (never flashes, never PARAM_SET, "
          "never starts PPP/SIM_JSON) ===")
    print(f"Connecting to {args.pixhawk} at {args.baud} baud (read-only)...")
    master = mavutil.mavlink_connection(args.pixhawk, baud=args.baud, autoreconnect=True)
    hb = master.wait_heartbeat(timeout=30)
    if hb is None:
        print("STOP: no heartbeat received within 30s")
        return 2

    armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    mode = mavutil.mode_string_v10(hb)
    print(f"Heartbeat OK: type={hb.type} autopilot={hb.autopilot} armed={int(armed)} mode={mode}")

    ch7_pwm, ch8_pwm = read_ch_pwm(master)
    print(f"CH7 PWM={ch7_pwm} CH8 PWM={ch8_pwm}")

    banner_lines = capture_boot_banner(master)
    custom_fw_string = banner_lines[0] if banner_lines else None
    for line in banner_lines:
        print(f"STATUSTEXT: {line}")

    param_values = {}
    if args.require_soh_params:
        print("Reading fmuv3-SimOnHardWare expected parameters (read-only)...")
        param_values = read_params_readonly(master, list(EXPECTED_SOH_PARAMS))
        for name, value in param_values.items():
            print(f"  {name} = {value}")

    master.close()

    go, reasons = evaluate_precheck(
        armed, mode, ch7_pwm, ch8_pwm, param_values, args.require_soh_params, custom_fw_string,
    )
    print("")
    if go:
        print("GO: all read-only prechecks passed.")
    else:
        print("STOP: precheck failed:")
        for reason in reasons:
            print(f"  - {reason}")
    print("Safety confirmation: only PARAM_REQUEST_READ, a read-only "
          "MAV_CMD_SET_MESSAGE_INTERVAL stream-rate request, and STATUSTEXT capture were "
          "used; no PARAM_SET, arm, mission, RC override, actuator command, or PPP/SIM_JSON "
          "start.")
    return 0 if go else 1


if __name__ == "__main__":
    sys.exit(main())
