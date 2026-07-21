#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Localhost-only SR-75 Layer 2J-B3 ArduPlane SITL control helper."""

import argparse
import math
import sys
import time
from typing import Dict, Iterable, Optional, Tuple

from pymavlink import mavutil


PLANE_TYPE = mavutil.mavlink.MAV_TYPE_FIXED_WING
ARDUPILOT_AUTOPILOT = mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA
ARMED_FLAG = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED

REQUESTED_MESSAGES = {
    "ATTITUDE": mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
    "RC_CHANNELS": mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS,
    "SERVO_OUTPUT_RAW": mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW,
    "EKF_STATUS_REPORT": mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT,
    "AHRS2": mavutil.mavlink.MAVLINK_MSG_ID_AHRS2,
    "STATUSTEXT": mavutil.mavlink.MAVLINK_MSG_ID_STATUSTEXT,
    "SYS_STATUS": mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS,
}


def parse_endpoint(endpoint: str) -> Tuple[str, str]:
    parts = endpoint.split(":")
    if len(parts) < 3:
        raise ValueError("endpoint must look like tcp:127.0.0.1:5760 or udpin:127.0.0.1:14550")
    scheme = parts[0]
    host = parts[1]
    return scheme, host


def endpoint_is_loopback(endpoint: str) -> bool:
    scheme, host = parse_endpoint(endpoint)
    return scheme in ("tcp", "udp", "udpin", "udpout") and host in ("127.0.0.1", "localhost", "::1")


def require_loopback(endpoint: str, allow_non_loopback: bool) -> None:
    if allow_non_loopback:
        return
    if not endpoint_is_loopback(endpoint):
        raise ValueError(f"refusing non-loopback MAVLink endpoint: {endpoint}")


def request_message_interval(master, message_id: int, rate_hz: float) -> None:
    interval_us = 0 if rate_hz <= 0.0 else int(1000000.0 / rate_hz)
    master.mav.command_long_send(
        master.target_system,
        command_target_component(master),
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        message_id,
        interval_us,
        0,
        0,
        0,
        0,
        0,
    )


def request_streams(master) -> None:
    for message_id in REQUESTED_MESSAGES.values():
        request_message_interval(master, message_id, 10.0)


def send_gcs_heartbeat(master) -> None:
    master.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0,
        0,
        mavutil.mavlink.MAV_STATE_ACTIVE,
    )


def mode_name(master, custom_mode: int) -> str:
    mapping = master.mode_mapping() or {}
    reverse = {mode: name for name, mode in mapping.items()}
    return reverse.get(custom_mode, str(custom_mode))


def heartbeat_armed(heartbeat) -> bool:
    return bool(heartbeat.base_mode & ARMED_FLAG)


def wait_heartbeat(master, timeout_s: float):
    heartbeat = master.wait_heartbeat(timeout=timeout_s)
    if heartbeat is None:
        raise TimeoutError("timed out waiting for ArduPlane heartbeat")
    if heartbeat.type != PLANE_TYPE or heartbeat.autopilot != ARDUPILOT_AUTOPILOT:
        raise RuntimeError(
            "unexpected heartbeat: "
            f"type={heartbeat.type} autopilot={heartbeat.autopilot}"
        )
    return heartbeat


def set_fbwa(master, timeout_s: float) -> None:
    mapping = master.mode_mapping() or {}
    if "FBWA" not in mapping:
        raise RuntimeError("FBWA mode is not present in ArduPlane mode mapping")
    master.set_mode(mapping["FBWA"])
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        heartbeat = master.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
        if heartbeat is not None and mode_name(master, heartbeat.custom_mode) == "FBWA":
            return
    raise TimeoutError("timed out waiting for FBWA mode")


def command_arm(master, arm: bool) -> None:
    master.mav.command_long_send(
        master.target_system,
        command_target_component(master),
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1.0 if arm else 0.0,
        0,
        0,
        0,
        0,
        0,
        0,
    )


def command_target_component(master) -> int:
    return master.target_component if master.target_component != 0 else 1


def result_name(result: int) -> str:
    enum = mavutil.mavlink.enums.get("MAV_RESULT", {})
    entry = enum.get(result)
    return str(result) if entry is None else entry.name


def wait_command_ack(master, command: int, timeout_s: float) -> Optional[object]:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        message = master.recv_match(type=("COMMAND_ACK", "STATUSTEXT"), blocking=True, timeout=0.5)
        if message is None:
            continue
        if message.get_type() == "STATUSTEXT":
            text = getattr(message, "text", "")
            if text:
                print(f"STATUSTEXT {text}")
            continue
        if getattr(message, "command", None) == command:
            return message
    return None


def wait_armed(master, armed: bool, timeout_s: float) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        heartbeat = master.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
        if heartbeat is not None and heartbeat_armed(heartbeat) == armed:
            return
    state = "armed" if armed else "disarmed"
    raise TimeoutError(f"timed out waiting for {state} state")


def message_fields(message, names: Iterable[str]) -> Tuple[object, ...]:
    if message is None:
        return tuple("" for _ in names)
    return tuple(getattr(message, name, "") for name in names)


def servo_values(message) -> Tuple[object, ...]:
    return message_fields(message, [f"servo{index}_raw" for index in range(1, 9)])


def rc_values(message) -> Tuple[object, ...]:
    return message_fields(message, [f"chan{index}_raw" for index in range(1, 9)])


def valid_primary_pwm(servo) -> bool:
    if servo is None:
        return False
    values = servo_values(servo)[:4]
    return all(isinstance(value, int) and 1000 <= value <= 2000 for value in values)


def print_table_row(
    started: float,
    heartbeat,
    attitude,
    rc_channels,
    servo,
    invalid_reason: str = "",
) -> None:
    roll = "" if attitude is None else f"{math.degrees(attitude.roll):.3f}"
    pitch = "" if attitude is None else f"{math.degrees(attitude.pitch):.3f}"
    servos = servo_values(servo)
    rc = rc_values(rc_channels)
    pwm1, pwm2, pwm3, pwm4, _, _, pwm7, _ = servos
    rc1, rc2, rc3, rc4, _, _, rc7, _ = rc
    armed = "" if heartbeat is None else int(heartbeat_armed(heartbeat))
    mode = "" if heartbeat is None else mode_name(NoneMaster(heartbeat.custom_mode), heartbeat.custom_mode)
    print(
        f"{time.monotonic() - started:.2f} "
        f"armed={armed} mode={mode} roll={roll} pitch={pitch} "
        f"PWM1={pwm1} PWM2={pwm2} PWM3={pwm3} PWM4={pwm4} PWM7={pwm7} "
        f"RC1={rc1} RC2={rc2} RC3={rc3} RC4={rc4} RC7={rc7} "
        f"{invalid_reason}"
    )


class NoneMaster:
    def __init__(self, custom_mode: int):
        self._custom_mode = custom_mode

    def mode_mapping(self) -> Dict[str, int]:
        return {
            "MANUAL": 0,
            "CIRCLE": 1,
            "STABILIZE": 2,
            "TRAINING": 3,
            "ACRO": 4,
            "FBWA": 5,
            "FBWB": 6,
            "CRUISE": 7,
            "AUTOTUNE": 8,
            "AUTO": 10,
            "RTL": 11,
            "LOITER": 12,
            "TAKEOFF": 13,
            "AVOID_ADSB": 14,
            "GUIDED": 15,
            "INITIALISING": 16,
            "QSTABILIZE": 17,
            "QHOVER": 18,
            "QLOITER": 19,
            "QLAND": 20,
            "QRTL": 21,
            "QAUTOTUNE": 22,
            "QACRO": 23,
            "THERMAL": 24,
            "LOITERALTQLAND": 25,
        }


def collect_status(master, duration_s: float) -> Dict[str, object]:
    latest: Dict[str, Optional[object]] = {
        "HEARTBEAT": None,
        "ATTITUDE": None,
        "RC_CHANNELS": None,
        "SERVO_OUTPUT_RAW": None,
        "EKF_STATUS_REPORT": None,
        "AHRS2": None,
        "SYS_STATUS": None,
    }
    statustext = []
    deadline = time.monotonic() + duration_s
    while time.monotonic() < deadline:
        message = master.recv_match(blocking=True, timeout=0.2)
        if message is None:
            continue
        message_type = message.get_type()
        if message_type == "BAD_DATA":
            continue
        if message_type in latest:
            latest[message_type] = message
        if message_type == "STATUSTEXT":
            text = getattr(message, "text", "")
            if text:
                statustext.append(text)
    latest["STATUSTEXT"] = statustext
    return latest


def print_status(master, latest: Dict[str, object]) -> None:
    heartbeat = latest.get("HEARTBEAT")
    ekf = latest.get("EKF_STATUS_REPORT")
    ahrs2 = latest.get("AHRS2")
    sys_status = latest.get("SYS_STATUS")
    rc = latest.get("RC_CHANNELS")
    servo = latest.get("SERVO_OUTPUT_RAW")
    if heartbeat is not None:
        print(
            "HEARTBEAT "
            f"armed={int(heartbeat_armed(heartbeat))} "
            f"mode={mode_name(master, heartbeat.custom_mode)} "
            f"type={heartbeat.type} autopilot={heartbeat.autopilot}"
        )
    print(f"EKF_STATUS_REPORT flags={'' if ekf is None else ekf.flags}")
    print(f"AHRS2 present={int(ahrs2 is not None)}")
    if sys_status is not None:
        print(
            "SYS_STATUS "
            f"present=0x{sys_status.onboard_control_sensors_present:x} "
            f"enabled=0x{sys_status.onboard_control_sensors_enabled:x} "
            f"health=0x{sys_status.onboard_control_sensors_health:x}"
        )
    print("RC_CHANNELS " + " ".join(f"RC{index}={value}" for index, value in enumerate(rc_values(rc), start=1)))
    print("SERVO_OUTPUT_RAW " + " ".join(f"PWM{index}={value}" for index, value in enumerate(servo_values(servo), start=1)))
    for text in latest.get("STATUSTEXT", []):
        print(f"STATUSTEXT {text}")


def monitor_table(master, duration_s: float) -> None:
    print("time armed mode roll pitch PWM1 PWM2 PWM3 PWM4 PWM7 RC1 RC2 RC3 RC4 RC7")
    latest: Dict[str, Optional[object]] = {
        "HEARTBEAT": None,
        "ATTITUDE": None,
        "RC_CHANNELS": None,
        "SERVO_OUTPUT_RAW": None,
    }
    next_print = time.monotonic()
    started = time.monotonic()
    while time.monotonic() - started < duration_s:
        message = master.recv_match(blocking=True, timeout=0.05)
        if message is not None and message.get_type() in latest:
            latest[message.get_type()] = message
        now = time.monotonic()
        if now >= next_print:
            print_table_row(
                started,
                latest["HEARTBEAT"],
                latest["ATTITUDE"],
                latest["RC_CHANNELS"],
                latest["SERVO_OUTPUT_RAW"],
            )
            next_print = now + 0.5


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="SR-75 B3 localhost-only ArduPlane SITL control helper")
    parser.add_argument("--connect", default="tcp:127.0.0.1:5760")
    parser.add_argument("--allow-non-loopback", action="store_true")
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--status-window", type=float, default=3.0)
    parser.add_argument("--monitor-seconds", type=float, default=5.0)
    parser.add_argument("--disarm", action="store_true", help="Disarm the localhost SITL instance and exit")
    parser.add_argument("--no-arm", action="store_true", help="Collect status and set FBWA without arming")
    return parser


def main() -> int:
    args = build_arg_parser().parse_args()
    try:
        require_loopback(args.connect, args.allow_non_loopback)
    except ValueError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2

    master = mavutil.mavlink_connection(args.connect, autoreconnect=False)
    try:
        heartbeat = wait_heartbeat(master, args.timeout)
        send_gcs_heartbeat(master)
        print(
            "CONNECTED "
            f"system={master.target_system} component={master.target_component} "
            f"armed={int(heartbeat_armed(heartbeat))} mode={mode_name(master, heartbeat.custom_mode)}"
        )
        request_streams(master)
        latest = collect_status(master, args.status_window)
        print_status(master, latest)

        if args.disarm:
            send_gcs_heartbeat(master)
            command_arm(master, False)
            wait_armed(master, False, args.timeout)
            print("DISARMED")
            return 0

        send_gcs_heartbeat(master)
        set_fbwa(master, args.timeout)
        print("MODE_SET FBWA")

        if not args.no_arm:
            send_gcs_heartbeat(master)
            command_arm(master, True)
            ack = wait_command_ack(master, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 3.0)
            if ack is None:
                print("ARM_ACK missing")
            else:
                print(f"ARM_ACK result={result_name(ack.result)}")
            try:
                wait_armed(master, True, args.timeout)
                print("ARMED")
            except TimeoutError as exc:
                print(f"ARM_FAILED {exc}", file=sys.stderr)
                latest = collect_status(master, args.status_window)
                print_status(master, latest)
                return 1

        monitor_table(master, args.monitor_seconds)
        latest = collect_status(master, 0.5)
        if valid_primary_pwm(latest.get("SERVO_OUTPUT_RAW")):
            print("ACTIVE_PWM_OK channels=1,2,3,4")
        else:
            print("ACTIVE_PWM_INVALID channels=1,2,3,4")
            return 1
    finally:
        master.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
