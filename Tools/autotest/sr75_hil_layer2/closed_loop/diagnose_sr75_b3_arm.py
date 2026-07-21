#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""SR-75 Layer 2J-B3 arming/command-transport diagnostic.

Connects only to a loopback MAVLink endpoint (tcp:127.0.0.1:5760 by default),
proves whether MAVLink command transport (COMMAND_LONG / COMMAND_ACK) works at
all, then attempts normal software arming and classifies exactly where the
arming path is blocked: transport, arming checks, state mismatch, or timing.

Does not force-arm, does not change parameters, does not disable arming
checks, and does not open serial devices.
"""

import argparse
import csv
import math
import os
import sys
import time
from typing import Dict, List, Optional, Sequence, Tuple

from pymavlink import mavutil

from sr75_b3_sitl_control import (
    command_target_component,
    endpoint_is_loopback,
    heartbeat_armed,
    mode_name,
    require_loopback,
    result_name,
    rc_values,
    send_gcs_heartbeat,
    servo_values,
    set_fbwa,
)

STREAM_MESSAGES = {
    "HEARTBEAT": mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT,
    "ATTITUDE": mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
    "AHRS2": mavutil.mavlink.MAVLINK_MSG_ID_AHRS2,
    "EKF_STATUS_REPORT": mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT,
    "SYS_STATUS": mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS,
    "RC_CHANNELS": mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS,
    "SERVO_OUTPUT_RAW": mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW,
    "VFR_HUD": mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD,
    "GLOBAL_POSITION_INT": mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
    "GPS_RAW_INT": mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT,
}

CSV_FIELDS = [
    "host_time",
    "heartbeat_armed",
    "mode",
    "system_status",
    "att_roll_deg",
    "att_pitch_deg",
    "att_yaw_deg",
    "ahrs2_roll_deg",
    "ahrs2_pitch_deg",
    "ahrs2_yaw_deg",
    "ekf_flags",
    "rc1",
    "rc2",
    "rc3",
    "rc4",
    "rc7",
    "pwm1",
    "pwm2",
    "pwm3",
    "pwm4",
    "pwm7",
    "vfr_airspeed",
    "vfr_alt",
    "gps_fix",
    "statustext",
    "command_ack_command",
    "command_ack_result",
]

MAV_RESULT_ACCEPTED = 0
MAV_RESULT_TEMPORARILY_REJECTED = 1
MAV_RESULT_DENIED = 2
MAV_RESULT_UNSUPPORTED = 3
MAV_RESULT_FAILED = 4
SET_MESSAGE_INTERVAL_COMMAND = mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL


class HeartbeatTicker:
    """Sends a GCS heartbeat once per second, driven by explicit tick() calls."""

    def __init__(self, master, period_s: float = 1.0):
        self.master = master
        self.period_s = period_s
        self.last_sent = 0.0

    def tick(self) -> None:
        now = time.monotonic()
        if now - self.last_sent >= self.period_s:
            send_gcs_heartbeat(self.master)
            self.last_sent = now


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


def wait_matching_ack(master, ticker: HeartbeatTicker, command: int, timeout_s: float):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        ticker.tick()
        message = master.recv_match(type=("COMMAND_ACK", "STATUSTEXT"), blocking=True, timeout=0.2)
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


def accepted_ack_count(acks: Sequence[object], command: int = SET_MESSAGE_INTERVAL_COMMAND) -> int:
    return sum(
        1 for ack in acks
        if getattr(ack, "command", None) == command and getattr(ack, "result", None) == MAV_RESULT_ACCEPTED
    )


def test_command_transport(master, ticker: HeartbeatTicker, timeout_s: float) -> int:
    """Send SET_MESSAGE_INTERVAL for ATTITUDE and require a matching COMMAND_ACK."""
    request_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 10.0)
    ack = wait_matching_ack(master, ticker, SET_MESSAGE_INTERVAL_COMMAND, timeout_s)
    if ack is None:
        print("COMMAND_TRANSPORT_PENDING accepted_ack_count=0")
        return 0
    print(f"COMMAND_TEST_ACK result={result_name(ack.result)}")
    if ack.result == MAV_RESULT_ACCEPTED:
        print("COMMAND_TRANSPORT_PASS accepted_ack_count=1")
        return 1
    print("COMMAND_TRANSPORT_FAIL accepted_ack_count=0")
    return 0


def request_streams(master, rate_hz: float) -> None:
    for name, message_id in STREAM_MESSAGES.items():
        if name == "HEARTBEAT":
            # HEARTBEAT is streamed by ArduPilot regardless; request anyway so a
            # working transport also confirms it accepts the interval change.
            request_message_interval(master, message_id, rate_hz)
            continue
        request_message_interval(master, message_id, rate_hz)


def print_rc_and_servo(rc_channels, servo) -> None:
    rc1, rc2, rc3, rc4, _, _, rc7, _ = rc_values(rc_channels)
    pwm1, pwm2, pwm3, pwm4, _, _, pwm7, _ = servo_values(servo)
    print(f"RC1={rc1} RC2={rc2} RC3={rc3} RC4={rc4} RC7={rc7}")
    print(f"PWM1={pwm1} PWM2={pwm2} PWM3={pwm3} PWM4={pwm4} PWM7={pwm7}")


def attitude_deg(attitude) -> Tuple[object, object, object]:
    if attitude is None:
        return "", "", ""
    return (
        f"{math.degrees(attitude.roll):.3f}",
        f"{math.degrees(attitude.pitch):.3f}",
        f"{math.degrees(attitude.yaw):.3f}",
    )


def ahrs2_deg(ahrs2) -> Tuple[object, object, object]:
    if ahrs2 is None:
        return "", "", ""
    return (
        f"{math.degrees(ahrs2.roll):.3f}",
        f"{math.degrees(ahrs2.pitch):.3f}",
        f"{math.degrees(ahrs2.yaw):.3f}",
    )


class CsvLogger:
    def __init__(self, path: Optional[str]):
        self.path = path
        self._file = None
        self._writer = None
        if path:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            self._file = open(path, "w", newline="", encoding="utf-8")
            self._writer = csv.DictWriter(self._file, fieldnames=CSV_FIELDS)
            self._writer.writeheader()

    def write(self, row: Dict[str, object]) -> None:
        if self._writer is None:
            return
        self._writer.writerow({field: row.get(field, "") for field in CSV_FIELDS})
        self._file.flush()

    def close(self) -> None:
        if self._file is not None:
            self._file.close()


def collect_evidence(
    master,
    ticker: HeartbeatTicker,
    duration_s: float,
    logger: CsvLogger,
) -> Dict[str, object]:
    latest: Dict[str, object] = {
        "HEARTBEAT": None,
        "ATTITUDE": None,
        "AHRS2": None,
        "EKF_STATUS_REPORT": None,
        "SYS_STATUS": None,
        "RC_CHANNELS": None,
        "SERVO_OUTPUT_RAW": None,
        "VFR_HUD": None,
        "GPS_RAW_INT": None,
    }
    acks: List[object] = []
    statustexts: List[str] = []
    armed_flags: List[bool] = []
    heartbeats: List[Tuple[float, str, int, bool]] = []
    last_mode_status: Optional[Tuple[str, int]] = None
    start_time = time.monotonic()
    deadline = time.monotonic() + duration_s
    while time.monotonic() < deadline:
        ticker.tick()
        message = master.recv_match(blocking=True, timeout=0.2)
        if message is None:
            continue
        message_type = message.get_type()
        if message_type == "BAD_DATA":
            continue
        if message_type in latest:
            latest[message_type] = message
        if message_type == "HEARTBEAT":
            armed = heartbeat_armed(message)
            mode = mode_name(master, message.custom_mode)
            status = int(message.system_status)
            rel_time = time.monotonic() - start_time
            armed_flags.append(armed)
            heartbeats.append((rel_time, mode, status, armed))
            mode_status = (mode, status)
            if mode_status != last_mode_status:
                print(
                    "MODE_TRANSITION "
                    f"t={rel_time:.3f} mode={mode} system_status={status} armed={int(armed)}"
                )
                last_mode_status = mode_status
        if message_type == "STATUSTEXT":
            text = getattr(message, "text", "")
            if text:
                statustexts.append(text)
                print(f"STATUSTEXT {text}")
        if message_type == "COMMAND_ACK":
            acks.append(message)
            print(f"COMMAND_ACK command={message.command} result={result_name(message.result)}")
        logger.write(build_csv_row(master, latest, statustexts, acks))
    return {
        "latest": latest,
        "acks": acks,
        "statustexts": statustexts,
        "armed_flags": armed_flags,
        "heartbeats": heartbeats,
    }


def valid_pwm(value: object) -> bool:
    try:
        pwm = int(value)
    except (TypeError, ValueError):
        return False
    return 800 <= pwm <= 2200


def rc_pwm_valid(value: object) -> bool:
    try:
        pwm = int(value)
    except (TypeError, ValueError):
        return False
    return 800 <= pwm <= 2200


def responder_row_has_valid_mapped_pwm(row: Dict[str, str]) -> bool:
    return all(valid_pwm(row.get(f"pwm{index}")) for index in (1, 2, 3, 4))


def responder_active_summary(path: Optional[str]) -> Dict[str, object]:
    summary: Dict[str, object] = {
        "rows": 0,
        "active_rows": 0,
        "duration_s": 0.0,
        "request_rate_hz": 0.0,
        "reply_rate_hz": 0.0,
        "timestamp_monotonic": False,
        "stale_count": 0,
        "warning_count": 0,
        "rato_max": 0.0,
    }
    if not path or not os.path.exists(path):
        return summary
    previous_timestamp = None
    first_active_time = None
    last_active_time = None
    with open(path, "r", newline="", encoding="utf-8") as csv_file:
        for row in csv.DictReader(csv_file):
            summary["rows"] += 1
            if row.get("reply_sent") != "1" or not responder_row_has_valid_mapped_pwm(row):
                continue
            summary["active_rows"] += 1
            if row.get("host_monotonic_time"):
                row_time = float(row["host_monotonic_time"])
                if first_active_time is None:
                    first_active_time = row_time
                last_active_time = row_time
            if row.get("simulation_timestamp"):
                current_timestamp = float(row["simulation_timestamp"])
                if previous_timestamp is not None and current_timestamp <= previous_timestamp:
                    summary["timestamp_monotonic"] = False
                elif previous_timestamp is None:
                    summary["timestamp_monotonic"] = True
                previous_timestamp = current_timestamp
            if (
                row.get("act_stale") == "1"
                or "STALE_STATE" in row.get("reply_reason", "")
                or "STALE_STATE" in row.get("error_reason", "")
            ):
                summary["stale_count"] += 1
            if row.get("act_warnings", ""):
                summary["warning_count"] += 1
            summary["rato_max"] = max(summary["rato_max"], float(row.get("act_rato_norm", "0") or 0.0))
    if summary["active_rows"] == 0:
        return summary
    if first_active_time is not None and last_active_time is not None and summary["active_rows"] >= 2:
        duration_s = last_active_time - first_active_time
        summary["duration_s"] = duration_s
        if duration_s > 0.0:
            summary["request_rate_hz"] = (summary["active_rows"] - 1) / duration_s
            summary["reply_rate_hz"] = summary["active_rows"] / duration_s
    return summary


def print_no_arm_acceptance(
    master,
    evidence: Dict[str, object],
    responder_log: Optional[str],
    initial_accepted_ack_count: int,
) -> bool:
    heartbeats = evidence["heartbeats"]
    latest = evidence["latest"]
    final_heartbeat = latest.get("HEARTBEAT")
    initial_mode = heartbeats[0][1] if heartbeats else ""
    final_mode = "" if final_heartbeat is None else mode_name(master, final_heartbeat.custom_mode)
    initial_status = heartbeats[0][2] if heartbeats else ""
    final_status = "" if final_heartbeat is None else final_heartbeat.system_status
    accepted = initial_accepted_ack_count + accepted_ack_count(evidence["acks"])
    rc = latest.get("RC_CHANNELS")
    servo = latest.get("SERVO_OUTPUT_RAW")
    rc1, rc2, rc3, rc4, _, _, rc7, _ = rc_values(rc)
    pwm1, pwm2, pwm3, pwm4, _, _, pwm7, _ = servo_values(servo)
    responder = responder_active_summary(responder_log)
    rc_valid = all(rc_pwm_valid(value) for value in (rc1, rc2, rc3, rc4))
    pwm_valid = all(valid_pwm(value) for value in (pwm1, pwm2, pwm3, pwm4))
    reasons = []
    if not heartbeats:
        reasons.append("NO_HEARTBEAT")
    if final_mode == "" or final_mode == "INITIALISING":
        reasons.append(f"FINAL_MODE_{final_mode or 'MISSING'}")
    if not rc_valid:
        reasons.append("RC1_RC4_INVALID")
    if not pwm_valid:
        reasons.append("PWM1_PWM4_INVALID")
    if responder["stale_count"] != 0:
        reasons.append(f"RESPONDER_STALE_{responder['stale_count']}")
    if responder["rato_max"] != 0.0:
        reasons.append(f"RATO_MAX_{responder['rato_max']:.3f}")
    if not responder["timestamp_monotonic"]:
        reasons.append("TIMESTAMP_NOT_STRICTLY_INCREASING")
    if responder["duration_s"] < 10.0:
        reasons.append(f"ACTIVE_WINDOW_SHORT_{responder['duration_s']:.3f}")

    if accepted:
        print(f"COMMAND_TRANSPORT_PASS accepted_ack_count={accepted}")
    else:
        print("COMMAND_TRANSPORT_FAIL accepted_ack_count=0")
    print(
        "HEARTBEAT_SUMMARY "
        f"initial_mode={initial_mode} initial_status={initial_status} "
        f"final_mode={final_mode} final_status={final_status} count={len(heartbeats)}"
    )
    print(f"RC_SUMMARY RC1={rc1} RC2={rc2} RC3={rc3} RC4={rc4} RC7={rc7}")
    print(f"PWM_SUMMARY PWM1={pwm1} PWM2={pwm2} PWM3={pwm3} PWM4={pwm4} PWM7={pwm7}")
    print(
        "RESPONDER_SUMMARY "
        f"rows={responder['active_rows']} request_rate_hz={responder['request_rate_hz']:.3f} "
        f"reply_rate_hz={responder['reply_rate_hz']:.3f} timestamp_monotonic={int(responder['timestamp_monotonic'])} "
        f"stale_count={responder['stale_count']} warning_count={responder['warning_count']} "
        f"rato_max={responder['rato_max']:.3f}"
    )
    if reasons:
        print(f"NO_ARM_ACCEPTANCE_FAIL reasons={','.join(reasons)}")
        return False
    print("NO_ARM_ACCEPTANCE_PASS")
    return True


def build_csv_row(master, latest: Dict[str, object], statustexts: List[str], acks: List[object]) -> Dict[str, object]:
    heartbeat = latest.get("HEARTBEAT")
    attitude = latest.get("ATTITUDE")
    ahrs2 = latest.get("AHRS2")
    ekf = latest.get("EKF_STATUS_REPORT")
    rc = latest.get("RC_CHANNELS")
    servo = latest.get("SERVO_OUTPUT_RAW")
    vfr = latest.get("VFR_HUD")
    gps = latest.get("GPS_RAW_INT")
    rc1, rc2, rc3, rc4, _, _, rc7, _ = rc_values(rc)
    pwm1, pwm2, pwm3, pwm4, _, _, pwm7, _ = servo_values(servo)
    att_roll, att_pitch, att_yaw = attitude_deg(attitude)
    ahrs2_roll, ahrs2_pitch, ahrs2_yaw = ahrs2_deg(ahrs2)
    last_ack = acks[-1] if acks else None
    return {
        "host_time": f"{time.time():.6f}",
        "heartbeat_armed": "" if heartbeat is None else int(heartbeat_armed(heartbeat)),
        "mode": "" if heartbeat is None else mode_name(master, heartbeat.custom_mode),
        "system_status": "" if heartbeat is None else heartbeat.system_status,
        "att_roll_deg": att_roll,
        "att_pitch_deg": att_pitch,
        "att_yaw_deg": att_yaw,
        "ahrs2_roll_deg": ahrs2_roll,
        "ahrs2_pitch_deg": ahrs2_pitch,
        "ahrs2_yaw_deg": ahrs2_yaw,
        "ekf_flags": "" if ekf is None else ekf.flags,
        "rc1": rc1,
        "rc2": rc2,
        "rc3": rc3,
        "rc4": rc4,
        "rc7": rc7,
        "pwm1": pwm1,
        "pwm2": pwm2,
        "pwm3": pwm3,
        "pwm4": pwm4,
        "pwm7": pwm7,
        "vfr_airspeed": "" if vfr is None else vfr.airspeed,
        "vfr_alt": "" if vfr is None else vfr.alt,
        "gps_fix": "" if gps is None else gps.fix_type,
        "statustext": statustexts[-1] if statustexts else "",
        "command_ack_command": "" if last_ack is None else last_ack.command,
        "command_ack_result": "" if last_ack is None else result_name(last_ack.result),
    }


def detect_armed_transition(armed_flags: Sequence[bool]) -> bool:
    """True if the sequence ever goes from not-armed to armed."""
    previous = None
    for value in armed_flags:
        if previous is False and value is True:
            return True
        previous = value
    return False


def classify_arm_result(
    transport_ok: bool,
    arm_ack: Optional[object],
    became_armed: bool,
) -> str:
    """Pure classification, independent of any live MAVLink connection."""
    if arm_ack is None:
        if not transport_ok:
            return "MAVLINK_COMMAND_PATH_FAILED"
        return "ARM_ACK_MISSING"
    result = getattr(arm_ack, "result", None)
    if result == MAV_RESULT_DENIED:
        return "ARM_DENIED"
    if result == MAV_RESULT_TEMPORARILY_REJECTED:
        return "ARM_TEMPORARILY_REJECTED"
    if result in (MAV_RESULT_FAILED, MAV_RESULT_UNSUPPORTED):
        return "ARM_FAILED"
    if result == MAV_RESULT_ACCEPTED:
        if became_armed:
            return "ARM_SUCCESS"
        return "ARM_HEARTBEAT_NOT_ARMED"
    if became_armed:
        return "ARM_SUCCESS"
    return "ARM_FAILED"


def run_diagnostic(args) -> int:
    master = mavutil.mavlink_connection(args.connect, autoreconnect=False)
    logger = CsvLogger(args.log_csv)
    try:
        heartbeat = master.wait_heartbeat(timeout=args.timeout)
        if heartbeat is None:
            print("ARM_HEARTBEAT_NOT_ARMED: no heartbeat received")
            return 1
        ticker = HeartbeatTicker(master)
        ticker.tick()
        print(
            "HEARTBEAT_OK "
            f"type={heartbeat.type} autopilot={heartbeat.autopilot} "
            f"target_system={master.target_system} target_component={master.target_component} "
            f"mode={mode_name(master, heartbeat.custom_mode)} "
            f"armed={int(heartbeat_armed(heartbeat))} "
            f"system_status={heartbeat.system_status}"
        )
        expected_type = mavutil.mavlink.MAV_TYPE_FIXED_WING
        expected_autopilot = mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA
        if heartbeat.type != expected_type or heartbeat.autopilot != expected_autopilot:
            print(
                "HEARTBEAT_UNEXPECTED "
                f"type={heartbeat.type} autopilot={heartbeat.autopilot}"
            )

        transport_accepted_ack_count = test_command_transport(master, ticker, args.timeout)
        transport_ok = transport_accepted_ack_count > 0
        request_streams(master, args.stream_rate_hz)

        if args.no_arm:
            evidence = collect_evidence(
                master,
                ticker,
                args.collect_seconds,
                logger,
            )
            latest = evidence["latest"]
            print_rc_and_servo(latest.get("RC_CHANNELS"), latest.get("SERVO_OUTPUT_RAW"))
            accepted = transport_accepted_ack_count + accepted_ack_count(evidence["acks"])
            transport_ok = transport_ok or accepted > 0
            print(f"NO_ARM requested transport_ok={int(transport_ok)}")
            acceptance_ok = print_no_arm_acceptance(
                master,
                evidence,
                args.responder_log,
                transport_accepted_ack_count,
            )
            return 0 if transport_ok and acceptance_ok else 1

        try:
            set_fbwa(master, args.timeout)
            print("MODE_SET FBWA")
        except (TimeoutError, RuntimeError) as exc:
            print(f"MODE_SET_FAILED {exc}")

        pre_arm = master.recv_match(type=("RC_CHANNELS", "SERVO_OUTPUT_RAW"), blocking=True, timeout=2.0)
        rc_message = pre_arm if pre_arm is not None and pre_arm.get_type() == "RC_CHANNELS" else None
        servo_message = pre_arm if pre_arm is not None and pre_arm.get_type() == "SERVO_OUTPUT_RAW" else None
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and (rc_message is None or servo_message is None):
            ticker.tick()
            message = master.recv_match(type=("RC_CHANNELS", "SERVO_OUTPUT_RAW"), blocking=True, timeout=0.2)
            if message is None:
                continue
            if message.get_type() == "RC_CHANNELS":
                rc_message = message
            else:
                servo_message = message
        print_rc_and_servo(rc_message, servo_message)

        ticker.tick()
        master.mav.command_long_send(
            master.target_system,
            command_target_component(master),
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1.0,
            0.0,
            0,
            0,
            0,
            0,
            0,
        )
        arm_ack = wait_matching_ack(master, ticker, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, args.timeout)
        if arm_ack is None:
            print("ARM_ACK missing")
        else:
            print(f"ARM_ACK result={result_name(arm_ack.result)}")

        evidence = collect_evidence(
            master,
            ticker,
            max(args.collect_seconds, 10.0),
            logger,
        )
        became_armed = detect_armed_transition(evidence["armed_flags"])
        final_heartbeat = evidence["latest"].get("HEARTBEAT")
        if final_heartbeat is not None and heartbeat_armed(final_heartbeat):
            became_armed = True

        classification = classify_arm_result(transport_ok, arm_ack, became_armed)
        print(classification)
        return 0 if classification == "ARM_SUCCESS" else 1
    finally:
        logger.close()
        master.close()


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="SR-75 B3 arming and command-transport diagnostic")
    parser.add_argument("--connect", default="tcp:127.0.0.1:5760")
    parser.add_argument("--allow-non-loopback", action="store_true")
    parser.add_argument("--timeout", type=float, default=10.0)
    parser.add_argument("--stream-rate-hz", type=float, default=10.0)
    parser.add_argument("--collect-seconds", type=float, default=10.0)
    parser.add_argument("--log-csv", default=None, help="Optional CSV path, e.g. /tmp/sr75_b3/sr75_b3_mavlink_diagnostic.csv")
    parser.add_argument("--responder-log", default=None, help="Optional responder CSV for no-arm acceptance checks")
    parser.add_argument("--no-arm", action="store_true", help="Observe only; do not set mode or send an arm command")
    parser.add_argument("--self-test", action="store_true", help="Run offline classification self-tests and exit")
    return parser


def _ack(result: int):
    class _Ack:
        pass

    ack = _Ack()
    ack.result = result
    return ack


def run_self_test() -> int:
    failures = []

    def check(name: str, actual, expected) -> None:
        status = "PASS" if actual == expected else "FAIL"
        print(f"SELF_TEST {name} {status} actual={actual} expected={expected}")
        if actual != expected:
            failures.append(name)

    check(
        "ack_accepted_armed",
        classify_arm_result(True, _ack(MAV_RESULT_ACCEPTED), True),
        "ARM_SUCCESS",
    )
    check(
        "ack_accepted_not_armed",
        classify_arm_result(True, _ack(MAV_RESULT_ACCEPTED), False),
        "ARM_HEARTBEAT_NOT_ARMED",
    )
    check(
        "ack_denied",
        classify_arm_result(True, _ack(MAV_RESULT_DENIED), False),
        "ARM_DENIED",
    )
    check(
        "ack_temporarily_rejected",
        classify_arm_result(True, _ack(MAV_RESULT_TEMPORARILY_REJECTED), False),
        "ARM_TEMPORARILY_REJECTED",
    )
    check(
        "ack_failed",
        classify_arm_result(True, _ack(MAV_RESULT_FAILED), False),
        "ARM_FAILED",
    )
    check(
        "ack_missing_transport_ok",
        classify_arm_result(True, None, False),
        "ARM_ACK_MISSING",
    )
    check(
        "ack_missing_transport_failed",
        classify_arm_result(False, None, False),
        "MAVLINK_COMMAND_PATH_FAILED",
    )
    check(
        "armed_heartbeat_transition_true",
        detect_armed_transition([False, False, True, True]),
        True,
    )
    check(
        "armed_heartbeat_transition_false",
        detect_armed_transition([False, False, False]),
        False,
    )
    check(
        "loopback_endpoint_accepted",
        endpoint_is_loopback("tcp:127.0.0.1:5760"),
        True,
    )
    check(
        "non_loopback_endpoint_rejected",
        endpoint_is_loopback("tcp:192.168.1.20:5760"),
        False,
    )

    if failures:
        print(f"SELF_TEST_RESULT FAIL count={len(failures)} names={','.join(failures)}")
        return 1
    print("SELF_TEST_RESULT PASS")
    return 0


def main() -> int:
    args = build_arg_parser().parse_args()
    if args.self_test:
        return run_self_test()
    try:
        require_loopback(args.connect, args.allow_non_loopback)
    except ValueError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    return run_diagnostic(args)


if __name__ == "__main__":
    sys.exit(main())
