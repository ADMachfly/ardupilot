#!/usr/bin/env python3
"""Local mock client for ArduPilot SIM_JSON UDP responders."""

import argparse
import json
import socket
import struct
import sys
import time
from typing import List, Tuple

from sr75_sim_json_actuator_bridge import neutral_pwm_values


SERVO16_MAGIC = 18458
SERVO16_STRUCT = struct.Struct("<HHI16H")
REQUIRED_TOP_LEVEL = ("timestamp", "imu", "velocity")


def parse_pwm_override(value: str) -> Tuple[int, int]:
    parts = value.split(":", 1)
    if len(parts) != 2:
        raise argparse.ArgumentTypeError("PWM override must use CH:PWM")
    try:
        channel = int(parts[0])
        pwm = int(parts[1])
    except ValueError as exc:
        raise argparse.ArgumentTypeError("PWM override CH and PWM must be integers") from exc
    if channel < 1 or channel > 16:
        raise argparse.ArgumentTypeError("PWM override channel must be 1..16")
    return channel, pwm


def build_packet(frame_rate: int, frame_count: int, pwm: List[int]) -> bytes:
    return SERVO16_STRUCT.pack(SERVO16_MAGIC, frame_rate, frame_count, *pwm)


def validate_reply(data: bytes) -> dict:
    if not data.endswith(b"\n"):
        raise ValueError("reply is not newline terminated")
    payload = json.loads(data.decode("utf-8"))
    for key in REQUIRED_TOP_LEVEL:
        if key not in payload:
            raise ValueError(f"missing JSON field {key}")
    if "gyro" not in payload["imu"] or "accel_body" not in payload["imu"]:
        raise ValueError("missing imu.gyro or imu.accel_body")
    if "attitude" not in payload and "quaternion" not in payload:
        raise ValueError("missing attitude/quaternion")
    return payload


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Mock ArduPilot SIM_JSON control packet client")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=9002)
    parser.add_argument("--rate-hz", type=float, default=20.0)
    parser.add_argument("--count", type=int, default=1, help="0 means run forever")
    parser.add_argument("--timeout", type=float, default=1.0)
    parser.add_argument("--pwm", action="append", type=parse_pwm_override, default=[], help="Override PWM as CH:PWM")
    parser.add_argument("--verbose", action="store_true")
    return parser


def main() -> int:
    args = build_arg_parser().parse_args()
    period = 1.0 / args.rate_hz if args.rate_hz > 0 else 0.0
    frame_rate = int(round(args.rate_hz)) if args.rate_hz > 0 else 0
    count = 0
    pwm = neutral_pwm_values()
    for channel, value in args.pwm:
        pwm[channel - 1] = value

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(args.timeout)

    try:
        while args.count == 0 or count < args.count:
            count += 1
            packet = build_packet(frame_rate, count, pwm)
            start = time.monotonic()
            sock.sendto(packet, (args.host, args.port))
            try:
                data, source = sock.recvfrom(65535)
            except socket.timeout:
                print(f"TIMEOUT waiting for reply from {args.host}:{args.port}")
                return 1
            rtt_ms = (time.monotonic() - start) * 1000.0
            try:
                payload = validate_reply(data)
            except (ValueError, json.JSONDecodeError) as exc:
                print(f"INVALID_REPLY: {exc}")
                return 1
            print(f"reply {count} from {source[0]}:{source[1]} bytes={len(data)} rtt_ms={rtt_ms:.3f}")
            if args.verbose:
                print(
                    f"  timestamp={payload['timestamp']} velocity={payload['velocity']} "
                    f"gyro={payload['imu']['gyro']} accel_body={payload['imu']['accel_body']}"
                )
            if period > 0.0 and (args.count == 0 or count < args.count):
                elapsed = time.monotonic() - start
                if elapsed < period:
                    time.sleep(period - elapsed)
    finally:
        sock.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
