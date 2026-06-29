#!/usr/bin/env python3
'''
SR-75 Layer 2 external JSBSim/Pixhawk HIL bridge skeleton.

This first version is intentionally bench-safe: it connects to a real
Pixhawk, requests monitoring streams, forwards telemetry optionally, and
logs Pixhawk outputs. It does not command JSBSim or any actuator hardware.
'''

import argparse
import csv
import os
import signal
import socket
import sys
import time
from datetime import datetime, timezone

from pymavlink import mavutil


MESSAGE_RATES_HZ = {
    "SERVO_OUTPUT_RAW": 5,
    "ATTITUDE": 10,
    "GLOBAL_POSITION_INT": 5,
    "VFR_HUD": 5,
    "RC_CHANNELS": 5,
}

SAFETY_WARNING = """
SR-75 LAYER 2 HIL BENCH SAFETY
No live engine, no fuel pump, no live RATO ignition, no live ejection.
Use dummy loads, LEDs, PWM loggers, or disconnected outputs only.
This skeleton does not command JSBSim or actuator hardware.
"""


def default_log_path():
    here = os.path.abspath(os.path.dirname(__file__))
    layer2_dir = os.path.abspath(os.path.join(here, os.pardir))
    logs_dir = os.path.join(layer2_dir, "logs")
    os.makedirs(logs_dir, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
    return os.path.join(logs_dir, f"sr75_layer2_bridge_{stamp}.csv")


def normalize_mp_out(mp_out):
    if mp_out is None:
        return None
    if mp_out.startswith("udp:"):
        return "udpout:" + mp_out[len("udp:"):]
    return mp_out


def parse_udp_endpoint(endpoint):
    if not endpoint.startswith("udp:"):
        raise ValueError(f"Expected udp:HOST:PORT endpoint, got {endpoint}")
    host_port = endpoint[len("udp:"):]
    host, port = host_port.rsplit(":", 1)
    return host, int(port)


def open_mp_in_socket(mp_in):
    if mp_in is None:
        return None
    host, port = parse_udp_endpoint(mp_in)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setblocking(False)
    sock.bind((host, port))
    print(f"Listening for Mission Planner inbound MAVLink on udp:{host}:{port}")
    return sock


def parse_args():
    parser = argparse.ArgumentParser(
        description="SR-75 Layer 2 Pixhawk MAVLink HIL bridge skeleton"
    )
    parser.add_argument("--pixhawk", required=True, help="Pixhawk serial device, e.g. /dev/ttyS4 or /dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200, help="Pixhawk serial baud rate")
    parser.add_argument("--mp-out", default=None, help="Optional Mission Planner MAVLink output, e.g. udp:192.168.1.20:14550")
    parser.add_argument("--mp-in", default=None, help="Optional Mission Planner MAVLink input, e.g. udp:0.0.0.0:14551")
    parser.add_argument("--log", default=default_log_path(), help="CSV log path")
    parser.add_argument(
        "--no-actuator-output",
        dest="no_actuator_output",
        action="store_true",
        default=True,
        help="Keep actuator/JSBSim output disabled. This is the default.",
    )
    parser.add_argument(
        "--allow-actuator-output",
        dest="no_actuator_output",
        action="store_false",
        help="Reserved for later versions. This skeleton still sends no actuator commands.",
    )
    return parser.parse_args()


def wait_for_pixhawk(master):
    print("Waiting for Pixhawk heartbeat...")
    heartbeat = master.wait_heartbeat(timeout=30)
    if heartbeat is None:
        raise RuntimeError("Timed out waiting for Pixhawk heartbeat")
    print(
        f"Pixhawk heartbeat: system={master.target_system} component={master.target_component} "
        f"type={heartbeat.type} autopilot={heartbeat.autopilot}"
    )
    return heartbeat


def send_bridge_heartbeat(master):
    master.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0,
        0,
        mavutil.mavlink.MAV_STATE_ACTIVE,
    )


def request_message_interval(master, message_name, rate_hz):
    message_id = getattr(mavutil.mavlink, f"MAVLINK_MSG_ID_{message_name}")
    interval_us = int(1_000_000 / rate_hz) if rate_hz > 0 else -1
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
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


def request_monitoring_streams(master):
    for message_name, rate_hz in MESSAGE_RATES_HZ.items():
        request_message_interval(master, message_name, rate_hz)
        print(f"Requested {message_name} at {rate_hz} Hz")


def mode_and_armed(heartbeat):
    if heartbeat is None:
        return "", ""
    mode = mavutil.mode_string_v10(heartbeat)
    armed = bool(heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    return mode, int(armed)


def msg_fields(msg, prefix, count):
    values = []
    for i in range(1, count + 1):
        values.append(getattr(msg, f"{prefix}{i}_raw", ""))
    return values


def make_row(start_time, latest):
    now = time.time()
    heartbeat = latest.get("HEARTBEAT")
    mode, armed = mode_and_armed(heartbeat)

    servo = latest.get("SERVO_OUTPUT_RAW")
    rc = latest.get("RC_CHANNELS")
    attitude = latest.get("ATTITUDE")
    global_position = latest.get("GLOBAL_POSITION_INT")
    vfr_hud = latest.get("VFR_HUD")

    row = {
        "time_iso": datetime.now(timezone.utc).isoformat(),
        "time_s": f"{now - start_time:.3f}",
        "mode": mode,
        "armed": armed,
    }

    servo_values = msg_fields(servo, "servo", 8) if servo is not None else [""] * 8
    rc_values = msg_fields(rc, "chan", 8) if rc is not None else [""] * 8
    for i, value in enumerate(servo_values, start=1):
        row[f"servo{i}_pwm"] = value
    for i, value in enumerate(rc_values, start=1):
        row[f"rc{i}_pwm"] = value

    row["roll_rad"] = getattr(attitude, "roll", "")
    row["pitch_rad"] = getattr(attitude, "pitch", "")
    row["yaw_rad"] = getattr(attitude, "yaw", "")
    row["lat_deg"] = getattr(global_position, "lat", "") / 1.0e7 if global_position is not None else ""
    row["lon_deg"] = getattr(global_position, "lon", "") / 1.0e7 if global_position is not None else ""
    row["alt_m"] = getattr(global_position, "alt", "") / 1000.0 if global_position is not None else ""
    row["relative_alt_m"] = getattr(global_position, "relative_alt", "") / 1000.0 if global_position is not None else ""
    row["airspeed_mps"] = getattr(vfr_hud, "airspeed", "")
    row["groundspeed_mps"] = getattr(vfr_hud, "groundspeed", "")
    row["throttle_pct"] = getattr(vfr_hud, "throttle", "")
    return row


def print_status(row):
    servos = " ".join(str(row[f"servo{i}_pwm"]) for i in range(1, 9))
    rcs = " ".join(str(row[f"rc{i}_pwm"]) for i in range(1, 9))
    print(
        f"t={row['time_s']} mode={row['mode']} armed={row['armed']} "
        f"servo=[{servos}] rc=[{rcs}] "
        f"rpy=({row['roll_rad']},{row['pitch_rad']},{row['yaw_rad']})"
    )


def main():
    args = parse_args()
    print(SAFETY_WARNING.strip())
    if args.no_actuator_output:
        print("Actuator/JSBSim output: DISABLED")
    else:
        print("Actuator output flag was enabled, but this skeleton still sends no actuator commands.")

    master = mavutil.mavlink_connection(args.pixhawk, baud=args.baud, autoreconnect=True)
    mp_in_sock = open_mp_in_socket(args.mp_in)
    mp_out_addr = parse_udp_endpoint(args.mp_out) if mp_in_sock is not None and args.mp_out is not None else None
    mp_out = normalize_mp_out(args.mp_out)
    mp_link = mavutil.mavlink_connection(mp_out, input=False) if mp_in_sock is None and mp_out is not None else None
    mp_in_seen = False
    if mp_out_addr is not None:
        print(f"Forwarding Pixhawk MAVLink traffic to Mission Planner at {args.mp_out} using the inbound UDP socket")
    if mp_link is not None:
        print(f"Forwarding Pixhawk MAVLink traffic to Mission Planner at {mp_out}")

    wait_for_pixhawk(master)
    request_monitoring_streams(master)

    latest = {}
    start_time = time.time()
    next_heartbeat = 0.0
    next_request = 0.0
    next_print = 0.0
    running = True

    def stop(_signum, _frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    fieldnames = list(make_row(start_time, latest).keys())
    with open(args.log, "w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        print(f"CSV log: {args.log}")

        while running:
            now = time.time()
            if now >= next_heartbeat:
                send_bridge_heartbeat(master)
                next_heartbeat = now + 1.0
            if now >= next_request:
                request_monitoring_streams(master)
                next_request = now + 10.0

            msg = master.recv_match(blocking=False)
            if msg is not None:
                msg_type = msg.get_type()
                if msg_type != "BAD_DATA":
                    latest[msg_type] = msg
                    if mp_in_sock is not None and mp_out_addr is not None:
                        mp_in_sock.sendto(msg.get_msgbuf(), mp_out_addr)
                    elif mp_link is not None:
                        mp_link.write(msg.get_msgbuf())

            if mp_in_sock is not None:
                while True:
                    try:
                        data, addr = mp_in_sock.recvfrom(4096)
                    except BlockingIOError:
                        break
                    if data:
                        if not mp_in_seen:
                            print(f"Received first Mission Planner inbound packet from {addr[0]}:{addr[1]}")
                            mp_in_seen = True
                        master.write(data)

            if now >= next_print:
                row = make_row(start_time, latest)
                writer.writerow(row)
                csv_file.flush()
                print_status(row)
                next_print = now + 1.0

            time.sleep(0.002)

    print("SR-75 Layer 2 bridge skeleton stopped.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
