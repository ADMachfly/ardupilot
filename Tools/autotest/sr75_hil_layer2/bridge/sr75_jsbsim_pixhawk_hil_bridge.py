#!/usr/bin/env python3
'''
SR-75 Layer 2 external JSBSim/Pixhawk HIL bridge skeleton.

This first version is intentionally bench-safe: it connects to a real
Pixhawk, requests monitoring streams, forwards telemetry optionally, and
logs Pixhawk outputs. It does not command JSBSim or any actuator hardware.
'''

import argparse
import csv
import math
import os
import signal
import socket
import subprocess
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

JSBSIM_FIELDS = [
    ("jsb_time_s", "/fdm/jsbsim/simulation/sim-time-sec", 1.0),
    ("jsb_lat_deg", "/fdm/jsbsim/position/lat-gc-deg", 1.0),
    ("jsb_lon_deg", "/fdm/jsbsim/position/long-gc-deg", 1.0),
    ("jsb_alt_m", "/fdm/jsbsim/position/h-sl-ft", 0.3048),
    ("jsb_agl_m", "/fdm/jsbsim/position/h-agl-ft", 0.3048),
    ("jsb_roll_rad", "/fdm/jsbsim/attitude/phi-deg", math.pi / 180.0),
    ("jsb_pitch_rad", "/fdm/jsbsim/attitude/theta-rad", 1.0),
    ("jsb_yaw_rad", "/fdm/jsbsim/attitude/psi-deg", math.pi / 180.0),
    ("jsb_airspeed_mps", "/fdm/jsbsim/velocities/vt-fps", 0.3048),
    ("jsb_alpha_rad", "/fdm/jsbsim/aero/alpha-rad", 1.0),
    ("jsb_vn_mps", "/fdm/jsbsim/velocities/v-north-fps", 0.3048),
    ("jsb_ve_mps", "/fdm/jsbsim/velocities/v-east-fps", 0.3048),
    ("jsb_vd_mps", "/fdm/jsbsim/velocities/v-down-fps", 0.3048),
    ("jsb_p_rad_s", "/fdm/jsbsim/velocities/p-rad_sec", 1.0),
    ("jsb_q_rad_s", "/fdm/jsbsim/velocities/q-rad_sec", 1.0),
    ("jsb_r_rad_s", "/fdm/jsbsim/velocities/r-rad_sec", 1.0),
    ("jsb_udot_mps2", "/fdm/jsbsim/accelerations/udot-ft_sec2", 0.3048),
    ("jsb_vdot_mps2", "/fdm/jsbsim/accelerations/vdot-ft_sec2", 0.3048),
    ("jsb_wdot_mps2", "/fdm/jsbsim/accelerations/wdot-ft_sec2", 0.3048),
]


def blank_jsbsim_row():
    return {field_name: "" for field_name, _column_name, _scale in JSBSIM_FIELDS}


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


def default_jsbsim_root():
    here = os.path.abspath(os.path.dirname(__file__))
    return os.path.abspath(os.path.join(here, os.pardir, os.pardir))


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


class JSBSimCSVMonitor:
    def __init__(self, path):
        self.path = path
        self.headers = None

    def blank_row(self):
        return blank_jsbsim_row()

    def mtime(self):
        try:
            return os.path.getmtime(self.path)
        except OSError:
            return None

    def _read_header(self):
        try:
            with open(self.path, "r", newline="", encoding="utf-8") as csv_file:
                self.headers = next(csv.reader(csv_file), None)
        except (OSError, StopIteration):
            self.headers = None

    def _read_last_line(self):
        try:
            with open(self.path, "rb") as csv_file:
                csv_file.seek(0, os.SEEK_END)
                end_pos = csv_file.tell()
                if end_pos == 0:
                    return None
                block_size = min(8192, end_pos)
                csv_file.seek(end_pos - block_size)
                data = csv_file.read(block_size).decode("utf-8", errors="ignore")
        except OSError:
            return None

        lines = [line for line in data.splitlines() if line.strip()]
        if not lines or lines[-1].startswith("Time,"):
            return None
        return lines[-1]

    def read_latest(self):
        row = self.blank_row()
        if self.headers is None:
            self._read_header()
        if not self.headers:
            return row

        last_line = self._read_last_line()
        if last_line is None:
            return row

        parsed_rows = list(csv.reader([last_line]))
        if not parsed_rows:
            return row
        values = parsed_rows[0]

        for field_name, column_name, scale in JSBSIM_FIELDS:
            try:
                index = self.headers.index(column_name)
                value = values[index]
                if value != "":
                    row[field_name] = float(value) * scale
            except (ValueError, IndexError):
                row[field_name] = ""
        return row


def remove_old_jsbsim_outputs(root):
    for filename in ("sr_75_6_dof_debug.csv", "sr_75_6_dof_cruise_test.csv"):
        path = os.path.join(root, filename)
        try:
            os.remove(path)
            print(f"Removed old JSBSim CSV output: {path}")
        except FileNotFoundError:
            pass
        except OSError as ex:
            print(f"Warning: could not remove old JSBSim CSV output {path}: {ex}")


def start_jsbsim_subprocess(args):
    if not args.start_jsbsim:
        return None
    if args.jsbsim_script is None:
        raise RuntimeError("--start-jsbsim requires --jsbsim-script")

    root = os.path.abspath(args.jsbsim_root)
    remove_old_jsbsim_outputs(root)
    cmd = [
        "JSBSim",
        f"--root={root}",
        f"--script={args.jsbsim_script}",
        "--realtime",
        f"--end={args.jsbsim_end}",
    ]
    print("Starting read-only JSBSim subprocess: " + " ".join(cmd))
    return subprocess.Popen(cmd)


def safe_float(row, field_name, default=None):
    value = row.get(field_name, "")
    if value == "":
        return default
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def euler_to_quaternion(roll, pitch, yaw):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return [
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ]


def cm_per_s(value):
    return int(value * 100.0)


def milli_g(value):
    return int(value / 9.80665 * 1000.0)


class HILInjector:
    def __init__(self, master, rate_hz, gps_rate_hz, dry_run):
        self.master = master
        self.rate_hz = rate_hz
        self.gps_rate_hz = gps_rate_hz
        self.dry_run = dry_run
        self.hil_state_sent_count = 0
        self.hil_gps_sent_count = 0
        self.hil_state_dryrun_count = 0
        self.hil_gps_dryrun_count = 0
        self.hil_gps_enabled = hasattr(master.mav, "hil_gps_send")
        self.hil_state_quaternion_enabled = hasattr(master.mav, "hil_state_quaternion_send")
        self.hil_state_enabled = not self.hil_state_quaternion_enabled and hasattr(master.mav, "hil_state_send")
        enabled = []
        if self.hil_gps_enabled:
            enabled.append("HIL_GPS")
        if self.hil_state_quaternion_enabled:
            enabled.append("HIL_STATE_QUATERNION")
        elif self.hil_state_enabled:
            enabled.append("HIL_STATE")
        print(f"HIL injection messages enabled: {', '.join(enabled) if enabled else 'none'}")

    def _common(self, row):
        lat = safe_float(row, "jsb_lat_deg")
        lon = safe_float(row, "jsb_lon_deg")
        alt = safe_float(row, "jsb_alt_m")
        if lat is None or lon is None or alt is None:
            return None
        sim_time = safe_float(row, "jsb_time_s", time.time())
        vn = safe_float(row, "jsb_vn_mps", 0.0)
        ve = safe_float(row, "jsb_ve_mps", 0.0)
        vd = safe_float(row, "jsb_vd_mps", 0.0)
        airspeed = safe_float(row, "jsb_airspeed_mps", 0.0)
        return {
            "time_usec": int(sim_time * 1.0e6),
            "lat": int(lat * 1.0e7),
            "lon": int(lon * 1.0e7),
            "alt": int(alt * 1000.0),
            "vn": vn,
            "ve": ve,
            "vd": vd,
            "airspeed": airspeed,
        }

    def send_gps(self, row):
        if not self.hil_gps_enabled:
            return False
        data = self._common(row)
        if data is None:
            return False
        horizontal_speed = math.hypot(data["vn"], data["ve"])
        cog = int((math.degrees(math.atan2(data["ve"], data["vn"])) % 360.0) * 100.0) if horizontal_speed > 0.1 else 0
        if not self.dry_run:
            self.master.mav.hil_gps_send(
                data["time_usec"],
                3,
                data["lat"],
                data["lon"],
                data["alt"],
                100,
                100,
                cm_per_s(horizontal_speed),
                cm_per_s(data["vn"]),
                cm_per_s(data["ve"]),
                cm_per_s(data["vd"]),
                cog,
                10,
            )
            self.hil_gps_sent_count += 1
        else:
            self.hil_gps_dryrun_count += 1
        return True

    def send_state(self, row):
        data = self._common(row)
        if data is None:
            return False
        roll = safe_float(row, "jsb_roll_rad")
        pitch = safe_float(row, "jsb_pitch_rad")
        yaw = safe_float(row, "jsb_yaw_rad")
        if roll is None or pitch is None or yaw is None:
            return False
        p = safe_float(row, "jsb_p_rad_s", 0.0)
        q = safe_float(row, "jsb_q_rad_s", 0.0)
        r = safe_float(row, "jsb_r_rad_s", 0.0)
        xacc = milli_g(safe_float(row, "jsb_udot_mps2", 0.0))
        yacc = milli_g(safe_float(row, "jsb_vdot_mps2", 0.0))
        zacc = milli_g(safe_float(row, "jsb_wdot_mps2", 0.0))
        if self.hil_state_quaternion_enabled:
            if not self.dry_run:
                self.master.mav.hil_state_quaternion_send(
                    data["time_usec"],
                    euler_to_quaternion(roll, pitch, yaw),
                    p,
                    q,
                    r,
                    data["lat"],
                    data["lon"],
                    data["alt"],
                    cm_per_s(data["vn"]),
                    cm_per_s(data["ve"]),
                    cm_per_s(data["vd"]),
                    cm_per_s(data["airspeed"]),
                    cm_per_s(data["airspeed"]),
                    xacc,
                    yacc,
                    zacc,
                )
                self.hil_state_sent_count += 1
            else:
                self.hil_state_dryrun_count += 1
            return True
        if self.hil_state_enabled:
            if not self.dry_run:
                self.master.mav.hil_state_send(
                    data["time_usec"],
                    roll,
                    pitch,
                    yaw,
                    p,
                    q,
                    r,
                    data["lat"],
                    data["lon"],
                    data["alt"],
                    cm_per_s(data["vn"]),
                    cm_per_s(data["ve"]),
                    cm_per_s(data["vd"]),
                    xacc,
                    yacc,
                    zacc,
                )
                self.hil_state_sent_count += 1
            else:
                self.hil_state_dryrun_count += 1
            return True
        return False

    def print_status(self, last_jsb_t):
        print(
            f"HIL: state_sent={self.hil_state_sent_count} gps_sent={self.hil_gps_sent_count} "
            f"dryrun_state={self.hil_state_dryrun_count} dryrun_gps={self.hil_gps_dryrun_count} "
            f"last_jsb_t={last_jsb_t if last_jsb_t is not None else ''}"
        )


def parse_args():
    parser = argparse.ArgumentParser(
        description="SR-75 Layer 2 Pixhawk MAVLink HIL bridge skeleton"
    )
    parser.add_argument("--pixhawk", required=True, help="Pixhawk serial device, e.g. /dev/ttyS4 or /dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200, help="Pixhawk serial baud rate")
    parser.add_argument("--mp-out", default=None, help="Optional Mission Planner MAVLink output, e.g. udp:192.168.1.20:14550")
    parser.add_argument("--mp-in", default=None, help="Optional Mission Planner MAVLink input, e.g. udp:0.0.0.0:14551")
    parser.add_argument("--jsbsim-csv", default=None, help="Optional read-only JSBSim CSV monitor path")
    parser.add_argument("--jsbsim-read-only", action="store_true", help="Monitor JSBSim CSV without commanding JSBSim")
    parser.add_argument("--jsbsim-script", default=None, help="Optional JSBSim script path, relative to --jsbsim-root")
    parser.add_argument("--jsbsim-root", default=default_jsbsim_root(), help="JSBSim root directory")
    parser.add_argument("--jsbsim-end", type=float, default=10.0, help="JSBSim subprocess end time in seconds")
    parser.add_argument("--start-jsbsim", action="store_true", help="Start JSBSim as a read-only subprocess")
    parser.add_argument("--hil-inject", action="store_true", help="Send JSBSim state to Pixhawk using MAVLink HIL messages")
    parser.add_argument("--hil-rate-hz", type=float, default=20.0, help="HIL state injection rate")
    parser.add_argument("--hil-gps-rate-hz", type=float, default=5.0, help="HIL GPS injection rate")
    parser.add_argument("--hil-dry-run", action="store_true", help="Compute HIL messages but do not send them")
    parser.add_argument("--hil-max-stale-s", type=float, default=1.0, help="Maximum JSBSim CSV staleness before HIL pauses")
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


def make_row(start_time, latest, jsbsim_row=None):
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
    row.update(jsbsim_row if jsbsim_row is not None else blank_jsbsim_row())
    return row


def print_status(row):
    servos = " ".join(str(row[f"servo{i}_pwm"]) for i in range(1, 9))
    rcs = " ".join(str(row[f"rc{i}_pwm"]) for i in range(1, 9))
    print(
        f"t={row['time_s']} mode={row['mode']} armed={row['armed']} "
        f"servo=[{servos}] rc=[{rcs}] "
        f"rpy=({row['roll_rad']},{row['pitch_rad']},{row['yaw_rad']})"
    )


def print_jsbsim_status(row):
    print(
        f"JSBSim: t={row['jsb_time_s']} alt_m={row['jsb_alt_m']} agl_m={row['jsb_agl_m']} "
        f"airspeed_mps={row['jsb_airspeed_mps']} "
        f"rpy=({row['jsb_roll_rad']},{row['jsb_pitch_rad']},{row['jsb_yaw_rad']}) "
        f"alpha={row['jsb_alpha_rad']}"
    )


def main():
    args = parse_args()
    print(SAFETY_WARNING.strip())
    if args.no_actuator_output:
        print("Actuator/JSBSim output: DISABLED")
    else:
        print("Actuator output flag was enabled, but this skeleton still sends no actuator commands.")
    jsbsim_monitor = JSBSimCSVMonitor(args.jsbsim_csv) if args.jsbsim_csv is not None else None
    if jsbsim_monitor is not None:
        print(f"JSBSim CSV monitoring: {args.jsbsim_csv} (read-only)")
    jsbsim_proc = start_jsbsim_subprocess(args)
    jsbsim_exit_reported = False

    master = mavutil.mavlink_connection(args.pixhawk, baud=args.baud, autoreconnect=True)
    if args.hil_inject:
        print("HIL injection: ENABLED")
        print(f"HIL injection rates: state {args.hil_rate_hz:g} Hz, GPS {args.hil_gps_rate_hz:g} Hz")
        if args.hil_dry_run:
            print("HIL dry-run: computing messages but not sending")
        hil_injector = HILInjector(master, args.hil_rate_hz, args.hil_gps_rate_hz, args.hil_dry_run)
        print(f"HIL_STATE_QUATERNION available: {'yes' if hil_injector.hil_state_quaternion_enabled else 'no'}")
        print(f"HIL_GPS available: {'yes' if hil_injector.hil_gps_enabled else 'no'}")
    else:
        print("HIL injection: DISABLED")
        hil_injector = None
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
    next_hil_state = 0.0
    next_hil_gps = 0.0
    next_hil_status = 0.0
    last_hil_jsb_t = None
    last_hil_jsb_mtime = None
    last_hil_advance_wall = time.time()
    hil_stale_reported = False
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

            if jsbsim_proc is not None and not jsbsim_exit_reported:
                returncode = jsbsim_proc.poll()
                if returncode is not None:
                    print(f"JSBSim subprocess exited with code {returncode}")
                    jsbsim_exit_reported = True

            if hil_injector is not None:
                if now >= next_hil_status:
                    hil_injector.print_status(last_hil_jsb_t)
                    next_hil_status = now + 1.0

                hil_state_due = now >= next_hil_state
                hil_gps_due = now >= next_hil_gps
                if jsbsim_monitor is not None and (hil_state_due or hil_gps_due):
                    hil_row = jsbsim_monitor.read_latest()
                    hil_jsb_t = safe_float(hil_row, "jsb_time_s")
                    hil_jsb_mtime = jsbsim_monitor.mtime()
                    data_advanced = (
                        hil_jsb_t is not None and hil_jsb_t != last_hil_jsb_t
                    ) or (
                        hil_jsb_mtime is not None and hil_jsb_mtime != last_hil_jsb_mtime
                    )
                    if data_advanced:
                        last_hil_advance_wall = now
                        hil_stale_reported = False
                    last_hil_jsb_t = hil_jsb_t if hil_jsb_t is not None else last_hil_jsb_t
                    last_hil_jsb_mtime = hil_jsb_mtime if hil_jsb_mtime is not None else last_hil_jsb_mtime

                    hil_stale = now - last_hil_advance_wall > args.hil_max_stale_s
                    if hil_stale:
                        if not hil_stale_reported:
                            print("HIL paused: stale JSBSim data")
                            hil_stale_reported = True
                    else:
                        if hil_state_due:
                            hil_injector.send_state(hil_row)
                        if hil_gps_due:
                            hil_injector.send_gps(hil_row)

                    if hil_state_due:
                        next_hil_state = now + (1.0 / max(args.hil_rate_hz, 0.1))
                    if hil_gps_due:
                        next_hil_gps = now + (1.0 / max(args.hil_gps_rate_hz, 0.1))

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
                jsbsim_row = jsbsim_monitor.read_latest() if jsbsim_monitor is not None else None
                row = make_row(start_time, latest, jsbsim_row)
                writer.writerow(row)
                csv_file.flush()
                print_status(row)
                if jsbsim_monitor is not None:
                    print_jsbsim_status(row)
                next_print = now + 1.0

            time.sleep(0.002)

    print("SR-75 Layer 2 bridge skeleton stopped.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
