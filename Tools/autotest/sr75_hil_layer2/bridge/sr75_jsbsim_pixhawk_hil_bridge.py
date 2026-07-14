#!/usr/bin/env python3
'''
SR-75 Layer 2 external JSBSim/Pixhawk HIL bridge skeleton.

This first version is intentionally bench-safe: it connects to a real
Pixhawk, requests monitoring streams, forwards telemetry optionally, and
logs Pixhawk outputs. It does not command JSBSim or any actuator hardware.
'''

import argparse
import csv
import inspect
import math
import os
import signal
import socket
import subprocess
import sys
import time
from datetime import datetime, timezone

from pymavlink import mavutil


GPS_EPOCH_UNIX_S = 315964800

MESSAGE_RATES_HZ = {
    "SERVO_OUTPUT_RAW": 5,
    "ATTITUDE": 10,
    "AHRS2": 5,
    "EKF_STATUS_REPORT": 2,
    "LOCAL_POSITION_NED": 5,
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

JSB_FEED_STALE_TIMEOUT_S = 2.0

JSB_FEED_ALIASES = {
    "jsb_feed_time_s": (
        "time_s", "jsb_time_s", "jsb_feed_time_s", "sim_time_s",
        "simulation/sim-time-sec", "/fdm/jsbsim/simulation/sim-time-sec",
    ),
    "jsb_feed_lat_deg": (
        "lat_deg", "jsb_lat_deg", "jsb_feed_lat_deg",
        "position/lat-gc-deg", "/fdm/jsbsim/position/lat-gc-deg",
    ),
    "jsb_feed_lon_deg": (
        "lon_deg", "jsb_lon_deg", "long_deg", "jsb_long_deg", "jsb_feed_lon_deg",
        "position/long-gc-deg", "/fdm/jsbsim/position/long-gc-deg",
    ),
    "jsb_feed_alt_m": (
        "alt_m", "jsb_alt_m", "altitude_m", "jsb_feed_alt_m",
        "position/h-sl-ft", "/fdm/jsbsim/position/h-sl-ft",
    ),
    "jsb_feed_vn_mps": (
        "vn_mps", "jsb_vn_mps", "v_north_mps", "jsb_feed_vn_mps",
        "velocities/v-north-fps", "/fdm/jsbsim/velocities/v-north-fps",
    ),
    "jsb_feed_ve_mps": (
        "ve_mps", "jsb_ve_mps", "v_east_mps", "jsb_feed_ve_mps",
        "velocities/v-east-fps", "/fdm/jsbsim/velocities/v-east-fps",
    ),
    "jsb_feed_vd_mps": (
        "vd_mps", "jsb_vd_mps", "v_down_mps", "jsb_feed_vd_mps",
        "velocities/v-down-fps", "/fdm/jsbsim/velocities/v-down-fps",
    ),
    "jsb_feed_airspeed_mps": (
        "airspeed_mps", "jsb_airspeed_mps", "vt_mps", "jsb_feed_airspeed_mps",
        "velocities/vc-kts", "/fdm/jsbsim/velocities/vc-kts",
        "velocities/vt-fps", "/fdm/jsbsim/velocities/vt-fps",
    ),
    "jsb_feed_roll_rad": (
        "roll_rad", "phi_rad", "jsb_roll_rad", "jsb_feed_roll_rad", "phi_deg",
        "attitude/phi-deg", "/fdm/jsbsim/attitude/phi-deg",
    ),
    "jsb_feed_pitch_rad": (
        "pitch_rad", "theta_rad", "jsb_pitch_rad", "jsb_feed_pitch_rad", "theta_deg",
        "attitude/theta-rad", "attitude/theta-deg", "/fdm/jsbsim/attitude/theta-rad",
    ),
    "jsb_feed_yaw_rad": (
        "yaw_rad", "psi_rad", "jsb_yaw_rad", "jsb_feed_yaw_rad", "psi_deg",
        "attitude/psi-deg", "/fdm/jsbsim/attitude/psi-deg",
    ),
}

JSB_FEED_ALIAS_SCALES = {column_name: scale for _field_name, column_name, scale in JSBSIM_FIELDS}
JSB_FEED_ALIAS_SCALES.update({
    "position/h-sl-ft": 0.3048,
    "velocities/v-north-fps": 0.3048,
    "velocities/v-east-fps": 0.3048,
    "velocities/v-down-fps": 0.3048,
    "velocities/vc-kts": 0.514444,
    "/fdm/jsbsim/velocities/vc-kts": 0.514444,
    "velocities/vt-fps": 0.3048,
    "phi_deg": math.pi / 180.0,
    "theta_deg": math.pi / 180.0,
    "psi_deg": math.pi / 180.0,
    "attitude/phi-deg": math.pi / 180.0,
    "attitude/theta-deg": math.pi / 180.0,
    "attitude/psi-deg": math.pi / 180.0,
})

JSB_FEED_TO_JSB_ROW = {
    "jsb_feed_time_s": "jsb_time_s",
    "jsb_feed_lat_deg": "jsb_lat_deg",
    "jsb_feed_lon_deg": "jsb_lon_deg",
    "jsb_feed_alt_m": "jsb_alt_m",
    "jsb_feed_vn_mps": "jsb_vn_mps",
    "jsb_feed_ve_mps": "jsb_ve_mps",
    "jsb_feed_vd_mps": "jsb_vd_mps",
    "jsb_feed_airspeed_mps": "jsb_airspeed_mps",
    "jsb_feed_roll_rad": "jsb_roll_rad",
    "jsb_feed_pitch_rad": "jsb_pitch_rad",
    "jsb_feed_yaw_rad": "jsb_yaw_rad",
}


def blank_jsbsim_row():
    return {field_name: "" for field_name, _column_name, _scale in JSBSIM_FIELDS}


def static_gps_input_row(args):
    row = blank_jsbsim_row()
    row.update({
        "jsb_time_s": f"{time.monotonic():.3f}",
        "jsb_lat_deg": args.gps_input_static_lat,
        "jsb_lon_deg": args.gps_input_static_lon,
        "jsb_alt_m": args.gps_input_static_alt_m,
        "jsb_vn_mps": args.gps_input_static_vn,
        "jsb_ve_mps": args.gps_input_static_ve,
        "jsb_vd_mps": args.gps_input_static_vd,
    })
    return row


def blank_gps_input_log_row(enabled=False):
    return {
        "gps_input_enabled": int(enabled),
        "gps_tx_lat_deg": "",
        "gps_tx_lon_deg": "",
        "gps_tx_alt_m": "",
        "gps_tx_vn_mps": "",
        "gps_tx_ve_mps": "",
        "gps_tx_vd_mps": "",
        "gps_tx_fix_type": "",
        "gps_tx_satellites": "",
        "gps_tx_count": 0,
    }


def blank_airspeed_log_row(enabled=False):
    return {
        "airspeed_input_enabled": int(enabled),
        "airspeed_tx_mps": "",
        "airspeed_tx_count": 0,
    }


def blank_attitude_log_row(enabled=False):
    return {
        "attitude_input_enabled": int(enabled),
        "att_tx_roll_rad": "",
        "att_tx_pitch_rad": "",
        "att_tx_yaw_rad": "",
        "att_tx_count": 0,
    }


def blank_jsb_feed_log_row(enabled=False):
    return {
        "jsb_feed_enabled": int(enabled),
        "jsb_feed_time_s": "",
        "jsb_feed_lat_deg": "",
        "jsb_feed_lon_deg": "",
        "jsb_feed_alt_m": "",
        "jsb_feed_vn_mps": "",
        "jsb_feed_ve_mps": "",
        "jsb_feed_vd_mps": "",
        "jsb_feed_airspeed_mps": "",
        "jsb_feed_roll_rad": "",
        "jsb_feed_pitch_rad": "",
        "jsb_feed_yaw_rad": "",
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


def normalize_header_name(name):
    return name.strip().lower()


def normalize_yaw_rad(yaw_rad):
    return (yaw_rad + math.pi) % (2.0 * math.pi) - math.pi


class JSBSimStateFeeder:
    def __init__(self, path, stale_timeout_s=JSB_FEED_STALE_TIMEOUT_S):
        self.path = path
        self.stale_timeout_s = stale_timeout_s
        self.monitor = JSBSimCSVMonitor(path)
        self.headers_reported = False
        self.warning_keys = set()
        self.last_valid_feed_row = None
        self.last_valid_jsb_row = None
        self.last_valid_wall = None
        self.last_feed_time_s = None
        self.last_source_mtime = None
        self.stale_reported = False

    def _warn_once(self, key, message):
        if key in self.warning_keys:
            return
        self.warning_keys.add(key)
        print(message)

    def _header_index(self):
        if self.monitor.headers is None:
            self.monitor._read_header()
        if not self.monitor.headers:
            return {}
        return {normalize_header_name(name): index for index, name in enumerate(self.monitor.headers)}

    def _read_raw_latest(self):
        header_index = self._header_index()
        if not header_index:
            self._warn_once("header", f"Warning: JSBSim state input has no readable header: {self.path}")
            return None
        if not self.headers_reported:
            print("JSBSim state input columns: " + ", ".join(self.monitor.headers))
            self.headers_reported = True

        last_line = self.monitor._read_last_line()
        if last_line is None:
            self._warn_once("row", f"Warning: JSBSim state input has no data rows yet: {self.path}")
            return None
        parsed_rows = list(csv.reader([last_line]))
        if not parsed_rows:
            return None
        values = parsed_rows[0]

        raw = {}
        for field_name, aliases in JSB_FEED_ALIASES.items():
            value = ""
            for alias in aliases:
                index = header_index.get(normalize_header_name(alias))
                if index is None:
                    continue
                try:
                    value = values[index]
                except IndexError:
                    value = ""
                scale = JSB_FEED_ALIAS_SCALES.get(alias)
                if scale is not None and value != "":
                    try:
                        value = float(value) * scale
                    except (TypeError, ValueError):
                        pass
                break
            raw[field_name] = value
        return raw

    def _parse_float(self, raw, field_name, default=None, required=False):
        value = raw.get(field_name, "")
        if value == "":
            if required:
                self._warn_once(field_name, f"Warning: JSBSim state input missing required field {field_name}")
            return default
        try:
            return float(value)
        except (TypeError, ValueError):
            self._warn_once(field_name, f"Warning: JSBSim state input field {field_name} is not numeric: {value}")
            return default

    def _validate(self, feed_row):
        reasons = []
        lat = feed_row["jsb_feed_lat_deg"]
        lon = feed_row["jsb_feed_lon_deg"]
        alt = feed_row["jsb_feed_alt_m"]
        airspeed = feed_row["jsb_feed_airspeed_mps"]
        roll = feed_row["jsb_feed_roll_rad"]
        pitch = feed_row["jsb_feed_pitch_rad"]
        yaw = feed_row["jsb_feed_yaw_rad"]
        if not math.isfinite(lat) or lat < -90.0 or lat > 90.0:
            reasons.append(f"latitude out of range: {lat}")
        if not math.isfinite(lon) or lon < -180.0 or lon > 180.0:
            reasons.append(f"longitude out of range: {lon}")
        if not math.isfinite(alt):
            reasons.append(f"altitude not finite: {alt}")
        for field_name in ("jsb_feed_vn_mps", "jsb_feed_ve_mps", "jsb_feed_vd_mps"):
            value = feed_row[field_name]
            if not math.isfinite(value):
                reasons.append(f"{field_name} not finite: {value}")
        if not math.isfinite(airspeed) or airspeed < 0.0 or airspeed > 150.0:
            reasons.append(f"airspeed out of range: {airspeed}")
        if not math.isfinite(roll) or abs(roll) > math.pi * 0.5:
            reasons.append(f"roll out of range: {roll}")
        if not math.isfinite(pitch) or abs(pitch) > math.pi * 0.5:
            reasons.append(f"pitch out of range: {pitch}")
        if not math.isfinite(yaw):
            reasons.append(f"yaw not finite: {yaw}")
        return reasons

    def _format_feed_row(self, feed_row):
        row = blank_jsb_feed_log_row(True)
        for field_name, value in feed_row.items():
            if field_name == "jsb_feed_time_s":
                row[field_name] = f"{value:.3f}" if value != "" else ""
            elif field_name in row:
                row[field_name] = f"{value:.7f}" if value != "" else ""
        return row

    def _jsb_row_from_feed(self, feed_row):
        row = blank_jsbsim_row()
        for feed_field, jsb_field in JSB_FEED_TO_JSB_ROW.items():
            row[jsb_field] = feed_row[feed_field]
        return row

    def read_latest(self, args):
        raw = self._read_raw_latest()
        if raw is None:
            return self.last_valid_jsb_row

        feed_row = {
            "jsb_feed_time_s": self._parse_float(raw, "jsb_feed_time_s", time.monotonic()),
            "jsb_feed_lat_deg": self._parse_float(raw, "jsb_feed_lat_deg", math.nan, required=True),
            "jsb_feed_lon_deg": self._parse_float(raw, "jsb_feed_lon_deg", math.nan, required=True),
            "jsb_feed_alt_m": self._parse_float(raw, "jsb_feed_alt_m", math.nan, required=True),
            "jsb_feed_vn_mps": self._parse_float(raw, "jsb_feed_vn_mps", args.gps_input_static_vn),
            "jsb_feed_ve_mps": self._parse_float(raw, "jsb_feed_ve_mps", args.gps_input_static_ve),
            "jsb_feed_vd_mps": self._parse_float(raw, "jsb_feed_vd_mps", args.gps_input_static_vd),
            "jsb_feed_airspeed_mps": self._parse_float(raw, "jsb_feed_airspeed_mps", math.nan, required=True),
            "jsb_feed_roll_rad": self._parse_float(raw, "jsb_feed_roll_rad", math.nan, required=True),
            "jsb_feed_pitch_rad": self._parse_float(raw, "jsb_feed_pitch_rad", math.nan, required=True),
            "jsb_feed_yaw_rad": normalize_yaw_rad(self._parse_float(raw, "jsb_feed_yaw_rad", math.nan, required=True)),
        }
        reasons = self._validate(feed_row)
        if reasons:
            self._warn_once("invalid", "Warning: JSBSim state input invalid: " + "; ".join(reasons))
            return self.last_valid_jsb_row

        source_mtime = self.monitor.mtime()
        data_advanced = (
            self.last_valid_wall is None or
            feed_row["jsb_feed_time_s"] != self.last_feed_time_s or
            source_mtime != self.last_source_mtime
        )
        self.last_valid_feed_row = self._format_feed_row(feed_row)
        self.last_valid_jsb_row = self._jsb_row_from_feed(feed_row)
        self.last_feed_time_s = feed_row["jsb_feed_time_s"]
        self.last_source_mtime = source_mtime
        if data_advanced:
            self.last_valid_wall = time.time()
            self.stale_reported = False
        return self.last_valid_jsb_row

    def log_row(self):
        if self.last_valid_feed_row is None:
            return blank_jsb_feed_log_row(True)
        return dict(self.last_valid_feed_row)

    def is_stale(self):
        return self.last_valid_wall is not None and time.time() - self.last_valid_wall > self.stale_timeout_s

    def maybe_print_stale(self):
        if self.is_stale() and not self.stale_reported:
            print(f"Warning: JSBSim state input stale for more than {self.stale_timeout_s:.1f}s")
            self.stale_reported = True

    def print_observer(self):
        if self.last_valid_feed_row is None:
            return
        self.maybe_print_stale()
        row = self.last_valid_feed_row
        print(
            "JSB_FEED "
            f"t={row['jsb_feed_time_s']} lat={row['jsb_feed_lat_deg']} lon={row['jsb_feed_lon_deg']} "
            f"alt={row['jsb_feed_alt_m']} airspeed={row['jsb_feed_airspeed_mps']} "
            f"roll={row['jsb_feed_roll_rad']} pitch={row['jsb_feed_pitch_rad']} yaw={row['jsb_feed_yaw_rad']}"
        )


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

    def print_status(self, last_jsb_t, gps_input_injector=None):
        gps_input_status = ""
        if gps_input_injector is not None:
            gps_input_status = (
                f" gps_input_sent={gps_input_injector.gps_input_sent_count}"
                f" gps_input_dryrun={gps_input_injector.gps_input_dryrun_count}"
            )
        print(
            f"HIL: state_sent={self.hil_state_sent_count} gps_sent={self.hil_gps_sent_count} "
            f"dryrun_state={self.hil_state_dryrun_count} dryrun_gps={self.hil_gps_dryrun_count} "
            f"last_jsb_t={last_jsb_t if last_jsb_t is not None else ''}{gps_input_status}"
        )


class GPSInputInjector:
    def __init__(self, master, rate_hz, dry_run, gps_id, ignore_flags, debug):
        self.master = master
        self.rate_hz = rate_hz
        self.dry_run = dry_run
        self.gps_id = gps_id
        self.ignore_flags = ignore_flags
        self.debug = debug
        self.gps_input_sent_count = 0
        self.gps_input_dryrun_count = 0
        self.gps_input_valid_count = 0
        self.gps_input_invalid_count = 0
        self.gps_input_send_exception_count = 0
        self.next_debug_print = 0.0
        self.last_packet = None
        self.last_time_usec = 0
        self.gps_input_enabled = hasattr(master.mav, "gps_input_send")
        print(f"GPS_INPUT available: {'yes' if self.gps_input_enabled else 'no'}")
        if self.gps_input_enabled:
            print(f"GPS_INPUT send signature: {inspect.signature(master.mav.gps_input_send)}")
            print("GPS_INPUT yaw extension available: no")

    def gps_time(self):
        unix_now = time.time()
        seconds_since_gps_epoch = unix_now - GPS_EPOCH_UNIX_S
        gps_week = int(seconds_since_gps_epoch // 604800)
        gps_week_ms = int((seconds_since_gps_epoch % 604800) * 1000)
        time_usec = int(unix_now * 1.0e6)
        if time_usec <= self.last_time_usec:
            time_usec = self.last_time_usec + 1
        self.last_time_usec = time_usec
        return time_usec, gps_week_ms, gps_week

    def validate_packet(self, packet):
        reasons = []
        if not math.isfinite(packet["lat_deg"]) or packet["lat_deg"] < -90.0 or packet["lat_deg"] > 90.0:
            reasons.append(f"lat_deg out of range: {packet['lat_deg']}")
        if not math.isfinite(packet["lon_deg"]) or packet["lon_deg"] < -180.0 or packet["lon_deg"] > 180.0:
            reasons.append(f"lon_deg out of range: {packet['lon_deg']}")
        if not math.isfinite(packet["alt"]):
            reasons.append(f"alt not finite: {packet['alt']}")
        if packet["fix_type"] != 3:
            reasons.append(f"fix_type is not 3: {packet['fix_type']}")
        if packet["satellites_visible"] <= 6:
            reasons.append(f"satellites_visible too low: {packet['satellites_visible']}")
        for field in ("vn", "ve", "vd"):
            if not math.isfinite(packet[field]):
                reasons.append(f"{field} not finite: {packet[field]}")
        return reasons

    def maybe_print_debug(self, packet, reasons):
        if not self.debug:
            return
        now = time.time()
        if now < self.next_debug_print:
            return
        self.next_debug_print = now + 1.0
        reason_text = "; ".join(reasons) if reasons else "valid"
        print(
            "GPS_INPUT sample: "
            f"time_usec={packet['time_usec']} gps_id={packet['gps_id']} "
            f"ignore_flags={packet['ignore_flags']} time_week_ms={packet['time_week_ms']} "
            f"time_week={packet['time_week']} fix_type={packet['fix_type']} "
            f"lat={packet['lat']} lon={packet['lon']} alt={packet['alt']} "
            f"hdop={packet['hdop']} vdop={packet['vdop']} "
            f"vn={packet['vn']} ve={packet['ve']} vd={packet['vd']} "
            f"speed_accuracy={packet['speed_accuracy']} horiz_accuracy={packet['horiz_accuracy']} "
            f"vert_accuracy={packet['vert_accuracy']} satellites_visible={packet['satellites_visible']} "
            f"yaw=unsupported status={reason_text}"
        )

    def send(self, row):
        if not self.gps_input_enabled:
            return False
        lat = safe_float(row, "jsb_lat_deg")
        lon = safe_float(row, "jsb_lon_deg")
        alt = safe_float(row, "jsb_alt_m")
        if lat is None or lon is None or alt is None:
            self.gps_input_invalid_count += 1
            if self.debug and time.time() >= self.next_debug_print:
                print("GPS_INPUT invalid: missing lat/lon/alt from JSBSim row")
                self.next_debug_print = time.time() + 1.0
            return False
        vn = safe_float(row, "jsb_vn_mps", 0.0)
        ve = safe_float(row, "jsb_ve_mps", 0.0)
        vd = safe_float(row, "jsb_vd_mps", 0.0)
        time_usec, time_week_ms, time_week = self.gps_time()
        packet = {
            "time_usec": time_usec,
            "gps_id": self.gps_id,
            "ignore_flags": self.ignore_flags,
            "time_week_ms": time_week_ms,
            "time_week": time_week,
            "fix_type": 3,
            "lat_deg": lat,
            "lon_deg": lon,
            "lat": int(lat * 1.0e7),
            "lon": int(lon * 1.0e7),
            "alt": alt,
            "hdop": 0.8,
            "vdop": 1.0,
            "vn": vn,
            "ve": ve,
            "vd": vd,
            "speed_accuracy": 0.5,
            "horiz_accuracy": 1.0,
            "vert_accuracy": 1.5,
            "satellites_visible": 12,
        }
        reasons = self.validate_packet(packet)
        self.maybe_print_debug(packet, reasons)
        if reasons:
            self.gps_input_invalid_count += 1
            if self.debug:
                print(f"GPS_INPUT invalid: {'; '.join(reasons)}")
            return False
        self.gps_input_valid_count += 1
        if not self.dry_run:
            try:
                self.master.mav.gps_input_send(
                    packet["time_usec"],
                    packet["gps_id"],
                    packet["ignore_flags"],
                    packet["time_week_ms"],
                    packet["time_week"],
                    packet["fix_type"],
                    packet["lat"],
                    packet["lon"],
                    packet["alt"],
                    packet["hdop"],
                    packet["vdop"],
                    packet["vn"],
                    packet["ve"],
                    packet["vd"],
                    packet["speed_accuracy"],
                    packet["horiz_accuracy"],
                    packet["vert_accuracy"],
                    packet["satellites_visible"],
                )
            except Exception as ex:
                self.gps_input_send_exception_count += 1
                print(f"GPS_INPUT send exception: {ex}")
                return False
            self.gps_input_sent_count += 1
        else:
            self.gps_input_dryrun_count += 1
        self.last_packet = packet
        return True

    def print_status(self, last_jsb_t):
        print(
            f"GPS_INPUT: sent={self.gps_input_sent_count} dryrun={self.gps_input_dryrun_count} "
            f"valid={self.gps_input_valid_count} invalid={self.gps_input_invalid_count} "
            f"exceptions={self.gps_input_send_exception_count} last_jsb_t={last_jsb_t if last_jsb_t is not None else ''}"
        )

    def print_static_status(self):
        print(f"GPS_INPUT_STATIC: sent={self.gps_input_sent_count} valid={self.gps_input_valid_count}")

    def print_observer(self):
        if self.last_packet is None:
            return
        packet = self.last_packet
        print(
            "GPS_TX "
            f"lat={packet['lat_deg']:.7f} lon={packet['lon_deg']:.7f} alt={packet['alt']:.2f} "
            f"vn={packet['vn']:.2f} ve={packet['ve']:.2f} vd={packet['vd']:.2f} "
            f"fix={packet['fix_type']} sats={packet['satellites_visible']} count={self.gps_input_sent_count}"
        )

    def log_row(self):
        row = blank_gps_input_log_row(True)
        row["gps_tx_count"] = self.gps_input_sent_count
        if self.last_packet is not None:
            packet = self.last_packet
            row.update({
                "gps_tx_lat_deg": f"{packet['lat_deg']:.7f}",
                "gps_tx_lon_deg": f"{packet['lon_deg']:.7f}",
                "gps_tx_alt_m": f"{packet['alt']:.2f}",
                "gps_tx_vn_mps": f"{packet['vn']:.2f}",
                "gps_tx_ve_mps": f"{packet['ve']:.2f}",
                "gps_tx_vd_mps": f"{packet['vd']:.2f}",
                "gps_tx_fix_type": packet["fix_type"],
                "gps_tx_satellites": packet["satellites_visible"],
            })
        return row


class AirspeedInjector:
    MAVLINK_NAME = b"AIRSPEED"

    def __init__(self, master, rate_hz, airspeed_mps):
        self.master = master
        self.rate_hz = rate_hz
        self.airspeed_mps = airspeed_mps
        self.airspeed_tx_count = 0
        self.airspeed_send_exception_count = 0
        self.last_airspeed_mps = ""
        self.airspeed_enabled = hasattr(master.mav, "named_value_float_send")
        print(f"AIRSPEED MAVLink transmit message: NAMED_VALUE_FLOAT")
        print(f"NAMED_VALUE_FLOAT available: {'yes' if self.airspeed_enabled else 'no'}")
        if self.airspeed_enabled:
            print(f"NAMED_VALUE_FLOAT send signature: {inspect.signature(master.mav.named_value_float_send)}")
        print(
            "AIRSPEED Pixhawk consumption: not direct; inbound NAMED_VALUE_FLOAT "
            "is logged by ArduPilot but not consumed by AP_Airspeed"
        )

    def validate(self, airspeed_mps=None):
        value = self.airspeed_mps if airspeed_mps is None else airspeed_mps
        if not math.isfinite(value):
            return "airspeed is not finite"
        if value < 0.0:
            return "airspeed is negative"
        if value > 150.0:
            return "airspeed is above 150 m/s"
        return None

    def send(self, airspeed_mps=None):
        if not self.airspeed_enabled:
            return False
        value = self.airspeed_mps if airspeed_mps is None else airspeed_mps
        reason = self.validate(value)
        if reason is not None:
            print(f"AIRSPEED invalid: {reason}")
            return False
        try:
            self.master.mav.named_value_float_send(
                int(time.monotonic() * 1000.0),
                self.MAVLINK_NAME,
                value,
            )
        except Exception as ex:
            self.airspeed_send_exception_count += 1
            print(f"AIRSPEED send exception: {ex}")
            return False
        self.airspeed_tx_count += 1
        self.last_airspeed_mps = value
        return True

    def print_observer(self):
        if self.last_airspeed_mps == "":
            return
        print(f"AS_TX airspeed={self.last_airspeed_mps:.2f} count={self.airspeed_tx_count}")

    def log_row(self):
        row = blank_airspeed_log_row(True)
        row["airspeed_tx_count"] = self.airspeed_tx_count
        if self.last_airspeed_mps != "":
            row["airspeed_tx_mps"] = f"{self.last_airspeed_mps:.2f}"
        return row


class AttitudeDisplayInjector:
    MAVLINK_NAMES = (
        ("roll", b"SR75_ROLL"),
        ("pitch", b"SR75_PITCH"),
        ("yaw", b"SR75_YAW"),
    )

    def __init__(self, master, rate_hz, roll_deg, pitch_deg, yaw_deg):
        self.master = master
        self.rate_hz = rate_hz
        self.roll_deg = roll_deg
        self.pitch_deg = pitch_deg
        self.yaw_deg = yaw_deg
        self.roll_rad = math.radians(roll_deg)
        self.pitch_rad = math.radians(pitch_deg)
        self.yaw_rad = math.radians(yaw_deg)
        self.attitude_tx_count = 0
        self.attitude_send_exception_count = 0
        self.last_sent = False
        self.attitude_enabled = hasattr(master.mav, "named_value_float_send")
        print("ATTITUDE display MAVLink transmit message: NAMED_VALUE_FLOAT")
        print(f"NAMED_VALUE_FLOAT available: {'yes' if self.attitude_enabled else 'no'}")
        if self.attitude_enabled:
            print(f"NAMED_VALUE_FLOAT send signature: {inspect.signature(master.mav.named_value_float_send)}")

    def values(self, attitude_rad=None):
        if attitude_rad is not None:
            return attitude_rad
        return {
            "roll": self.roll_rad,
            "pitch": self.pitch_rad,
            "yaw": self.yaw_rad,
        }

    def validate(self, attitude_rad=None):
        values = self.values(attitude_rad)
        for name, value in (
            ("roll", values["roll"]),
            ("pitch", values["pitch"]),
            ("yaw", values["yaw"]),
        ):
            if not math.isfinite(value):
                return f"{name} is not finite"
        if abs(values["roll"]) > math.pi * 0.5:
            return "roll is outside +/-90 deg"
        if abs(values["pitch"]) > math.pi * 0.5:
            return "pitch is outside +/-90 deg"
        if abs(values["yaw"]) > math.pi:
            return "yaw is outside +/-pi"
        return None

    def send(self, attitude_rad=None):
        if not self.attitude_enabled:
            return False
        values = self.values(attitude_rad)
        reason = self.validate(values)
        if reason is not None:
            print(f"ATTITUDE display invalid: {reason}")
            return False
        try:
            now_ms = int(time.monotonic() * 1000.0)
            for value_name, mavlink_name in self.MAVLINK_NAMES:
                self.master.mav.named_value_float_send(
                    now_ms,
                    mavlink_name,
                    values[value_name],
                )
        except Exception as ex:
            self.attitude_send_exception_count += 1
            print(f"ATTITUDE display send exception: {ex}")
            return False
        self.attitude_tx_count += 1
        self.last_sent = True
        self.roll_rad = values["roll"]
        self.pitch_rad = values["pitch"]
        self.yaw_rad = values["yaw"]
        self.roll_deg = math.degrees(values["roll"])
        self.pitch_deg = math.degrees(values["pitch"])
        self.yaw_deg = math.degrees(values["yaw"])
        return True

    def print_observer(self):
        if not self.last_sent:
            return
        print(
            "ATT_TX "
            f"roll_deg={self.roll_deg:.2f} pitch_deg={self.pitch_deg:.2f} "
            f"yaw_deg={self.yaw_deg:.2f} count={self.attitude_tx_count}"
        )

    def log_row(self):
        row = blank_attitude_log_row(True)
        row["att_tx_count"] = self.attitude_tx_count
        if self.last_sent:
            row.update({
                "att_tx_roll_rad": f"{self.roll_rad:.7f}",
                "att_tx_pitch_rad": f"{self.pitch_rad:.7f}",
                "att_tx_yaw_rad": f"{self.yaw_rad:.7f}",
            })
        return row


def parse_args():
    parser = argparse.ArgumentParser(
        description="SR-75 Layer 2 Pixhawk MAVLink HIL bridge skeleton"
    )
    parser.add_argument("--pixhawk", required=True, help="Pixhawk serial device, e.g. /dev/ttyS4 or /dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200, help="Pixhawk serial baud rate")
    parser.add_argument("--mp-out", default=None, help="Optional Mission Planner MAVLink output, e.g. udp:192.168.1.20:14550")
    parser.add_argument("--mp-in", default=None, help="Optional Mission Planner MAVLink input, e.g. udp:0.0.0.0:14551")
    parser.add_argument("--jsbsim-csv", default=None, help="Optional read-only JSBSim CSV monitor path")
    parser.add_argument("--jsbsim-state-input", default=None, help="Drive GPS_INPUT, AIRSPEED, and display attitude from a JSBSim state CSV path")
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
    parser.add_argument("--gps-input-inject", action="store_true", help="Send stationary Layer 2D GPS_INPUT data to Pixhawk")
    parser.add_argument("--gps-input-rate-hz", type=float, default=5.0, help="GPS_INPUT injection rate")
    parser.add_argument("--gps-rate-hz", dest="gps_input_rate_hz", type=float, default=argparse.SUPPRESS, help="Alias for --gps-input-rate-hz")
    parser.add_argument("--gps-input-dry-run", action="store_true", help="Compute GPS_INPUT messages but do not send them")
    parser.add_argument("--gps-input-id", type=int, default=0, help="GPS_INPUT gps_id field")
    parser.add_argument("--gps-input-ignore-flags", type=int, default=0, help="GPS_INPUT ignore_flags field")
    parser.add_argument("--gps-input-debug", action="store_true", help="Print one GPS_INPUT sample per second")
    parser.add_argument("--gps-input-static-test", action="store_true", help="Send static GPS_INPUT data independent of JSBSim CSV")
    parser.add_argument("--gps-input-static-lat", type=float, default=32.5378885, help="Static GPS_INPUT latitude")
    parser.add_argument("--gps-lat", dest="gps_input_static_lat", type=float, default=argparse.SUPPRESS, help="Alias for --gps-input-static-lat")
    parser.add_argument("--gps-input-static-lon", type=float, default=74.3661944, help="Static GPS_INPUT longitude")
    parser.add_argument("--gps-lon", dest="gps_input_static_lon", type=float, default=argparse.SUPPRESS, help="Alias for --gps-input-static-lon")
    parser.add_argument("--gps-input-static-alt-m", type=float, default=240.2, help="Static GPS_INPUT altitude in meters")
    parser.add_argument("--gps-alt-m", dest="gps_input_static_alt_m", type=float, default=argparse.SUPPRESS, help="Alias for --gps-input-static-alt-m")
    parser.add_argument("--gps-input-static-vn", type=float, default=0.0, help="Static GPS_INPUT north velocity in m/s")
    parser.add_argument("--gps-input-static-ve", type=float, default=0.0, help="Static GPS_INPUT east velocity in m/s")
    parser.add_argument("--gps-input-static-vd", type=float, default=0.0, help="Static GPS_INPUT down velocity in m/s")
    parser.add_argument("--airspeed-inject", action="store_true", help="Transmit synthetic airspeed as MAVLink NAMED_VALUE_FLOAT")
    parser.add_argument("--airspeed-mps", type=float, default=69.0, help="Synthetic airspeed value in m/s")
    parser.add_argument("--airspeed-rate-hz", type=float, default=10.0, help="Synthetic airspeed transmit rate")
    parser.add_argument("--attitude-inject", action="store_true", help="Transmit synthetic display attitude as MAVLink NAMED_VALUE_FLOAT")
    parser.add_argument("--att-roll-deg", type=float, default=0.0, help="Synthetic display roll angle in degrees")
    parser.add_argument("--att-pitch-deg", type=float, default=0.0, help="Synthetic display pitch angle in degrees")
    parser.add_argument("--att-yaw-deg", type=float, default=0.0, help="Synthetic display yaw angle in degrees")
    parser.add_argument("--attitude-rate-hz", type=float, default=10.0, help="Synthetic display attitude transmit rate")
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
    message_id = getattr(mavutil.mavlink, f"MAVLINK_MSG_ID_{message_name}", None)
    if message_id is None:
        print(f"{message_name} message is not available in this MAVLink dialect")
        return
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


def make_row(
    start_time,
    latest,
    jsbsim_row=None,
    gps_input_row=None,
    airspeed_input_row=None,
    attitude_input_row=None,
    jsb_feed_row=None,
):
    now = time.time()
    heartbeat = latest.get("HEARTBEAT")
    mode, armed = mode_and_armed(heartbeat)

    servo = latest.get("SERVO_OUTPUT_RAW")
    rc = latest.get("RC_CHANNELS")
    attitude = latest.get("ATTITUDE")
    ahrs2 = latest.get("AHRS2")
    ekf_status = latest.get("EKF_STATUS_REPORT")
    local_position_ned = latest.get("LOCAL_POSITION_NED")
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
    row["att_roll_rad"] = getattr(attitude, "roll", "")
    row["att_pitch_rad"] = getattr(attitude, "pitch", "")
    row["att_yaw_rad"] = getattr(attitude, "yaw", "")
    row["ahrs2_roll_rad"] = getattr(ahrs2, "roll", "")
    row["ahrs2_pitch_rad"] = getattr(ahrs2, "pitch", "")
    row["ahrs2_yaw_rad"] = getattr(ahrs2, "yaw", "")
    row["ekf_flags"] = getattr(ekf_status, "flags", "")
    row["local_ned_x_m"] = getattr(local_position_ned, "x", "")
    row["local_ned_y_m"] = getattr(local_position_ned, "y", "")
    row["local_ned_z_m"] = getattr(local_position_ned, "z", "")
    row["lat_deg"] = getattr(global_position, "lat", "") / 1.0e7 if global_position is not None else ""
    row["lon_deg"] = getattr(global_position, "lon", "") / 1.0e7 if global_position is not None else ""
    row["alt_m"] = getattr(global_position, "alt", "") / 1000.0 if global_position is not None else ""
    row["relative_alt_m"] = getattr(global_position, "relative_alt", "") / 1000.0 if global_position is not None else ""
    row["airspeed_mps"] = getattr(vfr_hud, "airspeed", "")
    row["groundspeed_mps"] = getattr(vfr_hud, "groundspeed", "")
    row["throttle_pct"] = getattr(vfr_hud, "throttle", "")
    row.update(jsbsim_row if jsbsim_row is not None else blank_jsbsim_row())
    row.update(gps_input_row if gps_input_row is not None else blank_gps_input_log_row())
    row.update(airspeed_input_row if airspeed_input_row is not None else blank_airspeed_log_row())
    row.update(attitude_input_row if attitude_input_row is not None else blank_attitude_log_row())
    row.update(jsb_feed_row if jsb_feed_row is not None else blank_jsb_feed_log_row())
    return row


def print_status(row):
    servos = " ".join(str(row[f"servo{i}_pwm"]) for i in range(1, 9))
    rcs = " ".join(str(row[f"rc{i}_pwm"]) for i in range(1, 9))
    print(
        f"t={row['time_s']} mode={row['mode']} armed={row['armed']} "
        f"servo=[{servos}] rc=[{rcs}] "
        f"rpy=({row['roll_rad']},{row['pitch_rad']},{row['yaw_rad']})"
    )


def print_ahrs_observer(row):
    print(
        "AHRS_OBS "
        f"roll={row['att_roll_rad']} pitch={row['att_pitch_rad']} yaw={row['att_yaw_rad']} "
        f"ahrs2_roll={row['ahrs2_roll_rad']} ahrs2_pitch={row['ahrs2_pitch_rad']} "
        f"ahrs2_yaw={row['ahrs2_yaw_rad']} ekf_flags={row['ekf_flags']} "
        f"pos_ned=({row['local_ned_x_m']},{row['local_ned_y_m']},{row['local_ned_z_m']})"
    )


def print_jsbsim_status(row):
    print(
        f"JSBSim: t={row['jsb_time_s']} alt_m={row['jsb_alt_m']} agl_m={row['jsb_agl_m']} "
        f"airspeed_mps={row['jsb_airspeed_mps']} "
        f"rpy=({row['jsb_roll_rad']},{row['jsb_pitch_rad']},{row['jsb_yaw_rad']}) "
        f"alpha={row['jsb_alpha_rad']}"
    )


def print_pixhawk_nav_compare(row, latest):
    if row is None or row.get("jsb_time_s", "") == "":
        return

    global_position = latest.get("GLOBAL_POSITION_INT")
    gps_raw = latest.get("GPS_RAW_INT")
    vfr_hud = latest.get("VFR_HUD")
    ekf_status = latest.get("EKF_STATUS_REPORT")

    pix_lat = getattr(global_position, "lat", "") / 1.0e7 if global_position is not None else ""
    pix_lon = getattr(global_position, "lon", "") / 1.0e7 if global_position is not None else ""
    global_alt_m = getattr(global_position, "alt", "") / 1000.0 if global_position is not None else ""
    rel_alt_m = getattr(global_position, "relative_alt", "") / 1000.0 if global_position is not None else ""
    gps_raw_alt_m = getattr(gps_raw, "alt", "") / 1000.0 if gps_raw is not None else ""
    vfr_alt_m = getattr(vfr_hud, "alt", "") if vfr_hud is not None else ""
    pix_groundspeed = getattr(vfr_hud, "groundspeed", "")
    pix_heading = getattr(vfr_hud, "heading", "")
    gps_fix = getattr(gps_raw, "fix_type", "")
    gps_sats = getattr(gps_raw, "satellites_visible", "")
    ekf_flags = getattr(ekf_status, "flags", "")
    jsb_alt_m = safe_float(row, "jsb_alt_m")
    alt_error_global_m = global_alt_m - jsb_alt_m if global_alt_m != "" and jsb_alt_m is not None else ""
    alt_error_gps_raw_m = gps_raw_alt_m - jsb_alt_m if gps_raw_alt_m != "" and jsb_alt_m is not None else ""

    print(
        "NAV_COMPARE: "
        f"jsb_lat={row['jsb_lat_deg']} jsb_lon={row['jsb_lon_deg']} "
        f"jsb_alt_m={row['jsb_alt_m']} gps_raw_alt_m={gps_raw_alt_m} "
        f"global_alt_m={global_alt_m} rel_alt_m={rel_alt_m} vfr_alt_m={vfr_alt_m} "
        f"alt_error_global_m={alt_error_global_m} alt_error_gps_raw_m={alt_error_gps_raw_m} "
        f"jsb_vn={row['jsb_vn_mps']} jsb_ve={row['jsb_ve_mps']} jsb_vd={row['jsb_vd_mps']} "
        f"pix_lat={pix_lat} pix_lon={pix_lon} pix_alt_m={global_alt_m} pix_rel_alt_m={rel_alt_m} "
        f"pix_groundspeed={pix_groundspeed} pix_heading={pix_heading} "
        f"gps_fix={gps_fix} gps_sats={gps_sats} ekf_flags={ekf_flags}"
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
    jsb_state_feeder = JSBSimStateFeeder(args.jsbsim_state_input) if args.jsbsim_state_input is not None else None
    if jsb_state_feeder is not None:
        print(f"JSBSim state input: ENABLED path={args.jsbsim_state_input}")
    else:
        print("JSBSim state input: DISABLED")
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
    gps_input_dry_run = args.gps_input_dry_run or args.hil_dry_run
    gps_input_enabled = args.gps_input_inject or args.gps_input_static_test or jsb_state_feeder is not None
    gps_input_static_enabled = (args.gps_input_inject or args.gps_input_static_test) and jsb_state_feeder is None
    if gps_input_enabled:
        print("GPS_INPUT injection: ENABLED")
        print(f"GPS_INPUT rate: {args.gps_input_rate_hz:g} Hz")
        if jsb_state_feeder is not None:
            print("GPS_INPUT origin: JSBSim state input")
        else:
            print(
                "GPS_INPUT origin: "
                f"lat={args.gps_input_static_lat} lon={args.gps_input_static_lon} "
                f"alt={args.gps_input_static_alt_m} m"
            )
        if gps_input_dry_run:
            print("GPS_INPUT dry-run: computing messages but not sending")
        gps_input_injector = GPSInputInjector(
            master,
            args.gps_input_rate_hz,
            gps_input_dry_run,
            args.gps_input_id,
            args.gps_input_ignore_flags,
            args.gps_input_debug,
        )
    else:
        print("GPS_INPUT injection: DISABLED")
        gps_input_injector = None
    airspeed_enabled = args.airspeed_inject or jsb_state_feeder is not None
    if airspeed_enabled:
        print("AIRSPEED injection: ENABLED")
        print(f"AIRSPEED rate: {args.airspeed_rate_hz:g} Hz")
        if jsb_state_feeder is not None:
            print("AIRSPEED value: JSBSim state input")
        else:
            print(f"AIRSPEED value: {args.airspeed_mps:.1f} m/s")
        airspeed_injector = AirspeedInjector(master, args.airspeed_rate_hz, args.airspeed_mps)
    else:
        print("AIRSPEED injection: DISABLED")
        airspeed_injector = None
    attitude_enabled = args.attitude_inject or jsb_state_feeder is not None
    if attitude_enabled:
        print("ATTITUDE display injection: ENABLED")
        print(f"ATTITUDE rate: {args.attitude_rate_hz:g} Hz")
        if jsb_state_feeder is not None:
            print("ATTITUDE value: JSBSim state input")
        else:
            print(
                "ATTITUDE value: "
                f"roll={args.att_roll_deg:.2f} deg "
                f"pitch={args.att_pitch_deg:.2f} deg "
                f"yaw={args.att_yaw_deg:.2f} deg"
            )
        attitude_injector = AttitudeDisplayInjector(
            master,
            args.attitude_rate_hz,
            args.att_roll_deg,
            args.att_pitch_deg,
            args.att_yaw_deg,
        )
    else:
        print("ATTITUDE display injection: DISABLED")
        attitude_injector = None
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
    next_gps_input = 0.0
    next_gps_input_status = 0.0
    next_airspeed = 0.0
    next_attitude = 0.0
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
                    hil_injector.print_status(last_hil_jsb_t, gps_input_injector)
                    next_hil_status = now + 1.0

            if gps_input_injector is not None:
                if hil_injector is None and not gps_input_static_enabled and now >= next_gps_input_status:
                    gps_input_injector.print_status(last_hil_jsb_t)
                    next_gps_input_status = now + 1.0

            if (
                hil_injector is not None or
                gps_input_injector is not None or
                airspeed_injector is not None or
                attitude_injector is not None
            ):
                hil_state_due = now >= next_hil_state
                hil_gps_due = now >= next_hil_gps
                gps_input_due = now >= next_gps_input
                airspeed_due = now >= next_airspeed
                attitude_due = now >= next_attitude
                jsb_feed_send_row = None
                if jsb_state_feeder is not None and (gps_input_due or airspeed_due or attitude_due):
                    jsb_feed_send_row = jsb_state_feeder.read_latest(args)
                    jsb_state_feeder.maybe_print_stale()
                if airspeed_injector is not None and airspeed_due:
                    if jsb_state_feeder is None or jsb_feed_send_row is not None:
                        airspeed = safe_float(jsb_feed_send_row, "jsb_airspeed_mps") if jsb_feed_send_row is not None else None
                        airspeed_injector.send(airspeed)
                    next_airspeed = now + (1.0 / max(args.airspeed_rate_hz, 0.1))
                if attitude_injector is not None and attitude_due:
                    if jsb_state_feeder is None or jsb_feed_send_row is not None:
                        attitude_rad = None
                        if jsb_feed_send_row is not None:
                            attitude_rad = {
                                "roll": safe_float(jsb_feed_send_row, "jsb_roll_rad", 0.0),
                                "pitch": safe_float(jsb_feed_send_row, "jsb_pitch_rad", 0.0),
                                "yaw": safe_float(jsb_feed_send_row, "jsb_yaw_rad", 0.0),
                            }
                        attitude_injector.send(attitude_rad)
                    next_attitude = now + (1.0 / max(args.attitude_rate_hz, 0.1))
                if gps_input_injector is not None and jsb_feed_send_row is not None and gps_input_due:
                    gps_input_injector.send(jsb_feed_send_row)
                    next_gps_input = now + (1.0 / max(args.gps_input_rate_hz, 0.1))
                    gps_input_due = False
                if gps_input_injector is not None and gps_input_static_enabled and gps_input_due:
                    gps_input_injector.send(static_gps_input_row(args))
                    next_gps_input = now + (1.0 / max(args.gps_input_rate_hz, 0.1))
                    gps_input_due = False
                if jsbsim_monitor is not None and (
                    (hil_injector is not None and (hil_state_due or hil_gps_due)) or
                    (gps_input_injector is not None and not gps_input_static_enabled and gps_input_due)
                ):
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
                        if hil_injector is not None and hil_state_due:
                            hil_injector.send_state(hil_row)
                        if hil_injector is not None and hil_gps_due:
                            hil_injector.send_gps(hil_row)
                        if gps_input_injector is not None and gps_input_due:
                            gps_input_injector.send(hil_row)

                    if hil_injector is not None and hil_state_due:
                        next_hil_state = now + (1.0 / max(args.hil_rate_hz, 0.1))
                    if hil_injector is not None and hil_gps_due:
                        next_hil_gps = now + (1.0 / max(args.hil_gps_rate_hz, 0.1))
                    if gps_input_injector is not None and gps_input_due:
                        next_gps_input = now + (1.0 / max(args.gps_input_rate_hz, 0.1))

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
                jsb_feed_row = None
                feed_jsbsim_row = None
                if jsb_state_feeder is not None:
                    feed_jsbsim_row = jsb_state_feeder.read_latest(args)
                    jsb_feed_row = jsb_state_feeder.log_row()
                jsbsim_row = feed_jsbsim_row if feed_jsbsim_row is not None else (
                    jsbsim_monitor.read_latest() if jsbsim_monitor is not None else None
                )
                gps_input_row = gps_input_injector.log_row() if gps_input_injector is not None else None
                airspeed_input_row = airspeed_injector.log_row() if airspeed_injector is not None else None
                attitude_input_row = attitude_injector.log_row() if attitude_injector is not None else None
                row = make_row(
                    start_time,
                    latest,
                    jsbsim_row,
                    gps_input_row,
                    airspeed_input_row,
                    attitude_input_row,
                    jsb_feed_row,
                )
                writer.writerow(row)
                csv_file.flush()
                print_status(row)
                print_ahrs_observer(row)
                if gps_input_injector is not None and (gps_input_static_enabled or jsb_state_feeder is not None):
                    gps_input_injector.print_observer()
                if airspeed_injector is not None:
                    airspeed_injector.print_observer()
                if attitude_injector is not None:
                    attitude_injector.print_observer()
                if jsb_state_feeder is not None:
                    jsb_state_feeder.print_observer()
                if jsbsim_monitor is not None:
                    print_jsbsim_status(row)
                if args.gps_input_inject or args.hil_inject or jsb_state_feeder is not None:
                    print_pixhawk_nav_compare(row, latest)
                next_print = now + 1.0

            time.sleep(0.002)

    print("SR-75 Layer 2 bridge skeleton stopped.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
