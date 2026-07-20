#!/usr/bin/env python3
"""
SR-75 host-side responder for ArduPilot's SIM_JSON UDP protocol.

This tool replies to SIM_JSON UDP control packets. It never opens a serial
port or emits MAVLink. JSBSim command output is disabled unless explicitly
enabled with --jsbsim-command-target.
"""

import argparse
import csv
import json
import math
import os
import socket
import struct
import sys
import time
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

from sr75_sim_json_actuator_bridge import (
    ActuatorChannelMap,
    ActuatorMapError,
    NormalizedActuatorCommand,
    SoftwareActuatorBridge,
    describe_channel_map,
)

JSBSIM_CONTROL_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "jsbsim_control"))
if JSBSIM_CONTROL_DIR not in sys.path:
    sys.path.insert(0, JSBSIM_CONTROL_DIR)

from sr75_jsbsim_command_sink import (  # noqa: E402
    JSBSimActuatorCommand,
    JSBSimCommandError,
    UDPJSBSimCommandSink,
)


DEFAULT_STATE_FILE = "/tmp/sr75_jsb_live_state.csv"
GRAVITY_MSS = 9.80665
FT_TO_M = 0.3048
FPS_TO_MPS = 0.3048
KTS_TO_MPS = 0.514444

SERVO16_MAGIC = 18458
SERVO32_MAGIC = 29569
SERVO16_STRUCT = struct.Struct("<HHI16H")
SERVO32_STRUCT = struct.Struct("<HHI32H")

LOG_FIELDS = [
    "host_monotonic_time",
    "request_count",
    "frame_rate",
    "frame_count",
    "source_ip",
    "source_port",
    "packet_valid",
    "state_age_ms",
    "simulation_timestamp",
    "latitude",
    "longitude",
    "altitude",
    "roll",
    "pitch",
    "yaw",
    "vn",
    "ve",
    "vd",
    "airspeed",
    "reply_bytes",
    "round_trip_or_processing_us",
    "error_reason",
    "act_left_elevon_norm",
    "act_right_elevon_norm",
    "act_elevator_norm",
    "act_aileron_norm",
    "act_rudder_norm",
    "act_throttle_left_norm",
    "act_throttle_right_norm",
    "act_turbojet_throttle_norm",
    "act_rato_norm",
    "act_stale",
    "act_warnings",
    "jsbsim_elevator_property",
    "jsbsim_elevator_value",
    "jsbsim_aileron_property",
    "jsbsim_aileron_value",
    "jsbsim_rudder_property",
    "jsbsim_rudder_value",
    "jsbsim_throttle_property",
    "jsbsim_throttle_value",
    "jsbsim_rato_property",
    "jsbsim_rato_value",
]


class StateError(Exception):
    """Raised when state cannot be converted to a valid SIM_JSON reply."""


def degrees_to_radians(value: float) -> float:
    return value * math.pi / 180.0


def radians_to_degrees(value: float) -> float:
    return value * 180.0 / math.pi


def feet_to_meters(value: float) -> float:
    return value * FT_TO_M


def fps_to_mps(value: float) -> float:
    return value * FPS_TO_MPS


def knots_to_mps(value: float) -> float:
    return value * KTS_TO_MPS


def normalize_yaw_rad(value: float) -> float:
    return (value + math.pi) % (2.0 * math.pi) - math.pi


def all_finite(values: Iterable[float]) -> bool:
    return all(math.isfinite(v) for v in values)


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    """Return ArduPilot Quaternion order q1,q2,q3,q4 from roll,pitch,yaw radians."""
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return normalize_quaternion((
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ))


def normalize_quaternion(quat: Sequence[float]) -> Tuple[float, float, float, float]:
    if len(quat) != 4:
        raise StateError("quaternion must have four elements")
    norm = math.sqrt(sum(q * q for q in quat))
    if not math.isfinite(norm) or norm <= 0.0:
        raise StateError("quaternion norm is invalid")
    return tuple(q / norm for q in quat)  # type: ignore[return-value]


def require_range(name: str, value: float, low: float, high: float) -> None:
    if not low <= value <= high:
        raise StateError(f"{name} out of range: {value}")


def first_present(row: Dict[str, str], names: Sequence[str]) -> Optional[str]:
    for name in names:
        if name in row and row[name] != "":
            return name
    return None


def parse_float(row: Dict[str, str], names: Sequence[str], required: bool = True) -> Tuple[Optional[float], Optional[str]]:
    name = first_present(row, names)
    if name is None:
        if required:
            raise StateError(f"missing CSV field: one of {', '.join(names)}")
        return None, None
    try:
        value = float(row[name])
    except ValueError as exc:
        raise StateError(f"invalid float in {name}: {row[name]!r}") from exc
    if not math.isfinite(value):
        raise StateError(f"non-finite value in {name}: {row[name]!r}")
    return value, name


@dataclass
class ControlPacket:
    magic: int
    frame_rate: int
    frame_count: int
    pwm: Tuple[int, ...]


@dataclass
class SimState:
    timestamp_s: float
    source_timestamp_s: float
    latitude_deg: float
    longitude_deg: float
    altitude_m: float
    roll_rad: float
    pitch_rad: float
    yaw_rad: float
    quaternion: Tuple[float, float, float, float]
    gyro_rad_s: Tuple[float, float, float]
    accel_body_mss: Tuple[float, float, float]
    velocity_ned_mps: Tuple[float, float, float]
    airspeed_mps: float
    row_signature: str
    row_monotonic_time: float
    reused_source_row: bool = False
    missing_fields: Tuple[str, ...] = ()

    def validate(self) -> None:
        require_range("latitude", self.latitude_deg, -90.0, 90.0)
        require_range("longitude", self.longitude_deg, -180.0, 180.0)
        if self.airspeed_mps < 0.0:
            raise StateError(f"airspeed is negative: {self.airspeed_mps}")
        values = [
            self.timestamp_s,
            self.source_timestamp_s,
            self.latitude_deg,
            self.longitude_deg,
            self.altitude_m,
            self.roll_rad,
            self.pitch_rad,
            self.yaw_rad,
            self.airspeed_mps,
            *self.quaternion,
            *self.gyro_rad_s,
            *self.accel_body_mss,
            *self.velocity_ned_mps,
        ]
        if not all_finite(values):
            raise StateError("state contains non-finite values")
        normalize_quaternion(self.quaternion)

    def to_json_bytes(self) -> bytes:
        payload = {
            "timestamp": self.timestamp_s,
            "latitude": self.latitude_deg,
            "longitude": self.longitude_deg,
            "altitude": self.altitude_m,
            "imu": {
                "gyro": list(self.gyro_rad_s),
                "accel_body": list(self.accel_body_mss),
            },
            "velocity": list(self.velocity_ned_mps),
            "attitude": [self.roll_rad, self.pitch_rad, self.yaw_rad],
            "quaternion": list(self.quaternion),
            "airspeed": self.airspeed_mps,
        }
        return (json.dumps(payload, separators=(",", ":"), sort_keys=True) + "\n").encode("utf-8")


class LatestCSVReader:
    def __init__(self, path: str):
        self.path = path
        self.headers: Optional[List[str]] = None
        self._header_mtime_ns: Optional[int] = None

    def _load_headers(self) -> None:
        stat = os.stat(self.path)
        if self.headers is not None and self._header_mtime_ns == stat.st_mtime_ns:
            return
        with open(self.path, "r", newline="", encoding="utf-8") as csv_file:
            self.headers = next(csv.reader(csv_file), None)
        self._header_mtime_ns = stat.st_mtime_ns
        if not self.headers:
            raise StateError(f"{self.path} has no CSV header")

    def read_latest(self) -> Tuple[Dict[str, str], str, float]:
        if not os.path.exists(self.path):
            raise StateError(f"state file does not exist: {self.path}")
        self._load_headers()
        assert self.headers is not None
        stat = os.stat(self.path)
        with open(self.path, "rb") as csv_file:
            csv_file.seek(0, os.SEEK_END)
            end_pos = csv_file.tell()
            block_size = min(65536, end_pos)
            csv_file.seek(end_pos - block_size)
            data = csv_file.read(block_size).decode("utf-8", errors="ignore")
        lines = [line for line in data.splitlines() if line.strip()]
        if len(lines) < 2:
            raise StateError(f"{self.path} has no complete data rows")
        last_line = lines[-1]
        parsed = list(csv.reader([last_line]))
        if not parsed:
            raise StateError("could not parse latest CSV row")
        values = parsed[0]
        row = dict(zip(self.headers, values))
        return row, last_line, stat.st_mtime


class StateMapper:
    def __init__(self, strict: bool):
        self.strict = strict
        self.last_source_timestamp: Optional[float] = None
        self.last_outgoing_timestamp: Optional[float] = None
        self.last_signature: Optional[str] = None

    def _missing_required(self, name: str, missing: List[str]) -> None:
        missing.append(name)
        if self.strict:
            raise StateError(f"missing required live field: {name}")

    def state_from_csv(self, row: Dict[str, str], signature: str, row_monotonic_time: float) -> SimState:
        timestamp, _ = parse_float(row, (
            "/fdm/jsbsim/simulation/sim-time-sec",
            "jsb_feed_time_s",
            "jsb_time_s",
            "time_s",
            "Time",
        ))
        lat, _ = parse_float(row, (
            "/fdm/jsbsim/position/lat-gc-deg",
            "jsb_feed_lat_deg",
            "jsb_lat_deg",
            "lat_deg",
            "latitude",
        ))
        lon, _ = parse_float(row, (
            "/fdm/jsbsim/position/long-gc-deg",
            "jsb_feed_lon_deg",
            "jsb_lon_deg",
            "lon_deg",
            "longitude",
        ))
        alt_raw, alt_name = parse_float(row, (
            "/fdm/jsbsim/position/h-sl-ft",
            "h-sl-ft",
            "alt_ft",
            "jsb_feed_alt_m",
            "jsb_alt_m",
            "alt_m",
            "altitude",
        ))
        altitude_m = feet_to_meters(alt_raw) if alt_name and alt_name.endswith("ft") else alt_raw

        vn_raw, vn_name = parse_float(row, (
            "/fdm/jsbsim/velocities/v-north-fps",
            "v-north-fps",
            "jsb_feed_vn_mps",
            "jsb_vn_mps",
            "vn_mps",
        ))
        ve_raw, ve_name = parse_float(row, (
            "/fdm/jsbsim/velocities/v-east-fps",
            "v-east-fps",
            "jsb_feed_ve_mps",
            "jsb_ve_mps",
            "ve_mps",
        ))
        vd_raw, vd_name = parse_float(row, (
            "/fdm/jsbsim/velocities/v-down-fps",
            "v-down-fps",
            "jsb_feed_vd_mps",
            "jsb_vd_mps",
            "vd_mps",
        ))
        vn = fps_to_mps(vn_raw) if vn_name and vn_name.endswith("fps") else vn_raw
        ve = fps_to_mps(ve_raw) if ve_name and ve_name.endswith("fps") else ve_raw
        vd = fps_to_mps(vd_raw) if vd_name and vd_name.endswith("fps") else vd_raw

        roll_raw, roll_name = parse_float(row, (
            "/fdm/jsbsim/attitude/phi-deg",
            "phi_deg",
            "roll_deg",
            "jsb_feed_roll_rad",
            "jsb_roll_rad",
            "roll_rad",
        ))
        pitch_raw, pitch_name = parse_float(row, (
            "/fdm/jsbsim/attitude/theta-rad",
            "theta_rad",
            "pitch_rad",
            "jsb_feed_pitch_rad",
            "jsb_pitch_rad",
            "theta_deg",
            "pitch_deg",
        ))
        yaw_raw, yaw_name = parse_float(row, (
            "/fdm/jsbsim/attitude/psi-deg",
            "psi_deg",
            "yaw_deg",
            "jsb_feed_yaw_rad",
            "jsb_yaw_rad",
            "yaw_rad",
        ))
        roll = degrees_to_radians(roll_raw) if roll_name and roll_name.endswith("deg") else roll_raw
        pitch = degrees_to_radians(pitch_raw) if pitch_name and pitch_name.endswith("deg") else pitch_raw
        yaw = normalize_yaw_rad(degrees_to_radians(yaw_raw) if yaw_name and yaw_name.endswith("deg") else yaw_raw)

        tas, tas_name = parse_float(row, (
            "/fdm/jsbsim/velocities/vt-fps",
            "vt-fps",
            "true_airspeed_fps",
            "airspeed_mps",
            "jsb_feed_airspeed_mps",
            "jsb_airspeed_mps",
            "/fdm/jsbsim/velocities/vc-kts",
            "vc-kts",
        ), required=False)
        if tas is None:
            airspeed = math.sqrt(vn * vn + ve * ve + vd * vd)
        elif tas_name and tas_name.endswith("fps"):
            airspeed = fps_to_mps(tas)
        elif tas_name and tas_name.endswith("kts"):
            airspeed = knots_to_mps(tas)
        else:
            airspeed = tas

        missing: List[str] = []
        p, _ = parse_float(row, (
            "p_rad_s",
            "roll_rate_rad_s",
            "jsb_p_rad_s",
            "/fdm/jsbsim/velocities/p-rad_sec",
        ), required=False)
        q, _ = parse_float(row, (
            "q_rad_s",
            "pitch_rate_rad_s",
            "jsb_q_rad_s",
            "/fdm/jsbsim/velocities/q-rad_sec",
        ), required=False)
        r, _ = parse_float(row, (
            "r_rad_s",
            "yaw_rate_rad_s",
            "jsb_r_rad_s",
            "/fdm/jsbsim/velocities/r-rad_sec",
        ), required=False)
        if p is None or q is None or r is None:
            self._missing_required("body gyro p/q/r rad/s", missing)
            p, q, r = 0.0, 0.0, 0.0

        ax, _ = parse_float(row, ("accel_body_x_mss", "ax_body_mss", "jsb_ax_body_mss"), required=False)
        ay, _ = parse_float(row, ("accel_body_y_mss", "ay_body_mss", "jsb_ay_body_mss"), required=False)
        az, _ = parse_float(row, ("accel_body_z_mss", "az_body_mss", "jsb_az_body_mss"), required=False)
        if ax is None or ay is None or az is None:
            self._missing_required("body accel X/Y/Z m/s^2", missing)
            ax, ay, az = 0.0, 0.0, -GRAVITY_MSS

        q1, _ = parse_float(row, ("q1", "quat_w", "quaternion_w"), required=False)
        q2, _ = parse_float(row, ("q2", "quat_x", "quaternion_x"), required=False)
        q3, _ = parse_float(row, ("q3", "quat_y", "quaternion_y"), required=False)
        q4, _ = parse_float(row, ("q4", "quat_z", "quaternion_z"), required=False)
        if None not in (q1, q2, q3, q4):
            quat = normalize_quaternion((q1, q2, q3, q4))
        else:
            quat = euler_to_quaternion(roll, pitch, yaw)

        if self.last_source_timestamp is not None and timestamp < self.last_source_timestamp:
            raise StateError(f"backward source timestamp: {timestamp} < {self.last_source_timestamp}")

        reused = self.last_signature == signature
        outgoing_timestamp = timestamp
        if self.last_outgoing_timestamp is not None and outgoing_timestamp <= self.last_outgoing_timestamp:
            outgoing_timestamp = self.last_outgoing_timestamp + 1.0e-6

        state = SimState(
            timestamp_s=outgoing_timestamp,
            source_timestamp_s=timestamp,
            latitude_deg=lat,
            longitude_deg=lon,
            altitude_m=altitude_m,
            roll_rad=roll,
            pitch_rad=pitch,
            yaw_rad=yaw,
            quaternion=quat,
            gyro_rad_s=(p, q, r),
            accel_body_mss=(ax, ay, az),
            velocity_ned_mps=(vn, ve, vd),
            airspeed_mps=airspeed,
            row_signature=signature,
            row_monotonic_time=row_monotonic_time,
            reused_source_row=reused,
            missing_fields=tuple(missing),
        )
        state.validate()
        return state

    def accept_state(self, state: SimState) -> None:
        self.last_signature = state.row_signature
        self.last_source_timestamp = state.source_timestamp_s
        self.last_outgoing_timestamp = state.timestamp_s


class MockStateSource:
    def __init__(self):
        self.start_mono = time.monotonic()

    def state(self) -> SimState:
        timestamp = time.monotonic() - self.start_mono
        roll = 0.0
        pitch = 0.0
        yaw = 0.0
        return SimState(
            timestamp_s=timestamp,
            source_timestamp_s=timestamp,
            latitude_deg=32.5378885,
            longitude_deg=74.3661944,
            altitude_m=240.2,
            roll_rad=roll,
            pitch_rad=pitch,
            yaw_rad=yaw,
            quaternion=euler_to_quaternion(roll, pitch, yaw),
            gyro_rad_s=(0.0, 0.0, 0.0),
            accel_body_mss=(0.0, 0.0, -GRAVITY_MSS),
            velocity_ned_mps=(0.0, 0.0, 0.0),
            airspeed_mps=0.0,
            row_signature=f"mock:{timestamp:.9f}",
            row_monotonic_time=time.monotonic(),
        )


def decode_control_packet(data: bytes) -> ControlPacket:
    if len(data) == SERVO16_STRUCT.size:
        values = SERVO16_STRUCT.unpack(data)
        magic = values[0]
        if magic != SERVO16_MAGIC:
            raise ValueError(f"bad 16-channel magic {magic}")
        return ControlPacket(magic, values[1], values[2], tuple(values[3:]))
    if len(data) == SERVO32_STRUCT.size:
        values = SERVO32_STRUCT.unpack(data)
        magic = values[0]
        if magic != SERVO32_MAGIC:
            raise ValueError(f"bad 32-channel magic {magic}")
        return ControlPacket(magic, values[1], values[2], tuple(values[3:]))
    raise ValueError(f"wrong packet length {len(data)}")


def open_log(path: Optional[str]):
    if not path:
        return None, None
    csv_file = open(path, "w", newline="", encoding="utf-8")
    writer = csv.DictWriter(csv_file, fieldnames=LOG_FIELDS)
    writer.writeheader()
    csv_file.flush()
    return csv_file, writer


def write_log(writer, csv_file, row: Dict[str, object]) -> None:
    if writer is None:
        return
    writer.writerow({field: row.get(field, "") for field in LOG_FIELDS})
    csv_file.flush()


def make_log_row(
    request_count: int,
    source: Tuple[str, int],
    packet_valid: bool,
    started: float,
    state: Optional[SimState] = None,
    reply_bytes: int = 0,
    error_reason: str = "",
    actuator_command: Optional[NormalizedActuatorCommand] = None,
    control_packet: Optional[ControlPacket] = None,
) -> Dict[str, object]:
    now = time.monotonic()
    row: Dict[str, object] = {
        "host_monotonic_time": f"{now:.9f}",
        "request_count": request_count,
        "frame_rate": "" if control_packet is None else control_packet.frame_rate,
        "frame_count": "" if control_packet is None else control_packet.frame_count,
        "source_ip": source[0],
        "source_port": source[1],
        "packet_valid": int(packet_valid),
        "state_age_ms": "" if state is None else f"{(now - state.row_monotonic_time) * 1000.0:.3f}",
        "simulation_timestamp": "" if state is None else f"{state.timestamp_s:.9f}",
        "latitude": "" if state is None else f"{state.latitude_deg:.9f}",
        "longitude": "" if state is None else f"{state.longitude_deg:.9f}",
        "altitude": "" if state is None else f"{state.altitude_m:.3f}",
        "roll": "" if state is None else f"{state.roll_rad:.9f}",
        "pitch": "" if state is None else f"{state.pitch_rad:.9f}",
        "yaw": "" if state is None else f"{state.yaw_rad:.9f}",
        "vn": "" if state is None else f"{state.velocity_ned_mps[0]:.6f}",
        "ve": "" if state is None else f"{state.velocity_ned_mps[1]:.6f}",
        "vd": "" if state is None else f"{state.velocity_ned_mps[2]:.6f}",
        "airspeed": "" if state is None else f"{state.airspeed_mps:.6f}",
        "reply_bytes": reply_bytes,
        "round_trip_or_processing_us": int((now - started) * 1000000.0),
        "error_reason": error_reason,
    }
    if actuator_command is not None:
        row.update(actuator_command.log_fields())
    return row


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="SR-75 host-side SIM_JSON UDP responder")
    parser.add_argument("--listen-host", default="0.0.0.0")
    parser.add_argument("--listen-port", type=int, default=9002)
    parser.add_argument("--state-file", default=DEFAULT_STATE_FILE)
    parser.add_argument("--rate-limit-hz", type=float, default=0.0, help="Maximum reply rate; 0 disables limiting")
    parser.add_argument("--state-timeout-ms", type=float, default=500.0)
    parser.add_argument("--verbose", action="store_true")
    parser.add_argument("--dry-run", action="store_true", help="Decode requests but do not send replies")
    parser.add_argument("--log-csv", default=None)
    parser.add_argument("--once", action="store_true")
    parser.add_argument("--mock-state", action="store_true")
    parser.add_argument("--strict", action="store_true")
    parser.add_argument("--actuator-map", default=None, help="JSON channel map for software-only actuator decoding")
    parser.add_argument("--actuator-timeout-ms", type=float, default=None, help="Override actuator stale-command timeout")
    parser.add_argument("--actuator-surface-deadband", type=float, default=None, help="Override surface command deadband")
    parser.add_argument(
        "--actuator-throttle-deadband",
        type=float,
        default=None,
        help="Override throttle/RATO command deadband",
    )
    parser.add_argument(
        "--jsbsim-command-target",
        default=None,
        help="Opt-in UDP JSBSim command target, for example udp:127.0.0.1:5600",
    )
    return parser


def build_actuator_channel_map(args: argparse.Namespace) -> ActuatorChannelMap:
    channel_map = ActuatorChannelMap.from_json_file(args.actuator_map)
    timeout_ms = channel_map.timeout_ms if args.actuator_timeout_ms is None else args.actuator_timeout_ms
    surface_deadband = (
        channel_map.surface_deadband if args.actuator_surface_deadband is None else args.actuator_surface_deadband
    )
    throttle_deadband = (
        channel_map.throttle_deadband if args.actuator_throttle_deadband is None else args.actuator_throttle_deadband
    )
    return ActuatorChannelMap(
        channels=dict(channel_map.channels),
        timeout_ms=timeout_ms,
        surface_deadband=surface_deadband,
        throttle_deadband=throttle_deadband,
    )


def jsbsim_command_from_actuator(
    actuator_command: NormalizedActuatorCommand,
    timestamp_s: Optional[float] = None,
) -> JSBSimActuatorCommand:
    timestamp = time.monotonic() if timestamp_s is None else timestamp_s
    return JSBSimActuatorCommand(
        timestamp_s=timestamp,
        elevator=actuator_command.elevator,
        aileron=actuator_command.aileron,
        rudder=actuator_command.rudder,
        turbojet_throttle=actuator_command.turbojet_throttle,
        rato_throttle=actuator_command.rato,
        stale=actuator_command.stale,
    )


def main() -> int:
    args = build_arg_parser().parse_args()
    try:
        actuator_channel_map = build_actuator_channel_map(args)
    except ActuatorMapError as exc:
        print(f"ERROR: actuator channel map invalid: {exc}", file=sys.stderr)
        return 2
    command_sink = None
    if args.jsbsim_command_target:
        try:
            command_sink = UDPJSBSimCommandSink(args.jsbsim_command_target)
        except JSBSimCommandError as exc:
            print(f"ERROR: JSBSim command target invalid: {exc}", file=sys.stderr)
            return 2

    print("SR75 SIM_JSON RESPONDER")
    if command_sink is None:
        print("ACTUATOR OUTPUT: DISABLED")
    else:
        print(f"ACTUATOR OUTPUT: SOFTWARE_JSBSIM_ONLY target={args.jsbsim_command_target}")
    print("SERIAL OUTPUT: DISABLED")
    print("RATO/ENGINE/RELAY OUTPUT: DISABLED")
    mode = "LOG_ONLY" if command_sink is None else "JSBSIM_COMMAND"
    print(f"ACTUATOR DECODE: {mode} {describe_channel_map(actuator_channel_map)}")

    csv_file, log_writer = open_log(args.log_csv)
    reader = LatestCSVReader(args.state_file)
    mapper = StateMapper(strict=args.strict)
    mock_source = MockStateSource()
    actuator_bridge = SoftwareActuatorBridge(actuator_channel_map)
    min_interval = 1.0 / args.rate_limit_hz if args.rate_limit_hz and args.rate_limit_hz > 0.0 else 0.0
    last_reply_mono = 0.0
    request_count = 0

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind((args.listen_host, args.listen_port))
    except OSError as exc:
        print(f"ERROR: bind failed on {args.listen_host}:{args.listen_port}: {exc}", file=sys.stderr)
        if command_sink is not None:
            command_sink.close()
        return 2
    if command_sink is not None:
        timeout_s = min(0.05, max(0.001, actuator_channel_map.timeout_ms / 2000.0))
        sock.settimeout(timeout_s)
    print(f"Listening on {args.listen_host}:{args.listen_port}")
    print(f"State source: {'mock-state' if args.mock_state else args.state_file}")

    try:
        while True:
            started = time.monotonic()
            try:
                data, source = sock.recvfrom(4096)
            except socket.timeout:
                if command_sink is not None:
                    actuator_command = actuator_bridge.current_command(started)
                    if actuator_command.stale:
                        sent = command_sink.send_stale_once(timestamp_s=started)
                        if args.verbose and sent is not None:
                            print("JSBSIM_COMMAND stale=1 elev=0.000 ail=0.000 rud=0.000 thr=0.000 rato=0.000")
                continue
            request_count += 1
            try:
                packet = decode_control_packet(data)
            except ValueError as exc:
                reason = f"MALFORMED_PACKET: {exc}"
                print(reason)
                write_log(log_writer, csv_file, make_log_row(request_count, source, False, started, error_reason=reason))
                if args.once:
                    return 1
                continue

            actuator_reason = ""
            try:
                actuator_command = actuator_bridge.update_from_pwm(packet.pwm, started)
            except ActuatorMapError as exc:
                actuator_reason = f"ACTUATOR_MAP_ERROR: {exc}"
                print(actuator_reason)
                actuator_command = actuator_bridge.current_command(started)

            if command_sink is not None:
                try:
                    command_sink.send(jsbsim_command_from_actuator(actuator_command, timestamp_s=started))
                    if args.verbose:
                        print(
                            "JSBSIM_COMMAND "
                            f"stale={int(actuator_command.stale)} "
                            f"elev={actuator_command.elevator:.3f} ail={actuator_command.aileron:.3f} "
                            f"rud={actuator_command.rudder:.3f} "
                            f"thr={actuator_command.turbojet_throttle:.3f} rato={actuator_command.rato:.3f}"
                        )
                except JSBSimCommandError as exc:
                    actuator_reason = f"JSBSIM_COMMAND_ERROR: {exc}"
                    print(actuator_reason)

            if args.verbose:
                print(
                    f"REQ {request_count} from {source[0]}:{source[1]} "
                    f"magic={packet.magic} frame_rate={packet.frame_rate} frame={packet.frame_count} "
                    f"pwm0={packet.pwm[0] if packet.pwm else ''}"
                )
                warnings = f" warnings={';'.join(actuator_command.warnings)}" if actuator_command.warnings else ""
                print(
                    "ACTUATOR_LOG_ONLY "
                    f"left={actuator_command.left_elevon:.3f} right={actuator_command.right_elevon:.3f} "
                    f"elev={actuator_command.elevator:.3f} ail={actuator_command.aileron:.3f} "
                    f"rud={actuator_command.rudder:.3f} "
                    f"thr_l={actuator_command.throttle_left:.3f} thr_r={actuator_command.throttle_right:.3f} "
                    f"rato={actuator_command.rato:.3f} stale={int(actuator_command.stale)}{warnings}"
                )

            now = time.monotonic()
            if min_interval > 0.0 and now - last_reply_mono < min_interval:
                reason = "RATE_LIMIT"
                write_log(
                    log_writer,
                    csv_file,
                    make_log_row(
                        request_count,
                        source,
                        True,
                        started,
                        error_reason=reason,
                        actuator_command=actuator_command,
                        control_packet=packet,
                    ),
                )
                if args.verbose:
                    print(reason)
                continue

            try:
                if args.mock_state:
                    state = mock_source.state()
                else:
                    row, signature, mtime = reader.read_latest()
                    row_age_ms = (time.time() - mtime) * 1000.0
                    if row_age_ms > args.state_timeout_ms:
                        raise StateError(f"STALE_STATE: latest row age {row_age_ms:.1f} ms")
                    state = mapper.state_from_csv(row, signature, time.monotonic() - (row_age_ms / 1000.0))
                payload = state.to_json_bytes()
            except StateError as exc:
                reason = str(exc)
                if reason.startswith("STALE_STATE"):
                    print(reason)
                else:
                    print(f"STATE_ERROR: {reason}")
                if actuator_reason:
                    reason = f"{reason};{actuator_reason}"
                write_log(
                    log_writer,
                    csv_file,
                    make_log_row(
                        request_count,
                        source,
                        True,
                        started,
                        error_reason=reason,
                        actuator_command=actuator_command,
                        control_packet=packet,
                    ),
                )
                if args.once:
                    return 1
                continue

            reply_bytes = 0
            if args.dry_run:
                reason = "DRY_RUN"
                if args.verbose:
                    print(f"DRY_RUN would send {len(payload)} bytes to {source[0]}:{source[1]}")
            else:
                reply_bytes = sock.sendto(payload, source)
                last_reply_mono = time.monotonic()
                reason = actuator_reason
                if not args.mock_state:
                    mapper.accept_state(state)
                if args.verbose:
                    missing = f" missing={','.join(state.missing_fields)}" if state.missing_fields else ""
                    reused = " REUSED_STATE" if state.reused_source_row else ""
                    print(
                        f"REPLY {reply_bytes} bytes t={state.timestamp_s:.6f} "
                        f"lat={state.latitude_deg:.7f} lon={state.longitude_deg:.7f} "
                        f"rpy=({state.roll_rad:.5f},{state.pitch_rad:.5f},{state.yaw_rad:.5f}){missing}{reused}"
                    )

            write_log(
                log_writer,
                csv_file,
                make_log_row(
                    request_count,
                    source,
                    True,
                    started,
                    state,
                    reply_bytes,
                    reason,
                    actuator_command,
                    packet,
                ),
            )
            if args.once:
                return 0
    except KeyboardInterrupt:
        print("Interrupted")
        return 0
    finally:
        sock.close()
        if command_sink is not None:
            command_sink.close()
        if csv_file is not None:
            csv_file.close()


if __name__ == "__main__":
    sys.exit(main())
