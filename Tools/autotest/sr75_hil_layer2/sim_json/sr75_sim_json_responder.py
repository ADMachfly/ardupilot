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
from dataclasses import dataclass, replace
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
MIN_OUTGOING_TIMESTAMP_STEP_S = 1.0e-6
FT_TO_M = 0.3048
FPS_TO_MPS = 0.3048
KTS_TO_MPS = 0.514444
SR75_DRY_BOOSTER_WEIGHT_LBS = 22.046226

SERVO16_MAGIC = 18458
SERVO32_MAGIC = 29569
SERVO16_STRUCT = struct.Struct("<HHI16H")
SERVO32_STRUCT = struct.Struct("<HHI32H")

LOG_FIELDS = [
    "host_monotonic_time",
    "request_count",
    "sim_json_request_sequence",
    "request_received_host_time",
    "previous_request_received_host_time",
    "request_host_dt",
    "pwm_frame_received_host_time",
    "startup_release_host_time",
    "reply_send_host_time",
    "state_csv_source_timestamp",
    "frame_rate",
    "frame_count",
    "source_ip",
    "source_port",
    "packet_valid",
    "state_age_ms",
    "simulation_timestamp",
    "source_simulation_timestamp",
    "previous_sent_timestamp",
    "state_reused",
    "reply_sent",
    "reply_reason",
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
    "act_eject_norm",
    "act_stale",
    "act_warnings",
    "pwm1",
    "pwm2",
    "pwm3",
    "pwm4",
    "pwm5",
    "pwm6",
    "pwm7",
    "pwm8",
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
    "jsbsim_dry_booster_property",
    "jsbsim_dry_booster_value",
    "control_phase",
    "precontrol_active",
    "first_valid_pwm_seen",
    "handover_count",
    "command_source",
    "elevator_cmd",
    "aileron_cmd",
    "rudder_cmd",
    "turbojet_throttle_cmd",
    "rato_cmd",
    "eject_cmd",
    "p_rad_s",
    "q_rad_s",
    "r_rad_s",
    "startup_sync_phase",
    "startup_sync_readiness_reason",
    "startup_sync_valid_pwm_frames",
    "startup_sync_fbwa_confirmed",
    "startup_sync_ahrs_ekf_type",
    "control_ready_timestamp",
    "release_timestamp",
    "first_post_release_state_timestamp",
    "scoring_start_timestamp",
    "timing_host_dt",
    "timing_source_simulation_dt",
    "timing_outgoing_simulation_dt",
    "timing_source_over_host_ratio",
    "timing_gate_state",
    "timing_baseline_row",
]


class StateError(Exception):
    """Raised when state cannot be converted to a valid SIM_JSON reply."""


class StateEnvelopeError(Exception):
    """Raised when B3 bounded-test state exceeds the allowed safety envelope."""

    def __init__(self, offenders: Sequence[str]):
        self.offenders = tuple(offenders)
        super().__init__(";".join(self.offenders))


class TimeDiscontinuityError(Exception):
    """Raised when consecutive scoring-window rows fail the B3 timing-quality gate."""

    def __init__(self, offenders: Sequence[str], previous: Dict[str, object], current: Dict[str, object]):
        self.offenders = tuple(offenders)
        self.previous = dict(previous)
        self.current = dict(current)
        super().__init__(";".join(self.offenders))


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


def gravity_body_mss(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float]:
    """Return the stationary gravity vector resolved into SIM_JSON body axes."""
    del yaw
    sr = math.sin(roll)
    cr = math.cos(roll)
    sp = math.sin(pitch)
    cp = math.cos(pitch)
    return (
        GRAVITY_MSS * sp,
        -GRAVITY_MSS * sr * cp,
        -GRAVITY_MSS * cr * cp,
    )


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
    source_row: Optional[Dict[str, str]] = None

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

    def to_json_bytes(self, rc_pwm: Optional[Sequence[int]] = None) -> bytes:
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
        if rc_pwm is not None:
            payload["rc"] = {
                f"rc_{index + 1}": int(value)
                for index, value in enumerate(rc_pwm)
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
        for line in reversed(lines[1:]):
            parsed = list(csv.reader([line]))
            if not parsed:
                continue
            values = parsed[0]
            if len(values) == len(self.headers):
                row = dict(zip(self.headers, values))
                return row, line, stat.st_mtime
        raise StateError(f"{self.path} has no complete data rows")


class StateMapper:
    def __init__(self, strict: bool, rebase_time: bool = True):
        self.strict = strict
        self.rebase_time = rebase_time
        self.source_time_zero: Optional[float] = None
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
            ax, ay, az = gravity_body_mss(roll, pitch, yaw)

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
        if self.rebase_time:
            if self.source_time_zero is None:
                self.source_time_zero = timestamp
            outgoing_timestamp = timestamp - self.source_time_zero + MIN_OUTGOING_TIMESTAMP_STEP_S
        else:
            outgoing_timestamp = timestamp
        if self.last_outgoing_timestamp is not None and outgoing_timestamp <= self.last_outgoing_timestamp:
            outgoing_timestamp = self.last_outgoing_timestamp + MIN_OUTGOING_TIMESTAMP_STEP_S
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
            source_row=dict(row),
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
    previous_sent_timestamp: Optional[float] = None,
    reply_reason: str = "",
    actuator_command: Optional[NormalizedActuatorCommand] = None,
    control_packet: Optional[ControlPacket] = None,
    command_source: str = "",
    precontrol: Optional["B3PrecontrolHandover"] = None,
    startup_sync: Optional["B3StartupSync"] = None,
    timing_quality: Optional[Tuple[float, float, float, Optional[float]]] = None,
    timing_gate: Optional["B3TimingQualityGate"] = None,
    previous_request_received_host_time: Optional[float] = None,
    request_received_host_time: Optional[float] = None,
    pwm_frame_received_host_time: Optional[float] = None,
    reply_send_host_time: Optional[float] = None,
    dry_booster_attached: bool = True,
) -> Dict[str, object]:
    now = time.monotonic()
    request_host_dt = (
        None if previous_request_received_host_time is None
        else started - previous_request_received_host_time
    )
    received_host_time = started if request_received_host_time is None else request_received_host_time
    row: Dict[str, object] = {
        "host_monotonic_time": f"{now:.9f}",
        "request_count": request_count,
        "sim_json_request_sequence": request_count,
        "request_received_host_time": f"{received_host_time:.9f}",
        "previous_request_received_host_time": (
            "" if previous_request_received_host_time is None
            else f"{previous_request_received_host_time:.9f}"
        ),
        "request_host_dt": "" if request_host_dt is None else f"{request_host_dt:.9f}",
        "pwm_frame_received_host_time": (
            "" if pwm_frame_received_host_time is None
            else f"{pwm_frame_received_host_time:.9f}"
        ),
        "startup_release_host_time": (
            "" if startup_sync is None or startup_sync.release_record is None
            else f"{startup_sync.release_record['host_time']:.9f}"
        ),
        "reply_send_host_time": "" if reply_send_host_time is None else f"{reply_send_host_time:.9f}",
        "state_csv_source_timestamp": "" if state is None else f"{state.source_timestamp_s:.9f}",
        "frame_rate": "" if control_packet is None else control_packet.frame_rate,
        "frame_count": "" if control_packet is None else control_packet.frame_count,
        "source_ip": source[0],
        "source_port": source[1],
        "packet_valid": int(packet_valid),
        "state_age_ms": "" if state is None else f"{(now - state.row_monotonic_time) * 1000.0:.3f}",
        "simulation_timestamp": "" if state is None else f"{state.timestamp_s:.9f}",
        "source_simulation_timestamp": "" if state is None else f"{state.source_timestamp_s:.9f}",
        "previous_sent_timestamp": "" if previous_sent_timestamp is None else f"{previous_sent_timestamp:.9f}",
        "state_reused": "" if state is None else int(state.reused_source_row),
        "reply_sent": int(reply_bytes > 0),
        "reply_reason": reply_reason,
        "latitude": "" if state is None else f"{state.latitude_deg:.9f}",
        "longitude": "" if state is None else f"{state.longitude_deg:.9f}",
        "altitude": "" if state is None else f"{state.altitude_m:.3f}",
        "roll": "" if state is None else f"{state.roll_rad:.9f}",
        "pitch": "" if state is None else f"{state.pitch_rad:.9f}",
        "yaw": "" if state is None else f"{state.yaw_rad:.9f}",
        "p_rad_s": "" if state is None else f"{state.gyro_rad_s[0]:.9f}",
        "q_rad_s": "" if state is None else f"{state.gyro_rad_s[1]:.9f}",
        "r_rad_s": "" if state is None else f"{state.gyro_rad_s[2]:.9f}",
        "vn": "" if state is None else f"{state.velocity_ned_mps[0]:.6f}",
        "ve": "" if state is None else f"{state.velocity_ned_mps[1]:.6f}",
        "vd": "" if state is None else f"{state.velocity_ned_mps[2]:.6f}",
        "airspeed": "" if state is None else f"{state.airspeed_mps:.6f}",
        "reply_bytes": reply_bytes,
        "round_trip_or_processing_us": int((now - started) * 1000000.0),
        "error_reason": error_reason,
        "jsbsim_dry_booster_property": "propulsion/tank[2]/contents-lbs",
        "jsbsim_dry_booster_value": f"{SR75_DRY_BOOSTER_WEIGHT_LBS if dry_booster_attached else 0.0:.6f}",
    }
    if actuator_command is not None:
        row.update(actuator_command.log_fields())
        row["elevator_cmd"] = f"{actuator_command.elevator:.6f}"
        row["aileron_cmd"] = f"{actuator_command.aileron:.6f}"
        row["rudder_cmd"] = f"{actuator_command.rudder:.6f}"
        row["turbojet_throttle_cmd"] = f"{actuator_command.turbojet_throttle:.6f}"
        row["rato_cmd"] = f"{actuator_command.rato:.6f}"
        row["eject_cmd"] = f"{actuator_command.eject:.6f}"
    if control_packet is not None:
        for index, pwm in enumerate(control_packet.pwm[:8], start=1):
            row[f"pwm{index}"] = pwm
    row["command_source"] = command_source
    if precontrol is not None:
        row["control_phase"] = precontrol.phase
        row["precontrol_active"] = int(precontrol.enabled)
        row["first_valid_pwm_seen"] = int(precontrol.first_valid_pwm_seen)
        row["handover_count"] = precontrol.handover_count
    else:
        row["control_phase"] = ""
        row["precontrol_active"] = 0
        row["first_valid_pwm_seen"] = ""
        row["handover_count"] = 0
    if startup_sync is not None:
        row["startup_sync_phase"] = startup_sync.phase
        row["startup_sync_readiness_reason"] = startup_sync.readiness_reason
        row["startup_sync_valid_pwm_frames"] = startup_sync.valid_pwm_frames
        row["startup_sync_fbwa_confirmed"] = int(startup_sync.readiness.fbwa_confirmed)
        row["startup_sync_ahrs_ekf_type"] = (
            "" if startup_sync.readiness.ahrs_ekf_type is None else startup_sync.readiness.ahrs_ekf_type
        )
        row["control_ready_timestamp"] = (
            "" if startup_sync.control_ready_record is None
            else f"{startup_sync.control_ready_record['host_time']:.9f}"
        )
        row["release_timestamp"] = (
            "" if startup_sync.release_record is None
            else f"{startup_sync.release_record['host_time']:.9f}"
        )
        row["first_post_release_state_timestamp"] = (
            "" if startup_sync.first_post_release_state_record is None
            else f"{startup_sync.first_post_release_state_record['simulation_timestamp']:.9f}"
        )
        row["scoring_start_timestamp"] = (
            "" if startup_sync.scoring_start_record is None
            else f"{startup_sync.scoring_start_record['simulation_timestamp']:.9f}"
        )
    else:
        row["startup_sync_phase"] = ""
        row["startup_sync_readiness_reason"] = ""
        row["startup_sync_valid_pwm_frames"] = ""
        row["startup_sync_fbwa_confirmed"] = ""
        row["startup_sync_ahrs_ekf_type"] = ""
        row["control_ready_timestamp"] = ""
        row["release_timestamp"] = ""
        row["first_post_release_state_timestamp"] = ""
        row["scoring_start_timestamp"] = ""
    if timing_quality is not None:
        host_dt, source_dt, outgoing_dt, ratio = timing_quality
        row["timing_host_dt"] = f"{host_dt:.9f}"
        row["timing_source_simulation_dt"] = f"{source_dt:.9f}"
        row["timing_outgoing_simulation_dt"] = f"{outgoing_dt:.9f}"
        row["timing_source_over_host_ratio"] = "" if ratio is None else f"{ratio:.6f}"
    else:
        row["timing_host_dt"] = ""
        row["timing_source_simulation_dt"] = ""
        row["timing_outgoing_simulation_dt"] = ""
        row["timing_source_over_host_ratio"] = ""
    if timing_gate is not None:
        row["timing_gate_state"] = timing_gate.state
        row["timing_baseline_row"] = timing_gate.baseline_label_for_request(request_count)
    else:
        row["timing_gate_state"] = ""
        row["timing_baseline_row"] = ""
    return row


B3_ENVELOPE_LIMITS = {
    "altitude_m": (-100.0, 10000.0),
    "airspeed_mps": (0.0, 250.0),
    "velocity_ned_mps_abs": 300.0,
    "velocity_total_mps": 350.0,
    "gyro_rad_s_abs": 20.0,
    "accel_body_mss_abs": 200.0,
    "quaternion_norm": (0.99, 1.01),
}


def b3_state_envelope_offenders(
    state: SimState,
    *,
    airspeed_max_mps: Optional[float] = None,
    airspeed_limit_name: str = "B3",
) -> Tuple[str, ...]:
    checks = {
        "timestamp_s": state.timestamp_s,
        "source_timestamp_s": state.source_timestamp_s,
        "latitude_deg": state.latitude_deg,
        "longitude_deg": state.longitude_deg,
        "altitude_m": state.altitude_m,
        "roll_rad": state.roll_rad,
        "pitch_rad": state.pitch_rad,
        "yaw_rad": state.yaw_rad,
        "airspeed_mps": state.airspeed_mps,
        "gyro_p_rad_s": state.gyro_rad_s[0],
        "gyro_q_rad_s": state.gyro_rad_s[1],
        "gyro_r_rad_s": state.gyro_rad_s[2],
        "accel_body_x_mss": state.accel_body_mss[0],
        "accel_body_y_mss": state.accel_body_mss[1],
        "accel_body_z_mss": state.accel_body_mss[2],
        "velocity_n_mps": state.velocity_ned_mps[0],
        "velocity_e_mps": state.velocity_ned_mps[1],
        "velocity_d_mps": state.velocity_ned_mps[2],
        "q1": state.quaternion[0],
        "q2": state.quaternion[1],
        "q3": state.quaternion[2],
        "q4": state.quaternion[3],
    }
    offenders = [
        f"{name}=nonfinite:{value}"
        for name, value in checks.items()
        if not math.isfinite(value)
    ]
    altitude_min, altitude_max = B3_ENVELOPE_LIMITS["altitude_m"]
    if not altitude_min <= state.altitude_m <= altitude_max:
        offenders.append(f"altitude_m={state.altitude_m:.6g}:outside[{altitude_min},{altitude_max}]")
    tas_min, b3_tas_max = B3_ENVELOPE_LIMITS["airspeed_mps"]
    tas_max = b3_tas_max if airspeed_max_mps is None else airspeed_max_mps
    if not tas_min <= state.airspeed_mps <= tas_max:
        offenders.append(f"airspeed_mps={state.airspeed_mps:.6g}:outside[{tas_min},{tas_max}]")
    elif tas_max > b3_tas_max and state.airspeed_mps > b3_tas_max:
        print(
            f"B3_STATE_ENVELOPE_{airspeed_limit_name}_AIRSPEED_EXCEEDANCE "
            f"airspeed_mps={state.airspeed_mps:.6g}:outside[{tas_min},{b3_tas_max}] "
            f"validation_limit={tas_max}"
        )
    velocity_axis_limit = B3_ENVELOPE_LIMITS["velocity_ned_mps_abs"]
    for axis, value in zip(("n", "e", "d"), state.velocity_ned_mps):
        if abs(value) >= velocity_axis_limit:
            offenders.append(f"velocity_{axis}_mps={value:.6g}:abs>={velocity_axis_limit}")
    velocity_total = math.sqrt(sum(value * value for value in state.velocity_ned_mps))
    velocity_total_limit = B3_ENVELOPE_LIMITS["velocity_total_mps"]
    if velocity_total >= velocity_total_limit:
        offenders.append(f"velocity_total_mps={velocity_total:.6g}:>={velocity_total_limit}")
    gyro_limit = B3_ENVELOPE_LIMITS["gyro_rad_s_abs"]
    for axis, value in zip(("p", "q", "r"), state.gyro_rad_s):
        if abs(value) >= gyro_limit:
            offenders.append(f"gyro_{axis}_rad_s={value:.6g}:abs>={gyro_limit}")
    accel_limit = B3_ENVELOPE_LIMITS["accel_body_mss_abs"]
    for axis, value in zip(("x", "y", "z"), state.accel_body_mss):
        if abs(value) >= accel_limit:
            offenders.append(f"accel_body_{axis}_mss={value:.6g}:abs>={accel_limit}")
    quat_norm = math.sqrt(sum(value * value for value in state.quaternion))
    quat_min, quat_max = B3_ENVELOPE_LIMITS["quaternion_norm"]
    if not quat_min <= quat_norm <= quat_max:
        offenders.append(f"quaternion_norm={quat_norm:.9f}:outside[{quat_min},{quat_max}]")
    return tuple(offenders)


def validate_b3_state_envelope(
    state: SimState,
    *,
    airspeed_max_mps: Optional[float] = None,
    airspeed_limit_name: str = "B3",
) -> None:
    offenders = b3_state_envelope_offenders(
        state,
        airspeed_max_mps=airspeed_max_mps,
        airspeed_limit_name=airspeed_limit_name,
    )
    if offenders:
        raise StateEnvelopeError(offenders)


def b3_rato_thrust_newtons(state: SimState) -> Optional[float]:
    if state.source_row is None:
        return None
    thrust_lbf, thrust_name = parse_float(state.source_row, (
        "/fdm/jsbsim/propulsion/engine[2]/thrust-lbs",
        "propulsion/engine[2]/thrust-lbs",
        "rato_thrust_lbs",
        "rato_thrust_lbf",
        "rato_thrust_n",
    ), required=False)
    if thrust_lbf is None:
        return None
    if thrust_name and thrust_name.endswith("_n"):
        return thrust_lbf
    return thrust_lbf * 4.4482216152605


def write_b3_state_envelope_abort(path: Optional[str], state: SimState, offenders: Sequence[str]) -> None:
    if not path:
        return
    directory = os.path.dirname(path)
    if directory:
        os.makedirs(directory, exist_ok=True)
    row = dict(state.source_row or {})
    row.update({
        "b3_abort_source_timestamp_s": f"{state.source_timestamp_s:.9f}",
        "b3_abort_outgoing_timestamp_s": f"{state.timestamp_s:.9f}",
        "b3_abort_offenders": ";".join(offenders),
    })
    fieldnames = list(row)
    with open(path, "w", newline="", encoding="utf-8") as abort_file:
        writer = csv.DictWriter(abort_file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerow(row)


B3_TIME_DISCONTINUITY_LIMITS = {
    "source_simulation_dt_max_s": 0.05,
    "outgoing_simulation_dt_max_s": 0.05,
    "source_over_host_ratio_max": 3.0,
}


def b3_time_discontinuity_offenders(
    host_dt: float, source_simulation_dt: float, outgoing_simulation_dt: float
) -> Tuple[str, ...]:
    offenders = []
    if host_dt <= 0.0:
        offenders.append(f"host_dt={host_dt:.9f}<=0")
    if source_simulation_dt <= 0.0:
        offenders.append(f"source_simulation_dt={source_simulation_dt:.9f}<=0")
    source_max = B3_TIME_DISCONTINUITY_LIMITS["source_simulation_dt_max_s"]
    if source_simulation_dt > source_max:
        offenders.append(f"source_simulation_dt={source_simulation_dt:.9f}>{source_max}")
    if outgoing_simulation_dt <= 0.0:
        offenders.append(f"outgoing_simulation_dt={outgoing_simulation_dt:.9f}<=0")
    outgoing_max = B3_TIME_DISCONTINUITY_LIMITS["outgoing_simulation_dt_max_s"]
    if outgoing_simulation_dt > outgoing_max:
        offenders.append(f"outgoing_simulation_dt={outgoing_simulation_dt:.9f}>{outgoing_max}")
    if host_dt > 0.0:
        ratio = source_simulation_dt / host_dt
        ratio_max = B3_TIME_DISCONTINUITY_LIMITS["source_over_host_ratio_max"]
        if ratio > ratio_max:
            offenders.append(f"source_simulation_dt/host_dt={ratio:.6f}>{ratio_max}")
    return tuple(offenders)


class B3TimingQualityGate:
    """SR-75 Layer 2J-B3C-C3 scoring-window timing-quality gate.

    The startup-sync handoff is phase-aware: held synthetic rows are not
    compared against the first real post-release row. On release, the gate
    enters WAIT_FIRST_REAL_ROW; that first real row is labeled
    TIMING_BASELINE, then later real rows are checked pairwise.

    Scoring-start uses option A from the B3C-C3A request: continue the
    already-active real-row sequence. The first real post-release row is the
    timing baseline, so the second real row can both become scoring_start and
    be protected by the same cadence gate.
    """

    def __init__(self, enabled: bool):
        self.enabled = enabled
        self.state = B3TimingGateState.ACTIVE if enabled else B3TimingGateState.INACTIVE
        self.previous: Optional[Dict[str, object]] = None
        self.final_held_outgoing_timestamp: Optional[float] = None
        self.release_count: Optional[int] = None
        self.release_host_time: Optional[float] = None
        self.last_baseline_request_count: Optional[int] = None

    def on_startup_sync_release(
        self,
        final_held_outgoing_timestamp: Optional[float],
        release_host_time: float,
        release_count: int,
    ) -> None:
        """Reset checked timestamps at STARTUP_SYNC_RELEASE."""
        if not self.enabled:
            return
        self.previous = None
        self.final_held_outgoing_timestamp = final_held_outgoing_timestamp
        self.release_host_time = release_host_time
        self.release_count = release_count
        self.last_baseline_request_count = None
        self.state = B3TimingGateState.WAIT_FIRST_REAL_ROW

    def baseline_label_for_request(self, request_count: int) -> str:
        if not self.enabled:
            return ""
        return "TIMING_BASELINE" if self.last_baseline_request_count == request_count else ""

    def _release_boundary_offenders(
        self,
        source_simulation_timestamp: float,
        outgoing_simulation_timestamp: float,
    ) -> Tuple[str, ...]:
        offenders = []
        if not math.isfinite(source_simulation_timestamp):
            offenders.append(f"source_simulation_timestamp=nonfinite:{source_simulation_timestamp}")
        if not math.isfinite(outgoing_simulation_timestamp):
            offenders.append(f"outgoing_simulation_timestamp=nonfinite:{outgoing_simulation_timestamp}")
        if self.release_count != 1:
            offenders.append(f"startup_sync_release_count={self.release_count}:expected1")
        if (
            self.final_held_outgoing_timestamp is not None
            and math.isfinite(outgoing_simulation_timestamp)
            and outgoing_simulation_timestamp <= self.final_held_outgoing_timestamp
        ):
            offenders.append(
                "release_outgoing_timestamp="
                f"{outgoing_simulation_timestamp:.9f}"
                f"<=final_held_timestamp={self.final_held_outgoing_timestamp:.9f}"
            )
        return tuple(offenders)

    def check(
        self,
        request_count: int,
        host_time: float,
        source_simulation_timestamp: float,
        outgoing_simulation_timestamp: float,
    ) -> Optional[Tuple[float, float, float, Optional[float]]]:
        """Return (host_dt, source_dt, outgoing_dt, ratio) for logging, or None
        if disabled or this is the first tracked row (nothing to compare yet).
        Raises TimeDiscontinuityError if any offending condition is met.
        """
        if not self.enabled:
            return None
        current = {
            "request_count": request_count,
            "host_time": host_time,
            "source_simulation_timestamp": source_simulation_timestamp,
            "outgoing_simulation_timestamp": outgoing_simulation_timestamp,
        }
        if self.state == B3TimingGateState.WAIT_FIRST_REAL_ROW:
            offenders = self._release_boundary_offenders(
                source_simulation_timestamp=source_simulation_timestamp,
                outgoing_simulation_timestamp=outgoing_simulation_timestamp,
            )
            if offenders:
                raise TimeDiscontinuityError(
                    offenders,
                    {
                        "request_count": "STARTUP_SYNC_RELEASE",
                        "host_time": self.release_host_time if self.release_host_time is not None else host_time,
                        "source_simulation_timestamp": "",
                        "outgoing_simulation_timestamp": (
                            "" if self.final_held_outgoing_timestamp is None
                            else self.final_held_outgoing_timestamp
                        ),
                    },
                    current,
                )
            self.previous = current
            self.last_baseline_request_count = request_count
            self.state = B3TimingGateState.ACTIVE
            return None
        previous = self.previous
        self.previous = current
        if previous is None:
            self.last_baseline_request_count = request_count
            return None
        self.last_baseline_request_count = None
        host_dt = current["host_time"] - previous["host_time"]
        source_dt = current["source_simulation_timestamp"] - previous["source_simulation_timestamp"]
        outgoing_dt = current["outgoing_simulation_timestamp"] - previous["outgoing_simulation_timestamp"]
        ratio = (source_dt / host_dt) if host_dt > 0.0 else None
        offenders = b3_time_discontinuity_offenders(host_dt, source_dt, outgoing_dt)
        if offenders:
            raise TimeDiscontinuityError(offenders, previous, current)
        return (host_dt, source_dt, outgoing_dt, ratio)


class B3TimingGateState:
    """SR-75 B3 timing-quality gate phase names."""

    INACTIVE = "INACTIVE"
    WAIT_FIRST_REAL_ROW = "WAIT_FIRST_REAL_ROW"
    ACTIVE = "ACTIVE"


def write_b3_time_discontinuity_abort(
    path: Optional[str],
    offenders: Sequence[str],
    previous: Dict[str, object],
    current: Dict[str, object],
) -> None:
    if not path:
        return
    directory = os.path.dirname(path)
    if directory:
        os.makedirs(directory, exist_ok=True)

    def format_timestamp(value: object) -> object:
        return f"{value:.9f}" if isinstance(value, float) else value

    row = {
        "previous_request_count": previous["request_count"],
        "previous_host_time": format_timestamp(previous["host_time"]),
        "previous_source_simulation_timestamp": format_timestamp(previous["source_simulation_timestamp"]),
        "previous_outgoing_simulation_timestamp": format_timestamp(previous["outgoing_simulation_timestamp"]),
        "current_request_count": current["request_count"],
        "current_host_time": format_timestamp(current["host_time"]),
        "current_source_simulation_timestamp": format_timestamp(current["source_simulation_timestamp"]),
        "current_outgoing_simulation_timestamp": format_timestamp(current["outgoing_simulation_timestamp"]),
        "offenders": ";".join(offenders),
    }
    with open(path, "w", newline="", encoding="utf-8") as abort_file:
        writer = csv.DictWriter(abort_file, fieldnames=list(row))
        writer.writeheader()
        writer.writerow(row)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="SR-75 host-side SIM_JSON UDP responder")
    parser.add_argument("--listen-host", default="0.0.0.0")
    parser.add_argument("--listen-port", type=int, default=9002)
    parser.add_argument("--state-file", default=DEFAULT_STATE_FILE)
    parser.add_argument("--rate-limit-hz", type=float, default=0.0, help="Maximum reply rate; 0 disables limiting")
    parser.add_argument(
        "--fresh-state-wait-ms",
        type=float,
        default=80.0,
        help="Maximum time to wait for a newer JSBSim CSV row before reporting NO_FRESH_STATE",
    )
    parser.add_argument(
        "--allow-state-reuse",
        action="store_true",
        help="Permit replying with a reused JSBSim source row by advancing the outgoing timestamp",
    )
    parser.add_argument(
        "--no-rebase-time",
        action="store_true",
        help="Send raw JSBSim source timestamps instead of rebasing the first accepted row to t=0",
    )
    parser.add_argument("--state-timeout-ms", type=float, default=500.0)
    parser.add_argument("--verbose", action="store_true")
    parser.add_argument("--dry-run", action="store_true", help="Decode requests but do not send replies")
    parser.add_argument("--log-csv", default=None)
    parser.add_argument("--save-first-request", default=None, help="Write the exact first SIM_JSON request bytes to this path")
    parser.add_argument("--save-first-reply", default=None, help="Write the exact first SIM_JSON reply bytes to this path")
    parser.add_argument(
        "--b3-state-envelope-guard",
        action="store_true",
        help="Enable SR-75 B3 bounded-test state envelope abort",
    )
    parser.add_argument(
        "--b3-state-envelope-abort-row",
        default="/tmp/sr75_b3/b3_state_envelope_abort.csv",
        help="CSV path for the offending source row when --b3-state-envelope-guard aborts",
    )
    parser.add_argument(
        "--b3-state-envelope-rato-validation-airspeed-max-mps",
        type=float,
        default=None,
        help=(
            "Optional SR-75 RATO validation airspeed limit. When set, the normal B3 250 m/s "
            "airspeed exceedance is logged during RATO boost and bounded post-RATO coast, and this "
            "separately named limit is used for abort decisions in those phases. Other envelope checks "
            "are unchanged."
        ),
    )
    parser.add_argument(
        "--b3-state-envelope-post-rato-coast-timeout-s",
        type=float,
        default=5.0,
        help="Maximum bounded post-RATO coast time before the normal B3 250 m/s airspeed limit must be restored",
    )
    parser.add_argument(
        "--b3-timing-quality-gate",
        action="store_true",
        help=(
            "Enable SR-75 B3 scoring-window timing-quality abort: checks host_dt, "
            "source_simulation_dt, outgoing_simulation_dt, and their ratio between "
            "consecutive real (non-held) state rows."
        ),
    )
    parser.add_argument(
        "--b3-timing-quality-abort-row",
        default="/tmp/sr75_b3/b3_time_discontinuity_abort.csv",
        help="CSV path for previous/current row timestamps when --b3-timing-quality-gate aborts",
    )
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
    parser.add_argument("--rc1-pwm", type=int, default=None, help="Optional fixed SIM_JSON RC1 input PWM")
    parser.add_argument("--rc2-pwm", type=int, default=None, help="Optional fixed SIM_JSON RC2 input PWM")
    parser.add_argument("--rc3-pwm", type=int, default=None, help="Optional fixed SIM_JSON RC3 input PWM")
    parser.add_argument("--rc4-pwm", type=int, default=None, help="Optional fixed SIM_JSON RC4 input PWM")
    parser.add_argument("--rc5-pwm", type=int, default=None, help="Optional fixed SIM_JSON RC5 input PWM")
    parser.add_argument("--rc6-pwm", type=int, default=None, help="Optional fixed SIM_JSON RC6 input PWM")
    parser.add_argument("--rc7-pwm", type=int, default=None, help="Optional fixed SIM_JSON RC7 input PWM")
    parser.add_argument("--rc8-pwm", type=int, default=None, help="Optional fixed SIM_JSON RC8 input PWM")
    parser.add_argument(
        "--invalid-active-pwm-neutral",
        action="store_true",
        help="Treat out-of-range PWM on mapped actuator channels as a neutral software command",
    )
    parser.add_argument(
        "--precontrol-hold",
        action="store_true",
        help=(
            "SR-75 B3 only: hold a fixed bounded command reference (see --precontrol-*) until "
            "ArduPlane's first valid required-channel (CH1-CH4) PWM frame, instead of the generic "
            "stale/neutral failsafe. Does not change behavior unless this flag is passed."
        ),
    )
    parser.add_argument("--precontrol-elevator", type=float, default=-0.42, help="Pre-control hold elevator command")
    parser.add_argument("--precontrol-aileron", type=float, default=0.0, help="Pre-control hold aileron command")
    parser.add_argument("--precontrol-rudder", type=float, default=0.0, help="Pre-control hold rudder command")
    parser.add_argument(
        "--precontrol-throttle", type=float, default=0.55, help="Pre-control hold turbojet throttle command"
    )
    parser.add_argument("--precontrol-rato", type=float, default=0.0, help="Pre-control hold RATO throttle command")
    parser.add_argument(
        "--precontrol-eject", type=float, default=0.0,
        help=(
            "F22-GZ-Z: pre-control hold eject command. B3PrecontrolReference "
            "already has an eject field (default 0.0, previously unreachable "
            "from the CLI); this exposes it so a harness can drive dry-booster "
            "ejection through SR75DryBoosterEjectionLatch.update() -- the same "
            "single authoritative path used for the live PWM RATO_EJ_CH case -- "
            "instead of overriding propulsion/tank[2]/contents-lbs directly in "
            "a JSBSim runscript, which the responder's own continuous command "
            "stream would silently overwrite on the very next frame."
        ),
    )
    parser.add_argument(
        "--precontrol-burn-duration-s", type=float, default=0.0,
        help=(
            "F22-GZ-AA: if > 0, this single responder process switches its own "
            "--precontrol-* reference from the burn-phase values to the "
            "--precontrol-postburn-* values once this many seconds have "
            "elapsed since the responder itself started (a proxy for RATO "
            "ignition time, since this harness's precontrol-hold reference is "
            "what commands rato-throttle-cmd-norm from t=0). Replaces killing "
            "and restarting a second responder process (F22-GZ-X), which "
            "could only react after ArduPlane's multi-second boot/heartbeat "
            "delay and made the switch fire ~1.8s after true burnout."
        ),
    )
    parser.add_argument("--precontrol-postburn-elevator", type=float, default=None,
                         help="F22-GZ-AA: elevator after --precontrol-burn-duration-s elapses")
    parser.add_argument("--precontrol-postburn-throttle", type=float, default=None,
                         help="F22-GZ-AA: turbojet throttle after --precontrol-burn-duration-s elapses")
    parser.add_argument("--precontrol-postburn-rato", type=float, default=None,
                         help="F22-GZ-AA: RATO throttle after --precontrol-burn-duration-s elapses")
    parser.add_argument("--precontrol-postburn-eject", type=float, default=None,
                         help="F22-GZ-AA: eject command after --precontrol-burn-duration-s elapses")
    parser.add_argument(
        "--auto-gate-hold",
        action="store_true",
        help=(
            "SR-75 F21-K only: keep sending the fixed --precontrol-* reference as the single "
            "JSBSim-bound command (overriding whatever live PWM decodes to, even after "
            "precontrol/startup-sync would normally release) until --auto-gate-release-file "
            "appears on disk. Intended to keep FBWA-driven live commands out of JSBSim until "
            "the external caller (which watches ArduPlane's MAVLink MODE) confirms AUTO is "
            "active. Does not change behavior unless this flag is passed. Uses the single "
            "existing command_sink -- no second command source, unlike the disabled F21-J gate."
        ),
    )
    parser.add_argument(
        "--auto-gate-live-rato-eject",
        action="store_true",
        help=(
            "F22-GZ-AE: while --auto-gate-hold is overriding elevator/aileron/rudder/"
            "throttle with the fixed --precontrol-* reference, still pass through the "
            "LIVE decoded rato/eject fields (from ArduPlane's actual RATO_IGN_CH/"
            "RATO_EJ_CH PWM) instead of the reference's fixed rato/eject values. Lets "
            "ArduPlane's own RATOController own RATO ignition/burn/eject timing while "
            "the harness keeps the airframe's attitude/speed stable via the proven "
            "precontrol schedule -- no second command source, same single "
            "command_sink.send() call, just a per-field substitution on the one "
            "outgoing command."
        ),
    )
    parser.add_argument(
        "--auto-gate-release-file",
        type=str,
        default=None,
        help="Path whose existence releases --auto-gate-hold (created externally once MODE=AUTO is confirmed)",
    )
    parser.add_argument(
        "--jsbsim-boot-hold-release-file",
        type=str,
        default=None,
        help=(
            "F22-GZ-AG: while set, continuously send forces/hold-down=1 as an extra "
            "wire field on the single JSBSim command (see JSBSimActuatorCommand.hold_down) "
            "-- clamping JSBSim's rocket-launch-pad hold-down restraint so "
            "position/attitude stay pinned exactly at the release-state IC while "
            "ArduPlane boots, instead of the aircraft gliding/diving open-loop with no "
            "RATO thrust for the ~10-14s boot delay. Requires the runscript's <input> "
            "property list to include forces/hold-down as its last entry, matching the "
            "extra field this flag adds. Releases (hold_down=0.0, permanently) once the "
            "given path exists on disk -- created externally once MODE=AUTO is confirmed, "
            "same moment RATO ignition begins via --auto-gate-live-rato-eject."
        ),
    )
    parser.add_argument(
        "--startup-sync",
        action="store_true",
        help=(
            "SR-75 B3 only: serve a fixed synthetic sensor state matching the intended release "
            "condition (see --startup-sync-*) until ArduPlane's first valid required-channel PWM "
            "frame, then release with a one-time attitude bias locking that first sample to the "
            "requested roll/pitch. Does not change behavior unless this flag is passed."
        ),
    )
    parser.add_argument(
        "--startup-sync-required-valid-pwm-frames",
        type=int,
        default=20,
        help="Direct validation readiness: consecutive valid, non-stale, RATO-zero PWM frames required before release",
    )
    parser.add_argument(
        "--startup-sync-fbwa-confirmed",
        action="store_true",
        help="Direct validation readiness: external harness has confirmed FBWA before startup-sync release",
    )
    parser.add_argument(
        "--startup-sync-ahrs-ekf-type",
        type=int,
        default=None,
        help="Direct validation readiness: loaded AHRS_EKF_TYPE confirmed by external harness",
    )
    parser.add_argument("--startup-sync-roll-deg", type=float, default=15.0, help="Startup-sync release roll target")
    parser.add_argument("--startup-sync-pitch-deg", type=float, default=-2.0, help="Startup-sync release pitch target")
    parser.add_argument("--startup-sync-yaw-deg", type=float, default=315.0, help="Startup-sync held heading")
    parser.add_argument("--startup-sync-altitude-m", type=float, default=3000.0, help="Startup-sync held altitude")
    parser.add_argument(
        "--startup-sync-airspeed-mps", type=float, default=69.0, help="Startup-sync held true airspeed"
    )
    parser.add_argument(
        "--b3-startup-scoring-debug-rows",
        type=int,
        default=0,
        help="Print B3 scoring diagnostics for the first N real post-release rows",
    )
    parser.add_argument(
        "--startup-sync-latitude-deg", type=float, default=32.5378085, help="Startup-sync held latitude"
    )
    parser.add_argument(
        "--startup-sync-longitude-deg", type=float, default=74.3661944, help="Startup-sync held longitude"
    )
    parser.add_argument(
        "--startup-sync-force-release-file",
        type=str,
        default=None,
        help=(
            "F22-GZ-AH: path whose existence forces STARTUP_SYNC_HOLD -> RELEASED "
            "immediately (via B3StartupSync.force_release()), bypassing the normal "
            "valid-PWM/FBWA/EKF readiness heuristic. Intended for a harness that "
            "delays launching the real (release-state-IC) JSBSim process until "
            "ArduPlane is armed and AUTO -- ArduPlane spends its entire boot seeing "
            "only the synthetic held_reply_state() (matching the intended release "
            "IC exactly, including airspeed, since JSBSim never actually diverges "
            "from it), and only switches to genuine JSBSim state once this file "
            "appears (created externally once the freshly-launched JSBSim's state "
            "CSV has real rows). Also resets the --precontrol-burn-duration-s "
            "reference clock to this same moment instead of responder-process-start, "
            "since the real burn only begins once JSBSim is actually running."
        ),
    )
    return parser


def fixed_rc_inputs(args: argparse.Namespace) -> Optional[Tuple[int, ...]]:
    values = (
        args.rc1_pwm,
        args.rc2_pwm,
        args.rc3_pwm,
        args.rc4_pwm,
        args.rc5_pwm,
        args.rc6_pwm,
        args.rc7_pwm,
        args.rc8_pwm,
    )
    if all(value is None for value in values):
        return None

    rc_pwm = []
    for index, value in enumerate(values, start=1):
        pwm = 1500 if value is None else value
        if pwm < 800 or pwm > 2200:
            raise ValueError(f"RC{index} PWM out of range: {pwm}")
        rc_pwm.append(pwm)
    return tuple(rc_pwm)


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
        optional_roles=tuple(channel_map.optional_roles),
        timeout_ms=timeout_ms,
        surface_deadband=surface_deadband,
        throttle_deadband=throttle_deadband,
    )


@dataclass(frozen=True)
class InvalidActivePWM:
    role: str
    channel: int
    pwm: object
    reason: str


def active_pwm_invalid(command: NormalizedActuatorCommand, channel_map: ActuatorChannelMap) -> Optional[InvalidActivePWM]:
    if not command.pwm:
        return None
    for role in sorted(channel_map.channels):
        channel = channel_map.channels[role]
        if channel < 1 or channel > len(command.pwm):
            if channel_map.role_is_optional(role):
                continue
            return InvalidActivePWM(role=role, channel=channel, pwm="", reason="missing")
        pwm = command.pwm[channel - 1]
        if channel_map.role_is_optional(role) and pwm == 0:
            continue
        if pwm < 800 or pwm > 2200:
            return InvalidActivePWM(role=role, channel=channel, pwm=pwm, reason="out_of_range")
    return None


class B3ControlPhase:
    """SR-75 Layer 2J-B3C-C1 responder control phase names."""

    PRECONTROL_HOLD = "PRECONTROL_HOLD"
    ACTIVE_CONTROL = "ACTIVE_CONTROL"


class B3CommandSource:
    """SR-75 Layer 2J-B3C-C1 command-source labels for logging."""

    PRECONTROL_REFERENCE = "PRECONTROL_REFERENCE"
    ARDUPLANE_PWM = "ARDUPLANE_PWM"
    ACTIVE_STALE_FAILSAFE = "ACTIVE_STALE_FAILSAFE"


@dataclass(frozen=True)
class B3PrecontrolReference:
    """The fixed bounded command reference held before ArduPlane takes over."""

    elevator: float = -0.42
    aileron: float = 0.0
    rudder: float = 0.0
    turbojet_throttle: float = 0.55
    rato: float = 0.0
    eject: float = 0.0

    def as_command(self) -> NormalizedActuatorCommand:
        return NormalizedActuatorCommand(
            pwm=(),
            left_elevon=0.0,
            right_elevon=0.0,
            elevator=self.elevator,
            aileron=self.aileron,
            rudder=self.rudder,
            throttle_left=self.turbojet_throttle,
            throttle_right=self.turbojet_throttle,
            turbojet_throttle=self.turbojet_throttle,
            rato=self.rato,
            eject=self.eject,
            stale=False,
        )


class B3PrecontrolHandover:
    """SR-75 Layer 2J-B3C-C1 pre-control hold / active-control handover.

    Before ArduPlane produces its first valid required-channel PWM frame,
    the responder holds the fixed bounded command reference so the JSBSim
    trim is not overwritten by the generic stale/neutral failsafe during
    ArduPlane's startup delay. Once CH1-CH4 are valid, control hands over to
    ArduPlane's decoded PWM permanently; the reference is never resumed.
    """

    def __init__(self, reference: Optional[B3PrecontrolReference]):
        self.reference = reference
        self.phase = B3ControlPhase.PRECONTROL_HOLD if reference is not None else B3ControlPhase.ACTIVE_CONTROL
        self.first_valid_pwm_seen = False
        self.handover_count = 0
        self.transition: Optional[Dict[str, object]] = None
        self.transition_reported = False

    @property
    def enabled(self) -> bool:
        return self.reference is not None

    def _record_transition(
        self,
        host_time: float,
        simulation_timestamp: Optional[float],
        pwm: Sequence[int],
        first_active_command: NormalizedActuatorCommand,
    ) -> None:
        assert self.reference is not None
        self.transition = {
            "host_time": host_time,
            "simulation_timestamp": simulation_timestamp,
            "pwm1_4": tuple(pwm[:4]),
            "precontrol_command": self.reference,
            "first_active_command": first_active_command,
        }

    def resolve(
        self,
        raw_decoded_command: NormalizedActuatorCommand,
        output_command: NormalizedActuatorCommand,
        channel_map: ActuatorChannelMap,
        host_time: float,
        simulation_timestamp: Optional[float] = None,
        hold_active_control: bool = False,
    ) -> Tuple[NormalizedActuatorCommand, str]:
        """Return (command_to_send, command_source); may advance the phase once."""
        if not self.enabled:
            source = B3CommandSource.ACTIVE_STALE_FAILSAFE if output_command.stale else B3CommandSource.ARDUPLANE_PWM
            return output_command, source

        if self.phase == B3ControlPhase.PRECONTROL_HOLD:
            required_ok = (
                not raw_decoded_command.stale
                and active_pwm_invalid(raw_decoded_command, channel_map) is None
            )
            if required_ok:
                self.first_valid_pwm_seen = True
                if hold_active_control:
                    assert self.reference is not None
                    return self.reference.as_command(), B3CommandSource.PRECONTROL_REFERENCE
                self.phase = B3ControlPhase.ACTIVE_CONTROL
                self.handover_count += 1
                self._record_transition(host_time, simulation_timestamp, raw_decoded_command.pwm, output_command)
                return output_command, B3CommandSource.ARDUPLANE_PWM
            assert self.reference is not None
            return self.reference.as_command(), B3CommandSource.PRECONTROL_REFERENCE

        # ACTIVE_CONTROL: never fall back to the pre-control reference again.
        source = B3CommandSource.ACTIVE_STALE_FAILSAFE if output_command.stale else B3CommandSource.ARDUPLANE_PWM
        return output_command, source


class B3StartupPhase:
    """SR-75 Layer 2J-B3C-C2 startup-synchronization phase names."""

    STARTUP_SYNC_HOLD = "STARTUP_SYNC_HOLD"
    RELEASED = "RELEASED"


@dataclass(frozen=True)
class B3StartupSyncTarget:
    """The intended release initial condition, held during STARTUP_SYNC_HOLD."""

    roll_deg: float = 15.0
    pitch_deg: float = -2.0
    yaw_deg: float = 315.0
    altitude_m: float = 3000.0
    airspeed_mps: float = 69.0
    latitude_deg: float = 32.5378085
    longitude_deg: float = 74.3661944

    def held_state(self, timestamp_s: float) -> SimState:
        roll = degrees_to_radians(self.roll_deg)
        pitch = degrees_to_radians(self.pitch_deg)
        yaw = degrees_to_radians(self.yaw_deg)
        vn = self.airspeed_mps * math.cos(yaw)
        ve = self.airspeed_mps * math.sin(yaw)
        return SimState(
            timestamp_s=timestamp_s,
            source_timestamp_s=timestamp_s,
            latitude_deg=self.latitude_deg,
            longitude_deg=self.longitude_deg,
            altitude_m=self.altitude_m,
            roll_rad=roll,
            pitch_rad=pitch,
            yaw_rad=yaw,
            quaternion=euler_to_quaternion(roll, pitch, yaw),
            gyro_rad_s=(0.0, 0.0, 0.0),
            accel_body_mss=gravity_body_mss(roll, pitch, yaw),
            velocity_ned_mps=(vn, ve, 0.0),
            airspeed_mps=self.airspeed_mps,
            row_signature=f"startup_sync_hold:{timestamp_s:.9f}",
            row_monotonic_time=time.monotonic(),
        )


@dataclass(frozen=True)
class B3StartupReadiness:
    """External readiness facts for SR-75 direct SIM_JSON validation."""

    required_valid_pwm_frames: int = 1
    fbwa_confirmed: bool = True
    ahrs_ekf_type: Optional[int] = 10
    required_ahrs_ekf_type: int = 10

    def __post_init__(self) -> None:
        if self.required_valid_pwm_frames < 1:
            raise ValueError("required_valid_pwm_frames must be >= 1")


class B3StartupSync:
    """SR-75 Layer 2J-B3C-C2 startup synchronization.

    Before direct-topology validation readiness, the responder serves a fixed
    synthetic sensor state matching the intended release condition, so
    ArduPlane never observes the natural drift that would otherwise occur
    while it boots.

    JSBSim itself is not paused (empirically, writing attitude/rate
    properties over a second QTJSBSIM input does not hold FGPropagate's
    integrator -- the values are silently overwritten every frame). Instead,
    at the release instant a one-time attitude bias is computed from
    (target - actual JSBSim roll/pitch) and applied to every subsequent real
    JSBSim row, so the first released sample exactly matches the requested
    initial condition while all later relative motion (rates, changes) stays
    physically genuine.
    """

    def __init__(self, target: Optional[B3StartupSyncTarget], readiness: Optional[B3StartupReadiness] = None):
        self.target = target
        self.readiness = B3StartupReadiness() if readiness is None else readiness
        self.phase = B3StartupPhase.STARTUP_SYNC_HOLD if target is not None else B3StartupPhase.RELEASED
        self.release_count = 0
        self.control_ready_record: Optional[Dict[str, object]] = None
        self.release_record: Optional[Dict[str, object]] = None
        self.first_post_release_state_record: Optional[Dict[str, object]] = None
        self.scoring_start_record: Optional[Dict[str, object]] = None
        self._roll_bias_rad = 0.0
        self._pitch_bias_rad = 0.0
        self._bias_locked = False
        self._fresh_rows_since_release = 0
        self._hold_start_mono = time.monotonic()
        self._last_hold_timestamp = 0.0
        self._release_wall_offset = 0.0
        self._valid_pwm_frames = 0
        self.readiness_reason = "startup_sync_disabled" if target is None else "waiting_for_valid_pwm"

    @property
    def enabled(self) -> bool:
        return self.target is not None

    @property
    def final_held_timestamp(self) -> Optional[float]:
        return self._last_hold_timestamp if self.enabled else None

    @property
    def valid_pwm_frames(self) -> int:
        return self._valid_pwm_frames

    def held_reply_state(self, host_time: float) -> SimState:
        """A monotonically-increasing synthetic reply held at the release target."""
        assert self.target is not None
        timestamp_s = max(MIN_OUTGOING_TIMESTAMP_STEP_S, host_time - self._hold_start_mono)
        if timestamp_s <= self._last_hold_timestamp:
            timestamp_s = self._last_hold_timestamp + MIN_OUTGOING_TIMESTAMP_STEP_S
        self._last_hold_timestamp = timestamp_s
        return self.target.held_state(timestamp_s)

    def maybe_release(
        self,
        raw_decoded_command: NormalizedActuatorCommand,
        channel_map: ActuatorChannelMap,
        host_time: float,
        pwm: Sequence[int],
    ) -> None:
        """Advance HOLD -> RELEASED once the direct-topology readiness contract is met."""
        if not self.enabled or self.phase == B3StartupPhase.RELEASED:
            return
        invalid_pwm = active_pwm_invalid(raw_decoded_command, channel_map)
        if raw_decoded_command.stale:
            self._valid_pwm_frames = 0
            self.readiness_reason = "waiting_for_stale_zero"
            return
        if invalid_pwm is not None:
            self._valid_pwm_frames = 0
            self.readiness_reason = (
                "waiting_for_valid_pwm "
                f"role={invalid_pwm.role} channel={invalid_pwm.channel} pwm={invalid_pwm.pwm}"
            )
            return
        if abs(raw_decoded_command.rato) > 1.0e-9:
            self._valid_pwm_frames = 0
            self.readiness_reason = f"waiting_for_rato_zero rato={raw_decoded_command.rato:.9f}"
            return
        self._valid_pwm_frames += 1
        if self._valid_pwm_frames < self.readiness.required_valid_pwm_frames:
            self.readiness_reason = (
                "waiting_for_consecutive_valid_pwm "
                f"{self._valid_pwm_frames}/{self.readiness.required_valid_pwm_frames}"
            )
            return
        if not self.readiness.fbwa_confirmed:
            self.readiness_reason = "waiting_for_external_fbwa_confirmation"
            return
        if self.readiness.ahrs_ekf_type != self.readiness.required_ahrs_ekf_type:
            self.readiness_reason = (
                "waiting_for_ahrs_ekf_type "
                f"loaded={self.readiness.ahrs_ekf_type} required={self.readiness.required_ahrs_ekf_type}"
            )
            return
        self.readiness_reason = (
            "ready:valid_pwm_frames="
            f"{self._valid_pwm_frames} stale=0 rato=0 fbwa=1 "
            f"ahrs_ekf_type={self.readiness.ahrs_ekf_type}"
        )
        self.control_ready_record = {
            "host_time": host_time,
            "pwm1_4": tuple(pwm[:4]),
            "reason": self.readiness_reason,
        }
        self.phase = B3StartupPhase.RELEASED
        self.release_count += 1
        self._release_wall_offset = self._last_hold_timestamp
        self.release_record = {"host_time": host_time}

    def force_release(self, host_time: float) -> None:
        """F22-GZ-AH: release HOLD -> RELEASED unconditionally, bypassing the
        valid-PWM/FBWA/EKF readiness heuristic in maybe_release(). Used when
        an external caller (the harness) has just launched a FRESH JSBSim
        process at the release-state IC -- e.g. once its state CSV has
        actual rows -- so ArduPlane should stop seeing the synthetic
        held_reply_state() and start seeing genuine JSBSim state from that
        exact moment. Sets _release_wall_offset the same way maybe_release()
        does, so process_post_release_state()'s timestamp rebasing stays
        continuous across the hold -> release boundary (no backward jump,
        no B3_TIME_DISCONTINUITY_ABORT). No-op if already released.
        """
        if not self.enabled or self.phase == B3StartupPhase.RELEASED:
            return
        self.readiness_reason = "released:external_force_release"
        self.control_ready_record = {"host_time": host_time, "reason": self.readiness_reason}
        self.phase = B3StartupPhase.RELEASED
        self.release_count += 1
        self._release_wall_offset = self._last_hold_timestamp
        self.release_record = {"host_time": host_time}

    def _bounds_ok(
        self,
        state: SimState,
        raw_decoded_command: NormalizedActuatorCommand,
        channel_map: Optional[ActuatorChannelMap],
    ) -> bool:
        """Item-3 scoring_start bounds: roll/pitch/rates/PWM-valid/RATO=0."""
        return bool(self.scoring_bounds_report(state, raw_decoded_command, channel_map)["bounds_ok"])

    def scoring_bounds_report(
        self,
        state: SimState,
        raw_decoded_command: NormalizedActuatorCommand,
        channel_map: Optional[ActuatorChannelMap],
    ) -> Dict[str, object]:
        """Return per-condition scoring-start diagnostics for one post-release row."""
        assert self.target is not None
        roll_deg = radians_to_degrees(state.roll_rad)
        pitch_deg = radians_to_degrees(state.pitch_rad)
        p, q, r = state.gyro_rad_s
        invalid_pwm = None if channel_map is None else active_pwm_invalid(raw_decoded_command, channel_map)
        roll_in_bounds = abs(roll_deg - self.target.roll_deg) <= 2.0
        pitch_in_bounds = abs(pitch_deg - self.target.pitch_deg) <= 2.0
        rates_in_bounds = max(abs(v) for v in state.gyro_rad_s) < 0.2
        stale_zero = not raw_decoded_command.stale
        pwm_valid = invalid_pwm is None
        rato_zero = abs(raw_decoded_command.rato) <= 1.0e-9
        bounds_ok = roll_in_bounds and pitch_in_bounds and rates_in_bounds and stale_zero and pwm_valid and rato_zero
        invalid_pwm_text = ""
        if invalid_pwm is not None:
            invalid_pwm_text = (
                f"role={invalid_pwm.role} channel={invalid_pwm.channel} "
                f"pwm={invalid_pwm.pwm} reason={invalid_pwm.reason}"
            )
        return {
            "roll_deg": roll_deg,
            "pitch_deg": pitch_deg,
            "p": p,
            "q": q,
            "r": r,
            "raw_stale": raw_decoded_command.stale,
            "active_pwm_invalid": invalid_pwm_text,
            "raw_rato": raw_decoded_command.rato,
            "roll_in_bounds": roll_in_bounds,
            "pitch_in_bounds": pitch_in_bounds,
            "rates_in_bounds": rates_in_bounds,
            "pwm_valid": pwm_valid,
            "stale_zero": stale_zero,
            "rato_zero": rato_zero,
            "bounds_ok": bounds_ok,
        }

    def process_post_release_state(
        self,
        state: SimState,
        raw_decoded_command: Optional[NormalizedActuatorCommand] = None,
        channel_map: Optional[ActuatorChannelMap] = None,
        scoring_debug: Optional[Dict[str, object]] = None,
    ) -> SimState:
        """Apply the one-time attitude bias, rebase the timestamp, track scoring readiness.

        The outgoing timestamp continues monotonically from wherever the held
        sequence left off: StateMapper rebases the first accepted post-release
        row to approximately zero, so adding the recorded hold-phase offset
        here keeps the full outgoing sequence strictly increasing across the
        hold -> release boundary, with no backward jump.

        scoring_start is recorded once at least two fresh (non-reused) rows
        have been seen since release AND this row itself satisfies the full
        item-3 bounds (roll/pitch/rates/PWM-valid/RATO=0) -- discarding rows
        that only satisfy the row-count requirement but not the state bounds.
        """
        assert self.target is not None
        state = replace(state)
        state.timestamp_s += self._release_wall_offset
        if not self._bias_locked:
            self._roll_bias_rad = degrees_to_radians(self.target.roll_deg) - state.roll_rad
            self._pitch_bias_rad = degrees_to_radians(self.target.pitch_deg) - state.pitch_rad
            self._bias_locked = True
            self.first_post_release_state_record = {"simulation_timestamp": state.timestamp_s}

        if not state.reused_source_row:
            self._fresh_rows_since_release += 1

        state.roll_rad += self._roll_bias_rad
        state.pitch_rad += self._pitch_bias_rad
        state.quaternion = euler_to_quaternion(state.roll_rad, state.pitch_rad, state.yaw_rad)

        scoring_before = self.scoring_start_record
        bounds_report = None
        if raw_decoded_command is not None:
            bounds_report = self.scoring_bounds_report(state, raw_decoded_command, channel_map)
        fresh_rows_ready = self._fresh_rows_since_release >= 2
        if (
            self.scoring_start_record is None
            and fresh_rows_ready
            and raw_decoded_command is not None
            and bounds_report is not None
            and bounds_report["bounds_ok"]
        ):
            self.scoring_start_record = {"simulation_timestamp": state.timestamp_s}
        if scoring_debug is not None:
            scoring_debug.update({
                "phase": self.phase,
                "source_timestamp_s": state.source_timestamp_s,
                "timestamp_s": state.timestamp_s,
                "reused_source_row": state.reused_source_row,
                "fresh_rows_since_release": self._fresh_rows_since_release,
                "fresh_rows_ready": fresh_rows_ready,
                "scoring_start_before": scoring_before,
                "scoring_start_after": self.scoring_start_record,
            })
            if bounds_report is not None:
                scoring_debug.update(bounds_report)
        return state

    def startup_scoring_debug_row_ready(self, state: SimState) -> bool:
        """True for fresh real post-release rows eligible for opt-in debug output."""
        return self.enabled and self.phase == B3StartupPhase.RELEASED and not state.reused_source_row

    def scoring_eligible(
        self,
        state: SimState,
        raw_decoded_command: NormalizedActuatorCommand,
        channel_map: Optional[ActuatorChannelMap] = None,
    ) -> bool:
        """True once item-3 scoring_start conditions hold for this row."""
        if self.scoring_start_record is None:
            return False
        if state.timestamp_s < self.scoring_start_record["simulation_timestamp"]:
            return False
        return self._bounds_ok(state, raw_decoded_command, channel_map)


def jsbsim_command_from_actuator(
    actuator_command: NormalizedActuatorCommand,
    timestamp_s: Optional[float] = None,
    dry_booster_attached: bool = True,
    hold_down: Optional[float] = None,
) -> JSBSimActuatorCommand:
    timestamp = time.monotonic() if timestamp_s is None else timestamp_s
    return JSBSimActuatorCommand(
        timestamp_s=timestamp,
        elevator=actuator_command.elevator,
        aileron=actuator_command.aileron,
        rudder=actuator_command.rudder,
        turbojet_throttle=actuator_command.turbojet_throttle,
        rato_throttle=actuator_command.rato,
        dry_booster_weight_lbs=SR75_DRY_BOOSTER_WEIGHT_LBS if dry_booster_attached else 0.0,
        stale=actuator_command.stale,
        hold_down=hold_down,
    )


class SR75DryBoosterEjectionLatch:
    def __init__(self, attached_weight_lbs: float = SR75_DRY_BOOSTER_WEIGHT_LBS):
        self.attached_weight_lbs = attached_weight_lbs
        self.attached = True
        self.eject_count = 0

    @property
    def dry_booster_weight_lbs(self) -> float:
        return self.attached_weight_lbs if self.attached else 0.0

    def update(self, act_eject_norm: float) -> bool:
        if act_eject_norm > 0.5 and self.attached:
            self.attached = False
            self.eject_count += 1
            return True
        return False


def wait_for_reply_slot(last_reply_mono: float, min_interval: float) -> bool:
    if min_interval <= 0.0 or last_reply_mono <= 0.0:
        return False
    remaining = min_interval - (time.monotonic() - last_reply_mono)
    if remaining <= 0.0:
        return False
    time.sleep(remaining)
    return True


def read_reply_state(
    args: argparse.Namespace,
    reader: LatestCSVReader,
    mapper: StateMapper,
    mock_source: MockStateSource,
) -> SimState:
    if args.mock_state:
        return mock_source.state()

    deadline = time.monotonic() + max(0.0, args.fresh_state_wait_ms) / 1000.0
    while True:
        row, signature, mtime = reader.read_latest()
        row_age_ms = (time.time() - mtime) * 1000.0
        if row_age_ms > args.state_timeout_ms:
            raise StateError(f"STALE_STATE: latest row age {row_age_ms:.1f} ms")
        state = mapper.state_from_csv(row, signature, time.monotonic() - (row_age_ms / 1000.0))
        if mapper.last_source_timestamp is None or state.source_timestamp_s > mapper.last_source_timestamp:
            return state

        if args.allow_state_reuse:
            if mapper.last_outgoing_timestamp is not None and state.timestamp_s <= mapper.last_outgoing_timestamp:
                state.timestamp_s = mapper.last_outgoing_timestamp + 1.0e-6
            return state

        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            raise StateError(
                "NO_FRESH_STATE: "
                f"latest source timestamp {state.source_timestamp_s:.9f} "
                f"matches previous sent timestamp {mapper.last_source_timestamp:.9f}"
            )
        time.sleep(min(0.002, remaining))


def print_b3_startup_scoring_debug(request_count: int, diagnostic: Dict[str, object]) -> None:
    def fmt_record(record: object) -> str:
        if not record:
            return ""
        if isinstance(record, dict) and "simulation_timestamp" in record:
            return f"{float(record['simulation_timestamp']):.9f}"
        return str(record)

    print(
        "B3_STARTUP_SCORING_DEBUG "
        f"request_count={request_count} "
        f"phase={diagnostic.get('phase', '')} "
        f"source_timestamp_s={float(diagnostic.get('source_timestamp_s', 0.0)):.9f} "
        f"timestamp_s={float(diagnostic.get('timestamp_s', 0.0)):.9f} "
        f"reused_source_row={int(bool(diagnostic.get('reused_source_row', False)))} "
        f"fresh_rows_since_release={diagnostic.get('fresh_rows_since_release', '')} "
        f"roll_deg={float(diagnostic.get('roll_deg', 0.0)):.6f} "
        f"pitch_deg={float(diagnostic.get('pitch_deg', 0.0)):.6f} "
        f"p={float(diagnostic.get('p', 0.0)):.9f} "
        f"q={float(diagnostic.get('q', 0.0)):.9f} "
        f"r={float(diagnostic.get('r', 0.0)):.9f} "
        f"raw_stale={int(bool(diagnostic.get('raw_stale', False)))} "
        f"active_pwm_invalid='{diagnostic.get('active_pwm_invalid', '')}' "
        f"raw_rato={float(diagnostic.get('raw_rato', 0.0)):.9f} "
        f"bounds_ok={int(bool(diagnostic.get('bounds_ok', False)))} "
        f"scoring_before='{fmt_record(diagnostic.get('scoring_start_before'))}' "
        f"scoring_after='{fmt_record(diagnostic.get('scoring_start_after'))}' "
        f"roll_in_bounds={int(bool(diagnostic.get('roll_in_bounds', False)))} "
        f"pitch_in_bounds={int(bool(diagnostic.get('pitch_in_bounds', False)))} "
        f"rates_in_bounds={int(bool(diagnostic.get('rates_in_bounds', False)))} "
        f"pwm_valid={int(bool(diagnostic.get('pwm_valid', False)))} "
        f"stale_zero={int(bool(diagnostic.get('stale_zero', False)))} "
        f"rato_zero={int(bool(diagnostic.get('rato_zero', False)))} "
        f"fresh_rows_ready={int(bool(diagnostic.get('fresh_rows_ready', False)))}",
        flush=True,
    )


def main() -> int:
    args = build_arg_parser().parse_args()
    try:
        actuator_channel_map = build_actuator_channel_map(args)
    except ActuatorMapError as exc:
        print(f"ERROR: actuator channel map invalid: {exc}", file=sys.stderr)
        return 2
    try:
        rc_pwm = fixed_rc_inputs(args)
    except ValueError as exc:
        print(f"ERROR: fixed RC input invalid: {exc}", file=sys.stderr)
        return 2
    command_sink = None
    if args.jsbsim_command_target:
        try:
            command_sink = UDPJSBSimCommandSink(args.jsbsim_command_target)
        except JSBSimCommandError as exc:
            print(f"ERROR: JSBSim command target invalid: {exc}", file=sys.stderr)
            return 2

    precontrol_reference = (
        B3PrecontrolReference(
            elevator=args.precontrol_elevator,
            aileron=args.precontrol_aileron,
            rudder=args.precontrol_rudder,
            turbojet_throttle=args.precontrol_throttle,
            rato=args.precontrol_rato,
            eject=args.precontrol_eject,
        )
        if args.precontrol_hold
        else None
    )
    precontrol = B3PrecontrolHandover(precontrol_reference)

    # F22-GZ-AA: optional single-process burn -> post-burn phase switch (see
    # --precontrol-burn-duration-s help above). postburn_reference is only
    # built if a duration was given; each postburn value falls back to its
    # burn-phase counterpart if not explicitly overridden, so specifying
    # only e.g. --precontrol-postburn-eject still works.
    postburn_reference = None
    burn_phase_switch_done = False
    responder_start_monotonic = time.monotonic()
    # F22-GZ-AH: when --startup-sync-force-release-file is used, the real
    # (release-state-IC) JSBSim process is launched well after this
    # responder starts (it services ArduPlane's boot via the synthetic
    # held_reply_state() with no live JSBSim at all), so the burn-duration
    # clock must be anchored to the force-release moment, not
    # responder_start_monotonic -- otherwise --precontrol-burn-duration-s
    # would already have elapsed (switching to postburn values) before
    # JSBSim even exists. None here means "not yet anchored"; the main loop
    # sets it the moment force_release() fires.
    burn_phase_reference_monotonic = (
        None if args.startup_sync_force_release_file else responder_start_monotonic
    )
    if args.precontrol_hold and args.precontrol_burn_duration_s > 0:
        postburn_reference = B3PrecontrolReference(
            elevator=args.precontrol_postburn_elevator if args.precontrol_postburn_elevator is not None else args.precontrol_elevator,
            aileron=args.precontrol_aileron,
            rudder=args.precontrol_rudder,
            turbojet_throttle=args.precontrol_postburn_throttle if args.precontrol_postburn_throttle is not None else args.precontrol_throttle,
            rato=args.precontrol_postburn_rato if args.precontrol_postburn_rato is not None else args.precontrol_rato,
            eject=args.precontrol_postburn_eject if args.precontrol_postburn_eject is not None else args.precontrol_eject,
        )

    # F21-K: separate, single-source command gate. Unlike the disabled F21-J
    # approach (a second UDP sender racing this responder's own packets into
    # JSBSim, which corrupted the RATO burn and induced tumbling), this gate
    # overrides the outgoing actuator_command in-place, right before the one
    # existing command_sink.send() call below -- there is still only ever
    # one JSBSim-bound command source. Independent of --precontrol-hold /
    # startup-sync (which release once PWM becomes valid, long before
    # ArduPlane's flight mode reaches AUTO); this gate instead releases only
    # when an external caller (which watches ArduPlane's MAVLink MODE)
    # creates --auto-gate-release-file.
    auto_gate_reference = B3PrecontrolReference(
        elevator=args.precontrol_elevator,
        aileron=args.precontrol_aileron,
        rudder=args.precontrol_rudder,
        turbojet_throttle=args.precontrol_throttle,
        rato=args.precontrol_rato,
    )
    auto_gate_released = not args.auto_gate_hold

    startup_sync_target = (
        B3StartupSyncTarget(
            roll_deg=args.startup_sync_roll_deg,
            pitch_deg=args.startup_sync_pitch_deg,
            yaw_deg=args.startup_sync_yaw_deg,
            altitude_m=args.startup_sync_altitude_m,
            airspeed_mps=args.startup_sync_airspeed_mps,
            latitude_deg=args.startup_sync_latitude_deg,
            longitude_deg=args.startup_sync_longitude_deg,
        )
        if args.startup_sync
        else None
    )
    startup_readiness = B3StartupReadiness(
        required_valid_pwm_frames=args.startup_sync_required_valid_pwm_frames,
        fbwa_confirmed=args.startup_sync_fbwa_confirmed,
        ahrs_ekf_type=args.startup_sync_ahrs_ekf_type,
    )
    startup_sync = B3StartupSync(startup_sync_target, startup_readiness)
    timing_gate = B3TimingQualityGate(args.b3_timing_quality_gate)

    print("SR75 SIM_JSON RESPONDER")
    if command_sink is None:
        print("ACTUATOR OUTPUT: DISABLED")
    else:
        print(f"ACTUATOR OUTPUT: SOFTWARE_JSBSIM_ONLY target={args.jsbsim_command_target}")
    print("SERIAL OUTPUT: DISABLED")
    print("RATO/ENGINE/RELAY OUTPUT: DISABLED")
    mode = "LOG_ONLY" if command_sink is None else "JSBSIM_COMMAND"
    print(f"ACTUATOR DECODE: {mode} {describe_channel_map(actuator_channel_map)}")
    if rc_pwm is not None:
        print(f"SIM_JSON RC INPUT: {','.join(str(value) for value in rc_pwm)}")
    if precontrol.enabled:
        ref = precontrol_reference
        print(
            "B3 PRECONTROL_HOLD: enabled "
            f"elevator={ref.elevator:.3f} aileron={ref.aileron:.3f} rudder={ref.rudder:.3f} "
            f"throttle={ref.turbojet_throttle:.3f} rato={ref.rato:.3f}"
        )
    if startup_sync.enabled:
        tgt = startup_sync_target
        print(
            "B3 STARTUP_SYNC: enabled "
            f"roll={tgt.roll_deg:.3f} pitch={tgt.pitch_deg:.3f} yaw={tgt.yaw_deg:.3f} "
            f"altitude={tgt.altitude_m:.1f} airspeed={tgt.airspeed_mps:.1f}"
        )
        print(
            "B3 STARTUP_SYNC READINESS: "
            f"valid_pwm_frames={startup_sync.readiness.required_valid_pwm_frames} "
            f"fbwa_confirmed={int(startup_sync.readiness.fbwa_confirmed)} "
            f"ahrs_ekf_type={startup_sync.readiness.ahrs_ekf_type} "
            f"required_ahrs_ekf_type={startup_sync.readiness.required_ahrs_ekf_type}"
        )

    csv_file, log_writer = open_log(args.log_csv)
    reader = LatestCSVReader(args.state_file)
    mapper = StateMapper(strict=args.strict, rebase_time=not args.no_rebase_time)
    mock_source = MockStateSource()
    actuator_bridge = SoftwareActuatorBridge(actuator_channel_map)
    min_interval = 1.0 / args.rate_limit_hz if args.rate_limit_hz and args.rate_limit_hz > 0.0 else 0.0
    last_reply_mono = 0.0
    request_count = 0
    previous_request_received_host_time: Optional[float] = None
    saved_first_request = False
    saved_first_reply = False
    startup_scoring_debug_logged = 0
    b3_guard_phase = "NORMAL"
    post_rato_coast_start_timestamp: Optional[float] = None
    dry_booster_latch = SR75DryBoosterEjectionLatch()
    boot_hold_released = not args.jsbsim_boot_hold_release_file

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

            if (
                args.startup_sync_force_release_file
                and startup_sync.enabled
                and startup_sync.phase != B3StartupPhase.RELEASED
                and os.path.exists(args.startup_sync_force_release_file)
            ):
                startup_sync.force_release(started)
                burn_phase_reference_monotonic = started
                print(f"STARTUP_SYNC_FORCE_RELEASED host_time={started:.9f}")

            if (
                postburn_reference is not None
                and not burn_phase_switch_done
                and burn_phase_reference_monotonic is not None
                and (started - burn_phase_reference_monotonic) >= args.precontrol_burn_duration_s
            ):
                precontrol.reference = postburn_reference
                burn_phase_switch_done = True
                print(
                    "PRECONTROL_BURN_PHASE_SWITCH "
                    f"elapsed_s={started - burn_phase_reference_monotonic:.3f} "
                    f"elev={postburn_reference.elevator:.3f} thr={postburn_reference.turbojet_throttle:.3f} "
                    f"rato={postburn_reference.rato:.3f} eject={postburn_reference.eject:.3f}"
                )

            if (
                args.jsbsim_boot_hold_release_file
                and not boot_hold_released
                and os.path.exists(args.jsbsim_boot_hold_release_file)
            ):
                boot_hold_released = True
                print(f"JSBSIM_BOOT_HOLD_RELEASED host_time={started:.9f}")
            hold_down_value = None if not args.jsbsim_boot_hold_release_file else (0.0 if boot_hold_released else 1.0)

            try:
                data, source = sock.recvfrom(4096)
            except socket.timeout:
                if command_sink is not None:
                    if precontrol.enabled and precontrol.phase == B3ControlPhase.PRECONTROL_HOLD:
                        assert precontrol.reference is not None
                        command_sink.send(
                            jsbsim_command_from_actuator(
                                precontrol.reference.as_command(),
                                timestamp_s=started,
                                dry_booster_attached=dry_booster_latch.attached,
                                hold_down=hold_down_value,
                            )
                        )
                        if args.verbose:
                            print(
                                "JSBSIM_COMMAND PRECONTROL_REFERENCE "
                                f"elev={precontrol.reference.elevator:.3f} ail={precontrol.reference.aileron:.3f} "
                                f"rud={precontrol.reference.rudder:.3f} thr={precontrol.reference.turbojet_throttle:.3f} "
                                f"rato={precontrol.reference.rato:.3f}"
                            )
                    else:
                        actuator_command = actuator_bridge.current_command(started)
                        if actuator_command.stale:
                            sent = command_sink.send_stale_once(
                                timestamp_s=started,
                                dry_booster_weight_lbs=dry_booster_latch.dry_booster_weight_lbs,
                            )
                            if args.verbose and sent is not None:
                                print("JSBSIM_COMMAND stale=1 elev=0.000 ail=0.000 rud=0.000 thr=0.000 rato=0.000")
                continue
            request_count += 1
            request_received_host_time = started
            request_previous_host_time = previous_request_received_host_time
            previous_request_received_host_time = request_received_host_time
            if args.save_first_request and not saved_first_request:
                with open(args.save_first_request, "wb") as request_file:
                    request_file.write(data)
                saved_first_request = True
                if args.verbose:
                    print(f"SAVED_FIRST_REQUEST {args.save_first_request} bytes={len(data)}")
            try:
                packet = decode_control_packet(data)
            except ValueError as exc:
                reason = f"MALFORMED_PACKET: {exc}"
                print(reason)
                malformed_command_source = ""
                if command_sink is not None and precontrol.enabled and precontrol.phase == B3ControlPhase.PRECONTROL_HOLD:
                    assert precontrol.reference is not None
                    command_sink.send(
                        jsbsim_command_from_actuator(
                            precontrol.reference.as_command(),
                            timestamp_s=started,
                            dry_booster_attached=dry_booster_latch.attached,
                            hold_down=hold_down_value,
                        )
                    )
                    malformed_command_source = B3CommandSource.PRECONTROL_REFERENCE
                write_log(
                    log_writer,
                    csv_file,
                    make_log_row(
                        request_count=request_count,
                        source=source,
                        packet_valid=False,
                        started=started,
                        error_reason=reason,
                        reply_reason=reason,
                        command_source=malformed_command_source,
                        precontrol=precontrol,
                        startup_sync=startup_sync,
                        timing_gate=timing_gate,
                        previous_request_received_host_time=request_previous_host_time,
                        request_received_host_time=request_received_host_time,
                        dry_booster_attached=dry_booster_latch.attached,
                    ),
                )
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
            raw_decoded_command = actuator_command
            invalid_pwm = active_pwm_invalid(actuator_command, actuator_channel_map)
            if args.invalid_active_pwm_neutral and invalid_pwm is not None:
                actuator_reason = (
                    "ACTUATOR_INVALID_ACTIVE_PWM_NEUTRAL "
                    f"role={invalid_pwm.role} channel={invalid_pwm.channel} "
                    f"pwm={invalid_pwm.pwm} reason={invalid_pwm.reason}"
                )
                print(
                    "INVALID_ACTIVE_PWM "
                    f"role={invalid_pwm.role} channel={invalid_pwm.channel} "
                    f"pwm={invalid_pwm.pwm} reason={invalid_pwm.reason}"
                )
                actuator_command = NormalizedActuatorCommand.neutral()

            was_holding = startup_sync.enabled and startup_sync.phase == B3StartupPhase.STARTUP_SYNC_HOLD
            actuator_command, command_source = precontrol.resolve(
                raw_decoded_command=raw_decoded_command,
                output_command=actuator_command,
                channel_map=actuator_channel_map,
                host_time=started,
                simulation_timestamp=mapper.last_outgoing_timestamp,
                hold_active_control=was_holding,
            )
            if precontrol.transition is not None and not precontrol.transition_reported:
                transition = precontrol.transition
                pre_cmd = transition["precontrol_command"]
                active_cmd = transition["first_active_command"]
                print(
                    "PRECONTROL_TO_ACTIVE "
                    f"host_time={transition['host_time']:.9f} "
                    f"simulation_timestamp={transition['simulation_timestamp']} "
                    f"pwm1_4={transition['pwm1_4']} "
                    f"precontrol_elev={pre_cmd.elevator:.3f} precontrol_ail={pre_cmd.aileron:.3f} "
                    f"precontrol_rud={pre_cmd.rudder:.3f} precontrol_thr={pre_cmd.turbojet_throttle:.3f} "
                    f"precontrol_rato={pre_cmd.rato:.3f} "
                    f"first_active_elev={active_cmd.elevator:.3f} first_active_ail={active_cmd.aileron:.3f} "
                    f"first_active_rud={active_cmd.rudder:.3f} first_active_thr={active_cmd.turbojet_throttle:.3f} "
                    f"first_active_rato={active_cmd.rato:.3f}"
                )
                precontrol.transition_reported = True

            if dry_booster_latch.update(actuator_command.eject):
                print(
                    "SR75_DRY_BOOSTER_EJECT "
                    f"request={request_count} host_time={started:.9f} "
                    f"act_eject={actuator_command.eject:.3f} "
                    f"dry_booster_weight_lbs=0.000000 eject_count={dry_booster_latch.eject_count}"
                )

            startup_sync.maybe_release(
                raw_decoded_command=raw_decoded_command,
                channel_map=actuator_channel_map,
                host_time=started,
                pwm=packet.pwm,
            )
            if was_holding and startup_sync.phase == B3StartupPhase.RELEASED:
                assert startup_sync.control_ready_record is not None
                ready = startup_sync.control_ready_record
                timing_gate.on_startup_sync_release(
                    final_held_outgoing_timestamp=startup_sync.final_held_timestamp,
                    release_host_time=started,
                    release_count=startup_sync.release_count,
                )
                print(
                    "STARTUP_SYNC_RELEASE "
                    f"control_ready_host_time={ready['host_time']:.9f} "
                    f"pwm1_4={ready['pwm1_4']} "
                    f"readiness_reason='{ready['reason']}'"
                )

            # F21-K: override only what is actually sent to JSBSim below --
            # precontrol/startup-sync/dry-booster state above all continue
            # tracking the real decoded command untouched.
            if args.auto_gate_hold and not auto_gate_released:
                if args.auto_gate_release_file and os.path.exists(args.auto_gate_release_file):
                    auto_gate_released = True
                    print(f"AUTO_GATE_RELEASED host_time={started:.9f}")
                elif args.auto_gate_live_rato_eject:
                    # F22-GZ-AE: reuse the same burn/postburn elevator-throttle
                    # switch built for F22-GZ-AA (postburn_reference is only
                    # non-None when --precontrol-burn-duration-s was given),
                    # so the airframe still gets an appropriate attitude/
                    # throttle reference across the whole (now longer, since
                    # ArduPlane's own RATO_EJECT_S coast phase follows
                    # burnout) run -- only rato/eject are live.
                    base_ref = (
                        postburn_reference
                        if (postburn_reference is not None and burn_phase_switch_done)
                        else auto_gate_reference
                    )
                    actuator_command = replace(
                        base_ref.as_command(),
                        rato=raw_decoded_command.rato,
                        eject=raw_decoded_command.eject,
                    )
                else:
                    actuator_command = auto_gate_reference.as_command()

            if command_sink is not None:
                try:
                    command_sink.send(
                        jsbsim_command_from_actuator(
                            actuator_command,
                            timestamp_s=started,
                            dry_booster_attached=dry_booster_latch.attached,
                            hold_down=hold_down_value,
                        )
                    )
                    if args.verbose:
                        print(
                            "JSBSIM_COMMAND "
                            f"source={command_source} "
                            f"stale={int(actuator_command.stale)} "
                            f"elev={actuator_command.elevator:.3f} ail={actuator_command.aileron:.3f} "
                            f"rud={actuator_command.rudder:.3f} "
                            f"thr={actuator_command.turbojet_throttle:.3f} rato={actuator_command.rato:.3f} "
                            f"eject={actuator_command.eject:.3f} "
                            f"dry_booster_lbs={dry_booster_latch.dry_booster_weight_lbs:.6f} "
                            f"hold_down={hold_down_value}"
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
                    f"rato={actuator_command.rato:.3f} eject={actuator_command.eject:.3f} "
                    f"stale={int(actuator_command.stale)}{warnings}"
                )

            previous_sent_timestamp = mapper.last_outgoing_timestamp
            rate_waited = wait_for_reply_slot(last_reply_mono, min_interval)

            timing_quality = None
            mapped_state = None
            try:
                if startup_sync.enabled and startup_sync.phase == B3StartupPhase.STARTUP_SYNC_HOLD:
                    state = startup_sync.held_reply_state(started)
                else:
                    mapped_state = read_reply_state(args, reader, mapper, mock_source)
                    state = mapped_state
                    if startup_sync.enabled:
                        scoring_debug = (
                            {}
                            if (
                                startup_scoring_debug_logged < max(0, args.b3_startup_scoring_debug_rows)
                                and startup_sync.startup_scoring_debug_row_ready(mapped_state)
                            )
                            else None
                        )
                        state = startup_sync.process_post_release_state(
                            mapped_state,
                            raw_decoded_command=raw_decoded_command,
                            channel_map=actuator_channel_map,
                            scoring_debug=scoring_debug,
                        )
                        if scoring_debug is not None:
                            startup_scoring_debug_logged += 1
                            print_b3_startup_scoring_debug(request_count, scoring_debug)
                    timing_quality = timing_gate.check(
                        request_count,
                        host_time=started,
                        source_simulation_timestamp=state.source_timestamp_s,
                        outgoing_simulation_timestamp=state.timestamp_s,
                    )
                if args.b3_state_envelope_guard:
                    airspeed_max_mps = None
                    airspeed_limit_name = "B3"
                    rato_validation_limit = args.b3_state_envelope_rato_validation_airspeed_max_mps
                    if rato_validation_limit is not None:
                        rato_validation_limit = min(rato_validation_limit, 350.0)
                    rato_active = actuator_command.rato > 0.5
                    if rato_active:
                        if b3_guard_phase != "RATO_BOOST":
                            print(
                                "B3_STATE_ENVELOPE_GUARD_PHASE "
                                f"{b3_guard_phase}->RATO_BOOST "
                                f"source_timestamp={state.source_timestamp_s:.9f} "
                                f"airspeed_mps={state.airspeed_mps:.6f}"
                            )
                        b3_guard_phase = "RATO_BOOST"
                        post_rato_coast_start_timestamp = None
                    elif b3_guard_phase == "RATO_BOOST":
                        b3_guard_phase = "POST_RATO_COAST"
                        post_rato_coast_start_timestamp = state.source_timestamp_s
                        print(
                            "B3_STATE_ENVELOPE_GUARD_PHASE "
                            f"RATO_BOOST->POST_RATO_COAST "
                            f"source_timestamp={state.source_timestamp_s:.9f} "
                            f"airspeed_mps={state.airspeed_mps:.6f}"
                        )
                    elif b3_guard_phase == "POST_RATO_COAST":
                        tas_min, tas_max = B3_ENVELOPE_LIMITS["airspeed_mps"]
                        if tas_min <= state.airspeed_mps <= tas_max:
                            print(
                                "B3_STATE_ENVELOPE_GUARD_PHASE "
                                f"POST_RATO_COAST->NORMAL "
                                f"source_timestamp={state.source_timestamp_s:.9f} "
                                f"airspeed_mps={state.airspeed_mps:.6f}"
                            )
                            b3_guard_phase = "NORMAL"
                            post_rato_coast_start_timestamp = None
                    if b3_guard_phase == "RATO_BOOST" and rato_validation_limit is not None:
                        airspeed_max_mps = rato_validation_limit
                        airspeed_limit_name = "RATO_VALIDATION"
                    elif b3_guard_phase == "POST_RATO_COAST" and rato_validation_limit is not None:
                        thrust_n = b3_rato_thrust_newtons(state)
                        post_elapsed = (
                            0.0
                            if post_rato_coast_start_timestamp is None
                            else state.source_timestamp_s - post_rato_coast_start_timestamp
                        )
                        post_offenders = []
                        if actuator_command.rato > 0.5:
                            post_offenders.append(f"post_rato_act_rato_norm={actuator_command.rato:.6g}:expected0")
                        if thrust_n is None:
                            post_offenders.append("post_rato_thrust_n=missing")
                        elif abs(thrust_n) > 100.0:
                            post_offenders.append(f"post_rato_thrust_n={thrust_n:.6g}:expected0")
                        if post_elapsed > args.b3_state_envelope_post_rato_coast_timeout_s:
                            post_offenders.append(
                                "post_rato_coast_elapsed_s="
                                f"{post_elapsed:.6g}:>{args.b3_state_envelope_post_rato_coast_timeout_s}"
                            )
                        if post_offenders:
                            raise StateEnvelopeError(post_offenders)
                        airspeed_max_mps = rato_validation_limit
                        airspeed_limit_name = "POST_RATO_COAST"
                    validate_b3_state_envelope(
                        state,
                        airspeed_max_mps=airspeed_max_mps,
                        airspeed_limit_name=airspeed_limit_name,
                    )
                payload = state.to_json_bytes(rc_pwm=rc_pwm)
            except TimeDiscontinuityError as exc:
                reason = f"B3_TIME_DISCONTINUITY_ABORT: {';'.join(exc.offenders)}"
                print(f"B3_TIME_DISCONTINUITY_ABORT offenders={';'.join(exc.offenders)}")
                write_b3_time_discontinuity_abort(
                    args.b3_timing_quality_abort_row, exc.offenders, exc.previous, exc.current
                )
                if command_sink is not None:
                    try:
                        command_sink.send(
                            jsbsim_command_from_actuator(
                                NormalizedActuatorCommand.neutral(),
                                timestamp_s=started,
                                dry_booster_attached=dry_booster_latch.attached,
                                hold_down=hold_down_value,
                            )
                        )
                        print("B3_TIME_DISCONTINUITY_ABORT neutral_jsbsim_command_sent=1")
                    except JSBSimCommandError as command_exc:
                        reason = f"{reason};JSBSIM_NEUTRAL_COMMAND_ERROR: {command_exc}"
                        print(f"B3_TIME_DISCONTINUITY_ABORT neutral_jsbsim_command_sent=0 error={command_exc}")
                write_log(
                    log_writer,
                    csv_file,
                    make_log_row(
                        request_count=request_count,
                        source=source,
                        packet_valid=True,
                        started=started,
                        error_reason=reason,
                        previous_sent_timestamp=previous_sent_timestamp,
                        reply_reason="B3_TIME_DISCONTINUITY_ABORT",
                        actuator_command=actuator_command,
                        control_packet=packet,
                        command_source=command_source,
                        precontrol=precontrol,
                        startup_sync=startup_sync,
                        timing_gate=timing_gate,
                        previous_request_received_host_time=request_previous_host_time,
                        request_received_host_time=request_received_host_time,
                        pwm_frame_received_host_time=request_received_host_time,
                        dry_booster_attached=dry_booster_latch.attached,
                    ),
                )
                return 4
            except StateEnvelopeError as exc:
                offenders = tuple(exc.offenders)
                reason = f"B3_STATE_ENVELOPE_ABORT: {';'.join(offenders)}"
                print(f"B3_STATE_ENVELOPE_ABORT offenders={';'.join(offenders)}")
                write_b3_state_envelope_abort(args.b3_state_envelope_abort_row, state, offenders)
                if command_sink is not None:
                    try:
                        command_sink.send(
                            jsbsim_command_from_actuator(
                                NormalizedActuatorCommand.neutral(),
                                timestamp_s=started,
                                dry_booster_attached=dry_booster_latch.attached,
                                hold_down=hold_down_value,
                            )
                        )
                        print("B3_STATE_ENVELOPE_ABORT neutral_jsbsim_command_sent=1")
                    except JSBSimCommandError as command_exc:
                        reason = f"{reason};JSBSIM_NEUTRAL_COMMAND_ERROR: {command_exc}"
                        print(f"B3_STATE_ENVELOPE_ABORT neutral_jsbsim_command_sent=0 error={command_exc}")
                write_log(
                    log_writer,
                    csv_file,
                    make_log_row(
                        request_count=request_count,
                        source=source,
                        packet_valid=True,
                        started=started,
                        state=state,
                        error_reason=reason,
                        previous_sent_timestamp=previous_sent_timestamp,
                        reply_reason="B3_STATE_ENVELOPE_ABORT",
                        actuator_command=actuator_command,
                        control_packet=packet,
                        command_source=command_source,
                        precontrol=precontrol,
                        startup_sync=startup_sync,
                        timing_quality=timing_quality,
                        timing_gate=timing_gate,
                        previous_request_received_host_time=request_previous_host_time,
                        request_received_host_time=request_received_host_time,
                        pwm_frame_received_host_time=request_received_host_time,
                        dry_booster_attached=dry_booster_latch.attached,
                    ),
                )
                return 3
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
                        request_count=request_count,
                        source=source,
                        packet_valid=True,
                        started=started,
                        error_reason=reason,
                        previous_sent_timestamp=previous_sent_timestamp,
                        reply_reason=reason,
                        actuator_command=actuator_command,
                        control_packet=packet,
                        command_source=command_source,
                        precontrol=precontrol,
                        startup_sync=startup_sync,
                        timing_quality=timing_quality,
                        timing_gate=timing_gate,
                        previous_request_received_host_time=request_previous_host_time,
                        request_received_host_time=request_received_host_time,
                        pwm_frame_received_host_time=request_received_host_time,
                        dry_booster_attached=dry_booster_latch.attached,
                    ),
                )
                if args.once:
                    return 1
                continue

            reply_bytes = 0
            reply_reason = "RATE_WAIT" if rate_waited else "OK"
            if args.dry_run:
                reason = "DRY_RUN"
                reply_reason = reason
                if args.verbose:
                    print(f"DRY_RUN would send {len(payload)} bytes to {source[0]}:{source[1]}")
            else:
                if args.save_first_reply and not saved_first_reply:
                    with open(args.save_first_reply, "wb") as reply_file:
                        reply_file.write(payload)
                    saved_first_reply = True
                    if args.verbose:
                        print(f"SAVED_FIRST_REPLY {args.save_first_reply} bytes={len(payload)}")
                reply_bytes = sock.sendto(payload, source)
                last_reply_mono = time.monotonic()
                reply_send_host_time = last_reply_mono
                reason = actuator_reason
                if actuator_reason and reply_reason == "OK":
                    reply_reason = actuator_reason
                startup_sync_holding = (
                    startup_sync.enabled and startup_sync.phase == B3StartupPhase.STARTUP_SYNC_HOLD
                )
                if not args.mock_state and not startup_sync_holding:
                    mapper.accept_state(mapped_state if mapped_state is not None else state)
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
                    request_count=request_count,
                    source=source,
                    packet_valid=True,
                    started=started,
                    state=state,
                    reply_bytes=reply_bytes,
                    error_reason=reason,
                    previous_sent_timestamp=previous_sent_timestamp,
                    reply_reason=reply_reason,
                    actuator_command=actuator_command,
                    control_packet=packet,
                    command_source=command_source,
                    precontrol=precontrol,
                    startup_sync=startup_sync,
                    timing_quality=timing_quality,
                    timing_gate=timing_gate,
                    previous_request_received_host_time=request_previous_host_time,
                    request_received_host_time=request_received_host_time,
                    pwm_frame_received_host_time=request_received_host_time,
                    reply_send_host_time=reply_send_host_time if not args.dry_run else None,
                    dry_booster_attached=dry_booster_latch.attached,
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
