#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Software-only SR-75 SIM_JSON PWM decoder.

This module converts ArduPilot SIM_JSON servo PWM packets into normalized
SR-75 JSBSim command values for logging and offline validation.  It does not
open devices or write commands to JSBSim/hardware.
"""

import json
import math
import time
from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Mapping, Optional, Sequence, Tuple


PWM_LOW = 1000
PWM_NEUTRAL = 1500
PWM_HIGH = 2000
PWM_HARD_LOW = 800
PWM_HARD_HIGH = 2200

DEFAULT_TIMEOUT_MS = 250.0
DEFAULT_SURFACE_DEADBAND = 0.02
DEFAULT_THROTTLE_DEADBAND = 0.01

DEFAULT_CHANNELS = {
    "left_elevon": 1,
    "right_elevon": 2,
    "rudder": 4,
    "throttle_left": 3,
    "throttle_right": 3,
    "rato": 7,
    "eject": 8,
}

JSBSIM_PROPERTIES = {
    "elevator": "fcs/elevator-cmd-norm",
    "aileron": "fcs/aileron-cmd-norm",
    "rudder": "fcs/rudder-cmd-norm",
    "turbojet_throttle": "fcs/turbojet-throttle-cmd-norm",
    "rato_throttle": "fcs/rato-throttle-cmd-norm",
    "dry_booster_weight": "propulsion/tank[2]/contents-lbs",
}


class ActuatorMapError(Exception):
    """Raised when the actuator channel map is invalid."""


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def apply_deadband(value: float, deadband: float) -> float:
    return 0.0 if abs(value) < deadband else value


def normalize_surface_pwm(pwm: int, deadband: float = DEFAULT_SURFACE_DEADBAND) -> float:
    return apply_deadband(clamp((pwm - PWM_NEUTRAL) / 500.0, -1.0, 1.0), deadband)


def normalize_throttle_pwm(pwm: int, deadband: float = DEFAULT_THROTTLE_DEADBAND) -> float:
    return apply_deadband(clamp((pwm - PWM_LOW) / 1000.0, 0.0, 1.0), deadband)


def neutral_pwm_values(channel_count: int = 16) -> List[int]:
    pwm = [PWM_NEUTRAL] * channel_count
    for name in ("throttle_left", "throttle_right", "rato", "eject"):
        channel = DEFAULT_CHANNELS[name]
        if channel <= channel_count:
            pwm[channel - 1] = PWM_LOW
    return pwm


@dataclass(frozen=True)
class ActuatorChannelMap:
    channels: Dict[str, int] = field(default_factory=lambda: dict(DEFAULT_CHANNELS))
    optional_roles: Tuple[str, ...] = field(default_factory=lambda: ("eject",))
    timeout_ms: float = DEFAULT_TIMEOUT_MS
    surface_deadband: float = DEFAULT_SURFACE_DEADBAND
    throttle_deadband: float = DEFAULT_THROTTLE_DEADBAND

    def __post_init__(self) -> None:
        object.__setattr__(self, "optional_roles", tuple(sorted(set(self.optional_roles) | {"eject"})))
        missing = set(DEFAULT_CHANNELS) - set(self.channels)
        if missing:
            raise ActuatorMapError(f"missing channel roles: {','.join(sorted(missing))}")
        for name, channel in self.channels.items():
            if name not in DEFAULT_CHANNELS:
                raise ActuatorMapError(f"unknown channel role: {name}")
            _parse_channel(name, channel)
        unknown_optional = set(self.optional_roles) - set(DEFAULT_CHANNELS)
        if unknown_optional:
            raise ActuatorMapError(f"unknown optional channel role: {','.join(sorted(unknown_optional))}")
        if self.timeout_ms <= 0.0 or not math.isfinite(self.timeout_ms):
            raise ActuatorMapError("timeout_ms must be finite and positive")
        if self.surface_deadband < 0.0 or not math.isfinite(self.surface_deadband):
            raise ActuatorMapError("surface_deadband must be finite and non-negative")
        if self.throttle_deadband < 0.0 or not math.isfinite(self.throttle_deadband):
            raise ActuatorMapError("throttle_deadband must be finite and non-negative")

    @classmethod
    def from_json_file(cls, path: Optional[str]) -> "ActuatorChannelMap":
        if path is None:
            return cls()
        with open(path, "r", encoding="utf-8") as map_file:
            data = json.load(map_file)
        if not isinstance(data, dict):
            raise ActuatorMapError("channel map must be a JSON object")

        channels = dict(DEFAULT_CHANNELS)
        raw_channels = data.get("channels", {})
        if not isinstance(raw_channels, dict):
            raise ActuatorMapError("channels must be a JSON object")
        for name, channel in raw_channels.items():
            if name not in DEFAULT_CHANNELS:
                raise ActuatorMapError(f"unknown channel role: {name}")
            channels[name] = _parse_channel(name, channel)

        optional_roles = tuple(sorted(set(_parse_optional_roles(data)) | {"eject"}))
        timeout_ms = _parse_float(data, "timeout_ms", DEFAULT_TIMEOUT_MS)
        surface_deadband = _parse_float(data, "surface_deadband", DEFAULT_SURFACE_DEADBAND)
        throttle_deadband = _parse_float(data, "throttle_deadband", DEFAULT_THROTTLE_DEADBAND)
        if timeout_ms <= 0.0:
            raise ActuatorMapError("timeout_ms must be positive")
        if surface_deadband < 0.0 or throttle_deadband < 0.0:
            raise ActuatorMapError("deadbands must be non-negative")
        return cls(channels, tuple(sorted(optional_roles)), timeout_ms, surface_deadband, throttle_deadband)

    def validate_for_pwm_count(self, pwm_count: int) -> None:
        for name, channel in self.channels.items():
            if channel < 1 or channel > pwm_count:
                if name in self.optional_roles:
                    continue
                raise ActuatorMapError(f"{name} channel {channel} outside packet channel count {pwm_count}")

    def role_is_optional(self, name: str) -> bool:
        return name in self.optional_roles


def _parse_channel(name: str, value: object) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ActuatorMapError(f"{name} channel must be an integer")
    if value < 1:
        raise ActuatorMapError(f"{name} channel must be 1-based and positive")
    return value


def _parse_float(data: Mapping[str, object], name: str, default: float) -> float:
    value = data.get(name, default)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ActuatorMapError(f"{name} must be numeric")
    value = float(value)
    if not math.isfinite(value):
        raise ActuatorMapError(f"{name} must be finite")
    return value


def _parse_optional_roles(data: Mapping[str, object]) -> Tuple[str, ...]:
    raw_roles = data.get("optional_roles", [])
    if not isinstance(raw_roles, list):
        raise ActuatorMapError("optional_roles must be a JSON list")
    roles = []
    for role in raw_roles:
        if not isinstance(role, str):
            raise ActuatorMapError("optional_roles values must be strings")
        if role not in DEFAULT_CHANNELS:
            raise ActuatorMapError(f"unknown optional channel role: {role}")
        roles.append(role)
    return tuple(sorted(set(roles)))


@dataclass(frozen=True)
class NormalizedActuatorCommand:
    pwm: Tuple[int, ...]
    left_elevon: float
    right_elevon: float
    elevator: float
    aileron: float
    rudder: float
    throttle_left: float
    throttle_right: float
    turbojet_throttle: float
    rato: float
    eject: float = 0.0
    stale: bool = False
    warnings: Tuple[str, ...] = ()

    @classmethod
    def neutral(cls) -> "NormalizedActuatorCommand":
        return cls(
            pwm=(),
            left_elevon=0.0,
            right_elevon=0.0,
            elevator=0.0,
            aileron=0.0,
            rudder=0.0,
            throttle_left=0.0,
            throttle_right=0.0,
            turbojet_throttle=0.0,
            rato=0.0,
            eject=0.0,
            stale=True,
        )

    def log_fields(self) -> Dict[str, object]:
        return {
            "act_left_elevon_norm": f"{self.left_elevon:.6f}",
            "act_right_elevon_norm": f"{self.right_elevon:.6f}",
            "act_elevator_norm": f"{self.elevator:.6f}",
            "act_aileron_norm": f"{self.aileron:.6f}",
            "act_rudder_norm": f"{self.rudder:.6f}",
            "act_throttle_left_norm": f"{self.throttle_left:.6f}",
            "act_throttle_right_norm": f"{self.throttle_right:.6f}",
            "act_turbojet_throttle_norm": f"{self.turbojet_throttle:.6f}",
            "act_rato_norm": f"{self.rato:.6f}",
            "act_eject_norm": f"{self.eject:.6f}",
            "act_stale": int(self.stale),
            "act_warnings": ";".join(self.warnings),
            "jsbsim_elevator_property": JSBSIM_PROPERTIES["elevator"],
            "jsbsim_elevator_value": f"{self.elevator:.6f}",
            "jsbsim_aileron_property": JSBSIM_PROPERTIES["aileron"],
            "jsbsim_aileron_value": f"{self.aileron:.6f}",
            "jsbsim_rudder_property": JSBSIM_PROPERTIES["rudder"],
            "jsbsim_rudder_value": f"{self.rudder:.6f}",
            "jsbsim_throttle_property": JSBSIM_PROPERTIES["turbojet_throttle"],
            "jsbsim_throttle_value": f"{self.turbojet_throttle:.6f}",
            "jsbsim_rato_property": JSBSIM_PROPERTIES["rato_throttle"],
            "jsbsim_rato_value": f"{self.rato:.6f}",
        }


class SoftwareActuatorBridge:
    def __init__(self, channel_map: ActuatorChannelMap):
        self.channel_map = channel_map
        self.last_command: Optional[NormalizedActuatorCommand] = None
        self.last_command_monotonic: Optional[float] = None

    def update_from_pwm(
        self,
        pwm: Sequence[int],
        now: Optional[float] = None,
    ) -> NormalizedActuatorCommand:
        now = time.monotonic() if now is None else now
        self.channel_map.validate_for_pwm_count(len(pwm))
        warnings = list(_validate_pwm_values(pwm, self.channel_map))

        def channel_pwm(name: str) -> int:
            channel = self.channel_map.channels[name]
            if self.channel_map.role_is_optional(name) and channel > len(pwm):
                return PWM_LOW
            raw_pwm = pwm[channel - 1]
            if self.channel_map.role_is_optional(name) and raw_pwm == 0:
                return PWM_LOW
            if isinstance(raw_pwm, bool) or not isinstance(raw_pwm, int):
                if name in ("throttle_left", "throttle_right", "rato", "eject"):
                    return PWM_LOW
                return PWM_NEUTRAL
            return int(raw_pwm)

        def packet_pwm_value(value: object) -> int:
            if isinstance(value, bool) or not isinstance(value, int):
                return 0
            return int(value)

        left = normalize_surface_pwm(channel_pwm("left_elevon"), self.channel_map.surface_deadband)
        right = normalize_surface_pwm(channel_pwm("right_elevon"), self.channel_map.surface_deadband)
        rudder = normalize_surface_pwm(channel_pwm("rudder"), self.channel_map.surface_deadband)
        throttle_left = normalize_throttle_pwm(channel_pwm("throttle_left"), self.channel_map.throttle_deadband)
        throttle_right = normalize_throttle_pwm(channel_pwm("throttle_right"), self.channel_map.throttle_deadband)
        rato = normalize_throttle_pwm(channel_pwm("rato"), self.channel_map.throttle_deadband)
        eject = normalize_throttle_pwm(channel_pwm("eject"), self.channel_map.throttle_deadband)

        elevator = clamp((left + right) * 0.5, -1.0, 1.0)
        aileron = clamp((left - right) * 0.5, -1.0, 1.0)
        turbojet_throttle = clamp((throttle_left + throttle_right) * 0.5, 0.0, 1.0)
        command = NormalizedActuatorCommand(
            pwm=tuple(packet_pwm_value(value) for value in pwm),
            left_elevon=left,
            right_elevon=right,
            elevator=elevator,
            aileron=aileron,
            rudder=rudder,
            throttle_left=throttle_left,
            throttle_right=throttle_right,
            turbojet_throttle=turbojet_throttle,
            rato=rato,
            eject=eject,
            stale=False,
            warnings=tuple(warnings),
        )
        self.last_command = command
        self.last_command_monotonic = now
        return command

    def current_command(self, now: Optional[float] = None) -> NormalizedActuatorCommand:
        now = time.monotonic() if now is None else now
        if self.last_command is None or self.last_command_monotonic is None:
            return NormalizedActuatorCommand.neutral()
        age_ms = (now - self.last_command_monotonic) * 1000.0
        if age_ms > self.channel_map.timeout_ms:
            return NormalizedActuatorCommand.neutral()
        return self.last_command


def _validate_pwm_values(pwm: Sequence[int], channel_map: ActuatorChannelMap) -> Iterable[str]:
    mapped_channels = set(channel_map.channels.values())
    optional_zero_channels = {
        channel_map.channels[role]
        for role in channel_map.optional_roles
        if role in channel_map.channels
    }
    for index, value in enumerate(pwm, start=1):
        if index not in mapped_channels:
            continue
        if index in optional_zero_channels and value == 0:
            continue
        if isinstance(value, bool) or not isinstance(value, int):
            yield f"ch{index}:malformed"
            continue
        if value < PWM_HARD_LOW or value > PWM_HARD_HIGH:
            yield f"ch{index}:out_of_range:{value}"


def describe_channel_map(channel_map: ActuatorChannelMap) -> str:
    parts: List[str] = []
    for name in sorted(channel_map.channels):
        optional = "?" if channel_map.role_is_optional(name) else ""
        parts.append(f"{name}=CH{channel_map.channels[name]}{optional}")
    parts.append(f"timeout_ms={channel_map.timeout_ms:g}")
    parts.append(f"surface_deadband={channel_map.surface_deadband:g}")
    parts.append(f"throttle_deadband={channel_map.throttle_deadband:g}")
    return " ".join(parts)
