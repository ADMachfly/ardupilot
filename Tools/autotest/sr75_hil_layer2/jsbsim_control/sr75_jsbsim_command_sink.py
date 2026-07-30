#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""UDP command sink for the SR-75 JSBSim software-only actuator bridge."""

import math
import socket
import time
from dataclasses import dataclass
from typing import Optional, Tuple


SURFACE_LOW = -1.0
SURFACE_HIGH = 1.0
THROTTLE_LOW = 0.0
THROTTLE_HIGH = 1.0

JSBSIM_INPUT_PROPERTIES = (
    "fcs/elevator-cmd-norm",
    "fcs/aileron-cmd-norm",
    "fcs/rudder-cmd-norm",
    "fcs/turbojet-throttle-cmd-norm",
    "fcs/rato-throttle-cmd-norm",
    "propulsion/tank[2]/contents-lbs",
)


class JSBSimCommandError(Exception):
    """Raised when a JSBSim command target or command value is invalid."""


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def parse_command_target(endpoint: str) -> Tuple[str, int]:
    target = endpoint
    if target.startswith("udp://"):
        target = target[len("udp://"):]
    elif target.startswith("udp:"):
        target = target[len("udp:"):]
    if ":" not in target:
        raise JSBSimCommandError("JSBSim command target must be udp:HOST:PORT")
    host, port_text = target.rsplit(":", 1)
    if not host:
        raise JSBSimCommandError("JSBSim command target host is empty")
    try:
        port = int(port_text)
    except ValueError as exc:
        raise JSBSimCommandError(f"invalid JSBSim command target port: {port_text}") from exc
    if port <= 0 or port > 65535:
        raise JSBSimCommandError(f"JSBSim command target port out of range: {port}")
    return host, port


@dataclass(frozen=True)
class JSBSimActuatorCommand:
    timestamp_s: float
    elevator: float
    aileron: float
    rudder: float
    turbojet_throttle: float
    rato_throttle: float
    dry_booster_weight_lbs: float = 22.046226
    stale: bool = False

    @classmethod
    def neutral(
        cls,
        timestamp_s: Optional[float] = None,
        stale: bool = True,
        dry_booster_weight_lbs: float = 22.046226,
    ) -> "JSBSimActuatorCommand":
        timestamp = time.monotonic() if timestamp_s is None else timestamp_s
        return cls(timestamp, 0.0, 0.0, 0.0, 0.0, 0.0, dry_booster_weight_lbs, stale)

    def validated(self) -> "JSBSimActuatorCommand":
        values = (
            self.timestamp_s,
            self.elevator,
            self.aileron,
            self.rudder,
            self.turbojet_throttle,
            self.rato_throttle,
            self.dry_booster_weight_lbs,
        )
        if not all(math.isfinite(value) for value in values):
            raise JSBSimCommandError("JSBSim command contains non-finite values")
        if self.timestamp_s < 0.0:
            raise JSBSimCommandError("JSBSim command timestamp is negative")
        return JSBSimActuatorCommand(
            timestamp_s=self.timestamp_s,
            elevator=clamp(self.elevator, SURFACE_LOW, SURFACE_HIGH),
            aileron=clamp(self.aileron, SURFACE_LOW, SURFACE_HIGH),
            rudder=clamp(self.rudder, SURFACE_LOW, SURFACE_HIGH),
            turbojet_throttle=clamp(self.turbojet_throttle, THROTTLE_LOW, THROTTLE_HIGH),
            rato_throttle=clamp(self.rato_throttle, THROTTLE_LOW, THROTTLE_HIGH),
            dry_booster_weight_lbs=clamp(self.dry_booster_weight_lbs, 0.0, 22.046226),
            stale=self.stale,
        )

    def packet_values(self) -> Tuple[float, float, float, float, float, float, float]:
        command = self.validated()
        return (
            command.timestamp_s,
            command.elevator,
            command.aileron,
            command.rudder,
            command.turbojet_throttle,
            command.rato_throttle,
            command.dry_booster_weight_lbs,
        )

    def packet_text(self) -> str:
        return ",".join(f"{value:.9f}" for value in self.packet_values())


class UDPJSBSimCommandSink:
    def __init__(self, endpoint: str):
        host, port = parse_command_target(endpoint)
        self.endpoint = (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._last_timestamp_s: Optional[float] = None
        self._last_sent_stale = False

    def close(self) -> None:
        self.sock.close()

    def _monotonic_command(self, command: JSBSimActuatorCommand) -> JSBSimActuatorCommand:
        command = command.validated()
        timestamp_s = command.timestamp_s
        if self._last_timestamp_s is not None and timestamp_s <= self._last_timestamp_s:
            timestamp_s = self._last_timestamp_s + 1.0e-6
        self._last_timestamp_s = timestamp_s
        return JSBSimActuatorCommand(
            timestamp_s=timestamp_s,
            elevator=command.elevator,
            aileron=command.aileron,
            rudder=command.rudder,
            turbojet_throttle=command.turbojet_throttle,
            rato_throttle=command.rato_throttle,
            dry_booster_weight_lbs=command.dry_booster_weight_lbs,
            stale=command.stale,
        )

    def send(self, command: JSBSimActuatorCommand) -> int:
        command = self._monotonic_command(command)
        payload = command.packet_text().encode("ascii")
        sent = self.sock.sendto(payload, self.endpoint)
        self._last_sent_stale = command.stale
        return sent

    def send_stale_once(
        self,
        timestamp_s: Optional[float] = None,
        dry_booster_weight_lbs: float = 22.046226,
    ) -> Optional[int]:
        if self._last_sent_stale:
            return None
        return self.send(
            JSBSimActuatorCommand.neutral(
                timestamp_s=timestamp_s,
                stale=True,
                dry_booster_weight_lbs=dry_booster_weight_lbs,
            )
        )
