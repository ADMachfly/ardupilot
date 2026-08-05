#!/usr/bin/env python3
"""HIL-F24-B: pure SIM_JSON test-profile generators for the fmuv3
Simulation-on-Hardware bench.

Reuses the exact SimState schema/serialization already validated in
sr75_sim_json_responder.py (SimState.to_json_bytes() is the same function
the live responder uses) -- no packet format is reinvented here.

No sockets, no hardware, no PPP, no actuator command path. Every function
in this module only ever PRODUCES outgoing sensor-state packets; none of
them decode, forward, or otherwise touch incoming actuator/PWM commands
(that machinery -- decode_control_packet()/SoftwareActuatorBridge -- lives
entirely in sr75_sim_json_responder.py and is never imported here).
"""
import math
from typing import List, Sequence, Tuple

from sr75_sim_json_responder import (
    SimState,
    euler_to_quaternion,
    gravity_body_mss,
)

# Same validated release point used throughout the SR-75 HIL task family.
BASE_LAT = 32.5378147
BASE_LON = 74.3661871
BASE_ALT_M = 3000.0
BASE_YAW_DEG = 315.091444

DEFAULT_RATE_HZ = 50.0


def make_state(
    t_s: float, lat_deg: float = BASE_LAT, lon_deg: float = BASE_LON,
    alt_m: float = BASE_ALT_M, roll_rad: float = 0.0, pitch_rad: float = 0.0,
    yaw_rad: float = 0.0, gyro_rad_s: Tuple[float, float, float] = (0.0, 0.0, 0.0),
    velocity_ned_mps: Tuple[float, float, float] = (0.0, 0.0, 0.0),
    airspeed_mps: float = 0.0,
) -> SimState:
    """Build one SimState sample. accel_body_mss is always the stationary
    gravity vector resolved into body axes for the given roll/pitch (via
    the same gravity_body_mss() the live responder uses) -- these are
    kinematic *attitude/rate* profiles, not full 6-DOF dynamics; that is
    sufficient to exercise SIM_JSON field/unit/rate correctness and
    downstream AHRS2/EKF response without needing a physics engine.
    """
    return SimState(
        timestamp_s=t_s,
        source_timestamp_s=t_s,
        latitude_deg=lat_deg,
        longitude_deg=lon_deg,
        altitude_m=alt_m,
        roll_rad=roll_rad,
        pitch_rad=pitch_rad,
        yaw_rad=yaw_rad,
        quaternion=euler_to_quaternion(roll_rad, pitch_rad, yaw_rad),
        gyro_rad_s=gyro_rad_s,
        accel_body_mss=gravity_body_mss(roll_rad, pitch_rad, yaw_rad),
        velocity_ned_mps=velocity_ned_mps,
        airspeed_mps=airspeed_mps,
        row_signature=f"profile:{t_s:.6f}",
        row_monotonic_time=t_s,
    )


def _timebase(duration_s: float, rate_hz: float) -> List[float]:
    dt = 1.0 / rate_hz
    n_steps = max(1, int(round(duration_s / dt)))
    return [round(i * dt, 6) for i in range(n_steps)]


def profile_static_level(
    duration_s: float = 5.0, rate_hz: float = DEFAULT_RATE_HZ, airspeed_mps: float = 0.0,
) -> List[SimState]:
    """Deterministic static-level test packet: wings-level, motionless,
    zero body rates, gravity-consistent accelerometer, constant altitude/
    position, zero NED velocity, and a fixed (default 0.0, i.e. stationary
    ground test) airspeed value -- the simplest possible coherent state.
    HIL-F24-C item 7 default invocation: duration_s=30.0, rate_hz=50..100.
    """
    return [make_state(t, airspeed_mps=airspeed_mps) for t in _timebase(duration_s, rate_hz)]


def profile_pitch_sweep(
    duration_s: float = 10.0, rate_hz: float = DEFAULT_RATE_HZ, max_pitch_deg: float = 20.0,
) -> List[SimState]:
    """Sinusoidal pitch sweep, one full cycle over duration_s, roll/yaw
    held level. Rate gyro (q) is the analytic derivative of pitch.
    """
    rows = []
    omega = 2.0 * math.pi / duration_s
    for t in _timebase(duration_s, rate_hz):
        pitch = math.radians(max_pitch_deg) * math.sin(omega * t)
        q = math.radians(max_pitch_deg) * omega * math.cos(omega * t)
        rows.append(make_state(t, pitch_rad=pitch, gyro_rad_s=(0.0, q, 0.0)))
    return rows


def profile_roll_sweep(
    duration_s: float = 10.0, rate_hz: float = DEFAULT_RATE_HZ, max_roll_deg: float = 30.0,
) -> List[SimState]:
    """Sinusoidal roll sweep, one full cycle over duration_s, pitch/yaw
    held level. Rate gyro (p) is the analytic derivative of roll.
    """
    rows = []
    omega = 2.0 * math.pi / duration_s
    for t in _timebase(duration_s, rate_hz):
        roll = math.radians(max_roll_deg) * math.sin(omega * t)
        p = math.radians(max_roll_deg) * omega * math.cos(omega * t)
        rows.append(make_state(t, roll_rad=roll, gyro_rad_s=(p, 0.0, 0.0)))
    return rows


def profile_yaw_sweep(
    duration_s: float = 20.0, rate_hz: float = DEFAULT_RATE_HZ, rate_deg_s: float = 9.0,
) -> List[SimState]:
    """Constant-rate yaw sweep (matches the ~2.25 deg/s-class rate limits
    used elsewhere in this task family, scaled up slightly since this is
    a pure kinematic/no-EKF-fusion-timing-constrained test), roll/pitch
    held level, wrapped to 0..360 deg via euler_to_quaternion/SimState's
    own yaw_rad field (not pre-wrapped here -- downstream consumers wrap).
    """
    rows = []
    for t in _timebase(duration_s, rate_hz):
        yaw = math.radians(BASE_YAW_DEG) + math.radians(rate_deg_s) * t
        r = math.radians(rate_deg_s)
        rows.append(make_state(t, yaw_rad=yaw, gyro_rad_s=(0.0, 0.0, r)))
    return rows


def profile_altitude_ramp(
    duration_s: float = 20.0, rate_hz: float = DEFAULT_RATE_HZ, climb_rate_mps: float = 1.0,
) -> List[SimState]:
    """Constant-rate altitude climb, level attitude, matching the
    NED-down-positive convention used throughout this task family
    (vd = -climb_rate_mps while climbing)."""
    rows = []
    for t in _timebase(duration_s, rate_hz):
        alt = BASE_ALT_M + climb_rate_mps * t
        rows.append(make_state(t, alt_m=alt, velocity_ned_mps=(0.0, 0.0, -climb_rate_mps)))
    return rows


PROFILES = {
    "static_level": profile_static_level,
    "pitch_sweep": profile_pitch_sweep,
    "roll_sweep": profile_roll_sweep,
    "yaw_sweep": profile_yaw_sweep,
    "altitude_ramp": profile_altitude_ramp,
}


def _piecewise_linear(t_s: float, waypoints: List[Tuple[float, float]]) -> Tuple[float, float]:
    """Given waypoints [(time_s, value), ...] sorted by time_s, return
    (value, rate) at t_s via piecewise-linear interpolation between the
    two bracketing waypoints. rate is the exact constant slope of the
    active segment -- used so gyro/velocity fields are the analytic
    derivative of the attitude/altitude profile, not a finite-difference
    approximation of it.
    """
    if t_s <= waypoints[0][0]:
        t0, v0 = waypoints[0]
        t1, v1 = waypoints[1]
        rate = (v1 - v0) / (t1 - t0)
        return v0, rate
    for i in range(len(waypoints) - 1):
        t0, v0 = waypoints[i]
        t1, v1 = waypoints[i + 1]
        if t0 <= t_s <= t1:
            rate = (v1 - v0) / (t1 - t0)
            return v0 + rate * (t_s - t0), rate
    t0, v0 = waypoints[-2]
    t1, v1 = waypoints[-1]
    rate = (v1 - v0) / (t1 - t0)
    return v1, rate


def _leg_waypoints(duration_s: float, leg_values: Sequence[float]) -> List[Tuple[float, float]]:
    n_legs = len(leg_values) - 1
    if n_legs < 1:
        raise ValueError("leg_values must have at least two points (one leg)")
    leg_duration = duration_s / n_legs
    return [(i * leg_duration, v) for i, v in enumerate(leg_values)]


def profile_roll_sweep_multileg(
    duration_s: float = 12.0, rate_hz: float = DEFAULT_RATE_HZ,
    leg_values_deg: Sequence[float] = (0.0, 15.0, -15.0, 0.0),
) -> List[SimState]:
    """HIL-F24-F explicit multi-leg roll profile: equal-duration straight
    legs between each consecutive pair of leg_values_deg (default
    0 -> +15 -> -15 -> 0 deg), pitch/yaw held level. p (roll rate) is the
    exact analytic derivative of the active linear segment.
    """
    waypoints = [(t, math.radians(v)) for t, v in _leg_waypoints(duration_s, leg_values_deg)]
    rows = []
    for t in _timebase(duration_s, rate_hz):
        roll, p = _piecewise_linear(t, waypoints)
        rows.append(make_state(t, roll_rad=roll, gyro_rad_s=(p, 0.0, 0.0)))
    return rows


def profile_pitch_sweep_multileg(
    duration_s: float = 12.0, rate_hz: float = DEFAULT_RATE_HZ,
    leg_values_deg: Sequence[float] = (0.0, 15.0, -10.0, 0.0),
) -> List[SimState]:
    """HIL-F24-F explicit multi-leg pitch profile: 0 -> +15 -> -10 -> 0 deg
    by default, roll/yaw held level. q (pitch rate) is the exact analytic
    derivative of the active linear segment."""
    waypoints = [(t, math.radians(v)) for t, v in _leg_waypoints(duration_s, leg_values_deg)]
    rows = []
    for t in _timebase(duration_s, rate_hz):
        pitch, q = _piecewise_linear(t, waypoints)
        rows.append(make_state(t, pitch_rad=pitch, gyro_rad_s=(0.0, q, 0.0)))
    return rows


def profile_yaw_sweep_multileg(
    duration_s: float = 20.0, rate_hz: float = DEFAULT_RATE_HZ,
    leg_values_deg: Sequence[float] = (315.0, 270.0, 315.0),
) -> List[SimState]:
    """HIL-F24-F explicit multi-leg yaw profile: 315 -> 270 -> 315 deg by
    default, roll/pitch held level. r (yaw rate) is the exact analytic
    derivative of the active linear segment."""
    waypoints = _leg_waypoints(duration_s, [math.radians(v) for v in leg_values_deg])
    rows = []
    for t in _timebase(duration_s, rate_hz):
        yaw, r = _piecewise_linear(t, waypoints)
        rows.append(make_state(t, yaw_rad=yaw, gyro_rad_s=(0.0, 0.0, r)))
    return rows


def profile_altitude_ramp_multileg(
    duration_s: float = 20.0, rate_hz: float = DEFAULT_RATE_HZ,
    leg_values_m: Sequence[float] = (3000.0, 3020.0, 2980.0, 3000.0),
) -> List[SimState]:
    """HIL-F24-F explicit multi-leg altitude profile: 3000 -> 3020 -> 2980
    -> 3000 m by default, level attitude. vd (NED down velocity) is the
    exact analytic derivative of altitude, negated per the NED down-
    positive convention used throughout this task family (climbing ==
    vd < 0)."""
    waypoints = _leg_waypoints(duration_s, leg_values_m)
    rows = []
    for t in _timebase(duration_s, rate_hz):
        alt, climb_rate = _piecewise_linear(t, waypoints)
        rows.append(make_state(t, alt_m=alt, velocity_ned_mps=(0.0, 0.0, -climb_rate)))
    return rows


def profile_combined_gentle(
    duration_s: float = 30.0, rate_hz: float = DEFAULT_RATE_HZ,
    roll_leg_values_deg: Sequence[float] = (0.0, 5.0, 0.0),
    pitch_leg_values_deg: Sequence[float] = (0.0, 3.0, 0.0),
    yaw_leg_values_deg: Sequence[float] = (315.0, 320.0, 315.0),
    alt_leg_values_m: Sequence[float] = (3000.0, 3010.0, 3000.0),
    airspeed_mps: float = 30.0,
) -> List[SimState]:
    """HIL-F24-F combined gentle trajectory: small, simultaneous roll/
    pitch/yaw/altitude excursions (default +-5 deg roll, +-3 deg pitch,
    +-5 deg yaw, +-10 m altitude) over one shared timebase -- deliberately
    "gentle" (small amplitude, low rate) since this exercises coherence of
    all axes moving together, not sweep-amplitude limits. Each axis's rate
    field is the exact analytic derivative of its own piecewise-linear leg,
    matching the single-axis multileg profiles above."""
    roll_wp = [(t, math.radians(v)) for t, v in _leg_waypoints(duration_s, roll_leg_values_deg)]
    pitch_wp = [(t, math.radians(v)) for t, v in _leg_waypoints(duration_s, pitch_leg_values_deg)]
    yaw_wp = _leg_waypoints(duration_s, [math.radians(v) for v in yaw_leg_values_deg])
    alt_wp = _leg_waypoints(duration_s, alt_leg_values_m)
    rows = []
    for t in _timebase(duration_s, rate_hz):
        roll, p = _piecewise_linear(t, roll_wp)
        pitch, q = _piecewise_linear(t, pitch_wp)
        yaw, r = _piecewise_linear(t, yaw_wp)
        alt, climb_rate = _piecewise_linear(t, alt_wp)
        rows.append(make_state(
            t, alt_m=alt, roll_rad=roll, pitch_rad=pitch, yaw_rad=yaw,
            gyro_rad_s=(p, q, r), velocity_ned_mps=(0.0, 0.0, -climb_rate),
            airspeed_mps=airspeed_mps,
        ))
    return rows


PROFILES_F24F = {
    "static_level": profile_static_level,
    "roll_sweep_multileg": profile_roll_sweep_multileg,
    "pitch_sweep_multileg": profile_pitch_sweep_multileg,
    "yaw_sweep_multileg": profile_yaw_sweep_multileg,
    "altitude_ramp_multileg": profile_altitude_ramp_multileg,
    "combined_gentle": profile_combined_gentle,
}
