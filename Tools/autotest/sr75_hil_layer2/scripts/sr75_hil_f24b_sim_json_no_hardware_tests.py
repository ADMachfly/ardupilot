#!/usr/bin/env python3
"""HIL-F24-B: no-hardware SIM_JSON validation harness for the fmuv3
Simulation-on-Hardware bench.

Generates representative SIM_JSON packets locally (via
sr75_sim_json_test_profiles.py) and validates fields/units/rates against
the exact schema traced from libraries/SITL/SIM_JSON.h/.cpp in the
HIL-F24-A/B audits. Also exercises the REAL stale-timeout and
missing-required-field validation code paths already implemented in
sr75_sim_json_responder.py (StateMapper.state_from_csv(),
read_reply_state()) -- not reimplementations of that logic.

No sockets are opened. No PPP, USB, or serial connection is made. No
actuator/PWM command is decoded, constructed, or forwarded anywhere in
this script.
"""
import argparse
import csv
import math
import sys
import tempfile
import time
from pathlib import Path

SIM_JSON_DIR = Path(__file__).resolve().parents[1] / "sim_json"
sys.path.insert(0, str(SIM_JSON_DIR))
import sr75_sim_json_responder as responder  # noqa: E402
import sr75_sim_json_test_profiles as profiles  # noqa: E402

# Field/unit/range expectations, traced from libraries/SITL/SIM_JSON.h's
# keytable[] and SimState.to_json_bytes() (HIL-F24-A/B reports).
REQUIRED_TOP_LEVEL_KEYS = ("timestamp", "imu", "velocity")
REQUIRED_IMU_KEYS = ("gyro", "accel_body")
FORBIDDEN_KEYS_ANY_LEVEL = ("pwm", "servo", "actuator", "throttle", "frame_rate", "frame_count")

GYRO_MAX_RAD_S = math.radians(720.0)     # generous sanity bound, not a spec limit
ACCEL_MAX_MSS = 100.0                    # generous sanity bound, not a spec limit
LAT_RANGE = (-90.0, 90.0)
LON_RANGE = (-180.0, 180.0)


class HarnessFailure(Exception):
    pass


def check_json_schema(state) -> None:
    """Confirm one SimState's serialized JSON matches the required
    SIM_JSON schema and contains no actuator/PWM keys at any level.
    """
    import json
    payload = json.loads(state.to_json_bytes())
    for key in REQUIRED_TOP_LEVEL_KEYS:
        if key not in payload:
            raise HarnessFailure(f"missing required top-level key: {key}")
    for key in REQUIRED_IMU_KEYS:
        if key not in payload["imu"]:
            raise HarnessFailure(f"missing required imu.{key}")

    def _walk_keys(obj):
        if isinstance(obj, dict):
            for k, v in obj.items():
                yield k
                yield from _walk_keys(v)
        elif isinstance(obj, list):
            for v in obj:
                yield from _walk_keys(v)

    found_keys = set(_walk_keys(payload))
    forbidden_found = found_keys & set(FORBIDDEN_KEYS_ANY_LEVEL)
    if forbidden_found:
        raise HarnessFailure(f"actuator/PWM-shaped key(s) found in outgoing state packet: {forbidden_found}")

    gyro = payload["imu"]["gyro"]
    accel = payload["imu"]["accel_body"]
    if len(gyro) != 3 or len(accel) != 3:
        raise HarnessFailure("imu.gyro/accel_body must have exactly 3 components")
    for v in gyro:
        if not math.isfinite(v) or abs(v) > GYRO_MAX_RAD_S:
            raise HarnessFailure(f"gyro component out of sane range (rad/s): {v}")
    for v in accel:
        if not math.isfinite(v) or abs(v) > ACCEL_MAX_MSS:
            raise HarnessFailure(f"accel_body component out of sane range (m/s^2): {v}")
    if not (LAT_RANGE[0] <= payload["latitude"] <= LAT_RANGE[1]):
        raise HarnessFailure(f"latitude out of range: {payload['latitude']}")
    if not (LON_RANGE[0] <= payload["longitude"] <= LON_RANGE[1]):
        raise HarnessFailure(f"longitude out of range: {payload['longitude']}")
    if len(payload["velocity"]) != 3:
        raise HarnessFailure("velocity must have exactly 3 (NED) components")
    if "attitude" not in payload and "quaternion" not in payload:
        raise HarnessFailure("neither attitude nor quaternion present (SIM_JSON requires one)")


def check_profile(name, rows, expected_rate_hz, expected_duration_s, rate_tol_hz=0.5, duration_tol_s=0.2):
    if not rows:
        raise HarnessFailure(f"{name}: produced zero rows")
    for state in rows:
        check_json_schema(state)
        state.validate()  # reuses the real SimState.validate() logic
    duration_s = rows[-1].timestamp_s - rows[0].timestamp_s
    if abs(duration_s - expected_duration_s) > duration_tol_s:
        raise HarnessFailure(f"{name}: duration {duration_s:.3f}s != expected {expected_duration_s:.3f}s")
    if len(rows) >= 2:
        dt_s = (rows[-1].timestamp_s - rows[0].timestamp_s) / (len(rows) - 1)
        rate_hz = 1.0 / dt_s
        if abs(rate_hz - expected_rate_hz) > rate_tol_hz:
            raise HarnessFailure(f"{name}: rate {rate_hz:.2f} Hz != expected {expected_rate_hz:.2f} Hz")
    return {"n_rows": len(rows), "duration_s": duration_s}


def test_all_profiles(duration_s=2.0, rate_hz=20.0):
    results = {}
    for name, fn in profiles.PROFILES.items():
        rows = fn(duration_s=duration_s, rate_hz=rate_hz)
        results[name] = check_profile(name, rows, expected_rate_hz=rate_hz, expected_duration_s=duration_s)
    return results


def test_pitch_sweep_bounds(duration_s=4.0, rate_hz=50.0, max_pitch_deg=20.0):
    rows = profiles.profile_pitch_sweep(duration_s=duration_s, rate_hz=rate_hz, max_pitch_deg=max_pitch_deg)
    peak = max(abs(math.degrees(s.pitch_rad)) for s in rows)
    if peak > max_pitch_deg + 0.5:
        raise HarnessFailure(f"pitch sweep exceeded commanded amplitude: {peak} > {max_pitch_deg}")
    for s in rows:
        if s.roll_rad != 0.0 or s.gyro_rad_s[0] != 0.0 or s.gyro_rad_s[2] != 0.0:
            raise HarnessFailure("pitch sweep must hold roll/yaw and p/r gyro at zero")


def test_roll_sweep_bounds(duration_s=4.0, rate_hz=50.0, max_roll_deg=30.0):
    rows = profiles.profile_roll_sweep(duration_s=duration_s, rate_hz=rate_hz, max_roll_deg=max_roll_deg)
    peak = max(abs(math.degrees(s.roll_rad)) for s in rows)
    if peak > max_roll_deg + 0.5:
        raise HarnessFailure(f"roll sweep exceeded commanded amplitude: {peak} > {max_roll_deg}")


def test_yaw_sweep_monotonic(duration_s=4.0, rate_hz=50.0, rate_deg_s=9.0):
    rows = profiles.profile_yaw_sweep(duration_s=duration_s, rate_hz=rate_hz, rate_deg_s=rate_deg_s)
    prev = None
    for s in rows:
        if prev is not None and s.yaw_rad < prev - 1e-9:
            raise HarnessFailure("yaw sweep must be monotonically increasing")
        prev = s.yaw_rad


def test_altitude_ramp_rate(duration_s=10.0, rate_hz=20.0, climb_rate_mps=1.0):
    rows = profiles.profile_altitude_ramp(duration_s=duration_s, rate_hz=rate_hz, climb_rate_mps=climb_rate_mps)
    total_climb = rows[-1].altitude_m - rows[0].altitude_m
    expected_climb = climb_rate_mps * (rows[-1].timestamp_s - rows[0].timestamp_s)
    if abs(total_climb - expected_climb) > 0.5:
        raise HarnessFailure(f"altitude ramp climb {total_climb} != expected {expected_climb}")
    for s in rows:
        if abs(s.velocity_ned_mps[2] - (-climb_rate_mps)) > 1e-6:
            raise HarnessFailure("altitude ramp vd must equal -climb_rate_mps (NED down-positive)")


def test_stale_timeout():
    """Exercises the REAL read_reply_state()/state_timeout_ms code path in
    sr75_sim_json_responder.py against a genuinely stale temp CSV file --
    not a reimplementation of the staleness check.
    """
    with tempfile.TemporaryDirectory() as tmpdir:
        state_path = Path(tmpdir) / "state.csv"
        with open(state_path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["time_s", "lat_deg", "lon_deg", "alt_m", "vn_mps", "ve_mps", "vd_mps"])
            w.writerow(["0.0", str(profiles.BASE_LAT), str(profiles.BASE_LON), str(profiles.BASE_ALT_M), "0.0", "0.0", "0.0"])
        # back-date the file's mtime so it is already stale
        old_time = time.time() - 5.0
        import os
        os.utime(state_path, (old_time, old_time))

        reader = responder.LatestCSVReader(str(state_path))
        mapper = responder.StateMapper(strict=True)
        mock_source = responder.MockStateSource()
        args = argparse.Namespace(
            mock_state=False, fresh_state_wait_ms=0.0, state_timeout_ms=500.0, allow_state_reuse=False,
        )
        try:
            responder.read_reply_state(args, reader, mapper, mock_source)
        except responder.StateError as exc:
            if "STALE_STATE" not in str(exc):
                raise HarnessFailure(f"expected STALE_STATE error, got: {exc}")
            return
        raise HarnessFailure("stale state file did not raise StateError")


def test_missing_required_fields():
    """Exercises the REAL StateMapper.state_from_csv() required-field
    validation with strict=True.
    """
    mapper = responder.StateMapper(strict=True)
    row_missing_lat = {
        "time_s": "0.0", "lon_deg": str(profiles.BASE_LON), "alt_m": str(profiles.BASE_ALT_M),
        "vn_mps": "0.0", "ve_mps": "0.0", "vd_mps": "0.0",
    }
    try:
        mapper.state_from_csv(row_missing_lat, "sig", time.monotonic())
    except responder.StateError as exc:
        if "latitude" not in str(exc):
            raise HarnessFailure(f"expected a latitude-related StateError, got: {exc}")
        return
    raise HarnessFailure("row missing latitude did not raise StateError under strict=True")


def test_malformed_row_rejected():
    mapper = responder.StateMapper(strict=True)
    row_malformed = {"time_s": "not-a-number", "lat_deg": "32.5", "lon_deg": "74.3", "alt_m": "3000"}
    try:
        mapper.state_from_csv(row_malformed, "sig", time.monotonic())
    except responder.StateError:
        return
    raise HarnessFailure("malformed (non-numeric) row did not raise StateError under strict=True")


def test_no_actuator_command_path():
    """Confirms this harness's own module set never imports the
    incoming-command decode path, and that generated packets never carry
    actuator-shaped keys (the schema check in check_json_schema() already
    covers the second part; this covers the import-boundary part)."""
    forbidden_names = ("decode_control_packet", "SoftwareActuatorBridge", "UDPJSBSimCommandSink")
    profiles_module_globals = set(vars(profiles).keys())
    for name in forbidden_names:
        if name in profiles_module_globals:
            raise HarnessFailure(f"sr75_sim_json_test_profiles.py must not import {name}")


TESTS = [
    ("all_profiles_schema_rate_duration", test_all_profiles),
    ("pitch_sweep_bounds", test_pitch_sweep_bounds),
    ("roll_sweep_bounds", test_roll_sweep_bounds),
    ("yaw_sweep_monotonic", test_yaw_sweep_monotonic),
    ("altitude_ramp_rate", test_altitude_ramp_rate),
    ("stale_timeout", test_stale_timeout),
    ("missing_required_fields", test_missing_required_fields),
    ("malformed_row_rejected", test_malformed_row_rejected),
    ("no_actuator_command_path", test_no_actuator_command_path),
]


def run_all():
    results = {}
    overall_ok = True
    for name, fn in TESTS:
        try:
            detail = fn()
            results[name] = (True, detail)
        except HarnessFailure as exc:
            results[name] = (False, str(exc))
            overall_ok = False
        except Exception as exc:  # noqa: BLE001 -- surface any unexpected error as a FAIL, not a crash
            results[name] = (False, f"unexpected exception: {exc!r}")
            overall_ok = False
    return overall_ok, results


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.parse_args()
    print("=== HIL-F24-B SIM_JSON no-hardware validation harness ===")
    print("No sockets opened. No PPP/USB/serial connection made. No actuator command decoded.")
    overall_ok, results = run_all()
    for name, (ok, detail) in results.items():
        print(f"  {'PASS' if ok else 'FAIL'}: {name} ({detail})")
    print(f"OVERALL: {'PASS' if overall_ok else 'FAIL'}")
    return 0 if overall_ok else 1


if __name__ == "__main__":
    sys.exit(main())
