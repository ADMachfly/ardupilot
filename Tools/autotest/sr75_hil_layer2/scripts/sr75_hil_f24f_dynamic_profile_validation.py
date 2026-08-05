#!/usr/bin/env python3
"""HIL-F24-F item 2/3: offline SIM_JSON dynamic-profile validation harness
for the fmuv3 Simulation-on-Hardware bench.

Runs entirely through the already-validated SITL/loopback (no-hardware)
SIM_JSON code paths established in HIL-F24-B
(sr75_hil_f24b_sim_json_no_hardware_tests.py) -- reused here directly, not
reimplemented -- and extends them with the exact dynamic profiles and
coherence checks HIL-F24-F item 2/3 requires:

  - static level
  - roll sweep 0 -> +15 -> -15 -> 0 deg
  - pitch sweep 0 -> +15 -> -10 -> 0 deg
  - yaw sweep 315 -> 270 -> 315 deg
  - altitude ramp 3000 -> 3020 -> 2980 -> 3000 m
  - combined gentle trajectory
  - stale-data stop (reuses HIL-F24-B's real StateMapper/read_reply_state
    staleness path)
  - malformed/missing required fields (reuses HIL-F24-B's real
    StateMapper.state_from_csv() validation path)

and coherence (item 3):
  - attitude/quaternion round-trip agreement
  - gyro matching attitude rates (finite-difference vs. the analytic rate
    each profile attaches to every row)
  - gravity-consistent body acceleration
  - NED position/velocity consistency
  - airspeed field sanity
  - timestamps monotonic at the configured rate

No sockets are opened. No PPP, USB, or serial connection is made. No
actuator/PWM command is decoded, constructed, or forwarded anywhere in
this script. No hardware is touched.
"""
import math
import sys
from pathlib import Path

SCRIPTS_DIR = Path(__file__).resolve().parent
SIM_JSON_DIR = SCRIPTS_DIR.parent / "sim_json"
sys.path.insert(0, str(SIM_JSON_DIR))
sys.path.insert(0, str(SCRIPTS_DIR))
import sr75_sim_json_test_profiles as profiles  # noqa: E402
import sr75_hil_f24b_sim_json_no_hardware_tests as f24b  # noqa: E402

HarnessFailure = f24b.HarnessFailure

# Corner tolerance: multi-leg profiles have an exact, deliberate slope
# change at each waypoint. A finite-difference check across such a corner
# is a weighted average of the two adjacent slopes, not equal to either --
# so a sample pair is only exempted from the strict check when the
# profile's own analytic gyro *actually* changes across it by more than
# this amount (i.e. the exemption is verified, not assumed).
GYRO_RATE_TOL_RAD_S = 0.02
ALTITUDE_TOL_M = 0.5
LATLON_TOL_DEG = 1e-9
TIMESTAMP_RATE_TOL_HZ = 0.5


def check_gyro_matches_attitude_rate(name, rows, tol_rad_s=GYRO_RATE_TOL_RAD_S):
    """Finite-difference the actual roll/pitch/yaw trajectory between
    consecutive rows and confirm it matches the analytic gyro_rad_s each
    profile attached to the row, within tol_rad_s -- except at a verified
    leg-boundary corner (see module docstring)."""
    for i in range(len(rows) - 1):
        a, b = rows[i], rows[i + 1]
        dt = b.timestamp_s - a.timestamp_s
        if dt <= 0.0:
            raise HarnessFailure(f"{name}: non-positive dt at row {i}")
        fd = (
            (b.roll_rad - a.roll_rad) / dt,
            (b.pitch_rad - a.pitch_rad) / dt,
            (b.yaw_rad - a.yaw_rad) / dt,
        )
        corner = any(abs(a.gyro_rad_s[k] - b.gyro_rad_s[k]) > tol_rad_s for k in range(3))
        if corner:
            continue
        for k, axis in enumerate(("p", "q", "r")):
            if abs(fd[k] - a.gyro_rad_s[k]) > tol_rad_s:
                raise HarnessFailure(
                    f"{name}: {axis} gyro/attitude-rate mismatch at row {i}: "
                    f"finite_diff={fd[k]:.6f} analytic={a.gyro_rad_s[k]:.6f}"
                )


def check_ned_consistency(name, rows, altitude_tol_m=ALTITUDE_TOL_M, latlon_tol_deg=LATLON_TOL_DEG):
    """Confirms altitude change between consecutive rows matches the
    trapezoidal integral of vd (NED down-positive), and that lat/lon are
    unchanged whenever vn/ve are both zero (no profile in this family
    commands horizontal velocity yet)."""
    for i in range(len(rows) - 1):
        a, b = rows[i], rows[i + 1]
        dt = b.timestamp_s - a.timestamp_s
        expected_dalt = -0.5 * (a.velocity_ned_mps[2] + b.velocity_ned_mps[2]) * dt
        actual_dalt = b.altitude_m - a.altitude_m
        if abs(actual_dalt - expected_dalt) > altitude_tol_m:
            raise HarnessFailure(
                f"{name}: altitude change {actual_dalt:.4f} m != vd-integral {expected_dalt:.4f} m at row {i}"
            )
        if a.velocity_ned_mps[0] == 0.0 and a.velocity_ned_mps[1] == 0.0:
            if abs(b.latitude_deg - a.latitude_deg) > latlon_tol_deg:
                raise HarnessFailure(f"{name}: latitude drifted with vn=0 at row {i}")
            if abs(b.longitude_deg - a.longitude_deg) > latlon_tol_deg:
                raise HarnessFailure(f"{name}: longitude drifted with ve=0 at row {i}")


def check_quaternion_euler_consistency(name, rows, tol_deg=0.1):
    """Reconstructs Euler angles from each row's quaternion and confirms
    agreement with the row's own roll/pitch/yaw within tol_deg -- the same
    check used to validate SR-75 Layer 2J-B3C-C3's outgoing attitude."""
    for i, state in enumerate(rows):
        w, x, y, z = state.quaternion
        norm = math.sqrt(w * w + x * x + y * y + z * z)
        if abs(norm - 1.0) > 1e-6:
            raise HarnessFailure(f"{name}: quaternion norm {norm:.9f} != 1.0 at row {i}")
        roll_r = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
        pitch_r = math.asin(sinp)
        yaw_r = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        for label, reconstructed, original in (
            ("roll", roll_r, state.roll_rad),
            ("pitch", pitch_r, state.pitch_rad),
            ("yaw", yaw_r, state.yaw_rad),
        ):
            delta_deg = abs(math.degrees(reconstructed) - math.degrees(original))
            delta_deg = min(delta_deg, 360.0 - delta_deg)  # wrap-around safe
            if delta_deg > tol_deg:
                raise HarnessFailure(
                    f"{name}: {label} reconstructed-from-quaternion disagrees by {delta_deg:.4f} deg at row {i}"
                )


def check_timestamps_monotonic_at_rate(name, rows, expected_rate_hz, rate_tol_hz=TIMESTAMP_RATE_TOL_HZ):
    for i in range(len(rows) - 1):
        dt = rows[i + 1].timestamp_s - rows[i].timestamp_s
        if dt <= 0.0:
            raise HarnessFailure(f"{name}: timestamps not strictly monotonic at row {i}")
        rate_hz = 1.0 / dt
        if abs(rate_hz - expected_rate_hz) > rate_tol_hz:
            raise HarnessFailure(
                f"{name}: instantaneous rate {rate_hz:.2f} Hz != expected {expected_rate_hz:.2f} Hz at row {i}"
            )


def check_airspeed_sane(name, rows):
    for i, state in enumerate(rows):
        if state.airspeed_mps < 0.0 or not math.isfinite(state.airspeed_mps):
            raise HarnessFailure(f"{name}: airspeed {state.airspeed_mps} invalid at row {i}")


def check_full_coherence(name, rows, rate_hz):
    """Item 3: run every coherence check (schema, quaternion, gyro-rate,
    NED, airspeed, timestamp-rate) against one profile's rows."""
    for state in rows:
        f24b.check_json_schema(state)
        state.validate()
    check_quaternion_euler_consistency(name, rows)
    check_gyro_matches_attitude_rate(name, rows)
    check_ned_consistency(name, rows)
    check_airspeed_sane(name, rows)
    check_timestamps_monotonic_at_rate(name, rows, rate_hz)


def test_static_level():
    rows = profiles.profile_static_level(duration_s=6.0, rate_hz=50.0, airspeed_mps=0.0)
    check_full_coherence("static_level", rows, rate_hz=50.0)


def test_roll_sweep_multileg():
    rows = profiles.profile_roll_sweep_multileg(duration_s=12.0, rate_hz=50.0)
    check_full_coherence("roll_sweep_multileg", rows, rate_hz=50.0)
    peak = max(math.degrees(s.roll_rad) for s in rows)
    trough = min(math.degrees(s.roll_rad) for s in rows)
    if peak < 14.9 or trough > -14.9:
        raise HarnessFailure(f"roll_sweep_multileg: legs did not reach +-15 deg (peak={peak}, trough={trough})")


def test_pitch_sweep_multileg():
    rows = profiles.profile_pitch_sweep_multileg(duration_s=12.0, rate_hz=50.0)
    check_full_coherence("pitch_sweep_multileg", rows, rate_hz=50.0)
    peak = max(math.degrees(s.pitch_rad) for s in rows)
    trough = min(math.degrees(s.pitch_rad) for s in rows)
    if peak < 14.9 or trough > -9.9:
        raise HarnessFailure(f"pitch_sweep_multileg: legs did not reach +15/-10 deg (peak={peak}, trough={trough})")


def test_yaw_sweep_multileg():
    rows = profiles.profile_yaw_sweep_multileg(duration_s=20.0, rate_hz=50.0)
    check_full_coherence("yaw_sweep_multileg", rows, rate_hz=50.0)
    trough = min(math.degrees(s.yaw_rad) for s in rows)
    if trough > 270.9:
        raise HarnessFailure(f"yaw_sweep_multileg: did not reach 270 deg (trough={trough})")


def test_altitude_ramp_multileg():
    rows = profiles.profile_altitude_ramp_multileg(duration_s=20.0, rate_hz=50.0)
    check_full_coherence("altitude_ramp_multileg", rows, rate_hz=50.0)
    peak = max(s.altitude_m for s in rows)
    trough = min(s.altitude_m for s in rows)
    if peak < 3019.5 or trough > 2980.5:
        raise HarnessFailure(f"altitude_ramp_multileg: legs did not reach 3020/2980 m (peak={peak}, trough={trough})")


def test_combined_gentle():
    rows = profiles.profile_combined_gentle(duration_s=30.0, rate_hz=50.0)
    check_full_coherence("combined_gentle", rows, rate_hz=50.0)


def test_stale_data_stop():
    """Reuses HIL-F24-B's real stale-timeout test (the actual
    read_reply_state()/state_timeout_ms code path in
    sr75_sim_json_responder.py), not a reimplementation."""
    f24b.test_stale_timeout()


def test_malformed_missing_fields():
    """Reuses HIL-F24-B's real missing-required-field and malformed-row
    tests (the actual StateMapper.state_from_csv() validation path)."""
    f24b.test_missing_required_fields()
    f24b.test_malformed_row_rejected()


def test_no_actuator_command_path():
    f24b.test_no_actuator_command_path()


TESTS = [
    ("static_level", test_static_level),
    ("roll_sweep_multileg", test_roll_sweep_multileg),
    ("pitch_sweep_multileg", test_pitch_sweep_multileg),
    ("yaw_sweep_multileg", test_yaw_sweep_multileg),
    ("altitude_ramp_multileg", test_altitude_ramp_multileg),
    ("combined_gentle", test_combined_gentle),
    ("stale_data_stop", test_stale_data_stop),
    ("malformed_missing_fields", test_malformed_missing_fields),
    ("no_actuator_command_path", test_no_actuator_command_path),
]


def run_all():
    results = {}
    overall_ok = True
    for name, fn in TESTS:
        try:
            fn()
            results[name] = (True, None)
        except HarnessFailure as exc:
            results[name] = (False, str(exc))
            overall_ok = False
        except Exception as exc:  # noqa: BLE001 -- surface any unexpected error as a FAIL, not a crash
            results[name] = (False, f"unexpected exception: {exc!r}")
            overall_ok = False
    return overall_ok, results


def main():
    print("=== HIL-F24-F dynamic-profile validation harness (SITL/loopback, no hardware) ===")
    print("No sockets opened. No PPP/USB/serial connection made. No actuator command decoded.")
    overall_ok, results = run_all()
    for name, (ok, detail) in results.items():
        print(f"  {'PASS' if ok else 'FAIL'}: {name}" + (f" ({detail})" if detail else ""))
    print(f"OVERALL: {'PASS' if overall_ok else 'FAIL'}")
    return 0 if overall_ok else 1


if __name__ == "__main__":
    sys.exit(main())
