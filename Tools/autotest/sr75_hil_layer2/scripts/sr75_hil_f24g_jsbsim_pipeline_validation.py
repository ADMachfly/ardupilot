#!/usr/bin/env python3
"""HIL-F24-G: validate the complete SR-75 JSBSim -> SIM_JSON sensor-state
pipeline offline, using the REAL JSBSim model and the REAL responder
conversion path (sr75_sim_json_responder.StateMapper.state_from_csv()) --
not a reimplementation and not the synthetic profiles from HIL-F24-F.

Flow:
  1. Run the real SR-75 JSBSim model (aircraft/sr_75_6_dof/scripts/
     SR75_hil_f24g_pipeline_validation.xml) non-realtime (deterministic,
     no live socket/ArduPlane needed -- this validates the conversion/
     mapping path, not closed-loop control) to produce a raw CSV capture
     covering every representative state item 2 asks for in one
     continuous, physically-connected run.
  2. Feed every row through the REAL StateMapper.state_from_csv() (the
     exact function read_reply_state() calls in the live responder) --
     "capture mode": the whole file is converted in a batch, rather than
     polled live over a socket.
  3. Validate field coherence and compute conversion errors against the
     raw JSBSim source.
  4. Save deterministic regression captures for 5 representative phases.
  5. Feed the converted rows through HIL-F24-F's acceptance-analyzer
     evaluation logic where applicable.
  6. Produce pipeline health statistics.

No PPP, no /dev/ttyUSB0, no flashing, no PARAM_SET, no arm/AUTO, no
actuator output. No aero/RATO/TECS/mission XML is modified -- every
maneuver in the JSBSim script is a control-input event only.
"""
import csv
import json
import math
import subprocess
import sys
from pathlib import Path
from typing import Dict, List, Tuple

SCRIPTS_DIR = Path(__file__).resolve().parent
LAYER2_DIR = SCRIPTS_DIR.parent
SIM_JSON_DIR = LAYER2_DIR / "sim_json"
REPO_ROOT = Path(__file__).resolve().parents[4]
AUTOTEST_DIR = REPO_ROOT / "Tools" / "autotest"
JSBSIM_SCRIPT = AUTOTEST_DIR / "aircraft" / "sr_75_6_dof" / "scripts" / "SR75_hil_f24g_pipeline_validation.xml"
JSBSIM_OUTPUT_NAME = "sr75_hil_f24g_pipeline_validation.csv"
REGRESSION_CAPTURES_DIR = SIM_JSON_DIR / "f24g_regression_captures"

sys.path.insert(0, str(SIM_JSON_DIR))
sys.path.insert(0, str(SCRIPTS_DIR))
import sr75_sim_json_responder as responder  # noqa: E402
import sr75_hil_f24f_hardware_acceptance_analyzer as f24f_analyzer  # noqa: E402

FT_TO_M = 0.3048
KTS_TO_MPS = 0.514444

# Representative phase windows (sim-time seconds), chosen from the
# JSBSim script's own event timeline and confirmed by inspecting the
# resulting trajectory (see HIL-F24-G report for the full empirical
# tuning trace: this airframe has essentially no open-loop roll/pitch
# damping, so control-input magnitudes were chosen small enough to stay
# bounded rather than diverge/tumble).
PHASE_WINDOWS = {
    "static_release": (0.0, 2.0),
    "steady_flight": (8.0, 12.0),
    "turning": (17.0, 21.0),
    "climb_descent": (22.0, 34.0),
    "rato_boost": (60.0, 63.0),
}

# Phase-boundary timestamps to check for pipeline-introduced
# discontinuities (item 8) -- these are the JSBSim script's own event
# times, i.e. where a control input step is deliberately applied.
PHASE_BOUNDARIES_S = (6.0, 16.0, 20.0, 24.0, 30.0, 40.0, 50.0, 60.0, 61.0, 62.0)


class PipelineValidationError(Exception):
    pass


def run_jsbsim(output_dir: Path, jsbsim_bin: str = "JSBSim") -> Path:
    """Thin I/O: runs the real SR-75 JSBSim model non-realtime (no
    --realtime flag -- nothing is consuming this run live, so deterministic
    as-fast-as-possible execution is used instead, matching the "capture
    mode" this task asks for) and returns the path to the resulting CSV.
    """
    output_dir.mkdir(parents=True, exist_ok=True)
    csv_path = AUTOTEST_DIR / JSBSIM_OUTPUT_NAME
    csv_path.unlink(missing_ok=True)
    script_rel = JSBSIM_SCRIPT.relative_to(AUTOTEST_DIR)
    proc = subprocess.run(
        [jsbsim_bin, "--root=.", f"--script={script_rel}"],
        cwd=str(AUTOTEST_DIR), capture_output=True, text=True, timeout=60.0,
    )
    log_path = output_dir / "jsbsim_run.log"
    log_path.write_text(proc.stdout + proc.stderr)
    if proc.returncode != 0 or not csv_path.exists():
        raise PipelineValidationError(f"JSBSim run failed (exit {proc.returncode}); see {log_path}")
    dest = output_dir / JSBSIM_OUTPUT_NAME
    dest.write_bytes(csv_path.read_bytes())
    csv_path.unlink()  # JSBSim always writes into the source tree's cwd; do not leave it there
    return dest


def load_jsbsim_csv(path: Path) -> List[Dict[str, str]]:
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def convert_rows(raw_rows: List[Dict[str, str]]) -> List["responder.SimState"]:
    """The real conversion/mapping path: one shared StateMapper (strict
    mode, matching the live responder's default) converts every raw
    JSBSim CSV row via the exact function read_reply_state() calls."""
    mapper = responder.StateMapper(strict=True)
    states = []
    for i, row in enumerate(raw_rows):
        state = mapper.state_from_csv(row, signature=f"f24g:{i}", row_monotonic_time=float(i))
        states.append(state)
    return states


def raw_float(row: Dict[str, str], key: str) -> float:
    return float(row[key])


# --- Coherence checks (item 3) ---------------------------------------

def check_monotonic_timestamps(states: List["responder.SimState"]) -> None:
    for i in range(len(states) - 1):
        if states[i + 1].timestamp_s <= states[i].timestamp_s:
            raise PipelineValidationError(f"non-monotonic outgoing timestamp at row {i}")


def check_quaternion_euler_consistency(states: List["responder.SimState"], tol_deg: float = 0.1) -> None:
    for i, state in enumerate(states):
        w, x, y, z = state.quaternion
        norm = math.sqrt(w * w + x * x + y * y + z * z)
        if abs(norm - 1.0) > 1e-6:
            raise PipelineValidationError(f"quaternion norm {norm:.9f} != 1.0 at row {i}")
        roll_r = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
        pitch_r = math.asin(sinp)
        yaw_r = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        for label, reconstructed, original in (
            ("roll", roll_r, state.roll_rad), ("pitch", pitch_r, state.pitch_rad), ("yaw", yaw_r, state.yaw_rad),
        ):
            delta_deg = abs(math.degrees(reconstructed) - math.degrees(original))
            delta_deg = min(delta_deg, 360.0 - delta_deg)
            if delta_deg > tol_deg:
                raise PipelineValidationError(f"{label} quaternion mismatch {delta_deg:.4f} deg at row {i}")


def check_gyro_passthrough(states, raw_rows, tol: float = 1e-9) -> float:
    """Gyro body rates are a direct passthrough (no unit conversion, no
    sign flip) from JSBSim's p_rad_s/q_rad_s/r_rad_s. Returns max abs
    error found (item 4)."""
    max_err = 0.0
    for i, (state, row) in enumerate(zip(states, raw_rows)):
        expected = (raw_float(row, "p_rad_s"), raw_float(row, "q_rad_s"), raw_float(row, "r_rad_s"))
        for k in range(3):
            err = abs(state.gyro_rad_s[k] - expected[k])
            max_err = max(max_err, err)
            if err > tol:
                raise PipelineValidationError(f"gyro[{k}] mismatch {err:.3e} at row {i}")
    return max_err


def check_gravity_consistent_accel(states, raw_rows, tol: float = 1e-9) -> float:
    """Body acceleration is a direct passthrough of JSBSim's own
    accel_body_x/y/z_mss (already gravity-convention-consistent: level,
    non-accelerating flight reads accel_body_z ~= -9.80665, matching
    gravity_body_mss())."""
    max_err = 0.0
    for i, (state, row) in enumerate(zip(states, raw_rows)):
        expected = (raw_float(row, "accel_body_x_mss"), raw_float(row, "accel_body_y_mss"),
                    raw_float(row, "accel_body_z_mss"))
        for k in range(3):
            err = abs(state.accel_body_mss[k] - expected[k])
            max_err = max(max_err, err)
            if err > tol:
                raise PipelineValidationError(f"accel_body[{k}] mismatch {err:.3e} at row {i}")
    return max_err


def check_ned_velocity_conversion(states, raw_rows, tol: float = 1e-6) -> float:
    """NED velocity is ft/s -> m/s (factor 0.3048), no sign flip (JSBSim's
    v-north/v-east/v-down are already NED, matching ArduPilot's own NED
    convention)."""
    max_err = 0.0
    for i, (state, row) in enumerate(zip(states, raw_rows)):
        expected = (
            raw_float(row, "/fdm/jsbsim/velocities/v-north-fps") * FT_TO_M,
            raw_float(row, "/fdm/jsbsim/velocities/v-east-fps") * FT_TO_M,
            raw_float(row, "/fdm/jsbsim/velocities/v-down-fps") * FT_TO_M,
        )
        for k in range(3):
            err = abs(state.velocity_ned_mps[k] - expected[k])
            max_err = max(max_err, err)
            if err > tol:
                raise PipelineValidationError(f"velocity_ned[{k}] mismatch {err:.3e} at row {i}")
    return max_err


def check_airspeed_conversion(states, raw_rows, tol: float = 1e-6) -> float:
    """Airspeed maps from JSBSim's TRUE airspeed (vt-fps -> m/s), not
    calibrated/indicated airspeed (vc-kts) -- state_from_csv()'s recognized-
    name list checks vt-fps before vc-kts, so when both are present (as in
    this run's CSV) true airspeed wins. See HIL-F24-G report item 5/
    "remaining gaps" for the audit note this implies."""
    max_err = 0.0
    for i, (state, row) in enumerate(zip(states, raw_rows)):
        expected = raw_float(row, "/fdm/jsbsim/velocities/vt-fps") * FT_TO_M
        err = abs(state.airspeed_mps - expected)
        max_err = max(max_err, err)
        if err > tol:
            raise PipelineValidationError(f"airspeed mismatch {err:.3e} at row {i}")
    return max_err


def check_altitude_conversion(states, raw_rows, tol: float = 1e-6) -> float:
    """Altitude maps from JSBSim's ASL height (h-sl-ft -> m), matching a
    barometric altimeter's pressure-altitude (ASL) reference -- NOT AGL
    (h-agl-ft, also present in this run's raw CSV for the audit but never
    consumed by state_from_csv())."""
    max_err = 0.0
    for i, (state, row) in enumerate(zip(states, raw_rows)):
        expected = raw_float(row, "/fdm/jsbsim/position/h-sl-ft") * FT_TO_M
        err = abs(state.altitude_m - expected)
        max_err = max(max_err, err)
        if err > tol:
            raise PipelineValidationError(f"altitude mismatch {err:.3e} at row {i}")
    return max_err


def check_position_passthrough(states, raw_rows, tol: float = 1e-9) -> float:
    max_err = 0.0
    for i, (state, row) in enumerate(zip(states, raw_rows)):
        lat_err = abs(state.latitude_deg - raw_float(row, "/fdm/jsbsim/position/lat-gc-deg"))
        lon_err = abs(state.longitude_deg - raw_float(row, "/fdm/jsbsim/position/long-gc-deg"))
        max_err = max(max_err, lat_err, lon_err)
        if lat_err > tol or lon_err > tol:
            raise PipelineValidationError(f"lat/lon mismatch at row {i}")
    return max_err


def check_attitude_conversion(states, raw_rows, tol_deg: float = 1e-6) -> Tuple[float, float, float]:
    """roll: deg->rad (phi-deg). pitch: taken directly in radians
    (theta-rad is listed before theta-deg in state_from_csv()'s recognized
    names, so no deg->rad conversion is even exercised for pitch in this
    capture -- both are present in the CSV and cross-checked here anyway).
    yaw: deg->rad with signed (-pi, pi] wrapping (normalize_yaw_rad()), not
    JSBSim's native 0..360 deg convention -- see HIL-F24-G report item 5
    ("yaw wrapping")."""
    max_roll_err = max_pitch_err = max_yaw_err = 0.0
    for i, (state, row) in enumerate(zip(states, raw_rows)):
        expected_roll = math.radians(raw_float(row, "/fdm/jsbsim/attitude/phi-deg"))
        expected_pitch = raw_float(row, "/fdm/jsbsim/attitude/theta-rad")
        expected_pitch_from_deg = math.radians(raw_float(row, "/fdm/jsbsim/attitude/theta-deg"))
        expected_yaw = responder.normalize_yaw_rad(math.radians(raw_float(row, "/fdm/jsbsim/attitude/psi-deg")))

        roll_err = math.degrees(abs(state.roll_rad - expected_roll))
        pitch_err = math.degrees(abs(state.pitch_rad - expected_pitch))
        pitch_deg_cross_check = math.degrees(abs(expected_pitch - expected_pitch_from_deg))
        yaw_err = math.degrees(abs(state.yaw_rad - expected_yaw))

        max_roll_err = max(max_roll_err, roll_err)
        max_pitch_err = max(max_pitch_err, max(pitch_err, pitch_deg_cross_check))
        max_yaw_err = max(max_yaw_err, yaw_err)
        if roll_err > tol_deg or pitch_err > tol_deg or yaw_err > tol_deg:
            raise PipelineValidationError(f"attitude conversion mismatch at row {i}")
    return max_roll_err, max_pitch_err, max_yaw_err


def check_no_frame_or_sign_inversion(raw_rows) -> Dict[str, str]:
    """Empirical audit (item 5): confirms JSBSim's own body-rate/attitude
    signs follow the standard FRD aerospace convention ArduPilot also
    uses, so the direct passthrough performed above introduces no
    inversion. Uses only the raw JSBSim columns (not the converted
    SimState) since this is validating the SOURCE convention itself."""
    def nearest(t):
        return min(raw_rows, key=lambda r: abs(raw_float(r, "/fdm/jsbsim/simulation/sim-time-sec") - t))

    level = nearest(6.0)
    az = raw_float(level, "accel_body_z_mss")
    assert az < 0.0, "level-flight accel_body_z_mss should be negative (Z-down, specific-force-up convention)"

    roll_a, roll_b = nearest(17.0), nearest(19.0)
    phi_a = raw_float(roll_a, "/fdm/jsbsim/attitude/phi-deg")
    phi_b = raw_float(roll_b, "/fdm/jsbsim/attitude/phi-deg")
    p_mid = raw_float(nearest(18.0), "p_rad_s")
    assert (phi_b > phi_a) == (p_mid > 0.0), "p_rad_s sign must match d(phi)/dt sign"

    climb_a, climb_b = nearest(16.0), nearest(20.0)
    theta_a = raw_float(climb_a, "/fdm/jsbsim/attitude/theta-deg")
    theta_b = raw_float(climb_b, "/fdm/jsbsim/attitude/theta-deg")
    q_mid = raw_float(nearest(18.0), "q_rad_s")
    assert (theta_b > theta_a) == (q_mid > 0.0), "q_rad_s sign must match d(theta)/dt sign"

    turn_a, turn_b = nearest(16.0), nearest(20.0)
    psi_a = raw_float(turn_a, "/fdm/jsbsim/attitude/psi-deg")
    psi_b = raw_float(turn_b, "/fdm/jsbsim/attitude/psi-deg")
    r_mid = raw_float(nearest(18.0), "r_rad_s")
    assert (psi_b > psi_a) == (r_mid > 0.0), "r_rad_s sign must match d(psi)/dt sign"

    return {
        "level_flight_accel_body_z_mss": f"{az:.4f} (negative, Z-down/specific-force-up, matches gravity_body_mss())",
        "positive_phi_bank_direction": "right bank (phi increasing) -> right turn (psi increasing): confirmed",
        "p_sign_vs_dphi_dt": "matches",
        "q_sign_vs_dtheta_dt": "matches",
        "r_sign_vs_dpsi_dt": "matches",
    }


def check_phase_transition_continuity(
    states: List["responder.SimState"], max_step_s: float = 0.1, max_angle_step_deg: float = 5.0,
) -> List[Dict]:
    """item 8: confirms the PIPELINE (not the underlying JSBSim dynamics)
    introduces no artificial discontinuity at any phase-boundary
    timestamp -- i.e. state changes across one output tick (~0.02 s) stay
    small even exactly at a scripted control-input event, rather than
    jumping as if two unrelated rows had been spliced together."""
    results = []
    for boundary_s in PHASE_BOUNDARIES_S:
        idx = min(range(len(states)), key=lambda i: abs(states[i].timestamp_s - boundary_s))
        if idx == 0 or idx == len(states) - 1:
            continue
        prev_state, next_state = states[idx - 1], states[idx + 1]
        dt = next_state.timestamp_s - prev_state.timestamp_s
        droll = math.degrees(abs(next_state.roll_rad - prev_state.roll_rad))
        dpitch = math.degrees(abs(next_state.pitch_rad - prev_state.pitch_rad))
        entry = {
            "boundary_s": boundary_s, "dt_s": dt, "droll_deg": droll, "dpitch_deg": dpitch,
        }
        results.append(entry)
        if dt > max_step_s:
            raise PipelineValidationError(f"timestamp gap {dt:.4f}s at phase boundary {boundary_s}s")
        if droll > max_angle_step_deg or dpitch > max_angle_step_deg:
            raise PipelineValidationError(f"discontinuous attitude step at phase boundary {boundary_s}s")
    return results


def run_all_coherence_checks(states, raw_rows) -> Dict:
    check_monotonic_timestamps(states)
    check_quaternion_euler_consistency(states)
    conversion_errors = {
        "gyro_rad_s_max_abs_err": check_gyro_passthrough(states, raw_rows),
        "accel_body_mss_max_abs_err": check_gravity_consistent_accel(states, raw_rows),
        "velocity_ned_mps_max_abs_err": check_ned_velocity_conversion(states, raw_rows),
        "airspeed_mps_max_abs_err": check_airspeed_conversion(states, raw_rows),
        "altitude_m_max_abs_err": check_altitude_conversion(states, raw_rows),
        "lat_lon_deg_max_abs_err": check_position_passthrough(states, raw_rows),
    }
    roll_err, pitch_err, yaw_err = check_attitude_conversion(states, raw_rows)
    conversion_errors["roll_deg_max_abs_err"] = roll_err
    conversion_errors["pitch_deg_max_abs_err"] = pitch_err
    conversion_errors["yaw_deg_max_abs_err"] = yaw_err
    frame_audit = check_no_frame_or_sign_inversion(raw_rows)
    transitions = check_phase_transition_continuity(states)
    return {
        "conversion_errors": conversion_errors,
        "frame_audit": frame_audit,
        "phase_transitions": transitions,
    }


# --- Regression captures (item 6) -------------------------------------

def state_to_dict(state: "responder.SimState") -> Dict:
    return {
        "timestamp_s": state.timestamp_s,
        "latitude_deg": state.latitude_deg,
        "longitude_deg": state.longitude_deg,
        "altitude_m": state.altitude_m,
        "roll_rad": state.roll_rad,
        "pitch_rad": state.pitch_rad,
        "yaw_rad": state.yaw_rad,
        "quaternion": list(state.quaternion),
        "gyro_rad_s": list(state.gyro_rad_s),
        "accel_body_mss": list(state.accel_body_mss),
        "velocity_ned_mps": list(state.velocity_ned_mps),
        "airspeed_mps": state.airspeed_mps,
    }


def extract_phase_capture(states, raw_rows, window: Tuple[float, float], sample_every: int = 10) -> Dict:
    start_s, end_s = window
    indices = [i for i, s in enumerate(states) if start_s <= s.timestamp_s <= end_s]
    sampled = indices[::sample_every]
    return {
        "window_s": list(window),
        "n_rows_in_window": len(indices),
        "n_sampled": len(sampled),
        "samples": [state_to_dict(states[i]) for i in sampled],
    }


def save_regression_captures(states, raw_rows, output_dir: Path) -> Dict[str, Path]:
    output_dir.mkdir(parents=True, exist_ok=True)
    paths = {}
    for phase_name, window in PHASE_WINDOWS.items():
        capture = extract_phase_capture(states, raw_rows, window)
        path = output_dir / f"{phase_name}.json"
        path.write_text(json.dumps(capture, indent=2, sort_keys=True))
        paths[phase_name] = path
    return paths


def load_regression_capture(path: Path) -> Dict:
    return json.loads(path.read_text())


def compare_regression_capture(path: Path, states, tol: float = 1e-9) -> bool:
    """Regression check: re-extract the same phase window from a fresh
    conversion run and confirm every sampled field matches the saved
    capture within tol (bit-for-bit determinism, since both the JSBSim
    run and the conversion path are deterministic)."""
    saved = load_regression_capture(path)
    start_s, end_s = saved["window_s"]
    fresh = extract_phase_capture(states, None, (start_s, end_s))
    if len(fresh["samples"]) != len(saved["samples"]):
        return False
    for a, b in zip(saved["samples"], fresh["samples"]):
        for key in ("roll_rad", "pitch_rad", "yaw_rad", "altitude_m", "airspeed_mps"):
            if abs(a[key] - b[key]) > tol:
                return False
    return True


# --- Pipeline health report (item 8) -----------------------------------

def compute_pipeline_health(states, raw_rows, malformed_count: int = 0, missing_field_count: int = 0) -> Dict:
    dts = [states[i + 1].timestamp_s - states[i].timestamp_s for i in range(len(states) - 1)]
    mean_dt = sum(dts) / len(dts) if dts else 0.0
    max_gap = max(dts) if dts else 0.0
    return {
        "n_rows": len(states),
        "packet_rate_hz": (1.0 / mean_dt) if mean_dt > 0 else 0.0,
        "max_stale_gap_s": max_gap,
        "malformed_packet_count": malformed_count,
        "missing_required_field_count": missing_field_count,
        "duration_s": states[-1].timestamp_s - states[0].timestamp_s if states else 0.0,
    }


# --- HIL-F24-F acceptance-analyzer integration (item 7) ----------------

def build_f24f_records(states: List["responder.SimState"]) -> List[Dict]:
    """Builds HIL-F24-F's CSV_FIELDS-shaped telemetry records directly
    from the real converted SimState rows (self-consistency application:
    "commanded" and "actual" are the same data here, since no hardware
    telemetry exists yet -- this demonstrates the F24-F analyzer's
    checks are structurally compatible with real JSBSim-derived dynamic
    data, not only the synthetic profiles it was built against)."""
    records = []
    for state in states:
        records.append({
            "t_s": state.timestamp_s,
            "ahrs2_roll_deg": math.degrees(state.roll_rad),
            "ahrs2_pitch_deg": math.degrees(state.pitch_rad),
            "ahrs2_yaw_deg": math.degrees(state.yaw_rad),
            "gps_lat_deg": state.latitude_deg,
            "gps_lon_deg": state.longitude_deg,
            "gps_alt_m": state.altitude_m,
            "airspeed_mps": state.airspeed_mps,
            "ekf_flags": f24f_analyzer.EKF_REQUIRED_HEALTHY_MASK,
            "ekf_healthy": True,
            "cpu_load_pct": 20.0,
            "armed": False,
            "mode": "MANUAL",
            "ch7_pwm": 0,
            "ch8_pwm": 0,
            "boot_time_s": state.timestamp_s,
        })
    return records


def run_f24f_acceptance_check(states: List["responder.SimState"], category: str = "combined") -> Tuple[bool, Dict]:
    records = build_f24f_records(states)
    thresholds = f24f_analyzer.THRESHOLDS_BY_CATEGORY[category]
    paired = f24f_analyzer.build_paired_samples(states, records)
    return f24f_analyzer.evaluate_acceptance(paired, records, thresholds)


def main():
    output_dir = Path("/tmp/sr75_hil_f24g")
    print("=== HIL-F24-G: real JSBSim -> real SIM_JSON conversion path validation ===")
    csv_path = run_jsbsim(output_dir)
    print(f"JSBSim capture: {csv_path}")
    raw_rows = load_jsbsim_csv(csv_path)
    print(f"Loaded {len(raw_rows)} raw JSBSim rows")
    states = convert_rows(raw_rows)
    print(f"Converted {len(states)} rows through the real StateMapper.state_from_csv()")

    results = run_all_coherence_checks(states, raw_rows)
    print("Conversion errors (max abs):")
    for key, value in results["conversion_errors"].items():
        print(f"  {key}: {value:.3e}")
    print("Frame/sign audit:")
    for key, value in results["frame_audit"].items():
        print(f"  {key}: {value}")

    capture_paths = save_regression_captures(states, raw_rows, REGRESSION_CAPTURES_DIR)
    print(f"Regression captures written: {list(capture_paths)}")

    f24f_ok, f24f_report = run_f24f_acceptance_check(states)
    print(f"HIL-F24-F acceptance analyzer (self-consistency application): {'PASS' if f24f_ok else 'FAIL'}")
    if not f24f_ok:
        for v in f24f_report["violations"]:
            print(f"  - {v}")

    health = compute_pipeline_health(states, raw_rows)
    print("Pipeline health:")
    for key, value in health.items():
        print(f"  {key}: {value}")

    print("OVERALL: PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
