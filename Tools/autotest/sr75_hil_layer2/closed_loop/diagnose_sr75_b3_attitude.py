#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""SR-75 Layer 2J-B3 attitude-path and corrective-sign diagnostic.

Compares JSBSim truth, the SIM_JSON responder's forwarded attitude, and
ArduPlane's own MAVLink ATTITUDE/AHRS2 estimate, then checks whether the
first aileron command actually drives roll error toward zero.

Timestamps across the three logs are on different clocks (JSBSim sim-time,
responder host-monotonic time, and the arm diagnostic's wall-clock time), so
this tool aligns them by time-since-first-row in each log. That is an
approximation appropriate for a diagnostic, not a precision time-sync tool.
"""

import argparse
import csv
import math
import os
import sys
import tempfile
from typing import Dict, List, Optional, Sequence, Tuple

RAD_TO_DEG = 180.0 / math.pi

EXPECTED_CASE_ATTITUDE_DEG = {
    "roll_pos": (15.0, -2.0),
    "roll_neg": (-15.0, -2.0),
    "pitch_pos": (0.0, 8.0),
    "pitch_neg": (0.0, -12.0),
    "combined": (15.0, -10.0),
}


def read_csv(path: str) -> List[Dict[str, str]]:
    with open(path, "r", newline="", encoding="utf-8") as csv_file:
        return list(csv.DictReader(csv_file))


def parse_float(row: Dict[str, str], names: Sequence[str], default: Optional[float] = None) -> Optional[float]:
    for name in names:
        value = row.get(name)
        if value not in (None, ""):
            return float(value)
    return default


def relative_times(rows: Sequence[Dict[str, str]], names: Sequence[str]) -> List[float]:
    raw = [parse_float(row, names, 0.0) or 0.0 for row in rows]
    if not raw:
        return []
    start = raw[0]
    return [value - start for value in raw]


def nearest_index(times: Sequence[float], target: float) -> int:
    best_index = 0
    best_diff = abs(times[0] - target)
    for index, value in enumerate(times):
        diff = abs(value - target)
        if diff < best_diff:
            best_diff = diff
            best_index = index
    return best_index


def responder_attitude_deg(row: Dict[str, str]) -> Tuple[float, float, float]:
    roll = (parse_float(row, ("roll",), 0.0) or 0.0) * RAD_TO_DEG
    pitch = (parse_float(row, ("pitch",), 0.0) or 0.0) * RAD_TO_DEG
    yaw = (parse_float(row, ("yaw",), 0.0) or 0.0) * RAD_TO_DEG
    return roll, pitch, yaw


def jsbsim_attitude_deg(row: Dict[str, str]) -> Tuple[float, float, float]:
    roll = parse_float(row, ("/fdm/jsbsim/attitude/phi-deg", "phi-deg", "roll_deg"), 0.0) or 0.0
    pitch_rad = parse_float(row, ("/fdm/jsbsim/attitude/theta-rad",), None)
    if pitch_rad is not None:
        pitch = pitch_rad * RAD_TO_DEG
    else:
        pitch = parse_float(row, ("theta-deg", "pitch_deg"), 0.0) or 0.0
    yaw = parse_float(row, ("/fdm/jsbsim/attitude/psi-deg", "psi-deg", "yaw_deg"), 0.0) or 0.0
    return roll, pitch, yaw


def wrap_deg(value_deg: float) -> float:
    return (value_deg + 180.0) % 360.0 - 180.0


def build_matched_rows(
    responder_rows: Sequence[Dict[str, str]],
    jsb_rows: Sequence[Dict[str, str]],
    mavlink_rows: Sequence[Dict[str, str]],
) -> List[Dict[str, object]]:
    responder_times = relative_times(responder_rows, ("simulation_timestamp", "host_monotonic_time"))
    jsb_times = relative_times(jsb_rows, ("/fdm/jsbsim/simulation/sim-time-sec", "simulation/sim-time-sec", "time_s"))
    mavlink_times = relative_times(mavlink_rows, ("host_time",)) if mavlink_rows else []

    matched = []
    for index, row in enumerate(responder_rows):
        sim_t = responder_times[index]
        jsb_row = jsb_rows[nearest_index(jsb_times, sim_t)] if jsb_rows else {}
        mav_row = mavlink_rows[nearest_index(mavlink_times, sim_t)] if mavlink_rows else {}

        resp_roll, resp_pitch, resp_yaw = responder_attitude_deg(row)
        jsb_roll, jsb_pitch, jsb_yaw = jsbsim_attitude_deg(jsb_row) if jsb_row else (0.0, 0.0, 0.0)
        att_roll = parse_float(mav_row, ("att_roll_deg",), None) if mav_row else None
        att_pitch = parse_float(mav_row, ("att_pitch_deg",), None) if mav_row else None
        ahrs2_roll = parse_float(mav_row, ("ahrs2_roll_deg",), None) if mav_row else None
        ahrs2_pitch = parse_float(mav_row, ("ahrs2_pitch_deg",), None) if mav_row else None

        matched.append({
            "sim_time_s": sim_t,
            "jsbsim_roll_deg": jsb_roll,
            "jsbsim_pitch_deg": jsb_pitch,
            "jsbsim_yaw_deg": jsb_yaw,
            "responder_roll_deg": resp_roll,
            "responder_pitch_deg": resp_pitch,
            "responder_yaw_deg": resp_yaw,
            "ardupilot_att_roll_deg": att_roll,
            "ardupilot_att_pitch_deg": att_pitch,
            "ardupilot_ahrs2_roll_deg": ahrs2_roll,
            "ardupilot_ahrs2_pitch_deg": ahrs2_pitch,
            "pwm1": row.get("pwm1", ""),
            "pwm2": row.get("pwm2", ""),
            "pwm3": row.get("pwm3", ""),
            "pwm4": row.get("pwm4", ""),
            "aileron_norm": parse_float(row, ("act_aileron_norm",), 0.0) or 0.0,
            "elevator_norm": parse_float(row, ("act_elevator_norm",), 0.0) or 0.0,
            "rudder_norm": parse_float(row, ("act_rudder_norm",), 0.0) or 0.0,
        })
    return matched


def max_and_rms(errors: Sequence[float]) -> Tuple[float, float]:
    if not errors:
        return 0.0, 0.0
    max_error = max(abs(value) for value in errors)
    rms_error = math.sqrt(sum(value * value for value in errors) / len(errors))
    return max_error, rms_error


def analyze(
    matched: Sequence[Dict[str, object]],
    desired_roll_deg: float,
    desired_pitch_deg: float,
    path_tolerance_deg: float,
) -> Dict[str, object]:
    if not matched:
        raise ValueError("no matched rows to analyze")

    jsb_vs_resp_roll = [wrap_deg(row["jsbsim_roll_deg"] - row["responder_roll_deg"]) for row in matched]
    jsb_vs_resp_pitch = [row["jsbsim_pitch_deg"] - row["responder_pitch_deg"] for row in matched]

    resp_vs_ap_roll = [
        wrap_deg(row["responder_roll_deg"] - row["ardupilot_att_roll_deg"])
        for row in matched if row["ardupilot_att_roll_deg"] is not None
    ]
    resp_vs_ap_pitch = [
        row["responder_pitch_deg"] - row["ardupilot_att_pitch_deg"]
        for row in matched if row["ardupilot_att_pitch_deg"] is not None
    ]

    jsb_resp_roll_max, jsb_resp_roll_rms = max_and_rms(jsb_vs_resp_roll)
    jsb_resp_pitch_max, jsb_resp_pitch_rms = max_and_rms(jsb_vs_resp_pitch)
    resp_ap_roll_max, resp_ap_roll_rms = max_and_rms(resp_vs_ap_roll)
    resp_ap_pitch_max, resp_ap_pitch_rms = max_and_rms(resp_vs_ap_pitch)

    final_roll = matched[-1]["responder_roll_deg"]
    roll_error = wrap_deg(final_roll - desired_roll_deg)
    roll_error_sign = (roll_error > 0) - (roll_error < 0)

    first_aileron_sign = 0
    first_aileron_time = None
    for row in matched:
        if abs(row["aileron_norm"]) > 1e-6:
            first_aileron_sign = (row["aileron_norm"] > 0) - (row["aileron_norm"] < 0)
            first_aileron_time = row["sim_time_s"]
            break

    is_corrective = bool(
        roll_error_sign != 0
        and first_aileron_sign != 0
        and (roll_error_sign * first_aileron_sign) < 0
    )

    path_pass = (
        jsb_resp_roll_max <= path_tolerance_deg
        and jsb_resp_pitch_max <= path_tolerance_deg
    )

    return {
        "rows": len(matched),
        "jsbsim_vs_responder_roll_max_deg": jsb_resp_roll_max,
        "jsbsim_vs_responder_roll_rms_deg": jsb_resp_roll_rms,
        "jsbsim_vs_responder_pitch_max_deg": jsb_resp_pitch_max,
        "jsbsim_vs_responder_pitch_rms_deg": jsb_resp_pitch_rms,
        "responder_vs_ardupilot_roll_max_deg": resp_ap_roll_max,
        "responder_vs_ardupilot_roll_rms_deg": resp_ap_roll_rms,
        "responder_vs_ardupilot_pitch_max_deg": resp_ap_pitch_max,
        "responder_vs_ardupilot_pitch_rms_deg": resp_ap_pitch_rms,
        "final_roll_deg": final_roll,
        "roll_error_deg": roll_error,
        "roll_error_sign": roll_error_sign,
        "first_aileron_sign": first_aileron_sign,
        "first_aileron_time_s": first_aileron_time,
        "is_corrective": is_corrective,
        "path_pass": path_pass,
    }


def print_report(result: Dict[str, object], case: Optional[str]) -> None:
    print("SR75 Layer 2J-B3 attitude-path summary")
    print(f"rows={result['rows']}")
    print(
        "jsbsim_vs_responder: "
        f"roll_max={result['jsbsim_vs_responder_roll_max_deg']:.4f} "
        f"roll_rms={result['jsbsim_vs_responder_roll_rms_deg']:.4f} "
        f"pitch_max={result['jsbsim_vs_responder_pitch_max_deg']:.4f} "
        f"pitch_rms={result['jsbsim_vs_responder_pitch_rms_deg']:.4f}"
    )
    print(
        "responder_vs_ardupilot: "
        f"roll_max={result['responder_vs_ardupilot_roll_max_deg']:.4f} "
        f"roll_rms={result['responder_vs_ardupilot_roll_rms_deg']:.4f} "
        f"pitch_max={result['responder_vs_ardupilot_pitch_max_deg']:.4f} "
        f"pitch_rms={result['responder_vs_ardupilot_pitch_rms_deg']:.4f}"
    )
    print(f"final_roll_deg={result['final_roll_deg']:.4f} roll_error_deg={result['roll_error_deg']:.4f}")
    print(f"roll_error_sign={result['roll_error_sign']} first_aileron_sign={result['first_aileron_sign']}")
    if case and case in EXPECTED_CASE_ATTITUDE_DEG:
        expected_roll, expected_pitch = EXPECTED_CASE_ATTITUDE_DEG[case]
        print(f"case={case} expected_roll_deg={expected_roll} expected_pitch_deg={expected_pitch}")

    print("ATTITUDE_PATH_PASS" if result["path_pass"] else "ATTITUDE_PATH_FAIL")
    if result["is_corrective"]:
        print("CORRECTIVE_SIGN_PASS")
    else:
        print(
            "CORRECTIVE_SIGN_FAIL "
            f"roll_error_sign={result['roll_error_sign']} "
            f"first_aileron_sign={result['first_aileron_sign']}"
        )


def _write_csv(path: str, fieldnames: Sequence[str], rows: Sequence[Dict[str, object]]) -> None:
    with open(path, "w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def _self_test_logs(tmpdir: str, corrective: bool) -> Tuple[str, str, str]:
    responder_path = os.path.join(tmpdir, "responder.csv")
    jsb_path = os.path.join(tmpdir, "jsb.csv")
    mavlink_path = os.path.join(tmpdir, "mavlink.csv")

    responder_fields = [
        "simulation_timestamp", "roll", "pitch", "yaw", "act_aileron_norm", "act_elevator_norm",
        "act_rudder_norm", "pwm1", "pwm2", "pwm3", "pwm4",
    ]
    jsb_fields = ["/fdm/jsbsim/simulation/sim-time-sec", "/fdm/jsbsim/attitude/phi-deg",
                  "/fdm/jsbsim/attitude/theta-rad", "/fdm/jsbsim/attitude/psi-deg"]
    mavlink_fields = ["host_time", "att_roll_deg", "att_pitch_deg", "ahrs2_roll_deg", "ahrs2_pitch_deg"]

    responder_rows = []
    jsb_rows = []
    mavlink_rows = []
    aileron_sign = -1.0 if corrective else 1.0
    for index in range(60):
        t_s = index * 0.05
        roll_deg = 15.0 * math.exp(-t_s / 2.0) if corrective else 15.0
        responder_rows.append({
            "simulation_timestamp": f"{t_s:.3f}",
            "roll": f"{math.radians(roll_deg):.9f}",
            "pitch": f"{math.radians(3.0):.9f}",
            "yaw": "0.0",
            "act_aileron_norm": f"{aileron_sign * 0.3:.3f}",
            "act_elevator_norm": "0.01",
            "act_rudder_norm": "0.0",
            "pwm1": "1400",
            "pwm2": "1600",
            "pwm3": "1550",
            "pwm4": "1500",
        })
        jsb_rows.append({
            "/fdm/jsbsim/simulation/sim-time-sec": f"{t_s:.3f}",
            "/fdm/jsbsim/attitude/phi-deg": f"{roll_deg:.6f}",
            "/fdm/jsbsim/attitude/theta-rad": f"{math.radians(3.0):.9f}",
            "/fdm/jsbsim/attitude/psi-deg": "0.0",
        })
        mavlink_rows.append({
            "host_time": f"{t_s:.3f}",
            "att_roll_deg": f"{roll_deg:.6f}",
            "att_pitch_deg": "3.0",
            "ahrs2_roll_deg": f"{roll_deg:.6f}",
            "ahrs2_pitch_deg": "3.0",
        })

    _write_csv(responder_path, responder_fields, responder_rows)
    _write_csv(jsb_path, jsb_fields, jsb_rows)
    _write_csv(mavlink_path, mavlink_fields, mavlink_rows)
    return responder_path, jsb_path, mavlink_path


def run_self_test() -> int:
    failures = []

    def check(name: str, condition: bool) -> None:
        status = "PASS" if condition else "FAIL"
        print(f"SELF_TEST {name} {status}")
        if not condition:
            failures.append(name)

    with tempfile.TemporaryDirectory(prefix="sr75_b3_attitude_") as tmpdir:
        responder_path, jsb_path, mavlink_path = _self_test_logs(tmpdir, corrective=True)
        matched = build_matched_rows(read_csv(responder_path), read_csv(jsb_path), read_csv(mavlink_path))
        result = analyze(matched, desired_roll_deg=0.0, desired_pitch_deg=3.0, path_tolerance_deg=1.0)
        check("attitude_agreement_path_pass", result["path_pass"] is True)
        check("corrective_roll_sign_pass", result["is_corrective"] is True)

        responder_path, jsb_path, mavlink_path = _self_test_logs(tmpdir, corrective=False)
        matched = build_matched_rows(read_csv(responder_path), read_csv(jsb_path), read_csv(mavlink_path))
        result = analyze(matched, desired_roll_deg=0.0, desired_pitch_deg=3.0, path_tolerance_deg=1.0)
        check("incorrect_roll_sign_detected", result["is_corrective"] is False)

    if failures:
        print(f"SELF_TEST_RESULT FAIL count={len(failures)} names={','.join(failures)}")
        return 1
    print("SELF_TEST_RESULT PASS")
    return 0


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="SR-75 B3 attitude-path and corrective-sign diagnostic")
    parser.add_argument("--responder-log", help="Responder CSV log path")
    parser.add_argument("--jsbsim-csv", help="JSBSim state CSV path")
    parser.add_argument("--mavlink-log-csv", default=None, help="Optional MAVLink diagnostic CSV from diagnose_sr75_b3_arm.py")
    parser.add_argument("--case", default=None, choices=sorted(EXPECTED_CASE_ATTITUDE_DEG.keys()))
    parser.add_argument("--desired-roll-deg", type=float, default=0.0)
    parser.add_argument("--desired-pitch-deg", type=float, default=3.0)
    parser.add_argument("--path-tolerance-deg", type=float, default=1.0)
    parser.add_argument("--self-test", action="store_true")
    return parser


def main() -> int:
    args = build_arg_parser().parse_args()
    if args.self_test:
        return run_self_test()
    if not args.responder_log or not args.jsbsim_csv:
        print("ERROR: --responder-log and --jsbsim-csv are required unless --self-test is used", file=sys.stderr)
        return 2

    responder_rows = read_csv(args.responder_log)
    jsb_rows = read_csv(args.jsbsim_csv)
    mavlink_rows = read_csv(args.mavlink_log_csv) if args.mavlink_log_csv and os.path.exists(args.mavlink_log_csv) else []

    matched = build_matched_rows(responder_rows, jsb_rows, mavlink_rows)
    result = analyze(matched, args.desired_roll_deg, args.desired_pitch_deg, args.path_tolerance_deg)
    print_report(result, args.case)
    return 0 if (result["path_pass"] and result["is_corrective"]) else 1


if __name__ == "__main__":
    sys.exit(main())
