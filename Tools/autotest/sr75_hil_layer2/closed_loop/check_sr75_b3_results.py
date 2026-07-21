#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Analyze SR-75 Layer 2J-B3 software-only closed-loop logs."""

import argparse
import csv
import math
import os
import tempfile
from typing import Dict, Iterable, List, Optional, Sequence, Tuple


SAMPLE_TIMES_S = (0.5, 1.0, 2.0, 5.0)
RAD_TO_DEG = 180.0 / math.pi
FT_TO_M = 0.3048
FPS_TO_MPS = 0.3048


def parse_float(row: Dict[str, str], names: Sequence[str], default: Optional[float] = None) -> Optional[float]:
    for name in names:
        value = row.get(name)
        if value not in (None, ""):
            return float(value)
    return default


def read_csv(path: str) -> List[Dict[str, str]]:
    with open(path, "r", newline="", encoding="utf-8") as csv_file:
        return list(csv.DictReader(csv_file))


def jsb_time(row: Dict[str, str]) -> float:
    value = parse_float(row, ("/fdm/jsbsim/simulation/sim-time-sec", "simulation/sim-time-sec", "time_s"))
    if value is None:
        raise ValueError("JSBSim row has no simulation time")
    return value


def nearest_by_time(rows: Sequence[Dict[str, str]], time_s: float) -> Dict[str, str]:
    if not rows:
        return {}
    return min(rows, key=lambda row: abs(jsb_time(row) - time_s))


def quat_norm(row: Dict[str, str]) -> Optional[float]:
    values = [parse_float(row, (name,), None) for name in ("q1", "q2", "q3", "q4")]
    if any(value is None for value in values):
        return None
    return math.sqrt(sum(float(value) * float(value) for value in values if value is not None))


def row_time_zero(rows: Sequence[Dict[str, str]]) -> float:
    first = parse_float(rows[0], ("simulation_timestamp", "timestamp", "time_s"), 0.0)
    return 0.0 if first is None else first


def sample_row(rows: Sequence[Dict[str, str]], relative_time_s: float, start_time_s: float) -> Dict[str, str]:
    target = start_time_s + relative_time_s
    return min(
        rows,
        key=lambda row: abs((parse_float(row, ("simulation_timestamp", "timestamp", "time_s"), 0.0) or 0.0) - target),
    )


def wrap_error_deg(actual_deg: float, desired_deg: float) -> float:
    return (actual_deg - desired_deg + 180.0) % 360.0 - 180.0


def responder_attitude_deg(row: Dict[str, str]) -> Tuple[float, float, float]:
    roll = (parse_float(row, ("roll",), 0.0) or 0.0) * RAD_TO_DEG
    pitch = (parse_float(row, ("pitch",), 0.0) or 0.0) * RAD_TO_DEG
    yaw = (parse_float(row, ("yaw",), 0.0) or 0.0) * RAD_TO_DEG
    return roll, pitch, yaw


def build_combined_rows(
    responder_rows: Sequence[Dict[str, str]],
    jsb_rows: Sequence[Dict[str, str]],
    desired_roll_deg: float,
    desired_pitch_deg: float,
) -> List[Dict[str, object]]:
    combined = []
    request0 = int(float(responder_rows[0].get("request_count", "0"))) if responder_rows else 0
    for row in responder_rows:
        sim_time = parse_float(row, ("simulation_timestamp",), 0.0) or 0.0
        jsb = nearest_by_time(jsb_rows, sim_time)
        roll_deg, pitch_deg, yaw_deg = responder_attitude_deg(row)
        qnorm = quat_norm(jsb)
        combined.append({
            "timestamp": f"{sim_time:.6f}",
            "desired_roll_deg": f"{desired_roll_deg:.6f}",
            "desired_pitch_deg": f"{desired_pitch_deg:.6f}",
            "actual_roll_deg": f"{roll_deg:.6f}",
            "actual_pitch_deg": f"{pitch_deg:.6f}",
            "actual_yaw_deg": f"{yaw_deg:.6f}",
            "p_rad_s": jsb.get("p_rad_s", ""),
            "q_rad_s": jsb.get("q_rad_s", ""),
            "r_rad_s": jsb.get("r_rad_s", ""),
            "pwm1": row.get("pwm1", ""),
            "pwm2": row.get("pwm2", ""),
            "pwm3": row.get("pwm3", ""),
            "pwm4": row.get("pwm4", ""),
            "pwm7": row.get("pwm7", ""),
            "elevator_norm": row.get("act_elevator_norm", ""),
            "aileron_norm": row.get("act_aileron_norm", ""),
            "rudder_norm": row.get("act_rudder_norm", ""),
            "throttle_norm": row.get("act_turbojet_throttle_norm", ""),
            "rato_norm": row.get("act_rato_norm", ""),
            "airspeed_mps": row.get("airspeed", ""),
            "altitude_m": row.get("altitude", ""),
            "state_age_ms": row.get("state_age_ms", ""),
            "request_count": row.get("request_count", ""),
            "relative_request_count": int(float(row.get("request_count", "0"))) - request0 + 1,
            "reply_bytes": row.get("reply_bytes", ""),
            "stale_command": row.get("act_stale", ""),
            "quaternion_norm": "" if qnorm is None else f"{qnorm:.9f}",
        })
    return combined


def write_combined(path: str, rows: Sequence[Dict[str, object]]) -> None:
    if not rows:
        return
    with open(path, "w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def max_abs(rows: Iterable[Dict[str, object]], field: str) -> float:
    values = []
    for row in rows:
        value = row.get(field, "")
        if value != "":
            values.append(abs(float(value)))
    return max(values) if values else 0.0


def finite_range(rows: Iterable[Dict[str, object]], field: str) -> Tuple[float, float]:
    values = [float(row[field]) for row in rows if row.get(field, "") != ""]
    if not values:
        return 0.0, 0.0
    return min(values), max(values)


def settling_time(rows: Sequence[Dict[str, object]], field: str, initial_error: float) -> Optional[float]:
    threshold = abs(initial_error) * 0.5
    if threshold <= 0.0:
        return 0.0
    start_time = float(rows[0]["timestamp"])
    for row in rows:
        if abs(float(row[field])) <= threshold:
            return float(row["timestamp"]) - start_time
    return None


def analyze(
    responder_log: str,
    jsbsim_csv: str,
    output_csv: str,
    desired_roll_deg: float,
    desired_pitch_deg: float,
) -> Dict[str, object]:
    responder_rows = [
        row for row in read_csv(responder_log)
        if row.get("simulation_timestamp") not in (None, "")
    ]
    jsb_rows = read_csv(jsbsim_csv)
    if not responder_rows:
        raise ValueError("responder log has no rows")
    if not jsb_rows:
        raise ValueError("JSBSim CSV has no rows")

    combined = build_combined_rows(responder_rows, jsb_rows, desired_roll_deg, desired_pitch_deg)
    for row in combined:
        row["roll_error_deg"] = f"{wrap_error_deg(float(row['actual_roll_deg']), desired_roll_deg):.6f}"
        row["pitch_error_deg"] = f"{float(row['actual_pitch_deg']) - desired_pitch_deg:.6f}"
    write_combined(output_csv, combined)

    start_time = float(combined[0]["timestamp"])
    first_roll_error = float(combined[0]["roll_error_deg"])
    first_pitch_error = float(combined[0]["pitch_error_deg"])
    samples = []
    for sample_s in SAMPLE_TIMES_S:
        row = min(combined, key=lambda item: abs(float(item["timestamp"]) - (start_time + sample_s)))
        samples.append({
            "t_s": sample_s,
            "roll_deg": float(row["actual_roll_deg"]),
            "pitch_deg": float(row["actual_pitch_deg"]),
            "roll_error_deg": float(row["roll_error_deg"]),
            "pitch_error_deg": float(row["pitch_error_deg"]),
        })

    request_counts = [int(float(row.get("request_count", "0"))) for row in responder_rows]
    expected_requests = list(range(request_counts[0], request_counts[0] + len(request_counts)))
    qnorm_min, qnorm_max = finite_range(combined, "quaternion_norm")
    airspeed_min, airspeed_max = finite_range(combined, "airspeed_mps")
    altitude_min, altitude_max = finite_range(combined, "altitude_m")
    stale_count = sum(1 for row in combined if str(row.get("stale_command", "")) == "1")
    reply_dropouts = sum(1 for row in combined if float(row.get("reply_bytes", "0") or 0.0) <= 0.0)
    rato_max = max_abs(combined, "rato_norm")

    return {
        "rows": len(combined),
        "samples": samples,
        "initial_roll_error_deg": first_roll_error,
        "initial_pitch_error_deg": first_pitch_error,
        "roll_settling_time_s": settling_time(combined, "roll_error_deg", first_roll_error),
        "pitch_settling_time_s": settling_time(combined, "pitch_error_deg", first_pitch_error),
        "roll_error_reduced_5s": abs(samples[-1]["roll_error_deg"]) <= abs(first_roll_error) * 0.5,
        "pitch_error_reduced_5s": abs(samples[-1]["pitch_error_deg"]) <= abs(first_pitch_error) * 0.5,
        "peak_aileron_norm": max_abs(combined, "aileron_norm"),
        "peak_elevator_norm": max_abs(combined, "elevator_norm"),
        "peak_rudder_norm": max_abs(combined, "rudder_norm"),
        "peak_p_rad_s": max_abs(combined, "p_rad_s"),
        "peak_q_rad_s": max_abs(combined, "q_rad_s"),
        "peak_r_rad_s": max_abs(combined, "r_rad_s"),
        "airspeed_range_mps": (airspeed_min, airspeed_max),
        "altitude_drift_m": altitude_max - altitude_min,
        "quaternion_norm_range": (qnorm_min, qnorm_max),
        "stale_count": stale_count,
        "reply_dropouts": reply_dropouts,
        "request_counts_sequential": request_counts == expected_requests,
        "rato_max": rato_max,
        "combined_csv": output_csv,
    }


def print_report(result: Dict[str, object]) -> None:
    print("SR75 Layer 2J-B3 result summary")
    print(f"rows={result['rows']} combined_csv={result['combined_csv']}")
    print(f"initial_roll_error_deg={result['initial_roll_error_deg']:.3f}")
    print(f"initial_pitch_error_deg={result['initial_pitch_error_deg']:.3f}")
    print("samples:")
    for sample in result["samples"]:
        print(
            f"  t={sample['t_s']:.1f}s roll={sample['roll_deg']:.3f} "
            f"pitch={sample['pitch_deg']:.3f} roll_err={sample['roll_error_deg']:.3f} "
            f"pitch_err={sample['pitch_error_deg']:.3f}"
        )
    print(f"roll_settling_time_s={result['roll_settling_time_s']}")
    print(f"pitch_settling_time_s={result['pitch_settling_time_s']}")
    print(f"roll_error_reduced_5s={result['roll_error_reduced_5s']}")
    print(f"pitch_error_reduced_5s={result['pitch_error_reduced_5s']}")
    print(f"peak_aileron_norm={result['peak_aileron_norm']:.3f}")
    print(f"peak_elevator_norm={result['peak_elevator_norm']:.3f}")
    print(f"peak_rudder_norm={result['peak_rudder_norm']:.3f}")
    print(f"peak_pqr_rad_s={result['peak_p_rad_s']:.3f},{result['peak_q_rad_s']:.3f},{result['peak_r_rad_s']:.3f}")
    print(f"airspeed_range_mps={result['airspeed_range_mps'][0]:.3f},{result['airspeed_range_mps'][1]:.3f}")
    print(f"altitude_drift_m={result['altitude_drift_m']:.3f}")
    print(f"quaternion_norm_range={result['quaternion_norm_range'][0]:.6f},{result['quaternion_norm_range'][1]:.6f}")
    print(f"stale_count={result['stale_count']} reply_dropouts={result['reply_dropouts']}")
    print(f"request_counts_sequential={result['request_counts_sequential']}")
    print(f"rato_max={result['rato_max']:.3f}")


def write_self_test_logs(tmpdir: str) -> Tuple[str, str]:
    responder_path = os.path.join(tmpdir, "responder.csv")
    jsb_path = os.path.join(tmpdir, "jsb.csv")
    responder_fields = [
        "request_count", "simulation_timestamp", "roll", "pitch", "yaw", "airspeed", "altitude",
        "reply_bytes", "state_age_ms", "act_elevator_norm", "act_aileron_norm", "act_rudder_norm",
        "act_turbojet_throttle_norm", "act_rato_norm", "act_stale", "pwm1", "pwm2", "pwm3", "pwm4", "pwm7",
    ]
    jsb_fields = [
        "/fdm/jsbsim/simulation/sim-time-sec", "p_rad_s", "q_rad_s", "r_rad_s", "q1", "q2", "q3", "q4",
    ]
    with open(responder_path, "w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=responder_fields)
        writer.writeheader()
        for index in range(101):
            t_s = index * 0.05
            roll_deg = 15.0 * math.exp(-t_s / 1.5)
            pitch_deg = 3.0
            writer.writerow({
                "request_count": index + 1,
                "simulation_timestamp": f"{t_s:.3f}",
                "roll": f"{math.radians(roll_deg):.9f}",
                "pitch": f"{math.radians(pitch_deg):.9f}",
                "yaw": "0.0",
                "airspeed": "69.0",
                "altitude": "3000.0",
                "reply_bytes": "240",
                "state_age_ms": "10",
                "act_elevator_norm": "0.02",
                "act_aileron_norm": "-0.35",
                "act_rudder_norm": "0.01",
                "act_turbojet_throttle_norm": "0.55",
                "act_rato_norm": "0.0",
                "act_stale": "0",
                "pwm1": "1325",
                "pwm2": "1675",
                "pwm3": "1550",
                "pwm4": "1505",
                "pwm7": "1000",
            })
    with open(jsb_path, "w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=jsb_fields)
        writer.writeheader()
        for index in range(101):
            t_s = index * 0.05
            writer.writerow({
                "/fdm/jsbsim/simulation/sim-time-sec": f"{t_s:.3f}",
                "p_rad_s": "0.12",
                "q_rad_s": "0.04",
                "r_rad_s": "0.02",
                "q1": "1.0",
                "q2": "0.0",
                "q3": "0.0",
                "q4": "0.0",
            })
    return responder_path, jsb_path


def main() -> int:
    parser = argparse.ArgumentParser(description="Analyze SR-75 Layer 2J-B3 closed-loop CSV logs")
    parser.add_argument("--responder-log", help="Responder CSV log")
    parser.add_argument("--jsbsim-csv", help="JSBSim state CSV")
    parser.add_argument("--output-csv", default="/tmp/sr75_b3_combined.csv", help="Combined output CSV")
    parser.add_argument("--desired-roll-deg", type=float, default=0.0)
    parser.add_argument("--desired-pitch-deg", type=float, default=3.0)
    parser.add_argument("--self-test", action="store_true", help="Run analyzer against generated software-only sample logs")
    args = parser.parse_args()

    if args.self_test:
        with tempfile.TemporaryDirectory(prefix="sr75_b3_check_") as tmpdir:
            responder_log, jsbsim_csv = write_self_test_logs(tmpdir)
            result = analyze(responder_log, jsbsim_csv, args.output_csv, args.desired_roll_deg, args.desired_pitch_deg)
    else:
        if not args.responder_log or not args.jsbsim_csv:
            parser.error("--responder-log and --jsbsim-csv are required unless --self-test is used")
        result = analyze(args.responder_log, args.jsbsim_csv, args.output_csv, args.desired_roll_deg, args.desired_pitch_deg)
    print_report(result)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
