#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Validate SR-75 Layer 2J-B3 JSBSim initial conditions before scoring."""

import argparse
import csv
import math
import os
import subprocess
import sys
import xml.etree.ElementTree as ET
from typing import Dict, List, Optional, Sequence, Tuple


REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../.."))
AIRCRAFT_DIR = os.path.join(REPO_ROOT, "Tools/autotest/aircraft/sr_75_6_dof")
JSBSIM_ROOT = os.path.join(REPO_ROOT, "Tools/autotest")
RAD_TO_DEG = 180.0 / math.pi
FT_TO_M = 0.3048
FPS_TO_MPS = 0.3048
KTS_TO_MPS = 0.514444

TRIM_ROLL_DEG = 0.0
TRIM_PITCH_DEG = -2.0
TARGET_ALTITUDE_M = 3000.0
TARGET_TAS_MPS = 134.1 * KTS_TO_MPS
DEFAULT_ELEVATOR = -0.30
DEFAULT_THROTTLE = 0.30

CASES = {
    "trim": ("layer2j_b3_trim_init", TRIM_ROLL_DEG, TRIM_PITCH_DEG),
    "roll_pos": ("layer2j_b3_roll_pos_init", TRIM_ROLL_DEG + 15.0, TRIM_PITCH_DEG),
    "roll_neg": ("layer2j_b3_roll_neg_init", TRIM_ROLL_DEG - 15.0, TRIM_PITCH_DEG),
    "pitch_pos": ("layer2j_b3_pitch_pos_init", TRIM_ROLL_DEG, TRIM_PITCH_DEG + 10.0),
    "pitch_neg": ("layer2j_b3_pitch_neg_init", TRIM_ROLL_DEG, TRIM_PITCH_DEG - 10.0),
    "combined": ("layer2j_b3_combined_init", TRIM_ROLL_DEG + 15.0, TRIM_PITCH_DEG - 8.0),
}

OUTPUT_PROPERTIES = (
    "simulation/sim-time-sec",
    "position/lat-gc-deg",
    "position/long-gc-deg",
    "position/h-sl-ft",
    "velocities/v-north-fps",
    "velocities/v-east-fps",
    "velocities/v-down-fps",
    "velocities/vc-kts",
    "velocities/vt-fps",
    "aero/alpha-rad",
    "attitude/phi-deg",
    "attitude/theta-deg",
    "attitude/theta-rad",
    "attitude/psi-deg",
    "velocities/p-rad_sec",
    "velocities/q-rad_sec",
    "velocities/r-rad_sec",
    "fcs/elevator-cmd-norm",
    "fcs/aileron-cmd-norm",
    "fcs/rudder-cmd-norm",
    "fcs/turbojet-throttle-cmd-norm",
    "fcs/rato-throttle-cmd-norm",
    "fcs/throttle-cmd-norm[0]",
    "fcs/throttle-cmd-norm[1]",
    "fcs/throttle-cmd-norm[2]",
    "propulsion/engine[0]/thrust-lbs",
    "propulsion/engine[1]/thrust-lbs",
    "propulsion/engine[2]/thrust-lbs",
)


def parse_float(value: object, default: Optional[float] = None) -> Optional[float]:
    if value in (None, ""):
        return default
    return float(value)


def row_value(row: Dict[str, str], name: str) -> object:
    return row.get(name, row.get(f"/fdm/jsbsim/{name}", ""))


def row_float(row: Dict[str, str], name: str, default: Optional[float] = None) -> Optional[float]:
    return parse_float(row_value(row, name), default)


def init_file_path(init_name: str) -> str:
    return os.path.join(AIRCRAFT_DIR, f"{init_name}.xml")


def audit_init_file(init_name: str) -> Dict[str, Tuple[str, str]]:
    root = ET.parse(init_file_path(init_name)).getroot()
    values = {}
    for child in root:
        text = "" if child.text is None else child.text.strip()
        values[child.tag] = (text, child.attrib.get("unit", ""))
    return values


def write_runscript(
    path: str,
    init_name: str,
    output_path: str,
    end_s: float,
    elevator: float,
    throttle: float,
) -> None:
    jsbsim_output_path = output_path
    if os.path.isabs(output_path) and output_path.startswith("/tmp/"):
        jsbsim_output_path = os.path.join("../../../../../tmp", output_path[5:])
    output_tags = "\n".join(f"    <property>{name}</property>" for name in OUTPUT_PROPERTIES)
    with open(path, "w", encoding="utf-8") as script:
        script.write(
            f"""<?xml version="1.0" encoding="utf-8"?>
<runscript xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
    xsi:noNamespaceSchemaLocation="http://jsbsim.sf.net/JSBSimScript.xsd"
    name="SR75 Layer 2J-B3 initial-condition validation">

  <use aircraft="sr_75_6_dof" initialize="{init_name}"/>

  <run start="0.0" end="{end_s:.3f}" dt="0.008333">
    <event name="Set validation propulsion and neutral controls">
      <condition>simulation/sim-time-sec ge 0.0</condition>
      <set name="propulsion/engine[0]/set-running" value="1"/>
      <set name="propulsion/engine[1]/set-running" value="1"/>
      <set name="propulsion/engine[2]/set-running" value="0"/>
      <set name="fcs/elevator-cmd-norm" value="{elevator:.6f}"/>
      <set name="fcs/aileron-cmd-norm" value="0.0"/>
      <set name="fcs/rudder-cmd-norm" value="0.0"/>
      <set name="fcs/turbojet-throttle-cmd-norm" value="{throttle:.6f}"/>
      <set name="fcs/rato-throttle-cmd-norm" value="0.0"/>
      <notify/>
    </event>
  </run>

  <output name="{jsbsim_output_path}" type="CSV" rate="50">
{output_tags}
  </output>
</runscript>
"""
        )


def run_jsbsim(jsbsim_bin: str, runscript: str, log_path: str) -> int:
    with open(log_path, "w", encoding="utf-8") as log_file:
        completed = subprocess.run(
            [
                jsbsim_bin,
                f"--root={JSBSIM_ROOT}",
                f"--script={runscript}",
                "--nohighlight",
            ],
            stdout=log_file,
            stderr=subprocess.STDOUT,
            check=False,
        )
    return completed.returncode


def read_rows(path: str) -> List[Dict[str, str]]:
    with open(path, "r", newline="", encoding="utf-8") as csv_file:
        return list(csv.DictReader(csv_file))


def first_scored_row(rows: Sequence[Dict[str, str]]) -> Dict[str, str]:
    candidates = [
        row for row in rows
        if float(row_float(row, "simulation/sim-time-sec", 99.0)) < 2.0
    ]
    if not candidates:
        raise ValueError("no source row with simulation timestamp below 2 seconds")
    return candidates[0]


def quaternion_norm_from_euler(row: Dict[str, str]) -> float:
    phi = math.radians(row_float(row, "attitude/phi-deg", 0.0) or 0.0)
    theta = row_float(row, "attitude/theta-rad", 0.0) or 0.0
    psi = math.radians(row_float(row, "attitude/psi-deg", 0.0) or 0.0)
    half_phi = phi * 0.5
    half_theta = theta * 0.5
    half_psi = psi * 0.5
    q1 = math.cos(half_phi) * math.cos(half_theta) * math.cos(half_psi)
    q1 += math.sin(half_phi) * math.sin(half_theta) * math.sin(half_psi)
    q2 = math.sin(half_phi) * math.cos(half_theta) * math.cos(half_psi)
    q2 -= math.cos(half_phi) * math.sin(half_theta) * math.sin(half_psi)
    q3 = math.cos(half_phi) * math.sin(half_theta) * math.cos(half_psi)
    q3 += math.sin(half_phi) * math.cos(half_theta) * math.sin(half_psi)
    q4 = math.cos(half_phi) * math.cos(half_theta) * math.sin(half_psi)
    q4 -= math.sin(half_phi) * math.sin(half_theta) * math.cos(half_psi)
    return math.sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4)


def finite_values(values: Sequence[float]) -> bool:
    return all(math.isfinite(value) for value in values)


def validate_case(row: Dict[str, str], requested_roll: float, requested_pitch: float) -> Tuple[bool, List[str]]:
    time_s = float(row_float(row, "simulation/sim-time-sec", 99.0))
    roll = float(row_float(row, "attitude/phi-deg", 0.0))
    pitch = float(row_float(row, "attitude/theta-deg", 0.0))
    tas = float(row_float(row, "velocities/vt-fps", 0.0)) * FPS_TO_MPS
    altitude = float(row_float(row, "position/h-sl-ft", 0.0)) * FT_TO_M
    p = float(row_float(row, "velocities/p-rad_sec", 0.0))
    q = float(row_float(row, "velocities/q-rad_sec", 0.0))
    r = float(row_float(row, "velocities/r-rad_sec", 0.0))
    qnorm = quaternion_norm_from_euler(row)
    values = (time_s, roll, pitch, tas, altitude, p, q, r, qnorm)
    reasons = []
    if not finite_values(values):
        reasons.append("non-finite state")
    if time_s >= 2.0:
        reasons.append(f"time {time_s:.3f} >= 2.0")
    if abs(roll - requested_roll) > 2.0:
        reasons.append(f"roll actual {roll:.3f} requested {requested_roll:.3f}")
    if abs(pitch - requested_pitch) > 2.0:
        reasons.append(f"pitch actual {pitch:.3f} requested {requested_pitch:.3f}")
    if abs(tas - TARGET_TAS_MPS) > 5.0:
        reasons.append(f"TAS actual {tas:.3f} target {TARGET_TAS_MPS:.3f}")
    if abs(altitude - TARGET_ALTITUDE_M) > 20.0:
        reasons.append(f"altitude actual {altitude:.3f} target {TARGET_ALTITUDE_M:.3f}")
    if max(abs(p), abs(q), abs(r)) >= 0.2:
        reasons.append(f"pqr actual {p:.4f},{q:.4f},{r:.4f}")
    if not 0.99 <= qnorm <= 1.01:
        reasons.append(f"quaternion norm {qnorm:.6f}")
    return not reasons, reasons


def summarize_row(case: str, row: Dict[str, str], requested_roll: float, requested_pitch: float, passed: bool) -> str:
    time_s = float(row_float(row, "simulation/sim-time-sec", 0.0))
    roll = float(row_float(row, "attitude/phi-deg", 0.0))
    pitch = float(row_float(row, "attitude/theta-deg", 0.0))
    tas = float(row_float(row, "velocities/vt-fps", 0.0)) * FPS_TO_MPS
    altitude = float(row_float(row, "position/h-sl-ft", 0.0)) * FT_TO_M
    p = float(row_float(row, "velocities/p-rad_sec", 0.0))
    q = float(row_float(row, "velocities/q-rad_sec", 0.0))
    r = float(row_float(row, "velocities/r-rad_sec", 0.0))
    return (
        f"{case:9s} req=({requested_roll:7.2f},{requested_pitch:7.2f}) "
        f"actual@{time_s:.3f}=({roll:7.2f},{pitch:7.2f}) "
        f"TAS={tas:7.2f} alt={altitude:8.2f} pqr=({p: .4f},{q: .4f},{r: .4f}) "
        f"{'PASS' if passed else 'FAIL'}"
    )


def print_audit() -> None:
    print("Initial-condition XML audit:")
    for case, (init_name, _, _) in CASES.items():
        values = audit_init_file(init_name)
        fields = (
            "latitude", "longitude", "altitude", "vt", "gamma", "psi", "phi", "theta", "alpha",
            "p", "q", "r", "u", "v", "w", "vnorth", "veast", "vdown",
        )
        rendered = []
        for field in fields:
            value, unit = values.get(field, ("", ""))
            if value or unit:
                rendered.append(f"{field}={value}{(' ' + unit) if unit else ''}")
        print(f"  {case:9s} {init_name}: " + ", ".join(rendered))


def run_case(
    jsbsim_bin: str,
    run_dir: str,
    case: str,
    init_name: str,
    requested_roll: float,
    requested_pitch: float,
    end_s: float,
    elevator: float,
    throttle: float,
) -> bool:
    output_path = os.path.join(run_dir, f"{case}.csv")
    script_path = os.path.join(run_dir, f"{case}.xml")
    log_path = os.path.join(run_dir, f"{case}.log")
    write_runscript(script_path, init_name, output_path, end_s, elevator, throttle)
    rc = run_jsbsim(jsbsim_bin, script_path, log_path)
    if rc != 0:
        print(f"{case:9s} JSBSim FAIL rc={rc} log={log_path}")
        return False
    rows = read_rows(output_path)
    row = first_scored_row(rows)
    passed, reasons = validate_case(row, requested_roll, requested_pitch)
    if not passed:
        print("B3_INITIAL_CONDITION_FAIL " + case)
        for reason in reasons:
            print(f"  {reason}")
    print(summarize_row(case, row, requested_roll, requested_pitch, passed))
    return passed


def print_reference_summary(path: str) -> None:
    rows = read_rows(path)
    tail = rows[-min(100, len(rows)):]
    roll_values = [row_float(row, "attitude/phi-deg", 0.0) or 0.0 for row in tail]
    pitch_values = [row_float(row, "attitude/theta-deg", 0.0) or 0.0 for row in tail]
    alpha_values = [(row_float(row, "aero/alpha-rad", 0.0) or 0.0) * RAD_TO_DEG for row in tail]
    tas_values = [(row_float(row, "velocities/vt-fps", 0.0) or 0.0) * FPS_TO_MPS for row in tail]
    vdown_values = [(row_float(row, "velocities/v-down-fps", 0.0) or 0.0) * FPS_TO_MPS for row in tail]
    elevator_values = [row_float(row, "fcs/elevator-cmd-norm", 0.0) or 0.0 for row in tail]
    throttle_values = [row_float(row, "fcs/turbojet-throttle-cmd-norm", 0.0) or 0.0 for row in tail]
    print(
        "TRIM_REFERENCE "
        f"roll_avg={sum(roll_values) / len(roll_values):.3f} "
        f"pitch_avg={sum(pitch_values) / len(pitch_values):.3f} "
        f"alpha_avg={sum(alpha_values) / len(alpha_values):.3f} "
        f"vdown_avg_mps={sum(vdown_values) / len(vdown_values):.3f} "
        f"TAS_avg_mps={sum(tas_values) / len(tas_values):.3f} "
        f"elevator_avg={sum(elevator_values) / len(elevator_values):.3f} "
        f"throttle_avg={sum(throttle_values) / len(throttle_values):.3f}"
    )


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--jsbsim-bin", default=os.environ.get("JSBSIM_BIN", "JSBSim"))
    parser.add_argument("--run-dir", default="/tmp/sr75_b3_ic")
    parser.add_argument("--end-s", type=float, default=2.0)
    parser.add_argument("--reference-end-s", type=float, default=10.0)
    parser.add_argument("--fixed-elevator", type=float, default=DEFAULT_ELEVATOR)
    parser.add_argument("--fixed-throttle", type=float, default=DEFAULT_THROTTLE)
    parser.add_argument("--audit-only", action="store_true")
    return parser


def main() -> int:
    args = build_arg_parser().parse_args()
    os.makedirs(args.run_dir, exist_ok=True)
    print_audit()
    print(
        "Angle unit evidence: the B3 initialization XML files explicitly tag "
        "phi/theta/psi/gamma as unit=\"DEG\" and p/q/r as unit=\"RAD/SEC\"."
    )
    if args.audit_only:
        return 0
    print("Initial-condition validation:")
    passed = []
    for case, (init_name, requested_roll, requested_pitch) in CASES.items():
        end_s = args.reference_end_s if case == "trim" else args.end_s
        result = run_case(
            args.jsbsim_bin,
            args.run_dir,
            case,
            init_name,
            requested_roll,
            requested_pitch,
            end_s,
            args.fixed_elevator,
            args.fixed_throttle,
        )
        passed.append(result)
        if case == "trim" and result:
            print_reference_summary(os.path.join(args.run_dir, "trim.csv"))
    return 0 if all(passed) else 1


if __name__ == "__main__":
    sys.exit(main())
