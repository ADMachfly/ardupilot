#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""SR-75 Layer 2J-B3 JSBSim-only propulsion and cruise-trim diagnostics."""

import argparse
import csv
import math
import os
import subprocess
import sys
from typing import Dict, Iterable, List, Optional, Sequence, Tuple


REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../.."))
JSBSIM_ROOT = os.path.join(REPO_ROOT, "Tools/autotest")
RUN_DIR_DEFAULT = "/tmp/sr75_b3_trim"
KTS_TO_MPS = 0.514444
FPS_TO_MPS = 0.3048
FT_TO_M = 0.3048
RAD_TO_DEG = 180.0 / math.pi
TARGET_TAS_MPS = 134.1 * KTS_TO_MPS
TARGET_ALTITUDE_M = 3000.0


def jsbsim_path(path: str) -> str:
    if os.path.isabs(path) and path.startswith("/tmp/"):
        return os.path.join("../../../../../tmp", path[5:])
    return path


def parse_float(row: Dict[str, str], field: str, default: Optional[float] = None) -> Optional[float]:
    value = row.get(field)
    if value in (None, ""):
        return default
    return float(value)


def write_init(path: str, pitch_deg: float, alpha_deg: float) -> None:
    with open(path, "w", encoding="utf-8") as init_file:
        init_file.write(
            f"""<?xml version="1.0"?>
<initialize name="SR75 Layer 2J-B3 generated trim candidate">
  <latitude unit="DEG">32.5378085</latitude>
  <longitude unit="DEG">74.3661944</longitude>
  <altitude unit="M">3000.0</altitude>
  <vt unit="KTS">134.1</vt>
  <gamma unit="DEG">0.0</gamma>
  <alpha unit="DEG">{alpha_deg:.6f}</alpha>
  <psi unit="DEG">315.0</psi>
  <phi unit="DEG">0.0</phi>
  <theta unit="DEG">{pitch_deg:.6f}</theta>
  <p unit="RAD/SEC">0.0</p>
  <q unit="RAD/SEC">0.0</q>
  <r unit="RAD/SEC">0.0</r>
</initialize>
"""
        )


def output_functions() -> str:
    fields = {
        "time_s": "simulation/sim-time-sec",
        "lat_deg": "position/lat-gc-deg",
        "lon_deg": "position/long-gc-deg",
        "alt_ft": "position/h-sl-ft",
        "vn_fps": "velocities/v-north-fps",
        "ve_fps": "velocities/v-east-fps",
        "vd_fps": "velocities/v-down-fps",
        "vc_kts": "velocities/vc-kts",
        "vt_fps": "velocities/vt-fps",
        "alpha_rad": "aero/alpha-rad",
        "roll_deg": "attitude/phi-deg",
        "pitch_deg": "attitude/theta-deg",
        "yaw_deg": "attitude/psi-deg",
        "p_rad_s": "velocities/p-rad_sec",
        "q_rad_s": "velocities/q-rad_sec",
        "r_rad_s": "velocities/r-rad_sec",
        "udot_fpss": "accelerations/udot-ft_sec2",
        "turbojet_throttle_cmd": "fcs/turbojet-throttle-cmd-norm",
        "throttle0": "fcs/throttle-cmd-norm[0]",
        "throttle1": "fcs/throttle-cmd-norm[1]",
        "throttle2": "fcs/throttle-cmd-norm[2]",
        "engine0_running": "propulsion/engine/set-running",
        "engine1_running": "propulsion/engine[1]/set-running",
        "engine2_running": "propulsion/engine[2]/set-running",
        "thrust0_lbs": "propulsion/engine/thrust-lbs",
        "thrust1_lbs": "propulsion/engine[1]/thrust-lbs",
        "thrust2_lbs": "propulsion/engine[2]/thrust-lbs",
        "n1_0": "propulsion/engine/n1",
        "n1_1": "propulsion/engine[1]/n1",
        "fuel0_pps": "propulsion/engine/fuel-flow-rate-pps",
        "fuel1_pps": "propulsion/engine[1]/fuel-flow-rate-pps",
        "elevator_cmd": "fcs/elevator-cmd-norm",
        "aileron_cmd": "fcs/aileron-cmd-norm",
        "rudder_cmd": "fcs/rudder-cmd-norm",
        "left_elevon_rad": "fcs/left-elevon-pos-rad",
        "right_elevon_rad": "fcs/right-elevon-pos-rad",
    }
    return "\n".join(
        f'    <function name="{name}"><property>{prop}</property></function>'
        for name, prop in fields.items()
    )


def write_runscript(
    path: str,
    init_path: str,
    output_path: str,
    end_s: float,
    elevator: float,
    throttle: float,
) -> None:
    with open(path, "w", encoding="utf-8") as script:
        script.write(
            f"""<?xml version="1.0" encoding="utf-8"?>
<runscript xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
    xsi:noNamespaceSchemaLocation="http://jsbsim.sf.net/JSBSimScript.xsd"
    name="SR75 Layer 2J-B3 trim diagnostic">

  <use aircraft="sr_75_6_dof" initialize="{init_path}"/>

  <run start="0.0" end="{end_s:.3f}" dt="0.008333">
    <event name="Set JSBSim-only trim candidate controls">
      <condition>simulation/sim-time-sec ge 0.0</condition>
      <set name="propulsion/engine[0]/set-running" value="1"/>
      <set name="propulsion/engine[1]/set-running" value="1"/>
      <set name="propulsion/engine[2]/set-running" value="0"/>
      <set name="fcs/elevator-cmd-norm" value="{elevator:.6f}"/>
      <set name="fcs/aileron-cmd-norm" value="0.0"/>
      <set name="fcs/rudder-cmd-norm" value="0.0"/>
      <set name="fcs/turbojet-throttle-cmd-norm" value="{throttle:.6f}"/>
      <set name="fcs/rato-throttle-cmd-norm" value="0.0"/>
    </event>
  </run>

  <output name="{jsbsim_path(output_path)}" type="CSV" rate="50">
{output_functions()}
  </output>
</runscript>
"""
        )


def run_jsbsim(jsbsim_bin: str, script_path: str, log_path: str) -> int:
    with open(log_path, "w", encoding="utf-8") as log_file:
        completed = subprocess.run(
            [jsbsim_bin, f"--root={JSBSIM_ROOT}", f"--script={script_path}", "--nohighlight"],
            stdout=log_file,
            stderr=subprocess.STDOUT,
            check=False,
        )
    return completed.returncode


def read_rows(path: str) -> List[Dict[str, str]]:
    with open(path, "r", newline="", encoding="utf-8") as csv_file:
        return list(csv.DictReader(csv_file))


def value_range(rows: Iterable[Dict[str, str]], field: str) -> Tuple[float, float]:
    values = [float(row[field]) for row in rows if row.get(field) not in (None, "")]
    return (min(values), max(values)) if values else (0.0, 0.0)


def mean(rows: Sequence[Dict[str, str]], field: str, start_s: float = 0.0, end_s: Optional[float] = None) -> float:
    values = []
    for row in rows:
        time_s = parse_float(row, "time_s", 0.0) or 0.0
        if time_s < start_s:
            continue
        if end_s is not None and time_s > end_s:
            continue
        values.append(parse_float(row, field, 0.0) or 0.0)
    return sum(values) / len(values) if values else 0.0


def finite_envelope_ok(rows: Sequence[Dict[str, str]]) -> bool:
    for row in rows:
        alt_m = (parse_float(row, "alt_ft", 0.0) or 0.0) * FT_TO_M
        tas_mps = (parse_float(row, "vt_fps", 0.0) or 0.0) * FPS_TO_MPS
        vn = (parse_float(row, "vn_fps", 0.0) or 0.0) * FPS_TO_MPS
        ve = (parse_float(row, "ve_fps", 0.0) or 0.0) * FPS_TO_MPS
        vd = (parse_float(row, "vd_fps", 0.0) or 0.0) * FPS_TO_MPS
        values = [
            alt_m, tas_mps, vn, ve, vd,
            parse_float(row, "p_rad_s", 0.0) or 0.0,
            parse_float(row, "q_rad_s", 0.0) or 0.0,
            parse_float(row, "r_rad_s", 0.0) or 0.0,
        ]
        if not all(math.isfinite(value) for value in values):
            return False
        if not -100.0 <= alt_m <= 10000.0:
            return False
        if not 0.0 <= tas_mps <= 250.0:
            return False
        if max(abs(vn), abs(ve), abs(vd)) >= 300.0:
            return False
        if math.sqrt(vn * vn + ve * ve + vd * vd) >= 350.0:
            return False
        if max(abs(values[5]), abs(values[6]), abs(values[7])) >= 20.0:
            return False
    return True


def summarize_run(rows: Sequence[Dict[str, str]]) -> Dict[str, float]:
    first = rows[0]
    last = rows[-1]
    start_alt = (parse_float(first, "alt_ft", 0.0) or 0.0) * FT_TO_M
    final_alt = (parse_float(last, "alt_ft", 0.0) or 0.0) * FT_TO_M
    start_tas = (parse_float(first, "vt_fps", 0.0) or 0.0) * FPS_TO_MPS
    final_tas = (parse_float(last, "vt_fps", 0.0) or 0.0) * FPS_TO_MPS
    final_q = parse_float(last, "q_rad_s", 0.0) or 0.0
    final_vd = (parse_float(last, "vd_fps", 0.0) or 0.0) * FPS_TO_MPS
    pitch0 = parse_float(first, "pitch_deg", 0.0) or 0.0
    pitch_min, pitch_max = value_range(rows, "pitch_deg")
    roll_min, roll_max = value_range(rows, "roll_deg")
    return {
        "alt_delta_m": final_alt - start_alt,
        "tas_delta_mps": final_tas - start_tas,
        "final_q_rad_s": final_q,
        "final_vd_mps": final_vd,
        "max_pitch_div_deg": max(abs(pitch_min - pitch0), abs(pitch_max - pitch0)),
        "max_abs_roll_deg": max(abs(roll_min), abs(roll_max)),
        "turbojet_throttle_cmd_avg": mean(rows, "turbojet_throttle_cmd", 0.0, 3.0),
        "throttle0_avg": mean(rows, "throttle0", 0.0, 3.0),
        "throttle1_avg": mean(rows, "throttle1", 0.0, 3.0),
        "thrust0_avg": mean(rows, "thrust0_lbs", 0.0, 3.0),
        "thrust1_avg": mean(rows, "thrust1_lbs", 0.0, 3.0),
        "thrust2_avg": mean(rows, "thrust2_lbs", 0.0, 3.0),
        "udot_avg_mss": mean(rows, "udot_fpss", 0.0, 3.0) * FPS_TO_MPS,
        "engine0_running_avg": mean(rows, "engine0_running", 0.0, 3.0),
        "engine1_running_avg": mean(rows, "engine1_running", 0.0, 3.0),
        "n1_0_avg": mean(rows, "n1_0", 0.0, 3.0),
        "n1_1_avg": mean(rows, "n1_1", 0.0, 3.0),
        "fuel0_pps_avg": mean(rows, "fuel0_pps", 0.0, 3.0),
        "fuel1_pps_avg": mean(rows, "fuel1_pps", 0.0, 3.0),
    }


def run_case(
    args: argparse.Namespace,
    name: str,
    pitch: float,
    alpha: float,
    elevator: float,
    throttle: float,
    end_s: float,
) -> Tuple[int, str, str, Dict[str, float]]:
    init_path = os.path.join(args.run_dir, f"{name}_init.xml")
    script_path = os.path.join(args.run_dir, f"{name}.xml")
    output_path = os.path.join(args.run_dir, f"{name}.csv")
    log_path = os.path.join(args.run_dir, f"{name}.log")
    write_init(init_path, pitch, alpha)
    write_runscript(script_path, init_path, output_path, end_s, elevator, throttle)
    rc = run_jsbsim(args.jsbsim_bin, script_path, log_path)
    if rc != 0 or not os.path.exists(output_path):
        return rc, output_path, log_path, {}
    rows = read_rows(output_path)
    if not rows:
        return rc, output_path, log_path, {}
    return rc, output_path, log_path, summarize_run(rows)


def propulsion_sanity(args: argparse.Namespace) -> List[Dict[str, float]]:
    results = []
    print("PROPULSION_SANITY")
    for throttle in (0.0, 0.55, 1.0):
        name = f"prop_{throttle:.2f}".replace(".", "p")
        rc, output_path, log_path, summary = run_case(args, name, 3.0, 3.0, 0.0, throttle, 3.0)
        if rc != 0 or not summary:
            print(f"  throttle={throttle:.2f} FAIL rc={rc} output={output_path} log={log_path}")
            continue
        first = read_rows(output_path)[0]
        last = read_rows(output_path)[-1]
        tas_delta = summary["tas_delta_mps"]
        alt_delta = summary["alt_delta_m"]
        result = {
            "requested_throttle": throttle,
            **summary,
        }
        results.append(result)
        print(
            f"  req={throttle:.2f} turbojet_cmd_avg={summary['turbojet_throttle_cmd_avg']:.3f} "
            f"idx0_avg={summary['throttle0_avg']:.3f} idx1_avg={summary['throttle1_avg']:.3f} "
            f"thrust0_avg={summary['thrust0_avg']:.3f} thrust1_avg={summary['thrust1_avg']:.3f} "
            f"thrust2_avg={summary['thrust2_avg']:.3f} udot_avg_mss={summary['udot_avg_mss']:.3f} "
            f"TAS_delta={tas_delta:.3f} alt_delta={alt_delta:.3f} "
            f"n1=({summary['n1_0_avg']:.3f},{summary['n1_1_avg']:.3f}) "
            f"fuel=({summary['fuel0_pps_avg']:.5f},{summary['fuel1_pps_avg']:.5f}) "
            f"start_tas={(parse_float(first, 'vt_fps', 0.0) or 0.0) * FPS_TO_MPS:.3f} "
            f"final_tas={(parse_float(last, 'vt_fps', 0.0) or 0.0) * FPS_TO_MPS:.3f}"
        )
    return results


def trim_score(summary: Dict[str, float], rows: Sequence[Dict[str, str]]) -> float:
    q_after_1 = max(
        abs(parse_float(row, "q_rad_s", 0.0) or 0.0)
        for row in rows if (parse_float(row, "time_s", 0.0) or 0.0) >= 1.0
    )
    return (
        abs(summary["alt_delta_m"]) / 50.0
        + abs(summary["tas_delta_mps"]) / 10.0
        + abs(summary["final_vd_mps"]) / 10.0
        + q_after_1 / 0.2
        + summary["max_pitch_div_deg"] / 10.0
        + summary["max_abs_roll_deg"] / 5.0
    )


def trim_accept(summary: Dict[str, float], rows: Sequence[Dict[str, str]]) -> bool:
    q_after_1 = max(
        abs(parse_float(row, "q_rad_s", 0.0) or 0.0)
        for row in rows if (parse_float(row, "time_s", 0.0) or 0.0) >= 1.0
    )
    return (
        abs(summary["alt_delta_m"]) <= 50.0
        and abs(summary["tas_delta_mps"]) <= 10.0
        and q_after_1 <= 0.2
        and summary["max_abs_roll_deg"] <= 5.0
        and summary["max_pitch_div_deg"] <= 10.0
        and summary["thrust0_avg"] > 0.0
        and summary["thrust1_avg"] > 0.0
        and abs(summary["thrust2_avg"]) <= 1.0e-6
        and finite_envelope_ok(rows)
    )


def trim_search(args: argparse.Namespace) -> Optional[Dict[str, object]]:
    candidates = []
    print("TRIM_SEARCH")
    pitches = [-2.0, 0.0, 2.0, 3.0, 4.0, 6.0, 8.0]
    elevators = [-0.3, -0.2, -0.1, 0.0, 0.1, 0.2, 0.3]
    throttles = [0.3, 0.45, 0.55, 0.7, 0.85, 1.0]
    for pitch in pitches:
        for elevator in elevators:
            for throttle in throttles:
                name = f"trim_p{pitch:+.1f}_e{elevator:+.1f}_t{throttle:.2f}"
                safe_name = name.replace("+", "p").replace("-", "m").replace(".", "p")
                rc, output_path, log_path, summary = run_case(args, safe_name, pitch, pitch, elevator, throttle, 5.0)
                if rc != 0 or not summary:
                    print(f"  {name} FAIL rc={rc} output={output_path} log={log_path}")
                    continue
                rows = read_rows(output_path)
                accepted = trim_accept(summary, rows)
                score = trim_score(summary, rows)
                candidate = {
                    "pitch": pitch,
                    "alpha": pitch,
                    "elevator": elevator,
                    "throttle": throttle,
                    "score": score,
                    "accepted": accepted,
                    "summary": summary,
                    "output": output_path,
                }
                candidates.append(candidate)
    candidates.sort(key=lambda item: (not item["accepted"], item["score"]))
    for candidate in candidates[:12]:
        summary = candidate["summary"]
        print(
            f"  pitch={candidate['pitch']:+.1f} elev={candidate['elevator']:+.2f} "
            f"thr={candidate['throttle']:.2f} score={candidate['score']:.3f} "
            f"accept={int(candidate['accepted'])} alt_d={summary['alt_delta_m']:.2f} "
            f"tas_d={summary['tas_delta_mps']:.2f} q_final={summary['final_q_rad_s']:.3f} "
            f"vd_final={summary['final_vd_mps']:.2f} pitch_div={summary['max_pitch_div_deg']:.2f} "
            f"roll_max={summary['max_abs_roll_deg']:.2f} "
            f"idx=({summary['throttle0_avg']:.3f},{summary['throttle1_avg']:.3f}) "
            f"thrust=({summary['thrust0_avg']:.2f},{summary['thrust1_avg']:.2f},{summary['thrust2_avg']:.2f})"
        )
    best = candidates[0] if candidates else None
    if best is not None:
        summary = best["summary"]
        print(
            "SELECTED_TRIM "
            f"pitch={best['pitch']:.3f} alpha={best['alpha']:.3f} elevator={best['elevator']:.3f} "
            f"throttle={best['throttle']:.3f} accepted={int(best['accepted'])} score={best['score']:.3f} "
            f"throttle_idx=({summary['throttle0_avg']:.3f},{summary['throttle1_avg']:.3f}) "
            f"thrust=({summary['thrust0_avg']:.3f},{summary['thrust1_avg']:.3f},{summary['thrust2_avg']:.3f}) "
            f"output={best['output']}"
        )
    return best


def write_selected_init(path: str, selected: Dict[str, object], roll_deg: float = 0.0) -> None:
    pitch = float(selected["pitch"])
    alpha = float(selected["alpha"])
    with open(path, "w", encoding="utf-8") as init_file:
        init_file.write(
            f"""<?xml version="1.0"?>
<initialize name="SR75 Layer 2J-B3 selected trim reference start">
  <latitude unit="DEG">32.5378085</latitude>
  <longitude unit="DEG">74.3661944</longitude>
  <altitude unit="M">3000.0</altitude>
  <vt unit="KTS">134.1</vt>
  <gamma unit="DEG">0.0</gamma>
  <alpha unit="DEG">{alpha:.6f}</alpha>
  <psi unit="DEG">315.0</psi>
  <phi unit="DEG">{roll_deg:.6f}</phi>
  <theta unit="DEG">{pitch:.6f}</theta>
  <p unit="RAD/SEC">0.0</p>
  <q unit="RAD/SEC">0.0</q>
  <r unit="RAD/SEC">0.0</r>
</initialize>
"""
        )


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--jsbsim-bin", default=os.environ.get("JSBSIM_BIN", "JSBSim"))
    parser.add_argument("--run-dir", default=RUN_DIR_DEFAULT)
    parser.add_argument("--write-selected-inits", action="store_true")
    return parser


def main() -> int:
    args = build_arg_parser().parse_args()
    os.makedirs(args.run_dir, exist_ok=True)
    propulsion_sanity(args)
    selected = trim_search(args)
    if selected is not None and args.write_selected_inits:
        aircraft_dir = os.path.join(JSBSIM_ROOT, "aircraft/sr_75_6_dof")
        write_selected_init(os.path.join(aircraft_dir, "layer2j_b3_trim_init.xml"), selected)
        write_selected_init(os.path.join(aircraft_dir, "layer2j_b3_roll_pos_init.xml"), selected, roll_deg=15.0)
    return 0 if selected is not None else 1


if __name__ == "__main__":
    sys.exit(main())
