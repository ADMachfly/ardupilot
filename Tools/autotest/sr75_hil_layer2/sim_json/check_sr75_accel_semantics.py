#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Analyze SR-75 JSBSim accelerometer semantics CSV output."""

import argparse
import csv
import math
import sys
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Sequence, Tuple


GROUND_CSV = "/tmp/sr75_accel_ground.csv"
LEVEL_FLIGHT_CSV = "/tmp/sr75_accel_level_flight.csv"
FREEFALL_CSV = "/tmp/sr75_accel_freefall.csv"
GRAVITY_MSS = 9.80665


@dataclass(frozen=True)
class Case:
    name: str
    path: str
    window: Tuple[float, float]
    target_mss: float
    tolerance_mss: float


@dataclass(frozen=True)
class Summary:
    samples: int
    avg_mss: float
    min_mss: float
    max_mss: float
    avg_z_mss: float
    agl_start_ft: float
    agl_end_ft: float
    vdown_start_fps: float
    vdown_end_fps: float
    wow_min: float
    wow_max: float
    max_body_rate_rad_s: float


def parse_window(value: str) -> Tuple[float, float]:
    try:
        start, end = value.split(":", 1)
        start_s = float(start)
        end_s = float(end)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("expected START:END seconds") from exc
    if end_s <= start_s:
        raise argparse.ArgumentTypeError("phase END must be greater than START")
    return start_s, end_s


def matching_key(row: Dict[str, str], name: str) -> str:
    for key in row:
        if key == name or key.endswith("/" + name):
            return key
    raise ValueError(f"missing required CSV column: {name}")


def parse_float(row: Dict[str, str], name: str) -> float:
    key = matching_key(row, name)
    try:
        value = float(row[key])
    except ValueError as exc:
        raise ValueError(f"invalid float in {key}: {row.get(key)!r}") from exc
    if not math.isfinite(value):
        raise ValueError(f"non-finite float in {key}: {row.get(key)!r}")
    return value


def optional_float(row: Dict[str, str], name: str) -> Optional[float]:
    try:
        return parse_float(row, name)
    except ValueError:
        return None


def load_rows(path: str) -> List[Dict[str, str]]:
    with open(path, newline="", encoding="utf-8") as csv_file:
        rows = list(csv.DictReader(csv_file))
    if not rows:
        raise ValueError(f"{path} contains no data rows")
    return rows


def magnitude(row: Dict[str, str]) -> float:
    ax = parse_float(row, "accel_body_x_mss")
    ay = parse_float(row, "accel_body_y_mss")
    az = parse_float(row, "accel_body_z_mss")
    return math.sqrt(ax * ax + ay * ay + az * az)


def row_wow(row: Dict[str, str]) -> float:
    values = []
    for name in ("gear/wow", "gear/unit/WOW", "gear/unit[1]/WOW", "gear/unit[2]/WOW"):
        value = optional_float(row, name)
        if value is not None:
            values.append(value)
    if not values:
        raise ValueError("missing required gear WOW columns")
    return max(values)


def rows_for_window(rows: Iterable[Dict[str, str]], window: Tuple[float, float]) -> List[Dict[str, str]]:
    start_s, end_s = window
    selected = []
    for row in rows:
        timestamp = parse_float(row, "simulation/sim-time-sec")
        if start_s <= timestamp <= end_s:
            selected.append(row)
    return selected


def summarize(rows: Sequence[Dict[str, str]]) -> Summary:
    mags = [magnitude(row) for row in rows]
    z_values = [parse_float(row, "accel_body_z_mss") for row in rows]
    body_rates = [
        max(
            abs(parse_float(row, "velocities/p-rad_sec")),
            abs(parse_float(row, "velocities/q-rad_sec")),
            abs(parse_float(row, "velocities/r-rad_sec")),
        )
        for row in rows
    ]
    wows = [row_wow(row) for row in rows]
    return Summary(
        samples=len(rows),
        avg_mss=sum(mags) / len(mags),
        min_mss=min(mags),
        max_mss=max(mags),
        avg_z_mss=sum(z_values) / len(z_values),
        agl_start_ft=parse_float(rows[0], "position/h-agl-ft"),
        agl_end_ft=parse_float(rows[-1], "position/h-agl-ft"),
        vdown_start_fps=parse_float(rows[0], "velocities/v-down-fps"),
        vdown_end_fps=parse_float(rows[-1], "velocities/v-down-fps"),
        wow_min=min(wows),
        wow_max=max(wows),
        max_body_rate_rad_s=max(body_rates),
    )


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Check SR-75 accelerometer specific-force semantics")
    parser.add_argument("--ground-csv", default=GROUND_CSV)
    parser.add_argument("--level-flight-csv", default=LEVEL_FLIGHT_CSV)
    parser.add_argument("--freefall-csv", default=FREEFALL_CSV)
    parser.add_argument("--ground-window", type=parse_window, default=(2.0, 8.0))
    parser.add_argument("--level-flight-window", type=parse_window, default=(2.0, 12.0))
    parser.add_argument("--freefall-window", type=parse_window, default=(0.2, 2.0))
    parser.add_argument("--one-g", type=float, default=GRAVITY_MSS)
    parser.add_argument("--one-g-tolerance", type=float, default=2.0)
    parser.add_argument("--freefall-max", type=float, default=2.0)
    parser.add_argument("--max-level-vdown-fps", type=float, default=25.0)
    parser.add_argument("--max-level-altitude-drift-ft", type=float, default=250.0)
    parser.add_argument("--max-level-body-rate-rad-s", type=float, default=0.5)
    return parser


def evaluate_case(case: Case, summary: Summary, args: argparse.Namespace) -> Tuple[bool, str]:
    if case.name == "ground":
        checks = [
            abs(summary.avg_mss - case.target_mss) <= case.tolerance_mss,
            summary.avg_z_mss < 0.0,
            summary.wow_min >= 0.5,
        ]
        detail = "one_g,z_negative,wow_present"
    elif case.name == "level_flight":
        checks = [
            abs(summary.avg_mss - case.target_mss) <= case.tolerance_mss,
            summary.wow_max <= 0.5,
            abs(summary.agl_end_ft - summary.agl_start_ft) <= args.max_level_altitude_drift_ft,
            abs(summary.vdown_end_fps) <= args.max_level_vdown_fps,
            summary.max_body_rate_rad_s <= args.max_level_body_rate_rad_s,
        ]
        detail = "one_g,airborne,altitude_stable,vdown_stable,low_rates"
    else:
        checks = [
            summary.max_mss <= case.tolerance_mss,
            summary.wow_max <= 0.5,
            summary.agl_end_ft < summary.agl_start_ft,
            summary.vdown_end_fps > summary.vdown_start_fps,
        ]
        detail = "near_zero_g,airborne,altitude_decreasing,down_velocity_increasing"
    return all(checks), detail


def main() -> int:
    args = build_arg_parser().parse_args()
    cases = [
        Case("ground", args.ground_csv, args.ground_window, args.one_g, args.one_g_tolerance),
        Case("level_flight", args.level_flight_csv, args.level_flight_window, args.one_g, args.one_g_tolerance),
        Case("freefall", args.freefall_csv, args.freefall_window, 0.0, args.freefall_max),
    ]

    try:
        failed = False
        print(
            "case,csv,start_s,end_s,samples,avg_mss,min_mss,max_mss,avg_z_mss,"
            "agl_start_ft,agl_end_ft,vdown_start_fps,vdown_end_fps,wow_min,wow_max,"
            "max_body_rate_rad_s,checks,result"
        )
        for case in cases:
            rows = rows_for_window(load_rows(case.path), case.window)
            if not rows:
                print(f"{case.name},{case.path},{case.window[0]:.3f},{case.window[1]:.3f},0,,,,,,,,,,,,,FAIL")
                failed = True
                continue
            summary = summarize(rows)
            passed, checks = evaluate_case(case, summary, args)
            failed = failed or not passed
            print(
                f"{case.name},{case.path},{case.window[0]:.3f},{case.window[1]:.3f},"
                f"{summary.samples},{summary.avg_mss:.6f},{summary.min_mss:.6f},"
                f"{summary.max_mss:.6f},{summary.avg_z_mss:.6f},"
                f"{summary.agl_start_ft:.3f},{summary.agl_end_ft:.3f},"
                f"{summary.vdown_start_fps:.3f},{summary.vdown_end_fps:.3f},"
                f"{summary.wow_min:.1f},{summary.wow_max:.1f},"
                f"{summary.max_body_rate_rad_s:.6f},{checks},{'PASS' if passed else 'FAIL'}"
            )
        return 1 if failed else 0
    except (OSError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
