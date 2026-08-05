#!/usr/bin/env python3
"""HIL-F24-D: static-state CSV feeder for sr75_sim_json_responder.py's
--state-file input.

The responder's built-in --mock-state (MockStateSource) and the
profile_static_level() helper in sr75_sim_json_test_profiles.py both use
their own fixed lat/lon/alt/yaw and do not expose a yaw override. This
task's exact required static state (lat=32.5378147, lon=74.3661871,
alt=3000m, yaw=315.091444 deg -- i.e. the same BASE_LAT/BASE_LON/
BASE_YAW_DEG constants already used throughout this task family) needs an
explicit feed, so this script repeatedly rewrites a single unchanging row
(fresh timestamp + fresh mtime each write, so StateMapper/LatestCSVReader
never see a stale row) at a fixed rate for a fixed duration, using exactly
the CSV column names StateMapper.state_from_csv() already recognizes.

No sockets, no MAVLink, no hardware. Only ever writes to the local CSV
file passed via --output.
"""
import argparse
import csv
import sys
import time

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from sr75_sim_json_test_profiles import BASE_ALT_M, BASE_LAT, BASE_LON, BASE_YAW_DEG  # noqa: E402
from sr75_sim_json_responder import gravity_body_mss  # noqa: E402

# StateMapper.state_from_csv() requires explicit accel_body_*_mss columns in
# strict mode (it will not silently fall back to a computed value) -- so
# these are always supplied explicitly here via the same gravity_body_mss()
# the responder itself uses, rather than left for the mapper to infer.
CSV_FIELDS = [
    "time_s", "lat_deg", "lon_deg", "alt_m",
    "roll_rad", "pitch_rad", "yaw_rad",
    "vn_mps", "ve_mps", "vd_mps",
    "p_rad_s", "q_rad_s", "r_rad_s",
    "accel_body_x_mss", "accel_body_y_mss", "accel_body_z_mss",
    "airspeed_mps",
]


def build_static_row(t_s, lat_deg=BASE_LAT, lon_deg=BASE_LON, alt_m=BASE_ALT_M,
                      yaw_deg=BASE_YAW_DEG, airspeed_mps=0.0):
    """Pure function: one static-state CSV row. roll/pitch/gyro/velocity are
    always zero -- this is a level, motionless static test state; only
    time_s advances between calls."""
    import math
    yaw_rad = math.radians(yaw_deg)
    ax, ay, az = gravity_body_mss(0.0, 0.0, yaw_rad)
    return {
        "time_s": f"{t_s:.6f}",
        "lat_deg": f"{lat_deg:.7f}",
        "lon_deg": f"{lon_deg:.7f}",
        "alt_m": f"{alt_m:.3f}",
        "roll_rad": "0.0",
        "pitch_rad": "0.0",
        "yaw_rad": f"{yaw_rad:.9f}",
        "vn_mps": "0.0",
        "ve_mps": "0.0",
        "vd_mps": "0.0",
        "p_rad_s": "0.0",
        "q_rad_s": "0.0",
        "r_rad_s": "0.0",
        "accel_body_x_mss": f"{ax:.6f}",
        "accel_body_y_mss": f"{ay:.6f}",
        "accel_body_z_mss": f"{az:.6f}",
        "airspeed_mps": f"{airspeed_mps:.3f}",
    }


def build_arg_parser():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--output", required=True, help="CSV path to (re)write; matches responder --state-file")
    parser.add_argument("--duration-s", type=float, default=30.0)
    parser.add_argument("--rate-hz", type=float, default=50.0)
    parser.add_argument("--lat-deg", type=float, default=BASE_LAT)
    parser.add_argument("--lon-deg", type=float, default=BASE_LON)
    parser.add_argument("--alt-m", type=float, default=BASE_ALT_M)
    parser.add_argument("--yaw-deg", type=float, default=BASE_YAW_DEG)
    parser.add_argument("--airspeed-mps", type=float, default=0.0)
    return parser


def main():
    args = build_arg_parser().parse_args()
    dt = 1.0 / args.rate_hz
    n_writes = max(1, int(round(args.duration_s / dt)))
    print(f"Writing static state to {args.output}: lat={args.lat_deg} lon={args.lon_deg} "
          f"alt={args.alt_m} yaw_deg={args.yaw_deg} airspeed_mps={args.airspeed_mps} "
          f"rate_hz={args.rate_hz} duration_s={args.duration_s}")
    start = time.monotonic()
    for i in range(n_writes):
        t_s = time.monotonic() - start
        row = build_static_row(t_s, args.lat_deg, args.lon_deg, args.alt_m, args.yaw_deg, args.airspeed_mps)
        with open(args.output, "w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=CSV_FIELDS)
            writer.writeheader()
            writer.writerow(row)
            f.flush()
        next_write = start + (i + 1) * dt
        sleep_s = next_write - time.monotonic()
        if sleep_s > 0:
            time.sleep(sleep_s)
    print(f"Done: {n_writes} writes over {time.monotonic() - start:.1f}s")
    return 0


if __name__ == "__main__":
    sys.exit(main())
