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
import os
import signal
import sys
import tempfile
import time
from dataclasses import dataclass

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


@dataclass
class SnapshotWriteStats:
    total_s: float
    fsync_s: float


def atomic_write_csv_snapshot(path, fieldnames, row, fsync=False):
    """HIL-F24-P: (re)write a single-row CSV snapshot atomically.

    The previous implementation opened `path` with mode "w" (truncating it
    immediately) and then wrote the header and the data row as two separate
    writes. A concurrent reader (LatestCSVReader.read_latest(), used by
    sr75_sim_json_responder.py) could observe the file in between any of
    those steps: freshly truncated (empty -- "no CSV header"), header-only
    ("no complete data rows"), or a torn/partial row. This is exactly the
    feeder/responder race seen on the bench (HIL-F24-P).

    Instead, the complete header + row is written to a temporary file in
    the *same directory* as `path` (so the final os.replace() is a rename
    within one filesystem, which POSIX guarantees is atomic), flushed (and
    optionally fsynced), then swapped into place. A reader opening `path`
    at any point in time therefore only ever observes either the previous
    complete snapshot or the new complete snapshot -- never a partial one.
    This atomicity guarantee comes entirely from os.replace()'s same-
    filesystem rename semantics and does NOT depend on fsync() at all.

    HIL-F24-Q: `fsync` defaults to False. fsync() only affects
    *crash/power-loss durability* -- whether this write would survive the
    host dying before the data reaches stable storage -- which is
    irrelevant for this transient, continuously-regenerated (every
    1/rate_hz seconds) live HIL snapshot: if the feeder process dies, the
    file is either regenerated within milliseconds by a restarted feeder
    or the whole test is aborted, so there is nothing here worth losing
    sleep over durability-wise. *Read-atomicity* (what concurrent readers
    observe) is unaffected either way. On the SR-75 bench's storage, a
    single fsync() call was found capable of stalling for over a second
    (see the HIL-F24-Q report's analysis of request 573's 1703.1ms
    STALE_STATE row age), which is exactly why per-write fsync must not be
    the default for this use case. Pass fsync=True (--fsync on the CLI)
    only if crash-durability of the very last snapshot is specifically
    required for some other use of this function.

    Returns a SnapshotWriteStats(total_s, fsync_s) so callers can track
    write-latency and isolate how much of it (if any) fsync() accounts for.
    """
    directory = os.path.dirname(os.path.abspath(path)) or "."
    fd, tmp_path = tempfile.mkstemp(prefix=".state_csv_", suffix=".tmp", dir=directory)
    write_start = time.perf_counter()
    fsync_s = 0.0
    try:
        with os.fdopen(fd, "w", newline="", encoding="utf-8") as tmp_file:
            writer = csv.DictWriter(tmp_file, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerow(row)
            tmp_file.flush()
            if fsync:
                fsync_start = time.perf_counter()
                os.fsync(tmp_file.fileno())
                fsync_s = time.perf_counter() - fsync_start
        os.replace(tmp_path, path)
    except BaseException:
        try:
            os.remove(tmp_path)
        except OSError:
            pass
        raise
    return SnapshotWriteStats(total_s=time.perf_counter() - write_start, fsync_s=fsync_s)


def _percentile(sorted_values, pct):
    """Linear-interpolation percentile over an already-sorted sequence."""
    if not sorted_values:
        return 0.0
    if len(sorted_values) == 1:
        return sorted_values[0]
    k = (len(sorted_values) - 1) * (pct / 100.0)
    f = int(k)
    c = min(f + 1, len(sorted_values) - 1)
    if f == c:
        return sorted_values[f]
    return sorted_values[f] + (sorted_values[c] - sorted_values[f]) * (k - f)


def print_write_summary(write_stats, longest_gap_s, fsync_enabled):
    """HIL-F24-Q: prints max/p95/p99 write latency, total fsync time, and
    the longest observed gap between successive write iterations (the
    real-world signature of a stall like request 573's: a single write
    that takes far longer than 1/rate_hz shows up here as a large gap)."""
    total_durations_ms = sorted(s.total_s * 1000.0 for s in write_stats)
    if not total_durations_ms:
        print("WRITE_SUMMARY no writes recorded")
        return
    fsync_total_ms = sum(s.fsync_s for s in write_stats) * 1000.0
    print(
        "WRITE_SUMMARY "
        f"write_count={len(write_stats)} "
        f"fsync_enabled={int(fsync_enabled)} "
        f"max_write_latency_ms={total_durations_ms[-1]:.3f} "
        f"p95_write_latency_ms={_percentile(total_durations_ms, 95):.3f} "
        f"p99_write_latency_ms={_percentile(total_durations_ms, 99):.3f} "
        f"total_fsync_time_ms={fsync_total_ms:.3f} "
        f"longest_scheduling_gap_ms={longest_gap_s * 1000.0:.3f}"
    )


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
    parser.add_argument(
        "--fsync", action="store_true",
        help=(
            "Call os.fsync() after each snapshot write. Default off: atomicity "
            "for concurrent readers comes entirely from the same-directory "
            "temp-file + os.replace() rename, not from fsync(); fsync() only "
            "affects crash/power-loss durability, which does not matter for "
            "this transient, continuously-regenerated live snapshot, and a "
            "single fsync() call has been observed to stall over a second on "
            "the SR-75 bench's storage (see the HIL-F24-Q report)."
        ),
    )
    return parser


class _FeederShutdownRequested(Exception):
    """HIL-F24-Q: raised by the SIGTERM handler below."""


def _raise_shutdown_on_sigterm(signum, frame) -> None:
    """HIL-F24-Q: with the orchestrator shutdown-ordering fix, the feeder
    is now *deliberately* still mid-run (see FEEDER_SHUTDOWN_MARGIN_S in
    sr75_hil_f24f_hardware_orchestrator.py) when the orchestrator stops it
    with SIGTERM. Without this handler the process would simply die
    without ever reaching print_write_summary() below, silently discarding
    the write-count/latency/scheduling-gap metrics accumulated so far --
    which are exactly the numbers task 5 asks this run to report."""
    raise _FeederShutdownRequested()


def main():
    signal.signal(signal.SIGTERM, _raise_shutdown_on_sigterm)
    args = build_arg_parser().parse_args()
    dt = 1.0 / args.rate_hz
    n_writes = max(1, int(round(args.duration_s / dt)))
    print(f"Writing static state to {args.output}: lat={args.lat_deg} lon={args.lon_deg} "
          f"alt={args.alt_m} yaw_deg={args.yaw_deg} airspeed_mps={args.airspeed_mps} "
          f"rate_hz={args.rate_hz} duration_s={args.duration_s} fsync={args.fsync}")
    start = time.monotonic()
    write_stats = []
    prev_iter_start = None
    longest_gap_s = 0.0
    try:
        for i in range(n_writes):
            iter_start = time.monotonic()
            if prev_iter_start is not None:
                longest_gap_s = max(longest_gap_s, iter_start - prev_iter_start)
            prev_iter_start = iter_start
            t_s = iter_start - start
            row = build_static_row(t_s, args.lat_deg, args.lon_deg, args.alt_m, args.yaw_deg, args.airspeed_mps)
            write_stats.append(atomic_write_csv_snapshot(args.output, CSV_FIELDS, row, fsync=args.fsync))
            next_write = start + (i + 1) * dt
            sleep_s = next_write - time.monotonic()
            if sleep_s > 0:
                time.sleep(sleep_s)
    except _FeederShutdownRequested:
        print("Stopped by SIGTERM")
    print(f"Done: {len(write_stats)} writes over {time.monotonic() - start:.1f}s")
    print_write_summary(write_stats, longest_gap_s, args.fsync)
    return 0


if __name__ == "__main__":
    sys.exit(main())
