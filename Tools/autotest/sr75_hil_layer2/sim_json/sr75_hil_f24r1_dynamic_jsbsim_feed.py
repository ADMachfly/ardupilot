#!/usr/bin/env python3
"""HIL-F24-R1: live SR-75 6-DOF JSBSim state feeder for
sr75_sim_json_responder.py's --state-file input.

Replaces sr75_hil_f24d_static_state_feed.py's single unchanging row with
the real, continuously-changing state of a live JSBSim simulation --
ground-level, motionless start, engines off, RATO/throttle/controls
pinned at zero for the entire run (aircraft/sr_75_6_dof/scripts/
SR75_hil_f24r1_dynamic_state_feed.xml). This script:

  1. Launches JSBSim as a read-only subprocess (never fed anything from
     the Pixhawk -- there is no feedback path into the simulation at all
     in HIL-F24-R1). JSBSim writes its own live-growing, non-atomic CSV
     to a fixed raw path.
  2. Tails that raw CSV (a read-only "last line" poll, tolerant of a
     torn read since JSBSim is still writing -- exactly the technique
     bridge/sr75_jsbsim_pixhawk_hil_bridge.py's JSBSimCSVMonitor already
     uses).
  3. Validates each new row (finite values, per-field unit/range checks,
     strictly-monotonic simulation time) before ever republishing it.
  4. Republishes only new, valid rows as an atomic state.csv snapshot via
     sr75_hil_f24d_static_state_feed.py's own atomic_write_csv_snapshot()
     (imported, not reimplemented) -- the exact same HIL-F24-P/Q/S/T
     atomic-write/timing/coherence/accounting mechanism the static feeder
     already uses, so sr75_sim_json_responder.py's --state-file reader
     needs no changes at all to consume this feeder's output.

Actuator commands received by the responder from the Pixhawk are never
applied here or to JSBSim -- this script has no code path that could even
attempt that; the responder itself must be started without
--jsbsim-command-target/--actuator-map (LOG_ONLY mode) for the same
reason the static-feed orchestrator profile already requires it.

No sockets other than JSBSim's own subprocess are opened by this script.
No MAVLink. No actuator/PWM output. No RATO/engine output.
"""
import argparse
import csv
import math
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sr75_hil_f24d_static_state_feed as feed  # noqa: E402

# HIL-F24-R1: Tools/autotest, the directory JSBSim's --root and this
# script's --jsbsim-script default are both resolved relative to.
AUTOTEST_DIR = Path(__file__).resolve().parents[2]
DEFAULT_JSBSIM_SCRIPT = "aircraft/sr_75_6_dof/scripts/SR75_hil_f24r1_dynamic_state_feed.xml"
# Must match the runscript's own <output name="..."> path exactly.
DEFAULT_RAW_OUTPUT = "/tmp/sr75_hil_f24r1_jsbsim_raw.csv"

# HIL-F24-R1 task 5: per-field plausibility bounds, generous enough to
# never reject a genuine (if aggressive) flight state but tight enough to
# catch a clearly broken/diverged JSBSim integration. lat/lon/alt are
# physical limits; the rest are round-number margins well above anything
# the SR-75 model is expected to produce.
RANGE_CHECKS_BY_FIELD = {
    "lat_deg": (-90.0, 90.0),
    "lon_deg": (-180.0, 180.0),
    "alt_m": (-500.0, 50000.0),
    "vn_mps": (-400.0, 400.0),
    "ve_mps": (-400.0, 400.0),
    "vd_mps": (-400.0, 400.0),
    "airspeed_mps": (0.0, 400.0),
    "p_rad_s": (-35.0, 35.0),
    "q_rad_s": (-35.0, 35.0),
    "r_rad_s": (-35.0, 35.0),
    "accel_body_x_mss": (-100.0, 100.0),
    "accel_body_y_mss": (-100.0, 100.0),
    "accel_body_z_mss": (-100.0, 100.0),
}


def read_existing_output_time_s(path):
    """HIL-F24-R3K handoff-continuity fix: reads `time_s` from whatever
    single-row state.csv snapshot (e.g. sr75_hil_f24d_static_state_feed.py's
    stationary feed) already exists at `path` -- the row this new feeder
    is about to take over from -- so its own published time_s can continue
    that series instead of restarting at JSBSim's own fresh sim-time-sec
    (which always starts near 0 in a new process).

    Returns None if `path` does not exist, is empty/malformed, or has no
    parseable time_s -- callers must treat that as "no prior feeder to
    continue from" (offset 0.0), not an error: this is also how HIL-F24-R1
    is used standalone, with no preceding stationary feeder."""
    try:
        with open(path, "r", newline="", encoding="utf-8") as f:
            rows = list(csv.DictReader(f))
    except OSError:
        return None
    if not rows:
        return None
    try:
        return float(rows[-1]["time_s"])
    except (KeyError, ValueError):
        return None


def validate_jsbsim_row(row, last_published_time_s):
    """HIL-F24-R1 task 5: finite-value, units/range, and timestamp-
    monotonicity checks on one raw JSBSim row, before it is ever
    republished as an atomic state.csv snapshot.

    Returns (time_s, None) if the row is valid and strictly newer than
    the last published row (time_s is None on the very first call);
    returns (None, reason) otherwise -- including the (non-error) case
    where the row is valid but hasn't advanced yet (JSBSim's own output
    file simply hasn't been rewritten since the last poll).
    """
    parsed = {}
    for field in feed.CSV_FIELDS:
        raw = row.get(field)
        if raw is None or raw == "":
            return None, f"missing_field:{field}"
        try:
            value = float(raw)
        except ValueError:
            return None, f"non_numeric:{field}"
        if not math.isfinite(value):
            return None, f"non_finite:{field}"
        parsed[field] = value

    for field, (lo, hi) in RANGE_CHECKS_BY_FIELD.items():
        value = parsed[field]
        if not (lo <= value <= hi):
            return None, f"out_of_range:{field}"

    time_s = parsed["time_s"]
    if last_published_time_s is not None and time_s < last_published_time_s:
        return None, "non_monotonic_time_s"
    if last_published_time_s is not None and time_s == last_published_time_s:
        return None, "not_advanced"

    return time_s, None


class RawJsbsimTail:
    """HIL-F24-R1: tails JSBSim's own live-growing, non-atomic CSV.

    This is a read-only "tail -1" poll, not the atomic-replace pattern
    used for state.csv itself -- JSBSim is the sole writer here, this
    script is the sole reader, and a torn read of the last line (JSBSim
    caught mid-write) is simply retried on the next poll a few ms later.
    Mirrors bridge/sr75_jsbsim_pixhawk_hil_bridge.py's JSBSimCSVMonitor
    tailing technique, generalized to read all columns by header name
    (the R1 runscript's <output> already names every column to match
    sr75_hil_f24d_static_state_feed.py's CSV_FIELDS, so no per-field
    property-path/unit-scale table is needed here at all).
    """

    def __init__(self, path):
        self.path = path
        self.headers = None

    def _load_headers(self):
        try:
            with open(self.path, "r", newline="", encoding="utf-8") as f:
                self.headers = next(csv.reader(f), None)
        except (OSError, StopIteration):
            self.headers = None

    def read_latest_row(self):
        if self.headers is None:
            self._load_headers()
        if not self.headers:
            return None
        try:
            with open(self.path, "rb") as f:
                f.seek(0, os.SEEK_END)
                end_pos = f.tell()
                if end_pos == 0:
                    return None
                block_size = min(65536, end_pos)
                f.seek(end_pos - block_size)
                data = f.read(block_size).decode("utf-8", errors="ignore")
        except OSError:
            return None
        lines = [line for line in data.splitlines() if line.strip()]
        if not lines:
            return None
        parsed = list(csv.reader([lines[-1]]))
        if not parsed:
            return None
        values = parsed[0]
        if values == self.headers:
            return None  # header-only file: no data row published yet
        if len(values) != len(self.headers):
            return None  # torn/partial line -- retry next poll
        return dict(zip(self.headers, values))


def start_jsbsim(args):
    """HIL-F24-R1: launches JSBSim read-only. Mirrors bridge/
    sr75_jsbsim_pixhawk_hil_bridge.py's start_jsbsim_subprocess() --
    same command shape, same --root convention. Never given anything
    derived from Pixhawk input; JSBSim's console output is left
    un-redirected (inherits this script's own stdout/stderr, which the
    orchestrator already captures into feeder.log) so the exact IC,
    trim, and gear-contact evidence JSBSim itself prints at startup is
    preserved as part of this run's recorded evidence (task 6)."""
    try:
        os.remove(args.raw_output)
    except OSError:
        pass
    cmd = [
        args.jsbsim_bin,
        f"--root={args.jsbsim_root}",
        f"--script={args.jsbsim_script}",
        "--realtime",
        f"--end={args.jsbsim_end}",
    ]
    print("Starting read-only JSBSim subprocess: " + " ".join(cmd))
    return subprocess.Popen(cmd)


def stop_jsbsim(proc):
    if proc is None or proc.poll() is not None:
        return
    proc.terminate()
    try:
        proc.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()


class _FeederShutdownRequested(Exception):
    """HIL-F24-R1: raised by the SIGTERM handler below -- same pattern as
    sr75_hil_f24d_static_state_feed.py's own signal handling, so the
    orchestrator's stop-responder-then-feeder sequence (HIL-F24-Q) still
    gets a clean summary print instead of a bare process kill."""


def _raise_shutdown_on_sigterm(signum, frame) -> None:
    raise _FeederShutdownRequested()


def build_arg_parser():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--output", required=True, help="Atomic state.csv path to (re)write; matches responder --state-file")
    parser.add_argument("--duration-s", type=float, default=30.0)
    parser.add_argument("--poll-rate-hz", type=float, default=50.0, help="How often to poll JSBSim's raw output for a new row")
    parser.add_argument("--jsbsim-bin", default="JSBSim")
    parser.add_argument("--jsbsim-root", default=str(AUTOTEST_DIR), help="JSBSim --root (Tools/autotest by default)")
    parser.add_argument("--jsbsim-script", default=DEFAULT_JSBSIM_SCRIPT, help="Runscript path, relative to --jsbsim-root")
    parser.add_argument("--jsbsim-end", type=float, default=3600.0, help="JSBSim's own --end ceiling; actual run length is controlled by --duration-s stopping this process")
    parser.add_argument("--raw-output", default=DEFAULT_RAW_OUTPUT, help="JSBSim's own raw (non-atomic) CSV output path -- must match the runscript's <output name=...>")
    parser.add_argument(
        "--jsbsim-stall-warn-s", type=float, default=2.0,
        help=(
            "Print a warning if JSBSim's own raw output stops advancing for this "
            "long (diagnostic only -- the responder's own monotonic freshness "
            "gate, HIL-F24-T, is what actually enforces staleness on the "
            "consuming side)"
        ),
    )
    parser.add_argument(
        "--time-offset-s", type=float, default=None,
        help=(
            "HIL-F24-R3K: added to every published row's time_s so it continues "
            "the previous feeder's series instead of restarting near 0 (this "
            "process's own JSBSim always starts its sim-time-sec at 0, which the "
            "responder's cross-feeder-lifetime backward-timestamp guard would "
            "otherwise reject for as long as it takes to catch back up). Default "
            "(unset): auto-detect from --output's existing time_s, if any; 0.0 "
            "if --output does not yet exist (e.g. run standalone, with no prior "
            "feeder). Does not affect position/attitude/rates, which are still "
            "computed by the runscript from its own unshifted sim-time-sec."
        ),
    )
    return parser


def main():
    signal.signal(signal.SIGTERM, _raise_shutdown_on_sigterm)
    args = build_arg_parser().parse_args()
    if args.time_offset_s is None:
        time_offset_s = read_existing_output_time_s(args.output) or 0.0
    else:
        time_offset_s = args.time_offset_s
    print(
        f"Starting live JSBSim state feed to {args.output}: "
        f"jsbsim_script={args.jsbsim_script} duration_s={args.duration_s} "
        f"raw_output={args.raw_output} time_offset_s={time_offset_s:.6f}"
    )

    jsbsim_proc = start_jsbsim(args)
    tail = RawJsbsimTail(args.raw_output)
    write_stats = []
    prev_iter_start = None
    longest_gap_s = 0.0
    published = 0
    rejected_counts = {}
    last_published_time_s = None
    min_alt_m = math.inf
    max_alt_m = -math.inf
    last_advance_monotonic = time.monotonic()
    stalled = False
    start = time.monotonic()
    poll_interval_s = 1.0 / args.poll_rate_hz

    try:
        try:
            while time.monotonic() - start < args.duration_s:
                iter_start = time.monotonic()
                if prev_iter_start is not None:
                    longest_gap_s = max(longest_gap_s, iter_start - prev_iter_start)
                prev_iter_start = iter_start

                row = tail.read_latest_row()
                if row is not None:
                    time_s, reason = validate_jsbsim_row(row, last_published_time_s)
                    if reason == "not_advanced":
                        pass  # JSBSim hasn't produced a new row since the last poll yet
                    elif reason is not None:
                        rejected_counts[reason] = rejected_counts.get(reason, 0) + 1
                    else:
                        # HIL-F24-R1: JSBSim's raw output always includes
                        # its own leading "Time" column ahead of the
                        # named fields our <output> block defines; strip
                        # down to exactly feed.CSV_FIELDS before handing
                        # off to atomic_write_csv_snapshot(), whose
                        # DictWriter rejects unexpected extra keys.
                        out_row = {field: row[field] for field in feed.CSV_FIELDS}
                        # HIL-F24-R3K: only the *published* time_s is shifted, so
                        # the responder's cross-feeder monotonicity check sees a
                        # continuous series; position/attitude/rates above are
                        # left untouched (still driven by the runscript's own
                        # unshifted sim-time-sec, which is what keeps the first
                        # published state matching the handed-off stationary
                        # state and the motion ramp shape unchanged).
                        out_row["time_s"] = f"{time_s + time_offset_s:.6f}"
                        write_stats.append(feed.atomic_write_csv_snapshot(args.output, feed.CSV_FIELDS, out_row))
                        published += 1
                        last_published_time_s = time_s
                        last_advance_monotonic = time.monotonic()
                        min_alt_m = min(min_alt_m, float(row["alt_m"]))
                        max_alt_m = max(max_alt_m, float(row["alt_m"]))
                        if stalled:
                            print(f"JSBSIM_STALL_CLEARED time_s={time_s:.3f}")
                            stalled = False

                if not stalled and (time.monotonic() - last_advance_monotonic) > args.jsbsim_stall_warn_s:
                    stalled = True
                    print(
                        f"JSBSIM_STALL_WARNING no new row for over {args.jsbsim_stall_warn_s:.1f}s "
                        f"(diagnostic only -- the responder's monotonic freshness gate enforces "
                        f"the actual state-timeout on the consuming side)"
                    )

                sleep_s = poll_interval_s - (time.monotonic() - iter_start)
                if sleep_s > 0:
                    time.sleep(sleep_s)
        except _FeederShutdownRequested:
            print("Stopped by SIGTERM")
    finally:
        stop_jsbsim(jsbsim_proc)

    elapsed_s = time.monotonic() - start
    print(f"Done: published {published} rows over {elapsed_s:.1f}s")
    feed.print_write_summary(write_stats, longest_gap_s, fsync_enabled=False)
    print(
        "JSBSIM_SUMMARY "
        f"published={published} "
        f"rejected_total={sum(rejected_counts.values())} "
        f"rejected_reasons={rejected_counts} "
        f"min_alt_m={(min_alt_m if math.isfinite(min_alt_m) else float('nan')):.3f} "
        f"max_alt_m={(max_alt_m if math.isfinite(max_alt_m) else float('nan')):.3f} "
        f"final_time_s={last_published_time_s}"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
