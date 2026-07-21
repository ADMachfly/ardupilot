#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""SR-75 Layer 2J-B3 packet-timing diagnostic.

Reads a sr75_sim_json_responder.py CSV log (LOG_FIELDS in
sr75_sim_json_responder.py) and reports SIM_JSON packet cadence, request/frame
sequencing defects, and stale-command windows. This is a read-only analysis
tool; it does not connect to any live process.
"""

import argparse
import csv
import math
import os
import statistics
import sys
import tempfile
from collections import deque
from typing import Deque, Dict, Iterable, List, Optional, Sequence, Tuple

PWM_FIELDS = ("pwm1", "pwm2", "pwm3", "pwm4", "pwm7")
ACTUATOR_NORM_FIELDS = ("act_elevator_norm", "act_aileron_norm", "act_rudder_norm")
NEUTRAL_FAILSAFE_MARKER = "ACTUATOR_INVALID_ACTIVE_PWM_NEUTRAL"


def read_csv(path: str) -> List[Dict[str, str]]:
    with open(path, "r", newline="", encoding="utf-8") as csv_file:
        return list(csv.DictReader(csv_file))


def parse_float(row: Dict[str, str], name: str) -> Optional[float]:
    value = row.get(name)
    if value in (None, ""):
        return None
    return float(value)


def parse_int(row: Dict[str, str], name: str) -> Optional[int]:
    value = parse_float(row, name)
    return None if value is None else int(value)


def row_time(row: Dict[str, str]) -> Optional[float]:
    return parse_float(row, "host_monotonic_time")


def is_stale(row: Dict[str, str]) -> bool:
    return str(row.get("act_stale", "")) == "1"


def is_valid_packet(row: Dict[str, str]) -> bool:
    return str(row.get("packet_valid", "")) == "1"


def is_non_neutral(row: Dict[str, str]) -> bool:
    for field in ACTUATOR_NORM_FIELDS:
        value = parse_float(row, field)
        if value is not None and abs(value) > 1e-6:
            return True
    return False


def has_valid_pwm(row: Dict[str, str]) -> bool:
    for field in PWM_FIELDS[:4]:
        value = parse_int(row, field)
        if value is None or not (1000 <= value <= 2000):
            return False
    return True


def sequence_gaps(values: Sequence[int]) -> int:
    gaps = 0
    for previous, current in zip(values, values[1:]):
        if current - previous != 1:
            gaps += 1
    return gaps


def duplicate_and_reordered(values: Sequence[int]) -> Tuple[int, int]:
    duplicates = 0
    reordered = 0
    for previous, current in zip(values, values[1:]):
        if current == previous:
            duplicates += 1
        elif current < previous:
            reordered += 1
    return duplicates, reordered


def percentile(values: Sequence[float], fraction: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    index = min(len(ordered) - 1, int(round(fraction * (len(ordered) - 1))))
    return ordered[index]


def pwm_summary(row: Dict[str, str]) -> str:
    values = [row.get(field, "") for field in PWM_FIELDS]
    return " ".join(f"{name.upper()}={value}" for name, value in zip(PWM_FIELDS, values))


def transition_category(rel_time: float, window_start: float, window_end: float) -> str:
    if rel_time < window_start:
        return "startup"
    if rel_time <= window_end:
        return "active"
    return "post"


def analyze(
    rows: Iterable[Dict[str, str]],
    timeout_ms: float,
    active_window_start: Optional[float],
    active_window_end: Optional[float],
    max_transition_lines: int = 20,
) -> Dict[str, object]:
    rows_iter = iter(rows)
    try:
        first_row = next(rows_iter)
    except StopIteration as exc:
        raise ValueError("responder log has no rows") from exc

    start_time = row_time(first_row)
    if start_time is None:
        raise ValueError("responder log has rows with no host_monotonic_time")

    window_start = 0.0 if active_window_start is None else active_window_start
    window_end = math.inf if active_window_end is None else active_window_end
    transition_head_count = max(0, max_transition_lines // 2)
    transition_tail_count = max(0, max_transition_lines - transition_head_count)
    stale_transition_first: List[Dict[str, str]] = []
    stale_transition_last: Deque[Dict[str, str]] = deque(maxlen=transition_tail_count)
    stale_transition_count = 0
    last_valid_time = None
    previous_time = start_time
    previous_rel_time = 0.0
    previous_request_count = parse_int(first_row, "request_count")
    previous_frame_count = parse_int(first_row, "frame_count")
    total_rows = 0
    request_gaps = 0
    frame_gaps = 0
    duplicate_frames = 0
    reordered_frames = 0
    active_request_gaps = 0
    active_frame_gaps = 0
    active_duplicate_frames = 0
    active_reordered_frames = 0
    periods: List[float] = []
    startup_stale_count = 0
    active_window_stale_count = 0
    post_run_stale_count = 0
    max_active_window_gap = 0.0
    active_window_periods: List[float] = []
    first_valid_pwm_time = None
    first_non_neutral_time = None
    longest_valid_run_s = 0.0
    run_start = None
    neutral_failsafe_count = 0
    timeout_s = timeout_ms / 1000.0
    final_rel_time = 0.0

    def consume_row(row: Dict[str, str]) -> None:
        nonlocal total_rows
        nonlocal previous_time
        nonlocal previous_rel_time
        nonlocal previous_request_count
        nonlocal previous_frame_count
        nonlocal request_gaps
        nonlocal frame_gaps
        nonlocal duplicate_frames
        nonlocal reordered_frames
        nonlocal active_request_gaps
        nonlocal active_frame_gaps
        nonlocal active_duplicate_frames
        nonlocal active_reordered_frames
        nonlocal last_valid_time
        nonlocal startup_stale_count
        nonlocal active_window_stale_count
        nonlocal post_run_stale_count
        nonlocal max_active_window_gap
        nonlocal first_valid_pwm_time
        nonlocal first_non_neutral_time
        nonlocal longest_valid_run_s
        nonlocal run_start
        nonlocal neutral_failsafe_count
        nonlocal stale_transition_count
        nonlocal final_rel_time

        current_time = row_time(row)
        if current_time is None:
            raise ValueError("responder log has rows with no host_monotonic_time")
        rel_time = current_time - start_time
        final_rel_time = rel_time

        if total_rows > 0:
            period = current_time - previous_time
            periods.append(period)
            if window_start <= rel_time <= window_end:
                active_window_periods.append(period)
                max_active_window_gap = max(max_active_window_gap, period)

        request_count = parse_int(row, "request_count")
        in_active_sequence = (
            total_rows > 0
            and window_start <= previous_rel_time <= window_end
            and window_start <= rel_time <= window_end
        )
        if (
            total_rows > 0
            and previous_request_count is not None
            and request_count is not None
            and request_count - previous_request_count != 1
        ):
            request_gaps += 1
            if in_active_sequence:
                active_request_gaps += 1
        if request_count is not None:
            previous_request_count = request_count

        frame_count = parse_int(row, "frame_count")
        if total_rows > 0 and previous_frame_count is not None and frame_count is not None:
            if frame_count - previous_frame_count != 1:
                frame_gaps += 1
                if in_active_sequence:
                    active_frame_gaps += 1
            if frame_count == previous_frame_count:
                duplicate_frames += 1
                if in_active_sequence:
                    active_duplicate_frames += 1
            elif frame_count < previous_frame_count:
                reordered_frames += 1
                if in_active_sequence:
                    active_reordered_frames += 1
        if frame_count is not None:
            previous_frame_count = frame_count

        stale = is_stale(row)
        if stale:
            category = transition_category(rel_time, window_start, window_end)
            if category == "startup":
                startup_stale_count += 1
            elif category == "active":
                active_window_stale_count += 1
            else:
                post_run_stale_count += 1
            packet_age_ms = "" if last_valid_time is None else f"{(current_time - last_valid_time) * 1000.0:.3f}"
            transition = {
                "category": category,
                "timestamp": f"{rel_time:.6f}",
                "last_valid_timestamp": "" if last_valid_time is None else f"{last_valid_time - start_time:.6f}",
                "packet_age_ms": packet_age_ms,
                "timeout_ms": f"{timeout_ms:.3f}",
                "request_count": row.get("request_count", ""),
                "frame_count": row.get("frame_count", ""),
                "pwm": pwm_summary(row),
            }
            stale_transition_count += 1
            if len(stale_transition_first) < transition_head_count:
                stale_transition_first.append(transition)
            elif transition_tail_count > 0:
                stale_transition_last.append(transition)
        else:
            last_valid_time = current_time
        if first_valid_pwm_time is None and has_valid_pwm(row):
            first_valid_pwm_time = rel_time
        if first_non_neutral_time is None and is_non_neutral(row):
            first_non_neutral_time = rel_time
        continuous_valid = is_valid_packet(row) and not is_stale(row)
        if continuous_valid:
            if run_start is None:
                run_start = rel_time
            longest_valid_run_s = max(longest_valid_run_s, rel_time - run_start)
        else:
            run_start = None
        if NEUTRAL_FAILSAFE_MARKER in str(row.get("error_reason", "")):
            neutral_failsafe_count += 1
        previous_time = current_time
        previous_rel_time = rel_time
        total_rows += 1

    consume_row(first_row)
    for row in rows_iter:
        consume_row(row)

    sequential_ok = active_request_gaps == 0 and active_frame_gaps == 0
    no_duplicate_or_reorder = active_duplicate_frames == 0 and active_reordered_frames == 0
    timing_pass = (
        sequential_ok
        and no_duplicate_or_reorder
        and active_window_stale_count == 0
        and max_active_window_gap < timeout_s
    )

    return {
        "total_rows": total_rows,
        "request_count_gaps": request_gaps,
        "frame_count_gaps": frame_gaps,
        "duplicate_frame_count": duplicate_frames,
        "reordered_frame_count": reordered_frames,
        "active_request_count_gaps": active_request_gaps,
        "active_frame_count_gaps": active_frame_gaps,
        "active_duplicate_frame_count": active_duplicate_frames,
        "active_reordered_frame_count": active_reordered_frames,
        "mean_period_s": statistics.fmean(periods) if periods else 0.0,
        "median_period_s": statistics.median(periods) if periods else 0.0,
        "p95_period_s": percentile(periods, 0.95),
        "p99_period_s": percentile(periods, 0.99),
        "max_packet_gap_s": max(periods) if periods else 0.0,
        "startup_stale_count": startup_stale_count,
        "active_window_stale_count": active_window_stale_count,
        "post_run_stale_count": post_run_stale_count,
        "first_valid_pwm_time_s": first_valid_pwm_time,
        "first_non_neutral_command_time_s": first_non_neutral_time,
        "longest_valid_continuous_interval_s": longest_valid_run_s,
        "neutral_failsafe_count": neutral_failsafe_count,
        "max_active_window_packet_gap_s": max_active_window_gap,
        "active_window": (window_start, final_rel_time if active_window_end is None else window_end),
        "timeout_s": timeout_s,
        "stale_transition_count": stale_transition_count,
        "stale_transition_first": stale_transition_first,
        "stale_transition_last": list(stale_transition_last),
        "max_transition_lines": max_transition_lines,
        "timing_pass": timing_pass,
    }


def print_report(result: Dict[str, object]) -> None:
    print("SR75 Layer 2J-B3 timing summary")
    print(f"total_rows={result['total_rows']}")
    print(f"request_count_gaps={result['request_count_gaps']}")
    print(f"frame_count_gaps={result['frame_count_gaps']}")
    print(f"duplicate_frame_count={result['duplicate_frame_count']}")
    print(f"reordered_frame_count={result['reordered_frame_count']}")
    print(f"active_request_count_gaps={result['active_request_count_gaps']}")
    print(f"active_frame_count_gaps={result['active_frame_count_gaps']}")
    print(f"active_duplicate_frame_count={result['active_duplicate_frame_count']}")
    print(f"active_reordered_frame_count={result['active_reordered_frame_count']}")
    print(f"mean_packet_period_s={result['mean_period_s']:.6f}")
    print(f"median_packet_period_s={result['median_period_s']:.6f}")
    print(f"p95_packet_period_s={result['p95_period_s']:.6f}")
    print(f"p99_packet_period_s={result['p99_period_s']:.6f}")
    print(f"max_packet_gap_s={result['max_packet_gap_s']:.6f}")
    print(f"startup_stale_count={result['startup_stale_count']}")
    print(f"active_window_stale_count={result['active_window_stale_count']}")
    print(f"post_run_stale_count={result['post_run_stale_count']}")
    print(f"first_valid_pwm_time_s={result['first_valid_pwm_time_s']}")
    print(f"first_non_neutral_command_time_s={result['first_non_neutral_command_time_s']}")
    print(f"longest_valid_continuous_interval_s={result['longest_valid_continuous_interval_s']:.6f}")
    print(f"neutral_failsafe_count={result['neutral_failsafe_count']}")
    window_start, window_end = result["active_window"]
    print(f"active_window=[{window_start:.3f},{window_end:.3f}] timeout_s={result['timeout_s']:.3f}")
    print(f"max_active_window_packet_gap_s={result['max_active_window_packet_gap_s']:.6f}")
    stale_count = result["stale_transition_count"]
    print(f"stale_transition_count={stale_count}")
    printed: List[Dict[str, str]] = list(result["stale_transition_first"])
    tail = list(result["stale_transition_last"])
    first_keys = {(row["timestamp"], row["request_count"], row["frame_count"]) for row in printed}
    printed.extend(row for row in tail if (row["timestamp"], row["request_count"], row["frame_count"]) not in first_keys)
    for transition in printed:
        print(
            "STALE_TRANSITION "
            f"category={transition['category']} "
            f"timestamp={transition['timestamp']} "
            f"last_valid_timestamp={transition['last_valid_timestamp']} "
            f"packet_age_ms={transition['packet_age_ms']} "
            f"timeout_ms={transition['timeout_ms']} "
            f"request_count={transition['request_count']} "
            f"frame_count={transition['frame_count']} "
            f"{transition['pwm']}"
        )
    omitted = stale_count - len(printed)
    if omitted > 0:
        print(f"STALE_TRANSITION_OMITTED count={omitted}")
    if result["timing_pass"]:
        print("ACTIVE_WINDOW_TIMING_PASS")
    else:
        print("ACTIVE_WINDOW_TIMING_FAIL")


def write_rows(path: str, fieldnames: Sequence[str], rows: Sequence[Dict[str, object]]) -> None:
    with open(path, "w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def _synthetic_row(
    index: int,
    time_s: float,
    request_count: int,
    frame_count: int,
    stale: str,
    pwm1: int = 1500,
) -> Dict[str, object]:
    return {
        "host_monotonic_time": f"{time_s:.6f}",
        "request_count": request_count,
        "frame_count": frame_count,
        "packet_valid": "1",
        "act_stale": stale,
        "act_elevator_norm": "0.02" if stale == "0" else "0.0",
        "act_aileron_norm": "0.0",
        "act_rudder_norm": "0.0",
        "pwm1": pwm1,
        "pwm2": 1500,
        "pwm3": 1550,
        "pwm4": 1505,
        "pwm7": 1000,
        "error_reason": "",
    }


FIELDNAMES = [
    "host_monotonic_time", "request_count", "frame_count", "packet_valid", "act_stale",
    "act_elevator_norm", "act_aileron_norm", "act_rudder_norm",
    "pwm1", "pwm2", "pwm3", "pwm4", "pwm7", "error_reason",
]


def run_self_test() -> int:
    failures = []

    def check(name: str, condition: bool) -> None:
        status = "PASS" if condition else "FAIL"
        print(f"SELF_TEST {name} {status}")
        if not condition:
            failures.append(name)

    with tempfile.TemporaryDirectory(prefix="sr75_b3_timing_") as tmpdir:
        # Clean, sequential rows at a 20 ms cadence: no gaps, no stale.
        clean_rows = [
            _synthetic_row(i, i * 0.02, i + 1, i, "0") for i in range(50)
        ]
        clean_path = os.path.join(tmpdir, "clean.csv")
        write_rows(clean_path, FIELDNAMES, clean_rows)
        clean_result = analyze(read_csv(clean_path), 250.0, None, None)
        check("no_gap_active_window_pass", clean_result["timing_pass"] is True)
        check("no_gap_zero_stale", clean_result["active_window_stale_count"] == 0)

        # Same cadence but with one 500 ms gap (above the 250 ms timeout) and a
        # stale row after it.
        gappy_rows = [_synthetic_row(i, i * 0.02, i + 1, i, "0") for i in range(20)]
        gappy_rows.append(_synthetic_row(20, 20 * 0.02 + 0.5, 21, 20, "1"))
        gappy_rows.extend(_synthetic_row(i, 20 * 0.02 + 0.5 + (i - 20) * 0.02, i + 1, i, "0") for i in range(21, 40))
        gappy_path = os.path.join(tmpdir, "gappy.csv")
        write_rows(gappy_path, FIELDNAMES, gappy_rows)
        gappy_result = analyze(read_csv(gappy_path), 250.0, None, None)
        check("packet_gap_above_timeout_detected", gappy_result["max_packet_gap_s"] > 0.25)
        check("packet_gap_above_timeout_fails_pass", gappy_result["timing_pass"] is False)
        check("packet_gap_stale_transition_recorded", gappy_result["stale_transition_count"] >= 1)

        # Duplicate and reordered frame_count values.
        bad_frames = [_synthetic_row(i, i * 0.02, i + 1, i, "0") for i in range(10)]
        bad_frames.append(_synthetic_row(10, 10 * 0.02, 11, 9, "0"))  # duplicate frame_count
        bad_frames.append(_synthetic_row(11, 11 * 0.02, 12, 8, "0"))  # reordered frame_count
        bad_path = os.path.join(tmpdir, "bad_frames.csv")
        write_rows(bad_path, FIELDNAMES, bad_frames)
        bad_result = analyze(read_csv(bad_path), 250.0, None, None)
        check("duplicate_frame_detected", bad_result["duplicate_frame_count"] >= 1)
        check("reordered_frame_detected", bad_result["reordered_frame_count"] >= 1)
        check("bad_frames_fail_pass", bad_result["timing_pass"] is False)

        startup_duplicate_rows = [
            _synthetic_row(0, 0.00, 1, 0, "1", pwm1=0),
            _synthetic_row(1, 0.02, 2, 0, "1", pwm1=0),
        ]
        startup_duplicate_rows.extend(
            _synthetic_row(i, i * 0.02, i + 1, i - 1, "0") for i in range(2, 50)
        )
        startup_duplicate_path = os.path.join(tmpdir, "startup_duplicate.csv")
        write_rows(startup_duplicate_path, FIELDNAMES, startup_duplicate_rows)
        startup_duplicate_result = analyze(read_csv(startup_duplicate_path), 250.0, 0.04, None)
        check("startup_duplicate_detected_globally", startup_duplicate_result["duplicate_frame_count"] == 1)
        check("startup_duplicate_ignored_for_active_window", startup_duplicate_result["timing_pass"] is True)

    if failures:
        print(f"SELF_TEST_RESULT FAIL count={len(failures)} names={','.join(failures)}")
        return 1
    print("SELF_TEST_RESULT PASS")
    return 0


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="SR-75 B3 SIM_JSON packet-timing diagnostic")
    parser.add_argument("--responder-log", help="Responder CSV log path")
    parser.add_argument("--actuator-timeout-ms", type=float, default=250.0)
    parser.add_argument("--active-window-start", type=float, default=None, help="Seconds relative to first row")
    parser.add_argument("--active-window-end", type=float, default=None, help="Seconds relative to first row")
    parser.add_argument("--max-transition-lines", type=int, default=20)
    parser.add_argument("--self-test", action="store_true")
    return parser


def main() -> int:
    args = build_arg_parser().parse_args()
    if args.self_test:
        return run_self_test()
    if not args.responder_log:
        print("ERROR: --responder-log is required unless --self-test is used", file=sys.stderr)
        return 2
    with open(args.responder_log, "r", newline="", encoding="utf-8") as csv_file:
        result = analyze(
            csv.DictReader(csv_file),
            args.actuator_timeout_ms,
            args.active_window_start,
            args.active_window_end,
            args.max_transition_lines,
        )
    print_report(result)
    return 0 if result["timing_pass"] else 1


if __name__ == "__main__":
    sys.exit(main())
