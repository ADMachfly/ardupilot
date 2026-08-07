#!/usr/bin/env python3
"""HIL-F24-P regression tests for the state.csv feeder/responder race.

Root cause (see the HIL-F24-P report for the full trace): the feeder
(sr75_hil_f24d_static_state_feed.py) previously opened its --output CSV
with mode "w" (truncating it immediately) and wrote the header and data
row as two separate writes, with no fsync or atomic rename. A concurrent
reader (LatestCSVReader.read_latest(), used by
sr75_sim_json_responder.py's --state-file input) could observe the file
mid-write: freshly truncated (empty -- "no CSV header"), header-only
("no complete data rows"), or (in principle) a torn row.

These tests exercise the real feeder and responder code (not
reimplementations), over the real filesystem, with no sockets, no
MAVLink, no hardware, and no actuator/mission logic involved.
"""
import io
import json
import sys
import tempfile
import threading
import time
import unittest
from contextlib import redirect_stdout
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import sr75_hil_f24d_static_state_feed as feed  # noqa: E402
import sr75_sim_json_responder as responder  # noqa: E402


def make_args(state_file, mock_state=False, fresh_state_wait_ms=80.0,
              state_timeout_ms=500.0, allow_state_reuse=False):
    """Build the minimal argparse.Namespace-like object read_reply_state()
    needs, without going through the real CLI parser (no sockets, no
    actuator/JSBSim wiring -- this only exercises the state-file plumbing)."""
    class Args:
        pass
    args = Args()
    args.state_file = state_file
    args.mock_state = mock_state
    args.fresh_state_wait_ms = fresh_state_wait_ms
    args.state_timeout_ms = state_timeout_ms
    args.allow_state_reuse = allow_state_reuse
    return args


def run_concurrency_stress(duration_s, num_readers=4, write_hz=50.0):
    """Runs one writer thread (atomic_write_csv_snapshot(), the HIL-F24-P
    fix) at ~write_hz alongside num_readers reader threads
    (LatestCSVReader.read_latest(), tight-looped) for duration_s seconds.
    Returns (write_count, read_attempt_count, malformed_error_messages).
    A correct atomic writer must produce zero malformed_error_messages
    regardless of how tightly the readers race it."""
    tmpdir = tempfile.mkdtemp(prefix="hil_f24p_stress_")
    path = str(Path(tmpdir) / "state.csv")
    # Seed the file so readers never hit the (expected, harmless) startup
    # not-ready state during this stress run.
    feed.atomic_write_csv_snapshot(path, feed.CSV_FIELDS, feed.build_static_row(0.0))

    stop = threading.Event()
    write_count = [0]
    read_attempt_count = [0] * num_readers
    errors = []
    errors_lock = threading.Lock()
    dt = 1.0 / write_hz

    def writer_loop():
        i = 0
        next_write = time.monotonic()
        while not stop.is_set():
            row = feed.build_static_row(i * dt)
            feed.atomic_write_csv_snapshot(path, feed.CSV_FIELDS, row)
            write_count[0] += 1
            i += 1
            next_write += dt
            sleep_s = next_write - time.monotonic()
            if sleep_s > 0:
                time.sleep(sleep_s)

    def reader_loop(index):
        reader = responder.LatestCSVReader(path)
        while not stop.is_set():
            read_attempt_count[index] += 1
            try:
                reader.read_latest()
            except responder.StateFileMalformedError as exc:
                with errors_lock:
                    errors.append(str(exc))
            except responder.StateFileNotReadyError:
                pass  # only possible before the initial seed write above

    writer_thread = threading.Thread(target=writer_loop)
    reader_threads = [threading.Thread(target=reader_loop, args=(i,)) for i in range(num_readers)]
    writer_thread.start()
    for t in reader_threads:
        t.start()
    time.sleep(duration_s)
    stop.set()
    writer_thread.join(timeout=5.0)
    for t in reader_threads:
        t.join(timeout=5.0)
    return write_count[0], sum(read_attempt_count), errors


class TestAtomicFeederWriteConcurrency(unittest.TestCase):
    """Proves the atomic-write fix: rapid writer iterations racing
    concurrent reader iterations never expose a torn file."""

    def test_concurrent_reads_never_see_missing_header_or_incomplete_row(self):
        write_count, read_count, errors = run_concurrency_stress(duration_s=2.0)
        self.assertGreater(write_count, 50)  # sanity: writer ran near 50 Hz for 2s
        self.assertGreater(read_count, 0)
        self.assertEqual(errors, [])


class TestFeederLeavesValidFinalSnapshot(unittest.TestCase):
    def test_final_file_is_header_plus_one_complete_row(self):
        tmpdir = tempfile.mkdtemp(prefix="hil_f24p_final_")
        path = str(Path(tmpdir) / "state.csv")
        for i in range(20):
            feed.atomic_write_csv_snapshot(path, feed.CSV_FIELDS, feed.build_static_row(i * 0.02))
        with open(path, "r", newline="", encoding="utf-8") as f:
            lines = [line for line in f.read().splitlines() if line.strip()]
        self.assertEqual(lines[0].split(","), feed.CSV_FIELDS)
        self.assertEqual(len(lines), 2)  # header + exactly one data row
        reader = responder.LatestCSVReader(path)
        row, _, _, _ = reader.read_latest()
        self.assertAlmostEqual(float(row["time_s"]), 19 * 0.02, places=6)


class TestResponderRetainsLastValidStateOnTransientFailure(unittest.TestCase):
    def test_retains_last_valid_state_and_dedupes_repeated_log_lines(self):
        tmpdir = tempfile.mkdtemp(prefix="hil_f24p_retain_")
        path = str(Path(tmpdir) / "state.csv")
        feed.atomic_write_csv_snapshot(path, feed.CSV_FIELDS, feed.build_static_row(1.0))

        reader = responder.LatestCSVReader(path)
        mapper = responder.StateMapper(strict=True)
        mock_source = responder.MockStateSource()
        args = make_args(path)

        # First read succeeds and establishes last_valid_state, as the real
        # main loop does by calling accept_state() after a successful reply.
        state1 = responder.read_reply_state(args, reader, mapper, mock_source)
        mapper.accept_state(state1)
        self.assertIsNotNone(mapper.last_valid_state)
        self.assertEqual(mapper.state_read_error_count, 0)
        self.assertFalse(state1.reused_source_row)

        # Deliberately corrupt the file: header only, no data row -- exactly
        # the transient shape a non-atomic write can expose mid-write.
        with open(path, "w", newline="", encoding="utf-8") as f:
            f.write(",".join(feed.CSV_FIELDS) + "\n")

        out1 = io.StringIO()
        with redirect_stdout(out1):
            state2 = responder.read_reply_state(args, reader, mapper, mock_source)
        mapper.accept_state(state2)

        # Retained, not skipped: a reply is still produced, using the last
        # valid state's content, clearly marked as reused.
        self.assertEqual(state2.latitude_deg, state1.latitude_deg)
        self.assertEqual(state2.longitude_deg, state1.longitude_deg)
        self.assertTrue(state2.reused_source_row)
        self.assertGreater(state2.timestamp_s, state1.timestamp_s)  # nudged forward, never duplicated
        self.assertEqual(mapper.state_read_error_count, 1)
        self.assertIn("STATE_FILE_MALFORMED", out1.getvalue())

        # Repeated identical failure (file still malformed, unchanged
        # message) must not flood the log a second time.
        out2 = io.StringIO()
        with redirect_stdout(out2):
            state3 = responder.read_reply_state(args, reader, mapper, mock_source)
        self.assertEqual(mapper.state_read_error_count, 2)
        self.assertTrue(state3.reused_source_row)
        self.assertEqual(out2.getvalue(), "")  # deduplicated: nothing printed

        # Recovery: a fresh valid row is picked up normally (not reused),
        # and error-log dedup re-arms for the next distinct failure.
        feed.atomic_write_csv_snapshot(path, feed.CSV_FIELDS, feed.build_static_row(2.0))
        state4 = responder.read_reply_state(args, reader, mapper, mock_source)
        self.assertFalse(state4.reused_source_row)
        mapper.accept_state(state4)

    def test_raises_state_file_not_ready_when_never_written(self):
        tmpdir = tempfile.mkdtemp(prefix="hil_f24p_notready_")
        path = str(Path(tmpdir) / "never_written.csv")
        reader = responder.LatestCSVReader(path)
        mapper = responder.StateMapper(strict=True)
        mock_source = responder.MockStateSource()
        args = make_args(path)

        # No valid state has ever been seen -- this is the true cold-start
        # condition, distinct from a malformed (previously-valid) file, and
        # must still propagate (the caller skips this one reply) rather
        # than fabricate a reply out of nothing.
        with self.assertRaises(responder.StateFileNotReadyError):
            responder.read_reply_state(args, reader, mapper, mock_source)
        self.assertEqual(mapper.state_read_error_count, 1)
        self.assertIsNone(mapper.last_valid_state)

    def test_malformed_and_not_ready_are_distinct_exception_types(self):
        self.assertTrue(issubclass(responder.StateFileNotReadyError, responder.StateError))
        self.assertTrue(issubclass(responder.StateFileMalformedError, responder.StateError))
        self.assertNotEqual(responder.StateFileNotReadyError, responder.StateFileMalformedError)


class TestSimJsonPacketFormatUnchanged(unittest.TestCase):
    """HIL-F24-P touched only the state.csv feeder/responder handoff --
    this pins down that SimState.to_json_bytes() (the actual SIM_JSON wire
    payload) is untouched."""

    def test_to_json_bytes_schema_unchanged(self):
        state = responder.SimState(
            timestamp_s=1.0, source_timestamp_s=1.0, latitude_deg=10.0, longitude_deg=20.0,
            altitude_m=30.0, roll_rad=0.0, pitch_rad=0.0, yaw_rad=0.0,
            quaternion=(1.0, 0.0, 0.0, 0.0), gyro_rad_s=(0.0, 0.0, 0.0),
            accel_body_mss=(0.0, 0.0, -9.8), velocity_ned_mps=(0.0, 0.0, 0.0),
            airspeed_mps=0.0, row_signature="sig", row_monotonic_time=0.0,
        )
        payload = json.loads(state.to_json_bytes())
        self.assertEqual(
            sorted(payload.keys()),
            sorted(["timestamp", "latitude", "longitude", "altitude", "imu",
                    "velocity", "attitude", "quaternion", "airspeed"]),
        )
        self.assertEqual(sorted(payload["imu"].keys()), sorted(["gyro", "accel_body"]))


if __name__ == "__main__":
    unittest.main()
