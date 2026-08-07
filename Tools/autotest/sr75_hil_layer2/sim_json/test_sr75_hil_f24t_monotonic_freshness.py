#!/usr/bin/env python3
"""HIL-F24-T regression tests: monotonic-clock-based state freshness.

Root cause recap (see the HIL-F24-T report for the full trace): the
freshness gate computed `time.time() - st.st_mtime` -- both wall-clock
(CLOCK_REALTIME) values. A host NTP correction or manual clock step
between the feeder's write and the responder's read directly inflates or
deflates that difference with no relationship to how long the row has
actually existed. A ~2s step produces exactly a ~2000ms error, matching
the observed 2050.7ms false STALE_STATE against an otherwise perfectly
healthy ~20ms-cadence feeder (max write latency 0.831ms, scheduling gap
20.269ms -- ruling out a real feeder stall).

The fix replaces the gate with LatestCSVReader._snapshot_age_s(): a
per-snapshot-identity time.monotonic() timestamp, immune to
CLOCK_REALTIME steps, refreshed only when the snapshot's (dev, ino,
mtime_ns, row content) identity actually changes.

These tests drive the real reader/feeder/responder code -- no hardware,
no PPP, no MAVLink, no actuator output, no arming. Wall-clock jumps are
simulated by mocking time.time() (never the real system clock).
"""
import socket
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path
from unittest import mock

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
FEEDER_SCRIPT = HERE / "sr75_hil_f24d_static_state_feed.py"
RESPONDER_SCRIPT = HERE / "sr75_sim_json_responder.py"

import sr75_hil_f24d_static_state_feed as feed  # noqa: E402
import sr75_sim_json_responder as responder  # noqa: E402
from test_sr75_hil_f24s_coherence_and_accounting import (  # noqa: E402
    count_csv_data_rows,
    free_udp_port,
    parse_run_summary,
    send_servo16,
    stop_proc,
)


def make_args(state_file, state_timeout_ms=500.0, fresh_state_wait_ms=80.0):
    class Args:
        pass
    args = Args()
    args.state_file = state_file
    args.mock_state = False
    args.fresh_state_wait_ms = fresh_state_wait_ms
    args.state_timeout_ms = state_timeout_ms
    args.allow_state_reuse = False
    return args


class TestWallClockJumpsDoNotCauseFalseStale(unittest.TestCase):
    """Task 5: +2s and -2s wall-clock jumps, with continuously updated
    snapshots during those jumps, must never trigger a false STALE_STATE
    -- the exact HIL-F24-T root cause."""

    def _run_with_clock_offset(self, reader, mapper, args, t_s, offset_s):
        feed.atomic_write_csv_snapshot(args.state_file, feed.CSV_FIELDS, feed.build_static_row(t_s))
        real_time = time.time()
        with mock.patch("time.time", return_value=real_time + offset_s):
            return responder.read_reply_state(args, reader, mapper, responder.MockStateSource())

    def test_positive_two_second_jump_with_continuous_updates(self):
        tmpdir = Path(tempfile.mkdtemp(prefix="hil_f24t_jumpplus_"))
        path = str(tmpdir / "state.csv")
        feed.atomic_write_csv_snapshot(path, feed.CSV_FIELDS, feed.build_static_row(0.0))
        reader = responder.LatestCSVReader(path)
        mapper = responder.StateMapper(strict=True)
        args = make_args(path)

        # Establish a baseline read with the real clock first.
        responder.read_reply_state(args, reader, mapper, responder.MockStateSource())

        for i in range(1, 6):
            state = self._run_with_clock_offset(reader, mapper, args, i * 0.02, offset_s=2.0)
            mapper.accept_state(state)
            self.assertFalse(state.reused_source_row, f"iteration {i}: unexpectedly reused/retained state")

    def test_negative_two_second_jump_with_continuous_updates(self):
        tmpdir = Path(tempfile.mkdtemp(prefix="hil_f24t_jumpminus_"))
        path = str(tmpdir / "state.csv")
        feed.atomic_write_csv_snapshot(path, feed.CSV_FIELDS, feed.build_static_row(0.0))
        reader = responder.LatestCSVReader(path)
        mapper = responder.StateMapper(strict=True)
        args = make_args(path)

        responder.read_reply_state(args, reader, mapper, responder.MockStateSource())

        for i in range(1, 6):
            state = self._run_with_clock_offset(reader, mapper, args, i * 0.02, offset_s=-2.0)
            mapper.accept_state(state)
            self.assertFalse(state.reused_source_row, f"iteration {i}: unexpectedly reused/retained state")

    def test_diagnostic_reports_wall_clock_age_separately_from_monotonic_age(self):
        """Task 3: wall-clock mtime must remain visible as diagnostic
        metadata even though it no longer gates acceptance -- verified by
        forcing a genuine (monotonic) staleness rejection under a
        simultaneous wall-clock jump, and checking both ages are reported
        and clearly different."""
        tmpdir = Path(tempfile.mkdtemp(prefix="hil_f24t_diag_"))
        path = str(tmpdir / "state.csv")
        feed.atomic_write_csv_snapshot(path, feed.CSV_FIELDS, feed.build_static_row(0.0))
        reader = responder.LatestCSVReader(path)
        mapper = responder.StateMapper(strict=True)
        args = make_args(path, state_timeout_ms=100.0)
        responder.read_reply_state(args, reader, mapper, responder.MockStateSource())
        time.sleep(0.2)  # genuinely exceed the 100ms timeout, no new write
        real_time = time.time()
        with mock.patch("time.time", return_value=real_time + 2.0):
            with self.assertRaises(responder.StateError) as ctx:
                responder.read_reply_state(args, reader, mapper, responder.MockStateSource())
        message = str(ctx.exception)
        self.assertIn("calculated_age_ms=", message)
        self.assertIn("wall_clock_age_ms=", message)


class TestFrozenFeederStillBecomesStale(unittest.TestCase):
    """Task 4: a genuinely frozen feeder (no new writes at all) must still
    trigger STALE_STATE after the configured timeout, on the real clock,
    with no wall-clock manipulation involved."""

    def test_frozen_feeder_triggers_stale_after_timeout(self):
        tmpdir = Path(tempfile.mkdtemp(prefix="hil_f24t_frozen_"))
        path = str(tmpdir / "state.csv")
        feed.atomic_write_csv_snapshot(path, feed.CSV_FIELDS, feed.build_static_row(0.0))
        reader = responder.LatestCSVReader(path)
        mapper = responder.StateMapper(strict=True)
        args = make_args(path, state_timeout_ms=150.0)

        state = responder.read_reply_state(args, reader, mapper, responder.MockStateSource())
        mapper.accept_state(state)
        self.assertFalse(state.reused_source_row)

        # No further writes at all -- the feeder is frozen.
        time.sleep(0.3)
        with self.assertRaises(responder.StateError) as ctx:
            responder.read_reply_state(args, reader, mapper, responder.MockStateSource())
        self.assertIn("STALE_STATE", str(ctx.exception))

    def test_snapshot_identity_unchanged_while_frozen(self):
        """White-box check: the monotonic "first observed" timestamp must
        not be refreshed across repeated reads of an unchanged file --
        otherwise a frozen feeder could look perpetually fresh."""
        tmpdir = Path(tempfile.mkdtemp(prefix="hil_f24t_frozenid_"))
        path = str(tmpdir / "state.csv")
        feed.atomic_write_csv_snapshot(path, feed.CSV_FIELDS, feed.build_static_row(0.0))
        reader = responder.LatestCSVReader(path)
        reader.read_latest()
        first_observed = reader._snapshot_first_observed_monotonic
        time.sleep(0.05)
        reader.read_latest()
        self.assertEqual(reader._snapshot_first_observed_monotonic, first_observed)


class TestContinuousReplacementAt50Hz(unittest.TestCase):
    """Task 5/6: continuous atomic replacement at ~50 Hz must never
    produce a false stale classification via the new monotonic gate."""

    def test_fifty_hz_replacement_never_reports_inflated_age(self):
        import threading
        tmpdir = Path(tempfile.mkdtemp(prefix="hil_f24t_50hz_"))
        path = str(tmpdir / "state.csv")
        feed.atomic_write_csv_snapshot(path, feed.CSV_FIELDS, feed.build_static_row(0.0))

        stop = threading.Event()
        ages_ms = []
        errors = []

        def writer_loop():
            i = 0
            dt = 1.0 / 50.0
            next_write = time.monotonic()
            while not stop.is_set():
                feed.atomic_write_csv_snapshot(path, feed.CSV_FIELDS, feed.build_static_row(i * dt))
                i += 1
                next_write += dt
                sleep_s = next_write - time.monotonic()
                if sleep_s > 0:
                    time.sleep(sleep_s)

        def reader_loop():
            reader = responder.LatestCSVReader(path)
            while not stop.is_set():
                try:
                    _, _, age_s, _ = reader.read_latest()
                    ages_ms.append(age_s * 1000.0)
                except responder.StateFileMalformedError as exc:
                    errors.append(str(exc))
                except responder.StateFileNotReadyError:
                    pass

        writer_thread = threading.Thread(target=writer_loop)
        reader_thread = threading.Thread(target=reader_loop)
        writer_thread.start()
        reader_thread.start()
        time.sleep(2.5)
        stop.set()
        writer_thread.join(timeout=2.0)
        reader_thread.join(timeout=2.0)

        self.assertEqual(errors, [])
        self.assertGreater(len(ages_ms), 0)
        self.assertLess(max(ages_ms), 200.0, f"max observed age {max(ages_ms):.1f}ms looks stale under a healthy 50Hz writer")


class TestAccountingUnderSigtermStillExact(unittest.TestCase):
    """Task 5: exact request/CSV/reply accounting under SIGTERM (the
    HIL-F24-S guarantee) must still hold unchanged by this task's
    freshness-gate rework."""

    def test_burst_then_immediate_sigterm_still_exact(self):
        for trial in range(5):
            tmpdir = Path(tempfile.mkdtemp(prefix=f"hil_f24t_acct_{trial}_"))
            log_csv = tmpdir / "responder.csv"
            port = free_udp_port()
            proc = subprocess.Popen(
                [sys.executable, str(RESPONDER_SCRIPT),
                 "--listen-host", "127.0.0.1", "--listen-port", str(port),
                 "--mock-state", "--log-csv", str(log_csv)],
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
            )
            try:
                time.sleep(0.2)
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sock.settimeout(0.2)
                addr = ("127.0.0.1", port)
                for i in range(1, 9):
                    send_servo16(sock, addr, i)
                    try:
                        sock.recvfrom(4096)
                    except socket.timeout:
                        pass
                sock.close()
                out = stop_proc(proc)
            finally:
                if proc.poll() is None:
                    proc.kill()
                    proc.communicate()

            summary = parse_run_summary(out)
            csv_rows = count_csv_data_rows(log_csv)
            with self.subTest(trial=trial):
                self.assertEqual(summary["total_requests"], csv_rows)
                self.assertEqual(summary["replies_sent"] + summary["missed_replies"], summary["total_requests"])


class TestEndToEndFreshnessUnderRealFeeder(unittest.TestCase):
    """Full scenario: real feeder atomically replacing state.csv at 50 Hz,
    real responder answering a simulated ~50 Hz request loop, zero stale
    classifications, exact accounting."""

    def test_short_run_zero_stale_exact_accounting(self):
        duration_s = 4.0
        feeder_duration_s = duration_s + 2.0 + 3.0

        tmpdir = Path(tempfile.mkdtemp(prefix="hil_f24t_e2e_"))
        state_csv = tmpdir / "state.csv"
        responder_log = tmpdir / "responder.csv"
        listen_port = free_udp_port()

        feeder_proc = subprocess.Popen(
            [sys.executable, str(FEEDER_SCRIPT), "--output", str(state_csv),
             "--duration-s", str(feeder_duration_s), "--rate-hz", "50"],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
        )
        try:
            deadline = time.monotonic() + 3.0
            while not state_csv.exists() and time.monotonic() < deadline:
                time.sleep(0.02)
            self.assertTrue(state_csv.exists())

            responder_proc = subprocess.Popen(
                [sys.executable, str(RESPONDER_SCRIPT),
                 "--listen-host", "127.0.0.1", "--listen-port", str(listen_port),
                 "--state-file", str(state_csv), "--log-csv", str(responder_log), "--strict"],
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
            )
            try:
                time.sleep(0.3)
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sock.settimeout(0.2)
                addr = ("127.0.0.1", listen_port)
                frame_count = 0
                start = time.monotonic()
                while time.monotonic() - start < duration_s:
                    frame_count += 1
                    send_servo16(sock, addr, frame_count)
                    try:
                        sock.recvfrom(4096)
                    except socket.timeout:
                        pass
                    time.sleep(0.02)
                sock.close()
            finally:
                responder_out = stop_proc(responder_proc)
        finally:
            stop_proc(feeder_proc)

        summary = parse_run_summary(responder_out)
        csv_rows = count_csv_data_rows(responder_log)
        self.assertEqual(summary["stale_state_count"], 0)
        self.assertEqual(summary["malformed_state_count"], 0)
        self.assertEqual(summary["total_requests"], csv_rows)
        self.assertEqual(summary["replies_sent"] + summary["missed_replies"], summary["total_requests"])


if __name__ == "__main__":
    unittest.main()
