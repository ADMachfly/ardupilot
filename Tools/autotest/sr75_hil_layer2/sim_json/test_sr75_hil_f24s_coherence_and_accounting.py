#!/usr/bin/env python3
"""HIL-F24-S regression tests: LatestCSVReader TOCTOU coherence, freshness-
rejection instrumentation, and shutdown request-accounting atomicity.

Root cause recap (see the HIL-F24-S report for the full trace): the reader
combined pathname-based os.stat()/os.path.exists() metadata with a
*separately opened* file's content -- up to four distinct filesystem
calls per read, any two of which could straddle the feeder's atomic
os.replace() and therefore observe two different snapshots. This produced
an incorrect (inflated) row age for an otherwise fresh row (request 784,
STALE_STATE age 1546.5ms). Separately, request_count could be incremented
for a request whose responder.csv row was never written if SIGTERM landed
in the (previously wide) window between the two.

These tests drive the real reader/feeder/responder code -- no hardware,
no PPP, no MAVLink, no actuator output, no arming.
"""
import builtins
import re
import socket
import struct
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


def free_udp_port():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


def send_servo16(sock, addr, frame_count, frame_rate=50):
    pkt = struct.pack("<HHI16H", 18458, frame_rate, frame_count, *([1500] * 16))
    sock.sendto(pkt, addr)


def parse_run_summary(output):
    m = re.search(r"RUN_SUMMARY (.+)", output)
    assert m, f"RUN_SUMMARY line not found in responder output:\n{output}"
    return {k: int(v) for k, v in (kv.split("=", 1) for kv in m.group(1).split())}


def stop_proc(proc, timeout_s=5.0):
    if proc.poll() is not None:
        return proc.communicate()[0]
    proc.terminate()
    try:
        out, _ = proc.communicate(timeout=timeout_s)
    except subprocess.TimeoutExpired:
        proc.kill()
        out, _ = proc.communicate()
    return out


def count_csv_data_rows(path):
    with open(path) as f:
        lines = [line for line in f.read().splitlines() if line.strip()]
    return max(0, len(lines) - 1)  # minus header


class TestReaderCoherence(unittest.TestCase):
    """Task 2/3: every read_latest() call must open the snapshot exactly
    once and use os.fstat() on that same descriptor -- never a second,
    separate os.stat(path) call that could race the feeder's replace()."""

    def test_source_never_calls_pathname_stat(self):
        """Walks the actual parsed AST (comments/docstrings don't produce
        Call nodes, so this can't false-positive on the class's own prose
        discussing the old os.stat()/os.path.exists() calls it replaced,
        unlike a plain substring search)."""
        import ast
        import inspect
        tree = ast.parse(inspect.getsource(responder.LatestCSVReader))
        call_dotted_names = []
        for node in ast.walk(tree):
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
                parts = []
                cur = node.func
                while isinstance(cur, ast.Attribute):
                    parts.append(cur.attr)
                    cur = cur.value
                if isinstance(cur, ast.Name):
                    parts.append(cur.id)
                call_dotted_names.append(".".join(reversed(parts)))
        self.assertNotIn("os.stat", call_dotted_names)
        self.assertNotIn("os.path.exists", call_dotted_names)
        self.assertIn("os.fstat", call_dotted_names)

    def test_read_latest_opens_the_file_exactly_once(self):
        tmpdir = Path(tempfile.mkdtemp(prefix="hil_f24s_singleopen_"))
        path = str(tmpdir / "state.csv")
        feed.atomic_write_csv_snapshot(path, feed.CSV_FIELDS, feed.build_static_row(0.0))
        reader = responder.LatestCSVReader(path)

        real_open = builtins.open
        open_calls = []

        def counting_open(*args, **kwargs):
            open_calls.append(args[0] if args else kwargs.get("file"))
            return real_open(*args, **kwargs)

        with mock.patch.object(responder, "open", side_effect=counting_open, create=True):
            reader.read_latest()
        self.assertEqual(len(open_calls), 1, f"expected exactly one open() call, saw {open_calls}")

    def test_header_cache_keyed_by_dev_ino_mtime_not_just_mtime(self):
        """Task 3: the header cache key must be (dev, ino, mtime_ns), not
        a bare mtime, so it can never be satisfied by a different inode
        that happens to share a timestamp."""
        tmpdir = Path(tempfile.mkdtemp(prefix="hil_f24s_headercache_"))
        path = str(tmpdir / "state.csv")
        feed.atomic_write_csv_snapshot(path, feed.CSV_FIELDS, feed.build_static_row(0.0))
        reader = responder.LatestCSVReader(path)
        reader.read_latest()
        self.assertIsNotNone(reader._header_cache_key)
        self.assertEqual(len(reader._header_cache_key), 3)  # (dev, ino, mtime_ns)

    def test_returned_stat_matches_the_content_just_read(self):
        """A direct coherence check: os.fstat() taken inside read_latest()
        must describe the exact file whose row was parsed. Verified by
        comparing it against an independent os.stat(path) taken
        immediately after read_latest() returns, with nothing else
        writing to the file in between -- both must describe the same
        (device, inode)."""
        import os
        tmpdir = Path(tempfile.mkdtemp(prefix="hil_f24s_fstat_"))
        path = str(tmpdir / "state.csv")
        feed.atomic_write_csv_snapshot(path, feed.CSV_FIELDS, feed.build_static_row(0.0))
        reader = responder.LatestCSVReader(path)
        _, _, _, st = reader.read_latest()
        st_after = os.stat(path)
        self.assertEqual((st.st_dev, st.st_ino), (st_after.st_dev, st_after.st_ino))


class TestZeroFalseStaleUnderConcurrentReplace(unittest.TestCase):
    """Task 6: continuously atomically replacing state.csv while readers
    repeatedly open it must never produce an inflated/incorrect age --
    the exact failure mode behind request 784."""

    def test_concurrent_reads_never_report_an_inflated_age(self):
        import threading
        tmpdir = Path(tempfile.mkdtemp(prefix="hil_f24s_falsestale_"))
        path = str(tmpdir / "state.csv")
        feed.atomic_write_csv_snapshot(path, feed.CSV_FIELDS, feed.build_static_row(0.0))

        stop = threading.Event()
        ages_ms = []
        malformed_errors = []
        write_hz = 100.0  # faster than the ~50 Hz hardware feeder, to stress harder

        def writer_loop():
            i = 0
            dt = 1.0 / write_hz
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
                    # HIL-F24-T: read_latest()'s 3rd element is a
                    # monotonic-clock-based age in seconds (see
                    # LatestCSVReader._snapshot_age_s()), not a wall-clock
                    # mtime -- it is used directly, not compared against
                    # time.time().
                    _, _, age_s, _ = reader.read_latest()
                    ages_ms.append(age_s * 1000.0)
                except responder.StateFileMalformedError as exc:
                    malformed_errors.append(str(exc))
                except responder.StateFileNotReadyError:
                    pass

        writer_thread = threading.Thread(target=writer_loop)
        reader_threads = [threading.Thread(target=reader_loop) for _ in range(4)]
        writer_thread.start()
        for t in reader_threads:
            t.start()
        time.sleep(2.5)
        stop.set()
        writer_thread.join(timeout=2.0)
        for t in reader_threads:
            t.join(timeout=2.0)

        self.assertEqual(malformed_errors, [])
        self.assertGreater(len(ages_ms), 0)
        # A genuinely fresh row under a ~10ms writer cadence should never
        # be reported as hundreds of ms old -- the TOCTOU bug would show
        # up here as isolated, wildly-inflated outliers (like request
        # 784's 1546.5ms among ~5-8ms neighbors).
        self.assertLess(max(ages_ms), 500.0, f"max observed age {max(ages_ms):.1f}ms looks like a TOCTOU mismatch")


class TestShutdownAccountingAtomicity(unittest.TestCase):
    """Task 5/6: total_requests must equal responder.csv's data-row count,
    and replies_sent + (total_requests - replies_sent) must equal
    total_requests, even when SIGTERM lands immediately after a burst of
    requests (maximizing the chance of catching the pre-fix race, where
    request_count could be incremented without its row ever being
    written)."""

    def test_burst_then_immediate_sigterm_never_mismatches(self):
        trials = 15
        for trial in range(trials):
            tmpdir = Path(tempfile.mkdtemp(prefix=f"hil_f24s_acct_{trial}_"))
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
                for i in range(1, 11):
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
                self.assertEqual(
                    summary["total_requests"], csv_rows,
                    f"trial {trial}: total_requests={summary['total_requests']} != csv_rows={csv_rows}",
                )
                self.assertEqual(
                    summary["replies_sent"] + summary["missed_replies"], summary["total_requests"],
                )


class TestEndToEndCoherenceAndAccountingUnderRealFeeder(unittest.TestCase):
    """The full task-6 scenario: real feeder atomically replacing
    state.csv continuously, real responder answering a simulated request
    loop, proving zero false-stale, zero malformed, and exact
    CSV/counter accounting end to end."""

    def test_short_run_zero_stale_zero_malformed_exact_accounting(self):
        duration_s = 4.0
        feeder_duration_s = duration_s + 2.0 + 3.0  # mirrors the orchestrator's HIL-F24-Q margins

        tmpdir = Path(tempfile.mkdtemp(prefix="hil_f24s_e2e_"))
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
                # ~50 Hz simulated request loop, matching the hardware rate.
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

        with open(state_csv) as f:
            lines = [line for line in f.read().splitlines() if line.strip()]
        self.assertEqual(len(lines), 2)  # final state.csv still valid: header + 1 row


if __name__ == "__main__":
    unittest.main()
