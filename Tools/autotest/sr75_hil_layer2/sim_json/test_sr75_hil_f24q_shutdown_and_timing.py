#!/usr/bin/env python3
"""HIL-F24-Q regression tests: eliminating the remaining static SIM_JSON
reply gaps found in the F24-P hardware run (1499 requests, 1487 replies,
12 missed -- 11 shutdown-tail misses after the feeder stopped, plus one
mid-run STALE_STATE caused by a single ~1.7s feeder write stall).

These tests drive the real feeder and responder scripts as subprocesses,
over real UDP loopback sockets, with a simulated ~5 Hz Pixhawk request
loop -- no hardware, no PPP, no MAVLink, no actuator output, no arming.
"""
import re
import socket
import struct
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
FEEDER_SCRIPT = HERE / "sr75_hil_f24d_static_state_feed.py"
RESPONDER_SCRIPT = HERE / "sr75_sim_json_responder.py"

import sr75_hil_f24d_static_state_feed as feed  # noqa: E402
import sr75_sim_json_responder as responder  # noqa: E402
from test_sr75_hil_f24p_state_csv_race import run_concurrency_stress  # noqa: E402

SERVO16_MAGIC = 18458


def free_udp_port():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


def send_servo16(sock, addr, frame_count, frame_rate=50):
    pkt = struct.pack("<HHI16H", SERVO16_MAGIC, frame_rate, frame_count, *([1500] * 16))
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


class TestEndToEndNoResponderTailAfterFeederStops(unittest.TestCase):
    """Mirrors the orchestrator's own shutdown ordering (feeder started
    first and given FEEDER_SHUTDOWN_MARGIN_S extra --duration-s, responder
    stopped first, then feeder) and proves it end to end: a simulated 5 Hz
    request loop running for the responder's nominal test window sees zero
    STALE_STATE, zero NO_FRESH_STATE, and zero missed replies."""

    def test_short_run_has_zero_stale_and_zero_no_fresh_state(self):
        duration_s = 4.0
        orchestrator_margin_s = 2.0
        feeder_shutdown_margin_s = 3.0
        feeder_duration_s = duration_s + orchestrator_margin_s + feeder_shutdown_margin_s

        tmpdir = Path(tempfile.mkdtemp(prefix="hil_f24q_e2e_"))
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
            self.assertTrue(state_csv.exists(), "feeder never produced its first snapshot")

            responder_proc = subprocess.Popen(
                [sys.executable, str(RESPONDER_SCRIPT),
                 "--listen-host", "127.0.0.1", "--listen-port", str(listen_port),
                 "--state-file", str(state_csv), "--log-csv", str(responder_log), "--strict"],
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
            )
            try:
                time.sleep(0.3)  # let the responder bind its socket

                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sock.settimeout(0.5)
                addr = ("127.0.0.1", listen_port)
                timeouts = 0
                frame_count = 0
                start = time.monotonic()
                # Simulated ~5 Hz Pixhawk SIM_JSON request loop, running for
                # exactly the responder's nominal test window (duration_s) --
                # the feeder (feeder_duration_s > duration_s +
                # orchestrator_margin_s) is still writing throughout.
                while time.monotonic() - start < duration_s:
                    frame_count += 1
                    send_servo16(sock, addr, frame_count)
                    try:
                        sock.recvfrom(4096)
                    except socket.timeout:
                        timeouts += 1
                    time.sleep(0.2)
                sock.close()
            finally:
                # Mirror the orchestrator's fixed shutdown order.
                responder_out = stop_proc(responder_proc)
        finally:
            stop_proc(feeder_proc)

        summary = parse_run_summary(responder_out)
        self.assertEqual(timeouts, 0, "at least one request got no reply at all")
        self.assertEqual(summary["stale_state_count"], 0)
        self.assertEqual(summary["no_fresh_state_count"], 0)
        self.assertEqual(summary["malformed_state_count"], 0)
        self.assertEqual(summary["missed_replies"], 0)
        self.assertGreater(summary["total_requests"], 0)
        self.assertEqual(summary["total_requests"], summary["replies_sent"])

        # Final state.csv must still be exactly one header + one row.
        with open(state_csv) as f:
            lines = [line for line in f.read().splitlines() if line.strip()]
        self.assertEqual(len(lines), 2)


class TestFeederPrintsSummaryWhenKilledMidRun(unittest.TestCase):
    """With the shutdown-ordering fix, the feeder is now *deliberately*
    still running when the orchestrator SIGTERMs it. Proves the feeder's
    write-count/latency/scheduling-gap metrics (task 5) still get printed
    in that case, instead of being silently lost."""

    def test_sigterm_mid_run_still_prints_write_summary(self):
        tmpdir = Path(tempfile.mkdtemp(prefix="hil_f24q_feeder_sigterm_"))
        path = tmpdir / "state.csv"
        proc = subprocess.Popen(
            [sys.executable, str(FEEDER_SCRIPT), "--output", str(path),
             "--duration-s", "10", "--rate-hz", "50"],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
        )
        time.sleep(1.0)
        out = stop_proc(proc)
        self.assertIn("Stopped by SIGTERM", out)
        m = re.search(r"WRITE_SUMMARY (.+)", out)
        self.assertIsNotNone(m, f"WRITE_SUMMARY line not found:\n{out}")
        fields = {}
        for kv in m.group(1).split():
            k, v = kv.split("=", 1)
            fields[k] = v
        self.assertGreater(int(fields["write_count"]), 0)
        self.assertEqual(fields["fsync_enabled"], "0")


class TestFsyncDefaultOffPreservesAtomicity(unittest.TestCase):
    """HIL-F24-Q changed atomic_write_csv_snapshot()'s fsync default from
    always-on to off. Confirms: (a) the default really is off and costs
    nothing; (b) --fsync=True still works and is timed; (c) the
    concurrency stress guarantee from HIL-F24-P (zero malformed snapshots
    under racing reads) still holds with the new default."""

    def test_default_fsync_is_off_and_recorded_as_zero(self):
        tmpdir = Path(tempfile.mkdtemp(prefix="hil_f24q_fsync_"))
        path = str(tmpdir / "state.csv")
        stats = feed.atomic_write_csv_snapshot(path, feed.CSV_FIELDS, feed.build_static_row(0.0))
        self.assertEqual(stats.fsync_s, 0.0)

    def test_fsync_true_is_timed_and_still_atomic(self):
        tmpdir = Path(tempfile.mkdtemp(prefix="hil_f24q_fsync_on_"))
        path = str(tmpdir / "state.csv")
        stats = feed.atomic_write_csv_snapshot(path, feed.CSV_FIELDS, feed.build_static_row(0.0), fsync=True)
        self.assertGreaterEqual(stats.fsync_s, 0.0)
        with open(path) as f:
            lines = [line for line in f.read().splitlines() if line.strip()]
        self.assertEqual(len(lines), 2)

    def test_concurrency_stress_still_zero_errors_with_fsync_off_default(self):
        write_count, read_count, errors = run_concurrency_stress(duration_s=2.0)
        self.assertGreater(write_count, 50)
        self.assertGreater(read_count, 0)
        self.assertEqual(errors, [])


class TestWriteLatencyInstrumentation(unittest.TestCase):
    """Proves the per-write timing/percentile/scheduling-gap instrumentation
    itself is correct, independent of any particular hardware stall."""

    def test_percentile_matches_known_distribution(self):
        values = sorted(float(v) for v in range(1, 101))  # 1..100
        self.assertAlmostEqual(feed._percentile(values, 50), 50.5, places=3)
        self.assertAlmostEqual(feed._percentile(values, 100), 100.0, places=3)
        self.assertAlmostEqual(feed._percentile(values, 0), 1.0, places=3)

    def test_print_write_summary_reports_expected_fields(self):
        import io
        from contextlib import redirect_stdout
        stats = [
            feed.SnapshotWriteStats(total_s=0.001, fsync_s=0.0),
            feed.SnapshotWriteStats(total_s=0.002, fsync_s=0.0),
            feed.SnapshotWriteStats(total_s=1.703, fsync_s=1.700),  # simulated request-573-style stall
        ]
        buf = io.StringIO()
        with redirect_stdout(buf):
            feed.print_write_summary(stats, longest_gap_s=1.703, fsync_enabled=True)
        out = buf.getvalue()
        self.assertIn("write_count=3", out)
        self.assertIn("max_write_latency_ms=1703.000", out)
        self.assertIn("longest_scheduling_gap_ms=1703.000", out)
        self.assertIn("fsync_enabled=1", out)


class TestSimJsonWireFormatUnchanged(unittest.TestCase):
    """HIL-F24-Q touched only feeder timing/fsync, responder run-summary
    counters, and orchestrator process lifecycle -- none of it is the
    SIM_JSON wire payload."""

    def test_to_json_bytes_schema_unchanged(self):
        state = responder.SimState(
            timestamp_s=1.0, source_timestamp_s=1.0, latitude_deg=10.0, longitude_deg=20.0,
            altitude_m=30.0, roll_rad=0.0, pitch_rad=0.0, yaw_rad=0.0,
            quaternion=(1.0, 0.0, 0.0, 0.0), gyro_rad_s=(0.0, 0.0, 0.0),
            accel_body_mss=(0.0, 0.0, -9.8), velocity_ned_mps=(0.0, 0.0, 0.0),
            airspeed_mps=0.0, row_signature="sig", row_monotonic_time=0.0,
        )
        import json
        payload = json.loads(state.to_json_bytes())
        self.assertEqual(
            sorted(payload.keys()),
            sorted(["timestamp", "latitude", "longitude", "altitude", "imu",
                    "velocity", "attitude", "quaternion", "airspeed"]),
        )


if __name__ == "__main__":
    unittest.main()
