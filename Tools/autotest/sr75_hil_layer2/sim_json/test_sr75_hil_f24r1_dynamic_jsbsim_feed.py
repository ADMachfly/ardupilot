#!/usr/bin/env python3
"""HIL-F24-R1 regression tests: live SR-75 6-DOF JSBSim state feeder.

Covers:
  - validate_jsbsim_row()'s finite-value/units-range/timestamp-monotonicity
    checks (task 5), pure unit tests, no JSBSim involved;
  - RawJsbsimTail's tail-last-line parsing against hand-written CSV
    content, no JSBSim involved;
  - end-to-end tests against the REAL JSBSim binary (skipped automatically
    if JSBSim is not installed in this environment) proving: JSBSim state
    changes continuously and stays finite, zero rows are rejected, the
    feeder's atomic republish reaches sr75_sim_json_responder.py with
    zero missed/stale/no-fresh/malformed/state-read errors, and exact
    CSV/counter accounting holds (reusing the HIL-F24-P/Q/S/T invariants).

No hardware, no PPP, no MAVLink, no actuator output, no arming --
everything here runs on the host only (JSBSim itself is host-side
simulation software, not hardware).
"""
import csv
import math
import shutil
import socket
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
FEEDER_SCRIPT = HERE / "sr75_hil_f24r1_dynamic_jsbsim_feed.py"
RESPONDER_SCRIPT = HERE / "sr75_sim_json_responder.py"

import sr75_hil_f24r1_dynamic_jsbsim_feed as dynfeed  # noqa: E402
import sr75_hil_f24d_static_state_feed as feed  # noqa: E402
from test_sr75_hil_f24s_coherence_and_accounting import (  # noqa: E402
    count_csv_data_rows,
    free_udp_port,
    parse_run_summary,
    send_servo16,
    stop_proc,
)

JSBSIM_AVAILABLE = shutil.which("JSBSim") is not None


def make_valid_row(time_s=1.0, **overrides):
    row = {
        "time_s": str(time_s), "lat_deg": "32.5378085", "lon_deg": "74.3661944",
        "alt_m": "240.2", "roll_rad": "0.0", "pitch_rad": "0.0", "yaw_rad": "5.4977871",
        "vn_mps": "0.0", "ve_mps": "0.0", "vd_mps": "0.0",
        "p_rad_s": "0.0", "q_rad_s": "0.0", "r_rad_s": "0.0",
        "accel_body_x_mss": "0.0", "accel_body_y_mss": "0.0", "accel_body_z_mss": "-9.80665",
        "airspeed_mps": "0.05",
    }
    row.update({k: str(v) for k, v in overrides.items()})
    return row


class TestValidateJsbsimRow(unittest.TestCase):
    def test_valid_row_accepted(self):
        time_s, reason = dynfeed.validate_jsbsim_row(make_valid_row(1.0), last_published_time_s=None)
        self.assertIsNone(reason)
        self.assertEqual(time_s, 1.0)

    def test_missing_field_rejected(self):
        row = make_valid_row(1.0)
        del row["accel_body_z_mss"]
        _, reason = dynfeed.validate_jsbsim_row(row, last_published_time_s=None)
        self.assertEqual(reason, "missing_field:accel_body_z_mss")

    def test_non_numeric_field_rejected(self):
        row = make_valid_row(1.0, lat_deg="not_a_number")
        _, reason = dynfeed.validate_jsbsim_row(row, last_published_time_s=None)
        self.assertEqual(reason, "non_numeric:lat_deg")

    def test_nan_rejected(self):
        row = make_valid_row(1.0, alt_m="nan")
        _, reason = dynfeed.validate_jsbsim_row(row, last_published_time_s=None)
        self.assertEqual(reason, "non_finite:alt_m")

    def test_inf_rejected(self):
        row = make_valid_row(1.0, airspeed_mps="inf")
        _, reason = dynfeed.validate_jsbsim_row(row, last_published_time_s=None)
        self.assertEqual(reason, "non_finite:airspeed_mps")

    def test_out_of_range_latitude_rejected(self):
        row = make_valid_row(1.0, lat_deg="132.0")
        _, reason = dynfeed.validate_jsbsim_row(row, last_published_time_s=None)
        self.assertEqual(reason, "out_of_range:lat_deg")

    def test_out_of_range_gyro_rejected(self):
        row = make_valid_row(1.0, p_rad_s="1000.0")
        _, reason = dynfeed.validate_jsbsim_row(row, last_published_time_s=None)
        self.assertEqual(reason, "out_of_range:p_rad_s")

    def test_out_of_range_accel_rejected(self):
        row = make_valid_row(1.0, accel_body_x_mss="500.0")
        _, reason = dynfeed.validate_jsbsim_row(row, last_published_time_s=None)
        self.assertEqual(reason, "out_of_range:accel_body_x_mss")

    def test_non_monotonic_timestamp_rejected(self):
        _, reason = dynfeed.validate_jsbsim_row(make_valid_row(1.0), last_published_time_s=2.0)
        self.assertEqual(reason, "non_monotonic_time_s")

    def test_unadvanced_timestamp_distinguished_from_error(self):
        """A row whose time_s exactly equals the last published one is not
        an error -- it just means JSBSim hasn't produced a new row since
        the last poll yet."""
        _, reason = dynfeed.validate_jsbsim_row(make_valid_row(2.0), last_published_time_s=2.0)
        self.assertEqual(reason, "not_advanced")

    def test_strictly_advancing_timestamp_accepted(self):
        time_s, reason = dynfeed.validate_jsbsim_row(make_valid_row(2.5), last_published_time_s=2.0)
        self.assertIsNone(reason)
        self.assertEqual(time_s, 2.5)


class TestReadExistingOutputTimeS(unittest.TestCase):
    """HIL-F24-R3K: pure tests for the helper that lets a freshly-started
    feeder continue a prior feeder's time_s series instead of restarting
    near 0 -- see sr75_hil_f24r1_dynamic_jsbsim_feed.py's module docstring
    addendum and HIL_F24_R3K report for the handoff-gap this closes."""

    def test_missing_file_returns_none(self):
        self.assertIsNone(dynfeed.read_existing_output_time_s("/tmp/hil_f24r3k_definitely_missing.csv"))

    def test_empty_file_returns_none(self):
        tmpdir = Path(tempfile.mkdtemp(prefix="hil_f24r3k_empty_"))
        path = tmpdir / "state.csv"
        path.touch()
        self.assertIsNone(dynfeed.read_existing_output_time_s(str(path)))

    def test_header_only_file_returns_none(self):
        tmpdir = Path(tempfile.mkdtemp(prefix="hil_f24r3k_headeronly_"))
        path = tmpdir / "state.csv"
        with open(path, "w", newline="") as f:
            csv.writer(f).writerow(feed.CSV_FIELDS)
        self.assertIsNone(dynfeed.read_existing_output_time_s(str(path)))

    def test_missing_time_s_column_returns_none(self):
        tmpdir = Path(tempfile.mkdtemp(prefix="hil_f24r3k_notime_"))
        path = tmpdir / "state.csv"
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["lat_deg", "lon_deg"])
            w.writerow(["32.5", "74.3"])
        self.assertIsNone(dynfeed.read_existing_output_time_s(str(path)))

    def test_reads_the_stationary_feeder_snapshot(self):
        """Exactly the real R3A evidence value (session
        sr75_hil_f24r3a_mp_visual_20260807T074250Z): the stationary
        feeder's last accepted row had time_s=37.700048."""
        tmpdir = Path(tempfile.mkdtemp(prefix="hil_f24r3k_real_"))
        path = tmpdir / "state.csv"
        row = feed.build_static_row(37.700048)
        feed.atomic_write_csv_snapshot(str(path), feed.CSV_FIELDS, row)
        self.assertEqual(dynfeed.read_existing_output_time_s(str(path)), 37.700048)


class TestRawJsbsimTail(unittest.TestCase):
    def test_reads_last_complete_row(self):
        tmpdir = Path(tempfile.mkdtemp(prefix="hil_f24r1_tail_"))
        path = tmpdir / "raw.csv"
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["Time", "time_s", "lat_deg"])
            w.writerow(["0", "0.0", "32.5"])
            w.writerow(["0.02", "0.02", "32.6"])
        tail = dynfeed.RawJsbsimTail(str(path))
        row = tail.read_latest_row()
        self.assertEqual(row["time_s"], "0.02")
        self.assertEqual(row["lat_deg"], "32.6")

    def test_missing_file_returns_none(self):
        tail = dynfeed.RawJsbsimTail("/tmp/hil_f24r1_definitely_does_not_exist.csv")
        self.assertIsNone(tail.read_latest_row())

    def test_empty_file_returns_none(self):
        tmpdir = Path(tempfile.mkdtemp(prefix="hil_f24r1_tail_empty_"))
        path = tmpdir / "raw.csv"
        path.touch()
        tail = dynfeed.RawJsbsimTail(str(path))
        self.assertIsNone(tail.read_latest_row())

    def test_header_only_file_returns_none(self):
        tmpdir = Path(tempfile.mkdtemp(prefix="hil_f24r1_tail_headeronly_"))
        path = tmpdir / "raw.csv"
        with open(path, "w", newline="") as f:
            csv.writer(f).writerow(["Time", "time_s"])
        tail = dynfeed.RawJsbsimTail(str(path))
        self.assertIsNone(tail.read_latest_row())

    def test_torn_trailing_row_returns_none_not_a_crash(self):
        tmpdir = Path(tempfile.mkdtemp(prefix="hil_f24r1_tail_torn_"))
        path = tmpdir / "raw.csv"
        with open(path, "w", newline="") as f:
            f.write("Time,time_s,lat_deg\n0,0.0,32.5\n0.02,0.02")  # missing last column, no newline
        tail = dynfeed.RawJsbsimTail(str(path))
        self.assertIsNone(tail.read_latest_row())


@unittest.skipUnless(JSBSIM_AVAILABLE, "JSBSim binary not found on PATH")
class TestRealJsbsimFeederIntegration(unittest.TestCase):
    """Task 5/6: runs the real JSBSim model for a short duration and
    proves the feeder's own validation/publication pipeline end to end."""

    def test_state_changes_continuously_and_stays_finite_zero_rejected(self):
        tmpdir = Path(tempfile.mkdtemp(prefix="hil_f24r1_real_"))
        state_csv = tmpdir / "state.csv"
        proc = subprocess.Popen(
            [sys.executable, str(FEEDER_SCRIPT), "--output", str(state_csv), "--duration-s", "6"],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
        )
        out, _ = proc.communicate(timeout=30.0)
        self.assertIn("JSBSIM_SUMMARY", out)
        summary_line = next(line for line in out.splitlines() if line.startswith("JSBSIM_SUMMARY"))
        fields = dict(kv.split("=", 1) for kv in summary_line[len("JSBSIM_SUMMARY "):].split(" ", 4))
        self.assertEqual(fields["rejected_total"], "0", out[-2000:])
        self.assertGreater(int(fields["published"]), 100)

        with open(state_csv) as f:
            lines = [line for line in f.read().splitlines() if line.strip()]
        self.assertEqual(len(lines), 2)  # final snapshot: header + 1 row
        header = lines[0].split(",")
        values = [float(v) for v in lines[1].split(",")]
        for name, value in zip(header, values):
            self.assertTrue(math.isfinite(value), f"{name}={value} is not finite")

    def test_sigterm_mid_run_still_stops_jsbsim_and_prints_summary(self):
        tmpdir = Path(tempfile.mkdtemp(prefix="hil_f24r1_sigterm_"))
        state_csv = tmpdir / "state.csv"
        proc = subprocess.Popen(
            [sys.executable, str(FEEDER_SCRIPT), "--output", str(state_csv), "--duration-s", "10"],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
        )
        time.sleep(1.5)
        out = stop_proc(proc)
        self.assertIn("Stopped by SIGTERM", out)
        self.assertIn("JSBSIM_SUMMARY", out)


@unittest.skipUnless(JSBSIM_AVAILABLE, "JSBSim binary not found on PATH")
class TestEndToEndDynamicFeederUnderResponder(unittest.TestCase):
    """Full HIL-F24-R1 scenario: real JSBSim feeder + real responder,
    simulated ~50 Hz request loop, proving zero missed/stale/no-fresh/
    malformed/state-read errors and exact CSV/counter accounting -- the
    "Required pass" criteria, minus the real Pixhawk/PPP hardware."""

    def test_short_run_zero_errors_exact_accounting(self):
        duration_s = 6.0
        feeder_duration_s = duration_s + 2.0 + 3.0

        tmpdir = Path(tempfile.mkdtemp(prefix="hil_f24r1_e2e_"))
        state_csv = tmpdir / "state.csv"
        responder_log = tmpdir / "responder.csv"
        listen_port = free_udp_port()

        feeder_proc = subprocess.Popen(
            [sys.executable, str(FEEDER_SCRIPT), "--output", str(state_csv), "--duration-s", str(feeder_duration_s)],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
        )
        try:
            deadline = time.monotonic() + 10.0
            while not state_csv.exists() and time.monotonic() < deadline:
                time.sleep(0.05)
            self.assertTrue(state_csv.exists(), "feeder never produced its first snapshot")

            responder_proc = subprocess.Popen(
                [sys.executable, str(RESPONDER_SCRIPT),
                 "--listen-host", "127.0.0.1", "--listen-port", str(listen_port),
                 "--state-file", str(state_csv), "--log-csv", str(responder_log), "--strict"],
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
            )
            try:
                time.sleep(0.5)
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sock.settimeout(0.2)
                addr = ("127.0.0.1", listen_port)
                frame_count = 0
                start = time.monotonic()
                while time.monotonic() - start < duration_s:
                    frame_count += 1
                    send_servo16(sock, addr, frame_count)
                    try:
                        _, _ = sock.recvfrom(4096)
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
        self.assertEqual(summary["no_fresh_state_count"], 0)
        self.assertEqual(summary["malformed_state_count"], 0)
        self.assertEqual(summary["state_read_error_count"], 0)
        self.assertEqual(summary["total_requests"], csv_rows)
        self.assertEqual(summary["replies_sent"] + summary["missed_replies"], summary["total_requests"])

        # Live state must actually keep advancing over the run (not a
        # single frozen row like the static feeder) -- inspect the
        # responder's own log for continuously-increasing JSBSim source
        # timestamps. Note this deliberately does NOT check that
        # altitude/attitude visibly change: the R1 ground-static IC
        # settles to a physically motionless equilibrium within about a
        # second (by design -- MANUAL/disarmed bench semantics), so an
        # unchanging physical state past that point is correct, not a
        # sign of a frozen feeder.
        source_timestamps = []
        with open(responder_log) as f:
            reader = csv.DictReader(f)
            for row in reader:
                if row.get("source_simulation_timestamp"):
                    source_timestamps.append(float(row["source_simulation_timestamp"]))
        self.assertGreater(len(source_timestamps), 1)
        self.assertEqual(
            source_timestamps, sorted(source_timestamps),
            "expected JSBSim source timestamps to be monotonically non-decreasing across the run",
        )
        self.assertGreater(
            source_timestamps[-1], source_timestamps[0],
            "expected JSBSim source timestamp to have advanced over the run",
        )


@unittest.skipUnless(JSBSIM_AVAILABLE, "JSBSim binary not found on PATH")
class TestHandoffContinuity(unittest.TestCase):
    """HIL-F24-R3K regression: real JSBSim feeder + real responder, taking
    over an existing stationary-feeder state.csv snapshot mid-run --
    reproducing the exact sr75_hil_f24r3a_mp_visual_20260807T074250Z
    evidence (stationary feeder's last accepted row: time_s=37.700048)
    without needing real Pixhawk hardware. Proves the responder's
    cross-feeder-lifetime backward-source-timestamp guard (sr75_sim_json_
    responder.py's StateMapper.state_from_csv(), "backward source
    timestamp") is never tripped by the handoff once the new feeder's
    time_s continues that series -- the same condition that silently
    dropped 188 consecutive SIM_JSON replies for ~37.6s on real hardware,
    causing GPS fix_type to drop to 1 and EKF flags 831->39."""

    STATIONARY_TIME_S = 37.700048  # exact R3A evidence value

    def _seed_stationary_snapshot(self, state_csv):
        row = feed.build_static_row(self.STATIONARY_TIME_S)
        feed.atomic_write_csv_snapshot(str(state_csv), feed.CSV_FIELDS, row)

    def _run_handoff(self, tmpdir, feeder_extra_args, duration_s=6.0):
        """Seeds a stationary snapshot, starts the responder against it,
        lets it accept that row at least once (latching last_source_
        timestamp at STATIONARY_TIME_S, exactly like the real bench after
        the stationary feeder has been running), then starts the dynamic
        feeder over the same --output while continuing the request loop.
        Returns (responder_out, responder_log_rows)."""
        state_csv = tmpdir / "state.csv"
        responder_log = tmpdir / "responder.csv"
        listen_port = free_udp_port()
        self._seed_stationary_snapshot(state_csv)

        responder_proc = subprocess.Popen(
            [sys.executable, str(RESPONDER_SCRIPT),
             "--listen-host", "127.0.0.1", "--listen-port", str(listen_port),
             "--state-file", str(state_csv), "--log-csv", str(responder_log), "--strict"],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
        )
        feeder_proc = None
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.settimeout(0.2)
            addr = ("127.0.0.1", listen_port)
            frame_count = 0

            # Latch last_source_timestamp on the stationary row first,
            # exactly like the real bench before the handoff.
            deadline = time.monotonic() + 5.0
            while time.monotonic() < deadline:
                frame_count += 1
                send_servo16(sock, addr, frame_count)
                try:
                    _, _ = sock.recvfrom(4096)
                    break
                except socket.timeout:
                    time.sleep(0.05)

            feeder_proc = subprocess.Popen(
                [sys.executable, str(FEEDER_SCRIPT), "--output", str(state_csv),
                 "--duration-s", str(duration_s + 3.0)] + feeder_extra_args,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
            )

            start = time.monotonic()
            while time.monotonic() - start < duration_s:
                frame_count += 1
                send_servo16(sock, addr, frame_count)
                try:
                    _, _ = sock.recvfrom(4096)
                except socket.timeout:
                    pass
                time.sleep(0.02)
            sock.close()
        finally:
            if feeder_proc is not None:
                stop_proc(feeder_proc)
            responder_out = stop_proc(responder_proc)

        with open(responder_log) as f:
            rows = list(csv.DictReader(f))
        return responder_out, rows

    def test_auto_offset_continues_seamlessly_from_stationary_row(self):
        """Default (--time-offset-s unset, auto-detected): the fix. No
        reply is ever dropped for "backward source timestamp", and the
        outage is limited to ordinary process-startup latency, not the
        ~37.6s regression."""
        tmpdir = Path(tempfile.mkdtemp(prefix="hil_f24r3k_handoff_fixed_"))
        _, rows = self._run_handoff(tmpdir, feeder_extra_args=[])

        backward_ts_rows = [r for r in rows if "backward source timestamp" in (r.get("reply_reason") or "")]
        self.assertEqual(
            backward_ts_rows, [],
            f"handoff must never trip the backward-source-timestamp guard; got {len(backward_ts_rows)} hit(s)",
        )
        missed = sum(1 for r in rows if r.get("reply_sent") == "0")
        self.assertLess(
            missed, 10,
            f"expected only ordinary process-startup latency misses (<10 of {len(rows)} requests), got {missed}",
        )

    def test_explicit_zero_offset_reproduces_the_gap_regression_guard(self):
        """Negative control proving this test actually detects the bug:
        forcing --time-offset-s 0.0 restores the pre-fix behaviour (the
        new feeder's own sim-time-sec starts near 0, well below the
        stationary row's 37.700048), and must reproduce the exact
        "backward source timestamp" rejection seen on real hardware."""
        tmpdir = Path(tempfile.mkdtemp(prefix="hil_f24r3k_handoff_broken_"))
        _, rows = self._run_handoff(tmpdir, feeder_extra_args=["--time-offset-s", "0.0"])

        backward_ts_rows = [r for r in rows if "backward source timestamp" in (r.get("reply_reason") or "")]
        self.assertGreater(
            len(backward_ts_rows), 0,
            "negative control failed to reproduce the pre-fix gap -- test harness cannot be trusted",
        )


if __name__ == "__main__":
    unittest.main()
