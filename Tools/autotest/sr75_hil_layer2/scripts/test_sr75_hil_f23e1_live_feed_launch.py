#!/usr/bin/env python3
"""HIL-F23-E1/E2A unit tests for sr75_hil_f23e1_live_feed_launch.py.

No hardware, no MAVLink connection, no JSBSim/ArduPlane process -- pure
function tests over runscript generation, command-building (including
--mp-out propagation), subprocess-supervision/early-exit detection,
producer-readiness gating (missing/header-only/malformed CSV, first valid
row), the comparison-summary math, and artifact-retention-on-failure.
"""
import csv
import math
import tempfile
import time
import unittest
from pathlib import Path

import sr75_hil_f23e1_live_feed_launch as launch


class TestBuildOpenloopRunscript(unittest.TestCase):
    def test_writes_runscript_and_returns_paths(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            runscript_path, state_csv = launch.build_openloop_runscript(
                "some_runtime_ic", Path(tmpdir), "unittest_case", 0.10, 0.37, 90.0,
            )
            self.assertTrue(runscript_path.exists())
            self.assertEqual(runscript_path.parent, Path(tmpdir))
            self.assertEqual(state_csv.name, "sr75_hil_f23e1_unittest_case_state.csv")
            content = runscript_path.read_text()
            self.assertIn('initialize="some_runtime_ic"', content)
            self.assertIn('<run start="0.0" end="90.0"', content)
            self.assertIn('value="0.1"', content)
            self.assertIn('value="0.37"', content)

    def test_no_actuator_input_port(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            runscript_path, _ = launch.build_openloop_runscript(
                "ic", Path(tmpdir), "case", 0.1, 0.37, 30.0,
            )
            content = runscript_path.read_text()
            self.assertNotIn("<input", content)

    def test_output_schema_matches_bridge_aliases(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            runscript_path, _ = launch.build_openloop_runscript(
                "ic", Path(tmpdir), "case", 0.1, 0.37, 30.0,
            )
            content = runscript_path.read_text()
            for required_property in (
                "simulation/sim-time-sec",
                "position/lat-gc-deg",
                "position/long-gc-deg",
                "position/h-sl-ft",
                "velocities/v-north-fps",
                "velocities/v-east-fps",
                "velocities/v-down-fps",
                "velocities/vt-fps",
                "attitude/phi-deg",
                "attitude/theta-deg",
                "attitude/psi-deg",
                "velocities/p-rad_sec",
                "velocities/q-rad_sec",
                "velocities/r-rad_sec",
            ):
                self.assertIn(required_property, content)


class TestMpOutPropagation(unittest.TestCase):
    """HIL-F23-E2A item 1: --mp-out must reach the bridge command unchanged."""

    def test_mp_out_included_when_given(self):
        cmd = launch.build_bridge_cmd(
            "/dev/ttyACM0", 115200, "/tmp/state.csv", "/tmp/mission.waypoints",
            32.5378147, 74.3661871, 43.85, "udp:192.168.64.1:14550", "/tmp/log.csv",
        )
        self.assertIn("--mp-out", cmd)
        self.assertEqual(cmd[cmd.index("--mp-out") + 1], "udp:192.168.64.1:14550")

    def test_mp_out_omitted_when_empty(self):
        cmd = launch.build_bridge_cmd(
            "/dev/ttyACM0", 115200, "/tmp/state.csv", "/tmp/mission.waypoints",
            32.5378147, 74.3661871, 43.85, "", "/tmp/log.csv",
        )
        self.assertNotIn("--mp-out", cmd)

    def test_default_cli_mp_out_matches_required_default(self):
        args = launch.build_arg_parser().parse_args([])
        self.assertEqual(args.mp_out, "udp:192.168.64.1:14550")

    def test_producer_only_flag_defaults_false_and_parses(self):
        args = launch.build_arg_parser().parse_args([])
        self.assertFalse(args.producer_only)
        args = launch.build_arg_parser().parse_args(["--producer-only"])
        self.assertTrue(args.producer_only)

    def test_never_includes_allow_actuator_output(self):
        cmd = launch.build_bridge_cmd(
            "/dev/ttyACM0", 115200, "/tmp/state.csv", "/tmp/mission.waypoints",
            32.5378147, 74.3661871, 43.85, "udp:192.168.64.1:14550", "/tmp/log.csv",
        )
        self.assertNotIn("--allow-actuator-output", cmd)

    def test_jsbsim_cmd_uses_absolute_root(self):
        cmd = launch.build_jsbsim_cmd("/home/missi/ardupilot_clean/Tools/autotest", "/tmp/run/case.xml")
        self.assertEqual(cmd[0], "JSBSim")
        self.assertIn("--root=/home/missi/ardupilot_clean/Tools/autotest", cmd)
        self.assertIn("--script=/tmp/run/case.xml", cmd)


class TestGpsInputSendYawPropagation(unittest.TestCase):
    """HIL-F23-F2B0: --gps-input-send-yaw must reach the bridge command
    unchanged, be absent by default, and never introduce
    --allow-actuator-output.
    """

    def _cmd_kwargs(self):
        return dict(
            pixhawk="/dev/ttyACM0", baud=115200, state_csv="/tmp/state.csv",
            mission_file="/tmp/mission.waypoints", release_lat=32.5378147,
            release_lon=74.3661871, release_groundspeed_mps=43.85,
            mp_out="udp:192.168.64.1:14550", bridge_log_csv="/tmp/log.csv",
        )

    def test_flag_accepted_by_parser(self):
        args = launch.build_arg_parser().parse_args(["--gps-input-send-yaw"])
        self.assertTrue(args.gps_input_send_yaw)

    def test_flag_absent_by_default(self):
        args = launch.build_arg_parser().parse_args([])
        self.assertFalse(args.gps_input_send_yaw)

    def test_flag_propagated_exactly_once_when_enabled(self):
        cmd = launch.build_bridge_cmd(**self._cmd_kwargs(), send_yaw=True)
        self.assertEqual(cmd.count("--gps-input-send-yaw"), 1)

    def test_flag_absent_from_generated_command_by_default(self):
        cmd = launch.build_bridge_cmd(**self._cmd_kwargs())
        self.assertNotIn("--gps-input-send-yaw", cmd)

    def test_flag_absent_when_explicitly_false(self):
        cmd = launch.build_bridge_cmd(**self._cmd_kwargs(), send_yaw=False)
        self.assertNotIn("--gps-input-send-yaw", cmd)

    def test_generated_command_otherwise_unchanged_when_flag_absent(self):
        # Same command with and without send_yaw=False must be byte-for-
        # byte identical -- absence of the flag must not perturb anything
        # else in the generated bridge command.
        cmd_default = launch.build_bridge_cmd(**self._cmd_kwargs())
        cmd_explicit_false = launch.build_bridge_cmd(**self._cmd_kwargs(), send_yaw=False)
        self.assertEqual(cmd_default, cmd_explicit_false)

    def test_never_introduces_allow_actuator_output(self):
        cmd = launch.build_bridge_cmd(**self._cmd_kwargs(), send_yaw=True)
        self.assertNotIn("--allow-actuator-output", cmd)

    def test_dry_run_prints_enabled_status(self):
        import io
        import contextlib

        argv = [
            "--dry-run",
            "--gps-input-send-yaw",
            "--mission-file", str(launch.DEFAULT_MISSION_FILE),
            "--source-ic", str(launch.DEFAULT_SOURCE_IC),
            "--run-dir", tempfile.mkdtemp(),
        ]
        with contextlib.redirect_stdout(io.StringIO()) as buf:
            import sys
            old_argv = sys.argv
            sys.argv = ["sr75_hil_f23e1_live_feed_launch.py"] + argv
            try:
                rc = launch.main()
            finally:
                sys.argv = old_argv
        self.assertEqual(rc, 0)
        self.assertIn("GPS_INPUT yaw transmission: ENABLED", buf.getvalue())

    def test_dry_run_prints_disabled_status_by_default(self):
        import io
        import contextlib
        import sys

        argv = [
            "--dry-run",
            "--mission-file", str(launch.DEFAULT_MISSION_FILE),
            "--source-ic", str(launch.DEFAULT_SOURCE_IC),
            "--run-dir", tempfile.mkdtemp(),
        ]
        with contextlib.redirect_stdout(io.StringIO()) as buf:
            old_argv = sys.argv
            sys.argv = ["sr75_hil_f23e1_live_feed_launch.py"] + argv
            try:
                rc = launch.main()
            finally:
                sys.argv = old_argv
        self.assertEqual(rc, 0)
        self.assertIn("GPS_INPUT yaw transmission: DISABLED", buf.getvalue())

    def test_producer_only_does_not_use_gps_input_send_yaw(self):
        # producer-only launches JSBSim alone -- no bridge, no Pixhawk, no
        # GPS_INPUT of any kind -- so run_producer_only() must not accept
        # or reference a yaw-transport parameter at all.
        import inspect
        sig = inspect.signature(launch.run_producer_only)
        self.assertNotIn("send_yaw", sig.parameters)
        self.assertNotIn("yaw", " ".join(sig.parameters.keys()))

    def test_producer_only_and_gps_input_send_yaw_both_parse_without_conflict(self):
        args = launch.build_arg_parser().parse_args(["--producer-only", "--gps-input-send-yaw"])
        self.assertTrue(args.producer_only)
        self.assertTrue(args.gps_input_send_yaw)


class _FakeProc:
    def __init__(self, returncode):
        self._returncode = returncode

    def poll(self):
        return self._returncode


class TestEarlyExitDetection(unittest.TestCase):
    def test_still_running_returns_none(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            log = Path(tmpdir) / "console.log"
            log.write_text("starting up\n")
            self.assertIsNone(launch.check_process_alive(_FakeProc(None), "JSBSim", log))

    def test_none_proc_returns_none(self):
        self.assertIsNone(launch.check_process_alive(None, "JSBSim", "/nonexistent"))

    def test_exited_returns_message_with_rc_and_tail(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            log = Path(tmpdir) / "console.log"
            log.write_text("\n".join(f"line{i}" for i in range(50)))
            message = launch.check_process_alive(_FakeProc(1), "JSBSim", log)
            self.assertIsNotNone(message)
            self.assertIn("rc=1", message)
            self.assertIn("JSBSim", message)
            self.assertIn("line49", message)
            self.assertNotIn("line10\n", message.split("---")[0])  # tail, not full log

    def test_tail_lines_limited_to_30(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            log = Path(tmpdir) / "console.log"
            log.write_text("\n".join(f"line{i}" for i in range(100)))
            lines = launch.tail_lines(log, 30)
            self.assertEqual(len(lines), 30)
            self.assertEqual(lines[-1], "line99")

    def test_tail_lines_missing_file_returns_empty(self):
        self.assertEqual(launch.tail_lines("/nonexistent/console.log", 30), [])


class TestProducerReadinessGate(unittest.TestCase):
    def test_missing_csv_fails(self):
        ok, message = launch.wait_for_producer_ready(
            "/nonexistent/state.csv", timeout_s=0.3, poll_interval_s=0.05,
        )
        self.assertFalse(ok)
        self.assertIn("never appeared", message)

    def test_header_only_csv_fails_within_timeout(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "state.csv"
            path.write_text("Time,/fdm/jsbsim/simulation/sim-time-sec,/fdm/jsbsim/position/lat-gc-deg\n")
            start = time.time()
            ok, message = launch.wait_for_producer_ready(path, timeout_s=0.3, poll_interval_s=0.05)
            elapsed = time.time() - start
            self.assertFalse(ok)
            self.assertIn("no parseable data row", message)
            self.assertLess(elapsed, 2.0)

    def test_malformed_header_fails(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "state.csv"
            path.write_text("foo,bar,baz\n1,2,3\n")
            ok, message = launch.wait_for_producer_ready(path, timeout_s=0.3, poll_interval_s=0.05)
            self.assertFalse(ok)
            self.assertIn("does not look like a JSBSim state header", message)

    def test_first_valid_row_ready_quickly(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "state.csv"
            path.write_text(
                "Time,/fdm/jsbsim/simulation/sim-time-sec,/fdm/jsbsim/position/lat-gc-deg\n"
                "0.0,0.016666,32.5378147\n"
            )
            start = time.time()
            ok, message = launch.wait_for_producer_ready(path, timeout_s=5.0, poll_interval_s=0.05)
            elapsed = time.time() - start
            self.assertTrue(ok)
            self.assertIn("ready", message)
            self.assertLess(elapsed, 1.0)

    def test_extra_alive_check_aborts_immediately(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "state.csv"  # never created
            start = time.time()
            ok, message = launch.wait_for_producer_ready(
                path, timeout_s=10.0, poll_interval_s=0.05,
                extra_alive_check=lambda: "JSBSim exited unexpectedly rc=1",
            )
            elapsed = time.time() - start
            self.assertFalse(ok)
            self.assertIn("producer died", message)
            self.assertLess(elapsed, 1.0)  # aborts fast, doesn't wait out the full 10s

    def test_header_looks_valid(self):
        self.assertTrue(launch.header_looks_valid("Time,/fdm/jsbsim/simulation/sim-time-sec,foo"))
        self.assertTrue(launch.header_looks_valid("time_s,jsb_lat_deg"))
        self.assertFalse(launch.header_looks_valid("foo,bar,baz"))
        self.assertFalse(launch.header_looks_valid(""))
        self.assertFalse(launch.header_looks_valid("only_one_column"))


class TestArtifactRetention(unittest.TestCase):
    def test_success_without_keep_removes_files(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            paths = [Path(tmpdir) / f"f{i}.txt" for i in range(3)]
            for p in paths:
                p.write_text("x")
            removed = launch.cleanup_artifacts(paths, keep_artifacts=False, run_succeeded=True)
            self.assertEqual(len(removed), 3)
            for p in paths:
                self.assertFalse(p.exists())

    def test_failure_retains_files_even_without_keep_flag(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            paths = [Path(tmpdir) / f"f{i}.txt" for i in range(3)]
            for p in paths:
                p.write_text("x")
            removed = launch.cleanup_artifacts(paths, keep_artifacts=False, run_succeeded=False)
            self.assertEqual(removed, [])
            for p in paths:
                self.assertTrue(p.exists())

    def test_keep_artifacts_flag_retains_files_even_on_success(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            paths = [Path(tmpdir) / f"f{i}.txt" for i in range(3)]
            for p in paths:
                p.write_text("x")
            removed = launch.cleanup_artifacts(paths, keep_artifacts=True, run_succeeded=True)
            self.assertEqual(removed, [])
            for p in paths:
                self.assertTrue(p.exists())

    def test_missing_paths_are_skipped_without_error(self):
        removed = launch.cleanup_artifacts(
            ["/nonexistent/a.txt", "/nonexistent/b.txt"], keep_artifacts=False, run_succeeded=True,
        )
        self.assertEqual(removed, [])


class TestComparisonSummary(unittest.TestCase):
    def _write_csv(self, tmpdir, rows, fieldnames):
        path = Path(tmpdir) / "bridge_log.csv"
        with open(path, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=fieldnames)
            w.writeheader()
            w.writerows(rows)
        return path

    def test_empty_log_returns_zero_rows(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            path = self._write_csv(tmpdir, [], ["time_s"])
            summary = launch.compute_comparison_summary(path)
            self.assertEqual(summary["rows_total"], 0)
            self.assertEqual(summary["rows_with_jsbsim_data"], 0)
            self.assertIsNone(summary["message_rate_hz"])

    def test_uses_jsb_feed_prefixed_columns_not_jsb_prefixed(self):
        # HIL-F23-E2A root cause: the bridge's --jsbsim-state-input feed logs
        # jsb_feed_lat_deg/jsb_feed_lon_deg, NOT jsb_lat_deg/jsb_lon_deg (that
        # older prefix belongs to a separate, unused --jsbsim-csv path).
        fieldnames = ["time_s", "jsb_lat_deg", "jsb_lon_deg", "jsb_feed_lat_deg", "jsb_feed_lon_deg", "lat_deg", "lon_deg"]
        rows = [
            {"time_s": "0.0", "jsb_lat_deg": "99.0", "jsb_lon_deg": "99.0",
             "jsb_feed_lat_deg": "32.5378147", "jsb_feed_lon_deg": "74.3661871",
             "lat_deg": "32.5378100", "lon_deg": "74.3661900"},
        ]
        with tempfile.TemporaryDirectory() as tmpdir:
            path = self._write_csv(tmpdir, rows, fieldnames)
            summary = launch.compute_comparison_summary(path)
            self.assertEqual(summary["rows_with_jsbsim_data"], 1)
            self.assertLess(summary["lat_deg_abs_error"]["max"], 0.001)

    def test_computes_lat_lon_alt_errors(self):
        fieldnames = ["time_s", "jsb_feed_lat_deg", "jsb_feed_lon_deg", "jsb_feed_alt_m", "lat_deg", "lon_deg", "alt_m"]
        rows = [
            {"time_s": "0.0", "jsb_feed_lat_deg": "32.5378147", "jsb_feed_lon_deg": "74.3661871",
             "jsb_feed_alt_m": "3000.35", "lat_deg": "32.5378100", "lon_deg": "74.3661900",
             "alt_m": "3000.50"},
            {"time_s": "1.0", "jsb_feed_lat_deg": "32.5380000", "jsb_feed_lon_deg": "74.3660000",
             "jsb_feed_alt_m": "3010.00", "lat_deg": "32.5379900", "lon_deg": "74.3660100",
             "alt_m": "3009.50"},
        ]
        with tempfile.TemporaryDirectory() as tmpdir:
            path = self._write_csv(tmpdir, rows, fieldnames)
            summary = launch.compute_comparison_summary(path)
            self.assertEqual(summary["rows_total"], 2)
            self.assertEqual(summary["rows_with_jsbsim_data"], 2)
            self.assertEqual(summary["lat_deg_abs_error"]["n"], 2)
            self.assertAlmostEqual(summary["duration_s"], 1.0, places=6)
            self.assertAlmostEqual(summary["message_rate_hz"], 2.0, places=6)
            self.assertLess(summary["lat_deg_abs_error"]["max"], 0.001)
            self.assertGreater(summary["alt_m_abs_error"]["max"], 0.0)

    def test_rows_without_jsbsim_data_are_excluded_from_error_stats(self):
        fieldnames = ["time_s", "jsb_feed_lat_deg", "jsb_feed_lon_deg", "lat_deg", "lon_deg"]
        rows = [
            {"time_s": "0.0", "jsb_feed_lat_deg": "", "jsb_feed_lon_deg": "", "lat_deg": "32.5", "lon_deg": "74.3"},
            {"time_s": "1.0", "jsb_feed_lat_deg": "32.5", "jsb_feed_lon_deg": "74.3", "lat_deg": "32.5", "lon_deg": "74.3"},
        ]
        with tempfile.TemporaryDirectory() as tmpdir:
            path = self._write_csv(tmpdir, rows, fieldnames)
            summary = launch.compute_comparison_summary(path)
            self.assertEqual(summary["rows_total"], 2)
            self.assertEqual(summary["rows_with_jsbsim_data"], 1)

    def test_yaw_error_wraps_across_360(self):
        fieldnames = ["time_s", "jsb_feed_lat_deg", "jsb_feed_lon_deg", "jsb_feed_yaw_rad", "ahrs2_yaw_rad"]
        rows = [
            {"time_s": "0.0", "jsb_feed_lat_deg": "0.0", "jsb_feed_lon_deg": "0.0",
             "jsb_feed_yaw_rad": str(math.radians(350.0)), "ahrs2_yaw_rad": str(math.radians(10.0))},
        ]
        with tempfile.TemporaryDirectory() as tmpdir:
            path = self._write_csv(tmpdir, rows, fieldnames)
            summary = launch.compute_comparison_summary(path)
            self.assertEqual(summary["yaw_deg_abs_error"]["n"], 1)
            self.assertAlmostEqual(summary["yaw_deg_abs_error"]["max"], 20.0, places=3)


if __name__ == "__main__":
    unittest.main()
