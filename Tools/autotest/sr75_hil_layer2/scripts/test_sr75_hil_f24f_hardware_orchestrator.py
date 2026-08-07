#!/usr/bin/env python3
"""HIL-F24-F unit tests for sr75_hil_f24f_hardware_orchestrator.py's pure
gating/planning logic. No hardware, no subprocess is ever spawned by these
tests -- they exercise should_proceed_to_hardware(), check_adapter_
presence(), build_session_dir(), and build_execution_plan() only.
"""
import argparse
import json
import math
import re
import shutil
import subprocess
import sys
import tempfile
import unittest
from datetime import datetime, timezone
from pathlib import Path
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parent))
import sr75_hil_f24f_hardware_orchestrator as orch  # noqa: E402
import sr75_hil_f24d_static_state_feed as state_feed  # noqa: E402


def make_args(**overrides):
    defaults = dict(
        pixhawk="/dev/ttyACM0", baud=115200, ppp_device="/dev/ttyUSB0", ppp_baud=921600,
        host_ip="192.168.144.2", pixhawk_ip="192.168.144.14", listen_port=9002,
        duration_s=30.0, rate_hz=50.0, sessions_dir="/tmp/sr75_hil_f24f_sessions",
        execute=False, confirm_static_only=False,
        # HIL-F24-R1
        profile="static", confirm_dynamic_jsbsim=False,
        # HIL-F24-R2
        stage="none", confirm_estimator_comparison=False,
        r2_alignment_tolerance_s=0.1, r2_settle_s=5.0,
        # HIL-F24-R2C
        confirm_origin_relatch=False, relatch_listen_s=6.0,
        relatch_truth_lat=32.5378085, relatch_truth_lon=74.3661944, relatch_truth_alt_m=240.201118,
        relatch_device_timeout_s=120.0, relatch_sim_json_timeout_s=60.0, relatch_min_replies=5,
        # HIL-F24-R2F
        relatch_operator_wait_budget_s=orch.DEFAULT_RELATCH_OPERATOR_WAIT_BUDGET_S,
        # HIL-F24-R2G
        relatch_ppp_retry_attempts=orch.DEFAULT_RELATCH_PPP_RETRY_ATTEMPTS,
        relatch_ppp_retry_delay_s=orch.DEFAULT_RELATCH_PPP_RETRY_DELAY_S,
        relatch_ppp_health_timeout_s=orch.DEFAULT_RELATCH_PPP_HEALTH_TIMEOUT_S,
        relatch_ppp_health_poll_s=orch.DEFAULT_RELATCH_PPP_HEALTH_POLL_S,
        confirm_mp_visualization=False, visualization_duration_s=orch.DEFAULT_VISUALIZATION_DURATION_S,
        r3a_ekf_readiness_timeout_s=orch.R3A_EKF_READY_TIMEOUT_S,
        r3a_ekf_readiness_consecutive=orch.R3A_EKF_READY_CONSECUTIVE_REPORTS,
        enable_mp_forward=False, mp_udp_host="127.0.0.1", mp_udp_port=14550,
    )
    defaults.update(overrides)
    return argparse.Namespace(**defaults)


class TestShouldProceedToHardware(unittest.TestCase):
    def test_dry_run_default_does_not_proceed(self):
        ok, reason = orch.should_proceed_to_hardware(execute=False, confirm_static_only=False)
        self.assertFalse(ok)
        self.assertIn("dry-run", reason)

    def test_execute_alone_does_not_proceed(self):
        ok, reason = orch.should_proceed_to_hardware(execute=True, confirm_static_only=False)
        self.assertFalse(ok)
        self.assertIn("--confirm-static-only", reason)

    def test_confirm_alone_does_not_proceed(self):
        ok, reason = orch.should_proceed_to_hardware(execute=False, confirm_static_only=True)
        self.assertFalse(ok)
        self.assertIn("--execute", reason)

    def test_both_flags_proceeds(self):
        ok, reason = orch.should_proceed_to_hardware(execute=True, confirm_static_only=True)
        self.assertTrue(ok)
        self.assertEqual(reason, "")


class TestCheckAdapterPresence(unittest.TestCase):
    def test_reports_missing_and_present_paths(self):
        with tempfile.NamedTemporaryFile() as tmp:
            result = orch.check_adapter_presence([tmp.name, "/dev/definitely_not_a_real_adapter_xyz"])
        self.assertTrue(result[tmp.name])
        self.assertFalse(result["/dev/definitely_not_a_real_adapter_xyz"])


class TestBuildSessionDir(unittest.TestCase):
    def test_stamp_is_utc_and_prefixed(self):
        now = datetime(2026, 8, 3, 12, 0, 0, tzinfo=timezone.utc)
        path = orch.build_session_dir(Path("/tmp/sessions"), now=now)
        self.assertEqual(path, Path("/tmp/sessions/sr75_hil_f24f_static_20260803T120000Z"))


class TestBuildExecutionPlan(unittest.TestCase):
    def test_plan_order_matches_item6(self):
        plan = orch.build_execution_plan(make_args())
        names = [step.name for step in plan]
        self.assertEqual(names, [
            "check_adapters", "precheck", "ppp_start", "ppp_verify",
            "start_feeder", "start_responder", "ppp_stop",
        ])

    def test_feeder_step_precedes_responder_step(self):
        plan = orch.build_execution_plan(make_args())
        names = [step.name for step in plan]
        self.assertLess(names.index("start_feeder"), names.index("start_responder"))

    def test_ppp_start_precedes_ppp_verify_precedes_sim_json_steps(self):
        plan = orch.build_execution_plan(make_args())
        names = [step.name for step in plan]
        self.assertLess(names.index("ppp_start"), names.index("ppp_verify"))
        self.assertLess(names.index("ppp_verify"), names.index("start_feeder"))

    def test_precheck_precedes_ppp_start(self):
        plan = orch.build_execution_plan(make_args())
        names = [step.name for step in plan]
        self.assertLess(names.index("precheck"), names.index("ppp_start"))

    def test_responder_command_has_no_actuator_or_jsbsim_flags(self):
        plan = orch.build_execution_plan(make_args())
        responder_step = next(s for s in plan if s.name == "start_responder")
        joined = " ".join(responder_step.command)
        self.assertNotIn("--jsbsim-command-target", joined)
        self.assertNotIn("--actuator-map", joined)

    def test_no_step_contains_arm_or_mission_flags(self):
        plan = orch.build_execution_plan(make_args())
        for step in plan:
            if step.command is None:
                continue
            joined = " ".join(step.command).lower()
            self.assertNotIn("arm", joined)
            self.assertNotIn("mission", joined)


class TestFeederOutlivesResponder(unittest.TestCase):
    """HIL-F24-Q: the F24-P hardware run's shutdown-tail misses (requests
    1489-1499, NO_FRESH_STATE then STALE_STATE) happened because the
    feeder was given exactly --duration-s and so exited on its own before
    the orchestrator's own pre-shutdown wait (--duration-s +
    ORCHESTRATOR_TEST_MARGIN_S) even finished, let alone before the
    explicit stop-responder-then-feeder sequence ran. These tests pin
    down the fixed arithmetic: the feeder's own --duration-s must now
    exceed the orchestrator's full pre-shutdown wait by at least
    FEEDER_SHUTDOWN_MARGIN_S."""

    def _feeder_duration_s(self, args):
        plan = orch.build_execution_plan(args)
        feeder_step = next(s for s in plan if s.name == "start_feeder")
        duration_flag_index = feeder_step.command.index("--duration-s")
        return float(feeder_step.command[duration_flag_index + 1])

    def test_feeder_duration_exceeds_orchestrator_wait_by_shutdown_margin(self):
        args = make_args(duration_s=30.0)
        feeder_duration_s = self._feeder_duration_s(args)
        orchestrator_total_wait_s = args.duration_s + orch.ORCHESTRATOR_TEST_MARGIN_S
        self.assertGreaterEqual(
            feeder_duration_s - orchestrator_total_wait_s,
            orch.FEEDER_SHUTDOWN_MARGIN_S,
        )

    def test_feeder_duration_scales_with_requested_duration(self):
        short_duration_s = self._feeder_duration_s(make_args(duration_s=10.0))
        long_duration_s = self._feeder_duration_s(make_args(duration_s=30.0))
        self.assertAlmostEqual(long_duration_s - short_duration_s, 20.0, places=6)

    def test_shutdown_stops_responder_before_feeder(self):
        """Pins down the fixed order in main()'s finally: block (source
        inspection, not a live process -- no hardware/subprocess here)."""
        import inspect
        source = inspect.getsource(orch.main)
        finally_index = source.index("finally:")
        responder_index = source.index('"responder"', finally_index)
        feeder_index = source.index('"feeder"', finally_index)
        ppp_stop_index = source.index("ppp_stop", finally_index)
        self.assertLess(responder_index, feeder_index)
        self.assertLess(feeder_index, ppp_stop_index)


class TestShouldProceedToHardwareDynamicProfile(unittest.TestCase):
    """HIL-F24-R1: --profile dynamic requires --confirm-dynamic-jsbsim,
    not --confirm-static-only -- the two guarded modes' confirmation
    flags must never substitute for one another."""

    def test_dynamic_profile_requires_confirm_dynamic_jsbsim(self):
        ok, reason = orch.should_proceed_to_hardware(
            execute=True, confirm_static_only=False, confirm_dynamic_jsbsim=True, profile="dynamic",
        )
        self.assertTrue(ok)
        self.assertEqual(reason, "")

    def test_dynamic_profile_with_only_confirm_static_only_stays_dry_run(self):
        ok, reason = orch.should_proceed_to_hardware(
            execute=True, confirm_static_only=True, confirm_dynamic_jsbsim=False, profile="dynamic",
        )
        self.assertFalse(ok)
        self.assertIn("--confirm-dynamic-jsbsim", reason)

    def test_static_profile_with_only_confirm_dynamic_jsbsim_stays_dry_run(self):
        ok, reason = orch.should_proceed_to_hardware(
            execute=True, confirm_static_only=False, confirm_dynamic_jsbsim=True, profile="static",
        )
        self.assertFalse(ok)
        self.assertIn("--confirm-static-only", reason)

    def test_dynamic_profile_confirm_alone_without_execute_stays_dry_run(self):
        ok, reason = orch.should_proceed_to_hardware(
            execute=False, confirm_static_only=False, confirm_dynamic_jsbsim=True, profile="dynamic",
        )
        self.assertFalse(ok)
        self.assertIn("--execute", reason)

    def test_legacy_two_argument_call_still_reproduces_static_behavior(self):
        """Backward compatibility: calling exactly as before HIL-F24-R1
        (2 args, no profile/confirm_dynamic_jsbsim) must reproduce the
        original static-only behavior byte-for-byte."""
        ok, reason = orch.should_proceed_to_hardware(execute=True, confirm_static_only=True)
        self.assertTrue(ok)
        self.assertEqual(reason, "")


class TestBuildExecutionPlanDynamicProfile(unittest.TestCase):
    """HIL-F24-R1: --profile dynamic must select the new live-JSBSim
    feeder without disturbing the static profile's plan at all."""

    def test_static_profile_plan_unchanged(self):
        plan = orch.build_execution_plan(make_args(profile="static"))
        feeder_step = next(s for s in plan if s.name == "start_feeder")
        self.assertIn(str(orch.FEEDER_SCRIPT), feeder_step.command)
        self.assertNotIn(str(orch.DYNAMIC_FEEDER_SCRIPT), feeder_step.command)
        self.assertIn("--rate-hz", feeder_step.command)

    def test_dynamic_profile_selects_dynamic_feeder_script(self):
        plan = orch.build_execution_plan(make_args(profile="dynamic"))
        feeder_step = next(s for s in plan if s.name == "start_feeder")
        self.assertIn(str(orch.DYNAMIC_FEEDER_SCRIPT), feeder_step.command)
        self.assertNotIn(str(orch.FEEDER_SCRIPT), feeder_step.command)

    def test_dynamic_profile_feeder_duration_matches_static_margin_arithmetic(self):
        args = make_args(profile="dynamic", duration_s=30.0)
        plan = orch.build_execution_plan(args)
        feeder_step = next(s for s in plan if s.name == "start_feeder")
        duration_index = feeder_step.command.index("--duration-s")
        feeder_duration_s = float(feeder_step.command[duration_index + 1])
        orchestrator_total_wait_s = args.duration_s + orch.ORCHESTRATOR_TEST_MARGIN_S
        self.assertGreaterEqual(feeder_duration_s - orchestrator_total_wait_s, orch.FEEDER_SHUTDOWN_MARGIN_S)

    def test_responder_command_unaffected_by_profile(self):
        """The responder must stay in LOG_ONLY actuator mode for BOTH
        profiles -- no --jsbsim-command-target/--actuator-map either way,
        since incoming actuator commands are never applied in HIL-F24-R1."""
        for profile in ("static", "dynamic"):
            plan = orch.build_execution_plan(make_args(profile=profile))
            responder_step = next(s for s in plan if s.name == "start_responder")
            joined = " ".join(responder_step.command)
            with self.subTest(profile=profile):
                self.assertNotIn("--jsbsim-command-target", joined)
                self.assertNotIn("--actuator-map", joined)

    def test_no_step_contains_arm_or_mission_flags_dynamic_profile(self):
        plan = orch.build_execution_plan(make_args(profile="dynamic"))
        for step in plan:
            if step.command is None:
                continue
            joined = " ".join(step.command).lower()
            self.assertNotIn("arm", joined)
            self.assertNotIn("mission", joined)


class TestBuildSessionDirProfile(unittest.TestCase):
    def test_static_profile_name_unchanged(self):
        now = datetime(2026, 8, 3, 12, 0, 0, tzinfo=timezone.utc)
        path = orch.build_session_dir(Path("/tmp/sessions"), now=now, profile="static")
        self.assertEqual(path, Path("/tmp/sessions/sr75_hil_f24f_static_20260803T120000Z"))

    def test_dynamic_profile_name_distinct(self):
        now = datetime(2026, 8, 3, 12, 0, 0, tzinfo=timezone.utc)
        path = orch.build_session_dir(Path("/tmp/sessions"), now=now, profile="dynamic")
        self.assertEqual(path, Path("/tmp/sessions/sr75_hil_f24r1_dynamic_20260803T120000Z"))

    def test_default_profile_matches_static(self):
        now = datetime(2026, 8, 3, 12, 0, 0, tzinfo=timezone.utc)
        self.assertEqual(
            orch.build_session_dir(Path("/tmp/sessions"), now=now),
            orch.build_session_dir(Path("/tmp/sessions"), now=now, profile="static"),
        )


class TestShouldProceedToHardwareR2Stage(unittest.TestCase):
    """HIL-F24-R2: --stage r2 requires --confirm-estimator-comparison,
    distinct from either of the other two confirmation flags."""

    def test_r2_stage_requires_confirm_estimator_comparison(self):
        ok, reason = orch.should_proceed_to_hardware(
            execute=True, confirm_static_only=False, confirm_dynamic_jsbsim=False,
            profile="dynamic", confirm_estimator_comparison=True, stage="r2",
        )
        self.assertTrue(ok)
        self.assertEqual(reason, "")

    def test_r2_stage_with_only_confirm_dynamic_jsbsim_stays_dry_run(self):
        ok, reason = orch.should_proceed_to_hardware(
            execute=True, confirm_static_only=False, confirm_dynamic_jsbsim=True,
            profile="dynamic", confirm_estimator_comparison=False, stage="r2",
        )
        self.assertFalse(ok)
        self.assertIn("--confirm-estimator-comparison", reason)

    def test_r2_stage_with_only_confirm_static_only_stays_dry_run(self):
        ok, reason = orch.should_proceed_to_hardware(
            execute=True, confirm_static_only=True, confirm_dynamic_jsbsim=False,
            profile="dynamic", confirm_estimator_comparison=False, stage="r2",
        )
        self.assertFalse(ok)
        self.assertIn("--confirm-estimator-comparison", reason)

    def test_confirm_estimator_comparison_alone_does_not_authorize_non_r2_stage(self):
        """confirm_estimator_comparison must never substitute for the other
        two flags when stage is NOT "r2" (e.g. a caller bug/typo)."""
        ok, reason = orch.should_proceed_to_hardware(
            execute=True, confirm_static_only=False, confirm_dynamic_jsbsim=False,
            profile="dynamic", confirm_estimator_comparison=True, stage="none",
        )
        self.assertFalse(ok)
        self.assertIn("--confirm-dynamic-jsbsim", reason)

    def test_r2_stage_confirm_alone_without_execute_stays_dry_run(self):
        ok, reason = orch.should_proceed_to_hardware(
            execute=False, confirm_static_only=False, confirm_dynamic_jsbsim=False,
            profile="dynamic", confirm_estimator_comparison=True, stage="r2",
        )
        self.assertFalse(ok)
        self.assertIn("--execute", reason)

    def test_legacy_calls_without_stage_kwarg_still_reproduce_static_behavior(self):
        """Backward compatibility: calling exactly as before HIL-F24-R2 (no
        stage/confirm_estimator_comparison kwargs) must be unaffected."""
        ok, reason = orch.should_proceed_to_hardware(
            execute=True, confirm_static_only=False, confirm_dynamic_jsbsim=True, profile="dynamic",
        )
        self.assertTrue(ok)
        self.assertEqual(reason, "")


class TestBuildExecutionPlanR2Stage(unittest.TestCase):
    """HIL-F24-R2: --stage r2 must append the capture/comparison steps
    without disturbing the static or dynamic-only plans at all."""

    def test_default_stage_none_plan_unchanged(self):
        plan = orch.build_execution_plan(make_args(profile="dynamic"))
        names = [step.name for step in plan]
        self.assertEqual(names, [
            "check_adapters", "precheck", "ppp_start", "ppp_verify",
            "start_feeder", "start_responder", "ppp_stop",
        ])

    def test_r2_stage_appends_capture_and_comparison_steps_before_ppp_stop(self):
        plan = orch.build_execution_plan(make_args(profile="dynamic", stage="r2"))
        names = [step.name for step in plan]
        self.assertEqual(names, [
            "check_adapters", "precheck", "ppp_start", "ppp_verify",
            "start_feeder", "start_responder",
            "start_estimator_capture", "run_estimator_comparison", "ppp_stop",
        ])

    def test_capture_step_uses_pixhawk_device_read_only_and_no_actuator_flags(self):
        plan = orch.build_execution_plan(make_args(profile="dynamic", stage="r2"))
        capture_step = next(s for s in plan if s.name == "start_estimator_capture")
        joined = " ".join(capture_step.command).lower()
        self.assertIn(str(orch.R2_CAPTURE_SCRIPT).lower(), joined)
        for forbidden in ("param_set", "--arm", "--mode", "--mission", "rc-override", "actuator"):
            self.assertNotIn(forbidden, joined)

    def test_comparison_step_uses_truth_and_pixhawk_csv_placeholders(self):
        plan = orch.build_execution_plan(make_args(profile="dynamic", stage="r2"))
        comparison_step = next(s for s in plan if s.name == "run_estimator_comparison")
        joined = " ".join(comparison_step.command)
        self.assertIn("{jsbsim_truth_csv}", joined)
        self.assertIn("{pixhawk_estimator_csv}", joined)
        self.assertIn("{estimator_comparison_csv}", joined)
        self.assertIn("{estimator_summary_json}", joined)
        self.assertIn("{estimator_summary_md}", joined)

    def test_capture_duration_matches_feeder_duration(self):
        """Capture must be kept alive at least as long as the feeder (same
        margin arithmetic) so it is never stopped early relative to it."""
        args = make_args(profile="dynamic", stage="r2", duration_s=30.0)
        plan = orch.build_execution_plan(args)
        feeder_step = next(s for s in plan if s.name == "start_feeder")
        capture_step = next(s for s in plan if s.name == "start_estimator_capture")
        feeder_duration_s = float(feeder_step.command[feeder_step.command.index("--duration-s") + 1])
        capture_duration_s = float(capture_step.command[capture_step.command.index("--duration-s") + 1])
        self.assertEqual(feeder_duration_s, capture_duration_s)

    def test_static_profile_plan_unaffected_by_stage_default(self):
        plan = orch.build_execution_plan(make_args(profile="static"))
        names = [step.name for step in plan]
        self.assertNotIn("start_estimator_capture", names)
        self.assertNotIn("run_estimator_comparison", names)

    def test_no_step_contains_arm_or_mission_flags_r2_stage(self):
        plan = orch.build_execution_plan(make_args(profile="dynamic", stage="r2"))
        for step in plan:
            if step.command is None:
                continue
            joined = " ".join(step.command).lower()
            self.assertNotIn("arm", joined)
            self.assertNotIn("mission", joined)


class TestBuildSessionDirR2Stage(unittest.TestCase):
    def test_r2_stage_name_distinct(self):
        now = datetime(2026, 8, 3, 12, 0, 0, tzinfo=timezone.utc)
        path = orch.build_session_dir(Path("/tmp/sessions"), now=now, profile="dynamic", stage="r2")
        self.assertEqual(path, Path("/tmp/sessions/sr75_hil_f24r2_estimator_20260803T120000Z"))

    def test_default_stage_matches_profile_only_naming(self):
        now = datetime(2026, 8, 3, 12, 0, 0, tzinfo=timezone.utc)
        self.assertEqual(
            orch.build_session_dir(Path("/tmp/sessions"), now=now, profile="dynamic"),
            orch.build_session_dir(Path("/tmp/sessions"), now=now, profile="dynamic", stage="none"),
        )


class TestSessionArtifactPaths(unittest.TestCase):
    def test_all_placeholders_present(self):
        paths = orch.session_artifact_paths(Path("/tmp/sessions/example"))
        expected_keys = {
            "state_csv", "responder_csv", "jsbsim_truth_csv", "pixhawk_estimator_csv",
            "estimator_comparison_csv", "estimator_summary_json", "estimator_summary_md",
            "origin_relatch_summary_json", "mp_checklist_md",
        }
        self.assertEqual(set(paths.keys()), expected_keys)
        for key, value in paths.items():
            self.assertTrue(value.startswith("/tmp/sessions/example/"), f"{key}={value}")

    def test_print_plan_renders_r2_plan_without_keyerror(self):
        import io
        import contextlib
        plan = orch.build_execution_plan(make_args(profile="dynamic", stage="r2"))
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            orch.print_plan(plan, make_args(profile="dynamic", stage="r2"))
        output = buf.getvalue()
        self.assertIn("jsbsim_truth.csv", output)
        self.assertIn("estimator_summary.md", output)


class TestSudoAndRunStepSafety(unittest.TestCase):
    def test_expired_sudo_credential_aborts_before_ppp(self):
        def fake_run(cmd, stdout=None, stderr=None, timeout=None):
            self.assertEqual(cmd, ["sudo", "-n", "true"])
            return subprocess.CompletedProcess(cmd, returncode=1)

        ok, reason = orch.check_sudo_noninteractive(run_fn=fake_run)
        self.assertFalse(ok)
        self.assertEqual(reason, orch.SUDO_CREDENTIAL_ABORT)

    def test_sudo_timeout_uses_same_operator_instruction(self):
        def fake_run(cmd, stdout=None, stderr=None, timeout=None):
            raise subprocess.TimeoutExpired(cmd=cmd, timeout=timeout)

        ok, reason = orch.check_sudo_noninteractive(run_fn=fake_run)
        self.assertFalse(ok)
        self.assertIn(orch.SUDO_CREDENTIAL_ABORT, reason)

    def test_orchestrator_managed_ppp_sudo_commands_are_noninteractive(self):
        args = make_args(profile="visual", stage="r3a")
        plan = orch.build_execution_plan(args)
        for name in ("ppp_start", "ppp_stop"):
            step = next(s for s in plan if s.name == name)
            with self.subTest(name=name):
                self.assertEqual(step.command[:2], ["sudo", "-n"])
        for step in orch.ppp_cleanup_steps(args.ppp_device):
            with self.subTest(name=step.name):
                self.assertEqual(step.command[:2], ["sudo", "-n"])

    def test_run_step_timeout_becomes_controlled_abort(self):
        original_run = orch.subprocess.run

        def fake_run(cmd, stdout=None, stderr=None, timeout=None):
            raise subprocess.TimeoutExpired(cmd=cmd, timeout=timeout)

        tmpdir = Path(tempfile.mkdtemp(prefix="hil_f24r3f_timeout_"))
        step = orch.Step("slow", "Slow command", ["slow-command", "--arg"])
        try:
            orch.subprocess.run = fake_run
            with self.assertRaises(orch.OrchestratorAbort) as ctx:
                orch.run_step(step, tmpdir, tmpdir / "slow.log", timeout_s=7.0)
        finally:
            orch.subprocess.run = original_run
        msg = str(ctx.exception)
        self.assertIn("Command timed out after 7.0s", msg)
        self.assertIn("slow-command --arg", msg)
        self.assertIn(str(tmpdir / "slow.log"), msg)


class TestR3aEkfReadinessGate(unittest.TestCase):
    class Msg:
        def __init__(self, flags):
            self.flags = flags

    HEALTHY = orch.EKF_ATTITUDE | orch.EKF_POS_HORIZ_REL

    def _recv_from_flags(self, flags, clock):
        flags = list(flags)

        def recv(timeout_s):
            clock["t"] += 0.5
            if not flags:
                return None
            return self.Msg(flags.pop(0))

        return recv

    def test_consecutive_healthy_reports_are_required(self):
        clock = {"t": 0.0}
        recv = self._recv_from_flags([
            self.HEALTHY,
            orch.EKF_UNINITIALIZED,
            self.HEALTHY,
            self.HEALTHY,
            self.HEALTHY,
        ], clock)
        ok, reason = orch.wait_for_consecutive_ekf_ready(
            recv, timeout_s=10.0, consecutive_required=3, now_fn=lambda: clock["t"],
        )
        self.assertTrue(ok)
        self.assertIn("3 consecutive", reason)

    def test_uninitialized_timeout_blocks_motion(self):
        clock = {"t": 0.0}
        recv = self._recv_from_flags([orch.EKF_UNINITIALIZED] * 20, clock)
        ok, reason = orch.wait_for_consecutive_ekf_ready(
            recv, timeout_s=3.0, consecutive_required=3, now_fn=lambda: clock["t"],
        )
        self.assertFalse(ok)
        self.assertIn("timed out", reason)
        self.assertIn("EKF_UNINITIALIZED", reason)

    def test_position_flag_is_required(self):
        ok, reason = orch.ekf_flags_ready(orch.EKF_ATTITUDE)
        self.assertFalse(ok)
        self.assertIn("position flag missing", reason)

    class PremotionMsg:
        def __init__(self, msg_type, **kwargs):
            self._msg_type = msg_type
            for key, value in kwargs.items():
                setattr(self, key, value)

        def get_type(self):
            return self._msg_type

    def _premotion_summary(self):
        return {
            "gps": {},
            "ekf": {
                "message_count": 0,
                "first_uninitialized_cleared_time": None,
                "first_attitude_flag_time": None,
                "first_horizontal_position_flag_time": None,
                "last_flags": None,
                "last_velocity_variance": None,
                "last_pos_horiz_variance": None,
                "last_pos_vert_variance": None,
                "last_compass_variance": None,
            },
        }

    def test_premotion_summary_records_r2c_like_gps_and_ekf_pass(self):
        summary = self._premotion_summary()
        orch.update_r3a_premotion_summary(
            summary,
            self.PremotionMsg(
                "GPS_RAW_INT", fix_type=3, satellites_visible=15,
                eph=100, epv=100, vel=0, lat=int(32.5378085e7), lon=int(74.3661944e7), alt=240200,
            ),
            1.0,
        )
        orch.update_r3a_premotion_summary(
            summary,
            self.PremotionMsg("EKF_STATUS_REPORT", flags=self.HEALTHY, velocity_variance=0.01),
            2.0,
        )
        self.assertEqual(summary["gps"]["GPS_RAW_INT"]["first_fix_type_ge_3_time"], 1.0)
        self.assertEqual(summary["gps"]["GPS_RAW_INT"]["first_satellite_count"], 15)
        self.assertEqual(summary["gps"]["GPS_RAW_INT"]["message_count"], 1)
        self.assertEqual(summary["ekf"]["message_count"], 1)
        self.assertEqual(summary["ekf"]["first_uninitialized_cleared_time"], 2.0)
        self.assertEqual(summary["ekf"]["first_attitude_flag_time"], 2.0)
        self.assertEqual(summary["ekf"]["first_horizontal_position_flag_time"], 2.0)

    def test_premotion_timeout_reason_reports_bad_fix(self):
        summary = self._premotion_summary()
        orch.update_r3a_premotion_summary(
            summary,
            self.PremotionMsg("GPS_RAW_INT", fix_type=1, satellites_visible=15, eph=65535, epv=65535, vel=0),
            1.0,
        )
        reason = orch.r3a_premotion_timeout_reason(
            summary, "EKF_UNINITIALIZED set flags=1024", timeout_s=15.0, consecutive=0, required=3,
        )
        self.assertIn("EKF_UNINITIALIZED", reason)
        self.assertIn("GPS_RAW_INT never reached fix_type>=3", reason)
        self.assertIn("GPS2_RAW not received", reason)

    def test_premotion_message_row_includes_gps_and_ekf_fields(self):
        gps_row = orch.r3a_premotion_message_to_row(
            self.PremotionMsg("GPS2_RAW", fix_type=3, satellites_visible=15, eph=100, epv=100, vel=25),
            7.25,
        )
        ekf_row = orch.r3a_premotion_message_to_row(
            self.PremotionMsg("EKF_STATUS_REPORT", flags=self.HEALTHY, pos_horiz_variance=0.02),
            8.5,
        )
        self.assertEqual(gps_row["msg_type"], "GPS2_RAW")
        self.assertEqual(gps_row["fix_type"], 3)
        self.assertEqual(gps_row["satellites_visible"], 15)
        self.assertEqual(ekf_row["ekf_flags"], self.HEALTHY)
        self.assertEqual(ekf_row["ekf_pos_horiz_variance"], 0.02)

    def test_premotion_message_row_includes_attitude_ahrs_and_imu_fields(self):
        att_row = orch.r3a_premotion_message_to_row(
            self.PremotionMsg("ATTITUDE", roll=0.1, pitch=-0.2, yaw=0.3, rollspeed=0.01, pitchspeed=-0.02, yawspeed=0.03),
            9.0,
        )
        ahrs2_row = orch.r3a_premotion_message_to_row(
            self.PremotionMsg("AHRS2", roll=0.2, pitch=-0.1, yaw=0.4, altitude=12.5, lat=325378085, lng=743661944),
            10.0,
        )
        imu_row = orch.r3a_premotion_message_to_row(
            self.PremotionMsg("SCALED_IMU2", xacc=1, yacc=2, zacc=-1000, xgyro=3, ygyro=4, zgyro=5),
            11.0,
        )
        self.assertAlmostEqual(att_row["att_roll_deg"], math.degrees(0.1))
        self.assertAlmostEqual(att_row["att_pitch_deg"], math.degrees(-0.2))
        self.assertAlmostEqual(att_row["att_q_deg_s"], math.degrees(-0.02))
        self.assertAlmostEqual(ahrs2_row["ahrs2_roll_deg"], math.degrees(0.2))
        self.assertEqual(ahrs2_row["ahrs2_alt_m"], 12.5)
        self.assertEqual(ahrs2_row["ahrs2_lat_deg"], 32.5378085)
        self.assertEqual(ahrs2_row["ahrs2_lon_deg"], 74.3661944)
        self.assertEqual(imu_row["imu_xacc"], 1)
        self.assertEqual(imu_row["imu_zgyro"], 5)

    def test_premotion_summary_tracks_diagnostic_message_counts(self):
        summary = orch.r3a_empty_premotion_summary(3)
        orch.update_r3a_premotion_summary(
            summary,
            self.PremotionMsg("ATTITUDE", roll=0.1, pitch=-0.2, yaw=0.3),
            1.0,
        )
        orch.update_r3a_premotion_summary(
            summary,
            self.PremotionMsg("AHRS2", roll=0.2, pitch=-0.1, yaw=0.4),
            2.0,
        )
        orch.update_r3a_premotion_summary(
            summary,
            self.PremotionMsg("RAW_IMU", xacc=1, yacc=2, zacc=-1000, xgyro=3, ygyro=4, zgyro=5),
            3.0,
        )
        self.assertEqual(summary["message_counts"]["ATTITUDE"], 1)
        self.assertEqual(summary["message_counts"]["AHRS2"], 1)
        self.assertEqual(summary["message_counts"]["RAW_IMU"], 1)
        self.assertAlmostEqual(summary["last_attitude"]["pitch_deg"], math.degrees(-0.2))
        self.assertAlmostEqual(summary["last_ahrs"]["AHRS2"]["yaw_deg"], math.degrees(0.4))
        self.assertEqual(summary["last_imu"]["RAW_IMU"]["zacc"], -1000)

    class FakeMav:
        def __init__(self):
            self.commands = []

        def command_long_send(self, *args):
            self.commands.append(args)

    class FakeMaster:
        target_system = 1
        target_component = 1

        def __init__(self, messages):
            self.messages = list(messages)
            self.mav = TestR3aEkfReadinessGate.FakeMav()
            self.closed = False

        def wait_heartbeat(self, timeout):
            return object()

        def recv_match(self, type=None, blocking=False, timeout=None):
            if not self.messages:
                return None
            return self.messages.pop(0)

        def close(self):
            self.closed = True

    def test_missing_origin_path_preserves_premotion_diagnostic_artifacts(self):
        tmpdir = Path(tempfile.mkdtemp(prefix="hil_f24r3g_missing_origin_"))
        fake_master = self.FakeMaster([
            self.PremotionMsg("GPS_RAW_INT", fix_type=1, satellites_visible=15, eph=65535, epv=65535, vel=0),
            self.PremotionMsg("EKF_STATUS_REPORT", flags=orch.EKF_UNINITIALIZED, velocity_variance=0.1),
        ])
        capture = orch.R3aPremotionDiagnosticCapture(
            "/dev/fake", 115200, tmpdir / "r3a_premotion_gps_ekf.csv",
            tmpdir / "r3a_premotion_gps_ekf_summary.json",
            mavlink_connection_factory=lambda *args, **kwargs: fake_master,
        )
        capture.start()
        capture.drain(duration_s=0.0)
        ok, reason = orch.evaluate_origin_relatch_summary(
            {"gps_global_origin_mismatch_m": None}, max_mismatch_m=50.0,
        )
        self.assertFalse(ok)
        self.assertIn("not received", reason)
        capture.close()
        self.assertTrue(fake_master.closed)
        csv_text = (tmpdir / "r3a_premotion_gps_ekf.csv").read_text()
        summary = json.loads((tmpdir / "r3a_premotion_gps_ekf_summary.json").read_text())
        self.assertIn("GPS_RAW_INT", csv_text)
        self.assertEqual(summary["gps"]["GPS_RAW_INT"]["message_count"], 1)
        self.assertEqual(summary["ekf"]["message_count"], 1)
        self.assertEqual(summary["gps"]["GPS_RAW_INT"]["last_fix_type"], 1)
        self.assertNotEqual(summary["reason"], "no EKF_STATUS_REPORT received")
        self.assertIn("EKF_STATUS_REPORT received", summary["reason"])
        self.assertIn("EKF_UNINITIALIZED set flags=1024", summary["reason"])


class TestMainR2StageValidation(unittest.TestCase):
    """HIL-F24-R2: --stage r2 without --profile dynamic must abort as a
    dry-run (never reach should_proceed_to_hardware/hardware) -- exercised
    via CLI subprocess since this validation lives at the top of main()."""

    def test_stage_r2_without_dynamic_profile_aborts_dry_run(self):
        import subprocess
        result = subprocess.run(
            [sys.executable, str(orch.__file__), "--stage", "r2"],
            capture_output=True, text=True, timeout=30,
        )
        self.assertEqual(result.returncode, 0)
        self.assertIn("--stage r2 requires --profile dynamic", result.stdout)

    def test_stage_r2_with_dynamic_profile_dry_run_shows_full_plan(self):
        import subprocess
        result = subprocess.run(
            [sys.executable, str(orch.__file__), "--profile", "dynamic", "--stage", "r2"],
            capture_output=True, text=True, timeout=30,
        )
        self.assertEqual(result.returncode, 0)
        self.assertIn("start_estimator_capture", result.stdout)
        self.assertIn("run_estimator_comparison", result.stdout)
        self.assertIn("dry-run", result.stdout)


class TestWaitForDevices(unittest.TestCase):
    """HIL-F24-R2C step 3: pure/testable via dependency injection -- no
    real device, no real sleep."""

    def test_returns_true_immediately_when_all_already_exist(self):
        calls = []
        ok = orch.wait_for_devices(
            ["/dev/a", "/dev/b"], timeout_s=10.0,
            exists_fn=lambda p: True, sleep_fn=lambda s: calls.append(s), now_fn=lambda: 0.0,
        )
        self.assertTrue(ok)
        self.assertEqual(calls, [])

    def test_returns_true_once_devices_appear_after_some_polls(self):
        appear_after = {"/dev/a": 2, "/dev/b": 3}
        poll_count = {"n": 0}

        def exists_fn(p):
            return poll_count["n"] >= appear_after[p]

        def sleep_fn(s):
            poll_count["n"] += 1

        ok = orch.wait_for_devices(
            ["/dev/a", "/dev/b"], timeout_s=100.0, poll_interval_s=1.0,
            exists_fn=exists_fn, sleep_fn=sleep_fn, now_fn=lambda: poll_count["n"],
        )
        self.assertTrue(ok)

    def test_returns_false_on_timeout(self):
        ok = orch.wait_for_devices(
            ["/dev/never"], timeout_s=3.0, poll_interval_s=1.0,
            exists_fn=lambda p: False, sleep_fn=lambda s: None,
            now_fn=iter([0.0, 1.0, 2.0, 3.0, 4.0]).__next__,
        )
        self.assertFalse(ok)

    def test_never_touches_a_real_device_path(self):
        """exists_fn is only ever called with the exact paths given --
        confirms this never falls back to a real filesystem check when
        exists_fn is supplied."""
        seen = []
        orch.wait_for_devices(
            ["/dev/x"], timeout_s=1.0,
            exists_fn=lambda p: seen.append(p) or True, sleep_fn=lambda s: None, now_fn=lambda: 0.0,
        )
        self.assertEqual(seen, ["/dev/x"])


class TestWaitForSimJsonReplies(unittest.TestCase):
    """HIL-F24-R2C step 4: pure/testable via dependency injection."""

    def test_returns_true_once_enough_new_replies_seen(self):
        counts = iter([0, 0, 2, 5, 5])

        def count_fn(path):
            return next(counts)

        ok = orch.wait_for_sim_json_replies(
            "/tmp/fake_responder.csv", min_new_replies=5, timeout_s=100.0, poll_interval_s=1.0,
            count_fn=count_fn, sleep_fn=lambda s: None, now_fn=iter([0.0, 1.0, 2.0, 3.0]).__next__,
        )
        self.assertTrue(ok)

    def test_counts_only_new_replies_since_the_call_began(self):
        """A responder.csv that already had 100 replies before this call
        must not satisfy min_new_replies=5 just from pre-existing rows."""
        counts = iter([100, 100, 100])
        ok = orch.wait_for_sim_json_replies(
            "/tmp/fake_responder.csv", min_new_replies=5, timeout_s=2.0, poll_interval_s=1.0,
            count_fn=lambda path: next(counts), sleep_fn=lambda s: None,
            now_fn=iter([0.0, 1.0, 2.0, 3.0]).__next__,
        )
        self.assertFalse(ok)

    def test_returns_false_on_timeout(self):
        ok = orch.wait_for_sim_json_replies(
            "/tmp/fake_responder.csv", min_new_replies=5, timeout_s=2.0, poll_interval_s=1.0,
            count_fn=lambda path: 0, sleep_fn=lambda s: None, now_fn=iter([0.0, 1.0, 2.0, 3.0]).__next__,
        )
        self.assertFalse(ok)

    def test_count_sim_json_replies_missing_file_returns_zero(self):
        self.assertEqual(orch.count_sim_json_replies("/tmp/definitely_missing_responder.csv"), 0)

    def test_count_sim_json_replies_counts_only_reply_sent_rows(self):
        import csv
        import tempfile
        with tempfile.NamedTemporaryFile(mode="w", suffix=".csv", delete=False, newline="") as f:
            writer = csv.DictWriter(f, fieldnames=["reply_sent", "other"])
            writer.writeheader()
            writer.writerow({"reply_sent": "1", "other": "x"})
            writer.writerow({"reply_sent": "0", "other": "x"})
            writer.writerow({"reply_sent": "1", "other": "x"})
            path = f.name
        try:
            self.assertEqual(orch.count_sim_json_replies(path), 2)
        finally:
            Path(path).unlink()


class TestPromptOperatorPowerCycle(unittest.TestCase):
    def test_blocks_on_input_and_never_sends_anything_to_a_device(self):
        prints = []
        inputs = []

        def fake_input(prompt):
            inputs.append(prompt)
            return ""

        orch.prompt_operator_power_cycle(input_fn=fake_input, print_fn=lambda s: prints.append(s))
        self.assertEqual(len(inputs), 1)
        self.assertIn("power-cycle", " ".join(prints).lower())
        self.assertIn("ENTER", inputs[0])


class TestEvaluateOriginRelatchSummary(unittest.TestCase):
    def test_within_tolerance_passes(self):
        ok, reason = orch.evaluate_origin_relatch_summary(
            {"gps_global_origin_mismatch_m": 0.5}, max_mismatch_m=50.0,
        )
        self.assertTrue(ok)
        self.assertIn("0.50", reason)

    def test_beyond_tolerance_fails(self):
        ok, reason = orch.evaluate_origin_relatch_summary(
            {"gps_global_origin_mismatch_m": 11231266.38}, max_mismatch_m=50.0,
        )
        self.assertFalse(ok)
        self.assertIn("exceeds", reason)

    def test_missing_origin_fails(self):
        ok, reason = orch.evaluate_origin_relatch_summary({"gps_global_origin_mismatch_m": None}, max_mismatch_m=50.0)
        self.assertFalse(ok)
        self.assertIn("not received", reason.lower())

    def test_exactly_at_tolerance_passes(self):
        ok, _ = orch.evaluate_origin_relatch_summary({"gps_global_origin_mismatch_m": 50.0}, max_mismatch_m=50.0)
        self.assertTrue(ok)


class TestShouldProceedToHardwareR2cStage(unittest.TestCase):
    def test_r2c_stage_requires_confirm_origin_relatch(self):
        ok, reason = orch.should_proceed_to_hardware(
            execute=True, confirm_static_only=False, confirm_dynamic_jsbsim=False,
            profile="dynamic", confirm_estimator_comparison=False, stage="r2c", confirm_origin_relatch=True,
        )
        self.assertTrue(ok)
        self.assertEqual(reason, "")

    def test_r2c_stage_with_confirm_estimator_comparison_alone_stays_dry_run(self):
        """--confirm-estimator-comparison (r2's own flag) must not
        authorize r2c -- the two are non-substitutable."""
        ok, reason = orch.should_proceed_to_hardware(
            execute=True, confirm_static_only=False, confirm_dynamic_jsbsim=False,
            profile="dynamic", confirm_estimator_comparison=True, stage="r2c", confirm_origin_relatch=False,
        )
        self.assertFalse(ok)
        self.assertIn("--confirm-origin-relatch", reason)

    def test_confirm_origin_relatch_alone_does_not_authorize_r2_stage(self):
        ok, reason = orch.should_proceed_to_hardware(
            execute=True, confirm_static_only=False, confirm_dynamic_jsbsim=False,
            profile="dynamic", confirm_estimator_comparison=False, stage="r2", confirm_origin_relatch=True,
        )
        self.assertFalse(ok)
        self.assertIn("--confirm-estimator-comparison", reason)

    def test_r2c_stage_confirm_alone_without_execute_stays_dry_run(self):
        ok, reason = orch.should_proceed_to_hardware(
            execute=False, confirm_static_only=False, confirm_dynamic_jsbsim=False,
            profile="dynamic", confirm_estimator_comparison=False, stage="r2c", confirm_origin_relatch=True,
        )
        self.assertFalse(ok)
        self.assertIn("--execute", reason)

    def test_legacy_calls_without_r2c_kwargs_still_reproduce_static_behavior(self):
        ok, reason = orch.should_proceed_to_hardware(execute=True, confirm_static_only=True)
        self.assertTrue(ok)
        self.assertEqual(reason, "")


class TestShouldProceedToHardwareR3aStage(unittest.TestCase):
    def test_r3a_stage_requires_confirm_mp_visualization(self):
        ok, reason = orch.should_proceed_to_hardware(
            execute=True, confirm_static_only=False, confirm_dynamic_jsbsim=False,
            profile="visual", confirm_estimator_comparison=False, stage="r3a",
            confirm_origin_relatch=False, confirm_mp_visualization=True,
        )
        self.assertTrue(ok)
        self.assertEqual(reason, "")

    def test_r3a_stage_with_other_confirm_stays_dry_run(self):
        ok, reason = orch.should_proceed_to_hardware(
            execute=True, confirm_static_only=False, confirm_dynamic_jsbsim=False,
            profile="visual", confirm_estimator_comparison=True, stage="r3a",
            confirm_origin_relatch=True, confirm_mp_visualization=False,
        )
        self.assertFalse(ok)
        self.assertIn("--confirm-mp-visualization", reason)


class TestBuildExecutionPlanR2cStage(unittest.TestCase):
    def test_r2c_stage_step_order(self):
        plan = orch.build_execution_plan(make_args(profile="dynamic", stage="r2c"))
        names = [step.name for step in plan]
        self.assertEqual(names, [
            "check_adapters", "precheck",
            "start_feeder", "start_responder",
            "prompt_power_cycle", "wait_for_reattach", "ppp_start", "ppp_verify",
            "wait_for_sim_json_replies", "run_origin_relatch_check",
            "start_estimator_capture", "run_estimator_comparison", "ppp_stop",
        ])

    def test_feeder_and_responder_precede_ppp_start_for_r2c(self):
        """The whole point of HIL-F24-R2C: feeder/responder must already
        be running before PPP (re)starts, unlike every other stage."""
        plan = orch.build_execution_plan(make_args(profile="dynamic", stage="r2c"))
        names = [step.name for step in plan]
        self.assertLess(names.index("start_feeder"), names.index("ppp_start"))
        self.assertLess(names.index("start_responder"), names.index("ppp_start"))

    def test_ppp_start_precedes_ppp_verify_precedes_origin_check_precedes_capture(self):
        plan = orch.build_execution_plan(make_args(profile="dynamic", stage="r2c"))
        names = [step.name for step in plan]
        self.assertLess(names.index("wait_for_reattach"), names.index("ppp_start"))
        self.assertLess(names.index("ppp_start"), names.index("ppp_verify"))
        self.assertLess(names.index("ppp_verify"), names.index("wait_for_sim_json_replies"))
        self.assertLess(names.index("wait_for_sim_json_replies"), names.index("run_origin_relatch_check"))
        self.assertLess(names.index("run_origin_relatch_check"), names.index("start_estimator_capture"))

    def test_origin_diagnostic_command_uses_relatch_truth_and_tolerance(self):
        args = make_args(profile="dynamic", stage="r2c", relatch_truth_lat=1.0, relatch_truth_lon=2.0)
        plan = orch.build_execution_plan(args)
        step = next(s for s in plan if s.name == "run_origin_relatch_check")
        joined = " ".join(step.command)
        self.assertIn(str(orch.R2B_ORIGIN_DIAGNOSTIC_SCRIPT), joined)
        self.assertIn("--truth-lat 1.0", joined)
        self.assertIn("--truth-lon 2.0", joined)
        self.assertIn("{origin_relatch_summary_json}", joined)

    def test_r2_stage_plan_unaffected_by_r2c_addition(self):
        plan = orch.build_execution_plan(make_args(profile="dynamic", stage="r2"))
        names = [step.name for step in plan]
        self.assertEqual(names, [
            "check_adapters", "precheck", "ppp_start", "ppp_verify",
            "start_feeder", "start_responder",
            "start_estimator_capture", "run_estimator_comparison", "ppp_stop",
        ])

    def test_static_and_dynamic_plans_unaffected_by_r2c_addition(self):
        for profile in ("static", "dynamic"):
            plan = orch.build_execution_plan(make_args(profile=profile))
            names = [step.name for step in plan]
            with self.subTest(profile=profile):
                self.assertEqual(names, [
                    "check_adapters", "precheck", "ppp_start", "ppp_verify",
                    "start_feeder", "start_responder", "ppp_stop",
                ])

    def test_no_step_contains_arm_or_mission_flags_r2c_stage(self):
        plan = orch.build_execution_plan(make_args(profile="dynamic", stage="r2c"))
        for step in plan:
            if step.command is None:
                continue
            joined = " ".join(step.command).lower()
            self.assertNotIn("arm", joined)
            self.assertNotIn("mission", joined)
            self.assertNotIn("param_set", joined)


class TestBuildExecutionPlanR3aStage(unittest.TestCase):
    def test_r3a_stage_uses_visual_feeder_and_capture_comparison(self):
        plan = orch.build_execution_plan(make_args(profile="visual", stage="r3a"))
        names = [step.name for step in plan]
        self.assertEqual(names, [
            "check_adapters", "precheck",
            "start_feeder", "start_responder",
            "prompt_power_cycle", "wait_for_reattach", "ppp_start", "ppp_verify",
            "wait_for_sim_json_replies", "run_origin_relatch_check", "wait_for_ekf_ready",
            "start_motion_feeder",
            "start_estimator_capture", "run_estimator_comparison", "ppp_stop",
        ])
        feeder_step = next(s for s in plan if s.name == "start_feeder")
        joined = " ".join(feeder_step.command)
        self.assertIn(str(orch.FEEDER_SCRIPT), joined)
        self.assertIn("--airspeed-mps 0.0", joined)
        motion_step = next(s for s in plan if s.name == "start_motion_feeder")
        motion_joined = " ".join(motion_step.command)
        self.assertIn(str(orch.DYNAMIC_FEEDER_SCRIPT), motion_joined)
        self.assertIn(orch.VISUAL_FEEDER_JSBSIM_SCRIPT, motion_joined)
        self.assertIn(orch.VISUAL_FEEDER_RAW_OUTPUT, motion_joined)
        self.assertIn("--duration-s 65.0", motion_joined)
        self.assertIn("--jsbsim-end 65.0", motion_joined)
        capture_step = next(s for s in plan if s.name == "start_estimator_capture")
        self.assertNotIn("--enable-mp-forward", " ".join(capture_step.command))

    def test_r3a_motion_and_capture_wait_until_after_ekf_readiness(self):
        plan = orch.build_execution_plan(make_args(profile="visual", stage="r3a"))
        names = [step.name for step in plan]
        self.assertLess(names.index("start_feeder"), names.index("prompt_power_cycle"))
        self.assertLess(names.index("start_responder"), names.index("prompt_power_cycle"))
        self.assertLess(names.index("run_origin_relatch_check"), names.index("wait_for_ekf_ready"))
        self.assertLess(names.index("wait_for_ekf_ready"), names.index("start_motion_feeder"))
        self.assertLess(names.index("start_motion_feeder"), names.index("start_estimator_capture"))
        step = next(s for s in plan if s.name == "run_estimator_comparison")
        joined = " ".join(step.command)
        self.assertIn("--settle-s 0.0", joined)
        self.assertIn("--health-ignore-before-s 0.0", joined)
        self.assertIn("--require-ekf-ready-after-s 0.0", joined)

    def test_r3a_responder_remains_log_only(self):
        plan = orch.build_execution_plan(make_args(profile="visual", stage="r3a"))
        responder_step = next(s for s in plan if s.name == "start_responder")
        joined = " ".join(responder_step.command).lower()
        self.assertNotIn("--jsbsim-command-target", joined)
        self.assertNotIn("--actuator-map", joined)
        self.assertNotIn("arm", joined)
        self.assertNotIn("mission", joined)
        self.assertNotIn("param_set", joined)

    def test_r3a_mp_forwarding_is_opt_in_and_capture_remains_active(self):
        plan = orch.build_execution_plan(make_args(
            profile="visual", stage="r3a",
            enable_mp_forward=True, mp_udp_host="192.168.1.10", mp_udp_port=14550,
        ))
        names = [step.name for step in plan]
        self.assertIn("start_estimator_capture", names)
        capture_step = next(s for s in plan if s.name == "start_estimator_capture")
        joined = " ".join(capture_step.command)
        self.assertIn("--enable-mp-forward", joined)
        self.assertIn("--mp-udp-host 192.168.1.10", joined)
        self.assertIn("--mp-udp-port 14550", joined)
        self.assertIn("--pixhawk /dev/ttyACM0", joined)

    def test_r2c_plan_unaffected_by_r3a_addition(self):
        plan = orch.build_execution_plan(make_args(profile="dynamic", stage="r2c"))
        step = next(s for s in plan if s.name == "start_feeder")
        joined = " ".join(step.command)
        self.assertIn(str(orch.DYNAMIC_FEEDER_SCRIPT), joined)
        self.assertNotIn(orch.VISUAL_FEEDER_JSBSIM_SCRIPT, joined)
        self.assertNotIn(orch.VISUAL_FEEDER_RAW_OUTPUT, joined)


class TestBuildSessionDirR2cStage(unittest.TestCase):
    def test_r2c_stage_name_distinct(self):
        now = datetime(2026, 8, 6, 12, 0, 0, tzinfo=timezone.utc)
        path = orch.build_session_dir(Path("/tmp/sessions"), now=now, profile="dynamic", stage="r2c")
        self.assertEqual(path, Path("/tmp/sessions/sr75_hil_f24r2c_relatch_20260806T120000Z"))

    def test_r3a_stage_name_distinct(self):
        now = datetime(2026, 8, 6, 12, 0, 0, tzinfo=timezone.utc)
        path = orch.build_session_dir(Path("/tmp/sessions"), now=now, profile="visual", stage="r3a")
        self.assertEqual(path, Path("/tmp/sessions/sr75_hil_f24r3a_mp_visual_20260806T120000Z"))


class TestMainR2cStageValidation(unittest.TestCase):
    def test_stage_r2c_without_dynamic_profile_aborts_dry_run(self):
        import subprocess
        result = subprocess.run(
            [sys.executable, str(orch.__file__), "--stage", "r2c"],
            capture_output=True, text=True, timeout=30,
        )
        self.assertEqual(result.returncode, 0)
        self.assertIn("--stage r2c requires --profile dynamic", result.stdout)

    def test_stage_r2c_dry_run_shows_full_plan_and_never_prompts(self):
        """A dry run must never call input() -- confirmed by giving the
        subprocess closed stdin; if it tried to prompt, it would raise
        immediately rather than hang, and the process would still exit
        cleanly with the dry-run plan printed."""
        import subprocess
        result = subprocess.run(
            [sys.executable, str(orch.__file__), "--profile", "dynamic", "--stage", "r2c"],
            capture_output=True, text=True, timeout=30, stdin=subprocess.DEVNULL,
        )
        self.assertEqual(result.returncode, 0)
        self.assertIn("run_origin_relatch_check", result.stdout)
        self.assertIn("prompt_power_cycle", result.stdout)
        self.assertIn("dry-run", result.stdout)


class TestMainR3aStageValidation(unittest.TestCase):
    def test_stage_r3a_without_visual_profile_aborts_dry_run(self):
        result = subprocess.run(
            [sys.executable, str(orch.__file__), "--profile", "dynamic", "--stage", "r3a"],
            capture_output=True, text=True, timeout=30,
        )
        self.assertEqual(result.returncode, 0)
        self.assertIn("--stage r3a requires --profile visual", result.stdout)

    def test_visual_profile_without_r3a_aborts_dry_run(self):
        result = subprocess.run(
            [sys.executable, str(orch.__file__), "--profile", "visual"],
            capture_output=True, text=True, timeout=30,
        )
        self.assertEqual(result.returncode, 0)
        self.assertIn("--profile visual requires --stage r3a", result.stdout)

    def test_stage_r3a_dry_run_shows_visual_plan_and_never_prompts(self):
        result = subprocess.run(
            [sys.executable, str(orch.__file__), "--profile", "visual", "--stage", "r3a"],
            capture_output=True, text=True, timeout=30, stdin=subprocess.DEVNULL,
        )
        self.assertEqual(result.returncode, 0)
        self.assertIn(orch.VISUAL_FEEDER_JSBSIM_SCRIPT, result.stdout)
        self.assertIn("start_estimator_capture", result.stdout)
        self.assertIn("dry-run", result.stdout)


class TestBuildExecutionPlanR2eResponderBind(unittest.TestCase):
    """HIL-F24-R2E: R2C's responder starts before PPP exists, so it must
    bind to R2C_RESPONDER_LISTEN_HOST (0.0.0.0), not args.host_ip (the
    PPP-assigned address, not yet up) -- root cause of the zero-reply
    startup bug (see HIL_F24_R2E_r2c_zero_reply_startup_fix.md)."""

    def test_r2c_responder_binds_to_0_0_0_0_not_host_ip(self):
        args = make_args(profile="dynamic", stage="r2c", host_ip="192.168.144.2")
        plan = orch.build_execution_plan(args)
        step = next(s for s in plan if s.name == "start_responder")
        joined = " ".join(step.command)
        self.assertIn(f"--listen-host {orch.R2C_RESPONDER_LISTEN_HOST}", joined)
        self.assertNotIn("192.168.144.2", joined)

    def test_r2_stage_responder_still_binds_to_host_ip_unaffected(self):
        args = make_args(profile="dynamic", stage="r2", host_ip="192.168.144.2")
        plan = orch.build_execution_plan(args)
        step = next(s for s in plan if s.name == "start_responder")
        joined = " ".join(step.command)
        self.assertIn("--listen-host 192.168.144.2", joined)

    def test_static_and_dynamic_responder_still_bind_to_host_ip_unaffected(self):
        for profile in ("static", "dynamic"):
            args = make_args(profile=profile, host_ip="192.168.144.2")
            plan = orch.build_execution_plan(args)
            step = next(s for s in plan if s.name == "start_responder")
            joined = " ".join(step.command)
            with self.subTest(profile=profile):
                self.assertIn("--listen-host 192.168.144.2", joined)

    def test_r2c_responder_still_uses_given_listen_port_and_state_placeholders(self):
        args = make_args(profile="dynamic", stage="r2c", listen_port=9002)
        plan = orch.build_execution_plan(args)
        step = next(s for s in plan if s.name == "start_responder")
        joined = " ".join(step.command)
        self.assertIn("--listen-port 9002", joined)
        self.assertIn("{state_csv}", joined)
        self.assertIn("{responder_csv}", joined)
        self.assertIn("--strict", joined)
        self.assertNotIn("--jsbsim-command-target", joined)
        self.assertNotIn("--actuator-map", joined)


class TestWaitForProcessStartupHealth(unittest.TestCase):
    """HIL-F24-R2E task 3/4: pure/testable via a fake process object and
    injected sleep_fn -- no real subprocess, no real sleep."""

    class _FakeProc:
        def __init__(self, poll_result):
            self._poll_result = poll_result
            self.returncode = poll_result

        def poll(self):
            return self._poll_result

    def test_alive_process_is_healthy(self):
        sleeps = []
        ok = orch.wait_for_process_startup_health(
            self._FakeProc(poll_result=None), grace_s=1.0, sleep_fn=lambda s: sleeps.append(s),
        )
        self.assertTrue(ok)
        self.assertEqual(sleeps, [1.0])

    def test_dead_process_is_unhealthy(self):
        ok = orch.wait_for_process_startup_health(
            self._FakeProc(poll_result=2), grace_s=1.0, sleep_fn=lambda s: None,
        )
        self.assertFalse(ok)

    def test_default_grace_period_matches_documented_constant(self):
        sleeps = []
        orch.wait_for_process_startup_health(
            self._FakeProc(poll_result=None), sleep_fn=lambda s: sleeps.append(s),
        )
        self.assertEqual(sleeps, [orch.RESPONDER_STARTUP_HEALTH_CHECK_S])


class TestWaitForSimJsonRepliesEarlyExit(unittest.TestCase):
    """HIL-F24-R2E task 4: alive_fn lets wait_for_sim_json_replies() stop
    the instant the responder process has died, instead of waiting out
    the full timeout regardless."""

    def test_dead_process_exits_early_without_waiting_out_full_timeout(self):
        sleep_calls = []
        ok = orch.wait_for_sim_json_replies(
            "/tmp/does_not_matter_responder.csv", min_new_replies=5, timeout_s=1000.0, poll_interval_s=1.0,
            count_fn=lambda path: 0, sleep_fn=lambda s: sleep_calls.append(s),
            now_fn=iter([0.0, 1.0, 2.0]).__next__, alive_fn=lambda: False,
        )
        self.assertFalse(ok)
        # Returned on the very first iteration -- never even reached a sleep.
        self.assertEqual(sleep_calls, [])

    def test_alive_process_is_unaffected_still_waits_for_replies(self):
        counts = iter([0, 0, 5])
        ok = orch.wait_for_sim_json_replies(
            "/tmp/does_not_matter_responder.csv", min_new_replies=5, timeout_s=100.0, poll_interval_s=1.0,
            count_fn=lambda path: next(counts), sleep_fn=lambda s: None,
            now_fn=iter([0.0, 1.0, 2.0, 3.0]).__next__, alive_fn=lambda: True,
        )
        self.assertTrue(ok)

    def test_no_alive_fn_given_preserves_original_behavior(self):
        """Backward compatibility: omitting alive_fn (the pre-R2E
        signature) must behave exactly as before -- times out normally
        against a responder that never replies, with no early exit."""
        ok = orch.wait_for_sim_json_replies(
            "/tmp/does_not_matter_responder.csv", min_new_replies=5, timeout_s=2.0, poll_interval_s=1.0,
            count_fn=lambda path: 0, sleep_fn=lambda s: None, now_fn=iter([0.0, 1.0, 2.0, 3.0]).__next__,
        )
        self.assertFalse(ok)


class TestCleanupStaleRawJsbsimEvidence(unittest.TestCase):
    """HIL-F24-R2E task 5."""

    def test_removes_existing_stale_file(self):
        tmpdir = Path(tempfile.mkdtemp(prefix="hil_f24r2e_stale_"))
        stale_path = tmpdir / "sr75_hil_f24r1_jsbsim_raw.csv"
        stale_path.write_text("stale,data\n1,2\n")
        removed = orch.cleanup_stale_raw_jsbsim_evidence(stale_path)
        self.assertTrue(removed)
        self.assertFalse(stale_path.exists())

    def test_missing_file_is_a_no_op_not_an_error(self):
        tmpdir = Path(tempfile.mkdtemp(prefix="hil_f24r2e_stale_"))
        never_existed = tmpdir / "never_existed.csv"
        removed = orch.cleanup_stale_raw_jsbsim_evidence(never_existed)
        self.assertFalse(removed)

    def test_never_touches_a_different_path(self):
        tmpdir = Path(tempfile.mkdtemp(prefix="hil_f24r2e_stale_"))
        keep = tmpdir / "keep_me.csv"
        keep.write_text("do not delete\n")
        stale_path = tmpdir / "sr75_hil_f24r1_jsbsim_raw.csv"
        stale_path.write_text("stale\n")
        orch.cleanup_stale_raw_jsbsim_evidence(stale_path)
        self.assertTrue(keep.exists())


class TestResponderBindBeforePPP(unittest.TestCase):
    """HIL-F24-R2E task 1/2: real end-to-end regression against the
    actual responder script (no mocking) -- confirms the root cause
    (binding to a PPP-only address that doesn't exist yet fails
    immediately) and the fix (0.0.0.0 works immediately and stays
    alive), using real subprocesses exactly as the orchestrator itself
    spawns them."""

    def _free_udp_port(self):
        import socket as socket_module
        try:
            s = socket_module.socket(socket_module.AF_INET, socket_module.SOCK_DGRAM)
        except PermissionError as exc:
            raise unittest.SkipTest(f"UDP sockets unavailable in this sandbox: {exc}")
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
        s.close()
        return port

    def test_binding_to_unassigned_specific_ip_reproduces_the_original_bug(self):
        """The exact root cause: binding to an address not assigned to
        any local interface (standing in for 192.168.144.2 before PPP
        exists) fails immediately with a nonzero exit code -- this is
        the failure sr75_hil_f24r2c_relatch_20260806T063658Z/
        responder.log actually captured (`[Errno 99] Cannot assign
        requested address`)."""
        import subprocess
        tmpdir = Path(tempfile.mkdtemp(prefix="hil_f24r2e_bind_bug_"))
        port = self._free_udp_port()
        cmd = [
            sys.executable, str(orch.RESPONDER_SCRIPT),
            # A valid-looking but locally-unassigned address, standing in
            # for 192.168.144.2 before PPP has ever brought ppp0 up --
            # exercises the identical bind() failure path without
            # depending on any real PPP state in this test environment.
            "--listen-host", "10.255.255.1", "--listen-port", str(port),
            "--state-file", str(tmpdir / "state.csv"), "--log-csv", str(tmpdir / "responder.csv"), "--strict",
        ]
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        try:
            out, _ = proc.communicate(timeout=10.0)
        except subprocess.TimeoutExpired:
            proc.kill()
            self.fail("responder did not exit promptly when bind() should have failed")
        self.assertNotEqual(proc.returncode, 0)
        self.assertIn("bind failed", out)

    def test_binding_to_0_0_0_0_succeeds_and_stays_alive_before_ppp_exists(self):
        """The fix: 0.0.0.0 requires no specific interface/address to
        already exist, so it works immediately -- exactly the situation
        HIL-F24-R2C's responder is in (started before PPP)."""
        import subprocess
        tmpdir = Path(tempfile.mkdtemp(prefix="hil_f24r2e_bind_fix_"))
        port = self._free_udp_port()
        cmd = [
            sys.executable, str(orch.RESPONDER_SCRIPT),
            "--listen-host", orch.R2C_RESPONDER_LISTEN_HOST, "--listen-port", str(port),
            "--state-file", str(tmpdir / "state.csv"), "--log-csv", str(tmpdir / "responder.csv"), "--strict",
        ]
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        try:
            self.assertTrue(
                orch.wait_for_process_startup_health(proc, grace_s=1.0),
                "responder exited immediately when binding to 0.0.0.0 -- should never happen",
            )
        finally:
            proc.terminate()
            try:
                proc.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                proc.kill()


class TestR3aVisualTrajectoryXml(unittest.TestCase):
    XML_PATH = Path(__file__).resolve().parents[2] / orch.VISUAL_FEEDER_JSBSIM_SCRIPT
    GRAVITY_MSS = 9.80665
    LAT0_DEG = 32.5378085
    LON0_DEG = 74.3661944
    ALT0_M = 240.201118
    MOTION_S = 60.0
    LAT_DELTA_DEG = 0.00161694
    LON_DELTA_DEG = 0.00126792
    ALT_DELTA_M = 30.0
    ROLL_DELTA_RAD = 0.069813170
    PITCH_DELTA_RAD = 0.052359878
    YAW0_RAD = 5.497787144
    YAW_DELTA_RAD = 0.261799388
    NORTH_DELTA_M = 179.9977608
    EAST_DELTA_M = 118.9902933
    SPEED_DELTA_MPS = 217.8483046

    def _root(self):
        return ET.parse(self.XML_PATH).getroot()

    def _trajectory_sample(self, t_s):
        u = min(1.0, max(0.0, t_s / self.MOTION_S))
        smooth = u * u * (3.0 - 2.0 * u)
        smooth_dot = 0.1 * u * (1.0 - u)
        roll = self.ROLL_DELTA_RAD * smooth
        pitch = self.PITCH_DELTA_RAD * smooth
        yaw = self.YAW0_RAD + self.YAW_DELTA_RAD * smooth
        roll_dot = self.ROLL_DELTA_RAD * smooth_dot
        pitch_dot = self.PITCH_DELTA_RAD * smooth_dot
        yaw_dot = self.YAW_DELTA_RAD * smooth_dot
        sr = math.sin(roll)
        cr = math.cos(roll)
        sp = math.sin(pitch)
        cp = math.cos(pitch)
        p = roll_dot - yaw_dot * sp
        q = pitch_dot * cr + yaw_dot * sr * cp
        r = -pitch_dot * sr + yaw_dot * cr * cp
        ax = self.GRAVITY_MSS * sp
        ay = -self.GRAVITY_MSS * sr * cp
        az = -self.GRAVITY_MSS * cr * cp
        return {
            "lat_deg": self.LAT0_DEG + self.LAT_DELTA_DEG * smooth,
            "lon_deg": self.LON0_DEG + self.LON_DELTA_DEG * smooth,
            "alt_m": self.ALT0_M + self.ALT_DELTA_M * smooth,
            "roll": roll,
            "pitch": pitch,
            "yaw": yaw,
            "roll_dot": roll_dot,
            "pitch_dot": pitch_dot,
            "yaw_dot": yaw_dot,
            "p": p,
            "q": q,
            "r": r,
            "ax": ax,
            "ay": ay,
            "az": az,
            "vn_mps": self.NORTH_DELTA_M * smooth_dot,
            "ve_mps": self.EAST_DELTA_M * smooth_dot,
            "vd_mps": -self.ALT_DELTA_M * smooth_dot,
            "airspeed_mps": self.SPEED_DELTA_MPS * smooth_dot,
        }

    def _samples_0_to_65(self):
        return [self._trajectory_sample(i * 0.5) for i in range(131)]

    def test_output_fields_match_state_csv_contract(self):
        names = [fn.attrib["name"] for fn in self._root().findall(".//function")]
        self.assertEqual(set(names), set(state_feed.CSV_FIELDS))
        self.assertEqual(len(names), len(state_feed.CSV_FIELDS))

    def test_zero_control_and_propulsion_commands_only(self):
        root = self._root()
        sets = root.findall(".//set")
        self.assertGreater(len(sets), 0)
        for item in sets:
            name = item.attrib["name"]
            value = float(item.attrib["value"])
            with self.subTest(name=name):
                self.assertIn(name, {
                    "propulsion/engine[0]/set-running",
                    "propulsion/engine[1]/set-running",
                    "propulsion/engine[2]/set-running",
                    "fcs/turbojet-throttle-cmd-norm",
                    "fcs/rato-throttle-cmd-norm",
                    "fcs/throttle-cmd-norm",
                    "fcs/aileron-cmd-norm",
                    "fcs/elevator-cmd-norm",
                    "fcs/rudder-cmd-norm",
                })
                self.assertEqual(value, 0.0)

    def test_trajectory_samples_are_finite_over_visualization_window(self):
        for sample in self._samples_0_to_65():
            for name, value in sample.items():
                with self.subTest(name=name, value=value):
                    self.assertTrue(math.isfinite(value))

    def test_transition_starts_and_ends_without_position_attitude_or_rate_jump(self):
        start = self._trajectory_sample(0.0)
        just_after = self._trajectory_sample(0.02)
        end = self._trajectory_sample(self.MOTION_S)
        after_end = self._trajectory_sample(self.MOTION_S + 1.0)
        self.assertAlmostEqual(just_after["lat_deg"], start["lat_deg"], delta=1e-9)
        self.assertAlmostEqual(just_after["roll"], start["roll"], delta=1e-7)
        self.assertAlmostEqual(just_after["pitch"], start["pitch"], delta=1e-7)
        self.assertGreaterEqual(just_after["vn_mps"], 0.0)
        self.assertGreaterEqual(just_after["airspeed_mps"], 0.0)
        for field in ("roll_dot", "pitch_dot", "yaw_dot", "p", "q", "r", "vn_mps", "ve_mps", "vd_mps", "airspeed_mps"):
            self.assertAlmostEqual(start[field], 0.0, places=9)
            self.assertAlmostEqual(end[field], 0.0, places=9)
            self.assertAlmostEqual(after_end[field], 0.0, places=9)
        self.assertEqual(end["lat_deg"], after_end["lat_deg"])
        self.assertEqual(end["lon_deg"], after_end["lon_deg"])
        self.assertEqual(end["alt_m"], after_end["alt_m"])

    def test_gravity_vector_tracks_scripted_roll_and_pitch(self):
        for sample in self._samples_0_to_65():
            magnitude = math.sqrt(sample["ax"] ** 2 + sample["ay"] ** 2 + sample["az"] ** 2)
            self.assertAlmostEqual(magnitude, self.GRAVITY_MSS, places=6)
            self.assertAlmostEqual(sample["ax"], self.GRAVITY_MSS * math.sin(sample["pitch"]), places=9)
            self.assertAlmostEqual(
                sample["ay"],
                -self.GRAVITY_MSS * math.sin(sample["roll"]) * math.cos(sample["pitch"]),
                places=9,
            )
            self.assertAlmostEqual(
                sample["az"],
                -self.GRAVITY_MSS * math.cos(sample["roll"]) * math.cos(sample["pitch"]),
                places=9,
            )

    def test_body_rates_match_scripted_euler_rates(self):
        for sample in self._samples_0_to_65():
            roll = sample["roll"]
            pitch = sample["pitch"]
            p = sample["p"]
            q = sample["q"]
            r = sample["r"]
            roll_dot = p + q * math.sin(roll) * math.tan(pitch) + r * math.cos(roll) * math.tan(pitch)
            pitch_dot = q * math.cos(roll) - r * math.sin(roll)
            yaw_dot = (q * math.sin(roll) + r * math.cos(roll)) / math.cos(pitch)
            self.assertAlmostEqual(roll_dot, sample["roll_dot"], places=9)
            self.assertAlmostEqual(pitch_dot, sample["pitch_dot"], places=9)
            self.assertAlmostEqual(yaw_dot, sample["yaw_dot"], places=9)

    def test_trajectory_bounds_and_continuity_constants(self):
        duration_s = orch.DEFAULT_VISUALIZATION_DURATION_S
        first = self._trajectory_sample(0.0)
        last = self._trajectory_sample(duration_s)
        north_m = (last["lat_deg"] - first["lat_deg"]) * 111320.0
        east_m = (
            (last["lon_deg"] - first["lon_deg"])
            * 111320.0
            * math.cos(math.radians(first["lat_deg"]))
        )
        horizontal_m = math.hypot(north_m, east_m)
        alt_change_m = last["alt_m"] - first["alt_m"]
        yaw_change_deg = math.degrees(last["yaw"] - first["yaw"])
        self.assertGreaterEqual(horizontal_m, 100.0)
        self.assertLessEqual(horizontal_m, 300.0)
        self.assertGreaterEqual(alt_change_m, 20.0)
        self.assertLessEqual(alt_change_m, 40.0)
        self.assertLessEqual(max(abs(math.degrees(s["roll"])) for s in self._samples_0_to_65()), 5.0)
        self.assertLessEqual(max(abs(math.degrees(s["pitch"])) for s in self._samples_0_to_65()), 5.0)
        self.assertLessEqual(abs(yaw_change_deg), 20.0)
        self.assertTrue(math.isfinite(math.hypot(first["vn_mps"], first["ve_mps"])))
        self.assertGreater(last["lat_deg"], first["lat_deg"])
        self.assertGreater(last["lon_deg"], first["lon_deg"])

    def test_default_visualization_window_is_sixty_seconds(self):
        run = self._root().find(".//run")
        self.assertGreater(float(run.attrib["end"]), orch.DEFAULT_VISUALIZATION_DURATION_S)
        self.assertEqual(orch.DEFAULT_VISUALIZATION_DURATION_S, 60.0)

    def test_visual_time_source_uses_airborne_initializer(self):
        use = self._root().find(".//use")
        self.assertEqual(use.attrib["initialize"], "hil_f24r3a_visual_time_source_init")

    @unittest.skipUnless(shutil.which("JSBSim") is not None, "JSBSim binary not found on PATH")
    def test_real_time_visual_feeder_runs_full_sixty_five_seconds(self):
        tmpdir = Path(tempfile.mkdtemp(prefix="hil_f24r3g_visual_65s_"))
        state_csv = tmpdir / "state.csv"
        proc = subprocess.run(
            [
                sys.executable,
                str(Path(__file__).resolve().parents[1] / "sim_json" / "sr75_hil_f24r1_dynamic_jsbsim_feed.py"),
                "--output", str(state_csv),
                "--duration-s", "65",
                "--jsbsim-script", orch.VISUAL_FEEDER_JSBSIM_SCRIPT,
                "--raw-output", orch.VISUAL_FEEDER_RAW_OUTPUT,
                "--jsbsim-end", "65",
            ],
            capture_output=True, text=True, timeout=90,
        )
        self.assertEqual(proc.returncode, 0, proc.stdout + proc.stderr)
        self.assertIn("JSBSIM_SUMMARY", proc.stdout)
        match = re.search(r"final_time_s=([0-9.]+)", proc.stdout)
        self.assertIsNotNone(match, proc.stdout)
        self.assertGreaterEqual(float(match.group(1)), 64.5)


class FakeStateReader:
    def __init__(self, age_s):
        self.age_s = age_s

    def read_latest(self):
        return {"time_s": "1.0"}, "line", self.age_s, None


class FakeProcess:
    def __init__(self, returncode=None):
        self.returncode = returncode

    def poll(self):
        return self.returncode


class TestR3aTruthFeedMonitor(unittest.TestCase):
    def _clock(self):
        now = {"t": 0.0}
        return lambda: now["t"], lambda s: now.__setitem__("t", now["t"] + s)

    def test_early_feeder_exit_causes_immediate_abort(self):
        tmpdir = Path(tempfile.mkdtemp(prefix="hil_f24r3d_exit_"))
        log = tmpdir / "feeder.log"
        log.write_text("JSBSIM_STALL_WARNING\nJSBSIM_SUMMARY final_time_s=7.26\n")
        ok, reason = orch.monitor_r3a_truth_feed(
            FakeProcess(returncode=9), tmpdir / "state.csv", log, duration_s=60.0,
            reader_factory=lambda _path: FakeStateReader(0.0),
        )
        self.assertFalse(ok)
        self.assertIn("feeder exited early rc=9", reason)
        self.assertIn("final_time_s=7.26", reason)

    def test_stale_state_causes_immediate_abort(self):
        tmpdir = Path(tempfile.mkdtemp(prefix="hil_f24r3d_stale_"))
        log = tmpdir / "feeder.log"
        log.write_text("JSBSIM_STALL_WARNING\n")
        ok, reason = orch.monitor_r3a_truth_feed(
            FakeProcess(returncode=None), tmpdir / "state.csv", log, duration_s=60.0,
            max_state_age_s=0.5,
            reader_factory=lambda _path: FakeStateReader(1.0),
        )
        self.assertFalse(ok)
        self.assertIn("state.csv stale", reason)

    def test_fresh_state_full_window_succeeds(self):
        tmpdir = Path(tempfile.mkdtemp(prefix="hil_f24r3d_fresh_"))
        log = tmpdir / "feeder.log"
        log.write_text("running\n")
        now_fn, sleep_fn = self._clock()
        ok, reason = orch.monitor_r3a_truth_feed(
            FakeProcess(returncode=None), tmpdir / "state.csv", log, duration_s=1.0,
            max_state_age_s=0.5, poll_interval_s=0.25,
            reader_factory=lambda _path: FakeStateReader(0.01),
            now_fn=now_fn, sleep_fn=sleep_fn,
        )
        self.assertTrue(ok)
        self.assertIn("remained live", reason)

    def test_no_comparison_after_invalid_truth_feed(self):
        self.assertFalse(orch.should_run_estimator_comparison("r3a", object(), truth_feed_valid=False))
        self.assertTrue(orch.should_run_estimator_comparison("r3a", object(), truth_feed_valid=True))
        self.assertTrue(orch.should_run_estimator_comparison("r2c", object(), truth_feed_valid=True))
        self.assertFalse(orch.should_run_estimator_comparison("none", object(), truth_feed_valid=True))


class TestR2cPppReconnectRetries(unittest.TestCase):
    """HIL-F24-R2G: R2C-only bounded PPP reconnect retry logic."""

    HEALTHY_STATUS = """ppp0 link:
7: ppp0: <POINTOPOINT,MULTICAST,NOARP,UP,LOWER_UP> mtu 1500 qdisc fq_codel state UNKNOWN mode DEFAULT group default qlen 3

ppp0 address:
7: ppp0: <POINTOPOINT,MULTICAST,NOARP,UP,LOWER_UP> mtu 1500 qdisc fq_codel state UNKNOWN group default qlen 3
    inet 192.168.144.2 peer 192.168.144.14/32 scope global ppp0

ping Pixhawk PPP endpoint (192.168.144.14):
3 packets transmitted, 3 received, 0% packet loss, time 2002ms
"""

    DOWN_STATUS = """ppp0 link:
Device "ppp0" does not exist.

ppp0 address:
Device "ppp0" does not exist.

ping Pixhawk PPP endpoint (192.168.144.14):
3 packets transmitted, 0 received, 100% packet loss, time 2047ms
"""

    PARTIAL_PING_STATUS = """ppp0 link:
7: ppp0: <POINTOPOINT,MULTICAST,NOARP,UP,LOWER_UP> mtu 1500 qdisc fq_codel state UNKNOWN mode DEFAULT group default qlen 3

ppp0 address:
7: ppp0: <POINTOPOINT,MULTICAST,NOARP,UP,LOWER_UP> mtu 1500 qdisc fq_codel state UNKNOWN group default qlen 3
    inet 192.168.144.2 peer 192.168.144.14/32 scope global ppp0

ping Pixhawk PPP endpoint (192.168.144.14):
3 packets transmitted, 1 received, 66% packet loss, time 2002ms
"""

    def _plan(self):
        plan = orch.build_execution_plan(make_args(profile="dynamic", stage="r2c"))
        return {step.name: step for step in plan}

    def _runner(self, start_logs, verify_logs, calls):
        start_logs = list(start_logs)
        verify_logs = list(verify_logs)

        def fake_run_step(step, session_dir, log_path, timeout_s=None):
            calls.append((step.name, tuple(step.command or ()), Path(log_path).name, timeout_s))
            if "start" in Path(log_path).name:
                Path(log_path).write_text(start_logs.pop(0) if start_logs else "")
            elif "verify" in Path(log_path).name:
                Path(log_path).write_text(verify_logs.pop(0) if verify_logs else "")
            else:
                Path(log_path).write_text("cleanup\n")
            return 0

        return fake_run_step

    def _clock(self):
        now = {"t": 0.0}
        return now, lambda: now["t"], lambda s: now.__setitem__("t", now["t"] + s)

    def test_status_health_requires_up_addresses_and_ping(self):
        ok, reason = orch.ppp_status_health(self.HEALTHY_STATUS, "192.168.144.2", "192.168.144.14")
        self.assertTrue(ok)
        self.assertIn("ping 3/3", reason)

        ok, reason = orch.ppp_status_health(self.DOWN_STATUS, "192.168.144.2", "192.168.144.14")
        self.assertFalse(ok)
        self.assertIn("UP/LOWER_UP", reason)

    def test_status_health_ignores_ppp0_link_heading(self):
        heading_only = """ppp0 link:

ppp0 address:
Device "ppp0" does not exist.

ping Pixhawk PPP endpoint (192.168.144.14):
3 packets transmitted, 3 received, 0% packet loss, time 2002ms
"""
        ok, reason = orch.ppp_status_health(heading_only, "192.168.144.2", "192.168.144.14")
        self.assertFalse(ok)
        self.assertIn("UP/LOWER_UP", reason)

    def test_first_attempt_modem_hangup_second_succeeds(self):
        args = make_args(
            profile="dynamic", stage="r2c",
            relatch_ppp_retry_attempts=2, relatch_ppp_retry_delay_s=0.25,
            relatch_ppp_health_timeout_s=0.5, relatch_ppp_health_poll_s=0.5,
        )
        tmpdir = Path(tempfile.mkdtemp(prefix="hil_f24r2g_ppp_retry_"))
        calls = []
        _now, now_fn, sleep_fn = self._clock()
        ok, reason = orch.restart_r2c_ppp_with_retries(
            args, self._plan(), tmpdir,
            run_step_fn=self._runner(["Modem hangup\n", ""], [self.HEALTHY_STATUS], calls),
            sleep_fn=sleep_fn, now_fn=now_fn, print_fn=lambda s: None,
        )
        self.assertTrue(ok)
        self.assertIn("attempt 2/2 succeeded", reason)
        start_calls = [c for c in calls if c[2].endswith("_start.log")]
        self.assertEqual(len(start_calls), 2)

    def test_stale_pid_process_and_ppp0_cleanup_precedes_each_start(self):
        args = make_args(profile="dynamic", stage="r2c", relatch_ppp_retry_attempts=1)
        tmpdir = Path(tempfile.mkdtemp(prefix="hil_f24r2g_ppp_cleanup_"))
        calls = []
        ok, _reason = orch.restart_r2c_ppp_with_retries(
            args, self._plan(), tmpdir,
            run_step_fn=self._runner([""], [self.HEALTHY_STATUS], calls),
            sleep_fn=lambda s: None, print_fn=lambda s: None,
        )
        self.assertTrue(ok)
        commands = [" ".join(c[1]) for c in calls[:3]]
        self.assertIn(str(orch.PPP_STOP_SCRIPT), commands[0])
        self.assertIn("--device /dev/ttyUSB0", commands[0])
        self.assertIn("rm -f /tmp/sr75_ppp__dev_ttyUSB0.pid", commands[1])
        self.assertIn("ip link delete ppp0", commands[2])

    def test_exhausted_retries_abort_with_exact_logs(self):
        args = make_args(
            profile="dynamic", stage="r2c", relatch_ppp_retry_attempts=2,
            relatch_ppp_retry_delay_s=0.0, relatch_ppp_health_timeout_s=1.0, relatch_ppp_health_poll_s=1.0,
        )
        tmpdir = Path(tempfile.mkdtemp(prefix="hil_f24r2g_ppp_exhaust_"))
        calls = []
        _now, now_fn, sleep_fn = self._clock()
        ok, reason = orch.restart_r2c_ppp_with_retries(
            args, self._plan(), tmpdir,
            run_step_fn=self._runner(["Modem hangup\n", "Modem hangup\n"], [self.DOWN_STATUS, self.DOWN_STATUS], calls),
            sleep_fn=sleep_fn, now_fn=now_fn, print_fn=lambda s: None,
        )
        self.assertFalse(ok)
        self.assertIn("PPP reconnect failed after 2 attempts", reason)
        self.assertIn("ppp_r2c_attempt_2_start.log", reason)
        self.assertIn("last verify log: None", reason)
        self.assertIn("Modem hangup", reason)

    def test_normal_first_attempt_success(self):
        args = make_args(profile="dynamic", stage="r2c", relatch_ppp_retry_attempts=3)
        tmpdir = Path(tempfile.mkdtemp(prefix="hil_f24r2g_ppp_first_"))
        calls = []
        ok, reason = orch.restart_r2c_ppp_with_retries(
            args, self._plan(), tmpdir,
            run_step_fn=self._runner([""], [self.HEALTHY_STATUS], calls),
            sleep_fn=lambda s: None, print_fn=lambda s: None,
        )
        self.assertTrue(ok)
        self.assertIn("attempt 1/3 succeeded", reason)
        self.assertEqual(len([c for c in calls if c[2].endswith("_start.log")]), 1)

    def test_down_partial_ping_then_healthy_within_one_attempt(self):
        args = make_args(
            profile="dynamic", stage="r2c", relatch_ppp_retry_attempts=2,
            relatch_ppp_health_timeout_s=10.0, relatch_ppp_health_poll_s=1.0,
        )
        tmpdir = Path(tempfile.mkdtemp(prefix="hil_f24r2h_ppp_progress_"))
        calls = []
        _now, now_fn, sleep_fn = self._clock()
        ok, reason = orch.restart_r2c_ppp_with_retries(
            args, self._plan(), tmpdir,
            run_step_fn=self._runner([""], [self.DOWN_STATUS, self.PARTIAL_PING_STATUS, self.HEALTHY_STATUS], calls),
            sleep_fn=sleep_fn, now_fn=now_fn, print_fn=lambda s: None,
        )
        self.assertTrue(ok)
        self.assertIn("attempt 1/2 succeeded", reason)
        self.assertEqual(len([c for c in calls if c[0].startswith("ppp_cleanup")]), 3)
        self.assertEqual(len([c for c in calls if "_verify_" in c[2]]), 3)

    def test_true_timeout_triggers_next_retry_cleanup(self):
        args = make_args(
            profile="dynamic", stage="r2c", relatch_ppp_retry_attempts=2,
            relatch_ppp_retry_delay_s=0.0, relatch_ppp_health_timeout_s=2.0, relatch_ppp_health_poll_s=1.0,
        )
        tmpdir = Path(tempfile.mkdtemp(prefix="hil_f24r2h_ppp_timeout_"))
        calls = []
        _now, now_fn, sleep_fn = self._clock()
        ok, reason = orch.restart_r2c_ppp_with_retries(
            args, self._plan(), tmpdir,
            run_step_fn=self._runner(["", ""], [self.DOWN_STATUS, self.DOWN_STATUS, self.HEALTHY_STATUS], calls),
            sleep_fn=sleep_fn, now_fn=now_fn, print_fn=lambda s: None,
        )
        self.assertTrue(ok)
        self.assertIn("attempt 2/2 succeeded", reason)
        start_names = [c[2] for c in calls if c[2].endswith("_start.log")]
        self.assertEqual(start_names, ["ppp_r2c_attempt_1_start.log", "ppp_r2c_attempt_2_start.log"])


class TestR3aPppReadinessRetries(TestR2cPppReconnectRetries):
    """HIL-F24-R3C: R3A reuses the shared bounded PPP readiness helper."""

    def _plan(self):
        plan = orch.build_execution_plan(make_args(profile="visual", stage="r3a"))
        return {step.name: step for step in plan}

    def test_r3a_delayed_ppp_success_within_first_attempt(self):
        args = make_args(
            profile="visual", stage="r3a",
            relatch_ppp_retry_attempts=3,
            relatch_ppp_health_timeout_s=10.0,
            relatch_ppp_health_poll_s=2.0,
        )
        tmpdir = Path(tempfile.mkdtemp(prefix="hil_f24r3a_ppp_delay_"))
        calls = []
        _now, now_fn, sleep_fn = self._clock()
        ok, reason = orch.restart_ppp_with_retries(
            args, self._plan(), tmpdir, "R3A",
            run_step_fn=self._runner([""], [self.DOWN_STATUS, self.PARTIAL_PING_STATUS, self.HEALTHY_STATUS], calls),
            sleep_fn=sleep_fn, now_fn=now_fn, print_fn=lambda s: None,
        )
        self.assertTrue(ok)
        self.assertIn("attempt 1/3 succeeded", reason)
        self.assertEqual(len([c for c in calls if c[2].endswith("_start.log")]), 1)
        self.assertEqual(len([c for c in calls if c[0].startswith("ppp_cleanup")]), 3)
        self.assertEqual(len([c for c in calls if "_verify_" in c[2]]), 3)

    def test_r3a_first_attempt_timeout_second_succeeds(self):
        args = make_args(
            profile="visual", stage="r3a",
            relatch_ppp_retry_attempts=3,
            relatch_ppp_retry_delay_s=0.0,
            relatch_ppp_health_timeout_s=4.0,
            relatch_ppp_health_poll_s=2.0,
        )
        tmpdir = Path(tempfile.mkdtemp(prefix="hil_f24r3a_ppp_second_"))
        calls = []
        _now, now_fn, sleep_fn = self._clock()
        ok, reason = orch.restart_ppp_with_retries(
            args, self._plan(), tmpdir, "R3A",
            run_step_fn=self._runner(["", ""], [self.DOWN_STATUS, self.DOWN_STATUS, self.HEALTHY_STATUS], calls),
            sleep_fn=sleep_fn, now_fn=now_fn, print_fn=lambda s: None,
        )
        self.assertTrue(ok)
        self.assertIn("attempt 2/3 succeeded", reason)
        self.assertEqual(len([c for c in calls if c[2].endswith("_start.log")]), 2)
        self.assertEqual(len([c for c in calls if c[0].startswith("ppp_cleanup")]), 6)

    def test_r3a_no_cleanup_between_health_polls(self):
        args = make_args(
            profile="visual", stage="r3a",
            relatch_ppp_retry_attempts=3,
            relatch_ppp_health_timeout_s=10.0,
            relatch_ppp_health_poll_s=2.0,
        )
        tmpdir = Path(tempfile.mkdtemp(prefix="hil_f24r3a_ppp_no_cleanup_"))
        calls = []
        _now, now_fn, sleep_fn = self._clock()
        ok, _reason = orch.restart_ppp_with_retries(
            args, self._plan(), tmpdir, "R3A",
            run_step_fn=self._runner([""], [self.DOWN_STATUS, self.PARTIAL_PING_STATUS, self.HEALTHY_STATUS], calls),
            sleep_fn=sleep_fn, now_fn=now_fn, print_fn=lambda s: None,
        )
        self.assertTrue(ok)
        call_names = [c[0] for c in calls]
        self.assertEqual(call_names[:3], ["ppp_cleanup_stop", "ppp_cleanup_pid", "ppp_cleanup_ppp0"])
        self.assertEqual(call_names[3], "ppp_start")
        self.assertEqual(call_names[4:], ["ppp_verify", "ppp_verify", "ppp_verify"])


class TestR2cFeederDuration(unittest.TestCase):
    """HIL-F24-R2F tasks 1/2: r2c_feeder_duration_s() is the sum of every
    phase of the relatch workflow, not an arbitrary fixed value -- these
    tests pin down the exact formula and that each configurable phase
    timeout actually scales the result."""

    def _expected(
        self, operator_wait_budget_s, device_timeout_s, sim_json_timeout_s, duration_s,
        ppp_retry_attempts=orch.DEFAULT_RELATCH_PPP_RETRY_ATTEMPTS,
        ppp_retry_delay_s=orch.DEFAULT_RELATCH_PPP_RETRY_DELAY_S,
        ppp_health_timeout_s=orch.DEFAULT_RELATCH_PPP_HEALTH_TIMEOUT_S,
    ):
        ppp_total = ppp_retry_attempts * (orch.R2C_PPP_START_TIMEOUT_S + ppp_health_timeout_s)
        retry_delay_total = (ppp_retry_attempts - 1) * ppp_retry_delay_s
        return (
            operator_wait_budget_s + device_timeout_s
            + ppp_total + retry_delay_total
            + sim_json_timeout_s + orch.R2C_ORIGIN_CHECK_TIMEOUT_S
            + duration_s + orch.ORCHESTRATOR_TEST_MARGIN_S
            + orch.FEEDER_SHUTDOWN_MARGIN_S
        )

    def test_default_args_duration_matches_expected_sum(self):
        args = make_args(profile="dynamic", stage="r2c")
        result = orch.r2c_feeder_duration_s(
            operator_wait_budget_s=args.relatch_operator_wait_budget_s,
            device_timeout_s=args.relatch_device_timeout_s,
            sim_json_timeout_s=args.relatch_sim_json_timeout_s,
            duration_s=args.duration_s,
        )
        expected = self._expected(300.0, 120.0, 60.0, 30.0)
        self.assertAlmostEqual(result, expected, places=6)
        self.assertAlmostEqual(result, 795.0, places=6)

    def test_long_operator_delay_directly_increases_feeder_duration(self):
        """The one phase with no natural fixed value -- a human manually
        power-cycling the Pixhawk -- scales the result 1:1."""
        short = orch.r2c_feeder_duration_s(
            operator_wait_budget_s=60.0, device_timeout_s=120.0, sim_json_timeout_s=60.0, duration_s=30.0,
        )
        long = orch.r2c_feeder_duration_s(
            operator_wait_budget_s=600.0, device_timeout_s=120.0, sim_json_timeout_s=60.0, duration_s=30.0,
        )
        self.assertAlmostEqual(long - short, 540.0, places=6)

    def test_device_timeout_scales_result(self):
        base = orch.r2c_feeder_duration_s(
            operator_wait_budget_s=300.0, device_timeout_s=120.0, sim_json_timeout_s=60.0, duration_s=30.0,
        )
        longer = orch.r2c_feeder_duration_s(
            operator_wait_budget_s=300.0, device_timeout_s=220.0, sim_json_timeout_s=60.0, duration_s=30.0,
        )
        self.assertAlmostEqual(longer - base, 100.0, places=6)

    def test_sim_json_timeout_scales_result(self):
        base = orch.r2c_feeder_duration_s(
            operator_wait_budget_s=300.0, device_timeout_s=120.0, sim_json_timeout_s=60.0, duration_s=30.0,
        )
        longer = orch.r2c_feeder_duration_s(
            operator_wait_budget_s=300.0, device_timeout_s=120.0, sim_json_timeout_s=160.0, duration_s=30.0,
        )
        self.assertAlmostEqual(longer - base, 100.0, places=6)

    def test_capture_duration_scales_result(self):
        base = orch.r2c_feeder_duration_s(
            operator_wait_budget_s=300.0, device_timeout_s=120.0, sim_json_timeout_s=60.0, duration_s=30.0,
        )
        longer = orch.r2c_feeder_duration_s(
            operator_wait_budget_s=300.0, device_timeout_s=120.0, sim_json_timeout_s=60.0, duration_s=130.0,
        )
        self.assertAlmostEqual(longer - base, 100.0, places=6)

    def test_includes_the_same_shutdown_margin_every_other_profile_uses(self):
        """So the feeder is never stopped by its own internal timeout
        instead of the orchestrator's explicit shutdown sequence -- same
        HIL-F24-Q rationale static/dynamic/--stage r2's feeder_duration_s
        already relies on."""
        minimal = orch.r2c_feeder_duration_s(
            operator_wait_budget_s=0.0, device_timeout_s=0.0, sim_json_timeout_s=0.0, duration_s=0.0,
        )
        fixed_total = (
            orch.DEFAULT_RELATCH_PPP_RETRY_ATTEMPTS
            * (orch.R2C_PPP_START_TIMEOUT_S + orch.DEFAULT_RELATCH_PPP_HEALTH_TIMEOUT_S)
            + (orch.DEFAULT_RELATCH_PPP_RETRY_ATTEMPTS - 1) * orch.DEFAULT_RELATCH_PPP_RETRY_DELAY_S
            + orch.R2C_ORIGIN_CHECK_TIMEOUT_S + orch.ORCHESTRATOR_TEST_MARGIN_S + orch.FEEDER_SHUTDOWN_MARGIN_S
        )
        self.assertAlmostEqual(minimal, fixed_total, places=6)

    def test_ppp_retry_attempts_scale_result(self):
        one_attempt = orch.r2c_feeder_duration_s(
            operator_wait_budget_s=300.0, device_timeout_s=120.0, sim_json_timeout_s=60.0, duration_s=30.0,
            ppp_retry_attempts=1, ppp_retry_delay_s=5.0, ppp_health_timeout_s=30.0,
        )
        three_attempts = orch.r2c_feeder_duration_s(
            operator_wait_budget_s=300.0, device_timeout_s=120.0, sim_json_timeout_s=60.0, duration_s=30.0,
            ppp_retry_attempts=3, ppp_retry_delay_s=5.0, ppp_health_timeout_s=30.0,
        )
        self.assertAlmostEqual(
            three_attempts - one_attempt,
            2 * (orch.R2C_PPP_START_TIMEOUT_S + 30.0) + 2 * 5.0,
            places=6,
        )


class TestBuildExecutionPlanR2fFeederDuration(unittest.TestCase):
    """HIL-F24-R2F task 1/2/3/6: the plan's start_feeder/start_estimator_
    capture Steps carry the new r2c-specific duration for --stage r2c
    only; static/dynamic/--stage r2 are unaffected."""

    def _feeder_duration_from_plan(self, args):
        plan = orch.build_execution_plan(args)
        step = next(s for s in plan if s.name == "start_feeder")
        idx = step.command.index("--duration-s")
        return float(step.command[idx + 1])

    def _capture_duration_from_plan(self, args):
        plan = orch.build_execution_plan(args)
        step = next(s for s in plan if s.name == "start_estimator_capture")
        idx = step.command.index("--duration-s")
        return float(step.command[idx + 1])

    def test_r2c_feeder_duration_matches_r2c_feeder_duration_s(self):
        args = make_args(profile="dynamic", stage="r2c")
        expected = orch.r2c_feeder_duration_s(
            operator_wait_budget_s=args.relatch_operator_wait_budget_s,
            device_timeout_s=args.relatch_device_timeout_s,
            sim_json_timeout_s=args.relatch_sim_json_timeout_s,
            duration_s=args.duration_s,
        )
        self.assertAlmostEqual(self._feeder_duration_from_plan(args), expected, places=6)

    def test_r2c_capture_duration_also_matches_r2c_feeder_duration_s(self):
        """The capture process must stay alive at least as long as the
        feeder for r2c too -- not the shared (much shorter)
        feeder_duration_s static/dynamic/--stage r2 use."""
        args = make_args(profile="dynamic", stage="r2c")
        expected = orch.r2c_feeder_duration_s(
            operator_wait_budget_s=args.relatch_operator_wait_budget_s,
            device_timeout_s=args.relatch_device_timeout_s,
            sim_json_timeout_s=args.relatch_sim_json_timeout_s,
            duration_s=args.duration_s,
        )
        self.assertAlmostEqual(self._capture_duration_from_plan(args), expected, places=6)

    def test_r2c_feeder_duration_far_exceeds_r2_feeder_duration(self):
        r2c_duration = self._feeder_duration_from_plan(make_args(profile="dynamic", stage="r2c", duration_s=30.0))
        r2_duration = self._feeder_duration_from_plan(make_args(profile="dynamic", stage="r2", duration_s=30.0))
        self.assertGreater(r2c_duration, r2_duration)
        self.assertEqual(r2_duration, 35.0)  # unchanged, pre-R2F value

    def test_r2_stage_feeder_duration_unaffected(self):
        args = make_args(profile="dynamic", stage="r2", duration_s=30.0)
        self.assertEqual(self._feeder_duration_from_plan(args), 35.0)
        self.assertEqual(self._capture_duration_from_plan(args), 35.0)

    def test_static_and_dynamic_feeder_duration_unaffected(self):
        for profile in ("static", "dynamic"):
            args = make_args(profile=profile, duration_s=30.0)
            with self.subTest(profile=profile):
                self.assertEqual(self._feeder_duration_from_plan(args), 35.0)


class TestCheckR2cOriginCheckReadiness(unittest.TestCase):
    """HIL-F24-R2F tasks 4/5: the three preconditions, checked in order,
    each with an exact, distinguishable reason -- pure/testable via fake
    process/reader/count objects."""

    class _FakeProc:
        def __init__(self, poll_result):
            self._poll_result = poll_result
            self.returncode = poll_result

        def poll(self):
            return self._poll_result

    class _FakeReader:
        def __init__(self, age_s=None, exc=None):
            self._age_s = age_s
            self._exc = exc

        def read_latest(self):
            if self._exc is not None:
                raise self._exc
            return ({}, "fake-line", self._age_s, None)

    def _reader_factory(self, reader):
        return lambda path: reader

    def test_dead_feeder_is_not_ready(self):
        ready, reason = orch.check_r2c_origin_check_readiness(
            self._FakeProc(poll_result=1), "/tmp/state.csv", "/tmp/responder.csv",
            reader_factory=self._reader_factory(self._FakeReader(age_s=0.01)), count_fn=lambda p: 10,
        )
        self.assertFalse(ready)
        self.assertIn("feeder process exited", reason)

    def test_stale_state_is_not_ready(self):
        """The exact regression this task fixes: session
        sr75_hil_f24r2c_relatch_20260806T065842Z had stale_state_
        count=305 -- state.csv stopped updating because the feeder had
        already exited by the time real replies could flow."""
        ready, reason = orch.check_r2c_origin_check_readiness(
            self._FakeProc(poll_result=None), "/tmp/state.csv", "/tmp/responder.csv",
            reader_factory=self._reader_factory(self._FakeReader(age_s=10.0)), count_fn=lambda p: 10,
        )
        self.assertFalse(ready)
        self.assertIn("stale", reason.lower())

    def test_state_not_ready_exception_is_not_ready(self):
        exc = orch.responder.StateFileNotReadyError("state file does not exist: /tmp/state.csv")
        ready, reason = orch.check_r2c_origin_check_readiness(
            self._FakeProc(poll_result=None), "/tmp/state.csv", "/tmp/responder.csv",
            reader_factory=self._reader_factory(self._FakeReader(exc=exc)), count_fn=lambda p: 10,
        )
        self.assertFalse(ready)
        self.assertIn("state.csv not ready", reason)

    def test_too_few_replies_is_not_ready(self):
        ready, reason = orch.check_r2c_origin_check_readiness(
            self._FakeProc(poll_result=None), "/tmp/state.csv", "/tmp/responder.csv",
            reader_factory=self._reader_factory(self._FakeReader(age_s=0.01)), count_fn=lambda p: 2,
            min_replies=5,
        )
        self.assertFalse(ready)
        self.assertIn("only 2", reason)
        self.assertIn(">= 5", reason)

    def test_all_conditions_pass_is_ready(self):
        """Task 6's "normal R2C success path": feeder alive, fresh
        state.csv, enough replies -- exactly the state the workflow
        should be in immediately before the origin diagnostic runs."""
        ready, reason = orch.check_r2c_origin_check_readiness(
            self._FakeProc(poll_result=None), "/tmp/state.csv", "/tmp/responder.csv",
            reader_factory=self._reader_factory(self._FakeReader(age_s=0.01)), count_fn=lambda p: 7,
            min_replies=5,
        )
        self.assertTrue(ready)
        self.assertIn("feeder alive", reason)

    def test_dead_feeder_reason_reported_even_if_state_also_stale(self):
        """Task 5's "exact reason": checks run in a fixed order so the
        first genuine failure -- the more fundamental one -- is always
        what gets reported, not whichever happens to be checked last."""
        ready, reason = orch.check_r2c_origin_check_readiness(
            self._FakeProc(poll_result=2), "/tmp/state.csv", "/tmp/responder.csv",
            reader_factory=self._reader_factory(self._FakeReader(age_s=999.0)), count_fn=lambda p: 0,
        )
        self.assertFalse(ready)
        self.assertIn("feeder process exited", reason)

    def test_default_thresholds_match_documented_constants(self):
        ready, reason = orch.check_r2c_origin_check_readiness(
            self._FakeProc(poll_result=None), "/tmp/state.csv", "/tmp/responder.csv",
            reader_factory=self._reader_factory(
                self._FakeReader(age_s=orch.R2C_READINESS_MAX_STATE_AGE_S + 0.001),
            ),
            count_fn=lambda p: orch.R2C_READINESS_MIN_REPLIES,
        )
        self.assertFalse(ready)
        self.assertIn("stale", reason.lower())


class TestWaitForR2cLiveSimJsonReplies(unittest.TestCase):
    """HIL-F24-R2F: the post-PPP reply wait must not mask a dead feeder
    or stale state.csv as a generic reply timeout."""

    class _FakeProc:
        def __init__(self, poll_results):
            self._poll_results = list(poll_results)
            self.returncode = None

        def poll(self):
            if self._poll_results:
                value = self._poll_results.pop(0)
                if value is not None:
                    self.returncode = value
                return value
            return self.returncode

    class _FakeReader:
        def __init__(self, ages_s):
            self._ages_s = list(ages_s)

        def read_latest(self):
            age_s = self._ages_s.pop(0) if self._ages_s else 0.01
            return ({}, "fake-line", age_s, None)

    def _reader_factory(self, reader):
        return lambda path: reader

    def test_normal_r2c_success_path_reaches_required_new_replies(self):
        counts = iter([0, 0, 0, 2, 2, 5, 5])
        ok, reason = orch.wait_for_r2c_live_sim_json_replies(
            "/tmp/responder.csv", "/tmp/state.csv", self._FakeProc([None, None, None]),
            min_new_replies=5, timeout_s=10.0, poll_interval_s=1.0,
            count_fn=lambda p: next(counts),
            reader_factory=self._reader_factory(self._FakeReader([0.01, 0.01, 0.01])),
            sleep_fn=lambda s: None, now_fn=iter([0.0, 1.0, 2.0, 3.0]).__next__,
        )
        self.assertTrue(ok)
        self.assertIn("5 new SIM_JSON replies", reason)

    def test_stale_state_aborts_immediately_before_timeout(self):
        sleeps = []
        ok, reason = orch.wait_for_r2c_live_sim_json_replies(
            "/tmp/responder.csv", "/tmp/state.csv", self._FakeProc([None]),
            min_new_replies=5, timeout_s=1000.0, poll_interval_s=1.0,
            count_fn=lambda p: 0,
            reader_factory=self._reader_factory(self._FakeReader([10.0])),
            sleep_fn=lambda s: sleeps.append(s), now_fn=iter([0.0, 1.0]).__next__,
        )
        self.assertFalse(ok)
        self.assertIn("stale", reason.lower())
        self.assertEqual(sleeps, [])

    def test_feeder_exit_aborts_immediately(self):
        sleeps = []
        ok, reason = orch.wait_for_r2c_live_sim_json_replies(
            "/tmp/responder.csv", "/tmp/state.csv", self._FakeProc([7]),
            min_new_replies=5, timeout_s=1000.0, poll_interval_s=1.0,
            count_fn=lambda p: 0,
            reader_factory=self._reader_factory(self._FakeReader([0.01])),
            sleep_fn=lambda s: sleeps.append(s), now_fn=iter([0.0, 1.0]).__next__,
        )
        self.assertFalse(ok)
        self.assertIn("feeder process exited", reason)
        self.assertEqual(sleeps, [])

    def test_timeout_reports_reply_count_with_feeder_still_alive_and_fresh_state(self):
        counts = iter([0, 0, 2, 2])
        ok, reason = orch.wait_for_r2c_live_sim_json_replies(
            "/tmp/responder.csv", "/tmp/state.csv", self._FakeProc([None, None]),
            min_new_replies=5, timeout_s=2.0, poll_interval_s=1.0,
            count_fn=lambda p: next(counts),
            reader_factory=self._reader_factory(self._FakeReader([0.01, 0.01])),
            sleep_fn=lambda s: None, now_fn=iter([0.0, 1.0, 2.0, 3.0]).__next__,
        )
        self.assertFalse(ok)
        self.assertIn("only 2 new SIM_JSON replies", reason)


class TestFeederStaysAliveLongerWithLongerDuration(unittest.TestCase):
    """HIL-F24-R2F: real, bounded regression against the actual dynamic
    feeder script -- confirms the causal mechanism the fix depends on
    (a longer --duration-s argument genuinely keeps the real feeder
    subprocess alive longer), without needing to wait out
    r2c_feeder_duration_s()'s real (many-minutes) default total. This is
    the same feeder session sr75_hil_f24r2c_relatch_20260806T065842Z's
    feeder exited from at ~35s (JSBSim time 34.948602s) -- too short to
    survive the manual reboot + PPP restart that followed."""

    @unittest.skipUnless(shutil.which("JSBSim") is not None, "JSBSim binary not found on PATH")
    def test_feeder_still_alive_past_where_a_shorter_duration_would_have_exited(self):
        tmpdir = Path(tempfile.mkdtemp(prefix="hil_f24r2f_feeder_alive_"))
        state_csv = tmpdir / "state.csv"
        cmd = [
            sys.executable, str(orch.DYNAMIC_FEEDER_SCRIPT),
            "--output", str(state_csv),
            # Deliberately longer than the checkpoint below -- standing in
            # for r2c_feeder_duration_s()'s much larger real total, scaled
            # down so this test finishes in a few seconds instead of
            # several minutes.
            "--duration-s", "8.0",
        ]
        proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
        try:
            # A "duration-s 5.0"-style feeder (closer to the old, too-short
            # formula) would already have exited by this checkpoint.
            self.assertTrue(
                orch.wait_for_process_startup_health(proc, grace_s=6.0),
                "feeder exited before its longer --duration-s should have allowed",
            )
        finally:
            proc.terminate()
            try:
                proc.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                proc.kill()


if __name__ == "__main__":
    unittest.main()
