#!/usr/bin/env python3
"""HIL-F24-F unit tests for sr75_hil_f24f_hardware_orchestrator.py's pure
gating/planning logic. No hardware, no subprocess is ever spawned by these
tests -- they exercise should_proceed_to_hardware(), check_adapter_
presence(), build_session_dir(), and build_execution_plan() only.
"""
import argparse
import shutil
import subprocess
import sys
import tempfile
import unittest
from datetime import datetime, timezone
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import sr75_hil_f24f_hardware_orchestrator as orch  # noqa: E402


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
    def test_all_eight_placeholders_present(self):
        paths = orch.session_artifact_paths(Path("/tmp/sessions/example"))
        expected_keys = {
            "state_csv", "responder_csv", "jsbsim_truth_csv", "pixhawk_estimator_csv",
            "estimator_comparison_csv", "estimator_summary_json", "estimator_summary_md",
            "origin_relatch_summary_json",
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


class TestBuildSessionDirR2cStage(unittest.TestCase):
    def test_r2c_stage_name_distinct(self):
        now = datetime(2026, 8, 6, 12, 0, 0, tzinfo=timezone.utc)
        path = orch.build_session_dir(Path("/tmp/sessions"), now=now, profile="dynamic", stage="r2c")
        self.assertEqual(path, Path("/tmp/sessions/sr75_hil_f24r2c_relatch_20260806T120000Z"))


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
