#!/usr/bin/env python3
"""HIL-F24-F unit tests for sr75_hil_f24f_hardware_orchestrator.py's pure
gating/planning logic. No hardware, no subprocess is ever spawned by these
tests -- they exercise should_proceed_to_hardware(), check_adapter_
presence(), build_session_dir(), and build_execution_plan() only.
"""
import argparse
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


if __name__ == "__main__":
    unittest.main()
