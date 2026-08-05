#!/usr/bin/env python3
"""HIL-F24-C unit tests for sr75_hil_f24c_preflash_precheck.py's pure
STOP/GO evaluation logic.

No hardware, no MAVLink connection.
"""
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import sr75_hil_f24c_preflash_precheck as precheck  # noqa: E402


def good_soh_params():
    return dict(precheck.EXPECTED_SOH_PARAMS)


class TestEvaluatePrecheckBasic(unittest.TestCase):
    def test_disarmed_manual_quiet_channels_is_go(self):
        go, reasons = precheck.evaluate_precheck(
            armed=False, mode="MANUAL", ch7_pwm=0, ch8_pwm=0,
            param_values={}, require_soh_params=False, custom_fw_string=None,
        )
        self.assertTrue(go, reasons)
        self.assertEqual(reasons, [])

    def test_absent_ch_pwm_is_still_go(self):
        go, reasons = precheck.evaluate_precheck(
            armed=False, mode="MANUAL", ch7_pwm=None, ch8_pwm=None,
            param_values={}, require_soh_params=False, custom_fw_string=None,
        )
        self.assertTrue(go, reasons)

    def test_armed_is_stop(self):
        go, reasons = precheck.evaluate_precheck(
            armed=True, mode="MANUAL", ch7_pwm=0, ch8_pwm=0,
            param_values={}, require_soh_params=False, custom_fw_string=None,
        )
        self.assertFalse(go)
        self.assertTrue(any("ARMED" in r for r in reasons))

    def test_wrong_mode_is_stop(self):
        go, reasons = precheck.evaluate_precheck(
            armed=False, mode="AUTO", ch7_pwm=0, ch8_pwm=0,
            param_values={}, require_soh_params=False, custom_fw_string=None,
        )
        self.assertFalse(go)
        self.assertTrue(any("mode=AUTO" in r for r in reasons))

    def test_ch7_activity_is_stop(self):
        go, reasons = precheck.evaluate_precheck(
            armed=False, mode="MANUAL", ch7_pwm=1500, ch8_pwm=0,
            param_values={}, require_soh_params=False, custom_fw_string=None,
        )
        self.assertFalse(go)
        self.assertTrue(any("CH7" in r for r in reasons))

    def test_ch8_activity_is_stop(self):
        go, reasons = precheck.evaluate_precheck(
            armed=False, mode="MANUAL", ch7_pwm=0, ch8_pwm=1500,
            param_values={}, require_soh_params=False, custom_fw_string=None,
        )
        self.assertFalse(go)
        self.assertTrue(any("CH8" in r for r in reasons))

    def test_small_pwm_jitter_within_quiet_threshold_is_go(self):
        go, reasons = precheck.evaluate_precheck(
            armed=False, mode="MANUAL", ch7_pwm=12, ch8_pwm=0,
            param_values={}, require_soh_params=False, custom_fw_string=None,
        )
        self.assertTrue(go, reasons)


class TestEvaluatePrecheckSoHParams(unittest.TestCase):
    def test_soh_params_not_checked_when_flag_false(self):
        go, reasons = precheck.evaluate_precheck(
            armed=False, mode="MANUAL", ch7_pwm=0, ch8_pwm=0,
            param_values={}, require_soh_params=False, custom_fw_string=None,
        )
        self.assertTrue(go, reasons)  # empty param_values would fail if checked

    def test_all_correct_soh_params_and_banner_is_go(self):
        go, reasons = precheck.evaluate_precheck(
            armed=False, mode="MANUAL", ch7_pwm=0, ch8_pwm=0,
            param_values=good_soh_params(), require_soh_params=True,
            custom_fw_string="SR75-SIMULATION-FIRMWARE-fmuv3-SoH-DO-NOT-FLY",
        )
        self.assertTrue(go, reasons)

    def test_each_soh_param_individually_gates(self):
        for name in precheck.EXPECTED_SOH_PARAMS:
            params = good_soh_params()
            params[name] = -1
            go, reasons = precheck.evaluate_precheck(
                armed=False, mode="MANUAL", ch7_pwm=0, ch8_pwm=0,
                param_values=params, require_soh_params=True,
                custom_fw_string="SR75-SIMULATION-FIRMWARE-fmuv3-SoH-DO-NOT-FLY",
            )
            self.assertFalse(go, f"{name}=-1 should have failed")
            self.assertTrue(any(name in r for r in reasons))

    def test_missing_soh_param_response_is_stop(self):
        params = good_soh_params()
        params["AHRS_EKF_TYPE"] = None
        go, reasons = precheck.evaluate_precheck(
            armed=False, mode="MANUAL", ch7_pwm=0, ch8_pwm=0,
            param_values=params, require_soh_params=True,
            custom_fw_string="SR75-SIMULATION-FIRMWARE-fmuv3-SoH-DO-NOT-FLY",
        )
        self.assertFalse(go)
        self.assertTrue(any("did not respond" in r for r in reasons))

    def test_missing_simulation_banner_is_stop(self):
        go, reasons = precheck.evaluate_precheck(
            armed=False, mode="MANUAL", ch7_pwm=0, ch8_pwm=0,
            param_values=good_soh_params(), require_soh_params=True,
            custom_fw_string="ArduPlane V4.6.0",
        )
        self.assertFalse(go)
        self.assertTrue(any("SIMULATION" in r for r in reasons))

    def test_no_banner_captured_is_not_itself_a_stop(self):
        # a missed STATUSTEXT capture window shouldn't be treated the same
        # as a confirmed-wrong banner -- only an explicit mismatch stops.
        go, reasons = precheck.evaluate_precheck(
            armed=False, mode="MANUAL", ch7_pwm=0, ch8_pwm=0,
            param_values=good_soh_params(), require_soh_params=True,
            custom_fw_string=None,
        )
        self.assertTrue(go, reasons)


if __name__ == "__main__":
    unittest.main()
