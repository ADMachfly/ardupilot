#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Software-only tests for the SR-75 SIM_JSON actuator decoder."""

import math
import os
import unittest

from sr75_sim_json_actuator_bridge import (
    ActuatorChannelMap,
    ActuatorMapError,
    DEFAULT_CHANNELS,
    JSBSIM_PROPERTIES,
    NormalizedActuatorCommand,
    SoftwareActuatorBridge,
    neutral_pwm_values,
    normalize_surface_pwm,
    normalize_throttle_pwm,
)


def neutral_pwm():
    return neutral_pwm_values()


class TestSR75SIMJSONActuatorBridge(unittest.TestCase):
    def setUp(self):
        channel_map = ActuatorChannelMap(timeout_ms=100.0, surface_deadband=0.02, throttle_deadband=0.01)
        self.bridge = SoftwareActuatorBridge(channel_map)

    def test_surface_pwm_normalization(self):
        self.assertEqual(normalize_surface_pwm(1000, 0.0), -1.0)
        self.assertEqual(normalize_surface_pwm(1500, 0.0), 0.0)
        self.assertEqual(normalize_surface_pwm(2000, 0.0), 1.0)

    def test_throttle_pwm_normalization(self):
        self.assertEqual(normalize_throttle_pwm(1000, 0.0), 0.0)
        self.assertEqual(normalize_throttle_pwm(1500, 0.0), 0.5)
        self.assertEqual(normalize_throttle_pwm(2000, 0.0), 1.0)

    def test_neutral_packet_has_zero_throttle_and_rato_off(self):
        command = self.bridge.update_from_pwm(neutral_pwm(), now=1.0)
        self.assertEqual(command.left_elevon, 0.0)
        self.assertEqual(command.right_elevon, 0.0)
        self.assertEqual(command.rudder, 0.0)
        self.assertEqual(command.turbojet_throttle, 0.0)
        self.assertEqual(command.rato, 0.0)
        self.assertEqual(command.eject, 0.0)

    def test_documented_channel_map_matches_loaded_map(self):
        map_path = os.path.join(os.path.dirname(__file__), "SR75_SIM_JSON_CHANNEL_MAP.json")
        channel_map = ActuatorChannelMap.from_json_file(map_path)
        self.assertEqual(channel_map.channels, DEFAULT_CHANNELS)
        self.assertEqual(channel_map.channels["rato"], 7)
        self.assertEqual(channel_map.channels["eject"], 8)
        self.assertEqual(channel_map.optional_roles, ("eject",))

    def test_dry_booster_uses_writable_ballast_tank_property(self):
        self.assertEqual(JSBSIM_PROPERTIES["dry_booster_weight"], "propulsion/tank[2]/contents-lbs")

    def test_b3_channel_map_marks_rato_optional(self):
        map_path = os.path.join(
            os.path.dirname(__file__),
            "..",
            "closed_loop",
            "SR75_LAYER2J_B3_CHANNEL_MAP.json",
        )
        channel_map = ActuatorChannelMap.from_json_file(map_path)
        self.assertEqual(channel_map.channels, DEFAULT_CHANNELS)
        self.assertEqual(channel_map.optional_roles, ("eject", "rato"))

    def test_optional_rato_zero_is_off_without_warning(self):
        channel_map = ActuatorChannelMap(channels=DEFAULT_CHANNELS, optional_roles=("rato",))
        bridge = SoftwareActuatorBridge(channel_map)
        pwm = neutral_pwm()
        pwm[6] = 0
        command = bridge.update_from_pwm(pwm, now=1.0)
        self.assertEqual(command.rato, 0.0)
        self.assertNotIn("ch7:out_of_range:0", command.warnings)

    def test_unmapped_zero_pwm_channels_do_not_warn(self):
        channel_map = ActuatorChannelMap(channels=DEFAULT_CHANNELS, optional_roles=("rato",))
        bridge = SoftwareActuatorBridge(channel_map)
        pwm = neutral_pwm()
        for index in (4, 5, 7, 8, 9, 10, 11, 12, 13, 14, 15):
            pwm[index] = 0
        command = bridge.update_from_pwm(pwm, now=1.0)
        self.assertEqual(command.warnings, ())

    def test_optional_eject_zero_is_off_without_warning(self):
        pwm = neutral_pwm()
        pwm[7] = 0
        command = self.bridge.update_from_pwm(pwm, now=1.0)
        self.assertEqual(command.eject, 0.0)
        self.assertNotIn("ch8:out_of_range:0", command.warnings)

    def test_optional_rato_does_not_weaken_strict_map(self):
        pwm = neutral_pwm()
        pwm[6] = 0
        strict_command = self.bridge.update_from_pwm(pwm, now=1.0)
        self.assertEqual(strict_command.rato, 0.0)
        self.assertIn("ch7:out_of_range:0", strict_command.warnings)

    def test_out_of_range_pwm_is_warned_and_clamped(self):
        pwm = neutral_pwm()
        pwm[0] = 2300
        pwm[2] = 700
        command = self.bridge.update_from_pwm(pwm, now=1.0)
        self.assertEqual(command.left_elevon, 1.0)
        self.assertEqual(command.throttle_left, 0.0)
        self.assertIn("ch1:out_of_range:2300", command.warnings)
        self.assertIn("ch3:out_of_range:700", command.warnings)

    def test_stale_timeout_returns_failsafe_neutral(self):
        pwm = neutral_pwm()
        pwm[2] = 2000
        self.bridge.update_from_pwm(pwm, now=1.0)
        command = self.bridge.current_command(now=1.05)
        self.assertFalse(command.stale)
        stale = self.bridge.current_command(now=1.2)
        self.assertTrue(stale.stale)
        self.assertEqual(stale.elevator, 0.0)
        self.assertEqual(stale.aileron, 0.0)
        self.assertEqual(stale.rudder, 0.0)
        self.assertEqual(stale.turbojet_throttle, 0.0)
        self.assertEqual(stale.throttle_left, 0.0)
        self.assertEqual(stale.throttle_right, 0.0)
        self.assertEqual(stale.rato, 0.0)
        self.assertEqual(stale.eject, 0.0)

    def test_single_elevon_response(self):
        pwm = neutral_pwm()
        pwm[0] = 2000
        command = self.bridge.update_from_pwm(pwm, now=1.0)
        self.assertEqual(command.left_elevon, 1.0)
        self.assertEqual(command.right_elevon, 0.0)
        self.assertEqual(command.elevator, 0.5)
        self.assertEqual(command.aileron, 0.5)

    def test_symmetric_pitch_command(self):
        pwm = neutral_pwm()
        pwm[0] = 2000
        pwm[1] = 2000
        command = self.bridge.update_from_pwm(pwm, now=1.0)
        self.assertEqual(command.elevator, 1.0)
        self.assertEqual(command.aileron, 0.0)

    def test_differential_roll_command(self):
        pwm = neutral_pwm()
        pwm[0] = 2000
        pwm[1] = 1000
        command = self.bridge.update_from_pwm(pwm, now=1.0)
        self.assertEqual(command.elevator, 0.0)
        self.assertEqual(command.aileron, 1.0)

    def test_rudder_response(self):
        pwm = neutral_pwm()
        pwm[3] = 1000
        command = self.bridge.update_from_pwm(pwm, now=1.0)
        self.assertEqual(command.rudder, -1.0)

    def test_throttle_response(self):
        pwm = neutral_pwm()
        pwm[2] = 2000
        command = self.bridge.update_from_pwm(pwm, now=1.0)
        self.assertEqual(command.throttle_left, 1.0)
        self.assertEqual(command.throttle_right, 1.0)
        self.assertEqual(command.turbojet_throttle, 1.0)

    def test_rato_simulated_command_response(self):
        pwm = neutral_pwm()
        pwm[6] = 2000
        command = self.bridge.update_from_pwm(pwm, now=1.0)
        self.assertEqual(command.rato, 1.0)

    def test_eject_simulated_command_response(self):
        pwm = neutral_pwm()
        pwm[7] = 2000
        command = self.bridge.update_from_pwm(pwm, now=1.0)
        self.assertEqual(command.eject, 1.0)
        self.assertEqual(command.log_fields()["act_eject_norm"], "1.000000")

    def test_low_eject_pwm_is_off(self):
        pwm = neutral_pwm()
        pwm[7] = 1000
        command = self.bridge.update_from_pwm(pwm, now=1.0)
        self.assertEqual(command.eject, 0.0)

    def test_malformed_eject_pwm_warns_and_fails_low(self):
        for malformed in ("bad", math.nan):
            with self.subTest(malformed=malformed):
                pwm = neutral_pwm()
                pwm[7] = malformed
                command = self.bridge.update_from_pwm(pwm, now=1.0)
                self.assertEqual(command.eject, 0.0)
                self.assertIn("ch8:malformed", command.warnings)

    def test_no_physical_output_interface(self):
        command = self.bridge.update_from_pwm(neutral_pwm(), now=1.0)
        self.assertIsInstance(command, NormalizedActuatorCommand)
        self.assertFalse(hasattr(self.bridge, "serial"))
        self.assertFalse(hasattr(self.bridge, "gpio"))
        self.assertFalse(hasattr(self.bridge, "servo_output"))

    def test_malformed_channel_map_rejected(self):
        with self.assertRaises(ActuatorMapError):
            ActuatorChannelMap(channels=dict(DEFAULT_CHANNELS, rato=0))
        with self.assertRaises(ActuatorMapError):
            ActuatorChannelMap(channels=dict(DEFAULT_CHANNELS, unknown=5))
        with self.assertRaises(ActuatorMapError):
            ActuatorChannelMap(channels={"rato": 7})
        with self.assertRaises(ActuatorMapError):
            ActuatorChannelMap(timeout_ms=0.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
