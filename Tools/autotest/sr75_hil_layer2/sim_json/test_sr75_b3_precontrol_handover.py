#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Tests for the SR-75 Layer 2J-B3C-C1 pre-control hold / handover state machine."""

import unittest

from sr75_sim_json_actuator_bridge import ActuatorChannelMap, NormalizedActuatorCommand
from sr75_sim_json_responder import (
    B3CommandSource,
    B3ControlPhase,
    B3PrecontrolHandover,
    B3PrecontrolReference,
)

CHANNEL_MAP = ActuatorChannelMap(optional_roles=("rato",))


def decoded_command(pwm, stale=False):
    return NormalizedActuatorCommand(
        pwm=tuple(pwm),
        left_elevon=0.0,
        right_elevon=0.0,
        elevator=0.11,
        aileron=0.22,
        rudder=0.33,
        throttle_left=0.44,
        throttle_right=0.44,
        turbojet_throttle=0.44,
        rato=0.0,
        stale=stale,
    )


VALID_PWM = [1500, 1500, 1300, 1500, 1500, 1500, 0, 1500]
INVALID_PWM_CH1 = [0, 1500, 1300, 1500, 1500, 1500, 0, 1500]
INVALID_PWM_CH7_NONZERO_OUT_OF_RANGE = [1500, 1500, 1300, 1500, 1500, 1500, 50, 1500]


def reference():
    return B3PrecontrolReference(elevator=-0.42, aileron=0.0, rudder=0.0, turbojet_throttle=0.55, rato=0.0)


class TestB3PrecontrolHandover(unittest.TestCase):
    def test_a_no_valid_pwm_sends_reference_continuously(self):
        handover = B3PrecontrolHandover(reference())
        for _ in range(5):
            raw = decoded_command(INVALID_PWM_CH1)
            command, source = handover.resolve(raw, raw, CHANNEL_MAP, host_time=1.0)
            self.assertEqual(source, B3CommandSource.PRECONTROL_REFERENCE)
            self.assertAlmostEqual(command.elevator, -0.42)
            self.assertAlmostEqual(command.turbojet_throttle, 0.55)
            self.assertEqual(handover.phase, B3ControlPhase.PRECONTROL_HOLD)
        self.assertEqual(handover.handover_count, 0)

    def test_b_first_valid_frame_transitions_exactly_once(self):
        handover = B3PrecontrolHandover(reference())
        raw_invalid = decoded_command(INVALID_PWM_CH1)
        handover.resolve(raw_invalid, raw_invalid, CHANNEL_MAP, host_time=1.0)
        handover.resolve(raw_invalid, raw_invalid, CHANNEL_MAP, host_time=1.1)
        self.assertEqual(handover.handover_count, 0)

        raw_valid = decoded_command(VALID_PWM)
        handover.resolve(raw_valid, raw_valid, CHANNEL_MAP, host_time=1.2, simulation_timestamp=4.65)
        self.assertEqual(handover.handover_count, 1)
        self.assertEqual(handover.phase, B3ControlPhase.ACTIVE_CONTROL)
        self.assertTrue(handover.first_valid_pwm_seen)
        self.assertIsNotNone(handover.transition)
        self.assertEqual(handover.transition["pwm1_4"], tuple(VALID_PWM[:4]))

        # Further valid frames must not increment handover_count again.
        for _ in range(3):
            handover.resolve(raw_valid, raw_valid, CHANNEL_MAP, host_time=2.0)
        self.assertEqual(handover.handover_count, 1)

    def test_b2_valid_pwm_can_be_held_until_startup_sync_release(self):
        handover = B3PrecontrolHandover(reference())
        raw_valid = decoded_command(VALID_PWM)

        command, source = handover.resolve(
            raw_valid,
            raw_valid,
            CHANNEL_MAP,
            host_time=1.0,
            hold_active_control=True,
        )

        self.assertEqual(source, B3CommandSource.PRECONTROL_REFERENCE)
        self.assertAlmostEqual(command.elevator, -0.42)
        self.assertTrue(handover.first_valid_pwm_seen)
        self.assertEqual(handover.phase, B3ControlPhase.PRECONTROL_HOLD)
        self.assertEqual(handover.handover_count, 0)

        command, source = handover.resolve(raw_valid, raw_valid, CHANNEL_MAP, host_time=1.1)

        self.assertEqual(source, B3CommandSource.ARDUPLANE_PWM)
        self.assertEqual(handover.phase, B3ControlPhase.ACTIVE_CONTROL)
        self.assertEqual(handover.handover_count, 1)

    def test_c_after_transition_decoded_pwm_commands_are_sent(self):
        handover = B3PrecontrolHandover(reference())
        raw_valid = decoded_command(VALID_PWM)
        handover.resolve(raw_valid, raw_valid, CHANNEL_MAP, host_time=1.0)
        self.assertEqual(handover.phase, B3ControlPhase.ACTIVE_CONTROL)

        raw_next = decoded_command(VALID_PWM)
        raw_next2 = NormalizedActuatorCommand(
            pwm=raw_next.pwm, left_elevon=0.0, right_elevon=0.0,
            elevator=0.5, aileron=-0.5, rudder=0.1,
            throttle_left=0.6, throttle_right=0.6, turbojet_throttle=0.6, rato=0.0, stale=False,
        )
        command, source = handover.resolve(raw_next2, raw_next2, CHANNEL_MAP, host_time=1.1)
        self.assertEqual(source, B3CommandSource.ARDUPLANE_PWM)
        self.assertAlmostEqual(command.elevator, 0.5)
        self.assertAlmostEqual(command.aileron, -0.5)

    def test_d_stale_after_transition_uses_neutral_not_reference(self):
        handover = B3PrecontrolHandover(reference())
        raw_valid = decoded_command(VALID_PWM)
        handover.resolve(raw_valid, raw_valid, CHANNEL_MAP, host_time=1.0)

        neutral = NormalizedActuatorCommand.neutral()
        command, source = handover.resolve(neutral, neutral, CHANNEL_MAP, host_time=2.0)
        self.assertEqual(source, B3CommandSource.ACTIVE_STALE_FAILSAFE)
        self.assertAlmostEqual(command.elevator, 0.0)
        self.assertNotAlmostEqual(command.elevator, -0.42)
        self.assertEqual(handover.phase, B3ControlPhase.ACTIVE_CONTROL)

    def test_e_ch7_zero_is_still_a_valid_transition_frame(self):
        handover = B3PrecontrolHandover(reference())
        raw_valid = decoded_command(VALID_PWM)  # CH7 (index 7) is 0, an optional role.
        self.assertEqual(raw_valid.pwm[6], 0)
        _, source = handover.resolve(raw_valid, raw_valid, CHANNEL_MAP, host_time=1.0)
        self.assertEqual(source, B3CommandSource.ARDUPLANE_PWM)
        self.assertEqual(handover.handover_count, 1)

        handover2 = B3PrecontrolHandover(reference())
        raw_ch7_bad = decoded_command(INVALID_PWM_CH7_NONZERO_OUT_OF_RANGE)
        _, source2 = handover2.resolve(raw_ch7_bad, raw_ch7_bad, CHANNEL_MAP, host_time=1.0)
        self.assertEqual(source2, B3CommandSource.PRECONTROL_REFERENCE)
        self.assertEqual(handover2.handover_count, 0)

    def test_f_generic_behavior_unchanged_when_precontrol_absent(self):
        handover = B3PrecontrolHandover(None)
        self.assertFalse(handover.enabled)
        self.assertEqual(handover.phase, B3ControlPhase.ACTIVE_CONTROL)

        raw_invalid = decoded_command(INVALID_PWM_CH1)
        command, source = handover.resolve(raw_invalid, raw_invalid, CHANNEL_MAP, host_time=1.0)
        self.assertEqual(source, B3CommandSource.ARDUPLANE_PWM)
        self.assertIs(command, raw_invalid)
        self.assertEqual(handover.handover_count, 0)

        neutral = NormalizedActuatorCommand.neutral()
        command2, source2 = handover.resolve(neutral, neutral, CHANNEL_MAP, host_time=2.0)
        self.assertEqual(source2, B3CommandSource.ACTIVE_STALE_FAILSAFE)
        self.assertIs(command2, neutral)


if __name__ == "__main__":
    unittest.main()
