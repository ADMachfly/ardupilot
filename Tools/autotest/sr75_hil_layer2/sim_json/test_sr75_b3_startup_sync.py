#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Tests for the SR-75 Layer 2J-B3C-C2 startup-synchronization logic."""

import math
import unittest

from sr75_sim_json_actuator_bridge import ActuatorChannelMap, NormalizedActuatorCommand
from sr75_sim_json_responder import (
    B3StartupPhase,
    B3StartupSync,
    B3StartupSyncTarget,
    B3TimingQualityGate,
    StateMapper,
    degrees_to_radians,
)

CHANNEL_MAP = ActuatorChannelMap(optional_roles=("rato",))
VALID_PWM = [1500, 1500, 1300, 1500, 1500, 1500, 0, 1500]
INVALID_PWM = [0, 1500, 1300, 1500, 1500, 1500, 0, 1500]


def decoded_command(pwm, stale=False):
    return NormalizedActuatorCommand(
        pwm=tuple(pwm),
        left_elevon=0.0, right_elevon=0.0,
        elevator=0.1, aileron=0.0, rudder=0.0,
        throttle_left=0.5, throttle_right=0.5, turbojet_throttle=0.5, rato=0.0,
        stale=stale,
    )


def make_real_state(target, roll_deg, pitch_deg, timestamp_s, reused=False, p=0.0, q=0.0, r=0.0):
    from sr75_sim_json_responder import SimState, euler_to_quaternion

    roll = degrees_to_radians(roll_deg)
    pitch = degrees_to_radians(pitch_deg)
    yaw = degrees_to_radians(target.yaw_deg)
    return SimState(
        timestamp_s=timestamp_s,
        source_timestamp_s=timestamp_s,
        latitude_deg=target.latitude_deg,
        longitude_deg=target.longitude_deg,
        altitude_m=target.altitude_m,
        roll_rad=roll,
        pitch_rad=pitch,
        yaw_rad=yaw,
        quaternion=euler_to_quaternion(roll, pitch, yaw),
        gyro_rad_s=(p, q, r),
        accel_body_mss=(0.0, 0.0, -9.80665),
        velocity_ned_mps=(target.airspeed_mps, 0.0, 0.0),
        airspeed_mps=target.airspeed_mps,
        row_signature=f"row:{timestamp_s}",
        row_monotonic_time=timestamp_s,
        reused_source_row=reused,
    )


def target():
    return B3StartupSyncTarget(roll_deg=15.0, pitch_deg=-2.0, yaw_deg=315.0, altitude_m=3000.0, airspeed_mps=69.0)


class TestB3StartupSync(unittest.TestCase):
    def released_sync_with_offset(self, offset):
        sync = B3StartupSync(target())
        sync._last_hold_timestamp = offset
        raw_valid = decoded_command(VALID_PWM)
        sync.maybe_release(raw_valid, CHANNEL_MAP, host_time=10.0, pwm=raw_valid.pwm)
        return sync, raw_valid

    def test_no_scoring_before_control_readiness(self):
        sync = B3StartupSync(target())
        self.assertFalse(sync.scoring_eligible(sync.held_reply_state(1.0), decoded_command(VALID_PWM)))
        self.assertIsNone(sync.scoring_start_record)
        self.assertEqual(sync.phase, B3StartupPhase.STARTUP_SYNC_HOLD)

    def test_exactly_one_release(self):
        sync = B3StartupSync(target())
        raw_invalid = decoded_command(INVALID_PWM)
        for _ in range(3):
            sync.maybe_release(raw_invalid, CHANNEL_MAP, host_time=1.0, pwm=raw_invalid.pwm)
        self.assertEqual(sync.release_count, 0)

        raw_valid = decoded_command(VALID_PWM)
        sync.maybe_release(raw_valid, CHANNEL_MAP, host_time=2.0, pwm=raw_valid.pwm)
        self.assertEqual(sync.release_count, 1)
        self.assertEqual(sync.phase, B3StartupPhase.RELEASED)

        # Further calls, even with valid frames, must not release again.
        for _ in range(5):
            sync.maybe_release(raw_valid, CHANNEL_MAP, host_time=3.0, pwm=raw_valid.pwm)
        self.assertEqual(sync.release_count, 1)

    def test_pre_release_rows_excluded_from_scoring(self):
        sync = B3StartupSync(target())
        for i in range(10):
            sync.held_reply_state(host_time=1.0 + i * 0.02)
            self.assertIsNone(sync.scoring_start_record)
        self.assertEqual(sync._fresh_rows_since_release, 0)  # internal check: nothing counted pre-release

    def test_post_release_timestamps_monotonic(self):
        sync = B3StartupSync(target())
        last_held = None
        for i in range(20):
            state = sync.held_reply_state(host_time=1.0 + i * 0.02)
            if last_held is not None:
                self.assertGreater(state.timestamp_s, last_held)
            last_held = state.timestamp_s

        raw_valid = decoded_command(VALID_PWM)
        sync.maybe_release(raw_valid, CHANNEL_MAP, host_time=1.4, pwm=raw_valid.pwm)

        real1 = make_real_state(target(), roll_deg=12.0, pitch_deg=-1.2, timestamp_s=0.000001, p=0.01, q=0.01, r=0.01)
        out1 = sync.process_post_release_state(real1)
        self.assertGreater(out1.timestamp_s, last_held)

        real2 = make_real_state(target(), roll_deg=12.1, pitch_deg=-1.3, timestamp_s=0.05, p=0.01, q=0.01, r=0.01)
        out2 = sync.process_post_release_state(real2)
        self.assertGreater(out2.timestamp_s, out1.timestamp_s)

    def test_post_release_transform_does_not_mutate_mapper_state(self):
        sync, raw_valid = self.released_sync_with_offset(4.326260231)
        mapped = make_real_state(target(), roll_deg=12.0, pitch_deg=-1.0, timestamp_s=0.000001)

        published = sync.process_post_release_state(mapped, raw_decoded_command=raw_valid, channel_map=CHANNEL_MAP)

        self.assertIsNot(published, mapped)
        self.assertAlmostEqual(mapped.timestamp_s, 0.000001, places=9)
        self.assertAlmostEqual(published.timestamp_s, 4.326261231, places=9)
        self.assertNotAlmostEqual(mapped.roll_rad, published.roll_rad, places=9)

    def test_mapper_ownership_and_published_timestamps_keep_one_offset(self):
        offset = 4.326260231
        sync, raw_valid = self.released_sync_with_offset(offset)
        mapper = StateMapper(strict=True)
        timestamps = (0.000001, 0.016667, 0.033334)
        expected = (4.326261231, 4.342927231, 4.359594231)
        published = []

        for timestamp in timestamps:
            mapped = make_real_state(target(), roll_deg=12.0, pitch_deg=-1.0, timestamp_s=timestamp)
            out = sync.process_post_release_state(mapped, raw_decoded_command=raw_valid, channel_map=CHANNEL_MAP)
            mapper.accept_state(mapped)
            published.append(out.timestamp_s)

        for actual, want in zip(published, expected):
            self.assertAlmostEqual(actual, want, places=9)
        self.assertAlmostEqual(mapper.last_outgoing_timestamp, 0.033334, places=9)

    def test_mapper_last_outgoing_timestamp_after_first_row_remains_unoffset(self):
        sync, raw_valid = self.released_sync_with_offset(4.326260231)
        mapper = StateMapper(strict=True)
        mapped = make_real_state(target(), roll_deg=12.0, pitch_deg=-1.0, timestamp_s=0.000001)

        sync.process_post_release_state(mapped, raw_decoded_command=raw_valid, channel_map=CHANNEL_MAP)
        mapper.accept_state(mapped)

        self.assertAlmostEqual(mapper.last_outgoing_timestamp, 0.000001, places=9)

    def test_second_row_outgoing_dt_matches_mapper_dt(self):
        sync, raw_valid = self.released_sync_with_offset(4.326260231)
        mapped1 = make_real_state(target(), roll_deg=12.0, pitch_deg=-1.0, timestamp_s=0.000001)
        mapped2 = make_real_state(target(), roll_deg=12.1, pitch_deg=-1.1, timestamp_s=0.016667)

        out1 = sync.process_post_release_state(mapped1, raw_decoded_command=raw_valid, channel_map=CHANNEL_MAP)
        out2 = sync.process_post_release_state(mapped2, raw_decoded_command=raw_valid, channel_map=CHANNEL_MAP)

        self.assertAlmostEqual(out2.timestamp_s - out1.timestamp_s, 0.016666, places=9)

    def test_500_rows_do_not_accumulate_startup_offset(self):
        offset = 4.326260231
        sync, raw_valid = self.released_sync_with_offset(offset)
        mapper_timestamps = [0.000001 + i * 0.016666 for i in range(500)]
        published = []

        for timestamp in mapper_timestamps:
            mapped = make_real_state(target(), roll_deg=12.0, pitch_deg=-1.0, timestamp_s=timestamp)
            out = sync.process_post_release_state(mapped, raw_decoded_command=raw_valid, channel_map=CHANNEL_MAP)
            published.append(out.timestamp_s)

        dts = [b - a for a, b in zip(published, published[1:])]
        self.assertTrue(all(dt > 0.0 for dt in dts))
        self.assertLess(max(dts), 0.05)
        self.assertAlmostEqual(published[-1], mapper_timestamps[-1] + offset, places=9)

    def test_release_boundary_first_real_row_remains_timing_baseline(self):
        offset = 4.326260231
        gate = B3TimingQualityGate(True)
        gate.on_startup_sync_release(final_held_outgoing_timestamp=offset, release_host_time=10.0, release_count=1)
        result = gate.check(
            125,
            host_time=10.1,
            source_simulation_timestamp=4.483154,
            outgoing_simulation_timestamp=offset + 0.000001,
        )

        self.assertIsNone(result)
        self.assertEqual(gate.baseline_label_for_request(125), "TIMING_BASELINE")

    def test_disabled_generic_mapper_behavior_unchanged(self):
        mapper = StateMapper(strict=True)
        first = make_real_state(target(), roll_deg=0.0, pitch_deg=0.0, timestamp_s=1.0)
        second = make_real_state(target(), roll_deg=0.0, pitch_deg=0.0, timestamp_s=1.016666)

        mapper.accept_state(first)
        mapper.accept_state(second)

        self.assertAlmostEqual(mapper.last_outgoing_timestamp, 1.016666, places=9)

    def test_bias_locks_first_released_row_to_target_and_scoring_needs_two_fresh_rows(self):
        sync = B3StartupSync(target())
        raw_valid = decoded_command(VALID_PWM)
        sync.maybe_release(raw_valid, CHANNEL_MAP, host_time=1.0, pwm=raw_valid.pwm)

        real1 = make_real_state(target(), roll_deg=12.037, pitch_deg=-1.215, timestamp_s=4.383, p=0.01, q=0.01, r=0.03)
        out1 = sync.process_post_release_state(real1, raw_decoded_command=raw_valid, channel_map=CHANNEL_MAP)
        self.assertAlmostEqual(math.degrees(out1.roll_rad), 15.0, places=6)
        self.assertAlmostEqual(math.degrees(out1.pitch_rad), -2.0, places=6)
        self.assertFalse(sync.scoring_eligible(out1, raw_valid, CHANNEL_MAP))  # only 1 fresh row so far

        real2 = make_real_state(target(), roll_deg=12.05, pitch_deg=-1.22, timestamp_s=4.4, p=0.01, q=0.01, r=0.03)
        out2 = sync.process_post_release_state(real2, raw_decoded_command=raw_valid, channel_map=CHANNEL_MAP)
        self.assertTrue(sync.scoring_eligible(out2, raw_valid, CHANNEL_MAP))
        self.assertIsNotNone(sync.scoring_start_record)

    def test_scoring_not_eligible_if_rates_too_high(self):
        sync = B3StartupSync(target())
        raw_valid = decoded_command(VALID_PWM)
        sync.maybe_release(raw_valid, CHANNEL_MAP, host_time=1.0, pwm=raw_valid.pwm)
        real1 = make_real_state(target(), roll_deg=15.0, pitch_deg=-2.0, timestamp_s=1.0, p=0.5, q=0.0, r=0.0)
        sync.process_post_release_state(real1, raw_decoded_command=raw_valid, channel_map=CHANNEL_MAP)
        real2 = make_real_state(target(), roll_deg=15.0, pitch_deg=-2.0, timestamp_s=1.05, p=0.5, q=0.0, r=0.0)
        out2 = sync.process_post_release_state(real2, raw_decoded_command=raw_valid, channel_map=CHANNEL_MAP)
        self.assertFalse(sync.scoring_eligible(out2, raw_valid, CHANNEL_MAP))

    def test_scoring_not_eligible_if_rato_nonzero(self):
        sync = B3StartupSync(target())
        raw_valid = decoded_command(VALID_PWM)
        sync.maybe_release(raw_valid, CHANNEL_MAP, host_time=1.0, pwm=raw_valid.pwm)
        raw_rato = decoded_command(VALID_PWM)
        object.__setattr__(raw_rato, "rato", 0.2)
        real1 = make_real_state(target(), roll_deg=15.0, pitch_deg=-2.0, timestamp_s=1.0)
        sync.process_post_release_state(real1, raw_decoded_command=raw_rato, channel_map=CHANNEL_MAP)
        real2 = make_real_state(target(), roll_deg=15.0, pitch_deg=-2.0, timestamp_s=1.05)
        out2 = sync.process_post_release_state(real2, raw_decoded_command=raw_rato, channel_map=CHANNEL_MAP)
        self.assertIsNone(sync.scoring_start_record)
        self.assertFalse(sync.scoring_eligible(out2, raw_rato, CHANNEL_MAP))

    def test_scoring_not_eligible_if_pwm_invalid(self):
        sync = B3StartupSync(target())
        raw_valid = decoded_command(VALID_PWM)
        sync.maybe_release(raw_valid, CHANNEL_MAP, host_time=1.0, pwm=raw_valid.pwm)
        raw_bad_pwm = decoded_command(INVALID_PWM)
        real1 = make_real_state(target(), roll_deg=15.0, pitch_deg=-2.0, timestamp_s=1.0)
        sync.process_post_release_state(real1, raw_decoded_command=raw_bad_pwm, channel_map=CHANNEL_MAP)
        real2 = make_real_state(target(), roll_deg=15.0, pitch_deg=-2.0, timestamp_s=1.05)
        out2 = sync.process_post_release_state(real2, raw_decoded_command=raw_bad_pwm, channel_map=CHANNEL_MAP)
        self.assertIsNone(sync.scoring_start_record)
        self.assertFalse(sync.scoring_eligible(out2, raw_bad_pwm, CHANNEL_MAP))

    def test_generic_b1_b2_behavior_unchanged_when_disabled(self):
        sync = B3StartupSync(None)
        self.assertFalse(sync.enabled)
        self.assertEqual(sync.phase, B3StartupPhase.RELEASED)
        raw_invalid = decoded_command(INVALID_PWM)
        # maybe_release() must be a no-op when disabled.
        sync.maybe_release(raw_invalid, CHANNEL_MAP, host_time=1.0, pwm=raw_invalid.pwm)
        self.assertEqual(sync.release_count, 0)


if __name__ == "__main__":
    unittest.main()
