#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Tests for the SR-75 Layer 2J-B3C-C2 startup-synchronization logic."""

import math
import unittest
from contextlib import redirect_stdout
from io import StringIO

from sr75_sim_json_actuator_bridge import ActuatorChannelMap, NormalizedActuatorCommand
from sr75_sim_json_responder import (
    B3StartupPhase,
    B3StartupSync,
    B3StartupSyncTarget,
    B3TimingQualityGate,
    ControlPacket,
    SERVO16_MAGIC,
    StateMapper,
    degrees_to_radians,
    make_log_row,
    print_b3_startup_scoring_debug,
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


def command_with(**overrides):
    command = decoded_command(VALID_PWM)
    for name, value in overrides.items():
        object.__setattr__(command, name, value)
    return command


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


def negative_target():
    return B3StartupSyncTarget(
        roll_deg=-15.0,
        pitch_deg=-2.0,
        yaw_deg=315.0,
        altitude_m=3000.0,
        airspeed_mps=69.0,
    )


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

    def test_two_fresh_valid_post_release_rows_create_scoring_start(self):
        sync, raw_valid = self.released_sync_with_offset(4.0)
        real1 = make_real_state(target(), roll_deg=12.0, pitch_deg=-1.0, timestamp_s=0.000001)
        real2 = make_real_state(target(), roll_deg=12.01, pitch_deg=-1.01, timestamp_s=0.016667)

        out1 = sync.process_post_release_state(real1, raw_decoded_command=raw_valid, channel_map=CHANNEL_MAP)
        self.assertIsNone(sync.scoring_start_record)
        out2 = sync.process_post_release_state(real2, raw_decoded_command=raw_valid, channel_map=CHANNEL_MAP)

        self.assertIsNotNone(sync.scoring_start_record)
        self.assertAlmostEqual(sync.scoring_start_record["simulation_timestamp"], out2.timestamp_s, places=9)
        self.assertTrue(sync.scoring_eligible(out2, raw_valid, CHANNEL_MAP))
        self.assertFalse(sync.scoring_eligible(out1, raw_valid, CHANNEL_MAP))

    def test_scoring_start_timestamp_is_logged_on_same_qualifying_row(self):
        sync, raw_valid = self.released_sync_with_offset(4.0)
        first = make_real_state(target(), roll_deg=12.0, pitch_deg=-1.0, timestamp_s=0.000001)
        second = make_real_state(target(), roll_deg=12.01, pitch_deg=-1.01, timestamp_s=0.016667)
        sync.process_post_release_state(first, raw_decoded_command=raw_valid, channel_map=CHANNEL_MAP)

        out = sync.process_post_release_state(second, raw_decoded_command=raw_valid, channel_map=CHANNEL_MAP)
        row = make_log_row(
            request_count=126,
            source=("127.0.0.1", 50000),
            packet_valid=True,
            started=1.0,
            state=out,
            reply_bytes=100,
            actuator_command=raw_valid,
            control_packet=ControlPacket(SERVO16_MAGIC, 60, 124, tuple(VALID_PWM)),
            startup_sync=sync,
        )

        self.assertNotEqual(row["scoring_start_timestamp"], "")
        self.assertEqual(row["scoring_start_timestamp"], f"{out.timestamp_s:.9f}")

    def test_reused_first_row_then_two_fresh_rows_scores_after_two_fresh(self):
        sync, raw_valid = self.released_sync_with_offset(4.0)
        reused = make_real_state(target(), roll_deg=12.0, pitch_deg=-1.0, timestamp_s=0.000001, reused=True)
        fresh1 = make_real_state(target(), roll_deg=12.01, pitch_deg=-1.01, timestamp_s=0.016667)
        fresh2 = make_real_state(target(), roll_deg=12.02, pitch_deg=-1.02, timestamp_s=0.033333)

        sync.process_post_release_state(reused, raw_decoded_command=raw_valid, channel_map=CHANNEL_MAP)
        self.assertIsNone(sync.scoring_start_record)
        sync.process_post_release_state(fresh1, raw_decoded_command=raw_valid, channel_map=CHANNEL_MAP)
        self.assertIsNone(sync.scoring_start_record)
        out = sync.process_post_release_state(fresh2, raw_decoded_command=raw_valid, channel_map=CHANNEL_MAP)

        self.assertAlmostEqual(sync.scoring_start_record["simulation_timestamp"], out.timestamp_s, places=9)

    def test_stale_command_prevents_scoring(self):
        sync, _ = self.released_sync_with_offset(4.0)
        raw_stale = command_with(stale=True)
        for timestamp in (0.000001, 0.016667):
            state = make_real_state(target(), roll_deg=12.0, pitch_deg=-1.0, timestamp_s=timestamp)
            sync.process_post_release_state(state, raw_decoded_command=raw_stale, channel_map=CHANNEL_MAP)
        self.assertIsNone(sync.scoring_start_record)

    def test_roll_or_pitch_outside_window_prevents_scoring(self):
        for second_roll_deg, second_pitch_deg in ((15.1, -1.0), (12.0, 2.1)):
            sync, raw_valid = self.released_sync_with_offset(4.0)
            first = make_real_state(target(), roll_deg=12.0, pitch_deg=-1.0, timestamp_s=0.000001)
            second = make_real_state(
                target(),
                roll_deg=second_roll_deg,
                pitch_deg=second_pitch_deg,
                timestamp_s=0.016667,
            )
            sync.process_post_release_state(first, raw_decoded_command=raw_valid, channel_map=CHANNEL_MAP)
            sync.process_post_release_state(second, raw_decoded_command=raw_valid, channel_map=CHANNEL_MAP)
            self.assertIsNone(sync.scoring_start_record)

    def test_negative_roll_target_scores_like_positive_target(self):
        tgt = negative_target()
        sync = B3StartupSync(tgt)
        sync._last_hold_timestamp = 4.0
        raw_valid = decoded_command(VALID_PWM)
        sync.maybe_release(raw_valid, CHANNEL_MAP, host_time=10.0, pwm=raw_valid.pwm)

        first = make_real_state(tgt, roll_deg=-12.0, pitch_deg=-1.0, timestamp_s=0.000001)
        second = make_real_state(tgt, roll_deg=-12.01, pitch_deg=-1.01, timestamp_s=0.016667)
        out = sync.process_post_release_state(first, raw_decoded_command=raw_valid, channel_map=CHANNEL_MAP)
        self.assertAlmostEqual(math.degrees(out.roll_rad), -15.0, places=6)
        out = sync.process_post_release_state(second, raw_decoded_command=raw_valid, channel_map=CHANNEL_MAP)

        self.assertIsNotNone(sync.scoring_start_record)
        self.assertAlmostEqual(sync.scoring_start_record["simulation_timestamp"], out.timestamp_s, places=9)

    def test_scoring_not_eligible_if_rates_too_high(self):
        sync = B3StartupSync(target())
        raw_valid = decoded_command(VALID_PWM)
        sync.maybe_release(raw_valid, CHANNEL_MAP, host_time=1.0, pwm=raw_valid.pwm)
        real1 = make_real_state(target(), roll_deg=15.0, pitch_deg=-2.0, timestamp_s=1.0, p=0.2, q=0.0, r=0.0)
        sync.process_post_release_state(real1, raw_decoded_command=raw_valid, channel_map=CHANNEL_MAP)
        real2 = make_real_state(target(), roll_deg=15.0, pitch_deg=-2.0, timestamp_s=1.05, p=0.2, q=0.0, r=0.0)
        out2 = sync.process_post_release_state(real2, raw_decoded_command=raw_valid, channel_map=CHANNEL_MAP)
        self.assertIsNone(sync.scoring_start_record)
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

    def collect_startup_scoring_debug(self, sync, rows, limit, raw_command=None):
        logged = 0
        diagnostics = []
        raw_command = decoded_command(VALID_PWM) if raw_command is None else raw_command
        for request_count, row in enumerate(rows, start=1):
            scoring_debug = (
                {}
                if logged < limit and sync.startup_scoring_debug_row_ready(row)
                else None
            )
            sync.process_post_release_state(row, raw_decoded_command=raw_command, channel_map=CHANNEL_MAP,
                                            scoring_debug=scoring_debug)
            if scoring_debug is not None:
                logged += 1
                diagnostics.append((request_count, dict(scoring_debug)))
        return diagnostics

    def test_debug_count_10_emits_exactly_10_fresh_rows(self):
        sync, raw_valid = self.released_sync_with_offset(4.0)
        rows = [
            make_real_state(target(), roll_deg=12.0 + i * 0.01, pitch_deg=-1.0, timestamp_s=i * 0.016667)
            for i in range(15)
        ]

        diagnostics = self.collect_startup_scoring_debug(sync, rows, limit=10, raw_command=raw_valid)

        self.assertEqual(len(diagnostics), 10)
        self.assertEqual(diagnostics[-1][1]["fresh_rows_since_release"], 10)

    def test_debug_rows_continue_after_scoring_created_on_row_2(self):
        sync, raw_valid = self.released_sync_with_offset(4.0)
        rows = [
            make_real_state(target(), roll_deg=12.0 + i * 0.01, pitch_deg=-1.0, timestamp_s=i * 0.016667)
            for i in range(10)
        ]

        diagnostics = self.collect_startup_scoring_debug(sync, rows, limit=10, raw_command=raw_valid)

        self.assertEqual(len(diagnostics), 10)
        self.assertEqual(diagnostics[1][1]["scoring_start_before"], None)
        self.assertIsNotNone(diagnostics[1][1]["scoring_start_after"])
        self.assertIsNotNone(diagnostics[2][1]["scoring_start_before"])
        self.assertIsNotNone(diagnostics[9][1]["scoring_start_after"])

    def test_debug_rows_continue_after_rates_leave_bounds(self):
        sync, raw_valid = self.released_sync_with_offset(4.0)
        rows = []
        for i in range(10):
            q = 0.01 if i < 6 else 0.25
            rows.append(
                make_real_state(target(), roll_deg=12.0, pitch_deg=-1.0, timestamp_s=i * 0.016667, q=q)
            )

        diagnostics = self.collect_startup_scoring_debug(sync, rows, limit=10, raw_command=raw_valid)

        self.assertEqual(len(diagnostics), 10)
        self.assertTrue(diagnostics[5][1]["rates_in_bounds"])
        self.assertFalse(diagnostics[6][1]["rates_in_bounds"])
        self.assertFalse(diagnostics[9][1]["bounds_ok"])

    def test_debug_reused_rows_do_not_consume_count(self):
        sync, raw_valid = self.released_sync_with_offset(4.0)
        rows = [
            make_real_state(target(), roll_deg=12.0, pitch_deg=-1.0, timestamp_s=0.0, reused=True),
            make_real_state(target(), roll_deg=12.0, pitch_deg=-1.0, timestamp_s=0.016667),
            make_real_state(target(), roll_deg=12.0, pitch_deg=-1.0, timestamp_s=0.033334, reused=True),
            make_real_state(target(), roll_deg=12.0, pitch_deg=-1.0, timestamp_s=0.050001),
            make_real_state(target(), roll_deg=12.0, pitch_deg=-1.0, timestamp_s=0.066668),
        ]

        diagnostics = self.collect_startup_scoring_debug(sync, rows, limit=3, raw_command=raw_valid)

        self.assertEqual([request for request, _ in diagnostics], [2, 4, 5])
        self.assertEqual(diagnostics[-1][1]["fresh_rows_since_release"], 3)

    def test_debug_held_rows_do_not_consume_count(self):
        sync = B3StartupSync(target())
        held = sync.held_reply_state(host_time=1.0)
        self.assertFalse(sync.startup_scoring_debug_row_ready(held))

        raw_valid = decoded_command(VALID_PWM)
        sync.maybe_release(raw_valid, CHANNEL_MAP, host_time=1.1, pwm=raw_valid.pwm)
        fresh = make_real_state(target(), roll_deg=12.0, pitch_deg=-1.0, timestamp_s=0.0)
        diagnostics = self.collect_startup_scoring_debug(sync, [fresh], limit=1, raw_command=raw_valid)

        self.assertEqual(len(diagnostics), 1)
        self.assertEqual(diagnostics[0][1]["fresh_rows_since_release"], 1)

    def test_debug_count_0_emits_nothing(self):
        sync, raw_valid = self.released_sync_with_offset(4.0)
        rows = [
            make_real_state(target(), roll_deg=12.0 + i * 0.01, pitch_deg=-1.0, timestamp_s=i * 0.016667)
            for i in range(3)
        ]

        diagnostics = self.collect_startup_scoring_debug(sync, rows, limit=0, raw_command=raw_valid)

        self.assertEqual(diagnostics, [])

    def test_debug_print_contains_required_fields(self):
        sync, raw_valid = self.released_sync_with_offset(4.0)
        row = make_real_state(target(), roll_deg=12.0, pitch_deg=-1.0, timestamp_s=0.0)
        scoring_debug = {}
        sync.process_post_release_state(row, raw_decoded_command=raw_valid, channel_map=CHANNEL_MAP,
                                        scoring_debug=scoring_debug)

        output = StringIO()
        with redirect_stdout(output):
            print_b3_startup_scoring_debug(17, scoring_debug)

        line = output.getvalue()
        for field in (
            "request_count=17",
            "source_timestamp_s=",
            "timestamp_s=",
            "reused_source_row=",
            "fresh_rows_since_release=",
            "roll_deg=",
            "pitch_deg=",
            "p=",
            "q=",
            "r=",
            "fresh_rows_ready=",
            "roll_in_bounds=",
            "pitch_in_bounds=",
            "rates_in_bounds=",
            "pwm_valid=",
            "stale_zero=",
            "rato_zero=",
            "bounds_ok=",
            "scoring_before=",
            "scoring_after=",
        ):
            self.assertIn(field, line)

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
