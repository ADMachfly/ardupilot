#!/usr/bin/env python3
"""HIL-F24-F unit tests for sr75_hil_f24f_hardware_acceptance_analyzer.py.

No hardware, no live MAVLink connection. Uses only synthetic records (CSV
fixtures written to a temp file, and plain fake message objects for the
MAVLink-message-fusion path) so every check is exercised without needing a
real bench log to exist.
"""
import math
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import sr75_hil_f24f_hardware_acceptance_analyzer as analyzer  # noqa: E402


def perfect_record(
    t_s, commanded_state, armed=False, mode="MANUAL",
    ekf_flags=analyzer.EKF_REQUIRED_HEALTHY_MASK, cpu_load_pct=20.0,
    ch7_pwm=0, ch8_pwm=0, boot_time_s=None,
):
    """Build a telemetry record that exactly matches a commanded SimState
    -- the "everything is perfect" fixture each violation test starts from
    and then breaks exactly one field of."""
    return {
        "t_s": t_s,
        "ahrs2_roll_deg": math.degrees(commanded_state.roll_rad),
        "ahrs2_pitch_deg": math.degrees(commanded_state.pitch_rad),
        "ahrs2_yaw_deg": math.degrees(commanded_state.yaw_rad),
        "gps_lat_deg": commanded_state.latitude_deg,
        "gps_lon_deg": commanded_state.longitude_deg,
        "gps_alt_m": commanded_state.altitude_m,
        "airspeed_mps": commanded_state.airspeed_mps,
        "ekf_flags": ekf_flags,
        "ekf_healthy": (ekf_flags & analyzer.EKF_REQUIRED_HEALTHY_MASK) == analyzer.EKF_REQUIRED_HEALTHY_MASK,
        "cpu_load_pct": cpu_load_pct,
        "armed": armed,
        "mode": mode,
        "ch7_pwm": ch7_pwm,
        "ch8_pwm": ch8_pwm,
        "boot_time_s": t_s if boot_time_s is None else boot_time_s,
    }


class TestHaversine(unittest.TestCase):
    def test_zero_distance_for_identical_points(self):
        self.assertAlmostEqual(analyzer.haversine_m(32.5, 74.3, 32.5, 74.3), 0.0, places=6)

    def test_one_degree_latitude_is_roughly_111km(self):
        d = analyzer.haversine_m(0.0, 0.0, 1.0, 0.0)
        self.assertAlmostEqual(d, 111195.0, delta=200.0)


class TestLoadCsvRecords(unittest.TestCase):
    def test_round_trips_a_written_csv(self):
        import csv
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "log.csv"
            row = {field: "0" for field in analyzer.CSV_FIELDS}
            row.update({"mode": "MANUAL", "t_s": "1.5", "ahrs2_roll_deg": "2.0"})
            with open(path, "w", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=analyzer.CSV_FIELDS)
                writer.writeheader()
                writer.writerow(row)
            records = analyzer.load_csv_records(str(path))
        self.assertEqual(len(records), 1)
        self.assertEqual(records[0]["t_s"], 1.5)
        self.assertEqual(records[0]["ahrs2_roll_deg"], 2.0)
        self.assertEqual(records[0]["mode"], "MANUAL")
        self.assertFalse(records[0]["armed"])

    def test_missing_field_raises(self):
        import csv
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "log.csv"
            fields = [f for f in analyzer.CSV_FIELDS if f != "cpu_load_pct"]
            with open(path, "w", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=fields)
                writer.writeheader()
                writer.writerow({f: "0" for f in fields})
            with self.assertRaises(analyzer.AnalyzerError):
                analyzer.load_csv_records(str(path))


class FakeMavMessage:
    def __init__(self, mtype, **fields):
        self._mtype = mtype
        for k, v in fields.items():
            setattr(self, k, v)

    def get_type(self):
        return self._mtype


class TestRecordsFromMavlinkMessages(unittest.TestCase):
    def test_fuses_latest_values_per_heartbeat(self):
        from pymavlink import mavutil

        messages = [
            FakeMavMessage("AHRS2", roll=0.1, pitch=-0.05, yaw=1.0),
            FakeMavMessage("GLOBAL_POSITION_INT", lat=325378147, lon=743661871, alt=3000000),
            FakeMavMessage("VFR_HUD", airspeed=30.0),
            FakeMavMessage("SYS_STATUS", load=250),
            FakeMavMessage(
                "EKF_STATUS_REPORT", flags=analyzer.EKF_REQUIRED_HEALTHY_MASK,
            ),
            FakeMavMessage("SERVO_OUTPUT_RAW", servo7_raw=0, servo8_raw=0),
            FakeMavMessage(
                "HEARTBEAT", base_mode=0, custom_mode=0, type=1,
                autopilot=mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA, time_boot_ms=1000,
            ),
        ]
        records = analyzer.records_from_mavlink_messages(messages)
        self.assertEqual(len(records), 1)
        record = records[0]
        self.assertAlmostEqual(record["ahrs2_roll_deg"], math.degrees(0.1), places=4)
        self.assertAlmostEqual(record["gps_lat_deg"], 32.5378147, places=5)
        self.assertEqual(record["airspeed_mps"], 30.0)
        self.assertEqual(record["cpu_load_pct"], 25.0)
        self.assertTrue(record["ekf_healthy"])
        self.assertFalse(record["armed"])
        self.assertEqual(record["boot_time_s"], 1.0)

    def test_armed_flag_decoded_from_base_mode(self):
        from pymavlink import mavutil

        armed_mask = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        messages = [
            FakeMavMessage(
                "HEARTBEAT", base_mode=armed_mask, custom_mode=0, type=1,
                autopilot=mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA, time_boot_ms=0,
            ),
        ]
        records = analyzer.records_from_mavlink_messages(messages)
        self.assertTrue(records[0]["armed"])

    def test_unhealthy_ekf_flags_detected(self):
        from pymavlink import mavutil

        messages = [
            FakeMavMessage("EKF_STATUS_REPORT", flags=0x01),  # missing required bits
            FakeMavMessage(
                "HEARTBEAT", base_mode=0, custom_mode=0, type=1,
                autopilot=mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA, time_boot_ms=0,
            ),
        ]
        records = analyzer.records_from_mavlink_messages(messages)
        self.assertFalse(records[0]["ekf_healthy"])


class TestBuildPairedSamples(unittest.TestCase):
    def test_raises_when_telemetry_span_much_shorter_than_commanded(self):
        import sr75_sim_json_test_profiles as profiles

        rows = profiles.PROFILES_F24F["roll_sweep_multileg"](duration_s=12.0, rate_hz=50.0)
        short_records = [perfect_record(s.timestamp_s, s) for s in rows if s.timestamp_s < 2.0]
        with self.assertRaises(analyzer.AnalyzerError):
            analyzer.build_paired_samples(rows, short_records)

    def test_matching_span_does_not_raise(self):
        import sr75_sim_json_test_profiles as profiles

        rows = profiles.PROFILES_F24F["static_level"](duration_s=2.0, rate_hz=50.0)
        records = [perfect_record(s.timestamp_s, s) for s in rows]
        analyzer.build_paired_samples(rows, records)  # must not raise


class TestEvaluateAcceptance(unittest.TestCase):
    def _perfect_paired(self, profile_name="static_level", duration_s=2.0, rate_hz=50.0):
        import sr75_sim_json_test_profiles as profiles

        rows = profiles.PROFILES_F24F[profile_name](duration_s=duration_s, rate_hz=rate_hz)
        records = [perfect_record(s.timestamp_s, s) for s in rows]
        paired = analyzer.build_paired_samples(rows, records)
        return rows, records, paired

    def test_perfect_match_passes_static_thresholds(self):
        _, records, paired = self._perfect_paired()
        ok, report = analyzer.evaluate_acceptance(paired, records, analyzer.THRESHOLDS_STATIC)
        self.assertTrue(ok, report["violations"])
        self.assertEqual(report["violations"], [])

    def test_attitude_error_detected(self):
        rows, records, paired = self._perfect_paired()
        records[5]["ahrs2_roll_deg"] += 5.0
        paired = analyzer.build_paired_samples(rows, records)
        ok, report = analyzer.evaluate_acceptance(paired, records, analyzer.THRESHOLDS_STATIC)
        self.assertFalse(ok)
        self.assertTrue(any("attitude error" in v for v in report["violations"]))

    def test_position_error_detected(self):
        rows, records, paired = self._perfect_paired()
        records[5]["gps_lat_deg"] += 0.01
        paired = analyzer.build_paired_samples(rows, records)
        ok, report = analyzer.evaluate_acceptance(paired, records, analyzer.THRESHOLDS_STATIC)
        self.assertFalse(ok)
        self.assertTrue(any("position error" in v for v in report["violations"]))

    def test_altitude_error_detected(self):
        rows, records, paired = self._perfect_paired()
        records[5]["gps_alt_m"] += 10.0
        paired = analyzer.build_paired_samples(rows, records)
        ok, report = analyzer.evaluate_acceptance(paired, records, analyzer.THRESHOLDS_STATIC)
        self.assertFalse(ok)
        self.assertTrue(any("altitude error" in v for v in report["violations"]))

    def test_airspeed_error_detected(self):
        rows, records, paired = self._perfect_paired()
        records[5]["airspeed_mps"] += 5.0
        paired = analyzer.build_paired_samples(rows, records)
        ok, report = analyzer.evaluate_acceptance(paired, records, analyzer.THRESHOLDS_STATIC)
        self.assertFalse(ok)
        self.assertTrue(any("airspeed error" in v for v in report["violations"]))

    def test_ekf_unhealthy_detected(self):
        rows, records, paired = self._perfect_paired()
        records[5]["ekf_healthy"] = False
        ok, report = analyzer.evaluate_acceptance(paired, records, analyzer.THRESHOLDS_STATIC)
        self.assertFalse(ok)
        self.assertTrue(any("EKF unhealthy" in v for v in report["violations"]))

    def test_cpu_load_detected(self):
        rows, records, paired = self._perfect_paired()
        records[5]["cpu_load_pct"] = 99.0
        ok, report = analyzer.evaluate_acceptance(paired, records, analyzer.THRESHOLDS_STATIC)
        self.assertFalse(ok)
        self.assertTrue(any("CPU load" in v for v in report["violations"]))

    def test_stale_gap_detected(self):
        rows, records, paired = self._perfect_paired()
        records[5]["t_s"] = records[4]["t_s"] + 2.0
        ok, report = analyzer.evaluate_acceptance(paired, records, analyzer.THRESHOLDS_STATIC)
        self.assertFalse(ok)
        self.assertTrue(any("stale gap" in v for v in report["violations"]))

    def test_reboot_detected(self):
        rows, records, paired = self._perfect_paired()
        records[5]["boot_time_s"] = 0.0
        ok, report = analyzer.evaluate_acceptance(paired, records, analyzer.THRESHOLDS_STATIC)
        self.assertFalse(ok)
        self.assertTrue(any("reboot detected" in v for v in report["violations"]))

    def test_armed_detected(self):
        rows, records, paired = self._perfect_paired()
        records[5]["armed"] = True
        ok, report = analyzer.evaluate_acceptance(paired, records, analyzer.THRESHOLDS_STATIC)
        self.assertFalse(ok)
        self.assertTrue(any("ARMED" in v for v in report["violations"]))

    def test_non_manual_mode_detected(self):
        rows, records, paired = self._perfect_paired()
        records[5]["mode"] = "AUTO"
        ok, report = analyzer.evaluate_acceptance(paired, records, analyzer.THRESHOLDS_STATIC)
        self.assertFalse(ok)
        self.assertTrue(any("non-MANUAL" in v for v in report["violations"]))

    def test_ch7_ch8_activity_detected(self):
        rows, records, paired = self._perfect_paired()
        records[5]["ch7_pwm"] = 1500
        ok, report = analyzer.evaluate_acceptance(paired, records, analyzer.THRESHOLDS_STATIC)
        self.assertFalse(ok)
        self.assertTrue(any("CH7/CH8" in v for v in report["violations"]))


class TestAnalyzeEndToEnd(unittest.TestCase):
    def test_analyze_perfect_sweep_passes(self):
        import sr75_sim_json_test_profiles as profiles

        # analyze() runs the profile with its own default duration/rate --
        # the fixture record span must match, or "nearest timestamp"
        # matching clamps to the fixture's last record and looks like a
        # (spurious) huge tracking error for commanded rows beyond it.
        rows = profiles.PROFILES_F24F["roll_sweep_multileg"]()
        records = [perfect_record(s.timestamp_s, s) for s in rows]
        ok, report = analyzer.analyze("roll_sweep_multileg", records)
        self.assertTrue(ok, report["violations"])

    def test_analyze_unknown_profile_raises(self):
        with self.assertRaises(analyzer.AnalyzerError):
            analyzer.analyze("not_a_real_profile", [])

    def test_category_override(self):
        import sr75_sim_json_test_profiles as profiles

        rows = profiles.PROFILES_F24F["static_level"]()
        records = [perfect_record(s.timestamp_s, s) for s in rows]
        # Break attitude just enough to fail the tight static threshold but
        # pass the looser combined-category threshold, proving the
        # override actually changes which thresholds are applied.
        records[3]["ahrs2_roll_deg"] += 1.5
        ok_static, _ = analyzer.analyze("static_level", records, category_override="static")
        ok_combined, _ = analyzer.analyze("static_level", records, category_override="combined")
        self.assertFalse(ok_static)
        self.assertTrue(ok_combined)


if __name__ == "__main__":
    unittest.main()
