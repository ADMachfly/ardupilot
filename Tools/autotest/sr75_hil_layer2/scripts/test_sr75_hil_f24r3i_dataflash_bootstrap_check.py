#!/usr/bin/env python3
"""HIL-F24-R3I regression tests: onboard-DataFlash EKF3-bootstrap check.

Covers the pure XKF1/IMU/MSG row analysis (find_first_xkf1_divergence,
nearest_imu_sample_at_or_before, summarize_dataflash_rows, reusing
sr75_hil_f24r3g_ekf3_bootstrap_trace's tilt_from_accel()/compare_
predicted_to_observed() against the same real-bench regression fixture),
the DFReader integration point (analyze_bootstrap_from_dataflash, tested
via a monkeypatched fake DFReader -- no real binary log needed), and an
end-to-end run of main() against a PTY fake Pixhawk that only ever
speaks the standard read-only LOG_REQUEST_LIST/LOG_REQUEST_DATA/
LOG_REQUEST_END protocol. No real hardware, no PARAM_SET/arm/mission/
RC-override/actuator/LOG_ERASE command.
"""
import os
import pty
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import sr75_hil_f24r3i_dataflash_bootstrap_check as dfcheck  # noqa: E402

from pymavlink import mavutil  # noqa: E402
from pymavlink.dialects.v20 import ardupilotmega as mavlink2  # noqa: E402

# The real, frozen values captured live in sr75_hil_f24r3a_mp_visual_
# 20260807T040401Z (see HIL_F24_R3G_ekf3_tilt_alignment_diagnosis.md).
REAL_FROZEN_ROLL_DEG = 7.365185081095156
REAL_FROZEN_PITCH_DEG = 7.547061919635194
REAL_DERIVED_ACCEL_XYZ = (131.34050648071846, -127.08250315063593, -983.1580283710025)


def _xkf1(time_us, roll_deg, pitch_deg, yaw_deg=0.0):
    return {"TimeUS": time_us, "roll_deg": roll_deg, "pitch_deg": pitch_deg, "yaw_deg": yaw_deg}


def _imu(time_us, xacc, yacc, zacc):
    return {"TimeUS": time_us, "xacc": xacc, "yacc": yacc, "zacc": zacc}


class TestFindFirstXkf1Divergence(unittest.TestCase):
    def test_finds_first_row_exceeding_threshold_in_time_order(self):
        rows = [
            _xkf1(3000000, 7.4, 7.5),
            _xkf1(1000000, 0.1, 0.1),
            _xkf1(2000000, 0.2, 0.15),
        ]
        result = dfcheck.find_first_xkf1_divergence(rows, threshold_deg=1.0)
        self.assertEqual(result["TimeUS"], 3000000)

    def test_pitch_alone_can_trigger(self):
        rows = [_xkf1(1000000, 0.0, 5.0)]
        self.assertIsNotNone(dfcheck.find_first_xkf1_divergence(rows, threshold_deg=1.0))

    def test_no_divergence_returns_none(self):
        rows = [_xkf1(t, 0.1, 0.1) for t in (1000000, 2000000, 3000000)]
        self.assertIsNone(dfcheck.find_first_xkf1_divergence(rows, threshold_deg=1.0))


class TestNearestImuSample(unittest.TestCase):
    def test_prefers_most_recent_sample_at_or_before_target(self):
        rows = [_imu(1000000, 1, 0, -1000), _imu(2000000, 2, 0, -1000), _imu(4000000, 4, 0, -1000)]
        result = dfcheck.nearest_imu_sample_at_or_before(rows, target_time_us=3000000)
        self.assertEqual(result["TimeUS"], 2000000)

    def test_falls_back_to_nearest_overall_if_none_before(self):
        rows = [_imu(5000000, 1, 0, -1000)]
        result = dfcheck.nearest_imu_sample_at_or_before(rows, target_time_us=1000000)
        self.assertEqual(result["TimeUS"], 5000000)

    def test_none_when_no_imu_rows(self):
        self.assertIsNone(dfcheck.nearest_imu_sample_at_or_before([], target_time_us=1000000))


class TestSummarizeDataflashRows(unittest.TestCase):
    def test_real_bench_regression_fixture_reproduces_and_passes(self):
        """Synthetic single-boot dataflash timeline shaped like the
        hypothesis: level for a while, then the transient sample lands
        and XKF1 freezes at the exact real captured value, then the
        bootstrap-complete MSG fires."""
        xkf1_rows = [
            _xkf1(500000, 0.0, 0.0),
            _xkf1(1050000, REAL_FROZEN_ROLL_DEG, REAL_FROZEN_PITCH_DEG, -34.0),
            _xkf1(1600000, REAL_FROZEN_ROLL_DEG, REAL_FROZEN_PITCH_DEG, -34.0),
        ]
        imu_rows = [
            _imu(500000, 0.0, 0.0, -1000.0),
            _imu(1000000, *REAL_DERIVED_ACCEL_XYZ),
            _imu(1500000, 0.0, 0.0, -1000.0),
        ]
        msg_rows = [{"TimeUS": 1200000, "text": "EKF3 IMU0 initialised"}]

        summary = dfcheck.summarize_dataflash_rows(xkf1_rows, imu_rows, msg_rows, divergence_threshold_deg=1.0, match_tolerance_deg=0.5)

        self.assertEqual(summary["first_divergence_time_us"], 1050000)
        self.assertEqual(summary["imu_sample_at_divergence"]["TimeUS"], 1000000)
        self.assertTrue(summary["reproduces_bootstrap_math"])
        self.assertLess(summary["roll_residual_deg"], 0.5)
        self.assertLess(summary["pitch_residual_deg"], 0.5)
        self.assertTrue(summary["ekf3_initialised_statustext_seen"])

    def test_no_divergence_reports_none_not_a_crash(self):
        summary = dfcheck.summarize_dataflash_rows(
            [_xkf1(1000000, 0.1, 0.1)], [_imu(1000000, 0.0, 0.0, -1000.0)], [],
            divergence_threshold_deg=1.0, match_tolerance_deg=0.5,
        )
        self.assertIsNone(summary["first_divergence_time_us"])
        self.assertIsNone(summary["reproduces_bootstrap_math"])
        self.assertFalse(summary["ekf3_initialised_statustext_seen"])

    def test_mismatched_accel_does_not_falsely_confirm(self):
        summary = dfcheck.summarize_dataflash_rows(
            [_xkf1(1000000, 30.0, 30.0)], [_imu(1000000, 0.0, 0.0, -1000.0)], [],
            divergence_threshold_deg=1.0, match_tolerance_deg=1.0,
        )
        self.assertFalse(summary["reproduces_bootstrap_math"])
        self.assertGreater(summary["roll_residual_deg"], 1.0)

    def test_earliest_bootstrap_statustext_used_if_several(self):
        msg_rows = [
            {"TimeUS": 5000000, "text": "EKF3 IMU0 initialised"},
            {"TimeUS": 1000000, "text": "EKF3 IMU0 initialised"},
        ]
        summary = dfcheck.summarize_dataflash_rows([], [], msg_rows, divergence_threshold_deg=1.0, match_tolerance_deg=1.0)
        self.assertEqual(summary["ekf3_initialised_statustext"]["TimeUS"], 1000000)


class TestPickLatestLog(unittest.TestCase):
    def test_picks_highest_id(self):
        entries = [{"id": 3, "size": 100}, {"id": 7, "size": 200}, {"id": 5, "size": 50}]
        self.assertEqual(dfcheck.pick_latest_log(entries)["id"], 7)

    def test_empty_returns_none(self):
        self.assertIsNone(dfcheck.pick_latest_log([]))


class FakeDFReaderMessage:
    def __init__(self, msg_type, **fields):
        self._type = msg_type
        for k, v in fields.items():
            setattr(self, k, v)

    def get_type(self):
        return self._type


class FakeDFReader:
    """Stand-in for pymavlink.DFReader.DFReader_binary -- recv_msg()
    yields a fixed, pre-scripted sequence then None, so analyze_
    bootstrap_from_dataflash's DFReader-integration logic (looping,
    filtering by core/instance, building rows) is exercised without
    needing a real crafted binary dataflash log."""

    def __init__(self, path):
        self.path = path
        self._messages = list(FakeDFReader.SCRIPT)
        self._i = 0

    def recv_msg(self):
        if self._i >= len(self._messages):
            return None
        m = self._messages[self._i]
        self._i += 1
        return m

    SCRIPT = [
        FakeDFReaderMessage("XKF1", C=0, Roll=0.0, Pitch=0.0, Yaw=0.0, TimeUS=500000),
        FakeDFReaderMessage("IMU", I=0, AccX=0.0, AccY=0.0, AccZ=-1000.0, TimeUS=500000),
        FakeDFReaderMessage("IMU", I=1, AccX=99.0, AccY=99.0, AccZ=99.0, TimeUS=1000000),  # other instance, must be ignored
        FakeDFReaderMessage(
            "IMU", I=0, AccX=REAL_DERIVED_ACCEL_XYZ[0], AccY=REAL_DERIVED_ACCEL_XYZ[1],
            AccZ=REAL_DERIVED_ACCEL_XYZ[2], TimeUS=1000000,
        ),
        FakeDFReaderMessage("XKF1", C=1, Roll=45.0, Pitch=45.0, Yaw=0.0, TimeUS=1050000),  # other core, must be ignored
        FakeDFReaderMessage("XKF1", C=0, Roll=REAL_FROZEN_ROLL_DEG, Pitch=REAL_FROZEN_PITCH_DEG, Yaw=-34.0, TimeUS=1050000),
        FakeDFReaderMessage("MSG", TimeUS=1200000, ID=0, Seq=0, Message="EKF3 IMU0 initialised"),
        FakeDFReaderMessage("PARM", TimeUS=1300000, Name="LOG_BITMASK", Value=65535.0),  # unrelated type, must be ignored
    ]


class TestAnalyzeBootstrapFromDataflash(unittest.TestCase):
    def test_filters_by_core_and_instance_and_reproduces_math(self):
        import pymavlink.DFReader as real_module
        original = real_module.DFReader_binary
        real_module.DFReader_binary = FakeDFReader
        try:
            summary = dfcheck.analyze_bootstrap_from_dataflash("unused.bin", ekf_core=0, imu_instance=0)
        finally:
            real_module.DFReader_binary = original
        self.assertEqual(summary["xkf1_row_count"], 2)  # core 0 only: level + frozen
        self.assertEqual(summary["imu_row_count"], 2)  # instance 0 only
        self.assertEqual(summary["msg_row_count"], 1)
        self.assertTrue(summary["ekf3_initialised_statustext_seen"])
        self.assertTrue(summary["reproduces_bootstrap_math"])


class FakeMavlinkLogPixhawk:
    """PTY fake Pixhawk: streams HEARTBEAT, replies to LOG_REQUEST_LIST
    with a single LOG_ENTRY (repeated, matching the established
    "repeat, don't one-shot" fix for the CUT's own wait_heartbeat()
    startup race -- see sr75_hil_f24r3g's test file), and replies to
    each LOG_REQUEST_DATA with the matching LOG_DATA chunk from a fixed
    byte buffer. Never replies to, or sends, anything else."""

    def __init__(self, log_id=3, payload=b"FAKE_DATAFLASH_LOG_BYTES_" * 5):
        self.master_fd, self.slave_fd = pty.openpty()
        self.device_path = os.ttyname(self.slave_fd)
        self._mav = mavutil.mavlink.MAVLink(None, srcSystem=1, srcComponent=1)
        self._stop = threading.Event()
        self._thread = None
        self.log_id = log_id
        self.payload = payload
        self.sent_message_types = []
        self._parser = mavutil.mavlink.MAVLink(None, srcSystem=255, srcComponent=0)

    def _send(self, msg):
        os.write(self.master_fd, msg.pack(self._mav))

    def _loop(self):
        def hb_loop():
            while not self._stop.is_set():
                self._send(mavlink2.MAVLink_heartbeat_message(
                    type=1, autopilot=3, base_mode=81, custom_mode=0, system_status=3, mavlink_version=3,
                ))
                time.sleep(0.1)
        hb_thread = threading.Thread(target=hb_loop, daemon=True)
        hb_thread.start()

        while not self._stop.is_set():
            try:
                chunk = os.read(self.master_fd, 4096)
            except OSError:
                break
            if not chunk:
                time.sleep(0.01)
                continue
            for i in range(len(chunk)):
                try:
                    msg = self._parser.parse_char(chunk[i:i + 1])
                except mavutil.mavlink.MAVError:
                    continue
                if msg is None:
                    continue
                self.sent_message_types.append(msg.get_type())
                if msg.get_type() == "LOG_REQUEST_LIST":
                    for _ in range(3):
                        self._send(mavlink2.MAVLink_log_entry_message(
                            id=self.log_id, num_logs=1, last_log_num=self.log_id,
                            time_utc=0, size=len(self.payload),
                        ))
                        time.sleep(0.02)
                elif msg.get_type() == "LOG_REQUEST_DATA":
                    ofs = msg.ofs
                    want = msg.count
                    chunk_data = self.payload[ofs:ofs + want]
                    if chunk_data:
                        padded = list(chunk_data) + [0] * (90 - len(chunk_data))
                        self._send(mavlink2.MAVLink_log_data_message(
                            id=msg.id, ofs=ofs, count=len(chunk_data), data=padded,
                        ))

        hb_thread.join(timeout=1.0)

    def start(self):
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        for fd in (self.master_fd, self.slave_fd):
            try:
                os.close(fd)
            except OSError:
                pass


class TestListAndDownloadLog(unittest.TestCase):
    def test_list_log_entries_and_download_roundtrip(self):
        fake = FakeMavlinkLogPixhawk(log_id=5, payload=b"HELLO_DATAFLASH_LOG_" * 3)
        fake.start()
        try:
            master = mavutil.mavlink_connection(fake.device_path, baud=115200)
            self.assertIsNotNone(master.wait_heartbeat(timeout=5))
            entries = dfcheck.list_log_entries(master, timeout_s=3.0)
            self.assertEqual(len(entries), 1)
            self.assertEqual(entries[0]["id"], 5)
            self.assertEqual(entries[0]["size"], len(fake.payload))

            data = dfcheck.download_log(master, 5, len(fake.payload), chunk_size=20, timeout_s=2.0, retries=3)
            self.assertEqual(data, fake.payload)
            master.close()
        finally:
            fake.stop()

    def test_zero_logs_sentinel_is_treated_as_no_logs(self):
        """AP_Logger's real reply when nothing is stored (confirmed live
        against this bench's Pixhawk: num_logs=0, not silence) must be
        treated the same as "no logs", not as one real log of size 0."""
        class SentinelPixhawk(FakeMavlinkLogPixhawk):
            def _loop(self):
                def hb_loop():
                    while not self._stop.is_set():
                        self._send(mavlink2.MAVLink_heartbeat_message(
                            type=1, autopilot=3, base_mode=81, custom_mode=0, system_status=3, mavlink_version=3,
                        ))
                        time.sleep(0.1)
                hb_thread = threading.Thread(target=hb_loop, daemon=True)
                hb_thread.start()
                while not self._stop.is_set():
                    try:
                        chunk = os.read(self.master_fd, 4096)
                    except OSError:
                        break
                    if not chunk:
                        time.sleep(0.01)
                        continue
                    for i in range(len(chunk)):
                        try:
                            msg = self._parser.parse_char(chunk[i:i + 1])
                        except mavutil.mavlink.MAVError:
                            continue
                        if msg is not None and msg.get_type() == "LOG_REQUEST_LIST":
                            self._send(mavlink2.MAVLink_log_entry_message(
                                id=0, num_logs=0, last_log_num=0, time_utc=0, size=0,
                            ))
                hb_thread.join(timeout=1.0)

        fake = SentinelPixhawk()
        fake.start()
        try:
            master = mavutil.mavlink_connection(fake.device_path, baud=115200)
            self.assertIsNotNone(master.wait_heartbeat(timeout=5))
            entries = dfcheck.list_log_entries(master, timeout_s=2.0)
            self.assertEqual(entries, [])
            master.close()
        finally:
            fake.stop()

    def test_never_sends_forbidden_commands(self):
        fake = FakeMavlinkLogPixhawk()
        fake.start()
        try:
            master = mavutil.mavlink_connection(fake.device_path, baud=115200)
            master.wait_heartbeat(timeout=5)
            dfcheck.list_log_entries(master, timeout_s=2.0)
            dfcheck.download_log(master, fake.log_id, len(fake.payload), chunk_size=20, timeout_s=2.0, retries=2)
            master.mav.log_request_end_send(master.target_system, master.target_component)
            master.close()
            time.sleep(0.2)
        finally:
            fake.stop()
        forbidden = {"PARAM_SET", "COMMAND_LONG", "SET_MODE", "MISSION_ITEM", "MISSION_ITEM_INT", "RC_CHANNELS_OVERRIDE", "LOG_ERASE"}
        self.assertFalse(forbidden.intersection(set(fake.sent_message_types)))
        self.assertIn("LOG_REQUEST_LIST", fake.sent_message_types)
        self.assertIn("LOG_REQUEST_DATA", fake.sent_message_types)


class TestMainEndToEnd(unittest.TestCase):
    def test_no_logs_found_returns_1(self):
        """A fake Pixhawk that never replies to LOG_REQUEST_LIST (i.e. no
        SD card / no onboard logs) must be reported as insufficient, not
        crash or hang."""
        class NoLogsPixhawk(FakeMavlinkLogPixhawk):
            def _loop(self):
                def hb_loop():
                    while not self._stop.is_set():
                        self._send(mavlink2.MAVLink_heartbeat_message(
                            type=1, autopilot=3, base_mode=81, custom_mode=0, system_status=3, mavlink_version=3,
                        ))
                        time.sleep(0.1)
                hb_thread = threading.Thread(target=hb_loop, daemon=True)
                hb_thread.start()
                while not self._stop.is_set():
                    time.sleep(0.05)
                hb_thread.join(timeout=1.0)

        fake = NoLogsPixhawk()
        fake.start()
        tmpdir = Path(tempfile.mkdtemp(prefix="hil_f24r3i_nologs_"))
        try:
            cmd = [
                sys.executable, str(HERE / "sr75_hil_f24r3i_dataflash_bootstrap_check.py"),
                "--pixhawk", fake.device_path, "--baud", "115200",
                "--heartbeat-timeout-s", "5.0", "--list-timeout-s", "1.5",
                "--output-bin", str(tmpdir / "out.bin"),
            ]
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
            self.assertEqual(result.returncode, 1)
            self.assertIn("NO onboard logs found", result.stdout)
            self.assertFalse((tmpdir / "out.bin").exists())
        finally:
            fake.stop()

    def test_device_never_reachable_aborts_cleanly(self):
        tmpdir = Path(tempfile.mkdtemp(prefix="hil_f24r3i_nodev_"))
        cmd = [
            sys.executable, str(HERE / "sr75_hil_f24r3i_dataflash_bootstrap_check.py"),
            "--pixhawk", "/dev/definitely_not_a_real_device_xyz", "--baud", "115200",
            "--heartbeat-timeout-s", "0.5",
            "--output-bin", str(tmpdir / "out.bin"),
        ]
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 2)
        self.assertIn("no heartbeat", result.stdout)


if __name__ == "__main__":
    unittest.main()
