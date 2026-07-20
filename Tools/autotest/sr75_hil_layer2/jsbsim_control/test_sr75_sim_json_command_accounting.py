#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Responder request/accounting test for SR-75 JSBSim command output."""

import csv
import json
import os
import socket
import struct
import subprocess
import sys
import tempfile
import time
import unittest


THIS_DIR = os.path.dirname(os.path.abspath(__file__))
SR75_DIR = os.path.dirname(THIS_DIR)
SIM_JSON_DIR = os.path.join(SR75_DIR, "sim_json")
RESPONDER = os.path.join(SIM_JSON_DIR, "sr75_sim_json_responder.py")
if SIM_JSON_DIR not in sys.path:
    sys.path.insert(0, SIM_JSON_DIR)

from sr75_sim_json_actuator_bridge import neutral_pwm_values  # noqa: E402


SERVO16_MAGIC = 18458
SERVO16_STRUCT = struct.Struct("<HHI16H")


def free_udp_port() -> int:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])
    finally:
        sock.close()


def build_packet(frame_rate: int, frame_count: int) -> bytes:
    return SERVO16_STRUCT.pack(SERVO16_MAGIC, frame_rate, frame_count, *neutral_pwm_values())


def build_override_packet(frame_rate: int, frame_count: int, overrides) -> bytes:
    pwm = neutral_pwm_values()
    for channel, value in overrides.items():
        pwm[channel - 1] = value
    return SERVO16_STRUCT.pack(SERVO16_MAGIC, frame_rate, frame_count, *pwm)


class TestSR75SIMJSONCommandAccounting(unittest.TestCase):
    def test_100_packet_accounting(self):
        listen_port = free_udp_port()
        command_sink = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        log_path = tempfile.NamedTemporaryFile(prefix="sr75_accounting_", suffix=".csv", delete=False).name
        command_packets = []
        try:
            command_sink.bind(("127.0.0.1", 0))
            command_sink.settimeout(0.05)
            command_port = int(command_sink.getsockname()[1])
            client.settimeout(1.0)

            responder = subprocess.Popen(
                [
                    sys.executable,
                    "-u",
                    RESPONDER,
                    "--listen-host",
                    "127.0.0.1",
                    "--listen-port",
                    str(listen_port),
                    "--mock-state",
                    "--log-csv",
                    log_path,
                    "--jsbsim-command-target",
                    f"udp:127.0.0.1:{command_port}",
                    "--actuator-timeout-ms",
                    "1000",
                ],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
            )
            try:
                self._wait_for_responder(responder)
                replies = []
                for frame_count in range(1, 101):
                    client.sendto(build_packet(100, frame_count), ("127.0.0.1", listen_port))
                    data, _ = client.recvfrom(4096)
                    replies.append(json.loads(data.decode("utf-8")))

                deadline = time.monotonic() + 2.0
                while len(command_packets) < 100 and time.monotonic() < deadline:
                    try:
                        data, _ = command_sink.recvfrom(2048)
                    except socket.timeout:
                        continue
                    command_packets.append(data.decode("ascii"))
            finally:
                responder.terminate()
                try:
                    responder.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    responder.kill()
                    responder.wait(timeout=2.0)
                if responder.stdout is not None:
                    responder.stdout.close()

            with open(log_path, "r", newline="", encoding="utf-8") as csv_file:
                rows = list(csv.DictReader(csv_file))

            self.assertEqual(len(replies), 100)
            self.assertEqual(len(command_packets), 100)
            self.assertEqual(len(rows), 100)
            self.assertEqual([int(row["request_count"]) for row in rows], list(range(1, 101)))
            self.assertEqual([int(row["frame_count"]) for row in rows], list(range(1, 101)))
            self.assertEqual([int(row["frame_rate"]) for row in rows], [100] * 100)
            self.assertTrue(all(int(row["packet_valid"]) == 1 for row in rows))
            self.assertTrue(all(int(row["reply_bytes"]) > 0 for row in rows))

            timestamps = [float(packet.split(",", 1)[0]) for packet in command_packets]
            self.assertEqual(sorted(timestamps), timestamps)
            for packet in command_packets:
                values = packet.split(",")
                self.assertEqual(len(values), 6)
                self.assertEqual([float(value) for value in values[1:]], [0.0, 0.0, 0.0, 0.0, 0.0])
        finally:
            command_sink.close()
            client.close()
            try:
                os.unlink(log_path)
            except FileNotFoundError:
                pass

    def test_stale_command_sends_one_neutral_packet(self):
        listen_port = free_udp_port()
        command_sink = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        log_path = tempfile.NamedTemporaryFile(prefix="sr75_stale_", suffix=".csv", delete=False).name
        command_packets = []
        try:
            command_sink.bind(("127.0.0.1", 0))
            command_sink.settimeout(0.05)
            command_port = int(command_sink.getsockname()[1])
            client.settimeout(1.0)

            responder = subprocess.Popen(
                [
                    sys.executable,
                    "-u",
                    RESPONDER,
                    "--listen-host",
                    "127.0.0.1",
                    "--listen-port",
                    str(listen_port),
                    "--mock-state",
                    "--log-csv",
                    log_path,
                    "--jsbsim-command-target",
                    f"udp:127.0.0.1:{command_port}",
                    "--actuator-timeout-ms",
                    "100",
                ],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
            )
            try:
                self._wait_for_responder(responder)
                client.sendto(
                    build_override_packet(50, 1, {1: 2000, 3: 2000, 4: 2000, 7: 2000}),
                    ("127.0.0.1", listen_port),
                )
                client.recvfrom(4096)

                deadline = time.monotonic() + 0.7
                while time.monotonic() < deadline:
                    try:
                        data, _ = command_sink.recvfrom(2048)
                    except socket.timeout:
                        continue
                    command_packets.append([float(value) for value in data.decode("ascii").split(",")])
            finally:
                responder.terminate()
                try:
                    responder.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    responder.kill()
                    responder.wait(timeout=2.0)
                if responder.stdout is not None:
                    responder.stdout.close()

            active_indexes = [
                index for index, values in enumerate(command_packets)
                if values[1:] == [0.5, 0.5, 1.0, 1.0, 1.0]
            ]
            self.assertEqual(len(active_indexes), 1)
            packets_after_active = command_packets[active_indexes[0] + 1:]
            self.assertEqual(len(packets_after_active), 1)
            self.assertEqual(packets_after_active[0][1:], [0.0, 0.0, 0.0, 0.0, 0.0])
        finally:
            command_sink.close()
            client.close()
            try:
                os.unlink(log_path)
            except FileNotFoundError:
                pass

    def _wait_for_responder(self, responder: subprocess.Popen) -> None:
        assert responder.stdout is not None
        deadline = time.monotonic() + 5.0
        lines = []
        while time.monotonic() < deadline:
            line = responder.stdout.readline()
            if line:
                lines.append(line)
                if line.startswith("Listening on "):
                    return
            if responder.poll() is not None:
                self.fail("responder exited before listening:\n" + "".join(lines))
        self.fail("timed out waiting for responder:\n" + "".join(lines))


if __name__ == "__main__":
    unittest.main(verbosity=2)
