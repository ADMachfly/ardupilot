#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Tests for the SR-75 JSBSim UDP command sink."""

import math
import socket
import unittest

from sr75_jsbsim_command_sink import (
    JSBSIM_INPUT_PROPERTIES,
    JSBSimActuatorCommand,
    JSBSimCommandError,
    UDPJSBSimCommandSink,
    parse_command_target,
)


class TestSR75JSBSimCommandSink(unittest.TestCase):
    def test_parse_command_target(self):
        self.assertEqual(parse_command_target("udp:127.0.0.1:5600"), ("127.0.0.1", 5600))
        self.assertEqual(parse_command_target("udp://localhost:5601"), ("localhost", 5601))
        with self.assertRaises(JSBSimCommandError):
            parse_command_target("127.0.0.1")
        with self.assertRaises(JSBSimCommandError):
            parse_command_target("udp:127.0.0.1:99999")

    def test_packet_order_matches_jsbsim_input_properties(self):
        self.assertEqual(
            JSBSIM_INPUT_PROPERTIES,
            (
                "fcs/elevator-cmd-norm",
                "fcs/aileron-cmd-norm",
                "fcs/rudder-cmd-norm",
                "fcs/turbojet-throttle-cmd-norm",
                "fcs/rato-throttle-cmd-norm",
                "propulsion/tank[2]/contents-lbs",
            ),
        )
        command = JSBSimActuatorCommand(1.25, 0.2, -0.3, 0.4, 0.5, 1.0)
        self.assertEqual(
            command.packet_text(),
            "1.250000000,0.200000000,-0.300000000,0.400000000,0.500000000,1.000000000,22.046226000",
        )

    def test_command_clamps_and_rejects_bad_values(self):
        command = JSBSimActuatorCommand(1.0, 2.0, -2.0, 0.0, 1.5, -0.5, 99.0).validated()
        self.assertEqual(command.elevator, 1.0)
        self.assertEqual(command.aileron, -1.0)
        self.assertEqual(command.turbojet_throttle, 1.0)
        self.assertEqual(command.rato_throttle, 0.0)
        self.assertEqual(command.dry_booster_weight_lbs, 22.046226)
        self.assertEqual(JSBSimActuatorCommand(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0).validated().dry_booster_weight_lbs, 0.0)
        with self.assertRaises(JSBSimCommandError):
            JSBSimActuatorCommand(math.nan, 0.0, 0.0, 0.0, 0.0, 0.0).validated()
        with self.assertRaises(JSBSimCommandError):
            JSBSimActuatorCommand(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, math.nan).validated()

    def test_dry_booster_tank_packet_values(self):
        attached = JSBSimActuatorCommand(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 22.046226)
        ejected = JSBSimActuatorCommand(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self.assertEqual(attached.packet_text().split(",")[-1], "22.046226000")
        self.assertEqual(ejected.packet_text().split(",")[-1], "0.000000000")

    def test_udp_send_and_stale_once(self):
        server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        server.bind(("127.0.0.1", 0))
        server.settimeout(1.0)
        host, port = server.getsockname()
        sink = UDPJSBSimCommandSink(f"udp:{host}:{port}")
        try:
            sink.send(JSBSimActuatorCommand(2.0, 0.1, 0.2, -0.3, 0.4, 0.0))
            data, _ = server.recvfrom(2048)
            self.assertEqual(
                data.decode("ascii"),
                "2.000000000,0.100000000,0.200000000,-0.300000000,0.400000000,0.000000000,22.046226000",
            )

            self.assertIsNotNone(sink.send_stale_once(timestamp_s=3.0, dry_booster_weight_lbs=0.0))
            data, _ = server.recvfrom(2048)
            self.assertEqual(
                data.decode("ascii"),
                "3.000000000,0.000000000,0.000000000,0.000000000,0.000000000,0.000000000,0.000000000",
            )
            self.assertIsNone(sink.send_stale_once(timestamp_s=4.0))
        finally:
            sink.close()
            server.close()

    def test_no_physical_output_interface(self):
        sink = UDPJSBSimCommandSink("udp:127.0.0.1:5600")
        try:
            self.assertFalse(hasattr(sink, "serial"))
            self.assertFalse(hasattr(sink, "gpio"))
            self.assertFalse(hasattr(sink, "servo_output"))
        finally:
            sink.close()


if __name__ == "__main__":
    unittest.main(verbosity=2)
