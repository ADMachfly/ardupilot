#!/usr/bin/env python3
"""HIL-F24-R2: read-only estimator/truth capture over the HIL-F24-R1
dynamic PPP/SIM_JSON path.

Records two synchronized time-series CSVs, both timestamped by this
process's own time.monotonic() (host monotonic time -- never wall-clock,
per task 3):

  - jsbsim_truth.csv: every distinct new snapshot of the SAME atomic
    state.csv the responder itself reads (tailed via
    sr75_sim_json_responder.LatestCSVReader -- the exact HIL-F24-P/Q/S/T
    -hardened reader, reused unmodified), each row timestamped with the
    host monotonic time this process observed it.
  - pixhawk_estimator.csv: every HEARTBEAT, GLOBAL_POSITION_INT,
    LOCAL_POSITION_NED, ATTITUDE, GPS_RAW_INT, VFR_HUD,
    EKF_STATUS_REPORT, SYS_STATUS, and STATUSTEXT message received from
    a read-only MAVLink connection to the Pixhawk's USB port, each row
    timestamped the same way.

This script NEVER sends PARAM_SET, arm/disarm, mode change, mission
item, RC override, or actuator command. The only messages it ever sends
are a read-only MAV_CMD_SET_MESSAGE_INTERVAL stream-rate request per
message type (the same established pattern
sr75_hil_f24c_preflash_precheck.py already uses for SERVO_OUTPUT_RAW).
It never writes to, or otherwise touches, state.csv -- only reads it.
"""
import argparse
import csv
import math
import signal
import socket
import sys
import time
from pathlib import Path

SIM_JSON_DIR = Path(__file__).resolve().parents[1] / "sim_json"
sys.path.insert(0, str(SIM_JSON_DIR))
import sr75_sim_json_responder as responder  # noqa: E402

TRUTH_CSV_FIELDS = [
    "host_monotonic_time", "time_s", "lat_deg", "lon_deg", "alt_m",
    "roll_rad", "pitch_rad", "yaw_rad", "vn_mps", "ve_mps", "vd_mps",
    "p_rad_s", "q_rad_s", "r_rad_s",
    "accel_body_x_mss", "accel_body_y_mss", "accel_body_z_mss", "airspeed_mps",
]

PIXHAWK_CSV_FIELDS = [
    "host_monotonic_time", "host_wall_time", "msg_type",
    "hb_armed", "hb_mode", "hb_mode_is_manual", "hb_system_status",
    "gpi_lat_deg", "gpi_lon_deg", "gpi_alt_m", "gpi_relative_alt_m",
    "gpi_vn_mps", "gpi_ve_mps", "gpi_vd_mps", "gpi_hdg_deg",
    "lpn_x_m", "lpn_y_m", "lpn_z_m", "lpn_vx_mps", "lpn_vy_mps", "lpn_vz_mps",
    "att_roll_deg", "att_pitch_deg", "att_yaw_deg", "att_p_deg_s", "att_q_deg_s", "att_r_deg_s",
    "gri_fix_type", "gri_lat_deg", "gri_lon_deg", "gri_alt_m", "gri_satellites_visible",
    "vfr_airspeed_mps", "vfr_groundspeed_mps", "vfr_heading_deg", "vfr_alt_m", "vfr_climb_mps",
    "ekf_flags", "ekf_velocity_variance", "ekf_pos_horiz_variance",
    "ekf_pos_vert_variance", "ekf_compass_variance",
    "sys_onboard_control_sensors_health", "sys_battery_remaining",
    "statustext_severity", "statustext_text",
]

# HIL-F24-R2 task 1: message types requested read-only via
# MAV_CMD_SET_MESSAGE_INTERVAL (HEARTBEAT and STATUSTEXT are not
# rate-requested -- HEARTBEAT streams by default and STATUSTEXT is
# event-driven, not periodic).
RATE_REQUESTED_MESSAGES_HZ = {
    "GLOBAL_POSITION_INT": 10.0,
    "LOCAL_POSITION_NED": 10.0,
    "ATTITUDE": 10.0,
    "GPS_RAW_INT": 5.0,
    "VFR_HUD": 10.0,
    "EKF_STATUS_REPORT": 2.0,
    "SYS_STATUS": 1.0,
}


class MavlinkUdpForwarder:
    """One-way MAVLink telemetry fanout for Mission Planner viewing.

    This class deliberately has no receive path: it copies bytes already
    received from the Pixhawk serial link to a UDP endpoint and never writes
    anything back to the Pixhawk.
    """

    def __init__(self, host: str, port: int, socket_factory=socket.socket):
        self.endpoint = (host, int(port))
        self.socket = socket_factory(socket.AF_INET, socket.SOCK_DGRAM)
        self.packet_count = 0
        self.byte_count = 0

    def forward_message(self, msg) -> bool:
        get_msgbuf = getattr(msg, "get_msgbuf", None)
        if get_msgbuf is None:
            return False
        packet = bytes(get_msgbuf())
        if not packet:
            return False
        self.socket.sendto(packet, self.endpoint)
        self.packet_count += 1
        self.byte_count += len(packet)
        return True

    def close(self) -> None:
        self.socket.close()


def request_message_intervals(master, rates_hz):
    """HIL-F24-R2 task 1: a read-only stream-rate request per message
    type -- MAV_CMD_SET_MESSAGE_INTERVAL, never PARAM_SET/arm/mode/
    mission/RC-override/actuator. Mirrors sr75_hil_f24c_preflash_
    precheck.py's read_ch_pwm() request pattern exactly."""
    from pymavlink import mavutil
    for name, hz in rates_hz.items():
        msg_id = getattr(mavutil.mavlink, f"MAVLINK_MSG_ID_{name}", None)
        if msg_id is None:
            continue
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            msg_id, int(1e6 / hz), 0, 0, 0, 0, 0,
        )


def mavlink_message_to_row(msg, host_monotonic_time):
    """HIL-F24-R2: converts one received MAVLink message into a
    PIXHAWK_CSV_FIELDS-shaped row (None for every field that doesn't
    apply to this message's type). Pure function -- no I/O, directly
    unit-testable against synthetic/constructed message objects."""
    from pymavlink import mavutil

    msg_type = msg.get_type()
    row = {field: None for field in PIXHAWK_CSV_FIELDS}
    row["host_monotonic_time"] = host_monotonic_time
    row["host_wall_time"] = time.time()
    row["msg_type"] = msg_type

    if msg_type == "HEARTBEAT":
        armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        mode = mavutil.mode_string_v10(msg)
        row["hb_armed"] = 1.0 if armed else 0.0
        row["hb_mode"] = mode
        row["hb_mode_is_manual"] = 1.0 if mode == "MANUAL" else 0.0
        row["hb_system_status"] = float(msg.system_status)
    elif msg_type == "GLOBAL_POSITION_INT":
        row["gpi_lat_deg"] = msg.lat * 1.0e-7
        row["gpi_lon_deg"] = msg.lon * 1.0e-7
        row["gpi_alt_m"] = msg.alt * 1.0e-3
        row["gpi_relative_alt_m"] = msg.relative_alt * 1.0e-3
        row["gpi_vn_mps"] = msg.vx * 1.0e-2
        row["gpi_ve_mps"] = msg.vy * 1.0e-2
        row["gpi_vd_mps"] = msg.vz * 1.0e-2
        if msg.hdg != 65535:
            row["gpi_hdg_deg"] = msg.hdg * 1.0e-2
    elif msg_type == "LOCAL_POSITION_NED":
        row["lpn_x_m"] = msg.x
        row["lpn_y_m"] = msg.y
        row["lpn_z_m"] = msg.z
        row["lpn_vx_mps"] = msg.vx
        row["lpn_vy_mps"] = msg.vy
        row["lpn_vz_mps"] = msg.vz
    elif msg_type == "ATTITUDE":
        row["att_roll_deg"] = math.degrees(msg.roll)
        row["att_pitch_deg"] = math.degrees(msg.pitch)
        row["att_yaw_deg"] = math.degrees(msg.yaw)
        row["att_p_deg_s"] = math.degrees(msg.rollspeed)
        row["att_q_deg_s"] = math.degrees(msg.pitchspeed)
        row["att_r_deg_s"] = math.degrees(msg.yawspeed)
    elif msg_type == "GPS_RAW_INT":
        row["gri_fix_type"] = float(msg.fix_type)
        row["gri_lat_deg"] = msg.lat * 1.0e-7
        row["gri_lon_deg"] = msg.lon * 1.0e-7
        row["gri_alt_m"] = msg.alt * 1.0e-3
        row["gri_satellites_visible"] = float(msg.satellites_visible)
    elif msg_type == "VFR_HUD":
        row["vfr_airspeed_mps"] = float(msg.airspeed)
        row["vfr_groundspeed_mps"] = float(msg.groundspeed)
        row["vfr_heading_deg"] = float(msg.heading)
        row["vfr_alt_m"] = float(msg.alt)
        row["vfr_climb_mps"] = float(msg.climb)
    elif msg_type == "EKF_STATUS_REPORT":
        row["ekf_flags"] = float(msg.flags)
        row["ekf_velocity_variance"] = float(msg.velocity_variance)
        row["ekf_pos_horiz_variance"] = float(msg.pos_horiz_variance)
        row["ekf_pos_vert_variance"] = float(msg.pos_vert_variance)
        row["ekf_compass_variance"] = float(msg.compass_variance)
    elif msg_type == "SYS_STATUS":
        row["sys_onboard_control_sensors_health"] = float(msg.onboard_control_sensors_health)
        row["sys_battery_remaining"] = float(msg.battery_remaining)
    elif msg_type == "STATUSTEXT":
        row["statustext_severity"] = float(msg.severity)
        row["statustext_text"] = msg.text
    else:
        return None
    return row


class TruthTailer:
    """HIL-F24-R2: polls the same atomic state.csv the responder reads
    (via the unmodified, HIL-F24-P/Q/S/T-hardened LatestCSVReader) and
    reports each distinct new snapshot exactly once, tagged with the
    host monotonic time it was first observed -- the same "observe once,
    tag with time.monotonic()" principle HIL-F24-T's own snapshot-
    identity tracking already uses, applied here to build a full time
    series rather than a single latest-value.
    """

    def __init__(self, state_file):
        self.reader = responder.LatestCSVReader(state_file)
        self._last_line = None

    def poll(self):
        """Returns a TRUTH_CSV_FIELDS-shaped row if a new snapshot has
        appeared since the last poll, else None. Never raises on a
        transient read race (StateFileNotReadyError/StateFileMalformedError
        are treated as "nothing new yet", matching how a slow-starting
        feeder is expected to behave)."""
        try:
            row, line, _age_s, _st = self.reader.read_latest()
        except (responder.StateFileNotReadyError, responder.StateFileMalformedError):
            return None
        if line == self._last_line:
            return None
        self._last_line = line
        out = {"host_monotonic_time": time.monotonic()}
        for field in TRUTH_CSV_FIELDS:
            if field == "host_monotonic_time":
                continue
            out[field] = row.get(field)
        return out


class _CaptureShutdownRequested(Exception):
    """HIL-F24-R2: raised by the SIGTERM handler -- same pattern as the
    R1 feeder's own signal handling, so the orchestrator's stop sequence
    still gets a clean summary print instead of a bare process kill."""


def _raise_shutdown_on_sigterm(signum, frame) -> None:
    raise _CaptureShutdownRequested()


def build_arg_parser():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--pixhawk", default="/dev/ttyACM0", help="Pixhawk USB MAVLink device (read-only)")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--state-file", required=True, help="Same atomic state.csv the responder reads")
    parser.add_argument("--duration-s", type=float, default=30.0)
    parser.add_argument("--jsbsim-truth-csv", required=True)
    parser.add_argument("--pixhawk-estimator-csv", required=True)
    parser.add_argument("--poll-interval-s", type=float, default=0.02)
    parser.add_argument(
        "--enable-mp-forward", action="store_true",
        help="Forward received Pixhawk MAVLink telemetry one-way to Mission Planner over UDP",
    )
    parser.add_argument("--mp-udp-host", default="127.0.0.1", help="Mission Planner UDP host")
    parser.add_argument("--mp-udp-port", type=int, default=14550, help="Mission Planner UDP port")
    return parser


def main():
    signal.signal(signal.SIGTERM, _raise_shutdown_on_sigterm)
    args = build_arg_parser().parse_args()
    from pymavlink import mavutil

    print(f"HIL-F24-R2 estimator capture: connecting to {args.pixhawk} at {args.baud} baud (read-only)...")
    forwarder = MavlinkUdpForwarder(args.mp_udp_host, args.mp_udp_port) if args.enable_mp_forward else None
    if forwarder is not None:
        print(
            "Mission Planner UDP telemetry fanout enabled: "
            f"{args.mp_udp_host}:{args.mp_udp_port} (one-way, no UDP receive path)"
        )
    master = mavutil.mavlink_connection(args.pixhawk, baud=args.baud, autoreconnect=True)
    hb = master.wait_heartbeat(timeout=30)
    if hb is None:
        print("ABORT: no heartbeat received within 30s")
        return 2
    print(f"Heartbeat OK: type={hb.type} autopilot={hb.autopilot}")
    request_message_intervals(master, RATE_REQUESTED_MESSAGES_HZ)

    tailer = TruthTailer(args.state_file)
    truth_count = 0
    pixhawk_counts = {}

    with open(args.jsbsim_truth_csv, "w", newline="", encoding="utf-8") as truth_f, \
            open(args.pixhawk_estimator_csv, "w", newline="", encoding="utf-8") as pixhawk_f:
        truth_writer = csv.DictWriter(truth_f, fieldnames=TRUTH_CSV_FIELDS)
        truth_writer.writeheader()
        pixhawk_writer = csv.DictWriter(pixhawk_f, fieldnames=PIXHAWK_CSV_FIELDS)
        pixhawk_writer.writeheader()

        start = time.monotonic()
        try:
            while time.monotonic() - start < args.duration_s:
                iter_start = time.monotonic()

                truth_row = tailer.poll()
                if truth_row is not None:
                    truth_writer.writerow(truth_row)
                    truth_f.flush()
                    truth_count += 1

                while True:
                    msg = master.recv_match(blocking=False)
                    if msg is None:
                        break
                    if forwarder is not None:
                        forwarder.forward_message(msg)
                    row = mavlink_message_to_row(msg, time.monotonic())
                    if row is not None:
                        pixhawk_writer.writerow(row)
                        pixhawk_f.flush()
                        pixhawk_counts[row["msg_type"]] = pixhawk_counts.get(row["msg_type"], 0) + 1

                sleep_s = args.poll_interval_s - (time.monotonic() - iter_start)
                if sleep_s > 0:
                    time.sleep(sleep_s)
        except _CaptureShutdownRequested:
            print("Stopped by SIGTERM")
        finally:
            truth_f.flush()
            pixhawk_f.flush()

    master.close()
    if forwarder is not None:
        forwarder.close()
    print(
        "ESTIMATOR_CAPTURE_SUMMARY "
        f"truth_rows={truth_count} "
        f"pixhawk_rows={sum(pixhawk_counts.values())} "
        f"pixhawk_counts_by_type={pixhawk_counts} "
        f"mp_forward_packets={forwarder.packet_count if forwarder is not None else 0} "
        f"mp_forward_bytes={forwarder.byte_count if forwarder is not None else 0}"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
