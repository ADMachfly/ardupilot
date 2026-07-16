#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Offline SIM_JSON target checks for SR-75 PPP preparation."""

import argparse
import csv
import json
import os
import socket
import struct
import subprocess
import sys
import tempfile
import time
from pathlib import Path


SERVO16_MAGIC = 18458
SERVO16_STRUCT = struct.Struct("<HHI16H")
DEFAULT_TARGET_IP = "192.168.144.2"
DEFAULT_PORT = 9002


def build_packet() -> bytes:
    return SERVO16_STRUCT.pack(SERVO16_MAGIC, 100, 1, *([1500] * 16))


def can_bind_udp(address: str, port: int) -> tuple[bool, str]:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind((address, port))
    except OSError as exc:
        return False, str(exc)
    finally:
        sock.close()
    return True, "ok"


def find_free_udp_port() -> int:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])
    finally:
        sock.close()


def write_state_csv(path: Path) -> None:
    header = [
        "Time",
        "/fdm/jsbsim/simulation/sim-time-sec",
        "/fdm/jsbsim/position/lat-gc-deg",
        "/fdm/jsbsim/position/long-gc-deg",
        "/fdm/jsbsim/position/h-sl-ft",
        "/fdm/jsbsim/velocities/v-north-fps",
        "/fdm/jsbsim/velocities/v-east-fps",
        "/fdm/jsbsim/velocities/v-down-fps",
        "/fdm/jsbsim/velocities/vc-kts",
        "/fdm/jsbsim/velocities/vt-fps",
        "/fdm/jsbsim/attitude/phi-deg",
        "/fdm/jsbsim/attitude/theta-rad",
        "/fdm/jsbsim/attitude/psi-deg",
        "p_rad_s",
        "q_rad_s",
        "r_rad_s",
        "accel_body_x_mss",
        "accel_body_y_mss",
        "accel_body_z_mss",
        "q1",
        "q2",
        "q3",
        "q4",
    ]
    row = [
        "1.0",
        "1.0",
        "32.5378885",
        "74.3661944",
        "787.401575",
        "3.28084",
        "6.56168",
        "0.0",
        "134.125",
        "226.378",
        "20.0",
        "-0.17453293",
        "45.0",
        "0.01",
        "-0.02",
        "0.03",
        "0.0",
        "0.0",
        "-9.80665",
        "0.9005898",
        "0.1926659",
        "-0.0130987",
        "0.3894179",
    ]
    with path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        writer.writerow(row)


def wait_for_udp_bind(process: subprocess.Popen, host: str, port: int, timeout_s: float) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError(f"responder exited early with code {process.returncode}")
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(0.2)
        try:
            sock.sendto(build_packet(), (host, port))
            data, _ = sock.recvfrom(65535)
            if data.endswith(b"\n"):
                return
        except OSError:
            time.sleep(0.05)
        finally:
            sock.close()
    raise TimeoutError(f"responder did not answer on {host}:{port}")


def run_localhost_exchange(responder: Path) -> dict:
    port = find_free_udp_port()
    with tempfile.TemporaryDirectory(prefix="sr75_sim_json_target_") as tmpdir:
        state_file = Path(tmpdir) / "state.csv"
        write_state_csv(state_file)
        cmd = [
            sys.executable,
            str(responder),
            "--listen-host",
            "127.0.0.1",
            "--listen-port",
            str(port),
            "--state-file",
            str(state_file),
            "--state-timeout-ms",
            "60000",
            "--strict",
        ]
        process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        try:
            wait_for_udp_bind(process, "127.0.0.1", port, 5.0)
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.settimeout(2.0)
            try:
                sock.sendto(build_packet(), ("127.0.0.1", port))
                data, source = sock.recvfrom(65535)
            finally:
                sock.close()
            payload = json.loads(data.decode("utf-8"))
            return {
                "port": port,
                "source": f"{source[0]}:{source[1]}",
                "bytes": len(data),
                "timestamp": payload["timestamp"],
                "gyro": payload["imu"]["gyro"],
                "accel_body": payload["imu"]["accel_body"],
                "velocity": payload["velocity"],
                "airspeed": payload["airspeed"],
            }
        finally:
            process.terminate()
            try:
                process.communicate(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.communicate(timeout=5)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--target-ip", default=DEFAULT_TARGET_IP)
    parser.add_argument("--target-port", type=int, default=DEFAULT_PORT)
    parser.add_argument(
        "--responder",
        default="Tools/autotest/sr75_hil_layer2/sim_json/sr75_sim_json_responder.py",
        help="existing SR-75 SIM_JSON responder path",
    )
    parser.add_argument("--skip-localhost", action="store_true")
    return parser


def main() -> int:
    args = build_arg_parser().parse_args()
    responder = Path(args.responder)
    if not responder.exists():
        print(f"ERROR: responder not found: {responder}", file=sys.stderr)
        return 1

    print(f"Intended external SIM_JSON target: {args.target_ip}:{args.target_port}")
    ok, reason = can_bind_udp(args.target_ip, args.target_port)
    if ok:
        print(f"TARGET_BIND_OK: host can bind {args.target_ip}:{args.target_port}")
    else:
        print(f"TARGET_BIND_UNAVAILABLE: {args.target_ip}:{args.target_port} ({reason})")
        print("This is expected until the PPP interface owns the target host IP.")

    if not args.skip_localhost:
        result = run_localhost_exchange(responder)
        print("LOCALHOST_SIM_JSON_OK")
        print(f"  responder_port: {result['port']}")
        print(f"  reply_source: {result['source']}")
        print(f"  reply_bytes: {result['bytes']}")
        print(f"  timestamp: {result['timestamp']}")
        print(f"  gyro: {result['gyro']}")
        print(f"  accel_body: {result['accel_body']}")
        print(f"  velocity: {result['velocity']}")
        print(f"  airspeed: {result['airspeed']}")

    print("No serial hardware, firmware flash, arming, or actuator command was used.")
    return 0


if __name__ == "__main__":
    os.chdir(Path(__file__).resolve().parents[4])
    sys.exit(main())
