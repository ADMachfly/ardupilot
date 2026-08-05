#!/usr/bin/env python3
"""SR-75 HIL-F23-C2: read-only GPS/EKF onboard parameter audit.

Diagnoses why ArduPlane is not accepting MAVLink GPS_INPUT by reading the
live onboard values of every parameter that gates the AP_GPS_MAV backend
and its use by EKF3, and comparing them against the values required for
GPS_INPUT to be accepted (per libraries/AP_GPS/AP_GPS_MAV.cpp,
AP_GPS_Params.cpp, AP_NavEKF_Source.cpp, AP_NavEKF3.cpp, AP_AHRS.cpp in
this exact source tree).

Only PARAM_REQUEST_READ is sent (read-only). This script NEVER sends
PARAM_SET, arm/disarm, mission upload/start, RC override, or any
actuator/servo command.

SR-75 LAYER 2 HIL BENCH SAFETY:
No live engine, no fuel pump, no live RATO ignition, no live ejection.
CH7/CH8 must be wired to dummy loads, LEDs, a PWM logger, or left
disconnected only.
"""
import argparse
import csv
import sys
import time
from pathlib import Path

# (param_name, required_value_for_GPS_INPUT, note)
# required_value is None where "just report the current value" (no single
# right answer, or already fine at any common default).
CHECKS = [
    ("GPS1_TYPE", "14", "0=None (default!) blocks AP_GPS_MAV entirely; 14=MAVLink required for GPS_INPUT. Renamed from GPS_TYPE."),
    ("GPS2_TYPE", None, "second GPS instance; irrelevant unless GPS_id=1 is used (bridge default gps_id=0). Renamed from GPS_TYPE2."),
    ("GPS_AUTO_CONFIG", None, "controls auto-config of *serial* GPS receivers only; does not gate MAVLink/GPS_INPUT reception."),
    ("GPS_AUTO_SWITCH", None, "multi-GPS blend/switch policy; irrelevant with a single GPS1_TYPE=MAVLink instance."),
    ("EK3_ENABLE", "1", "must be 1 for EKF3 to run at all. RebootRequired=True if changed."),
    ("AHRS_EKF_TYPE", "3", "must be 3 to use EKF3 for flight control on real hardware. SITL param files use 10 (External/passthrough) -- NOT valid on real hardware."),
    ("EK3_SRC1_POSXY", "3", "AP_NavEKF_Source::SourceXY::GPS == 3. Default is already GPS."),
    ("EK3_SRC1_POSZ", None, "default is BARO (2), not GPS -- this is normal/expected and does not block GPS_INPUT use."),
    ("EK3_SRC1_VELXY", "3", "AP_NavEKF_Source::SourceXY::GPS == 3. Default is already GPS."),
    ("EK3_SRC1_VELZ", None, "default is GPS; fine either way for this diagnosis."),
    ("EK3_SRC1_YAW", None, "default is COMPASS (1), not GPS -- normal/expected, does not block GPS_INPUT use."),
    ("EK3_GPS_TYPE", None, "NOTE: this param no longer exists in this EKF3 version -- GPS source selection is via EK3_SRC1_* now. Expect NO_RESPONSE."),
    ("SERIAL1_PROTOCOL", None, "informational: TELEM1 state (2=MAVLink2, 48=PPP) -- irrelevant to a USB (/dev/ttyACM0) connection, relevant if GPS_INPUT is later sent over TELEM1/PPP instead."),
    ("SERIAL1_BAUD", None, "informational, see above."),
]


def fmt(v):
    if v is None:
        return "(none)"
    if isinstance(v, float) and v == int(v):
        return str(int(v))
    return str(v)


def read_params(master, names, timeout_s, retries, log):
    results = {}
    remaining = list(names)
    for attempt in range(retries):
        if not remaining:
            break
        for name in remaining:
            master.mav.param_request_read_send(
                master.target_system, master.target_component,
                name.encode("utf-8"), -1,
            )
        deadline = time.time() + timeout_s
        still_missing = set(remaining)
        while time.time() < deadline and still_missing:
            msg = master.recv_match(type=["PARAM_VALUE", "STATUSTEXT"], blocking=True, timeout=0.5)
            if msg is None:
                continue
            if msg.get_type() == "STATUSTEXT":
                log(f"STATUSTEXT: {msg.text}")
                continue
            pid = msg.param_id.rstrip("\x00") if isinstance(msg.param_id, str) else msg.param_id
            if pid in still_missing:
                results[pid] = msg.param_value
                still_missing.discard(pid)
                log(f"PARAM_VALUE {pid} = {msg.param_value}")
        remaining = list(still_missing)
        if remaining:
            log(f"attempt {attempt+1}/{retries}: still missing {remaining}, retrying...")
    for name in remaining:
        results[name] = None
    return results


def main():
    from pymavlink import mavutil

    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--pixhawk", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--param-timeout-s", type=float, default=4.0)
    parser.add_argument("--retries", type=int, default=3)
    parser.add_argument("--log", default=None)
    parser.add_argument("--csv", default=None)
    args = parser.parse_args()

    log_lines = []

    def log(line):
        print(line)
        log_lines.append(line)

    log(f"Connecting to {args.pixhawk} at {args.baud} baud (read-only)...")
    master = mavutil.mavlink_connection(args.pixhawk, baud=args.baud, autoreconnect=True)
    log("Waiting for heartbeat...")
    hb = master.wait_heartbeat(timeout=30)
    if hb is None:
        log("FAIL: no heartbeat")
        _write(args.log, log_lines)
        return 2
    armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    log(
        f"HEARTBEAT: type={hb.type} autopilot={hb.autopilot} base_mode={hb.base_mode} "
        f"custom_mode={hb.custom_mode} system_status={hb.system_status} armed={int(armed)} "
        f"mode={mavutil.mode_string_v10(hb)}"
    )

    names = [c[0] for c in CHECKS]
    log("")
    log("=== Reading GPS/EKF params (PARAM_REQUEST_READ only, read-only) ===")
    onboard = read_params(master, names, args.param_timeout_s, args.retries, log)

    log("")
    log("=== GPS_INPUT readiness audit ===")
    rows = []
    blockers = []
    for name, required, note in CHECKS:
        value = onboard.get(name)
        if value is None:
            status = "NO_RESPONSE (param likely absent from this firmware build)"
        elif required is None:
            status = "INFO"
        elif fmt(value) == required:
            status = "OK"
        else:
            status = f"BLOCKER (required={required})"
            blockers.append(name)
        line = f"{name:16s} = {fmt(value):>6s}  {status}"
        log(line)
        log(f"    {note}")
        rows.append({"param": name, "value": fmt(value), "status": status, "note": note})

    log("")
    if blockers:
        log(f"BLOCKING PARAMS: {', '.join(blockers)}")
    else:
        log("No blocking mismatches found among checked params (see NO_RESPONSE rows for firmware gaps).")

    log("")
    log("Safety confirmation: only PARAM_REQUEST_READ was sent (read-only); no "
        "PARAM_SET, arm, mission, RC override, or actuator command.")

    _write(args.log, log_lines)
    if args.csv:
        with open(args.csv, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=["param", "value", "status", "note"])
            w.writeheader()
            w.writerows(rows)
        print(f"CSV written to {args.csv}")

    return 1 if blockers else 0


def _write(path, lines):
    if not path:
        return
    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")
    print(f"Log written to {path}")


if __name__ == "__main__":
    sys.exit(main())
