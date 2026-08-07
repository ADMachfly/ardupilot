#!/usr/bin/env python3
"""HIL-F24-R3I: read-only onboard-DataFlash check for the EKF3 bootstrap
transient-accel hypothesis (see Tools/autotest/sr75_hil_layer2/reports/
HIL_F24_R3G_ekf3_tilt_alignment_diagnosis.md and HIL_F24_R3H_ekf3_
bootstrap_transient_accel_trace.md).

R3H's live USB MAVLink trace could only observe telemetry *after* this
process connects, which was already after EKF3's bootstrap had latched
(the board was already booted). This script instead retrieves the
Pixhawk's own onboard .bin dataflash log for the CURRENT (most recent)
boot via the standard, read-only MAVLink log-transfer protocol
(LOG_REQUEST_LIST / LOG_REQUEST_DATA / LOG_REQUEST_END) -- which never
modifies vehicle state, params, or memory, only reads back a stored
file -- and analyzes it offline with pymavlink's DFReader.

Dataflash logging on this bench is LOG_DISARMED=1 (logs from power-on,
never armed) and LOG_BITMASK=65535 (includes MASK_LOG_IMU, bit 7 --
filtered/main-loop-rate "IMU" messages with AccX/AccY/AccZ). EK3_LOG_
LEVEL is unset (default 0 = full EKF3 streaming logging), so "XKF1"
(per-core Roll/Pitch/Yaw, already in degrees) is logged every EKF3
update. "MSG" entries mirror every GCS_SEND_TEXT/gcs().send_text() call
(GCS_Common.cpp's send_textv() -> logger->Write_Message()), including
the exact "EKF3 IMU%u initialised" text. So IF an SD card is fitted and
already has this boot's log, this single boot's entire EKF3 bootstrap
-- from power-on through the first XKF1 sample -- may already be
recorded, with no reboot and no live capture window required.

MASK_LOG_IMU_RAW (bit 19) is NOT set by LOG_BITMASK=65535 (a 16-bit
value cannot reach bit 19), so the truly raw, unfiltered, full-rate
accelerometer samples are not separately logged -- only the filtered
"IMU" message at main-loop rate. This is expected to be close enough to
identify the sample InitialiseFilterBootstrap() used (it reads the same
AP_InertialSensor-filtered value "IMU" logs), but is not a bit-exact
raw capture.

This script NEVER sends PARAM_SET, arm/disarm, mode change, mission
item, RC override, or actuator command. The only messages it sends are
the standard read-only log-transfer protocol (LOG_REQUEST_LIST,
LOG_REQUEST_DATA, LOG_REQUEST_END) -- the same protocol MAVProxy's/QGC's
"download log" feature uses, which only reads a stored file back over
the link and never writes to, erases, or otherwise modifies the
vehicle.
"""
import argparse
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import sr75_hil_f24r3g_ekf3_bootstrap_trace as bootstrap_trace  # noqa: E402 -- reuses tilt_from_accel()/compare_predicted_to_observed()/EKF3_INITIALISED_RE

DEFAULT_DIVERGENCE_THRESHOLD_DEG = 1.0
DEFAULT_MATCH_TOLERANCE_DEG = 2.0
DEFAULT_CHUNK_SIZE = 90  # MAVLink LOG_DATA payload max per the message definition


def list_log_entries(master, timeout_s: float = 15.0) -> List[dict]:
    """Read-only: LOG_REQUEST_LIST_SEND(0, 0xffff), collecting LOG_ENTRY
    replies until every id in [0, last_log_num] has been seen or
    timeout_s elapses. Never sends anything else.

    AP_Logger's own convention: when there are zero stored logs, it
    replies with a single sentinel LOG_ENTRY carrying num_logs=0 (id/
    size are meaningless in that case) rather than staying silent --
    that sentinel is filtered out here so callers can treat an empty
    return value as "no logs exist" uniformly, whether that's because
    of this sentinel or because nothing replied at all before timeout_s."""
    master.mav.log_request_list_send(master.target_system, master.target_component, 0, 0xffff)
    entries: Dict[int, dict] = {}
    last_log_num = None
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        msg = master.recv_match(type="LOG_ENTRY", blocking=True, timeout=0.5)
        if msg is None:
            continue
        if msg.num_logs == 0:
            return []
        entries[msg.id] = {
            "id": msg.id,
            "num_logs": msg.num_logs,
            "last_log_num": msg.last_log_num,
            "time_utc": msg.time_utc,
            "size": msg.size,
        }
        last_log_num = msg.last_log_num
        if last_log_num is not None and len(entries) >= (last_log_num + 1):
            break
    return sorted(entries.values(), key=lambda e: e["id"])


def pick_latest_log(entries: List[dict]) -> Optional[dict]:
    """Pure: the current/most-recent boot's log is the highest id."""
    if not entries:
        return None
    return max(entries, key=lambda e: e["id"])


def download_log(
    master, log_id: int, size: int, chunk_size: int = DEFAULT_CHUNK_SIZE,
    timeout_s: float = 5.0, retries: int = 5, log=lambda s: None,
) -> bytes:
    """Read-only: downloads log_id in chunk_size-byte pieces via
    LOG_REQUEST_DATA_SEND/LOG_DATA, retrying any offset that doesn't
    reply within timeout_s up to `retries` times. Never sends
    LOG_ERASE. Sends LOG_REQUEST_END once complete (also read-only --
    just tells the vehicle the transfer is done)."""
    data = bytearray(size)
    received = bytearray(size)  # 0/1 marker per byte received, checked per-chunk
    offset = 0
    while offset < size:
        want = min(chunk_size, size - offset)
        got_chunk = False
        for _attempt in range(retries):
            master.mav.log_request_data_send(master.target_system, master.target_component, log_id, offset, want)
            msg = master.recv_match(type="LOG_DATA", blocking=True, timeout=timeout_s)
            if msg is None or msg.ofs != offset:
                continue
            n = min(len(msg.data), want)
            data[offset:offset + n] = bytes(msg.data[:n])
            received[offset:offset + n] = b"\x01" * n
            got_chunk = True
            break
        if not got_chunk:
            log(f"WARN: gave up on offset {offset} after {retries} retries")
            break
        offset += want
        if offset % (chunk_size * 50) == 0:
            log(f"...{offset}/{size} bytes")
    master.mav.log_request_end_send(master.target_system, master.target_component)
    return bytes(data)


def save_log_bytes(path: str, data: bytes) -> None:
    with open(path, "wb") as f:
        f.write(data)


def analyze_bootstrap_from_dataflash(
    bin_path: str,
    divergence_threshold_deg: float = DEFAULT_DIVERGENCE_THRESHOLD_DEG,
    match_tolerance_deg: float = DEFAULT_MATCH_TOLERANCE_DEG,
    ekf_core: int = 0,
    imu_instance: int = 0,
) -> dict:
    """Offline (no hardware I/O): parses a downloaded .bin with
    pymavlink's DFReader, extracting XKF1 (EKF3 core attitude, already
    in degrees), IMU (filtered accel, AccX/Y/Z), and MSG (STATUSTEXT
    mirror) rows, then reuses sr75_hil_f24r3g_ekf3_bootstrap_trace's
    tilt_from_accel()/compare_predicted_to_observed() to test the same
    hypothesis against genuine first-boot data."""
    from pymavlink import DFReader

    dfr = DFReader.DFReader_binary(bin_path)
    xkf1_rows = []
    imu_rows = []
    msg_rows = []
    while True:
        m = dfr.recv_msg()
        if m is None:
            break
        mtype = m.get_type()
        if mtype == "XKF1" and getattr(m, "C", None) == ekf_core:
            xkf1_rows.append({"TimeUS": m.TimeUS, "roll_deg": m.Roll, "pitch_deg": m.Pitch, "yaw_deg": m.Yaw})
        elif mtype == "IMU" and getattr(m, "I", None) == imu_instance:
            imu_rows.append({"TimeUS": m.TimeUS, "xacc": m.AccX, "yacc": m.AccY, "zacc": m.AccZ})
        elif mtype == "MSG":
            msg_rows.append({"TimeUS": m.TimeUS, "text": m.Message})

    return summarize_dataflash_rows(xkf1_rows, imu_rows, msg_rows, divergence_threshold_deg, match_tolerance_deg)


def find_first_xkf1_divergence(xkf1_rows: List[dict], threshold_deg: float) -> Optional[dict]:
    """Pure: first XKF1 row (in TimeUS order) whose |roll| or |pitch|
    exceeds threshold_deg -- the dataflash analogue of R3H's
    find_first_divergence(), operating on XKF1 instead of live AHRS2."""
    for row in sorted(xkf1_rows, key=lambda r: r["TimeUS"]):
        if abs(row["roll_deg"]) > threshold_deg or abs(row["pitch_deg"]) > threshold_deg:
            return row
    return None


def nearest_imu_sample_at_or_before(imu_rows: List[dict], target_time_us: int) -> Optional[dict]:
    """Pure: dataflash analogue of R3H's nearest_accel_sample_at_or_
    before(), keyed on TimeUS instead of host_monotonic_time."""
    if not imu_rows:
        return None
    before = [r for r in imu_rows if r["TimeUS"] <= target_time_us]
    if before:
        return max(before, key=lambda r: r["TimeUS"])
    return min(imu_rows, key=lambda r: abs(r["TimeUS"] - target_time_us))


def summarize_dataflash_rows(
    xkf1_rows: List[dict], imu_rows: List[dict], msg_rows: List[dict],
    divergence_threshold_deg: float, match_tolerance_deg: float,
) -> dict:
    """Pure: builds the same shape of answer set as R3H's summarize(),
    but from a single boot's dataflash rows instead of a live capture."""
    bootstrap_events = [r for r in msg_rows if bootstrap_trace.detect_bootstrap_complete(r["text"])]
    bootstrap_row = min(bootstrap_events, key=lambda r: r["TimeUS"]) if bootstrap_events else None

    divergence_row = find_first_xkf1_divergence(xkf1_rows, divergence_threshold_deg)

    summary = {
        "xkf1_row_count": len(xkf1_rows),
        "imu_row_count": len(imu_rows),
        "msg_row_count": len(msg_rows),
        "divergence_threshold_deg": divergence_threshold_deg,
        "match_tolerance_deg": match_tolerance_deg,
        "ekf3_initialised_statustext_seen": bootstrap_row is not None,
        "ekf3_initialised_statustext": bootstrap_row,
        "first_divergence_time_us": None if divergence_row is None else divergence_row["TimeUS"],
        "observed_xkf1_at_divergence": None if divergence_row is None else {
            "roll_deg": divergence_row["roll_deg"], "pitch_deg": divergence_row["pitch_deg"],
        },
        "imu_sample_at_divergence": None,
        "predicted_tilt_from_accel": None,
        "reproduces_bootstrap_math": None,
        "roll_residual_deg": None,
        "pitch_residual_deg": None,
    }

    if divergence_row is not None:
        imu_row = nearest_imu_sample_at_or_before(imu_rows, divergence_row["TimeUS"])
        if imu_row is not None:
            summary["imu_sample_at_divergence"] = imu_row
            predicted = bootstrap_trace.tilt_from_accel(imu_row["xacc"], imu_row["yacc"], imu_row["zacc"])
            if predicted is not None:
                predicted_roll, predicted_pitch = predicted
                summary["predicted_tilt_from_accel"] = {"roll_deg": predicted_roll, "pitch_deg": predicted_pitch}
                matches, roll_res, pitch_res = bootstrap_trace.compare_predicted_to_observed(
                    predicted_roll, predicted_pitch,
                    divergence_row["roll_deg"], divergence_row["pitch_deg"],
                    match_tolerance_deg,
                )
                summary["reproduces_bootstrap_math"] = matches
                summary["roll_residual_deg"] = roll_res
                summary["pitch_residual_deg"] = pitch_res

    return summary


def build_arg_parser():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--pixhawk", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--heartbeat-timeout-s", type=float, default=30.0)
    parser.add_argument("--list-timeout-s", type=float, default=15.0)
    parser.add_argument("--download-timeout-s", type=float, default=5.0)
    parser.add_argument("--download-retries", type=int, default=5)
    parser.add_argument("--chunk-size", type=int, default=DEFAULT_CHUNK_SIZE)
    parser.add_argument("--divergence-threshold-deg", type=float, default=DEFAULT_DIVERGENCE_THRESHOLD_DEG)
    parser.add_argument("--match-tolerance-deg", type=float, default=DEFAULT_MATCH_TOLERANCE_DEG)
    parser.add_argument("--output-bin", required=True, help="Where to save the downloaded .bin dataflash log")
    parser.add_argument("--log-id", type=int, default=None, help="Download this log id instead of auto-picking the latest")
    return parser


def main():
    args = build_arg_parser().parse_args()
    from pymavlink import mavutil

    print(f"=== HIL-F24-R3I dataflash bootstrap check (read-only; --pixhawk {args.pixhawk}) ===")
    try:
        master = mavutil.mavlink_connection(args.pixhawk, baud=args.baud, autoreconnect=True)
    except OSError as exc:
        print(f"ABORT: no heartbeat within {args.heartbeat_timeout_s}s (could not open {args.pixhawk}: {exc})")
        return 2
    hb = master.wait_heartbeat(timeout=args.heartbeat_timeout_s)
    if hb is None:
        print(f"ABORT: no heartbeat within {args.heartbeat_timeout_s}s")
        return 2
    print(f"Heartbeat OK: type={hb.type} autopilot={hb.autopilot}")

    print("Requesting log list (LOG_REQUEST_LIST -- read-only)...")
    entries = list_log_entries(master, timeout_s=args.list_timeout_s)
    if not entries:
        print("RESULT: NO onboard logs found (no SD card fitted, or LOG_BACKEND_TYPE does not include File). "
              "DataFlash is NOT sufficient -- fall back to a live capture (HIL-F24-R3H) across an operator reboot.")
        return 1
    print(f"Found {len(entries)} log(s): ids {[e['id'] for e in entries]}")

    target = next((e for e in entries if e["id"] == args.log_id), None) if args.log_id is not None else pick_latest_log(entries)
    if target is None:
        print(f"ABORT: requested log id {args.log_id} not found")
        return 2
    print(f"Downloading log id={target['id']} size={target['size']} bytes (LOG_REQUEST_DATA -- read-only)...")
    data = download_log(
        master, target["id"], target["size"],
        chunk_size=args.chunk_size, timeout_s=args.download_timeout_s, retries=args.download_retries, log=print,
    )
    save_log_bytes(args.output_bin, data)
    print(f"Saved {len(data)}/{target['size']} bytes to {args.output_bin}")
    master.close()

    if len(data) < target["size"]:
        print("WARN: download incomplete -- analysis below may be missing trailing records")

    print()
    print("Analyzing offline with DFReader (no further hardware I/O)...")
    summary = analyze_bootstrap_from_dataflash(
        args.output_bin, divergence_threshold_deg=args.divergence_threshold_deg,
        match_tolerance_deg=args.match_tolerance_deg,
    )
    print(f"XKF1 rows={summary['xkf1_row_count']} IMU rows={summary['imu_row_count']} MSG rows={summary['msg_row_count']}")
    print(
        "TRACE_RESULT "
        f"first_divergence_time_us={summary['first_divergence_time_us']} "
        f"reproduces_bootstrap_math={summary['reproduces_bootstrap_math']} "
        f"roll_residual_deg={summary['roll_residual_deg']} "
        f"pitch_residual_deg={summary['pitch_residual_deg']}"
    )
    print(
        "Safety confirmation: only LOG_REQUEST_LIST/LOG_REQUEST_DATA/LOG_REQUEST_END "
        "(standard read-only log-transfer protocol) were sent; no PARAM_SET, arm, mission, "
        "RC override, actuator command, or LOG_ERASE."
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
