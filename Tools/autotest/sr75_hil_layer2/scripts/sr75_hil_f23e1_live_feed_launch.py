#!/usr/bin/env python3
"""HIL-F23-E1/E2A: safe live JSBSim-state-feed launch for the SR-75 Pixhawk bench.

Orchestrates, in order:
  a. parse the V2 mission
  b. compute the mission-derived release bearing
  c. generate a temporary runtime IC copy (validated source XML untouched)
  d. establish Pixhawk heartbeat (read-only)
  e. confirm GPS1_TYPE=14 (read-only)
  f. start the bridge in no-actuator-output mode (the only mode it has),
     forwarding to Mission Planner via --mp-out
  g. start JSBSim LAST, open-loop (no <input> command port at all -- there
     is no actuator/command feedback path in this task), using the runtime IC
  h. wait for the producer-readiness gate (state CSV exists, has a valid
     header, and at least one parseable data row), THEN begin the timed
     live-feed observation window (GPS/airspeed/attitude transmission
     itself begins automatically inside the bridge, driven by
     JSBSimStateFeeder, once it sees that same first real row)

This script NEVER arms, NEVER uploads/starts a mission on the Pixhawk,
NEVER sends actuator commands, and NEVER passes --allow-actuator-output to
the bridge. CH7/CH8 stay at whatever RC/PWM state the Pixhawk itself is
already showing (unaffected by anything this script does).

HIL-F23-E2A fixes (see reports/ for the full root-cause writeup):
  - --mp-out was never forwarded to the bridge at all, so Mission Planner
    never received any telemetry regardless of what the bridge/JSBSim were
    doing -- now a first-class option, forwarded unchanged, defaulted to
    udp:192.168.64.1:14550, and printed in the startup summary.
  - compute_comparison_summary() was reading the wrong CSV column names
    (jsb_lat_deg/jsb_lon_deg/... instead of the bridge's actual
    jsb_feed_lat_deg/jsb_feed_lon_deg/... columns, which is what
    --jsbsim-state-input actually populates) -- this alone was enough to
    make a fully-working live feed report "rows_with_jsbsim_data=0".
    Fixed to use the correct jsb_feed_* column names.
  - Neither child process was polled or had its console output captured,
    so a JSBSim (or bridge) launch failure was completely silent. Both
    are now supervised: stdout/stderr go to jsbsim_console.log and
    bridge_console.log, both are polled continuously, and either exiting
    unexpectedly aborts immediately with its exit code and last 30
    console lines.
  - There was no gate between "JSBSim process started" and "start the
    60s observation window" -- the full window could elapse before
    JSBSim ever produced a first row. A producer-readiness gate now
    blocks (up to 10s, process-supervised) until the state CSV exists,
    has a recognizable header, and has at least one parseable data row.

SR-75 LAYER 2 HIL BENCH SAFETY:
No live engine, no fuel pump, no live RATO ignition, no live ejection.
CH7/CH8 must be wired to dummy loads, LEDs, a PWM logger, or left
disconnected only.
"""
import argparse
import csv
import math
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[4]
BRIDGE_DIR = REPO / "Tools/autotest/sr75_hil_layer2/bridge"
sys.path.insert(0, str(BRIDGE_DIR))
import sr75_mission_heading as smh  # noqa: E402

BRIDGE_SCRIPT = BRIDGE_DIR / "sr75_jsbsim_pixhawk_hil_bridge.py"
DEFAULT_MISSION_FILE = REPO / "Tools/autotest/sr75_hil_layer2/closed_loop/SR75_F22_EXTENDED_ALTITUDE_PROFILE_V2.waypoints"
DEFAULT_SOURCE_IC = REPO / "Tools/autotest/aircraft/sr_75_6_dof/f22gzo_release_state_init.xml"
DEFAULT_MP_OUT = "udp:192.168.64.1:14550"

JSBSIM_CONSOLE_LOG_NAME = "jsbsim_console.log"
BRIDGE_CONSOLE_LOG_NAME = "bridge_console.log"

# Validated preserved-release-state constants (F22-GZ-AH..AM2 / HIL-F23-D).
RELEASE_LAT = 32.5378147
RELEASE_LON = 74.3661871
RELEASE_GROUNDSPEED_MPS = 43.85  # release TAS; used as the initial groundspeed
                                   # reference only, since the IC has zero wind
                                   # components (vbody=wbody=0) at t=0 -- NOT a
                                   # general TAS==groundspeed assumption.
PRECONTROL_ELEVATOR = 0.10  # same validated open-loop trim value used
PRECONTROL_THROTTLE = 0.37  # throughout F22-GZ-T/U/V/AA/AH..AM2

# A JSBSim state CSV header must contain at least one of these (after
# lowercasing) to be considered plausible -- rejects a header from some
# unrelated/garbled file rather than silently matching zero real columns.
CSV_HEADER_TIME_CANDIDATES = (
    "simulation/sim-time-sec", "/fdm/jsbsim/simulation/sim-time-sec",
    "time_s", "sim_time_s", "jsb_time_s", "jsb_feed_time_s",
)


def build_openloop_runscript(ic_name: str, run_dir: Path, case_name: str,
                              elevator: float, throttle: float, end_s: float):
    """Write a JSBSim runscript with NO <input> command port at all -- there
    is deliberately no actuator/command feedback path for this task. A
    single scripted <event> sets a fixed elevator/throttle trim once at
    t=0 (matching the validated F22-GZ-T/U/V precontrol values, purely so
    the generated state is a realistic changing flight, not a frozen
    point), and JSBSim flies open-loop from there for the whole window.

    The output schema is copied verbatim from the validated AM2 harness's
    <output> block, so it matches JSB_FEED_ALIASES in the bridge exactly.

    Returns (runscript_path, state_csv_path).
    """
    run_dir = Path(run_dir)
    run_dir.mkdir(parents=True, exist_ok=True)
    state_csv = run_dir / f"sr75_hil_f23e1_{case_name}_state.csv"
    runscript_path = run_dir / f"{case_name}.xml"
    rel_output = os.path.relpath(state_csv, REPO / "Tools/autotest")

    runscript_path.write_text(
        f"""<?xml version="1.0" encoding="utf-8"?>
<runscript xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
    xsi:noNamespaceSchemaLocation="http://jsbsim.sf.net/JSBSimScript.xsd"
    name="HIL-F23-E1 open-loop live-state-feed generator (no actuator input)">

  <use aircraft="sr_75_6_dof" initialize="{ic_name}"/>

  <run start="0.0" end="{end_s:.1f}" dt="0.008333">
    <event name="init_trim">
      <condition>simulation/sim-time-sec ge 0.0</condition>
      <set name="propulsion/engine[0]/set-running" value="1"/>
      <set name="propulsion/engine[1]/set-running" value="1"/>
      <set name="propulsion/engine[2]/set-running" value="1"/>
      <set name="fcs/elevator-cmd-norm" value="{elevator}"/>
      <set name="fcs/turbojet-throttle-cmd-norm" value="{throttle}"/>
      <set name="fcs/rato-throttle-cmd-norm" value="0.0"/>
      <notify/>
    </event>
  </run>

  <output name="{rel_output}" type="CSV" rate="60">
    <property>simulation/sim-time-sec</property>
    <property>position/lat-gc-deg</property>
    <property>position/long-gc-deg</property>
    <property>position/h-sl-ft</property>
    <property>position/h-agl-ft</property>
    <property>velocities/v-north-fps</property>
    <property>velocities/v-east-fps</property>
    <property>velocities/v-down-fps</property>
    <property>velocities/vc-kts</property>
    <property>velocities/vt-fps</property>
    <property>accelerations/a-pilot-x-ft_sec2</property>
    <property>accelerations/a-pilot-y-ft_sec2</property>
    <property>accelerations/a-pilot-z-ft_sec2</property>
    <property>attitude/phi-deg</property>
    <property>attitude/theta-deg</property>
    <function name="pitch_deg"><property>attitude/theta-deg</property></function>
    <property>attitude/psi-deg</property>
    <property>velocities/p-rad_sec</property>
    <property>velocities/q-rad_sec</property>
    <property>velocities/r-rad_sec</property>
    <function name="accel_body_x_mss"><product><property>accelerations/a-pilot-x-ft_sec2</property><value>0.3048</value></product></function>
    <function name="accel_body_y_mss"><product><property>accelerations/a-pilot-y-ft_sec2</property><value>0.3048</value></product></function>
    <function name="accel_body_z_mss"><product><property>accelerations/a-pilot-z-ft_sec2</property><value>0.3048</value></product></function>
  </output>
</runscript>
"""
    )
    return runscript_path, state_csv


def build_bridge_cmd(pixhawk, baud, state_csv, mission_file, release_lat, release_lon,
                      release_groundspeed_mps, mp_out, bridge_log_csv,
                      bridge_script=BRIDGE_SCRIPT, python_exe=None, send_yaw=False):
    """Pure command-list builder (no subprocess launch) so --mp-out/
    --gps-input-send-yaw propagation and the rest of the argument set can
    be unit tested. Never includes --allow-actuator-output.
    """
    cmd = [
        python_exe or sys.executable, str(bridge_script),
        "--pixhawk", str(pixhawk), "--baud", str(baud),
        "--jsbsim-state-input", str(state_csv),
        "--gps-input-inject", "--airspeed-inject", "--attitude-inject",
        "--mission-heading-from", str(mission_file),
        "--gps-lat", str(release_lat), "--gps-lon", str(release_lon),
        "--release-groundspeed-mps", str(release_groundspeed_mps),
        "--alignment-check",
        "--log", str(bridge_log_csv),
    ]
    if mp_out:
        cmd += ["--mp-out", str(mp_out)]
    if send_yaw:
        cmd += ["--gps-input-send-yaw"]
    return cmd


def build_jsbsim_cmd(root_dir, runscript_path):
    """Pure command-list builder for the JSBSim subprocess."""
    return ["JSBSim", f"--root={root_dir}", f"--script={runscript_path}", "--realtime"]


def tail_lines(path, n=30):
    """Return up to the last n lines of a text file, or [] if unreadable."""
    try:
        text = Path(path).read_text(errors="replace")
    except OSError:
        return []
    lines = text.splitlines()
    return lines[-n:]


def check_process_alive(proc, label, console_log_path):
    """Returns None while proc is still running (or proc is None). Returns
    an abort-ready diagnostic string, including exit code and the last 30
    console lines, the moment proc has exited.
    """
    if proc is None:
        return None
    rc = proc.poll()
    if rc is None:
        return None
    lines = tail_lines(console_log_path, 30)
    detail = "\n".join(lines) if lines else "(no console output captured)"
    return (
        f"{label} exited unexpectedly rc={rc}\n"
        f"--- last {len(lines)} line(s) of {console_log_path} ---\n{detail}"
    )


def header_looks_valid(header_line):
    """A JSBSim state CSV header must be non-empty, comma-separated, and
    contain at least one recognizable time-like column."""
    if not header_line:
        return False
    columns = [c.strip().lower() for c in header_line.split(",")]
    if len(columns) < 2:
        return False
    return any(candidate in columns for candidate in CSV_HEADER_TIME_CANDIDATES)


def read_first_two_lines(path):
    try:
        with open(path, "r", newline="", encoding="utf-8") as f:
            header = f.readline().strip()
            data = f.readline().strip()
    except OSError:
        return None, None
    return (header or None), (data or None)


def wait_for_producer_ready(state_csv_path, timeout_s=10.0, poll_interval_s=0.2,
                             extra_alive_check=None):
    """Block (up to timeout_s) until state_csv_path exists, has a
    recognizable header, and has at least one parseable data row.

    extra_alive_check: optional zero-arg callable returning an abort-ready
    diagnostic string (or None if everything is still alive) -- used to
    bail out immediately if a supervised subprocess dies while waiting,
    instead of waiting out the full timeout pointlessly.

    Returns (ok: bool, message: str).
    """
    deadline = time.time() + timeout_s
    header, data = None, None
    while True:
        if extra_alive_check is not None:
            abort_message = extra_alive_check()
            if abort_message is not None:
                return False, f"producer died while waiting for readiness:\n{abort_message}"

        header, data = read_first_two_lines(state_csv_path)
        if header is not None and header_looks_valid(header) and data:
            fields = data.split(",")
            if len(fields) >= 2:
                try:
                    float(fields[1])  # first real JSBSim column, right after JSBSim's own "Time"
                    return True, "producer ready: state CSV has a valid header and a parseable data row"
                except ValueError:
                    pass

        if time.time() >= deadline:
            if not Path(state_csv_path).exists():
                return False, f"producer readiness timed out: state CSV never appeared at {state_csv_path}"
            if header is None:
                return False, f"producer readiness timed out: state CSV has no readable header: {state_csv_path}"
            if not header_looks_valid(header):
                return False, f"producer readiness timed out: state CSV header does not look like a JSBSim state header: {header!r}"
            return False, f"producer readiness timed out: state CSV has a header but no parseable data row within {timeout_s:.0f}s"
        time.sleep(poll_interval_s)


def cleanup_artifacts(paths, keep_artifacts, run_succeeded):
    """Delete each existing path in `paths` only when the run succeeded AND
    --keep-artifacts was not requested. On any failure/abort, or whenever
    --keep-artifacts is set, everything is left in place for debugging.
    Returns the list of paths actually removed.
    """
    if keep_artifacts or not run_succeeded:
        return []
    removed = []
    for p in paths:
        p = Path(p)
        if p.exists():
            p.unlink()
            removed.append(p)
    return removed


def compute_comparison_summary(bridge_log_csv):
    """Post-process the bridge's own CSV log into the required JSBSim-vs-
    Pixhawk comparison summary. Pure function, no hardware/process access
    -- operates entirely on an already-written CSV file.

    HIL-F23-E2A fix: the bridge's --jsbsim-state-input feed logs its
    columns with a jsb_feed_* prefix (jsb_feed_lat_deg, jsb_feed_lon_deg,
    ...), NOT jsb_lat_deg/jsb_lon_deg (those belong to a separate, unused
    --jsbsim-csv code path). Reading the wrong prefix made
    rows_with_jsbsim_data read 0 even on a fully-working feed.
    """
    with open(bridge_log_csv, newline="") as f:
        rows = list(csv.DictReader(f))

    def f_(row, key):
        v = row.get(key, "")
        try:
            return float(v)
        except (TypeError, ValueError):
            return None

    lat_err, lon_err, alt_err, gs_err, as_err, yaw_err = ([], [], [], [], [], [])
    n_with_jsb = 0
    for row in rows:
        jsb_lat, jsb_lon, jsb_alt = f_(row, "jsb_feed_lat_deg"), f_(row, "jsb_feed_lon_deg"), f_(row, "jsb_feed_alt_m")
        if jsb_lat is None or jsb_lon is None:
            continue
        n_with_jsb += 1
        pix_lat, pix_lon, pix_alt = f_(row, "lat_deg"), f_(row, "lon_deg"), f_(row, "alt_m")
        if pix_lat is not None:
            lat_err.append(abs(pix_lat - jsb_lat))
        if pix_lon is not None:
            lon_err.append(abs(pix_lon - jsb_lon))
        if pix_alt is not None and jsb_alt is not None:
            alt_err.append(abs(pix_alt - jsb_alt))

        jsb_vn, jsb_ve = f_(row, "jsb_feed_vn_mps"), f_(row, "jsb_feed_ve_mps")
        pix_gs = f_(row, "groundspeed_mps")
        if jsb_vn is not None and jsb_ve is not None and pix_gs is not None:
            jsb_gs = math.hypot(jsb_vn, jsb_ve)
            gs_err.append(abs(pix_gs - jsb_gs))

        jsb_as = f_(row, "jsb_feed_airspeed_mps")
        pix_as = f_(row, "airspeed_mps")
        if jsb_as is not None and pix_as is not None:
            as_err.append(abs(pix_as - jsb_as))

        jsb_yaw_rad = f_(row, "jsb_feed_yaw_rad")
        pix_yaw_rad = f_(row, "ahrs2_yaw_rad")
        if pix_yaw_rad is None:
            pix_yaw_rad = f_(row, "att_yaw_rad")
        if jsb_yaw_rad is not None and pix_yaw_rad is not None:
            jsb_yaw_deg = math.degrees(jsb_yaw_rad)
            pix_yaw_deg = math.degrees(pix_yaw_rad)
            yaw_err.append(abs(smh.wrap_angle_error_deg(jsb_yaw_deg, pix_yaw_deg)))

    def stats(values):
        if not values:
            return {"n": 0, "max": None, "mean": None}
        return {"n": len(values), "max": max(values), "mean": sum(values) / len(values)}

    t_start = f_(rows[0], "time_s") if rows else None
    t_end = f_(rows[-1], "time_s") if rows else None
    duration_s = (t_end - t_start) if (t_start is not None and t_end is not None) else None
    message_rate_hz = (len(rows) / duration_s) if duration_s else None

    return {
        "rows_total": len(rows),
        "rows_with_jsbsim_data": n_with_jsb,
        "duration_s": duration_s,
        "message_rate_hz": message_rate_hz,
        "lat_deg_abs_error": stats(lat_err),
        "lon_deg_abs_error": stats(lon_err),
        "alt_m_abs_error": stats(alt_err),
        "groundspeed_mps_abs_error": stats(gs_err),
        "airspeed_mps_abs_error": stats(as_err),
        "yaw_deg_abs_error": stats(yaw_err),
    }


def print_comparison_summary(summary):
    print("=== JSBSim vs Pixhawk comparison summary ===")
    print(f"rows_total={summary['rows_total']} rows_with_jsbsim_data={summary['rows_with_jsbsim_data']}")
    print(f"duration_s={summary['duration_s']} message_rate_hz={summary['message_rate_hz']}")
    for label, key in (
        ("latitude (deg)", "lat_deg_abs_error"),
        ("longitude (deg)", "lon_deg_abs_error"),
        ("altitude (m)", "alt_m_abs_error"),
        ("groundspeed (m/s)", "groundspeed_mps_abs_error"),
        ("airspeed (m/s)", "airspeed_mps_abs_error"),
        ("yaw (deg, wrapped)", "yaw_deg_abs_error"),
    ):
        s = summary[key]
        print(f"  {label}: n={s['n']} max_abs_error={s['max']} mean_abs_error={s['mean']}")


def preflight_checks(pixhawk_device: str, baud: int, param_timeout_s: float = 15.0):
    """Read-only Pixhawk heartbeat + GPS1_TYPE confirmation. Returns
    (ok: bool, message: str). Never sends PARAM_SET, arm, mission, RC
    override, or actuator commands. Closes its own connection before
    returning so the bridge subprocess can open the port fresh.
    """
    from pymavlink import mavutil

    print(f"Preflight: connecting to {pixhawk_device} at {baud} baud (read-only)...")
    master = mavutil.mavlink_connection(pixhawk_device, baud=baud, autoreconnect=True)
    hb = master.wait_heartbeat(timeout=30)
    if hb is None:
        master.close()
        return False, "no heartbeat received within 30s"
    armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    print(
        f"Preflight heartbeat OK: type={hb.type} autopilot={hb.autopilot} "
        f"armed={int(armed)} mode={mavutil.mode_string_v10(hb)}"
    )
    if armed:
        master.close()
        return False, "vehicle reports ARMED -- refusing to proceed (this script never arms/disarms)"

    master.mav.param_request_read_send(master.target_system, master.target_component, b"GPS1_TYPE", -1)
    deadline = time.time() + param_timeout_s
    gps1_type = None
    while time.time() < deadline:
        msg = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
        if msg is None:
            continue
        pid = msg.param_id.rstrip("\x00") if isinstance(msg.param_id, str) else msg.param_id
        if pid == "GPS1_TYPE":
            gps1_type = msg.param_value
            break
    master.close()
    if gps1_type is None:
        return False, "GPS1_TYPE did not respond to PARAM_REQUEST_READ"
    if int(gps1_type) != 14:
        return False, f"GPS1_TYPE={int(gps1_type)}, expected 14 (MAVLink) -- refusing to proceed"
    print(f"Preflight GPS1_TYPE OK: {int(gps1_type)} (MAVLink)")
    return True, "ok"


def build_arg_parser():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--pixhawk", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--mission-file", default=str(DEFAULT_MISSION_FILE))
    parser.add_argument("--source-ic", default=str(DEFAULT_SOURCE_IC))
    parser.add_argument("--run-dir", default="/tmp/sr75_hil_f23e1")
    parser.add_argument("--duration-s", type=float, default=60.0, help="Live-feed observation window")
    parser.add_argument("--readiness-timeout-s", type=float, default=10.0,
                         help="Max time to wait for the first parseable JSBSim state row before aborting")
    parser.add_argument("--mp-out", default=DEFAULT_MP_OUT,
                         help="Mission Planner MAVLink output, forwarded unchanged to the bridge's --mp-out "
                              "(e.g. udp:192.168.64.1:14550). Pass an empty string to disable forwarding.")
    parser.add_argument("--gps-input-send-yaw", action="store_true", default=False,
                         help="HIL-F23-F2B0: forward --gps-input-send-yaw unchanged to the bridge, "
                              "enabling the GPS_INPUT.yaw MAVLink2 extension field (mission-derived or "
                              "live-JSBSim-feed heading). Disabled by default so every previously-"
                              "validated command is unaffected. Requires a separately-authorized "
                              "EK3_SRC1_YAW=2 parameter change on the Pixhawk to actually affect EKF "
                              "fusion -- this flag alone only changes what is transmitted.")
    parser.add_argument("--dry-run", action="store_true",
                         help="Perform steps a-c (mission/bearing/runtime-IC) and print what would be "
                              "launched. Never connects to hardware, never starts a subprocess.")
    parser.add_argument("--producer-only", action="store_true",
                         help="Perform steps a-c, then start JSBSim alone (no Pixhawk connection, no "
                              "bridge) and run it through the producer-readiness gate plus a short "
                              "observation window, printing the state CSV's own row count/columns. "
                              "Validates the JSBSim runscript/launch/supervision path in isolation, "
                              "with zero hardware involvement.")
    parser.add_argument("--keep-artifacts", action="store_true",
                         help="Do not delete the generated runtime IC/runscript/state-CSV/console logs "
                              "even after a successful run (they are always kept automatically on failure)")
    return parser


def run_producer_only(runscript_path, state_csv, run_dir, readiness_timeout_s, duration_s):
    """Launch JSBSim alone (no Pixhawk, no bridge) and run it through the
    same supervision + producer-readiness gate as the real pipeline.
    Zero hardware involvement -- pure software/simulation validation of
    steps (g)/(h)'s JSBSim side. Returns (ok: bool, jsbsim_console_log: Path).
    """
    jsbsim_console_log = run_dir / JSBSIM_CONSOLE_LOG_NAME
    jsb_cmd = build_jsbsim_cmd(REPO / "Tools/autotest", runscript_path)
    print(f"[producer-only] Starting JSBSim (open-loop, no actuator input path): {' '.join(jsb_cmd)}")
    jsb_stdout = open(jsbsim_console_log, "w")
    jsb_proc = subprocess.Popen(jsb_cmd, stdout=jsb_stdout, stderr=subprocess.STDOUT,
                                 start_new_session=True, cwd=str(REPO))
    try:
        def alive_check():
            return check_process_alive(jsb_proc, "JSBSim", jsbsim_console_log)

        ready_ok, ready_message = wait_for_producer_ready(
            state_csv, timeout_s=readiness_timeout_s, extra_alive_check=alive_check,
        )
        print(f"[producer-only] {ready_message}")
        if not ready_ok:
            print(f"[producer-only] ABORT: {ready_message}")
            return False, jsbsim_console_log

        print(f"[producer-only] Observation window: {duration_s:.0f}s")
        obs_deadline = time.time() + duration_s
        while time.time() < obs_deadline:
            abort_message = alive_check()
            if abort_message is not None:
                print(f"[producer-only] ABORT: {abort_message}")
                return False, jsbsim_console_log
            time.sleep(1.0)
    finally:
        if jsb_proc.poll() is None:
            try:
                os.killpg(jsb_proc.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            time.sleep(1.0)
            if jsb_proc.poll() is None:
                try:
                    os.killpg(jsb_proc.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
        jsb_stdout.close()

    try:
        with open(state_csv, newline="") as f:
            rows = list(csv.reader(f))
        print(f"[producer-only] state CSV columns: {', '.join(rows[0])}")
        print(f"[producer-only] state CSV data rows written: {max(0, len(rows) - 1)}")
        if len(rows) > 1:
            print(f"[producer-only] last row: {rows[-1]}")
    except OSError as exc:
        print(f"[producer-only] WARNING: could not read state CSV for summary: {exc}")

    print("[producer-only] PASS: JSBSim launched, produced a valid header, and produced parseable "
          "data rows -- no Pixhawk connection was ever made.")
    return True, jsbsim_console_log


def main():
    args = build_arg_parser().parse_args()
    run_dir = Path(args.run_dir)
    case_name = "f23e1_live_feed"
    runtime_ic_path = None
    runscript_path = None
    state_csv = None
    jsbsim_console_log = None
    bridge_console_log = None
    run_succeeded = False

    try:
        # a/b: parse mission + compute bearing (pure, no hardware)
        result = smh.compute_mission_heading(args.mission_file, RELEASE_LAT, RELEASE_LON)
        print("=== HIL-F23-E1/E2A startup: mission-derived heading ===")
        print(f"selected mission item: seq={result.item.seq} lat={result.item.lat:.7f} lon={result.item.lon:.7f} alt={result.item.alt:.1f}m")
        print(f"release: lat={result.release_lat:.7f} lon={result.release_lon:.7f}")
        print(f"bearing 0..360: {result.bearing_0_360_deg:.6f} deg  signed: {result.bearing_signed_deg:.6f} deg")
        print(f"Mission Planner forwarding (--mp-out): {args.mp_out or '(disabled)'}")
        print(f"GPS_INPUT yaw transmission: {'ENABLED' if args.gps_input_send_yaw else 'DISABLED'}")

        # c: runtime IC copy -- written alongside the source (same directory)
        # so JSBSim's aircraft-relative `initialize="..."` lookup can find it
        # by name; the SOURCE file itself is only ever opened read-only.
        source_ic_path = Path(args.source_ic)
        runtime_ic_path = smh.generate_runtime_ic_copy(
            source_ic_path, result.bearing_0_360_deg, dest_dir=str(source_ic_path.parent),
        )
        print(f"runtime IC copy: {runtime_ic_path} (source untouched: {source_ic_path})")

        runscript_path, state_csv = build_openloop_runscript(
            runtime_ic_path.stem, run_dir, case_name,
            PRECONTROL_ELEVATOR, PRECONTROL_THROTTLE, args.duration_s + 30.0,
        )
        print(f"runscript: {runscript_path}")
        print(f"state CSV: {state_csv}")

        if args.dry_run:
            print("")
            print("DRY RUN: stopping before any hardware connection or subprocess launch.")
            print("Would next: (d) wait_heartbeat, (e) confirm GPS1_TYPE=14, "
                  "(f) start bridge --no-actuator-output with --mp-out forwarding, (g) start JSBSim last, "
                  f"(h) wait up to {args.readiness_timeout_s:.0f}s for producer readiness, then observe for "
                  f"{args.duration_s:.0f}s, then print the comparison summary.")
            run_succeeded = True
            return 0

        if args.producer_only:
            print("")
            print("PRODUCER-ONLY: JSBSim only, no Pixhawk connection, no bridge subprocess.")
            ok, jsbsim_console_log = run_producer_only(
                runscript_path, state_csv, run_dir, args.readiness_timeout_s, args.duration_s,
            )
            run_succeeded = ok
            return 0 if ok else 2

        # d/e: read-only preflight
        ok, message = preflight_checks(args.pixhawk, args.baud)
        if not ok:
            print(f"ABORT: preflight failed: {message}")
            return 2

        # f: bridge, no-actuator-output (the only mode -- --allow-actuator-output
        # is deliberately never passed), forwarding to Mission Planner
        bridge_log_csv = run_dir / f"sr75_hil_f23e1_bridge_log_{time.strftime('%Y%m%d_%H%M%S')}.csv"
        bridge_console_log = run_dir / BRIDGE_CONSOLE_LOG_NAME
        bridge_cmd = build_bridge_cmd(
            args.pixhawk, args.baud, state_csv, args.mission_file,
            RELEASE_LAT, RELEASE_LON, RELEASE_GROUNDSPEED_MPS,
            args.mp_out, bridge_log_csv, send_yaw=args.gps_input_send_yaw,
        )
        print(f"Starting bridge (no-actuator-output, the default): {' '.join(bridge_cmd)}")
        bridge_stdout = open(bridge_console_log, "w")
        bridge_proc = subprocess.Popen(bridge_cmd, stdout=bridge_stdout, stderr=subprocess.STDOUT, start_new_session=True)

        jsb_proc = None
        jsb_stdout = None
        try:
            time.sleep(2.0)  # let the bridge establish its own Pixhawk connection first
            early_abort = check_process_alive(bridge_proc, "bridge", bridge_console_log)
            if early_abort is not None:
                print(f"ABORT: {early_abort}")
                return 2

            # g: JSBSim launched LAST, open-loop, using the runtime IC.
            # cwd=REPO is set explicitly (belt-and-suspenders, matching the
            # validated AM2 launch pattern) even though --root is already
            # absolute here.
            jsb_cmd = build_jsbsim_cmd(REPO / "Tools/autotest", runscript_path)
            print(f"Starting JSBSim (open-loop, no actuator input path): {' '.join(jsb_cmd)}")
            jsbsim_console_log = run_dir / JSBSIM_CONSOLE_LOG_NAME
            jsb_stdout = open(jsbsim_console_log, "w")
            jsb_proc = subprocess.Popen(jsb_cmd, stdout=jsb_stdout, stderr=subprocess.STDOUT,
                                         start_new_session=True, cwd=str(REPO))

            def alive_check():
                for proc, label, log in (
                    (bridge_proc, "bridge", bridge_console_log),
                    (jsb_proc, "JSBSim", jsbsim_console_log),
                ):
                    msg = check_process_alive(proc, label, log)
                    if msg is not None:
                        return msg
                return None

            # h(pre): producer-readiness gate -- do not start the observation
            # timer until the state CSV genuinely has usable data.
            ready_ok, ready_message = wait_for_producer_ready(
                state_csv, timeout_s=args.readiness_timeout_s, extra_alive_check=alive_check,
            )
            print(ready_message)
            if not ready_ok:
                print(f"ABORT: {ready_message}")
                return 2

            # h: live feed observation window, polling both children so an
            # unexpected exit aborts immediately instead of silently running
            # out the clock.
            print(f"Live-feed observation window: {args.duration_s:.0f}s")
            obs_deadline = time.time() + args.duration_s
            while time.time() < obs_deadline:
                abort_message = alive_check()
                if abort_message is not None:
                    print(f"ABORT: {abort_message}")
                    return 2
                time.sleep(1.0)
        finally:
            for proc in (jsb_proc, bridge_proc):
                if proc is not None and proc.poll() is None:
                    try:
                        os.killpg(proc.pid, signal.SIGTERM)
                    except ProcessLookupError:
                        pass
            time.sleep(1.0)
            for proc in (jsb_proc, bridge_proc):
                if proc is not None and proc.poll() is None:
                    try:
                        os.killpg(proc.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
            bridge_stdout.close()
            if jsb_stdout is not None:
                jsb_stdout.close()

        if bridge_log_csv.exists():
            summary = compute_comparison_summary(bridge_log_csv)
            print_comparison_summary(summary)
        else:
            print(f"WARNING: bridge log CSV not found at {bridge_log_csv}, cannot compute comparison summary")

        run_succeeded = True
        return 0
    finally:
        removed = cleanup_artifacts(
            [p for p in (runtime_ic_path, runscript_path, state_csv, jsbsim_console_log, bridge_console_log) if p is not None],
            args.keep_artifacts, run_succeeded,
        )
        for p in removed:
            print(f"cleaned up temporary file: {p}")
        if not run_succeeded and (runtime_ic_path is not None or runscript_path is not None):
            print("Run did not succeed -- runtime IC, runscript, state CSV, and console logs were retained for debugging.")


if __name__ == "__main__":
    sys.exit(main())
