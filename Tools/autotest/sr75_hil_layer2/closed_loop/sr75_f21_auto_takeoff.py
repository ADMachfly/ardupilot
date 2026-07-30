#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""SR-75 Layer 2D-F21 full AUTO mission closed-loop test.

Drives JSBSim + sr75_sim_json_responder.py + a real ArduPlane SITL binary
through mission upload, arm, AUTO, RATO ignition/burn/handoff/ejection, and
prints a diagnostic summary. Uses the responder's --auto-gate-hold so the
fixed precontrol reference (elevator/throttle trimmed for
layer2j_b3_cruise_trim_init) is the only command JSBSim ever sees until
MODE=AUTO is confirmed -- this keeps ArduPlane's default FBWA boot mode from
ever driving the FDM before RATO can legally ignite (RATOController.update()
is itself gated on AUTO+TAKEOFF, see ArduPlane/commands_logic.cpp).
"""
import csv
import math
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

os.environ["MAVLINK20"] = "1"
from pymavlink import mavutil

REPO = Path(__file__).resolve().parents[4]
RUN = Path(os.environ.get("SR75_D3_RUN", "/tmp/sr75_layer2df20d3_A"))
MISSION_FILE = REPO / "Tools/autotest/sr75_hil_layer2/closed_loop/SR75_LAYER2D_F21_AUTO_TAKEOFF.waypoints"
AUTO_GATE_RELEASE_FILE = None  # set in main() to RUN / "auto_gate_release.flag"
# Pre-release precontrol elevator trim. A JSBSim-only sweep (elevator
# -0.15..-0.50 @ throttle=0.37, 5s hold from layer2j_b3_cruise_trim_init)
# found this value gives essentially zero net pitch drift (+2.92deg at t=5s
# vs the IC's +2.90deg) with the lowest peak transient in the sweep.
PRECONTROL_ELEVATOR = -0.49
CASE = "rato_auto_takeoff"
JSB_PORT = 5600
SIM_JSON_PORT = 9002
MAVLINK_PORT = 5770


def shell_quiet(cmd):
    subprocess.run(cmd, cwd=REPO, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def stop_stale():
    shell_quiet(["Tools/autotest/sr75_hil_layer2/closed_loop/stop_sr75_b3.sh"])
    for pattern in (
        "sr75_sim_json_responder.py",
        "JSBSim.*sr75_f6_rato_auto_takeoff.xml",
        "build/sitl/bin/arduplane.*json:127.0.0.1",
    ):
        subprocess.run(["pkill", "-f", pattern], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(0.5)


def write_files():
    RUN.mkdir(parents=True, exist_ok=True)
    retain_dry_booster = os.environ.get("SR75_D3_RETAIN_DRY", "0") == "1"
    param = RUN / "f6_rato_auto_takeoff.param"
    param.write_text(
        "\n".join(
            [
                "AHRS_EKF_TYPE 10",
                "STAB_PITCH_DOWN 0",
                "RC2_TRIM 1500",
                "ARSPD_SKIP_CAL 1",
                "ARSPD_OFFSET 0",
                "RLL_RATE_P 0.15",
                "RLL_RATE_I 0.25",
                "RLL_RATE_D 0.017430",
                "RLL_RATE_FF 0.08",
                "RLL2SRV_RMAX 90",
                "RATO_ENABLE 1",
                "RATO_IGN_CH 7",
                "RATO_EJ_CH 8",
                "RATO_BURN_S 3.0",
                "RATO_PITCH 20.0",
                "RATO_REL_MODE 1",
                "RATO_EJECT_S 10.0",
                "SERVO7_FUNCTION 0",
                "SERVO8_FUNCTION 0",
                "ARMING_CHECK 0",
                "ARMING_REQUIRE 0",
                "LOG_DISARMED 1",
                "",
            ]
        )
    )
    state_csv = RUN / f"sr75_b3_state_{CASE}.csv"
    runscript = RUN / "sr75_f20d2_rato_auto_takeoff.xml"
    rel_output = "../../../../../tmp/sr75_layer2df20d2/sr75_b3_state_rato_auto_takeoff.csv"
    runscript = RUN / "sr75_f20d3_rato_auto_takeoff.xml"
    rel_output = f"../../../../../tmp/{RUN.name}/sr75_b3_state_rato_auto_takeoff.csv"

    runscript.write_text(
        f"""<?xml version="1.0" encoding="utf-8"?>
<runscript xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
    xsi:noNamespaceSchemaLocation="http://jsbsim.sf.net/JSBSimScript.xsd"
    name="SR75 Layer 2D-F6 AUTO RATO closed-loop input">

  <use aircraft="sr_75_6_dof" initialize="layer2j_b3_cruise_trim_init"/>

  <input type="QTJSBSIM" port="{JSB_PORT}" rate="60">
    <property>fcs/elevator-cmd-norm</property>
    <property>fcs/aileron-cmd-norm</property>
    <property>fcs/rudder-cmd-norm</property>
    <property>fcs/turbojet-throttle-cmd-norm</property>
    <property>fcs/rato-throttle-cmd-norm</property>
    <property>propulsion/tank[2]/contents-lbs</property>
  </input>

  <run start="0.0" end="80.0" dt="0.008333">
    <event name="Set propulsion running and controls trimmed">
      <condition>simulation/sim-time-sec ge 0.0</condition>
      <set name="propulsion/engine[0]/set-running" value="1"/>
      <set name="propulsion/engine[1]/set-running" value="1"/>
      <set name="propulsion/engine[2]/set-running" value="1"/>
      <set name="propulsion/tank[1]/contents-lbs" value="20.238414"/>
      <set name="propulsion/tank[2]/contents-lbs" value="22.046226"/>
      <set name="fcs/elevator-cmd-norm" value="{PRECONTROL_ELEVATOR}"/>
      <set name="fcs/turbojet-throttle-cmd-norm" value="0.37"/>
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
    <function name="q1"><sum><product><cos><product><property>attitude/phi-rad</property><value>0.5</value></product></cos><cos><product><property>attitude/theta-rad</property><value>0.5</value></product></cos><cos><product><property>attitude/psi-rad</property><value>0.5</value></product></cos></product><product><sin><product><property>attitude/phi-rad</property><value>0.5</value></product></sin><sin><product><property>attitude/theta-rad</property><value>0.5</value></product></sin><sin><product><property>attitude/psi-rad</property><value>0.5</value></product></sin></product></sum></function>
    <function name="q2"><difference><product><sin><product><property>attitude/phi-rad</property><value>0.5</value></product></sin><cos><product><property>attitude/theta-rad</property><value>0.5</value></product></cos><cos><product><property>attitude/psi-rad</property><value>0.5</value></product></cos></product><product><cos><product><property>attitude/phi-rad</property><value>0.5</value></product></cos><sin><product><property>attitude/theta-rad</property><value>0.5</value></product></sin><sin><product><property>attitude/psi-rad</property><value>0.5</value></product></sin></product></difference></function>
    <function name="q3"><sum><product><cos><product><property>attitude/phi-rad</property><value>0.5</value></product></cos><sin><product><property>attitude/theta-rad</property><value>0.5</value></product></sin><cos><product><property>attitude/psi-rad</property><value>0.5</value></product></cos></product><product><sin><product><property>attitude/phi-rad</property><value>0.5</value></product></sin><cos><product><property>attitude/theta-rad</property><value>0.5</value></product></cos><sin><product><property>attitude/psi-rad</property><value>0.5</value></product></sin></product></sum></function>
    <function name="q4"><difference><product><cos><product><property>attitude/phi-rad</property><value>0.5</value></product></cos><cos><product><property>attitude/theta-rad</property><value>0.5</value></product></cos><sin><product><property>attitude/psi-rad</property><value>0.5</value></product></sin></product><product><sin><product><property>attitude/phi-rad</property><value>0.5</value></product></sin><sin><product><property>attitude/theta-rad</property><value>0.5</value></product></sin><cos><product><property>attitude/psi-rad</property><value>0.5</value></product></cos></product></difference></function>
    <property>fcs/elevator-cmd-norm</property>
    <property>fcs/aileron-cmd-norm</property>
    <property>fcs/turbojet-throttle-cmd-norm</property>
    <property>fcs/rato-throttle-cmd-norm</property>
    <property>fcs/left-elevon-pos-rad</property>
    <property>fcs/right-elevon-pos-rad</property>
    <property>fcs/throttle-cmd-norm[0]</property>
    <property>fcs/throttle-cmd-norm[1]</property>
    <property>fcs/throttle-cmd-norm[2]</property>
    <property>propulsion/engine[0]/set-running</property>
    <property>propulsion/engine[1]/set-running</property>
    <property>propulsion/engine[2]/set-running</property>
    <property>propulsion/engine[0]/thrust-lbs</property>
    <property>propulsion/engine[1]/thrust-lbs</property>
    <property>propulsion/engine[2]/thrust-lbs</property>
    <property>propulsion/tank[0]/contents-lbs</property>
    <property>propulsion/tank[1]/contents-lbs</property>
    <property>propulsion/tank[2]/contents-lbs</property>
    <property>inertia/weight-lbs</property>
    <property>inertia/cg-x-in</property>
    <property>inertia/cg-y-in</property>
    <property>inertia/cg-z-in</property>
    <property>inertia/iyy-slugs_ft2</property>
    <property>inertia/ixx-slugs_ft2</property>
    <property>inertia/izz-slugs_ft2</property>
  </output>
</runscript>
"""
    )
    channel_map = RUN / "channel_map.json"
    source_map = REPO / "Tools/autotest/sr75_hil_layer2/closed_loop/SR75_LAYER2J_B3_CHANNEL_MAP.json"
    channel_text = source_map.read_text()
    if retain_dry_booster:
        channel_text = channel_text.replace('"eject": 8', '"eject": 16')
    channel_map.write_text(channel_text)
    return param, runscript, state_csv, channel_map


def start(cmd, log, cwd=REPO):
    f = open(log, "w")
    p = subprocess.Popen(cmd, cwd=cwd, stdout=f, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, start_new_session=True)
    return p, f


def read_rows(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def fval(row, name, default=0.0):
    if name not in row:
        if name == "propulsion/tank[0]/contents-lbs":
            for key in row:
                if key.endswith("/propulsion/tank/contents-lbs"):
                    name = key
                    break
        for key in row:
            if key.endswith("/" + name):
                name = key
                break
    try:
        return float(row.get(name, default) or default)
    except ValueError:
        return default


def dry_lbs(row):
    return fval(row, "propulsion/tank[2]/contents-lbs")


def dry_kg(row):
    return dry_lbs(row) * 0.45359237


def tail(path, n=80):
    try:
        return "\n".join(Path(path).read_text(errors="replace").splitlines()[-n:])
    except FileNotFoundError:
        return ""


def upload_takeoff(master):
    lat = 32.5378085
    lon = 74.3661944
    alt = 3020.0
    comp = master.target_component
    print("mission_clear_send")
    master.mav.mission_clear_all_send(master.target_system, comp)
    clear_deadline = time.time() + 3
    while time.time() < clear_deadline:
        send_gcs_heartbeat(master)
        msg = master.recv_match(type=["MISSION_ACK", "STATUSTEXT"], blocking=True, timeout=0.5)
        if msg is None:
            continue
        if msg.get_type() == "STATUSTEXT":
            print("statustext", msg.text)
        else:
            print("mission_clear_ack", msg.type)
            break
    mission_items = []
    with MISSION_FILE.open() as mission_file:
        header = mission_file.readline().strip()
        if header != "QGC WPL 110":
            raise RuntimeError(f"unexpected mission header: {header}")
        for line in mission_file:
            if not line.strip():
                continue
            parts = line.split()
            mission_items.append({
                "seq": int(parts[0]),
                "current": int(parts[1]),
                "frame": int(parts[2]),
                "command": int(parts[3]),
                "p1": float(parts[4]),
                "p2": float(parts[5]),
                "p3": float(parts[6]),
                "p4": float(parts[7]),
                "lat": float(parts[8]),
                "lon": float(parts[9]),
                "alt": float(parts[10]),
                "autocontinue": int(parts[11]),
            })
    master.mav.mission_count_send(master.target_system, comp, len(mission_items))
    print("mission_file", str(MISSION_FILE))
    print("mission_count_send", len(mission_items))
    sent = set()
    final_ack = None
    deadline = time.time() + 8
    while time.time() < deadline and len(sent) < len(mission_items):
        send_gcs_heartbeat(master)
        msg = master.recv_match(type=["MISSION_REQUEST", "MISSION_REQUEST_INT", "MISSION_ACK", "STATUSTEXT"], blocking=True, timeout=1)
        if msg is None:
            print("mission_wait_timeout_partial", sorted(sent))
            continue
        if msg.get_type() == "STATUSTEXT":
            print("statustext", msg.text)
            continue
        if msg.get_type() == "MISSION_ACK":
            final_ack = msg.type
            print("mission_early_ack", msg.type)
            break
        seq = int(msg.seq)
        print("mission_request", msg.get_type(), seq)
        item = mission_items[seq]
        master.mav.mission_item_int_send(
            master.target_system, comp, item["seq"], item["frame"], item["command"],
            item["current"], item["autocontinue"],
            item["p1"], item["p2"], item["p3"], item["p4"],
            int(item["lat"] * 1e7), int(item["lon"] * 1e7), item["alt"],
        )
        sent.add(seq)
    # allow final ACK to arrive
    msg = master.recv_match(type=["MISSION_ACK", "STATUSTEXT"], blocking=True, timeout=3)
    if msg is not None and msg.get_type() == "STATUSTEXT":
        print("statustext", msg.text)
        msg = master.recv_match(type="MISSION_ACK", blocking=True, timeout=2)
    if msg is not None:
        final_ack = msg.type
        print("mission_final_ack", final_ack)
    print("mission_upload_result", "sent", sorted(sent), "ack", final_ack)
    master.mav.mission_set_current_send(master.target_system, comp, 1)
    print("mission_set_current_send 1")
    deadline = time.time() + 3
    while time.time() < deadline:
        msg = master.recv_match(type=["MISSION_CURRENT", "STATUSTEXT"], blocking=True, timeout=0.5)
        if msg is None:
            continue
        if msg.get_type() == "STATUSTEXT":
            print("statustext", msg.text)
        else:
            print("mission_current", msg.seq)
            break
    master.mav.mission_request_list_send(master.target_system, comp)
    msg = master.recv_match(type=["MISSION_COUNT", "STATUSTEXT"], blocking=True, timeout=3)
    count = None
    if msg is not None:
        if msg.get_type() == "STATUSTEXT":
            print("statustext", msg.text)
        else:
            print("mission_verify_count", msg.count)
            count = msg.count
    if count is not None:
        for seq in range(int(count)):
            master.mav.mission_request_int_send(master.target_system, comp, seq)
            item = master.recv_match(type=["MISSION_ITEM_INT", "MISSION_ITEM", "STATUSTEXT"], blocking=True, timeout=3)
            if item is None:
                print("mission_read_item_timeout", seq)
                continue
            if item.get_type() == "STATUSTEXT":
                print("statustext", item.text)
                continue
            print("mission_item", item.seq, item.command, item.current, item.autocontinue)
    return final_ack == mavutil.mavlink.MAV_MISSION_ACCEPTED


def prove_tx(master):
    comp = master.target_component
    tx_frames = []
    original_write = master.write

    def logged_write(buf):
        data = bytes(buf)
        tx_frames.append(data)
        return original_write(buf)

    master.write = logged_write
    print("tx_proof_param_request MAV_SYSID target", master.target_system, comp)
    before_counts = (
        getattr(master.mav, "total_packets_sent", None),
        getattr(master.mav, "total_packets_received", None),
        getattr(master.mav, "total_receive_errors", None),
    )
    requests = [
        ("param_id_comp1", comp, b"MAV_SYSID", -1),
        ("param_id_comp0", 0, b"MAV_SYSID", -1),
        ("param_index0_comp1", comp, b"", 0),
    ]
    got_param = False
    for req_name, req_comp, req_id, req_index in requests:
        print("param_variant", req_name, "target", master.target_system, req_comp, "id", req_id, "index", req_index)
        deadline = time.time() + 5
        sent_at = 0.0
        while time.time() < deadline:
            send_gcs_heartbeat(master)
            now = time.time()
            if now - sent_at > 1.0:
                tx_frames.clear()
                master.mav.param_request_read_send(master.target_system, req_comp, req_id, req_index)
                if tx_frames:
                    frame = tx_frames[-1]
                    if frame and frame[0] == 0xFD and len(frame) >= 10:
                        msgid = frame[7] | (frame[8] << 8) | (frame[9] << 16)
                        src_sys = frame[5]
                        src_comp = frame[6]
                        if msgid == mavutil.mavlink.MAVLINK_MSG_ID_PARAM_REQUEST_READ and len(frame) >= 14:
                            target_sys = frame[12]
                            target_comp = frame[13]
                        else:
                            target_sys = None
                            target_comp = None
                    else:
                        msgid = src_sys = src_comp = target_sys = target_comp = None
                    print(
                        "tcp_tx_frame",
                        "variant", req_name,
                        "bytes", len(frame),
                        "msgid", msgid,
                        "src", f"{src_sys}/{src_comp}",
                        "target", f"{target_sys}/{target_comp}",
                        "hex", frame.hex(),
                    )
                sent_at = now
            msg = master.recv_match(type=["PARAM_VALUE", "STATUSTEXT", "HEARTBEAT", "COMMAND_ACK"], blocking=True, timeout=0.5)
            if msg is None:
                continue
            if msg.get_type() == "STATUSTEXT":
                print("statustext", msg.text)
                continue
            if msg.get_type() == "HEARTBEAT":
                print("tx_seen_heartbeat", msg.get_srcSystem(), msg.get_srcComponent(), mavutil.mode_string_v10(msg))
                continue
            if msg.get_type() == "COMMAND_ACK":
                print("tx_seen_command_ack", msg.command, msg.result)
                continue
            pid = msg.param_id
            if isinstance(pid, bytes):
                pid = pid.decode("ascii", errors="replace").rstrip("\x00")
            print("tx_proof_param_value", pid, msg.param_value, "variant", req_name, "target_comp", req_comp)
            got_param = True
            break
        if got_param:
            break
    if not got_param:
        print("tx_proof_param_failed")
        after_counts = (
            getattr(master.mav, "total_packets_sent", None),
            getattr(master.mav, "total_packets_received", None),
            getattr(master.mav, "total_receive_errors", None),
        )
        print("mav_counters", "before", before_counts, "after", after_counts)
        master.write = original_write
        try:
            master.close()
        except Exception:
            pass
        mavproxy_ok = prove_mavproxy_udp()
        print("mavproxy_udp_result", mavproxy_ok)
        return False
    master.write = original_write

    print("tx_proof_request_message SYS_STATUS")
    master.mav.command_long_send(
        master.target_system,
        comp,
        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
        0,
        mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS,
        0,
        0,
        0,
        0,
        0,
        0,
    )
    deadline = time.time() + 5
    got_ack = False
    got_msg = False
    while time.time() < deadline:
        send_gcs_heartbeat(master)
        msg = master.recv_match(type=["COMMAND_ACK", "SYS_STATUS", "STATUSTEXT"], blocking=True, timeout=0.5)
        if msg is None:
            continue
        if msg.get_type() == "STATUSTEXT":
            print("statustext", msg.text)
        elif msg.get_type() == "COMMAND_ACK":
            print("tx_proof_command_ack", msg.command, msg.result)
            if msg.command == mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE:
                got_ack = msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED
        elif msg.get_type() == "SYS_STATUS":
            print("tx_proof_sys_status", msg.onboard_control_sensors_health)
            got_msg = True
        if got_ack and got_msg:
            return True
    print("tx_proof_request_message_failed", int(got_ack), int(got_msg))
    return False


def prove_mavproxy_udp():
    log_path = RUN / "mavproxy.log"
    with open(log_path, "w") as log:
        proc = subprocess.Popen(
            [
                "mavproxy.py",
                "--master=tcp:127.0.0.1:5760",
                "--out=udp:127.0.0.1:14555",
                "--non-interactive",
            ],
            cwd=REPO,
            stdout=log,
            stderr=subprocess.STDOUT,
            stdin=subprocess.DEVNULL,
            start_new_session=True,
        )
        try:
            udp = mavutil.mavlink_connection(
                "udpin:127.0.0.1:14555",
                source_system=255,
                source_component=mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER,
                dialect="ardupilotmega",
            )
            hb = udp.wait_heartbeat(timeout=10)
            if hb is None:
                print("mavproxy_udp no heartbeat")
                return False
            udp.target_system = hb.get_srcSystem()
            udp.target_component = hb.get_srcComponent()
            print("mavproxy_udp heartbeat", udp.target_system, udp.target_component, mavutil.mode_string_v10(hb))
            deadline = time.time() + 8
            sent_at = 0.0
            while time.time() < deadline:
                send_gcs_heartbeat(udp)
                now = time.time()
                if now - sent_at > 1.0:
                    udp.mav.param_request_read_send(udp.target_system, udp.target_component, b"SYSID_THISMAV", -1)
                    sent_at = now
                msg = udp.recv_match(type=["PARAM_VALUE", "HEARTBEAT", "STATUSTEXT"], blocking=True, timeout=0.5)
                if msg is None:
                    continue
                if msg.get_type() == "PARAM_VALUE":
                    pid = msg.param_id
                    if isinstance(pid, bytes):
                        pid = pid.decode("ascii", "replace").rstrip("\x00")
                    print("mavproxy_udp_param_value", pid, msg.param_value)
                    return pid == "SYSID_THISMAV"
                print("mavproxy_udp_seen", msg.get_type())
            return False
        finally:
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGTERM)
                time.sleep(0.5)
                if proc.poll() is None:
                    os.killpg(proc.pid, signal.SIGKILL)


def set_mode(master, mode):
    mapping = master.mode_mapping()
    print("mode_request", mode, mapping.get(mode))
    master.set_mode(mapping[mode])
    deadline = time.time() + 5
    while time.time() < deadline:
        msg = master.recv_match(type=["HEARTBEAT", "COMMAND_ACK", "STATUSTEXT"], blocking=True, timeout=1)
        if msg is None:
            continue
        if msg.get_type() == "STATUSTEXT":
            print("statustext", msg.text)
            continue
        if msg.get_type() == "COMMAND_ACK":
            print("mode_command_ack", msg.command, msg.result)
            continue
        hb = msg
        if hb:
            rev = {v: k for k, v in mapping.items()}
            seen = rev.get(hb.custom_mode, str(hb.custom_mode))
            print("mode_seen", seen, "base", hb.base_mode, "custom", hb.custom_mode)
            if seen == mode:
                return True
    return False


def send_gcs_heartbeat(master):
    master.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0,
        0,
        mavutil.mavlink.MAV_STATE_ACTIVE,
    )


def request_msg(master, msg_id, hz):
    comp = master.target_component
    master.mav.command_long_send(
        master.target_system,
        comp,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        msg_id,
        int(1_000_000 / hz),
        0,
        0,
        0,
        0,
        0,
    )


def arm(master):
    comp = master.target_component
    master.mav.command_long_send(
        master.target_system,
        comp,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1,
        0,
        0,
        0,
        0,
        0,
        0,
    )
    deadline = time.time() + 5
    while time.time() < deadline:
        msg = master.recv_match(type=["COMMAND_ACK", "HEARTBEAT", "STATUSTEXT"], blocking=True, timeout=1)
        if msg is None:
            continue
        if msg.get_type() == "STATUSTEXT":
            print("statustext", msg.text)
        if msg.get_type() == "COMMAND_ACK":
            print("command_ack", msg.command, msg.result)
        if msg.get_type() == "HEARTBEAT":
            armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            print("armed_heartbeat", int(armed), "mode", mavutil.mode_string_v10(msg), "base", msg.base_mode)
            if armed:
                return True
    return False


def main():
    stop_stale()
    param, runscript, state_csv, channel_map = write_files()
    procs, files = [], []
    try:
        jsb, jf = start(["JSBSim", f"--root={REPO / 'Tools/autotest'}", f"--script={runscript}", "--realtime"], RUN / "jsbsim.log")
        procs.append(jsb); files.append(jf)
        deadline = time.time() + 8
        while time.time() < deadline:
            if jsb.poll() is not None:
                raise RuntimeError("JSBSim exited early\n" + tail(RUN / "jsbsim.log"))
            if state_csv.exists() and sum(1 for _ in state_csv.open()) > 3:
                break
            time.sleep(0.1)
        else:
            raise RuntimeError("JSBSim state CSV did not start")

        # F21-K: single-source command gate now lives inside the responder
        # (see sr75_sim_json_responder.py --auto-gate-hold). This file's mere
        # existence is the release signal; touched right after MODE=AUTO is
        # confirmed below. Must not exist yet when the responder starts.
        global AUTO_GATE_RELEASE_FILE
        AUTO_GATE_RELEASE_FILE = RUN / "auto_gate_release.flag"
        if AUTO_GATE_RELEASE_FILE.exists():
            AUTO_GATE_RELEASE_FILE.unlink()

        responder_csv = RUN / "responder.csv"
        resp_cmd = [
            "python3", "Tools/autotest/sr75_hil_layer2/sim_json/sr75_sim_json_responder.py",
            "--listen-host", "127.0.0.1", "--listen-port", str(SIM_JSON_PORT),
            "--verbose",
            "--state-file", str(state_csv),
            "--state-timeout-ms", "500",
            "--rate-limit-hz", "60",
            "--fresh-state-wait-ms", "25",
            "--strict",
            "--log-csv", str(responder_csv),
            "--b3-state-envelope-guard",
            "--b3-state-envelope-abort-row", str(RUN / "envelope_abort.csv"),
            "--b3-state-envelope-rato-validation-airspeed-max-mps", "350",
            "--actuator-map", str(channel_map),
            "--actuator-timeout-ms", "250",
            "--invalid-active-pwm-neutral",
            "--jsbsim-command-target", f"udp:127.0.0.1:{JSB_PORT}",
            "--rc1-pwm", "1500", "--rc2-pwm", "1500", "--rc3-pwm", "1500", "--rc4-pwm", "1500", "--rc7-pwm", "1000",
            "--precontrol-hold", "--precontrol-elevator", str(PRECONTROL_ELEVATOR), "--precontrol-aileron", "0.0",
            "--precontrol-rudder", "0.0", "--precontrol-throttle", "0.37", "--precontrol-rato", "0.0",
            # F21-K: single-source gate -- keep sending the --precontrol-*
            # reference as the actual JSBSim command (overriding live PWM
            # decode) until AUTO_GATE_RELEASE_FILE is created below, right
            # after MODE=AUTO is confirmed via MAVLink.
            "--auto-gate-hold", "--auto-gate-release-file", str(AUTO_GATE_RELEASE_FILE),
            # startup-sync kept at its normal readiness threshold for its own
            # purpose (readiness/bias status) -- command suppression is now
            # handled independently by --auto-gate-hold above.
            "--startup-sync", "--startup-sync-required-valid-pwm-frames", "20",
            "--startup-sync-fbwa-confirmed", "--startup-sync-ahrs-ekf-type", "10",
            "--startup-sync-roll-deg", "0.0", "--startup-sync-pitch-deg", "20.0",
            "--startup-sync-yaw-deg", "315.0", "--startup-sync-altitude-m", "3000.0",
            "--startup-sync-airspeed-mps", "69.0", "--startup-sync-latitude-deg", "32.5378085",
            "--startup-sync-longitude-deg", "74.3661944",
        ]
        resp, rf = start(resp_cmd, RUN / "responder.log")
        procs.append(resp); files.append(rf)
        time.sleep(0.5)
        if resp.poll() is not None:
            raise RuntimeError("responder exited early\n" + tail(RUN / "responder.log"))

        defaults = ",".join([
            str(REPO / "Tools/autotest/default_params/plane-jsbsim.parm"),
            str(REPO / "Tools/autotest/models/sr75.parm"),
            str(REPO / "Tools/autotest/models/sr75_sim.parm"),
            str(REPO / "Tools/autotest/sr75_hil_layer2/closed_loop/SR75_LAYER2J_B3_FBWA.param"),
            str(REPO / "Tools/autotest/sr75_hil_layer2/closed_loop/SR75_LAYER2J_B3_DIRECT_SIM_JSON_VALIDATION.param"),
            str(param),
        ])
        ap, af = start([
            str(REPO / "build/sitl/bin/arduplane"),
            "--wipe",
            "--model", "json:127.0.0.1",
            "--speedup", "1",
            "--defaults", defaults,
            "--home", "32.5378085,74.3661944,3000,315",
            "--serial0", f"tcp:{MAVLINK_PORT}:wait",
        ], RUN / "arduplane.log", cwd=RUN)
        procs.append(ap); files.append(af)
        deadline = time.time() + 14
        last_error = None
        while time.time() < deadline:
            try:
                master = mavutil.mavlink_connection(
                    f"tcp:127.0.0.1:{MAVLINK_PORT}",
                    source_system=255,
                    source_component=190,
                    dialect="ardupilotmega",
                )
                break
            except (ConnectionRefusedError, OSError) as e:
                last_error = e
                time.sleep(0.2)
        else:
            raise RuntimeError(f"pymavlink could not connect TCP {MAVLINK_PORT}: {last_error}\n" + tail(RUN / "arduplane.log"))
        hb0 = master.wait_heartbeat(timeout=10)
        hb_sys = hb0.get_srcSystem()
        hb_comp = hb0.get_srcComponent()
        master.target_system = hb_sys
        master.target_component = hb_comp
        master.mav.srcSystem = 255
        master.mav.srcComponent = mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER
        print(
            "heartbeat_source",
            hb_sys,
            hb_comp,
            "gcs_source",
            master.mav.srcSystem,
            master.mav.srcComponent,
            "mode",
            mavutil.mode_string_v10(hb0),
            "base_mode",
            hb0.base_mode,
            "custom",
            hb0.custom_mode,
        )
        send_gcs_heartbeat(master)
        release_deadline = time.time() + 12
        while time.time() < release_deadline:
            send_gcs_heartbeat(master)
            # Drain status text while waiting for startup-sync release.
            msg = master.recv_match(type=["STATUSTEXT", "HEARTBEAT"], blocking=False)
            if msg is not None and msg.get_type() == "STATUSTEXT":
                print("statustext", msg.text)
            if responder_csv.exists():
                rows = read_rows(responder_csv)
                if rows and any(r["startup_sync_phase"] == "RELEASED" for r in rows):
                    break
            time.sleep(0.1)
        # Let ArduPlane finish post-JSON initialisation before arming.
        init_deadline = time.time() + 8
        while time.time() < init_deadline:
            send_gcs_heartbeat(master)
            msg = master.recv_match(type=["STATUSTEXT", "HEARTBEAT"], blocking=True, timeout=0.5)
            if msg is not None and msg.get_type() == "STATUSTEXT":
                print("statustext", msg.text)
                if "ArduPilot Ready" in msg.text:
                    break
        hb_deadline = time.time() + 8
        while time.time() < hb_deadline:
            send_gcs_heartbeat(master)
            hb = master.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
            if hb is not None:
                mode = mavutil.mode_string_v10(hb)
                print("ready_mode_seen", mode)
                if mode != "INITIALISING":
                    break
        tx_ok = prove_tx(master)
        print("tx_ok", tx_ok)
        if not tx_ok:
            raise RuntimeError("MAVLink TX proof failed")
        for pname in (b"SERIAL0_PROTOCOL", b"SERIAL0_BAUD"):
            master.mav.param_request_read_send(master.target_system, master.target_component, pname, -1)
            deadline = time.time() + 4
            while time.time() < deadline:
                send_gcs_heartbeat(master)
                msg = master.recv_match(type=["PARAM_VALUE", "STATUSTEXT"], blocking=True, timeout=0.5)
                if msg is None:
                    continue
                if msg.get_type() == "STATUSTEXT":
                    print("statustext", msg.text)
                    continue
                pid = msg.param_id
                if isinstance(pid, bytes):
                    pid = pid.decode("ascii", errors="replace").rstrip("\x00")
                print("param_value", pid, msg.param_value)
                if pid == pname.decode("ascii"):
                    break
        # F21-D: the fixed 20s post-check liveness monitor that used to run here
        # only drained STATUSTEXT/HEARTBEAT and printed a diagnostic summary --
        # it gated nothing. It was the delay that let the aircraft sit in FBWA
        # (with no TECS speed/altitude protection) for ~20s before arm+AUTO
        # were ever commanded. Removed so arm()+set_mode(AUTO) run immediately
        # after startup sync, param and safety checks above, right before RATO
        # ignition (which itself only starts once AUTO+TAKEOFF verification is
        # active, per RATOController::update() gating in commands_logic.cpp).
        if resp.poll() is not None:
            raise RuntimeError(f"responder exited rc={resp.returncode}\n" + tail(RUN / "responder.log"))
        if jsb.poll() is not None:
            raise RuntimeError(f"JSBSim exited rc={jsb.returncode}\n" + tail(RUN / "jsbsim.log"))
        mission_ok = upload_takeoff(master)
        print("mission_ok", mission_ok)
        print("post_upload_target", master.target_system, master.target_component)
        armed_ok = arm(master)
        print("armed", armed_ok)
        auto_ok = set_mode(master, "AUTO")
        print("mode_auto", auto_ok)
        # F21-K: MODE=AUTO confirmed (set_mode polls HEARTBEAT until the mode
        # matches before returning) -- release the responder's single-source
        # command gate now. Atomic create-then-rename avoids the responder
        # ever observing a partially-written file.
        auto_gate_tmp = AUTO_GATE_RELEASE_FILE.with_suffix(".tmp")
        auto_gate_tmp.write_text(f"AUTO confirmed host_time={time.time():.9f}\n")
        auto_gate_tmp.replace(AUTO_GATE_RELEASE_FILE)
        print("auto_gate_release_file_created", str(AUTO_GATE_RELEASE_FILE))

        for msg_id in (
            mavutil.mavlink.MAVLINK_MSG_ID_MISSION_CURRENT,
            mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW,
            mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS,
        ):
            request_msg(master, msg_id, 5)

        end = time.time() + 75
        first_rato_source_ts = None
        first_eject_source_ts = None
        while time.time() < end:
            send_gcs_heartbeat(master)
            msg = master.recv_match(type=["STATUSTEXT", "MISSION_CURRENT", "SERVO_OUTPUT_RAW", "HEARTBEAT"], blocking=True, timeout=0.2)
            if msg is None:
                continue
            if msg.get_type() == "STATUSTEXT":
                print("statustext", msg.text)
            elif msg.get_type() == "MISSION_CURRENT":
                print("mission_current", msg.seq)
            if responder_csv.exists():
                rr_now = read_rows(responder_csv)
                rato_active_rows = [
                    r for r in rr_now
                    if r.get("act_rato_norm") not in ("", None) and float(r["act_rato_norm"]) > 0.5 and r.get("source_simulation_timestamp")
                ]
                if rato_active_rows and first_rato_source_ts is None:
                    first_rato_source_ts = float(rato_active_rows[0]["source_simulation_timestamp"])
                    print("rato_first_source_ts", f"{first_rato_source_ts:.3f}")
                eject_rows = [
                    r for r in rr_now
                    if r.get("act_eject_norm") not in ("", None)
                    and float(r["act_eject_norm"]) > 0.5
                    and r.get("source_simulation_timestamp")
                ]
                if eject_rows and first_eject_source_ts is None:
                    first_eject_source_ts = float(eject_rows[0]["source_simulation_timestamp"])
                    print("eject_first_source_ts", f"{first_eject_source_ts:.3f}")
                if first_rato_source_ts is not None:
                    last_ts = max(float(r["source_simulation_timestamp"]) for r in rr_now if r.get("source_simulation_timestamp"))
                    if first_eject_source_ts is not None and last_ts >= first_eject_source_ts + 30.0:
                        break

        rows = read_rows(state_csv)
        rr = read_rows(responder_csv)
        kg = lambda lb: lb * 0.45359237
        newton = lambda lb: lb * 4.448221615
        thrust_rows = [r for r in rows if newton(fval(r, "propulsion/engine[2]/thrust-lbs")) > 100]
        rato_rows = [r for r in rr if r.get("act_rato_norm") not in ("", None) and float(r["act_rato_norm"]) > 0.5]
        t0 = fval(thrust_rows[0], "simulation/sim-time-sec") if thrust_rows else math.nan
        t1 = fval(thrust_rows[-1], "simulation/sim-time-sec") if thrust_rows else math.nan
        print("release_request", next((r["request_count"] for r in rr if r["startup_sync_phase"] == "RELEASED"), ""))
        print("readiness", next((r["startup_sync_readiness_reason"] for r in rr if r["startup_sync_phase"] == "RELEASED"), ""))
        print("act_rato_max", max([float(r["act_rato_norm"]) for r in rr if r.get("act_rato_norm") not in ("", None)] or [0.0]))
        print("pwm7_min_max", min([int(r["pwm7"]) for r in rr if r.get("pwm7")] or [0]), max([int(r["pwm7"]) for r in rr if r.get("pwm7")] or [0]))
        print("act_rato_rows", len(rato_rows))
        print("thrust_duration", t1 - t0 if thrust_rows else math.nan)
        print("thrust_max_N", max([newton(fval(r, "propulsion/engine[2]/thrust-lbs")) for r in rows] or [0.0]))
        print("fuel_initial_final_kg", kg(fval(rows[0], "propulsion/tank[0]/contents-lbs")), kg(fval(rows[-1], "propulsion/tank[0]/contents-lbs")))
        print("prop_initial_final_kg", kg(fval(rows[0], "propulsion/tank[1]/contents-lbs")), kg(fval(rows[-1], "propulsion/tank[1]/contents-lbs")))
        print("dry_initial_final_kg", dry_kg(rows[0]), dry_kg(rows[-1]))
        print("stale_max", max([float(r["act_stale"]) for r in rr if r.get("act_stale") not in ("", None)] or [0.0]))
        print("initialization_proof",
              "prop_kg", f"{kg(fval(rows[0], 'propulsion/tank[1]/contents-lbs')):.3f}",
              "dry_kg", f"{dry_kg(rows[0]):.3f}",
              "eject_cmd", "responder_act_eject",
              "engine2_running", f"{fval(rows[0], 'propulsion/engine[2]/set-running'):.0f}")
        first_rato = rato_rows[0] if rato_rows else None
        first_rato_t = float(first_rato["source_simulation_timestamp"]) if first_rato else math.nan
        def near_rr_for_sim(simt):
            return min(rr, key=lambda x: abs(float(x["source_simulation_timestamp"] or 0.0) - simt))
        def first_after(label, predicate):
            for r in rows:
                simt = fval(r, "simulation/sim-time-sec")
                if not math.isfinite(first_rato_t) or simt < first_rato_t:
                    continue
                near = near_rr_for_sim(simt)
                if predicate(r, near):
                    print(
                        label,
                        "t", f"{simt:.3f}",
                        "dt_rato", f"{simt - first_rato_t:.3f}",
                        "pwm7", near.get("pwm7"),
                        "act_rato", near.get("act_rato_norm"),
                        "thrustN", f"{newton(fval(r, 'propulsion/engine[2]/thrust-lbs')):.1f}",
                        "propkg", f"{kg(fval(r, 'propulsion/tank[1]/contents-lbs')):.3f}",
                        "drykg", f"{dry_kg(r):.3f}",
                        "tas", f"{fval(r, 'velocities/vt-fps') * 0.3048:.2f}",
                        "agl_m", f"{fval(r, 'position/h-agl-ft') * 0.3048:.2f}",
                    )
                    return r
            print(label, "not_found")
            return None
        first_after("first_pwm7_low_after_rato", lambda r, near: near.get("pwm7") and int(near["pwm7"]) < 1800)
        first_after("first_act_rato_low_after_rato", lambda r, near: near.get("act_rato_norm") not in ("", None) and float(near["act_rato_norm"]) < 0.5)
        first_after("first_thrust_low_after_rato", lambda r, near: newton(fval(r, "propulsion/engine[2]/thrust-lbs")) < 100.0)
        first_after("first_prop_empty", lambda r, near: kg(fval(r, "propulsion/tank[1]/contents-lbs")) <= 0.05)
        dry_loss_row = first_after("first_dry_mass_lost", lambda r, near: dry_kg(r) < 9.0)
        print("timeline")
        if thrust_rows:
            for dt in [0, 0.5, 1, 2, 2.5, 3, 3.5, 4, 5, 6]:
                target = t0 + dt
                r = min(rows, key=lambda x: abs(fval(x, "simulation/sim-time-sec") - target))
                simt = fval(r, "simulation/sim-time-sec")
                near = min(rr, key=lambda x: abs(float(x["source_simulation_timestamp"] or 0.0) - simt))
                print(
                    f"dt={dt:.1f} t={simt:.3f} pwm7={near['pwm7']} act_rato={near['act_rato_norm']} "
                    f"thrustN={newton(fval(r,'propulsion/engine[2]/thrust-lbs')):.1f} "
                    f"propkg={kg(fval(r,'propulsion/tank[1]/contents-lbs')):.3f} drykg={dry_kg(r):.3f} "
                    f"pitch={fval(r,'attitude/theta-deg'):.2f} q={fval(r,'velocities/q-rad_sec'):.3f}"
                )
        if dry_loss_row is not None:
            eject_t = fval(dry_loss_row, "simulation/sim-time-sec")
            pre_rows = [r for r in rows if fval(r, "simulation/sim-time-sec") < eject_t]
            post_rows = [r for r in rows if fval(r, "simulation/sim-time-sec") >= eject_t]
            before = pre_rows[-1] if pre_rows else dry_loss_row
            after = dry_loss_row
            post10 = [
                r for r in rows
                if eject_t <= fval(r, "simulation/sim-time-sec") <= eject_t + 10.0
            ]
            rr_post = [
                r for r in rr
                if r.get("source_simulation_timestamp")
                and eject_t <= float(r["source_simulation_timestamp"]) <= eject_t + 10.0
            ]
            dry_events = []
            prev = dry_lbs(rows[0])
            for r in rows[1:]:
                curr = dry_lbs(r)
                if abs(curr - prev) > 1.0:
                    dry_events.append((fval(r, "simulation/sim-time-sec"), prev, curr, curr - prev))
                prev = curr
            mass_events = []
            prev = fval(rows[0], "inertia/weight-lbs")
            for r in rows[1:]:
                curr = fval(r, "inertia/weight-lbs")
                if abs(curr - prev) > 1.0:
                    mass_events.append((fval(r, "simulation/sim-time-sec"), prev, curr, curr - prev))
                prev = curr
            act_eject_rows = [
                r for r in rr
                if r.get("act_eject_norm") not in ("", None) and float(r["act_eject_norm"]) > 0.5
            ]
            pwm8_rows = [r for r in rr if r.get("pwm8") and int(r["pwm8"]) > 1800]
            sat_rows = [
                r for r in rr_post
                if abs(float(r.get("act_left_elevon_norm") or 0.0)) >= 0.999
                or abs(float(r.get("act_right_elevon_norm") or 0.0)) >= 0.999
            ]
            state_errors = [r for r in rr if r.get("error_reason")]
            print("D2_SUMMARY_BEGIN")
            print(
                "eject_event",
                f"t={eject_t:.3f}",
                f"pwm8_start={float(pwm8_rows[0]['source_simulation_timestamp']):.3f}" if pwm8_rows else "pwm8_start=nan",
                f"pwm8_end={float(pwm8_rows[-1]['source_simulation_timestamp']):.3f}" if pwm8_rows else "pwm8_end=nan",
                f"act_eject_start={float(act_eject_rows[0]['source_simulation_timestamp']):.3f}" if act_eject_rows else "act_eject_start=nan",
                f"act_eject_end={float(act_eject_rows[-1]['source_simulation_timestamp']):.3f}" if act_eject_rows else "act_eject_end=nan",
            )
            for label, r in (("before", before), ("after", after)):
                print(
                    "mass_state", label,
                    f"t={fval(r, 'simulation/sim-time-sec'):.3f}",
                    f"fuel_kg={kg(fval(r, 'propulsion/tank[0]/contents-lbs')):.6f}",
                    f"prop_kg={kg(fval(r, 'propulsion/tank[1]/contents-lbs')):.6f}",
                    f"dry_kg={dry_kg(r):.6f}",
                    f"total_kg={kg(fval(r, 'inertia/weight-lbs')):.6f}",
                    f"cg_x_in={fval(r, 'inertia/cg-x-in'):.6f}",
                    f"cg_y_in={fval(r, 'inertia/cg-y-in'):.6f}",
                    f"cg_z_in={fval(r, 'inertia/cg-z-in'):.6f}",
                    f"ixx={fval(r, 'inertia/ixx-slugs_ft2'):.6f}",
                    f"iyy={fval(r, 'inertia/iyy-slugs_ft2'):.6f}",
                    f"izz={fval(r, 'inertia/izz-slugs_ft2'):.6f}",
                )
            print(
                "mass_delta",
                f"dry_kg={dry_kg(after) - dry_kg(before):.6f}",
                f"total_kg={kg(fval(after, 'inertia/weight-lbs') - fval(before, 'inertia/weight-lbs')):.6f}",
                f"fuel_kg={kg(fval(after, 'propulsion/tank[0]/contents-lbs') - fval(before, 'propulsion/tank[0]/contents-lbs')):.6f}",
                f"prop_kg={kg(fval(after, 'propulsion/tank[1]/contents-lbs') - fval(before, 'propulsion/tank[1]/contents-lbs')):.6f}",
            )
            print("dry_events", dry_events)
            print("mass_events", mass_events)
            if post10:
                print(
                    "post10_dynamics",
                    f"pitch_min={min(fval(r, 'attitude/theta-deg') for r in post10):.3f}",
                    f"pitch_max={max(fval(r, 'attitude/theta-deg') for r in post10):.3f}",
                    f"roll_abs_max={max(abs(fval(r, 'attitude/phi-deg')) for r in post10):.3f}",
                    f"q_abs_max={max(abs(fval(r, 'velocities/q-rad_sec')) for r in post10):.6f}",
                    f"tas_min={min(fval(r, 'velocities/vt-fps') * 0.3048 for r in post10):.3f}",
                    f"tas_max={max(fval(r, 'velocities/vt-fps') * 0.3048 for r in post10):.3f}",
                    f"alt_min={min(fval(r, 'position/h-sl-ft') * 0.3048 for r in post10):.3f}",
                    f"alt_max={max(fval(r, 'position/h-sl-ft') * 0.3048 for r in post10):.3f}",
                )
            print(
                "counts",
                f"stale_max={max([float(r['act_stale']) for r in rr if r.get('act_stale') not in ('', None)] or [0.0]):.0f}",
                f"state_error_count={len(state_errors)}",
                f"surface_sat_post10_count={len(sat_rows)}",
                f"responder_rows={len(rr)}",
            )
            print("D2_SUMMARY_END")
    except Exception as e:
        print(f"F6_RUN_ERROR: {e}", file=sys.stderr)
        for name in ("jsbsim.log", "responder.log", "arduplane.log"):
            print(f"--- {name} ---", file=sys.stderr)
            print(tail(RUN / name), file=sys.stderr)
        return 2
    finally:
        for p in reversed(procs):
            if p.poll() is None:
                try:
                    os.killpg(p.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
        time.sleep(0.5)
        for p in reversed(procs):
            if p.poll() is None:
                try:
                    os.killpg(p.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
        for f in files:
            f.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
