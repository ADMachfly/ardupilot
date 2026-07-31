#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""F22-GZ-AH: preserve Gazebo release velocity while eliminating pre-AUTO fall.

F22-GZ-AG eliminated the pre-AUTO pitch collapse using JSBSim's
forces/hold-down restraint, but hold-down also drives reported velocity to
~0 while engaged -- so BOOST resumed from ~27 m/s instead of the intended
43.85 m/s release speed, not a clean reproduction of the actual Gazebo
handoff state.

This harness instead uses option A from the task ("delay JSBSim/release-IC
start until after ArduPlane heartbeat, params, mission, AUTO are ready"):
JSBSim is not launched at all until ArduPlane is armed and AUTO has been
requested. During the entire boot sequence, ArduPlane only ever sees the
existing --startup-sync synthetic held_reply_state() (B3StartupSync,
sr75_sim_json_responder.py), which is generated purely from the configured
target parameters (roll=0, pitch=20, yaw=315, altitude=3000m,
airspeed=43.85 m/s -- the exact release IC) with NO live JSBSim connection
required at all. --startup-sync-required-valid-pwm-frames is set absurdly
high so the responder's normal early-release heuristic never fires; a new
--startup-sync-force-release-file flag (B3StartupSync.force_release(), a
new small method added in this task) instead switches ArduPlane over to
genuine JSBSim state at the exact external moment this harness chooses --
right after the freshly-launched, --realtime JSBSim process's state CSV has
real rows. Because JSBSim's own script/IC (f22gzo_release_state_init.xml)
sets theta=20deg and ubody=43.85 m/s directly, and it has had zero elapsed
wall-clock time to drift before this release, both attitude AND velocity
are genuinely at the intended release IC when ArduPlane first sees real
data -- unlike hold-down, no property destroys velocity, and unlike
simulation/pause or simulation/reset (both found to deadlock JSBSim's
executive in F22-GZ-AG), nothing needs to be paused or reset mid-run.

The existing --precontrol-burn-duration-s burn/postburn elevator-throttle
switch (F22-GZ-AA) is re-anchored to this same force-release moment
instead of responder-process-start (a small change in
sr75_sim_json_responder.py), since the real burn only begins once JSBSim
is actually running. Everything else matches F22-GZ-AE/AG: the same
proven precontrol schedule, the same --auto-gate-live-rato-eject hybrid
so ArduPlane's own RATOController owns RATO ignition/burn/eject timing,
single command source throughout, auto-gate release file never created.
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
RUN = Path(os.environ.get("SR75_D3_RUN", "/tmp/sr75_f22gzah_resume"))
MISSION_FILE = REPO / "Tools/autotest/sr75_hil_layer2/closed_loop/SR75_LAYER2D_F21_AUTO_TAKEOFF.waypoints"
AUTO_GATE_RELEASE_FILE = None  # set in main() to RUN / "auto_gate_release.flag" -- intentionally never created
FORCE_RELEASE_FILE = None      # set in main() to RUN / "startup_sync_force_release.flag"
IC_NAME = "f22gzo_release_state_init"
CASE = "f22gzah_resume_preserved"
JSB_PORT = 5609
SIM_JSON_PORT = 9002
MAVLINK_PORT = 5779
HOME_LAT = 32.5378147
HOME_LON = 74.3661871
HOME_ALT = 3000

# Pre-AUTO precontrol hold: same proven values as F22-GZ-T/U/V/AA/AE/AG for
# this exact IC/speed. Only matters from the moment JSBSim actually
# launches (mid-boot, no real JSBSim exists before that in this design).
PRECONTROL_ELEVATOR = 0.10
PRECONTROL_THROTTLE = 0.37
PROP_REMAINING_LBS = 15.057396137417392  # 9.18kg * (2.232/3.0)
DRY_BOOSTER_LBS = 22.046226               # 10.0kg
RATO_RES_BURN_S = 0.768
RATO_RES_BURN_S_REMAINING = 3.0 - RATO_RES_BURN_S  # 2.232s, matches ArduPlane's own RATO_BURN_S - RATO_RES_BURN


def shell_quiet(cmd):
    subprocess.run(cmd, cwd=REPO, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def stop_stale():
    shell_quiet(["Tools/autotest/sr75_hil_layer2/closed_loop/stop_sr75_b3.sh"])
    for pattern in (
        "sr75_sim_json_responder.py",
        f"JSBSim.*{CASE}.xml",
        f"build/sitl/bin/arduplane.*{MAVLINK_PORT}",
    ):
        subprocess.run(["pkill", "-f", pattern], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(0.5)


def write_files():
    RUN.mkdir(parents=True, exist_ok=True)
    param = RUN / f"{CASE}.param"
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
                # F22-GZ-AD: SITL/test-only mid-BOOST resume (F22-GZ-AC).
                "RATO_RESUME 1",
                f"RATO_RES_BURN {RATO_RES_BURN_S}",
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
    runscript = RUN / f"{CASE}.xml"
    rel_output = f"../../../../../tmp/{RUN.name}/sr75_b3_state_{CASE}.csv"

    runscript.write_text(
        f"""<?xml version="1.0" encoding="utf-8"?>
<runscript xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
    xsi:noNamespaceSchemaLocation="http://jsbsim.sf.net/JSBSimScript.xsd"
    name="F22-GZ-AH ArduPlane-owned RATO resume, delayed JSBSim launch">

  <use aircraft="sr_75_6_dof" initialize="{IC_NAME}"/>

  <input type="QTJSBSIM" port="{JSB_PORT}" rate="60">
    <property>fcs/elevator-cmd-norm</property>
    <property>fcs/aileron-cmd-norm</property>
    <property>fcs/rudder-cmd-norm</property>
    <property>fcs/turbojet-throttle-cmd-norm</property>
    <property>fcs/rato-throttle-cmd-norm</property>
    <property>propulsion/tank[2]/contents-lbs</property>
  </input>

  <run start="0.0" end="30.0" dt="0.008333">
    <event name="init_partial_burn">
      <condition>simulation/sim-time-sec ge 0.0</condition>
      <set name="propulsion/engine[0]/set-running" value="1"/>
      <set name="propulsion/engine[1]/set-running" value="1"/>
      <set name="propulsion/engine[2]/set-running" value="1"/>
      <set name="propulsion/tank[1]/contents-lbs" value="{PROP_REMAINING_LBS}"/>
      <set name="propulsion/tank[2]/contents-lbs" value="{DRY_BOOSTER_LBS}"/>
      <set name="fcs/elevator-cmd-norm" value="{PRECONTROL_ELEVATOR}"/>
      <set name="fcs/turbojet-throttle-cmd-norm" value="{PRECONTROL_THROTTLE}"/>
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
    channel_map.write_text(source_map.read_text())
    return param, runscript, state_csv, channel_map


def start(cmd, log, cwd=REPO):
    f = open(log, "w")
    p = subprocess.Popen(cmd, cwd=cwd, stdout=f, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, start_new_session=True)
    return p, f


def read_rows(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def tail(path, n=80):
    try:
        return "\n".join(Path(path).read_text(errors="replace").splitlines()[-n:])
    except FileNotFoundError:
        return ""


def upload_takeoff(master):
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
    return final_ack == mavutil.mavlink.MAV_MISSION_ACCEPTED


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
        global AUTO_GATE_RELEASE_FILE, FORCE_RELEASE_FILE
        AUTO_GATE_RELEASE_FILE = RUN / "auto_gate_release.flag"
        if AUTO_GATE_RELEASE_FILE.exists():
            AUTO_GATE_RELEASE_FILE.unlink()
        FORCE_RELEASE_FILE = RUN / "startup_sync_force_release.flag"
        if FORCE_RELEASE_FILE.exists():
            FORCE_RELEASE_FILE.unlink()

        # F22-GZ-AH: responder launches FIRST, well before JSBSim exists.
        # It services ArduPlane's ENTIRE boot via the synthetic
        # startup_sync held_reply_state() (exactly the release IC: pitch
        # 20deg, airspeed 43.85 m/s), no live JSBSim connection needed.
        # --startup-sync-required-valid-pwm-frames is absurdly high so the
        # normal early-release heuristic never fires within this run;
        # --startup-sync-force-release-file is the only thing that ever
        # releases it, written by this harness once JSBSim is actually up.
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
            "--precontrol-rudder", "0.0", "--precontrol-throttle", str(PRECONTROL_THROTTLE), "--precontrol-rato", "0.0",
            "--precontrol-burn-duration-s", str(RATO_RES_BURN_S_REMAINING),
            "--precontrol-postburn-elevator", "0.0",
            "--precontrol-postburn-throttle", str(PRECONTROL_THROTTLE),
            "--precontrol-postburn-rato", "0.0",
            "--precontrol-postburn-eject", "0.0",
            "--auto-gate-hold", "--auto-gate-live-rato-eject",
            "--auto-gate-release-file", str(AUTO_GATE_RELEASE_FILE),
            "--startup-sync", "--startup-sync-required-valid-pwm-frames", "1000000000",
            "--startup-sync-fbwa-confirmed", "--startup-sync-ahrs-ekf-type", "10",
            "--startup-sync-roll-deg", "0.0", "--startup-sync-pitch-deg", "20.0",
            "--startup-sync-yaw-deg", "315.0", "--startup-sync-altitude-m", str(HOME_ALT),
            "--startup-sync-airspeed-mps", "43.85", "--startup-sync-latitude-deg", str(HOME_LAT),
            "--startup-sync-longitude-deg", str(HOME_LON),
            "--startup-sync-force-release-file", str(FORCE_RELEASE_FILE),
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
            "--home", f"{HOME_LAT},{HOME_LON},{HOME_ALT},315",
            "--serial0", f"tcp:{MAVLINK_PORT}:wait",
        ], RUN / "arduplane.log", cwd=RUN)
        procs.append(ap); files.append(af)
        deadline = time.time() + 14
        last_error = None
        master = None
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
        if master is None:
            raise RuntimeError(f"pymavlink could not connect TCP {MAVLINK_PORT}: {last_error}\n" + tail(RUN / "arduplane.log"))
        hb0 = master.wait_heartbeat(timeout=10)
        if hb0 is None:
            raise RuntimeError("no heartbeat\n" + tail(RUN / "arduplane.log"))
        hb_sys = hb0.get_srcSystem()
        hb_comp = hb0.get_srcComponent()
        master.target_system = hb_sys
        master.target_component = hb_comp
        master.mav.srcSystem = 255
        master.mav.srcComponent = mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER
        print("heartbeat_source", hb_sys, hb_comp, "mode", mavutil.mode_string_v10(hb0))
        send_gcs_heartbeat(master)

        # This entire wait sequence runs against the SYNTHETIC startup_sync
        # held state -- no real JSBSim exists yet. Unlike AE/AG, startup_sync
        # never naturally reaches RELEASED here (required_valid_pwm_frames is
        # absurdly high), so this is just a short fixed settle window for
        # PWM/heartbeat flow to stabilize before moving on.
        settle_deadline = time.time() + 2.0
        while time.time() < settle_deadline:
            send_gcs_heartbeat(master)
            msg = master.recv_match(type=["STATUSTEXT", "HEARTBEAT"], blocking=False)
            if msg is not None and msg.get_type() == "STATUSTEXT":
                print("statustext", msg.text)
            time.sleep(0.1)
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

        if resp.poll() is not None:
            raise RuntimeError(f"responder exited rc={resp.returncode}\n" + tail(RUN / "responder.log"))

        mission_ok = upload_takeoff(master)
        print("mission_ok", mission_ok)
        armed_ok = arm(master)
        print("armed", armed_ok)

        # F22-GZ-AH: NOW launch the real, release-state-IC JSBSim -- as
        # late as possible, right before requesting AUTO, so it has had
        # zero elapsed wall-clock time to drift from the IC (theta=20deg,
        # ubody=43.85 m/s) by the time it's released to ArduPlane.
        jsb, jf = start(["JSBSim", f"--root={REPO / 'Tools/autotest'}", f"--script={runscript}", "--realtime"], RUN / "jsbsim.log")
        procs.append(jsb); files.append(jf)

        # Request AUTO concurrently with JSBSim's own boot latency (rather
        # than serially after it), to minimize the total elapsed time
        # before force-release.
        auto_ok = set_mode(master, "AUTO")
        print("mode_auto", auto_ok)
        print("auto_gate_release_file_intentionally_not_created", str(AUTO_GATE_RELEASE_FILE))

        jsb_ready_deadline = time.time() + 10
        jsb_ready = False
        while time.time() < jsb_ready_deadline:
            if jsb.poll() is not None:
                raise RuntimeError("JSBSim exited early\n" + tail(RUN / "jsbsim.log"))
            if state_csv.exists() and sum(1 for _ in state_csv.open()) > 3:
                jsb_ready = True
                break
            time.sleep(0.02)
        if not jsb_ready:
            raise RuntimeError("JSBSim state CSV did not start\n" + tail(RUN / "jsbsim.log"))
        print("jsbsim_ready", str(state_csv))

        # F22-GZ-AH: release the instant JSBSim is confirmed up -- this is
        # the ONLY thing that switches ArduPlane from the synthetic held
        # state to genuine JSBSim state, and it also re-anchors the
        # responder's --precontrol-burn-duration-s clock to this exact
        # moment (see sr75_sim_json_responder.py's force_release()).
        FORCE_RELEASE_FILE.write_text(f"jsbsim_ready host_time={time.time():.9f}\n")
        print("force_release_file_created", str(FORCE_RELEASE_FILE))

        end = time.time() + 30
        while time.time() < end:
            send_gcs_heartbeat(master)
            msg = master.recv_match(type=["STATUSTEXT", "HEARTBEAT"], blocking=True, timeout=0.3)
            if msg is not None and msg.get_type() == "STATUSTEXT":
                print("statustext", msg.text)

        print("RUN_WINDOW_END")

        rows = read_rows(state_csv)
        kg = lambda lb: lb * 0.45359237
        newton = lambda lb: lb * 4.448221615
        MPS = 0.3048

        def g(row, name):
            for k in row:
                if k.endswith(name):
                    try:
                        return float(row[k])
                    except ValueError:
                        return None
            return None

        alt0 = None
        burnout_row = None
        eject_row = None
        prev_e2 = None
        prev_dry = None
        max_theta = -1e9
        min_theta = 1e9
        max_q = 0.0
        max_engine_diff = 0.0
        snapshots = {}
        first_theta = None
        first_tas_mps = None
        for row in rows:
            t = g(row, "sim-time-sec")
            h = g(row, "h-sl-ft")
            theta = g(row, "theta-deg")
            vt = g(row, "vt-fps")
            q = g(row, "q-rad_sec")
            e0 = g(row, "engine/thrust-lbs") or 0.0
            e1 = g(row, "engine[1]/thrust-lbs") or 0.0
            e2 = g(row, "engine[2]/thrust-lbs") or 0.0
            dry = g(row, "tank[2]/contents-lbs")
            if alt0 is None:
                alt0 = h
            if first_theta is None and theta is not None:
                first_theta = theta
            if first_tas_mps is None and vt is not None:
                first_tas_mps = vt * MPS
            if theta is not None:
                max_theta = max(max_theta, theta)
                min_theta = min(min_theta, theta)
            if q is not None:
                max_q = max(max_q, abs(q))
            max_engine_diff = max(max_engine_diff, abs(e0 - e1))
            if burnout_row is None and prev_e2 is not None and prev_e2 > 10 and e2 < 10:
                burnout_row = (t, theta, vt, h)
            if eject_row is None and prev_dry is not None and prev_dry > 0.01 and dry is not None and dry <= 0.01:
                eject_row = (t, dry * kg(1.0), prev_dry * kg(1.0))
            prev_e2 = e2
            prev_dry = dry
            for snap_t in (5.0, 10.0, 15.0, 20.0, 30.0):
                key = f"t{int(snap_t)}"
                if key not in snapshots and t is not None and t >= snap_t:
                    snapshots[key] = (theta, (h - alt0) * MPS, vt * MPS, e0 * newton(1.0), e1 * newton(1.0))

        rato_state_lines = [ln for ln in tail(RUN / "arduplane.log", 4000).splitlines() if "RATO state=" in ln or "RATO: resume" in ln or "RATO ign" in ln or "RATO abort" in ln or "RATO:" in ln]
        print("RATO_LOG_SAMPLE_FIRST_10:")
        for ln in rato_state_lines[:10]:
            print(" ", ln)
        print("RATO_LOG_SAMPLE_LAST_10:")
        for ln in rato_state_lines[-10:]:
            print(" ", ln)

        print(f"JSBSIM_FIRST_ROW theta={first_theta} tas_mps={first_tas_mps}")

        if burnout_row:
            bt, btheta, bvt, bh = burnout_row
            print(f"BURNOUT t={bt:.3f}s theta={btheta:.2f} TAS={bvt*MPS:.2f} m/s alt_gain={(bh-alt0)*MPS:.2f} m")
        else:
            print("BURNOUT not_detected")
        if eject_row:
            et, dry_after, dry_before = eject_row
            print(f"EJECT   t={et:.3f}s dry_before={dry_before:.3f}kg dry_after={dry_after:.3f}kg")
        else:
            print("EJECT not_detected")
        print(f"max_theta={max_theta:.2f} min_theta={min_theta:.2f} max_q={max_q:.3f} rad/s")
        print(f"max |engine[0]-engine[1]| thrust diff over full run: {max_engine_diff*newton(1.0):.4f} N")
        print(f"{'t':>4} {'theta':>8} {'alt_m':>9} {'tas':>8} {'e0_N':>9} {'e1_N':>9}")
        for key in ("t5", "t10", "t15", "t20", "t30"):
            if key in snapshots:
                th, alt, tas, e0n, e1n = snapshots[key]
                print(f"{key:>4} {th:8.2f} {alt:9.2f} {tas:8.2f} {e0n:9.1f} {e1n:9.1f}")

    except Exception as e:
        print(f"F22GZAH_RUN_ERROR: {e}", file=sys.stderr)
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
