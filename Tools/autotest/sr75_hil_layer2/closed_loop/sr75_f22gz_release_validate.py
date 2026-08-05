#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""F22-GZ-L: short validation of JSBSim/ArduPilot from the Gazebo
detachable-rail release-state IC (f22gz_release_state_init.xml).

This is NOT a full mission run. It starts JSBSim + sr75_sim_json_responder.py
+ a real ArduPlane SITL binary, waits for boot, and then just holds a fixed
precontrol reference (no arm, no mission upload, no mode change) for ~10s,
logging pitch/heading/TAS so the IC's numerical sanity can be checked before
ever bringing AUTO/RATO into the loop. RATO cannot fire here regardless
(RATOController.update() is only ever called from Plane::verify_takeoff()
while executing a NAV_TAKEOFF item in AUTO -- see ArduPlane/commands_logic.cpp
and rato.cpp), so "RATO/AUTO timing behavior" for this validation is simply
confirming RATO stays untouched (mode never leaves the FBWA boot default).
"""
import csv
import os
import subprocess
import sys
import time
from pathlib import Path

os.environ["MAVLINK20"] = "1"
from pymavlink import mavutil

REPO = Path(__file__).resolve().parents[4]
RUN = Path(os.environ.get("SR75_D3_RUN", "/tmp/sr75_f22gzl_validate"))
# F22-GZ-N: overridable so this same harness can validate other IC
# variants (e.g. f22gzn_release_state_init for the 45 m/s target) without
# duplicating the script.
IC_NAME = os.environ.get("SR75_F22GZ_IC_NAME", "f22gz_release_state_init")
PRECONTROL_ELEVATOR = -0.49
PRECONTROL_THROTTLE = 0.0
CASE = os.environ.get("SR75_F22GZ_CASE", "f22gz_release_validate")
JSB_PORT = int(os.environ.get("SR75_F22GZ_JSB_PORT", "5602"))
SIM_JSON_PORT = 9002
MAVLINK_PORT = int(os.environ.get("SR75_F22GZ_MAVLINK_PORT", "5772"))


def shell_quiet(cmd):
    subprocess.run(cmd, cwd=REPO, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def stop_stale():
    shell_quiet(["Tools/autotest/sr75_hil_layer2/closed_loop/stop_sr75_b3.sh"])
    for pattern in (
        "sr75_sim_json_responder.py",
        "JSBSim.*sr75_f22gz_release_validate.xml",
        f"build/sitl/bin/arduplane.*{MAVLINK_PORT}",
    ):
        subprocess.run(["pkill", "-f", pattern], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(0.5)


def write_files():
    RUN.mkdir(parents=True, exist_ok=True)
    param = RUN / "f22gzl.param"
    param.write_text(
        "\n".join([
            "AHRS_EKF_TYPE 10",
            "STAB_PITCH_DOWN 0",
            "RC2_TRIM 1500",
            "ARSPD_SKIP_CAL 1",
            "ARSPD_OFFSET 0",
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
        ])
    )
    state_csv = RUN / f"sr75_b3_state_{CASE}.csv"
    runscript = RUN / "sr75_f22gz_release_validate.xml"
    rel_output = f"../../../../../tmp/{RUN.name}/sr75_b3_state_{CASE}.csv"
    runscript.write_text(f"""<?xml version="1.0" encoding="utf-8"?>
<runscript xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
    xsi:noNamespaceSchemaLocation="http://jsbsim.sf.net/JSBSimScript.xsd"
    name="F22-GZ-L Gazebo release-state IC validation">

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
    <event name="Set propulsion running and controls trimmed">
      <condition>simulation/sim-time-sec ge 0.0</condition>
      <set name="propulsion/engine[0]/set-running" value="1"/>
      <set name="propulsion/engine[1]/set-running" value="1"/>
      <set name="propulsion/engine[2]/set-running" value="1"/>
      <set name="propulsion/tank[1]/contents-lbs" value="20.238414"/>
      <set name="propulsion/tank[2]/contents-lbs" value="22.046226"/>
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
""")
    channel_map = RUN / "channel_map.json"
    source_map = REPO / "Tools/autotest/sr75_hil_layer2/closed_loop/SR75_LAYER2J_B3_CHANNEL_MAP.json"
    channel_map.write_text(source_map.read_text())
    return param, runscript, state_csv, channel_map


def start(cmd, log, cwd=REPO):
    f = open(log, "w")
    p = subprocess.Popen(cmd, cwd=cwd, stdout=f, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, start_new_session=True)
    return p, f


def tail(path, n=60):
    try:
        return "\n".join(Path(path).read_text(errors="replace").splitlines()[-n:])
    except FileNotFoundError:
        return ""


def main():
    stop_stale()
    param, runscript, state_csv, channel_map = write_files()
    procs, files = [], []
    try:
        jsb, jf = start(["JSBSim", f"--root={REPO / 'Tools/autotest'}", f"--script={runscript}", "--realtime"], RUN / "jsbsim.log")
        procs.append(jsb); files.append(jf)
        deadline = time.time() + 25
        while time.time() < deadline:
            if jsb.poll() is not None:
                raise RuntimeError("JSBSim exited early\n" + tail(RUN / "jsbsim.log"))
            if state_csv.exists() and sum(1 for _ in state_csv.open()) > 3:
                break
            time.sleep(0.1)
        else:
            raise RuntimeError("JSBSim state CSV did not start\n" + tail(RUN / "jsbsim.log"))

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
            "--actuator-map", str(channel_map),
            "--actuator-timeout-ms", "250",
            "--invalid-active-pwm-neutral",
            "--jsbsim-command-target", f"udp:127.0.0.1:{JSB_PORT}",
            "--rc1-pwm", "1500", "--rc2-pwm", "1500", "--rc3-pwm", "1500", "--rc4-pwm", "1500", "--rc7-pwm", "1000",
            "--precontrol-hold",
            "--precontrol-elevator", str(PRECONTROL_ELEVATOR),
            "--precontrol-aileron", "0.0",
            "--precontrol-rudder", "0.0",
            "--precontrol-throttle", str(PRECONTROL_THROTTLE),
            "--precontrol-rato", "0.0",
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
            "--home", "32.5378147,74.3661871,3000,315",
            "--serial0", f"tcp:{MAVLINK_PORT}:wait",
        ], RUN / "arduplane.log", cwd=RUN)
        procs.append(ap); files.append(af)

        deadline = time.time() + 20
        master = None
        last_error = None
        while time.time() < deadline:
            try:
                master = mavutil.mavlink_connection(f"tcp:127.0.0.1:{MAVLINK_PORT}", source_system=255, source_component=190, dialect="ardupilotmega")
                break
            except (ConnectionRefusedError, OSError) as e:
                last_error = e
                time.sleep(0.2)
        if master is None:
            raise RuntimeError(f"pymavlink could not connect TCP {MAVLINK_PORT}: {last_error}\n" + tail(RUN / "arduplane.log"))

        hb = master.wait_heartbeat(timeout=15)
        if hb is None:
            raise RuntimeError("no heartbeat\n" + tail(RUN / "arduplane.log"))
        print("heartbeat_source", hb.get_srcSystem(), hb.get_srcComponent(), "mode", mavutil.mode_string_v10(hb))

        # No arm, no mission upload, no mode change -- just hold and observe.
        print("VALIDATE_WINDOW_BEGIN")
        t_end = time.time() + 10
        last_mode = None
        while time.time() < t_end:
            msg = master.recv_match(type=["HEARTBEAT", "STATUSTEXT"], blocking=True, timeout=1)
            if msg is None:
                continue
            if msg.get_type() == "STATUSTEXT":
                print("statustext", msg.text)
            elif msg.get_type() == "HEARTBEAT":
                mode = mavutil.mode_string_v10(msg)
                if mode != last_mode:
                    print("mode", mode, "armed", bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED))
                    last_mode = mode
        print("VALIDATE_WINDOW_END")

        rows = list(csv.DictReader(state_csv.open()))
        print("F22GZL_ROW_COUNT", len(rows))

        def g(row, name):
            for k in row:
                if k.endswith(name):
                    try:
                        return float(row[k])
                    except ValueError:
                        return None
            return None

        MPS = 0.3048
        print("t,theta_deg,psi_deg,tas_mps,thrust2_lbf")
        for i, row in enumerate(rows):
            if i % 30 != 0 and i != len(rows) - 1:
                continue
            t = g(row, "sim-time-sec")
            theta = g(row, "theta-deg")
            psi = g(row, "psi-deg")
            vt = g(row, "vt-fps")
            thrust2 = g(row, "engine[2]/thrust-lbs")
            tas = (vt or 0) * MPS
            print(f"{t:.3f},{theta:.3f},{psi:.3f},{tas:.3f},{thrust2}")

    finally:
        for p in procs:
            try:
                p.terminate()
            except Exception:
                pass
        time.sleep(0.5)
        for p in procs:
            try:
                p.kill()
            except Exception:
                pass
        for f in files:
            try:
                f.close()
            except Exception:
                pass


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"F22GZL_RUN_ERROR: {e}", file=sys.stderr)
        sys.exit(1)
