#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""F22-GZ-X: post-release partial-RATO/burnout/eject profile through the
full JSBSim + sr75_sim_json_responder.py + ArduPlane pipeline.

Starts from the Gazebo release-state IC (f22gzo_release_state_init.xml,
ubody=43.85 m/s, theta=20deg, psi=315deg) with partial RATO propellant
(6.83kg, ~2.232s remaining burn) and dry booster attached (10kg). Dry
booster ejection (tank[2] -> 0 at propellant depletion) is a pure FDM/mass
event, injected via a JSBSim runscript condition -- it does not compete
with any live command source.

The elevator/turbojet/RATO schedule (elevator=+0.10 during the burn,
elevator=0.00 after burnout/eject; turbojet held via the proven responder
precontrol path, not a bare JSBSim runscript override) is switched by
running the responder in --precontrol-hold mode and swapping which
responder process is alive: responder #1 (burn-phase precontrol) is
killed and responder #2 (post-burn precontrol) started the instant the
state CSV shows propellant depletion. Only one responder is ever running
at a time, so this avoids the F21-J dual-command-source hazard (two
senders racing into the same JSBSim UDP input) while still allowing a
scheduled precontrol change without editing responder.py or racing a
second live sender against it.
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
RUN = Path(os.environ.get("SR75_D3_RUN", "/tmp/sr75_f22gzx_full"))
IC_NAME = "f22gzo_release_state_init"
CASE = "f22gzx_full_pipeline"
JSB_PORT = 5605
SIM_JSON_PORT = 9002
MAVLINK_PORT = 5775

BURN_ELEVATOR = 0.10
POSTBURN_ELEVATOR = 0.00
TURBOJET_THROTTLE = 0.37  # F21's established cruise value, not a new tuning
PROP_REMAINING_LBS = 15.057396137417392  # 9.18kg * (2.232/3.0)
DRY_BOOSTER_LBS = 22.046226               # 10.0kg


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
    param = RUN / "f22gzx.param"
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
    runscript = RUN / f"{CASE}.xml"
    rel_output = f"../../../../../tmp/{RUN.name}/sr75_b3_state_{CASE}.csv"
    runscript.write_text(f"""<?xml version="1.0" encoding="utf-8"?>
<runscript xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
    xsi:noNamespaceSchemaLocation="http://jsbsim.sf.net/JSBSimScript.xsd"
    name="F22-GZ-X full pipeline post-release profile">

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
      <notify/>
    </event>
    <event name="eject_dry_booster" persistent="false">
      <condition>propulsion/tank[1]/contents-lbs le 0.01</condition>
      <set name="propulsion/tank[2]/contents-lbs" value="0.0"/>
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


def responder_cmd(state_csv, channel_map, responder_csv, elevator, throttle, rato):
    return [
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
        "--precontrol-elevator", str(elevator),
        "--precontrol-aileron", "0.0",
        "--precontrol-rudder", "0.0",
        "--precontrol-throttle", str(throttle),
        "--precontrol-rato", str(rato),
    ]


def g(row, name):
    for k in row:
        if k.endswith(name):
            try:
                return float(row[k])
            except ValueError:
                return None
    return None


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
        resp, rf = start(
            responder_cmd(state_csv, channel_map, responder_csv, BURN_ELEVATOR, TURBOJET_THROTTLE, 1.0),
            RUN / "responder_burn.log",
        )
        procs.append(resp); files.append(rf)
        time.sleep(0.5)
        if resp.poll() is not None:
            raise RuntimeError("responder (burn) exited early\n" + tail(RUN / "responder_burn.log"))

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

        t_start = time.time()
        burnout_switched = False
        last_mode = None
        while time.time() - t_start < 30:
            msg = master.recv_match(type=["HEARTBEAT"], blocking=True, timeout=0.2)
            if msg is not None:
                mode = mavutil.mode_string_v10(msg)
                if mode != last_mode:
                    print("mode", mode, "t", time.time() - t_start)
                    last_mode = mode

            if not burnout_switched and state_csv.exists():
                try:
                    with state_csv.open() as f:
                        last_lines = f.readlines()[-3:]
                    if last_lines:
                        reader = csv.DictReader([open(state_csv).readline()] + last_lines)
                        rows = list(reader)
                        if rows:
                            prop = g(rows[-1], "tank[1]/contents-lbs")
                            if prop is not None and prop <= 0.01:
                                print(f"BURNOUT_DETECTED wall_t={time.time()-t_start:.3f} prop_lbs={prop}")
                                resp.terminate()
                                time.sleep(0.3)
                                resp.kill()
                                rf.close()
                                resp2, rf2 = start(
                                    responder_cmd(state_csv, channel_map, responder_csv, POSTBURN_ELEVATOR, TURBOJET_THROTTLE, 0.0),
                                    RUN / "responder_postburn.log",
                                )
                                procs[1] = resp2
                                files[1] = rf2
                                resp, rf = resp2, rf2
                                burnout_switched = True
                                print(f"RESPONDER_SWITCHED_TO_POSTBURN wall_t={time.time()-t_start:.3f}")
                except Exception as e:
                    print("burnout_poll_error", e)

        print("RUN_WINDOW_END")

        rows = list(csv.DictReader(state_csv.open()))
        print("F22GZX_ROW_COUNT", len(rows))

        MPS = 0.3048
        LB_TO_KG = 0.45359237
        LBF_TO_N = 4.4482216153

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
        for row in rows:
            t = g(row, "sim-time-sec")
            h = g(row, "h-sl-ft")
            theta = g(row, "theta-deg")
            psi = g(row, "psi-deg")
            vt = g(row, "vt-fps")
            q = g(row, "q-rad_sec")
            # F22-GZ-Y: JSBSim's CSV header uses bare "engine/..." for the
            # first indexed element (no "[0]"), unlike engine[1]/engine[2].
            # The previous "engine[0]/thrust-lbs" lookup matched nothing,
            # silently fell back to 0.0 via the "or 0.0" default, and made
            # engine[0] look permanently unthrusted -- there was never a
            # real asymmetry; both engines were identical the whole run.
            e0 = g(row, "engine/thrust-lbs") or 0.0
            e1 = g(row, "engine[1]/thrust-lbs") or 0.0
            e2 = g(row, "engine[2]/thrust-lbs") or 0.0
            dry = g(row, "tank[2]/contents-lbs")
            if alt0 is None:
                alt0 = h
            if theta is not None:
                max_theta = max(max_theta, theta)
                min_theta = min(min_theta, theta)
            if q is not None:
                max_q = max(max_q, abs(q))
            max_engine_diff = max(max_engine_diff, abs(e0 - e1))
            if burnout_row is None and prev_e2 is not None and prev_e2 > 10 and e2 < 10:
                burnout_row = (t, theta, vt, h)
            if eject_row is None and prev_dry is not None and prev_dry > 0.01 and dry is not None and dry <= 0.01:
                eject_row = (t, dry * LB_TO_KG, prev_dry * LB_TO_KG)
            prev_e2 = e2
            prev_dry = dry
            for snap_t in (5.0, 10.0, 15.0, 20.0, 30.0):
                key = f"t{int(snap_t)}"
                if key not in snapshots and t is not None and t >= snap_t:
                    snapshots[key] = (theta, (h - alt0) * MPS, vt * MPS, e0 * LBF_TO_N, e1 * LBF_TO_N)

        if burnout_row:
            bt, btheta, bvt, bh = burnout_row
            print(f"BURNOUT t={bt:.3f}s theta={btheta:.2f} TAS={bvt*MPS:.2f} m/s alt_gain={(bh-alt0)*MPS:.2f} m")
        if eject_row:
            et, dry_after, dry_before = eject_row
            print(f"EJECT   t={et:.3f}s dry_before={dry_before:.3f}kg dry_after={dry_after:.3f}kg")
        print(f"max_theta={max_theta:.2f} min_theta={min_theta:.2f} max_q={max_q:.3f} rad/s")
        print(f"max |engine[0]-engine[1]| thrust diff over full run: {max_engine_diff*LBF_TO_N:.4f} N")
        print(f"{'t':>4} {'theta':>8} {'alt_m':>9} {'tas':>8} {'e0_N':>9} {'e1_N':>9}")
        for key in ("t5", "t10", "t15", "t20", "t30"):
            if key in snapshots:
                th, alt, tas, e0n, e1n = snapshots[key]
                print(f"{key:>4} {th:8.2f} {alt:9.2f} {tas:8.2f} {e0n:9.1f} {e1n:9.1f}")

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
        print(f"F22GZX_RUN_ERROR: {e}", file=sys.stderr)
        sys.exit(1)
