#!/usr/bin/env bash
set -euo pipefail

CASE_NAME="${1:-roll_pos}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"
RUN_DIR="${SR75_B3_RUN_DIR:-/tmp/sr75_b3}"
JSBSIM_BIN="${JSBSIM_BIN:-JSBSim}"
JSBSIM_PORT="${SR75_B3_JSBSIM_PORT:-5600}"
JSBSIM_RATE="${SR75_B3_JSBSIM_RATE:-50}"
JSBSIM_END="${SR75_B3_JSBSIM_END:-120}"
TRIM_ELEVATOR="${SR75_B3_TRIM_ELEVATOR:--0.30}"
TRIM_THROTTLE="${SR75_B3_TRIM_THROTTLE:-0.30}"

case "${CASE_NAME}" in
    roll_pos) INIT_NAME="layer2j_b3_roll_pos_init" ;;
    roll_neg) INIT_NAME="layer2j_b3_roll_neg_init" ;;
    pitch_pos) INIT_NAME="layer2j_b3_pitch_pos_init" ;;
    pitch_neg) INIT_NAME="layer2j_b3_pitch_neg_init" ;;
    combined) INIT_NAME="layer2j_b3_combined_init" ;;
    *) echo "Unknown B3 case: ${CASE_NAME}" >&2; exit 2 ;;
esac

mkdir -p "${RUN_DIR}"
STATE_CSV="${RUN_DIR}/sr75_b3_state_${CASE_NAME}.csv"
if [[ "${STATE_CSV}" == /tmp/* ]]; then
    JSBSIM_STATE_OUTPUT="../../../../../tmp/${STATE_CSV#/tmp/}"
else
    JSBSIM_STATE_OUTPUT="${STATE_CSV}"
fi
JSBSIM_LOG="${RUN_DIR}/sr75_b3_jsbsim_${CASE_NAME}.log"
PID_FILE="${RUN_DIR}/jsbsim_${CASE_NAME}.pid"
RUNSCRIPT="${RUN_DIR}/sr75_b3_${CASE_NAME}.xml"

if [[ -f "${PID_FILE}" ]] && kill -0 "$(cat "${PID_FILE}")" 2>/dev/null; then
    echo "JSBSim already running for ${CASE_NAME}: pid $(cat "${PID_FILE}")" >&2
    exit 1
fi

rm -f "${STATE_CSV}" "${JSBSIM_LOG}" "${PID_FILE}" "${RUNSCRIPT}"

cat > "${RUNSCRIPT}" <<EOF
<?xml version="1.0" encoding="utf-8"?>
<runscript xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
    xsi:noNamespaceSchemaLocation="http://jsbsim.sf.net/JSBSimScript.xsd"
    name="SR75 Layer 2J-B3 ${CASE_NAME} closed-loop input">

  <use aircraft="sr_75_6_dof" initialize="${INIT_NAME}"/>

  <input type="QTJSBSIM" port="${JSBSIM_PORT}" rate="${JSBSIM_RATE}">
    <property>fcs/elevator-cmd-norm</property>
    <property>fcs/aileron-cmd-norm</property>
    <property>fcs/rudder-cmd-norm</property>
    <property>fcs/turbojet-throttle-cmd-norm</property>
    <property>fcs/rato-throttle-cmd-norm</property>
  </input>

  <run start="0.0" end="${JSBSIM_END}" dt="0.008333">
    <event name="Set simulation-only propulsion running and RATO off">
      <condition>simulation/sim-time-sec ge 0.0</condition>
      <set name="propulsion/engine[0]/set-running" value="1"/>
      <set name="propulsion/engine[1]/set-running" value="1"/>
      <set name="propulsion/engine[2]/set-running" value="0"/>
      <set name="fcs/elevator-cmd-norm" value="${TRIM_ELEVATOR}"/>
      <set name="fcs/turbojet-throttle-cmd-norm" value="${TRIM_THROTTLE}"/>
      <set name="fcs/rato-throttle-cmd-norm" value="0.0"/>
      <notify/>
    </event>
  </run>

  <output name="${JSBSIM_STATE_OUTPUT}" type="CSV" rate="${JSBSIM_RATE}">
    <property>simulation/sim-time-sec</property>
    <property>position/lat-gc-deg</property>
    <property>position/long-gc-deg</property>
    <property>position/h-sl-ft</property>
    <property>velocities/v-north-fps</property>
    <property>velocities/v-east-fps</property>
    <property>velocities/v-down-fps</property>
    <property>velocities/vc-kts</property>
    <property>velocities/vt-fps</property>
    <property>attitude/phi-deg</property>
    <property>attitude/theta-rad</property>
    <property>attitude/psi-deg</property>
    <property>aero/beta-rad</property>
    <function name="p_rad_s"><property>velocities/p-rad_sec</property></function>
    <function name="q_rad_s"><property>velocities/q-rad_sec</property></function>
    <function name="r_rad_s"><property>velocities/r-rad_sec</property></function>
    <function name="accel_body_x_mss"><product><property>accelerations/a-pilot-x-ft_sec2</property><value>0.3048</value></product></function>
    <function name="accel_body_y_mss"><product><property>accelerations/a-pilot-y-ft_sec2</property><value>0.3048</value></product></function>
    <function name="accel_body_z_mss"><product><property>accelerations/a-pilot-z-ft_sec2</property><value>0.3048</value></product></function>
    <function name="q1"><sum><product><cos><product><property>attitude/phi-rad</property><value>0.5</value></product></cos><cos><product><property>attitude/theta-rad</property><value>0.5</value></product></cos><cos><product><property>attitude/psi-rad</property><value>0.5</value></product></cos></product><product><sin><product><property>attitude/phi-rad</property><value>0.5</value></product></sin><sin><product><property>attitude/theta-rad</property><value>0.5</value></product></sin><sin><product><property>attitude/psi-rad</property><value>0.5</value></product></sin></product></sum></function>
    <function name="q2"><difference><product><sin><product><property>attitude/phi-rad</property><value>0.5</value></product></sin><cos><product><property>attitude/theta-rad</property><value>0.5</value></product></cos><cos><product><property>attitude/psi-rad</property><value>0.5</value></product></cos></product><product><cos><product><property>attitude/phi-rad</property><value>0.5</value></product></cos><sin><product><property>attitude/theta-rad</property><value>0.5</value></product></sin><sin><product><property>attitude/psi-rad</property><value>0.5</value></product></sin></product></difference></function>
    <function name="q3"><sum><product><cos><product><property>attitude/phi-rad</property><value>0.5</value></product></cos><sin><product><property>attitude/theta-rad</property><value>0.5</value></product></sin><cos><product><property>attitude/psi-rad</property><value>0.5</value></product></cos></product><product><sin><product><property>attitude/phi-rad</property><value>0.5</value></product></sin><cos><product><property>attitude/theta-rad</property><value>0.5</value></product></cos><sin><product><property>attitude/psi-rad</property><value>0.5</value></product></sin></product></sum></function>
    <function name="q4"><difference><product><cos><product><property>attitude/phi-rad</property><value>0.5</value></product></cos><cos><product><property>attitude/theta-rad</property><value>0.5</value></product></cos><sin><product><property>attitude/psi-rad</property><value>0.5</value></product></sin></product><product><sin><product><property>attitude/phi-rad</property><value>0.5</value></product></sin><sin><product><property>attitude/theta-rad</property><value>0.5</value></product></sin><cos><product><property>attitude/psi-rad</property><value>0.5</value></product></cos></product></difference></function>
    <property>fcs/elevator-cmd-norm</property>
    <property>fcs/aileron-cmd-norm</property>
    <property>fcs/rudder-cmd-norm</property>
    <property>fcs/turbojet-throttle-cmd-norm</property>
    <property>fcs/rato-throttle-cmd-norm</property>
    <property>fcs/left-elevon-pos-rad</property>
    <property>fcs/right-elevon-pos-rad</property>
    <property>fcs/rudder3-pos-rad</property>
    <property>fcs/rudder4-pos-rad</property>
    <property>fcs/throttle-cmd-norm[0]</property>
    <property>fcs/throttle-cmd-norm[1]</property>
    <property>fcs/throttle-cmd-norm[2]</property>
    <property>propulsion/engine[0]/thrust-lbs</property>
    <property>propulsion/engine[1]/thrust-lbs</property>
    <property>propulsion/engine[2]/thrust-lbs</property>
  </output>
</runscript>
EOF

setsid "${JSBSIM_BIN}" --root="${REPO_ROOT}/Tools/autotest" --script="${RUNSCRIPT}" --realtime \
    >"${JSBSIM_LOG}" 2>&1 < /dev/null &
echo "$!" > "${PID_FILE}"

deadline=$((SECONDS + 3))
while (( SECONDS < deadline )); do
    if ! kill -0 "$(cat "${PID_FILE}")" 2>/dev/null; then
        echo "JSBSim exited before producing a valid state CSV" >&2
        echo "Generated runscript: ${RUNSCRIPT}" >&2
        echo "Init XML: ${INIT_NAME}" >&2
        echo "Requested state CSV: ${STATE_CSV}" >&2
        echo "Resolved JSBSim output: ${JSBSIM_STATE_OUTPUT}" >&2
        echo "Last 100 JSBSim log lines:" >&2
        tail -100 "${JSBSIM_LOG}" >&2 || true
        rm -f "${PID_FILE}"
        exit 1
    fi
    if [[ -f "${STATE_CSV}" ]] && [[ "$(wc -l < "${STATE_CSV}")" -ge 3 ]]; then
        break
    fi
    sleep 0.1
done

if [[ ! -f "${STATE_CSV}" ]] || [[ "$(wc -l < "${STATE_CSV}")" -lt 3 ]]; then
    echo "JSBSim did not produce header plus two state rows within 3 seconds" >&2
    echo "Generated runscript: ${RUNSCRIPT}" >&2
    echo "Init XML: ${INIT_NAME}" >&2
    echo "Requested state CSV: ${STATE_CSV}" >&2
    echo "Resolved JSBSim output: ${JSBSIM_STATE_OUTPUT}" >&2
    echo "Run directory exists: $(test -d "${RUN_DIR}" && echo yes || echo no)" >&2
    echo "Run directory writable: $(test -w "${RUN_DIR}" && echo yes || echo no)" >&2
    echo "Last 100 JSBSim log lines:" >&2
    tail -100 "${JSBSIM_LOG}" >&2 || true
    kill "$(cat "${PID_FILE}")" 2>/dev/null || true
    rm -f "${PID_FILE}"
    exit 1
fi

echo "Started JSBSim case=${CASE_NAME} pid=$(cat "${PID_FILE}")"
echo "State CSV: ${STATE_CSV}"
echo "Command UDP input: 127.0.0.1:${JSBSIM_PORT}"
echo "Log: ${JSBSIM_LOG}"
