#!/usr/bin/env bash
set -euo pipefail

CASE_NAME="${1:-roll_pos}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"
RUN_DIR="${SR75_B3_RUN_DIR:-/tmp/sr75_b3}"
ARDUPLANE_BIN="${SR75_B3_ARDUPLANE_BIN:-${REPO_ROOT}/build/sitl/bin/arduplane}"
LISTEN_TARGET="${SR75_B3_JSON_TARGET:-127.0.0.1}"
DEFAULTS="${REPO_ROOT}/Tools/autotest/default_params/plane-jsbsim.parm,${REPO_ROOT}/Tools/autotest/models/sr75.parm,${REPO_ROOT}/Tools/autotest/models/sr75_sim.parm,${REPO_ROOT}/Tools/autotest/sr75_hil_layer2/closed_loop/SR75_LAYER2J_B3_FBWA.param"
ARDUPILOT_LOG="${RUN_DIR}/sr75_b3_ardupilot_${CASE_NAME}.log"
PID_FILE="${RUN_DIR}/ardupilot_${CASE_NAME}.pid"

mkdir -p "${RUN_DIR}"
if [[ -f "${PID_FILE}" ]] && kill -0 "$(cat "${PID_FILE}")" 2>/dev/null; then
    echo "ArduPlane already running for ${CASE_NAME}: pid $(cat "${PID_FILE}")" >&2
    exit 1
fi
if [[ ! -x "${ARDUPLANE_BIN}" ]]; then
    echo "Missing executable: ${ARDUPLANE_BIN}" >&2
    echo "Build it with: ./waf configure --board sitl && ./waf plane" >&2
    exit 1
fi

rm -f "${ARDUPILOT_LOG}" "${PID_FILE}"

cd "${REPO_ROOT}"
setsid "${ARDUPLANE_BIN}" \
    --model "json:${LISTEN_TARGET}" \
    --speedup 1 \
    --defaults "${DEFAULTS}" \
    --home "32.5378085,74.3661944,3000,315" \
    >"${ARDUPILOT_LOG}" 2>&1 < /dev/null &
echo "$!" > "${PID_FILE}"

sleep 2
if ! kill -0 "$(cat "${PID_FILE}")" 2>/dev/null; then
    echo "ArduPlane exited during startup" >&2
    echo "Last 100 ArduPlane log lines:" >&2
    tail -100 "${ARDUPILOT_LOG}" >&2 || true
    rm -f "${PID_FILE}"
    exit 1
fi

if ! ss -ltn sport = :5760 | grep -q ':5760'; then
    echo "ArduPlane MAVLink TCP port 5760 is not listening" >&2
    echo "Last 100 ArduPlane log lines:" >&2
    tail -100 "${ARDUPILOT_LOG}" >&2 || true
    rm -f "${PID_FILE}"
    exit 1
fi

echo "Started ArduPlane SITL case=${CASE_NAME} pid=$(cat "${PID_FILE}")"
echo "Model: json:${LISTEN_TARGET} (SIM_JSON control port defaults to 9002)"
echo "Mode: FBWA via INITIAL_MODE=5"
echo "MAVLink endpoint: tcp:127.0.0.1:5760"
echo "Log: ${ARDUPILOT_LOG}"
