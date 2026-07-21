#!/usr/bin/env bash
set -euo pipefail

CASE_NAME="${1:-roll_pos}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"
RUN_DIR="${SR75_B3_RUN_DIR:-/tmp/sr75_b3}"
LISTEN_HOST="${SR75_B3_LISTEN_HOST:-127.0.0.1}"
LISTEN_PORT="${SR75_B3_LISTEN_PORT:-9002}"
JSBSIM_PORT="${SR75_B3_JSBSIM_PORT:-5600}"
STATE_CSV="${RUN_DIR}/sr75_b3_state_${CASE_NAME}.csv"
RESPONDER_LOG="${RUN_DIR}/sr75_b3_responder_${CASE_NAME}.log"
RESPONDER_CSV="${RUN_DIR}/sr75_b3_responder_${CASE_NAME}.csv"
PID_FILE="${RUN_DIR}/responder_${CASE_NAME}.pid"

mkdir -p "${RUN_DIR}"
if [[ -f "${PID_FILE}" ]] && kill -0 "$(cat "${PID_FILE}")" 2>/dev/null; then
    echo "Responder already running for ${CASE_NAME}: pid $(cat "${PID_FILE}")" >&2
    exit 1
fi

rm -f "${RESPONDER_LOG}" "${RESPONDER_CSV}" "${PID_FILE}"

cd "${REPO_ROOT}"
setsid python3 Tools/autotest/sr75_hil_layer2/sim_json/sr75_sim_json_responder.py \
    --listen-host "${LISTEN_HOST}" \
    --listen-port "${LISTEN_PORT}" \
    --state-file "${STATE_CSV}" \
    --state-timeout-ms "${SR75_B3_STATE_TIMEOUT_MS:-500}" \
    --rate-limit-hz "${SR75_B3_REPLY_RATE_HZ:-60}" \
    --fresh-state-wait-ms "${SR75_B3_FRESH_STATE_WAIT_MS:-25}" \
    --strict \
    --log-csv "${RESPONDER_CSV}" \
    --b3-state-envelope-guard \
    --b3-state-envelope-abort-row "${RUN_DIR}/sr75_b3_state_envelope_abort_${CASE_NAME}.csv" \
    --actuator-map Tools/autotest/sr75_hil_layer2/closed_loop/SR75_LAYER2J_B3_CHANNEL_MAP.json \
    --actuator-timeout-ms "${SR75_B3_ACTUATOR_TIMEOUT_MS:-250}" \
    --invalid-active-pwm-neutral \
    --jsbsim-command-target "udp:127.0.0.1:${JSBSIM_PORT}" \
    --rc1-pwm "${SR75_B3_RC1_PWM:-1500}" \
    --rc2-pwm "${SR75_B3_RC2_PWM:-1500}" \
    --rc3-pwm "${SR75_B3_RC3_PWM:-1300}" \
    --rc4-pwm "${SR75_B3_RC4_PWM:-1500}" \
    --rc7-pwm "${SR75_B3_RC7_PWM:-1000}" \
    >"${RESPONDER_LOG}" 2>&1 < /dev/null &
echo "$!" > "${PID_FILE}"

echo "Started responder case=${CASE_NAME} pid=$(cat "${PID_FILE}")"
echo "SIM_JSON UDP: ${LISTEN_HOST}:${LISTEN_PORT}"
echo "JSBSim command target: udp:127.0.0.1:${JSBSIM_PORT}"
echo "Responder CSV: ${RESPONDER_CSV}"
echo "Log: ${RESPONDER_LOG}"
