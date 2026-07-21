#!/usr/bin/env bash
set -euo pipefail

# SR-75 Layer 2J-B3 diagnostic runner.
#
# Verifies the three B3-owned processes (JSBSim, responder, ArduPlane SITL)
# are already running for the given case, then runs the arm, timing, and
# attitude diagnostics against them and against their logs. Does not start or
# kill any process; use start_sr75_b3_*.sh / stop_sr75_b3.sh for that.

CASE_NAME="${1:-roll_pos}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUN_DIR="${SR75_B3_RUN_DIR:-/tmp/sr75_b3}"

CONNECT="${SR75_B3_CONNECT:-tcp:127.0.0.1:5760}"
REPORT="${RUN_DIR}/sr75_b3_diagnostic_report.txt"
MAVLINK_CSV="${RUN_DIR}/sr75_b3_mavlink_diagnostic.csv"
RESPONDER_CSV="${RUN_DIR}/sr75_b3_responder_${CASE_NAME}.csv"
JSBSIM_CSV="${RUN_DIR}/sr75_b3_state_${CASE_NAME}.csv"
ACTUATOR_TIMEOUT_MS="${SR75_B3_ACTUATOR_TIMEOUT_MS:-250}"

mkdir -p "${RUN_DIR}"

check_running() {
    local role="$1"
    local pid_file="${RUN_DIR}/${role}_${CASE_NAME}.pid"
    if [[ ! -f "${pid_file}" ]]; then
        echo "MISSING: no PID file for ${role} (${pid_file})" >&2
        return 1
    fi
    local pid
    pid="$(cat "${pid_file}")"
    if ! kill -0 "${pid}" 2>/dev/null; then
        echo "MISSING: ${role} pid ${pid} from ${pid_file} is not running" >&2
        return 1
    fi
    echo "RUNNING: ${role} pid=${pid}"
    return 0
}

{
    echo "SR75 Layer 2J-B3 diagnostic report"
    echo "case=${CASE_NAME} generated=$(date -u +%Y-%m-%dT%H:%M:%SZ)"
    echo

    echo "=== Process check ==="
    process_status=0
    check_running jsbsim || process_status=1
    check_running responder || process_status=1
    check_running ardupilot || process_status=1
    if [[ "${process_status}" -ne 0 ]]; then
        echo "One or more required B3 processes are not running for case=${CASE_NAME}."
        echo "Start them with start_sr75_b3_jsbsim.sh / start_sr75_b3_responder.sh / start_sr75_b3_ardupilot.sh."
    fi
    echo

    if [[ "${process_status}" -eq 0 ]]; then
        echo "=== Arm diagnostic ==="
        arm_status=0
        python3 "${SCRIPT_DIR}/diagnose_sr75_b3_arm.py" \
            --connect "${CONNECT}" \
            --log-csv "${MAVLINK_CSV}" || arm_status=$?
        echo "arm_diagnostic_exit_code=${arm_status}"
        echo

        echo "=== Timing diagnostic ==="
        timing_status=0
        if [[ -f "${RESPONDER_CSV}" ]]; then
            python3 "${SCRIPT_DIR}/diagnose_sr75_b3_timing.py" \
                --responder-log "${RESPONDER_CSV}" \
                --actuator-timeout-ms "${ACTUATOR_TIMEOUT_MS}" || timing_status=$?
        else
            echo "SKIPPED: responder CSV not found (${RESPONDER_CSV})"
            timing_status=2
        fi
        echo "timing_diagnostic_exit_code=${timing_status}"
        echo

        echo "=== Attitude diagnostic ==="
        attitude_status=0
        if [[ -f "${MAVLINK_CSV}" && -f "${JSBSIM_CSV}" && -f "${RESPONDER_CSV}" ]]; then
            python3 "${SCRIPT_DIR}/diagnose_sr75_b3_attitude.py" \
                --responder-log "${RESPONDER_CSV}" \
                --jsbsim-csv "${JSBSIM_CSV}" \
                --mavlink-log-csv "${MAVLINK_CSV}" \
                --case "${CASE_NAME}" || attitude_status=$?
        else
            echo "SKIPPED: one of MAVLINK CSV / JSBSim CSV / responder CSV is missing"
            echo "mavlink_csv=${MAVLINK_CSV} jsbsim_csv=${JSBSIM_CSV} responder_csv=${RESPONDER_CSV}"
            attitude_status=2
        fi
        echo "attitude_diagnostic_exit_code=${attitude_status}"
    else
        echo "=== Arm/timing/attitude diagnostics skipped: required processes are not running ==="
    fi
} | tee "${REPORT}"

echo
echo "Report written to ${REPORT}"
