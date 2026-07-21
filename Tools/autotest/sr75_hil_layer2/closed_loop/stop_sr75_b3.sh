#!/usr/bin/env bash
set -euo pipefail

CASE_NAME="${1:-}"
RUN_DIR="${SR75_B3_RUN_DIR:-/tmp/sr75_b3}"

if [[ ! -d "${RUN_DIR}" ]]; then
    echo "No B3 run directory: ${RUN_DIR}"
    exit 0
fi

patterns=("jsbsim" "responder" "ardupilot")
for role in "${patterns[@]}"; do
    if [[ -n "${CASE_NAME}" ]]; then
        files=("${RUN_DIR}/${role}_${CASE_NAME}.pid")
    else
        files=("${RUN_DIR}/${role}_"*.pid)
    fi
    for pid_file in "${files[@]}"; do
        [[ -e "${pid_file}" ]] || continue
        pid="$(cat "${pid_file}")"
        if kill -0 "${pid}" 2>/dev/null; then
            echo "Stopping ${role} pid=${pid} (${pid_file})"
            kill "${pid}"
        fi
        rm -f "${pid_file}"
    done
done
