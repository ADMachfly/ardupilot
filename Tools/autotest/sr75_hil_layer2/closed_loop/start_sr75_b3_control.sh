#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"
MAVLINK_TARGET="${SR75_B3_MAVLINK_TARGET:-tcp:127.0.0.1:5760}"

cd "${REPO_ROOT}"
python3 Tools/autotest/sr75_hil_layer2/closed_loop/sr75_b3_sitl_control.py \
    --connect "${MAVLINK_TARGET}" \
    --monitor-seconds "${SR75_B3_MONITOR_SECONDS:-5}"
