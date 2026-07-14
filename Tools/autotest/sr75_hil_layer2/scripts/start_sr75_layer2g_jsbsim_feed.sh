#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"
STATE_CSV="/tmp/sr75_jsb_live_state.csv"
JSBSIM_BIN="${JSBSIM_BIN:-JSBSim}"
JSBSIM_SCRIPT="aircraft/sr_75_6_dof/scripts/SR75_layer2g_state_feed_test.xml"
JSBSIM_END="${SR75_LAYER2G_END:-3600}"

rm -f "${STATE_CSV}"

echo "Starting SR-75 Layer 2G JSBSim state feed"
echo "State CSV: ${STATE_CSV}"
echo
echo "Bridge command:"
echo "python3 Tools/autotest/sr75_hil_layer2/bridge/sr75_jsbsim_pixhawk_hil_bridge.py \\"
echo "  --pixhawk /dev/ttyACM0 \\"
echo "  --baud 115200 \\"
echo "  --mp-out udp:192.168.64.1:14550 \\"
echo "  --jsbsim-state-input ${STATE_CSV} \\"
echo "  --gps-input-rate-hz 5 \\"
echo "  --airspeed-rate-hz 30 \\"
echo "  --attitude-rate-hz 10 \\"
echo "  --no-actuator-output"
echo
cd "$(dirname "${STATE_CSV}")"

exec "${JSBSIM_BIN}" \
    --root="${REPO_ROOT}/Tools/autotest" \
    --script="${JSBSIM_SCRIPT}" \
    --realtime \
    --end="${JSBSIM_END}"
