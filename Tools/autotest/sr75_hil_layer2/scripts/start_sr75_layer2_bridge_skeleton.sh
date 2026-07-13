#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LAYER2_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
BRIDGE="${LAYER2_DIR}/bridge/sr75_jsbsim_pixhawk_hil_bridge.py"

# Replace 192.168.1.20 with the Windows host IP running Mission Planner.
# Add --gps-input-inject for Layer 2D stationary synthetic GPS_INPUT testing.
# Add --airspeed-inject for Layer 2E synthetic NAMED_VALUE_FLOAT airspeed transmit testing.
python3 "${BRIDGE}" \
    --pixhawk /dev/ttyS4 \
    --baud 115200 \
    --mp-out udp:192.168.1.20:14550 \
    --no-actuator-output
