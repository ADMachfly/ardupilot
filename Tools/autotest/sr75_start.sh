#!/bin/bash
# =============================================
# SR-75 SITL Startup Script
# Usage: bash sr75_start.sh
# =============================================

ARDUPILOT_DIR="/mnt/f/ardupilot_dev/ardupilot"
SR75_PARM="$ARDUPILOT_DIR/Tools/autotest/models/sr75.parm"
HOME_LOC="32.5378885,74.3661944,240.2,0"

echo "====================================="
echo "  Starting SR-75 SITL Simulation"
echo "  Weight: 82.5 kg | Thrust: 800N"
echo "  Cruise: 69 m/s | Max: 125 m/s"
echo "====================================="

cd "$ARDUPILOT_DIR/ArduPlane"

sim_vehicle.py -v ArduPlane \
    --console \
    --map \
    --add-param-file="$SR75_PARM" \
    -l "$HOME_LOC" \
    -w

echo "SR-75 SITL stopped."
