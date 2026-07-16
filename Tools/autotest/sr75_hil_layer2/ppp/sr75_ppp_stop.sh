#!/usr/bin/env bash
set -euo pipefail

DEVICE="/dev/ttyUSB0"

usage() {
    cat <<EOF
Usage: $0 [--device PATH]

Stops only the SR-75 PPP process recorded for the selected device.
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --device)
            DEVICE="${2:?missing value for --device}"
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "ERROR: unknown argument: $1" >&2
            usage >&2
            exit 2
            ;;
    esac
done

PID_FILE="/tmp/sr75_ppp_${DEVICE//\//_}.pid"

if [[ ! -e "$PID_FILE" ]]; then
    echo "No SR-75 PPP pid file for $DEVICE: $PID_FILE"
    exit 0
fi

PID="$(cat "$PID_FILE")"
if [[ ! "$PID" =~ ^[0-9]+$ ]]; then
    echo "ERROR: malformed pid file: $PID_FILE" >&2
    exit 1
fi

if ! kill -0 "$PID" 2>/dev/null; then
    echo "Recorded SR-75 PPP process is not running: $PID"
    rm -f "$PID_FILE"
    exit 0
fi

CMDLINE="$(tr '\0' ' ' < "/proc/$PID/cmdline" 2>/dev/null || true)"
if [[ "$CMDLINE" != *"pppd"* || "$CMDLINE" != *"$DEVICE"* ]]; then
    echo "ERROR: pid $PID does not look like the SR-75 pppd for $DEVICE" >&2
    echo "cmdline: $CMDLINE" >&2
    exit 1
fi

echo "Stopping SR-75 PPP pid $PID for $DEVICE"
kill "$PID"
for _ in {1..20}; do
    if ! kill -0 "$PID" 2>/dev/null; then
        rm -f "$PID_FILE"
        echo "Stopped"
        exit 0
    fi
    sleep 0.2
done

echo "ERROR: pppd did not stop after SIGTERM; leaving pid file in place" >&2
exit 1
