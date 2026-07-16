#!/usr/bin/env bash
set -euo pipefail

DEVICE="/dev/ttyUSB0"
BAUD="921600"
HOST_IP="192.168.144.2"
PIXHAWK_IP="192.168.144.14"
PID_FILE="/tmp/sr75_ppp_${DEVICE//\//_}.pid"
BACKGROUND=0

usage() {
    cat <<EOF
Usage: $0 [--device PATH] [--baud BAUD] [--host-ip IP] [--pixhawk-ip IP] [--background]

Starts a PPP link for SR-75 SIM_JSON over a selected telemetry UART.
No GPIO, relay, servo, engine, RATO, fuel, or actuator interface is touched.
EOF
}

valid_ip() {
    [[ "$1" =~ ^([0-9]{1,3}\.){3}[0-9]{1,3}$ ]]
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --device)
            DEVICE="${2:?missing value for --device}"
            shift 2
            ;;
        --baud)
            BAUD="${2:?missing value for --baud}"
            shift 2
            ;;
        --host-ip)
            HOST_IP="${2:?missing value for --host-ip}"
            shift 2
            ;;
        --pixhawk-ip)
            PIXHAWK_IP="${2:?missing value for --pixhawk-ip}"
            shift 2
            ;;
        --background)
            BACKGROUND=1
            shift
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

if [[ ! -e "$DEVICE" ]]; then
    echo "ERROR: serial device does not exist: $DEVICE" >&2
    exit 1
fi

if ! command -v pppd >/dev/null 2>&1; then
    echo "ERROR: pppd is not installed or not in PATH" >&2
    exit 1
fi

if [[ ! "$BAUD" =~ ^[0-9]+$ ]]; then
    echo "ERROR: baud must be numeric: $BAUD" >&2
    exit 2
fi

if ! valid_ip "$HOST_IP" || ! valid_ip "$PIXHAWK_IP"; then
    echo "ERROR: malformed IP address: host=$HOST_IP pixhawk=$PIXHAWK_IP" >&2
    exit 2
fi

if [[ -e "$PID_FILE" ]] && kill -0 "$(cat "$PID_FILE")" 2>/dev/null; then
    echo "ERROR: SR-75 PPP already appears to be running for $DEVICE, pid $(cat "$PID_FILE")" >&2
    exit 1
fi

echo "SR-75 PPP start"
echo "  device:     $DEVICE"
echo "  baud:       $BAUD"
echo "  host IP:    $HOST_IP"
echo "  Pixhawk IP: $PIXHAWK_IP"
echo "  pid file:   $PID_FILE"

PPP_ARGS=(
    "$DEVICE"
    "$BAUD"
    "$HOST_IP:$PIXHAWK_IP"
    noauth
    local
    nocrtscts
    lock
    nodetach
    ipcp-accept-local
    ipcp-accept-remote
)

if [[ "$BACKGROUND" -eq 1 ]]; then
    echo "Starting pppd in background"
    pppd "${PPP_ARGS[@]}" &
    echo "$!" > "$PID_FILE"
else
    echo "Starting pppd in foreground"
    printf '%s\n' "$$" > "$PID_FILE"
    trap 'rm -f "$PID_FILE"' EXIT
    exec pppd "${PPP_ARGS[@]}"
fi
