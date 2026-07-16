#!/usr/bin/env bash
set -euo pipefail

PIXHAWK_IP="192.168.144.14"
RESPONDER_IP="192.168.144.2"
RESPONDER_PORT="9002"

usage() {
    cat <<EOF
Usage: $0 [--pixhawk-ip IP] [--responder-ip IP] [--responder-port PORT]

Shows SR-75 PPP link status without changing serial, GPIO, relay, servo,
engine, RATO, fuel, or actuator state.
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --pixhawk-ip)
            PIXHAWK_IP="${2:?missing value for --pixhawk-ip}"
            shift 2
            ;;
        --responder-ip)
            RESPONDER_IP="${2:?missing value for --responder-ip}"
            shift 2
            ;;
        --responder-port)
            RESPONDER_PORT="${2:?missing value for --responder-port}"
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

echo "ppp0 link:"
ip link show ppp0 || true

echo
echo "ppp0 address:"
ip addr show dev ppp0 || true

echo
echo "routes involving ppp0:"
ip route show dev ppp0 || true

echo
echo "ping Pixhawk PPP endpoint ($PIXHAWK_IP):"
ping -c 3 -W 1 "$PIXHAWK_IP" || true

echo
echo "UDP responder listener check ($RESPONDER_IP:$RESPONDER_PORT):"
if command -v ss >/dev/null 2>&1; then
    ss -H -lun "sport = :$RESPONDER_PORT" | grep -E "(^|[[:space:]])($RESPONDER_IP|0\\.0\\.0\\.0|\\*)[: ]$RESPONDER_PORT" || true
else
    netstat -lun 2>/dev/null | grep "[.:]$RESPONDER_PORT " || true
fi

echo
echo "Optional packet inspection command:"
echo "  sudo tcpdump -ni ppp0 udp port $RESPONDER_PORT"
