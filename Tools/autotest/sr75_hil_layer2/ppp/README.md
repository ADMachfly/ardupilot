# SR-75 Layer 2I-B7 PPP Preparation

This directory is an offline preparation package for the later fmuv3
Simulation-on-Hardware JSON-over-PPP bench test. It does not flash firmware,
open actuator outputs, arm the vehicle, or touch engine, RATO, relay, fuel,
ejection, GPIO, or servo hardware.

## Architecture

```text
JSBSim
-> live state CSV
-> sr75_sim_json_responder.py
-> UDP 192.168.144.2:9002
-> PPP over TELEM1
-> fmuv3 SoH SIM_JSON
```

## SIM_JSON Target

`libraries/SITL/SIM_JSON.h` defaults the target to `127.0.0.1` and the control
port to `9002`. `libraries/SITL/SIM_JSON.cpp` reads the target IP from the JSON
frame string after the first colon. The SoH helper writes that frame string as
`AP_SIM_FRAME_STRING` when `--frame` is supplied.

Use this build command for the SR-75 fmuv3 SoH JSON+PPP image:

```sh
./Tools/scripts/sitl-on-hardware/sitl-on-hw.py \
  --board fmuv3 \
  --vehicle plane \
  --simclass JSON \
  --frame json:192.168.144.2 \
  --enable-PPP
```

No source modification is required for `192.168.144.2:9002`. The IP is compiled
into the SoH image through `AP_SIM_FRAME_STRING`; the UDP port remains the
SIM_JSON default `9002`.

## Planned IPs

```text
Host PPP IP:    192.168.144.2
Pixhawk PPP IP: 192.168.144.14
SIM_JSON UDP:   192.168.144.2:9002
```

The normal PPP backend starts when a serial port is configured with protocol
`48` and networking is enabled. For fmuv3, `SERIAL1` is TELEM1.

## Future Wiring

```text
Pixhawk TELEM1 TX -> 3.3 V USB-UART RX
Pixhawk TELEM1 RX -> 3.3 V USB-UART TX
Pixhawk GND       -> USB-UART GND
```

Warnings:

- Do not connect 5 V from the USB-UART adapter.
- Do not use RS-232 voltage levels.
- Use a 3.3 V TTL UART adapter.
- Keep the vehicle disarmed.
- Keep engine, fuel, ignition, ejection, relay, RATO, and servo hardware disconnected.
- USB remains available for Mission Planner while TELEM1 carries PPP.
- CTS/RTS are not required for this plan; hardware flow control is disabled.

## Pixhawk Parameters

Load `SR75_SoH_JSON_PPP_TELEM1.param` after flashing the prepared SoH firmware:

```text
NET_ENABLE 1
NET_OPTIONS 0
SERIAL1_PROTOCOL 48
SERIAL1_BAUD 921
SERIAL1_OPTIONS 0
BRD_SER1_RTSCTS 0
```

Rollback to normal TELEM1 MAVLink2 with `SR75_TELEM1_MAVLINK_ROLLBACK.param`:

```text
SERIAL1_PROTOCOL 2
SERIAL1_BAUD 57
SERIAL1_OPTIONS 0
BRD_SER1_RTSCTS 2
NET_ENABLE 0
```

Reboot after changing these parameters.

## WSL and Serial Device

On Windows with WSL2, attach the USB-UART device to WSL before starting PPP:

```powershell
usbipd list
usbipd bind --busid <BUSID>
usbipd attach --wsl --busid <BUSID>
```

In WSL/Linux, identify the adapter:

```sh
dmesg -w
ls -l /dev/serial/by-id/
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

Prefer a stable `/dev/serial/by-id/...` path when available.

## Host PPP

Start PPP in the foreground:

```sh
sudo Tools/autotest/sr75_hil_layer2/ppp/sr75_ppp_start.sh \
  --device /dev/ttyUSB0 \
  --baud 921600
```

If 921600 baud is unstable on the adapter or wiring, try the fallback:

```sh
sudo Tools/autotest/sr75_hil_layer2/ppp/sr75_ppp_start.sh \
  --device /dev/ttyUSB0 \
  --baud 460800
```

Stop only the SR-75 PPP process recorded for the selected device:

```sh
sudo Tools/autotest/sr75_hil_layer2/ppp/sr75_ppp_stop.sh --device /dev/ttyUSB0
```

Check link status:

```sh
Tools/autotest/sr75_hil_layer2/ppp/sr75_ppp_status.sh
ip addr show dev ppp0
ip route show dev ppp0
ping -c 3 192.168.144.14
```

## Responder

Run the responder after `ppp0` owns `192.168.144.2`:

```sh
python3 Tools/autotest/sr75_hil_layer2/sim_json/sr75_sim_json_responder.py \
  --listen-host 192.168.144.2 \
  --listen-port 9002 \
  --state-file /tmp/sr75_jsb_live_state.csv \
  --state-timeout-ms 500 \
  --strict \
  --verbose
```

Inspect packets without starting an indefinite capture automatically:

```sh
sudo tcpdump -ni ppp0 udp port 9002
```

## Offline Validation

Run the target preparation test without hardware:

```sh
python3 Tools/autotest/sr75_hil_layer2/ppp/test_sim_json_target.py
```

Before PPP is up, `192.168.144.2:9002` should normally report unavailable.
The localhost fallback should still exchange one SIM_JSON control packet with
the responder.
