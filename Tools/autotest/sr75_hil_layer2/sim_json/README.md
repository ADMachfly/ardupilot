# SR-75 SIM_JSON Responder

This directory contains a host-side UDP responder for ArduPilot `SIM_JSON`.
It is intended for SR-75 Layer 2I-B bench validation before PPP hardware is
available.

The responder never opens a Pixhawk serial port, never sends MAVLink, never
forwards PWM values, and never drives actuators, relays, RATO, engine, FADEC,
fuel, or ejection logic. Incoming PWM/control packets are decoded only to
validate the SIM_JSON protocol and for optional logging.

## Protocol

ArduPilot `libraries/SITL/SIM_JSON.cpp` sends one binary servo/control packet
then waits for one newline-terminated JSON sensor message.

The 16-channel packet is the normal fmuv3/SR-75 case:

```text
little-endian native C struct
uint16 magic       = 18458
uint16 frame_rate
uint32 frame_count
uint16 pwm[16]
size = 40 bytes
```

`SIM_JSON` also supports a 32-channel packet:

```text
little-endian native C struct
uint16 magic       = 29569
uint16 frame_rate
uint32 frame_count
uint16 pwm[32]
size = 72 bytes
```

The responder replies to the exact UDP source address and source port of each
valid control packet. The JSON reply must end with `\n`; ArduPilot replaces the
newline with a NUL terminator before parsing.

## JSON Schema

Required fields from `SIM_JSON.h`:

```json
{
  "timestamp": 0.01,
  "imu": {
    "gyro": [0.0, 0.0, 0.0],
    "accel_body": [0.0, 0.0, -9.80665]
  },
  "velocity": [0.0, 0.0, 0.0],
  "attitude": [0.0, 0.0, 0.0]
}
```

Either `attitude` or `quaternion` must be present. This responder sends both.

Useful optional fields:

```json
{
  "latitude": 32.5378885,
  "longitude": 74.3661944,
  "altitude": 240.2,
  "quaternion": [1.0, 0.0, 0.0, 0.0],
  "airspeed": 69.0,
  "no_time_sync": false,
  "no_lockstep": false
}
```

Units and frames:

```text
timestamp       seconds, monotonic simulation time
latitude        degrees
longitude       degrees
altitude        meters absolute, converted by ArduPilot to Location alt cm
attitude        [roll, pitch, yaw] radians
quaternion      [q1, q2, q3, q4] ArduPilot order, normalized
velocity        [north, east, down] m/s
imu.gyro        body-frame [p, q, r] rad/s
imu.accel_body  body-frame accelerometer specific force, m/s^2
airspeed        m/s
```

For stationary level mock state, `imu.accel_body` is `[0, 0, -9.80665]`,
matching the accelerometer specific force convention used by
`SIM_Aircraft.cpp`.

## CSV Mapping

The current `/tmp/sr75_jsb_live_state.csv` header observed during this task is:

```text
Time
/fdm/jsbsim/simulation/sim-time-sec
/fdm/jsbsim/position/lat-gc-deg
/fdm/jsbsim/position/long-gc-deg
/fdm/jsbsim/position/h-sl-ft
/fdm/jsbsim/velocities/v-north-fps
/fdm/jsbsim/velocities/v-east-fps
/fdm/jsbsim/velocities/v-down-fps
/fdm/jsbsim/velocities/vc-kts
/fdm/jsbsim/velocities/vt-fps
/fdm/jsbsim/attitude/phi-deg
/fdm/jsbsim/attitude/theta-rad
/fdm/jsbsim/attitude/psi-deg
p_rad_s
q_rad_s
r_rad_s
accel_body_x_mss
accel_body_y_mss
accel_body_z_mss
q1
q2
q3
q4
```

Conversions:

```text
sim-time-sec -> timestamp seconds
lat-gc-deg   -> latitude degrees
long-gc-deg  -> longitude degrees
h-sl-ft      -> altitude meters, ft * 0.3048
v-north-fps  -> velocity[0] m/s, ft/s * 0.3048
v-east-fps   -> velocity[1] m/s, ft/s * 0.3048
v-down-fps   -> velocity[2] m/s, ft/s * 0.3048
phi-deg      -> roll radians
theta-rad    -> pitch radians
psi-deg      -> yaw radians
vt-fps       -> airspeed m/s, ft/s * 0.3048
vc-kts       -> fallback airspeed m/s, kt * 0.514444
p_rad_s      -> imu.gyro[0] rad/s
q_rad_s      -> imu.gyro[1] rad/s
r_rad_s      -> imu.gyro[2] rad/s
accel_body_* -> imu.accel_body m/s^2, body specific force
q1..q4       -> quaternion, normalized by the responder before sending
```

The SR-75 live JSBSim producer maps `accel_body_*` from
`accelerations/a-pilot-*-ft_sec2 * 0.3048`. JSBSim evaluates these sensed
accelerations at the configured pilot eyepoint, so rotational lever-arm terms
may be present when body rates or angular accelerations are nonzero. The
eyepoint should later be aligned with the real autopilot installation position
before treating this as a final IMU-location model.

Layer 2I-B6 updates the SR-75 JSBSim live CSV producer to append body gyro,
body specific force, and quaternion columns. In `--strict` live mode, the
responder rejects rows missing body gyro or body acceleration. Non-strict live
mode uses stationary fallback values and prints/logs the missing fields, which
is only suitable for transport testing.

## Local Mock Test

Terminal 1:

```sh
python3 Tools/autotest/sr75_hil_layer2/sim_json/sr75_sim_json_responder.py \
  --mock-state \
  --verbose
```

Terminal 2:

```sh
python3 Tools/autotest/sr75_hil_layer2/sim_json/sr75_sim_json_mock_client.py \
  --count 10 \
  --rate-hz 20 \
  --verbose
```

Expected responder startup:

```text
SR75 SIM_JSON RESPONDER
ACTUATOR OUTPUT: DISABLED
SERIAL OUTPUT: DISABLED
RATO/ENGINE/RELAY OUTPUT: DISABLED
Listening on 0.0.0.0:9002
```

## Live CSV Test

```sh
python3 Tools/autotest/sr75_hil_layer2/sim_json/sr75_sim_json_responder.py \
  --state-file /tmp/sr75_jsb_live_state.csv \
  --state-timeout-ms 500 \
  --verbose
```

Strict mode:

```sh
python3 Tools/autotest/sr75_hil_layer2/sim_json/sr75_sim_json_responder.py \
  --state-file /tmp/sr75_jsb_live_state.csv \
  --state-timeout-ms 500 \
  --strict \
  --verbose
```

## Future PPP Run Command

Use this when the host has the PPP address `192.168.144.2` and the Pixhawk JSON
firmware is configured to send control packets to that host:

```sh
python3 Tools/autotest/sr75_hil_layer2/sim_json/sr75_sim_json_responder.py \
  --listen-host 192.168.144.2 \
  --listen-port 9002 \
  --state-file /tmp/sr75_jsb_live_state.csv \
  --state-timeout-ms 500 \
  --verbose
```

Future target:

```text
host PPP IP: 192.168.144.2
UDP port:    9002
```

## Safety

Safety guarantees in these tools:

```text
No serial device is opened.
No Pixhawk hardware connection is required.
No MAVLink is generated.
No actuator command is forwarded.
No GPIO, relay, RATO, engine, FADEC, fuel, or ejection output is touched.
PWM values from SIM_JSON packets are decoded only for validation/logging.
```
