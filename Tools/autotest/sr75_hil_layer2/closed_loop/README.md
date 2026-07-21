# SR-75 Layer 2J-B3 Closed-Loop Attitude Stabilization

This directory contains the software-only B3 harness for ArduPlane SITL,
the host SIM_JSON responder, and the SR-75 JSBSim model. It does not open
serial ports, run PPP, flash firmware, arm physical hardware, or drive GPIO,
servos, relays, engine hardware, or RATO hardware.

## Architecture

```text
ArduPlane SITL --model json:127.0.0.1
  -> SIM_JSON PWM packet on UDP 9002
  -> sr75_sim_json_responder.py
  -> SR-75 actuator decoder
  -> opt-in UDP JSBSim command sink on 127.0.0.1:5600
  -> SR-75 JSBSim dynamics
  -> /tmp/sr75_b3/sr75_b3_state_<case>.csv
  -> SIM_JSON JSON sensor reply
  -> ArduPlane EKF/control loop
```

The responder remains log-only unless started with `--jsbsim-command-target`.
B3 uses that explicit opt-in and points it at the local JSBSim QTJSBSIM input.

`sr75_b3_sitl_control.py` connects only to a loopback MAVLink endpoint by
default. It waits for an ArduPlane heartbeat, requests diagnostic streams, sets
FBWA, performs normal software arming of the localhost SITL process, and prints
RC and SERVO evidence. It does not open serial devices or force-arm.

## Controller Mode

B3 starts ArduPlane in FBWA with `INITIAL_MODE 5` and `FLTMODE_CH 0` from
`SR75_LAYER2J_B3_FBWA.param`. The responder supplies fixed RC inputs:

```text
RC1 roll     = 1500 us
RC2 pitch    = 1500 us
RC3 throttle = 1300 us for the software-only bounded cruise command
RC4 yaw      = 1500 us
RC7 RATO     = 1000 us
```

For the B3 scoring harness this means desired roll is 0 deg and desired pitch
is the trim pitch documented for the run, normally 3 deg to match the JSBSim
initial cruise attitude.

The B3 closed-loop campaign uses full command ownership for propulsion: the
runscript starts only turbojet engine[0] and engine[1], seeds the selected
JSBSim trim elevator/throttle before SIM_JSON packets arrive, and then lets
ArduPlane CH3 drive both turbojets through `fcs/turbojet-throttle-cmd-norm`.
RATO engine[2] is not started and `RC7=1000` keeps `fcs/rato-throttle-cmd-norm`
at zero.

The RC JSON shape is `rc.rc_1` through `rc.rc_8`, matching the current
`libraries/SITL/SIM_JSON.*` parser. The B3 responder script uses
`SR75_LAYER2J_B3_CHANNEL_MAP.json`; CH1, CH2, CH3 and CH4 are required active
PWM channels, while CH7 is optional because `SERVO7_FUNCTION=0` keeps simulated
RATO off for attitude stabilization.

## Cases

```text
trim       0 deg roll, -2 deg pitch
roll_pos   +15 deg roll, -2 deg pitch
roll_neg   -15 deg roll, -2 deg pitch
pitch_pos  0 deg roll, 8 deg pitch
pitch_neg  0 deg roll, -12 deg pitch
combined   +15 deg roll, -10 deg pitch
```

All cases initialize at 3000 m AMSL and 134.1 kt true speed, approximately
69 m/s. Pitch cases are built relative to the -2 deg bounded reference found
by the JSBSim-only B3C-B trim search with elevator -0.30 and throttle 0.30. Because
the initial condition also specifies `gamma=0 deg`, the matching `alpha` value
is set with `theta` so JSBSim starts at the requested pitch instead of relaxing
pitch to the flight-path angle.

Before running the disturbance-rejection campaign, validate the source initial
conditions:

```sh
python3 Tools/autotest/sr75_hil_layer2/closed_loop/validate_sr75_b3_initial_conditions.py
```

## Launch

From the repository root, start one case with three terminals:

```sh
Tools/autotest/sr75_hil_layer2/closed_loop/start_sr75_b3_jsbsim.sh roll_pos
Tools/autotest/sr75_hil_layer2/closed_loop/start_sr75_b3_responder.sh roll_pos
Tools/autotest/sr75_hil_layer2/closed_loop/start_sr75_b3_ardupilot.sh roll_pos
Tools/autotest/sr75_hil_layer2/closed_loop/start_sr75_b3_control.sh
```

Stop only B3-owned processes:

```sh
Tools/autotest/sr75_hil_layer2/closed_loop/stop_sr75_b3.sh roll_pos
```

The scripts use `/tmp/sr75_b3` for PID files and logs.

## Analysis

```sh
python3 Tools/autotest/sr75_hil_layer2/closed_loop/check_sr75_b3_results.py \
  --responder-log /tmp/sr75_b3/sr75_b3_responder_roll_pos.csv \
  --jsbsim-csv /tmp/sr75_b3/sr75_b3_state_roll_pos.csv \
  --output-csv /tmp/sr75_b3/sr75_b3_combined_roll_pos.csv \
  --desired-roll-deg 0 \
  --desired-pitch-deg 3
```

The combined CSV includes timestamp, desired attitude, actual attitude, body
rates, PWM channels, normalized actuator commands, airspeed, altitude, state
age, request/reply count, stale-command flag, and quaternion norm.

## Sign Audit

The writable JSBSim property map is:

```text
elevator           -> fcs/elevator-cmd-norm
aileron            -> fcs/aileron-cmd-norm
rudder             -> fcs/rudder-cmd-norm
turbojet_throttle  -> fcs/turbojet-throttle-cmd-norm
rato_throttle      -> fcs/rato-throttle-cmd-norm
```

The SR-75 FCS maps `fcs/turbojet-throttle-cmd-norm` through the FADEC actuator
to `fcs/throttle-cmd-norm[0]` and `[1]`; `fcs/rato-throttle-cmd-norm` maps to
`fcs/throttle-cmd-norm[2]`. Elevon and rudder chains are in:

```text
Tools/autotest/aircraft/sr_75_6_dof/Systems/SR75_fcs.xml
Tools/autotest/aircraft/sr_75_6_dof/Systems/SR75_aerodynamics.xml
Tools/autotest/aircraft/sr_75_6_dof/Systems/SR75_propulsion.xml
```

Do not change aerodynamic coefficients to compensate for any sign issue. Use
`SERVOx_REVERSED` or the software channel map if a sign correction is needed.
