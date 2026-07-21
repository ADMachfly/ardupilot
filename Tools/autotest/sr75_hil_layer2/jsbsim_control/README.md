# SR-75 JSBSim Command Control

Layer 2J-B2 adds an explicit opt-in software-only path from decoded SIM_JSON
PWM packets to a running local JSBSim SR-75 model. The default responder mode
remains log-only and does not write commands anywhere.

Enable the command sink only with:

```sh
python3 Tools/autotest/sr75_hil_layer2/sim_json/sr75_sim_json_responder.py \
  --state-file /tmp/sr75_jsb_layer2j_b2_state.csv \
  --strict \
  --jsbsim-command-target udp:127.0.0.1:5600 \
  --verbose
```

The JSBSim script side uses a local UDP input socket:

```xml
<input type="QTJSBSIM" port="5600" rate="50">
  <property>fcs/elevator-cmd-norm</property>
  <property>fcs/aileron-cmd-norm</property>
  <property>fcs/rudder-cmd-norm</property>
  <property>fcs/turbojet-throttle-cmd-norm</property>
  <property>fcs/rato-throttle-cmd-norm</property>
</input>
```

The UDP packet is ASCII CSV in the order JSBSim `FGUDPInputSocket` expects:

```text
timestamp_s,elevator,aileron,rudder,turbojet_throttle,rato_throttle
```

`stale` is kept in the responder command object and logs; it is not sent as a
JSBSim property because the SR-75 model has no stale-command input property.
When SIM_JSON servo packets time out, the sink sends one neutral command:
neutral surfaces, zero turbojet throttle, and RATO throttle off.

## Active JSBSim Properties

The SR-75 FCS maps the decoded elevon pair to the existing mixer inputs:

```text
elevator -> fcs/elevator-cmd-norm
aileron  -> fcs/aileron-cmd-norm
rudder   -> fcs/rudder-cmd-norm
```

The active twin turbojet FADEC actuator in
`Tools/autotest/aircraft/sr_75_6_dof/Systems/SR75_fcs.xml` reads
`fcs/turbojet-throttle-cmd-norm` and writes `fcs/throttle-cmd-norm[0]` and
`fcs/throttle-cmd-norm[1]`. The scalar `fcs/throttle-cmd-norm` name aliases the
engine-0 indexed throttle in JSBSim CSV/catalog output, so B2/B3 use the
dedicated turbojet command property as the shared FADEC input.

The simulated RATO command is:

```text
rato_throttle -> fcs/rato-throttle-cmd-norm
```

The B2 JSBSim test script starts engine[2] in software at startup, matching the
RATO model note that the placeholder stays running and produces useful thrust
only when `fcs/rato-throttle-cmd-norm` is nonzero.

This is still software-only JSBSim property input. It does not open serial
devices, GPIO, relays, servo outputs, engine hardware, or RATO hardware.
