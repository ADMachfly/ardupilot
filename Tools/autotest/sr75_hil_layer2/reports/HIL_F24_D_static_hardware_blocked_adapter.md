# HIL-F24-D: fmuv3-SR75-SoH Static Hardware SIM_JSON Test — Status

Status: **BLOCKED_EXTERNAL_ADAPTER**

**This is not a firmware, parameter, or SIM_JSON-protocol failure.** The
fmuv3-SimOnHardWare firmware is flashed and HIL-F24-C's read-only precheck
tooling reports **GO** for the static-sensor bench test. Every software
prerequisite for the static hardware SIM_JSON test is in place and has
been validated as far as it can be without a working serial link (see
HIL-F24-F for the full offline/SITL-loopback validation of the SIM_JSON
profiles, coherence checks, and hardware-facing tooling built around this
blocker).

## What is actually blocked

The bench's **CP2102 USB-UART adapter** used for the PPP/TELEM1 link fails
its own TX/RX loopback test (shorting the adapter's TX and RX pins and
confirming bytes written are read back) — i.e. the failure is external to
the Pixhawk and to any ArduPilot code, and is confirmed before the adapter
is ever connected to the flight controller. With a loopback-failing
adapter, the PPP link over `SERIAL1`/TELEM1 cannot come up (`sr75_ppp_
start.sh` would start `pppd` against a UART that cannot reliably move
bytes in both directions), so the static SIM_JSON hardware test
(`sr75_hil_f24d_static_state_feed.py` feeding `sr75_sim_json_responder.py`
over PPP into the Pixhawk) cannot be exercised end-to-end against real
hardware yet.

## What is explicitly NOT blocked / already validated

- Firmware artifact: flashed, identity-verified, SR-75 code present
  (HIL-F24-C §1).
- Parameter set: fully audited against every required safety value, no
  discrepancies (HIL-F24-C §5).
- Read-only precheck tooling (`sr75_hil_f24c_preflash_precheck.py`):
  tested, GO.
- Static-state feeder (`sr75_hil_f24d_static_state_feed.py`): implemented,
  unit-tested (6/6), its CSV schema proven compatible with the real
  `StateMapper.state_from_csv()`.
- SIM_JSON profile generation, schema, and coherence (attitude/
  quaternion, gyro-vs-attitude-rate, gravity-consistent accelerometer,
  NED position/velocity, airspeed, 50 Hz monotonic timestamps): fully
  validated through the SITL/loopback (no-hardware) path in HIL-F24-F,
  independent of the adapter.
- Future hardware acceptance analyzer and orchestrator tooling
  (HIL-F24-F items 4–7): built and unit-tested now, ready to run the
  instant a working adapter is substituted in — no further code changes
  anticipated on that side.

## Per this task's instruction

This task's instructions explicitly excluded investigating or requiring
the physical UART adapter, and this report does not attempt to diagnose
or repair the CP2102 adapter itself. The corrective action (replacing the
adapter) is a hardware bench task, not a software one; HIL-F24-F item 8
defines the exact replacement-adapter checklist a human operator should
satisfy before the next attempt.

## Next action

1. Obtain a replacement 3.3 V TTL USB-UART adapter meeting HIL-F24-F's
   adapter checklist (3.3 V TTL only, TX/RX/GND, verified 921600 baud
   support, passes its own TX/RX loopback test, no RS-232 levels, 5 V
   line disconnected).
2. Re-run the adapter's own loopback test in isolation, before connecting
   to the Pixhawk at all.
3. Once the loopback test passes, proceed exactly per HIL-F24-C §4 (PPP
   wiring) and HIL-F24-C §7/§8 (static SIM_JSON test procedure and PASS
   criteria) — no firmware or parameter work is needed first.
4. Use `sr75_hil_f24f_hardware_orchestrator.py --execute --confirm-
   static-only` (HIL-F24-F) to run the actual bench session once the
   adapter is replaced; it performs the precheck → PPP start → PPP verify
   → feeder → responder sequence and records all artifacts automatically.

## PASS/FAIL

Not applicable — **BLOCKED_EXTERNAL_ADAPTER**, pending a hardware adapter
replacement outside this task's scope.
