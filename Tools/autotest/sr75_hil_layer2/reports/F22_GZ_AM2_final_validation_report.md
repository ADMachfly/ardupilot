# SR-75 Preserved-State RATO Handoff and V2 Extended Mission — Final Validation Report

**Series**: F22-GZ-AH through F22-GZ-AM2
**Repo**: `~/ardupilot_clean`
**Status**: PASS
**Scope**: JSBSim/ArduPlane SITL only — no hardware, no ground landing

---

## 1. Executive Summary

This report closes out the F22-GZ-AH → AM2 validation arc for the SR-75 "Gazebo rail launch → JSBSim/ArduPlane RATO resume → full AUTO mission" pipeline.

Starting from F22-GZ-AH, the pipeline eliminated a pre-AUTO open-loop attitude/velocity decay by delaying the release-state JSBSim process until ArduPlane is armed and AUTO-ready, then handed increasing authority to live ArduPlane control (F22-GZ-AI), validated stability over long AUTO windows (F22-GZ-AJ), validated a full short waypoint mission with autonomous RTL (F22-GZ-AK), built and ran a long 8-phase altitude-profile mission (F22-GZ-AL), diagnosed a mission-geometry-only shortfall in descent altitude tracking (F22-GZ-AM1), and fixed it purely through waypoint geometry in a V2 mission (F22-GZ-AM2).

**Result**: The V2 mission (`SR75_F22_EXTENDED_ALTITUDE_PROFILE_V2.waypoints`, 14 items, ~211km) runs end-to-end in SITL with an exact preserved release state (θ≈20°, TAS≈43.85 m/s), a fully live-control AUTO flight, a correctly resumed and completed RATO burn/eject sequence, all 14 waypoints captured cleanly, both altitude-critical descent targets landing inside the preferred ±300m tolerance, exact engine symmetry throughout, zero real failsafes, and a clean autonomous transition to RTL on mission completion. No aero, gain, TECS, RATOController, engine-mapping, or responder-protocol code was changed to achieve this — every fix in the AH→AM2 arc was either a test-harness sequencing change or, in AM2, pure mission waypoint geometry.

This validates the **simulated** flight-software and mission-logic pipeline. It does **not** validate real hardware, ground landing, or timing behavior on physical flight controllers or engines (see §12).

---

## 2. What Was Validated, F22-GZ-AH Through AM2

| Task | What it added/validated | Key change |
|---|---|---|
| **AH** | Eliminated pre-AUTO open-loop pitch/velocity decay. Release-state JSBSim process launched fresh, exactly once ArduPlane is armed and AUTO is requested; ArduPlane's entire boot is serviced by a synthetic held state (`B3StartupSync`) matching the intended release IC exactly. | New harness sequencing + `B3StartupSync.force_release()` in the responder |
| **AI** | Validated **live** ArduPlane AUTO/TECS control (CH1–CH4) from the first dynamic frame, replacing the fixed precontrol reference used through AH. Trajectory was healthier under live control than under the fixed reference. | Removed `--auto-gate-hold` override; relies on `B3PrecontrolHandover`'s existing self-release |
| **AJ** | Extended the live-control run to 120s. AUTO mode never dropped; no failsafe; sustained controlled climb, not a runaway. | Run-window extension only |
| **AK** | Validated a full **short AUTO mission** (2 waypoints) end-to-end: RATO resume → burnout → eject → complete → waypoint tracking → **mission complete → automatic RTL**. | Mission upload + `MISSION_CURRENT` tracking added |
| **AL** | Designed and ran a new **8-phase altitude-profile mission** (climb 5000m → cruise → descend 1000m → loiter → ascend 5000m → cruise → descend 500m → final/RTL) over ~1800s, ~160km. All 9 waypoints reached; RTL triggered cleanly. | New mission file (V1) + new harness, run window extended to 1800s |
| **AM1** | Phase-by-phase forensic analysis of the AL run. Found both descent legs captured their waypoint **laterally** well before **altitude** convergence (+1417m error at the 1000m target, +2489m error at the 500m target) — explicitly diagnosed as a mission-geometry issue, not a RATO/aero/TECS defect. | Analysis only, no changes |
| **AM2** | Fixed the diagnosed issue purely via mission geometry: replaced each single-leg descent with a 3–4 waypoint step-down, giving the **final** leg into each terminal altitude the most distance and gentlest grade. Reran the full mission (V2, 14 items, ~211km, ~2400s). | New mission file (V2) + new harness; same RATO/handoff/control logic, unchanged |

Across every step, the preserved-state launch ordering, the RATO resume mechanism (`RATOController::resume_boost()`, built in an earlier task pair not covered by this report), and the live-control architecture were kept byte-for-byte unchanged — each task added exactly one validated capability on top of the last.

---

## 3. Final V2 Mission Item Table

`Tools/autotest/sr75_hil_layer2/closed_loop/SR75_F22_EXTENDED_ALTITUDE_PROFILE_V2.waypoints` — 14 items:

| Seq | Command | Lat | Lon | Alt (m, relative) | Phase |
|---|---|---|---|---|---|
| 0 | (home marker) | 32.5378085 | 74.3661944 | 3000 | Home |
| 1 | NAV_TAKEOFF | 32.5378085 | 74.3661944 | 20 | RATO/takeoff (identical to validated F21/F22 original) |
| 2 | NAV_WAYPOINT | 32.7285809 | 74.1399025 | 5000 | Climb to cruise |
| 3 | NAV_WAYPOINT | 32.8239640 | 74.0267602 | 5000 | Cruise outbound |
| 4 | NAV_WAYPOINT | 32.9193471 | 73.9136179 | 3500 | Descent step 1 |
| 5 | NAV_WAYPOINT | 33.0147302 | 73.8004756 | 2200 | Descent step 2 |
| 6 | NAV_WAYPOINT | 33.2245731 | 73.5515625 | 1000 | Descent → loiter entry (gentlest final-approach grade) |
| 7 | NAV_LOITER_TIME (90s, 500m radius) | 33.2245731 | 73.5515625 | 1000 | Loiter near 1000m |
| 8 | NAV_WAYPOINT | 33.0210891 | 73.7929328 | 5000 | Ascend back |
| 9 | NAV_WAYPOINT | 32.9257060 | 73.9060751 | 5000 | Cruise return |
| 10 | NAV_WAYPOINT | 32.8493995 | 73.9965889 | 3500 | Final descent step 1 |
| 11 | NAV_WAYPOINT | 32.7730930 | 74.0871027 | 2200 | Final descent step 2 |
| 12 | NAV_WAYPOINT | 32.6967865 | 74.1776166 | 1200 | Final descent step 3 |
| 13 | NAV_WAYPOINT | 32.5696091 | 74.3284730 | 500 | Final recovery → RTL trigger (gentlest final-approach grade) |

Total planned path ≈ 211 km, all legs on a single 315° outbound/return bearing. Ground landing is explicitly excluded from this mission (final target is 500m, not 0m).

---

## 4. RATO Timeline (V2 / AM2 run)

| Event | Time |
|---|---|
| Preserved release state confirmed | θ=20.0°, TAS=43.850000066652015 m/s (exact IC match) |
| RATO resume seed | `burn_elapsed=0.76 remaining=2.23` |
| BURNOUT (JSBSim ground truth) | t=2.317s, θ=21.08°, TAS=144.62 m/s, altitude gain=+73.48m |
| ENGINE_TAKEOVER | immediately follows burnout |
| EJECT (JSBSim ground truth) | t=9.483s |
| RATO: complete | shortly after eject |

Identical (within measurement noise) to every prior preserved-state run since AH — no regression introduced by the mission-geometry work in AM2.

---

## 5. Dry Booster Mass Result

| | Value |
|---|---|
| Before eject | 10.000 kg |
| After eject | 0.000 kg |
| Post-eject maximum (entire ~2400s run) | **0.0000 kg** |

Single clean eject event; mass never re-attaches for the remainder of the ~40-minute simulated flight.

---

## 6. Engine Symmetry Result

**max\|engine[0] − engine[1]\| = 0.0000 N**, sampled continuously across the full ~2400s run. Exact symmetry maintained through RATO burn, engine takeover, and the entire subsequent turbojet-powered mission.

---

## 7. Waypoint Progression Table (V2)

| Seq becomes | Time (s since force-release) | Waypoint just satisfied | Capture distance |
|---|---|---|---|
| 1 | 0.54 | mission start | — |
| 2 | 10.07 | NAV_TAKEOFF | — |
| 3 | 339.70 | WP2 climb-to-cruise (5000m) | 1m |
| 4 | 485.99 | WP3 cruise-outbound-end (5000m) | 3m |
| 5 | 631.99 | WP4 descend-step1 (3500m) | 12m |
| 6 | 789.70 | WP5 descend-step2 (2200m) | 0m |
| 7 (LOITER) | 1160.88 | WP6 descend-to-loiter (1000m) | 135m |
| 8 | 1272.88 | LOITER complete (90s hold) | — |
| 9 | 1621.88 | WP8 ascend-back (5000m) | 1m |
| 10 | 1763.14 | WP9 cruise-return-end (5000m) | 3m |
| 11 | 1878.63 | WP10 final-descent-step1 (3500m) | 11m |
| 12 | 2003.63 | WP11 final-descent-step2 (2200m) | 7m |
| 13 | 2138.64 | WP12 final-descent-step3 (1200m) | 1m |
| mission complete | ~2372.49 | WP13 final-recovery (500m) | 125m |

All 14 mission items reached/passed cleanly — capture distances 0–135m, no wide misses, no repeated capture attempts.

---

## 8. Altitude Tracking: AL (V1) vs AM2 (V2)

| Target | AL (V1) achieved altitude | AL error | AM2 (V2) achieved altitude | AM2 error | Goal |
|---|---|---|---|---|---|
| 1000m loiter entry | 2417m (rel) | **+1417m** | 1275.4m (rel), at t=1160.89s | **+275.4m** | ±300m preferred, ±500m acceptable |
| 500m final recovery | 2989m (rel) | **+2489m** | 793.7m (rel), at t=2374.07s | **+293.7m** | ±300m preferred, ±500m acceptable |

Both AM2 errors land **inside the preferred ±300m band** — a >5x reduction in error at the 1000m target and a >8x reduction at the 500m target, achieved purely by giving the final approach leg into each terminal altitude the most distance (33km / 20km respectively) and the gentlest grade (~36 m/km / ~35 m/km, ≈2° descent), while intermediate step-down legs stayed shorter since they only needed to establish descending direction, not full convergence.

---

## 9. Stability Summary (V2 / AM2 run)

| | Value |
|---|---|
| Pitch min/max | −13.71° / +27.04° |
| TAS range | ~43.6 – 144.7 m/s (peak during RATO burn; settles to ~77–110 m/s cruise/climb/descent bands) |
| Altitude range (relative) | 0 → ~5280m, tracking the planned 5000m/1000m/5000m/500m profile throughout |
| Loiter behavior | Held 1103.2–1326.5m (mean 1236.4m) relative altitude over the ~112s loiter window (90s configured + entry/exit) — tight, stable, no drift or oscillation |
| Divergence/oscillation | None observed at any phase transition across the full ~2400s run |

---

## 10. Mode/Failsafe Summary (V2 / AM2 run)

| | Value |
|---|---|
| AUTO start | t=0.92s |
| RTL transition | t=2372.49s (immediately following `"Mission complete, changing mode to RTL"`) |
| Unexpected mode changes | None — exactly two mode transitions in the entire run (AUTO, then RTL) |
| Real failsafe/crash count | **0** (this run did not even produce the benign `"Throttle failsafe off"` message seen once in the AL/V1 run) |

---

## 11. What This Proves

- The preserved-state Gazebo→JSBSim→ArduPlane handoff mechanism (delayed JSBSim launch, synthetic held boot state, force-release on first real row) reliably reproduces the exact intended release IC (θ≈20°, TAS≈43.85 m/s) with **zero** elapsed decay, across every run in this series.
- `RATOController::resume_boost()` correctly resumes a partially-completed RATO burn (seeded from `RATO_RES_BURN≈0.768s`) and drives the full BOOST→BURNOUT→ENGINE_TAKEOVER→EJECT→COMPLETE state machine to completion under live ArduPlane control, with exact dry-booster-mass and engine-thrust-symmetry behavior, every time.
- ArduPlane's own live AUTO/TECS control — with **no gain or TECS tuning** — can fly the resumed aircraft through a long (~40 minute, ~211km), multi-phase altitude mission (climb, cruise, descend, loiter, ascend, cruise, descend, recovery) and reach a clean, automatic RTL on completion.
- Mission **waypoint geometry** alone (leg length and grade, independent of any control-law change) is sufficient to bring descent-phase altitude tracking from a >1400m miss down to within the preferred ±300m tolerance — confirming the AM1 diagnosis that the original shortfall was a mission-design issue, not a flight-control defect.
- All of the above holds simultaneously, in one continuous simulated flight, with exact engine symmetry and zero real failsafes.

---

## 12. What Is NOT Yet Validated

This entire series is **SITL-only** (JSBSim + ArduPlane software, no hardware in the loop). The following are explicitly **not** covered by this validation and must not be inferred from it:

- **Real ground landing.** Every mission in this series (V1 and V2) deliberately ends at a safe altitude (500m) and RTL, not a touchdown. Landing dynamics, flare, ground effect, and gear/runway interaction are untested.
- **Pixhawk/HIL hardware execution.** ArduPlane has only run as a SITL binary against a software JSBSim model. Real flight-controller hardware (CPU load, real-time scheduling jitter, hardware EKF/sensor drivers, real serial/CAN bus timing) has not been exercised.
- **Sensor/actuator hardware timing.** All sensor data (IMU, GPS, airspeed, barometer) has come from a perfect software model via the `sim_json` responder; real sensor noise, latency, dropout, and actuator (servo/ESC) response timing are untested.
- **Real engine/FADEC/RATO hardware.** RATO ignition/eject and turbojet thrust are simulated via JSBSim's propulsion model and PWM-driven latches in the test responder. Real RATO igniter behavior, FADEC engine control, and physical booster separation dynamics are untested.

---

## 13. Recommended Next Phase

**Migrate the validated V2 mission and RATO-resume logic to a Pixhawk/HIL bench.** Concretely:

1. Load the same `RATO_RESUME`/`RATO_RES_BURN`/`RATO_ENABLE` parameter set and the V2 mission (`SR75_F22_EXTENDED_ALTITUDE_PROFILE_V2.waypoints`) onto real Pixhawk-class flight-controller hardware running the same ArduPlane build validated here.
2. Exercise the RATO resume/eject logic against a HIL rig (real flight controller, simulated or bench-emulated RATO ignition/eject channels, real IMU/GPS hardware where feasible) to validate hardware timing that SITL cannot expose (scheduler jitter, real serial/CAN latency, actual servo/relay response time for `RATO_IGN_CH`/`RATO_EJ_CH`).
3. Only after HIL bench validation, consider extending the mission profile toward an actual landing phase — outside the scope of both this report and the current mission files.

---

*No code, aero, TECS, gain, RATOController, engine-mapping, or responder-protocol changes were made in the production of this report. No mission files were modified. Not pushed.*
