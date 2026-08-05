# HIL-F24-G: Real SR-75 JSBSim → SIM_JSON Sensor-State Pipeline Validation (Offline)

Status: **PASS**. Hardware PPP remains **BLOCKED_EXTERNAL_ADAPTER**
(unchanged from HIL-F24-D) — not investigated, per instruction. This
report does not repeat HIL-F24-F's synthetic-profile/acceptance-tooling
audits; it validates the **real SR-75 JSBSim model's** output through the
**real** `sr75_sim_json_responder.py` conversion path instead.

No PPP, no `/dev/ttyUSB0`, no flashing, no `PARAM_SET`, no arm/AUTO, no
actuator output. No aero/RATO/TECS/mission XML was modified — every
maneuver in the JSBSim script is a control-input event only.

## 1/2. Real JSBSim model through the real responder conversion path

New JSBSim runscript: `Tools/autotest/aircraft/sr_75_6_dof/scripts/
SR75_hil_f24g_pipeline_validation.xml` — **one continuous 74 s, 50 Hz run**
(3702 rows) through the real SR-75 6-DOF model, starting from the
already-validated `layer2j_b3_trim_init` reference, transitioning through
every representative state via timed control-input events only:

| Phase | Window (s) | What it demonstrates |
|---|---|---|
| static/release | 0–6 | trimmed hand-off IC (same as B3-family precontrol reference) |
| steady level flight | 6–16 | held trim, no additional input |
| gentle roll turn | 16–24 | bank rises to ~13°, settles near level; also the heading-change case (see below) |
| pitch climb | naturally 16–28 (theta rising) | see note below |
| descent | naturally 30–50 (theta falling) | see note below |
| RATO boost | 60–61 | partial-throttle (0.3), 1 s burn |
| post-burn/coast | 61–74 | RATO burnout, turbojet-only |

**Empirical note on maneuver tuning**: this airframe has essentially no
open-loop roll/pitch damping (a sustained control surface command
produces a roughly constant *rate*, not a converging angle — confirmed
empirically; consistent with `SR75_test_roll.xml`/`SR75_test_pitch.xml`
being pure sign diagnostics, not stability demonstrations, and with there
being no ArduPilot stabilization in this open-loop pipeline test). Initial
attempts using the same control magnitudes as those diagnostic scripts
(aileron 0.12–0.3, elevator delta 0.13, RATO throttle 1.0) produced a
divergent tumble. The script now uses small, empirically-verified-bounded
inputs instead: a bang-bang aileron pulse (±0.01) for the turn, small
elevator deltas (±0.02–0.04) for climb/descent, and partial (0.3) RATO
throttle for a 1 s burn. Because of the resulting phugoid coupling, the
clearest **climb** window is 16–28 s (theta 4.8° → 13.2°) and the clearest
**descent** window is 30–50 s (theta 12.0° → 0.4°), rather than lining up
exactly with the scripted elevator-step timestamps — the data is fully
valid and physically continuous either way; this is a data-pipeline
coherence test, not a flight-dynamics realism test. Full ejection/
separation dynamics (mass/aero property change) are out of scope per
instruction 10; only the post-burn/coast state is exercised.

**"Loopback or capture mode"**: implemented as **capture mode** — JSBSim
runs non-realtime (no `--realtime`; nothing consumes it live, so
deterministic as-fast-as-possible execution is used, matching prior HIL
sessions' finding that non-realtime JSBSim is far more reproducible than
`--realtime` when no live socket consumer is involved) producing a CSV,
which is then converted in one batch through the real
`StateMapper.state_from_csv()` — the exact function
`read_reply_state()` calls in the live responder. No ArduPlane, no PPP,
no live socket was needed to validate this stage of the pipeline.

New: `Tools/autotest/sr75_hil_layer2/scripts/sr75_hil_f24g_jsbsim_pipeline_validation.py`
— `run_jsbsim()` (thin I/O), `load_jsbsim_csv()`, `convert_rows()` (the
real conversion path), plus every check/report function below.

## 3. Exact conversion map

| SIM_JSON `SimState` field | Source JSBSim column | Transform |
|---|---|---|
| `latitude_deg`, `longitude_deg` | `position/lat-gc-deg`, `position/long-gc-deg` | passthrough |
| `altitude_m` | `position/h-sl-ft` (**ASL**, not `h-agl-ft`) | × 0.3048 |
| `roll_rad` | `attitude/phi-deg` | `radians()` |
| `pitch_rad` | `attitude/theta-rad` | passthrough (radians already; `theta-deg` also present, cross-checked, agrees) |
| `yaw_rad` | `attitude/psi-deg` (0–360°) | `radians()` then `normalize_yaw_rad()` → signed (−π, π] |
| `quaternion` | `q1..q4` (JSBSim-computed) or derived from roll/pitch/yaw if absent | passthrough / `euler_to_quaternion()` |
| `gyro_rad_s` (p,q,r) | `p_rad_s`/`q_rad_s`/`r_rad_s` (← `velocities/{p,q,r}-rad_sec`) | passthrough, **no sign flip** |
| `accel_body_mss` | `accel_body_{x,y,z}_mss` (← `accelerations/a-pilot-{x,y,z}-ft_sec2`) | × 0.3048, **no sign flip** |
| `velocity_ned_mps` | `velocities/v-{north,east,down}-fps` | × 0.3048, **no sign flip** (already NED) |
| `airspeed_mps` | `velocities/vt-fps` (**true** airspeed — checked before `vc-kts` in `state_from_csv()`'s recognized-name list) | × 0.3048 |

## 4. Frame/unit audit (item 5)

| Convention | Finding |
|---|---|
| JSBSim FRD vs ArduPilot body frame | **Same convention** (both X-forward/Y-right/Z-down); direct passthrough is correct. Empirically confirmed: `p_rad_s`/`q_rad_s`/`r_rad_s` signs match d(phi)/dt, d(theta)/dt, d(psi)/dt respectively throughout the capture. |
| NED signs | Confirmed correct: positive bank (φ>0, right bank) produces increasing ψ (right turn) — standard aerospace convention, no inversion. |
| Yaw wrapping | JSBSim outputs 0–360° heading; `normalize_yaw_rad()` wraps to **signed (−π, π] radians**, not 0–2π. Documented, not a defect. |
| Degrees vs radians | `phi-deg`/`psi-deg` are deg→rad converted; `theta-rad` is taken directly (already radians) — cross-checked against `theta-deg` independently, agrees to <1e-13°. |
| ft/s vs m/s | NED velocity and airspeed: × 0.3048, confirmed exact (< 1e-6 m/s residual, pure float rounding). |
| knots vs m/s | `vc-kts` (calibrated/indicated airspeed) exists in the raw JSBSim output but is **not** what gets transmitted — see "remaining gaps" below. |
| g vs m/s² | JSBSim's `accelerations/a-pilot-*-ft_sec2` are already specific-force (accelerometer-equivalent) values in ft/s²; only a unit conversion (× 0.3048) is applied, no g-to-m/s² scaling needed since JSBSim doesn't output g-units here. Level flight reads accel_body_z ≈ **−9.20 m/s²** (negative = Z-down/specific-force-up convention, matching `gravity_body_mss()`). |
| ASL vs AGL altitude | `altitude_m` maps from **ASL** (`h-sl-ft`), matching a barometric altimeter's pressure-altitude reference. `h-agl-ft` was added to this run's raw CSV specifically for this audit and confirmed **not** consumed by `state_from_csv()` — correct, since a barometer is not an AGL sensor. |
| Magnetic-field derivation prerequisites | `SimState.to_json_bytes()` carries no magnetic-field key at all — SIM_JSON's schema (as implemented) never transmits a raw mag vector; ArduPilot's own compass emulation (if enabled) would derive it internally via its WMM model from lat/lon + attitude. Those prerequisite fields (lat/lon, yaw/quaternion) are present and validated elsewhere in this report; no protocol gap requiring a code change. |

## 5. Conversion-error table (item 4)

Computed across all 3702 rows of the full capture (max absolute error):

| Field | Max abs error |
|---|---|
| `gyro_rad_s` (p,q,r) | 0.000e+00 (exact passthrough) |
| `accel_body_mss` | 0.000e+00 (exact passthrough) |
| `velocity_ned_mps` | 0.000e+00 |
| `airspeed_mps` | 0.000e+00 |
| `altitude_m` | 0.000e+00 |
| `latitude_deg`/`longitude_deg` | 0.000e+00 |
| `roll_deg` | 1.590e-15 |
| `pitch_deg` | 3.181e-15 |
| `yaw_deg` | 1.018e-13 |

All errors are at or below float64 machine precision — the conversion is
lossless; every non-zero residual is pure floating-point round-off from
`radians()`/multiplication, not a mapping defect.

## 6. Regression captures (item 6)

New: `Tools/autotest/sr75_hil_layer2/sim_json/f24g_regression_captures/`
— one deterministic JSON capture per representative phase (window,
sampled `SimState` fields):

| Capture | Window (s) |
|---|---|
| `static_release.json` | 0.0–2.0 |
| `steady_flight.json` | 8.0–12.0 |
| `turning.json` | 17.0–21.0 |
| `climb_descent.json` | 22.0–34.0 |
| `rato_boost.json` | 60.0–63.0 |

`save_regression_captures()`/`compare_regression_capture()` support
re-running the same JSBSim script and confirming bit-for-bit (< 1e-9)
determinism against a saved capture — both the JSBSim model and the
conversion path are fully deterministic, confirmed by test
`test_compare_regression_capture_matches_fresh_conversion`.

## 7. HIL-F24-F acceptance-analyzer integration (item 7)

`run_f24f_acceptance_check()` builds F24-F's `CSV_FIELDS`-shaped telemetry
records directly from the real converted `SimState` rows and runs them
through F24-F's real `evaluate_acceptance()` (self-consistency
application — no hardware telemetry exists yet, so "commanded" and
"actual" are the same data here). Result: **PASS**, 0 violations,
demonstrating F24-F's analyzer/thresholds are structurally compatible
with real dynamic JSBSim-derived data, not only the synthetic profiles it
was originally built against.

## 8. Pipeline health report (item 8)

| Metric | Value |
|---|---|
| Packet rate | 50.00 Hz (measured from 3701 inter-row deltas) |
| Max stale gap | 0.0200 s (no gaps — clean, complete CSV) |
| Malformed packets | 0 / 3702 |
| Missing required fields | 0 / 3702 |
| Max conversion error (any field) | 1.018e-13° (yaw), all others ≤ 3.18e-15 |
| Phase-transition discontinuities | **0 found** across all 10 scripted event boundaries (6, 16, 20, 24, 30, 40, 50, 60, 61, 62 s) — `check_phase_transition_continuity()` confirms every one-tick (0.02 s) state delta stays within 5°/0.1 s bounds, i.e. the pipeline itself introduces no artificial jump even exactly at a control-input step |

The (deliberately reused, not reimplemented) malformed/missing-field
staleness code paths were already re-verified in HIL-F24-F
(`test_stale_timeout`/`test_missing_required_fields`/
`test_malformed_row_rejected`) — not repeated here per instruction.

## Remaining gaps

- **True vs. indicated/calibrated airspeed**: the pipeline transmits
  JSBSim's **true** airspeed (`vt-fps`); calibrated/indicated airspeed
  (`vc-kts`) is present in JSBSim's own output but never reaches SIM_JSON.
  Whether ArduPilot's `ARSPD_TYPE=100` SITL backend expects true or
  indicated airspeed was not verified here (no ArduPilot in this task's
  loop) — flagged for a future task that does exercise ArduPilot's
  airspeed/EKF response, not a defect found in this one.
- **RATO at cruise IC, not launch IC**: the RATO segment in this run is
  exercised at cruise altitude/airspeed (continuous-script constraint,
  documented in the runscript), not `rato_init.xml`'s intended low-speed
  launch condition — realistic RATO-launch dynamics remain covered by
  the pre-existing `SR75_rato_takeoff_test.xml`, not duplicated here.
- **No real hardware telemetry yet**: item 7's F24-F integration is
  necessarily a self-consistency check (PPP still BLOCKED_EXTERNAL_
  ADAPTER); a true commanded-vs-actual comparison awaits HIL-F24-D's
  adapter replacement.

## Future real-JSBSim hardware command (NOT executed)

Once the adapter is replaced (HIL-F24-D/F24-F checklist) and PPP is
confirmed up, feeding this same real JSBSim model to the Pixhawk over
SIM_JSON would look like (still gated by HIL-F24-F's orchestrator
`--dry-run` default; shown here for reference only, not executed):

```sh
# 1. Run the real JSBSim model non-realtime is NOT appropriate for a live
#    hardware feed (Pixhawk needs a real-time-paced stream) -- use --realtime:
JSBSim --root=Tools/autotest \
  --script=aircraft/sr_75_6_dof/scripts/SR75_hil_f24g_pipeline_validation.xml \
  --realtime &

# 2. Feed its live-updating CSV through the real responder over PPP,
#    LOG_ONLY (no actuator/JSBSim command target -- never enables output):
python3 Tools/autotest/sr75_hil_layer2/sim_json/sr75_sim_json_responder.py \
  --listen-host 192.168.144.2 --listen-port 9002 \
  --state-file Tools/autotest/sr75_hil_f24g_pipeline_validation.csv \
  --log-csv /tmp/sr75_hil_f24g_hw/responder.csv --strict

# 3. Orchestrated end-to-end (precheck -> PPP start/verify -> feeder/
#    responder -> always stop PPP), per HIL-F24-F:
python3 Tools/autotest/sr75_hil_layer2/scripts/sr75_hil_f24f_hardware_orchestrator.py \
  --execute --confirm-static-only
```

(The orchestrator's current scope is the static test only; extending it
to accept this JSBSim-driven dynamic feed is noted as follow-up scope,
not built in this task.)

## Files changed

New:
- `Tools/autotest/aircraft/sr_75_6_dof/scripts/SR75_hil_f24g_pipeline_validation.xml`
- `Tools/autotest/sr75_hil_layer2/scripts/sr75_hil_f24g_jsbsim_pipeline_validation.py`
- `Tools/autotest/sr75_hil_layer2/scripts/test_sr75_hil_f24g_jsbsim_pipeline_validation.py`
- `Tools/autotest/sr75_hil_layer2/sim_json/f24g_regression_captures/{static_release,steady_flight,turning,climb_descent,rato_boost}.json`
- `Tools/autotest/sr75_hil_layer2/reports/HIL_F24_G_jsbsim_sim_json_pipeline_validation.md` (this file)

No existing HIL-F24-F/B/C/D files modified (item 7 imports F24-F's
analyzer module directly; nothing in it was changed).

Not committed (only commit when explicitly asked).

## Regression results

New this task: 30 tests, all passing
(`test_sr75_hil_f24g_jsbsim_pipeline_validation.py`) — including 9
deliberate-fault-injection tests confirming each coherence check actually
detects a broken row (not a tautological pass), 2 frame/sign-audit tests
(one confirming the real capture's conventions, one confirming an
inverted-sign series is caught), and one live end-to-end run of the
script itself (`OVERALL: PASS`, shown above).

Full regression this session: `scripts/` 291 passing (261 pre-existing +
30 new), `sim_json/` 114 passing (unchanged), `bridge/` 52 passing
(unchanged), `jsbsim_control/` 6 passing + **2 pre-existing failures**
(`test_sr75_sim_json_command_accounting.py`, unrelated — untouched in
this task, same as noted in HIL-F24-B/C/F). py_compile / flake8
(`.flake8`) / XML well-formedness / `git diff --check`: all clean.

## PASS/FAIL

**PASS.** The real SR-75 JSBSim model, run through the real
`sr75_sim_json_responder.py` conversion path, produces coherent,
loss-free (machine-precision) SIM_JSON sensor fields across every
representative state requested — static/release, steady flight, gentle
roll turn with heading change, climb, descent, RATO boost, and post-burn
coast — with no frame or sign inversion found, no phase-transition
discontinuities introduced by the pipeline, and full compatibility with
HIL-F24-F's acceptance-analyzer tooling. Hardware execution remains
**BLOCKED_EXTERNAL_ADAPTER**, unchanged and not investigated per
instruction.
