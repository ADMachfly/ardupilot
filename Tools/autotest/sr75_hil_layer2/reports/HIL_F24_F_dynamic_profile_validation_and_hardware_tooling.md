# HIL-F24-F: Offline SIM_JSON Dynamic-Profile Validation & Hardware-Ready Acceptance Tooling

Status: **PASS** (offline/SITL-loopback scope). Hardware execution remains
**BLOCKED_EXTERNAL_ADAPTER** (see `HIL_F24_D_static_hardware_blocked_
adapter.md`) — not investigated or required in this task, per instruction.

No hardware sensor injection, arming, AUTO, actuator output, flashing,
`PARAM_SET`, Pixhawk configuration change, or PPP start occurred in this
task. Everything below ran through the existing SITL/loopback (no-
hardware) SIM_JSON code paths already established in HIL-F24-B.

## 1. F24-D status preserved

`HIL_F24_D_static_hardware_blocked_adapter.md` (new) records **BLOCKED_
EXTERNAL_ADAPTER**, explicitly distinguishing "the CP2102 adapter fails
its own TX/RX loopback test" from any firmware/parameter/protocol defect.
Firmware readiness (HIL-F24-C) remains **GO**; only the physical adapter
blocks the real-hardware static test. Not investigated further here, per
this task's instruction.

## 2/3. Dynamic-profile validation through SITL/loopback SIM_JSON paths

All profiles run through the same no-hardware validation approach
established in HIL-F24-B (`sr75_hil_f24b_sim_json_no_hardware_tests.py`,
reused directly — not reimplemented) via the new harness
`sr75_hil_f24f_dynamic_profile_validation.py`.

### Profile definitions (new, in `sr75_sim_json_test_profiles.py`)

All use an exact piecewise-linear multi-leg model (`_piecewise_linear()` /
`_leg_waypoints()`) so each profile's rate field (`gyro_rad_s`/NED `vd`) is
the analytic derivative of its own leg, not an approximation:

| Profile | Function | Default legs |
|---|---|---|
| static level | `profile_static_level` (HIL-F24-B, reused unchanged) | level, motionless |
| roll sweep | `profile_roll_sweep_multileg` | 0 → +15 → −15 → 0 deg |
| pitch sweep | `profile_pitch_sweep_multileg` | 0 → +15 → −10 → 0 deg |
| yaw sweep | `profile_yaw_sweep_multileg` | 315 → 270 → 315 deg |
| altitude ramp | `profile_altitude_ramp_multileg` | 3000 → 3020 → 2980 → 3000 m |
| combined gentle | `profile_combined_gentle` | roll 0↔5°, pitch 0↔3°, yaw 315↔320°, alt 3000↔3010 m, simultaneously |

Registered in a new `PROFILES_F24F` dict, additive alongside HIL-F24-B's
existing `PROFILES` dict (left completely unchanged — verified by
regression test `test_original_profiles_registry_unchanged`).

- **Stale-data stop**: reuses HIL-F24-B's `test_stale_timeout()` — the
  real `read_reply_state()`/`state_timeout_ms` code path against a
  genuinely back-dated CSV file.
- **Malformed/missing required fields**: reuses HIL-F24-B's
  `test_missing_required_fields()` / `test_malformed_row_rejected()` — the
  real `StateMapper.state_from_csv()` validation path.

### Coherence checks (item 3, new in the F24-F harness)

- **Attitude/quaternion**: reconstructs Euler angles from each row's
  quaternion (norm checked == 1.0) and requires agreement with the row's
  own roll/pitch/yaw within **0.1°** (same method used to validate SR-75
  Layer 2J-B3C-C3's outgoing attitude).
- **Gyro matching attitude rates**: finite-differences the actual roll/
  pitch/yaw trajectory between consecutive rows and requires it match the
  profile's own analytic gyro within **0.02 rad/s**, except at a verified
  leg-boundary corner (a sample pair is only exempted when the profile's
  own analytic gyro *actually* changes across it by more than the
  tolerance — the exemption is checked, not assumed).
- **Gravity-consistent body acceleration**: recomputed via the same
  `gravity_body_mss()` the responder itself uses.
- **NED position/velocity**: altitude change between consecutive rows
  must match the trapezoidal integral of `vd`; lat/lon must not drift
  whenever `vn`/`ve` are both zero.
- **Airspeed**: finite, non-negative at every row.
- **Timestamps**: strictly monotonic, instantaneous rate within 0.5 Hz of
  the configured 50 Hz.

All six coherence checks were also run against HIL-F24-B's original
sinusoidal/ramp profiles (`roll_sweep`, `pitch_sweep`, `yaw_sweep`,
`altitude_ramp`) as an extra cross-check — all pass unchanged.

## 4/5. Hardware acceptance analyzer + per-category thresholds

New: `sr75_hil_f24f_hardware_acceptance_analyzer.py`. Offline, read-only —
analyzes an already-captured log; never opens a live connection to
hardware, never sends `PARAM_SET`/arm/mission/actuator commands.

Accepts **either** format, both producing the same internal record shape
so `evaluate_acceptance()` is one pure, fully-tested code path:
- `--csv PATH`: the `CSV_FIELDS` schema this report defines (the schema
  the future orchestrator writes once hardware exists).
- `--mavlink PATH`: any pymavlink-readable log, fused via `AHRS2`,
  `GLOBAL_POSITION_INT`, `VFR_HUD`, `EKF_STATUS_REPORT`, `SYS_STATUS`,
  `SERVO_OUTPUT_RAW`, `HEARTBEAT` (last-known-value fusion, one merged
  record per `HEARTBEAT`, same read-only `recv_match()` pattern already
  used throughout this task family).

Checks performed per matched (commanded, actual) sample and across the
whole recording: commanded-vs-AHRS2 attitude error; GPS/global-position
error (haversine, meters); altitude error; airspeed error; EKF flags/
health (`EKF_STATUS_REPORT` required-healthy bitmask) and reboot detection
(`boot_time_s` going backwards); CPU load (`SYS_STATUS.load`); stale
inter-record gaps; `armed`/mode-not-MANUAL; CH7/CH8 PWM activity (reusing
HIL-F24-C's `CH_PWM_QUIET_MAX` convention).

A guard in `build_paired_samples()` raises `AnalyzerError` (rather than
silently mismatching) if the telemetry span is far shorter than the
commanded profile's span — this was caught by its own test during
development (`test_raises_when_telemetry_span_much_shorter_than_
commanded`) and prevents a duration/rate mismatch from being misreported
as a tracking-error violation.

### Thresholds by category (item 5)

| | attitude (deg) | position (m) | altitude (m) | airspeed (m/s) | CPU load max (%) | stale gap max (s) |
|---|---|---|---|---|---|---|
| **static** | 1.0 | 2.0 | 1.5 | 1.0 | 90 | 0.5 |
| **individual sweep** | 3.0 | 3.0 | 3.0 | 1.5 | 90 | 0.5 |
| **combined gentle** | 4.0 | 4.0 | 4.0 | 2.0 | 95 | 0.75 |

Static is tightest (no dynamic tracking lag expected). Individual sweeps
loosen attitude/rate tolerance for normal EKF/servo/aero tracking lag
while keeping position/altitude/airspeed tight (only one axis moves at a
time). Combined gentle is loosest, since simultaneous multi-axis
excitation is expected to produce somewhat larger transient error even
though each individual axis's amplitude is small by design. These
thresholds are a documented judgment call — no real hardware data exists
yet to calibrate against; they should be revisited once a real static/
sweep log is captured.

## 6/7. Future hardware orchestrator, `--dry-run` default

New: `sr75_hil_f24f_hardware_orchestrator.py`. Scope is intentionally
narrow: **static SIM_JSON test only** (dynamic profiles are not wired to
real hardware in this task — validated only via SITL/loopback above).

Plan (`build_execution_plan()`, same list drives both the dry-run print
and real execution, so they cannot drift apart):
1. Check `/dev/ttyACM0` (Pixhawk) and `/dev/ttyUSB0` (PPP adapter) exist.
2. Read-only SoH precheck (subprocess: `sr75_hil_f24c_preflash_
   precheck.py`) — abort here on STOP, before touching PPP.
3. Start PPP (`sr75_ppp_start.sh --background`) — only reached with
   `--execute --confirm-static-only`.
4. Verify `ppp0` up + ping the Pixhawk PPP endpoint
   (`sr75_ppp_status.sh`) — abort (and still stop PPP) before SIM_JSON if
   this fails.
5. Start the feeder (`sr75_hil_f24d_static_state_feed.py`) **before** the
   responder.
6. Start the responder in **LOG_ONLY** mode — no `--jsbsim-command-
   target`/`--actuator-map` at all, so there is no actuator/PWM command
   path in this configuration (verified by test:
   `test_responder_command_has_no_actuator_or_jsbsim_flags`).
7. Records every artifact (precheck/PPP/feeder/responder logs) under a
   timestamped `hardware_sessions/sr75_hil_f24f_static_<UTC>/` directory.
8. `finally:` block **always** stops the feeder/responder and **always**
   stops PPP, whether the run succeeded, aborted, or raised.

`--dry-run` is the default (no flag needed) — verified live:
```sh
$ python3 Tools/autotest/sr75_hil_layer2/scripts/sr75_hil_f24f_hardware_orchestrator.py
dry-run (default): pass --execute --confirm-static-only to run against real hardware
Execution plan (dry-run -- nothing below has been executed):
  1. [check_adapters] ...
  ...
```
Neither `--execute` alone nor `--confirm-static-only` alone proceeds
(`should_proceed_to_hardware()`, unit-tested for all four flag
combinations) — **both** are required together, exactly matching item 7.

## 8. Adapter-replacement checklist

Before substituting a replacement USB-UART adapter for the failing
CP2102 unit (see `HIL_F24_D_static_hardware_blocked_adapter.md`):

- [ ] **3.3 V TTL levels only** — never RS-232 voltage levels.
- [ ] **TX/RX/GND wired**, TX↔RX **crossed** (Pixhawk TELEM1 TX → adapter
      RX, Pixhawk TELEM1 RX → adapter TX), GND common to both ends.
- [ ] **Verified 921600 baud support** — confirm the adapter's chipset
      datasheet lists 921600 as a supported rate (`SERIAL1_BAUD 921` is
      the bench's parameter value) before wiring it up at all.
- [ ] **Loopback test required BEFORE Pixhawk connection** — short the
      adapter's own TX and RX pins and confirm bytes written are read
      back correctly, at 921600 baud, with the adapter alone (this is the
      exact test the current CP2102 unit fails).
- [ ] **No RS-232 voltage adapter** in the signal path — RS-232 levels
      (±12 V-class) will damage the Pixhawk's 3.3 V UART.
- [ ] **5 V line disconnected** — many USB-UART adapters expose a 5 V pin
      alongside TX/RX/GND; it must stay disconnected from the Pixhawk
      side (power comes from USB only, never fed back into TELEM1).

Only once every box above is checked should PPP wiring proceed per
HIL-F24-C §4, followed by `sr75_hil_f24f_hardware_orchestrator.py
--execute --confirm-static-only`.

## 9. Explicit non-actions (per instruction)

No flashing. No `PARAM_SET`. No change to current Pixhawk configuration.
No PPP started. No hardware SIM_JSON injected. No arm/AUTO. No actuator
feedback enabled. No RATO/aero/TECS/mission logic modified. Confirmed by
code review of every new/modified file (no `PARAM_SET`, `arm`, or mission
call anywhere) and by `test_no_step_contains_arm_or_mission_flags` /
`test_responder_command_has_no_actuator_or_jsbsim_flags` in the
orchestrator's own test suite.

## Files changed

New:
- `Tools/autotest/sr75_hil_layer2/reports/HIL_F24_D_static_hardware_blocked_adapter.md`
- `Tools/autotest/sr75_hil_layer2/reports/HIL_F24_F_dynamic_profile_validation_and_hardware_tooling.md` (this file)
- `Tools/autotest/sr75_hil_layer2/scripts/sr75_hil_f24f_dynamic_profile_validation.py`
- `Tools/autotest/sr75_hil_layer2/scripts/test_sr75_hil_f24f_dynamic_profile_validation.py`
- `Tools/autotest/sr75_hil_layer2/scripts/sr75_hil_f24f_hardware_acceptance_analyzer.py`
- `Tools/autotest/sr75_hil_layer2/scripts/test_sr75_hil_f24f_hardware_acceptance_analyzer.py`
- `Tools/autotest/sr75_hil_layer2/scripts/sr75_hil_f24f_hardware_orchestrator.py`
- `Tools/autotest/sr75_hil_layer2/scripts/test_sr75_hil_f24f_hardware_orchestrator.py`
- `Tools/autotest/sr75_hil_layer2/sim_json/test_sr75_sim_json_test_profiles.py`

Modified:
- `Tools/autotest/sr75_hil_layer2/sim_json/sr75_sim_json_test_profiles.py`
  (additive: `_piecewise_linear`/`_leg_waypoints` helpers, 5 new multi-leg
  profile functions, `PROFILES_F24F` registry; also reformatted several
  pre-existing function signatures that were not actually flake8-clean —
  no behavior change, confirmed by the unchanged HIL-F24-B suite passing)

Not committed (only commit when explicitly asked).

## Test results

New this task: 20 (`test_sr75_sim_json_test_profiles.py`) + 14
(`test_sr75_hil_f24f_dynamic_profile_validation.py`) + 24
(`test_sr75_hil_f24f_hardware_acceptance_analyzer.py`) + 12
(`test_sr75_hil_f24f_hardware_orchestrator.py`) = **70 new tests, all
passing**. Standalone harness run: `sr75_hil_f24f_dynamic_profile_
validation.py` → `OVERALL: PASS` (9/9 checks).

Full regression this session:
- `bridge/`: 52 passing (unchanged)
- `scripts/`: 261 passing (209 pre-existing + this task's additions)
- `sim_json/`: 114 passing (unchanged pre-existing + this task's
  additions)
- `jsbsim_control/`: 6 passing, **2 pre-existing failures**
  (`test_sr75_sim_json_command_accounting.py`, unrelated —
  `jsbsim_control/` was not touched in this task, matches the
  pre-existing failure already noted in HIL-F24-B/C)
- `ppp/`: 0 tests collected (no hardware code touched here either)

py_compile / flake8 (`.flake8`, max-line-length=127): clean on every
new/modified file. `git diff --check`: clean.

## PASS/FAIL

**PASS** for everything in this task's actual scope (offline SITL/
loopback dynamic-profile validation, coherence checks, acceptance
analyzer + thresholds, dry-run-gated orchestrator, adapter checklist).
Real hardware execution remains **BLOCKED_EXTERNAL_ADAPTER**
(`HIL_F24_D_static_hardware_blocked_adapter.md`) — explicitly not
investigated or required per this task's instruction, and not a firmware
or software failure.
