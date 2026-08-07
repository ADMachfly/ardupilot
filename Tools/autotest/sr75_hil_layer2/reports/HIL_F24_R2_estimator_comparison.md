# HIL-F24-R2: Pixhawk Estimator vs. Live SR-75 JSBSim Truth Comparison

Status: implemented and validated host-side, including a real-JSBSim +
synthetic-Pixhawk end-to-end pipeline run. **No real Pixhawk/PPP hardware
was touched by this task** -- that remains the operator's own guarded,
manual step (see "Guarded 30-second hardware command" below). No
firmware was modified or flashed. No PARAM_SET, arm, mode change,
mission item, RC override, or actuator command is ever sent by anything
in this task -- the capture process only ever requests read-only
`MAV_CMD_SET_MESSAGE_INTERVAL` stream rates and receives telemetry.

## What changed, in one sentence

A new read-only capture process records synchronized Pixhawk-estimator
and live-JSBSim-truth time series over the already-validated HIL-F24-R1
dynamic PPP/SIM_JSON path, and a new offline comparison script aligns
them by host-monotonic timestamp and scores the estimator against truth
-- both wired into the orchestrator behind a new, uniquely-guarded
`--stage r2` flag that leaves the static and R1-dynamic profiles
untouched.

## Task 1: read-only capture process

`sr75_hil_f24r2_estimator_capture.py` connects to `--pixhawk` (default
`/dev/ttyACM0`) at `--baud` (default 115200) via
`mavutil.mavlink_connection(..., autoreconnect=True)`, waits for a
heartbeat, then calls `request_message_intervals()` -- a
`MAV_CMD_SET_MESSAGE_INTERVAL` command per message type (GLOBAL_POSITION_INT,
LOCAL_POSITION_NED, ATTITUDE, GPS_RAW_INT, VFR_HUD, EKF_STATUS_REPORT,
SYS_STATUS at 10/10/10/5/10/2/1 Hz respectively -- HEARTBEAT streams by
default and STATUSTEXT is event-driven, so neither is rate-requested),
the same read-only request pattern `sr75_hil_f24c_preflash_precheck.py`
already uses for `SERVO_OUTPUT_RAW`. It then loops until `--duration-s`
elapses, draining `master.recv_match(blocking=False)` and converting
every recognized message via the pure function `mavlink_message_to_row()`.
**No PARAM_SET, arm/disarm, mode change, mission item, RC override, or
actuator command exists anywhere in this file** -- confirmed by
inspection and by an orchestrator-level test
(`test_capture_step_uses_pixhawk_device_read_only_and_no_actuator_flags`)
that scans the built command line for those substrings.

## Task 2: recorded files, existing artifacts preserved

Every session directory now gets, in addition to the unchanged
`state.csv`/`responder.csv`/`feeder.log`/`responder.log`/`ppp_*.log`/
(for `--profile dynamic`) `jsbsim_state.csv`/`postcheck.log`:

- `jsbsim_truth.csv` -- one row per distinct new `state.csv` snapshot
  (via `TruthTailer`, built on the **unmodified**
  `sr75_sim_json_responder.LatestCSVReader`), tagged with this process's
  own `time.monotonic()`.
- `pixhawk_estimator.csv` -- one row per received MAVLink message
  (wide/sparse schema, `PIXHAWK_CSV_FIELDS`; a field only populated for
  the message types it applies to), tagged the same way.
- `estimator_comparison.csv` -- one row per aligned `GLOBAL_POSITION_INT`
  sample, with per-metric errors.
- `estimator_summary.json` and `estimator_summary.md` -- aggregate
  stats, checks, and overall PASS/FAIL.

Nothing pre-existing is renamed, moved, or overwritten -- `session_
artifact_paths()` (new orchestrator helper) adds these 5 new
`{placeholder}` names alongside the 2 that already existed
(`state_csv`, `responder_csv`).

## Task 3: alignment by host monotonic timestamp only

`nearest_within_tolerance()` (bisect-based, O(log n)) matches each
Pixhawk sample to the nearest truth/co-Pixhawk-message-type sample by
`host_monotonic_time` alone -- `host_wall_time` is recorded in
`pixhawk_estimator.csv` for human debugging only and is never read by
any alignment or comparison logic. A match beyond `--alignment-tolerance-s`
(default 0.1s, `DEFAULT_ALIGNMENT_TOLERANCE_S`) is treated as "no truth
available for this sample" rather than a bad match. `GLOBAL_POSITION_INT`
is the primary comparison tick (task 4's position/velocity requirement);
the nearest `ATTITUDE` and `VFR_HUD` samples within tolerance are folded
into the same row. `align_and_compare()` returns both the row list and
`coverage` (fraction of GLOBAL_POSITION_INT samples that found a truth
match within tolerance) -- the basis for the `alignment_coverage >= 0.95`
check.

## Task 4: comparison metrics

Per aligned row, `align_and_compare()` computes:

| Task 4 requirement | Field(s) | Method |
|---|---|---|
| Lat/lon horizontal distance | `horizontal_error_m` | `horizontal_distance_m()`, equirectangular approximation |
| Altitude error | `altitude_error_m` | `gpi_alt_m - truth.alt_m` |
| VN/VE/VD error | `vn/ve/vd_error_mps` | GLOBAL_POSITION_INT NED velocity minus truth |
| Roll/pitch/yaw error | `roll/pitch/yaw_error_deg` | ATTITUDE minus truth, yaw via `yaw_error_deg()` (wrap-aware) |
| p/q/r error | `p/q/r_error_deg_s` | ATTITUDE body rates minus truth |
| Airspeed error | `airspeed_error_mps` | VFR_HUD `airspeed` minus truth `airspeed_mps` |
| Transport/estimator latency | `alignment_dt_s` | signed `pixhawk_t - truth_t` for the matched pair, recorded per row |

`compute_summary()` aggregates max/mean/RMS/p95 per metric (via the
reused `sr75_hil_f24d_static_state_feed._percentile()`) over the
post-settle steady-state window (task 7).

## Task 5: convention validation

| Convention | Truth (JSBSim, via `state.csv`) | Pixhawk message | Handling |
|---|---|---|---|
| Altitude datum | `alt_m`, geodetic (runscript `position/geod-alt-m`, matching the R1-validated field mapping) | GLOBAL_POSITION_INT `alt` is AMSL (mm) | Compared directly; any fixed datum offset is expected to show up as a **constant bias** in `altitude_error_m`'s `mean`, reported separately from `rms` in the summary table -- not folded into or hidden by the RMS check |
| NED signs, VD positive down | `vd_mps` (truth) and `gpi_vd_mps = msg.vz * 1e-2` (Pixhawk) both positive-down by MAVLink/JSBSim NED convention | -- | Compared directly with no sign flip; a sign-convention bug would show up as a large, systematic (not random) `vd_error_mps` |
| Radians vs. degrees | Truth stored in radians (`roll_rad`/`pitch_rad`/`yaw_rad`/`p_rad_s`/`q_rad_s`/`r_rad_s`, matching R1's JSBSim runscript fields) | ATTITUDE is radians on the wire; converted to degrees in `mavlink_message_to_row()` (`att_roll_deg = math.degrees(msg.roll)`, etc.) | All error math done in **degrees**; truth is converted via `math.degrees()` at comparison time, never compared in mixed units |
| Lat/lon scaling | Truth: plain degrees (`lat_deg`/`lon_deg`) | GLOBAL_POSITION_INT/GPS_RAW_INT: `degE7` on the wire | Converted in `mavlink_message_to_row()` (`* 1.0e-7`) before ever reaching comparison code |
| Yaw wrap | -- | ATTITUDE yaw and GLOBAL_POSITION_INT hdg can wrap at ±180°/360° | `yaw_error_deg()` computes `(a - b + 180) % 360 - 180`, confirmed by test (`yaw_error_deg(179, -179) == -2`, not 358); `detect_attitude_jumps()` uses the same helper so a legitimate wrap is never misreported as a discontinuity |
| Airspeed source/units | Truth `airspeed_mps` (JSBSim true airspeed, m/s, matching R1's field mapping) | VFR_HUD `airspeed` (m/s on the wire, EAS/CAS depending on AHRS config) | Compared directly in m/s; if the Pixhawk's true-vs-equivalent airspeed source differs from JSBSim's TAS at non-trivial altitude/density, that is a **provisional-threshold caveat** noted below, not something this task's math can distinguish from a real estimator error |

`hdg == 65535` (MAVLink's "unknown heading" sentinel) is explicitly
excluded (`gpi_hdg_deg` left `None`), confirmed by test
(`test_global_position_int_hdg_65535_treated_as_none`).

## Task 6: ground-static "no control input" validation

R1's JSBSim runscript is ground-static (engines off, all controls
pinned at zero, `do_simple_trim`) -- there is no control input in this
architecture at all, so this requirement reduces to the comparison
script correctly *detecting* whether the Pixhawk followed that stable
truth, which the summary's checks already cover directly:

- **No large position drift**: `horizontal_error_rms` /
  `altitude_error_rms` checks against 5m/3m thresholds.
- **Attitude stays close to truth**: `roll_error_rms` / `pitch_error_rms`
  / `yaw_error_rms` checks against 2°/2°/5° thresholds.
- **Velocities near zero**: `vn/ve/vd_error_rms` checks against 1.5 m/s
  thresholds (truth itself is ~0 m/s in all three axes on a ground-static
  run, so a passing VN/VE/VD RMS check here also directly demonstrates
  "near zero", not just "close to truth").
- **Airspeed near simulated value**: `airspeed_error_rms` check against
  2 m/s.
- **No EKF reset/lane switch/unhealthy status**:
  `scan_ekf_and_statustext_health()` -- `EKF_STATUS_REPORT.flags` bitwise
  against `EKF_UNHEALTHY_FLAGS` (`EKF_CONST_POS_MODE|EKF_UNINITIALIZED|
  EKF_GPS_GLITCHING = 33920`), `STATUSTEXT` substring scan
  (`RESET_STATUSTEXT_SUBSTRINGS`, case-insensitive), plus HEARTBEAT
  armed/non-MANUAL scanning as defense-in-depth alongside the pre/post
  precheck -- feeds the `no_ekf_or_statustext_reset_events` check.

## Task 7: automated checks

`compute_summary()`'s `checks` dict (11 checks, all boolean, `overall_pass
= all(checks.values())`, `first_failure` = first failing check name in
insertion order):

`finite_values`, `monotonic_truth_timestamps`, `monotonic_pixhawk_
timestamps`, `sample_count_minimum` (>= 20 GLOBAL_POSITION_INT and >= 20
ATTITUDE samples, `DEFAULT_MIN_SAMPLES`), `update_rate_sane` (median
inter-sample interval <= 1.0s for GLOBAL_POSITION_INT/ATTITUDE/VFR_HUD,
`DEFAULT_MAX_MEDIAN_INTERVAL_S`), `alignment_coverage`,
`no_ekf_or_statustext_reset_events`, `no_large_attitude_jump` (via
`detect_attitude_jumps()`, > 10° consecutive-sample jump on roll, pitch,
or yaw), then the 9 RMS threshold checks (horizontal/altitude/roll/
pitch/yaw/vn/ve/vd/airspeed).

`finite_values` is enforced by construction, not by a post-hoc scan:
`load_truth_csv()`/`load_pixhawk_csv()` `raise ValueError` immediately on
any non-finite numeric field while parsing -- there is no code path by
which a NaN/Inf value could ever reach `compute_summary()` in the first
place (confirmed by test, `test_nonfinite_truth_field_raises`).

**Startup settling period**: `--settle-s` (default 5.0,
`DEFAULT_SETTLE_S`) excludes comparison rows whose
`host_monotonic_time - run_start < settle_s` from every metric's
max/mean/RMS/p95 and from every RMS-threshold check -- but **every** row,
settled or not, is still written to `estimator_comparison.csv` in full,
so nothing is hidden, only excluded from steady-state *scoring*.

## Task 8: guarded orchestrator stage flag

`sr75_hil_f24f_hardware_orchestrator.py` gained:

- `--stage {none,r2}` (default `none` -- every existing invocation
  without this flag behaves byte-for-byte as before; confirmed by
  `test_default_stage_none_plan_unchanged` and `test_legacy_calls_
  without_stage_kwarg_still_reproduce_static_behavior`).
- `--confirm-estimator-comparison`, a **third**, uniquely-required
  confirmation flag -- `should_proceed_to_hardware()` checks `stage ==
  "r2"` first (requiring this flag specifically), then falls back to the
  existing `profile`-based dynamic/static logic. Neither
  `--confirm-static-only` nor `--confirm-dynamic-jsbsim` can authorize a
  `--stage r2` run, and `--confirm-estimator-comparison` cannot authorize
  a non-r2 run either way (5 dedicated tests, `TestShouldProceedToHardwareR2Stage`).
- `--stage r2` requires `--profile dynamic`; `main()` checks this
  explicitly and aborts as a (still harmless) dry run with a clear
  message (`"--stage r2 requires --profile dynamic -- staying in
  dry-run"`) if violated, confirmed by a CLI subprocess test.
- `--r2-alignment-tolerance-s` (default 0.1) / `--r2-settle-s` (default
  5.0), passed straight through to the comparison script's own flags of
  the same meaning.
- `build_execution_plan()` appends two new `Step`s only when
  `stage == "r2"` (`start_estimator_capture`, `run_estimator_comparison`)
  between `start_responder` and `ppp_stop` -- the static and
  dynamic-without-r2 plans are unaffected (confirmed:
  `test_default_stage_none_plan_unchanged`,
  `test_static_profile_plan_unaffected_by_stage_default`).
- `build_session_dir()` names r2 sessions
  `sr75_hil_f24r2_estimator_<timestamp>`, distinct from both
  `sr75_hil_f24f_static_<timestamp>` and `sr75_hil_f24r1_dynamic_
  <timestamp>` (confirmed by test).
- New `session_artifact_paths(session_dir)` helper centralizes all 7
  `{placeholder}` names so `print_plan()`/`run_step()` can never drift
  out of sync with what a `Step`'s command actually references
  (confirmed: `test_all_seven_placeholders_present`,
  `test_print_plan_renders_r2_plan_without_keyerror`).
- `main()`'s shutdown sequence stops `capture_proc` **last**, after both
  responder and feeder (so it stays alive for the entire window either
  could have produced data), then -- only if `stage == "r2"` and capture
  actually started -- runs the comparison step as a foreground
  `run_step()` call before `record_dynamic_profile_evidence()`/PPP-stop.
  A non-zero comparison exit code sets the orchestrator's own exit code
  to 4 (without overriding an earlier abort's exit code 3) and prints
  where to look (`comparison.log`, `estimator_summary.md`); the run is
  never treated as silently fine on a threshold failure.
- The capture step's own `--duration-s` is set equal to the feeder's
  (same `FEEDER_SHUTDOWN_MARGIN_S` margin), so it is never stopped early
  relative to the feeder/responder it is scoring (confirmed by test,
  `test_capture_duration_matches_feeder_duration`).

## Task 9: host-only tests

**`test_sr75_hil_f24r2_estimator_comparison.py`** -- 25 tests:
`TestHorizontalDistance`, `TestYawErrorWrap`, `TestNearestWithinTolerance`,
`TestAlignAndCompare` (3), `TestAttitudeJumpDetection` (3, including
"yaw wrap is not mistaken for a jump"), `TestHealthEventScanning` (5),
`TestComputeSummaryEndToEnd` (3: well-behaved-passes, large-error-fails-
only-the-relevant-check, non-finite-raises), `TestMainEndToEnd` (1, CLI
subprocess run producing all 3 output files with the correct exit code).

**`test_sr75_hil_f24r2_estimator_capture.py`** -- 17 tests:
`TestMavlinkMessageToRow` (11, covering all 9 message types plus the
`hdg==65535` sentinel and an unrecognized-type-returns-None case),
`TestTruthTailer` (4, including one against the **real** R1 JSBSim
feeder, `@unittest.skipUnless(JSBSIM_AVAILABLE, ...)`), and
`TestCaptureMainEndToEnd` (2): one against a synthetic truth CSV + a
PTY-based fake Pixhawk (a real filesystem device path from
`pty.openpty()`/`os.ttyname()` that the real `main()` opens exactly like
`/dev/ttyACM0`, driven by a background thread sending real packed
MAVLink bytes), and one -- `test_full_pipeline_real_r1_feeder_plus_
fake_pixhawk_plus_comparison`, the strongest host-only validation in this
task -- chaining the **real** R1 JSBSim feeder subprocess, the PTY fake
Pixhawk, the real capture script as a subprocess, and the real
comparison script as a subprocess, asserting the full pipeline runs to
completion with a positive `GLOBAL_POSITION_INT` sample count and finite
RMS values (not asserting `overall_pass`, since the Pixhawk side is
synthetic, not a real EKF).

**`test_sr75_hil_f24f_hardware_orchestrator.py`** -- 19 new tests across
`TestShouldProceedToHardwareR2Stage` (5), `TestBuildExecutionPlanR2Stage`
(6), `TestBuildSessionDirR2Stage` (2), `TestSessionArtifactPaths` (2),
`TestMainR2StageValidation` (2, CLI subprocess dry-run checks), plus 2
more folded into the existing session-dir/plan test classes. All 28
pre-existing tests continue to pass unchanged.

No real hardware is run automatically anywhere in this task -- the two
`skipUnless(JSBSIM_AVAILABLE, ...)` tests use the real *JSBSim* binary
(already installed in this environment from HIL-F24-R1) but never open a
real serial device; every Pixhawk-side test uses the PTY fake.

## Problem found and fixed during this task

The capture script's main loop initially wrote each CSV row without an
explicit `.flush()` -- on an abrupt SIGTERM/kill, buffered rows could be
lost, undermining the "preserve existing... evidence" and general
durability intent of task 2. Fixed by flushing both `truth_f` and
`pixhawk_f` after every `writerow()` call (visible in the `main()` loop
above); confirmed the fix doesn't affect throughput in the 30-second-
equivalent stress runs below.

Two of the newly-written comparison-script tests
(`test_well_behaved_run_passes_all_checks`,
`test_cli_run_produces_all_output_files_and_correct_exit_code`) initially
failed with `alignment_coverage=0.673` against the `>=0.95` threshold --
not an implementation bug, but a test-fixture mismatch: the synthetic
truth CSV span (20s) was shorter than the synthetic Pixhawk CSV span
(30s) in those two tests, so the tail of the Pixhawk stream legitimately
had no truth to align to. Fixed by extending the truth fixture to 32s
(matching the Pixhawk fixture's span) in both tests.

## Exact files changed

### New: `Tools/autotest/sr75_hil_layer2/scripts/sr75_hil_f24r2_estimator_capture.py`
Full new file (~290 lines) -- see Task 1/2 above.

### New: `Tools/autotest/sr75_hil_layer2/scripts/sr75_hil_f24r2_estimator_comparison.py`
Full new file (~520 lines) -- see Tasks 3/4/5/6/7 above.

### New: `Tools/autotest/sr75_hil_layer2/scripts/test_sr75_hil_f24r2_estimator_capture.py`
Full new file, 17 tests -- see Task 9.

### New: `Tools/autotest/sr75_hil_layer2/scripts/test_sr75_hil_f24r2_estimator_comparison.py`
Full new file, 23 tests -- see Task 9.

### `Tools/autotest/sr75_hil_layer2/scripts/sr75_hil_f24f_hardware_orchestrator.py`

```diff
+R2_CAPTURE_SCRIPT = SCRIPTS_DIR / "sr75_hil_f24r2_estimator_capture.py"
+R2_COMPARISON_SCRIPT = SCRIPTS_DIR / "sr75_hil_f24r2_estimator_comparison.py"

 def should_proceed_to_hardware(
     execute: bool, confirm_static_only: bool,
     confirm_dynamic_jsbsim: bool = False, profile: str = "static",
+    confirm_estimator_comparison: bool = False, stage: str = "none",
 ) -> tuple:
-    if profile == "dynamic":
+    if stage == "r2":
+        confirm, confirm_flag_name = confirm_estimator_comparison, "--confirm-estimator-comparison"
+    elif profile == "dynamic":
         confirm, confirm_flag_name = confirm_dynamic_jsbsim, "--confirm-dynamic-jsbsim"
     else:
         confirm, confirm_flag_name = confirm_static_only, "--confirm-static-only"
     ...

 def build_session_dir(
-    base_dir: Path, now: Optional[datetime] = None, profile: str = "static",
+    base_dir: Path, now: Optional[datetime] = None, profile: str = "static", stage: str = "none",
 ) -> Path:
     stamp = ...
-    name = "sr75_hil_f24r1_dynamic" if profile == "dynamic" else "sr75_hil_f24f_static"
+    if stage == "r2":
+        name = "sr75_hil_f24r2_estimator"
+    elif profile == "dynamic":
+        name = "sr75_hil_f24r1_dynamic"
+    else:
+        name = "sr75_hil_f24f_static"
     return base_dir / f"{name}_{stamp}"

 def build_execution_plan(args) -> List[Step]:
     ...
     plan = [check_adapters, precheck, ppp_start, ppp_verify, start_feeder, start_responder]
+    if getattr(args, "stage", "none") == "r2":
+        capture_duration_s = feeder_duration_s  # never stopped early relative to the feeder
+        plan.append(Step("start_estimator_capture", ..., capture_cmd))
+        plan.append(Step("run_estimator_comparison", ..., comparison_cmd))
     plan.append(Step("ppp_stop", ..., ppp_stop_cmd))
     return plan

+def session_artifact_paths(session_dir) -> dict:
+    # all 7 {placeholder} names -> paths under session_dir
+    ...

 def print_plan(plan, args):
-    for step in plan: ... .format(state_csv=..., responder_csv=...)
+    placeholders = session_artifact_paths("<session_dir>")
+    for step in plan: ... .format(**placeholders)

 def run_step(step, session_dir, log_path, timeout_s=None):
-    rendered = [part.format(state_csv=..., responder_csv=...) for part in step.command]
+    rendered = [part.format(**session_artifact_paths(session_dir)) for part in step.command]

 def build_arg_parser():
     ...
+    parser.add_argument("--stage", choices=("none", "r2"), default="none", help=...)
+    parser.add_argument("--confirm-estimator-comparison", action="store_true", help=...)
+    parser.add_argument("--r2-alignment-tolerance-s", type=float, default=0.1, help=...)
+    parser.add_argument("--r2-settle-s", type=float, default=5.0, help=...)

 def main():
     args = build_arg_parser().parse_args()
-    print(f"=== HIL-F24-F hardware orchestrator (--profile {args.profile}) ===")
+    print(f"=== HIL-F24-F hardware orchestrator (--profile {args.profile} --stage {args.stage}) ===")
+    if args.stage == "r2":
+        print("HIL-F24-R2 stage: read-only ... comparison ...")
+        if args.profile != "dynamic":
+            print("ABORT: --stage r2 requires --profile dynamic -- staying in dry-run")
+            print_plan(build_execution_plan(args), args)
+            return 0
     plan = build_execution_plan(args)
     proceed, reason = should_proceed_to_hardware(
         args.execute, args.confirm_static_only,
         confirm_dynamic_jsbsim=args.confirm_dynamic_jsbsim, profile=args.profile,
+        confirm_estimator_comparison=args.confirm_estimator_comparison, stage=args.stage,
     )
     ...
-    session_dir = build_session_dir(Path(args.sessions_dir), profile=args.profile)
+    session_dir = build_session_dir(Path(args.sessions_dir), profile=args.profile, stage=args.stage)
     ...
     capture_proc = None
     ...
         responder_proc = subprocess.Popen(...)
+        if args.stage == "r2":
+            capture_proc = subprocess.Popen(capture_cmd, ...)
         time.sleep(args.duration_s + ORCHESTRATOR_TEST_MARGIN_S)
     finally:
-        for proc, label in ((responder_proc, "responder"), (feeder_proc, "feeder")):
+        for proc, label in ((responder_proc, "responder"), (feeder_proc, "feeder"), (capture_proc, "estimator capture")):
             ...  # capture stopped LAST
+        if args.stage == "r2" and capture_proc is not None:
+            comparison_rc = run_step(plan_by_name["run_estimator_comparison"], ...)
+            exit_code = exit_code or (4 if comparison_rc != 0 else 0)
         if args.profile == "dynamic":
             record_dynamic_profile_evidence(args, session_dir)
         if ppp_started: ...
```

Module docstring updated to describe `--stage r2`; static and `--profile
dynamic`-without-`--stage r2` plans and behavior are otherwise byte-for-
byte unchanged (confirmed by test and by dry-run diffing, below).

### `Tools/autotest/sr75_hil_layer2/scripts/test_sr75_hil_f24f_hardware_orchestrator.py`

`make_args()` gained `stage="none", confirm_estimator_comparison=False,
r2_alignment_tolerance_s=0.1, r2_settle_s=5.0` defaults. Extended with
`TestShouldProceedToHardwareR2Stage` (5), `TestBuildExecutionPlanR2Stage`
(6), `TestBuildSessionDirR2Stage` (2), `TestSessionArtifactPaths` (2),
`TestMainR2StageValidation` (2). All 28 pre-existing tests unchanged.

## Test results

```
$ python3 -m pytest scripts/test_sr75_hil_f24r2_estimator_comparison.py -v
25 passed

$ python3 -m pytest scripts/test_sr75_hil_f24r2_estimator_capture.py -v
17 passed
(includes: TestTruthTailer::test_against_real_r1_feeder [real JSBSim],
TestCaptureMainEndToEnd::test_full_pipeline_real_r1_feeder_plus_fake_pixhawk_plus_comparison
[real JSBSim + PTY fake Pixhawk + real capture subprocess + real comparison subprocess])

$ python3 -m pytest scripts/test_sr75_hil_f24f_hardware_orchestrator.py -v
47 passed (2 subtests)
(28 pre-existing R1-and-earlier tests unchanged + 19 new R2 tests, all passing)
```

**Full regression check**, everything under `sim_json/` and `scripts/`:

```
$ python3 -m pytest sim_json scripts -v
530 passed, 24 subtests passed in 63.62s
```

Zero regressions across every prior HIL-F24-B/D/F/G/P/Q/S/T/R1 test file.

### `py_compile` / `flake8` / `git diff --check`

```
$ python3 -m py_compile <6 new/modified R2 files>
(clean)
$ python3 -m flake8 --max-line-length=200 <same 6 files>
(clean -- 7 findings caught and fixed during this task: 2 over-indented
continuation lines in the orchestrator's new argparse calls, 1 unused
`import os` in the capture script, 4 over-indented continuation lines
in the comparison script's markdown-writer f-strings and def signature
-- all in new code, none pre-existing)
$ git diff --check -- <same 6 files>
(clean, no whitespace errors)
```

### Dry-run plan confirmation (all three modes)

```
$ python3 sr75_hil_f24f_hardware_orchestrator.py                              # static, unchanged
... 7-step plan, "sr75_hil_f24f_static_..." session name ...

$ python3 sr75_hil_f24f_hardware_orchestrator.py --profile dynamic --stage r2 # R2
... 9-step plan (adds start_estimator_capture, run_estimator_comparison),
    all 5 new {placeholder}s resolved, "sr75_hil_f24r2_estimator_..." ...

$ python3 sr75_hil_f24f_hardware_orchestrator.py --stage r2                   # invalid: r2 without dynamic
ABORT: --stage r2 requires --profile dynamic -- staying in dry-run
```

## Guarded 30-second hardware command

Exactly as the operator would run it (dry-run first, then the guarded
real run) -- **not executed by this task**:

```sh
# 1. Dry run (default) -- prints the exact plan, touches nothing:
python3 Tools/autotest/sr75_hil_layer2/scripts/sr75_hil_f24f_hardware_orchestrator.py \
    --profile dynamic --stage r2 --duration-s 30

# 2. The guarded real run, once the printed plan has been reviewed:
python3 Tools/autotest/sr75_hil_layer2/scripts/sr75_hil_f24f_hardware_orchestrator.py \
    --profile dynamic --stage r2 --duration-s 30 \
    --execute --confirm-estimator-comparison
```

All existing defaults apply unchanged (`--pixhawk /dev/ttyACM0`,
`--ppp-device /dev/ttyUSB0`, `--host-ip 192.168.144.2`, `--pixhawk-ip
192.168.144.14`, `--listen-port 9002`, `--r2-alignment-tolerance-s 0.1`,
`--r2-settle-s 5.0`). The orchestrator will, in order: check adapters,
run the read-only precheck (abort on STOP -- confirm MANUAL/disarmed/
CH7=0/CH8=0 here first), start PPP, verify the link, start the live-
JSBSim feeder, start the responder (LOG_ONLY), start the read-only
estimator capture, run for 30s, then always stop responder → feeder →
capture → PPP, score the comparison, and finally record
`jsbsim_state.csv`/`postcheck.log` alongside the usual artifacts.

### Extraction commands

From a completed `--stage r2` session directory
(`sr75_hil_layer2/hardware_sessions/sr75_hil_f24r2_estimator_<timestamp>/`):

```sh
SESSION=Tools/autotest/sr75_hil_layer2/hardware_sessions/sr75_hil_f24r2_estimator_<timestamp>

# Overall verdict and the first failing check (if any):
cat "$SESSION/estimator_summary.md"

# Machine-readable summary (all checks, all metric stats, thresholds used):
python3 -m json.tool "$SESSION/estimator_summary.json"

# Every aligned sample's per-metric error, for plotting/manual inspection:
head -5 "$SESSION/estimator_comparison.csv"

# Raw synchronized truth/estimator time series (for a from-scratch re-analysis):
wc -l "$SESSION/jsbsim_truth.csv" "$SESSION/pixhawk_estimator.csv"

# Re-run the comparison standalone with a different tolerance/settle period,
# without re-capturing (useful for sensitivity-checking the provisional thresholds):
python3 Tools/autotest/sr75_hil_layer2/scripts/sr75_hil_f24r2_estimator_comparison.py \
    --truth-csv "$SESSION/jsbsim_truth.csv" --pixhawk-csv "$SESSION/pixhawk_estimator.csv" \
    --comparison-csv /tmp/recheck_comparison.csv \
    --summary-json /tmp/recheck_summary.json --summary-md /tmp/recheck_summary.md \
    --alignment-tolerance-s 0.1 --settle-s 5.0

# Pre/post safety evidence (mode/armed/CH7/CH8), same as R1:
cat "$SESSION/precheck.log" "$SESSION/postcheck.log"
```

## PASS/FAIL thresholds (`DEFAULT_THRESHOLDS`, post-settle steady-state)

| Check | Threshold | Provisional? |
|---|---|---|
| Alignment coverage | >= 95% | Provisional -- depends on real Pixhawk telemetry rates and real serial/PPP latency, neither exercised on real hardware yet |
| No NaN/Inf | must hold (enforced structurally) | Not provisional -- a parse-time invariant, independent of the bench |
| No Pixhawk reboot / estimator reset | 0 health/reset events | Provisional detection logic (EKF flags + STATUSTEXT substrings + HEARTBEAT scan) -- never exercised against a real EKF, so an unknown real-world reset signature could go undetected until the first bench capture |
| Horizontal position error RMS | <= 5 m | **Provisional** -- entirely unvalidated against a real EKF; only synthetic/fake-Pixhawk data has exercised this check so far |
| Altitude error RMS | <= 3 m (datum offset reported separately as `mean`, not folded into RMS) | **Provisional**, same caveat; additionally the JSBSim-vs-Pixhawk altitude datum match (task 5) is unverified on real hardware |
| Roll/pitch RMS | <= 2 deg each | **Provisional**, same caveat |
| Yaw RMS | <= 5 deg | **Provisional**, same caveat |
| VN/VE/VD RMS | <= 1.5 m/s each | **Provisional**, same caveat |
| Airspeed RMS | <= 2 m/s | **Provisional** -- additionally depends on whether the Pixhawk's VFR_HUD airspeed source (TAS/EAS/CAS per AHRS config) matches JSBSim's true airspeed closely enough at the bench's actual altitude/density; unverified |
| No single unexplained attitude jump | > 10 deg triggers a fail | Provisional threshold value; the detection logic itself (yaw-wrap-aware) is validated by test, but the 10° cutoff itself is untuned against real EKF noise |
| Pre/post safety | MANUAL, disarmed, CH7=0, CH8=0 | Not provisional -- reuses the already-hardware-validated `sr75_hil_f24c_preflash_precheck.py` unchanged |

**Per this task's explicit instruction: if a threshold fails on the
first real bench capture, the script reports the first causal mismatch
(`first_failure` in the summary) and nothing in this task tunes the
estimator or changes any parameter in response.** All 10 numeric
thresholds above remain provisional until validated against that first
real capture; they were chosen directly from the task specification, not
derived from any bench data (none existed before this task).

## Acceptance report format

For the operator to fill in after the guarded run above, from the
session directory's own recorded artifacts:

```
HIL-F24-R2 estimator-comparison hardware acceptance report
Session directory: <sr75_hil_layer2/hardware_sessions/sr75_hil_f24r2_estimator_YYYYMMDDTHHMMSSZ>

Pre-run safety evidence (precheck.log):
  Pixhawk mode: ____________   armed: ____________
  CH7 PWM: ____________        CH8 PWM: ____________

PPP (ppp_start.log / ppp_verify.log):
  ppp0 up: yes/no    ping result: ___ / ___ received, ___% loss

Capture (capture.log -- ESTIMATOR_CAPTURE_SUMMARY):
  truth_rows=____  pixhawk_rows=____
  pixhawk_counts_by_type=____ (expect all of HEARTBEAT/GLOBAL_POSITION_INT/
    LOCAL_POSITION_NED/ATTITUDE/GPS_RAW_INT/VFR_HUD/EKF_STATUS_REPORT/
    SYS_STATUS present; STATUSTEXT may be zero if nothing was logged)

Comparison (comparison.log / estimator_summary.md):
  Overall: PASS / FAIL
  First failing check (if FAIL): ____________________
  Alignment coverage: ____%  (threshold >= 95%)
  Alignment tolerance used: ____ ms   Settle period excluded: ____ s

  horizontal_error_m   rms=____ (<=5)      max=____   mean=____   p95=____
  altitude_error_m     rms=____ (<=3)      max=____   mean=____ (datum offset)  p95=____
  roll_error_deg       rms=____ (<=2)      max=____   mean=____   p95=____
  pitch_error_deg      rms=____ (<=2)      max=____   mean=____   p95=____
  yaw_error_deg        rms=____ (<=5)      max=____   mean=____   p95=____
  vn_error_mps         rms=____ (<=1.5)    max=____   mean=____   p95=____
  ve_error_mps         rms=____ (<=1.5)    max=____   mean=____   p95=____
  vd_error_mps         rms=____ (<=1.5)    max=____   mean=____   p95=____
  airspeed_error_mps   rms=____ (<=2)      max=____   mean=____   p95=____

  Attitude jumps (>10 deg): ____ count (must be 0)
  Health/reset events: ____ count (must be 0)
  Sample counts: GLOBAL_POSITION_INT=____ ATTITUDE=____ VFR_HUD=____ (each >=20)
  Update rates (median interval, s): GPI=____ ATT=____ VFR=____ (each <=1.0)

Post-run safety evidence (postcheck.log):
  Pixhawk mode: ____________   armed: ____________
  CH7 PWM: ____________        CH8 PWM: ____________

PASS/FAIL (all must be true for PASS):
  [ ] estimator_summary.md reports Overall: PASS
  [ ] alignment_coverage >= 95%
  [ ] no NaN/Inf anywhere (structurally guaranteed if the script ran to completion)
  [ ] no Pixhawk reboot/reconnect observed on the bench
  [ ] no EKF/statustext reset event, no armed/non-MANUAL event during the run
  [ ] horizontal/altitude/roll/pitch/yaw/VN/VE/VD/airspeed RMS all within threshold
  [ ] no single attitude jump > 10 deg
  [ ] Pre- and post-run mode == MANUAL, armed == 0, CH7 PWM == 0, CH8 PWM == 0
  [ ] No actuator/servo/engine output observed on the bench

Overall: PASS / FAIL
If FAIL: first causal mismatch reported (do not tune the estimator or
change parameters as part of this task -- report and stop).
Notes: ______________________________________________
```

## Files and line references

- New: `Tools/autotest/sr75_hil_layer2/scripts/sr75_hil_f24r2_estimator_capture.py`
  -- `request_message_intervals()`, `mavlink_message_to_row()`,
  `TruthTailer`, `main()`.
- New: `Tools/autotest/sr75_hil_layer2/scripts/sr75_hil_f24r2_estimator_comparison.py`
  -- `horizontal_distance_m()`, `yaw_error_deg()`, `load_truth_csv()`,
  `load_pixhawk_csv()`, `nearest_within_tolerance()`,
  `align_and_compare()`, `detect_attitude_jumps()`,
  `scan_ekf_and_statustext_health()`, `compute_summary()`,
  `write_comparison_csv()`/`write_summary_json()`/`write_summary_markdown()`,
  `main()`.
- New: `Tools/autotest/sr75_hil_layer2/scripts/test_sr75_hil_f24r2_estimator_capture.py`.
- New: `Tools/autotest/sr75_hil_layer2/scripts/test_sr75_hil_f24r2_estimator_comparison.py`.
- `Tools/autotest/sr75_hil_layer2/scripts/sr75_hil_f24f_hardware_orchestrator.py`
  -- `R2_CAPTURE_SCRIPT`/`R2_COMPARISON_SCRIPT` constants,
  `should_proceed_to_hardware()`, `build_session_dir()`,
  `build_execution_plan()`'s r2-stage branch, `session_artifact_paths()`
  (new), `print_plan()`, `run_step()`, `build_arg_parser()`, `main()`.
- `Tools/autotest/sr75_hil_layer2/scripts/test_sr75_hil_f24f_hardware_orchestrator.py`
  -- `make_args()`, `TestShouldProceedToHardwareR2Stage`,
  `TestBuildExecutionPlanR2Stage`, `TestBuildSessionDirR2Stage`,
  `TestSessionArtifactPaths`, `TestMainR2StageValidation`.
- Reused, unmodified: `sr75_sim_json_responder.LatestCSVReader` (all of
  HIL-F24-P/Q/S/T's atomic/coherence/monotonic-freshness hardening),
  `sr75_hil_f24d_static_state_feed._percentile()`,
  `sr75_hil_f24c_preflash_precheck.py` (pre/post safety evidence,
  unchanged), `sr75_hil_f24r1_dynamic_jsbsim_feed.py` and its runscript
  (the dynamic profile this stage runs on top of, untouched).
