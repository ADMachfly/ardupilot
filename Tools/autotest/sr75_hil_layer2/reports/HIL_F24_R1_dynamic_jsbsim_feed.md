# HIL-F24-R1: Live SR-75 JSBSim State Feed on the Real Pixhawk PPP Path

Status: implemented and validated host-side, including full end-to-end
runs against the *real* JSBSim binary (installed in this environment).
**No real Pixhawk/PPP hardware was touched by this task** -- that
remains the operator's own guarded, manual step (see "Guarded 30-second
hardware command" below). No firmware was modified or flashed, no
PARAM_SET, no arm, no AUTO/mission, no actuator/servo/engine/RATO
output, no MAVLink connection was opened by anything in this task.

## What changed, in one sentence

The orchestrator gained a second, explicitly-guarded `--profile dynamic`
mode that runs the real SR-75 6-DOF JSBSim model (ground-level,
motionless, engines off) instead of the static feeder's single fixed
row, republishing its live state through the exact same
HIL-F24-P/Q/S/T-hardened atomic `state.csv` interface the validated
static path and responder already use, unchanged.

## Task 1: guarded dynamic-JSBSim mode, static mode untouched

`sr75_hil_f24f_hardware_orchestrator.py` gained:

- `--profile {static,dynamic}` (default `static` -- every existing
  invocation without this flag behaves byte-for-byte as before).
- `--confirm-dynamic-jsbsim`, a **separate** confirmation flag from
  `--confirm-static-only`. `should_proceed_to_hardware()` now checks
  whichever flag matches the selected `--profile`; `--confirm-static-only`
  cannot authorize a dynamic run and `--confirm-dynamic-jsbsim` cannot
  authorize a static one (verified by test, see below). Calling the
  function exactly as before (2 args, no profile) reproduces the
  original static-only behavior exactly -- confirmed by a dedicated
  backward-compatibility test and by all 15 pre-existing orchestrator
  tests passing completely unchanged.
- `build_execution_plan()`'s feeder-command construction now branches on
  `--profile`: `static` still builds the exact same 3-argument static
  feeder command as before; `dynamic` builds a command for the new
  `sr75_hil_f24r1_dynamic_jsbsim_feed.py` instead. Every other step
  (adapter check, precheck, PPP start/verify, responder, PPP stop) is
  identical for both profiles -- untouched by this branch.
- `build_session_dir()` names dynamic-profile sessions
  `sr75_hil_f24r1_dynamic_<timestamp>` instead of
  `sr75_hil_f24f_static_<timestamp>`, so bench artifacts are never mixed
  up; the static name is unchanged (confirmed by test).
- The responder command (`--state-file`/`--log-csv`/`--strict`,
  deliberately no `--jsbsim-command-target`/`--actuator-map`) is **not**
  branched by profile at all -- LOG_ONLY actuator mode is unconditional
  either way (task 4), confirmed by test.

## Task 2: live JSBSim state, continuously published into state.csv

New JSBSim runscript,
`Tools/autotest/aircraft/sr_75_6_dof/scripts/SR75_hil_f24r1_dynamic_state_feed.xml`:

- `<use aircraft="sr_75_6_dof" initialize="accel_ground_init"/>` -- the
  same ground-level, motionless IC the already-validated
  `SR75_accel_ground_test.xml` accelerometer-semantics test uses (lat
  32.5378085, lon 74.3661944, alt 240.2m, vt=0.1kts, wings level).
- An event, active for the entire run, that explicitly sets all three
  engines (`propulsion/engine[0..2]/set-running`) to `0`, pins
  `fcs/rato-throttle-cmd-norm`, `fcs/turbojet-throttle-cmd-norm`,
  `fcs/throttle-cmd-norm`, `fcs/aileron-cmd-norm`, `fcs/elevator-cmd-norm`,
  `fcs/rudder-cmd-norm` at `0.0`, and runs `simulation/do_simple_trim`
  (JSBSim's own internal numerical trim -- not driven by, or connected
  to, anything from the Pixhawk) so the aircraft settles to a genuinely
  stable ground contact instead of free-falling from its IC (confirmed:
  without `do_simple_trim`, the model visibly free-falls -- see "Problem
  found and fixed" below).
- Every `<output>` field is computed via a `<function name="...">` using
  the **exact** field names `sr75_hil_f24d_static_state_feed.py`'s
  `CSV_FIELDS` and `StateMapper.state_from_csv()` already recognize
  (`time_s`, `lat_deg`, `lon_deg`, `alt_m`, `roll_rad`, `pitch_rad`,
  `yaw_rad`, `vn_mps`, `ve_mps`, `vd_mps`, `p_rad_s`, `q_rad_s`, `r_rad_s`,
  `accel_body_x_mss`, `accel_body_y_mss`, `accel_body_z_mss`,
  `airspeed_mps`), all in SI units, at 50 Hz. This means the Python
  feeder needs **zero unit-conversion or renaming logic of its own** --
  every column already matches the target schema verbatim.

New feeder, `Tools/autotest/sr75_hil_layer2/sim_json/
sr75_hil_f24r1_dynamic_jsbsim_feed.py`:

1. Launches JSBSim read-only (`JSBSim --root=<Tools/autotest>
   --script=<runscript> --realtime --end=3600`), mirroring
   `bridge/sr75_jsbsim_pixhawk_hil_bridge.py`'s own
   `start_jsbsim_subprocess()` command shape.
2. `RawJsbsimTail` tails JSBSim's own live-growing, non-atomic CSV output
   (a read-only "last complete line" poll, tolerant of a torn read since
   JSBSim is still writing -- the same technique the bridge's
   `JSBSimCSVMonitor` already uses, generalized to read all columns by
   header name since the runscript already names them to match the
   target schema).
3. **Reuses** `sr75_hil_f24d_static_state_feed.py`'s
   `atomic_write_csv_snapshot()`, `CSV_FIELDS`, `SnapshotWriteStats`, and
   `print_write_summary()` directly (imported, not reimplemented) -- so
   every HIL-F24-P/Q/S/T fix (same-directory temp-file + `os.replace()`
   atomicity, write-latency instrumentation, `fsync` defaulted off) is
   inherited automatically, with zero new atomic-write code of its own.
4. Only republishes a row once it passes validation (task 5, below) and
   is strictly newer than the last one published.
5. Same SIGTERM-handling pattern as the static feeder (raises a custom
   exception, prints a summary, and additionally terminates the JSBSim
   subprocess in a `finally:` block) -- so the orchestrator's existing
   HIL-F24-Q stop-responder-then-feeder sequencing needs no changes to
   work with this feeder too.

## Task 3: field mapping (validated)

| Requirement | JSBSim runscript `<output>` field | Units |
|---|---|---|
| Latitude | `lat_deg` | deg |
| Longitude | `lon_deg` | deg |
| Altitude | `alt_m` | m |
| NED velocity | `vn_mps`, `ve_mps`, `vd_mps` | m/s |
| Roll, pitch, yaw | `roll_rad`, `pitch_rad`, `yaw_rad` | rad |
| Body angular rates | `p_rad_s`, `q_rad_s`, `r_rad_s` | rad/s |
| Body acceleration | `accel_body_x_mss`, `accel_body_y_mss`, `accel_body_z_mss` | m/s² |
| Airspeed | `airspeed_mps` | m/s |

Confirmed against a real JSBSim run (`--end=1.0`, then `--end=10.0`):
CSV header exactly `Time,time_s,lat_deg,lon_deg,alt_m,roll_rad,pitch_rad,
yaw_rad,vn_mps,ve_mps,vd_mps,airspeed_mps,p_rad_s,q_rad_s,r_rad_s,
accel_body_x_mss,accel_body_y_mss,accel_body_z_mss` (the extra leading
`Time` column is JSBSim's own default, stripped by the feeder before
publishing). Every field then flows unmodified through
`StateMapper.state_from_csv()` -- reused from `sr75_hil_f24g_jsbsim_
pipeline_validation.py`'s own already-validated conversion path -- with
no field-name/unit mismatch.

## Task 4: actuator commands remain LOG_ONLY

Nothing in this task introduces any path from the Pixhawk into JSBSim:

- The responder command is unconditionally built without
  `--jsbsim-command-target`/`--actuator-map` for **both** profiles
  (confirmed by test, `test_responder_command_unaffected_by_profile`).
- The new dynamic feeder script has no network listener, no MAVLink
  connection, and no code path that reads anything from the responder or
  the Pixhawk -- it only ever reads JSBSim's own output file and writes
  `state.csv`. There is no way for an incoming actuator command to reach
  JSBSim in this architecture, confirmed by inspection (no such
  code exists to remove).

## Task 5: finite-value, units/range, timestamp-monotonicity, update-rate checks

`validate_jsbsim_row(row, last_published_time_s)` in the new feeder,
called on every candidate row before it is ever republished:

- **Finite-value**: every `CSV_FIELDS` entry must be present, parse as a
  float, and satisfy `math.isfinite()` -- rejects missing fields,
  non-numeric text, `NaN`, and `Inf` (`missing_field:*`,
  `non_numeric:*`, `non_finite:*`).
- **Units/range**: per-field plausibility bounds (`lat_deg` ∈
  [-90,90], `lon_deg` ∈ [-180,180], `alt_m` ∈ [-500,50000],
  `vn/ve/vd_mps`/`airspeed_mps` ∈ generous m/s bounds, `p/q/r_rad_s` ∈
  ±35 rad/s, `accel_body_*_mss` ∈ ±100 m/s² -- generous enough to never
  reject a genuine SR-75 state, tight enough to catch a divergent
  integration) (`out_of_range:*`).
- **Timestamp-monotonicity**: a row whose `time_s` is *less than* the
  last published one is rejected (`non_monotonic_time_s`); a row whose
  `time_s` exactly *equals* the last one is recognized as "not yet
  advanced" (`not_advanced`) and distinguished from a genuine error --
  it just means JSBSim hasn't produced a new row since the last poll.
- **Update-rate**: the feeder tracks wall-clock (`time.monotonic()`)
  time since the last *accepted* row; if JSBSim's own output stops
  advancing for more than `--jsbsim-stall-warn-s` (default 2.0s), it
  prints `JSBSIM_STALL_WARNING` (and `JSBSIM_STALL_CLEARED` once it
  resumes) -- **diagnostic only**. The actual staleness *enforcement*
  on the consuming side is intentionally left to the responder's already
  -fixed monotonic freshness gate (HIL-F24-T's
  `LatestCSVReader._snapshot_age_s()`), per this task's explicit "Reuse"
  instruction -- this feeder does not reimplement that gate.

## Task 6: recorded evidence

For `--profile dynamic`, the orchestrator's `finally:` block now also
calls `record_dynamic_profile_evidence()`, which:

- Copies JSBSim's own raw state CSV (`/tmp/sr75_hil_f24r1_jsbsim_raw.csv`)
  into `<session_dir>/jsbsim_state.csv` (best-effort; a missing file --
  e.g. an early precheck/PPP abort before the feeder ever started -- is
  logged as a warning, not a crash).
- Re-runs the same read-only safety precheck used *before* the test
  (`sr75_hil_f24c_preflash_precheck.py`, unchanged -- Pixhawk
  mode/armed-state/CH7/CH8 PWM via `HEARTBEAT`/`SERVO_OUTPUT_RAW`) a
  second time *after* the test, into `<session_dir>/postcheck.log`,
  alongside the pre-test one already captured in `precheck.log` -- giving
  before-and-after safety evidence instead of a single snapshot.
- `responder.csv`/`responder.log`/`feeder.log`/`ppp_*.log` continue to be
  recorded exactly as they were for the static profile, unchanged.

## Problem found and fixed during this task

The first draft of the JSBSim runscript free-fell: without
`simulation/do_simple_trim`, the ground IC's near-zero airspeed and
JSBSim's own gear/ground-reaction dynamics let the aircraft visibly
descend and pitch over within about a second (`alt_m` dropped ~5m,
`accel_body_z_mss` swung to -2.97, in a 1-second real JSBSim run). Fixed
by adding the exact same `<set name="simulation/do_simple_trim"
value="2"/>` the already-validated `SR75_accel_ground_test.xml` uses for
this identical IC -- confirmed by a second real JSBSim run: gear contact
established on all three gear units, and the state settles to a genuinely
stationary equilibrium (velocities/rates ~1e-9) within about a second.
(`do_simple_trim` is JSBSim's own internal numerical trim procedure --
it only ever adjusts *JSBSim's own* simulated control-surface state to
find a level starting condition; it has no connection to, and is not
driven by, anything from the Pixhawk or hardware.)

A second issue was caught by this task's own regression tests, not by
manual inspection: `RawJsbsimTail.read_latest_row()` could return the
CSV *header line itself* as if it were a data row when the raw output
file contained only a header and no data yet (a real startup-race
condition, before JSBSim's first output tick lands). Fixed by comparing
the parsed values against the header row and returning `None` (nothing
to publish yet) when they match.

## Exact diff

### New: `Tools/autotest/aircraft/sr_75_6_dof/scripts/SR75_hil_f24r1_dynamic_state_feed.xml`

Full new JSBSim runscript file (summarized above; see "Task 2").

### New: `Tools/autotest/sr75_hil_layer2/sim_json/sr75_hil_f24r1_dynamic_jsbsim_feed.py`

Full new file (~280 lines): `validate_jsbsim_row()`, `RawJsbsimTail`,
`start_jsbsim()`/`stop_jsbsim()`, `_FeederShutdownRequested`/
`_raise_shutdown_on_sigterm` (mirroring the static feeder's own SIGTERM
pattern), `build_arg_parser()`, `main()`. Imports and reuses
`sr75_hil_f24d_static_state_feed`'s `CSV_FIELDS`,
`atomic_write_csv_snapshot`, `print_write_summary` directly.

### `Tools/autotest/sr75_hil_layer2/scripts/sr75_hil_f24f_hardware_orchestrator.py`

```diff
+DYNAMIC_FEEDER_SCRIPT = SIM_JSON_DIR / "sr75_hil_f24r1_dynamic_jsbsim_feed.py"
+DYNAMIC_FEEDER_RAW_OUTPUT = "/tmp/sr75_hil_f24r1_jsbsim_raw.csv"

-def should_proceed_to_hardware(execute: bool, confirm_static_only: bool) -> tuple:
-    if execute and confirm_static_only:
-        return True, ""
-    ...
+def should_proceed_to_hardware(
+    execute: bool, confirm_static_only: bool,
+    confirm_dynamic_jsbsim: bool = False, profile: str = "static",
+) -> tuple:
+    if profile == "dynamic":
+        confirm, confirm_flag_name = confirm_dynamic_jsbsim, "--confirm-dynamic-jsbsim"
+    else:
+        confirm, confirm_flag_name = confirm_static_only, "--confirm-static-only"
+    if execute and confirm:
+        return True, ""
+    ...  # (same shape as before, parameterized by confirm/confirm_flag_name)

-def build_session_dir(base_dir: Path, now: Optional[datetime] = None) -> Path:
-    stamp = ...
-    return base_dir / f"sr75_hil_f24f_static_{stamp}"
+def build_session_dir(base_dir: Path, now: Optional[datetime] = None, profile: str = "static") -> Path:
+    stamp = ...
+    name = "sr75_hil_f24r1_dynamic" if profile == "dynamic" else "sr75_hil_f24f_static"
+    return base_dir / f"{name}_{stamp}"

 def build_execution_plan(args) -> List[Step]:
     ...
     feeder_duration_s = args.duration_s + ORCHESTRATOR_TEST_MARGIN_S + FEEDER_SHUTDOWN_MARGIN_S
-    feeder_cmd = [sys.executable, str(FEEDER_SCRIPT), "--output", "{state_csv}",
-                  "--duration-s", str(feeder_duration_s), "--rate-hz", str(args.rate_hz)]
+    profile = getattr(args, "profile", "static")
+    if profile == "dynamic":
+        feeder_cmd = [sys.executable, str(DYNAMIC_FEEDER_SCRIPT), "--output", "{state_csv}",
+                      "--duration-s", str(feeder_duration_s)]
+        feeder_description = "Start live JSBSim state feeder (HIL-F24-R1; before responder)"
+    else:
+        feeder_cmd = [sys.executable, str(FEEDER_SCRIPT), "--output", "{state_csv}",
+                      "--duration-s", str(feeder_duration_s), "--rate-hz", str(args.rate_hz)]
+        feeder_description = "Start static-state feeder (before responder)"
     responder_cmd = [...]  # unchanged, unconditional for both profiles

+def record_dynamic_profile_evidence(args, session_dir: Path) -> None:
+    # copies jsbsim_state.csv, re-runs the precheck into postcheck.log
+    ...

 def build_arg_parser():
     ...
+    parser.add_argument("--profile", choices=("static", "dynamic"), default="static", help=...)
+    parser.add_argument("--confirm-dynamic-jsbsim", action="store_true", help=...)

 def main():
     args = build_arg_parser().parse_args()
-    print("=== HIL-F24-F hardware orchestrator (static SIM_JSON test only) ===")
+    print(f"=== HIL-F24-F hardware orchestrator (--profile {args.profile}) ===")
     ...
-    proceed, reason = should_proceed_to_hardware(args.execute, args.confirm_static_only)
+    proceed, reason = should_proceed_to_hardware(
+        args.execute, args.confirm_static_only,
+        confirm_dynamic_jsbsim=args.confirm_dynamic_jsbsim, profile=args.profile,
+    )
     ...
-    session_dir = build_session_dir(Path(args.sessions_dir))
+    session_dir = build_session_dir(Path(args.sessions_dir), profile=args.profile)
     ...
     finally:
         for proc, label in ((responder_proc, "responder"), (feeder_proc, "feeder")):
             ...
+        if args.profile == "dynamic":
+            record_dynamic_profile_evidence(args, session_dir)
         if ppp_started:
             ...
```

Docstring/banner text updated to describe both profiles (see the file
itself); no structural change to the plan's sequencing or cleanup logic.

### `Tools/autotest/sr75_hil_layer2/scripts/test_sr75_hil_f24f_hardware_orchestrator.py`

Extended with `TestShouldProceedToHardwareDynamicProfile` (5 tests),
`TestBuildExecutionPlanDynamicProfile` (5 tests), and
`TestBuildSessionDirProfile` (3 tests); `make_args()` gained
`profile="static", confirm_dynamic_jsbsim=False` defaults. All 15
pre-existing tests unchanged and still passing.

## Host-only regression test results (task 7)

```
$ python3 -m unittest sim_json.test_sr75_hil_f24r1_dynamic_jsbsim_feed -v
test_valid_row_accepted ... ok
test_missing_field_rejected ... ok
test_non_numeric_field_rejected ... ok
test_nan_rejected ... ok
test_inf_rejected ... ok
test_out_of_range_latitude_rejected ... ok
test_out_of_range_gyro_rejected ... ok
test_out_of_range_accel_rejected ... ok
test_non_monotonic_timestamp_rejected ... ok
test_unadvanced_timestamp_distinguished_from_error ... ok
test_strictly_advancing_timestamp_accepted ... ok
test_reads_last_complete_row ... ok
test_missing_file_returns_none ... ok
test_empty_file_returns_none ... ok
test_header_only_file_returns_none ... ok
test_torn_trailing_row_returns_none_not_a_crash ... ok
test_state_changes_continuously_and_stays_finite_zero_rejected ... ok   [real JSBSim]
test_sigterm_mid_run_still_stops_jsbsim_and_prints_summary ... ok       [real JSBSim]
test_short_run_zero_errors_exact_accounting ... ok                      [real JSBSim + real responder]
Ran 19 tests in 14.2s -- OK

$ python3 -m unittest scripts.test_sr75_hil_f24f_hardware_orchestrator -v
... 28 tests (15 pre-existing + 13 new) ... Ran 28 tests -- OK
```

The three real-JSBSim tests are decorated `@unittest.skipUnless(shutil.
which("JSBSim"), ...)` so the suite degrades gracefully (skips, doesn't
fail) in an environment without the JSBSim binary installed; JSBSim
1.3.2.dev1 is installed here, so all three ran for real.

**Full regression check**, all pre-existing suites unaffected:

```
sim_json:  test_sr75_hil_f24p_state_csv_race (6/6), test_sr75_hil_f24d_static_state_feed (5/5),
           test_sr75_hil_f24q_shutdown_and_timing (8/8), test_sr75_hil_f24s_coherence_and_accounting (7/7),
           test_sr75_hil_f24t_monotonic_freshness (8/8), test_sr75_hil_f24r1_dynamic_jsbsim_feed (19/19)
           -- 53/53
scripts:   test_sr75_hil_f24f_hardware_orchestrator (28/28), test_sr75_hil_f24b_sim_json_no_hardware_tests (23/23),
           test_sr75_hil_f24g_jsbsim_pipeline_validation (30/30) -- 81/81
jsbsim_control.test_sr75_sim_json_command_accounting: same 2 pre-existing, unrelated failures
  (actuator/JSBSim-command-accounting code this task does not touch; first identified as
  pre-existing in the HIL-F24-P report, reconfirmed unchanged in every task since).
```

### `py_compile` / `flake8` / `git diff --check`

```
$ python3 -m py_compile <4 new/modified files>
(clean)
$ python3 -m flake8 --max-line-length=200 <same 4 files>
(clean -- 2 findings caught and fixed during this task: an over-long
help string and an unused import, both in new files, neither pre-existing)
$ git diff --check -- <same 4 files, plus the new XML>
(clean, no whitespace errors)
```

## 30-second stress-test summary (task 6/"Required pass" evidence)

Real feeder (real JSBSim) + real responder subprocesses, simulated ~50 Hz
Pixhawk-equivalent request loop, for the full 30 seconds:

```
RUN_SUMMARY total_requests=1466 replies_sent=1466 stale_state_count=0
             no_fresh_state_count=0 malformed_state_count=0
             state_read_error_count=0 missed_replies=0
WRITE_SUMMARY write_count=1522 fsync_enabled=0 max_write_latency_ms=0.659
              p95_write_latency_ms=0.146 p99_write_latency_ms=0.205
              total_fsync_time_ms=0.000 longest_scheduling_gap_ms=20.260
JSBSIM_SUMMARY published=1522 rejected_total=0 rejected_reasons={}
              min_alt_m=0.170 max_alt_m=240.201 final_time_s=32.349
final state.csv: header + 1 row, every field finite
responder.csv data rows: 1466 (== total_requests)
replies_sent + missed_replies == total_requests: True
```

**Required-pass criteria, host-side status** (real PPP/Pixhawk hardware
was not touched by this task -- see the guarded command below for the
operator's own run):

| Criterion | Host-side result |
|---|---|
| missed/stale/no-fresh/malformed/state-read errors all zero | **PASS** (0/0/0/0/0 above) |
| JSBSim state changes continuously and remains finite | **PASS** (`final_time_s` advances continuously; every published field finite, verified by test) |
| PPP stable | not exercised here (no PPP in a host-only run) -- unaffected by this task's changes either way |
| no Pixhawk reset | not exercised here (no Pixhawk in a host-only run) |
| MANUAL, disarmed, CH7=0, CH8=0 | not exercised here -- enforced by the existing, unmodified `sr75_hil_f24c_preflash_precheck.py` STOP/GO gate, now run both before *and* after the test for the dynamic profile |
| no actuator output or actuator application | **PASS** by construction: responder LOG_ONLY unconditionally for both profiles (confirmed by test); feeder has no code path that could apply anything to JSBSim or hardware |

## Guarded 30-second hardware command

Exactly as the operator would run it (dry-run first, then the guarded
real run) -- **not executed by this task**:

```sh
# 1. Dry run (default) -- prints the exact plan, touches nothing:
python3 Tools/autotest/sr75_hil_layer2/scripts/sr75_hil_f24f_hardware_orchestrator.py \
    --profile dynamic --duration-s 30

# 2. The guarded real run, once the printed plan has been reviewed:
python3 Tools/autotest/sr75_hil_layer2/scripts/sr75_hil_f24f_hardware_orchestrator.py \
    --profile dynamic --duration-s 30 \
    --execute --confirm-dynamic-jsbsim
```

All existing defaults apply unchanged (`--pixhawk /dev/ttyACM0`,
`--ppp-device /dev/ttyUSB0`, `--host-ip 192.168.144.2`, `--pixhawk-ip
192.168.144.14`, `--listen-port 9002`). The orchestrator will, in order:
check adapters, run the read-only precheck (abort on STOP -- confirm
MANUAL/disarmed/CH7=0/CH8=0 here first), start PPP, verify the link,
start the live-JSBSim feeder, start the responder (LOG_ONLY), run for
30s, then always stop responder → feeder → PPP, and finally record
`jsbsim_state.csv` and `postcheck.log` alongside the usual artifacts.

## Acceptance report format

For the operator to fill in after the guarded run above, from the
session directory's own recorded artifacts:

```
HIL-F24-R1 dynamic-profile hardware acceptance report
Session directory: <sr75_hil_layer2/hardware_sessions/sr75_hil_f24r1_dynamic_YYYYMMDDTHHMMSSZ>

Pre-run safety evidence (precheck.log):
  Pixhawk mode: ____________   armed: ____________
  CH7 PWM: ____________        CH8 PWM: ____________

PPP (ppp_start.log / ppp_verify.log):
  ppp0 up: yes/no    ping result: ___ / ___ received, ___% loss

Run window (feeder.log / responder.log):
  Requested duration: 30s   Actual PPP-connected duration: ______s
  Any Pixhawk USB/PPP disconnect or reset observed: yes/no (describe if yes)

Responder RUN_SUMMARY (from responder.log):
  total_requests=____ replies_sent=____ stale_state_count=____
  no_fresh_state_count=____ malformed_state_count=____
  state_read_error_count=____ missed_replies=____
  responder.csv data rows: ____  (must equal total_requests)

Feeder WRITE_SUMMARY / JSBSIM_SUMMARY (from feeder.log):
  write_count=____ max_write_latency_ms=____ longest_scheduling_gap_ms=____
  published=____ rejected_total=____ rejected_reasons=____
  min_alt_m=____ max_alt_m=____ final_time_s=____

Post-run safety evidence (postcheck.log):
  Pixhawk mode: ____________   armed: ____________
  CH7 PWM: ____________        CH8 PWM: ____________

JSBSim state evidence: jsbsim_state.csv present: yes/no, row count: ____

PASS/FAIL (all must be true for PASS):
  [ ] missed_replies == 0
  [ ] stale_state_count == 0
  [ ] no_fresh_state_count == 0
  [ ] malformed_state_count == 0
  [ ] state_read_error_count == 0
  [ ] responder.csv rows == total_requests
  [ ] JSBSim final_time_s advanced continuously, no stall warnings in feeder.log
  [ ] PPP remained up for the entire run, no Pixhawk reset/reconnect observed
  [ ] Pre- and post-run mode == MANUAL, armed == 0, CH7 PWM == 0, CH8 PWM == 0
  [ ] No actuator/servo/engine output observed on the bench

Overall: PASS / FAIL
Notes: ______________________________________________
```

## Files and line references

- New: `Tools/autotest/aircraft/sr_75_6_dof/scripts/SR75_hil_f24r1_dynamic_state_feed.xml`.
- New: `Tools/autotest/sr75_hil_layer2/sim_json/sr75_hil_f24r1_dynamic_jsbsim_feed.py`
  -- `validate_jsbsim_row()`, `RawJsbsimTail`, `start_jsbsim()`/
  `stop_jsbsim()`, `main()`.
- New: `Tools/autotest/sr75_hil_layer2/sim_json/test_sr75_hil_f24r1_dynamic_jsbsim_feed.py`.
- `Tools/autotest/sr75_hil_layer2/scripts/sr75_hil_f24f_hardware_orchestrator.py`
  -- `DYNAMIC_FEEDER_SCRIPT`/`DYNAMIC_FEEDER_RAW_OUTPUT` constants,
  `should_proceed_to_hardware()`, `build_session_dir()`,
  `build_execution_plan()`'s feeder-command branch,
  `record_dynamic_profile_evidence()` (new), `build_arg_parser()`,
  `main()`.
- `Tools/autotest/sr75_hil_layer2/scripts/test_sr75_hil_f24f_hardware_orchestrator.py`
  -- `make_args()`, `TestShouldProceedToHardwareDynamicProfile`,
  `TestBuildExecutionPlanDynamicProfile`, `TestBuildSessionDirProfile`.
- Reused, unmodified: `sr75_hil_f24d_static_state_feed.py` (`CSV_FIELDS`,
  `atomic_write_csv_snapshot`, `print_write_summary`),
  `sr75_sim_json_responder.py` (`StateMapper.state_from_csv`,
  `LatestCSVReader`, all of HIL-F24-P/Q/S/T), `sr75_hil_f24c_preflash_
  precheck.py` (mode/armed/CH7/CH8 readback), `bridge/sr75_jsbsim_
  pixhawk_hil_bridge.py`'s tailing technique (adapted, not imported, since
  the bridge's own class is coupled to a different fixed-field table).
