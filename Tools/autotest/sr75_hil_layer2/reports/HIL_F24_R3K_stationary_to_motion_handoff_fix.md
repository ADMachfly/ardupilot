# HIL-F24-R3K: Stationary→Motion Feeder Handoff Gap (Fix)

Builds on the confirmed R3G bootstrap/origin fix. No firmware, EKF,
parameter, threshold, or control change is made here -- this is a
feeder/responder sequencing fix only, per the task's explicit
constraint.

Evidence session: `sr75_hil_f24r3a_mp_visual_20260807T074250Z`.

## 1. First causal handoff failure

`sr75_sim_json_responder.py`'s `StateMapper` is instantiated **once**
for the responder's entire lifetime and persists unchanged across the
orchestrator's feeder swap (stationary feeder stopped and awaited,
*then* the visual/dynamic feeder started -- confirmed in
`sr75_hil_f24f_hardware_orchestrator.py`, stop-before-start, no
state.csv truncation in between). The stationary feeder's last accepted
row had `time_s=37.700048`. The new dynamic feeder
(`sr75_hil_f24r1_dynamic_jsbsim_feed.py`) launches a **fresh JSBSim
process**, whose `time_s` column is `simulation/sim-time-sec` read
directly off the runscript's `<output>` block
(`Tools/autotest/aircraft/sr_75_6_dof/scripts/SR75_hil_f24r3a_mp_visual_trajectory.xml`)
-- a clock that always restarts at 0 in a new process. Every row from
the new feeder therefore has `time_s` far below the responder's
carried-over `last_source_timestamp` (37.700048) until its own clock
naturally climbs back past that value, ~37.6 seconds later.

Confirmed directly in `hardware_sessions/sr75_hil_f24r3a_mp_visual_20260807T074250Z/responder.csv`:
188 consecutive rows, `host_monotonic_time` 60121.210 → 60158.805, all
`reply_sent=0`, `reply_reason="backward source timestamp: 0.04 <
37.700048"` … `"37.640000000000285 < 37.700048"`. **No SIM_JSON reply
is sent at all for the whole outage** -- not a stale/degraded reply,
literally none -- which is what starves the Pixhawk's SITL/JSON backend
into reporting `fix_type=1`. The first recovered row
(`source_simulation_timestamp=37.860`) lands at the trajectory's
already-advanced position (JSBSim had been running in realtime the
whole rejection window), producing the observed ~150 m jump; EKF flags
degrade 831→39 during the outage and recover to 831 only after GPS
fix_type returns to 3.

Item 5's answer: **position** already matches at handoff (the
trajectory's smoothstep functions are 0 at `sim-time-sec=0`, which is
exactly the stationary lat/lon/alt by construction) -- only **time**
fails to continue.

## 2. File:line

- `Tools/autotest/sr75_hil_layer2/sim_json/sr75_sim_json_responder.py:681-682` --
  `StateMapper.state_from_csv()`: `if self.last_source_timestamp is not
  None and timestamp < self.last_source_timestamp: raise
  StateError(f"backward source timestamp: ...")`. This is a legitimate
  safety check (catches real backward-time corruption within one
  feeder's own stream) that has no notion of "a new feeder process
  legitimately restarted its clock" -- it is not itself modified by this
  fix (see "no scoring settle workaround" below).
- `Tools/autotest/sr75_hil_layer2/sim_json/sr75_hil_f24r1_dynamic_jsbsim_feed.py:286-289`
  (pre-fix): `out_row = {field: row[field] for field in feed.CSV_FIELDS}` --
  `time_s` was republished verbatim from JSBSim's own fresh
  `simulation/sim-time-sec`, with no continuity from whatever feeder
  ran before it.
- `Tools/autotest/aircraft/sr_75_6_dof/scripts/SR75_hil_f24r3a_mp_visual_trajectory.xml:37` --
  `<function name="time_s"><property>simulation/sim-time-sec</property></function>`,
  the source of the reset-to-0 clock.

## 3. Minimum code fix

Fixed at the true source of the discontinuity -- the feeder's own
published `time_s` -- rather than by relaxing the responder's
monotonicity guard (which would be exactly the kind of workaround the
task ruled out: it would also silently accept a *genuine* backward-time
corruption). `sr75_hil_f24r1_dynamic_jsbsim_feed.py`:

- New pure helper `read_existing_output_time_s(path)`: reads whatever
  single-row state.csv snapshot already exists at `--output` (the row
  being handed off from) and returns its `time_s`, or `None` if there
  is none (e.g. run standalone, matching original HIL-F24-R1 usage).
- New `--time-offset-s` argument (default: unset/auto). At startup,
  `time_offset_s = read_existing_output_time_s(args.output) or 0.0`
  unless explicitly overridden.
- Every published row's `time_s` becomes `time_s + time_offset_s`
  instead of the raw JSBSim value. Only this one field is shifted --
  position/attitude/rates are left computed from the runscript's own
  unshifted `simulation/sim-time-sec`, so the first published state and
  the motion ramp shape are unchanged; only the *identity* of "how far
  along this clock is" now continues instead of resetting.

```diff
+def read_existing_output_time_s(path):
+    try:
+        with open(path, "r", newline="", encoding="utf-8") as f:
+            rows = list(csv.DictReader(f))
+    except OSError:
+        return None
+    if not rows:
+        return None
+    try:
+        return float(rows[-1]["time_s"])
+    except (KeyError, ValueError):
+        return None
+
 def validate_jsbsim_row(row, last_published_time_s):
     ...

 def build_arg_parser():
     ...
+    parser.add_argument("--time-offset-s", type=float, default=None, help=...)
     return parser

 def main():
     ...
+    if args.time_offset_s is None:
+        time_offset_s = read_existing_output_time_s(args.output) or 0.0
+    else:
+        time_offset_s = args.time_offset_s
     ...
                         out_row = {field: row[field] for field in feed.CSV_FIELDS}
+                        out_row["time_s"] = f"{time_s + time_offset_s:.6f}"
                         write_stats.append(feed.atomic_write_csv_snapshot(args.output, feed.CSV_FIELDS, out_row))
```

Full diff in the working tree at
`Tools/autotest/sr75_hil_layer2/sim_json/sr75_hil_f24r1_dynamic_jsbsim_feed.py`.
No change to `sr75_sim_json_responder.py`, EKF, parameters, GPS/SIM_JSON
protocol, controls, or R3A orchestration logic.

## 4. Regression test proving no GPS/EKF validity drop

`Tools/autotest/sr75_hil_layer2/sim_json/test_sr75_hil_f24r1_dynamic_jsbsim_feed.py`,
new `TestHandoffContinuity` (real JSBSim + real responder, skipped if
JSBSim is not on `PATH`; it is available in this environment and both
tests pass):

- **`test_auto_offset_continues_seamlessly_from_stationary_row`**:
  seeds a state.csv with the exact R3A stationary row
  (`time_s=37.700048`), starts the real responder against it, lets it
  accept that row once (latching `last_source_timestamp`, exactly the
  pre-handoff bench state), then starts the real dynamic feeder over
  the same `--output` while a request loop keeps running -- asserts
  **zero** responder log rows ever contain `"backward source
  timestamp"` in `reply_reason`, and fewer than 10 missed replies total
  (covering only ordinary process-startup latency, not the ~37.6s
  regression).
- **`test_explicit_zero_offset_reproduces_the_gap_regression_guard`**
  (negative control): the same scenario but with `--time-offset-s 0.0`
  forced, deliberately reproducing the pre-fix behavior -- asserts
  `"backward source timestamp"` **does** appear, proving the test
  actually detects the bug rather than passing vacuously.
- New pure `TestReadExistingOutputTimeS` (5 tests): missing file,
  empty file, header-only file, missing `time_s` column all return
  `None`; a real stationary snapshot at `time_s=37.700048` (the exact
  R3A value) round-trips correctly.

Full file: 26/26 tests passing. `flake8 --max-line-length=200`,
`py_compile`, and `git diff --check` clean on both changed files.

## Files referenced

- `Tools/autotest/sr75_hil_layer2/sim_json/sr75_hil_f24r1_dynamic_jsbsim_feed.py` (fix)
- `Tools/autotest/sr75_hil_layer2/sim_json/test_sr75_hil_f24r1_dynamic_jsbsim_feed.py` (tests)
- `Tools/autotest/sr75_hil_layer2/sim_json/sr75_sim_json_responder.py` (`StateMapper.state_from_csv()`, read-only reference, unmodified)
- `Tools/autotest/sr75_hil_layer2/scripts/sr75_hil_f24f_hardware_orchestrator.py` (feeder-swap sequencing, read-only reference, unmodified)
- `Tools/autotest/aircraft/sr_75_6_dof/scripts/SR75_hil_f24r3a_mp_visual_trajectory.xml` (trajectory `time_s` source, unmodified)
- `hardware_sessions/sr75_hil_f24r3a_mp_visual_20260807T074250Z/responder.csv`, `pixhawk_estimator.csv`
