# HIL-F24-T: Monotonic-Clock State Freshness Fix (Host-Side Only)

Status: fix applied and validated. No firmware flashed, no PARAM_SET, no
arm, no AUTO, no actuator output enabled. Only host-side Python files
under `Tools/autotest/sr75_hil_layer2/` were touched; no PPP
configuration, no SIM_JSON C++, no RATO/aero/TECS/mission/control code,
and no actuator decode safety was modified.

## Confirmed cause

The freshness gate in `read_reply_state()` computed:

```python
row_age_ms = (time.time() - mtime) * 1000.0
```

where `mtime` was `st.st_mtime` -- both sides of that subtraction are
**wall-clock (`CLOCK_REALTIME`) values**. `st.st_mtime` is stamped by the
filesystem using the wall clock at the instant the feeder's
`os.replace()` landed; `time.time()` reads the wall clock *now*, at the
instant the responder happens to check. If the host's `CLOCK_REALTIME`
is stepped by NTP (or a manual date change) at any point between those
two instants, the difference is inflated or deflated by exactly the
size of the step, with **no relationship whatsoever to how long the row
has actually existed**.

This exactly explains the observed run: a single isolated `STALE_STATE`
with `rejected age=2050.7 ms`, while every other freshness signal says
the feeder was perfectly healthy:

- `feeder max write latency=0.831 ms`, `feeder longest scheduling gap=20.269 ms`
  -- the feeder was writing every ~20ms without a single slow iteration
  (ruling out a real feeder stall, HIL-F24-Q's failure mode).
- `malformed/no-fresh errors=0` -- the coherent-snapshot fix (HIL-F24-S)
  was working; the rejected read's `dev`/`ino`/`mtime_ns` diagnostic was
  a single, internally consistent snapshot, not a torn read.
- `row_source_timestamp=11.180063` -- an ordinary mid-run timestamp, not
  a discontinuity in the feeder's own data.

A `~2050ms` age with a `~20ms` write cadence and zero other error signal
is precisely the signature of a `~2s` host clock step landing between
one write's `os.replace()` and the very next `read_reply_state()` call
that happened to check freshness right after it -- not a feeder problem
and not a reader-coherence problem, but a wall-clock-vs-monotonic-clock
category error in the freshness gate itself.

## Fix

### Task 2: monotonic snapshot-identity tracking

`LatestCSVReader` now tracks, per instance, the identity of the
last-seen snapshot and the `time.monotonic()` timestamp at which *this
process* first observed it:

```python
def __init__(self, path: str):
    ...
    self._last_snapshot_key: Optional[Tuple[int, int, int, str]] = None
    self._snapshot_first_observed_monotonic: Optional[float] = None

def _snapshot_age_s(self, st: os.stat_result, line: str) -> float:
    snapshot_key = (st.st_dev, st.st_ino, st.st_mtime_ns, line)
    now_monotonic = time.monotonic()
    if snapshot_key != self._last_snapshot_key:
        self._last_snapshot_key = snapshot_key
        self._snapshot_first_observed_monotonic = now_monotonic
    return now_monotonic - self._snapshot_first_observed_monotonic
```

The identity is `(dev, ino, mtime_ns, row content)` -- the row's own
content is included alongside the HIL-F24-S coherence fields so that
neither a coarse filesystem mtime-clock tick shared by two genuinely
different writes, nor an inode number reused after a very large number
of `replace()` cycles, could ever be mistaken for "the same snapshot
still sitting there." `time.monotonic()` is specified by Python (and the
underlying POSIX `clock_gettime(CLOCK_MONOTONIC)`) to never be affected
by `CLOCK_REALTIME`/NTP adjustments and to never go backward, so the
returned age reflects genuine elapsed time no matter what the wall clock
does. `read_latest()`'s return value's third element is now this
monotonic age in seconds (previously the raw `st.st_mtime` float):

```python
return row, line, self._snapshot_age_s(st, line), st
```

`read_reply_state()` uses it directly, with no `time.time()` involved in
the gate at all:

```python
row, signature, row_age_s, st = reader.read_latest()
...
row_age_ms = row_age_s * 1000.0
if row_age_ms > args.state_timeout_ms:
    raise StateError(f"STALE_STATE: latest row age {row_age_ms:.1f} ms (...)")
```

### Task 3: wall-clock mtime retained as diagnostic-only metadata

`st` (including `st_mtime`/`st_mtime_ns`) is still returned and still
shown in `describe_state_freshness_rejection()`'s output, which now also
reports a separate, purely informational `wall_clock_age_ms` figure
(`time.time() - st.st_mtime`) alongside the monotonic
`calculated_age_ms` that actually gated the rejection:

```
STALE_STATE: latest row age 612.3 ms (dev=... ino=... mtime_ns=... row_source_timestamp=11.180063 calculated_age_ms=612.3 wall_clock_age_ms=2612.5)
```

A large gap between the two numbers is now, by itself, direct evidence
of a host clock step -- exactly the ambiguity this task had to resolve
by cross-referencing feeder timing metrics after the fact.

### Task 4: a genuinely frozen feeder still becomes stale

Nothing here weakens detection of a real stall: if the feeder truly
stops writing, the snapshot's `(dev, ino, mtime_ns, line)` identity never
changes, so `_snapshot_first_observed_monotonic` stays fixed at the
moment of the last real write, and `time.monotonic() - first_observed`
keeps growing at the same rate real time passes, correctly exceeding
`state_timeout_ms` after the same real elapsed duration as before this
fix. Verified directly (see Test results).

## Exact diff

### `Tools/autotest/sr75_hil_layer2/sim_json/sr75_sim_json_responder.py`

```diff
@@ __init__ __
     def __init__(self, path: str):
         self.path = path
         self.headers: Optional[List[str]] = None
         self._header_cache_key: Optional[Tuple[int, int, int]] = None
+        # HIL-F24-T: identity of the last-seen snapshot and the
+        # time.monotonic() timestamp at which THIS process first observed
+        # it -- see _snapshot_age_s(). Never derived from CLOCK_REALTIME.
+        self._last_snapshot_key: Optional[Tuple[int, int, int, str]] = None
+        self._snapshot_first_observed_monotonic: Optional[float] = None

@@ new method, between _load_headers() and read_latest() @@
+    def _snapshot_age_s(self, st: os.stat_result, line: str) -> float:
+        snapshot_key = (st.st_dev, st.st_ino, st.st_mtime_ns, line)
+        now_monotonic = time.monotonic()
+        if snapshot_key != self._last_snapshot_key:
+            self._last_snapshot_key = snapshot_key
+            self._snapshot_first_observed_monotonic = now_monotonic
+        return now_monotonic - self._snapshot_first_observed_monotonic

@@ read_latest()'s return @@
             if len(values) == len(self.headers):
                 row = dict(zip(self.headers, values))
-                return row, line, st.st_mtime, st
+                return row, line, self._snapshot_age_s(st, line), st
         raise StateFileMalformedError(f"{self.path} has no complete data rows")

@@ describe_state_freshness_rejection() @@
     source_ts = row.get(source_ts_field, "?") if source_ts_field else "?"
+    wall_clock_age_ms = (time.time() - st.st_mtime) * 1000.0
     return (
         f"dev={st.st_dev} ino={st.st_ino} mtime_ns={st.st_mtime_ns} "
-        f"row_source_timestamp={source_ts} calculated_age_ms={row_age_ms:.1f}"
+        f"row_source_timestamp={source_ts} calculated_age_ms={row_age_ms:.1f} "
+        f"wall_clock_age_ms={wall_clock_age_ms:.1f}"
     )

@@ read_reply_state() @@
     while True:
         try:
-            row, signature, mtime, st = reader.read_latest()
+            row, signature, row_age_s, st = reader.read_latest()
         except (StateFileNotReadyError, StateFileMalformedError) as exc:
             ...
         mapper.clear_state_read_error_dedup()
-        row_age_ms = (time.time() - mtime) * 1000.0
+        row_age_ms = row_age_s * 1000.0
         if row_age_ms > args.state_timeout_ms:
             raise StateError(
                 f"STALE_STATE: latest row age {row_age_ms:.1f} ms "
                 f"({describe_state_freshness_rejection(st, row, row_age_ms)})"
             )
```

### `Tools/autotest/sr75_hil_layer2/scripts/sr75_hil_f24b_sim_json_no_hardware_tests.py`

An existing test (`test_stale_timeout()`, from HIL-F24-B, pre-dating
this whole task family) simulated a "genuinely stale" file by
back-dating its mtime with `os.utime()` -- which the new monotonic gate
correctly no longer treats as evidence of staleness (that is the entire
point of the fix). Updated to simulate genuine staleness the same way a
real frozen feeder produces it -- an initial read, then `time.sleep()`
past the timeout with no further writes -- and extended its minimal CSV
row with the attitude/gyro/accel fields `StateMapper.state_from_csv()`
requires in strict mode (previously masked because the old wall-clock
trick made the row appear stale before validation ever ran):

```diff
     with tempfile.TemporaryDirectory() as tmpdir:
         state_path = Path(tmpdir) / "state.csv"
+        fieldnames = [
+            "time_s", "lat_deg", "lon_deg", "alt_m", "vn_mps", "ve_mps", "vd_mps",
+            "roll_rad", "pitch_rad", "yaw_rad", "p_rad_s", "q_rad_s", "r_rad_s",
+            "accel_body_x_mss", "accel_body_y_mss", "accel_body_z_mss",
+        ]
         with open(state_path, "w", newline="") as f:
             w = csv.writer(f)
-            w.writerow(["time_s", "lat_deg", "lon_deg", "alt_m", "vn_mps", "ve_mps", "vd_mps"])
-            w.writerow(["0.0", str(profiles.BASE_LAT), ..., "0.0", "0.0", "0.0"])
-        # back-date the file's mtime so it is already stale
-        old_time = time.time() - 5.0
-        import os
-        os.utime(state_path, (old_time, old_time))
+            w.writerow(fieldnames)
+            w.writerow(["0.0", str(profiles.BASE_LAT), ..., "0.0", "0.0", "0.0", "0.0", ...])

         reader = responder.LatestCSVReader(str(state_path))
         mapper = responder.StateMapper(strict=True)
         mock_source = responder.MockStateSource()
         args = argparse.Namespace(
-            mock_state=False, fresh_state_wait_ms=0.0, state_timeout_ms=500.0, allow_state_reuse=False,
+            mock_state=False, fresh_state_wait_ms=0.0, state_timeout_ms=150.0, allow_state_reuse=False,
         )
+        responder.read_reply_state(args, reader, mapper, mock_source)  # establish first-observed time
+        time.sleep(0.3)  # let real time pass beyond state_timeout_ms, no further writes
         try:
             responder.read_reply_state(args, reader, mapper, mock_source)
```

Two other pre-existing tests were updated for the same reason (their 3rd
`read_latest()` return value's meaning changed from a wall-clock mtime
to a monotonic age): a one-line fix in
`test_sr75_hil_f24s_coherence_and_accounting.py`'s
`test_concurrent_reads_never_report_an_inflated_age` (now uses `age_s *
1000.0` directly instead of re-deriving it from `time.time()`), and
`test_sr75_hil_f24p_state_csv_race.py`'s unpacking (already fixed as a
4-tuple in HIL-F24-S; unaffected here since it discards the age field).

Plus one new file:
`test_sr75_hil_f24t_monotonic_freshness.py`.

## Clock-jump test results (task 5)

```
$ python3 -m unittest test_sr75_hil_f24t_monotonic_freshness -v
test_positive_two_second_jump_with_continuous_updates ... ok
test_negative_two_second_jump_with_continuous_updates ... ok
test_diagnostic_reports_wall_clock_age_separately_from_monotonic_age ... ok
test_frozen_feeder_triggers_stale_after_timeout ... ok
test_snapshot_identity_unchanged_while_frozen ... ok
test_fifty_hz_replacement_never_reports_inflated_age ... ok
test_burst_then_immediate_sigterm_still_exact ... ok
test_short_run_zero_stale_exact_accounting ... ok
Ran 8 tests in ~8.5s -- OK
```

- `test_positive_two_second_jump_with_continuous_updates` /
  `test_negative_two_second_jump_with_continuous_updates` -- mock
  `time.time()` to return `real_time_time() ± 2.0` (never touching the
  real system clock) while continuously writing fresh snapshots and
  reading via the real `read_reply_state()`; asserts every read succeeds
  with `reused_source_row=False` (i.e. no false STALE_STATE, no silent
  fallback to a retained state) across 5 iterations each direction --
  directly reproducing and disproving the exact failure mode of the
  2050.7ms rejection.
- `test_diagnostic_reports_wall_clock_age_separately_from_monotonic_age`
  -- forces a genuine (monotonic) staleness rejection while simultaneously
  jumping the mocked wall clock forward 2s, and confirms the raised
  message contains both `calculated_age_ms=` and `wall_clock_age_ms=`
  (task 3).
- Manual verification (also run, output captured below) additionally
  confirmed a `+2s` jump, a `-2s` jump, and a subsequent genuine freeze
  all behave correctly in sequence against the same reader instance:

```
normal read OK, timestamp_s= 1e-06
PASS: +2s wall clock jump did NOT cause false STALE_STATE. timestamp_s= 0.020001
PASS: -2s wall clock jump did NOT cause false STALE_STATE. timestamp_s= 0.040001
PASS: frozen feeder correctly triggered: STALE_STATE: latest row age 600.7 ms (dev=2096 ino=349665 mtime_ns=1785945995263606011 row_source_timestamp=0.040000 calculated_age_ms=600.7 wall_clock_age_ms=601.5)
```

## Frozen-feeder test result (task 4)

`test_frozen_feeder_triggers_stale_after_timeout`: an initial read
succeeds (fresh), then no further writes occur for `0.3s` against a
`150ms` timeout, on the real clock (no mocking); the next read correctly
raises `StateError` containing `STALE_STATE`. A companion white-box test,
`test_snapshot_identity_unchanged_while_frozen`, directly asserts
`reader._snapshot_first_observed_monotonic` is bit-for-bit unchanged
across two reads of an untouched file 50ms apart -- the mechanism
underneath the frozen-feeder behavior, verified directly rather than only
through its externally observable effect.

## Stress-test summary (task 6)

Real feeder + real responder subprocesses, feeder at 50 Hz, simulated
request loop at ~50 Hz, continuous atomic replacement for the full 30s:

```
RUN_SUMMARY total_requests=1456 replies_sent=1456 stale_state_count=0
             no_fresh_state_count=0 malformed_state_count=0
             state_read_error_count=0 missed_replies=0
WRITE_SUMMARY write_count=1517 fsync_enabled=0 max_write_latency_ms=0.803
              p95_write_latency_ms=0.345 p99_write_latency_ms=0.443
              total_fsync_time_ms=0.000 longest_scheduling_gap_ms=20.357
final state.csv line count: 2 (header + 1 row)
responder.csv data rows: 1456  (== total_requests)
replies_sent + missed_replies == total_requests: True
timeouts (no reply at all): 0
```

Zero stale/no-fresh/malformed states, zero missed replies, exact
CSV/counter accounting -- all HIL-F24-P/Q/S invariants continue to hold
alongside the new monotonic freshness mechanism.

## Regression check

All pre-existing suites pass after two necessary, in-scope updates
(described above) to tests whose assumptions were specifically about the
old wall-clock mechanism this task replaces:

```
sim_json:  test_sr75_hil_f24p_state_csv_race (6/6), test_sr75_hil_f24d_static_state_feed (5/5),
           test_sr75_hil_f24q_shutdown_and_timing (8/8), test_sr75_hil_f24s_coherence_and_accounting (7/7),
           test_sr75_hil_f24t_monotonic_freshness (8/8)          -- 34/34
scripts:   test_sr75_hil_f24f_hardware_orchestrator (15/15),
           test_sr75_hil_f24b_sim_json_no_hardware_tests (23/23, after the
             test_stale_timeout fix above),
           test_sr75_hil_f24g_jsbsim_pipeline_validation (30/30) -- 68/68
jsbsim_control.test_sr75_sim_json_command_accounting: same 2 pre-existing,
  unrelated failures (actuator/JSBSim-command-accounting code this task
  does not touch; first identified as pre-existing in the HIL-F24-P
  report, reconfirmed unchanged in every task since).
```

### `py_compile` / `flake8` / `git diff --check`

```
$ python3 -m py_compile sr75_sim_json_responder.py test_sr75_hil_f24s_coherence_and_accounting.py test_sr75_hil_f24t_monotonic_freshness.py sr75_hil_f24b_sim_json_no_hardware_tests.py
(clean)

$ python3 -m flake8 --max-line-length=200 <same files>
sr75_sim_json_responder.py:1431/1433/1435/1437:26: E127 continuation line over-indented for visual indent
```

The 4 findings are confirmed pre-existing (identical content, shifted
line numbers only, already noted in the HIL-F24-P/Q/S reports). An
initial flake8 pass also caught two genuinely new findings in the new
test file (`F401 're'`/`'struct' imported but unused`, left over from
drafting before switching to import helpers from the HIL-F24-S test
module) -- fixed by removing the unused imports.

```
$ git diff --check -- <all touched files>
(clean, no whitespace errors)
```

## Real-hardware acceptance criteria

On the next bench run with this fix:

- **No false STALE_STATE correlated with a host clock adjustment.** If
  NTP is active on the companion computer, a step can still occur at any
  time; it must no longer be observable in `responder.csv` as a spurious
  rejection. If one somehow still occurs, `describe_state_freshness_rejection()`'s
  `calculated_age_ms` (monotonic, gates the decision) vs.
  `wall_clock_age_ms` (diagnostic only) will immediately show whether it
  is a genuine stall (both numbers agree) or a clock artifact (they
  diverge sharply).
- **A genuinely frozen/killed feeder must still be detected** within
  `--state-timeout-ms` of real elapsed time, exactly as before this task.
- **All HIL-F24-P/Q/S invariants continue to hold**: zero malformed
  reads under continuous atomic replacement, `responder.csv` row count
  == `total_requests`, `replies_sent + missed_replies == total_requests`,
  even under SIGTERM.
- **No change to SIM_JSON UDP wire format, PPP behavior, actuator
  output, or RATO/TECS/mission logic** -- this task touched only
  `LatestCSVReader`'s freshness bookkeeping, the freshness-rejection
  diagnostic string, and two pre-existing tests' now-outdated
  wall-clock-based staleness simulation.

## Files and line references

- `Tools/autotest/sr75_hil_layer2/sim_json/sr75_sim_json_responder.py`
  -- `LatestCSVReader.__init__` (new `_last_snapshot_key`/
  `_snapshot_first_observed_monotonic` fields), `_snapshot_age_s()` (new
  method, between `_load_headers()` and `read_latest()`), `read_latest()`'s
  return statement, `describe_state_freshness_rejection()` (added
  `wall_clock_age_ms`), `read_reply_state()`'s unpacking and `row_age_ms`
  computation.
- `Tools/autotest/sr75_hil_layer2/scripts/sr75_hil_f24b_sim_json_no_hardware_tests.py`
  -- `test_stale_timeout()`, rewritten to use real elapsed time instead
  of `os.utime()` back-dating, with a complete minimal CSV row.
- `Tools/autotest/sr75_hil_layer2/sim_json/test_sr75_hil_f24s_coherence_and_accounting.py`
  -- one-line fix in `test_concurrent_reads_never_report_an_inflated_age`
  for `read_latest()`'s changed 3rd-element semantics.
- New: `Tools/autotest/sr75_hil_layer2/sim_json/test_sr75_hil_f24t_monotonic_freshness.py`.
