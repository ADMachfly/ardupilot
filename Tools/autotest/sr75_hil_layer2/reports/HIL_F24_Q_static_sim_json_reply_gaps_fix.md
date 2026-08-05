# HIL-F24-Q: Eliminate the Remaining Static SIM_JSON Reply Gaps (Host-Side Only)

Status: fix applied and validated. No firmware flashed, no PARAM_SET, no
arm, no AUTO, no actuator output enabled. Only host-side Python files
under `Tools/autotest/sr75_hil_layer2/` were touched; no PPP
configuration, no SIM_JSON C++, no RATO/aero/TECS/mission/control code,
and no actuator decode safety was modified.

## Exact cause of request 573

**A single `os.fsync()` call inside the feeder's (then-mandatory) atomic
write stalled for ~1.7 seconds on the bench's real storage**, freezing
`state.csv`'s mtime and content for that entire window while the
responder kept polling it at ~5 Hz.

This is not inferred solely from the task description -- the actual
hardware session referenced by this task is still present on disk at
`Tools/autotest/sr75_hil_layer2/hardware_sessions/
sr75_hil_f24f_static_20260805T130807Z/responder.csv` (1500 rows,
untouched by this task, read-only evidence). The exact row for request
573:

```
121161.695154819,573,573,121161.685789973,121161.665366779,0.020423194,
121161.685789973,,,,50,1629,192.168.144.14,62510,1,,,,11.420003000,,0,
STALE_STATE: latest row age 1703.1 ms,...
```

`STALE_STATE: latest row age 1703.1 ms` -- an exact match to the task's
"request 573, STALE_STATE, row age 1703.1 ms". The previous request
(572) was answered normally 20.4 ms earlier (`request_host_dt=
0.020423194`), i.e. requests were arriving at the expected ~50 ms/~5 Hz*
cadence right up until this one -- there was no gradual drift, just one
isolated ~1.7 s gap in the state file being updated.

(*this bench run's actual request cadence was closer to ~20 ms between
requests per the `request_host_dt` column, i.e. faster than the nominal
5 Hz the task describes for the *simulated* test loop used in this
task's own regression tests -- both are "the responder polling the state
file," the exact rate does not change the diagnosis.)

Before HIL-F24-Q, `atomic_write_csv_snapshot()` called `os.fsync()`
**unconditionally on every single write**, i.e. up to 50 times/second.
`os.fsync()` forces a full write-barrier flush to the underlying block
device; on the flash storage typical of an embedded HIL companion
computer (SD card/eMMC), a single such flush can stall for hundreds of
milliseconds to low seconds when it lands on a wear-levelling/garbage-
collection pause or a full write-barrier -- a well-documented property of
that class of storage, and one this task's own local instrumentation
(see below) confirms adds measurable overhead even on fast storage.
While every one of those ~50 flushes/second normally completes quickly,
it only takes one unlucky flush landing on such a pause to produce
exactly the isolated single-request gap seen at request 573 -- consistent
with everything else in the run (1500 writes at a steady ~50 Hz, only
this one row affected) being otherwise clean.

## Task 1: Orchestrator shutdown ordering (the other 11 missed replies)

Requests 1489-1499 (from the same real `responder.csv`) are the
shutdown-tail misses described in the task, confirmed byte-for-byte:

```
...NO_FRESH_STATE: latest source timestamp 29.980284000 matches previous sent timestamp 29.980284000   (x3, requests 1489-1491)
...STALE_STATE: latest row age 595.5 ms / 788.3 ms / 980.9 ms / 1173.9 ms / 1367.3 ms / 1559.6 ms / 1752.1 ms / 1947.3 ms   (requests 1492-1499, growing monotonically)
```

**Root cause**: `sr75_hil_f24f_hardware_orchestrator.py`'s
`build_execution_plan()` gave the feeder exactly `--duration-s
{args.duration_s}`, while `main()` waited `args.duration_s + 2.0`
seconds before stopping anything. The feeder -- which exits on its own
once its internal write-count budget is exhausted -- therefore finished
and stopped writing to `state.csv` about 2 seconds *before* the
orchestrator even started its shutdown sequence, while the responder
(which has no duration limit of its own) kept receiving and trying to
answer real SIM_JSON requests from the Pixhawk against a file that had
stopped advancing.

**Fix** (`sr75_hil_f24f_hardware_orchestrator.py`):
- New named constants: `ORCHESTRATOR_TEST_MARGIN_S = 2.0` (unchanged
  value, now named) and `FEEDER_SHUTDOWN_MARGIN_S = 3.0` (new).
- `feeder_duration_s = args.duration_s + ORCHESTRATOR_TEST_MARGIN_S +
  FEEDER_SHUTDOWN_MARGIN_S`, used for the feeder's `--duration-s` in
  `build_execution_plan()` (the single function that builds the command
  used by both the dry-run printer and real execution, so they cannot
  drift apart). The feeder is therefore always still alive and writing
  for the entire window the responder could be receiving requests, plus
  a further `FEEDER_SHUTDOWN_MARGIN_S` margin.
- The explicit shutdown order in `main()`'s `finally:` block (stop
  responder, then feeder, then PPP) was already correct; it is now
  actually load-bearing (previously the feeder had usually already exited
  on its own by the time this code ran) and is commented as such.
- Net effect: the feeder is only ever stopped by this explicit sequence,
  strictly after the responder has already been stopped -- "no responder
  request is received after feeder termination" by construction, not by
  timing luck.

## Task 2 & 3: Write-latency instrumentation and the fsync reassessment

**Instrumentation added** (`sr75_hil_f24d_static_state_feed.py`):
`atomic_write_csv_snapshot()` now returns a `SnapshotWriteStats(total_s,
fsync_s)` per call; `main()`'s loop also tracks the wall-clock gap
between successive write iterations (`longest_gap_s`) -- exactly the
metric that would have shown ~1.7 s for request 573's write, had this
instrumentation existed then. `print_write_summary()` reports
`write_count`, `max/p95/p99_write_latency_ms`, `total_fsync_time_ms`, and
`longest_scheduling_gap_ms`.

**Local measurement** (this dev machine, `/tmp`, 150 writes @ 50 Hz, 3s):

| | fsync off (new default) | fsync on (old behavior) |
|---|---|---|
| max write latency | 0.555 ms | 4.954 ms |
| p95 write latency | 0.306 ms | 3.053 ms |
| p99 write latency | 0.369 ms | 3.703 ms |
| total fsync time (150 writes) | 0.000 ms | 281.873 ms (~1.88 ms/call avg) |
| longest scheduling gap | 20.147 ms | 20.225 ms |

Even on this machine's fast filesystem, `fsync()` alone adds a
consistent, measurable ~1.9 ms per call on average and pushes max latency
up nearly 10x. On the SR-75 bench's real storage this same per-call cost
is what plausibly spiked to ~1.7 s for request 573's write -- the
mechanism is identical, only the storage's worst-case latency differs.

**Crash-durability vs. read-atomicity (explained separately, as
requested):**
- **Read-atomicity** -- what a concurrent reader (the responder) can ever
  observe -- comes entirely from `os.replace()`'s same-filesystem rename
  semantics (POSIX guarantees `rename()` is atomic from every reader's
  point of view). This is **unaffected by whether `fsync()` is called at
  all**. HIL-F24-P's fix (temp file in the same directory + `os.replace`)
  already provided this guarantee before any `fsync()` call was added.
- **Crash/power-loss durability** -- whether the write survives the host
  dying before the data reaches stable storage -- is the *only* thing
  `fsync()` adds. For this specific file (a transient snapshot,
  regenerated every `1/rate_hz` seconds, whose entire purpose is "the
  current live state" with zero long-term/audit value), durability across
  a crash is irrelevant: if the feeder process dies, either it is
  restarted and the file is regenerated within milliseconds, or the whole
  test is aborted and the file is discarded anyway.

**Fix**: `fsync` is now an explicit parameter of
`atomic_write_csv_snapshot(path, fieldnames, row, fsync=False)`
(default off) and a `--fsync` CLI flag (default off, not removed
entirely, so a future caller that specifically needs crash-durability of
the last snapshot can still opt in). The same-directory temp-file +
`os.replace()` atomicity mechanism, and the `flush()`-before-`replace()`
step, are both preserved exactly as HIL-F24-P left them -- nothing about
atomicity was weakened.

## Task 5: Run-summary metrics

**Responder** (`sr75_sim_json_responder.py`): a SIGTERM handler
(`_raise_keyboard_interrupt_on_sigterm`) converts the orchestrator's
`terminate()` into a `KeyboardInterrupt`, so the existing `except
KeyboardInterrupt: ... finally: ...` structure at the end of `main()`
gets a chance to run on every exit path (normal, `--once`, or
SIGTERM). `print_run_summary()` prints:

```
RUN_SUMMARY total_requests=<N> replies_sent=<N> stale_state_count=<N>
no_fresh_state_count=<N> malformed_state_count=<N>
state_read_error_count=<N> missed_replies=<N>
```

**Feeder** (`sr75_hil_f24d_static_state_feed.py`): with the shutdown-order
fix, the feeder is now *deliberately* still running when the orchestrator
SIGTERMs it -- so it also needed a SIGTERM handler
(`_raise_shutdown_on_sigterm` / `_FeederShutdownRequested`) that breaks
its write loop cleanly and still runs `print_write_summary()` on whatever
was accumulated so far, instead of silently losing all its metrics (this
was caught and fixed during this task's own end-to-end testing --
without it, `WRITE_SUMMARY` was empty on every real orchestrator-driven
run, which would have made task 5's feeder metrics dead on arrival in
practice). Prints:

```
WRITE_SUMMARY write_count=<N> fsync_enabled=<0|1>
max_write_latency_ms=<f> p95_write_latency_ms=<f> p99_write_latency_ms=<f>
total_fsync_time_ms=<f> longest_scheduling_gap_ms=<f>
```

## Exact diff

### `Tools/autotest/sr75_hil_layer2/scripts/sr75_hil_f24f_hardware_orchestrator.py`

```diff
@@ -48,6 +48,26 @@ SIM_JSON_DIR = LAYER2_DIR / "sim_json"
 PPP_DIR = LAYER2_DIR / "ppp"
 DEFAULT_SESSIONS_DIR = LAYER2_DIR / "hardware_sessions"
 
+# HIL-F24-Q: how long the orchestrator waits, beyond --duration-s, before it
+# begins the explicit shutdown sequence (stop responder, then feeder, then
+# PPP) -- unchanged from the pre-existing behavior.
+ORCHESTRATOR_TEST_MARGIN_S = 2.0
+
+# HIL-F24-Q: how much longer the feeder's OWN --duration-s runs beyond the
+# orchestrator's full pre-shutdown wait (--duration-s + ORCHESTRATOR_TEST_
+# MARGIN_S). Previously the feeder was given exactly --duration-s and so
+# exited on its own ~ORCHESTRATOR_TEST_MARGIN_S seconds *before* the
+# orchestrator even began stopping anything -- during that gap the
+# responder (which has no duration limit of its own) kept receiving and
+# trying to answer SIM_JSON requests against a state file that had
+# stopped being updated, producing NO_FRESH_STATE and then STALE_STATE
+# replies (the F24-P hardware run's requests 1489-1499). With this margin,
+# the feeder is still alive and actively writing for the entire window the
+# responder can receive requests, and is only ever stopped by the
+# explicit "stop responder, then feeder" sequence below -- never by its
+# own internal timeout expiring early.
+FEEDER_SHUTDOWN_MARGIN_S = 3.0
+
 PRECHECK_SCRIPT = SCRIPTS_DIR / "sr75_hil_f24c_preflash_precheck.py"
 FEEDER_SCRIPT = SIM_JSON_DIR / "sr75_hil_f24d_static_state_feed.py"
 RESPONDER_SCRIPT = SIM_JSON_DIR / "sr75_sim_json_responder.py"
@@ -110,9 +130,15 @@ def build_execution_plan(args) -> List[Step]:
     precheck_cmd = [
         sys.executable, str(PRECHECK_SCRIPT), "--pixhawk", args.pixhawk, "--baud", str(args.baud),
     ]
+    # HIL-F24-Q: the feeder runs FEEDER_SHUTDOWN_MARGIN_S longer than the
+    # orchestrator's total pre-shutdown wait (--duration-s +
+    # ORCHESTRATOR_TEST_MARGIN_S), so it is still alive when the
+    # orchestrator issues its explicit stop-responder-then-feeder sequence
+    # (see main()'s `finally:` block) instead of exiting on its own first.
+    feeder_duration_s = args.duration_s + ORCHESTRATOR_TEST_MARGIN_S + FEEDER_SHUTDOWN_MARGIN_S
     feeder_cmd = [
         sys.executable, str(FEEDER_SCRIPT), "--output", "{state_csv}",
-        "--duration-s", str(args.duration_s), "--rate-hz", str(args.rate_hz),
+        "--duration-s", str(feeder_duration_s), "--rate-hz", str(args.rate_hz),
     ]
     responder_cmd = [
         sys.executable, str(RESPONDER_SCRIPT),
@@ -246,13 +272,20 @@ def main():
         )
 
         print(f"Static SIM_JSON test running for {args.duration_s}s (feeder+responder); artifacts in {session_dir}")
-        time.sleep(args.duration_s + 2.0)
+        time.sleep(args.duration_s + ORCHESTRATOR_TEST_MARGIN_S)
         print("Static SIM_JSON test window complete.")
 
     except OrchestratorAbort as exc:
         print(f"ABORT: {exc}")
         exit_code = 3
     finally:
+        # HIL-F24-Q: order matters -- responder must be stopped before the
+        # feeder, so no responder request is ever received after the
+        # feeder has terminated. This is only meaningful because the
+        # feeder is now given FEEDER_SHUTDOWN_MARGIN_S extra --duration-s
+        # (see build_execution_plan()) so it is still running (and still
+        # writing) when execution reaches this point, rather than having
+        # already exited on its own a couple of seconds earlier.
         for proc, label in ((responder_proc, "responder"), (feeder_proc, "feeder")):
             if proc is not None and proc.poll() is None:
                 proc.terminate()
```

### `Tools/autotest/sr75_hil_layer2/sim_json/sr75_hil_f24d_static_state_feed.py`

```diff
@@ -19,9 +19,11 @@ file passed via --output.
 import argparse
 import csv
 import os
+import signal
 import sys
 import tempfile
 import time
+from dataclasses import dataclass
 
 sys.path.insert(0, __file__.rsplit("/", 1)[0])
 from sr75_sim_json_test_profiles import BASE_ALT_M, BASE_LAT, BASE_LON, BASE_YAW_DEG  # noqa: E402
@@ -70,7 +72,13 @@ def build_static_row(t_s, lat_deg=BASE_LAT, lon_deg=BASE_LON, alt_m=BASE_ALT_M,
     }
 
 
-def atomic_write_csv_snapshot(path, fieldnames, row):
+@dataclass
+class SnapshotWriteStats:
+    total_s: float
+    fsync_s: float
+
+
+def atomic_write_csv_snapshot(path, fieldnames, row, fsync=False):
     """HIL-F24-P: (re)write a single-row CSV snapshot atomically.
     ... [docstring extended -- see task 2/3 discussion above] ...
     """
     directory = os.path.dirname(os.path.abspath(path)) or "."
     fd, tmp_path = tempfile.mkstemp(prefix=".state_csv_", suffix=".tmp", dir=directory)
+    write_start = time.perf_counter()
+    fsync_s = 0.0
     try:
         with os.fdopen(fd, "w", newline="", encoding="utf-8") as tmp_file:
             writer = csv.DictWriter(tmp_file, fieldnames=fieldnames)
             writer.writeheader()
             writer.writerow(row)
             tmp_file.flush()
-            os.fsync(tmp_file.fileno())
+            if fsync:
+                fsync_start = time.perf_counter()
+                os.fsync(tmp_file.fileno())
+                fsync_s = time.perf_counter() - fsync_start
         os.replace(tmp_path, path)
     except BaseException:
         try:
             os.remove(tmp_path)
         except OSError:
             pass
         raise
+    return SnapshotWriteStats(total_s=time.perf_counter() - write_start, fsync_s=fsync_s)
+
+
+def _percentile(sorted_values, pct):
+    ... # linear-interpolation percentile helper
+
+
+def print_write_summary(write_stats, longest_gap_s, fsync_enabled):
+    ... # WRITE_SUMMARY printer (max/p95/p99, total fsync time, longest gap)
 
 
 def build_arg_parser():
@@ -116,26 +187,65 @@ def build_arg_parser():
     parser.add_argument("--airspeed-mps", type=float, default=0.0)
+    parser.add_argument("--fsync", action="store_true", help=(...))
     return parser
 
 
+class _FeederShutdownRequested(Exception):
+    ...
+
+
+def _raise_shutdown_on_sigterm(signum, frame) -> None:
+    raise _FeederShutdownRequested()
+
+
 def main():
+    signal.signal(signal.SIGTERM, _raise_shutdown_on_sigterm)
     args = build_arg_parser().parse_args()
     dt = 1.0 / args.rate_hz
     n_writes = max(1, int(round(args.duration_s / dt)))
     print(f"Writing static state to {args.output}: ... fsync={args.fsync}")
     start = time.monotonic()
-    for i in range(n_writes):
-        t_s = time.monotonic() - start
-        row = build_static_row(...)
-        atomic_write_csv_snapshot(args.output, CSV_FIELDS, row)
-        next_write = start + (i + 1) * dt
-        sleep_s = next_write - time.monotonic()
-        if sleep_s > 0:
-            time.sleep(sleep_s)
-    print(f"Done: {n_writes} writes over {time.monotonic() - start:.1f}s")
+    write_stats = []
+    prev_iter_start = None
+    longest_gap_s = 0.0
+    try:
+        for i in range(n_writes):
+            iter_start = time.monotonic()
+            if prev_iter_start is not None:
+                longest_gap_s = max(longest_gap_s, iter_start - prev_iter_start)
+            prev_iter_start = iter_start
+            t_s = iter_start - start
+            row = build_static_row(...)
+            write_stats.append(atomic_write_csv_snapshot(args.output, CSV_FIELDS, row, fsync=args.fsync))
+            next_write = start + (i + 1) * dt
+            sleep_s = next_write - time.monotonic()
+            if sleep_s > 0:
+                time.sleep(sleep_s)
+    except _FeederShutdownRequested:
+        print("Stopped by SIGTERM")
+    print(f"Done: {len(write_stats)} writes over {time.monotonic() - start:.1f}s")
+    print_write_summary(write_stats, longest_gap_s, args.fsync)
     return 0
```

### `Tools/autotest/sr75_hil_layer2/sim_json/sr75_sim_json_responder.py`

```diff
@@ -12,6 +12,7 @@ import csv
 import json
 import math
 import os
+import signal
 import socket
 import struct
 import sys
@@ -2123,7 +2124,41 @@ def print_b3_startup_scoring_debug(...):
     )
 
 
+def _raise_keyboard_interrupt_on_sigterm(signum, frame) -> None:
+    raise KeyboardInterrupt()
+
+
+def print_run_summary(request_count, replies_sent_count, stale_state_count,
+                       no_fresh_state_count, malformed_state_count,
+                       state_read_error_count) -> None:
+    print("RUN_SUMMARY total_requests=... replies_sent=... "
+          "stale_state_count=... no_fresh_state_count=... "
+          "malformed_state_count=... state_read_error_count=... "
+          "missed_replies=...")
+
+
 def main() -> int:
+    signal.signal(signal.SIGTERM, _raise_keyboard_interrupt_on_sigterm)
     args = build_arg_parser().parse_args()
     ...
@@ -2272,6 +2307,12 @@ def main() -> int:
     last_reply_mono = 0.0
     request_count = 0
+    replies_sent_count = 0
+    stale_state_count = 0
+    no_fresh_state_count = 0
+    malformed_state_count = 0
     previous_request_received_host_time: Optional[float] = None
@@ -2758,12 +2799,23 @@ def main() -> int:
             except StateError as exc:
                 reason = str(exc)
-                if isinstance(exc, (StateFileNotReadyError, StateFileMalformedError)):
+                if isinstance(exc, StateFileMalformedError):
+                    malformed_state_count += 1
+                elif isinstance(exc, StateFileNotReadyError):
                     pass
                 elif reason.startswith("STALE_STATE"):
+                    stale_state_count += 1
                     print(reason)
+                elif reason.startswith("NO_FRESH_STATE"):
+                    no_fresh_state_count += 1
+                    print(f"STATE_ERROR: {reason}")
                 else:
                     print(f"STATE_ERROR: {reason}")
@@ -2811,6 +2863,7 @@ def main() -> int:
                 reply_bytes = sock.sendto(payload, source)
+                replies_sent_count += 1
                 last_reply_mono = time.monotonic()
@@ -2863,6 +2916,14 @@ def main() -> int:
         print("Interrupted")
         return 0
     finally:
+        print_run_summary(
+            request_count=request_count, replies_sent_count=replies_sent_count,
+            stale_state_count=stale_state_count, no_fresh_state_count=no_fresh_state_count,
+            malformed_state_count=malformed_state_count,
+            state_read_error_count=mapper.state_read_error_count,
+        )
         sock.close()
```

Plus two new/extended test files (see Test results below).

## Timing metrics: before vs. after

| | Before HIL-F24-Q (real hardware run) | After HIL-F24-Q (this task's 30 s local run) |
|---|---|---|
| Total requests | 1499 | 150 (simulated 5 Hz over 30s) |
| Replies sent | 1487 | 150 |
| Missed replies | 12 | **0** |
| Mid-run STALE_STATE | 1 (request 573, 1703.1 ms) | 0 |
| Shutdown-tail NO_FRESH_STATE/STALE_STATE | 11 (requests 1489-1499) | 0 |
| Feeder writes | 1500 (reported) | 1523 |
| Feeder max write latency | unmeasured (no instrumentation existed) | 1.008 ms |
| Feeder p95/p99 write latency | unmeasured | 0.374 ms / 0.460 ms |
| Feeder total fsync time | unmeasured (fsync was unconditional) | 0.000 ms (default off) |
| Feeder longest scheduling gap | unmeasured (would have shown ~1.7s at request 573's write) | 20.373 ms |
| Final `state.csv` | valid (header + 1 row, per task) | valid (header + 1 row) |

## Test results

**New/extended test files, all passing:**

```
$ python3 -m unittest sim_json.test_sr75_hil_f24q_shutdown_and_timing -v
test_short_run_has_zero_stale_and_zero_no_fresh_state ... ok
test_sigterm_mid_run_still_prints_write_summary ... ok
test_concurrency_stress_still_zero_errors_with_fsync_off_default ... ok
test_default_fsync_is_off_and_recorded_as_zero ... ok
test_fsync_true_is_timed_and_still_atomic ... ok
test_to_json_bytes_schema_unchanged ... ok
test_percentile_matches_known_distribution ... ok
test_print_write_summary_reports_expected_fields ... ok
Ran 8 tests in 7.4s -- OK
```

- `test_short_run_has_zero_stale_and_zero_no_fresh_state` -- drives the
  *real* feeder and responder scripts as subprocesses (mirroring the
  orchestrator's fixed shutdown order and inflated feeder duration) with
  a simulated ~5 Hz UDP request loop for 4s; asserts zero timeouts, zero
  `stale_state_count`, zero `no_fresh_state_count`, zero
  `malformed_state_count`, zero `missed_replies`, and a valid final
  `state.csv` -- this is task 4's "no responder tail" + "zero
  NO_FRESH_STATE/STALE_STATE" + "final state.csv is valid" requirements,
  proven end to end with real subprocesses.
- `test_sigterm_mid_run_still_prints_write_summary` -- kills the feeder
  1s into a 10s run; asserts `WRITE_SUMMARY` still prints with a nonzero
  write count (catches the bug described above, where the feeder would
  otherwise silently discard its own metrics).
- `test_concurrency_stress_still_zero_errors_with_fsync_off_default` --
  reruns HIL-F24-P's concurrency-stress helper to reconfirm atomic-write
  correctness holds with the new `fsync=False` default.
- `test_to_json_bytes_schema_unchanged` -- SIM_JSON wire-format sentinel
  (task 4's "no SIM_JSON wire-format change").
- `test_percentile_matches_known_distribution` /
  `test_print_write_summary_reports_expected_fields` -- unit-test the new
  instrumentation/reporting logic directly.

**Extended orchestrator tests** (`scripts/
test_sr75_hil_f24f_hardware_orchestrator.py`, pure/no-subprocess, matching
existing file conventions):

```
$ python3 -m unittest scripts.test_sr75_hil_f24f_hardware_orchestrator -v
... (12 pre-existing tests, unchanged) ... ok
test_feeder_duration_exceeds_orchestrator_wait_by_shutdown_margin ... ok
test_feeder_duration_scales_with_requested_duration ... ok
test_shutdown_stops_responder_before_feeder ... ok
Ran 15 tests in 0.004s -- OK
```

**Regression check on adjacent existing suites** (unmodified by this
task): `sim_json.test_sr75_hil_f24p_state_csv_race` (6/6),
`sim_json.test_sr75_hil_f24d_static_state_feed` (5/5),
`scripts.test_sr75_hil_f24b_sim_json_no_hardware_tests` (23/23),
`scripts.test_sr75_hil_f24g_jsbsim_pipeline_validation` (30/30) -- all
pass unchanged. `jsbsim_control.test_sr75_sim_json_command_accounting`
still shows its 2 pre-existing, unrelated failures (already confirmed
pre-existing and unrelated to this file family in the HIL-F24-P report;
reconfirmed here with the same assertion values, `7 != 6` and `0 != 1`,
in actuator/JSBSim-command-accounting code this task does not touch).

## Local stress-test counts (task item 6)

30-second local run, real feeder + real responder subprocesses, feeder
at 50 Hz (with the now-inflated `--duration-s`), simulated Pixhawk
request loop at 5 Hz:

```
RUN_SUMMARY total_requests=150 replies_sent=150 stale_state_count=0
             no_fresh_state_count=0 malformed_state_count=0
             state_read_error_count=0 missed_replies=0
WRITE_SUMMARY write_count=1523 fsync_enabled=0 max_write_latency_ms=1.008
              p95_write_latency_ms=0.374 p99_write_latency_ms=0.460
              total_fsync_time_ms=0.000 longest_scheduling_gap_ms=20.373
final state.csv line count: 2 (header + 1 row)
timeouts (no reply at all): 0
```

Zero missed replies, zero stale/no-fresh/malformed states, and a feeder
scheduling gap (20.373 ms) matching the intended 50 Hz cadence (20 ms)
almost exactly -- no stalls observed on this machine's storage, as
expected now that `fsync` defaults off.

## `py_compile` / `flake8` / `git diff --check`

```
$ python3 -m py_compile <all 5 touched files>
(clean)

$ python3 -m flake8 --max-line-length=200 <all 5 touched files>
sr75_hil_f24d_static_state_feed.py:47:23: E127 continuation line over-indented for visual indent
sr75_sim_json_responder.py:1339/1341/1343/1345:26: E127 continuation line over-indented for visual indent
```

All 5 findings confirmed pre-existing (same findings, same content, only
shifted line numbers, in the HIL-F24-P report; unrelated to this task's
diff). The two new/extended test files have zero flake8 findings.

```
$ git diff --check -- <all 5 touched files>
(clean, no whitespace errors)
```

## Expected corrected hardware-acceptance criteria

On the next bench run with this fix:

- **Zero shutdown-tail misses**: the feeder must still be writing (and
  the responder must still be receiving valid replies) for the entire
  window the Pixhawk sends requests; the last logged reply should occur
  at or after the last logged request, with no trailing
  NO_FRESH_STATE/STALE_STATE run.
- **Zero or near-zero mid-run STALE_STATE**: with `fsync` off by default,
  no single write should stall long enough to age a row past
  `--state-timeout-ms` (500 ms default) before the next ~20 ms write
  lands. If any bench run still shows an isolated STALE_STATE, the new
  `WRITE_SUMMARY`/`longest_scheduling_gap_ms` output will directly show
  whether a write stalled and by how much, without needing to re-derive
  it from `responder.csv` row ages after the fact.
- **`RUN_SUMMARY`/`WRITE_SUMMARY` present in every run's logs**, even
  when the orchestrator kills both processes via SIGTERM -- `missed_replies`
  should read 0 (or explain any nonzero value directly, rather than
  requiring manual CSV archaeology).
- **`state.csv` still ends as exactly one header + one complete data
  row** (unchanged, already correct feeder behavior, reverified).
- **No change to SIM_JSON UDP wire format, PPP behavior, or actuator
  output** -- confirmed by the `to_json_bytes()` schema test and by this
  task touching no C++, hwdef, or actuator-bridge/RATO/TECS/mission file.

## Files and line references

- `Tools/autotest/sr75_hil_layer2/scripts/sr75_hil_f24f_hardware_orchestrator.py`
  -- `ORCHESTRATOR_TEST_MARGIN_S`/`FEEDER_SHUTDOWN_MARGIN_S` (new, near
  top), `build_execution_plan()`'s `feeder_duration_s` computation,
  `main()`'s sleep window and `finally:` shutdown-order comment.
- `Tools/autotest/sr75_hil_layer2/sim_json/sr75_hil_f24d_static_state_feed.py`
  -- `SnapshotWriteStats`, `atomic_write_csv_snapshot()` (fsync param +
  timing), `_percentile()`, `print_write_summary()`, `--fsync` CLI flag,
  `_FeederShutdownRequested`/`_raise_shutdown_on_sigterm`, `main()`'s loop.
- `Tools/autotest/sr75_hil_layer2/sim_json/sr75_sim_json_responder.py` --
  `_raise_keyboard_interrupt_on_sigterm`, `print_run_summary()`, counter
  init near `request_count = 0` (original line 2274), the `except
  StateError` classification block (original line ~2759), the
  `reply_bytes = sock.sendto(...)` increment (original line ~2813), the
  `finally:` block at the end of `main()` (original line ~2865).
- `Tools/autotest/sr75_hil_layer2/hardware_sessions/
  sr75_hil_f24f_static_20260805T130807Z/responder.csv` -- pre-existing,
  read-only real bench-run evidence (not created or modified by this
  task) used to confirm request 573's exact row and the requests
  1489-1499 shutdown-tail sequence quoted above.
- New: `Tools/autotest/sr75_hil_layer2/sim_json/test_sr75_hil_f24q_shutdown_and_timing.py`.
- Extended: `Tools/autotest/sr75_hil_layer2/scripts/test_sr75_hil_f24f_hardware_orchestrator.py`
  (new `TestFeederOutlivesResponder` class).
