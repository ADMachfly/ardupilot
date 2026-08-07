# HIL-F24-S: Reader TOCTOU Coherence and Shutdown Accounting Fix (Host-Side Only)

Status: fix applied and validated. No firmware flashed, no PARAM_SET, no
arm, no AUTO, no actuator output enabled. Only
`Tools/autotest/sr75_hil_layer2/sim_json/sr75_sim_json_responder.py` was
modified (plus a one-line mechanical fix to an existing test's
unpacking, and one new test file); no PPP configuration, no SIM_JSON
C++, no RATO/aero/TECS/mission/control code, and no actuator decode
safety was touched.

## Confirmed root cause of request 784

`LatestCSVReader.read_latest()` combined pathname-based metadata and
file content from up to **four separate filesystem calls** against the
same path:

```
1. os.path.exists(self.path)                         <- read_latest()
2. os.stat(self.path)          + open(self.path, "r") <- _load_headers()
3. os.stat(self.path)          + open(self.path, "rb") <- read_latest()
```

Each of these is an independent syscall. The feeder's atomic
`os.replace()` (from HIL-F24-P) can fire at any point between any two of
them, so calls 3's `os.stat()` (whose `st_mtime` becomes the row's
reported age) and its own `open()` a few lines later could each observe
a **different** snapshot of the file -- one older, one newer -- with no
relationship between the `st_mtime` used for the age calculation and the
row content actually parsed. This is a textbook TOCTOU
(time-of-check-to-time-of-use) race.

This exactly explains request 784: an isolated, single-request
`STALE_STATE` with a wildly inflated age (1546.5 ms) sandwiched between
two completely normal neighbors (request 783: 4.577 ms, request 785:
7.513 ms) is precisely the signature of a metadata/content mismatch
hitting unluckily on one read cycle -- not a real feeder stall (which
would show up as HIL-F24-Q's `longest_scheduling_gap_ms`, and this run's
feeder reported a clean max write latency of 0.838 ms and a scheduling
gap of 20.312 ms, ruling that out) and not a sustained problem (only one
row was ever affected out of 1588 requests).

**Evidence note:** unlike HIL-F24-Q (where the exact hardware capture
referenced by that task, `hardware_sessions/
sr75_hil_f24f_static_20260805T130807Z/responder.csv`, was still on disk
and its request 573 row was directly quoted and verified), no on-disk
capture exists for *this* run (1588 requests) -- that same directory's
capture is the earlier, 1499-request F24-Q run, and its own request 784
is unrelated and unremarkable (a normal, fresh ~10ms-age row). This
task's request 784/783/785 figures are therefore taken as given from the
task description, not independently re-verified against a raw CSV in
this session; the mechanism identified above is what the codebase's
actual (pre-fix) source code structurally permits, and the fix and its
tests are validated independently against real (freshly generated, not
historical) atomic-replace races in the Test results section below.

## Confirmed cause of the unlogged second request

`request_count` was incremented immediately after `sock.recvfrom()`
succeeded, at the very top of each request's processing -- but the
corresponding `responder.csv` row was only written much later, after
potentially hundreds of lines of B3/actuator/precontrol logic, via one
of five separate `write_log(...)` call sites. If the orchestrator's
SIGTERM landed anywhere in that (previously wide) window -- after the
counter was bumped but before that request's row was actually written
and flushed -- `request_count` would end up one higher than the number of
rows `responder.csv` actually contains. This matches the observed "CSV
has one fewer data record than total_requests" exactly, and is a
distinct bug from request 784's read-coherence issue -- it is a
count/log-ordering race on the *write* side, not a read-side TOCTOU.

## Task 2/3: Making every read coherent

`LatestCSVReader.read_latest()` now opens the file **exactly once** per
call, takes its metadata via `os.fstat()` on that **same open file
descriptor**, and reads both the header (task 3) and the trailing data
block from that same descriptor:

```python
def read_latest(self) -> Tuple[Dict[str, str], str, float, os.stat_result]:
    try:
        csv_file = open(self.path, "rb")
    except FileNotFoundError:
        raise StateFileNotReadyError(f"state file does not exist: {self.path}")
    try:
        st = os.fstat(csv_file.fileno())      # same fd, not os.stat(path)
        self._load_headers(csv_file, st)      # same fd
        csv_file.seek(0, os.SEEK_END)
        ...
        data = csv_file.read(block_size).decode("utf-8", errors="ignore")
    finally:
        csv_file.close()
    ...
    return row, line, st.st_mtime, st
```

This works because of a POSIX guarantee: once a file is `open()`ed, the
resulting file descriptor continues to reference the *same inode* for
its entire lifetime, no matter what `os.replace()` subsequently does to
the pathname. There is no longer a second filesystem call that could
observe a different snapshot -- metadata and content are now
structurally (not just probabilistically) guaranteed to agree.

`_load_headers()` (task 3) now takes the already-open file and the
caller's `os.fstat()` result directly, and caches on `(st_dev, st_ino,
st_mtime_ns)` instead of a bare mtime -- so the header cache can never be
satisfied by a *different inode* that happens to share a timestamp,
which a bare-mtime cache key could not rule out.

`read_latest()`'s return type grew a fourth element, the `os.stat_result`
itself, so callers that need it (the freshness-rejection diagnostic,
task 4) don't need a second, separate stat call of their own.

## Task 4: Freshness-rejection instrumentation

Added `describe_state_freshness_rejection()`, called only from the one
place `STALE_STATE` is raised in `read_reply_state()` -- never on the
normal/fresh path, so it adds zero per-request log volume:

```python
if row_age_ms > args.state_timeout_ms:
    raise StateError(
        f"STALE_STATE: latest row age {row_age_ms:.1f} ms "
        f"({describe_state_freshness_rejection(st, row, row_age_ms)})"
    )
```

producing, e.g.:

```
STALE_STATE: latest row age 612.3 ms (dev=64769 ino=1234567 mtime_ns=1712345678901234000 row_source_timestamp=11.420003 calculated_age_ms=612.3)
```

With the coherence fix in place, a metadata/content mismatch is no
longer structurally possible, so there is no separate "mismatch"
detection path to add -- the diagnostic covers the one rejection
condition (`STALE_STATE`) that can still legitimately occur (a genuinely
stale feeder), and now carries everything needed to diagnose a future
reader-side regression at a glance instead of requiring the kind of
`responder.csv` archaeology this task itself needed to confirm request
784.

## Task 5: Shutdown request-accounting atomicity

Every `write_log(log_writer, csv_file, make_log_row(...))` call site (5
of them: malformed packet, B3 time-discontinuity abort, B3 state-envelope
abort, generic `StateError`, and the normal reply path) now goes through
a single new closure, `_log_row_and_count()`, defined inside `main()`:

```python
def _log_row_and_count(row: Dict[str, object], sent: bool) -> None:
    nonlocal logged_request_count, replies_sent_count
    signal.pthread_sigmask(signal.SIG_BLOCK, _SIGTERM_BLOCK_SET)
    try:
        write_log(log_writer, csv_file, row)
        logged_request_count += 1
        if sent:
            replies_sent_count += 1
    finally:
        signal.pthread_sigmask(signal.SIG_UNBLOCK, _SIGTERM_BLOCK_SET)
```

**Definition of "counted" (task 5's explicit ask)**: a request is
counted -- i.e. contributes to `print_run_summary()`'s `total_requests`
and `replies_sent` -- **only at the moment its `responder.csv` row has
actually been written**, not when the UDP datagram was received. The
pre-existing `request_count` variable (used throughout the huge
per-request body for labeling, B3 scoring, timing-gate sequencing, etc.)
is completely unchanged; a *new* variable, `logged_request_count`, is
what `print_run_summary()` now reports as `total_requests`. In every
uninterrupted run the two are identical (every received request already
led to exactly one `write_log()` call before this task); they can only
differ in the exact shutdown race this task fixes.

`signal.pthread_sigmask(SIG_BLOCK, ...)` defers delivery of SIGTERM for
the duration of the row-write + counter-increment: if SIGTERM arrives
while blocked, the kernel marks it pending and it is delivered (invoking
`_raise_keyboard_interrupt_on_sigterm`, raising `KeyboardInterrupt`)
immediately after the matching `SIG_UNBLOCK` call -- by which point the
row is already written and both counters already incremented. This is a
structural guarantee (a kernel-enforced signal mask), not a
probabilistic narrowing of a race window.

`replies_sent_count += 1` was moved out of its old standalone site
(right after `sock.sendto()`) and into `_log_row_and_count(..., sent=not
args.dry_run)`, so it can never be incremented without its row also
being logged in the same atomic step -- otherwise a SIGTERM landing
between the send and the old, separate `write_log()` call could still
have produced `replies_sent_count > logged_request_count`, breaking the
"replies_sent + missed == total_requests" invariant even after fixing
`total_requests` alone.

Responder-before-feeder shutdown ordering (HIL-F24-Q) is untouched.

## Exact diff

### `Tools/autotest/sr75_hil_layer2/sim_json/sr75_sim_json_responder.py`

```diff
@@ -361,33 +361,76 @@ class SimState:
 
 
 class LatestCSVReader:
+    """HIL-F24-S: ... [see task 2/3 discussion above for the full docstring] ..."""
+
     def __init__(self, path: str):
         self.path = path
         self.headers: Optional[List[str]] = None
-        self._header_mtime_ns: Optional[int] = None
-
-    def _load_headers(self) -> None:
-        stat = os.stat(self.path)
-        if self.headers is not None and self._header_mtime_ns == stat.st_mtime_ns:
+        self._header_cache_key: Optional[Tuple[int, int, int]] = None
+
+    def _load_headers(self, csv_file, st: os.stat_result) -> None:
+        cache_key = (st.st_dev, st.st_ino, st.st_mtime_ns)
+        if self.headers is not None and self._header_cache_key == cache_key:
             return
-        with open(self.path, "r", newline="", encoding="utf-8") as csv_file:
-            self.headers = next(csv.reader(csv_file), None)
-        self._header_mtime_ns = stat.st_mtime_ns
+        csv_file.seek(0)
+        header_line = csv_file.readline().decode("utf-8", errors="ignore")
+        parsed = list(csv.reader([header_line]))
+        self.headers = parsed[0] if parsed else None
+        self._header_cache_key = cache_key
         if not self.headers:
             raise StateFileMalformedError(f"{self.path} has no CSV header")
 
-    def read_latest(self) -> Tuple[Dict[str, str], str, float]:
-        if not os.path.exists(self.path):
+    def read_latest(self) -> Tuple[Dict[str, str], str, float, os.stat_result]:
+        try:
+            csv_file = open(self.path, "rb")
+        except FileNotFoundError:
             raise StateFileNotReadyError(f"state file does not exist: {self.path}")
-        self._load_headers()
-        assert self.headers is not None
-        stat = os.stat(self.path)
-        with open(self.path, "rb") as csv_file:
+        try:
+            st = os.fstat(csv_file.fileno())
+            self._load_headers(csv_file, st)
+            assert self.headers is not None
             csv_file.seek(0, os.SEEK_END)
             end_pos = csv_file.tell()
             block_size = min(65536, end_pos)
             csv_file.seek(end_pos - block_size)
             data = csv_file.read(block_size).decode("utf-8", errors="ignore")
+        finally:
+            csv_file.close()
         lines = [line for line in data.splitlines() if line.strip()]
         if len(lines) < 2:
             raise StateFileMalformedError(f"{self.path} has no complete data rows")
@@ -398,7 +441,7 @@ class LatestCSVReader:
             values = parsed[0]
             if len(values) == len(self.headers):
                 row = dict(zip(self.headers, values))
-                return row, line, stat.st_mtime
+                return row, line, st.st_mtime, st
         raise StateFileMalformedError(f"{self.path} has no complete data rows")
 
 
@@ -2011,6 +2054,24 @@ class SR75DryBoosterEjectionLatch:
         return False
 
 
+def describe_state_freshness_rejection(st: os.stat_result, row: Dict[str, str], row_age_ms: float) -> str:
+    source_ts_field = first_present(row, (
+        "/fdm/jsbsim/simulation/sim-time-sec", "jsb_feed_time_s", "jsb_time_s", "time_s", "Time",
+    ))
+    source_ts = row.get(source_ts_field, "?") if source_ts_field else "?"
+    return (
+        f"dev={st.st_dev} ino={st.st_ino} mtime_ns={st.st_mtime_ns} "
+        f"row_source_timestamp={source_ts} calculated_age_ms={row_age_ms:.1f}"
+    )
+
+
 def wait_for_reply_slot(last_reply_mono: float, min_interval: float) -> bool:
@@ -2033,7 +2094,7 @@ def read_reply_state(
     while True:
         try:
-            row, signature, mtime = reader.read_latest()
+            row, signature, mtime, st = reader.read_latest()
         except (StateFileNotReadyError, StateFileMalformedError) as exc:
@@ -2066,7 +2127,16 @@ def read_reply_state(
         mapper.clear_state_read_error_dedup()
         row_age_ms = (time.time() - mtime) * 1000.0
         if row_age_ms > args.state_timeout_ms:
-            raise StateError(f"STALE_STATE: latest row age {row_age_ms:.1f} ms")
+            raise StateError(
+                f"STALE_STATE: latest row age {row_age_ms:.1f} ms "
+                f"({describe_state_freshness_rejection(st, row, row_age_ms)})"
+            )
         state = mapper.state_from_csv(row, signature, time.monotonic() - (row_age_ms / 1000.0))
@@ -2134,8 +2204,15 @@ def _raise_keyboard_interrupt_on_sigterm(signum, frame) -> None:
     raise KeyboardInterrupt()
 
 
+_SIGTERM_BLOCK_SET = {signal.SIGTERM}
+
+
 def print_run_summary(
-    request_count: int,
+    total_requests: int,
     replies_sent_count: int,
     ...
 ) -> None:
     print(
         "RUN_SUMMARY "
-        f"total_requests={request_count} "
+        f"total_requests={total_requests} "
         ...
-        f"missed_replies={request_count - replies_sent_count}"
+        f"missed_replies={total_requests - replies_sent_count}"
     )
 
 
@@ -2309,6 +2393,10 @@ def main() -> int:
     request_count = 0
+    logged_request_count = 0
     replies_sent_count = 0
     ...
@@ -2336,6 +2424,30 @@ def main() -> int:
     print(f"Listening on {args.listen_host}:{args.listen_port}")
     print(f"State source: {'mock-state' if args.mock_state else args.state_file}")
 
+    def _log_row_and_count(row: Dict[str, object], sent: bool) -> None:
+        nonlocal logged_request_count, replies_sent_count
+        signal.pthread_sigmask(signal.SIG_BLOCK, _SIGTERM_BLOCK_SET)
+        try:
+            write_log(log_writer, csv_file, row)
+            logged_request_count += 1
+            if sent:
+                replies_sent_count += 1
+        finally:
+            signal.pthread_sigmask(signal.SIG_UNBLOCK, _SIGTERM_BLOCK_SET)
+
     try:
         while True:
             started = time.monotonic()
@@ -2432,9 +2544,7 @@ def main() -> int:
                     malformed_command_source = B3CommandSource.PRECONTROL_REFERENCE
-                write_log(
-                    log_writer,
-                    csv_file,
+                _log_row_and_count(
                     make_log_row(...),
+                    sent=False,
                 )
                 if args.once:
                     return 1
                 continue
@@ -2728,9 +2839,7 @@ def main() -> int:
                         print("B3_TIME_DISCONTINUITY_ABORT neutral_jsbsim_command_sent=0 error={command_exc}")
-                write_log(
-                    log_writer,
-                    csv_file,
+                _log_row_and_count(
                     make_log_row(...),
+                    sent=False,
                 )
                 return 4
@@ -2771,9 +2881,7 @@ def main() -> int:
                         print("B3_STATE_ENVELOPE_ABORT neutral_jsbsim_command_sent=0 error={command_exc}")
-                write_log(
-                    log_writer,
-                    csv_file,
+                _log_row_and_count(
                     make_log_row(...),
+                    sent=False,
                 )
                 return 3
             except StateError as exc:
@@ -2820,9 +2929,7 @@ def main() -> int:
                 if actuator_reason:
                     reason = f"{reason};{actuator_reason}"
-                write_log(
-                    log_writer,
-                    csv_file,
+                _log_row_and_count(
                     make_log_row(...),
+                    sent=False,
                 )
                 if args.once:
                     return 1
                 continue
@@ -2863,7 +2971,12 @@ def main() -> int:
                 reply_bytes = sock.sendto(payload, source)
-                replies_sent_count += 1
+                # (moved into _log_row_and_count() below, see comment there)
                 last_reply_mono = time.monotonic()
@@ -2883,9 +2996,7 @@ def main() -> int:
                     )
 
-            write_log(
-                log_writer,
-                csv_file,
+            _log_row_and_count(
                 make_log_row(...),
+                sent=not args.dry_run,
             )
             if args.once:
                 return 0
@@ -2917,7 +3029,7 @@ def main() -> int:
         return 0
     finally:
         print_run_summary(
-            request_count=request_count,
+            total_requests=logged_request_count,
             replies_sent_count=replies_sent_count,
             ...
```

### `Tools/autotest/sr75_hil_layer2/sim_json/test_sr75_hil_f24p_state_csv_race.py`

Mechanical fix for `read_latest()`'s new 4-tuple return (the only other
caller in the codebase besides `read_reply_state()`, already updated
above):

```diff
-        row, _, _ = reader.read_latest()
+        row, _, _, _ = reader.read_latest()
```

Plus one new file: `test_sr75_hil_f24s_coherence_and_accounting.py`.

## Request/CSV/reply accounting invariants (task 8)

Two invariants now hold **unconditionally**, including under SIGTERM at
an arbitrary point:

1. `total_requests == count(responder.csv data rows)` -- every counted
   request has exactly one CSV row, by construction (task 5).
2. `replies_sent + missed_replies == total_requests`, where
   `missed_replies = total_requests - replies_sent` -- both terms on the
   left are incremented together with `total_requests` inside the same
   atomic `_log_row_and_count()` call, so they can never drift apart.

## Test results

New file `test_sr75_hil_f24s_coherence_and_accounting.py`, 7 tests, all
using the real reader/feeder/responder code (no mocked filesystem):

```
$ python3 -m unittest test_sr75_hil_f24s_coherence_and_accounting -v
test_read_latest_opens_the_file_exactly_once ... ok
test_source_never_calls_pathname_stat ... ok
test_returned_stat_matches_the_content_just_read ... ok
test_header_cache_keyed_by_dev_ino_mtime_not_just_mtime ... ok
test_concurrent_reads_never_report_an_inflated_age ... ok
test_burst_then_immediate_sigterm_never_mismatches ... ok
test_short_run_zero_stale_zero_malformed_exact_accounting ... ok
Ran 7 tests in ~10s -- OK
```

- `test_source_never_calls_pathname_stat` -- walks the real parsed AST of
  `LatestCSVReader` (not a substring search, which would false-positive
  on the class's own docstring discussing the old bug) and asserts no
  `os.stat`/`os.path.exists` call nodes remain, and `os.fstat` is present.
- `test_read_latest_opens_the_file_exactly_once` -- wraps `open()` with a
  counting shim and asserts exactly one call per `read_latest()`
  invocation -- the structural property the whole fix rests on.
- `test_returned_stat_matches_the_content_just_read` -- confirms the
  `os.stat_result` returned by `read_latest()` describes the same
  (device, inode) as an independent `os.stat()` taken immediately after.
- `test_header_cache_keyed_by_dev_ino_mtime_not_just_mtime` -- confirms
  the cache key is the 3-tuple `(dev, ino, mtime_ns)` (task 3).
- `test_concurrent_reads_never_report_an_inflated_age` -- a real writer
  thread replacing the file at 100 Hz (faster than the ~50 Hz hardware
  feeder, for extra stress) racing 4 reader threads tight-looping
  `read_latest()` for 2.5s; asserts zero malformed reads and that no
  observed age ever exceeds 500 ms (a TOCTOU mismatch under a ~10ms
  writer cadence would show up as a wild, isolated outlier, exactly like
  request 784's 1546.5ms).
- `test_burst_then_immediate_sigterm_never_mismatches` -- 15 trials,
  each firing a burst of 10 requests as fast as possible and terminating
  the responder immediately after, asserting `total_requests ==
  csv_data_rows` and the reply-accounting invariant every time (0/15
  mismatches).
- `test_short_run_zero_stale_zero_malformed_exact_accounting` -- full
  end-to-end: real feeder continuously atomically replacing `state.csv`,
  real responder answering a simulated ~50 Hz request loop for 4s; zero
  `stale_state_count`, zero `malformed_state_count`, exact CSV/counter
  accounting, valid final `state.csv`.

**Regression check**, all pre-existing suites unaffected:

```
sim_json:  test_sr75_hil_f24p_state_csv_race (6/6), test_sr75_hil_f24d_static_state_feed (5/5),
           test_sr75_hil_f24q_shutdown_and_timing (8/8)          -- 19/19 (+7 new = 26/26)
scripts:   test_sr75_hil_f24f_hardware_orchestrator (15/15),
           test_sr75_hil_f24b_sim_json_no_hardware_tests (23/23),
           test_sr75_hil_f24g_jsbsim_pipeline_validation (30/30) -- 68/68
jsbsim_control.test_sr75_sim_json_command_accounting: same 2 pre-existing,
  unrelated failures reconfirmed (actuator/JSBSim-command-accounting code
  this task does not touch; first identified as pre-existing in the
  HIL-F24-P report).
```

### `py_compile` / `flake8` / `git diff --check`

```
$ python3 -m py_compile sr75_sim_json_responder.py test_sr75_hil_f24p_state_csv_race.py test_sr75_hil_f24s_coherence_and_accounting.py
(clean)

$ python3 -m flake8 --max-line-length=200 <same 3 files>
sr75_sim_json_responder.py:1382/1384/1386/1388:26: E127 continuation line over-indented for visual indent
```

All 4 findings confirmed pre-existing (identical content, shifted line
numbers only, already noted in the HIL-F24-P/Q reports). The new test
file has zero flake8 findings.

```
$ git diff --check -- <same files>
(clean, no whitespace errors)
```

## 30-second subprocess stress test (task 7)

Real feeder + real responder subprocesses, feeder at 50 Hz, simulated
request loop at ~50 Hz (matching the hardware run's actual cadence),
atomic replacement continuously active for the full 30s:

```
RUN_SUMMARY total_requests=1458 replies_sent=1458 stale_state_count=0
             no_fresh_state_count=0 malformed_state_count=0
             state_read_error_count=0 missed_replies=0
WRITE_SUMMARY write_count=1517 fsync_enabled=0 max_write_latency_ms=0.759
              p95_write_latency_ms=0.326 p99_write_latency_ms=0.377
              total_fsync_time_ms=0.000 longest_scheduling_gap_ms=20.280
final state.csv line count: 2 (header + 1 row)
responder.csv data rows: 1458  (== total_requests)
replies_sent + missed_replies == total_requests: True
timeouts (no reply at all): 0
```

Zero stale/no-fresh/malformed states, zero missed replies, exact
CSV/counter accounting under continuous atomic replacement at the
hardware's actual request rate.

## Expected real-hardware acceptance criteria

On the next bench run with this fix:

- **Zero isolated STALE_STATE outliers** surrounded by otherwise-fresh
  neighbors -- if one still occurs, `describe_state_freshness_rejection()`'s
  `dev`/`ino`/`mtime_ns`/`row_source_timestamp` fields will immediately
  show whether it's a genuine feeder stall (large
  `longest_scheduling_gap_ms` in the feeder's own `WRITE_SUMMARY` for
  that same window) or something new, without needing to reconstruct
  timing from raw CSV rows after the fact.
- **`responder.csv`'s data-row count must equal `RUN_SUMMARY`'s
  `total_requests`** on every run, including ones stopped by the
  orchestrator's SIGTERM -- no more "one fewer data record than
  total_requests."
- **`replies_sent + missed_replies == total_requests`** must hold
  exactly, every run.
- **No change to SIM_JSON UDP wire format, PPP behavior, actuator
  output, or RATO/TECS/mission logic** -- confirmed by this task
  touching only `LatestCSVReader`/`read_reply_state`/`print_run_summary`/
  the five `write_log` call sites' wrapper, none of which are on the
  actuator/control path.

## Files and line references

- `Tools/autotest/sr75_hil_layer2/sim_json/sr75_sim_json_responder.py` --
  `LatestCSVReader` (original line 363: class + `__init__`/`_load_headers`/
  `read_latest`), `describe_state_freshness_rejection()` (new, near
  original line 2054), `read_reply_state()`'s `STALE_STATE` raise
  (original line ~2111), `_SIGTERM_BLOCK_SET`/`print_run_summary()`
  (original line ~2207), counter init near `request_count = 0` (original
  line 2393), `_log_row_and_count()` closure and the five call-site
  replacements (original lines 2435/2731/2774/2823/2886, per the
  HIL-F24-P/Q reports' own line references), the `finally:` block's
  `print_run_summary()` call (original line ~3031).
- `Tools/autotest/sr75_hil_layer2/sim_json/test_sr75_hil_f24p_state_csv_race.py`
  -- one-line unpacking fix for `read_latest()`'s new 4-tuple return.
- New: `Tools/autotest/sr75_hil_layer2/sim_json/test_sr75_hil_f24s_coherence_and_accounting.py`.

Note: `Tools/autotest/sr75_hil_layer2/hardware_sessions/
sr75_hil_f24f_static_20260805T130807Z/responder.csv` (pre-existing,
untouched by this task) was checked as a possible source of direct
evidence for request 784, but it is the earlier 1499-request HIL-F24-Q
capture, not the 1588-request run this task describes -- see the
"Evidence note" above.
