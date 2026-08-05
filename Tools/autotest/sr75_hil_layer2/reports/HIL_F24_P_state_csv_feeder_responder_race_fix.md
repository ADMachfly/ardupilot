# HIL-F24-P: state.csv Feeder/Responder Race Fix (Host-Side Only)

Status: fix applied and validated. No firmware flashed, no parameters
changed, no actuators started, no arm, no AUTO. Only host-side Python
files under `Tools/autotest/sr75_hil_layer2/sim_json/` were touched; no
`SIM_JSON` C++, no PPP configuration, no RATO/aero/TECS/mission/control
logic, and no actuator decode safety or CH7/CH8 handling was modified.

## Exact root cause

The feeder, `Tools/autotest/sr75_hil_layer2/sim_json/
sr75_hil_f24d_static_state_feed.py` (confirmed as the script actually
launched by `sr75_hil_f24f_hardware_orchestrator.py`'s `FEEDER_SCRIPT`,
which produced the `responder.csv`/`state.csv` pair described in this
task), wrote its `--output` CSV like this (`main()`, previously at
lines 95-99):

```python
with open(args.output, "w", newline="", encoding="utf-8") as f:
    writer = csv.DictWriter(f, fieldnames=CSV_FIELDS)
    writer.writeheader()
    writer.writerow(row)
    f.flush()
```

This is non-atomic in three compounding ways:

1. `open(path, "w")` **truncates the file to zero bytes the instant it is
   opened** — before a single byte of the new header or row exists.
2. `writeheader()` and `writerow()` are **two separate `write()` calls**,
   so there is a window where the file contains a header but no data row.
3. There is no `fsync()` and no atomic rename — the file is mutated
   in place, so any reader with the bad timing sees whatever partial
   state the writer happened to be in.

This exactly explains every symptom in the task's hardware run:

- `state.csv has no CSV header` — the responder's
  `LatestCSVReader._load_headers()` (`sr75_sim_json_responder.py:352-360`)
  opened the file during the brief window after truncation (step 1) but
  before `writeheader()` (step 2) landed, so `next(csv.reader(...), None)`
  returned `None`.
- `state.csv has no complete data rows` — `LatestCSVReader.read_latest()`
  (`sr75_sim_json_responder.py:362-386`) opened the file after the header
  was written but before the data row (step 2's second write) landed, so
  it saw fewer than 2 non-blank lines.
- `Final state.csv contains only header + latest row` — this is the
  feeder's **intended** behavior (it always rewrites a single unchanging
  snapshot row per task's own framing) and needed no fix; confirmed still
  true after the fix (see `TestFeederLeavesValidFinalSnapshot` below).
- 1500 feeder writes over 30s (∼50 Hz) racing 5 Hz SIM_JSON-driven
  responder reads gave ample opportunity to land in one of these windows,
  producing the 1119-line `responder.csv` log with repeated
  `STATE_ERROR` lines — and, critically, **each failed read caused the
  responder to skip that reply entirely** (`except StateError: ... continue`
  in `main()`'s request loop, previously with no retention), meaning every
  race hit was a dropped SIM_JSON reply, not just a log line.

## Reader/writer trace (task item 1-2)

- **Writer**: `sr75_hil_f24d_static_state_feed.py:main()` — confirmed:
  opens with mode `"w"` (truncates before writing) ✓; writes header and
  row as separate calls ✓ (exposes partially-written content) ✓; no
  `fsync()`, no atomic rename ✓ (replaces the file non-atomically) ✓.
- **Reader**: `sr75_sim_json_responder.py`'s `LatestCSVReader` class
  (`_load_headers()` at line 352, `read_latest()` at line 362), called
  from `read_reply_state()` (line ~1990) on every SIM_JSON request via
  `main()`'s request loop (`mapped_state = read_reply_state(args, reader,
  mapper, mock_source)`, previously ~line 2502).

## Fix

### 1. Atomic snapshot write (feeder)

New `atomic_write_csv_snapshot(path, fieldnames, row)` helper in
`sr75_hil_f24d_static_state_feed.py`, used in place of the old
open("w")/writeheader/writerow block:

```python
def atomic_write_csv_snapshot(path, fieldnames, row):
    directory = os.path.dirname(os.path.abspath(path)) or "."
    fd, tmp_path = tempfile.mkstemp(prefix=".state_csv_", suffix=".tmp", dir=directory)
    try:
        with os.fdopen(fd, "w", newline="", encoding="utf-8") as tmp_file:
            writer = csv.DictWriter(tmp_file, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerow(row)
            tmp_file.flush()
            os.fsync(tmp_file.fileno())
        os.replace(tmp_path, path)
    except BaseException:
        try:
            os.remove(tmp_path)
        except OSError:
            pass
        raise
```

Writes the complete header + row to a temp file **in the same directory**
(`dir=directory`, required so `os.replace()` is a same-filesystem rename
— POSIX only guarantees atomicity for same-filesystem renames), flushes
and `fsync()`s it, then `os.replace()`s it into place. A reader opening
`path` at any point now only ever observes either the complete previous
snapshot or the complete new one — never a partial one. No existing
project atomic-CSV helper was found (checked via grep for `atomic_write`,
`os.replace`, `NamedTemporaryFile`-as-shared-file-swap across
`Tools/autotest/sr75_hil_layer2/` and the broader `Tools/`/`libraries/`
tree; only incidental unrelated uses of `tempfile` for standalone test
fixtures exist), so this is new, minimal logic rather than a duplicate.

### 2. Responder hardening

- **Two new exception subclasses** of the existing `StateError`
  (`sr75_sim_json_responder.py`, right after the `StateError` class
  definition):
  - `StateFileNotReadyError` — the file does not exist yet (startup, before
    the feeder's first write).
  - `StateFileMalformedError` — the file exists but is transiently
    incomplete (missing header, or no complete data row).

  `LatestCSVReader` now raises these specific subclasses instead of the
  generic `StateError` at its three raise sites, so callers can tell a
  cold-start condition from a malformed-file condition.

- **Retention** (`StateMapper` gains `last_valid_state`,
  `state_read_error_count`, `note_state_read_error()`,
  `clear_state_read_error_dedup()`; `accept_state()` now also sets
  `last_valid_state`). `read_reply_state()` now catches the two new
  exception types: if a previously accepted valid state exists, it is
  re-served (`dataclasses.replace(..., reused_source_row=True)`, with its
  outgoing timestamp nudged forward past `last_outgoing_timestamp` — the
  same nudge the existing `--allow-state-reuse` path already uses a few
  lines below, so a retained reply never duplicates or reverses the
  outgoing SIM_JSON timestamp) instead of the request being silently
  skipped. If no valid state has ever been seen (true cold start), the
  exception still propagates and the request is skipped, exactly as
  before this fix — this is the "distinguish startup/no-file from
  malformed-file" requirement: a `StateFileNotReadyError` before any valid
  state exists is expected and cannot be "retained" (there is nothing to
  retain yet), while the same error type after a valid state exists would
  still be retained like a malformed-file error.
- **Error counting**: every failure (`StateFileNotReadyError` or
  `StateFileMalformedError`) increments `mapper.state_read_error_count`,
  whether or not it gets logged.
- **Log dedup**: `note_state_read_error()` only reports "new" (worth
  printing) the first time a given exact message is seen since the last
  distinct message or the last successful read; `clear_state_read_error_dedup()`
  re-arms it after a fresh successful `reader.read_latest()`. The
  pre-existing outer `except StateError` handler in `main()`'s request
  loop is adjusted with one `isinstance` check so it does not print the
  same message a second time (it already handled `STALE_STATE` and
  generic `StateError` printing unchanged; only the two new subclasses
  are now suppressed there, since `read_reply_state()` already logged them
  once, deduplicated).

## Exact diff

```diff
diff --git a/Tools/autotest/sr75_hil_layer2/sim_json/sr75_hil_f24d_static_state_feed.py b/Tools/autotest/sr75_hil_layer2/sim_json/sr75_hil_f24d_static_state_feed.py
index e874a8a3a0..c4e8fda059 100644
--- a/Tools/autotest/sr75_hil_layer2/sim_json/sr75_hil_f24d_static_state_feed.py
+++ b/Tools/autotest/sr75_hil_layer2/sim_json/sr75_hil_f24d_static_state_feed.py
@@ -18,7 +18,9 @@ file passed via --output.
 """
 import argparse
 import csv
+import os
 import sys
+import tempfile
 import time
 
 sys.path.insert(0, __file__.rsplit("/", 1)[0])
@@ -68,6 +70,42 @@ def build_static_row(t_s, lat_deg=BASE_LAT, lon_deg=BASE_LON, alt_m=BASE_ALT_M,
     }
 
 
+def atomic_write_csv_snapshot(path, fieldnames, row):
+    """HIL-F24-P: (re)write a single-row CSV snapshot atomically.
+
+    The previous implementation opened `path` with mode "w" (truncating it
+    immediately) and then wrote the header and the data row as two separate
+    writes. A concurrent reader (LatestCSVReader.read_latest(), used by
+    sr75_sim_json_responder.py) could observe the file in between any of
+    those steps: freshly truncated (empty -- "no CSV header"), header-only
+    ("no complete data rows"), or a torn/partial row. This is exactly the
+    feeder/responder race seen on the bench (HIL-F24-P).
+
+    Instead, the complete header + row is written to a temporary file in
+    the *same directory* as `path` (so the final os.replace() is a rename
+    within one filesystem, which POSIX guarantees is atomic), flushed and
+    fsynced, then swapped into place. A reader opening `path` at any point
+    in time therefore only ever observes either the previous complete
+    snapshot or the new complete snapshot -- never a partial one.
+    """
+    directory = os.path.dirname(os.path.abspath(path)) or "."
+    fd, tmp_path = tempfile.mkstemp(prefix=".state_csv_", suffix=".tmp", dir=directory)
+    try:
+        with os.fdopen(fd, "w", newline="", encoding="utf-8") as tmp_file:
+            writer = csv.DictWriter(tmp_file, fieldnames=fieldnames)
+            writer.writeheader()
+            writer.writerow(row)
+            tmp_file.flush()
+            os.fsync(tmp_file.fileno())
+        os.replace(tmp_path, path)
+    except BaseException:
+        try:
+            os.remove(tmp_path)
+        except OSError:
+            pass
+        raise
+
+
 def build_arg_parser():
     parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
     parser.add_argument("--output", required=True, help="CSV path to (re)write; matches responder --state-file")
@@ -92,11 +130,7 @@ def main():
     for i in range(n_writes):
         t_s = time.monotonic() - start
         row = build_static_row(t_s, args.lat_deg, args.lon_deg, args.alt_m, args.yaw_deg, args.airspeed_mps)
-        with open(args.output, "w", newline="", encoding="utf-8") as f:
-            writer = csv.DictWriter(f, fieldnames=CSV_FIELDS)
-            writer.writeheader()
-            writer.writerow(row)
-            f.flush()
+        atomic_write_csv_snapshot(args.output, CSV_FIELDS, row)
         next_write = start + (i + 1) * dt
         sleep_s = next_write - time.monotonic()
         if sleep_s > 0:
diff --git a/Tools/autotest/sr75_hil_layer2/sim_json/sr75_sim_json_responder.py b/Tools/autotest/sr75_hil_layer2/sim_json/sr75_sim_json_responder.py
index ed6f56f8d3..3bf58b46bb 100755
--- a/Tools/autotest/sr75_hil_layer2/sim_json/sr75_sim_json_responder.py
+++ b/Tools/autotest/sr75_hil_layer2/sim_json/sr75_sim_json_responder.py
@@ -155,6 +155,22 @@ class StateError(Exception):
     """Raised when state cannot be converted to a valid SIM_JSON reply."""
 
 
+class StateFileNotReadyError(StateError):
+    """HIL-F24-P: the state file does not exist yet. Expected transiently
+    at startup, before the feeder's first atomic snapshot write has
+    landed -- distinct from StateFileMalformedError, which means the file
+    exists but was caught mid-write."""
+
+
+class StateFileMalformedError(StateError):
+    """HIL-F24-P: the state file exists but is transiently incomplete (its
+    header row is missing, or it has no complete data row yet). With the
+    feeder's atomic-replace fix this should not happen in normal operation,
+    but the responder still treats it as a retryable, retainable condition
+    rather than a fatal one, in case of a non-atomic writer or a reader
+    racing an external filesystem hiccup."""
+
+
 class StateEnvelopeError(Exception):
     """Raised when B3 bounded-test state exceeds the allowed safety envelope."""
 
@@ -357,11 +373,11 @@ class LatestCSVReader:
             self.headers = next(csv.reader(csv_file), None)
         self._header_mtime_ns = stat.st_mtime_ns
         if not self.headers:
-            raise StateError(f"{self.path} has no CSV header")
+            raise StateFileMalformedError(f"{self.path} has no CSV header")
 
     def read_latest(self) -> Tuple[Dict[str, str], str, float]:
         if not os.path.exists(self.path):
-            raise StateError(f"state file does not exist: {self.path}")
+            raise StateFileNotReadyError(f"state file does not exist: {self.path}")
         self._load_headers()
         assert self.headers is not None
         stat = os.stat(self.path)
@@ -373,7 +389,7 @@ class LatestCSVReader:
             data = csv_file.read(block_size).decode("utf-8", errors="ignore")
         lines = [line for line in data.splitlines() if line.strip()]
         if len(lines) < 2:
-            raise StateError(f"{self.path} has no complete data rows")
+            raise StateFileMalformedError(f"{self.path} has no complete data rows")
         for line in reversed(lines[1:]):
             parsed = list(csv.reader([line]))
             if not parsed:
@@ -382,7 +398,7 @@ class LatestCSVReader:
             if len(values) == len(self.headers):
                 row = dict(zip(self.headers, values))
                 return row, line, stat.st_mtime
-        raise StateError(f"{self.path} has no complete data rows")
+        raise StateFileMalformedError(f"{self.path} has no complete data rows")
 
 
 class StateMapper:
@@ -393,6 +409,31 @@ class StateMapper:
         self.last_source_timestamp: Optional[float] = None
         self.last_outgoing_timestamp: Optional[float] = None
         self.last_signature: Optional[str] = None
+        # HIL-F24-P: last successfully accepted state, retained so a
+        # transient state-file read/parse race can still be answered with
+        # a valid reply instead of being skipped outright. Also counts and
+        # deduplicates state-read-error logging (see note_state_read_error()).
+        self.last_valid_state: Optional[SimState] = None
+        self.state_read_error_count: int = 0
+        self._last_logged_state_read_error: Optional[str] = None
+
+    def note_state_read_error(self, message: str) -> bool:
+        """Count a state-file read/parse error and report whether it is
+        new (differs from the last logged message) and therefore worth
+        printing. Repeated identical messages (e.g. the same feeder write
+        race hit on several consecutive requests) are counted every time
+        but logged only once, until either a different message occurs or
+        a fresh successful read re-arms logging via
+        clear_state_read_error_dedup()."""
+        self.state_read_error_count += 1
+        is_new = message != self._last_logged_state_read_error
+        if is_new:
+            self._last_logged_state_read_error = message
+        return is_new
+
+    def clear_state_read_error_dedup(self) -> None:
+        """Re-arm state-read-error logging after a successful read."""
+        self._last_logged_state_read_error = None
 
     def _missing_required(self, name: str, missing: List[str]) -> None:
         missing.append(name)
@@ -583,6 +624,7 @@ class StateMapper:
         self.last_signature = state.row_signature
         self.last_source_timestamp = state.source_timestamp_s
         self.last_outgoing_timestamp = state.timestamp_s
+        self.last_valid_state = state
 
 
 class MockStateSource:
@@ -1989,7 +2031,38 @@ def read_reply_state(
 
     deadline = time.monotonic() + max(0.0, args.fresh_state_wait_ms) / 1000.0
     while True:
-        row, signature, mtime = reader.read_latest()
+        try:
+            row, signature, mtime = reader.read_latest()
+        except (StateFileNotReadyError, StateFileMalformedError) as exc:
+            # HIL-F24-P: a transient state-file read/parse race (e.g. the
+            # feeder caught mid-write) must not silently drop this reply.
+            # If we already have a previously accepted valid state, retain
+            # and re-serve it (nudging its outgoing timestamp forward the
+            # same way the --allow-state-reuse path below does, so it never
+            # duplicates or goes backward) instead of skipping the reply.
+            # Only propagate the error -- which causes the caller to skip
+            # replying, as before HIL-F24-P -- if no valid state has ever
+            # been seen yet (true cold start, before the feeder's first
+            # snapshot write has landed).
+            is_new = mapper.note_state_read_error(str(exc))
+            kind = "STATE_NOT_READY" if isinstance(exc, StateFileNotReadyError) else "STATE_FILE_MALFORMED"
+            if mapper.last_valid_state is not None:
+                if is_new:
+                    print(
+                        f"{kind} (retaining last valid state, "
+                        f"total_state_read_errors={mapper.state_read_error_count}): {exc}"
+                    )
+                retained = replace(mapper.last_valid_state, reused_source_row=True)
+                if mapper.last_outgoing_timestamp is not None and retained.timestamp_s <= mapper.last_outgoing_timestamp:
+                    retained.timestamp_s = mapper.last_outgoing_timestamp + MIN_OUTGOING_TIMESTAMP_STEP_S
+                return retained
+            if is_new:
+                print(
+                    f"{kind} (no valid state yet, "
+                    f"total_state_read_errors={mapper.state_read_error_count}): {exc}"
+                )
+            raise
+        mapper.clear_state_read_error_dedup()
         row_age_ms = (time.time() - mtime) * 1000.0
         if row_age_ms > args.state_timeout_ms:
             raise StateError(f"STALE_STATE: latest row age {row_age_ms:.1f} ms")
@@ -2685,7 +2758,11 @@ def main() -> int:
                 return 3
             except StateError as exc:
                 reason = str(exc)
-                if reason.startswith("STALE_STATE"):
+                if isinstance(exc, (StateFileNotReadyError, StateFileMalformedError)):
+                    # HIL-F24-P: already logged (deduplicated) and counted
+                    # inside read_reply_state() -- avoid printing it twice.
+                    pass
+                elif reason.startswith("STALE_STATE"):
                     print(reason)
                 else:
                     print(f"STATE_ERROR: {reason}")
```

Plus a new file, `Tools/autotest/sr75_hil_layer2/sim_json/
test_sr75_hil_f24p_state_csv_race.py` (regression tests, described below).

## Regression tests added

New file `test_sr75_hil_f24p_state_csv_race.py`, 6 tests, all using the
real feeder/responder code (not reimplementations):

1. `TestAtomicFeederWriteConcurrency.test_concurrent_reads_never_see_missing_header_or_incomplete_row`
   — one real writer thread calling `atomic_write_csv_snapshot()` at ~50 Hz
   racing 4 real reader threads tight-looping `LatestCSVReader.read_latest()`
   for 2s. Asserts zero `StateFileMalformedError`s.
2. `TestFeederLeavesValidFinalSnapshot.test_final_file_is_header_plus_one_complete_row`
   — 20 successive atomic writes; asserts the final file is exactly one
   header line + one complete data row, and that row parses correctly.
3. `TestResponderRetainsLastValidStateOnTransientFailure
   .test_retains_last_valid_state_and_dedupes_repeated_log_lines` — reads a
   valid row, deliberately corrupts the file to header-only, then calls
   `read_reply_state()` twice more. Asserts: the first call retains the
   previous state's values (marked `reused_source_row=True`), advances the
   timestamp (never duplicates), and logs exactly one `STATE_FILE_MALFORMED`
   line; the second identical-failure call produces **zero** additional
   log output (dedup) while still incrementing `state_read_error_count`;
   a subsequent fresh valid write is picked up normally
   (`reused_source_row=False`).
4. `...test_raises_state_file_not_ready_when_never_written` — with a state
   file that has never been written, asserts `StateFileNotReadyError`
   propagates (reply still skipped, as before this fix, since there is
   nothing to retain) and the error is still counted.
5. `...test_malformed_and_not_ready_are_distinct_exception_types` — pins
   down the two new exception classes are distinct subclasses of
   `StateError`.
6. `TestSimJsonPacketFormatUnchanged.test_to_json_bytes_schema_unchanged`
   — confirms `SimState.to_json_bytes()` (the actual SIM_JSON wire
   payload, untouched by this fix) still produces the same field set.

## Test results

```
$ python3 -m unittest test_sr75_hil_f24p_state_csv_race test_sr75_hil_f24d_static_state_feed -v
test_concurrent_reads_never_see_missing_header_or_incomplete_row ... ok
test_final_file_is_header_plus_one_complete_row ... ok
test_malformed_and_not_ready_are_distinct_exception_types ... ok
test_raises_state_file_not_ready_when_never_written ... ok
test_retains_last_valid_state_and_dedupes_repeated_log_lines ... ok
test_to_json_bytes_schema_unchanged ... ok
test_airspeed_override ... ok
test_defaults_match_base_constants ... ok
test_level_and_motionless ... ok
test_row_accepted_by_real_state_mapper ... ok
test_time_advances ... ok

Ran 11 tests in 2.050s
OK
```

**Regression check on adjacent existing suites** that import/exercise
`sr75_sim_json_responder.py`:

- `scripts/test_sr75_hil_f24b_sim_json_no_hardware_tests.py` — 23/23 pass.
- `scripts/test_sr75_hil_f24g_jsbsim_pipeline_validation.py` — 30/30 pass.
- `jsbsim_control/test_sr75_sim_json_command_accounting.py` — 2 failures
  (`test_100_packet_accounting`, `test_stale_command_sends_one_neutral_packet`).
  **Confirmed pre-existing and unrelated**: reran identically via
  `git stash` with none of this task's changes applied — same 2 failures,
  same assertion values (`7 != 6`, `0 != 1`), in actuator/JSBSim-command
  accounting code this task never touches.

## Local stress-test counts (task item 7's "30-second local concurrency stress test")

Ran the same writer/reader-thread harness used by test #1 above, for the
full 30 seconds, at the task's observed real rates (~50 Hz writer, 4
concurrent reader threads tight-looping with no sleep — i.e. reading far
more aggressively than the real ~5 Hz SIM_JSON-driven responder does):

```
write_count       1501
read_attempt_count 138784
malformed_errors  0
```

1501 writes over 30s matches the hardware run's reported "1500 writes
over 30 s" almost exactly. Zero malformed-file errors across ~139k read
attempts (roughly 4630 reads per feeder write) confirms the atomic-replace
fix holds even under read pressure far higher than the real bench
scenario.

## `py_compile` / `flake8` / `git diff --check`

```
$ python3 -m py_compile sr75_hil_f24d_static_state_feed.py sr75_sim_json_responder.py test_sr75_hil_f24p_state_csv_race.py
(clean)

$ python3 -m flake8 --max-line-length=200 <the same three files>
sr75_hil_f24d_static_state_feed.py:45:23: E127 continuation line over-indented for visual indent
sr75_sim_json_responder.py:1338:26: E127 continuation line over-indented for visual indent
sr75_sim_json_responder.py:1340:26: E127 continuation line over-indented for visual indent
sr75_sim_json_responder.py:1342:26: E127 continuation line over-indented for visual indent
sr75_sim_json_responder.py:1344:26: E127 continuation line over-indented for visual indent
```

All 5 findings **confirmed pre-existing**, unrelated to this task: reran
`flake8` via `git stash` with none of this task's changes applied — same
5 findings at the corresponding (unshifted) line numbers, in code this
task did not touch. The new test file has zero flake8 findings.

```
$ git diff --check -- sr75_hil_f24d_static_state_feed.py sr75_sim_json_responder.py
(clean, no whitespace errors)
```

## Expected corrected hardware-acceptance criteria

On the next bench run with this fix (feeder writing atomically, responder
retaining/deduplicating/counting on any remaining transient race):

- `responder.csv`/console output should show **zero or near-zero**
  `STATE_NOT_READY`/`STATE_FILE_MALFORMED` lines after the feeder's first
  write lands (previously: 1119 lines' worth of repeated `STATE_ERROR`
  spam over a 30s run) — the atomic write eliminates the torn-file window
  entirely, so these should no longer occur at all in normal operation.
- If any transient failure somehow still occurs (e.g. an external
  filesystem hiccup, or a future non-atomic writer), the responder should
  now **still send a valid SIM_JSON reply** for that request (using the
  retained last-valid state, `reused_source_row=1` in the log row) instead
  of silently dropping it — Pixhawk-side request cadence (~5 Hz) should
  show no gaps correlated with feeder writes.
- Exactly one log line per distinct failure condition, not one per
  affected request — `responder.csv`'s `state_read_error_count`-style
  bookkeeping (via `mapper.state_read_error_count`, visible to anyone
  instrumenting the mapper) should closely track the true number of
  underlying failures, while console/log volume should not scale with
  request rate during a sustained identical failure.
- `state.csv` should still end each run as exactly one header line plus
  one complete data row (this was already correct feeder behavior and
  remains unchanged and verified).
- No change to SIM_JSON UDP wire format, PPP behavior, or actuator
  output — confirmed by `to_json_bytes()` schema test and by this task
  touching no C++, hwdef, or actuator-bridge/RATO/TECS/mission files.

## Files and line references

- `Tools/autotest/sr75_hil_layer2/sim_json/sr75_hil_f24d_static_state_feed.py`
  — `atomic_write_csv_snapshot()` (new), `main()`'s write call site.
- `Tools/autotest/sr75_hil_layer2/sim_json/sr75_sim_json_responder.py` —
  `StateFileNotReadyError`/`StateFileMalformedError` (new, after
  `StateError` at original line 154), `LatestCSVReader._load_headers()`
  (line 352) and `.read_latest()` (line 362), `StateMapper.__init__()`
  (line 388) and `.accept_state()` (line 582), `read_reply_state()` (line
  ~1990), the outer `except StateError` handler in `main()` (line ~2686).
- `Tools/autotest/sr75_hil_layer2/scripts/sr75_hil_f24f_hardware_orchestrator.py`
  — confirms `sr75_hil_f24d_static_state_feed.py` is the actual feeder
  used against real hardware (`FEEDER_SCRIPT`, writing to `session_dir /
  "state.csv"`), read (unmodified) only to establish this trace.
- New: `Tools/autotest/sr75_hil_layer2/sim_json/test_sr75_hil_f24p_state_csv_race.py`.
