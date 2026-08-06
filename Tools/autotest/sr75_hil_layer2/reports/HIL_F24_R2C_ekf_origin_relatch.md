# HIL-F24-R2C: Guarded EKF-Origin Relatch Workflow

Builds on `HIL_F24_R2B_ahrs_backend_diagnosis.md` (root cause: EKF3's
origin, `GPS_GLOBAL_ORIGIN`, is one-shot latched to ArduPilot's compiled-in
default SITL home at boot, before this bench's SIM_JSON link has ever
delivered a real position; PPP alone cannot fix this because the latch
already happened before PPP/the feeder connect). This task adds a new,
guarded orchestrator stage that fixes the *operational* problem: get a
correct origin latched, automatically verify it before committing to a
capture, and abort cleanly if it isn't.

Host-side only. No firmware/PPP/JSBSim/control code was modified, no
parameter was set, nothing was flashed, armed, or run in AUTO, no output
was enabled. Safety invariants (MANUAL, disarmed, CH7=0, CH8=0) continue
to be enforced by the existing, unmodified `sr75_hil_f24c_preflash_
precheck.py` read-only gate, run before anything else, exactly as in
every prior stage.

## What changed, in one sentence

A new `--stage r2c` reorders the orchestrator's startup sequence so the
dynamic feeder and responder are already serving valid SIM_JSON data
*before* prompting the operator to manually power-cycle the Pixhawk,
then waits for the hardware to reappear, restarts PPP, waits for live
post-reboot SIM_JSON replies, and runs a new read-only origin diagnostic
that **aborts the entire run — never proceeding to HIL-F24-R2
capture/comparison — unless `GPS_GLOBAL_ORIGIN` relatched within 50 m of
truth**.

## The 7 steps, as implemented

| # | Task step | Implementation |
|---|---|---|
| 1 | Start dynamic feeder and responder before the relatch step | `main()`'s `stage == "r2c"` branch starts both via `subprocess.Popen` immediately after the precheck, *before* PPP is touched at all |
| 2 | Prompt operator to power-cycle Pixhawk manually | `prompt_operator_power_cycle()` — blocks on `input()`, sends nothing to the Pixhawk |
| 3 | Reattach/wait for `/dev/ttyACM0` and `/dev/ttyUSB0` | `wait_for_devices()` — polls `os.path.exists()` for both paths, `--relatch-device-timeout-s` (default 120s), aborts on timeout |
| 4 | Start/restart PPP and wait for SIM_JSON replies | Existing `ppp_start`/`ppp_verify` steps (reused unchanged, just moved after step 3), then `wait_for_sim_json_replies()` polling `responder.csv` for `--relatch-min-replies` (default 5) new `reply_sent=1` rows within `--relatch-sim-json-timeout-s` (default 60s) |
| 5 | Run read-only origin diagnostic automatically | `sr75_hil_f24r2b_ahrs_origin_diagnostics.py` (HIL-F24-R2B, extended with `--summary-json`), run as a subprocess |
| 6 | Abort unless `GPS_GLOBAL_ORIGIN` is within 50 m of truth | `evaluate_origin_relatch_summary()` reads the diagnostic's JSON summary's `gps_global_origin_mismatch_m` field specifically (not `HOME_POSITION`, which HIL-F24-R2B already confirmed is unaffected) against `ORIGIN_RELATCH_MAX_MISMATCH_M = 50.0`; a failure raises `OrchestratorAbort`, which skips capture entirely (see "Why this is safe by construction" below) |
| 7 | Only then run R2 capture/comparison | The existing HIL-F24-R2 `start_estimator_capture`/`run_estimator_comparison` steps, reused completely unchanged, now gated behind step 6 |

## Why this is safe by construction, not just by convention

`capture_proc` is a local variable initialized to `None` at the top of
`main()` and is only ever assigned inside the `if args.stage in ("r2",
"r2c"):` block, which is reached *after* the entire `r2c`-branch relatch
sequence above completes without raising. If `evaluate_origin_relatch_
summary()` returns not-ok, `OrchestratorAbort` is raised immediately,
control jumps straight to `finally:`, and `capture_proc` is still `None`
— so the existing guard `if args.stage in ("r2", "r2c") and capture_proc
is not None:` (unchanged logic, just widened from `"r2"`) *cannot* run
the comparison step, and no capture process was ever started. This isn't
an extra `if` a future edit could accidentally bypass; it's the same
"was this process actually started" check every other stage already
relies on for its own shutdown sequencing (`feeder_proc`/`responder_proc`
follow the identical pattern).

## Task 1: relevant EKF/GPS/AHRS configuration and initialization paths checked

No new paths beyond HIL-F24-R2B — this task reuses `sr75_hil_gps_ekf_
readonly_audit.py`'s `CHECKS` (via the unmodified R2B diagnostic script)
and adds no new parameter reads. The relatch workflow's only new
read-only MAVLink traffic is what `sr75_hil_f24r2b_ahrs_origin_
diagnostics.py` already sent in R2B (`PARAM_REQUEST_READ`, `MAV_CMD_
GET_HOME_POSITION`, `MAV_CMD_REQUEST_MESSAGE(GPS_GLOBAL_ORIGIN)`), now
invoked programmatically with a `--summary-json` output the orchestrator
parses.

## Exact files changed

### `Tools/autotest/sr75_hil_layer2/scripts/sr75_hil_f24r2b_ahrs_origin_diagnostics.py`

- New `--summary-json PATH` (optional; omitting it preserves the exact
  R2B behavior/output, confirmed by a dedicated regression test).
- `home_mismatch_m`/`gps_global_origin_mismatch_m` are now tracked as
  two **separate** values (previously only a combined running-max
  existed internally) so a caller — specifically HIL-F24-R2C's gate —
  can read the exact figure that matters to it (`GPS_GLOBAL_ORIGIN`,
  not the max of both) rather than an ambiguous combined number. All
  existing stdout/exit-code behavior (and all 10 pre-existing R2B tests)
  is unchanged.

### `Tools/autotest/sr75_hil_layer2/scripts/sr75_hil_f24f_hardware_orchestrator.py`

New pure/testable functions (all dependency-injected — `exists_fn`/
`sleep_fn`/`now_fn`/`count_fn`/`input_fn`/`print_fn` — so every one is
unit-tested without a real device, a real sleep, or real console input):

- `wait_for_devices(paths, timeout_s, ...)` — step 3.
- `count_sim_json_replies(responder_csv_path)` / `wait_for_sim_json_
  replies(responder_csv_path, min_new_replies, timeout_s, ...)` — step 4
  ("wait for SIM_JSON replies", not just "PPP is up" — `ppp_verify`
  already checks link-level connectivity; this additionally confirms
  the responder is actually answering *real* post-reboot requests).
- `prompt_operator_power_cycle(input_fn, print_fn)` — step 2.
- `evaluate_origin_relatch_summary(summary, max_mismatch_m)` — step 6,
  pure decision logic separated from the subprocess/JSON-parsing glue
  so it's trivially unit-testable against the real captured fixture.

New constants: `R2B_ORIGIN_DIAGNOSTIC_SCRIPT`, `ORIGIN_RELATCH_MAX_
MISMATCH_M = 50.0` (matches the task's own number and the diagnostic
script's `ORIGIN_MISMATCH_WARN_M`, documented-constant convention).

`should_proceed_to_hardware()`: `stage == "r2c"` now requires `--confirm-
origin-relatch` specifically (checked *before* the `stage == "r2"`
branch), non-substitutable with any of the other three confirm flags —
5 new tests, mirroring the existing R1/R2 pattern exactly.

`build_session_dir()`: `stage == "r2c"` sessions are named
`sr75_hil_f24r2c_relatch_<timestamp>`, distinct from static/dynamic/r2.

`build_execution_plan()`: `stage == "r2c"` builds a **differently
ordered** plan (feeder/responder before PPP — the entire point of this
task) with 4 new steps (`prompt_power_cycle`, `wait_for_reattach`,
`wait_for_sim_json_replies`, `run_origin_relatch_check`), then falls
into the same `stage in ("r2", "r2c")` branch that appends `start_
estimator_capture`/`run_estimator_comparison` (now shared between r2 and
r2c, unchanged for r2). Static/dynamic/r2 plans are confirmed
byte-identical to before (dedicated tests).

`session_artifact_paths()`: added `origin_relatch_summary_json` (8th
placeholder, up from 7).

`build_arg_parser()`: `--stage` gained the `"r2c"` choice; new
`--confirm-origin-relatch`, `--relatch-listen-s` (default 6.0),
`--relatch-truth-lat/-lon/-alt-m` (default: R1's runscript IC),
`--relatch-device-timeout-s` (default 120.0), `--relatch-sim-json-
timeout-s` (default 60.0), `--relatch-min-replies` (default 5).

`main()`: validates `--stage r2c` also requires `--profile dynamic`
(same pattern as r2); the `stage == "r2c"` branch implements steps 1-6
inline (feeder/responder start, prompt, reattach-wait, PPP (re)start/
verify, SIM_JSON-reply-wait, origin-diagnostic-run-and-gate) before
falling through to the shared capture-start/sleep/shutdown/comparison
logic already used by `stage == "r2"`. The `finally:` block's
comparison-scoring condition widened from `args.stage == "r2"` to
`args.stage in ("r2", "r2c")` — the only change to previously-existing
logic in that block; everything else (stop order, `record_dynamic_
profile_evidence`, PPP stop) is untouched.

### `Tools/autotest/sr75_hil_layer2/scripts/test_sr75_hil_f24f_hardware_orchestrator.py`

`make_args()` gained r2c defaults. 29 new tests: `TestWaitForDevices`
(4), `TestWaitForSimJsonReplies` (5), `TestPromptOperatorPowerCycle` (1),
`TestEvaluateOriginRelatchSummary` (4, including the real captured
poisoned-origin mismatch value), `TestShouldProceedToHardwareR2cStage`
(5), `TestBuildExecutionPlanR2cStage` (7, including step-order and
feeder/responder-precede-PPP assertions — the core behavioral guarantee
of this task), `TestBuildSessionDirR2cStage` (1), `TestMainR2cStageValidation`
(2, CLI subprocess dry-run checks, one confirming a dry run never calls
`input()` by running with `stdin=DEVNULL`). One pre-existing test
(`test_all_seven_placeholders_present`) renamed/updated to `test_all_
eight_placeholders_present` to account for the new `origin_relatch_
summary_json` key. All 47 pre-existing tests otherwise unchanged.

### `Tools/autotest/sr75_hil_layer2/scripts/test_sr75_hil_f24r2b_ahrs_origin_diagnostics.py`

3 new tests (`TestSummaryJsonOutput`): a healthy case's JSON fields, the
real poisoned-origin fixture's JSON fields (confirms the ~11,231 km
mismatch is present in the *machine-readable* output HIL-F24-R2C's gate
actually reads, not just stdout), and confirmation that omitting
`--summary-json` writes no file (backward compatibility). All 10
pre-existing R2B tests unchanged.

## Test results

```
$ python3 -m pytest scripts/test_sr75_hil_f24f_hardware_orchestrator.py -v
76 passed, 4 subtests passed   (47 pre-existing + 29 new)

$ python3 -m pytest scripts/test_sr75_hil_f24r2b_ahrs_origin_diagnostics.py -v
13 passed                       (10 pre-existing + 3 new)

$ python3 -m pytest sim_json scripts -q
584 passed, 19 subtests passed, 1 failed
```

The 1 failure (`test_sr75_hil_f24s_coherence_and_accounting.py::
TestShutdownAccountingAtomicity::test_burst_then_immediate_sigterm_
never_mismatches`) is the same pre-existing, timing-sensitive
burst+immediate-SIGTERM subprocess race already documented as flaky in
`HIL_F24_R2A_first_hardware_capture_diagnosis.md` and reconfirmed
unrelated here too — it passed cleanly on an immediate isolated re-run
(`1 passed, 15 subtests passed`), and this task touched neither that
file nor anything it depends on.

`flake8 --max-line-length=200`, `py_compile`, and `git diff --check` are
clean on all 4 modified files.

### Dry-run plan confirmation

```
$ python3 sr75_hil_f24f_hardware_orchestrator.py --profile dynamic --stage r2c
... 13-step plan: check_adapters, precheck, start_feeder, start_responder,
    prompt_power_cycle, wait_for_reattach, ppp_start, ppp_verify,
    wait_for_sim_json_replies, run_origin_relatch_check,
    start_estimator_capture, run_estimator_comparison, ppp_stop ...

$ python3 sr75_hil_f24f_hardware_orchestrator.py --profile dynamic --stage r2
... unchanged 9-step plan (confirmed byte-identical to before this task) ...

$ python3 sr75_hil_f24f_hardware_orchestrator.py
... unchanged 7-step static plan (confirmed byte-identical to before this task) ...
```

## Guarded rerun command

```sh
# 1. Dry run (default) -- prints the exact 13-step plan, touches nothing:
python3 Tools/autotest/sr75_hil_layer2/scripts/sr75_hil_f24f_hardware_orchestrator.py \
    --profile dynamic --stage r2c --duration-s 30

# 2. The guarded real run -- will PROMPT for a manual Pixhawk power-cycle
#    partway through, and will ABORT (printing why, never reaching
#    capture) unless GPS_GLOBAL_ORIGIN relatches within 50 m of truth:
python3 Tools/autotest/sr75_hil_layer2/scripts/sr75_hil_f24f_hardware_orchestrator.py \
    --profile dynamic --stage r2c --duration-s 30 \
    --execute --confirm-origin-relatch
```

All existing defaults apply unchanged (`--pixhawk /dev/ttyACM0`,
`--ppp-device /dev/ttyUSB0`, `--host-ip 192.168.144.2`, `--pixhawk-ip
192.168.144.14`, `--listen-port 9002`), plus the new relatch defaults
(`--relatch-listen-s 6.0`, `--relatch-truth-lat 32.5378085 --relatch-
truth-lon 74.3661944 --relatch-truth-alt-m 240.201118` — R1's runscript
IC, matching R2B's diagnostic default — `--relatch-device-timeout-s
120.0`, `--relatch-sim-json-timeout-s 60.0`, `--relatch-min-replies 5`).

Session artifacts (`sr75_hil_f24r2c_relatch_<timestamp>/`) include
everything R2/R1 already record, plus `origin_relatch.log` and
`origin_relatch_summary.json` from step 5/6's diagnostic run.

## Safety

Unchanged from every prior stage: MANUAL, disarmed, CH7=0, CH8=0 (read-
only precheck, run first, before PPP is even touched); no `PARAM_SET`,
flash, `AUTO`/mission, or actuator/servo/engine output anywhere in this
task's code (the responder stays `LOG_ONLY`, the capture and origin
diagnostics remain read-only-only). The one new MAVLink traffic pattern
(`MAV_CMD_GET_HOME_POSITION`/`MAV_CMD_REQUEST_MESSAGE(GPS_GLOBAL_
ORIGIN)`) was already introduced, safety-reviewed, and tested in
HIL-F24-R2B; this task adds no new command types, only automates running
that existing read-only script and gates on its output.

## Files changed

- `Tools/autotest/sr75_hil_layer2/scripts/sr75_hil_f24r2b_ahrs_origin_diagnostics.py`
  -- `--summary-json` output added.
- `Tools/autotest/sr75_hil_layer2/scripts/sr75_hil_f24f_hardware_orchestrator.py`
  -- `wait_for_devices()`, `count_sim_json_replies()`/`wait_for_sim_json_
  replies()`, `prompt_operator_power_cycle()`, `evaluate_origin_relatch_
  summary()` (new); `R2B_ORIGIN_DIAGNOSTIC_SCRIPT`/`ORIGIN_RELATCH_MAX_
  MISMATCH_M` (new constants); `should_proceed_to_hardware()`,
  `build_session_dir()`, `build_execution_plan()`, `session_artifact_
  paths()`, `build_arg_parser()`, `main()` (all extended for `--stage
  r2c`, static/dynamic/`--stage r2` behavior unchanged).
- `Tools/autotest/sr75_hil_layer2/scripts/test_sr75_hil_f24f_hardware_orchestrator.py`
  -- `make_args()` extended; 29 new tests across 8 new test classes; 1
  pre-existing test renamed to reflect the 8th placeholder.
- `Tools/autotest/sr75_hil_layer2/scripts/test_sr75_hil_f24r2b_ahrs_origin_diagnostics.py`
  -- 3 new tests (`TestSummaryJsonOutput`).
- Unchanged: `sr75_hil_f24r2_estimator_capture.py`, `sr75_hil_f24r2_
  estimator_comparison.py`, `sr75_hil_gps_ekf_readonly_audit.py`, and
  all firmware/PPP/JSBSim/control source.
