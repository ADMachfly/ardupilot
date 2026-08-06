# HIL-F24-R2E: Fix R2C Zero-Reply Startup

Builds on `HIL_F24_R2C_ekf_origin_relatch.md` (the `--stage r2c` guarded
relatch workflow: feeder + responder start before PPP, then the operator
is prompted to power-cycle the Pixhawk). This task fixes a real, captured
R2C run (`sr75_hil_f24r2c_relatch_20260806T063658Z`) that aborted with
zero SIM_JSON replies. Host-side Python only. No firmware, parameters,
control code, or R2/R2B/R2C thresholds were touched.

## Root cause (task 1)

`responder.log` from the real failed session, in full:

```
ERROR: bind failed on 192.168.144.2:9002: [Errno 99] Cannot assign requested address
SR75 SIM_JSON RESPONDER
...
```

R2C starts the responder **before** PPP exists (that's the entire point
of the stage — see R2C's report). The orchestrator's `responder_cmd`,
shared with every other profile/stage, always passed `--listen-host
{args.host_ip}` (`192.168.144.2`, the PPP-assigned address). `192.168.144.2`
is not assigned to any local interface until `pppd` negotiates it —
binding a UDP socket to a specific address that doesn't exist yet fails
immediately with `EADDRNOTAVAIL` (`sr75_sim_json_responder.py`'s own
`sock.bind()` call, which on failure prints the error and returns exit
code 2, no retry). The responder process died within milliseconds of
starting. The feeder (a separate, independent process) had no way to
know this and ran its full 35-second `SR75_hil_f24r1_dynamic_state_feed.xml`
JSBSim session to completion regardless (confirmed: `feeder.log` shows
`Done: published 1737 rows over 35.0s`) — with no responder alive to
answer anything, for the entire window. By the time `wait_for_sim_json_
replies()` polled `responder.csv`, there was no process left that could
ever produce a reply, so it correctly (if slowly) timed out at zero
replies, and the origin diagnostic/capture (steps 5-7) never ran.

## Minimal fix (tasks 2-5)

**Task 2 — bind address.** `sr75_sim_json_responder.py`'s own
`--listen-host` default is already `0.0.0.0` (all interfaces) — the
orchestrator's shared `responder_cmd` construction explicitly overrides
this to `args.host_ip` for every profile/stage. `0.0.0.0` requires no
specific interface/address to exist yet: a socket bound to it accepts
traffic on *any* local address, including one assigned to an interface
*after* the bind call — standard, well-defined behavior, not a race.
Chose this over the task's other offered option ("start/restart responder
immediately after PPP is UP") because it is strictly smaller: no new
restart/rebind logic, no second responder process, no changed step
ordering — a one-argument change scoped to exactly the step that needed
it.

Implemented as a **new, R2C-only** `responder_cmd` variant
(`R2C_RESPONDER_LISTEN_HOST = "0.0.0.0"`, used only inside `build_
execution_plan()`'s `stage == "r2c"` branch) — the pre-existing shared
`responder_cmd` (used by static/dynamic/`--stage r2`, all of which only
ever start the responder *after* `ppp_verify` has already confirmed
`args.host_ip` is up) is completely untouched, satisfying task 6 by
construction, not just by testing.

**Task 3/4 — responder-alive check + early-exit process-health check.**
`wait_for_process_startup_health(proc, grace_s=1.0)` (new, pure,
dependency-injected): a brief grace period then a single `proc.poll()`
check. Called immediately after `Popen`-ing the responder, **before**
the (slow, operator-blocking) power-cycle prompt — if the responder died
on startup for any reason, the operator is never sent to go power-cycle
hardware for a workflow that could never succeed; the run aborts
immediately with the responder's own log tail in the error message.
`wait_for_sim_json_replies()` gained an optional `alive_fn` parameter
(default `None`, preserving the exact pre-R2E signature/behavior for any
other caller): checked on every poll iteration, and the function returns
`False` the instant it reports the process has died, instead of always
waiting out the full `--relatch-sim-json-timeout-s`. Wired in `main()` as
`alive_fn=lambda: responder_proc.poll() is None`.

**Task 5 — stale raw JSBSim evidence.** New `cleanup_stale_raw_jsbsim_
evidence(path)`, called on the fixed `/tmp/sr75_hil_f24r1_jsbsim_raw.csv`
path (`DYNAMIC_FEEDER_RAW_OUTPUT`) at the very start of R2C's branch,
before the feeder starts. The feeder itself already does this identical
cleanup internally (`sr75_hil_f24r1_dynamic_jsbsim_feed.py`'s
`start_jsbsim()`), so this is a defensive, harmless duplicate specific to
R2C — added because R2C's longer, multi-phase, operator-involving run
(spanning a real power-cycle) makes stale-evidence hygiene more visible/
important than for R1/R2's single-shot runs, which are left untouched.

## Task 6: static/R1/R2 unchanged

Verified both by construction and by test:
- `build_execution_plan()`'s pre-existing `responder_cmd` (built once,
  shared by static/dynamic/`--stage r2`) is never touched by the new
  `r2c_responder_cmd` variable — a separate, additional local variable
  used only inside the `stage == "r2c"` branch.
- `wait_for_sim_json_replies()`'s new `alive_fn` parameter defaults to
  `None`; every existing call site (there were none outside R2C, and R2C
  itself is the only caller) is unaffected unless explicitly passed.
- `cleanup_stale_raw_jsbsim_evidence()` is called only inside the
  `stage == "r2c"` branch.
- Dedicated regression tests (below) directly assert static/dynamic/
  `--stage r2` still bind to `args.host_ip`, and the pre-existing 76
  orchestrator tests continue to pass unmodified.

## Task 7: regression tests added

All in `test_sr75_hil_f24f_hardware_orchestrator.py` (91 tests total, up
from 76 — 15 new):

- **`TestBuildExecutionPlanR2eResponderBind`** (4) — r2c's `start_
  responder` Step command contains `--listen-host 0.0.0.0` and *not*
  `192.168.144.2`; static/dynamic/`--stage r2` still bind to `args.
  host_ip`, unaffected; r2c's responder command still carries the
  correct port/state/responder-csv placeholders and no actuator flags.
- **`TestWaitForProcessStartupHealth`** (3) — pure, fake-process tests:
  an alive process is healthy; a dead process (`poll()` returns an exit
  code) is unhealthy; the default grace period matches the documented
  `RESPONDER_STARTUP_HEALTH_CHECK_S` constant.
- **`TestWaitForSimJsonRepliesEarlyExit`** (3) — a dead process (`alive_
  fn` returns `False`) exits on the very first iteration, before any
  `sleep_fn` call, rather than waiting out the full timeout; an alive
  process is unaffected and still waits for real replies; omitting
  `alive_fn` entirely reproduces the exact pre-R2E timeout behavior
  (backward compatibility).
- **`TestCleanupStaleRawJsbsimEvidence`** (3) — removes an existing
  stale file and reports `True`; a missing file is a no-op (`False`, no
  exception); never touches an unrelated path in the same directory.
- **`TestResponderBindBeforePPP`** (2) — **real subprocess regression
  tests against the actual `sr75_sim_json_responder.py` script, not
  mocks**: binding to an unassigned specific address (standing in for
  `192.168.144.2` before PPP exists) reproduces the exact original bug
  (nonzero exit, `"bind failed"` in output) within 10s; binding to
  `0.0.0.0` on the same kind of pre-PPP environment succeeds and the
  process stays alive past the startup-health grace period. These two
  tests are the direct, empirical proof that both the root-cause
  diagnosis and the fix are correct — not just internally consistent.

### Test results

```
$ python3 -m pytest scripts/test_sr75_hil_f24f_hardware_orchestrator.py -v
91 passed, 6 subtests passed   (76 pre-existing + 15 new)

$ python3 -m pytest sim_json scripts -q
599 passed, 30 subtests passed in 109.79s
```

Zero regressions. `flake8 --max-line-length=200`, `py_compile`, and `git
diff --check` are clean on both modified files.

### Dry-run plan confirmation

```
$ python3 sr75_hil_f24f_hardware_orchestrator.py --profile dynamic --stage r2c
  4. [start_responder] Start responder in LOG_ONLY mode, bound to 0.0.0.0 (before PPP exists)
       $ ... sr75_sim_json_responder.py --listen-host 0.0.0.0 --listen-port 9002 ...

$ python3 sr75_hil_f24f_hardware_orchestrator.py --profile dynamic --stage r2
  6. [start_responder] Start responder in LOG_ONLY mode (no actuator output)
       $ ... sr75_sim_json_responder.py --listen-host 192.168.144.2 --listen-port 9002 ...   # unchanged

$ python3 sr75_hil_f24f_hardware_orchestrator.py
  6. [start_responder] Start responder in LOG_ONLY mode (no actuator output)
       $ ... sr75_sim_json_responder.py --listen-host 192.168.144.2 --listen-port 9002 ...   # unchanged
```

## Guarded rerun command

Unchanged from R2C's own report — no CLI surface changed, only the
responder's bind address for this one stage:

```sh
python3 Tools/autotest/sr75_hil_layer2/scripts/sr75_hil_f24f_hardware_orchestrator.py \
    --profile dynamic --stage r2c --duration-s 30 \
    --execute --confirm-origin-relatch
```

Expected difference from the failed run: `responder.log` should now show
`Listening on 0.0.0.0:9002` (not a bind error), the orchestrator should
print `Responder is alive.` before the power-cycle prompt, and — once
the operator power-cycles the Pixhawk and PPP comes back up — `Live
SIM_JSON replies confirmed.` before the origin diagnostic gate runs.

## Files changed

- `Tools/autotest/sr75_hil_layer2/scripts/sr75_hil_f24f_hardware_orchestrator.py`
  — new `R2C_RESPONDER_LISTEN_HOST`/`RESPONDER_STARTUP_HEALTH_CHECK_S`
  constants; new `wait_for_process_startup_health()`, `cleanup_stale_
  raw_jsbsim_evidence()` functions; `wait_for_sim_json_replies()` gained
  the optional `alive_fn` parameter; `build_execution_plan()`'s r2c
  branch builds its own `r2c_responder_cmd`; `main()`'s r2c branch calls
  the new cleanup/health-check functions and wires `alive_fn` into the
  reply-wait call.
- `Tools/autotest/sr75_hil_layer2/scripts/test_sr75_hil_f24f_hardware_orchestrator.py`
  — 15 new tests across 5 new test classes.
- Unchanged: `sr75_sim_json_responder.py`, `sr75_hil_f24r1_dynamic_
  jsbsim_feed.py`, `sr75_hil_f24r2_estimator_capture.py`/`_comparison.py`,
  `sr75_hil_f24r2b_ahrs_origin_diagnostics.py`, all firmware/PPP/JSBSim
  source, all thresholds.
