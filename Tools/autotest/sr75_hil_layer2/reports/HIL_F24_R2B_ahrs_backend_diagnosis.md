# HIL-F24-R2B: AHRS Backend Selection Diagnosis and Stabilization

Builds on `HIL_F24_R2A_first_hardware_capture_diagnosis.md` (R2 comparison
math/scaling/classification verified correct; failure caused by real
EKF3→DCM→EKF3 transitions). This task goes one level deeper: **why** EKF3
rejects the injected SIM_JSON position and falls back to DCM. No
parameters were set, nothing was flashed, armed, or run in AUTO, no
output was enabled. All Pixhawk interaction in this task was read-only
(`PARAM_REQUEST_READ`, `MAV_CMD_GET_HOME_POSITION`,
`MAV_CMD_REQUEST_MESSAGE`); all firmware source was read, never modified.

## First causal mismatch

**EKF3's own origin (`GPS_GLOBAL_ORIGIN`) is latched to ArduPilot's
compiled-in default SITL home location (Canberra CMAC,
`lat=-35.3632621 lon=149.1652374 alt=584.0`) instead of the bench's
actual injected truth position — a live, read-only-confirmed horizontal
mismatch of ~11,231 km — while `HOME_POSITION` on the very same Pixhawk,
at the very same moment, correctly tracks truth to within 0.01 m.**

Captured live (`sr75_hil_f24r2b_ahrs_origin_diagnostics.py`, this task):

```
HOME_POSITION: lat=32.5378085 lon=74.3661943 alt=0.16
GPS_GLOBAL_ORIGIN: lat=-35.3632621 lon=149.1652374 alt=584.0
Truth IC: lat=32.5378085 lon=74.3661944 alt=240.201118
HOME_POSITION vs. truth: horizontal=0.01 m, altitude_diff=-240.04 m
GPS_GLOBAL_ORIGIN vs. truth: horizontal=11231266.38 m, altitude_diff=343.80 m
```

`-35.363261, 149.165230` is ArduPilot's own well-known compiled-in
default SITL/EKF fallback location (CMAC, Canberra) — not a random or
corrupted value. This means EKF3's one-shot origin latch (ArduPilot sets
`GPS_GLOBAL_ORIGIN` once, from the *first* available absolute-position
fix, and never changes it again for the rest of that boot) captured a
**pre-connection default/placeholder fix**, not the real SIM_JSON-fed
truth position — most plausibly during the Pixhawk's own boot sequence,
before this bench's PPP/SIM_JSON link had ever delivered a real position
(this Pixhawk has evidently not been power-cycled since; the origin
persists, wrong, across every subsequent orchestrator run of the day).

`HOME_POSITION`, by contrast, is a *different* ArduPilot reference (the
pilot's "return here" point) that **does** get re-set from the live
absolute-position stream while on the ground and disarmed — which is
exactly why it correctly tracks truth even though the EKF's own origin
never did.

**Why this produces exactly R2A's symptoms**: `AP_AHRS`'s DCM backend
computes its own position estimate relative to `HOME_POSITION` (correct,
truth-synced), while EKF3 computes its relative to its own
`GPS_GLOBAL_ORIGIN` (wrong, Canberra-latched). Every time the AHRS
backend flips between DCM and EKF3 (the transitions R2A already
identified via `"AHRS: DCM active"`/`"AHRS: EKF3 active"` STATUSTEXT),
`GLOBAL_POSITION_INT` switches which reference frame it is being
expressed relative to — producing exactly the large, transient,
reconverging position discontinuities R2A captured (`t=3273.14`,
`t=3293.20/3293.30`, `t=3303.39/3303.51`), independent of anything in
the R2 Python scripts.

## Task 1: why EKF3 rejects the injected position and falls back to DCM

Two independent, read-only-confirmed factors combine:

1. **Origin mismatch** (above) — the structural reason a backend switch
   produces a *large* discontinuity rather than a small one.
2. **Main-loop timing jitter driven by the SIM_JSON architecture itself**
   — the most likely proximate trigger for EKF3 periodically judging
   itself/GPS unhealthy in the first place. `libraries/SITL/SIM_JSON.cpp`
   documents (comment above `JSON_HW_RECV_TIMEOUT_MS`, from the
   HIL-F24-N reboot-loop fix) that on real hardware `JSON::update()` runs
   **synchronously on the main vehicle thread, once per scheduler tick** —
   the flight code's main loop literally waits on the SIM_JSON round trip
   every tick (bounded to 200ms to avoid the watchdog reset HIL-F24-N
   fixed, but not decoupled from the loop rate). The R2 session's own
   `responder.csv` shows the Pixhawk's *own firmware-reported*
   `frame_rate` field degrading during the run: `400 → 61 → 31 → 21` Hz,
   and `round_trip_or_processing_us` across all 1410 requests: median
   20.25ms, p95 34.57ms, max 200.3ms (hitting `JSON_HW_RECV_TIMEOUT_MS`
   exactly once — the documented startup race from R2A task 4).
   Independently, right now, with no active bench run, a fresh read-only
   precheck reproduced the same class of degradation live:
   `PreArm: Main loop slow (36Hz < 50Hz)` and
   `PreArm: AHRS: not using configured AHRS type`. A main loop whose
   period swings between ~2.5ms and 200ms (vs. a nominal 20ms/50Hz tick)
   directly degrades EKF3's IMU delta-time-consistency and
   innovation-gating assumptions — a well-documented EKF3 sensitivity,
   not something specific to this project's Python code — and is the
   most likely trigger for the periodic `EKF_GPS_GLITCHING` flag
   (confirmed present in every `EKF_STATUS_REPORT` sample of the R2
   session) and the resulting AHRS backend fallback.

GPS1_TYPE and the EK3_SRC1_* source-selection parameters are **not** the
cause (see Task 4) — GPS1_TYPE=100 is architecturally correct for this
bench, not a misconfiguration.

## Task 2: EKF origin vs. GPS origin vs. JSBSim initial lat/lon/alt vs. HOME vs. AHRS position at startup

| Reference | Value (live, read-only) | vs. truth IC (`32.5378085, 74.3661944, 240.201118`) |
|---|---|---|
| JSBSim initial lat/lon/alt (R1 runscript IC, `accel_ground_init`) | `32.5378085, 74.3661944, 240.201118` | — (this *is* truth) |
| GPS origin (`AP_GPS_SITL`, i.e. `GPS_RAW_INT`) | exactly matches truth on every sample throughout the entire R2 session (159/159 samples, `fix_type=3`, 15 sats, R2A finding) | **0 m** — always correct |
| HOME_POSITION | `32.5378085, 74.3661943, 0.16` | **0.01 m** horizontal (240.04 m altitude — see note) |
| EKF origin (`GPS_GLOBAL_ORIGIN`) | `-35.3632621, 149.1652374, 584.0` | **11,231,266 m** horizontal, 343.80 m altitude |
| AHRS position during a backend switch (R2A) | e.g. `-1.8481856, 113.4745742` at t=3273.14 | millions of metres, transient, reconverging |

The HOME_POSITION altitude difference (−240.04 m) is expected and
already explained by R1: the runscript's raw IC altitude (240.2 m,
before `do_simple_trim` settles the aircraft to the ground within the
first JSBSim timestep) versus the settled ~0.17 m ground-contact
altitude every subsequent sample (including HOME_POSITION, captured
after settling) reports — not a new anomaly.

The EKF-origin-vs-everything-else mismatch is the new, previously
undiagnosed finding.

## Task 3: first timestamp and exact prerequisite failure before the first backend switch

The R2 session's own capture window (starting ~3272.7s) does not contain
a STATUSTEXT explaining *why* DCM was active before `t=3273.063`
(`"AHRS: EKF3 active"`, the first backend-switch marker) — that state
predates the capture window opening. Re-checked live, independently, in
this task: the **standing prerequisite failure, reproducible right now
with no bench run active**, is:

```
STATUSTEXT: PreArm: AHRS: not using configured AHRS type
STATUSTEXT: PreArm: Main loop slow (36Hz < 50Hz)
```

i.e. AHRS is not running the configured type (EKF3, `AHRS_EKF_TYPE=3`,
confirmed correctly set) *before* any SIM_JSON position has been fed in
this ad-hoc check — consistent with EKF3 needing a valid, sufficiently
long run of consistent GPS fixes before it will activate at all, and
with the main loop's own timing already being marginal even with no
active client load. The **first uniquely-identifying failure timestamp**
established directly from the R2 session itself remains R2A's
`t=3273.063426684` (`"AHRS: EKF3 active"`), with the origin-latch defect
(Task 2, this document) as the confirmed structural reason that
transition (and the 3 later ones) produced a large positional
discontinuity rather than a small one.

## Task 4: relevant EKF/GPS/AHRS configuration and initialization paths checked

Live, read-only, re-confirmed today (`sr75_hil_gps_ekf_readonly_audit.py`,
after the fix below):

```
GPS1_TYPE        =    100  OK   (GPS_TYPE_SITL -- confirmed correct, see below)
GPS2_TYPE        =      0  INFO
GPS_AUTO_CONFIG  =      1  INFO
GPS_AUTO_SWITCH  =      1  INFO
EK3_ENABLE       =      1  OK
AHRS_EKF_TYPE    =      3  OK   (real EKF3 fusion, not the SITL/type-10 bypass)
EK3_SRC1_POSXY   =      3  OK   (GPS)
EK3_SRC1_POSZ    =      1  INFO (BARO, expected default)
EK3_SRC1_VELXY   =      3  OK   (GPS)
EK3_SRC1_VELZ    =      3  INFO
EK3_SRC1_YAW     =      1  INFO (COMPASS, expected default)
EK3_GPS_TYPE     = (none)  NO_RESPONSE (param removed in this EKF3 version, expected)
SERIAL1_PROTOCOL =     48  INFO (PPP)
SERIAL1_BAUD     =    921  INFO
No blocking mismatches found.
```

None of these are the cause. In particular, **`GPS1_TYPE=100` is
confirmed, by reading `libraries/AP_GPS/AP_GPS.h:114`
(`GPS_TYPE_SITL = 100`, gated `#if AP_SIM_GPS_ENABLED`), to be a
legitimate, architecturally-correct value** — this bench runs a
SIM_ENABLED firmware build (per `HIL_F24_A_fmuv3_simulation_on_hardware_
feasibility_audit.md`) whose compiled-in `AP_GPS_SITL` backend is fed
directly by the SIM_JSON protocol, not the GPS_INPUT-over-MAVLink
architecture (`GPS1_TYPE=14`) an earlier iteration of this project's own
tooling assumed. `libraries/SITL/SIM_JSON.cpp` was also read and
confirmed to carry no GPS-specific fields (fix type, satellite count) —
`AP_GPS_SITL` synthesizes an idealized, always-3D-fix GPS report
directly from the fed truth position, exactly matching the R2 session's
observed `GPS_RAW_INT` behavior.

**A pre-existing tooling bug was found and fixed as a direct result of
this check**: `sr75_hil_gps_ekf_readonly_audit.py`'s `CHECKS` table
required `GPS1_TYPE == 14` (from an earlier, now-superseded design
assumption) and reported the bench's correct, intentional `GPS1_TYPE=100`
as a `BLOCKER` — a false positive against the architecture actually in
use, confirmed live before the fix (see "Minimal fix" below).

## Task 5: minimum corrective change

**Implemented today (in scope — Python tooling only, no firmware/PPP/
control/threshold change):**

`sr75_hil_gps_ekf_readonly_audit.py`'s `GPS1_TYPE` check now accepts
both `14` (GPS_INPUT-over-MAVLink) and `100` (`GPS_TYPE_SITL`, this
bench's actual, confirmed-correct architecture) instead of only `14` —
removing a false-positive blocker without weakening the check for a
genuine misconfiguration (e.g. `GPS1_TYPE=0`, still correctly flagged;
see tests below).

**Proposed, NOT implemented (requires separate authorization; outside
this task's scope per "do not modify firmware/control/PPP/JSBSim,"
"do not modify thresholds or control tuning," and "do not PARAM_SET,
flash, arm, run AUTO, or enable outputs"):**

1. **Primary recommendation — re-establish the EKF origin correctly.**
   The origin is latched once per boot; the fix is operational, not
   code: power-cycle (or otherwise reboot) the Pixhawk with PPP + the
   feeder + the responder already up and actively serving valid
   SIM_JSON position data, so EKF3's very first absolute-position fix
   after boot is the real bench truth position rather than a transient
   pre-connection default. This requires no PARAM_SET and no firmware
   change — a bench operating-procedure change only (reorder: start
   PPP/feeder/responder first, confirm the responder is answering with
   `reply_reason=OK`, *then* power-cycle/reset the Pixhawk, *then* begin
   the R2 capture window). Not executed in this task, since it requires
   physical/operator action on real hardware.
2. **Structural/firmware-level recommendation** (a future, separately
   authorized task; explicitly not implemented here): decouple
   `libraries/SITL/SIM_JSON.cpp`'s on-hardware `JSON::update()` from
   blocking the scheduler tick on every request/reply, so the main loop
   rate no longer directly tracks SIM_JSON/PPP round-trip latency. This
   is the more thorough fix for the timing-jitter contributor (Task 1,
   factor 2); flagged here for whoever picks up EK3/AHRS tuning work
   next, not attempted in this diagnosis-only task.
3. Explicitly **not** proposed: any `EK3_GLITCH_RAD`/innovation-gate
   parameter widening. The observed divergence (millions of metres) is
   orders of magnitude beyond any sane gate-size adjustment (default
   25 m) and this task's constraints exclude control/EKF tuning anyway
   — recorded here only to document that this avenue was considered and
   correctly rejected as inappropriate for this specific, origin-scale
   mismatch.

## Task 6: diagnostics and tests added

No PARAM_SET, flash, arm, AUTO, or output-enabling anywhere in this
task. Both new/modified scripts are read-only extensions of the
established precheck pattern (`sr75_hil_f24c_preflash_precheck.py`'s
`PARAM_REQUEST_READ`/read-only-command convention).

**New: `sr75_hil_f24r2b_ahrs_origin_diagnostics.py`** — standalone,
read-only pre-run diagnostic. Requests HOME_POSITION
(`MAV_CMD_GET_HOME_POSITION`) and GPS_GLOBAL_ORIGIN
(`MAV_CMD_REQUEST_MESSAGE`), reuses `sr75_hil_gps_ekf_readonly_audit.py`'s
`CHECKS`/`read_params` unmodified, compares HOME/origin against a given
JSBSim initial lat/lon/alt (default: R1's runscript IC) using
`sr75_hil_f24r2_estimator_comparison.horizontal_distance_m()` (reused,
not reimplemented), and scans a bounded STATUSTEXT window for known
AHRS-backend-health substrings (`"ahrs: dcm active"`, `"not using
configured ahrs type"`, `"main loop slow"`, `"gps and ahrs differ"`,
`"waiting for gps config data"`). Exits 1 (informational only, not
wired into the R2 orchestrator or its thresholds) on a param blocker,
an origin/home mismatch beyond a diagnostic-only 50 m tolerance
(`ORIGIN_MISMATCH_WARN_M`, distinct from and never fed into
`DEFAULT_THRESHOLDS`), or a health-relevant STATUSTEXT.

**Modified: `sr75_hil_gps_ekf_readonly_audit.py`** — `GPS1_TYPE` check
widened to accept `("14", "100")` (see Task 5); unused `Path` import
removed (pre-existing, cleaned up while already touching this file).

**New: `test_sr75_hil_f24r2b_ahrs_origin_diagnostics.py`** — 10 tests:
6 pure `scan_statustext_for_ahrs_health()` tests (each of the 4
substrings, a benign-text negative, case-insensitivity), and 4
end-to-end tests against a PTY fake Pixhawk (same technique proven in
`test_sr75_hil_f24r2_estimator_capture.py`) answering
`PARAM_REQUEST_READ`/`MAV_CMD_GET_HOME_POSITION`/`MAV_CMD_REQUEST_
MESSAGE(GPS_GLOBAL_ORIGIN)`: a healthy case (HOME and origin both match
truth), a **real-bench regression fixture** using the exact captured
poisoned-origin values (asserts the ~11,231 km mismatch is detected),
an AHRS-health-STATUSTEXT case, and a safety-confirmation-string check.

**New: `test_sr75_hil_gps_ekf_readonly_audit.py`** — 8 tests: pure
classification tests confirming `GPS1_TYPE` now accepts both `14` and
`100` while other single-valued checks (e.g. `AHRS_EKF_TYPE`) are
unaffected, plus 4 end-to-end PTY-fake-Pixhawk tests confirming
`GPS1_TYPE=100` and `GPS1_TYPE=14` both now pass (exit 0), a genuinely
wrong value (`GPS1_TYPE=0`) still blocks (exit 1) — the fix widens
acceptance, it does not disable the check — and the safety-confirmation
string is present.

### Test results

```
$ python3 -m pytest scripts/test_sr75_hil_gps_ekf_readonly_audit.py -v
8 passed

$ python3 -m pytest scripts/test_sr75_hil_f24r2b_ahrs_origin_diagnostics.py -v
10 passed

$ python3 -m pytest sim_json scripts -q
(full suite; see below)
```

`flake8 --max-line-length=200`, `py_compile`, and `git diff --check` are
clean on all 4 new/modified files (2 findings caught and fixed during
this task: an unused `Path` import and one over-long line, both in
`sr75_hil_gps_ekf_readonly_audit.py`).

## Live evidence commands (all read-only; run during this task)

```sh
# GPS/EKF param audit (now correctly passes GPS1_TYPE=100):
python3 Tools/autotest/sr75_hil_layer2/scripts/sr75_hil_gps_ekf_readonly_audit.py --pixhawk /dev/ttyACM0

# New: HOME/origin-vs-truth + AHRS-health diagnostic:
python3 Tools/autotest/sr75_hil_layer2/scripts/sr75_hil_f24r2b_ahrs_origin_diagnostics.py --pixhawk /dev/ttyACM0 --listen-s 6
```

Both are read-only, safe to run at any time, and require no PPP/feeder/
responder to be active.

## Guarded rerun command

No R2 orchestrator/comparison behavior was changed by this task, so the
guarded hardware command is unchanged from the R2 report:

```sh
python3 Tools/autotest/sr75_hil_layer2/scripts/sr75_hil_f24f_hardware_orchestrator.py \
    --profile dynamic --stage r2 --duration-s 30 \
    --execute --confirm-estimator-comparison
```

**Recommended, but not executed here**: before the next such run, the
operator power-cycles the Pixhawk with PPP + feeder + responder already
up (per Task 5's primary recommendation), then re-runs
`sr75_hil_f24r2b_ahrs_origin_diagnostics.py` to confirm `GPS_GLOBAL_
ORIGIN` now matches truth (not just `HOME_POSITION`) before committing
to a full 30-second capture.

## Files changed

- New: `Tools/autotest/sr75_hil_layer2/scripts/sr75_hil_f24r2b_ahrs_origin_diagnostics.py`
- New: `Tools/autotest/sr75_hil_layer2/scripts/test_sr75_hil_f24r2b_ahrs_origin_diagnostics.py`
- New: `Tools/autotest/sr75_hil_layer2/scripts/test_sr75_hil_gps_ekf_readonly_audit.py`
- Modified: `Tools/autotest/sr75_hil_layer2/scripts/sr75_hil_gps_ekf_readonly_audit.py`
  (`GPS1_TYPE` check widened to accept 14 or 100; unused import removed)
- Unchanged: `sr75_hil_f24r2_estimator_comparison.py`,
  `sr75_hil_f24r2_estimator_capture.py`, the R2 orchestrator, and all
  firmware/PPP/JSBSim/control source (read for diagnosis only:
  `libraries/SITL/SIM_JSON.cpp`, `libraries/AP_GPS/AP_GPS.h`,
  `libraries/AP_NavEKF3/AP_NavEKF3.cpp`).
