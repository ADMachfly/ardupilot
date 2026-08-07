# HIL-F24-R3E: Why EKF_STATUS_REPORT Stays flags=1024 Through a Clean R3A Run

Diagnosis only, per this task's explicit constraint: no parameter,
threshold, firmware, control, or trajectory code was changed. All
hardware interaction in this task was read-only (`PARAM_REQUEST_READ`,
`MAV_CMD_GET_HOME_POSITION`, `MAV_CMD_REQUEST_MESSAGE`), reusing the
existing, already-safety-reviewed HIL-F24-R2B diagnostic scripts
unmodified.

Session diagnosed:
`Tools/autotest/sr75_hil_layer2/hardware_sessions/sr75_hil_f24r3a_mp_visual_20260806T145425Z`
(identified from the task's own description: 15s stationary warm-up +
60s visualization = 75s total, matching this session's
`mission_planner_checklist.md` exactly).

## Root cause

**The Pixhawk's current boot has never had EKF3 initialize successfully
at all** -- a standing, boot-scoped condition, not something R3A's own
code or trajectory caused. R3A's `--stage r3a` starts PPP, then the
feeder, then the responder directly against whatever state the Pixhawk
already happens to be in (the same ordering as `static`/`dynamic`/
`--stage r2`) -- it never performs the power-cycle/relatch procedure
`--stage r2c` uses. On this bench, per `HIL_F24_R2D_gps_sitl_fix_gating.md`'s
firmware-level trace, `AP_HAL::SIMState::fdm_input_local()`'s
`update_home()` call races ahead of `SITL::JSON::recv_fdm()`'s first
successful packet parse, and can leave EKF3 without ever latching a
valid origin from real truth data during a given boot -- a one-shot,
per-boot condition that a clean, long, 100%-aligned truth feed cannot
retroactively fix once EKF3 has already failed to initialize earlier in
that same boot. R2D's firmware fix for this was built but, per that
report, explicitly never flashed. R2C's host-side relatch workaround
(start feeder+responder *before* prompting a manual power-cycle, so the
reboot's first GPS fix is real truth) is proven to work when followed --
but R3A does not invoke it, and evidence below shows the Pixhawk was
rebooted again, outside of that procedure, between the last successful
relatch and the first R3A session.

## Evidence, with files/lines

**1. Every EKF_STATUS_REPORT sample in the R3A session shows flags=1024,
for the entire 77s window:**

```
$ python3 -c "..."   # pixhawk_estimator.csv, EKF_STATUS_REPORT rows
total EKF_STATUS_REPORT rows: 158
flags distribution: Counter({'1024.0': 158})
first: t=36316.583454414   last: t=36393.3247543
```
(`hardware_sessions/sr75_hil_f24r3a_mp_visual_20260806T145425Z/pixhawk_estimator.csv`)

**2. The comparison script's own summary confirms this is the *only*
real failure** -- everything the task's description claims is
independently confirmed accurate and unrelated:

```json
"alignment_coverage": true,      // "100% alignment" -- confirmed, coverage=1.0
"finite_values": true,
"ekf_ready_before_scoring": false,
"no_ekf_or_statustext_reset_events": false,   // first_failure
```
`sample_counts`: `{"ATTITUDE": 786, "GLOBAL_POSITION_INT": 786, "VFR_HUD": 786}`,
`truth_rows: 3799` -- "zero stale states" and "valid position/altitude
tracking" both confirmed (`altitude_error_m` mean 0.20m, `horizontal_error_m`
mean 0.61m in the same file's `metrics`).
(`hardware_sessions/sr75_hil_f24r3a_mp_visual_20260806T145425Z/estimator_summary.json`)

117 `ekf_unhealthy_flags` health events recorded, **all** `flags: 1024`
(checked programmatically against the full JSON list, not just the
markdown's truncated 20-entry preview).

**3. STATUSTEXT confirms *why*, repeating throughout the run, never
clearing:**

```
36323.297067688 PreArm: AHRS: not using configured AHRS type
36353.418023156 PreArm: AHRS: not using configured AHRS type
36383.557631098 PreArm: AHRS: not using configured AHRS type
```
(same `pixhawk_estimator.csv`) -- AHRS is not running EKF3 (the
configured `AHRS_EKF_TYPE=3`) at all; it has fallen back to DCM (per
`HIL_F24_R2B_ahrs_backend_diagnosis.md`'s prior finding, DCM's
own complementary-filter output explains why position/attitude still
track truth reasonably well even while flags=1024 the whole time).

**4. R3A's startup order matches `static`/`dynamic`/`--stage r2`, not
`--stage r2c`'s relatch sequence (task 2's explicit comparison):**

```
Listening on 192.168.144.2:9002
```
(`hardware_sessions/sr75_hil_f24r3a_mp_visual_20260806T145425Z/responder.log`)
-- the responder bound directly to the PPP address, meaning PPP was
already up *before* the responder started (R2C's responder instead
binds `0.0.0.0` because it starts *before* PPP exists --
`sr75_hil_f24f_hardware_orchestrator.py`'s `R2C_RESPONDER_LISTEN_HOST`
constant and its surrounding comment). No `prompt_operator_power_cycle`-
equivalent artifact, no `origin_relatch_summary.json`, and no
`ppp_r3a_attempt_N_*` reconnect-retry sequence beyond a single normal
attempt exists in this session directory -- R3A never reboots the
Pixhawk.

**5. Task 3's answer, proven both ways -- relatching works when
performed, and not performing it leaves the board poisoned:**

`hardware_sessions/sr75_hil_f24r2c_relatch_20260806T083459Z/origin_relatch_summary.json`
(08:34:59, *before* the first R3A session at 09:26:45):
```json
{"ok": true, "gps_global_origin_mismatch_m": 0.009374140282766152, ...}
```
GPS_GLOBAL_ORIGIN matched truth to under a centimetre. **Then, between
that successful relatch and the first R3A session ~52 minutes later,
the Pixhawk was rebooted again outside of the relatch procedure**,
re-triggering the same one-shot per-boot race and leaving EKF3
uninitialized for the rest of the day's R3A runs (confirmed live, item
6 below).

**6. Confirmed live, right now, with no feeder/responder/PPP running at
all** -- proving this is a standing boot-state, not something specific
to any one R3A run:

```
$ python3 Tools/autotest/sr75_hil_layer2/scripts/sr75_hil_f24r2b_ahrs_origin_diagnostics.py --pixhawk /dev/ttyACM0 --listen-s 6
GPS_GLOBAL_ORIGIN: NOT RECEIVED within 6.0s listen window
STATUSTEXT: PreArm: AHRS: not using configured AHRS type
STATUSTEXT: PreArm: GPS 1: Bad fix
WARN: HOME/origin vs. truth horizontal mismatch 215.51 m exceeds diagnostic-only tolerance 50.0 m
```
`AHRS_EKF_TYPE`/`GPS1_TYPE` read back unchanged (3/100) via
`sr75_hil_gps_ekf_readonly_audit.py`, confirming R2D's firmware fix was
not flashed and no parameter drifted. `HOME_POSITION` (32.5394253,
74.3674622, 270.19m) matches the *end* of the last R3A trajectory almost
exactly (checklist: "to about 32.5394254 / 74.3674623 / 270.2 m") --
just a stale leftover from the last live feed, not itself evidence of a
current fix; `GPS_GLOBAL_ORIGIN` (the actual EKF3 origin, and the field
that matters) never even answered.

## Task 1: does EKF receive valid SIM_JSON GPS/IMU before or after boot?

After -- but too late. The feeder/responder only start once R3A's plan
runs (`feeder.log`: `Starting live JSBSim state feed to ... duration_s=80.0`
at session start, ~14:54), long after this Pixhawk's actual last power-on.
By the time real SIM_JSON data starts flowing, EKF3 has already, once,
irreversibly (for this boot) either latched a bad origin or failed to
initialize at all (live check: origin was never received this boot) --
`update_home()` had already run and left `home_is_set` (and therefore
`AP_GPS_SITL`'s reported fix) in whatever state it was in before the
first real packet arrived. A long, clean, subsequent feed cannot
retroactively unstick this within the same boot.

## Task 4: first timestamp where flags should clear, and why they don't

There isn't one, within this session. Flags were already 1024 at the
very first captured `EKF_STATUS_REPORT` sample (`t=36316.583454414`,
before the 15s warm-up had even finished) and stayed 1024 through the
very last (`t=36393.3247543`). They cannot clear because nothing in
R3A's flow gives EKF3 a fresh, correctly-ordered boot to latch onto real
truth from -- the condition was already fixed (boot-scoped) before this
session's own timeline began.

## Minimum next change (not implemented -- proposal only)

R3A's execution plan should reuse `--stage r2c`'s already-implemented,
already-proven-working relatch steps (start feeder+responder, prompt/
perform a Pixhawk power-cycle, wait for reattach, restart PPP, wait for
live SIM_JSON replies) *before* R3A's own 15s warm-up begins, instead of
starting PPP→feeder→responder directly against an unknown boot state.
This is a host-side orchestration/ordering change to `--stage r3a`'s own
plan construction (reusing existing R2C steps), not a firmware,
parameter, threshold, control, or trajectory change -- out of scope to
implement in this diagnosis-only task.

## Guarded test command

Read-only, safe to run at any time, reconfirms the live diagnosis
without touching anything:

```sh
python3 Tools/autotest/sr75_hil_layer2/scripts/sr75_hil_f24r2b_ahrs_origin_diagnostics.py \
    --pixhawk /dev/ttyACM0 --listen-s 6
```

If GPS_GLOBAL_ORIGIN is still not received / mismatched, the board
needs a fresh relatch before the next R3A attempt. The existing, already
-guarded, already-proven-successful command to perform that relatch
(operator-executed, not run by this task):

```sh
python3 Tools/autotest/sr75_hil_layer2/scripts/sr75_hil_f24f_hardware_orchestrator.py \
    --profile dynamic --stage r2c --duration-s 30 \
    --execute --confirm-origin-relatch
```
