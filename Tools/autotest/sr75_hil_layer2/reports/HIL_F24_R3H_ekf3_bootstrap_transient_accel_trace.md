# HIL-F24-R3H: EKF3 Bootstrap Transient-Accel Trace Tool (Diagnostic Only)

Diagnostic only, per this task's explicit constraint: no firmware,
parameter, threshold, control, or trajectory was changed. All hardware
interaction available to this tool is read-only (`MAV_CMD_SET_MESSAGE_
INTERVAL` requests only); no `PARAM_SET`, arm, mode change, mission
item, RC override, or actuator command is ever sent.

This builds directly on
`HIL_F24_R3G_ekf3_tilt_alignment_diagnosis.md`'s "Minimum read-only
diagnostic needed" section, which asked for capture to begin at device
reattach (after an operator power-cycle) and run through the first
`"EKF3 IMU%u initialised"` STATUSTEXT, so the actual transient accel
sample latched by `AP_NavEKF3_core.cpp:511-512` could be observed
directly instead of inferred after the fact.

## What was built

`Tools/autotest/sr75_hil_layer2/scripts/sr75_hil_f24r3g_ekf3_bootstrap_trace.py`
-- waits for the Pixhawk USB device to reappear (`wait_for_devices()`,
reused unmodified from `sr75_hil_f24f_hardware_orchestrator.py`),
connects over USB (independent of PPP/SIM_JSON), requests read-only
message-interval streams for `GPS_RAW_INT`, `RAW_IMU`, `SCALED_IMU`,
`SCALED_IMU2`, `ATTITUDE`, `AHRS2`, `EKF_STATUS_REPORT`, and captures
all of these plus `STATUSTEXT` into a timestamped CSV until the first
STATUSTEXT matching `EKF3 IMU\d+ initialised` is seen (or a timeout).
It then computes, purely from the captured rows:

- the first GPS `FIX_3D` timestamp,
- the first `EKF_STATUS_REPORT` timestamp and flags,
- whether/when the bootstrap-complete STATUSTEXT was seen,
- the first `AHRS2` timestamp diverging from level by more than
  `--divergence-threshold-deg` (default 1.0°),
- the nearest accelerometer sample at or before that divergence,
- the tilt that sample predicts via the **exact**
  `AP_NavEKF3_core.cpp:514-524` formula
  (`pitch = asin(x_norm)`, `roll = atan2(-y_norm, -z_norm)`), and
- whether that prediction matches the observed AHRS2 value within
  `--match-tolerance-deg` (default 2.0°).

`Tools/autotest/sr75_hil_layer2/scripts/test_sr75_hil_f24r3g_ekf3_bootstrap_trace.py`
-- 29 tests, all passing: unit tests for every pure function
(`tilt_from_accel`, `detect_bootstrap_complete`, `find_first_
divergence`, `nearest_accel_sample_at_or_before`, `compare_predicted_
to_observed`, `summarize`, `mavlink_message_to_row`), including a
regression fixture built from R3G's real captured values
(`REAL_FROZEN_AHRS2_ROLL_DEG=7.365185081095156`,
`REAL_FROZEN_AHRS2_PITCH_DEG=7.547061919635194`,
`REAL_DERIVED_ACCEL_XYZ=(131.34050648071846, -127.08250315063593,
-983.1580283710025)`), and an end-to-end run of `main()` against a
PTY-based fake Pixhawk (`FakeMavlinkBootstrapPixhawk`) that never sends
anything but the same message types a real Pixhawk would, confirming
the tool stops correctly on the STATUSTEXT and never transmits
`PARAM_SET`/arm/mission/RC-override/actuator commands.

Verified independently: `tilt_from_accel(131.34, -127.08, -983.16)` ->
`roll=7.365026993796915°, pitch=7.547020852002051°`, matching the real
frozen R3G values to ~0.0002° -- confirms the formula implementation is
correct before relying on it for a live capture.

Full regression suite (`python3 -m pytest sim_json scripts -q` from
`Tools/autotest/sr75_hil_layer2/`): **718 passed, 2538 subtests passed,
0 failed** -- no regression from adding these two files. `flake8
--max-line-length=200` and `py_compile` clean on both; `git diff
--check` clean.

## Why a genuine live capture was not obtained in this session

The tool's precondition is a **device reattach event** -- it starts
listening the moment `/dev/ttyACM0` reappears after the operator
physically power-cycles the Pixhawk, which is the only point at which
the actual transient (hypothesized to occur once, ~1 second after
EKF3's first `InitialiseFilterBootstrap()` call, per R3G) can be
observed in flight. At the time this tool was built, `/dev/ttyACM0`
was already present and the board already fully booted (not a fresh
reattach) -- `statesInitialised` was almost certainly already latched
from whatever accel sample was read during that earlier, unobserved
boot. I do not have a means to physically power-cycle the hardware
myself, and re-running the tool against an already-booted board would
only re-observe the same already-frozen AHRS2 value R3G already
captured, not the actual transition moment. Genuinely answering item 1
below requires the operator to run the guarded command in item 6 across
their next manual reboot.

## Answers to the six requested items

1. **First divergence timestamp**: **not obtained in this session.**
   No operator-performed reboot occurred while this tool was listening.
   The existing R3G capture began *after* boot and only shows the
   already-frozen value, not the transition; R3G's own report says this
   explicitly ("not observable after the fact from this capture alone,
   since the capture began after boot, not at `t=0`"). This tool is
   what closes that gap, but it requires a live reboot to run.

2. **Accel vector immediately before/at divergence**: **not obtained
   live**, for the same reason. The best available data point remains
   R3G's back-derived vector from the frozen AHRS2 value: normalized
   ~`(0.131, -0.127, -0.983)`, i.e. ~`(131.34, -127.08, -983.16)` mg
   before normalization -- a real ~1g vector tilted ~10° from vertical,
   not zero/garbage.

3. **Does it reproduce EKF3 bootstrap roll/pitch mathematically?**
   **YES -- confirmed**, using R3G's real captured accel vector as
   input to the exact `AP_NavEKF3_core.cpp:514-524` formula:
   `tilt_from_accel(131.34, -127.08, -983.16)` yields
   `roll=7.365026993796915°, pitch=7.547020852002051°`, matching the
   real observed AHRS2 (`roll=7.365185081095156°,
   pitch=7.547061919635194°`) to within 0.0002° -- far inside any
   reasonable tolerance. This is not a new finding (R3G already worked
   this backward by hand); what's new here is an automated,
   regression-tested implementation of the same check, ready to run
   forward against a live capture.

4. **Root cause PASS/FAIL**: **PASS on the mathematical mechanism,
   FAIL (not yet attempted) on live reproduction of the actual
   transient event.** The claim "EKF3 latches a single, real,
   off-level accelerometer sample during `InitialiseFilterBootstrap()`,
   and that sample alone explains the frozen ~7.4°/7.5° AHRS2 offset"
   is proven consistent with the numbers to 4 significant figures and
   with the code (`AP_NavEKF3_core.cpp:511`'s own "TODO we should
   average accel readings over several cycles" comment, `AP_NavEKF3_
   core.cpp:559`'s permanent `statesInitialised=true` latch, and R2C's
   passing session on identical code proving this is a race rather than
   a sustained bug). What remains unproven is the live *first
   divergence timestamp* and a same-boot causal chain from a specific
   SIM_JSON/PPP-pipeline event (e.g. `STATE_NOT_READY`, a PPP
   renegotiation) to that specific accel sample -- that requires the
   tool below to run across an actual reboot.

5. **Minimum fix**: not proposed as new -- unchanged from R3G's
   proposal, since nothing here weakens or strengthens it beyond
   "mathematically consistent": average `initAccVec` over the existing
   1-second accumulation window (`AP_NavEKF3_core.cpp:501-503`) instead
   of taking the single instantaneous sample at line 512. This remains
   a genuine EKF3 firmware change and is out of scope for this
   diagnostic task.

6. **Guarded rerun command** (read-only; run by the operator across
   their next manual power-cycle):

   ```
   cd Tools/autotest/sr75_hil_layer2/scripts
   python3 sr75_hil_f24r3g_ekf3_bootstrap_trace.py \
       --pixhawk /dev/ttyACM0 \
       --wait-for-device-timeout-s 120 \
       --heartbeat-timeout-s 60 \
       --capture-timeout-s 90 \
       --divergence-threshold-deg 1.0 \
       --match-tolerance-deg 2.0 \
       --output-csv ../hardware_sessions/r3h_bootstrap_trace_$(date -u +%Y%m%dT%H%M%SZ).csv \
       --summary-json ../hardware_sessions/r3h_bootstrap_trace_$(date -u +%Y%m%dT%H%M%SZ)_summary.json
   ```

   Start this command **before** power-cycling the Pixhawk (it waits
   for the device to disappear and reappear via the same `wait_for_
   devices()` used by the existing hardware orchestrator), then perform
   the manual power cycle. It prints `TRACE_RESULT
   first_divergence_time=... reproduces_bootstrap_math=...
   roll_residual_deg=... pitch_residual_deg=...` and a safety
   confirmation line on completion, and sends only `MAV_CMD_SET_
   MESSAGE_INTERVAL` requests -- no `PARAM_SET`, arm, mission, RC
   override, or actuator command.

## Files referenced

- `Tools/autotest/sr75_hil_layer2/scripts/sr75_hil_f24r3g_ekf3_bootstrap_trace.py` (new)
- `Tools/autotest/sr75_hil_layer2/scripts/test_sr75_hil_f24r3g_ekf3_bootstrap_trace.py` (new)
- `Tools/autotest/sr75_hil_layer2/scripts/sr75_hil_f24f_hardware_orchestrator.py` (`wait_for_devices()`, reused unmodified)
- `Tools/autotest/sr75_hil_layer2/reports/HIL_F24_R3G_ekf3_tilt_alignment_diagnosis.md`
- `libraries/AP_NavEKF3/AP_NavEKF3_core.cpp` (`InitialiseFilterBootstrap()`)
