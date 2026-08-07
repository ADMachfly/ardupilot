# HIL-F24-R3L: EKF_STATUS_REPORT bit 32768 in the R3K Trajectory (Scorer Fix)

Diagnosis plus a scorer-only fix, per this task's explicit constraint:
no EKF/estimator, firmware, or feeder code was touched.

Evidence session: `sr75_hil_f24r3a_mp_visual_20260807T080924Z`. Observed
`EKF_STATUS_REPORT.flags`: healthy earlier = 831, later = 33599 = 831 +
32768. All other checks pass (zero backward-timestamp rejects, GPS fix
stays 3, no attitude jump, all RMS metrics pass).

## 1/2. Exact meaning of bit 32768 -- file:function:line evidence

- `libraries/AP_NavEKF/AP_Nav_Common.h:41,64` -- `nav_filter_status`'s
  internal bit 14, `gps_glitching`: "true if GPS glitching is affecting
  navigation accuracy".
- `libraries/AP_NavEKF3/AP_NavEKF3_Control.cpp:815` -- where it's set:
  `status.flags.gps_glitching = !gpsAccuracyGood && (PV_AidingMode ==
  AID_ABSOLUTE) && (frontend->sources.getPosXYSource(core_index) ==
  AP_NavEKF_Source::SourceXY::GPS);`
- `libraries/AP_NavEKF3/AP_NavEKF3_VehicleStatus.cpp:299-317` -- what
  `gpsAccuracyGood` actually is: a **1-second-fail / 10-second-pass
  hysteresis** on normalised GPS position/velocity innovation ratios
  (`velTestRatio`/`posTestRatio` vs. 0.7/1.0 thresholds) ANDed with a
  GPS-speed-accuracy check -- a **self-clearing quality advisory**, not
  a state reset. It says nothing about whether the underlying
  attitude/velocity/position estimates are currently valid.
- `libraries/AP_NavEKF3/AP_NavEKF3_Outputs.cpp:608-647`
  (`send_status_report()`) -- the internal-to-MAVLink bit packing:
  `if (filterStatus.flags.gps_glitching) { flags |= (1<<15); }` --
  `(1<<15) = 32768` on the wire. This is the **only** place `bit 15` of
  the outgoing MAVLink flags is ever set; every other internal
  `nav_filter_status` bit not explicitly packed here (including
  `gps_quality_good`, a *different* internal bit that is never sent at
  all) is irrelevant to the wire value.
- `Tools/autotest/sr75_hil_layer2/scripts/sr75_hil_f24r2_estimator_comparison.py:41-48`
  (pre-fix): `EKF_UNHEALTHY_FLAGS = EKF_CONST_POS_MODE | EKF_UNINITIALIZED
  | EKF_GPS_GLITCHING` -- the scorer's own classifier, which lumped this
  advisory bit in with the two genuine reset/fault bits.

## 3. Why it appears during this trajectory

`SR75_hil_f24r3a_mp_visual_trajectory.xml`'s position/attitude/rate
functions are all smoothstep-shaped (`u=t/60` clamped to `[0,1]`,
`S(u)=3u²-2u³`). `S'(u)=6u-6u²` is zero at both `u=0` and `u=1`
(velocity is continuous and momentarily zero at the endpoints), but
`S''(u)=6-12u` is **discontinuous** at those same endpoints (+6 just
after `u=0`, -6 just before `u=1`) -- a real, by-design kinematic
acceleration step at the start and end of the 60s ramp. A momentary
acceleration/velocity-innovation spike at that boundary is exactly
what `velTestRatio`/`posTestRatio` are designed to catch, and the
1-second-fail hysteresis latches `gps_glitching=true` for at least a
second even though the underlying position/velocity/attitude estimates
never actually degrade (confirmed by 831's other bits staying set the
entire time, and by the RMS/attitude-jump checks passing throughout).

## 4. Whether the scorer is wrong

**Partially, yes -- but not simply "wrong all along."** The scorer's
`EKF_UNHEALTHY_FLAGS` including `EKF_GPS_GLITCHING` was a *deliberate,
evidence-based* decision from `HIL_F24_R2A_first_hardware_capture_
diagnosis.md`: in that earlier capture, the same bit correlated with a
**genuine** EKF3<->DCM backend flap (`STATUSTEXT "AHRS: EKF3 active"`/
`"AHRS: DCM active"`), a real ~2.5-million-metre position divergence,
and 5 real attitude jumps -- so gating on it there was correct.

The R3L evidence shows the *same bit* can also fire from an ordinary,
self-clearing innovation blip with **no** accompanying backend flap, no
position divergence, and no attitude jump. The bit alone is therefore
not a reliable discriminator between "genuine reset/instability" and
"momentary advisory" -- it needs to be judged by what else is
happening, not gated on unconditionally. Verified this doesn't lose
detection power for the original R2A scenario: that capture's ~2.5
million metre divergence and 5 attitude jumps are independently caught
by `horizontal_error_rms` and `no_large_attitude_jump` regardless of
this bit (both already covered by existing tests
`test_real_bench_ahrs_backend_switch_produces_large_finite_error_not_nan_or_clipped`
and `test_real_bench_ahrs_backend_switch_spike_flagged_as_in_and_out_jumps`).

## 5. Minimum scorer fix

`sr75_hil_f24r2_estimator_comparison.py`:

- `EKF_UNHEALTHY_FLAGS` now excludes `EKF_GPS_GLITCHING` (contains only
  `EKF_CONST_POS_MODE | EKF_UNINITIALIZED` -- the two bits that are
  genuinely "reset/never-healthy" conditions, not advisories).
- `scan_ekf_and_statustext_health()` now returns
  `(events, gps_glitch_events)`: `events` (unchanged meaning, still
  gates `no_ekf_or_statustext_reset_events`) no longer includes
  GPS-glitch occurrences; `gps_glitch_events` is a new, separate,
  **non-gating** list that still surfaces every GPS-glitch occurrence
  in `estimator_summary.json`'s `"gps_glitch_events"` field and in the
  markdown report's new "GPS-glitch advisory events" section -- nothing
  is silently dropped, it's just no longer misclassified as a reset.
- No threshold, EKF, parameter, or feeder change of any kind.

## 6. Regression test

`Tools/autotest/sr75_hil_layer2/scripts/test_sr75_hil_f24r2_estimator_comparison.py`
(34/34 passing):

- **`test_r3l_momentary_gps_glitch_flag_does_not_fail_an_otherwise_healthy_run`**
  (new): reproduces the exact R3L scenario end-to-end through
  `compute_summary()` -- a fully healthy synthetic run with one
  mid-run `EKF_STATUS_REPORT` row at the real captured value
  (`flags=33599`) and nothing else anomalous. Asserts
  `checks["no_ekf_or_statustext_reset_events"]` and `overall_pass` are
  both `True`, and that the glitch is still recorded in
  `summary["gps_glitch_events"]`.
- **`test_real_bench_glitch_flags_32807_and_33599_tracked_as_advisory_not_unhealthy`**
  (rewrite of the original R2A fixture test, same two real literal flag
  values from both captures): now asserts both values produce zero
  `events` and exactly one `gps_glitch_events` entry each, documenting
  in its docstring exactly why the earlier R2A classification doesn't
  generalize and why R2A's own scenario remains independently covered.
- `test_all_healthy_status_bits_together_not_misclassified_as_fault`
  and `test_unhealthy_flags_detected` updated for the tuple return
  signature; all other call sites (`TestR3aEkfWarmupGate`,
  `test_reset_statustext_detected`, `test_armed_during_run_detected`,
  etc.) updated identically.

`flake8 --max-line-length=200`, `py_compile`, and `git diff --check`
clean on both changed files. Full `sim_json`+`scripts` suite re-run
after this change to confirm zero regressions elsewhere.

## Files referenced

- `Tools/autotest/sr75_hil_layer2/scripts/sr75_hil_f24r2_estimator_comparison.py` (fix)
- `Tools/autotest/sr75_hil_layer2/scripts/test_sr75_hil_f24r2_estimator_comparison.py` (tests)
- `libraries/AP_NavEKF/AP_Nav_Common.h` (`nav_filter_status`)
- `libraries/AP_NavEKF3/AP_NavEKF3_Control.cpp` (`gps_glitching` assignment)
- `libraries/AP_NavEKF3/AP_NavEKF3_VehicleStatus.cpp` (`gpsAccuracyGood`/innovation hysteresis)
- `libraries/AP_NavEKF3/AP_NavEKF3_Outputs.cpp` (`send_status_report()`, MAVLink bit packing)
- `Tools/autotest/aircraft/sr_75_6_dof/scripts/SR75_hil_f24r3a_mp_visual_trajectory.xml` (smoothstep kinematics)
- `Tools/autotest/sr75_hil_layer2/reports/HIL_F24_R2A_first_hardware_capture_diagnosis.md` (original classification and its evidence, superseded for this bit only)
- `hardware_sessions/sr75_hil_f24r3a_mp_visual_20260807T080924Z/`
