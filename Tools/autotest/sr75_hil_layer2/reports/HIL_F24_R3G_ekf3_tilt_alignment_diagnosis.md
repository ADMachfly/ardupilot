# HIL-F24-R3G: EKF3-vs-Primary Attitude Divergence (~7.4°/7.5° Roll/Pitch)

Diagnosis only, per this task's explicit constraint: no firmware,
parameter, threshold, control, or trajectory code was changed. All
hardware interaction was read-only (`PARAM_REQUEST_READ`,
`MAV_CMD_GET_HOME_POSITION`, `MAV_CMD_REQUEST_MESSAGE`), reusing the
existing `sr75_hil_f24r2b_ahrs_origin_diagnostics.py`/`sr75_hil_gps_ekf_
readonly_audit.py` unmodified.

Session diagnosed:
`Tools/autotest/sr75_hil_layer2/hardware_sessions/sr75_hil_f24r3a_mp_visual_20260807T040401Z`
(`r3a_premotion_gps_ekf.csv`/`_summary.json`, `origin_relatch.log`/
`_summary.json`), compared against the previously-successful
`sr75_hil_f24r2c_relatch_20260806T083459Z`.

## 1. `AP_AHRS::attitudes_consistent()` -- which two attitudes/backends

`libraries/AP_AHRS/AP_AHRS.cpp:2704-2799`. Compares:

- **`primary_quat`** (line 2708, `get_quat_body_to_ned()`) -- the
  vehicle's *active* attitude source. Live evidence: `"PreArm: AHRS: not
  using configured AHRS type"` (this and every prior R2B/R3E session)
  means `active_EKF_type()` is currently **DCM**, not the configured
  EKF3 -- so `primary_quat` here is DCM's quaternion, matching the
  primary `ATTITUDE` MAVLink message (~0.10°/0.09°, level).
- **`EKF3.getQuaternionBodyToNED(i, ekf3_quat)`** (line 2745), for each
  EKF3 core, entered because `configured_ekf_type() == EKFType::THREE`
  (`AHRS_EKF_TYPE=3`) at line 2742's `||` condition, even though EKF3 is
  not the *active* type.
- Line 2748-2751: `rp_diff_rad = primary_quat.roll_pitch_difference(ekf3_quat)`;
  if it exceeds `ATTITUDE_CHECK_THRESH_ROLL_PITCH_RAD`,
  `set_failure_inconsistent_message("EKF3", "Roll/Pitch", ...)` -- this
  is the exact, and only, source of `"PreArm: AHRS: EKF3 Roll/Pitch
  inconsistent 10 deg."`, captured verbatim in this session's
  `origin_relatch.log`.

So the comparison is **DCM (primary, level) vs. EKF3's own raw
quaternion (secondary, ~7.4°/7.5°)** -- not two EKF3 cores against each
other, and not DCM against itself.

## 2. What AHRS2 represents on this build

`libraries/GCS_MAVLink/GCS_Common.cpp:617-633` (`send_ahrs2()`) sends
`ahrs.get_secondary_attitude(euler)` --
`libraries/AP_AHRS/AP_AHRS.cpp:1313-1343` (`_get_secondary_attitude()`),
which switches on `_get_secondary_EKF_type()`
(`AP_AHRS.cpp:2244-2270`):

```cpp
case EKFType::DCM:                                   // active_EKF_type() == DCM (our case)
    if ((EKFType)_ekf_type.get() == EKFType::THREE) { // configured AHRS_EKF_TYPE == 3
        secondary_ekf_type = EKFType::THREE;
        return true;
    }
```
then (`AP_AHRS.cpp:1338-1343`): `case EKFType::THREE: EKF3.getEulerAngles(eulers); return _ekf3_started;`

**AHRS2 = EKF3's own raw attitude estimate**, reported as "secondary"
specifically because DCM is currently the *active/primary* backend.
This is confirmed numerically, not just by code reading: AHRS2
(7.365185081095156°, 7.547061919635194°) is **byte-identical** to the
EKF3 quaternion `attitudes_consistent()` compares against (implied by
the STATUSTEXT firing at ~10.4° difference from a level primary) across
all 12 AHRS2 samples in the capture -- i.e. AHRS2 *is* the EKF3 attitude
involved in the inconsistency check.

## 3/4/5. Startup/tilt-alignment trace, sensor selection, and the first divergence point

**Strongest proven narrowing** (not yet 100%-confirmed at the exact
sample level -- see the diagnostic proposed below):

`libraries/AP_NavEKF3/AP_NavEKF3_core.cpp:469-571`
(`InitialiseFilterBootstrap()`), the *only* place EKF3 ever computes its
initial roll/pitch:

```cpp
475:  if (assume_zero_sideslip() && dal.gps().status(preferred_gps) < AP_GPS_FixType::FIX_3D) {
          ...; statesInitialised = false; return false;   // blocks entirely until FIX_3D
      }
498-503:  if (firstInitTime_ms == 0) { firstInitTime_ms = imuSampleTime_ms; return false; }
          else if (imuSampleTime_ms - firstInitTime_ms < 1000) { return false; }
511-512:  // TODO we should average accel readings over several cycles
          initAccVec = dal.ins().get_accel(accel_index_active).toftype();
514-524:  initAccVec.normalize(); pitch = asinF(initAccVec.x); roll = atan2F(-initAccVec.y, -initAccVec.z);
527:  stateStruct.quat.from_euler(roll, pitch, 0.0f);
559:  statesInitialised = true;   // latched permanently for this boot
567:  GCS_SEND_TEXT(MAV_SEVERITY_INFO, "EKF3 IMU%u initialised", ...);
```

This is a **single, un-averaged accelerometer sample** (the code's own
comment at line 511 flags this as a known simplification), taken exactly
~1000ms after EKF3's first bootstrap attempt, gated on GPS already
reporting `FIX_3D`. Working the frozen AHRS2 angles backward through
this exact formula gives a normalized accel vector of approximately
`(0.131, -0.127, -0.983)` -- a real, ~1g-magnitude vector tilted ~10°
from vertical, **not** zero/garbage/out-of-range. This is consistent
with a genuine (if transient) off-level accelerometer sample being
latched -- most plausibly during a startup transient in the SIM_JSON/PPP
pipeline right after the manual reboot (a `STATE_NOT_READY` moment, a
PPP renegotiation blip, or simply a sample taken before the feeder's
first fully-settled row) -- rather than a sustained real tilt, since the
live truth and every subsequent RAW_IMU/SCALED_IMU/SCALED_IMU2 sample in
this same capture read essentially exactly level (`xacc=0, yacc=0,
zacc=-1001`).

Once `statesInitialised=true` latches (line 559), this bad seed is never
revisited within that boot -- consistent with `flags` staying `1024`
(`EKF_UNINITIALIZED`, "has never been healthy") for the *entire* 21s
capture and, per the prior R3E diagnosis, the entire boot. **First
divergence point: `AP_NavEKF3_core.cpp:512`'s single accelerometer
sample read, at whatever wall-clock moment was exactly
`firstInitTime_ms + 1000ms` after this boot's first
`InitialiseFilterBootstrap()` call** -- not observable after the fact
from this capture alone, since the capture began after boot, not at
`t=0`.

**Checked, not implicated**: GPS/IMU instance selection.
`preferred_gps` (`AP_NavEKF3_core.h:1652`, set in
`AP_NavEKF3_Measurements.cpp:1178/1184`) can only resolve to instance 0
here since `GPS2_TYPE=0` (no second GPS configured). `SCALED_IMU`
(instance 0) and `SCALED_IMU2` (instance 1) report **identical** values
in this capture (`xacc=0, yacc=0, zacc=-1001` both), arguing against a
second-IMU-instance-specific corruption or a stale/unfed second
instance. Board orientation/rotation is applied inside
`AP_InertialSensor` before either backend sees the vector, so a
per-backend rotation mismatch would require EKF3 to read a genuinely
different instance than DCM/primary -- not supported by the identical
SCALED_IMU/SCALED_IMU2 readings.

## 6. Comparison with the passing R2C session

`sr75_hil_f24r2c_relatch_20260806T083459Z/pixhawk_estimator.csv`:
`ekf_flags=831` for all 63 samples (`ATTITUDE(1)|VELOCITY_HORIZ(2)|
VELOCITY_VERT(4)|POS_HORIZ_REL(8)|POS_HORIZ_ABS(16)|POS_VERT_ABS(32)|
PRED_POS_HORIZ_REL(256)|PRED_POS_HORIZ_ABS(512) = 831`, every "good" bit
set, no fault bit) -- a **fully healthy** EKF3, and that same session's
`origin_relatch_summary.json` shows `GPS_GLOBAL_ORIGIN` matched truth to
0.0094 m. The *only* difference between that boot and this one is which
way this exact non-deterministic race (`InitialiseFilterBootstrap()`'s
un-averaged first sample, raced against the SIM_JSON/PPP pipeline's own
post-reboot settling) happened to resolve -- both sessions use identical
code, parameters, and trajectory. This is two outcomes of the same race,
not two different bugs.

## Minimum safe fix proposal (not implemented)

No firmware/parameter/threshold change is proposed for *this*
diagnostic. The safest, smallest fix consistent with "average accel
readings over several cycles" (the code's own acknowledged TODO at
`AP_NavEKF3_core.cpp:511`) would average `initAccVec` over several IMU
samples spanning the 1-second accumulation window already being waited
on (lines 501-503), rather than a single instantaneous sample at line
512 -- directly reducing sensitivity to exactly the kind of momentary
startup transient implicated here. This is a genuine EKF3 firmware
change and is explicitly out of scope to apply in this task.

## Minimum read-only diagnostic needed to move from "strongest proven narrowing" to "proven"

Capture MAVLink traffic (`AHRS2`, `EKF_STATUS_REPORT`, `STATUSTEXT`,
`GPS_RAW_INT`) starting from the moment device reattach is confirmed
after the *next* relatch reboot (i.e., extend
`sr75_hil_f24r2b_ahrs_origin_diagnostics.py`'s or the orchestrator's
existing read-only listen window to begin immediately post-reattach,
before PPP/SIM_JSON even reconnect) through to the first `"EKF3 IMU%u
initialised"` STATUSTEXT (currently uncaptured in every session
inspected -- it either predates every capture window or was never
observed). If `AHRS2` is level for the first several seconds and then
snaps to a fixed offset at an identifiable timestamp, correlate that
timestamp against `responder.csv`/`state.csv` for a concurrent
`STATE_NOT_READY`/stale-state/PPP-renegotiation event. This is
read-only, requires only listening for a few extra seconds after
reattach, and needs no code change to run once (a slightly earlier
`--listen-s` start point in the existing diagnostic).

## Files referenced

- `libraries/AP_AHRS/AP_AHRS.cpp` (`attitudes_consistent()`,
  `_get_secondary_attitude()`, `_get_secondary_EKF_type()`)
- `libraries/GCS_MAVLink/GCS_Common.cpp` (`send_ahrs2()`)
- `libraries/AP_NavEKF3/AP_NavEKF3_core.cpp`
  (`InitialiseFilterBootstrap()`)
- `libraries/AP_NavEKF3/AP_NavEKF3_Measurements.cpp` (`preferred_gps`)
- `hardware_sessions/sr75_hil_f24r3a_mp_visual_20260807T040401Z/*`
- `hardware_sessions/sr75_hil_f24r2c_relatch_20260806T083459Z/*`
