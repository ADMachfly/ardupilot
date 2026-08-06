# HIL-F24-R2A: Diagnosis of the First R2 Hardware Capture Failure

Session analyzed: `sr75_hil_layer2/hardware_sessions/sr75_hil_f24r2_estimator_20260806T050616Z`
(`estimator_summary.md`: Overall **FAIL**, first failing check
`no_ekf_or_statustext_reset_events`, alignment coverage 100.0%.)

No parameters were tuned, no firmware/control/PPP/JSBSim code was
modified. This is diagnosis plus regression tests only.

## First causal mismatch

**A real ArduPilot AHRS-backend transition on the Pixhawk (EKF3 → DCM →
EKF3), first occurring at t≈3273.06s**, is the single root event behind
every failing check in this run. It is not a bug in the R2 capture or
comparison scripts. Evidence chain, in order:

1. `t=3272.903`: the very first `EKF_STATUS_REPORT` sample already
   carries `flags=32807`, which includes `EKF_GPS_GLITCHING` (bit
   32768) -- ArduPilot's own real-time judgment that GPS input looked
   faulty, present continuously for the rest of the run (63 of 64
   `EKF_STATUS_REPORT` samples show `flags=33599`, the other 1 shows
   `32807` -- both contain bit 32768).
2. `t=3273.063`: `STATUSTEXT` `"AHRS: EKF3 active"` -- a genuine
   ArduPilot backend-selection event, reported by the firmware itself.
3. `t=3273.144`: `GLOBAL_POSITION_INT` and `ATTITUDE` both diverge
   sharply from truth in the very next sample after that transition
   (see Task 1/3 below), then smoothly reconverge over ~1.5s -- the
   classic signature of DCM's own dead-reckoned position/attitude
   estimate being reported while EKF3 re-initializes/re-converges.
4. `t=3294.104`: `STATUSTEXT` `"PreArm: GPS and AHRS differ by
   2525060.m"` -- ArduPilot's own internal healthcheck, printed at a
   moment when `GLOBAL_POSITION_INT` read `(17.7736, 93.4691)` against
   a truth/GPS position of `(32.5378, 74.3662)`. Recomputing that same
   distance independently (haversine) gives **2,520,218 m** -- within
   0.3% of the firmware's own printed figure, directly confirming the
   firmware's own diagnosis: at that instant, "AHRS" (then reporting
   the DCM backend's stale estimate) and "GPS" (the correct, truth-fed
   fix) disagreed by ~2.5 million metres.
5. Three more `"AHRS: DCM active"` / `"AHRS: EKF3 active"` pairs occur
   later in the run (`t≈3283.04/3283.10`, `t≈3293.12/3293.22`,
   `t≈3303.35/3303.51`), each one coinciding exactly with one of the
   remaining 4 flagged attitude jumps (Task 3).

Everything the R2 comparison script flagged -- the large
`horizontal_error_m`, the 5 attitude jumps, and the
`no_ekf_or_statustext_reset_events` failure -- are correctly-reported
symptoms of this one real hardware/firmware behavior, not
comparison-script defects. `GPS_RAW_INT` (the primary GPS instance)
stayed rock-steady and correct the entire run (`fix_type=3`, 15
satellites, `lat/lon` exactly matching truth for all 159 samples), so
the divergence is specifically in the momentarily-active DCM backend's
own position/attitude output, not a second/rogue GPS source.

## Task 1: horizontal-position divergence

First divergent `estimator_comparison.csv` row (row index 2):

```
host_monotonic_time=3273.143953256  truth_host_monotonic_time=3273.143732039  alignment_dt_s=0.000221 (well under the 100ms tolerance)
horizontal_error_m=5675035.589336136   altitude_error_m=-0.000396 (essentially zero)
```

Aligned truth vs. `GLOBAL_POSITION_INT`, printed directly from the raw
CSVs:

| Field | Truth (`jsbsim_truth.csv`) | GLOBAL_POSITION_INT (`pixhawk_estimator.csv`) |
|---|---|---|
| t | 3273.143732039 | 3273.143953256 |
| lat | 32.5378085292243 | **-1.8481856** |
| lon | 74.3661943653382 | **113.4745742** |
| alt | 0.170395741 m | 0.17 m (matches) |

Scaling/origin/field check:

- **Scaling**: `gpi_lat_deg = msg.lat * 1.0e-7`, `gpi_lon_deg = msg.lon
  * 1.0e-7` in `mavlink_message_to_row()` -- standard MAVLink degE7
  scaling, unchanged from R1/R2's validated field mapping. Confirmed
  correct: the two GLOBAL_POSITION_INT samples immediately *before*
  this one (`t=3272.903`, `t=3273.043`) read `32.5378085, 74.3661943`
  -- an exact match to truth, using the identical scaling code path.
  If scaling were wrong, every sample would be wrong, not just this
  one.
- **Origin/datum**: altitude stayed sane and continuous through the
  entire divergence (`0.2 → 0.17 → 0.17 → 0.11 → ...`, never jumping),
  ruling out a wholesale field-offset/packing bug -- a byte-alignment
  or origin-remap bug would be expected to corrupt altitude, velocity,
  or heading too, not lat/lon alone.
- **Distance calculation**: `horizontal_distance_m()`'s equirectangular
  approximation is correct at this scale to within 0.3% (cross-checked
  against haversine for the `t=3294.1` sample above); it is not the
  source of the anomaly -- the *input* positions themselves are what
  diverge, not the distance math.
- **Conclusion**: the mismatch is **not** datum/origin/field/scaling
  related. It is a real, momentary flight-controller-side position
  estimate (from the DCM AHRS backend, confirmed by the STATUSTEXT
  correlation above), decoded, scaled, and compared correctly by the
  R2 scripts.

## Task 2: EKF_STATUS_REPORT flags 32807 / 33599, bit-by-bit

Decoded against the MAVLink `EKF_STATUS_FLAGS` enum as defined in the
pymavlink/ArduPilot dialect actually loaded by this build:

| Bit | Name | Meaning (from the dialect) | Set in 32807? | Set in 33599? |
|---|---|---|---|---|
| 1 | `EKF_ATTITUDE` | attitude estimate is **good** | yes | yes |
| 2 | `EKF_VELOCITY_HORIZ` | horiz. velocity estimate is **good** | yes | yes |
| 4 | `EKF_VELOCITY_VERT` | vert. velocity estimate is **good** | yes | yes |
| 8 | `EKF_POS_HORIZ_REL` | horiz. rel. position estimate is **good** | no | yes |
| 16 | `EKF_POS_HORIZ_ABS` | horiz. abs. position estimate is **good** | no | yes |
| 32 | `EKF_POS_VERT_ABS` | vert. abs. position estimate is **good** | yes | yes |
| 64 | `EKF_POS_VERT_AGL` | vert. AGL estimate is good | no | no |
| **128** | `EKF_CONST_POS_MODE` | **FAULT**: unknown abs/rel position | no | no |
| 256 | `EKF_PRED_POS_HORIZ_REL` | predicted horiz. rel. position is **good** | no | yes |
| 512 | `EKF_PRED_POS_HORIZ_ABS` | predicted horiz. abs. position is **good** | no | yes |
| **1024** | `EKF_UNINITIALIZED` | **FAULT**: EKF has never been healthy | no | no |
| **32768** | `EKF_GPS_GLITCHING` | **FAULT**: EKF believes GPS input is faulty | **yes** | **yes** |

Bits 1, 2, 4, 8, 16, 32, 64, 256, 512 are each documented `"Set if ...
is good"` in the MAVLink common dialect -- they are **healthy-status**
bits, not fault indicators. Only 128 (`EKF_CONST_POS_MODE`), 1024
(`EKF_UNINITIALIZED`), and 32768 (`EKF_GPS_GLITCHING`) are genuine
fault bits.

`sr75_hil_f24r2_estimator_comparison.py`'s classifier:

```python
EKF_CONST_POS_MODE = 128
EKF_UNINITIALIZED = 1024
EKF_GPS_GLITCHING = 32768
EKF_UNHEALTHY_FLAGS = EKF_CONST_POS_MODE | EKF_UNINITIALIZED | EKF_GPS_GLITCHING  # = 33920
```

**Verified correct -- no fix required.** `EKF_UNHEALTHY_FLAGS`
contains exactly the 3 genuine fault bits and none of the 9 healthy
bits (`(1|2|4|8|16|32|64|256|512) & 33920 == 0`, locked in by
`test_all_healthy_status_bits_together_not_misclassified_as_fault`,
below). Both 32807 and 33599 are correctly classified unhealthy
*specifically and only* because both set bit 32768
(`EKF_GPS_GLITCHING`); every other set bit in either value is a
healthy/good-status bit and does not contribute to the classification.
33599's four extra bits over 32807 (8, 16, 256, 512) mark the position
estimate transitioning from "not yet available" to "available" as the
filter re-stabilizes -- also healthy bits, not additional faults.

## Task 3: the five yaw jumps, ±3 neighbors

Truth yaw is essentially constant at 315.0° (JSBSim ground-static,
never rotates) for the entire run, which rules out a legitimate
±180°/360° wrap crossing as an explanation for any of these -- there is
no wrap event in the truth data to be near. `ATTITUDE` neighbors around
each flagged jump (`att_roll_deg`/`att_pitch_deg`/`att_yaw_deg`):

```
t=3273.143890  yaw=-47.35  roll=0.28  pitch=-1.54   <- FIRST jump (sustained ~1.5s reconvergence, see Task 1)
t=3273.244574  yaw=-47.20  roll=0.26  pitch=-1.55
t=3273.345041  yaw=-46.99  roll=0.22  pitch=-1.58
  (baseline before: t=3272.902859 yaw=-34.35 roll=0.11 pitch=-1.72; t=3273.043340 yaw=-34.35 roll=0.11 pitch=-1.70)

t=3293.095585  yaw=-49.46  roll=-0.127  pitch=-2.042   <- baseline (EKF3)
t=3293.196462  yaw=-34.38  roll= 0.101  pitch=-1.730   <- SPIKE (single sample; STATUSTEXT: "AHRS: DCM active" @3293.116, "AHRS: EKF3 active" @3293.217 bracket it)
t=3293.296995  yaw=-49.46  roll=-0.124  pitch=-2.037   <- reverts to baseline on the very next sample

t=3303.308513  yaw=-49.23  roll= 0.125  pitch=-1.725   <- baseline (EKF3)
t=3303.389074  yaw=-34.38  roll= 0.101  pitch=-1.726   <- SPIKE (single sample; STATUSTEXT: "AHRS: DCM active" @3303.349, "AHRS: EKF3 active" @3303.510 bracket it)
t=3303.509957  yaw=-49.22  roll= 0.132  pitch=-1.717   <- reverts to baseline on the very next sample
```

**Determination: real discontinuities caused by AHRS backend switching
(DCM ↔ EKF3) on the Pixhawk, not EKF yaw resets, not wrap artifacts, and
not comparison-script alignment artifacts.**

- **Not wrap**: truth yaw never approaches ±180°/360°; `yaw_error_deg()`
  is not involved in producing these values (`detect_attitude_jumps()`
  operates on raw Pixhawk-to-Pixhawk consecutive deltas, wrap-aware,
  and correctly did not flag the much larger but legitimate transition
  the wrap-aware helper is designed for -- see the pre-existing
  `test_yaw_wrap_not_mistaken_for_jump`).
- **Not alignment artifact**: `alignment_dt_s` for the comparison row
  nearest the first jump is 0.22ms, far inside the 100ms tolerance;
  the jump detector itself doesn't even use truth/alignment at all, it
  only compares consecutive real `ATTITUDE` samples.
- **Not an EKF "reset"** in the sense of a persistent re-initialization
  (covariance reset, origin reset): the exact same numeric signature
  (yaw≈-34.3° to -47.4°, roll≈0.10-0.28°, pitch≈-1.53 to -1.73°)
  recurs at all 3 later spikes and reverts to a *stable, unchanged*
  baseline (-49.2° to -49.5°) on the very next sample every time --
  consistent with DCM's own (different, roughly-constant) attitude
  solution being reported for exactly one sample each time the backend
  toggles, not a persistent state change.
- The first (t≈3273.14) event is qualitatively different from the
  other three: it is a *sustained* ~1.5s reconvergence rather than a
  single-sample spike-and-revert, because it corresponds to EKF3's
  initial re-activation (`"AHRS: EKF3 active"` at 3273.063, no prior
  `"DCM active"` message captured -- DCM was presumably already active
  before this session's capture window started) followed by EKF3's own
  convergence, whereas the later three are brief DCM flickers bracketed
  by both transition messages.
- 5 flagged jumps = **3 distinct real events** (one sustained EKF3
  reactivation + 2 brief DCM flickers), not 5 independent faults: a
  single spike-and-revert sample necessarily produces 2 flagged
  transitions (into the spike, then back out) by design of a
  consecutive-delta detector -- this is documented and pinned down by
  the new regression test below, not a bug.

## Task 4: the single responder `state_read_error` / missed reply

`responder.log`:

```
STATE_NOT_READY (no valid state yet, total_state_read_errors=1): state file does not exist: <session_dir>/state.csv
...
RUN_SUMMARY total_requests=1410 replies_sent=1409 stale_state_count=0 no_fresh_state_count=0 malformed_state_count=0 state_read_error_count=1 missed_replies=1
```

`responder.csv` row 0 (the exact, only, first row):

```
request_count=1  request_received_host_time=3272.518803364
reply_sent=0
reply_reason='state file does not exist: <session_dir>/state.csv'
error_reason='state file does not exist: <session_dir>/state.csv'
```

Row 1 (the very next request, `request_count=2`) already has
`reply_sent=1`, `reply_reason='OK'`, `state_age_ms=0.234` -- fully
healthy from the second request onward.

**Exact exception**: `sr75_sim_json_responder.StateFileNotReadyError`,
raised at `sr75_sim_json_responder.py:461` with message `"state file
does not exist: {self.path}"`. This is the exact, already-documented
HIL-F24-P startup race: `state.csv` did not exist yet when the very
first SIM_JSON request from the Pixhawk arrived (`t=3272.519`, before
the R1 feeder's first write). The responder's designed behavior for
this case is to correctly decline to reply (never send a stale/garbage
frame) and count it as `state_read_error_count`, not
`malformed_state_count` -- exactly what happened.

**Determination: not a bug.** `state_read_error_count` (1) and
`missed_replies` (1) match exactly, `total_requests` (1410) −
`replies_sent` (1409) = 1, consistent accounting with no discrepancy.
This exact scenario is already covered by a pre-existing regression
test: `sim_json/test_sr75_hil_f24p_state_csv_race.py::
TestLatestCSVReader::test_raises_state_file_not_ready_when_never_written`.
No new test added for this item -- coverage already exists.

## Task 5: regression tests added

All added to `Tools/autotest/sr75_hil_layer2/scripts/
test_sr75_hil_f24r2_estimator_comparison.py` (no code in
`sr75_hil_f24r2_estimator_comparison.py` itself changed -- every check
above verified the existing logic correct; these tests lock that
verification in against future regressions, using literal values from
the real incident as fixtures):

1. **`TestHealthEventScanning::test_all_healthy_status_bits_together_
   not_misclassified_as_fault`** -- sets all 9 documented "good" bits
   simultaneously (1|2|4|8|16|32|64|256|512 = 911) with no fault bit
   set; asserts `EKF_UNHEALTHY_FLAGS` shares none of those bits and
   `scan_ekf_and_statustext_health()` reports no event. Directly
   answers task 2's "fix the classifier if it incorrectly treats
   positive flags as faults" -- it doesn't, and this pins that down.
2. **`TestHealthEventScanning::test_real_bench_glitch_flags_32807_and_
   33599_detected_via_gps_glitching_bit`** -- the exact two real
   captured flag values; asserts both are classified unhealthy
   specifically via the `EKF_GPS_GLITCHING` bit.
3. **`TestAttitudeJumpDetection::test_real_bench_ahrs_backend_switch_
   spike_flagged_as_in_and_out_jumps`** -- the exact real
   roll/pitch/yaw values from the `t=3293.10-3293.40` spike-and-revert;
   asserts exactly 2 jumps are flagged with equal-and-opposite yaw
   deltas (within the real baseline's own small drift), documenting
   that this is expected 1-event→2-flags behavior, not a
   double-counting bug.
4. **`TestAlignAndCompare::test_real_bench_ahrs_backend_switch_
   produces_large_finite_error_not_nan_or_clipped`** -- the exact real
   truth/GPI values from the `t=3273.14` divergence; asserts
   `horizontal_error_m` is finite (never NaN/Inf) and matches the real
   captured ~5.68-million-metre value within 1%, confirming no
   datum/scaling logic silently clips or corrupts a genuine large
   divergence.

No change to `DEFAULT_THRESHOLDS` or any other provisional threshold.

### Test results

```
$ python3 -m pytest scripts/test_sr75_hil_f24r2_estimator_comparison.py -v
29 passed, 2 subtests passed   (25 pre-existing + 4 new, all pass)

$ python3 -m pytest sim_json scripts -q
532 passed, 19 subtests passed, 2 failed
```

The 2 failures (`test_sr75_hil_f24s_coherence_and_accounting.py::
TestShutdownAccountingAtomicity::test_burst_then_immediate_sigterm_
never_mismatches`, `test_sr75_hil_f24t_monotonic_freshness.py::
TestAccountingUnderSigtermStillExact::test_burst_then_immediate_
sigterm_still_exact`) are **pre-existing and unrelated** -- confirmed
by re-running them with this task's changes fully `git stash`ed: they
fail identically against the untouched baseline (a timing-sensitive
burst+immediate-SIGTERM subprocess race, reproducing under current
system load, in files this task did not touch).

`flake8 --max-line-length=200`, `py_compile`, and `git diff --check`
on the modified test file are all clean.

## Rerun command

No code fix was made, so nothing needs to be re-run against hardware
to validate a fix. To re-run the new regression tests alone:

```sh
python3 -m pytest Tools/autotest/sr75_hil_layer2/scripts/test_sr75_hil_f24r2_estimator_comparison.py -v
```

To capture a fresh bench run for comparison against this one (unchanged
from the R2 report's guarded command):

```sh
python3 Tools/autotest/sr75_hil_layer2/scripts/sr75_hil_f24f_hardware_orchestrator.py \
    --profile dynamic --stage r2 --duration-s 30 \
    --execute --confirm-estimator-comparison
```

## Files changed

- `Tools/autotest/sr75_hil_layer2/scripts/test_sr75_hil_f24r2_estimator_comparison.py`
  -- 4 new tests (listed above), no other changes.
- `sr75_hil_f24r2_estimator_comparison.py` and
  `sr75_hil_f24r2_estimator_capture.py`: **unchanged** -- every check
  investigated was verified correct.

## Note for the operator (informational, not implemented here)

The recurring `EKF_GPS_GLITCHING` flag and periodic DCM fallback appear
tied to how the bench's real GPS/AHRS health disagreement is triggered
-- worth investigating on the bench/firmware-configuration side (e.g.
whether `GPS_TYPE`/EKF source-selection parameters are appropriate for
a SIM_JSON-fed, no-antenna bench setup). Per this task's explicit
constraint, no parameter was changed and none is recommended here --
this is a pointer for whoever picks up the next task, not an action
taken.
