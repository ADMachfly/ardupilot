# HIL-F23-F1: SR-75 JSBSim-to-Pixhawk Estimator Injection Architecture Audit

Status: **audit only** — no code changes, no parameter changes, no arming, no AUTO,
no actuator output, no CH7/CH8 activity. All findings below are grounded in this
exact repository's source (`~/ardupilot_clean`, branch
`sr75-layer2j-f21-full-auto-mission`) as of this audit, not general ArduPilot
documentation.

## 1. Context: validated state and the observed gap

F23-E2B (PASS) validated a dynamic, open-loop, disarmed hardware feed: live
JSBSim GPS position/velocity and airspeed reach the Pixhawk and are genuinely
fused (GPS horizontal position/velocity track JSBSim, mission-derived heading
alignment error ≈0.1°, `GPS1_TYPE=14`). The gap observed in that same run:

- JSBSim pitch reached ≈-77.6°; the *custom* `ATTITUDE` MAVLink message
  followed it, but `AHRS2` pitch stayed near the physical Pixhawk's real
  attitude (bench sitting on a table).
- JSBSim altitude ≈2118 m; `GPS_RAW_INT` altitude ≈2189 m (close — GPS-path
  altitude, as expected); `GLOBAL_POSITION_INT`/`VFR_HUD` altitude ≈2520 m,
  because `EK3_SRC1_POSZ=1` (BARO) — that altitude is the physical barometer
  reading, not JSBSim's.

This is the estimator gap this audit exists to explain and scope a fix for.

## 2. Audit of available real-hardware simulation paths

### 2.1 HIL_SENSOR + HIL_GPS, and HIL_STATE_QUATERNION — REJECTED, not implemented at all

Grepped the entire source tree (`libraries/`, `ArduPlane/`, all `.cpp`/`.h`,
excluding the MAVLink-generated message headers themselves) for
`MAVLINK_MSG_ID_HIL_STATE`, `MAVLINK_MSG_ID_HIL_STATE_QUATERNION`,
`MAVLINK_MSG_ID_HIL_GPS`, `MAVLINK_MSG_ID_HIL_SENSOR`:

```
$ grep -rn "MAVLINK_MSG_ID_HIL_STATE\b\|MAVLINK_MSG_ID_HIL_STATE_QUATERNION\|MAVLINK_MSG_ID_HIL_GPS\|MAVLINK_MSG_ID_HIL_SENSOR" libraries/ ArduPlane/
(no output)
```

Zero matches, in **either** SITL or hardware build code. A second, broader pass
for the literal strings `HIL_STATE`/`HIL_SENSOR`/`HIL_GPS` across the whole repo
found exactly two non-generated hits: a stale doc-comment in
`libraries/AP_GPS/AP_GPS_MAV.cpp:40` ("handles an incoming mavlink message
(HIL_GPS)...") describing what the file *used to* handle — the actual code in
that file only handles `MAVLINK_MSG_ID_GPS_INPUT` (confirmed by reading the
function) — and our own `sr75_jsbsim_pixhawk_hil_bridge.py`'s `HILInjector`
class, which sends these messages into a firmware that has no handler for them.

**Conclusion: `HIL_STATE`, `HIL_STATE_QUATERNION`, `HIL_GPS`, and `HIL_SENSOR`
support was removed from this ArduPilot version entirely.** This holds
regardless of `AHRS_EKF_TYPE`/`EK3_ENABLE` (task item 5): these parameters
govern which EKF core runs, not whether a MAVLink message has a dispatch
handler. Since `GCS_MAVLink`'s message switch has no `case` for these message
IDs anywhere in the tree, they are silently dropped on arrival on **both**
SITL and the real fmuv3 build — the bridge's `HILInjector` code path is dead
weight against this firmware version. **Do not use it.**

### 2.2 Simulation-on-Hardware (SoH) / SIM_JSON on real firmware — theoretically correct, requires a new firmware image

`libraries/SITL/SIM_JSON.{h,cpp}` implements the JSON/UDP physics-injection
protocol (same one the F22-GZ SITL series used against `ArduPlane.elf`). It is
gated by `AP_SIM_JSON_ENABLED` (`libraries/SITL/SIM_config.h`), which in turn
depends on `AP_SIM_ENABLED` — normally only set for `CONFIG_HAL_BOARD ==
HAL_BOARD_SITL`.

There **is** an officially-supported way to get this compiled into real
ChibiOS firmware: `libraries/AP_HAL_ChibiOS/hwdef/include/SimOnHW.inc` sets
`env SIM_ENABLED 1` and disables a long list of unrelated features
(`HAL_NAVEKF2_AVAILABLE 0`, `HAL_ADSB_ENABLED 0`, etc.). Pre-built hwdef
targets exist for **`CubeOrange-SimOnHardWare`** and
**`CubeOrangePlus-SimOnHardWare`** only. **There is no `fmuv3-SimOnHardWare`
hwdef target** — `libraries/AP_HAL_ChibiOS/hwdef/fmuv3/hwdef.dat` has zero
`SIM_ENABLED`/`AP_SIM`-related lines. `Tools/scripts/sitl-on-hardware/` (the
generic build+flash helper, documented for MatekH743) can in principle target
an arbitrary board+`--simclass`+`--frame` combination without a pre-baked
hwdef folder — this repo's own
`Tools/autotest/sr75_hil_layer2/ppp/README.md` already documents exactly this
for fmuv3 (`sitl-on-hw.py --board fmuv3 --vehicle plane --simclass JSON
--frame json:192.168.144.2 --enable-PPP`), plus a full PPP-over-TELEM1 wiring
and parameter plan.

When it runs, SoH genuinely replaces the **real** IMU/baro/compass HAL
backends with `AP_InertialSensor_SITL`/`AP_Baro_SITL`/`AP_Compass_SITL`, so
JSBSim state placed into the SIM_JSON socket is fused by the *actual* running
EKF3 exactly as it is in plain SITL — this is the only path in this audit that
gives genuinely coherent accel/gyro/baro/compass/GPS fusion on real hardware.

**Why this is not the current recommendation, despite being architecturally
correct:** it requires building and flashing an **entirely different firmware
image** to the bench Pixhawk (not the currently-flashed, already-validated
firmware), a **full parameter wipe** (`FORMAT_VERSION=0` / `wipe_parameters`)
per `Tools/scripts/sitl-on-hardware/README.md`, and PPP/TELEM1 rewiring — none
of which are achievable under this task's "no code changes, no parameter
changes" constraint, and none of which should be attempted without a
dedicated, explicitly-authorized task (this is a hard-to-reverse, board-level
change, not a bench-software change). It also has not been validated against
this specific fmuv3 board's pin/peripheral map — `SimOnHW.inc` was written and
tested against CubeOrange-class boards.

### 2.3 SIM_JSON on the *host* side (already in use, distinct from 2.2)

`sr75_sim_json_responder.py` (used throughout the F22-GZ SITL series) speaks
the SIM_JSON protocol to an `ArduPlane.elf` **SITL** binary. It is unrelated
to real-hardware sensor fusion — it never runs against the physical Pixhawk.
Not a candidate for real-hardware estimator injection by itself; it only
becomes relevant to hardware once paired with the SoH firmware in §2.2, over
the `ppp/` link.

### 2.4 Existing custom SR75 receiver (`sr75_jsbsim_pixhawk_hil_bridge.py` + firmware `SR75_*` params) — RECOMMENDED, already partially validated

This is the **only** path in the audit that is (a) already running against
the bench's currently-flashed firmware, (b) already validated end-to-end
(F23-E2B PASS), and (c) requires zero new firmware.

It has two structurally different message types, and this is where task item
4 ("do not assume Mission Planner display ATTITUDE drives EKF") is decisively
confirmed by firmware source, not inference:

- **`GPS_INPUT` → genuinely fused into EK3.** `libraries/AP_GPS/AP_GPS_MAV.cpp`
  (`AP_GPS_MAV::handle_msg`) writes `state.location`, `state.velocity`,
  `state.hdop/vdop`, `state.gps_yaw` (see §6) straight into the GPS frontend
  state that EK3 consumes via `EK3_SRC1_POSXY`/`VELXY`/`POSZ`/`VELZ`
  (default GPS) — this is real sensor fusion, not display.
- **`NAMED_VALUE_FLOAT` (`AIRSPEED`, `SR75_ROLL`/`SR75_PITCH`/`SR75_YAW`) →
  display-only, firmware-documented as such.** Traced end to end:
  - `ArduPlane/Parameters.cpp:1296-1308` — `SR75_ARSPD_EN`/`SR75_ATT_EN`
    (`AP_GROUPINFO` indices 43/44), whose own `@Description` states *"for
    VFR_HUD reporting"* / *"for ATTITUDE telemetry reporting"*.
  - `ArduPlane/GCS_MAVLink_Plane.cpp:1149` decodes incoming
    `NAMED_VALUE_FLOAT` and stores `plane.sr75_ext_airspeed_mps` /
    `sr75_ext_roll_rad/pitch_rad/yaw_rad` with per-field validity flags and a
    timeout (`ArduPlane/Plane.cpp:158-190`,
    `sr75_external_airspeed()`/`sr75_external_attitude()`).
  - Both accessors are called from exactly two places:
    `GCS_MAVLINK_Plane::send_attitude()` (`GCS_MAVLink_Plane.cpp:134-174`) —
    substitutes `r/p/y` in the **outgoing** `ATTITUDE` message only (note:
    even here, the gyro-rate fields `omega.x/y/z` in that same outgoing
    message are always the *real* `ahrs.get_gyro()`, never overridden); and
    `GCS_MAVLINK_Plane::vfr_hud_airspeed()` (`GCS_MAVLink_Plane.cpp:269-294`)
    — substitutes the **outgoing** `VFR_HUD` airspeed field only, ahead of
    the real `plane.airspeed`/`AP::ahrs().airspeed_EAS()` fallbacks. Neither
    accessor is referenced anywhere in `AP_AHRS`, `AP_NavEKF3`, or
    `TECS`/`AP_TECS`. **`AP::ahrs()` itself is never touched.** This is a
    clean GCS-telemetry substitution, exactly matching the observed
    behaviour (custom `ATTITUDE` followed JSBSim; `AHRS2`, which reports the
    EKF's own internal state, did not).

## 3. Firmware compile-time/runtime support summary

| Path | Compiled into current bench firmware? | Runtime toggle |
|---|---|---|
| `HIL_STATE`/`HIL_STATE_QUATERNION`/`HIL_GPS`/`HIL_SENSOR` | No — no handler exists in this source tree at all | N/A |
| SIM_JSON / SoH sensor backends | No — requires a distinct `SIM_ENABLED=1` firmware build; no `fmuv3-SimOnHardWare` hwdef exists today | N/A (build-time only) |
| `GPS_INPUT` (`AP_GPS_MAV`, `AP_GPS_MAV_ENABLED`) | **Yes** — already validated | `GPS1_TYPE=14` (already set, confirmed PASS) |
| `NAMED_VALUE_FLOAT` SR75 display override | **Yes**, this is a bench-custom firmware fork (`SR75_ARSPD_EN`, `SR75_ATT_EN`, plus the whole `RATO_*` family from earlier tasks) | `SR75_ARSPD_EN`, `SR75_ATT_EN` (both already enabled per current bench behaviour) |

## 4. Does not assume display ATTITUDE drives EKF (task item 4) — confirmed false, with citations

Directly answered in §2.4: the outgoing `ATTITUDE`/`VFR_HUD` substitution is
strictly one-directional telemetry cosmetics. `AHRS2` (`ArduPlane` sends this
from the EKF's own internal roll/pitch/yaw estimate, unrelated to
`send_attitude()`) is the correct message to watch for genuine estimator
state, and it is exactly what the observed-gap data shows: it stayed near the
physical bench attitude throughout, because nothing in this firmware feeds
`SR75_ROLL`/`PITCH`/`YAW` into `AP_AHRS`/`AP_NavEKF3`.

## 5. HIL_SENSOR acceptance under AHRS_EKF_TYPE=3, EK3_ENABLE=1 (task item 5)

**Not accepted, unconditionally.** As shown in §2.1, there is no
`MAVLINK_MSG_ID_HIL_SENSOR` case anywhere in this tree's `GCS_MAVLink`
dispatch, on SITL or on fmuv3 hardware. `AHRS_EKF_TYPE`/`EK3_ENABLE` select
which EKF core is active; they have no bearing on whether a message ID has a
handler. Sending `HIL_SENSOR` to this firmware, with any EKF configuration,
is a no-op — the message is parsed by the MAVLink library layer (valid CRC,
valid message) and then silently dropped for lack of a consumer.

## 6. Frame/unit mapping (JSBSim → MAVLink), for the paths that are actually usable

Only `GPS_INPUT` (position, velocity, and — currently unused — yaw) is a
genuine estimator-fusion path today; the mapping below documents that path,
plus what the *display-only* messages already do for completeness.

| Quantity | JSBSim (AM2-schema CSV) | Frame/units | MAVLink field | Frame/units | Notes |
|---|---|---|---|---|---|
| Latitude/Longitude | `position/lat-gc-deg`, `position/long-gc-deg` | geocentric deg | `GPS_INPUT.lat/lon` | WGS84 degE7 (int32) | bridge already does `round(deg * 1e7)`; validated |
| Altitude (MSL) | `position/h-sl-ft` | feet, ASL | `GPS_INPUT.alt` | metres, ASL (float) | ×0.3048; validated. Feeds `EK3_SRC1_POSZ` **only if** that param is switched from BARO(1) to GPS(3) — not done yet, see §7 |
| Velocity N/E/D | `velocities/v-north-fps`, `v-east-fps`, `v-down-fps` | ft/s, NED | `GPS_INPUT.vn/ve/vd` | m/s, NED (float) | ×0.3048; validated, matches MAVLink's NED convention directly — **no axis flips needed** |
| True airspeed | `velocities/vt-fps` (or `vc-kts` calibrated) | ft/s or kts | `NAMED_VALUE_FLOAT("AIRSPEED")` | m/s | ×0.3048 / ×0.514444; **display only**, see §2.4 — does not reach `AP_Airspeed`/TECS |
| Attitude (roll/pitch/yaw) | `attitude/phi-deg`, `theta-deg`, `psi-deg` | degrees, right-handed body/local (JSBSim: phi/theta/psi Euler, NED-referenced) | `NAMED_VALUE_FLOAT("SR75_ROLL/PITCH/YAW")` | radians | `radians(deg)`; **display only**, does not reach `AP_AHRS` |
| Yaw as a genuine EKF source (unused today) | `attitude/psi-deg` | degrees, 0-360 clockwise from true north | `GPS_INPUT.yaw` | **centidegrees**, 0-35999, 0=unavailable | Currently the bridge never sets this field (always sent as 0 ⇒ `have_yaw=false` in `AP_GPS_MAV.cpp`). This is a real, already-compiled-in EKF yaw-fusion path (`EK3_SRC1_YAW=GPS(2)`), unused — see §7 |
| Body acceleration (ax/ay/az) | `accelerations/a-pilot-{x,y,z}-ft_sec2` | ft/s², body frame | *(no live path)* | — | No MAVLink message in this firmware feeds `AP_InertialSensor`; would require SoH (§2.2) or a new `AP_ExternalAHRS` backend (§8) |
| Body rates (p/q/r) | `velocities/{p,q,r}-rad_sec` | rad/s, body frame | *(no live path)* | — | Same as above — `ATTITUDE.rollspeed/pitchspeed/yawspeed` is an **outgoing-only** field in this firmware (populated from real `ahrs.get_gyro()`, §2.4); there is no incoming gyro-injection message handled |
| Barometric pressure/altitude | `position/h-sl-ft` (no raw pressure channel emitted today) | feet | *(no live path)* | — | `AP_Baro` has no MAVLink-driven backend in this tree (only `AP_Baro_SITL`/`AP_Baro_ExternalAHRS`); GPS-sourced altitude (row above) is the closest coherent substitute once `EK3_SRC1_POSZ=GPS` |

## 7. Estimator-source configuration required for coherent fusion (task item 7)

Two tiers, matching what is/isn't achievable without new firmware:

**Tier 1 — immediately achievable, zero firmware changes, parameter-only
(recommended next authorized step):**
- Extend the bridge's `GPS_INPUT` packet to populate `yaw` from JSBSim
  `attitude/psi-deg` (currently sent as 0/unavailable) — a bridge-side code
  change, not firmware.
- `EK3_SRC1_YAW = 2` (GPS) — currently default `1` (COMPASS). Fuses the
  injected yaw into EKF3 for real, closing the "custom ATTITUDE is
  display-only" gap for **yaw** specifically.
- `EK3_SRC1_POSZ = 3` (GPS) — currently `1` (BARO). Makes
  `GLOBAL_POSITION_INT`/`VFR_HUD` altitude track the injected `GPS_INPUT.alt`
  (i.e. JSBSim) instead of the physical barometer, closing the altitude gap
  reported in §1.
- This tier gives coherent position, velocity, and yaw fusion. It does
  **not** give roll/pitch fusion or accel/gyro/true-barometric fusion — EK3
  has no "external roll/pitch source" concept independent of accel/gyro
  fusion; roll/pitch necessarily come from the real IMU in this tier.

**Tier 2 — full coherent fusion (accel, gyro, "compass"/tilt reference,
barometric pressure), requires new firmware:**
- Simulation-on-Hardware (§2.2): build+flash a `SIM_ENABLED=1` fmuv3 image
  (no existing hwdef target; would need to be produced via
  `sitl-on-hw.py --board fmuv3 ...`, unvalidated against this exact board),
  full parameter wipe, PPP/TELEM1 rewire. This is the only path that gives
  genuinely coherent accel/gyro/baro/compass fusion, because it replaces the
  physical-sensor HAL backends themselves.
- Alternative Tier-2 route: a new `AP_ExternalAHRS` backend (the framework
  already exists — `AP_InertialSensor_ExternalAHRS`, `AP_Baro_ExternalAHRS`,
  `AP_Compass_ExternalAHRS`, `AP_Airspeed_External` all already compile in —
  but every existing concrete backend, e.g. VectorNav/MicroStrain/SBG,
  speaks a vendor binary protocol; there is no generic MAVLink/JSON
  `AP_ExternalAHRS` backend today). Writing one is a genuine firmware
  C++ development task, not a bench-configuration task.

Neither Tier-2 route is in scope for this audit (no code changes) or for
"exact next implementation scope" below.

## 8. Conflicts with real IMU/barometer/compass sensors (task item 8)

- The physical IMU, barometer, and compass on the bench Pixhawk **keep
  running and keep being fused** under every path in this audit except
  Tier 2/SoH (which physically replaces those backends). This is by design
  and is not a conflict to "fix" — it is the reason `AHRS2` correctly stayed
  near the real bench attitude while the display-only `ATTITUDE` message
  followed JSBSim.
- If Tier 1 (`EK3_SRC1_POSZ=GPS`) is adopted: the physical barometer is
  simply no longer the active Z source. It is *not* disabled — `EK3_SRC2_*`
  (still default BARO-derived if left unset — verify before changing) could
  be used as a manual fallback set, but ArduPilot does not automatically
  fail over SRC1→SRC2 without an explicit `EK3_SRC_OPTIONS`/lane-switch
  configuration. Practical implication for the bench: once `GPS_INPUT` goes
  stale (bridge stops, JSBSim exits, etc.), `GLOBAL_POSITION_INT` altitude
  will hold/degrade with the GPS fix rather than silently reverting to the
  physical baro. Fine for a disarmed bench test; must be flagged before ever
  considering flight-relevant use.
- If Tier 1 (`EK3_SRC1_YAW=GPS`) is adopted: real compass data is still
  read and logged, just not the active yaw source. No hardware conflict;
  purely a source-selection change.
- Tier 2 (SoH) is the one path with a **real** conflict: it removes the
  physical sensors from the estimation loop entirely (by HAL-backend
  substitution), so it cannot be used for genuine hardware sensor validation
  simultaneously — it is a pure software-in-the-loop-on-hardware mode, not a
  hybrid.

## 9. Staged disarmed test plan (task item 9), for Tier 1 once authorized

All stages: Pixhawk stays disarmed/MANUAL throughout (matches current bench
practice), no actuator output, no CH7/CH8 activity, `GPS1_TYPE=14`
unchanged, mission file/IC untouched (open-loop JSBSim producer, same as
F23-E1/E2A/E2B tooling).

a. **Static level state.** JSBSim IC held at zero rates, wings-level,
   constant lat/lon/alt, `psi` fixed. Confirm `GPS_INPUT.yaw` is accepted
   (`have_yaw=true` path in `AP_GPS_MAV.cpp`) and `EK3_SRC1_YAW=GPS` yaw
   settles near the commanded value in `AHRS2`/`ATTITUDE_QUATERNION`,
   without inducing an EKF yaw-reset event (watch `EKF_STATUS_REPORT`
   flags).

b. **Controlled pitch sweep.** JSBSim `theta` ramped through a known
   profile (e.g. ±20° at a slow, bounded rate) with lat/lon/alt held
   constant. Since Tier 1 does **not** fuse pitch, this stage's purpose is
   to confirm the *display-only* `ATTITUDE` pitch (already validated in
   F23-E2B) continues to track correctly and does **not** perturb
   `AHRS2`/EKF pitch — i.e., re-confirm the §2.4 finding under a larger
   pitch excursion than the -77.6° incidental case.

c. **Controlled roll sweep.** Same rationale as (b), for roll.

d. **Altitude ramp.** JSBSim `h-sl-ft` ramped through a bounded, known
   profile with lat/lon/attitude held constant. With `EK3_SRC1_POSZ=GPS`,
   confirm `GLOBAL_POSITION_INT`/`VFR_HUD` altitude tracks the ramp within
   the same tolerance already demonstrated for horizontal position in
   F23-E2B, and that the physical-baro-vs-GPS altitude divergence from §1
   is resolved.

e. **Moving GPS state.** Reuse the existing open-loop mission-bearing
   trajectory (already validated), now with yaw populated, to confirm
   combined position+velocity+yaw fusion is coherent (e.g. velocity vector
   heading and fused yaw agree) over a multi-minute run, not just a static
   snapshot.

Each stage should be run through the existing `wait_for_producer_ready`
gate and stale-CSV pause logic already implemented in
`sr75_hil_f23e1_live_feed_launch.py`/`sr75_jsbsim_pixhawk_hil_bridge.py`
(§10) before being considered complete.

## 10. Timeout/stale/failsafe behaviour (task item 10)

- **Producer side (already implemented, F23-E1/E2A):** `JSBSimStateFeeder`
  pauses all GPS/airspeed/attitude transmission the moment the state CSV
  stops advancing past `JSB_FEED_STALE_TIMEOUT_S` (2.0 s default) — it does
  not resend the last row indefinitely. `max_observed_gap_s` is logged; the
  F23-E2B run's `0.016 s` maximum gap indicates this has not been exercised
  under real staleness on this bench yet.
- **GPS/EKF side (real firmware behaviour, unchanged by this audit):** once
  `GPS_INPUT` stops arriving, `AP_GPS` ages out the fix through its normal
  timeout path (independent of the injected-vs-physical distinction — this
  firmware has no special-case handling for MAVLink-sourced GPS staleness).
  EK3 will flag `GPS_QUALITY_GOOD=false`/increase innovation variances
  through `EKF_STATUS_REPORT`; because `ARMING_REQUIRE`/disarmed state is
  maintained throughout every stage in §9, no flight-mode failsafe (RTL,
  etc.) can trigger — but this must be re-verified explicitly, not assumed,
  before any future arming-adjacent task.
- **If `EK3_SRC1_POSZ`/`YAW` are switched to GPS (Tier 1):** there is no
  automatic SRC1→SRC2 failover without explicit `EK3_SRC_OPTIONS`
  configuration (§8) — a GPS_INPUT stall degrades the *active* Z/yaw source
  directly. Acceptable for a disarmed bench test, called out explicitly as a
  pre-condition to check before any future flight-relevant recommendation.

## Return

**Recommended architecture:** Tier 1 — extend the already-validated
`GPS_INPUT` MAVLink path (no new firmware) to also populate the `yaw` field
from JSBSim `psi`, paired with `EK3_SRC1_YAW=GPS` and `EK3_SRC1_POSZ=GPS`
parameter changes (not applied in this audit). This closes the altitude and
yaw-fusion parts of the observed gap using infrastructure already proven on
this exact bench. Roll/pitch and true accel/gyro/barometric fusion remain
out of reach without new firmware (Tier 2).

**Rejected alternatives and reasons:**
- `HIL_SENSOR`/`HIL_GPS`/`HIL_STATE_QUATERNION` — no handler exists anywhere
  in this codebase (SITL or hardware); confirmed by exhaustive grep, not
  parameter-dependent.
- Simulation-on-Hardware / SIM_JSON on real firmware — architecturally the
  only true full-sensor path, but requires a new, unvalidated-for-fmuv3
  firmware build, full parameter wipe, and PPP rewiring; out of scope for a
  no-code-change bench-configuration task.
- A new `AP_ExternalAHRS` backend — the framework exists but no
  MAVLink/JSON-speaking backend exists today; writing one is firmware C++
  development, not configuration.

**Required parameters/build flags (for Tier 1, not applied here):**
`EK3_SRC1_YAW=2`, `EK3_SRC1_POSZ=3`. No build flags required — all consumed
infrastructure (`AP_GPS_MAV_ENABLED`, `GPS_INPUT` handling, `EK3_SRC1_*`) is
already compiled into the current bench firmware.

**MAVLink messages and rates:** `GPS_INPUT` (already at the bridge's
existing `--gps-input-rate-hz`, default 5 Hz) — no new message type; only
the previously-unused `yaw`/`yaw_accuracy` fields need populating.

**Frame/unit mapping:** §6 table above.

**Staged test plan:** §9 (a-e), reusing existing producer-readiness/stale
gating.

**Safety gates:** disarmed/MANUAL throughout, `GPS1_TYPE=14` unchanged, no
actuator/CH7/CH8 activity, mission/IC files unchanged, no parameter writes
performed by this audit (Tier 1 params identified but not applied).

**Exact next implementation scope (if authorized):** (1) bridge-side code
change to populate `GPS_INPUT.yaw`/`yaw_accuracy` from the JSBSim state feed
[code change — requires separate authorization]; (2) two parameter changes
on the bench (`EK3_SRC1_YAW=2`, `EK3_SRC1_POSZ=3`) [parameter change —
requires separate authorization]; (3) run staged tests (a)-(e) disarmed.
None of this was performed in this audit.

**PASS/FAIL readiness:** **PASS** for the audit itself (architecture
identified, grounded in source, actionable next scope defined). Bench
remains **NOT YET COHERENT** for full sensor fusion — Tier 1 closes
position/velocity/altitude/yaw; roll/pitch/accel/gyro/true-baro fusion is
gated on a Tier 2 (new firmware) decision that has not been made.
