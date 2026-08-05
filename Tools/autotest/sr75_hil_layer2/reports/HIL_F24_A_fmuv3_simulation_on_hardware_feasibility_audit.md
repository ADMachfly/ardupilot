# HIL-F24-A: fmuv3 Simulation-on-Hardware Feasibility Audit

Status: **audit only**. No hardware flashed, no Pixhawk parameters changed, no
arming, no actuator output, no flight/RATO/aero/TECS logic changed. A
temporary, audit-only `fmuv3-SimOnHardWare` hwdef target was created,
configured, and built to obtain real flash/RAM numbers, then **deleted**
(never committed) — see §4 and the Commands Executed list.

## Verdict

**FEASIBLE.** A custom `fmuv3-SimOnHardWare` target compiles cleanly against
this exact source tree, with **no hwdef pin/DMA/timer changes required** (the
existing `SimOnHW.inc` overlay is a pure feature-flag/compile-define layer —
it never touches a single pin), and the resulting ArduPlane image **fits with
more free flash than the current baseline** (630,392 B vs 543,832 B free) at
the cost of a modest ~9.4 KB static-RAM increase. The SR-75 custom code
(`rato.cpp`, `mode_sr75_vland.cpp`, `SR75_ARSPD_EN`/`SR75_ATT_EN` params)
compiles into this target unmodified. No feature reduction is required to
*fit*; further trims are optional headroom insurance for runtime
(dynamic heap/CPU), which this audit could not measure.

## 1. Existing SoH architecture (exact)

Two targets exist, both Copter-oriented, both purely additive over their base
board:

```
libraries/AP_HAL_ChibiOS/hwdef/CubeOrange-SimOnHardWare/hwdef.dat:
  include ../CubeOrange/hwdef.dat
  include ../include/SimOnHW.inc
  define CHIBIOS_SHORT_BOARD_NAME "CubeOrangeSimOnHardWare"
  define AP_BOOTLOADER_FLASHING_ENABLED 0
  AUTOBUILD_TARGETS Copter

libraries/AP_HAL_ChibiOS/hwdef/CubeOrangePlus-SimOnHardWare/hwdef.dat:
  include ../CubeOrangePlus/hwdef.dat
  include ../include/SimOnHW.inc
  undef INS_AUX_INSTANCES / define INS_AUX_INSTANCES 0
  define CHIBIOS_SHORT_BOARD_NAME "CubeOrange+SimOnHW"
  define AP_BOOTLOADER_FLASHING_ENABLED 0
  AUTOBUILD_TARGETS Copter
```

`libraries/AP_HAL_ChibiOS/hwdef/include/SimOnHW.inc` (shared, board-agnostic):
sets `env SIM_ENABLED 1` (the one directive that actually matters — everything
else is feature trimming) and disables `HAL_NAVEKF2_AVAILABLE`,
`EK3_FEATURE_BODY_ODOM/EXTERNAL_NAV/DRAG_FUSION`, `HAL_ADSB_ENABLED`,
`HAL_MOUNT_ENABLED`, `HAL_PROXIMITY_ENABLED`, `HAL_VISUALODOM_ENABLED`,
`HAL_GENERATOR_ENABLED`, `HAL_CRSF_TELEM_ENABLED`, `AP_LANDINGGEAR_ENABLED`,
`HAL_MSP_OPTICALFLOW_ENABLED`, `HAL_SUPPORT_RCOUT_SERIAL`,
`HAL_HOTT_TELEM_ENABLED`, `AP_SIM_INS_FILE_ENABLED`, and sets
`HAL_COMPASS_MAX_SENSORS 2`. **No pin, DMA, or timer directives anywhere in
this file** — it is a pure `#define`/env-var overlay.

`defaults.parm` (CubeOrange-SimOnHardWare, Copter-specific) sets
`AHRS_EKF_TYPE 10` (the `AP_AHRS::EKFType::SIM` backend — reads JSBSim/SITL's
*true* state directly, **bypassing EKF3 fusion entirely**) and `GPS1_TYPE
100`/`ARSPD_TYPE 100` (SITL-specific sensor-backend enum values, confirmed
from `libraries/SITL/SIM_JSON.cpp`'s own `sim_defaults[]` table — these are
**not** the `GPS1_TYPE=14` MAVLink/GPS_INPUT value our current SR-75 bench
uses; they select the *compiled-in* SITL sensor backend directly).

**Critical distinction for our purpose:** `AHRS_EKF_TYPE=10` is a *default
parameter choice* in the stock Copter `defaults.parm`, not an architectural
requirement. The underlying sensor **backends**
(`AP_InertialSensor_SITL`/`AP_Baro_SITL`/`AP_Compass_SITL`, gated by
`AP_SIM_ENABLED`, confirmed in `AP_Baro_config.h`/`AP_Compass_config.h`/etc.)
feed the same `dal.ins()`/`dal.baro()`/`dal.compass()`/`dal.gps()` accessors
EKF3 always reads from, entirely independent of `AHRS_EKF_TYPE`. Setting
`AHRS_EKF_TYPE=3` on a SoH image should let EKF3 genuinely fuse the simulated
sensors — which is what SR-75 actually needs (this was not build-tested in
this audit; it is a parameter-level claim grounded in the config-flag
dependency chain, to be confirmed empirically in F24-B).

## 2. Required simulated backends (traced)

`env SIM_ENABLED 1` → `Tools/ardupilotwaf/boards.py` defines
`AP_SIM_ENABLED=1` at compile time → gates, per-subsystem:

| Subsystem | Flag | File |
|---|---|---|
| INS/IMU | `AP_SIM_ENABLED` (feeds `AP_InertialSensor_SITL`) | `AP_InertialSensor` |
| Baro | `AP_SIM_BARO_ENABLED = AP_SIM_ENABLED` | `AP_Baro_config.h:103` |
| Compass | `AP_COMPASS_SITL_ENABLED = ...&& AP_SIM_ENABLED` | `AP_Compass_config.h:43` |
| GPS | `AP_SIM_GPS_ENABLED = AP_SIM_ENABLED` | `SITL/SIM_config.h:142` |
| Airspeed | `AP_AIRSPEED_SITL_ENABLED = ...&& AP_SIM_ENABLED` | `AP_Airspeed_config.h:55` |
| AHRS bypass | `AP_AHRS_SIM_ENABLED = ...&& AP_SIM_ENABLED && AP_INERTIALSENSOR_ENABLED` | `AP_AHRS_config.h:37` |
| Optical flow | `AP_OPTICALFLOW_SITL_ENABLED` | `AP_OpticalFlow_config.h:48` |
| Battery | not SIM-specific; `defaults.parm` just sets `BATT_MONITOR 0` | — |

All of these are physically fed by the JSBSim/SITL physics state arriving over
the SIM_JSON UDP protocol (`libraries/SITL/SIM_JSON.cpp`) — see §6.

## 3. fmuv3 compatibility

`libraries/AP_HAL_ChibiOS/hwdef/fmuv3/hwdef.dat`: `MCU STM32F4xx
STM32F427xx`, `FLASH_SIZE_KB 2048`. (CubeOrange: `STM32H7xx STM32H743xx` —
materially more RAM; this is the one real architectural gap between the
existing precedent and fmuv3, addressed empirically in §4.)

- **hwdef changes required:** none beyond the two `include` lines +
  `CHIBIOS_SHORT_BOARD_NAME` override + `AP_BOOTLOADER_FLASHING_ENABLED 0` +
  `AUTOBUILD_TARGETS Plane` (not `Copter`) — i.e. exactly the same 4-line
  pattern as the existing CubeOrange targets, just pointed at
  `../fmuv3/hwdef.dat`.
- **Pin/DMA/timer conflicts:** none possible in principle (`SimOnHW.inc`
  contains zero pin directives) and none observed in practice — `./waf
  configure --board fmuv3-SimOnHardWare` produced zero warnings/errors when
  grepped for `warn|error|conflict|duplicate|overlap`.
- **USB MAVLink:** unaffected — fmuv3's existing USB CDC config (confirmed via
  `USBID=0x1209/0x5741` in the configure log, inherited unchanged from
  `fmuv3/hwdef.dat`) is untouched by `SimOnHW.inc`. Mission Planner over USB
  continues to work exactly as today.
- **SIM_JSON input transport:** fmuv3 has no Ethernet MAC, so this is **not**
  a raw-USB input path — it is the same PPP-over-TELEM1 path this repo
  already prepared and documented (`Tools/autotest/sr75_hil_layer2/ppp/
  README.md`, itself referencing `sitl-on-hw.py --board fmuv3 ... --enable-
  PPP`). Confirmed the ChibiOS PPP networking backend is compiled in for
  fmuv3-class boards: `AP_NETWORKING_BACKEND_PPP = (...&& CONFIG_HAL_BOARD ==
  HAL_BOARD_CHIBIOS && !HAL_USE_MAC)` (`AP_Networking_Config.h:35`) — fmuv3
  has no MAC peripheral, so this resolves true. Nothing new to build here;
  the existing `ppp/` prep work is directly reusable.

## 4. Flash/RAM feasibility — real build numbers

**Baseline** (`./waf configure --board fmuv3 && ./waf plane`):

```
Target         Text (B)  Data (B)  BSS (B)  Total Flash Used (B)  Free Flash (B)
bin/arduplane   1532400      4528   100612               1536928          543832
```
Static RAM (Data+BSS) = 105,140 B (~102.7 KB).

**Audit-only fmuv3-SimOnHardWare** (temporary hwdef, built, then deleted —
never committed):
```
Target         Text (B)  Data (B)  BSS (B)  Total Flash Used (B)  Free Flash (B)
bin/arduplane   1446088      4280  110268               1450368          630392
```
Static RAM (Data+BSS) = 114,548 B (~111.9 KB).

**Delta:** Flash used **decreased** by 86,560 B (SimOnHW.inc's feature
removals outweigh the added SITL-backend code); free flash **increased** by
86,560 B. Static RAM **increased** by 9,408 B (~9.2 KB) — plausible headroom
against fmuv3's SRAM (F427: 192 KB SRAM1/2 + 64 KB CCM ≈ 256 KB total,
conventionally cited for this MCU; not independently re-derived from the
linker script in this audit).

**Not measured (real gap):** dynamic heap/stack usage under actual runtime
load (SIM_JSON parsing buffers, socket buffers, EKF3 core state at
`AHRS_EKF_TYPE=3`) — `./waf plane`'s size report is static-only. This is the
single largest unverified risk and should be the first thing checked once
real hardware is flashed (F24-B), via ArduPilot's own free-memory reporting
(`STATUSTEXT`/`MEMINFO` or the `AP_HAL::Util::available_memory()` telemetry).

The custom SR-75 code compiled into the audit build unmodified — confirmed
directly in the build log: `Compiling ArduPlane/rato.cpp`, `Compiling
ArduPlane/mode_sr75_vland.cpp`, `Compiling ArduPlane/Parameters.cpp` (which
carries `SR75_ARSPD_EN`/`SR75_ATT_EN`).

## 5. Feature reduction

Based on the measured numbers, **no feature reduction is required to fit**.
`SimOnHW.inc` already strips ADSB/Mount/Proximity/VisualOdom/Generator/
CRSF-telem/MSP-optical-flow/HOTT-telem/landing-gear — none of which are on
the "must keep" list, and this alone is why flash usage *dropped* rather than
grew. If F24-B's real-hardware runtime testing reveals dynamic-memory or CPU
pressure, the following are additional, safe-to-remove candidates (none on
the must-keep list): Lua scripting, CAN/DroneCAN, terrain, avoidance,
optical flow (if not already stripped), rangefinder backends, unused
extra GPS/compass/baro backends, unused telemetry protocols (FrSky, LTM,
etc.). None of these were removed in this audit's build — the measured
numbers already include them.

## 6. Input architecture — exact SIM_JSON protocol

Traced directly from `libraries/SITL/SIM_JSON.h`/`.cpp`:

- **Transport:** UDP. Default `target_ip="127.0.0.1"`, `control_port=9002`
  (frame string after the first `:` overrides the IP, e.g.
  `json:192.168.144.2`, exactly as our `ppp/README.md` already documents).
  `UDP_TIMEOUT_MS = 100` — a receive that doesn't complete within 100ms is
  treated as a timeout for that cycle.
- **Framing:** newline-delimited JSON objects, one per physics step,
  accumulated in an internal 65,000-byte buffer and parsed line-by-line
  (`parse_sensors()`, "very simple JSON parser... not general purpose").
- **Exact field table** (`keytable[36]` in `SIM_JSON.h`, verbatim
  `{section, key, target-field, type, required}`):

  | JSON key | Type | Target | Required |
  |---|---|---|---|
  | `timestamp` | double | `state.timestamp_s` | **yes** |
  | `latitude`/`longitude`/`altitude` | double | lat/lon/alt (deg/deg/m) | no |
  | `imu.gyro` | Vector3f | body-frame rad/s | **yes** |
  | `imu.accel_body` | Vector3f | body-frame accel | **yes** |
  | `position` | Vector3d | NED, alternative to lat/lon/alt | no |
  | `attitude` | Vector3f | Euler roll/pitch/yaw (rad) | no* |
  | `quaternion` | Quaternion | alternative to `attitude` | no* |
  | `velocity` | Vector3f | NED | **yes** |
  | `rng_1`..`rng_6` | float | rangefinder | no |
  | `velocity_wind` | Vector3f | | no |
  | `windvane.direction`/`speed` | float | | no |
  | `airspeed` | float | m/s | no |
  | `no_time_sync`/`no_lockstep` | bool | pacing control flags | no |
  | `rc_1`..`rc_12` | float | | no |
  | `battery.voltage`/`battery.current` | float | | no |

  \* code enforces "must get either `attitude` or `quaternion`" as a combined
  requirement even though neither is individually marked `required`.
- **Coordinate frames:** body frame for IMU (gyro/accel), NED for
  position/velocity — the same conventions our existing bridge already uses
  for `GPS_INPUT.vn/ve/vd`, so no new frame-conversion logic would be needed
  on the host side.
- **Timeout behavior:** a stale/absent UDP stream simply stops updating
  `state` past the 100ms receive timeout; there is no separate "failsafe"
  packet type — this mirrors (and is less permissive than) our own bridge's
  existing 2.0s `JSB_FEED_STALE_TIMEOUT_S` pause logic.

## 7. Migration impact

- **`sr75_jsbsim_pixhawk_hil_bridge.py`:** its entire `GPS_INPUT`/
  `AP_GPS_MAV` architecture becomes **irrelevant** for a true SoH setup — SoH
  never uses `GPS_INPUT`, it feeds the SITL sensor backends directly via
  SIM_JSON. This is not new development, though: `sr75_sim_json_responder.py`
  (already built and validated against `ArduPlane.elf` throughout the
  F22-GZ SITL series) speaks exactly this protocol already — migrating means
  pointing that **already-working** responder at the real Pixhawk over the
  PPP link, not writing a new one.
- **HIL orchestrator scripts** (`sr75_hil_f23e1_live_feed_launch.py`,
  `sr75_hil_f2b3_combined_nav_profile.py`, etc.): built entirely around the
  GPS_INPUT bridge; would not apply directly. Their *patterns* (preflight
  read-only checks, phase-based summaries, staleness handling,
  dry-run/producer-only modes) are directly reusable in a new SoH-oriented
  harness.
- **Parameter files:** none of the current bench `.param` files
  (`SR75_LAYER2_AM2_HIL_SAFE.param` etc.) carry over. Official SoH procedure
  requires a full parameter wipe (`Tools/scripts/sitl-on-hardware/README.md`)
  before first use, and `GPS1_TYPE`/`ARSPD_TYPE` would need the SITL-specific
  values (100), not 14 — an entirely separate parameter set from the current
  bench.
- **Mission Planner connection:** unaffected — still plain MAVLink2 over USB.
- **PWM/actuator feedback:** this is the most important safety-relevant
  migration delta. Our current bench structurally cannot drive actuators
  (the orchestrator never sends actuator commands, and `--allow-actuator-
  output` is a hardcoded no-op) — safety is enforced *host-side*. A genuine
  SoH image, by design, *can* drive simulated actuator output through to real
  PWM outputs, because that is the point of "Simulation-on-Hardware" (closing
  the loop through real servos in a bench rig). This means safety for a SoH
  image must be enforced *board-side/procedurally* (§9), not just by what the
  host script chooses to send.
- **Existing SR75 custom ArduPlane code:** compiles unmodified into the SoH
  target (confirmed, §4); no source changes anticipated.

## 8. Risks

- **Flash overflow:** low risk — measured 630 KB free on the audit build,
  more headroom than baseline.
- **RAM exhaustion:** unverified for dynamic use; static usage rose ~9.2 KB,
  acceptable margin against F427's ~256 KB, but heap/stack behavior under
  real SIM_JSON parsing + EKF3 load is untested. **Primary open risk.**
- **CPU load:** unmeasured — F427 (Cortex-M4, ~168–180 MHz class) is
  significantly less powerful than CubeOrange's H743 (Cortex-M7, 480 MHz);
  the officially-supported SoH precedent has never been CPU-validated on
  F4-class silicon. Real risk of missed real-time deadlines / watchdog trips
  under load; must be measured in F24-B, not assumed from this audit.
- **fmuv3 feature limitations:** this exact board+SoH combination has no
  prior precedent anywhere in this source tree or its history — everything
  in this audit is derived from static analysis and one successful compile,
  not a runtime trial.
- **Bootloader/upload recovery:** `AP_BOOTLOADER_FLASHING_ENABLED 0` (matches
  existing SoH pattern) — the bootloader itself is untouched by this hwdef;
  recovery is the normal DFU/bootloader reflash procedure, no added risk
  there specifically.
- **Accidental use of simulation firmware on real aircraft:** real and
  serious, given §7's PWM/actuator-passthrough distinction — a SoH image
  flashed onto a board with live servos/ESCs/RATO igniters connected could
  drive them from simulated commands. This is the task's explicit reason for
  §9.
- **Parameter wipe/reset risk:** confirmed required by the official
  procedure — switching to/from a SoH image means the current bench's
  validated `EK3_SRC1_*`/`GPS1_TYPE=14` configuration is not preserved
  automatically; a saved `.param` backup and an explicit restore step are
  required (§9).

## 9. Safety separation (recommended, not implemented in this audit)

- **Unique board/firmware name:** `CHIBIOS_SHORT_BOARD_NAME "fmuv3-SoH"` (or
  similar), distinct from `fmuv3`'s normal identifier — already how the
  existing CubeOrange targets self-identify; directly reusable pattern.
- **Console/startup banner:** ArduPilot already prints board name and build
  options over MAVLink `STATUSTEXT`/console at boot; a SoH-specific banner
  string (e.g. via a `#define` in the SoH-only hwdef) makes the distinction
  visible immediately in Mission Planner's messages tab, no code change
  needed beyond the define.
- **Dedicated branch:** build/maintain the `fmuv3-SimOnHardWare` hwdef only
  on a clearly-named branch (e.g. `sr75-fmuv3-soh-audit` or similar),
  never merged into the branch that produces flight firmware for this
  airframe.
- **Simulation-only parameter:** a dedicated param (e.g. reusing the
  `SR75_*` custom-param convention already established, `SR75_SOH_ACTIVE` or
  similar) settable only in the SoH `defaults.parm`, checked by any future
  RATO/ignition-adjacent code path as an additional inhibit — not built in
  this audit (no code changes were made).
- **No propulsion/RATO outputs:** the existing `RATO_IGN_CH`/`RATO_EJ_CH`
  channel-based design already requires an explicit channel assignment;
  the SoH `defaults.parm` should explicitly zero/unassign those channels by
  default, on top of the general "CH7/CH8 disconnected on the bench" bench
  safety rule already followed throughout this task family.
- **Restoration procedure:** keep the current, already-validated bench
  firmware image and its exact `.param` file archived before ever flashing a
  SoH image; restoring means reflashing that archived image and reloading
  the archived `.param` file (full wipe + reload, matching §7/§8's
  parameter-wipe finding) — this must be written down and tested once, in
  F24-B, before the SoH firmware is used routinely.

## Recommended F24-B implementation scope

1. Create the real (not audit-only) `fmuv3-SimOnHardWare` hwdef + a Plane-
   specific `defaults.parm` (using `AHRS_EKF_TYPE=3`, not the Copter
   default's `10`) on a dedicated branch — never on the flight-firmware
   branch.
2. Build and flash to the bench Pixhawk 2.4.8 (separately authorized step,
   not part of this audit).
3. Do a full parameter wipe + fresh load of the SoH-specific param set.
4. Bring up PPP-over-TELEM1 using the already-documented `ppp/` procedure
   and confirm `sr75_sim_json_responder.py` (already built, already validated
   against SITL) talks to the real board.
5. Measure real dynamic RAM/CPU headroom on hardware — the one gap this
   static audit could not close.
6. Only then attempt genuine combined EKF3 fusion of simulated IMU/baro/
   compass/GPS/airspeed with `AHRS_EKF_TYPE=3`, and compare against the
   GPS_INPUT-only fusion results already obtained (F23-F2B3-family).

## Files inspected

- `libraries/AP_HAL_ChibiOS/hwdef/CubeOrange-SimOnHardWare/hwdef.dat`, `defaults.parm`
- `libraries/AP_HAL_ChibiOS/hwdef/CubeOrangePlus-SimOnHardWare/hwdef.dat`
- `libraries/AP_HAL_ChibiOS/hwdef/include/SimOnHW.inc`
- `libraries/AP_HAL_ChibiOS/hwdef/fmuv3/hwdef.dat`
- `libraries/AP_HAL_ChibiOS/hwdef/CubeOrange/hwdef.dat` (MCU line only)
- `Tools/ardupilotwaf/boards.py` (`AP_SIM_ENABLED` propagation)
- `libraries/AP_Baro/AP_Baro_config.h`, `AP_Compass/AP_Compass_config.h`,
  `AP_Airspeed/AP_Airspeed_config.h`, `AP_AHRS/AP_AHRS_config.h`,
  `SITL/SIM_config.h`, `AP_OpticalFlow/AP_OpticalFlow_config.h`
- `libraries/AP_AHRS/AP_AHRS.h` (`EKFType::SIM = 10`)
- `libraries/SITL/SIM_JSON.h`, `SIM_JSON.cpp` (full protocol/keytable trace)
- `libraries/AP_Networking/AP_Networking_Config.h`
- `Tools/autotest/sr75_hil_layer2/ppp/README.md` (existing prep work, cross-referenced)
- `Tools/scripts/sitl-on-hardware/README.md` (official parameter-wipe procedure)
- Build log output for both `fmuv3` and the temporary `fmuv3-SimOnHardWare` targets

## Commands executed

```sh
git status --porcelain
./waf configure --board fmuv3
./waf plane
# --- temporary, audit-only target, created for this audit ---
mkdir -p libraries/AP_HAL_ChibiOS/hwdef/fmuv3-SimOnHardWare
# (wrote hwdef.dat: include ../fmuv3/hwdef.dat + include ../include/SimOnHW.inc
#  + CHIBIOS_SHORT_BOARD_NAME + AP_BOOTLOADER_FLASHING_ENABLED 0 + AUTOBUILD_TARGETS Plane)
./waf configure --board fmuv3-SimOnHardWare
./waf plane
# --- cleanup: audit-only target deleted, never committed ---
rm -rf libraries/AP_HAL_ChibiOS/hwdef/fmuv3-SimOnHardWare
rm -rf build/fmuv3-SimOnHardWare
git status --porcelain   # confirmed clean
```

No hardware was connected or flashed at any point. No `PARAM_SET`, arming,
mission, or actuator command was sent (this audit made no MAVLink connection
at all — it is a source-tree and build-system audit only).

## PASS/FAIL

**PASS.** Feasibility verdict: **FEASIBLE**, with the real, measured
build-size evidence in §4, a fully-traced SIM_JSON protocol spec in §6, and
an honest statement of the one gap this static audit cannot close (dynamic
RAM/CPU headroom, §8) carried forward as the first item of the recommended
F24-B scope.
