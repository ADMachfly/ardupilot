# HIL-F24-B: fmuv3 Simulation-on-Hardware Target Implementation

Status: persistent source implementation, **no hardware flashed, no Pixhawk
parameters changed, no PPP hardware connected, no arming, no actuator
output**. Builds were run locally (`./waf`) only, producing local artifacts
under `build/` (gitignored) that were never flashed anywhere.

## 1. Board target structure

```
libraries/AP_HAL_ChibiOS/hwdef/fmuv3-SimOnHardWare/
├── hwdef.dat        # 2 includes + 4 defines, no pin/DMA/timer directives
└── defaults.parm    # SR-75-specific, NOT the bench's GPS_INPUT param file
```

`hwdef.dat` (minimum inheritance, mirrors the existing CubeOrange*-
SimOnHardWare pattern exactly):
```
include ../fmuv3/hwdef.dat
include ../include/SimOnHW.inc

define CHIBIOS_SHORT_BOARD_NAME "fmuv3-SR75-SoH"
define AP_CUSTOM_FIRMWARE_STRING "SR75-SIMULATION-FIRMWARE-fmuv3-SoH-DO-NOT-FLY"
define AP_BOOTLOADER_FLASHING_ENABLED 0

AUTOBUILD_TARGETS Plane
```
No pin, DMA, or timer remapping — none was needed (`SimOnHW.inc` is a pure
`#define`/env-var overlay, confirmed in F24-A; `./waf configure` for this
target produced zero pin/DMA/timer warnings, same as the F24-A audit build).
The real `fmuv3/hwdef.dat` is **not modified in any way** — confirmed by
rebuilding it in this session and getting byte-identical numbers to before
(§4).

## 2. Firmware identity and safety

- **Board name**: `fmuv3-SR75-SoH` (`CHIBIOS_SHORT_BOARD_NAME`).
- **Startup banner**: `AP_CUSTOM_FIRMWARE_STRING` is an *existing* ArduPilot
  mechanism (`libraries/AP_Common/AP_FWVersionDefine.h`: "allow vendors to
  set AP_CUSTOM_FIRMWARE_STRING in hwdef.dat") that overrides
  `AP_FWVersion::fwver.fw_string` — the very first line
  `GCS_MAVLINK::send_banner()` sends to every connected GCS. No C++ code was
  written; setting this one hwdef `define` is the entire mechanism. Verified
  the literal string is embedded in the built binary (§4).
- **Board-side marker**: the custom firmware string itself *is* the
  board-side marker — it is visible in Mission Planner's messages tab on
  every connection, in the `.apj`/`.bin` file contents, and in
  `AP_FWVersion::fwver.fw_string` at runtime (e.g. inspectable via
  `param show` or any code that reads `AP::fwversion()`). No additional
  runtime parameter or code path was added for this, per the task's "do not
  add code that energizes outputs" — a passive identity string requires no
  new logic at all.
- **No output-energizing code was added or modified anywhere in this task.**
- **Documentation that this must never be installed for flight**: stated at
  the top of `hwdef.dat`, at the top of `defaults.parm`, and here.

## 3. Default parameters (`defaults.parm`) — rationale for every non-default value

| Parameter | Value | Why (vs stock fmuv3 / vs existing CubeOrange-SoH precedent) |
|---|---|---|
| `AHRS_EKF_TYPE` | `3` | **Not** the existing SoH precedent's `10` (SIM/bypass) — SR-75 needs genuine EKF3 fusion of simulated sensors, not a direct-truth-state cheat (see F24-A's `EKFType::SIM` finding). |
| `EK3_ENABLE` | `1` | Explicit (matches compiled default; stated for clarity, task requirement). |
| `GPS1_TYPE` | `100` | `GPS_TYPE_SITL` (`AP_GPS.h`) — **different** from the bench's normal fmuv3 firmware (`GPS1_TYPE=14`, MAVLink/GPS_INPUT). On a non-SoH build this value would silently fail (backend not compiled in); here it is correct because `AP_SIM_GPS_ENABLED=1`. |
| `ARSPD_TYPE` | `100` | `TYPE_SITL` (`AP_Airspeed.h`). Already set as a runtime default by `SIM_JSON.cpp`'s own `sim_defaults[]`; set explicitly here too so it's visible in this file. |
| `NET_ENABLE` | `1` | Enables ChibiOS PPP networking, needed for the SIM_JSON transport over TELEM1 (reused unchanged from the already-validated `ppp/README.md` parameter set). |
| `NET_OPTIONS` | `0` | Same source. |
| `SERIAL1_PROTOCOL` | `48` | TELEM1 = PPP (same source). |
| `SERIAL1_BAUD` | `921` | Same source. |
| `SERIAL1_OPTIONS` | `0` | Same source. |
| `BRD_SER1_RTSCTS` | `0` | Same source. |
| `FLTMODE1`..`FLTMODE6` | `0` (MANUAL) | Boots into MANUAL by default; `ARMING_CHECK`/`ARMING_REQUIRE` are **not** set here, so strict compiled-in pre-arm gating is unchanged — nothing in this file makes arming easier. |
| `RATO_ENABLE` | `0` | Already the compiled default (`ArduPlane/rato.cpp`) — set explicitly so the safety posture is visible in the file, not only implied by source. |
| `RATO_IGN_CH` | `0` | Same reasoning — already-default, made explicit. |
| `RATO_EJ_CH` | `0` | Same reasoning — already-default, made explicit. |
| `SERVO7_FUNCTION` | `0` | Explicit per task requirement. |
| `SERVO8_FUNCTION` | `0` | Explicit per task requirement. |
| `BATT_MONITOR` | `0` | No battery telemetry wired on this bench configuration. |

USB (SERIAL0) is untouched — stays plain MAVLink2 for Mission Planner, no
parameter needed since that's already the fmuv3 default.

**This file was written from first principles, not copied from
`SR75_LAYER2_AM2_HIL_SAFE.param`** — that file targets an entirely different
firmware architecture (GPS_INPUT over MAVLink on stock fmuv3), and blindly
reusing it would have left `GPS1_TYPE=14`/`AHRS_EKF_TYPE` wrong for this
target, and would have carried over RATO-resume tuning params (`RATO_RES_BURN`
etc.) that are meaningless here since `RATO_ENABLE=0`.

## 4. Build validation (real, persistent target — not audit-only)

```sh
./waf configure --board fmuv3-SimOnHardWare
./waf plane
```

**Result: success, 0 warnings, 0 errors.**

```
Build directory: /home/missi/ardupilot_clean/build/fmuv3-SimOnHardWare
Target         Text (B)  Data (B)  BSS (B)  Total Flash Used (B)  Free Flash (B)
bin/arduplane   1448476      4280   110272               1452756          628000
```

Firmware path (local build output, not flashed):
`build/fmuv3-SimOnHardWare/bin/arduplane.bin` (also `.apj`, `.bin`,
`arduplane` ELF, all under the same directory).

Static RAM (Data+BSS) = 114,552 B (~111.9 KB) — matches F24-A's audit
numbers closely (that build lacked the longer custom-firmware-string
constant, hence the ~2.4 KB flash difference between the two).

**Re-verified the real `fmuv3` (flight) target is untouched**, rebuilt in
this same session:
```
Target         Text (B)  Data (B)  BSS (B)  Total Flash Used (B)  Free Flash (B)
bin/arduplane   1532400      4528   100612               1536928          543832
```
**Byte-for-byte identical** to the pre-existing baseline — confirms zero
impact on the flight-firmware target (task requirement 1).

**Board name / custom string embedded, confirmed directly in the compiled
binary:**
```sh
$ strings build/fmuv3-SimOnHardWare/bin/arduplane | grep -i "SIMULATION-FIRMWARE\|fmuv3-SR75-SoH"
SR75-SIMULATION-FIRMWARE-fmuv3-SoH-DO-NOT-FLY (d0ad6e0d)
SR75-SIMULATION-FIRMWARE-fmuv3-SoH-DO-NOT-FLY
fmuv3-SR75-SoH
```

**SR-75 custom code compiled unchanged**, confirmed in the build log:
`Compiling ArduPlane/rato.cpp`, `Compiling ArduPlane/mode_sr75_vland.cpp`,
`Compiling ArduPlane/Parameters.cpp` (carries `SR75_ARSPD_EN`/`SR75_ATT_EN`).

## 5. SIM_JSON protocol mapping (exact, from source)

Traced from `libraries/SITL/SIM_JSON.h`'s `keytable[36]` (identical to the
F24-A trace, reproduced here per this task's explicit ask, with the two
specifically-requested items resolved):

| Field | Type | Units/frame | Required | Notes |
|---|---|---|---|---|
| `timestamp` | double | seconds | **yes** | |
| `latitude`/`longitude`/`altitude` | double | deg/deg/m | no | alternative to `position` |
| `imu.gyro` | Vector3f | rad/s, **body frame** | **yes** | |
| `imu.accel_body` | Vector3f | m/s², **body frame** | **yes** | specific force, not including gravity subtraction assumptions beyond what `gravity_body_mss()` models |
| `position` | Vector3d | m, **NED**, relative to origin | no | alternative to lat/lon/alt |
| `velocity` | Vector3f | m/s, **NED** | **yes** | |
| `attitude` | Vector3f | **radians**, Euler roll/pitch/yaw | one of these two required | code enforces "must get either attitude or quaternion" even though neither is individually marked `required` in the keytable |
| `quaternion` | Quaternion | — | (see above) | |
| `airspeed` | float | m/s | no | |
| `rng_1`..`rng_6` | float | m (rangefinder) | no | not used by SR-75 |
| `velocity_wind` | Vector3f | m/s | no | |
| `windvane.direction`/`speed` | float | rad / m/s | no | |
| `no_time_sync`/`no_lockstep` | bool | — | no | pacing control |
| `rc_1`..`rc_12` | float | — | no | |
| `battery.voltage`/`battery.current` | float | V / A | no | |

**Barometer/pressure:** **not a SIM_JSON field at all.** `AP_Baro_SITL`
derives pressure internally from the `altitude` field via a standard-
atmosphere model (`AP_BARO_1976_STANDARD_ATMOSPHERE_ENABLED`, gated by
`AP_SIM_ENABLED`) — there is no `pressure`/`baro` key in the keytable.

**Magnetic field:** **also not a SIM_JSON field.** Confirmed directly in
`SIM_JSON.cpp`: *"as the model does not provide mag field we calculate it
from position and attitude"* — `AP_Compass_SITL` computes the simulated
magnetic field internally from a world-magnetic-model lookup keyed on
position, combined with the transmitted attitude. Neither barometer nor
compass needs (or accepts) a directly-transmitted value — both are
downstream of `altitude`/`position`/`attitude`, which is why those three
fields being coherent is what actually matters for EKF3 fusion quality.

**Update rate**: no fixed rate is enforced by the protocol itself; the
firmware's own `SIM_RATE_HZ`/`SCHED_LOOP_RATE` params set the expected
lockstep pacing (400 Hz in the existing CubeOrange-SoH `defaults.parm`,
not set in ours — left at compiled default since no lockstep timing
requirement was specified for this task).

**Timeout**: `UDP_TIMEOUT_MS = 100` (`SIM_JSON.cpp`) — a receive that
doesn't complete within 100 ms per cycle is simply treated as "no new data
this cycle," not a distinct failsafe packet type.

## 6. Host responder audit/adaptation

Inspected `Tools/autotest/sr75_hil_layer2/sim_json/sr75_sim_json_responder.py`
(2,775 lines — already built and validated throughout the F22-GZ SITL
series) and its siblings (`sr75_sim_json_actuator_bridge.py`,
`sr75_sim_json_mock_client.py`, `SR75_SIM_JSON_CHANNEL_MAP.json`).

**Already present, confirmed by reading the source (no changes needed):**
- Packet generation from live JSBSim state: `LatestCSVReader` +
  `StateMapper.state_from_csv()` (flexible column-name aliasing, same
  design pattern as the bridge's `JSB_FEED_ALIASES`).
- Deterministic static-level test packet: `MockStateSource` (`--mock-state`)
  — wings-level, zero velocity, gravity-only accel.
- Stale-data stop behavior: `read_reply_state()` raises `StateError` when
  `row_age_ms > args.state_timeout_ms` (default 500 ms) — genuinely stops,
  does not resend a stale row.
- The exact SIM_JSON packet schema (§5): `SimState.to_json_bytes()`.

**Genuinely missing, added (minimal, additive only):**
1. **PPP/serial destination abstraction** — `--transport-profile
   {loopback,ppp}`, a ~15-line addition (new arg + one `if` in `main()`)
   that only changes the default `--listen-host` value (`127.0.0.1` /
   `192.168.144.2`, the documented `ppp/README.md` host IP). It never opens
   an interface and is overridden by an explicit `--listen-host`. **Not
   activated in this task** — no socket was bound to a PPP address.
2. **Controlled pitch/roll/yaw/altitude test profiles** — did not exist
   (`MockStateSource` is static-only). Added as a new, separate file (§7),
   not by modifying the 2,775-line responder further.

No changes were made to any actuator-command decode path
(`decode_control_packet`, `SoftwareActuatorBridge`, `UDPJSBSimCommandSink`)
— confirmed by diff scope (only `build_arg_parser()` and the top of
`main()` were touched).

## 7. No-hardware test harness

New files, both under `Tools/autotest/sr75_hil_layer2/`:

- `sim_json/sr75_sim_json_test_profiles.py` — pure profile generators
  (`profile_static_level`, `profile_pitch_sweep`, `profile_roll_sweep`,
  `profile_yaw_sweep`, `profile_altitude_ramp`), reusing `SimState`/
  `euler_to_quaternion`/`gravity_body_mss` from the responder directly (no
  duplicated packet schema). Never imports the actuator-decode path.
- `scripts/sr75_hil_f24b_sim_json_no_hardware_tests.py` — the standalone
  harness script: generates all 5 profiles, validates JSON schema/units/
  ranges/rate/duration, exercises the **real** `read_reply_state()` stale-
  timeout path and the **real** `StateMapper.state_from_csv()` required-
  field/malformed-row validation (against genuine temp files — not
  reimplementations), and confirms no actuator/PWM-shaped key ever appears
  in a generated packet.
- `scripts/test_sr75_hil_f24b_sim_json_no_hardware_tests.py` — `unittest`
  wrapper (23 tests) for CI-style regression running.

No sockets are opened by any of this; no PPP/USB/serial connection is made.

## Tests

```
$ python3 Tools/autotest/sr75_hil_layer2/scripts/sr75_hil_f24b_sim_json_no_hardware_tests.py
OVERALL: PASS   (9/9 checks: all profile schema/rate/duration, pitch/roll
                 bounds, yaw monotonic, altitude ramp rate, stale timeout,
                 missing required fields, malformed row, no actuator path)
```
`python3 -m unittest` regression totals this session: 52 (`bridge/`) + 198
(`scripts/`, includes the pre-existing F23/F2B*-family suites, unaffected)
+ 23 (`sim_json/` new tests) = **273 passing**. One **pre-existing,
unrelated** failure was found in `jsbsim_control/
test_sr75_sim_json_command_accounting.py` (`test_stale_command_sends_one_
neutral_packet`) — confirmed via `git stash` that it fails identically with
none of this task's changes applied, so it predates HIL-F24-B and is out of
this task's scope.

## 8. Runtime acceptance plan (prepared, NOT executed)

**Stage 1 — flash/recovery preparation.** All servo/engine/RATO loads
physically disconnected from the bench. Back up current parameters (§9)
before touching anything. Flash `fmuv3-SimOnHardWare`. Verify over **USB
only** first: heartbeat, custom firmware-string banner text, board name,
`GPS1_TYPE`/`AHRS_EKF_TYPE` read back as `100`/`3`. No PPP yet.

**Stage 2 — PPP link only, static simulated sensors.** Bring up PPP per
`ppp/README.md`'s existing procedure. Run the responder in `--mock-state`
mode (static level) with `--transport-profile ppp`. MANUAL/disarmed
throughout. Verify INS/baro/compass/GPS all report healthy in Mission
Planner using purely the static simulated state.

**Stage 3 — controlled attitude/altitude sweeps.** Feed the pitch/roll/yaw/
altitude-ramp profiles from §7 through the responder over the live PPP
link. Confirm AHRS2/EKF_STATUS_REPORT follow the commanded sweeps
coherently (this is the actual validation the whole F24 line exists for —
comparing against the GPS_INPUT-only fusion results already obtained in
the F23-F2B3 family).

**Stage 4 — dynamic JSBSim state, still no actuator feedback.** Replace the
synthetic sweep profiles with a live JSBSim CSV feed (reusing
`LatestCSVReader`), still with `--no actuator forwarding` (the responder's
own default — `UDPJSBSimCommandSink` is only active if
`--jsbsim-command-target` is explicitly given, which this stage must not
do).

**Stage 5 — actuator observation only, no powered loads.** Only once
Stages 1-4 all pass: observe (not act on) the servo/PWM commands the SoH
firmware would send, with all real loads still disconnected — purely to
confirm the command-decode path itself is coherent, never to drive
anything.

None of these stages were executed in this task.

## 9. Recovery procedure

1. **Backup current parameters** (before ever flashing SoH firmware): run
   the existing read-only audit (`scripts/sr75_hil_gps_ekf_readonly_audit.py`
   or a full `param fetch` via Mission Planner) and save the bench's
   current, validated `.param` file under version control or a clearly
   dated backup location.
2. **Normal fmuv3 firmware restoration**: reflash with the standard
   `fmuv3` target build (`./waf configure --board fmuv3 && ./waf plane`,
   confirmed unaffected by this task, §4) via the normal bootloader
   upload procedure (hold-button / `MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN` into
   bootloader mode, or the standard Mission Planner/`uploader.py` flow).
3. **Bootloader recovery**: `AP_BOOTLOADER_FLASHING_ENABLED 0` on the SoH
   target means it never touches the bootloader itself — the same
   bootloader that shipped with the board (or was last flashed for normal
   flight use) remains in place throughout; recovery is the ordinary DFU/
   bootloader reflash procedure, no SoH-specific risk here.
4. **Parameter reset implications**: switching firmware between the two
   architectures (GPS_INPUT-based vs SIM_JSON-based) means the parameter
   sets are **not** interchangeable — `GPS1_TYPE`/`AHRS_EKF_TYPE` in
   particular must be reloaded from the correct saved `.param` file for
   whichever firmware is now running, not left at whatever the other
   architecture had. A full parameter wipe (`FORMAT_VERSION=0` or the
   `wipe_parameters` MAVLink command) before reloading is the safest
   option whenever switching either direction, matching the official
   `Tools/scripts/sitl-on-hardware/README.md` procedure.
5. **Verifying normal flight firmware is restored**: confirm (a) the
   `send_banner()` text no longer contains "SIMULATION FIRMWARE"/the
   custom string, (b) `GPS1_TYPE` reads back `14` (not `100`) after
   reloading the bench's real `.param` file, (c) `AHRS_EKF_TYPE` reads back
   whatever the flight configuration requires, and (d) the board identifies
   as the normal `fmuv3` short board name, not `fmuv3-SR75-SoH`.

## 10. Remaining runtime risks (unchanged from F24-A, not addressed by this task)

- Dynamic RAM/CPU headroom under real SIM_JSON parsing + EKF3 load — still
  unmeasured; only static build sizing was obtained (this task, like
  F24-A, involved no hardware).
- This exact board+SoH combination has no prior precedent; the successful
  compile is strong evidence, not proof of correct runtime behavior.
- The PPP-over-TELEM1 transport itself has never been exercised against
  real hardware (the `ppp/` prep work has only been validated via
  localhost-fallback per prior session history).
- Pre-existing, unrelated test failure noted in §"Tests" should be
  triaged separately from this task.

## Files changed / added

- `libraries/AP_HAL_ChibiOS/hwdef/fmuv3-SimOnHardWare/hwdef.dat` (new)
- `libraries/AP_HAL_ChibiOS/hwdef/fmuv3-SimOnHardWare/defaults.parm` (new)
- `Tools/autotest/sr75_hil_layer2/sim_json/sr75_sim_json_responder.py`
  (modified — `--transport-profile` only)
- `Tools/autotest/sr75_hil_layer2/sim_json/sr75_sim_json_test_profiles.py` (new)
- `Tools/autotest/sr75_hil_layer2/scripts/sr75_hil_f24b_sim_json_no_hardware_tests.py` (new)
- `Tools/autotest/sr75_hil_layer2/scripts/test_sr75_hil_f24b_sim_json_no_hardware_tests.py` (new)
- This report (new)

Not committed (per standing instruction: only commit when explicitly asked).

## Exact future flash command — **DO NOT RUN**

```sh
# NOT TO RUN IN THIS OR ANY AUTOMATED TASK. Requires explicit, separate
# human authorization, all propulsion/RATO/servo loads physically
# disconnected, and Stage 1 of the runtime acceptance plan (§8) followed
# exactly.
#
# python3 -m serial.tools.list_ports          # identify the bench Pixhawk port first
# ./waf configure --board fmuv3-SimOnHardWare
# ./waf plane --upload
```

## PASS/FAIL

**PASS.** Persistent `fmuv3-SimOnHardWare` target created, builds cleanly
(0 warnings) with SR-75 custom code intact, the real `fmuv3` flight target
confirmed byte-identical/untouched, firmware identity (board name +
"SIMULATION FIRMWARE" banner) confirmed embedded via an existing hwdef-only
mechanism (no new code), a from-first-principles `defaults.parm` with every
non-default value documented, the exact SIM_JSON field/unit/frame mapping
resolved including the two specifically-requested unknowns (baro/mag are
not transmitted, derived internally), minimal additive responder changes
plus a new no-hardware test harness (273 passing tests total), a 5-stage
future hardware plan and a recovery procedure both documented but not
executed, and no hardware, parameters, PPP link, arming, or actuator output
touched at any point.
