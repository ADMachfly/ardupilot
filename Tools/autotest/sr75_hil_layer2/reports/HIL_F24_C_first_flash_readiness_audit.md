# HIL-F24-C: fmuv3-SR75-SoH First Flash & Static-Sensor Bench Test — Readiness Audit

Status: **audit + prep only. No hardware execution.** No flashing, no
`PARAM_SET`, no arming, no actuator output, no engine/fuel/RATO/relay
activity, no PPP hardware connected, no changes to validated flight/aero/
TECS/RATO logic. All commands below are documented for a human operator to
run later; none were executed against real hardware in this task.

## 1. Firmware artifact verification

```
Artifact directory: build/fmuv3-SimOnHardWare/bin/
  arduplane.bin   1,452,768 bytes
  arduplane.apj   1,309,664 bytes
  arduplane       2,725,796 bytes (ELF, for symbol inspection only)
```

**SHA256:**
```
41189cf41f42ecb8c800ee04c591e28f12dd1079a3c3a02379c4861ce435d6f5  arduplane.bin
b31f43c6340c10a881f1b44943eff494445522e1e8b6387bc1eb51149487d8cd  arduplane.apj
```

**Embedded identity strings** (`strings build/fmuv3-SimOnHardWare/bin/arduplane`):
```
SR75-SIMULATION-FIRMWARE-fmuv3-SoH-DO-NOT-FLY (d0ad6e0d)
SR75-SIMULATION-FIRMWARE-fmuv3-SoH-DO-NOT-FLY
fmuv3-SR75-SoH
```

**Normal fmuv3 (flight) artifact confirmed separate**, rebuilt in this
session for direct comparison:
```
SHA256: 305f9dd6889252a5cb769ca2303caed9853cfabe08f005dd3763da71d9e88561  build/fmuv3/bin/arduplane.bin
```
Different path, different SHA256, different embedded identity string (no
`SIMULATION-FIRMWARE` string present in the normal build).

**SR-75 custom code confirmed present** via symbol table
(`arm-none-eabi-nm build/fmuv3-SimOnHardWare/bin/arduplane`):
```
_ZN13ModeSR75VLand6_enterEv
_ZN13ModeSR75VLand6updateEv
_ZN13ModeSR75VLandC1Ev
_ZN14RATOController12resume_boostERK8Locationfffff
_ZN14RATOController19set_ejection_outputEb
_ZL12sr75_fuel_ml / sr75_fuel_empty / sr75_fuel_flow_mlmin
_ZL19sr75_named_value_isPKcS0_
```

## 2. Parameter backup — exact read-only commands

New script: `Tools/autotest/sr75_hil_layer2/scripts/sr75_hil_f24c_param_backup.py`.
Sends only `PARAM_REQUEST_LIST` and a read-only `AUTOPILOT_VERSION` request
— never `PARAM_SET`.

```sh
# BEFORE flashing (run against whatever is currently on the board):
python3 Tools/autotest/sr75_hil_layer2/scripts/sr75_hil_f24c_param_backup.py \
  --pixhawk /dev/ttyACM0 --baud 115200 --label pre-flash

# AFTER flashing (run again once the new firmware is confirmed booted):
python3 Tools/autotest/sr75_hil_layer2/scripts/sr75_hil_f24c_param_backup.py \
  --pixhawk /dev/ttyACM0 --baud 115200 --label post-flash
```

Writes, under `Tools/autotest/sr75_hil_layer2/backups/` (created in this
task, currently empty — no hardware connection was made):
```
sr75_hil_f24c_params_<UTC-timestamp>_pre-flash.parm    (every parameter, NAME<TAB>VALUE)
sr75_hil_f24c_version_<UTC-timestamp>_pre-flash.txt     (heartbeat/AUTOPILOT_VERSION/STATUSTEXT banner)
```
and prints (from the SAME single fetch, no extra requests) the specific
subsets task item 2 asks for: GPS/EKF source (`GPS1_TYPE`, `GPS2_TYPE`,
`AHRS_EKF_TYPE`, `EK3_ENABLE`, `EK3_SRC1_*`, `EK3_SRC2_*`), airspeed
(`ARSPD_TYPE`), serial (`SERIAL0_*`, `SERIAL1_*`, `NET_ENABLE`,
`NET_OPTIONS`, `BRD_SER1_RTSCTS`), servo function (`SERVO7/8_FUNCTION`),
and RATO (`RATO_*`).

## 3. Physical bench checklist (required before flashing)

- [ ] Servo power disconnected
- [ ] Engines/FADEC disconnected
- [ ] Fuel pump disconnected
- [ ] RATO ignition/ejection disconnected
- [ ] Relays/pyro loads disconnected
- [ ] CH7/CH8 loads disconnected (or dummy load/LED/PWM-logger only)
- [ ] Only USB and the *passive* PPP-adapter wiring connected (PPP not yet
      started — wiring present is fine, active link is not required or
      wanted at flash time)
- [ ] SD card inserted (required for dataflash logging if wanted during
      the static test)
- [ ] A separate, known-good recovery USB cable available before starting

## 4. PPP wiring/config audit (from the existing `ppp/README.md`, unchanged)

- **TELEM1 pin mapping**: 3-wire (TX, RX, GND) — `Pixhawk TELEM1 TX → USB-
  UART RX`, `Pixhawk TELEM1 RX → USB-UART TX` (**crossed**, standard UART
  convention), `Pixhawk GND → USB-UART GND` (**required**, common ground).
- **Voltage level**: 3.3 V TTL only. **5 V from the USB-UART adapter must
  stay disconnected** (explicit warning in `ppp/README.md`). RS-232 levels
  must never be used.
- **RTS/CTS**: not required — `BRD_SER1_RTSCTS 0`, hardware flow control
  disabled on both ends.
- **Linux serial interface expected**: `/dev/ttyUSB0` typically, prefer the
  stable `/dev/serial/by-id/...` path when available; identify via `dmesg
  -w` / `ls -l /dev/serial/by-id/`. On WSL2, attach via `usbipd` first.
- **Baud/protocol**: `SERIAL1_PROTOCOL 48` (PPP), `SERIAL1_BAUD 921`
  (921600) on the Pixhawk side; host `sr75_ppp_start.sh --baud 921600`
  (fallback `460800` if unstable).
- **PPP startup** (host side, foreground):
  ```sh
  sudo Tools/autotest/sr75_hil_layer2/ppp/sr75_ppp_start.sh --device /dev/ttyUSB0 --baud 921600
  ```
- **PPP shutdown**:
  ```sh
  sudo Tools/autotest/sr75_hil_layer2/ppp/sr75_ppp_stop.sh --device /dev/ttyUSB0
  ```
- **Link status check**: `Tools/autotest/sr75_hil_layer2/ppp/sr75_ppp_status.sh`,
  `ip addr show dev ppp0`, `ping -c 3 192.168.144.14`.
- Planned IPs: host `192.168.144.2`, Pixhawk `192.168.144.14`, SIM_JSON UDP
  `192.168.144.2:9002`.

None of this was activated in this task.

## 5. Safe defaults audit — `defaults.parm` verified line-by-line

Every parameter task item 5 lists is present and correct in
`libraries/AP_HAL_ChibiOS/hwdef/fmuv3-SimOnHardWare/defaults.parm` (built
in HIL-F24-B, re-verified here by direct read):

| Required | Found | OK |
|---|---|---|
| `AHRS_EKF_TYPE=3` | `AHRS_EKF_TYPE 3` | ✓ |
| `EK3_ENABLE=1` | `EK3_ENABLE 1` | ✓ |
| `GPS1_TYPE=100` | `GPS1_TYPE 100` | ✓ |
| `ARSPD_TYPE=100` | `ARSPD_TYPE 100` | ✓ |
| `SERIAL1_PROTOCOL`/`BAUD` for PPP | `SERIAL1_PROTOCOL 48` / `SERIAL1_BAUD 921` | ✓ |
| `SERVO7_FUNCTION=0` | `SERVO7_FUNCTION 0` | ✓ |
| `SERVO8_FUNCTION=0` | `SERVO8_FUNCTION 0` | ✓ |
| `RATO_ENABLE=0` | `RATO_ENABLE 0` | ✓ |
| RATO ignition/ejection assignments disabled | `RATO_IGN_CH 0` / `RATO_EJ_CH 0` | ✓ |
| Arming manual, disarmed at boot | `FLTMODE1..6 = 0` (MANUAL); `ARMING_CHECK`/`ARMING_REQUIRE` **not weakened** (left at strict compiled defaults); armed state itself is never a persistent parameter — ArduPilot always boots disarmed regardless | ✓ |

No discrepancies found.

## 6. First-boot acceptance (read-only checks, define but do not execute)

1. Custom firmware identity visible — connect via USB MAVLink, read the
   connect-time `STATUSTEXT` banner, confirm it contains
   `SR75-SIMULATION-FIRMWARE-fmuv3-SoH-DO-NOT-FLY`.
2. Board boots without loop/reset — heartbeat arrives within the normal
   ~1-2s window and continues at a steady rate (no repeated
   disconnect/reconnect cycling).
3. USB MAVLink heartbeat present (`HEARTBEAT.type`/`autopilot` sane).
4. `armed=0`, `mode=MANUAL` (from the same heartbeat).
5. Expected parameters loaded — read back all of §5's table via
   `PARAM_REQUEST_READ` (exactly what `--require-soh-params` does in the
   item-10 script, §10).
6. No servo/relay/RATO output activity — `SERVO_OUTPUT_RAW.servo7_raw`/
   `servo8_raw` at/near 0 (or unchanged from a disconnected-channel
   baseline), no output pulses observed.
7. CPU load/free memory/scheduler health — `MEMINFO`/`STATUSTEXT`
   scheduler-overrun warnings if present in the boot log (this task could
   not exercise this — no hardware).
8. **No EKF acceptance is required before SIM_JSON begins** — per task
   spec, first-boot acceptance only requires the vehicle to be alive and
   safe (armed=0, MANUAL, no output activity); EKF convergence is
   evaluated separately in the static-test PASS criteria (§8), only once
   simulated sensor data is actually flowing.

## 7. Static SIM_JSON test — prepared, NOT executed

Reuses `sr75_sim_json_test_profiles.profile_static_level()` (built in
HIL-F24-B, extended in this task with an explicit `airspeed_mps` parameter
so "fixed airspeed" is a real, settable field rather than an implicit
zero):

```python
# Local packet generation only -- prepared, not executed against hardware:
from sr75_sim_json_test_profiles import profile_static_level
rows = profile_static_level(duration_s=30.0, rate_hz=50.0, airspeed_mps=0.0)
```

Matches every task item 7 requirement: level attitude (roll=pitch=yaw=0),
zero body rates (`gyro_rad_s=(0,0,0)`), gravity-consistent accelerometer
(`gravity_body_mss(0,0,0)` = `(0, 0, -9.80665)`), fixed position/altitude
(`BASE_LAT`/`BASE_LON`/`BASE_ALT_M`), zero NED velocity, fixed airspeed
(configurable, default 0.0 for a stationary ground test), 50 Hz update rate
(within the requested 50-100 Hz band), 30 s duration.

Once PPP is up (not in this task), the exact command to actually transmit
this to the real Pixhawk would be (using the responder's existing
`--mock-state` static path, or a small future adaptation feeding
`profile_static_level()`'s rows through the same UDP reply mechanism):
```sh
python3 Tools/autotest/sr75_hil_layer2/sim_json/sr75_sim_json_responder.py \
  --transport-profile ppp --listen-port 9002 \
  --mock-state --verbose
```
(`--mock-state` already reproduces a static/level/zero-rate/zero-velocity
state — matching this test's intent using existing, already-validated
code; a dedicated `--profile static_level` wiring into the responder for
the exact 30 s/50 Hz/airspeed-configurable variant is noted as follow-up
scope, not built in this task since it is not required to prepare the
command.)

## 8. Static-test PASS criteria (defined, not evaluated — no hardware run)

- PPP link remains stable (`ip addr show dev ppp0` stays up, no drops).
- SIM_JSON packets accepted (responder log shows no `STALE_STATE`/decode
  errors; firmware side shows no repeated `UDP_TIMEOUT_MS` gaps).
- INS, gyro, accelerometer, barometer, compass, GPS, and airspeed all
  report healthy (no `EKF_GPS_GLITCHING`, no sensor-unhealthy STATUSTEXT).
- AHRS2/EKF roll/pitch/yaw match the commanded static state (0/0/0,
  within normal EKF noise).
- Altitude and position match the commanded fixed values.
- No scheduler overruns (`STATUSTEXT`/`PERF` messages clean).
- No heap/stack failure (no `Internal Errors`/`STATUSTEXT` panic messages).
- No output activity (`SERVO_OUTPUT_RAW` unchanged from baseline).
- `armed=0`, `mode=MANUAL` throughout the full 30 s window.

## 9. Failure and recovery procedure

- **No heartbeat after flash**: check USB connection/cable, confirm the
  board actually left bootloader mode (LED pattern), retry
  `sr75_hil_f24c_preflash_precheck.py`; if still silent, proceed to
  bootloader recovery below.
- **Boot loop**: disconnect, attempt bootloader-mode reconnect (hold-button
  or short BOOT pin per board convention), reflash the SoH image again; if
  it recurs, reflash the **normal `fmuv3`** image instead and treat the SoH
  build as suspect pending investigation.
- **PPP failure**: `sr75_ppp_stop.sh`, re-check wiring/voltage (§4), retry
  at the fallback baud (460800), confirm `SERIAL1_PROTOCOL`/`NET_ENABLE`
  actually match §5's table via a read-only param check before assuming a
  wiring fault.
- **Parameter load failure** (defaults.parm didn't take): re-run
  `sr75_hil_f24c_param_backup.py --label post-flash`, diff against §5's
  expected table; if wrong, this indicates the flashed image did not embed
  the intended `defaults.parm` — do not proceed to PPP/SIM_JSON testing.
- **EKF failure** (stuck unhealthy / `EKF_CONST_POS_MODE`-equivalent):
  stop, do not proceed to Stage 3+ of the runtime acceptance plan; this is
  exactly the failure mode HIL-F23-F2B3A/C investigated at length for the
  GPS_INPUT architecture — expect a similar diagnostic approach (decode
  `EKF_STATUS_REPORT.flags`, check `AHRS_EKF_TYPE` actually reads back 3,
  not defaulted/reverted to something else).
- **Firmware restore to normal fmuv3**: `./waf configure --board fmuv3 &&
  ./waf plane`, reflash `build/fmuv3/bin/arduplane.bin` (rebuilt in this
  session, confirmed byte-identical to the pre-existing baseline, SHA256
  in §1) via the normal bootloader upload procedure.
- **Bootloader recovery**: unaffected by any of this work
  (`AP_BOOTLOADER_FLASHING_ENABLED 0` on the SoH target never touches the
  bootloader) — use the board's standard DFU/bootloader reflash procedure
  if the application firmware itself is unreachable.
- **Restoring saved parameters**: after reflashing normal `fmuv3`, do a
  full parameter wipe (`FORMAT_VERSION=0` or `wipe_parameters`) then reload
  the bench's own validated `.param` file (not any backup captured while
  the SoH firmware was running — those reflect the SoH parameter
  architecture, not the flight one).
- **Confirming normal flight firmware is restored**: re-run
  `sr75_hil_f24c_preflash_precheck.py` **without** `--require-soh-params`
  and separately confirm (a) the boot banner no longer contains
  "SIMULATION FIRMWARE", (b) `GPS1_TYPE` reads back `14` (not `100`) once
  the bench `.param` file is reloaded, (c) board identifies as `fmuv3`,
  not `fmuv3-SR75-SoH`.

## 10. Operator checklist/script

New: `Tools/autotest/sr75_hil_layer2/scripts/sr75_hil_f24c_preflash_precheck.py`.

- **Read-only only**: sends `PARAM_REQUEST_READ`, one read-only
  `MAV_CMD_SET_MESSAGE_INTERVAL` (SERVO_OUTPUT_RAW stream-rate request),
  and one read-only `MAV_CMD_REQUEST_MESSAGE` (AUTOPILOT_VERSION). Nothing
  else.
- **Never flashes** — has no upload/flash capability at all.
- **Never changes parameters** — no `PARAM_SET` anywhere in the file.
- **Never starts PPP or SIM_JSON** — never touches `SERIAL1`/`TELEM1`
  configuration, never opens a UDP socket.
- Prints an explicit `GO`/`STOP` verdict with itemized reasons.

Usage (not executed in this task):
```sh
# PRE-flash (confirm current board state is safe to proceed):
python3 Tools/autotest/sr75_hil_layer2/scripts/sr75_hil_f24c_preflash_precheck.py \
  --pixhawk /dev/ttyACM0 --baud 115200

# POST-flash (also verify the new firmware's parameters/banner):
python3 Tools/autotest/sr75_hil_layer2/scripts/sr75_hil_f24c_preflash_precheck.py \
  --pixhawk /dev/ttyACM0 --baud 115200 --require-soh-params
```

## Files changed / added

- `Tools/autotest/sr75_hil_layer2/scripts/sr75_hil_f24c_param_backup.py` (new)
- `Tools/autotest/sr75_hil_layer2/scripts/sr75_hil_f24c_preflash_precheck.py` (new)
- `Tools/autotest/sr75_hil_layer2/scripts/test_sr75_hil_f24c_preflash_precheck.py` (new, 13 tests)
- `Tools/autotest/sr75_hil_layer2/backups/README.md` (new, empty backup dir prep)
- `Tools/autotest/sr75_hil_layer2/sim_json/sr75_sim_json_test_profiles.py`
  (modified — `profile_static_level()` gained an `airspeed_mps` parameter,
  backward-compatible default `0.0`)
- This report (new)

Not committed (only commit when explicitly asked).

## Tests

New: 13/13 pass (`test_sr75_hil_f24c_preflash_precheck.py`, pure
`evaluate_precheck()` logic — armed/mode/CH7-8/param/banner gating, each
independently tested).

Full regression this session: 52 (`bridge/`) + 211 (`scripts/`) = **263
passing** (up from 250 pre-existing before this task's 13 new tests), 0
new failures. The one pre-existing, unrelated failure noted in HIL-F24-B
(`jsbsim_control/test_sr75_sim_json_command_accounting.py`) is unaffected
and out of this task's scope.

## Final GO/NO-GO readiness verdict

**GO for the *procedure*, NO-GO for *execution* in this task** (by design —
this task explicitly forbids hardware execution). Every prerequisite this
audit could verify statically is satisfied: the firmware artifact exists,
is correctly identified, differs from the flight firmware, and contains
the SR-75 custom code; the parameter set is fully audited against every
required safety value with no discrepancies; the physical/PPP wiring
procedure is fully specified from already-validated documentation; backup,
first-boot, static-test, and recovery procedures are all defined; and a
tested, read-only, flash-incapable operator script exists to gate the
actual future session. **The next action is a human-authorized real bench
session following this document's §2 (backup) → §3 (physical checklist) →
§10 (precheck, expect GO) → flash → §10 again with `--require-soh-params`
→ §4 (PPP) → §7/§8 (static test).**

## PASS/FAIL

**PASS.**
