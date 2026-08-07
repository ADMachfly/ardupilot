# HIL-F24-R3I: Onboard DataFlash Check for the EKF3 Bootstrap Transient-Accel Hypothesis

Diagnostic only. No firmware, parameter, threshold, control, or
trajectory was changed. All hardware interaction was read-only: the
standard MAVLink log-transfer protocol (`LOG_REQUEST_LIST`,
`LOG_REQUEST_DATA`, `LOG_REQUEST_END` -- the same protocol MAVProxy's/
QGC's "download log" feature uses, which only reads a stored file back
and never writes to, erases, or otherwise modifies the vehicle) and
`PARAM_REQUEST_READ`. No `PARAM_SET`, arm, mode change, mission item,
RC override, actuator command, or `LOG_ERASE` was ever sent.

## Whether existing DataFlash is sufficient

**No -- confirmed live, and now root-caused.** `LOG_REQUEST_LIST`
against the currently-attached Pixhawk (`/dev/ttyACM0`) returned AP_
Logger's own "no logs" sentinel (`LOG_ENTRY` with `num_logs=0`), not a
single stored log. A follow-up **read-only** `PARAM_REQUEST_READ` of
the relevant params explains why:

| Param | Live value | Expected (per this bench's `.param` files) |
|---|---|---|
| `LOG_BACKEND_TYPE` | **1** (File/SD dataflash) | 1 |
| `LOG_BITMASK` | **65535** (all logging types) | 65535 |
| `LOG_DISARMED` | **0** | **1** (per `SR75_LAYER2_HIL.param`, `SR75_LAYER2_AM2_HIL_SAFE.param`, `closed_loop/SR75_LAYER2J_B3_FBWA.param`) |
| `LOG_FILE_BUFSIZE` | 16 | -- |
| `LOG_REPLAY` | 0 | -- |

**Root cause of the empty DataFlash: `LOG_DISARMED` is live-set to 0,
not 1 as every param file for this bench intends.** With
`LOG_DISARMED=0`, AP_Logger only opens/writes a log file while the
vehicle is *armed* -- and this bench's entire safety posture is
disarmed-only (never arm, MANUAL mode, LOG_ONLY actuators). So every
boot on this bench, including the one that produced R3G's frozen
7.4°/7.5° AHRS2 reading, has logged **nothing at all** to the SD card,
regardless of `LOG_BACKEND_TYPE`/`LOG_BITMASK` both being correctly
configured for full logging. This is a live-parameter drift from the
intended bench configuration, not a hardware or SD-card-presence
problem -- `LOG_BACKEND_TYPE=1` confirms an SD card is fitted and the
File backend is active; it simply has never been triggered to write
because the vehicle never arms and `LOG_DISARMED` is 0.

## Exact log messages/fields (would have been used, had a log existed)

- **`XKF1`** (`libraries/AP_NavEKF3/LogStructure.h:436`): `TimeUS, C,
  Roll, Pitch, Yaw, VN, VE, VD, dPD, PN, PE, PD, GX, GY, GZ, OH` --
  `Roll`/`Pitch` already in degrees (format chars `cc`, int16
  centidegrees auto-scaled by DFReader), per EKF3 core (`C`). Gated on
  `EK3_LOG_LEVEL` (`AP_NavEKF3.cpp:729`, default 0 = full logging --
  confirmed unset/default on this bench) plus `LOG_BITMASK`'s attitude
  bits, both of which check out.
- **`IMU`** (`libraries/AP_InertialSensor/LogStructure.h:138`):
  `TimeUS, I, GyrX, GyrY, GyrZ, AccX, AccY, AccZ, EG, EA, T, GH, AH,
  GHz, AHz` -- filtered, main-loop-rate accel/gyro per instance (`I`).
  Gated on `MASK_LOG_IMU` (bit 7, `ArduPlane/defines.h:110`), included
  in `LOG_BITMASK=65535`. Note: **raw/unfiltered full-rate IMU samples
  are a separate bit, `MASK_LOG_IMU_RAW` (bit 19, value 524288,
  `ArduPlane/defines.h:119`), which a 16-bit `LOG_BITMASK` value cannot
  reach** -- only the filtered "IMU" message would have been available,
  not a bit-exact raw capture.
- **`MSG`** (`libraries/AP_Logger/LogStructure.h:1198`): `TimeUS, ID,
  Seq, Message` -- mirrors every `gcs().send_text()`/`GCS_SEND_TEXT`
  call (`GCS_Common.cpp:2599`, `Write_Message()`), including the exact
  `"EKF3 IMU%u initialised"` text this whole investigation is anchored
  on.

## Command to retrieve/analyze the latest boot log

New tool built for this task (read-only; never sends `LOG_ERASE`):

```
cd Tools/autotest/sr75_hil_layer2/scripts
python3 sr75_hil_f24r3i_dataflash_bootstrap_check.py \
    --pixhawk /dev/ttyACM0 \
    --output-bin ../hardware_sessions/r3i_dataflash_check_$(date -u +%Y%m%dT%H%M%SZ)/latest.bin
```

It lists onboard logs (`LOG_REQUEST_LIST`), downloads the highest-id
(most recent) one (`LOG_REQUEST_DATA`, retried per-chunk), then
analyzes it offline with pymavlink's `DFReader` -- finding the first
`XKF1` row exceeding `--divergence-threshold-deg` (default 1.0°), the
nearest `IMU` sample at/before that time, and reproducing the exact
`AP_NavEKF3_core.cpp:514-524` tilt-from-accel formula (reused,
unmodified, from `sr75_hil_f24r3g_ekf3_bootstrap_trace.py`) to check
whether it explains the observed XKF1 freeze -- printing the same
`TRACE_RESULT ...` line format as R3H's live tool.

## Result: logs already exist?

**No.** Run live against this bench's Pixhawk just now:
```
=== HIL-F24-R3I dataflash bootstrap check (read-only; --pixhawk /dev/ttyACM0) ===
Heartbeat OK: type=1 autopilot=3
Requesting log list (LOG_REQUEST_LIST -- read-only)...
RESULT: NO onboard logs found (no SD card fitted, or LOG_BACKEND_TYPE
does not include File). DataFlash is NOT sufficient -- fall back to a
live capture (HIL-F24-R3H) across an operator reboot.
```
(That printed message's SD-card-absence framing turned out to be the
less likely of its two stated causes -- the follow-up `LOG_BACKEND_
TYPE=1` param read confirms an SD card/File backend IS active; the
actual cause is `LOG_DISARMED=0`, per the table above.)

## Minimum next diagnostic, since DataFlash is currently insufficient

This is a param-drift finding, not a code finding, and this task's
constraint is read-only diagnosis -- no `PARAM_SET` was or should be
sent here. Two independent paths forward, neither executed in this
task:

1. **If/when the bench's `LOG_DISARMED=1` is (re-)applied** (e.g. the
   next time `SR75_LAYER2_HIL.param`/`SR75_LAYER2_AM2_HIL_SAFE.param`
   is actually pushed to this board, correcting the drift found here)
   and the board is rebooted, this same `sr75_hil_f24r3i_dataflash_
   bootstrap_check.py` can be re-run afterward -- no live capture
   window needed, no operator standing by during the reboot -- to pull
   that boot's `XKF1`/`IMU`/`MSG` trace and answer HIL-F24-R3H's
   outstanding item 1 (genuine first-divergence timestamp) directly
   from the log.
2. **Until then**, HIL-F24-R3H's live USB MAVLink trace
   (`sr75_hil_f24r3g_ekf3_bootstrap_trace.py`), run by the operator
   starting *before* their next manual power-cycle, remains the only
   available path to a live first-divergence timestamp.

## Files referenced

- `Tools/autotest/sr75_hil_layer2/scripts/sr75_hil_f24r3i_dataflash_bootstrap_check.py` (new)
- `Tools/autotest/sr75_hil_layer2/scripts/test_sr75_hil_f24r3i_dataflash_bootstrap_check.py` (new, 18 tests passing)
- `Tools/autotest/sr75_hil_layer2/scripts/sr75_hil_f24r3g_ekf3_bootstrap_trace.py` (`tilt_from_accel()`/`compare_predicted_to_observed()`, reused unmodified)
- `libraries/AP_NavEKF3/LogStructure.h` (`XKF1`), `libraries/AP_InertialSensor/LogStructure.h` (`IMU`), `libraries/AP_Logger/LogStructure.h` (`MSG`)
- `libraries/AP_NavEKF3/AP_NavEKF3.cpp:723-729` (`EK3_LOG_LEVEL`)
- `ArduPlane/defines.h:103-122` (`LOG_BITMASK` bit definitions)
- `SR75_LAYER2_HIL.param`, `SR75_LAYER2_AM2_HIL_SAFE.param`, `closed_loop/SR75_LAYER2J_B3_FBWA.param` (intended `LOG_DISARMED=1`, vs. live-read `0`)
