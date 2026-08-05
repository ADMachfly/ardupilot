# HIL-F24-H: TELEM1 PPP No-Traffic Diagnosis (Diagnostic Only)

Status: **root cause found, high confidence, source-proven**. No hardware
was touched, no parameter was set, nothing was flashed. This report is
pure source analysis plus a proposed (not applied) one-line hwdef patch.

## Root-cause candidates, ranked by evidence

### 1. `AP_NETWORKING_BACKEND_PPP` is compiled out for this target (confirmed from source, highest confidence)

`fmuv3-SimOnHardWare` never compiles in ArduPilot's PPP networking backend
at all. `SERIAL1_PROTOCOL=48` is accepted and stored as a valid parameter
value, but **no code in the firmware image ever acts on it** — the UART
is opened generically and nothing reads or writes PPP/LCP frames to it.
This is not a wiring, baud, parameter, or hardware issue; it is a
compile-time configuration gap in the hwdef chain. Full evidence trail:

- `libraries/AP_Networking/AP_Networking_Config.h:11-18` — when nothing
  else defines `AP_NETWORKING_ENABLED`, it falls back to
  `(CONFIG_HAL_BOARD == HAL_BOARD_LINUX) || (CONFIG_HAL_BOARD == HAL_BOARD_SITL)`.
  For a ChibiOS hardware build (`fmuv3`/`fmuv3-SimOnHardWare`),
  `CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS`, so this evaluates to **0**.
- `libraries/AP_Networking/AP_Networking_Config.h:34-36` —
  `AP_NETWORKING_BACKEND_PPP` is defined as
  `(AP_NETWORKING_BACKEND_DEFAULT_ENABLED && (CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS) && !HAL_USE_MAC)`,
  and `AP_NETWORKING_BACKEND_DEFAULT_ENABLED` defaults to
  `AP_NETWORKING_ENABLED` (line 20-22) — i.e. **0**, so
  `AP_NETWORKING_BACKEND_PPP` is also **0**.
- Nothing in the hwdef chain overrides this. Checked exhaustively:
  - `libraries/AP_HAL_ChibiOS/hwdef/fmuv3/hwdef.dat` — zero occurrences of
    `NETWORK`, `PPP`, or `ETH1` (grep-confirmed).
  - `libraries/AP_HAL_ChibiOS/hwdef/fmuv3-SimOnHardWare/hwdef.dat` — only
    `include ../fmuv3/hwdef.dat` and `include ../include/SimOnHW.inc`,
    plus board-identity/bootloader defines; no networking define.
  - `libraries/AP_HAL_ChibiOS/hwdef/include/SimOnHW.inc` — 27 lines, none
    of them networking-related (it disables `HAL_NAVEKF2_AVAILABLE`,
    `HAL_ADSB_ENABLED`, etc. for flash/RAM budget; sets `SIM_ENABLED 1`).
  - `libraries/AP_HAL_ChibiOS/hwdef/scripts/chibios_hwdef.py:868-874,
    939-952, 1013-1014` — the build script's own
    `enable_networking()`/auto-`AP_NETWORKING_ENABLED 1` path only fires
    if the board has an `ETH1` pin (fmuv3 has none) or the hwdef text
    itself already contains `define AP_NETWORKING_ENABLED 1` (it doesn't).
- Consequence, traced through every gate:
  - `libraries/AP_Networking/AP_Networking.h:6` / `.cpp:4` — the entire
    `AP_Networking` class (including its `NET_*` parameter table,
    `var_info[]` at `.cpp:35`) is `#if AP_NETWORKING_ENABLED`-gated —
    **does not exist in this firmware image**. `NET_ENABLE`/`NET_OPTIONS`
    are not real parameters in this build.
  - `libraries/AP_Vehicle/AP_Vehicle.h:450` (member declaration),
    `AP_Vehicle.cpp:216-251` (param registration),
    `AP_Vehicle.cpp:387-388` (`networking.init()` call), and
    `AP_Vehicle.cpp:662-663` (10 Hz `update()` scheduler task) are **all**
    individually `#if AP_NETWORKING_ENABLED`-gated — none of this exists
    either.
  - `libraries/AP_Networking/AP_Networking_PPP.cpp:4` /
    `AP_Networking_PPP.h:5` — gated on `AP_NETWORKING_BACKEND_PPP` — the
    entire PPP backend (LWIP `pppos_create`, the dedicated `"ppp"` thread,
    `ppp_connect()`, the `"PPP[%u]: started"` STATUSTEXT) **does not
    exist**.
  - `libraries/AP_SerialManager/AP_SerialManager.cpp:579-582` — the
    `case SerialProtocol_PPP: break;` is itself
    `#if AP_NETWORKING_BACKEND_PPP`-gated and doesn't exist either, so
    `SERIAL1_PROTOCOL=48` falls through to
    `AP_SerialManager.cpp:586-587`'s `default: uart->begin(state[i].baudrate());`
    — the UART is opened at the configured baud and then never touched
    again by anything.

This single gap fully explains every confirmed symptom: the FC never
transmits (nothing drives the UART as PPP → "raw adapter RX reads 0
bytes"), the FC never responds to LCP (nothing parses incoming bytes as
PPP → host pppd's "LCP: timeout sending Config-Requests"), and every
hardware/wiring/parameter check legitimately passes, because none of them
are wrong.

### 2. Corroborating evidence: `defaults.parm` already assumes the broken parameter exists

`libraries/AP_HAL_ChibiOS/hwdef/fmuv3-SimOnHardWare/defaults.parm:56`
sets `NET_ENABLE 1` — a default value for a parameter that (per
candidate 1) **is not registered in this firmware at all**. This is not
itself the defect (a default for a non-existent parameter is silently
ignored at boot, it does not cause a fault), but it is strong
corroborating evidence: it shows the original intent was for networking
to be enabled, and traces to a pre-existing, unchanged reference —
`Tools/autotest/sr75_hil_layer2/ppp/SR75_SoH_JSON_PPP_TELEM1.param`
(labelled "SR-75 Layer 2I-B7 planned ... parameters") — which itself
already assumed `NET_ENABLE`/`SERIAL1_PROTOCOL=48` alone would be
sufficient, without ever verifying the separate, compile-time
`AP_NETWORKING_BACKEND_PPP` hwdef gate. The gap was carried forward
unmodified into the SimOnHardWare `defaults.parm`.

### 3. Ruled out (checked, not the cause)

- **SERIAL1 physical mapping** (task item 3): not misrouted. See mapping
  table below — confirmed correct and unmodified by the custom target.
- **Baud conversion of `SERIAL1_BAUD=921`**: `AP_SerialManager.cpp:757`
  — `case 921: return 921600;` (a special-cased table entry, not a ×1000
  generic multiply) — resolves correctly to 921600, matching the
  adapter's confirmed-working loopback baud.
- **RTS/CTS**: `BRD_SER1_RTSCTS=0` disables hardware flow control in
  software regardless of the physical CTS/RTS pins existing on USART2
  (`PD3`/`PD4` per the base hwdef) — already confirmed not in use, and
  irrelevant to this defect regardless (the UART is never driven as PPP
  either way).
- **DMA conflicts**: `fmuv3-SimOnHardWare/hwdef.dat` and `SimOnHW.inc` add
  zero `DMA_NOSHARE`/`DMA_PRIORITY` overrides; only the base
  `fmuv3/hwdef.dat`'s pre-existing, unmodified DMA config applies (its
  only override, `DMA_NOSHARE USART6_TX ADC1`, is for USART6/IOMCU, not
  USART2).
- **Conflict with SIM_JSON/console/MAVLink/another serial owner**:
  SERIAL0=OTG1 (USB MAVLink, confirmed working) and SERIAL1=USART2 are
  distinct, non-overlapping owners. SIM_JSON itself is a UDP/IP protocol,
  not tied to any specific UART — on real hardware it is designed to ride
  over the IP link PPP would establish, so it is *downstream of*, not in
  conflict with, TELEM1/PPP.

## Exact SERIAL1 → UART → connector mapping

| ArduPilot index | hwdef `SERIAL_ORDER` entry | UART peripheral | Physical connector | Source |
|---|---|---|---|---|
| SERIAL0 | `OTG1` | USB OTG1 | USB | `fmuv3/hwdef.dat:132` |
| **SERIAL1** | **`USART2`** | **USART2** | **TELEM1** | `fmuv3/hwdef.dat:132` (order), `:302-306` (pins: `PD3` CTS, `PD4` RTS, `PD5` TX, `PD6` RX), comment `# USART2 serial2 telem1` |
| SERIAL2 | `USART3` | USART3 | TELEM2 | `fmuv3/hwdef.dat:132,309-313` |
| SERIAL3 | `UART4` | UART4 | GPS1 | `fmuv3/hwdef.dat:132,175-177` |
| SERIAL4 | `UART8` | UART8 | GPS2 | `fmuv3/hwdef.dat:132,336-338` |
| SERIAL5 | `UART7` | UART7 | (telem/aux) | `fmuv3/hwdef.dat:132,350-352` |

`fmuv3-SimOnHardWare/hwdef.dat` does not contain a `SERIAL_ORDER` line,
any `PD*`/`PA*`/pin definition, or an `UNDEF`/redefinition of any of the
above — the entire table is inherited unchanged from `../fmuv3/hwdef.dat`
via its single `include` line (`fmuv3-SimOnHardWare/hwdef.dat:26`).
**SERIAL1 = USART2 = TELEM1 is confirmed correct; there is no accidental
remap** (task item 3 answered: no defect here).

The hwdef comment text itself says "USART2 **serial2** telem1" — the
"serial2" wording is leftover PX4/NuttX-era terminology (that project's
internal `/dev/ttyS2` device numbering), not ArduPilot's own
`SERIALx` parameter numbering; it does **not** indicate USART2 maps to
ArduPilot's `SERIAL2`. `SERIAL_ORDER`'s list position (index 1, 0-based)
is what actually determines the `SERIALx` parameter number, and that
unambiguously makes USART2 = `SERIAL1`.

## PPP initialization call chain (as it would run if compiled in)

Traced from `libraries/AP_Networking/AP_Networking_PPP.cpp` (all of which
is presently **not compiled in** for this target):

1. `AP_Vehicle::init_ardupilot()` → `networking.init()`
   (`AP_Vehicle.cpp:388`, itself gated on `AP_NETWORKING_ENABLED`).
2. `AP_Networking::init()` constructs the configured backend(s); for
   `SerialProtocol_PPP` this is `AP_Networking_PPP`.
3. `AP_Networking_PPP::init()` (`AP_Networking_PPP.cpp:184-241`) calls
   `AP::serialmanager().find_serial(SerialProtocol_PPP, i)` to locate the
   UART configured with `SERIALx_PROTOCOL=48` (`:195`), creates the LWIP
   PPP control block (`pppos_create`, `:221`), and — if successful —
   spawns a dedicated `"ppp"` thread (`:235-237`) and emits
   `GCS_SEND_TEXT(MAV_SEVERITY_INFO, "PPP[%u]: started", ...)` (`:231`).
4. The `"ppp"` thread (`ppp_loop`, `:324-369`) calls
   `uart->begin(baud, PPP_BUFSIZE_RX, PPP_BUFSIZE_TX)` (`:342`) — **this
   is the actual `uart->begin()` for the PPP UART**; `AP_SerialManager`'s
   own init loop deliberately does nothing for `SerialProtocol_PPP`
   (`AP_SerialManager.cpp:580-581`, empty `case`) precisely because
   `AP_Networking_PPP` owns and re-opens the UART itself with
   PPP-appropriate buffer sizes.
5. `restart_instance()` (`:374-437`) calls `ppp_connect(inst.ppp, 0)`
   (`:412`/`:429`) — **this is what starts LCP negotiation**, i.e. what
   would make the Pixhawk autonomously begin sending LCP Config-Requests
   after boot.
6. `update_instance()` (`:513-563`) is polled continuously from the same
   thread: reads UART RX bytes, feeds them to `pppos_input()` (`:534`),
   and detects `PPPERR_PEERDEAD`/timeout to trigger a reconnect.

**Answer to task item 2**: yes — *if* `AP_NETWORKING_BACKEND_PPP` were
compiled in, `SERIAL1_PROTOCOL=48` alone (no other action) is sufficient
to make the firmware autonomously call `ppp_connect()` and begin emitting
LCP traffic on TELEM1 within a few hundred milliseconds of boot, entirely
without user intervention. The mechanism is real and correctly designed;
it simply isn't present in this compiled firmware.

## Comparison against normal fmuv3 and a known-working SimOnHardWare target (task item 4)

| | `fmuv3` (normal flight) | `fmuv3-SimOnHardWare` (this target) | `CubeOrangePlus-SimOnHardWare` (comparable SimOnHW precedent) |
|---|---|---|---|
| Base hwdef networking define | none | none (inherited from `fmuv3`) | **`define AP_NETWORKING_BACKEND_PPP 1`** — `CubeOrangePlus/hwdef.dat:103` |
| `ETH1` pin present | no | no | no (checked — the define is explicit, not auto-detected) |
| `AP_NETWORKING_BACKEND_PPP` at compile time | 0 | **0** | **1** |
| SimOnHardWare overlay (`SimOnHW.inc`) | n/a | included, unmodified, no networking content | included, unmodified, no networking content |
| `defaults.parm` sets `NET_ENABLE 1` | n/a | yes (`fmuv3-SimOnHardWare/defaults.parm:56`) — **has no effect**, parameter does not exist | not applicable to this comparison (CubeOrangePlus-SimOnHardWare's own defaults.parm was not audited here; out of scope) |
| PPP actually functional | n/a (not attempted on flight fw) | **No — confirmed root cause** | Yes, expected to work (compile-time gate present) |

**The defect is precisely and only this**: `CubeOrangePlus/hwdef.dat`
carries the one line `fmuv3/hwdef.dat` (and therefore
`fmuv3-SimOnHardWare/hwdef.dat`) never acquired. Every other relevant
setting (`SERIAL1_PROTOCOL`, `SERIAL1_BAUD`, `SERIAL1_OPTIONS`,
`BRD_SER1_RTSCTS`, RATO/SERVO7-8 disablement) is correct and matches
across targets.

(For completeness: `Pixhawk6X-PPPGW` and `CubeRedPrimary`/
`CubeRedSecondary` also carry `define AP_NETWORKING_BACKEND_PPP 1` in
their base hwdefs — four independent, real ArduPilot-mainline precedents
for this exact one-line pattern being both necessary and sufficient for
a non-Ethernet ChibiOS board. `Pixhawk6X-PPPGW` is an `AP_Periph` target
with several additional `AP_PERIPH_*` defines not relevant to a vehicle
firmware like `fmuv3`; `CubeOrangePlus`/`CubeRedPrimary`/`CubeRedSecondary`
are ordinary vehicle firmware, the same family as `fmuv3`, and use
nothing beyond the one-line define — this is the directly comparable
precedent.)

## Diagnostic instrumentation (task item 5): none needed

No new C++ instrumentation is required — stock, already-compiled
ArduPilot mechanisms are sufficient to confirm every piece task item 5
asks for, once cross-referenced against the source proof above:

- **PPP backend created/not created**: the absence of a `"PPP[0]:
  started"` STATUSTEXT after boot (would come from
  `AP_Networking_PPP.cpp:231`, USB MAVLink-visible) is consistent with —
  though, given the code doesn't exist, not independently required to
  confirm — the root cause. The **decisive** check is below.
- **Selected UART instance / configured baud**: fully determined from
  source (USART2 / 921600); not runtime-queryable in stock ArduPilot
  (there is no MAVLink message that reports hwdef pin mapping), and
  doesn't need to be — the hwdef *is* the source of truth here.
- **Bytes TX/RX counters**: `AP_HAL::UARTDriver::get_total_tx_bytes()`/
  `get_total_rx_bytes()` (`libraries/AP_HAL_ChibiOS/UARTDriver.h:293-294`)
  are already logged to dataflash as the `UART` message
  (`log_stats()`, `libraries/AP_HAL/UARTDriver.cpp:196-`, gated on
  `HAL_UART_STATS_ENABLED`, which defaults on and is not disabled by
  `SimOnHW.inc`) — **already available with zero code changes**,
  provided an SD card is fitted and a dataflash log is pulled after a
  bench run.
- **Initialization failure reason**: N/A — initialization is never
  attempted at all (the code path doesn't exist), so there is no failure
  reason to surface; this is itself the finding.

## Read-only bench verification commands (task item 6)

All commands below are read-only: no `PARAM_SET`, no arm, no mode change,
no actuator output. Recommended order:

```sh
# 1. DECISIVE check: does NET_ENABLE exist at all in this firmware?
#    If AP_NETWORKING_ENABLED is really 0, this parameter is not
#    registered and PARAM_REQUEST_READ will get no PARAM_VALUE reply.
#    (Uses the existing read-only pattern already validated in
#    sr75_hil_f24c_preflash_precheck.py -- no new script needed.)
python3 -c "
from pymavlink import mavutil
m = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
m.wait_heartbeat(timeout=30)
m.mav.param_request_read_send(m.target_system, m.target_component, b'NET_ENABLE', -1)
msg = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=5)
print('NET_ENABLE PARAM_VALUE:', msg)
"

# 2. Capture STATUSTEXT for 10 s immediately after a fresh boot/reconnect,
#    looking for the (expected-absent) 'PPP[0]: started' message:
python3 -c "
from pymavlink import mavutil, time
m = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
m.wait_heartbeat(timeout=30)
deadline = time.time() + 10
while time.time() < deadline:
    msg = m.recv_match(type='STATUSTEXT', blocking=True, timeout=0.5)
    if msg:
        print('STATUSTEXT:', msg.text)
"

# 3. Confirm SERIAL1_PROTOCOL/BAUD/OPTIONS still read back as expected
#    (already-established read-only pattern, sr75_hil_f24c_param_backup.py):
python3 Tools/autotest/sr75_hil_layer2/scripts/sr75_hil_f24c_param_backup.py \
  --pixhawk /dev/ttyACM0 --baud 115200 --label f24h-diagnosis

# 4. (After a bench run, if an SD card is fitted) pull the dataflash log
#    and inspect the UART message for the SERIAL1 instance's TX/RX byte
#    counters -- expect both to stay flat/zero if PPP is truly never
#    driving the port:
#    mavlogdump.py --types UART <log>.bin
```

Expected results if the root-cause finding is correct: (1) no
`PARAM_VALUE` reply for `NET_ENABLE` (timeout); (2) no `"PPP[0]:
started"` STATUSTEXT ever appears; (3) `SERIAL1_PROTOCOL=48`/
`SERIAL1_BAUD=921`/`SERIAL1_OPTIONS=0` read back unchanged (confirming
the parameter *values* were never the problem); (4) the UART message's
TX byte counter for the SERIAL1 instance stays at 0 (or flat) throughout
a bench run.

## Defect found

**Compile-time configuration gap**: `libraries/AP_HAL_ChibiOS/hwdef/
fmuv3-SimOnHardWare/hwdef.dat` (and its base, `fmuv3/hwdef.dat`, and the
shared `SimOnHW.inc` overlay) never define `AP_NETWORKING_BACKEND_PPP`,
so ArduPilot's PPP networking backend is entirely absent from the
compiled firmware, making `SERIAL1_PROTOCOL=48` a value with no
corresponding implemented behavior.

## Minimal proposed patch (NOT applied — diagnostic task, requires a rebuild + reflash to take effect)

```diff
--- a/libraries/AP_HAL_ChibiOS/hwdef/fmuv3-SimOnHardWare/hwdef.dat
+++ b/libraries/AP_HAL_ChibiOS/hwdef/fmuv3-SimOnHardWare/hwdef.dat
@@
 include ../fmuv3/hwdef.dat
 include ../include/SimOnHW.inc
 
+# HIL-F24-H: enable ArduPilot's PPP networking backend so
+# SERIAL1_PROTOCOL=48 (TELEM1) actually has PPP code behind it. Without
+# this, AP_NETWORKING_ENABLED/AP_NETWORKING_BACKEND_PPP default to 0 for
+# any non-Ethernet ChibiOS board (see AP_Networking_Config.h), so the
+# entire AP_Networking/AP_Networking_PPP backend -- and the NET_*
+# parameter table itself -- is compiled out, and SERIAL1_PROTOCOL=48
+# silently falls through to a plain uart->begin() with no PPP handling
+# at all. Matches the existing, real ArduPilot precedent used by
+# CubeOrangePlus/CubeRedPrimary/CubeRedSecondary (vehicle firmware, same
+# one-line pattern, no other changes needed for a non-Ethernet board).
+define AP_NETWORKING_BACKEND_PPP 1
+
 # Unique board/firmware identity (HIL-F24-B item 2). CHIBIOS_SHORT_BOARD_NAME
```

This is a single-line, additive, non-flight-logic hwdef change, directly
mirrored from an existing ArduPilot-mainline precedent (`CubeOrangePlus/
hwdef.dat:103`) used on a target of the same class (ordinary vehicle
firmware, non-Ethernet ChibiOS board). It touches only the SimOnHardWare
diagnostic/simulation target, never `fmuv3/hwdef.dat` (the real flight
firmware), and does not modify RATO/aero/TECS/mission logic, SERIAL1's
mapping, or any parameter default. **Not applied in this task** — it
requires a full rebuild (`./waf configure --board fmuv3-SimOnHardWare &&
./waf plane`) and a subsequent, separately-authorized reflash + bench
verification cycle (repeat HIL-F24-C's precheck → flash → precheck
procedure) before it can be confirmed effective; per instruction, no
flashing was performed here.

## Files and line references (summary)

| File | Relevance |
|---|---|
| `libraries/AP_Networking/AP_Networking_Config.h:5-18,20-22,34-36` | `AP_NETWORKING_ENABLED`/`AP_NETWORKING_BACKEND_PPP` default-0 fallback logic |
| `libraries/AP_Networking/AP_Networking.h:6`, `.cpp:4,35,124` | `AP_Networking` class/param-table/`init()`, gated on `AP_NETWORKING_ENABLED` |
| `libraries/AP_Networking/AP_Networking_PPP.cpp:184-241,324-369,374-437,513-563` | PPP init/thread/connect/poll call chain |
| `libraries/AP_SerialManager/AP_SerialManager.h:87` | `SerialProtocol_PPP = 48` definition |
| `libraries/AP_SerialManager/AP_SerialManager.cpp:579-582,586-587` | PPP case (gated, empty) vs. `default: uart->begin()` fallback actually taken |
| `libraries/AP_SerialManager/AP_SerialManager.cpp:757` | `SERIAL1_BAUD=921` → 921600 conversion (confirmed correct) |
| `libraries/AP_Vehicle/AP_Vehicle.h:450`, `.cpp:216-251,387-388,662-663` | `networking` member/params/`init()`/scheduler task, all gated |
| `libraries/AP_HAL_ChibiOS/hwdef/fmuv3/hwdef.dat:132,302-313` | `SERIAL_ORDER`, USART2/TELEM1 & USART3/TELEM2 pin definitions |
| `libraries/AP_HAL_ChibiOS/hwdef/fmuv3-SimOnHardWare/hwdef.dat:26-27` | Confirms no pin/networking override in the custom target |
| `libraries/AP_HAL_ChibiOS/hwdef/include/SimOnHW.inc` | Confirms no networking content in the shared SimOnHW overlay |
| `libraries/AP_HAL_ChibiOS/hwdef/scripts/chibios_hwdef.py:868-874,939-952,1013-1014` | Build-script `enable_networking()` auto-trigger conditions (ETH1 or explicit define only) |
| `libraries/AP_HAL_ChibiOS/hwdef/CubeOrangePlus/hwdef.dat:103` | Working precedent: `define AP_NETWORKING_BACKEND_PPP 1` |
| `libraries/AP_HAL_ChibiOS/hwdef/CubeRedPrimary/hwdef.dat:300`, `CubeRedSecondary/hwdef.dat:156`, `Pixhawk6X-PPPGW/hwdef.dat:25` | Additional real-world precedents for the same define |
| `libraries/AP_HAL_ChibiOS/hwdef/fmuv3-SimOnHardWare/defaults.parm:56` | `NET_ENABLE 1` default for a parameter that doesn't exist in this build (corroborating evidence) |
| `Tools/autotest/sr75_hil_layer2/ppp/SR75_SoH_JSON_PPP_TELEM1.param` | Origin of the `NET_ENABLE`/`SERIAL1_PROTOCOL` parameter set that never accounted for the compile-time gate |

## Not done (per instruction)

No flashing, no `PARAM_SET`, no arm/mode change, no actuator enable, no
RATO/aero/TECS/mission change, no change to SERIAL1's mapping (proven
correct, so instruction 7's "prove the mapping defect first" condition
for touching it was never met — it was not touched). The proposed patch
above was written into this report only, not applied to the working tree.

## PASS/FAIL for firmware-side PPP readiness

**FAIL.** The compiled `fmuv3-SimOnHardWare` firmware currently has no
PPP networking backend at all; `SERIAL1_PROTOCOL=48` cannot produce any
PPP/LCP traffic on TELEM1 regardless of wiring, adapter, or parameter
correctness (all of which are independently confirmed fine). A one-line,
precedented hwdef fix has been identified and is ready for a
human-authorized rebuild-and-reflash cycle, but has not been applied or
built in this task.
