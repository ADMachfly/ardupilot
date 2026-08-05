# HIL-F24-M: JSON+PPP USB Connect/Disconnect Reboot Loop Diagnosis (Diagnostic Only)

Status: root cause found, confirmed from source, from the already-built
binary, and from two additional **build-only** comparison variants built
specifically for this task (never flashed). No firmware was flashed, no
`uploader.py` invoked, no parameter changed, no PPP hardware started, no
actuator enabled. Two temporary hwdef board directories were created
under `libraries/AP_HAL_ChibiOS/hwdef/` purely to obtain comparable
build-only artifacts for this diagnosis; both they and their `build/`
output were deleted before finishing this task (confirmed via `git
status`).

## PASS/FAIL for JSON+PPP firmware boot stability

**FAIL.** The current `fmuv3-SimOnHardWare` firmware (JSON+PPP,
`AP_SIM_FRAME_CLASS=JSON`) will reliably hang the main vehicle thread on
its very first scheduler tick and be force-reset roughly 1.8-2.1 seconds
later, every boot, for as long as the external JSON feed (over PPP) is
not already answering within that window — which it structurally cannot
be, since PPP itself takes several seconds to negotiate LCP/IPCP before
any IP packet, let alone a UDP reply, can route at all. This exactly
reproduces "repeatedly connects and disconnects" / "board itself
resets/re-enumerates."

## Ranked root-cause candidates, with evidence

### 1. (Primary, highest confidence) `SITL::JSON::update()`'s blocking receive loop runs on the main vehicle thread and starves the watchdog

**Exact first failing call / smallest confirmed failure boundary:**
`libraries/SITL/SIM_JSON.cpp:350-360`, the `while (ret <= 0)` loop inside
`JSON::recv_fdm()`, reached via the unconditional, main-thread call chain
below. This loop has **no bound on total elapsed time or attempt count**
— it only exits when `sock.recv()` returns `> 0` bytes, i.e. when a real
UDP reply from the host responder arrives.

```
libraries/SITL/SIM_JSON.cpp:332-360
void JSON::recv_fdm(const struct sitl_input &input)
{
    // Receive sensor packet
    ssize_t ret = sock.recv(&sensor_buffer[sensor_buffer_len], sizeof(sensor_buffer)-sensor_buffer_len, UDP_TIMEOUT_MS);
    uint32_t wait_ms = UDP_TIMEOUT_MS;

    if (state.no_lockstep && ret <= 0) {
        ...
        return;
    }

    while (ret <= 0) {
        ret = sock.recv(&sensor_buffer[sensor_buffer_len], sizeof(sensor_buffer)-sensor_buffer_len, UDP_TIMEOUT_MS);
        wait_ms += UDP_TIMEOUT_MS;
        // if no sensor message is received after 10 second resend servos ...
        if (wait_ms > 1000) {
            wait_ms = 0;
            printf("No JSON sensor message received, resending servos\n");
            output_servos(input);
        }
    }
```

(Note: `state.no_lockstep` defaults false and is only ever set from a
field inside a JSON reply packet already received — on the very first
call, before any reply has ever arrived, this early-exit cannot help.)

**Exact call chain proving this runs on the single main vehicle thread,
with no thread boundary anywhere in between:**

```
libraries/AP_HAL_ChibiOS/HAL_ChibiOS_Class.cpp:331-348  (HAL_ChibiOS::run(), the main thread)
    while (true) {
        g_callbacks->loop();                    // <- never returns while stuck below
        ...
        schedulerInstance.watchdog_pat();       // <- only reached AFTER loop() returns
    }
                |
                v
AP_Scheduler.cpp:428-430  (called every AP_Scheduler::loop() iteration)
    #if AP_SIM_ENABLED && CONFIG_HAL_BOARD != HAL_BOARD_SITL
        hal.simstate->update();
    #endif
                |
                v
libraries/AP_HAL/SIMState.cpp:80-90 (SIMState::update())
    if (!init_done) {
        AP::sitl()->init();
        init_done = true;
        sitl_model = SITL::AP_SIM_FRAME_CLASS::create(AP_SIM_FRAME_STRING);  // SITL::JSON::create(...)
    }
    _fdm_input_step();
                |
                v
SIMState.cpp:106-126 (_fdm_input_step -> fdm_input_local)
    _simulator_servos(input);
    sitl_model->update_home();
    sitl_model->update_model(input);   // virtual dispatch -> JSON::update()
                |
                v
libraries/SITL/SIM_JSON.cpp:631-637 (JSON::update())
    output_servos(input);   // one UDP sendto -- does not block meaningfully
    recv_fdm(input);        // <-- THE BLOCKING CALL, see above
```

Every one of these calls is a direct, synchronous, same-thread function
call — **no thread is ever spawned for the SIM_JSON model's update
loop**, unlike `AP_Networking_PPP::init()` (`libraries/AP_Networking/
AP_Networking_PPP.cpp:234-237`), which deliberately spawns its own
`"ppp"` thread specifically so PPP's own blocking I/O never touches the
main vehicle thread. `AP_HAL::SIMState`'s JSON integration does not
follow that same safe pattern.

**Why this reliably triggers a forced reset within ~1.8-2.1s, every
boot:**

```
libraries/AP_HAL_ChibiOS/Scheduler.cpp:414-484  (_monitor_thread, a *separate* ChibiOS thread,
                                                   enabled by default: see below)
    while (true) {
        sched->delay(100);
        ...
        uint32_t loop_delay = now - sched->last_watchdog_pat_ms;
        if (loop_delay >= 500 && !sched->in_expected_delay()) {
            AP::internalerror().error(AP_InternalError::error_t::main_loop_stuck, ...);
        }
#if AP_CRASHDUMP_ENABLED
        if (loop_delay >= 1800 && using_watchdog) {
            // we are about to watchdog, better to trigger a hardfault
            // now and get a crash dump file
            void *ptr = (void*)0xE000FFFF;
            typedef void (*fptr)();
            fptr gptr = (fptr) (void *)ptr;
            gptr();                     // <-- deliberate self-inflicted hardfault
        }
#endif
```

- `HAL_MONITOR_THREAD_ENABLED` defaults to **1** (`Scheduler.cpp:67-69`,
  `#ifndef .../#define ... 1`) for any normal vehicle build (only
  `AP_Periph`/IOFirmware minimal targets turn it off) — confirmed
  present for this build.
- `using_watchdog` (`AP_BoardConfig::watchdog_enabled()`) is **true** by
  default for any ChibiOS, non-Replay, non-Unknown vehicle build:
  `libraries/AP_BoardConfig/AP_BoardConfig.cpp:78-88` —
  `HAL_BRD_OPTIONS_DEFAULT = BOARD_OPTION_WATCHDOG` (with or without
  `HAL_DEBUG_BUILD`). Neither `fmuv3/hwdef.dat` nor
  `fmuv3-SimOnHardWare/hwdef.dat` nor `SimOnHW.inc` overrides this.
- `AP_CRASHDUMP_ENABLED` is confirmed **1** for this exact build:
  `build/fmuv3-SimOnHardWare/hwdef.h:98,107-108` —
  `#define BOARD_FLASH_SIZE 2048` / `#define AP_CRASHDUMP_ENABLED 1`
  (chibios_hwdef.py's own default is `flash_size >= 2048`, and this
  board's flash is exactly 2048 KB).
- So at `loop_delay >= 1800 ms` the monitor thread deliberately jumps to
  an invalid address to force a hardfault (to capture a CrashCatcher
  dump before the "real" watchdog fires anyway). Even if this path were
  somehow bypassed, the raw STM32 IWDG hardware watchdog independently
  resets the MCU at `STM32_WDG_TIMEOUT_MS = 2048` ms
  (`libraries/AP_HAL_ChibiOS/hwdef/common/watchdog.c:28-29`) regardless,
  since `stm32_watchdog_pat()` (inside `watchdog_pat()`) is also never
  reached again.
- A full MCU reset re-initializes the USB peripheral from scratch,
  producing exactly the observed USB disconnect/re-enumerate cycle on
  every reset.

**A subtle, related, self-defeating detail**: `SIM_JSON.cpp:48,103-104`'s
own `sim_defaults[]` table sets `AP_Param::set_default_by_name("BRD_OPTIONS", 0)`
in the `JSON` constructor — an apparent acknowledgment by the SIM_JSON
author that the watchdog and a blocking JSON link can conflict. This
mitigation does not help here for two reasons: (a) it only changes the
*default* shown for an unconfigured parameter, it does not override an
already-saved value; and more importantly (b) `stm32_watchdog_init()`
(`HAL_ChibiOS_Class.cpp:298-313`) runs during early HAL bring-up, **long
before** `SIMState::update()`'s first call (which happens on the first
scheduler tick, well after HAL init) — so by the time this default-value
change executes, the watchdog has already been configured using whatever
value existed before it.

### 2. (Secondary, real, independently severe) The `JSON` model object is ~65 KB+ against a 78.7 KB total heap, with no null-check after `create()`

```
libraries/SITL/SIM_JSON.h:81-82
    // buffer for parsing pose data in JSON format
    uint8_t sensor_buffer[65000];
    uint32_t sensor_buffer_len;
```

`SITL::JSON` is allocated with `NEW_NOTHROW JSON(frame_str)`
(`SIM_JSON.h:36-38`) — this single object (base `Aircraft` state +
`sensor_buffer[65000]` + the 36-entry `keytable[]` + `SocketAPM sock`)
is, by a wide margin, the single largest heap allocation anywhere in this
firmware's boot sequence. Confirmed available heap for this exact build
(`arm-none-eabi-size -A build/fmuv3-SimOnHardWare/bin/arduplane`):

```
.heap   78748   536988772
```

Only **78,748 bytes of heap total**. If this allocation fails (return
`nullptr`, per `NEW_NOTHROW`'s contract) — plausible given everything
else that has already allocated heap by this point in boot (PPP/lwIP's
own pools now that `AP_NETWORKING_BACKEND_PPP=1`, GCS_MAVLink buffers,
the `_SITL` sensor backend instances, etc.) — the very next lines run
with **no null check at all**:

```
libraries/AP_HAL/SIMState.cpp:124-126
    // update the model
    sitl_model->update_home();
    sitl_model->update_model(input);
```

A null `sitl_model` here is an immediate null-pointer virtual dispatch —
a synchronous hardfault, not a timed watchdog event, and it would happen
on literally the first call, sub-millisecond after boot rather than
~1.8-2.1 s later. This is ranked **below** candidate 1 only because
candidate 1 is *guaranteed* regardless of whether this allocation
succeeds or fails (PPP negotiation alone takes several seconds, so
`recv_fdm()` blocks either way) — but if this allocation is in fact
failing on the real board, it would be the very first symptom, and it
independently needs the same class of fix (bound/avoid the blocking
path, and/or shrink this buffer, and/or add a null check).

**Disambiguating the two on the real bench** (task item 6, read-only):
capture the `WDG: ...` STATUSTEXT on the brief reconnect window after a
reset (see below). `TN:` (thread name) will read the main thread's name
either way, but the **timing** distinguishes them: a reset within a
fraction of a second of USB enumeration points to candidate 2
(null-deref); a reset consistently ~1.8-2.1 s after enumeration points to
candidate 1 (watchdog/monitor-thread timeout). `FA` (fault address) near
0x0-0x20 would also directly indicate a null/near-null dereference.

### 3. Ruled out / checked, not implicated

- **Repeated model creation**: guarded by `static bool init_done`
  (`SIMState.cpp:82-87`) — `create()` runs exactly once per boot, not
  repeatedly.
- **Invalid frame string handling**: `"json:192.168.144.2"` is parsed
  correctly (`SIM_JSON.cpp:98-101`, `strchr` finds `:`, `target_ip` is
  set to `"192.168.144.2"`); no malformed-string edge case applies here.
- **Interaction between PPP reconnect and the SIM_JSON send loop**: none
  directly — they run on independent threads (PPP has its own `"ppp"`
  thread). The *consequence* (SIM_JSON can never succeed until PPP's
  independent negotiation completes) is exactly what guarantees
  candidate 1's multi-second stall, but there is no direct
  deadlock/interaction between the two mechanisms themselves.
- **USB scheduler starvation from a high JSON update *rate***: not
  applicable — the problem is not a high rate of small stalls, it is one
  single, unbounded stall on the very first call.
- **ChibiOS internal `chDbgAssert`/panic paths**: no evidence found of a
  ChibiOS-level assertion firing; the reset mechanism identified above is
  ArduPilot's own *deliberate* self-inflicted hardfault (or, failing
  that, the plain hardware IWDG) — not a ChibiOS kernel panic.
- **Stack size of the calling thread**: main thread stack
  (`HAL_PROCESS_STACK_SIZE = 0x1C00` = 7168 bytes, confirmed in
  `build/fmuv3-SimOnHardWare/hwdef.h:48`, matching the linked `.pstack`
  section) is modest but JSON's own local-variable footprint
  (`servo_packet_32`/`servo_packet_16` structs, a handful of locals in
  `parse_sensors`/`recv_fdm`) is small relative to it; not flagged as a
  likely contributor.
- **Flash overflow**: explicitly not the cause — see item 9 below.

## Reset/crash evidence already supported by this target (task item 5)

ArduPilot/ChibiOS on this board already has a complete, no-code-needed
evidence trail for exactly this failure class:

| Evidence | Mechanism | Where |
|---|---|---|
| Reset was a watchdog reset | `hal.util->was_watchdog_reset()` reads a reset-cause flag saved from `RCC` reset-status bits before reset | `libraries/AP_HAL_ChibiOS/hwdef/common/watchdog.c:122-126` (`stm32_was_watchdog_reset`) |
| What the main thread was doing at the moment of the freeze | `hal.util->last_persistent_data` (preserved across the reset in backup-domain RAM): `fault_addr`, `fault_icsr`, `fault_lr`, `fault_line`, `fault_type`, `fault_thd_prio`, `thread_name4`, `scheduler_task`, `semaphore_line`, `internal_errors` | `libraries/AP_HAL/Util.h:59-86`; saved via `stm32_watchdog_save()` |
| Automatic STATUSTEXT on the very next boot | `AP_Vehicle::send_watchdog_reset_statustext()` — sends `"WDG: T%d SL%u FL%u FT%u FA%x FTP%u FLR%x FICSR%u MM%u MC%u IE%u IEC%u TN:%.4s"` at `MAV_SEVERITY_CRITICAL` if `was_watchdog_reset()` | `libraries/AP_Vehicle/AP_Vehicle.cpp:761-785` |
| Full register/memory crash dump in flash | CrashCatcher-based dump to the `.crash_log` linker region (`__crash_log_base__`/`__crash_log_end__`) whenever the deliberate/real hardfault fires | `libraries/AP_HAL_ChibiOS/hwdef/common/crashdump.c`; decodable offline with `Tools/debug/crash_debugger.py` |
| Main-loop-stuck internal error | `AP_InternalError::error_t::main_loop_stuck`, raised at 500 ms of no-pat, well before the 1.8 s hardfault-preemption point | `AP_HAL_ChibiOS/Scheduler.cpp:463-465` |

No new instrumentation is required for basic evidence-gathering — this
existing mechanism already answers "was it a watchdog reset, and what
was the main thread doing" without any firmware change (task 7's premise
— "only if needed" — is not met for this purpose).

## Read-only commands to capture this evidence (task item 6)

```sh
# 1. Arm a STATUSTEXT capture BEFORE plugging in USB, to catch the brief
#    reconnect window (the WDG: ... message is sent once per boot, early):
python3 -c "
from pymavlink import mavutil
import time
m = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
t0 = time.time()
hb = m.wait_heartbeat(timeout=15)
print(f'heartbeat at t={time.time()-t0:.3f}s' if hb else 'no heartbeat')
deadline = time.time() + 10
while time.time() < deadline:
    msg = m.recv_match(type='STATUSTEXT', blocking=True, timeout=0.3)
    if msg:
        print(f't={time.time()-t0:.3f}s STATUSTEXT[{msg.severity}]: {msg.text}')
"
# expect (if this diagnosis is correct): a heartbeat, then within ~1.8-2.1s
# a 'WDG: ...' STATUSTEXT, then the connection drops as the board resets.

# 2. AUTOPILOT_VERSION when briefly available (read-only, matches the
#    existing sr75_hil_f24c_param_backup.py pattern):
python3 -c "
from pymavlink import mavutil
m = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
m.wait_heartbeat(timeout=15)
m.mav.command_long_send(m.target_system, m.target_component,
    mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0,
    mavutil.mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION, 0,0,0,0,0,0)
msg = m.recv_match(type='AUTOPILOT_VERSION', blocking=True, timeout=3)
print(msg)
"

# 3. USB enumeration timestamps (host-side, read-only, no firmware needed):
#    Linux/WSL:
sudo dmesg -w | grep -i "usb\|cdc_acm\|ttyACM"
#    Windows host (from an elevated PowerShell), watch Device Manager /
#    event log churn:
#    Get-WinEvent -FilterHashtable @{LogName='System'; ProviderName='Microsoft-Windows-Kernel-PnP'} -MaxEvents 20

# 4. Boot count / free memory / scheduler load, once briefly connected
#    (all standard, read-only MAVLink requests):
python3 -c "
from pymavlink import mavutil
m = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
m.wait_heartbeat(timeout=15)
msg = m.recv_match(type=['MEMINFO','SYS_STATUS'], blocking=True, timeout=5)
print(msg)
"

# 5. Full crash dump decode, if a .crash_log capture is later pulled from
#    the board's flash (requires the bootloader-mode MSD/DFU path, not
#    covered further here since it is out of this task's no-flash scope):
python3 Tools/debug/crash_debugger.py --help
```

## Good-versus-bad build comparison table (task item 1, empirical)

Built two additional **build-only** variants for this diagnosis (never
flashed; temporary hwdef dirs and `build/` output both deleted after
gathering these numbers):

| Variant | `AP_SIM_JSON_ENABLED` | `AP_SIM_FRAME_CLASS` | text | data | bss | Free flash | `SITL::JSON` symbols | `SITL::Plane::update` symbol |
|---|---|---|---|---|---|---|---|---|
| **A: PPP-only** (reconstructed "last known good") | unset (0) | unset → default `Plane` | 1,512,800 | 4,292 | 113,796 | 563,672 | 0 | present |
| **B: JSON compiled, Plane still selected** (isolates "compile" from "select" — task item 8) | 1 | unset → default `Plane` | 1,512,812 (+12 B) | 4,292 | 113,800 | 563,656 | **0** (dead-code-eliminated) | present |
| **C: current tracked build** (JSON selected — the unstable one) | 1 | `JSON` | 1,490,688 | 4,256 | ~113,740 | 585,820\* | **8** | **absent** (dead-code-eliminated) |

\* The build report's "free flash" figure for variant C is the size of
the `.crash_log` reserved region (585,820 bytes, confirmed via
`size -A`) — this is a **reserved crash-dump partition, not literally
free/unused flash**. All three variants have effectively the same real
margin; **flash size is not the issue** (task item 9's own caveat
confirmed).

**This directly and empirically answers task item 8**: compiling
`SIM_JSON.cpp` in without selecting it (variant B) changes the binary by
only 12 bytes and introduces zero `SITL::JSON` symbols — the linker's
dead-code elimination fully removes it. **Only *selecting* `JSON` as
`AP_SIM_FRAME_CLASS` (variant C) pulls in and activates the code path
that hangs.** The overall size *decrease* from B→C (1,512,812 →
1,490,688) is explained by `SITL::Plane`'s much larger aerodynamic/force
model (including this project's SR75-specific fuel/RATO logic) being
dead-code-eliminated once nothing references it anymore, more than
offsetting JSON's own added code.

Scheduler threads/priorities and startup call paths are otherwise
identical between all three variants (same `fmuv3`/`SimOnHW.inc` base,
same `AP_NETWORKING_BACKEND_PPP=1`) — the only material difference is
exactly the one line, `AP_SIM_FRAME_CLASS`.

## Minimal proposed fix (NOT applied)

The smallest, most targeted change is to make `recv_fdm()`'s retry loop
**bounded** instead of unconditional, so a single call to `JSON::update()`
can never stall the main thread for more than one small, fixed slice —
letting the *next* scheduler tick retry naturally instead of spinning
in-place:

```diff
--- a/libraries/SITL/SIM_JSON.cpp
+++ b/libraries/SITL/SIM_JSON.cpp
@@ void JSON::recv_fdm(const struct sitl_input &input)
     while (ret <= 0) {
+        // HIL-F24-M: on real hardware (CONFIG_HAL_BOARD != HAL_BOARD_SITL)
+        // this loop runs on the main vehicle thread, which the ChibiOS
+        // watchdog/monitor-thread expects to be pat'ed at least every
+        // ~500ms-1.8s (AP_HAL_ChibiOS/Scheduler.cpp). Never spin here
+        // longer than one bounded slice on that platform; let the next
+        // scheduler tick retry instead of blocking indefinitely.
+#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
+        if (wait_ms >= 200) {
+            return;
+        }
+#endif
         ret = sock.recv(&sensor_buffer[sensor_buffer_len], sizeof(sensor_buffer)-sensor_buffer_len, UDP_TIMEOUT_MS);
         wait_ms += UDP_TIMEOUT_MS;
         if (wait_ms > 1000) {
```

Complementary, also-not-applied changes worth considering (task item
10):
- **Heap headroom**: `sensor_buffer[65000]` (`SIM_JSON.h:81`) is far
  larger than any observed JSON sensor packet in this project's own
  captures (a few KB at most, per HIL-F24-G's regression captures);
  shrinking it (e.g. to 8192 bytes) would remove essentially all of the
  heap-exhaustion risk identified in candidate 2, independent of
  candidate 1's fix.
- **Add a null check** after `sitl_model = SITL::AP_SIM_FRAME_CLASS::
  create(...)` (`SIMState.cpp:86`) before `fdm_input_local()` dereferences
  it — cheap, safe, and turns a silent hardfault into a clean, logged
  failure if allocation ever does fail.
- **Deferred JSON startup until PPP/IP is ready** is the more complete
  (but not minimal) architectural fix: gate the first `create()`/
  `update()` call on `AP::network()`-style link-up confirmation. Not
  proposed as a diff here since it is materially larger than the
  one-line bound above and the bounded-loop fix alone is sufficient to
  stop the reboot loop regardless of how long PPP takes to come up.

None of the above were applied to any tracked file.

## Staged build-only validation plan (no flashing)

```sh
# Stage 1 (already done for this report): compile JSON, keep Plane
# selected -- confirms "just compiling" is inert (0 SITL::JSON symbols).

# Stage 2: apply ONLY the bounded-recv-loop diff above to a scratch copy
# of SIM_JSON.cpp, in a temporary hwdef variant identical to the current
# fmuv3-SimOnHardWare (JSON selected), build-only:
./waf configure --board fmuv3-SimOnHardWare
./waf plane
arm-none-eabi-nm -C build/fmuv3-SimOnHardWare/bin/arduplane | grep -c "SITL::JSON"
# expect: 8 (unchanged -- the fix doesn't remove JSON, just bounds it)

# Stage 3: confirm the size delta from the bounded loop is negligible
# (a handful of instructions, well under 100 bytes):
arm-none-eabi-size build/fmuv3-SimOnHardWare/bin/arduplane

# Only after Stage 2/3 look clean would an actual bench flash + the
# read-only STATUSTEXT/dmesg capture commands above be the appropriate
# next step -- not performed in this task.
```

## Exact manual flashing and rollback instructions (for the user; NOT executed)

**Rollback to the last known-good PPP-only firmware** (immediate, if
needed right now):
```sh
# If a pre-JSON .bin/.apj was kept from before this change, flash that
# directly with your normal bootloader-mode uploader, e.g.:
python3 Tools/scripts/uploader.py --port /dev/ttyACM0 <path-to-old>/arduplane.apj
```
(Not run here — this is the user's own rollback action, provided for
reference only.)

**Once the minimal fix above is applied and Stage 2/3 above are clean**,
re-flash the corrected JSON+PPP build the same way:
```sh
./waf configure --board fmuv3-SimOnHardWare && ./waf plane
python3 Tools/scripts/uploader.py --port /dev/ttyACM0 build/fmuv3-SimOnHardWare/bin/arduplane.apj
```
Then repeat HIL-F24-C's precheck → confirm GO → bench verification
sequence before relying on it further.

## Files and line references (summary)

| File | Relevance |
|---|---|
| `libraries/SITL/SIM_JSON.cpp:332-360,631-637` | The blocking `recv_fdm()` loop; `JSON::update()` |
| `libraries/SITL/SIM_JSON.h:36-38,59-70,81-82` | `create()`; hardware-branch `SocketAPM sock`; oversized `sensor_buffer[65000]` |
| `libraries/AP_HAL/SIMState.cpp:80-90,106-126` | `SIMState::update()`/`_fdm_input_step()`/`fdm_input_local()` — no null check on `sitl_model` |
| `libraries/AP_Scheduler/AP_Scheduler.cpp:428-430` | `hal.simstate->update()` called every main-loop tick |
| `libraries/AP_HAL_ChibiOS/HAL_ChibiOS_Class.cpp:298-320,331-348` | Watchdog init (early boot); main loop's `watchdog_pat()` only after `loop()` returns |
| `libraries/AP_HAL_ChibiOS/Scheduler.cpp:67-69,414-484` | `HAL_MONITOR_THREAD_ENABLED` default; the monitor thread's 500ms/1800ms thresholds and deliberate self-hardfault |
| `libraries/AP_BoardConfig/AP_BoardConfig.cpp:78-88` | `HAL_BRD_OPTIONS_DEFAULT` includes `BOARD_OPTION_WATCHDOG` by default |
| `libraries/AP_HAL_ChibiOS/hwdef/common/watchdog.c:28-29,122-135` | `STM32_WDG_TIMEOUT_MS=2048`; `stm32_was_watchdog_reset()` |
| `libraries/AP_HAL_ChibiOS/hwdef/common/crashdump.c` | CrashCatcher flash crash-dump mechanism |
| `libraries/AP_HAL/Util.h:59-86` | `PersistentData`/`last_persistent_data` fields |
| `libraries/AP_Vehicle/AP_Vehicle.cpp:761-785` | `send_watchdog_reset_statustext()` — the `"WDG: ..."` message |
| `build/fmuv3-SimOnHardWare/hwdef.h:48,98,107-108` | `HAL_PROCESS_STACK_SIZE`, `BOARD_FLASH_SIZE=2048`, `AP_CRASHDUMP_ENABLED=1` for this exact build |
| `libraries/AP_Networking/AP_Networking_PPP.cpp:234-237` | Contrast: PPP's own dedicated thread, the safe pattern SIM_JSON does not follow |
| `Tools/debug/crash_debugger.py` | Existing tool to decode a pulled `.crash_log` dump |

## Not done (per instruction)

No flash, no `uploader.py`, no Mission Planner, no `PARAM_SET`, no
arm/mode change, no hardware PPP started, no actuator enabled, no
RATO/aero/TECS/control/mission logic modified. Two temporary hwdef board
directories were created and built (build-only, waf configure/plane) to
obtain the empirical comparison table above; both the source directories
and their `build/` output were deleted before completing this task —
confirmed via `git status --short` showing no trace of them.
