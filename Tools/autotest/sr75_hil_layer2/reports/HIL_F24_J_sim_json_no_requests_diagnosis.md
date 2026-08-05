# HIL-F24-J: SIM_JSON Client No-Requests Diagnosis (Diagnostic Only)

Status: **root cause found, confirmed from source AND from the actual
built binary**. No hardware touched, no `PARAM_SET`, nothing flashed.
This is a materially **larger** gap than HIL-F24-H's PPP finding: it is
not a missing one-line define, it is a **missing subsystem** — real new
firmware code, not just a config change, is required to close it.

## Ranked root cause

### 1. (Primary, architectural, confirmed from source) No ChibiOS-side code ever creates a `SITL::Aircraft`/`SITL::JSON` model instance or drives its update loop

On desktop SITL, the entire "which simulated aircraft model to run, and
call its `update()` every tick" mechanism lives in `AP_HAL_SITL`
(`SITL_cmdline.cpp`, `SITL_State.cpp`) — a HAL implementation that is
**not part of a ChibiOS board build at all**. Specifically:

- `libraries/AP_HAL_SITL/SITL_cmdline.cpp:594` —
  `sitl_model = model_constructors[i].constructor(model_str);` — this is
  where a `--model json:<ip>`-style command-line string is parsed and
  dispatched to `JSON::create()`. This code path only exists in the
  desktop SITL binary; a ChibiOS firmware image has no command line and
  no equivalent dispatch table.
- `libraries/AP_HAL_SITL/SITL_State.cpp:226-227` —
  `sitl_model->update_home(); sitl_model->update_model(input);` — the
  per-tick driver of the model, which internally calls the virtual
  `update()` (→ `JSON::update()`, `SIM_JSON.cpp:631`, which calls
  `output_servos()` — the actual UDP send — then `recv_fdm()`).
- `grep`-confirmed: `Aircraft::create`/`SITL_State` symbols exist **only**
  in `libraries/AP_HAL_SITL/*.cpp` and `libraries/AP_HAL_SITL/
  SITL_Periph_State.cpp` — zero occurrences anywhere in
  `libraries/AP_HAL_ChibiOS/`.
- `libraries/AP_HAL_ChibiOS/HAL_ChibiOS_Class.cpp:366-368` — the only
  SITL-related hook that runs on this build is `AP::sitl()->init();`
  (gated on `AP_SIM_ENABLED`), and `SITL::SIM::init()`
  (`libraries/SITL/SITL.h:138-165`) **only sets up `AP_Param` defaults**
  — it does not select a model, open a socket, or start any update loop.

Consequence: even with every other gap below fixed, nothing in the
current firmware would ever call `JSON::create()` or `JSON::update()`.
This is why `AP_Baro_SITL`/`AP_Compass_SITL`/`AP_InertialSensor_SITL`
(confirmed selected/working per HIL-F24-B/C) read from
`SITL::SIM`'s shared `state` struct (e.g.
`AP_InertialSensor_SITL.cpp:80-82` reads `sitl->state.xAccel` etc.) but
that struct is **never populated** on this hardware build — those
backends were confirmed to *exist and be selected*, not confirmed to be
*receiving live data*, and this task shows they are not.

### 2. (Contributing, compile-time, confirmed from source AND binary) `AP_SIM_JSON_ENABLED` defaults to SITL-only and is never overridden

- `libraries/SITL/SIM_config.h:345-347`:
  ```c
  #ifndef AP_SIM_JSON_ENABLED
  #define AP_SIM_JSON_ENABLED (CONFIG_HAL_BOARD == HAL_BOARD_SITL)
  #endif
  ```
  For any ChibiOS hardware build, `CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS`,
  so this is **0** unless overridden.
- Exhaustive search (`grep -rl "AP_SIM_JSON_ENABLED"` across every
  `.dat`/`.inc`/`.h` in the tree) finds it **only** in `SIM_JSON.h` and
  `SIM_config.h` themselves — **no hwdef anywhere in ArduPilot**
  (including `fmuv3-SimOnHardWare`, `CubeOrange(Plus)-SimOnHardWare`, or
  any non-SimOnHardWare target) ever defines it. This is not specific to
  this bench's target; it is a gap in every existing `*-SimOnHardWare`
  target in the codebase, and there is no working precedent to copy
  (unlike HIL-F24-H's PPP finding, where `CubeOrangePlus` provided a
  real, working precedent).
- `libraries/SITL/SIM_JSON.h:19` / `SIM_JSON.cpp:21` — the entire `JSON`
  class and its implementation are `#if AP_SIM_JSON_ENABLED`-gated, so
  with the define at 0 **none of it is compiled into the firmware at
  all**.

### 3. Binary evidence (task item 7) — directly confirms candidates 1 and 2

Inspected the already-built `build/fmuv3-SimOnHardWare/bin/arduplane`
(built earlier in this HIL-F24 session; not rebuilt or reflashed for this
diagnosis):

```
$ strings build/fmuv3-SimOnHardWare/bin/arduplane | grep -iE \
    "json control interface|Starting SITL: JSON|JSON received|JSON sensor message"
(no output -- none of SIM_JSON.cpp's printf strings are present)

$ arm-none-eabi-nm build/fmuv3-SimOnHardWare/bin/arduplane | grep -i "4JSON\|3JSON"
(no output -- no SITL::JSON class symbols at all)

$ arm-none-eabi-nm build/fmuv3-SimOnHardWare/bin/arduplane | grep "4SITL8Aircraft"
080ceb70 W _ZN4SITL8Aircraft10set_configEPKc
080d2cd8 T _ZN4SITL8Aircraft11rand_normalEdd
080d3578 T _ZN4SITL8Aircraft12update_modelERK10sitl_input
... (base Aircraft class methods present, linked as a general SITL
     dependency, but no JSON-derived subclass exists to ever be
     instantiated)

$ arm-none-eabi-nm build/fmuv3-SimOnHardWare/bin/arduplane | grep "4SITL3SIM"
20015768 B _ZN4SITL3SIM10_singletonE
080ff36c W _ZN4SITL3SIM4initEv
... (SITL::SIM singleton and its init() exist -- matches candidate 1's
     finding that only parameter-default setup happens, nothing more)
```

The only "JSON" string matches anywhere in the binary are
`SITL::Plane::parse_float`/`parse_vector3` — unrelated `AP_JSON::value`
config-parsing helpers in the airframe model code, not `SITL::JSON`.

**This is direct, binary-level confirmation: the SIM_JSON client does not
exist in the currently-flashed firmware, full stop** — independent of any
runtime parameter, PPP state, or feeder behavior.

### 4. Ruled out

- **PPP/transport**: HIL-F24-H's finding (PPP backend not compiled in)
  and this task's finding are independent defects stacked on top of each
  other. Even if HIL-F24-H's fix were applied and PPP came up cleanly,
  it would only provide a working IP link — it would still carry zero
  SIM_JSON traffic, because nothing on the Pixhawk originates any.
- **Feeder/responder behavior**: not re-audited per instruction; nothing
  found here implicates them — the feeder and responder are host-side
  and correctly waiting for a client that does not exist on the FC.
  side.
- **A runtime parameter controlling SIM_JSON target IP/port**: none
  exists. The only mechanism for setting `target_ip`/`control_port`
  (`SIM_JSON.h:59-63`) is the model-creation string (`JSON(const char
  *frame_str)`, `SIM_JSON.cpp:92-101`, parses text after `:`) or
  `set_interface_ports()` (`SIM_JSON.cpp:118-129`) — both are called
  only from `AP_HAL_SITL`'s desktop command-line/model-selection code.
  There is no `SIM_JSON_IP`/`SIM_JSON_PORT`-style `AP_Param` (task item 8
  answered: no default parameter would help; a compile-time define alone
  would not help either, per candidate 1).

## Expected destination IP/port (task item 4)

If a client existed and were wired up per this bench's intended
configuration, it would target **192.168.144.2:9002** — matching the
host responder's confirmed bind address and the existing
`SR75_SoH_JSON_PPP_TELEM1.param`/`fmuv3-SimOnHardWare/defaults.parm`
design intent. The *default* baked into `SIM_JSON.h:60-63` is
`127.0.0.1:9002` (loopback) — this would need to be overridden at model
construction/`set_interface_ports()` time to `192.168.144.2`, exactly the
same way desktop SITL's `--model JSON:192.168.144.2` command line would
override it. **No code currently does this override on this build.**

## Startup call chain (as designed for desktop SITL; entirely absent for ChibiOS)

1. `AP_HAL_SITL::SITL_cmdline.cpp:594` — parse `--model json:<ip>`,
   dispatch to `JSON::create()`.
2. `SIM_JSON.cpp:92` (`JSON::JSON()`) — parse target IP from the model
   string, set SITL-appropriate `AP_Param` defaults.
3. `AP_HAL_SITL::SITL_State.cpp:226-227` — per-tick:
   `sitl_model->update_home(); sitl_model->update_model(input);`
4. `SITL::Aircraft::update_model()` → virtual `update()` →
   `SIM_JSON.cpp:631` (`JSON::update()`).
5. `JSON::update()` → `output_servos()` (`SIM_JSON.cpp:134-166`) — **this
   is the UDP "request" packet the host responder is waiting for** —
   then `recv_fdm()` (`SIM_JSON.cpp:332-626`) — blocking receive of the
   sensor-state reply, parses it into `state`, updates `position`/
   `accel_body`/`gyro`/etc., which `Aircraft`'s base-class machinery then
   exposes via the shared `SITL::SIM` singleton for `AP_Baro_SITL`/
   `AP_Compass_SITL`/`AP_InertialSensor_SITL` to read.

Steps 1, 3, and the calling context of 4-5 (the periodic scheduler
driving it) exist **only** in `AP_HAL_SITL`. `AP_HAL_ChibiOS` has no
equivalent — this is the fundamental, structural gap.

## Compile-time and runtime gates (task item 3)

| Gate | Type | Current value (this build) | Required |
|---|---|---|---|
| `CONFIG_HAL_BOARD` | compile | `HAL_BOARD_CHIBIOS` | n/a (fixed by board) |
| `AP_SIM_ENABLED` | compile (hwdef `env SIM_ENABLED 1`) | 1 (confirmed, `HAL_ChibiOS_Class.cpp` links `xsimstate`/calls `sitl()->init()`) | 1 (already correct) |
| `AP_SIM_JSON_ENABLED` | compile (`SIM_config.h` fallback) | **0** | 1 (needs explicit hwdef override — no board in the tree does this) |
| Model-selection / `sitl_model` creation | runtime, but code only exists in `AP_HAL_SITL` | **does not exist for ChibiOS** | new code needed |
| Per-tick `update()` driver | runtime, `AP_HAL_SITL`-only | **does not exist for ChibiOS** | new code needed |
| `SIM_JSON` target IP/port param | none exists | n/a | new code/param needed |

## Minimal proposed fix — honestly **not minimal**; two parts, neither applied

Unlike HIL-F24-H's one-line PPP fix, closing this gap requires actual new
firmware code, not just a hwdef define. Proposed shape (not written or
applied in this task):

**Part A — compile-time (small, low-risk, but insufficient alone):**
```diff
--- a/libraries/AP_HAL_ChibiOS/hwdef/fmuv3-SimOnHardWare/hwdef.dat
+++ b/libraries/AP_HAL_ChibiOS/hwdef/fmuv3-SimOnHardWare/hwdef.dat
@@
 include ../fmuv3/hwdef.dat
 include ../include/SimOnHW.inc
 
+# HIL-F24-J: compile in the SIM_JSON client class itself. On its own
+# this does NOT make SIM_JSON work -- see the HIL-F24-J report for the
+# still-missing ChibiOS-side model-creation/update-loop glue.
+define AP_SIM_JSON_ENABLED 1
+
 # Unique board/firmware identity ...
```

**Part B — new runtime glue (the actual fix; real development work, not
proposed as a diff here since it does not yet exist in any form):**

A new, small ChibiOS-specific module (candidate location: a new file
alongside `HAL_ChibiOS_Class.cpp`, or a new `AP_HAL_ChibiOS`-scoped class
guarded by `#if AP_SIM_ENABLED && AP_SIM_JSON_ENABLED`) that, once per
boot:
1. Constructs a `SITL::JSON` instance directly (bypassing the
   command-line/model-string mechanism entirely, since ChibiOS has no
   command line) with the target fixed at `192.168.144.2:9002` (or, more
   flexibly, sourced from a **new** `AP_Param`, e.g. `SIM_JSON_IP`/
   `SIM_JSON_PORT`, which does not currently exist and would need to be
   added to `SITL::SIM`'s param table).
2. Starts a dedicated low-priority scheduler thread/task (mirroring the
   pattern `AP_Networking_PPP::init()` already uses for its own `"ppp"`
   thread) that calls `model->update(input)` at a fixed rate (e.g. 50 Hz,
   matching the feeder), with `input.servos[]` left at neutral/zero
   (task instruction 10 — no actuator output required for this to
   function; `output_servos()` only needs *some* value to send, it does
   not need to reflect a real actuator command for the request/reply
   wire protocol to work).
3. Read-only diagnostics (task item 9) to add alongside this new code,
   all via `GCS_SEND_TEXT`/`STATUSTEXT` (matching the existing
   `AP_Networking_PPP` pattern) — none of these exist yet since the
   surrounding code doesn't exist:
   - `"SIM_JSON: backend selected"` / target IP:port at creation.
   - `"SIM_JSON: socket created"` / socket-creation failure reason.
   - Periodic (e.g. 10 s) `"SIM_JSON: sent=%u recv=%u"` counters
     (`frame_counter` from `SIM_JSON.cpp:72` already exists as the
     sent-request counter; a received-reply counter would need adding
     next to it).

This was **not implemented** in this task — it is new functionality, not
a config change, and "not applied unless clearly safe" for a
diagnostic-only task means real source additions of this size are
out of scope here regardless of confidence.

## Comparison against a known-working SimOnHardware target (task item 6)

**None exists.** Every `*-SimOnHardWare` hwdef target in the tree
(`fmuv3-SimOnHardWare`, `CubeOrange-SimOnHardWare`,
`CubeOrangePlus-SimOnHardWare`) shares the identical `SimOnHW.inc`
overlay, and none of them (nor any of their base targets) ever defines
`AP_SIM_JSON_ENABLED`. This is a first-of-its-kind gap for this whole
family, not a target-specific misconfiguration — there is nothing to
diff against that already works.

## Next read-only bench/build verification commands (no hardware needed for most of these)

```sh
# 1. Confirm (already done in this task, reproducible) that the current
#    binary has no SIM_JSON client code -- no hardware needed:
strings build/fmuv3-SimOnHardWare/bin/arduplane | grep -i "Starting SITL: JSON"
arm-none-eabi-nm build/fmuv3-SimOnHardWare/bin/arduplane | grep -i "4JSON"
# expect: no output (confirms the finding)

# 2. On the bench, read-only, confirm no STATUSTEXT ever mentions JSON/SIM
#    during a boot + feeder/responder run (expected: none, consistent):
python3 -c "
from pymavlink import mavutil, time
m = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
m.wait_heartbeat(timeout=30)
deadline = time.time() + 15
while time.time() < deadline:
    msg = m.recv_match(type='STATUSTEXT', blocking=True, timeout=0.5)
    if msg:
        print('STATUSTEXT:', msg.text)
"

# 3. Confirm no live GCS/host-side traffic exists on the responder port
#    for the duration of a bench run (already established by the task's
#    own "Confirmed" list; repeatable read-only sanity check):
tcpdump -ni ppp0 udp port 9002 -c 5 -w /tmp/f24j_check.pcap  # expect: 0 packets captured, times out
```

No further bench action can distinguish beyond what source + binary
analysis already prove; the next real step is design/implementation of
Part B above (new code), followed by a full rebuild and a
separately-authorized flash + bench verification cycle — none of which
was performed in this task.

## Files and line references (summary)

| File | Relevance |
|---|---|
| `libraries/SITL/SIM_config.h:345-347` | `AP_SIM_JSON_ENABLED` SITL-only default |
| `libraries/SITL/SIM_JSON.h:19,59-70` | `JSON` class gate; default IP/port; hardware-aware socket type (`SocketAPM` vs `SocketAPM_native`) |
| `libraries/SITL/SIM_JSON.cpp:21,92-113,118-129,134-166,332-626,631-655` | Class gate; constructor/IP parsing; `set_interface_ports()`; `output_servos()` (the missing UDP request); `recv_fdm()`; `update()` |
| `libraries/AP_HAL_SITL/SITL_cmdline.cpp:594,617` | `--model` string → `JSON::create()` dispatch (desktop-only) |
| `libraries/AP_HAL_SITL/SITL_State.cpp:226-227` | Per-tick `update_model()` driver (desktop-only) |
| `libraries/AP_HAL_ChibiOS/HAL_ChibiOS_Class.cpp:112-114,169-171,366-368` | The *only* SITL hook ChibiOS has: `xsimstate`/`AP::sitl()->init()` — param defaults only, no model/socket |
| `libraries/SITL/SITL.h:122-165` | `SITL::SIM::init()` — confirmed to only set `AP_Param` defaults |
| `libraries/AP_InertialSensor/AP_InertialSensor_SITL.cpp:80-82` | Confirms `AP_InertialSensor_SITL` merely reads `sitl->state`, never populated on this build |
| `build/fmuv3-SimOnHardWare/bin/arduplane` | Binary evidence: zero SIM_JSON strings/symbols (commands above) |
| `libraries/AP_HAL_ChibiOS/hwdef/fmuv3-SimOnHardWare/hwdef.dat`, `CubeOrange(Plus)-SimOnHardWare/hwdef.dat`, `include/SimOnHW.inc` | Confirmed: none define `AP_SIM_JSON_ENABLED`; gap is universal across this target family |

## Not done (per instruction)

No flashing, no `PARAM_SET`, no arm/AUTO, no actuator enable, no PPP/
RATO/aero/TECS/mission change. No new code was written or applied — Part
A/B above are proposals only, described but not implemented, per this
being a diagnostic-only task and the honest scale of Part B (new
firmware functionality) exceeding what "minimal, not applied unless
clearly safe" should cover in a diagnosis task.

## PASS/FAIL for SIM_JSON client readiness

**FAIL.** The `fmuv3-SimOnHardWare` firmware currently has no SIM_JSON
client at all — confirmed independently from source analysis and from
`strings`/`nm` on the actual built binary. Fixing HIL-F24-H's PPP gap
would only provide a working transport; it would not cause any SIM_JSON
traffic to appear, because nothing in this firmware originates it. A
real fix requires new ChibiOS-side glue code (model instantiation +
periodic update-loop driver), not merely a parameter or single hwdef
define, and none of that exists in ArduPilot mainline today for any
`*-SimOnHardWare` target.
