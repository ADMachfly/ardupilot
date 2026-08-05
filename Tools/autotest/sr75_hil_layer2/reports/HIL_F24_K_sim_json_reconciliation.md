# HIL-F24-K: Reconciliation of the HIL-F24-J Contradiction (Codex vs. Claude)

Status: **resolved. Claude's HIL-F24-J conclusion (`NEW_CHIBIOS_GLUE_REQUIRED`)
was wrong. Codex's claim A was correct.** Corrected conclusion:
**`DEFINES_SUFFICIENT`**.

This audit is read-only: no file was edited, no firmware was built or
flashed, no parameter was changed. Verification beyond static reading used
only `-fsyntax-only` compiler invocations (no object/binary output
produced or altered) against the exact command lines already recorded in
`build/fmuv3-SimOnHardWare/compile_commands.json` for the
already-existing build artifact — this is inspection of generated
build configuration, not a build.

## What HIL-F24-J got wrong, and why

The prior HIL-F24-J audit searched `libraries/AP_HAL_ChibiOS/` and
`libraries/AP_HAL_SITL/` for the model-creation/update-loop call chain,
found it only in `AP_HAL_SITL/SITL_cmdline.cpp` and `SITL_State.cpp`
(desktop-SITL-only), and concluded no equivalent existed for ChibiOS. **It
never read `libraries/AP_HAL/SIMState.cpp`/`.h`** — a *third*,
board-generic location, in the shared `AP_HAL` library (not
`AP_HAL_ChibiOS`, not `AP_HAL_SITL`), that implements a deliberately
"cut-down" equivalent of `AP_HAL_SITL`'s model-driving logic specifically
for non-SITL boards. It also never located
`Tools/scripts/sitl-on-hardware/sitl-on-hw.py`, the official, documented
ArduPilot tool for exactly this use case. Both are quoted in full below.

## 1. Exact quoted source blocks

### `libraries/AP_HAL/SIMState.h`

```
libraries/AP_HAL/SIMState.h:57-58,60,69,84-85,87-88,121-122
class AP_HAL::SIMState {
public:

#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
    // simulated airspeed, sonar and battery monitor
    ...
    void update();
    ...
private:
    SITL::SIM *_sitl;

#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
    void _sitl_setup(const char *home_str);
    ...
    // internal SITL model
    SITL::Aircraft *sitl_model;
```

This class (member `sitl_model`, method `update()`) is compiled **only**
for `CONFIG_HAL_BOARD != HAL_BOARD_SITL` — i.e. it is the non-SITL
(hardware) counterpart to `AP_HAL_SITL`'s desktop machinery, not a
SITL-only artifact.

### `libraries/AP_HAL/SIMState.cpp`

```
libraries/AP_HAL/SIMState.cpp:11,25,38-56,58-76,79-90,106-109,114-126
#include <SITL/SIM_Plane.h>
...
#include <SITL/SIM_JSON.h>
...
#ifndef AP_SIM_FRAME_CLASS
#if APM_BUILD_TYPE(APM_BUILD_ArduCopter)
#define AP_SIM_FRAME_CLASS MultiCopter
...
#elif APM_BUILD_TYPE(APM_BUILD_ArduPlane)
#define AP_SIM_FRAME_CLASS Plane
...
#endif
#endif

#ifndef AP_SIM_FRAME_STRING
#if APM_BUILD_TYPE(APM_BUILD_ArduCopter)
#define AP_SIM_FRAME_STRING "+"
...
#elif APM_BUILD_TYPE(APM_BUILD_ArduPlane)
#define AP_SIM_FRAME_STRING "plane"
...
#endif
#endif

#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
void SIMState::update()
{
    static bool init_done;
    if (!init_done) {
        AP::sitl()->init();
        init_done = true;
        sitl_model = SITL::AP_SIM_FRAME_CLASS::create(AP_SIM_FRAME_STRING);
    }

    _fdm_input_step();
}
...
void SIMState::_fdm_input_step(void)
{
    fdm_input_local();
}

/*
  get FDM input from a local model
 */
void SIMState::fdm_input_local(void)
{
    struct sitl_input input;

    // construct servos structure for FDM
    _simulator_servos(input);
    ...
    // update the model
    sitl_model->update_home();
    sitl_model->update_model(input);

    // get FDM output from the model
    if (_sitl == nullptr) {
        _sitl = AP::sitl();
    }
    if (_sitl) {
        sitl_model->fill_fdm(_sitl->state);
    }
```

**This is precisely `SITL::AP_SIM_FRAME_CLASS::create(AP_SIM_FRAME_STRING)`
(line 86) and the per-tick `sitl_model->update_model(input)` driver
(line 126)** that Codex's claim A described. It exists, unconditionally,
for any `CONFIG_HAL_BOARD != HAL_BOARD_SITL` board with `AP_SIM_ENABLED`
— including ChibiOS.

### `libraries/AP_HAL_ChibiOS/HAL_ChibiOS_Class.cpp`

```
libraries/AP_HAL_ChibiOS/HAL_ChibiOS_Class.cpp:112-114
#if AP_SIM_ENABLED
static AP_HAL::SIMState xsimstate;
#endif
```

```
libraries/AP_HAL_ChibiOS/HAL_ChibiOS_Class.cpp:366-368
#if AP_SIM_ENABLED
    AP::sitl()->init();
#endif  // AP_SIM_ENABLED
```

`xsimstate` is the static instance; it is wired into the HAL's driver
table (line 169-171, `&xsimstate,`) and exposed as `hal.simstate`. Its
`update()` is **not** called from anywhere in `AP_HAL_ChibiOS` itself —
see the scheduler call site below, which is common (not
ChibiOS-specific) code.

### `libraries/AP_Scheduler/AP_Scheduler.cpp` (the missing piece from HIL-F24-J — the actual driver)

```
libraries/AP_Scheduler/AP_Scheduler.cpp:428-430
#if AP_SIM_ENABLED && CONFIG_HAL_BOARD != HAL_BOARD_SITL
    hal.simstate->update();
#endif
```

This line is inside `void AP_Scheduler::loop()` (function begins at
`AP_Scheduler.cpp:348`) — **the main vehicle scheduler loop**, called
every loop iteration. This is the call HIL-F24-J failed to find (it
searched `AP_HAL_ChibiOS`/`AP_HAL_SITL` only; this file is neither).

### `libraries/SITL/SIM_config.h`

```
libraries/SITL/SIM_config.h:345-347
#ifndef AP_SIM_JSON_ENABLED
#define AP_SIM_JSON_ENABLED (CONFIG_HAL_BOARD == HAL_BOARD_SITL)
#endif  // AP_SIM_JSON_ENABLED
```

Unchanged from HIL-F24-J's finding — this alone is why `SITL::JSON`
doesn't currently exist in the fmuv3-SimOnHardWare binary. What HIL-F24-J
got wrong was concluding that fixing this define alone would be
insufficient because "nothing calls `create()`" — it does (see above).

### `libraries/SITL/SIM_JSON.h` / `.cpp`

```
libraries/SITL/SIM_JSON.h:59-63,65-70
    // default connection_info_.ip_address
    const char *target_ip = "127.0.0.1";

    // default connection_info_.sitl_ip_port
    uint16_t control_port = 9002;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    SocketAPM_native sock;
#else
    // sim-on-hardware
    SocketAPM sock;
#endif
```

```
libraries/SITL/SIM_JSON.cpp:92-101,118-129
JSON::JSON(const char *frame_str) :
    Aircraft(frame_str),
    sock(true)
{
    printf("Starting SITL: JSON\n");

    const char *colon = strchr(frame_str, ':');
    if (colon) {
        target_ip = colon+1;
    }
    ...
void JSON::set_interface_ports(const char* address, const int port_in, const int port_out)
{
    sock.set_blocking(false);
    sock.reuseaddress();

    if (strcmp("127.0.0.1",address) != 0) {
        target_ip = address;
    }
    control_port = port_out;

    printf("JSON control interface set to %s:%u\n", target_ip, control_port);
}
```

The constructor (called by `create()`, called by `SIMState::update()`)
parses the target IP directly out of the `frame_str` passed to it — i.e.
out of `AP_SIM_FRAME_STRING`. `"json:192.168.144.2"` → `target_ip =
"192.168.144.2"`, `control_port` stays at its default `9002` (matching
the responder's bind address exactly; `set_interface_ports()` is an
alternate/legacy override path not needed here since the frame string
already carries the IP).

### `Tools/scripts/sitl-on-hardware/sitl-on-hw.py`

```
Tools/scripts/sitl-on-hardware/sitl-on-hw.py:1-4
#!/usr/bin/env python3
'''
script to build a firmware for SITL-on-hardware
see https://ardupilot.org/dev/docs/sim-on-hardware.html
```

```
Tools/scripts/sitl-on-hardware/sitl-on-hw.py:114-122
if args.simclass:
    if args.simclass == 'Glider':
        hwdef_write("define AP_SIM_GLIDER_ENABLED 1\n")
    elif args.simclass == 'JSON':
        hwdef_write("define AP_SIM_JSON_ENABLED 1\n")
        defaults_write("ARSPD_TYPE 100\n")
    hwdef_write("define AP_SIM_FRAME_CLASS %s\n" % args.simclass)
if args.frame:
    hwdef_write('define AP_SIM_FRAME_STRING "%s"\n' % args.frame)
```

**This is the official, documented, upstream ArduPilot tool for this
exact scenario**, and for `--simclass JSON --frame json:192.168.144.2` it
generates exactly the three defines Codex proposed, plus one default
parameter (`ARSPD_TYPE 100` — already present in the SR75 bench's
`fmuv3-SimOnHardWare/defaults.parm`, confirmed in HIL-F24-C's audit, so
no gap there either).

## 2. Proof for a ChibiOS SimOnHardware Plane build

| Question | Answer | Evidence |
|---|---|---|
| Where is `AP_HAL::SIMState` instantiated? | `static AP_HAL::SIMState xsimstate;` | `HAL_ChibiOS_Class.cpp:113`, exposed as `hal.simstate` via the driver table at `:169-171` |
| Where is `SIMState::init`/`update` called? | `hal.simstate->update();` every scheduler loop | `AP_Scheduler.cpp:429`, inside `AP_Scheduler::loop()` (`:348`), gated `#if AP_SIM_ENABLED && CONFIG_HAL_BOARD != HAL_BOARD_SITL` — both true for this build |
| Does `AP_SIM_FRAME_CLASS::create()` execute? | **Yes, currently, as `SITL::Plane::create("plane")`** — confirmed by direct binary inspection below | `SIMState.cpp:86`; binary evidence below |
| Does the resulting model's `update()` execute repeatedly? | Yes — `sitl_model->update_model(input)` runs every call to `fdm_input_local()` (`:126`), which runs every `SIMState::update()`, which runs every scheduler loop | `SIMState.cpp:106-109,114-126` |
| Which scheduler/thread, what rate? | `AP_Scheduler::loop()`, the main vehicle thread. `SCHED_LOOP_RATE` defaults to **50 Hz** for ArduPlane (`AP_Scheduler.cpp:44-47`: `APM_BUILD_COPTER_OR_HELI \|\| ArduSub` → 400, else → 50); not overridden anywhere in the SR75 defaults chain, so 50 Hz for this build. | `AP_Scheduler.cpp:44-47`; confirmed no `SCHED_LOOP_RATE` override in `fmuv3-SimOnHardWare/defaults.parm` |

### Binary evidence (already-built `build/fmuv3-SimOnHardWare/bin/arduplane`, not rebuilt for this task)

```
$ arm-none-eabi-nm -C build/fmuv3-SimOnHardWare/bin/arduplane | grep -iE "SIMState|SITL::Plane"
20015380 b xsimstate
080ffa84 T AP_HAL::SIMState::update()
080ff7e8 T AP_HAL::SIMState::fdm_input_local()
080ff5cc T AP_HAL::SIMState::_simulator_servos(sitl_input&)
080cf0cc T SITL::Plane::Plane(char const*)
080d05b0 T SITL::Plane::update(sitl_input const&)
081676f0 T vtable for SITL::Plane
```

`xsimstate`, `SIMState::update()`, and a fully-linked, instantiable
`SITL::Plane` (constructor + `update()` + vtable) are **all present and
active** in the currently-built firmware. This directly proves the
mechanism runs today — just with the wrong model class (`Plane`, the
`AP_SIM_FRAME_CLASS` default for an ArduPlane build) instead of `JSON`.
No `SITL::JSON` symbols exist (matching `AP_SIM_JSON_ENABLED=0`, confirmed
again below) — consistent with HIL-F24-J's finding about `JSON`
specifically, just not its conclusion about the surrounding machinery.

*(Side note, not part of this reconciliation's scope: `SITL::Plane`'s
`calculate_forces()` contains SR75-specific statics — `sr75_fuel_ml`,
`sr75_rato_burning`, etc. — confirming this generic local-physics model
has itself been customized for this project and is likely what has
actually been driving the barometer/compass/IMU `_SITL` backends all
along, independent of any external JSBSim feed, since nothing selects
`JSON` today.)*

## 3. Preprocessor resolution, before and after (empirical, via `-fsyntax-only`)

Method: extracted the exact compiler invocation for
`libraries/AP_HAL/SIMState.cpp` and `libraries/SITL/SIM_JSON.cpp` from
`build/fmuv3-SimOnHardWare/compile_commands.json` (the real flags waf
used for this exact target), stripped only the `-o <obj>` output flag,
and ran `-fsyntax-only` (or `-E -dM`) — no object file or binary was
written or altered; this is inspection, not a build.

**Before** (real flags, no extra `-D`), confirmed via `#if`/`#pragma
message` probes compiled with these exact flags:

```
AP_SIM_JSON_ENABLED_IS_FALSE
AP_NETWORKING_ENABLED_IS_TRUE
AP_NETWORKING_SOCKETS_ENABLED_IS_TRUE
AP_SIM_FRAME_CLASS = Plane
AP_SIM_FRAME_STRING = plane
```

(`AP_NETWORKING_ENABLED`/`AP_NETWORKING_SOCKETS_ENABLED` are already
**true** in the current source tree — `fmuv3-SimOnHardWare/hwdef.dat`
now contains `define AP_NETWORKING_BACKEND_PPP 1` at line 28, which was
not present when HIL-F24-H's report was written; it has evidently been
applied since. This also means `SocketAPM` — the portable,
`AP_NETWORKING_SOCKETS_ENABLED`-gated socket class `SIM_JSON.h:69`'s
hardware branch uses — is already available, confirmed by
`SocketAPM::SocketAPM/sendto/recv/connect/bind` all being linked in the
current binary.)

**After** (adding only the three proposed `-D` flags):

```
$ ... -DAP_SIM_JSON_ENABLED=1 -DAP_SIM_FRAME_CLASS=JSON \
      -DAP_SIM_FRAME_STRING=\"json:192.168.144.2\" -fsyntax-only
AP_SIM_FRAME_CLASS = JSON
AP_SIM_FRAME_STRING = json:192.168.144.2
```

**Full real-file compile check** — the actual `SIMState.cpp` and
`SIM_JSON.cpp`, with the real build flags plus only the three defines,
`-fsyntax-only`:

```
$ <exact SIMState.cpp compile args> -DAP_SIM_JSON_ENABLED=1 \
    -DAP_SIM_FRAME_CLASS=JSON -DAP_SIM_FRAME_STRING=\"json:192.168.144.2\" \
    -fsyntax-only
(no output, RC 0 — compiles cleanly)

$ <exact SIM_JSON.cpp compile args> -DAP_SIM_JSON_ENABLED=1 -fsyntax-only
(no output, RC 0 — compiles cleanly)
```

Both files compile without error or warning with only these three
defines added — no other source change was needed to make them
syntactically/semantically consistent.

## 4. Comparison with `sitl-on-hw.py --simclass JSON --frame json:192.168.144.2`

Reproduced from `sitl-on-hw.py:114-122` (quoted above): this exact
invocation generates precisely:
```
define AP_SIM_JSON_ENABLED 1
define AP_SIM_FRAME_CLASS JSON
define AP_SIM_FRAME_STRING "json:192.168.144.2"
```
plus `ARSPD_TYPE 100` as a default parameter (already present in the SR75
bench's defaults). The script's base hwdef fragments (`plane-extra-hwdef-
sitl-on-hw.dat`) add only `env SIM_ENABLED 1` and a list of
feature-disabling defines (`HAL_NAVEKF2_AVAILABLE 0`, `EK3_FEATURE_*`,
`HAL_ADSB_ENABLED 0`, etc. — functionally the same set `SimOnHW.inc`
already applies for the SR75 targets) plus default parameters
(`SIM_RATE_HZ 400`, `SCHED_LOOP_RATE 400`, `AHRS_EKF_TYPE 10`, etc.). The
build step is a plain `./waf configure --extra-hwdef=... --default-
param=...` followed by `./waf plane` — **no additional C++ source file,
no new class, no new scheduler task is ever generated or required.**

One notable difference worth flagging (not a blocker): the stock script's
`plane-default.param` sets `AHRS_EKF_TYPE 10` (a SITL-stub EKF backend
used for simple validation), whereas the SR75 bench deliberately runs
`AHRS_EKF_TYPE 3` (real EKF3 — confirmed in HIL-F24-C) since exercising
real EKF3 fusion against external JSBSim data is this whole project's
purpose. **Do not copy `AHRS_EKF_TYPE 10` from the stock script** — the
three `AP_SIM_*` defines are the only pieces relevant here; the SR75
bench's own `defaults.parm` should be left as-is otherwise.

The README (`Tools/scripts/sitl-on-hardware/README.md:53-84`) shows a
real boot log from a **real ChibiOS board (MatekH743)** running this
mechanism successfully, including live `EKF3 IMU0/IMU1 initialised` /
`tilt alignment complete` / `origin set` messages — direct, independent,
documented confirmation that `AP_HAL::SIMState`'s model-creation/update
loop works end-to-end on real ChibiOS hardware, not just in theory.

## 5. Conclusion

# `DEFINES_SUFFICIENT`

`AP_HAL::SIMState` (in the shared `AP_HAL` library, not `AP_HAL_ChibiOS`
or `AP_HAL_SITL`) already creates `SITL::AP_SIM_FRAME_CLASS::create(
AP_SIM_FRAME_STRING)` and drives its `update()` every scheduler loop
(50 Hz for this ArduPlane build) via `AP_Scheduler.cpp:429`, entirely
independent of any board-specific glue. This is currently instantiating
`SITL::Plane` (the ArduPlane-build default) rather than `SITL::JSON`
purely because `AP_SIM_FRAME_CLASS`/`AP_SIM_FRAME_STRING` have never been
overridden for this target and `AP_SIM_JSON_ENABLED` is not set. All
three are ordinary hwdef `define`s, verified via direct compilation
(the exact real build flags, `-fsyntax-only`, no build artifacts
produced) to resolve correctly and compile cleanly, and match exactly
what the official, documented `sitl-on-hw.py --simclass JSON --frame
json:<ip>` invocation would generate.

## 6. Exact three-line patch (not applied) and build-only validation plan

```diff
--- a/libraries/AP_HAL_ChibiOS/hwdef/fmuv3-SimOnHardWare/hwdef.dat
+++ b/libraries/AP_HAL_ChibiOS/hwdef/fmuv3-SimOnHardWare/hwdef.dat
@@
 include ../fmuv3/hwdef.dat
 include ../include/SimOnHW.inc
 
+# HIL-F24-K: select the JSON SIM_JSON client as this target's simulated
+# aircraft model (AP_HAL::SIMState's AP_SIM_FRAME_CLASS::create() call,
+# libraries/AP_HAL/SIMState.cpp:86), targeting the bench responder over
+# the already-working PPP link (HIL-F24-H). Matches exactly what
+# Tools/scripts/sitl-on-hardware/sitl-on-hw.py --simclass JSON --frame
+# json:192.168.144.2 generates.
+define AP_SIM_JSON_ENABLED 1
+define AP_SIM_FRAME_CLASS JSON
+define AP_SIM_FRAME_STRING "json:192.168.144.2"
+
 define AP_NETWORKING_BACKEND_PPP 1
 
 # Unique board/firmware identity ...
```

### Build-only validation plan (no flashing, no parameters, no arming)

```sh
# 1. Configure + build only -- do not upload/flash:
./waf configure --board fmuv3-SimOnHardWare
./waf plane

# 2. Confirm SIM_JSON is now actually linked in (read-only inspection,
#    mirrors this report's own evidence-gathering method):
strings build/fmuv3-SimOnHardWare/bin/arduplane | grep -i "Starting SITL: JSON\|JSON control interface set to"
arm-none-eabi-nm -C build/fmuv3-SimOnHardWare/bin/arduplane | grep -i "SITL::JSON"
# expect: both now present (previously absent, per HIL-F24-J)

# 3. Confirm the target IP/port string is embedded correctly:
strings build/fmuv3-SimOnHardWare/bin/arduplane | grep "192.168.144.2"

# 4. Confirm AP_SIM_FRAME_CLASS/STRING resolution directly (same
#    -fsyntax-only technique used in this report, no build artifacts):
#    re-run this report's probe commands against the new build's
#    compile_commands.json and confirm "AP_SIM_FRAME_CLASS = JSON".
```

Only after a clean build and the above read-only confirmations would a
separately-authorized flash + bench verification (repeat HIL-F24-C's
precheck → flash → precheck cycle, then confirm `tcpdump -ni ppp0 udp
port 9002` now shows outgoing request packets) be the appropriate next
step — not performed here.

## Not done (per instruction)

No file was edited. No firmware was built (only `-fsyntax-only`/`-E`
compiler probes against already-recorded build flags, producing no
object or binary output). No flash. No `PARAM_SET`. PPP analysis was not
repeated (HIL-F24-H's finding is cited only where its current
already-applied state — `AP_NETWORKING_BACKEND_PPP 1` in the current
source tree — is directly relevant to confirming `SocketAPM`'s
availability for this task's own conclusion).
