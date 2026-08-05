# HIL-F24-N: JSON+PPP Boot Watchdog Loop Fix and Corrected-Artifact Validation (Build-Only)

Status: fix applied, compiled, linked, and tested. No firmware was
flashed, no `uploader.py` invoked, no Mission Planner access, no
`PARAM_SET`, no hardware PPP started, no arm/AUTO, no actuator output
enabled. Only the three files listed below plus one new host-only test
file were touched; no RATO/aero/TECS/control/mission-logic file was
modified.

## PASS/FAIL for corrected JSON+PPP artifact readiness

**PASS.** The root cause identified in
`HIL_F24_M_json_ppp_reboot_loop_diagnosis.md` (an unbounded main-thread
receive loop in `JSON::recv_fdm()` that starved the ChibiOS watchdog pat)
is fixed with a bounded, hardware-only receive path. The secondary
heap-exhaustion risk (an unchecked `AP_SIM_FRAME_CLASS::create()` and an
oversized `sensor_buffer`) is also addressed. The `fmuv3-SimOnHardWare`
target rebuilds cleanly from scratch, JSON and PPP symbols and the
embedded target IP string are all still present, flash/RAM stay within
limits, and three new host-side regression tests exercise the fix over
real loopback sockets and all pass, alongside the existing SITL test
suite (no regressions).

## 1. Exact diff

### `libraries/SITL/SIM_JSON.cpp`

```diff
@@ -36,6 +36,19 @@
 #define UDP_TIMEOUT_MS 100
 #define JSON_AIRSPEED_MAX_PRESSURE_PA 50000.0f
 
+// HIL-F24-N: on real hardware (CONFIG_HAL_BOARD != HAL_BOARD_SITL),
+// JSON::update() runs synchronously on the main vehicle thread (called
+// from AP_HAL::SIMState::update(), itself called every AP_Scheduler::
+// loop() tick -- see HIL-F24-M). An unbounded wait for a UDP reply here
+// therefore starves the ChibiOS watchdog/monitor-thread pat and forces a
+// hardfault/reset after ~1.8-2.1s if no reply arrives in time (this was
+// the confirmed cause of the JSON+PPP boot reboot loop). Bound the wait
+// to a couple of the existing 100ms recv() timeouts and let the next
+// scheduler tick retry naturally instead of blocking here -- long enough
+// to catch a reply that is already in flight, short enough to never
+// approach the ~500ms "main loop stuck" internal-error threshold.
+#define JSON_HW_RECV_TIMEOUT_MS 200
+
 extern const AP_HAL::HAL& hal;
 
 using namespace SITL;
@@ -325,6 +338,23 @@ uint64_t JSON::parse_sensors(const char *json)
     return received_bitmask;
 }
 
+/*
+    HIL-F24-N: single bounded-wait receive attempt, used by the non-SITL
+    branch of recv_fdm() below. See the declaration in SIM_JSON.h for why
+    this is its own unconditionally-compiled method rather than inlined
+    inside the `#if CONFIG_HAL_BOARD != HAL_BOARD_SITL` block.
+*/
+ssize_t JSON::recv_fdm_bounded(uint32_t timeout_ms)
+{
+    ssize_t ret = 0;
+    uint32_t waited_ms = 0;
+    while (ret <= 0 && waited_ms < timeout_ms) {
+        ret = sock.recv(&sensor_buffer[sensor_buffer_len], sizeof(sensor_buffer)-sensor_buffer_len, UDP_TIMEOUT_MS);
+        waited_ms += UDP_TIMEOUT_MS;
+    }
+    return ret;
+}
+
 /*
     Receive new sensor data from simulator
     This is a blocking function
@@ -347,6 +377,21 @@ void JSON::recv_fdm(const struct sitl_input &input)
         return;
     }
 
+#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
+    // Bounded wait: never block the caller (the main vehicle thread) for
+    // more than JSON_HW_RECV_TIMEOUT_MS. If nothing has arrived by then,
+    // return without parsing; JSON::update() is called again on the very
+    // next scheduler tick (a few ms later at typical loop rates), which
+    // will resend servos and try again -- so no separate "resend after
+    // N ms" fallback is needed here the way the desktop SITL path below
+    // needs one for its much longer unbounded wait.
+    if (ret <= 0 && wait_ms < JSON_HW_RECV_TIMEOUT_MS) {
+        ret = recv_fdm_bounded(JSON_HW_RECV_TIMEOUT_MS - wait_ms);
+    }
+    if (ret <= 0) {
+        return;
+    }
+#else
     while (ret <= 0) {
         //printf("No JSON sensor message received - %s\n", strerror(errno));
         ret = sock.recv(&sensor_buffer[sensor_buffer_len], sizeof(sensor_buffer)-sensor_buffer_len, UDP_TIMEOUT_MS);
@@ -358,6 +403,7 @@ void JSON::recv_fdm(const struct sitl_input &input)
             output_servos(input);
         }
     }
+#endif
 
     // convert '\n' into nul
     while (uint8_t *p = (uint8_t *)memchr(&sensor_buffer[sensor_buffer_len], '\n', ret)) {
```

The desktop SITL `#else` branch is byte-for-byte the original code (only
wrapped in the new `#if`/`#else`/`#endif`); nothing in its logic, timing,
or resend behavior changed.

### `libraries/SITL/SIM_JSON.h`

```diff
@@ -40,6 +40,14 @@ public:
     /* Create and set in/out socket for JSON generic simulator */
     void set_interface_ports(const char* address, const int port_in, const int port_out) override;
 
+    // HIL-F24-N: grants host-side unit tests (libraries/SITL/tests/test_sim_json.cpp)
+    // access to otherwise-private members needed to exercise recv_fdm()/
+    // recv_fdm_bounded() end-to-end over a real loopback socket, without
+    // widening JSON's public API. Mirrors the existing `friend class Ship;`-
+    // style test/interop access pattern already used elsewhere in this
+    // directory (SIM_Ship.h, SIM_ADSB.h, SIM_SlungPayload.h).
+    friend class JSONTestAccess;
+
 private:
 
     struct servo_packet_16 {
@@ -75,10 +83,41 @@ private:
     void output_servos(const struct sitl_input &input);
     void recv_fdm(const struct sitl_input &input);
 
+    // HIL-F24-N: single bounded-wait receive attempt used by the non-SITL
+    // (real hardware) branch of recv_fdm(). Factored out of recv_fdm() as its
+    // own method -- rather than left inline inside the `#if CONFIG_HAL_BOARD
+    // != HAL_BOARD_SITL` block -- so that it compiles unconditionally on
+    // every board (it only touches `sock`, whose interface is identical for
+    // both the SocketAPM and SocketAPM_native cases) and can therefore be
+    // exercised directly by host-side unit tests even though a desktop test
+    // build (CONFIG_HAL_BOARD == HAL_BOARD_SITL) never itself takes the
+    // non-SITL branch of recv_fdm(). See libraries/SITL/tests/test_sim_json.cpp.
+    // Blocks for at most `timeout_ms`, in UDP_TIMEOUT_MS-sized slices, and
+    // returns whatever the last sock.recv() call returned (<=0 if nothing
+    // arrived in time).
+    ssize_t recv_fdm_bounded(uint32_t timeout_ms);
+
     uint64_t parse_sensors(const char *json);
 
-    // buffer for parsing pose data in JSON format
-    uint8_t sensor_buffer[65000];
+    // buffer for parsing pose data in JSON format.
+    //
+    // HIL-F24-N: reduced from 65000 to 8192 bytes. Evidence: the
+    // largest real SR-75 bench reply (all optional fields populated,
+    // 16 RC channels included) measures 661 bytes
+    // (Tools/autotest/sr75_hil_layer2/sim_json/sr75_sim_json_test_profiles.py's
+    // own generators via SimState.to_json_bytes()); the protocol's
+    // fixed 36-entry keytable[] bounds the theoretical worst case (every
+    // optional field present, maximally verbose float text) to a low
+    // single-digit KB. 8192 bytes keeps a >12x margin over the largest
+    // observed real payload while cutting sizeof(SITL::JSON) from a
+    // measured 68016 bytes to 11208 bytes (compiled with the exact
+    // fmuv3-SimOnHardWare build flags) -- against a 78748-byte total
+    // heap on that target, the original size left only ~10.7KB of
+    // margin for everything else allocated during boot; the reduced
+    // size leaves ~67.5KB. This buffer is also used to accumulate
+    // multiple queued datagrams (see recv_fdm()'s memmove/memrchr
+    // logic), so it is not reduced to the bare single-message size.
+    uint8_t sensor_buffer[8192];
     uint32_t sensor_buffer_len;
```

### `libraries/AP_HAL/SIMState.cpp`

```diff
@@ -28,6 +28,7 @@
 #include <AP_Baro/AP_Baro.h>
 
 #include <AP_BoardConfig/AP_BoardConfig.h>
+#include <AP_InternalError/AP_InternalError.h>
 
 extern const AP_HAL::HAL& hal;
 
@@ -84,6 +85,16 @@ void SIMState::update()
         AP::sitl()->init();
         init_done = true;
         sitl_model = SITL::AP_SIM_FRAME_CLASS::create(AP_SIM_FRAME_STRING);
+        if (sitl_model == nullptr) {
+            // HIL-F24-N: create() can fail (e.g. heap exhaustion for a
+            // large model). Fail safely with a logged internal error
+            // instead of letting _fdm_input_step() dereference a null
+            // sitl_model below on every subsequent tick.
+            INTERNAL_ERROR(AP_InternalError::error_t::mem_guard);
+        }
+    }
+    if (sitl_model == nullptr) {
+        return;
     }
 
     _fdm_input_step();
```

### New file: `libraries/SITL/tests/test_sim_json.cpp` (162 lines)

Three new gtest cases (`JSON.RecvFdmBoundedTimesOutWithoutBusyLooping`,
`JSON.RecvFdmBoundedReturnsPromptlyWhenReplyAlreadyQueued`,
`JSON.RecvFdmParsesQueuedValidReplyOnUnchangedSitlPath`) plus a small
`friend`-only `JSONTestAccess` accessor. Full listing in the file itself;
behavior described in section 4 below.

## 2. Explanation of the bounded receive behavior

Previously, `JSON::recv_fdm()` had one receive loop, shared by every
board: `while (ret <= 0) { ret = sock.recv(...); ... if (wait_ms > 1000)
{ resend servos } }` — no upper bound at all. HIL-F24-M traced this to
being called synchronously from `AP_HAL::SIMState::update()`, itself
called every `AP_Scheduler::loop()` tick on the main vehicle thread, so
any wait here directly starves the ChibiOS monitor thread's watchdog pat
(`libraries/AP_HAL_ChibiOS/Scheduler.cpp`), producing a deliberate
self-hardfault after ~1.8-2.1s.

The fix splits this into two branches, selected the same way the rest of
the codebase already distinguishes desktop SITL from real hardware
(`#if CONFIG_HAL_BOARD != HAL_BOARD_SITL`):

- **Non-SITL (real hardware) branch:** after the existing shared first
  `sock.recv()` call (100ms timeout) fails, at most one more `100ms`
  attempt is made via the new `recv_fdm_bounded()` method, for a total
  cap of `JSON_HW_RECV_TIMEOUT_MS = 200ms`. If still nothing has arrived,
  `recv_fdm()` returns immediately without parsing. Nothing is lost:
  `JSON::update()` (and therefore `recv_fdm()`) runs again on the very
  next scheduler tick, a few milliseconds later at typical loop rates,
  which re-sends servos and tries again — so the bounded path never
  needs its own "resend after N ms" fallback the way the desktop path
  does.
- **Desktop SITL branch (`#else`):** completely unchanged, byte-for-byte,
  including its unbounded wait and its 1-second servo-resend fallback —
  this preserves existing lockstep behavior exactly as required.

`recv_fdm_bounded()` itself was factored out into its own,
unconditionally-compiled method (rather than left inline inside the
`#if`) specifically so it can be exercised directly by a host-side gtest
even though gtest can only be built for `CONFIG_HAL_BOARD ==
HAL_BOARD_SITL` targets (`./waf configure --board fmuv3-SimOnHardWare`
prints "Gtest: STM32 boards currently don't support compiling gtest",
confirmed again during this task) — see section 4.

## 3. Timeout used and why

`JSON_HW_RECV_TIMEOUT_MS = 200`, built from two `UDP_TIMEOUT_MS = 100`ms
`sock.recv()` timeouts (the first already shared with the no-lockstep
check above it, the second via `recv_fdm_bounded()`). Rationale,
carried over from the HIL-F24-M diagnosis and re-verified here:

- Long enough to catch a reply that is already in flight on a healthy
  PPP link (a normal round-trip is well under 100ms on a local link).
- Short enough to stay well clear of both watchdog thresholds
  established in HIL-F24-M: the `AP_InternalError::error_t::
  main_loop_stuck` warning at ~500ms, and the deliberate
  hardfault-preemption jump at ~1800ms. 200ms leaves a >2.5x margin to
  the first threshold and a >9x margin to the second.
- Matches the existing 100ms granularity already used elsewhere in this
  function, rather than inventing a new timeout constant/mechanism.

## 4. Heap / object-size analysis

Unchanged from HIL-F24-M's measurements, empirically re-confirmed for
this task using the exact `fmuv3-SimOnHardWare` build flags via the
compile-time `sizeof()`-via-compile-error technique used throughout this
session:

| | `sensor_buffer` size | `sizeof(SITL::JSON)` |
|---|---|---|
| Before HIL-F24-N | 65000 bytes | 68016 bytes |
| After HIL-F24-N | 8192 bytes | 11208 bytes |

Against this target's `78748`-byte total heap
(`build/fmuv3-SimOnHardWare/hwdef.h`'s `HAL_MEMORY_MAX`/allocator
region), the original 65000-byte buffer left only ~10.7KB of margin for
every other heap allocation made during boot (AP_Param, GCS buffers,
logging, scripting, etc.) — a single heap-allocated `SITL::JSON`
instance consumed 87% of the entire heap on its own. The reduced buffer
leaves ~67.5KB (86% free), a >6x improvement in available margin.

**Evidence for the 8192-byte choice (not reduced without evidence, per
the task's explicit instruction):** the SR-75 bench's own JSON payload
generator (`Tools/autotest/sr75_hil_layer2/sim_json/
sr75_sim_json_test_profiles.py`'s use of `SimState.to_json_bytes()`,
also used by the real bench responder) produces replies of 336-455 bytes
without RC channels and 542-661 bytes with all 16 RC channels populated
— the largest real payload observed anywhere in this project's captures
is 661 bytes. The wire format's parser (`JSON::parse_sensors()`) is
driven by a fixed 36-entry `keytable[]`, which upper-bounds the
theoretical worst case (every optional field present, maximally verbose
floating-point text) to a low single-digit-KB message. 8192 bytes keeps
a >12x margin over the largest real payload ever observed. The buffer
was kept well above the single-message size (rather than shrunk further)
because `recv_fdm()`'s memmove/memrchr logic accumulates multiple queued
`\n`-delimited datagrams in it before parsing the appropriate one.

**Null-check:** `AP_HAL::SIMState::update()` previously called
`SITL::AP_SIM_FRAME_CLASS::create(...)` and then unconditionally called
`_fdm_input_step()`, which dereferences `sitl_model`, with no check that
`create()` (a `NEW_NOTHROW` allocation) actually succeeded. It now logs
`INTERNAL_ERROR(AP_InternalError::error_t::mem_guard)` — the existing,
semantically-closest error code for a failed/guarded memory allocation,
avoiding a new enum value — and returns early on every subsequent tick if
`sitl_model` is null, rather than crashing on a null-pointer dereference.
`mem_guard` was chosen over inventing a new `error_t` value since
`AP_InternalError` already models this exact class of failure.

## 5. Test results

### New tests (`libraries/SITL/tests/test_sim_json.cpp`, host build, `--board sitl`)

Built and run via `./waf configure --board sitl && ./waf tests &&
./build/sitl/tests/test_sim_json`:

```
[==========] Running 3 tests from 1 test suite.
[----------] Global test environment set-up.
[----------] 3 tests from JSON
[ RUN      ] JSON.RecvFdmBoundedTimesOutWithoutBusyLooping
[       OK ] JSON.RecvFdmBoundedTimesOutWithoutBusyLooping (200 ms)
[ RUN      ] JSON.RecvFdmBoundedReturnsPromptlyWhenReplyAlreadyQueued
[       OK ] JSON.RecvFdmBoundedReturnsPromptlyWhenReplyAlreadyQueued (1 ms)
[ RUN      ] JSON.RecvFdmParsesQueuedValidReplyOnUnchangedSitlPath
[       OK ] JSON.RecvFdmParsesQueuedValidReplyOnUnchangedSitlPath (0 ms)
[----------] 3 tests from JSON (201 ms total)
[  PASSED  ] 3 tests.
```

What each test proves, over real loopback UDP sockets (not mocked):

1. **`RecvFdmBoundedTimesOutWithoutBusyLooping`** — binds `JSON`'s own
   socket, sends nothing, calls `recv_fdm_bounded(200)` directly.
   Asserts: (a) it returns `<= 0` rather than hanging — this is exactly
   the behavior that, before this fix, would have starved the watchdog
   and forced a hardfault after ~1.8-2.1s; (b) wall-clock elapsed is
   180-1000ms (bounded, not instant, not unbounded); (c) CPU time
   consumed is far less than wall-clock time elapsed, proving the wait
   blocks on the socket's own `poll()`-based timeout rather than
   busy-spinning — directly satisfying "no busy-loop is introduced."
2. **`RecvFdmBoundedReturnsPromptlyWhenReplyAlreadyQueued`** — a peer
   socket sends a datagram before `recv_fdm_bounded(200)` is called;
   asserts it returns `> 0` in well under the 200ms bound (< 150ms),
   proving a reply already in flight is still picked up promptly.
3. **`RecvFdmParsesQueuedValidReplyOnUnchangedSitlPath`** — a peer sends
   a real SR-75-format JSON reply (mirroring `SimState.to_json_bytes()`)
   before calling `recv_fdm()` (not `recv_fdm_bounded()`); this exercises
   the desktop SITL `#else` branch, left byte-for-byte unchanged, plus
   the parsing tail shared unconditionally by both platforms. Asserts
   `state.timestamp_s` matches the sent value, proving valid replies are
   still parsed correctly and the untouched SITL branch still works.

**Known limitation, stated plainly:** gtest cannot be compiled for
`CONFIG_HAL_BOARD != HAL_BOARD_SITL` targets at all (`Gtest: STM32
boards currently don't support compiling gtest`), so the actual `#if
CONFIG_HAL_BOARD != HAL_BOARD_SITL` branch of `recv_fdm()` itself cannot
be executed inside this host test binary — only `recv_fdm_bounded()`,
the method it calls, can be (and is, in tests 1 and 2 above, which
exercise the exact bounded-loop logic and timeout constant that branch
uses). The `#if != SITL` branch's call site was separately verified via
`-fsyntax-only` against the real `fmuv3-SimOnHardWare` build flags (RC
0, zero warnings) and via the full `./waf plane` link below.

### Existing SITL test suite (regression check)

`./waf check --alltests` on `--board sitl`: 60 test binaries ran,
including all pre-existing `libraries/SITL/tests/*` binaries
(`test_battery`, `test_sim_aircraft_filtered_servo_angle`,
`test_sim_ms5525`, `test_sim_ms5611`) plus the new `test_sim_json` — all
five returned exit code 0. 4 of 60 binaries had pre-existing failures
(`test_bitmask`, `test_math_double`, `test_rotations`, `test_math` — all
in `AP_Math`, all death-test/sandbox-environment artifacts such as
`Death test: ... Result: failed to die`). Confirmed via `git stash` that
these same 4 failures reproduce identically with none of this task's
changes applied — they are pre-existing and unrelated to `SIM_JSON.cpp`,
`SIM_JSON.h`, or `SIMState.cpp`.

### `py_compile` / `flake8` / `git diff --check`

This task's diff touches only `.cpp`/`.h` files — no Python file was
added or modified, so `py_compile`/`flake8` have nothing new to check.
`git diff --check` on the three modified files: clean, no whitespace
errors.

## 6. Build size

Clean rebuild via `./waf clean && ./waf configure --board
fmuv3-SimOnHardWare && ./waf plane` (all three commands run exactly as
specified, all succeeded):

| | Before (pre-HIL-F24-N) | After (HIL-F24-N applied) |
|---|---|---|
| `.text` | 1,490,200 | 1,490,236 (+36 B) |
| `.data` | 4,256 | 4,256 |
| `.bss` | 105,036 | 105,036 |
| `.heap` (reserved) | 78,748 | 78,748 |
| Total Flash Used | — | 1,494,980 |
| Free Flash | — | 585,780 |

`.bss`/`.data` are unchanged because `SITL::JSON` is heap-allocated via
`NEW_NOTHROW` in `SIMState::update()`, not a static/global object — the
`sensor_buffer` reduction changes heap consumption at runtime (see
section 4), not any static section size. `.text` grew by a negligible 36
bytes despite adding a new function and null-check logic, offset by
other codegen differences. The build completed with no linker errors
(no flash overflow), and `waf`'s own summary confirms 585,780 bytes of
flash still free.

## 7. `strings`/`nm` evidence

```
$ arm-none-eabi-nm -C build/fmuv3-SimOnHardWare/bin/arduplane | grep "SITL::JSON"
081184c8 T SITL::JSON::output_servos(sitl_input const&)
081185bc T SITL::JSON::parse_sensors(char const*)
081188a4 T SITL::JSON::recv_fdm_bounded(unsigned long)   <- new
08117f78 T SITL::JSON::set_interface_ports(char const*, int, int)
08118ff4 T SITL::JSON::update(sitl_input const&)
081188e0 T SITL::JSON::recv_fdm(sitl_input const&)
08117fbc T SITL::JSON::JSON(char const*)
08117fbc T SITL::JSON::JSON(char const*)
0816cd34 T vtable for SITL::JSON

$ arm-none-eabi-nm -C build/fmuv3-SimOnHardWare/bin/arduplane | grep -ci ppp
40   (unchanged from the pre-HIL-F24-N baseline)

$ strings build/fmuv3-SimOnHardWare/bin/arduplane | grep 192.168.144.2
json:192.168.144.2

$ arm-none-eabi-nm -C build/fmuv3-SimOnHardWare/bin/arduplane | grep "SIMState::update"
080f9ff8 T AP_HAL::SIMState::update_simulated_wind(sitl_input&)
080fa654 T AP_HAL::SIMState::update()
20015989 b AP_HAL::SIMState::update()::init_done
```

All JSON symbols present (8 pre-existing + the new `recv_fdm_bounded`),
all PPP symbols present and unchanged in count, the target simulator IP
string is still embedded, and `SIMState::update()`'s null-check compiled
in as expected.

`git status`/`git diff --check` confirm only the three intended files
changed plus the one new test file — no RATO, aero, TECS, control, or
mission-logic file was touched by this task.

## 8. Exact artifact path

- ELF: `build/fmuv3-SimOnHardWare/bin/arduplane`
- Raw binary: `build/fmuv3-SimOnHardWare/bin/arduplane.bin`
- Flashable package: `build/fmuv3-SimOnHardWare/bin/arduplane.apj`

## 9. Manual Mission Planner flashing instructions (for later, user-driven action — not executed)

1. Connect the Pixhawk 2.4.8 (fmuv3) via USB.
2. In Mission Planner: **SETUP → Install Firmware → Load custom
   firmware**, select `build/fmuv3-SimOnHardWare/bin/arduplane.apj`.
3. Confirm the board identifies itself as `fmuv3-SR75-SoH` /
   `SR75-SIMULATION-FIRMWARE-fmuv3-SoH-DO-NOT-FLY` (per
   `CHIBIOS_SHORT_BOARD_NAME`/`AP_CUSTOM_FIRMWARE_STRING` in
   `fmuv3-SimOnHardWare/hwdef.dat`) before proceeding with any bench test.
4. After flashing, monitor USB serial output through a full boot cycle
   with the JSON/PPP simulator peer both absent and present, to confirm
   the board now stays up (no repeated connect/disconnect) even when no
   simulator reply ever arrives.

## 10. PASS/FAIL

**PASS.** Root cause fixed (bounded hardware-only receive, desktop SITL
path unchanged), secondary heap risk addressed with evidence-based
sizing and a safe null-check, all three edits verified via
`-fsyntax-only` against real build flags before rebuilding, a full clean
`./waf plane` rebuild succeeds with JSON/PPP symbols and the target IP
string intact and flash/RAM within limits, three new regression tests
pass, the pre-existing SITL test suite shows no new failures, and
`git diff --check` is clean. The corrected `fmuv3-SimOnHardWare` artifact
is ready for the user to flash and bench-test manually.
