# HIL-F24-R2D: Prevent EKF3 From Latching the Default Canberra Origin

Builds on `HIL_F24_R2B_ahrs_backend_diagnosis.md` (root cause: EKF3's
`GPS_GLOBAL_ORIGIN` permanently latched to ArduPilot's compiled-in
default SITL start location) and `HIL_F24_R2C_ekf_origin_relatch.md`
(host-only mitigation: reorder boot so real data arrives before the
Pixhawk boots — insufficient by itself because the Pixhawk's own boot
sequence still races ahead of the very first SIM_JSON packet). This task
fixes the actual firmware-level race: `AP_GPS_SITL` (the compiled-in GPS
backend that feeds EKF3 on `SimOnHardware`/SIM_JSON boards) no longer
publishes a fix until a real position has been parsed, and withdraws it
if the feed goes stale.

**No hardware was flashed. No PARAM_SET, arm, AUTO, mission, output,
RATO, or control-tuning change was made anywhere in this task.** This is
firmware source + host-only build/test only.

## Root cause, traced (task 1)

`libraries/AP_HAL/SIMState.cpp:136`, `fdm_input_local()` — the real-hardware
scheduler-tick entry point — runs, in this exact order, every tick:

```cpp
sitl_model->update_home();      // (a)
sitl_model->update_model(input); // (b) -- calls JSON::recv_fdm(), bounded to 200ms on real hardware
sitl_model->fill_fdm(_sitl->state); // (c) -- always runs, regardless of whether (b) got fresh data
```

`Aircraft::update_home()` (`libraries/SITL/SIM_Aircraft.cpp:708`):
```cpp
void Aircraft::update_home()
{
    if (!home_is_set) {
        const Location loc{ ...sitl->opos.lat/lng/alt... };  // SIM_OPOS_LAT/LNG default: -35.363261, 149.165230 ("CMAC", Canberra)
        set_start_location(loc, sitl->opos.hdg.get());       // sets home_is_set = true
    }
}
```

This runs **before** `recv_fdm()` gets a chance to receive anything, on
every tick including the very first. On real hardware, `recv_fdm()`'s
receive is bounded to `JSON_HW_RECV_TIMEOUT_MS` (200ms, HIL-F24-N) and
simply returns if nothing has arrived yet — entirely plausible for the
first several ticks after boot, before PPP/the responder have connected.
So `update_home()` wins the race on tick 1, latching `home_is_set = true`
with the Canberra default. `JSON::recv_fdm()`'s own correct handling
(`SIM_JSON.cpp:471`, `if (!home_is_set) { set_start_location(new_loc, ...); }`)
can then **never fire again for the rest of the boot**, even once real
SIM_JSON packets start arriving continuously — `home_is_set` is already
true. `fill_fdm()` (step c) runs every tick regardless, converting
whatever `position`/`home` currently hold into `AP::sitl()->state`, which
`AP_GPS_SITL::read()` (`AP_GPS_SITL.cpp`) reads **unconditionally** and
reports as `FIX_3D` — including on the very first tick, before any real
data has ever arrived. EKF3 then one-shot-latches `GPS_GLOBAL_ORIGIN`
from that first (Canberra) fix, exactly matching the 11,231 km mismatch
captured live in HIL-F24-R2B.

The smallest existing signal proving a fresh, real position packet has
actually been parsed is the check already gating the *correct* branch of
`JSON::recv_fdm()` (`SIM_JSON.cpp:462`):

```cpp
if ((received_bitmask & (LATITUDE | LONGITUDE | ALTITUDE)) == (LATITUDE | LONGITUDE | ALTITUDE)) {
```

This task reuses that exact check — no new parsing logic — to set two
new fields the gate reads.

## The fix (tasks 2-5)

1. **`libraries/SITL/SITL.h`**: two new fields on `SITL::SIM` —
   `json_position_valid` (bool) and `json_position_last_update_ms`
   (uint32_t) — set only by `JSON::recv_fdm()`, deliberately independent
   of `home_is_set` (which the race above can poison).
2. **`libraries/SITL/SIM_JSON.cpp`**: inside the exact `received_bitmask`
   branch traced above, sets both new fields, unconditionally (every
   platform, every tick that branch is taken) — no platform `#if` here,
   since setting the signal is always correct and harmless; only *acting*
   on it is board-gated (next item).
3. **`libraries/AP_GPS/AP_GPS_SITL.h`/`.cpp`**: a new, pure,
   **unconditionally-compiled** static method,
   `AP_GPS_SITL::json_position_is_valid(position_valid, last_update_ms,
   now_ms, stale_ms)` — mirrors the `recv_fdm_bounded()` pattern
   HIL-F24-N already established for exactly this "hardware-only logic
   needs a desktop-testable pure core" problem. `read()` calls it inside
   a **new** `#if CONFIG_HAL_BOARD != HAL_BOARD_SITL` block (task 4 —
   this compiles out entirely on desktop SITL, so desktop behavior is
   provably, not just empirically, unchanged): if not valid, `state.status
   = AP_GPS_FixType::NONE` and return (never publish Canberra). A 2000ms
   staleness threshold (`JSON_POSITION_STALE_MS`, task 5) withdraws the
   fix again if `now - last_update_ms` exceeds it — generous relative to
   a single normal 200ms bounded-receive miss, tight enough to react
   promptly to a genuine feed stall (feeder/responder/PPP going down).

`AP_GPS_FixType::NONE` ("Receiving valid GPS messages but no lock") was
chosen over `NO_GPS` ("No GPS connected") — the backend *is* present and
running, it just doesn't yet/no-longer have a trustworthy position,
exactly matching how a real receiver reports "no fix" vs. "not
connected."

## Exact diff

```diff
--- a/libraries/SITL/SITL.h
+++ b/libraries/SITL/SITL.h
@@ struct sitl_fdm state; float throttle;
+    // HIL-F24-R2D: set by SITL::JSON::recv_fdm() (SIM_JSON.cpp) the
+    // instant a JSON packet actually containing a fresh latitude,
+    // longitude, and altitude has been parsed -- distinct from
+    // Aircraft::home_is_set, which ... can be latched true from the
+    // compiled-in default SITL start location ... before any real
+    // position has ever arrived ...
+    bool json_position_valid;
+    uint32_t json_position_last_update_ms;

--- a/libraries/SITL/SIM_JSON.cpp
+++ b/libraries/SITL/SIM_JSON.cpp
@@ recv_fdm(): inside `if (received_bitmask & (LATITUDE|LONGITUDE|ALTITUDE) == ...)`
         position = origin.get_distance_NED_double(new_loc);
+        if (sitl != nullptr) {
+            sitl->json_position_valid = true;
+            sitl->json_position_last_update_ms = AP_HAL::millis();
+        }

--- a/libraries/AP_GPS/AP_GPS_SITL.h
+++ b/libraries/AP_GPS/AP_GPS_SITL.h
@@ private:
+    friend class AP_GPS_SITL_Test;
     uint32_t last_update_ms;
+    static bool json_position_is_valid(
+        bool position_valid, uint32_t last_position_update_ms, uint32_t now_ms, uint32_t stale_ms);

--- a/libraries/AP_GPS/AP_GPS_SITL.cpp
+++ b/libraries/AP_GPS/AP_GPS_SITL.cpp
@@ file scope, before read()
+static const uint32_t JSON_POSITION_STALE_MS = 2000;
+bool AP_GPS_SITL::json_position_is_valid(bool position_valid, uint32_t last_position_update_ms, uint32_t now_ms, uint32_t stale_ms)
+{
+    if (!position_valid) { return false; }
+    return (now_ms - last_position_update_ms) <= stale_ms;
+}
@@ read(), right after `auto *sitl = AP::sitl();`
+#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
+    if (!json_position_is_valid(sitl->json_position_valid, sitl->json_position_last_update_ms, now, JSON_POSITION_STALE_MS)) {
+        state.status = AP_GPS_FixType::NONE;
+        return true;
+    }
+#endif
```

(Full diff: `git diff -- libraries/AP_GPS/AP_GPS_SITL.h libraries/AP_GPS/AP_GPS_SITL.cpp libraries/SITL/SITL.h libraries/SITL/SIM_JSON.cpp` — 5 files, 175 insertions, 0 deletions, purely additive.)

## Task 4: desktop SITL behavior preserved

Not just tested — **provable by construction**: the entire gating check
lives inside `#if CONFIG_HAL_BOARD != HAL_BOARD_SITL`, which is false for
every desktop SITL build. That preprocessor branch physically does not
exist in a desktop SITL binary; `read()`'s pre-existing behavior
(unconditional `FIX_3D` from `sitl->state`) is untouched byte-for-byte on
desktop. The new `json_position_is_valid()` function and the new
`SITL::SIM` fields *are* compiled on desktop SITL too (so they're
directly unit-testable there, task 6), but nothing on desktop ever reads
or branches on them.

## Task 6: tests added

**New: `libraries/AP_GPS/tests/test_gps_sitl_json_gating.cpp`** (4
tests, `AP_GPS_SITL_Test` friend accessor, mirrors `AP_GPS_NMEA_Test`'s
established pattern) — exercises the pure `json_position_is_valid()`
function directly:
- `NeverValidBeforeFirstPacketIsNotTrustworthy` — **"boot before first
  JSON packet: no 3D fix."**
- `FreshValidPositionIsTrustworthy` — **"first valid position packet: 3D
  fix at supplied truth"** (the gate opens; the existing, unmodified rest
  of `read()` then reports the real `sitl->state` position — see the next
  test for confirmation that state is in fact the supplied truth).
- `StalePositionIsWithdrawn` — **"stale feed: fix withdrawn."**
- `HandlesMillisWraparoundCorrectly` — unsigned-subtraction wraparound
  safety at the `millis()` 49.7-day rollover.

**Modified: `libraries/SITL/tests/test_sim_json.cpp`** (2 new tests, plus
a new file-scope `SITL::SIM sitl_singleton;` so `AP::sitl()` is non-null
in this test binary — required for the new assertions to actually
execute rather than skip; the 3 pre-existing tests are unaffected by its
presence, confirmed by them still passing unchanged):
- `PacketWithoutPositionDoesNotMarkJsonPositionValid` — a JSON packet
  lacking lat/lon/alt (the exact same minimal payload the pre-existing
  `RecvFdmParsesQueuedValidReplyOnUnchangedSitlPath` test already uses)
  must not set `json_position_valid`.
- `PacketWithPositionMarksJsonPositionValidAtSuppliedTruth` — a real
  packet over a real loopback UDP socket, `latitude=32.5378085
  longitude=74.3661944 altitude=240.201118` (R1's runscript IC), sets
  `json_position_valid=true` and a non-zero `json_position_last_update_ms`
  — this is task 1/2's signal-setting side, proven end-to-end over the
  real parser, not just asserted.

**"Desktop SITL unchanged"**: proven by construction (above) and by
regression — all 3 pre-existing `test_sim_json` tests and the
pre-existing `test_gps` (`AP_GPS_NMEA`) test continue to pass unchanged.

### Test results

```
$ ./waf configure --board sitl
$ ./waf build --target tests/test_gps,tests/test_gps_sitl_json_gating,tests/test_sim_json
'build' finished successfully

$ ./build/sitl/tests/test_gps_sitl_json_gating
[==========] 4 tests from 1 test suite ran.
[  PASSED  ] 4 tests.

$ ./build/sitl/tests/test_sim_json
[==========] 5 tests from 1 test suite ran.
[  PASSED  ] 5 tests.       (3 pre-existing + 2 new, all pass)

$ ./build/sitl/tests/test_gps
[==========] 1 test from 1 test suite ran.
[  PASSED  ] 1 test.        (pre-existing, unaffected)

$ ./waf plane        # full ArduPlane SITL, not just the unit tests
'plane' finished successfully
Target         Text (B)  Data (B)  BSS (B)  Total Flash Used (B)
bin/arduplane   4345006    211949   222784   4556955
```

## Task 7: fmuv3-SimOnHardWare build result

```
$ ./waf configure --board fmuv3-SimOnHardWare
'configure' finished successfully

$ ./waf plane
'plane' finished successfully (32.7s)
Target         Text (B)  Data (B)  BSS (B)  Total Flash Used (B)  Free Flash (B)
bin/arduplane   1490712      4256   113748   1494968               585796
```

**`.apj` path**: `/home/missi/ardupilot_clean/build/fmuv3-SimOnHardWare/bin/arduplane.apj`
(also `arduplane.bin`, `arduplane` ELF, same directory; MD5 of the `.apj`:
`4fe0bc2a4828bf1fcfaae1a31188bf7b`).

No hwdef/`defaults.parm` change was needed or made — `GPS1_TYPE 100`/
`AHRS_EKF_TYPE 3` (HIL-F24-B's existing target) are unaffected by this
fix; the fix changes GPS *behavior before a fix is available*, not any
parameter or the GPS type selection itself.

## Manual flash steps (operator-executed only — NOT run in this task)

Following HIL-F24-B's established Stage-1 procedure exactly (see
`HIL_F24_B_fmuv3_simulation_on_hardware_implementation.md` §8-9):

1. **Physically disconnect** all servo/engine/RATO/actuator loads from
   the bench Pixhawk.
2. **Back up current parameters** first: `python3 Tools/autotest/
   sr75_hil_layer2/scripts/sr75_hil_gps_ekf_readonly_audit.py --pixhawk
   /dev/ttyACM0` (read-only) and/or a full Mission Planner param fetch,
   saved to a dated backup location.
3. **Flash**:
   ```sh
   python3 -m serial.tools.list_ports          # identify the bench Pixhawk port first
   ./waf configure --board fmuv3-SimOnHardWare
   ./waf plane --upload
   ```
   (or load `build/fmuv3-SimOnHardWare/bin/arduplane.apj` via Mission
   Planner's firmware-upload dialog — equivalent, operator's choice.)
4. **Verify over USB only first** (no PPP yet): heartbeat, the
   `"SIMULATION FIRMWARE"`/`AP_CUSTOM_FIRMWARE_STRING` banner text,
   board name `fmuv3-SR75-SoH`, `GPS1_TYPE`/`AHRS_EKF_TYPE` read back as
   `100`/`3`.
5. Only then bring up PPP and proceed with HIL-F24-R2/R2C's existing
   guarded procedures.

## Post-flash read-only origin verification command

Reuses HIL-F24-R2B/R2C's existing, unmodified, read-only diagnostic —
now expected to show `GPS_GLOBAL_ORIGIN` matching truth immediately
after boot (not just `HOME_POSITION`, which was already correct even
before this fix):

```sh
python3 Tools/autotest/sr75_hil_layer2/scripts/sr75_hil_f24r2b_ahrs_origin_diagnostics.py \
    --pixhawk /dev/ttyACM0 --listen-s 6
```

Expected result after this fix (once PPP/feeder/responder are up and
answering): `GPS_GLOBAL_ORIGIN vs. truth: horizontal=<small, e.g. < 50m>
m` (previously `11231266.38 m`, the Canberra default). Before PPP/the
feeder connect at all, this diagnostic should now report `GPS_GLOBAL_
ORIGIN: NOT RECEIVED` or a `NO_FIX`-consistent state rather than a
confidently-wrong Canberra value — itself a direct, observable
confirmation the fix is working, obtainable without ever running a full
R2/R2C capture.

## Files changed

- `libraries/SITL/SITL.h` — 2 new fields.
- `libraries/SITL/SIM_JSON.cpp` — sets them at the existing validity
  check.
- `libraries/AP_GPS/AP_GPS_SITL.h` — new private static method +
  friend-test declaration.
- `libraries/AP_GPS/AP_GPS_SITL.cpp` — the method's implementation, the
  staleness constant, and the new real-hardware-only gate in `read()`.
- `libraries/SITL/tests/test_sim_json.cpp` — 2 new tests + a file-scope
  `SITL::SIM` singleton needed for them to execute (not skip).
- `libraries/AP_GPS/tests/test_gps_sitl_json_gating.cpp` — new, 4 tests.
- This report (new).
- Unchanged: `fmuv3-SimOnHardWare/hwdef.dat`/`defaults.parm`, every other
  SITL aircraft model, all control/RATO/mission code, all Python
  orchestrator/bench tooling from R1/R2/R2A/R2B/R2C.

Not committed, not flashed (per standing instruction and this task's
explicit constraint).
