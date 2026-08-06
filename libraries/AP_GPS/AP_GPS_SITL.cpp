/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AP_GPS_SITL.h"

#if AP_SIM_GPS_ENABLED

#include <ctype.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/time.h>

extern const AP_HAL::HAL& hal;

// HIL-F24-R2D: how long a fix is trusted after the last real JSON
// latitude/longitude/altitude packet before it is withdrawn (NONE) if
// SIM_JSON truth stops updating -- generous relative to a single missed
// receive cycle (JSON_HW_RECV_TIMEOUT_MS in SIM_JSON.cpp is 200ms, and a
// single bounded-wait timeout there is expected/normal, not a real
// outage), tight enough to withdraw promptly on a genuine feed stall
// (e.g. the feeder/responder/PPP link actually going down). Referenced
// only from the `#if CONFIG_HAL_BOARD != HAL_BOARD_SITL` block in
// read() below, but left unconditionally compiled/visible (like
// json_position_is_valid() itself) for the desktop-SITL unit tests.
static const uint32_t JSON_POSITION_STALE_MS = 2000;

bool AP_GPS_SITL::json_position_is_valid(
    bool position_valid, uint32_t last_position_update_ms, uint32_t now_ms, uint32_t stale_ms)
{
    if (!position_valid) {
        return false;
    }
    return (now_ms - last_position_update_ms) <= stale_ms;
}

/*
  return GPS time of week in milliseconds
 */
/*
  get timeval using simulation time
 */
static void simulation_timeval(struct timeval *tv)
{
    uint64_t now = AP_HAL::micros64();
    static uint64_t first_usec;
    static struct timeval first_tv;
    if (first_usec == 0) {
        first_usec = now;
        first_tv.tv_sec = AP::sitl()->start_time_UTC;
    }
    *tv = first_tv;
    tv->tv_sec += now / 1000000ULL;
    uint64_t new_usec = tv->tv_usec + (now % 1000000ULL);
    tv->tv_sec += new_usec / 1000000ULL;
    tv->tv_usec = new_usec % 1000000ULL;
}
static void gps_time(uint16_t *time_week, uint32_t *time_week_ms)
{
    struct timeval tv;
    simulation_timeval(&tv);
    const uint32_t epoch = 86400*(10*365 + (1980-1969)/4 + 1 + 6 - 2) - (GPS_LEAPSECONDS_MILLIS / 1000ULL);
    uint32_t epoch_seconds = tv.tv_sec - epoch;
    *time_week = epoch_seconds / AP_SEC_PER_WEEK;
    uint32_t t_ms = tv.tv_usec / 1000;
    // round time to nearest 200ms
    *time_week_ms = (epoch_seconds % AP_SEC_PER_WEEK) * AP_MSEC_PER_SEC + ((t_ms/200) * 200);
}

bool AP_GPS_SITL::read(void)
{
    const uint32_t now = AP_HAL::millis();
    if (now - last_update_ms < 200) {
        return false;
    }
    last_update_ms = now;

    auto *sitl = AP::sitl();

#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
    // HIL-F24-R2D: on real hardware (SimOnHardware/SIM_JSON boards
    // only -- this branch compiles out entirely for desktop SITL, see
    // #if above), never publish a fix built from sitl->state before
    // SITL::JSON::recv_fdm() has actually parsed a real latitude/
    // longitude/altitude packet (json_position_valid), and withdraw the
    // fix again if that feed goes stale. Without this, Aircraft::
    // update_home() can latch home_is_set (and therefore sitl->state)
    // to the compiled-in default SITL start location -- ArduPilot's
    // well-known CMAC/Canberra OPOS default -- on the very first
    // scheduler tick, before SIM_JSON/PPP has connected at all, and
    // that default then permanently poisons EKF3's one-shot GPS_GLOBAL_
    // ORIGIN latch for the rest of the boot (see HIL_F24_R2B_ahrs_
    // backend_diagnosis.md / HIL_F24_R2D_gps_sitl_fix_gating.md).
    if (!json_position_is_valid(sitl->json_position_valid, sitl->json_position_last_update_ms, now, JSON_POSITION_STALE_MS)) {
        state.status = AP_GPS_FixType::NONE;
        return true;
    }
#endif

    double latitude =sitl->state.latitude;
    double longitude = sitl->state.longitude;
    float altitude = sitl->state.altitude;
    const double speedN = sitl->state.speedN;
    const double speedE = sitl->state.speedE;
    const double speedD = sitl->state.speedD;
    // const double yaw = sitl->state.yawDeg;

    uint16_t time_week;
    uint32_t time_week_ms;

    gps_time(&time_week, &time_week_ms);

    state.time_week = time_week;
    state.time_week_ms = time_week_ms;
    state.status = AP_GPS_FixType::FIX_3D;
    state.num_sats = 15;

    state.location = Location{
        int32_t(latitude*1e7),
        int32_t(longitude*1e7),
        int32_t(altitude*100),
        Location::AltFrame::ABSOLUTE
    };

    state.hdop = 100;
    state.vdop = 100;

    state.have_vertical_velocity = true;
    state.velocity.x = speedN;
    state.velocity.y = speedE;
    state.velocity.z = speedD;

    velocity_to_speed_course(state);

    state.have_speed_accuracy = true;
    state.have_horizontal_accuracy = true;
    state.have_vertical_accuracy = true;
    state.have_vertical_velocity = true;

    // state.horizontal_accuracy = pkt.horizontal_pos_accuracy;
    // state.vertical_accuracy = pkt.vertical_pos_accuracy;
    // state.speed_accuracy = pkt.horizontal_vel_accuracy;

    state.last_gps_time_ms = now;

    return true;
}

#endif  // AP_SIM_GPS_ENABLED
