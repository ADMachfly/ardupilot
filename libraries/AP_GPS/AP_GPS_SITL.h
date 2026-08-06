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

#pragma once

#include "GPS_Backend.h"

#include <SITL/SITL.h>

#if AP_SIM_GPS_ENABLED

class AP_GPS_SITL : public AP_GPS_Backend
{

public:

    using AP_GPS_Backend::AP_GPS_Backend;

    bool        read() override;

    const char *name() const override { return "SITL"; }

private:

    friend class AP_GPS_SITL_Test;

    uint32_t last_update_ms;

    // HIL-F24-R2D: pure decision logic, unconditionally compiled (unlike
    // the `#if CONFIG_HAL_BOARD != HAL_BOARD_SITL` branch in read() that
    // calls it on real hardware only) so it is directly unit-testable on
    // desktop SITL -- same pattern as SITL::JSON::recv_fdm_bounded()
    // (see libraries/SITL/tests/test_sim_json.cpp). Returns true only if
    // a JSON position has ever been marked valid AND it is not older
    // than stale_ms.
    static bool json_position_is_valid(
        bool position_valid, uint32_t last_position_update_ms, uint32_t now_ms, uint32_t stale_ms);
};

#endif  // AP_SIM_GPS_ENABLED
