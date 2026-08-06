/*
    HIL-F24-R2D regression tests for AP_GPS_SITL's real-hardware-only
    SIM_JSON fix-gating logic.

    HIL_F24_R2B_ahrs_backend_diagnosis.md found that on the SR-75
    SimOnHardware bench, EKF3's GPS_GLOBAL_ORIGIN was permanently latched
    to ArduPilot's compiled-in default SITL start location (CMAC,
    Canberra -- SIM_OPOS_LAT/LNG) instead of the real bench truth
    position. Root cause, traced in this task: SITL::Aircraft::
    update_home() can latch home_is_set true (from that default) before
    SITL::JSON::recv_fdm() has ever parsed a real latitude/longitude/
    altitude packet, and AP_GPS_SITL::read() (AP_GPS_SITL.cpp)
    unconditionally published a FIX_3D built from whatever position that
    left in sitl->state -- including the poisoned default.

    The fix adds two new fields to SITL::SIM (json_position_valid,
    json_position_last_update_ms -- set only by SITL::JSON::recv_fdm(),
    see libraries/SITL/tests/test_sim_json.cpp for that side) and a
    small, pure, unconditionally-compiled decision function,
    AP_GPS_SITL::json_position_is_valid(), that read()'s `#if
    CONFIG_HAL_BOARD != HAL_BOARD_SITL` branch (real hardware /
    SimOnHardware boards only) uses to withhold or withdraw the fix.
    That branch itself cannot be exercised from this desktop-SITL-only
    gtest binary (same constraint HIL-F24-N's recv_fdm_bounded() tests
    already document/work around) -- these tests instead exercise the
    pure decision function directly, the same pattern used there.
*/
#include <AP_gtest.h>

#include <AP_GPS/AP_GPS_SITL.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

#if AP_SIM_GPS_ENABLED

// Minimal friend-only accessor (mirrors AP_GPS_SITL.h's `friend class
// AP_GPS_SITL_Test;`), giving these tests access to the private static
// decision function without widening AP_GPS_SITL's public API.
class AP_GPS_SITL_Test
{
public:
    static bool json_position_is_valid(bool position_valid, uint32_t last_update_ms, uint32_t now_ms, uint32_t stale_ms)
    {
        return AP_GPS_SITL::json_position_is_valid(position_valid, last_update_ms, now_ms, stale_ms);
    }
};

// "Boot before first JSON packet: no 3D fix" -- position_valid is still
// false (SITL::JSON::recv_fdm() has never parsed a lat/lon/alt packet).
TEST(AP_GPS_SITL, NeverValidBeforeFirstPacketIsNotTrustworthy)
{
    EXPECT_FALSE(AP_GPS_SITL_Test::json_position_is_valid(false, 0, 0, 2000));
    EXPECT_FALSE(AP_GPS_SITL_Test::json_position_is_valid(false, 0, 60000, 2000));
    // Even a last_update_ms value present (e.g. left over from a stale
    // struct) must not matter while position_valid itself is false.
    EXPECT_FALSE(AP_GPS_SITL_Test::json_position_is_valid(false, 59999, 60000, 2000));
}

// "First valid position packet: 3D fix at supplied truth" -- once
// position_valid is true and the update is fresh (age 0), the position
// is trustworthy; read()'s existing (unmodified) code then reports
// FIX_3D built directly from sitl->state, which SITL::JSON::recv_fdm()
// has, by this point, already populated from the real parsed packet
// (see test_sim_json.cpp's JsonPacketWithPositionMarksJsonPositionValid).
TEST(AP_GPS_SITL, FreshValidPositionIsTrustworthy)
{
    EXPECT_TRUE(AP_GPS_SITL_Test::json_position_is_valid(true, 1000, 1000, 2000));
    EXPECT_TRUE(AP_GPS_SITL_Test::json_position_is_valid(true, 1000, 1500, 2000));
    // Exactly at the stale boundary is still trustworthy (<=, not <).
    EXPECT_TRUE(AP_GPS_SITL_Test::json_position_is_valid(true, 1000, 3000, 2000));
}

// "Stale feed: fix withdrawn" -- position_valid stays true (a packet
// *was* received at some point) but too much time has passed since.
TEST(AP_GPS_SITL, StalePositionIsWithdrawn)
{
    EXPECT_FALSE(AP_GPS_SITL_Test::json_position_is_valid(true, 1000, 3001, 2000));
    EXPECT_FALSE(AP_GPS_SITL_Test::json_position_is_valid(true, 0, 1000000, 2000));
}

// millis() wraps every ~49.7 days; unsigned subtraction handles this
// correctly as long as the true elapsed time is within range, which it
// always is here -- confirms the wrap case isn't accidentally treated
// as "infinitely stale".
TEST(AP_GPS_SITL, HandlesMillisWraparoundCorrectly)
{
    const uint32_t last_update = 0xFFFFFFF0u;  // 16ms before wraparound
    const uint32_t now_after_wrap = 10u;       // 26ms of wall-clock time after wraparound
    EXPECT_TRUE(AP_GPS_SITL_Test::json_position_is_valid(true, last_update, now_after_wrap, 2000));
}

#endif  // AP_SIM_GPS_ENABLED

AP_GTEST_MAIN()
