/*
 * RATO.cpp  –  Rocket-Assisted Take-Off manager for ArduPlane
 *
 * SR-75 reference numbers
 *   Airframe  : 80 kg
 *   Booster   : 10 kg, 5800 N, 3 s burn
 *   Target release envelope : 600 m ground-track / 160 m AGL / 180 m/s
 *   Expected boost acceleration : ~6.5 g (64 m/s²) before aero losses
 *
 * This file is self-contained.  The only external call-sites are:
 *   commands_logic.cpp :: do_takeoff()     → rato.init()
 *   takeoff.cpp        :: verify_takeoff() → rato.update() / rato.is_complete()
 *   Plane.cpp          :: (scheduler)      → rato.write_log()   [optional]
 *   SIM_Plane.cpp      :: update()         → RATOPhysics helpers
 */

#include "rato.h"

#include <AP_HAL/AP_HAL.h>

// ══════════════════════════════════════════════════════════════════════════════
//  Parameter table
//  NOTE: indices must not collide with other ParametersG2 sub-groups.
//  Safe to start at index 50 if the project has not used that block yet;
//  adjust if needed.
// ══════════════════════════════════════════════════════════════════════════════

const AP_Param::GroupInfo RATOController::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: RATO enable
    // @Description: Enables RATO assisted takeoff logic
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("ENABLE", 1, RATOController, enable, 0),

    // @Param: THR_N
    // @DisplayName: RATO thrust
    // @Description: RATO booster thrust in Newtons for SITL and logic reference
    // @Units: N
    // @User: Advanced
    AP_GROUPINFO("THR_N",  2, RATOController, thrust_n, 5800),

    // @Param: MASS_KG
    // @DisplayName: RATO mass
    // @Description: RATO booster mass in kilograms
    // @Units: kg
    // @User: Advanced    
    AP_GROUPINFO("MASS",   3, RATOController, mass_kg, 10),

    // @Param: BURN_S
    // @DisplayName: RATO burn time
    // @Description: RATO booster burn duration
    // @Units: s
    // @User: Advanced    
    AP_GROUPINFO("BURN_S", 4, RATOController, burn_time, 3.0f),

    // @Param: PITCH
    // @DisplayName: RATO pitch angle
    // @Description: Pitch angle command during RATO boost
    // @Units: deg
    // @User: Advanced  
    AP_GROUPINFO("PITCH",  5, RATOController, pitch_target_deg, 20.0f),

    // @Param: REL_ALT
    // @DisplayName: RATO release altitude
    // @Description: Altitude gain required before RATO release
    // @Units: m
    // @User: Advanced  
    AP_GROUPINFO("REL_ALT",  6, RATOController, rel_alt, 160),

    // @Param: REL_DIST
    // @DisplayName: RATO release distance
    // @Description: Distance from launch required before RATO release
    // @Units: m
    // @User: Advanced    
    AP_GROUPINFO("REL_DST",  7, RATOController, rel_dist, 600),

    // @Param: REL_SPD
    // @DisplayName: RATO release speed
    // @Description: Speed required before RATO release
    // @Units: m/s
    // @User: Advanced
    AP_GROUPINFO("REL_SPD",  8, RATOController, rel_spd, 180),

    // @Param: MAX_G
    // @DisplayName: RATO maximum G
    // @Description: Maximum allowed acceleration during RATO boost
    // @User: Advanced    
    AP_GROUPINFO("MAX_G",  9, RATOController, max_g, 7.0f),

    // @Param: MIN_G
    // @DisplayName: RATO minimum G
    // @Description: Minimum expected acceleration after RATO ignition. Used to detect weak/failed ignition or insufficient booster acceleration.
    // @User: Advanced
    AP_GROUPINFO("MIN_G", 10, RATOController, min_g, 2.0f),

    // @Param: TMO_S
    // @DisplayName: RATO timeout
    // @Description: Maximum allowed time in RATO takeoff logic before timeout handling. If the release envelope is not reached within this time, RATO logic can abort or fall back.
    // @Units: s
    // @User: Advanced    
    AP_GROUPINFO("TMO_S", 11, RATOController, timeout_s, 8.0f),

    // @Param: IGN_CH
    // @DisplayName: RATO ignition channel
    // @Description: Servo or relay channel used to command RATO ignition. Set to 0 to disable output command during testing.
    // @User: Advanced    
    AP_GROUPINFO("IGN_CH", 12, RATOController, ign_chan, 0),

    // @Param: EJ_CH
    // @DisplayName: RATO eject channel
    // @Description: Servo or relay channel used to command RATO booster ejection after burn/release conditions are satisfied. Set to 0 to disable output command during testing.
    // @User: Advanced    
    AP_GROUPINFO("EJ_CH",  13, RATOController, eject_chan, 0),
    AP_GROUPEND
};

RATOController::RATOController() :
    state(State::DISABLED),
    start_ms(0)
{
    AP_Param::setup_object_defaults(this, var_info);

}

void RATOController::reset()
{
    state = State::DISABLED;
    start_ms = 0;
}

void RATOController::init()
{
    if (enable.get() <= 0) {
        reset();
        return;
    }

    start_ms = AP_HAL::millis();
    state = State::READY;
}

bool RATOController::update()
{
    if (enable.get() <= 0) {
        reset();
        return true;
    }

    if (timeout_s.get() > 0.0f && elapsed_s() > timeout_s.get()) {
        state = State::ABORT;
        return true;
    }

    switch (state) {
    case State::DISABLED:
        return true;

    case State::READY:
        state = State::IGNITION;
        return false;

    case State::IGNITION:
        state = State::BOOST;
        return false;

    case State::BOOST:
        if (elapsed_s() >= burn_time.get()) {
            state = State::BURNOUT;
        }
        return false;

    case State::BURNOUT:
        state = State::ENGINE_TAKEOVER;
        return false;

    case State::ENGINE_TAKEOVER:
        state = State::EJECT;
        return false;

    case State::EJECT:
        state = State::COMPLETE;
        return true;

    case State::COMPLETE:
    case State::ABORT:
    default:
        return true;
    }
}

bool RATOController::is_active() const
{
    return state != State::DISABLED &&
           state != State::COMPLETE &&
           state != State::ABORT;
}

bool RATOController::is_complete() const
{
    return state == State::COMPLETE;
}

bool RATOController::is_aborted() const
{
    return state == State::ABORT;
}

float RATOController::elapsed_s() const
{
    if (start_ms == 0) {
        return 0.0f;
    }

    return (AP_HAL::millis() - start_ms) * 0.001f;
}

const char *RATOController::state_name() const
{
    switch (state) {
    case State::DISABLED:        return "DISABLED";
    case State::READY:           return "READY";
    case State::IGNITION:        return "IGNITION";
    case State::BOOST:           return "BOOST";
    case State::BURNOUT:         return "BURNOUT";
    case State::ENGINE_TAKEOVER: return "ENGINE_TAKEOVER";
    case State::EJECT:           return "EJECT";
    case State::COMPLETE:        return "COMPLETE";
    case State::ABORT:           return "ABORT";
    default:                     return "UNKNOWN";
    }
}