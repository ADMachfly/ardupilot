/*
 * RATO.h  –  Rocket-Assisted Take-Off manager for ArduPlane
 *
 * Designed for the SR-75 UAV:
 *   Airframe mass : 80 kg
 *   RATO booster  : 10 kg  /  5800 N  /  3 s burn
 *   Target release: 600 m ground-track, 160 m AGL, 180+ m/s
 *
 * Integration points (minimal changes to existing files):
 *   Parameters.h / Parameters.cpp  – declare g2.rato (AP_SUBGROUPINFO)
 *   Plane.h                         – add  RATO rato;
 *   takeoff.cpp  verify_takeoff()   – call  rato.update() and check
 *                                           rato.is_complete()
 *   commands_logic.cpp do_takeoff() – call  rato.init()
 *   SIM_Plane.cpp  update()         – call  rato_physics.update(accel_body, mass, dt)
 *                                     (see RATOPhysics helper at bottom)
 */

#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Common/Location.h>

class RATOController {
public:
    static const struct AP_Param::GroupInfo var_info[];

    enum class ReleaseMode : uint8_t {
        ENVELOPE = 0,
        TIME = 1,
        HYBRID = 2
    };

//    RATO state machine
//    DISABLED        : RATO not active
    /*
      READY           : TAKEOFF command started and RATO is ready
      IGNITION        : ignition command phase
      BOOST           : booster burn phase
      BURNOUT         : booster burn time completed
      ENGINE_TAKEOVER : main engine / normal takeoff continues
      EJECT           : release/ejection condition satisfied
      COMPLETE        : RATO takeoff complete; mission can advance
      ABORT           : timeout/failure condition──────────────────────────────────────────────────────────────────────────────   
    */

        enum class State : uint8_t {
        DISABLED = 0,
        READY,
        IGNITION,
        BOOST,
        BURNOUT,
        ENGINE_TAKEOVER,
        EJECT,
        COMPLETE,
        ABORT
        };

    RATOController();

    // Reset controller back to disabled state.
    void reset();
    void init(const Location& launch_loc, float launch_alt_m);
    bool update(float dist_m, float alt_gain_m, float speed_mps);

    bool is_active() const;
    bool is_complete() const;
    bool is_aborted() const;
    bool is_boosting() const; // ← new, sits next to is_active(), is_complete(), is_aborted()
    bool is_post_boost_handoff() const
    {
        return state == State::BURNOUT || state == State::ENGINE_TAKEOVER;
    }
    bool ignition_commanded() const { return state == State::IGNITION || state == State::BOOST; }
    bool ejection_commanded() const { return state == State::EJECT; }
    float get_burn_elapsed_s() const { return burn_elapsed_s(); }

    State get_state() const { return state; }
    const char *state_name() const;
    
    // ── Parameters exposed to Mission Planner / MAVProxy ─────────────────────

    AP_Int8     enable;             // RATO_ENABLE   0=off 1=on
    AP_Float    thrust_n;           // RATO_THR_N    booster thrust [N]
    AP_Float    mass_kg;            // RATO_MASS_KG  booster mass [kg]
    AP_Float    burn_time;          // RATO_BURN_S   burn duration [s]
    AP_Float    pitch_target_deg;   // RATO_PITCH    launch pitch [deg]
    AP_Float    rel_alt;            // RATO_REL_ALT  release altitude AGL [m]
    AP_Float    rel_dist;           // RATO_REL_DIST release ground distance [m]
    AP_Float    rel_spd;            // RATO_REL_SPD  release min speed [m/s]
    AP_Float    eject_time_s;       // RATO_EJECT_S  time after launch before release [s]
    AP_Int8     release_mode;       // RATO_REL_MODE release logic selection
    AP_Float    max_g;              // RATO_MAX_G    max allowed G-load
    AP_Float    min_g;              // RATO_MIN_G    min expected G during boost (abort if below)
    AP_Float    timeout_s;          // RATO_TIMEOUT  overall RATO phase timeout [s]
    AP_Int8     ign_chan;           // RATO_IGN_CH   relay/servo channel for ignition (1-based)
    AP_Int8     eject_chan;         // RATO_EJECT_CH relay/servo channel for ejection (1-based)

    // ── Public API ────────────────────────────────────────────────────────────

private:
    State state;
    uint32_t start_ms;
    uint32_t burn_start_ms;

    // Stored launch reference values.
    Location launch_location;
    float launch_alt_m;

    // Last measured values passed from Plane::verify_takeoff().
    float last_dist_m;
    float last_alt_gain_m;
    float last_speed_mps;

    // Seconds since RATO initialisation.
    float elapsed_s() const; 
    float burn_elapsed_s() const;
    
    // Returns true when altitude + distance + speed release envelope is satisfied.
    bool release_envelope_met() const;
    bool release_time_met() const;
    bool release_condition_met() const;

    // Commands ignition servo/relay channel on or off.
    void set_ignition_output(bool on);    // ← ADD THIS LINE
};
