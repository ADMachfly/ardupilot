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

#include <AP_Param/AP_Param.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_Mission/AP_Mission.h>
#include <GCS_MAVLink/GCS.h>

// ──────────────────────────────────────────────────────────────────────────────
// Forward declarations
// ──────────────────────────────────────────────────────────────────────────────
class Plane;

// ──────────────────────────────────────────────────────────────────────────────
// RATO state machine
// ──────────────────────────────────────────────────────────────────────────────
enum class RATOState : uint8_t {
    DISABLED       = 0,   // RATO_ENABLE == 0
    READY          = 1,   // armed, waiting for TAKEOFF command
    IGNITION       = 2,   // relay/servo fired, timer started
    BOOST          = 3,   // booster burning; pitch/heading locked
    BURNOUT        = 4,   // burn time elapsed; engine taking over
    ENGINE_TAKEOVER= 5,   // main engine ramping up; still checking envelope
    EJECT          = 6,   // eject command sent
    COMPLETE       = 7,   // TAKEOFF may be marked complete
    ABORT          = 8,   // something wrong; fall back to normal takeoff
};

// ──────────────────────────────────────────────────────────────────────────────
// Main RATO manager class  (registered as a sub-group under ParametersG2)
// ──────────────────────────────────────────────────────────────────────────────
class RATO {
public:
    RATO();

    static const struct AP_Param::GroupInfo var_info[];

    // ── Parameters exposed to Mission Planner / MAVProxy ─────────────────────
    AP_Int8   enable;          // RATO_ENABLE   0=off 1=on
    AP_Float  thrust_n;        // RATO_THR_N    booster thrust [N]
    AP_Float  mass_kg;         // RATO_MASS_KG  booster mass [kg]
    AP_Float  burn_s;          // RATO_BURN_S   burn duration [s]
    AP_Float  pitch_deg;       // RATO_PITCH    launch pitch [deg]
    AP_Float  rel_alt;       // RATO_REL_ALT  release altitude AGL [m]
    AP_Float  rel_dist;      // RATO_REL_DIST release ground distance [m]
    AP_Float  rel_spd;      // RATO_REL_SPD  release min speed [m/s]
    AP_Float  max_g;           // RATO_MAX_G    max allowed G-load
    AP_Float  min_g;           // RATO_MIN_G    min expected G during boost (abort if below)
    AP_Float  timeout;       // RATO_TIMEOUT  overall RATO phase timeout [s]
    AP_Int8   ign_chan;        // RATO_IGN_CH   relay/servo channel for ignition (1-based)
    AP_Int8   ej_chan;      // RATO_EJECT_CH relay/servo channel for ejection (1-based)

    // ── Public API ────────────────────────────────────────────────────────────

    // Call from do_takeoff() when a TAKEOFF mission command starts
    void init(const Location &launch_loc, float launch_alt_m);

    // Call every cycle from verify_takeoff() (or AUTO update).
    // Returns true when the TAKEOFF mission item should be marked complete.
    bool update();

    // True while the booster is actively adding thrust (used by SITL physics)
    bool boost_active() const { return _state == RATOState::BOOST; }

    // True while the booster is still physically attached (affects mass)
    bool booster_attached() const {
        return _state >= RATOState::IGNITION &&
               _state <= RATOState::ENGINE_TAKEOVER;
    }

    // True once autopilot should hand off to normal navigation
    bool is_complete() const { return _state == RATOState::COMPLETE; }

    bool is_aborted()  const { return _state == RATOState::ABORT; }

    RATOState state() const { return _state; }

    // Current filtered G-load (useful for logging / health monitoring)
    float g_load_filtered() const { return _g_filt; }

    // Seconds elapsed since ignition
    float time_since_ignition_s() const;

    // Distance from launch location (ground-track, metres)
    float distance_from_launch_m() const;

    // Altitude gained since ignition (metres AGL)
    float alt_gain_m() const;

    // ── Logging ───────────────────────────────────────────────────────────────
    void write_log();

private:
    // ── Internal helpers ─────────────────────────────────────────────────────
    bool _pre_flight_checks() const;
    void _fire_ignition();
    void _fire_eject();
    void _update_measurements();
    bool _release_envelope_reached() const;
    bool _timeout_exceeded() const;
    const char *_state_name(RATOState s) const;

    // ── Runtime state (not persisted) ────────────────────────────────────────
    RATOState _state       { RATOState::DISABLED };
    uint32_t  _ignition_ms { 0 };
    Location  _launch_loc;
    float     _launch_alt_m{ 0.0f };

    // Measured values (updated each cycle)
    float _dist_m          { 0.0f };
    float _alt_gain_m      { 0.0f };
    float _spd_mps         { 0.0f };
    float _g_raw           { 0.0f };
    float _g_filt          { 0.0f };

    // Guard against repeated ignition/eject commands
    bool  _ign_sent        { false };
    bool  _eject_sent      { false };
};

// ──────────────────────────────────────────────────────────────────────────────
// RATOPhysics  –  SITL-only helper injected into SIM_Plane
//
// Usage in libraries/SITL/SIM_Plane.cpp  update():
//
//   #include "../../ArduPlane/RATO.h"   // or via forward include path
//   static RATOPhysics rato_phys;
//
//   // On ignition signal from autopilot (e.g. via SIM parameter or relay):
//   rato_phys.ignite(plane.g2.rato.thrust_n,
//                    plane.g2.rato.mass_kg,
//                    plane.g2.rato.burn_s,
//                    plane.g2.rato.pitch_deg);
//
//   // Each physics step:
//   rato_phys.update(accel_body, mass, delta_time);
//
//   // When autopilot sets eject flag:
//   rato_phys.eject(mass);
// ──────────────────────────────────────────────────────────────────────────────
class RATOPhysics {
public:
    RATOPhysics() = default;

    // Call once when ignition signal is received
    void ignite(float thrust_n, float booster_mass_kg,
                float burn_s,   float pitch_deg);

    // Call every SITL physics timestep.
    // Adds booster acceleration to accel_body (body-frame, NED convention,
    // same coordinate system as SIM_Aircraft::accel_body).
    // 'total_mass_kg' is passed by reference so this function can account
    // for the booster mass while it is attached.
    void update(Vector3f &accel_body, float &total_mass_kg, float dt);

    // Call when ejection signal arrives: removes booster mass
    void eject(float &total_mass_kg);
    float attached_mass_kg() const;

    bool is_active()   const { return _active; }
    bool is_attached() const { return _attached; }
    float burn_remaining_s() const;

private:
    bool    _active    { false };
    bool    _attached  { false };
    float   _thrust_n  { 0.0f };
    float   _mass_kg   { 0.0f };
    float   _burn_s    { 0.0f };
    float   _pitch_rad { 0.0f };
    float   _elapsed_s { 0.0f };
};
