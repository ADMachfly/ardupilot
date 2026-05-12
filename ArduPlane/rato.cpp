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

#include "RATO.h"
#include "Plane.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Logger/AP_Logger.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_Arming/AP_Arming.h>

extern const AP_HAL::HAL &hal;

// ══════════════════════════════════════════════════════════════════════════════
//  Parameter table
//  NOTE: indices must not collide with other ParametersG2 sub-groups.
//  Safe to start at index 50 if the project has not used that block yet;
//  adjust if needed.
// ══════════════════════════════════════════════════════════════════════════════

const AP_Param::GroupInfo RATO::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: RATO enable
    // @Description: Enables RATO assisted takeoff logic
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("ENABLE", 1, RATO, enable, 0),

    // @Param: THR_N
    // @DisplayName: RATO thrust
    // @Description: RATO booster thrust in Newtons for SITL and logic reference
    // @Units: N
    // @User: Advanced
    AP_GROUPINFO("THR_N", 2, RATO, thrust_n, 5800),

    // @Param: MASS_KG
    // @DisplayName: RATO mass
    // @Description: RATO booster mass in kilograms
    // @Units: kg
    // @User: Advanced
    AP_GROUPINFO("MASS_KG", 3, RATO, mass_kg, 10),

    // @Param: BURN_S
    // @DisplayName: RATO burn time
    // @Description: RATO booster burn duration
    // @Units: s
    // @User: Advanced
    AP_GROUPINFO("BURN_S", 4, RATO, burn_s, 3.0f),

    // @Param: REL_ALT
    // @DisplayName: RATO release altitude
    // @Description: Altitude gain required before RATO release
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("REL_ALT", 5, RATO, rel_alt, 160),

    // @Param: REL_DIST
    // @DisplayName: RATO release distance
    // @Description: Distance from launch required before RATO release
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("REL_DIST", 6, RATO, rel_dist, 600),

    // @Param: REL_SPD
    // @DisplayName: RATO release speed
    // @Description: Speed required before RATO release
    // @Units: m/s
    // @User: Advanced
    AP_GROUPINFO("REL_SPD", 7, RATO, rel_spd, 180),

    // @Param: MAX_G
    // @DisplayName: RATO maximum G
    // @Description: Maximum allowed acceleration during RATO boost
    // @User: Advanced
    AP_GROUPINFO("MAX_G", 8, RATO, max_g, 7.0f),

    // @Param: PITCH
    // @DisplayName: RATO pitch angle
    // @Description: Pitch angle command during RATO boost
    // @Units: deg
    // @User: Advanced
    AP_GROUPINFO("PITCH", 9, RATO, pitch_deg, 20),

    AP_GROUPINFO("MIN_G", 10, RATO, min_g, 4.0f),
    AP_GROUPINFO("TIMEOUT", 11, RATO, timeout, 8.0f),
    AP_GROUPINFO("IGN_CH", 12, RATO, ign_chan, 0),
    AP_GROUPINFO("EJECT_CH", 13, RATO, ej_chan, 0),
    AP_GROUPEND
};

RATO::RATO()
{
    AP_Param::setup_object_defaults(this, var_info);
}

// ══════════════════════════════════════════════════════════════════════════════
//  Public API
// ══════════════════════════════════════════════════════════════════════════════

// ---------------------------------------------------------------------------
// init() – called from do_takeoff() when a TAKEOFF command is loaded
// ---------------------------------------------------------------------------
void RATO::init(const Location &launch_loc, float launch_alt_m)
{
    if (!enable) {
        _state = RATOState::DISABLED;
        return;
    }

    _launch_loc   = launch_loc;
    _launch_alt_m = launch_alt_m;
    _ignition_ms  = 0;
    _ign_sent     = false;
    _eject_sent   = false;
    _g_filt       = 1.0f;
    _g_raw        = 1.0f;
    _dist_m       = 0.0f;
    _alt_gain_m   = 0.0f;
    _spd_mps      = 0.0f;

    _state = RATOState::READY;
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "RATO: init complete – READY");
}

// ---------------------------------------------------------------------------
// update() – called every cycle from verify_takeoff()
// Returns true when the TAKEOFF mission item should be marked complete.
// ---------------------------------------------------------------------------
bool RATO::update()
{
    if (_state == RATOState::DISABLED ||
        _state == RATOState::COMPLETE ||
        _state == RATOState::ABORT) {
        return is_complete();
    }

    _update_measurements();

    switch (_state) {

    // ── READY ────────────────────────────────────────────────────────────────
    case RATOState::READY:
        if (_pre_flight_checks()) {
            _state = RATOState::IGNITION;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "RATO: pre-flight OK – IGNITION");
        }
        break;

    // ── IGNITION ─────────────────────────────────────────────────────────────
    case RATOState::IGNITION:
        if (!_ign_sent) {
            _fire_ignition();
        }
        // Transition immediately to BOOST; the physics model is now active
        _state = RATOState::BOOST;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "RATO: IGNITION fired – BOOST");
        break;

    // ── BOOST ────────────────────────────────────────────────────────────────
    case RATOState::BOOST: {
       // Abort : helps detect failed ignition 
        if (time_since_ignition_s() > 0.2f && _g_filt < min_g) {
             GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,
                  "RATO: ABORT - low boost G %.1f", (double)_g_filt);
            _state = RATOState::ABORT;
            break;
        }
        // Abort: G too high (structural overload)
        if (_g_filt > max_g) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,
                          "RATO: ABORT – G overload %.1f", (double)_g_filt);
            _state = RATOState::ABORT;
            break;
        }
        // Abort: timeout
        if (_timeout_exceeded()) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "RATO: ABORT – timeout");
            _state = RATOState::ABORT;
            break;
        }
        // Burnout check
        if (time_since_ignition_s() >= burn_s) {
            _state = RATOState::BURNOUT;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                          "RATO: BURNOUT at t=%.2f s  spd=%.1f m/s  alt=%.0f m  dist=%.0f m",
                          (double)time_since_ignition_s(),
                          (double)_spd_mps,
                          (double)_alt_gain_m,
                          (double)_dist_m);
        }
        break;
    }

    // ── BURNOUT ──────────────────────────────────────────────────────────────
    case RATOState::BURNOUT:
        // Physics model turns off thrust automatically; let main engine spool
        _state = RATOState::ENGINE_TAKEOVER;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "RATO: ENGINE_TAKEOVER");
        break;

    // ── ENGINE_TAKEOVER ──────────────────────────────────────────────────────
    case RATOState::ENGINE_TAKEOVER:
        if (_timeout_exceeded()) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                          "RATO: timeout during engine takeover – forcing eject");
            _state = RATOState::EJECT;
            break;
        }
        if (_release_envelope_reached()) {
            _state = RATOState::EJECT;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                          "RATO: release envelope reached – EJECT  dist=%.0f m  alt=%.0f m  spd=%.1f m/s",
                          (double)_dist_m, (double)_alt_gain_m, (double)_spd_mps);
        }
        break;

    // ── EJECT ────────────────────────────────────────────────────────────────
    case RATOState::EJECT:
        if (!_eject_sent) {
            _fire_eject();
        }
        _state = RATOState::COMPLETE;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "RATO: COMPLETE – handing off to mission");
        break;

    default:
        break;
    }

    return is_complete();
}

// ══════════════════════════════════════════════════════════════════════════════
//  Measurement helpers
// ══════════════════════════════════════════════════════════════════════════════

float RATO::time_since_ignition_s() const
{
    if (_ignition_ms == 0) { return 0.0f; }
    return (AP_HAL::millis() - _ignition_ms) * 0.001f;
}

float RATO::distance_from_launch_m() const { return _dist_m; }
float RATO::alt_gain_m()             const { return _alt_gain_m; }

// ══════════════════════════════════════════════════════════════════════════════
//  Logging
// ══════════════════════════════════════════════════════════════════════════════
void RATO::write_log()
{
#if HAL_LOGGING_ENABLED
    // @LoggerMessage: RATO
    // @Description: RATO takeoff state and measured values
    // @Field: TimeUS: Time since system startup
    // @Field: State: RATO state machine value (0-8)
    // @Field: T: Seconds since ignition
    // @Field: Dist: Ground distance from launch (m)
    // @Field: Alt: Altitude gain since ignition (m)
    // @Field: Spd: Groundspeed (m/s)
    // @Field: G: Filtered G-load
    AP::logger().WriteStreaming(
        "RATO", "TimeUS,State,T,Dist,Alt,Spd,G",
        "QBfffff",
        AP_HAL::micros64(),
        (uint8_t)_state,
        time_since_ignition_s(),
        _dist_m,
        _alt_gain_m,
        _spd_mps,
        _g_filt);
#endif
}

// ══════════════════════════════════════════════════════════════════════════════
//  Private helpers
// ══════════════════════════════════════════════════════════════════════════════

void RATO::_update_measurements()
{
    AP_AHRS &ahrs = AP::ahrs();
    AP_InertialSensor &ins = AP::ins();

    // --- ground-track distance from launch --------------------------------
    Location cur_loc;
    if (ahrs.get_location(cur_loc)) {
        _dist_m = cur_loc.get_distance(_launch_loc);
    }

    // --- altitude gain (EKF/baro relative altitude) -----------------------
    float alt_rel_home = 0.0f;
    ahrs.get_relative_position_D_home(alt_rel_home);  // negative = above home
    float cur_alt_m = -alt_rel_home;
    _alt_gain_m = cur_alt_m - _launch_alt_m;
    if (_alt_gain_m < 0.0f) { _alt_gain_m = 0.0f; }

    // --- speed ------------------------------------------------------------
    _spd_mps = ahrs.groundspeed();

    // --- G-load (filtered IIR, τ ≈ 5 cycles @ 400 Hz ≈ 12 ms) -----------
    const Vector3f accel = ins.get_accel();
    _g_raw  = accel.length() / GRAVITY_MSS;
    _g_filt = 0.85f * _g_filt + 0.15f * _g_raw;
}

bool RATO::_pre_flight_checks() const
{
    AP_AHRS &ahrs = AP::ahrs();

    if (!AP::arming().is_armed()) {
        return false;  // wait for arm
    }
    if (!ahrs.healthy()) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "RATO: waiting – AHRS unhealthy");
        return false;
    }
    if (!ahrs.have_inertial_nav()) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "RATO: waiting – no inertial nav");
        return false;
    }
    return true;
}

void RATO::_fire_ignition()
{
    _ign_sent    = true;
    _ignition_ms = AP_HAL::millis();

    if (ign_chan > 0) {
        // Set channel HIGH (2000 µs) to trigger ignition relay/pyro
        SRV_Channels::set_output_pwm_chan_timeout(
            (uint8_t)(ign_chan - 1), 2000, 500);  // 500 ms pulse
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "RATO: ignition command sent (ch %d)", (int)ign_chan.get());
}

void RATO::_fire_eject()
{
    _eject_sent = true;

    if (ej_chan > 0) {
        SRV_Channels::set_output_pwm_chan_timeout(
            (uint8_t)(ej_chan - 1), 2000, 500);
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "RATO: eject command sent (ch %d)", (int)ej_chan.get());
}

bool RATO::_release_envelope_reached() const
{
    return (_dist_m    >= rel_dist) &&
           (_alt_gain_m >= rel_alt)  &&
           (_spd_mps   >= rel_spd);
}

bool RATO::_timeout_exceeded() const
{
    if (_ignition_ms == 0) { return false; }
    return time_since_ignition_s() > timeout;
}

const char *RATO::_state_name(RATOState s) const
{
    switch (s) {
    case RATOState::DISABLED:        return "DISABLED";
    case RATOState::READY:           return "READY";
    case RATOState::IGNITION:        return "IGNITION";
    case RATOState::BOOST:           return "BOOST";
    case RATOState::BURNOUT:         return "BURNOUT";
    case RATOState::ENGINE_TAKEOVER: return "ENGINE_TAKEOVER";
    case RATOState::EJECT:           return "EJECT";
    case RATOState::COMPLETE:        return "COMPLETE";
    case RATOState::ABORT:           return "ABORT";
    default:                         return "UNKNOWN";
    }
}

// ══════════════════════════════════════════════════════════════════════════════
//  RATOPhysics  –  SITL-only booster force model
// ══════════════════════════════════════════════════════════════════════════════

void RATOPhysics::ignite(float thrust_n, float booster_mass_kg,
                          float burn_s,   float pitch_deg)
{
    _thrust_n  = thrust_n;
    _mass_kg   = booster_mass_kg;
    _burn_s    = burn_s;
    _pitch_rad = radians(pitch_deg);
    _elapsed_s = 0.0f;
    _active    = true;
    _attached  = true;
}

// ---------------------------------------------------------------------------
// update() – inject booster acceleration into SITL body_accel
//
// Coordinate convention (same as SIM_Aircraft):
//   Body X = forward
//   Body Z = up (NED: negative = up, but SIM_Aircraft uses +Z = up in body)
//
// accel_body is the raw body-frame acceleration BEFORE gravity is added
// by SIM_Aircraft::update().  We just push the booster thrust into it.
// ---------------------------------------------------------------------------
void RATOPhysics::update(Vector3f &accel_body, float &total_mass_kg, float dt)
{
    if (!_active || !_attached) { return; }

    // Accumulate total mass while booster is attached
    // (caller should initialise total_mass_kg to the bare airframe value)
    total_mass_kg += _mass_kg;

    if (_elapsed_s <= _burn_s) {
        // Decompose thrust into body axes:
        //   forward (X) component
        //   upward  (Z) component  – SIM_Aircraft uses +Z up in body frame
        float fx = _thrust_n * cosf(_pitch_rad);
        float fz = _thrust_n * sinf(_pitch_rad);

        // Acceleration = Force / total_mass
        accel_body.x += fx / total_mass_kg;
        accel_body.z += fz / total_mass_kg;

        _elapsed_s += dt;
    } else {
        // Burn complete; keep attached until eject() is called
        _active = false;
    }
}

void RATOPhysics::eject(float &total_mass_kg)
{
    if (_attached) {
        total_mass_kg -= _mass_kg;
        _attached = false;
        _active = false;
    }
}

float RATOPhysics::attached_mass_kg() const
{
    return _attached ? _mass_kg : 0.0f;
}
