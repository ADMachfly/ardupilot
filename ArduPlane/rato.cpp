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

#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>
#include <SRV_Channel/SRV_Channel.h>
#include <GCS_MAVLink/GCS.h>     // ← ADD THIS for gcs()

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
    AP_GROUPINFO("TMO_S", 11, RATOController, timeout_s, 15.0f),

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

    // @Param: EJECT_S
    // @DisplayName: RATO eject time
    // @Description: Time after RATO launch before booster ejection when time-based release logic is used
    // @Range: 7 15
    // @Units: s
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("EJECT_S", 14, RATOController, eject_time_s, 10.0f),

    // @Param: REL_MODE
    // @DisplayName: RATO release mode
    // @Description: Selects whether RATO booster release uses the altitude distance speed envelope, the eject timer, or both requirements together
    // @Values: 0:Envelope,1:Time,2:Hybrid
    // @User: Advanced
    AP_GROUPINFO("REL_MODE", 15, RATOController, release_mode, 1),

    // @Param: EJ_PULSE
    // @DisplayName: RATO eject pulse duration
    // @Description: Duration of the one-shot RATO booster ejection output pulse after burnout and release conditions are satisfied
    // @Range: 0.1 2.0
    // @Units: s
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("EJ_PULSE", 16, RATOController, eject_pulse_s, 0.5f),
    AP_GROUPEND
};

RATOController::RATOController() :
    state(State::DISABLED),
    start_ms(0),
    launch_alt_m(0.0f),
    last_dist_m(0.0f),
    last_alt_gain_m(0.0f),
    last_speed_mps(0.0f),
    last_roll_deg(0.0f),
    last_pitch_rate_rad_s(0.0f),
    ejection_pulsed(false),
    ejection_pulse_start_ms(0)
{
    AP_Param::setup_object_defaults(this, var_info);

}

void RATOController::reset()
{
    state = State::DISABLED;
    start_ms = 0;
    burn_start_ms = 0;

    // Clear launch reference and measured values.
    launch_alt_m = 0.0f;
    last_dist_m = 0.0f;
    last_alt_gain_m = 0.0f;
    last_speed_mps = 0.0f;
    last_roll_deg = 0.0f;
    last_pitch_rate_rad_s = 0.0f;
    ejection_pulsed = false;
    ejection_pulse_start_ms = 0;
    set_ignition_output(false);   // ← ADD THIS
    set_ejection_output(false);
}

void RATOController::init(const Location& loc, float alt_m)
{
    if (enable.get() <= 0) {
        reset();
        return;
    }

        // Store launch reference.
    launch_location = loc;
    launch_alt_m = alt_m;

    // Start RATO timer.
    start_ms = AP_HAL::millis();
    burn_start_ms = 0;

    // Clear previous measured values.
    last_dist_m = 0.0f;
    last_alt_gain_m = 0.0f;
    last_speed_mps = 0.0f;
    last_roll_deg = 0.0f;
    last_pitch_rate_rad_s = 0.0f;
    ejection_pulsed = false;
    ejection_pulse_start_ms = 0;
    set_ignition_output(false);
    set_ejection_output(false);

    if (timeout_s.get() > 0.0f && timeout_s.get() <= eject_time_s.get()) {
        gcs().send_text(MAV_SEVERITY_WARNING,
                        "RATO timeout %.1f <= eject %.1f",
                        (double)timeout_s.get(),
                        (double)eject_time_s.get());
    }
     
    // Enter active state machine.    
    state = State::READY;
}

bool RATOController::update(float dist_m, float alt_gain_m, float speed_mps, float roll_deg, float pitch_rate_rad_s)
{
    if (enable.get() <= 0) {
        reset();
        return true;
    }

    // Store latest measured values for envelope checking.
    last_dist_m = dist_m;
    last_alt_gain_m = alt_gain_m;
    last_speed_mps = speed_mps;
    last_roll_deg = roll_deg;
    last_pitch_rate_rad_s = pitch_rate_rad_s;

    gcs().send_text(MAV_SEVERITY_INFO,
                "RATO state=%s t=%.2f burn=%.2f d=%.1f alt=%.1f spd=%.1f",
                state_name(),
                (double)elapsed_s(),
                (double)burn_elapsed_s(),
                (double)dist_m,
                (double)alt_gain_m,
                (double)speed_mps);
    
    // Timeout safety: prevents staying stuck in RATO forever.    
    if (timeout_s.get() > 0.0f && elapsed_s() > timeout_s.get()) {

            // ADD THIS BLOCK ↓
        gcs().send_text(MAV_SEVERITY_WARNING,
                        "RATO abort timeout t=%.2f TMO=%.2f",
                        (double)elapsed_s(),
                        (double)timeout_s.get());
                        
        state = State::ABORT;
        set_ignition_output(false);
        set_ejection_output(false);
        return true;
    }

    switch (state) {
    case State::DISABLED:
    // No RATO action. Let normal takeoff continue.
        return true;

case State::READY:
    set_ignition_output(false);
    set_ejection_output(false);

    // Wait until AUTO takeoff has actually started moving.
    // This prevents RATO burn from expiring before arming/launch.
    if (speed_mps < 1.0f && dist_m < 1.0f) {
        return false;
    }

    // Restart RATO timer at actual launch.
    start_ms = AP_HAL::millis();
    state = State::IGNITION;
    return false;

    case State::IGNITION:
    // Later this state will command ignition output channel.    
        set_ignition_output(true);
        set_ejection_output(false);
        burn_start_ms = AP_HAL::millis();
        state = State::BOOST;
        return false;

    case State::BOOST:
        set_ignition_output(true);
        set_ejection_output(false);
    /*
        Stay in BOOST until burn time is complete.

        Physics thrust will be added later in SIM_Plane.cpp.
        For now this is only timing logic.
    */    
        if (burn_elapsed_s() >= burn_time.get()) {
            set_ignition_output(false);
            state = State::BURNOUT;
        }
        return false;

    case State::BURNOUT:
        set_ignition_output(false);
        set_ejection_output(false);
        /*
          Booster burn is complete.

          After this, the main engine / normal takeoff should continue,
          but RATO release should wait until altitude/distance/speed envelope.
        */
        state = State::ENGINE_TAKEOVER;
        return false;

    // case State::ENGINE_TAKEOVER:
        /*
          Wait here until the measured release envelope is achieved:

              distance >= RATO_REL_DST
              altitude >= RATO_REL_ALT
              speed    >= RATO_REL_SPD
        */
        // state = State::EJECT;
        // return false;
        // Edits for the JSBSim test The fix makes it wait until release_envelope_met() returns true before moving to EJECT.

    case State::ENGINE_TAKEOVER:
        set_ignition_output(false);
        set_ejection_output(false);
        if (release_condition_met() && attitude_stable_for_ejection()) {
            state = State::EJECT;
        }
        return false;

    case State::EJECT:
        set_ignition_output(false);
        if (!ejection_pulsed) {
            ejection_pulsed = true;
            ejection_pulse_start_ms = AP_HAL::millis();
            set_ejection_output(true);
            return false;
        }
        if ((AP_HAL::millis() - ejection_pulse_start_ms) * 0.001f < MAX(eject_pulse_s.get(), 0.0f)) {
            set_ejection_output(true);
            return false;
        }
        set_ejection_output(false);
        state = State::COMPLETE;
        return true;

    case State::COMPLETE:
    // RATO is complete; mission TAKEOFF may complete.
    set_ignition_output(false);
    set_ejection_output(false);
    return true;

    case State::ABORT:
    set_ignition_output(false);
    set_ejection_output(false);
    // RATO aborted; caller may fall back to normal takeoff.
    return true;    
    
    default:
        return true;
    }
}

bool RATOController::release_envelope_met() const
{
    /*
      Release/ejection envelope.

      These are measured values passed from Plane::verify_takeoff().
      Do not assume the aircraft reached these values from physics;
      always verify with EKF/GPS/airspeed-derived measurements.
    */
    return last_dist_m >= rel_dist.get() &&
           last_alt_gain_m >= rel_alt.get() &&
           last_speed_mps >= rel_spd.get();
}

bool RATOController::release_time_met() const
{
    return eject_time_s.get() > 0.0f && elapsed_s() >= eject_time_s.get();
}

bool RATOController::release_condition_met() const
{
    switch (ReleaseMode(release_mode.get())) {
    case ReleaseMode::ENVELOPE:
        return release_envelope_met();
    case ReleaseMode::TIME:
        return release_time_met();
    case ReleaseMode::HYBRID:
        return release_time_met() && release_envelope_met();
    default:
        return release_time_met();
    }
}

bool RATOController::attitude_stable_for_ejection() const
{
    return fabsf(last_roll_deg) < 10.0f && fabsf(last_pitch_rate_rad_s) < 0.8f;
}


bool RATOController::is_active() const
{
    // Active means RATO is in progress and TAKEOFF should not complete yet.
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

// ADD THIS BLOCK immediately after: edits for the JSBSim test
bool RATOController::is_boosting() const
{
    return state == State::IGNITION || state == State::BOOST;
}
// this block only for JSBSim test. 

void RATOController::set_ignition_output(bool on)
{
    const int8_t ch = ign_chan.get();
    if (ch <= 0) {
        return;
    }
    const uint16_t pwm = on ? 2000 : 1000;
    gcs().send_text(MAV_SEVERITY_INFO,
                    "RATO ign ch=%d pwm=%u on=%d", (int)ch, (unsigned)pwm, (int)on);
    // set_output_pwm_chan() only sets have_pwm_mask — if that bit is cleared by
    // set_output_scaled(k_none,...) the value is silently overwritten next calc_pwm().
    // set_output_pwm_chan_timeout() additionally holds override_active=true for the
    // timeout duration, blocking calc_pwm() from recalculating the value.
    // 300 ms >> 100 ms navigate() period, so BOOST stays at 2000 throughout.
    SRV_Channels::set_output_pwm_chan_timeout(ch - 1, pwm, 300);
}

void RATOController::set_ejection_output(bool on)
{
    const int8_t ch = eject_chan.get();
    if (ch <= 0) {
        return;
    }
    const uint16_t pwm = on ? 2000 : 1000;
    SRV_Channels::set_output_pwm_chan_timeout(ch - 1, pwm, 300);
}

float RATOController::elapsed_s() const
{
    if (start_ms == 0) {
        return 0.0f;
    }

    return (AP_HAL::millis() - start_ms) * 0.001f;
}

float RATOController::burn_elapsed_s() const
{
    if (burn_start_ms == 0) {
        return 0.0f;
    }

    return (AP_HAL::millis() - burn_start_ms) * 0.001f;
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
