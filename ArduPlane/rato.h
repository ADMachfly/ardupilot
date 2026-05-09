#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>

class RATOController {
public:
    static const struct AP_Param::GroupInfo var_info[];
    
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

    void reset();
    void init();
    bool update();

    bool is_active() const;
    bool is_complete() const;
    bool is_aborted() const;

    State get_state() const { return state; }
    const char *state_name() const;
    
    // Parameters
    AP_Int8     enable;             // RATO_ENABLE   0=off 1=on
    AP_Float    thrust_n;           // RATO_THR_N    booster thrust [N]
    AP_Float    mass_kg;            // RATO_MASS_KG  booster mass [kg]
    AP_Float    burn_time;          // RATO_BURN_S   burn duration [s]
    AP_Float    pitch_target_deg;   // RATO_PITCH    launch pitch [deg]
    AP_Float    rel_alt;            // RATO_REL_ALT  release altitude AGL [m]
    AP_Float    rel_dist;           // RATO_REL_DIST release ground distance [m]
    AP_Float    rel_spd;            // RATO_REL_SPD  release min speed [m/s]
    AP_Float    max_g;              // RATO_MAX_G    max allowed G-load
    AP_Float    min_g;              // RATO_MIN_G    min expected G during boost (abort if below)
    AP_Float    timeout_s;          // RATO_TIMEOUT  overall RATO phase timeout [s]
    AP_Int8     ign_chan;           // RATO_IGN_CH   relay/servo channel for ignition (1-based)
    AP_Int8     eject_chan;         // RATO_EJECT_CH relay/servo channel for ejection (1-based)

private:
    State state;
    uint32_t start_ms;

    float elapsed_s() const;    
};