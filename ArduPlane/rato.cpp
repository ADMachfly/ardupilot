#include "rato.h"

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

RATOController::RATOController()
{
    AP_Param::setup_object_defaults(this, var_info);

}