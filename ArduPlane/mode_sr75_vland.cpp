#include "Plane.h"

ModeSR75VLand::ModeSR75VLand() :
    Mode()
{
    stage = VLandStage::DESCENT;
    stage_start_ms = 0;
    last_status_ms = 0;

    touchdown_set = false;

    recovery_distance_m = 8000.0f;
    terminal_alt_m = 1000.0f;
    cobra_alt_m = 700.0f;
    vertical_stabilize_alt_m = 300.0f;
    commit_alt_m = 100.0f;
}

bool ModeSR75VLand::_enter()
{
    gcs().send_text(MAV_SEVERITY_INFO, "SR75 VLAND: entered");

if (!touchdown_set) {
    init_default_target();
} else {
    gcs().send_text(MAV_SEVERITY_INFO,
                    "SR75 VLAND: using stored touchdown target");
}

calculate_recovery_geometry();

    stage = VLandStage::DESCENT;
    stage_start_ms = AP_HAL::millis();
    last_status_ms = 0;

    gcs().send_text(MAV_SEVERITY_INFO, "SR75 VLAND stage: %s", stage_name(stage));
    return true;
}

void ModeSR75VLand::run()
{
    update();
}

void ModeSR75VLand::update()
{
    update_stage();

    const uint32_t now = AP_HAL::millis();
    if (now - last_status_ms > 1000) {
        last_status_ms = now;
        gcs().send_text(MAV_SEVERITY_INFO,
                        "SR75 VLAND: %s alt=%.1f gs=%.1f",
                        stage_name(stage),
                        get_relative_alt_m(),
                        get_groundspeed_ms());
    }
}

void ModeSR75VLand::set_stage(const VLandStage new_stage)
{
    if (stage == new_stage) {
        return;
    }

    stage = new_stage;
    stage_start_ms = AP_HAL::millis();

    gcs().send_text(MAV_SEVERITY_INFO,
                "SR75 VLAND: %s alt=%.1f gs=%.1f",
                stage_name(stage),
                get_relative_alt_m(),
                get_groundspeed_ms());
}

const char* ModeSR75VLand::stage_name(const VLandStage s) const
{
    switch (s) {
    case VLandStage::DESCENT:
        return "DESCENT";
    case VLandStage::RECOVERY_GATE:
        return "RECOVERY_GATE";
    case VLandStage::COBRA_ENTRY:
        return "COBRA_ENTRY";
    case VLandStage::PITCH_TO_VERTICAL:
        return "PITCH_TO_VERTICAL";
    case VLandStage::VERTICAL_STABILIZE:
        return "VERTICAL_STABILIZE";
    case VLandStage::LEG_DEPLOY:
        return "LEG_DEPLOY";
    case VLandStage::PRECISION_DESCENT:
        return "PRECISION_DESCENT";
    case VLandStage::TOUCHDOWN:
        return "TOUCHDOWN";
    case VLandStage::ABORT:
        return "ABORT";
    }

    return "UNKNOWN";
}

void ModeSR75VLand::update_stage()
{
    const float alt_m = get_relative_alt_m();
    const float gs_ms = get_groundspeed_ms();

    switch (stage) {
    case VLandStage::DESCENT:
        if (alt_m <= terminal_alt_m) {
            set_stage(VLandStage::RECOVERY_GATE);
        }
        break;

    case VLandStage::RECOVERY_GATE:
        if (alt_m <= cobra_alt_m && gs_ms <= 45.0f) {
            set_stage(VLandStage::COBRA_ENTRY);
        }
        break;

    case VLandStage::COBRA_ENTRY:
        if (alt_m <= 500.0f) {
            set_stage(VLandStage::PITCH_TO_VERTICAL);
        }
        break;

    case VLandStage::PITCH_TO_VERTICAL:
        if (alt_m <= vertical_stabilize_alt_m) {
            set_stage(VLandStage::VERTICAL_STABILIZE);
        }
        break;

    case VLandStage::VERTICAL_STABILIZE:
        if (alt_m <= 250.0f) {
            set_stage(VLandStage::LEG_DEPLOY);
        }
        break;

    case VLandStage::LEG_DEPLOY:
        if (alt_m <= 200.0f) {
            set_stage(VLandStage::PRECISION_DESCENT);
        }
        break;

    case VLandStage::PRECISION_DESCENT:
        if (alt_m <= 2.0f) {
            set_stage(VLandStage::TOUCHDOWN);
        }
        break;

    case VLandStage::TOUCHDOWN:
    case VLandStage::ABORT:
        break;
    }
}


float ModeSR75VLand::get_relative_alt_m() const
{
    Location current_loc;
    if (!plane.ahrs.get_location(current_loc)) {
        return 0.0f;
    }

    return current_loc.alt * 0.01f;
}

float ModeSR75VLand::get_groundspeed_ms() const
{
    return plane.ahrs.groundspeed();
}

bool ModeSR75VLand::reached_altitude_below(const float alt_m) const
{
    return get_relative_alt_m() <= alt_m;
}

void ModeSR75VLand::init_default_target()
{
    Location current_loc;
    if (plane.ahrs.get_location(current_loc)) {
        touchdown_loc = current_loc;
        touchdown_loc.alt = 0;
        touchdown_set = true;

        gcs().send_text(MAV_SEVERITY_INFO,
                        "SR75 VLAND target: current position default");
    } else {
        touchdown_set = false;
        gcs().send_text(MAV_SEVERITY_WARNING,
                        "SR75 VLAND: no location for target");
    }
}

void ModeSR75VLand::calculate_recovery_geometry()
{
    recovery_distance_m = 8000.0f;
    terminal_alt_m = 1000.0f;
    cobra_alt_m = 700.0f;
    vertical_stabilize_alt_m = 300.0f;
    commit_alt_m = 100.0f;

    gcs().send_text(MAV_SEVERITY_INFO,
                    "SR75 VLAND geometry: rec=%.0fm term=%.0fm cobra=%.0fm",
                    recovery_distance_m,
                    terminal_alt_m,
                    cobra_alt_m);
}

void ModeSR75VLand::set_touchdown_target(const Location &loc)
{
    touchdown_loc = loc;
    touchdown_set = true;

    gcs().send_text(MAV_SEVERITY_INFO,
                    "SR75 VLAND target set: lat=%ld lon=%ld alt=%.1fm",
                    (long)touchdown_loc.lat,
                    (long)touchdown_loc.lng,
                    touchdown_loc.alt * 0.01f);
}

bool ModeSR75VLand::start_from_mission_target(const Location &loc)
{
    set_touchdown_target(loc);

    gcs().send_text(MAV_SEVERITY_INFO,
                    "SR75 VLAND mission trigger");

    return true;
}