#include "Plane.h"

ModeSR75VLand::ModeSR75VLand() :
    Mode()
{
    stage = VLandStage::DESCENT;
    stage_start_ms = 0;
    last_status_ms = 0;
}

bool ModeSR75VLand::_enter()
{
    gcs().send_text(MAV_SEVERITY_INFO, "SR75 VLAND: entered");
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

    // Phase 2 only: skeleton heartbeat
    const uint32_t now = AP_HAL::millis();
    if (now - last_status_ms > 1000) {
        last_status_ms = now;
        gcs().send_text(MAV_SEVERITY_INFO, "SR75 VLAND: %s", stage_name(stage));

    }
}

void ModeSR75VLand::set_stage(const VLandStage new_stage)
{
    if (stage == new_stage) {
        return;
    }

    stage = new_stage;
    stage_start_ms = AP_HAL::millis();

    gcs().send_text(MAV_SEVERITY_INFO, "SR75 VLAND stage: %s", stage_name(stage));
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
    const uint32_t now = AP_HAL::millis();
    const uint32_t elapsed_ms = now - stage_start_ms;

    // Phase 2 temporary timer-based transitions only
    switch (stage) {
    case VLandStage::DESCENT:
        if (elapsed_ms > 5000) {
            set_stage(VLandStage::RECOVERY_GATE);
        }
        break;

    case VLandStage::RECOVERY_GATE:
        if (elapsed_ms > 3000) {
            set_stage(VLandStage::COBRA_ENTRY);
        }
        break;

    case VLandStage::COBRA_ENTRY:
        if (elapsed_ms > 3000) {
            set_stage(VLandStage::PITCH_TO_VERTICAL);
        }
        break;

    case VLandStage::PITCH_TO_VERTICAL:
        if (elapsed_ms > 3000) {
            set_stage(VLandStage::VERTICAL_STABILIZE);
        }
        break;

    case VLandStage::VERTICAL_STABILIZE:
        if (elapsed_ms > 3000) {
            set_stage(VLandStage::LEG_DEPLOY);
        }
        break;

    case VLandStage::LEG_DEPLOY:
        if (elapsed_ms > 2000) {
            set_stage(VLandStage::PRECISION_DESCENT);
        }
        break;

    case VLandStage::PRECISION_DESCENT:
        if (elapsed_ms > 5000) {
            set_stage(VLandStage::TOUCHDOWN);
        }
        break;

    case VLandStage::TOUCHDOWN:
    case VLandStage::ABORT:
        break;
    }
}