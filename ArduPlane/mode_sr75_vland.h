#pragma once

#include "mode.h"

class ModeSR75VLand : public Mode
{
public:
    ModeSR75VLand();

    bool _enter() override;
    void update() override;

    const char *name() const override { return "SR75_VLAND"; }
    const char *name4() const override { return "VLND"; }

private:
    enum class VLandStage : uint8_t {
        DESCENT = 0,
        RECOVERY_GATE,
        COBRA_ENTRY,
        PITCH_TO_VERTICAL,
        VERTICAL_STABILIZE,
        LEG_DEPLOY,
        PRECISION_DESCENT,
        TOUCHDOWN,
        ABORT
    };

    VLandStage stage;
    uint32_t stage_start_ms;
    uint32_t last_status_ms;

    void set_stage(VLandStage new_stage);
    void ModeSR75VLand::update_stage()
{
    const float alt_m = get_relative_alt_m();
    const float gs_ms = get_groundspeed_ms();

    switch (stage) {
    case VLandStage::DESCENT:
        if (alt_m <= 1000.0f) {
            set_stage(VLandStage::RECOVERY_GATE);
        }
        break;

    case VLandStage::RECOVERY_GATE:
        if (alt_m <= 700.0f && gs_ms <= 45.0f) {
            set_stage(VLandStage::COBRA_ENTRY);
        }
        break;

    case VLandStage::COBRA_ENTRY:
        if (alt_m <= 500.0f) {
            set_stage(VLandStage::PITCH_TO_VERTICAL);
        }
        break;

    case VLandStage::PITCH_TO_VERTICAL:
        if (alt_m <= 300.0f) {
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
    const char* stage_name(VLandStage s) const;
};