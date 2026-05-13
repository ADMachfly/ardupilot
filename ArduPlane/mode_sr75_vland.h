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
    void update_stage();
    const char* stage_name(VLandStage s) const;
};