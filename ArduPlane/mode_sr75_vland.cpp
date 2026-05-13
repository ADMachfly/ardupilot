#include "Plane.h"

ModeSR75VLand::ModeSR75VLand() :
    Mode()
{
}

bool ModeSR75VLand::_enter()
{
    last_status_ms = 0;
    gcs().send_text(MAV_SEVERITY_INFO, "SR75 VLAND: entered");
    return true;
}

void ModeSR75VLand::update()
{
    const uint32_t now_ms = AP_HAL::millis();

    if (now_ms - last_status_ms >= 1000) {
        last_status_ms = now_ms;
        gcs().send_text(MAV_SEVERITY_INFO, "SR75 VLAND: skeleton active");
    }
}