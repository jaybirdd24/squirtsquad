#pragma once

#include "percepetion.h"
#include "movement.h"

class LightSeek
{
public:
    LightSeek(percepetion *perc, movement *mot);

    // Call once per loop() — reads phototransistors and steers toward light
    void update();

private:
    percepetion *perc;
    movement    *mot;

    static constexpr int   DRIVE_SPEED    = 150;
    static constexpr int   MAX_STEER      = 250;
    static constexpr int   STEER_DEADBAND = 20;
    static constexpr float STEER_SCALE    = 0.12f;
};
