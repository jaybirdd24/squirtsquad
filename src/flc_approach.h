#pragma once
#include <Arduino.h>
#include "percepetion.h"
#include "movement.h"

// Analog pins for left/right photosensors — adjust to match wiring
constexpr uint8_t PIN_PHOTO_LEFT  = A13;
constexpr uint8_t PIN_PHOTO_RIGHT = A14;
// Fan servo pin (matches base code turret pin)
constexpr uint8_t PIN_FAN = 11;

// Sum of both photo sensors — fire is "visible" above this
constexpr int FIRE_DETECT_THRESH = 100;
// Sum threshold indicating robot is very close to fire
constexpr int FIRE_CLOSE_THRESH  = 500;
// Fire is "centred" when offset is within ±this value
constexpr float FIRE_CENTRE_THRESH = 150.0f;
// Close enough by ultrasonic (cm)
constexpr float FIRE_CLOSE_CM = 25.0f;

class FLCApproach {
public:
    FLCApproach(percepetion* p, movement* m);
    void setup();

    // Call every loop() tick while in APPROACH state.
    // Reads sensors, runs fuzzy logic, writes motor commands.
    void update();

    // True when close to and centred on the fire.
    bool atFire()      const;
    // True if photo sensors detect fire above visibility threshold.
    bool fireVisible() const;
    // Raw fire direction error: photo_right - photo_left.
    int  fireOffset()  const;

private:
    percepetion* perc;
    movement*    mot;
    int photo_l, photo_r;

    // Triangular membership function: 0 outside (a,c), 1.0 at b.
    static float trimf(float x, float a, float b, float c);
    // Weighted-average defuzzification.
    static float defuzz(const float* strengths, const float* values, int n);
};
