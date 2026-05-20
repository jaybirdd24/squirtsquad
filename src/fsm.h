#pragma once

#include <Arduino.h>
#include <Servo.h>
#include "percepetion.h"
#include "movement.h"

constexpr uint8_t PIN_PT_SERVO = 7;
constexpr uint8_t PIN_FAN      = 8;  // TODO: update when fan is wired

class FSM {
public:
    FSM(percepetion *perc, movement *mot);
    void init();
    void update();

private:
    enum class State { SCAN, ALIGN, APPROACH, FAN_ON, DONE };

    percepetion *perc;
    movement    *mot;
    Servo        ptServo;   // kept for when servo is fixed

    State state;
    int   sourcesHandled;

    // SCAN — 360° spin
    bool  scanInitialized;
    float peakHeading;      // heading (deg) at which brightest reading occurred
    int   peakBrightness;

    // ALIGN
    bool  alignInitialized;
    float alignTarget;      // degrees to rotate (+ = CCW, - = CW), shortest path

    // APPROACH
    float filteredDiff;         // EMA-smoothed left/right brightness diff
    int   approachPeakBright;   // brightness at start of approach (stray detection)

    // FAN_ON
    unsigned long dimStartMs;
    bool          dimTimerActive;

    // ── Sensor helpers ────────────────────────────────────────────
    int  longBrightness();   // inverted sum of long-range pair
    int  shortBrightness();  // inverted sum of short-range pair
    int  totalBrightness();  // all 4 combined
    int  minShortADC();      // raw ADC of brighter short-range sensor

    void doScan();
    void doAlign();
    void doApproach();
    void doFanOn();

    // ── Tuning ────────────────────────────────────────────────────
    static constexpr int   SCAN_ROTATE_SPEED  = 225;  // spin speed for scan
    static constexpr int   ROTATE_SPEED       = 150;
    static constexpr int   APPROACH_SPEED     = 150;
    static constexpr int   ALIGN_DEG          = 8;
    static constexpr int   MAX_STEER          = 250;
    static constexpr int   STEER_DEADBAND     = 20;
    static constexpr float STEER_SCALE        = 0.12f;
    static constexpr int   SHORT_ACTIVE_ADC   = 800;  // below this, short-range sensors are detecting light
    static constexpr int   CLOSE_ADC          = 50;   // short-range ADC < 50 → within ~10mm
    static constexpr int   DIM_BRIGHTNESS     = 300;
    static constexpr int   DIM_CONFIRM_MS     = 500;
    static constexpr float DIFF_EMA_ALPHA     = 0.2f; // smoothing on steering diff (lower = smoother)
    static constexpr float STRAY_FRACTION     = 0.4f; // re-scan if brightness drops below this fraction of approach peak
};
