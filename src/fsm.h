#pragma once

#include <Arduino.h>
#include "percepetion.h"
#include "movement.h"
#include "comms.h"

class FSM {
public:
    FSM();

    // One-time setup: initialise all subsystems
    void fsmInit();

    // Called every loop(); advances the state machine by one tick
    void fsmUpdate();

private:
    static const int MAX_LIGHT_SAMPLES = 150;

    struct LightSample {
        float    heading;
        uint16_t longRightRaw;
        uint16_t longLeftRaw;
    };

    // ── Subsystem instances ───────────────────────────────────────────
    percepetion perception;
    movement    motors;
    comms       radio;

    // ── State machine ─────────────────────────────────────────────────
    enum class State {
        INITIALISING,    // accumulating gyro bias before movement
        SCANNING,        // rotate 360 deg and record long-range light readings
        ANALYZING,       // choose the target heading from the scan
        COARSE_ALIGN,    // rotate toward the target heading
        APPROACH_LIGHT,  // fuzzy fire homing + obstacle avoidance
        FINE_ALIGN,      // close-range light alignment before stopping
        ALIGNED          // target reached/aligned
    };
    State state;

    // ── Cached sensor readings (refreshed each tick by cacheSensors) ──
    float distFrontA;   // front-left IR distance (mm)
    float distFrontB;   // front-right IR distance (mm)
    float distLeft;     // left side IR distance (mm)
    float distRight;    // right side IR distance (mm)
    float distSonar;    // centre ultrasonic forward distance (cm)

    // ── Path data ─────────────────────────────────────────────────────
    float targetHeading;

    // ── Light homing data ─────────────────────────────────────────────
    LightSample   lightSamples[MAX_LIGHT_SAMPLES];
    int           lightSampleCount;
    unsigned long lastLightLogMs;
    unsigned long coarseAlignStartMs;
    unsigned long fineAlignStartMs;
    uint16_t      lightCloseRightRaw;
    uint16_t      lightLongRightRaw;
    uint16_t      lightLongLeftRaw;
    uint16_t      lightCloseLeftRaw;
    int           lastVx;
    int           lastVy;
    int           lastWz;
    float         lastFireOffset;
    float         lastObstacleCm;
    float         lastSidePreference;

    // ── Avoid-obstacle data ───────────────────────────────────────────
    int           avoidDirection;        // +1 = left first, -1 = right first
    unsigned long avoidSuppressUntilMs;  // obstacle avoidance suppressed until this timestamp

    // ── Multi-light tracking ──────────────────────────────────────────
    int           lightsFound;           // number of lights reached so far
    unsigned long alignedAtMs;           // timestamp when first ALIGNED was entered
    unsigned long lastExtinguishLogMs;
    float         extinguishBaselineRightV;
    float         extinguishBaselineLeftV;

    // ── Startup data ──────────────────────────────────────────────────
    unsigned long lastGyroBiasMs;
    unsigned long gyroBiasSamples;

    // ── Tick timer ───────────────────────────────────────────────────
    unsigned long lastTickMs;

    // ── Sensor cache ──────────────────────────────────────────────────
    void cacheSensors();

    // ── State handlers (dispatched from fsmUpdate switch) ────────────
    void initialising();
    void scanForLight();
    void analyzeLightScan();
    void coarseAlignToLight();
    void approachLight();
    void fineAlignToLight();
    void aligned();

    // ── Light helpers ─────────────────────────────────────────────────
    void  resetLightScan();
    float analyzeScans();
    void  setFanDuty(uint8_t duty) const;
    float nearestForwardObstacleCm() const;
    float sideClearancePreference() const;
    bool  obstacleRelevantForApproach(float obstacleCm) const;
    int   selectAvoidDirectionForApproach(float obstacleCm);
    void  fuzzyApproach(float fireOffset, float obstacleCm, float sidePreference,
                        int avoidDirection, int &vx, int &vy, int &wz) const;
    char  applySideGuard(int &vx, int &vy, int &wz) const;

    // ── Avoidance helpers ─────────────────────────────────────────────
    bool forwardObstacleDetected() const;
    bool sideClearForDirection(int direction) const;
    int  chooseAvoidDirection() const;
    void sendTelemetry();
};
