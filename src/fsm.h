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
    // ── Subsystem instances ───────────────────────────────────────────
    percepetion perception;
    movement    motors;
    comms       radio;

    // ── State machine ─────────────────────────────────────────────────
    enum class State {
        INITIALISING,   // accumulating gyro bias before movement
        DRIVE_FORWARD,  // isolation test: drive straight along the latched path
        AVOID_OBSTACLE  // move around a forward obstacle, then return to path
    };
    State state;

    // ── Cached sensor readings (refreshed each tick by cacheSensors) ──
    float distFrontA;   // front-left IR distance (mm)
    float distFrontB;   // front-right IR distance (mm)
    float distLeft;     // left side IR distance (mm)
    float distRight;    // right side IR distance (mm)
    float distSonar;    // centre ultrasonic forward distance (cm)

    // ── Path data ─────────────────────────────────────────────────────
    float pathHeading; // heading to hold before/during/after avoidance

    // ── Avoid-obstacle data ───────────────────────────────────────────
    int           avoidDirection; // +1 = left first, -1 = right first
    unsigned int  obstacleConfirmTicks;
    unsigned int  obstacleClearTicks;
    unsigned long lastAvoidExitMs;

    // ── Startup data ──────────────────────────────────────────────────
    unsigned long lastGyroBiasMs;
    unsigned long gyroBiasSamples;

    // ── Tick timer ───────────────────────────────────────────────────
    unsigned long lastTickMs;

    // ── Sensor cache ──────────────────────────────────────────────────
    void cacheSensors();

    // ── State handlers (dispatched from fsmUpdate switch) ────────────
    void initialising();   // accumulate gyro bias; -> DRIVE_FORWARD when ready
    void driveForward();   // drive on pathHeading; obstacles -> AVOID_OBSTACLE
    void avoidObstacle();  // strafe until the forward sensors are clear

    // ── Avoidance helpers ─────────────────────────────────────────────
    bool forwardObstacleDetected() const;
    bool forwardPathClear() const;
    bool sideClearForDirection(int direction) const;
    int  chooseAvoidDirection() const;
    void beginAvoidance();
    void sendTelemetry();
};
