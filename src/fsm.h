#pragma once

#include <Arduino.h>
#include "percepetion.h"
#include "movement.h"
#include "comms.h"

class FSM {
public:
    FSM(); // initialises motors with a pointer to the local perception instance

    // One-time setup: initialise all subsystems and enter INITIALISING
    void fsmInit();

    // Called every loop(); advances the state machine by one tick
    void fsmUpdate();

private:
    // ── Subsystem instances ───────────────────────────────────────────
    percepetion perception; // sensor suite (IR, ultrasonic, IMU, battery)
    movement    motors;     // Mecanum drive + heading/wall PID; holds pointer to perception
    comms       radio;      // HC-12 wireless telemetry

    // ── Top-level robot state ─────────────────────────────────────────
    enum class RobotState {
        INITIALISING, // powering up, accumulating gyro bias
        RUNNING,      // executing mission behaviours
        STOPPED       // battery low; waiting to recover
    };
    RobotState robotState; // current top-level state

    // ── Motion command vocabulary ─────────────────────────────────────
    enum class Motion {
        STOP,          // halt all wheels
        FORWARD,       // drive straight ahead
        BACKWARD,      // drive straight back
        STRAFE_LEFT,   // lateral strafe to the left
        STRAFE_RIGHT,  // lateral strafe to the right
        TURN_LEFT,     // in-place CCW rotation
        TURN_RIGHT,    // in-place CW rotation
        FORWARD_LEFT,  // diagonal: forward + strafe left
        FORWARD_RIGHT  // diagonal: forward + strafe right
    };

    // ── Cruise sub-state machine ──────────────────────────────────────
    enum class CruiseState {
        FIND_WALL,     // rotate/advance until a wall is acquired
        SWEEP_FORWARD, // drive forward along the current lane
        LANE_STRAFE,   // lateral step to the next lane
        DONE           // all lanes swept; mission complete
    };

    // ── Cached sensor readings (updated each tick by cacheSensors) ────
    float distFrontLeft;  // IR medium-front distance (mm)
    float distFrontRight; // IR medium-right distance (mm)
    float distRear;       // IR long-rear distance (mm)
    float distSonar;      // ultrasonic forward distance (cm)
    float photoLeft;      // left photodiode raw ADC value
    float photoRight;     // right photodiode raw ADC value
    bool  fireDetected;   // true when a photodiode exceeds the fire threshold

    // ── Per-behaviour active flags (set inside each behaviour*) ──────
    bool cruiseActive;     // cruise behaviour has a command this tick
    bool avoidActive;      // obstacle-avoidance behaviour has a command this tick
    bool seekFireActive;   // fire-seeking behaviour has a command this tick
    bool extinguishActive; // extinguish behaviour has a command this tick
    bool escapeActive;     // escape behaviour has a command this tick
    bool haltActive;       // halt behaviour has a command this tick

    // ── Per-behaviour motion commands (set alongside their Active flag) ─
    Motion cruiseCmd;     // motion requested by behaviourCruise
    Motion avoidCmd;      // motion requested by behaviourAvoid
    Motion seekFireCmd;   // motion requested by behaviourSeekFire
    Motion extinguishCmd; // motion requested by behaviourExtinguish
    Motion escapeCmd;     // motion requested by behaviourEscape
    Motion haltCmd;       // motion requested by behaviourHalt

    // ── Mission counters ──────────────────────────────────────────────
    int           firesExtinguished; // total fire sources put out so far
    bool          fanRunning;        // true while the extinguisher fan is on
    unsigned long fanStartMs;        // millis() timestamp when fan was switched on

    // ── Cruise sub-state data ─────────────────────────────────────────
    CruiseState   cruiseState;   // active step within the cruise sub-machine
    int           laneCount;     // number of completed lane-strafe steps
    bool          sweepForward;  // true while driving forward in SWEEP_FORWARD
    unsigned long strafeStartMs; // millis() when the current LANE_STRAFE step began

    // ── Escape timer ─────────────────────────────────────────────────
    unsigned long escapeStartMs; // millis() when the escape manoeuvre started
    bool          escaping;      // true while escape reverse is in progress

    // ── Tick timer ───────────────────────────────────────────────────
    unsigned long lastTickMs; // millis() at the start of the previous FSM tick

    // ── Top-level state handlers ──────────────────────────────────────
    void stateInitialising(); // accumulates gyro bias; transitions to RUNNING when done
    void stateRunning();      // runs the full behaviour pipeline each tick
    void stateStopped();      // monitors battery; transitions back to RUNNING on recovery

    // ── Sensor cache ──────────────────────────────────────────────────
    void cacheSensors(); // copies all perception getters into local cached fields

    // ── Behaviour pipeline helpers ────────────────────────────────────
    void clearFlags();              // zeros all Active flags and Cmd fields before each tick
    void arbitrate();               // selects the highest-priority active command
    void executeMotion(Motion cmd); // maps a Motion value to movement API calls

    // ── Individual behaviours (each sets its Active flag + Cmd) ──────
    void behaviourCruise();     // wall-following lane sweep (lowest priority)
    void behaviourAvoid();      // reactive obstacle avoidance
    void behaviourSeekFire();   // steer toward a detected fire source
    void behaviourExtinguish(); // stop and run fan to extinguish fire
    void behaviourEscape();     // reverse away after collision or extinguish
    void behaviourHalt();       // stop all motion (battery low or mission done)

    // ── Cruise sub-state handlers ─────────────────────────────────────
    void cruiseFindWall();     // FIND_WALL: search for and approach the first wall
    void cruiseSweepForward(); // SWEEP_FORWARD: drive forward along the current lane
    void cruiseLaneStrafe();   // LANE_STRAFE: step laterally to the next lane

    // ── Debug ─────────────────────────────────────────────────────────
    void debugPrint(); // emits a state snapshot over Serial and comms
};
