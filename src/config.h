#pragma once

#include <Arduino.h>

namespace Config {

    // -- Light homing ----------------------------------------------------------
    constexpr uint8_t PIN_LIGHT_CLOSE_RIGHT = A4; // close range right of fan
    constexpr uint8_t PIN_LIGHT_LONG_RIGHT  = A5; // long range right of fan
    constexpr uint8_t PIN_LIGHT_LONG_LEFT   = A6; // long range left of fan
    constexpr uint8_t PIN_LIGHT_CLOSE_LEFT  = A7; // close range left of fan

    constexpr uint8_t PIN_FAN = 5;
    constexpr bool    FAN_ACTIVE_HIGH = true;
    constexpr uint8_t FAN_FULL_DUTY = 255;

    // Set true only for bench testing when the robot is already facing the light.
    constexpr bool LIGHT_DIRECT_APPROACH_TEST_MODE = false;

    constexpr int           FIRES_TO_EXTINGUISH       = 2;
    constexpr int           LIGHT_SCAN_SPIN_SPEED        = 200;
    constexpr float         LIGHT_SCAN_TARGET_DEG        = 360.0f;
    constexpr unsigned long LIGHT_LOG_INTERVAL_MS        = 50;
    constexpr float         LIGHT_DETECT_THRESHOLD_V     = 4.5f;
    constexpr float         LIGHT_HEADING_TOLERANCE_DEG  = 3.0f;
    constexpr float         LIGHT_STOP_VOLTAGE_V         = 1.0f;
    constexpr float         LIGHT_AVOID_DISABLE_V        = 1.0f;
    constexpr float         LIGHT_CLOSE_SUPPRESS_V       = 1.5f;
    constexpr unsigned long LIGHT_CLOSE_SUPPRESS_MS      = 10000;
    constexpr float         LIGHT_FINE_ALIGN_TOLERANCE_V = 0.25f;
    constexpr int           LIGHT_FINE_ALIGN_SPEED       = 150;
    constexpr unsigned long LIGHT_COARSE_ALIGN_TIMEOUT_MS = 4000;
    constexpr unsigned long LIGHT_FINE_ALIGN_TIMEOUT_MS   = 5000;
    constexpr unsigned long LIGHT_EXTINGUISH_LOG_MS       = 250;
    constexpr float         LIGHT_EXTINGUISHED_LONG_V     = 4.0f; // long-range sensor threshold above which fire is considered extinguished

    // Fuzzy light-approach outputs. Signs match movement::drive():
    // vx+ = forward, vy+ = left strafe, wz+ = right/CW turn.
    constexpr float FLC_DIRECTION_SELECT_CM = 20.0f;
    constexpr float FLC_AVOID_RELEASE_CM    = 32.0f;
    constexpr float FLC_SIDE_BIAS           = 180.0f;
    constexpr float FLC_SIDE_FULL_LEFT_MM   = 650.0f;
    constexpr float FLC_SIDE_FULL_RIGHT_MM  = 300.0f;
    constexpr int   FLC_VX_FAR              = 180;
    constexpr int   FLC_VX_MEDIUM           = 85;
    constexpr int   FLC_VX_CLOSE            = 25;
    constexpr int   FLC_TURN_FAST           = 200;
    constexpr int   FLC_TURN_MEDIUM         = 110;
    constexpr int   FLC_TURN_SLOW           = 50;
    constexpr int   FLC_STRAFE_SLOW         = 80;
    constexpr int   FLC_STRAFE_MEDIUM       = 130;
    constexpr int   FLC_STRAFE_FAST         = 200;
    constexpr int   FLC_MIN_ESCAPE_STRAFE   = 180;
    constexpr int   FLC_MIN_HOMING_VX       = 140;
    constexpr float SIDE_STRAFE_TRIGGER_MM = 150.0f; // strafe away from side obstacle below this distance
    constexpr int   SIDE_STRAFE_SPEED      = FLC_STRAFE_FAST;

    // ── Distances ────────────────────────────────────────────────────────
    constexpr float OBSTACLE_AVOID_MM = 110.0f; // either front IR threshold to trigger avoidance (mm)
    constexpr float OBSTACLE_SONAR_CM = 11.0f;  // sonar threshold to trigger avoidance (cm)
    constexpr float OBSTACLE_CLEAR_MM = 165.0f; // front-left/front-right IR must rise above this to count as clear
    constexpr float OBSTACLE_SONAR_CLEAR_CM = 30.0f; // sonar must rise above this to count as clear
    constexpr float SIDE_CLEAR_MIN_MM = 55.0f; // side gap preferred before strafing toward that side
    constexpr float SIDE_CLEAR_MARGIN_MM = 55.0f; // side-gap difference to override direction
    constexpr float SIDE_OPEN_COMPARE_CAP_MM = 300.0f; // cap side comparison to the shorter side sensor range
    constexpr float SIDE_OPEN_SWITCH_MARGIN_MM = 50.0f; // clearance advantage needed before switching escape side
    constexpr float SIDE_ESCAPE_MIN_MM = 50.0f; // absolute minimum side gap for an escape strafe

    // ── Speeds (0–1000) ──────────────────────────────────────────────────
    constexpr int SPEED_DRIVE  = 200; // forward speed during the isolation test
    constexpr int SPEED_SLOW   = 100; // forward speed while clearing the obstacle
    constexpr int SPEED_STRAFE = 180; // lateral speed during obstacle avoidance

    // Battery compensation is feed-forward only: it helps voltage sag, but
    // wheel/position feedback is still needed for slip and floor variation.
    constexpr bool BATTERY_COMPENSATION_ENABLED = false;
    constexpr float BATTERY_REFERENCE_V         = 4.2f; // fully charged LiPo cell voltage
    constexpr float BATTERY_MIN_COMPENSATED_V   = 3.5f; // do not boost harder below low-battery range
    constexpr float BATTERY_MAX_SCALE           = 1.20f;

    // Set false if front sensor A is physically on the robot's right side.
    constexpr bool FRONT_A_IS_LEFT = true;

    // ── Timing (ms) ──────────────────────────────────────────────────────
    constexpr unsigned long FSM_TICK_MS        = 10;  // main FSM update period
    constexpr unsigned long GYRO_BIAS_SAMPLES  = 100; // samples to accumulate for gyro bias
    constexpr unsigned long GYRO_BIAS_DELAY_MS = 10;  // delay between bias samples during INITIALISING
    constexpr unsigned int OBSTACLE_CONFIRM_TICKS = 3; // consecutive blocked ticks before avoiding
    constexpr unsigned int OBSTACLE_CLEAR_TICKS   = 6; // consecutive clear ticks before driving forward again
    constexpr unsigned long AVOID_DIRECTION_STICKY_MS = 1000; // reuse direction if obstacle reappears soon

    // ── Pins ─────────────────────────────────────────────────────────────
    constexpr uint8_t PIN_MOTOR_LF = 46; // left-front motor PWM servo pin
    constexpr uint8_t PIN_MOTOR_LR = 47; // left-rear motor PWM servo pin
    constexpr uint8_t PIN_MOTOR_RR = 50; // right-rear motor PWM servo pin
    constexpr uint8_t PIN_MOTOR_RF = 51; // right-front motor PWM servo pin

} // namespace Config
