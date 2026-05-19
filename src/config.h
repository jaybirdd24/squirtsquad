#pragma once

#include <Arduino.h>

namespace Config {

    // ── Distances ────────────────────────────────────────────────────────
    constexpr float OBSTACLE_AVOID_MM = 150.0f; // either front IR threshold to trigger avoidance (mm)
    constexpr float OBSTACLE_SONAR_CM = 20.0f;  // sonar threshold to trigger avoidance (cm)
    constexpr float OBSTACLE_CLEAR_MM = 170.0f; // front-left/front-right IR must rise above this to count as clear
    constexpr float OBSTACLE_SONAR_CLEAR_CM = 26.0f; // sonar must rise above this to count as clear
    constexpr float SIDE_CLEAR_MIN_MM = 180.0f; // side gap preferred before strafing toward that side
    constexpr float SIDE_CLEAR_MARGIN_MM = 50.0f; // side-gap difference to override direction

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
