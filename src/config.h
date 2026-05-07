#pragma once

#include <Arduino.h>

namespace Config {

    // ── Distances (mm) ──────────────────────────────────────────────────
    constexpr float WALL_FOLLOW_SETPOINT_MM   = 250.0f; // target lateral gap to wall
    constexpr float OBSTACLE_STOP_MM          = 100.0f; // stop if any sensor reads below this
    constexpr float OBSTACLE_AVOID_MM         = 150.0f; // begin avoidance above this range
    constexpr float FIRE_DETECT_MM            = 300.0f; // photo sensor threshold to declare fire detected
    constexpr float SONAR_WALL_MM             = 200.0f; // forward sonar threshold to detect wall ahead (cm→mm: stored in mm for consistency)
    constexpr float LANE_WIDTH_MM             = 400.0f; // expected corridor lane width for lane-strafe manoeuvre

    // ── Speeds ───────────────────────────────────────────────────────────
    constexpr int SPEED_CRUISE        = 350; // default forward cruise speed (0–1000 units)
    constexpr int SPEED_SLOW          = 200; // reduced speed for precise manoeuvres (0–1000 units)
    constexpr int SPEED_FAST          = 500; // aggressive speed for open-field movement (0–1000 units)
    constexpr int SPEED_TURN          = 300; // rotation speed for in-place turns (0–1000 units)
    constexpr int SPEED_STRAFE        = 300; // lateral strafe speed for lane changes (0–1000 units)
    constexpr int SPEED_ESCAPE        = 400; // reverse speed during escape manoeuvre (0–1000 units)
    constexpr int SPEED_FAN           = 700; // fan/extinguisher motor output level (0–1000 units)

    // ── Timing (ms) ──────────────────────────────────────────────────────
    constexpr unsigned long FSM_TICK_MS          = 10;   // main FSM update period
    constexpr unsigned long GYRO_BIAS_SAMPLES    = 100;  // number of samples to accumulate for gyro bias
    constexpr unsigned long GYRO_BIAS_DELAY_MS   = 10;   // delay between bias samples during INITIALISING
    constexpr unsigned long ESCAPE_DURATION_MS   = 600;  // how long to reverse during escape manoeuvre
    constexpr unsigned long FAN_RUN_MS           = 2000; // how long to run the fan after fire detected
    constexpr unsigned long STRAFE_DURATION_MS   = 800;  // how long to strafe during a lane-change step
    constexpr unsigned long SWEEP_DURATION_MS    = 1200; // how long to sweep forward between lane changes

    // ── Thresholds ───────────────────────────────────────────────────────
    constexpr int   BATTERY_LOW_RAW         = 717;   // ADC count below which battery is considered low
    constexpr int   BATTERY_RECOVER_RAW     = 730;   // ADC count above which battery is considered recovered
    constexpr int   BATTERY_STABLE_COUNT    = 10;    // consecutive readings above BATTERY_RECOVER_RAW to resume
    constexpr float PHOTO_FIRE_THRESHOLD    = 500.0f; // raw ADC value above which photodiode declares fire
    constexpr float HEADING_TOLERANCE_DEG   = 3.0f;  // acceptable heading error before considering turn complete

    // ── Mission ──────────────────────────────────────────────────────────
    constexpr int   TOTAL_FIRES             = 2;     // number of fire sources expected in the arena
    constexpr int   LANE_COUNT_MAX          = 4;     // maximum lane-strafe steps before declaring arena swept
    constexpr bool  FOLLOW_LEFT_WALL        = true;  // true = follow left wall, false = follow right wall

    // ── Pins ─────────────────────────────────────────────────────────────
    constexpr uint8_t PIN_MOTOR_LF   = 46; // left-front motor PWM servo pin
    constexpr uint8_t PIN_MOTOR_LR   = 47; // left-rear motor PWM servo pin
    constexpr uint8_t PIN_MOTOR_RR   = 50; // right-rear motor PWM servo pin
    constexpr uint8_t PIN_MOTOR_RF   = 51; // right-front motor PWM servo pin
    constexpr uint8_t PIN_HC12_RX    = 10; // SoftwareSerial RX for HC-12 wireless module
    constexpr uint8_t PIN_HC12_TX    = 11; // SoftwareSerial TX for HC-12 wireless module
    constexpr uint8_t PIN_FAN        = 9;  // digital output pin driving the extinguisher fan

} // namespace Config
