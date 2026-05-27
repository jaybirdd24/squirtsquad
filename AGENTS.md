# AGENTS.md

Guidance for Codex and other coding agents working in this repository.

## Project Overview

This is a Mechatronics Arduino robot project for an Arduino Mega2560, built with PlatformIO and the Arduino framework. The robot uses Mecanum wheels, IR distance sensors, an ultrasonic sensor, a BNO08x IMU, HC-12 wireless telemetry, and phototransistor-based light homing.

The main firmware is in `src/` and is orchestrated by `FSM` in `src/fsm.h` / `src/fsm.cpp`. `src/main.cpp` only creates the single `FSM` instance and calls `fsmInit()` / `fsmUpdate()`.

## Project Specification And Goals

Source: `Project 2 Information Sheet 2026.pdf`.

The project goal is an autonomous firefighting robot for a walled tabletop arena. The robot starts from an unknown position and orientation, must find and extinguish two fires as quickly as possible, avoid collisions with fires, obstacles, and the table walls, and stop immediately after the second fire is extinguished.

Key task conditions:

- Fires and obstacles are cylinders about 10 cm in diameter and 12 cm tall, randomly positioned on the table.
- The number of obstacles is unknown; most are static, but one or two may move unpredictably toward the robot during a run.
- Objects can be touching each other or placed against a wall, so avoidance logic should not assume clean gaps around every obstacle.
- A fire is an LED/thermistor assembly on a cylinder. It is extinguished by running the fan for up to 10 seconds; if the light goes out earlier, the robot can proceed.
- A fire can only be extinguished when the robot centre is within a 20 cm radius of the fire centre.
- The robot must cease all movement immediately after extinguishing the second fire.

Design-kit constraints:

- The kit includes the Arduino Mega mobile base, four drive motors, battery, shield, small servo, BNO08x gyro, HC-SR04 ultrasonic sensor, four phototransistors, fan, HC-12 wireless modules, MOSFET, 5 V regulator, and LiPo battery.
- The kit provides four medium-range IR sensors and four long-range IR sensors, but only two IR sensors of each type may be used in the design.
- Self-made 3D-printed brackets are allowed.

Demonstration priorities:

- Each robot gets 5 minutes for two runs; the better run is used.
- The demonstration uses two self-powered lights and a common randomized setup for all groups.
- Demonstration marking is 60% of the project: firefighting 20%, obstacle avoidance 20%, and time 20%.
- Engineering priority should be: extinguish both fires, avoid collisions, then reduce elapsed time. Time receives only base credit if the robot fails completely at firefighting and obstacle avoidance.
- Best time score requires 60 seconds or less; 100 seconds or more receives the lowest non-base time score.

Report expectations:

- The technical report is 40% of the project assessment.
- Suggested chapters are fire sensing/extinguishing, obstacle sensing, behaviour control, system integration, and project management.
- Subsystem chapters should cover what was built, how it was designed/interfaced/tested/calibrated, and evidence/results such as data, charts, diagrams, or discussion.
- System integration should document software/hardware configuration, MCU pin allocations and functions, flow charts, architecture, and pseudocode.
- Report marking emphasizes technical content first, then presentation quality and formatting/language.

## Build And Test Commands

Use PlatformIO commands from the repo root:

```bash
pio run
pio run -t upload
pio run -t clean
pio test
pio device monitor
pio run -e ir_calibrate -t upload
```

Useful named environments in `platformio.ini`:

- `megaatmega2560`: main robot firmware.
- `ir_calibrate`: IR calibration sketch from `test/ir_calibrate.cpp`.
- `scan360`: 360-degree scan firmware.
- `light_detection_test`: phototransistor test firmware.
- `light_detection_homing`: light-homing test firmware.
- `ultrasonic_test`: ultrasonic/perception test firmware.

The project depends on `arduino-libraries/Servo` and `adafruit/Adafruit BNO08x`; a first build may need network access to install PlatformIO packages.

## Core Files

- `src/fsm.h`, `src/fsm.cpp`: top-level state machine for gyro startup calibration, light scan, heading alignment, fuzzy light approach, obstacle avoidance, fine alignment, and telemetry.
- `src/config.h`: shared constants for pins, thresholds, timings, speeds, light homing, obstacle avoidance, and battery compensation.
- `src/movement.h`, `src/movement.cpp`: Mecanum inverse kinematics, heading PID, wall-following PID, slew limiting, battery feed-forward compensation, and motor commands.
- `src/percepetion.h`, `src/percepetion.cpp`: sensor abstraction for IR sensors, ultrasonic, IMU gyro, battery voltage, and gyro bias calibration. The misspelling `percepetion` is part of the current API and filename; do not rename it unless the task explicitly includes the repo-wide migration.
- `src/comms.h`, `src/comms.cpp`: HC-12 telemetry over `SoftwareSerial`, including Teleplot-friendly output.
- `tools/serial_bridge.py`: desktop serial-to-Teleplot UDP bridge and CSV logger.
- `scripts/plot_scan.py`: plotting helper for scan data.
- `base_codes/RobotBaseCodes2026.ino`: monolithic reference implementation from the course material.

## Runtime Flow

The main FSM currently uses these states:

```text
INITIALISING -> SCANNING -> ANALYZING -> COARSE_ALIGN -> APPROACH_LIGHT -> FINE_ALIGN -> ALIGNED
```

Startup behavior matters:

- `INITIALISING` must keep the robot stationary while `perception.feedGyroBias()` accumulates about `Config::GYRO_BIAS_SAMPLES` samples at `Config::GYRO_BIAS_DELAY_MS` spacing.
- Call `perception.freezeGyroBias()`, then `motors.resetHeading()` and `motors.latchHeading()` before movement.
- Do not accumulate gyro bias while the wheels are moving.

## Hardware And Sign Conventions

Pin assignments are split between `src/config.h`, `src/movement.cpp`, and `src/percepetion.h`; keep them consistent if wiring changes.

Important pins:

- Motors: LF `46`, LR `47`, RR `50`, RF `51`.
- Ultrasonic: trigger `48`, echo `49`.
- HC-12: SoftwareSerial RX `10`, TX `11`.
- Battery divider: `A0`.
- IR distance sensors: front medium `A10`, left long `A8`, right medium `A12`, second/front long `A9`.
- Phototransistors: close right `A4`, long right `A5`, long left `A6`, close left `A7`.

Movement sign conventions from `movement::drive(vx, vy, wz)`:

- `vx > 0`: forward.
- `vy > 0`: strafe left.
- `wz > 0`: rotation correction matching the current code comments; check existing `RotateCW` / `RotateCCW` usage before changing signs.

Motor commands use the reference speed range `-1000..1000`, with servo neutral at `1500 us`.

## Coding Notes

- Prefer changing constants in `src/config.h` over hard-coding thresholds or speeds in control logic.
- Keep sensor reads centralized through `percepetion` unless a subsystem intentionally reads raw ADC for calibration or phototransistor light sensing.
- Avoid `snprintf` with `%f` on AVR; existing comms code uses `print()` chains because AVR libc float formatting is unreliable by default.
- Keep loops non-blocking where possible. Short, intentional delays already exist in fine alignment, but the main FSM tick is designed around `Config::FSM_TICK_MS`.
- Be careful with heap use and large stack allocations on ATmega2560. Prefer static-sized arrays and simple data structures.
- Preserve telemetry prefixes (`$S`, `$C`, `$A`, `$L`) unless updating `tools/serial_bridge.py` and any Teleplot expectations at the same time.
- Existing comments include hardware calibration notes. Treat them as test data, not style examples to expand unnecessarily.

## Verification

For logic-only changes, run at least:

```bash
pio run
```

When touching tests or portable helper code, also run:

```bash
pio test
```

For hardware-facing changes, note which firmware environment should be uploaded and what serial/telemetry output should be checked. Do not assume successful compilation proves behavior on the physical robot.
