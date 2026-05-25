#include "fsm.h"
#include "config.h"

namespace {
    bool validDistance(float distance)
    {
        return distance > 0.0f;
    }
}

FSM::FSM()
    : motors(&perception),
      state(State::INITIALISING),
      distFrontA(0.0f),
      distFrontB(0.0f),
      distLeft(0.0f),
      distRight(0.0f),
      distSonar(0.0f),
      pathHeading(0.0f),
      avoidDirection(-1),
      obstacleConfirmTicks(0),
      obstacleClearTicks(0),
      lastAvoidExitMs(0),
      lastGyroBiasMs(0),
      gyroBiasSamples(0),
      lastTickMs(0)
{
}

void FSM::fsmInit() {
    perception.init();
    radio.init(115200);
    motors.enable();
    motors.Stop(true);

    state = State::INITIALISING;
    lastTickMs = millis();
    lastGyroBiasMs = lastTickMs;
}

void FSM::fsmUpdate() {
    unsigned long now = millis();
    if (now - lastTickMs < Config::FSM_TICK_MS) return;
    lastTickMs = now;

    cacheSensors();

    switch (state) {
        case State::INITIALISING:   initialising();   break;
        case State::DRIVE_FORWARD:  driveForward();   break;
        case State::AVOID_OBSTACLE: avoidObstacle();  break;
    }

    sendTelemetry();
}

void FSM::cacheSensors() {
    perception.update();

    distFrontA = perception.getIRMedFront();
    distFrontB = perception.getIRLongRear();
    distLeft   = perception.getIRLongLeft();
    distRight  = perception.getIRMedRight();
    distSonar  = perception.getUltrasonicCm();
}

void FSM::initialising() {
    unsigned long now = millis();
    motors.Stop();

    if (now - lastGyroBiasMs < Config::GYRO_BIAS_DELAY_MS) return;

    perception.feedGyroBias();
    lastGyroBiasMs = now;
    gyroBiasSamples++;

    if (gyroBiasSamples >= Config::GYRO_BIAS_SAMPLES) {
        perception.freezeGyroBias();
        motors.resetHeading();
        motors.latchHeading();
        pathHeading = motors.getHeading();
        state = State::DRIVE_FORWARD;
    }
}

void FSM::driveForward() {
    if (forwardObstacleDetected()) {
        obstacleConfirmTicks++;
        if (obstacleConfirmTicks >= Config::OBSTACLE_CONFIRM_TICKS) {
            beginAvoidance();
        }
        return;
    }

    obstacleConfirmTicks = 0;
    motors.setTargetHeading(pathHeading);
    motors.MoveForward(Config::SPEED_DRIVE);
}

void FSM::avoidObstacle() {
    if (avoidDirection == 0) {
        motors.Stop(true);
        return;
    }

    if (forwardPathClear()) {
        obstacleClearTicks++;
        if (obstacleClearTicks >= Config::OBSTACLE_CLEAR_TICKS) {
            obstacleConfirmTicks = 0;
            obstacleClearTicks = 0;
            lastAvoidExitMs = millis();
            state = State::DRIVE_FORWARD;
            return;
        }
    } else {
        obstacleClearTicks = 0;
    }

    // No heading correction during avoidance: pure side strafe until clear.
    motors.drive(0, avoidDirection * Config::SPEED_STRAFE, 0);
}

bool FSM::forwardObstacleDetected() const {
    bool frontAIrBlocked = validDistance(distFrontA) &&
                           distFrontA <= Config::OBSTACLE_AVOID_MM;
    bool frontBIrBlocked = validDistance(distFrontB) &&
                           distFrontB <= Config::OBSTACLE_AVOID_MM;
    bool sonarBlocked = validDistance(distSonar) &&
                        distSonar <= Config::OBSTACLE_SONAR_CM;
    return frontAIrBlocked || frontBIrBlocked || sonarBlocked;
}

bool FSM::forwardPathClear() const {
    bool frontAClear = !validDistance(distFrontA) ||
                       distFrontA >= Config::OBSTACLE_CLEAR_MM;
    bool frontBClear = !validDistance(distFrontB) ||
                       distFrontB >= Config::OBSTACLE_CLEAR_MM;
    return frontAClear && frontBClear;
}

bool FSM::sideClearForDirection(int direction) const {
    if (direction > 0) {
        return !validDistance(distLeft) || distLeft >= Config::SIDE_CLEAR_MIN_MM;
    }
    if (direction < 0) {
        return !validDistance(distRight) || distRight >= Config::SIDE_CLEAR_MIN_MM;
    }
    return false;
}

int FSM::chooseAvoidDirection() const {
    bool frontABlocked = validDistance(distFrontA) &&
                         distFrontA <= Config::OBSTACLE_AVOID_MM;
    bool frontBBlocked = validDistance(distFrontB) &&
                         distFrontB <= Config::OBSTACLE_AVOID_MM;

    int direction = -1;

    bool leftClear = !validDistance(distLeft) ||
                     distLeft >= Config::SIDE_CLEAR_MIN_MM;
    bool rightClear = !validDistance(distRight) ||
                      distRight >= Config::SIDE_CLEAR_MIN_MM;

    if (leftClear && !rightClear) return 1;
    if (rightClear && !leftClear) return -1;
    if (!leftClear && !rightClear) return 0;

    if (frontABlocked != frontBBlocked) {
        bool blockedSideIsLeft = frontABlocked == Config::FRONT_A_IS_LEFT;
        direction = blockedSideIsLeft ? -1 : 1;
    } else if (validDistance(distLeft) && validDistance(distRight)) {
        if (distLeft > distRight + Config::SIDE_CLEAR_MARGIN_MM) {
            direction = 1;
        } else if (distRight > distLeft + Config::SIDE_CLEAR_MARGIN_MM) {
            direction = -1;
        }
    }

    bool leftTooClose = validDistance(distLeft) &&
                        distLeft < Config::SIDE_CLEAR_MIN_MM;
    bool rightTooClose = validDistance(distRight) &&
                         distRight < Config::SIDE_CLEAR_MIN_MM;

    if (direction == 1 && leftTooClose && !rightTooClose) {
        direction = -1;
    } else if (direction == -1 && rightTooClose && !leftTooClose) {
        direction = 1;
    }

    return direction;
}

void FSM::beginAvoidance() {
    motors.Stop(true);
    pathHeading = motors.getTargetHeading();

    bool shouldReuseDirection = avoidDirection != 0 &&
                                millis() - lastAvoidExitMs <= Config::AVOID_DIRECTION_STICKY_MS &&
                                sideClearForDirection(avoidDirection);
    if (!shouldReuseDirection) {
        avoidDirection = chooseAvoidDirection();
    }

    obstacleConfirmTicks = 0;
    obstacleClearTicks = 0;
    state = State::AVOID_OBSTACLE;
}

void FSM::sendTelemetry() {
    radio.sendAvoidanceData(
        distFrontA,
        distFrontB,
        distLeft,
        distRight,
        distSonar,
        motors.getHeading(),
        motors.getTargetHeading(),
        motors.getLastHeadingError(),
        (int)state,
        0,
        forwardObstacleDetected() ? 1 : 0,
        avoidDirection
    );
}
