#include "fsm.h"
#include "config.h"

namespace {
    enum OffsetMf {
        OFFSET_NL,
        OFFSET_NM,
        OFFSET_ZE,
        OFFSET_PM,
        OFFSET_PL,
        OFFSET_COUNT
    };

    enum DistanceMf {
        DIST_VERY_CLOSE,
        DIST_CLOSE,
        DIST_MEDIUM,
        DIST_FAR,
        DIST_COUNT
    };

    enum SideMf {
        SIDE_CLEAR_RIGHT,
        SIDE_NEUTRAL,
        SIDE_CLEAR_LEFT,
        SIDE_COUNT
    };

    bool validDistance(float distance)
    {
        return distance > 0.0f;
    }

    float adcToVolts(int raw)
    {
        return (float)raw * 5.0f / 1023.0f;
    }

    float minf(float a, float b)
    {
        return (a < b) ? a : b;
    }


    uint8_t fanOutputForDuty(uint8_t duty)
    {
        return Config::FAN_ACTIVE_HIGH ? duty : 255 - duty;
    }

    float trimf(float x, float a, float b, float c)
    {
        if (a == b) {
            if (x <= b) return 1.0f;
            if (x >= c) return 0.0f;
            return (c - x) / (c - b);
        }

        if (b == c) {
            if (x <= a) return 0.0f;
            if (x >= b) return 1.0f;
            return (x - a) / (b - a);
        }

        if (x <= a || x >= c) return 0.0f;
        if (x == b) return 1.0f;
        if (x < b) return (x - a) / (b - a);
        return (c - x) / (c - b);
    }

    float defuzzify(const float *strengths, const float *values, int count)
    {
        float numerator = 0.0f;
        float denominator = 0.0f;

        for (int i = 0; i < count; i++) {
            numerator += strengths[i] * values[i];
            denominator += strengths[i];
        }

        return (denominator > 0.0f) ? numerator / denominator : 0.0f;
    }

    void addRule(float *strengths, float *values, int &count,
                 float strength, float value)
    {
        strengths[count] = strength;
        values[count] = value;
        count++;
    }

    float normalizeAngle(float degrees)
    {
        while (degrees > 180.0f) degrees -= 360.0f;
        while (degrees < -180.0f) degrees += 360.0f;
        return degrees;
    }

    float sideClearScore(float distanceMm, float fullClearMm)
    {
        if (!validDistance(distanceMm)) {
            return 1.0f;
        }

        float score = (distanceMm - Config::SIDE_CLEAR_MIN_MM) /
                      (fullClearMm - Config::SIDE_CLEAR_MIN_MM);
        return constrain(score, 0.0f, 1.0f);
    }

    float cappedSideClearance(float distanceMm)
    {
        if (!validDistance(distanceMm)) {
            return -1.0f;
        }

        return constrain(distanceMm, 0.0f, Config::SIDE_OPEN_COMPARE_CAP_MM);
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
      targetHeading(0.0f),
      lightSampleCount(0),
      lastLightLogMs(0),
      coarseAlignStartMs(0),
      fineAlignStartMs(0),
      lightCloseRightRaw(0),
      lightLongRightRaw(0),
      lightLongLeftRaw(0),
      lightCloseLeftRaw(0),
      lastVx(0),
      lastVy(0),
      lastWz(0),
      lastFireOffset(0.0f),
      lastObstacleCm(80.0f),
      lastSidePreference(0.0f),
      avoidDirection(0),
      avoidSuppressUntilMs(0),
      sideGuardBlockedSide(0),
      sideGuardHoldUntilMs(0),
      blindSpotLeftUntilMs(0),
      blindSpotRightUntilMs(0),
      reverseEscapeUntilMs(0),
      recentObstacleHits(0),
      lastObstacleHitMs(0),
      lightsFound(0),
      alignedAtMs(0),
      lastExtinguishLogMs(0),
      extinguishBaselineRightV(5.0f),
      extinguishBaselineLeftV(5.0f),
      lastGyroBiasMs(0),
      gyroBiasSamples(0),
      lastTickMs(0)
{
}

void FSM::fsmInit() {
    Serial.begin(115200);

    perception.init();
    radio.init(115200);
    motors.enable();
    motors.Stop(true);
    pinMode(Config::PIN_FAN, OUTPUT);
    setFanDuty(0);

    state = State::INITIALISING;
    lastTickMs = millis();
    lastGyroBiasMs = lastTickMs;
    Serial.println(F("# Initialising combined light homing + obstacle avoidance FSM"));
}

void FSM::fsmUpdate() {
    unsigned long now = millis();
    if (now - lastTickMs < Config::FSM_TICK_MS) return;
    lastTickMs = now;

    cacheSensors();

    switch (state) {
        case State::INITIALISING:   initialising();        break;
        case State::SCANNING:       scanForLight();        break;
        case State::ANALYZING:      analyzeLightScan();    break;
        case State::COARSE_ALIGN:   coarseAlignToLight();  break;
        case State::APPROACH_LIGHT: approachLight();       break;
        case State::FINE_ALIGN:     fineAlignToLight();    break;
        case State::ALIGNED:        aligned();             break;
        case State::REVERSE_ESCAPE: reverseEscape();       break;
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

    lightCloseRightRaw = analogRead(Config::PIN_LIGHT_CLOSE_RIGHT);
    lightLongRightRaw  = analogRead(Config::PIN_LIGHT_LONG_RIGHT);
    lightLongLeftRaw   = analogRead(Config::PIN_LIGHT_LONG_LEFT);
    lightCloseLeftRaw  = analogRead(Config::PIN_LIGHT_CLOSE_LEFT);
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
        resetLightScan();
        if (Config::LIGHT_DIRECT_APPROACH_TEST_MODE) {
            state = State::APPROACH_LIGHT;
            Serial.println(F("# Direct approach test mode: face the robot at the light"));
            Serial.println(F("# off,obs_cm,side,dir,guard,vx,vy,wz,frontA_mm,frontB_mm,left_mm,right_mm,sonar_cm,A4_V,A4_raw,A5_raw,A6_raw,A7_V,A7_raw"));
        } else {
            state = State::SCANNING;
            Serial.println(F("# Heading_deg,A4_V,A5_V,A6_V,A7_V"));
        }
    }
}

void FSM::resetLightScan()
{
    lightSampleCount = 0;
    lastLightLogMs = 0;
    coarseAlignStartMs = 0;
    fineAlignStartMs = 0;
    targetHeading = 0.0f;
    avoidDirection = 0;
    avoidSuppressUntilMs = 0;
    sideGuardBlockedSide = 0;
    sideGuardHoldUntilMs = 0;
    blindSpotLeftUntilMs = 0;
    blindSpotRightUntilMs = 0;
    reverseEscapeUntilMs = 0;
    recentObstacleHits = 0;
    lastObstacleHitMs = 0;
    alignedAtMs = 0;
    lastExtinguishLogMs = 0;
    extinguishBaselineRightV = 5.0f;
    extinguishBaselineLeftV = 5.0f;
    lastVx = lastVy = lastWz = 0;
    lastFireOffset = 0.0f;
    lastObstacleCm = 80.0f;
    lastSidePreference = 0.0f;
}

void FSM::scanForLight()
{
    motors.RotateCCW(Config::LIGHT_SCAN_SPIN_SPEED);

    unsigned long now = millis();
    if (now - lastLightLogMs < Config::LIGHT_LOG_INTERVAL_MS) return;
    lastLightLogMs = now;

    float heading = motors.getHeading();
    float vA4 = adcToVolts(lightCloseRightRaw);
    float vA5 = adcToVolts(lightLongRightRaw);
    float vA6 = adcToVolts(lightLongLeftRaw);
    float vA7 = adcToVolts(lightCloseLeftRaw);

    Serial.print(heading, 1); Serial.print(',');
    Serial.print(vA4, 3);     Serial.print(',');
    Serial.print(vA5, 3);     Serial.print(',');
    Serial.print(vA6, 3);     Serial.print(',');
    Serial.println(vA7, 3);
    radio.sendLightData(heading, vA4, vA5, vA6, vA7);

    if (lightSampleCount < MAX_LIGHT_SAMPLES) {
        lightSamples[lightSampleCount++] = {
            heading,
            lightLongRightRaw,
            lightLongLeftRaw
        };
    }

    if (heading >= Config::LIGHT_SCAN_TARGET_DEG) {
        motors.Stop(true);
        Serial.println(F("# Scan done. Analyzing..."));
        state = State::ANALYZING;
    }
}

float FSM::analyzeScans()
{
    float minRightV = 5.0f;
    float minLeftV = 5.0f;
    float bestRightHeading = -1.0f;
    float bestLeftHeading = -1.0f;

    for (int i = 0; i < lightSampleCount; i++) {
        float rightV = adcToVolts(lightSamples[i].longRightRaw);
        float leftV  = adcToVolts(lightSamples[i].longLeftRaw);

        if (rightV < Config::LIGHT_DETECT_THRESHOLD_V && rightV < minRightV) {
            minRightV = rightV;
            bestRightHeading = lightSamples[i].heading;
        }

        if (leftV < Config::LIGHT_DETECT_THRESHOLD_V && leftV < minLeftV) {
            minLeftV = leftV;
            bestLeftHeading = lightSamples[i].heading;
        }
    }

    bool foundRight = bestRightHeading >= 0.0f;
    bool foundLeft = bestLeftHeading >= 0.0f;

    if (!foundRight && !foundLeft) {
        Serial.println(F("# No light source detected."));
        return -1.0f;
    }

    float result;
    if (foundRight && foundLeft) result = (bestRightHeading + bestLeftHeading) * 0.5f;
    else if (foundRight)         result = bestRightHeading;
    else                         result = bestLeftHeading;

    Serial.print(F("# A5 peak: ")); Serial.print(bestRightHeading, 1);
    Serial.print(F(" deg  A6 peak: ")); Serial.print(bestLeftHeading, 1);
    Serial.print(F(" deg  => target: ")); Serial.print(result, 1);
    Serial.println(F(" deg"));

    return result;
}

void FSM::setFanDuty(uint8_t duty) const
{
    analogWrite(Config::PIN_FAN, fanOutputForDuty(duty));
}

void FSM::analyzeLightScan()
{
    targetHeading = analyzeScans();
    if (targetHeading < 0.0f) {
        state = State::ALIGNED;
        return;
    }

    float current = motors.getHeading();
    targetHeading = current + normalizeAngle(targetHeading - current);
    motors.latchHeading();
    motors.setTargetHeading(targetHeading);
    coarseAlignStartMs = 0;

    Serial.print(F("# Rotating to "));
    Serial.println(targetHeading, 1);
    state = State::COARSE_ALIGN;
}

void FSM::coarseAlignToLight()
{
    if (coarseAlignStartMs == 0) coarseAlignStartMs = millis();

    motors.headingCorrection();

    float error = normalizeAngle(targetHeading - motors.getHeading());
    bool settled = fabsf(error) < Config::LIGHT_HEADING_TOLERANCE_DEG;
    bool timedOut = (millis() - coarseAlignStartMs) > Config::LIGHT_COARSE_ALIGN_TIMEOUT_MS;

    if (settled || timedOut) {
        motors.Stop(true);
        Serial.println(timedOut ? F("# Coarse align timeout; fuzzy approach will correct")
                                : F("# Coarse aligned. Approaching..."));
        lastLightLogMs = 0;
        avoidDirection = 0;
        state = State::APPROACH_LIGHT;
        return;
    }

    int speed = (fabsf(error) > 20.0f)
              ? Config::LIGHT_SCAN_SPIN_SPEED
              : Config::LIGHT_FINE_ALIGN_SPEED + 170;

    if (error > 0.0f) motors.RotateCCW(speed);
    else              motors.RotateCW(speed);
}

float FSM::nearestForwardObstacleCm() const
{
    float nearestCm = 80.0f;

    if (validDistance(distSonar)) {
        nearestCm = minf(nearestCm, constrain(distSonar, 0.0f, 80.0f));
    }

    if (validDistance(distFrontB)) {
        nearestCm = minf(nearestCm, constrain(distFrontB / 10.0f, 0.0f, 80.0f));
    }

    // The medium front sensor saturates near 300 mm, so only let it pull the
    // fuzzy distance down once it is inside the normal avoidance threshold.
    if (validDistance(distFrontA) && distFrontA <= Config::OBSTACLE_CLEAR_MM) {
        nearestCm = minf(nearestCm, constrain(distFrontA / 10.0f, 0.0f, 80.0f));
    }

    return nearestCm;
}

float FSM::sideClearancePreference() const
{
    float leftScore = sideClearScore(distLeft, Config::FLC_SIDE_FULL_LEFT_MM);
    float rightScore = sideClearScore(distRight, Config::FLC_SIDE_FULL_RIGHT_MM);
    return constrain((leftScore - rightScore) * 500.0f, -500.0f, 500.0f);
}

bool FSM::obstacleRelevantForApproach(float obstacleCm) const
{
    return forwardObstacleDetected() ||
           obstacleCm <= Config::FLC_DIRECTION_SELECT_CM;
}

int FSM::selectAvoidDirectionForApproach(float obstacleCm)
{
    bool obstacleRelevant = obstacleRelevantForApproach(obstacleCm);

    if (!obstacleRelevant) {
        if (obstacleCm >= Config::FLC_AVOID_RELEASE_CM) {
            avoidDirection = 0;
        }
        return 0;
    }

    if (avoidDirection != 0) {
        bool latchedSideSafe = sideClearForDirection(avoidDirection);
        bool oppositeSideSafe = sideClearForDirection(-avoidDirection);
        float latchedClearance = cappedSideClearance(avoidDirection > 0 ? distLeft : distRight);
        float oppositeClearance = cappedSideClearance(avoidDirection > 0 ? distRight : distLeft);
        bool oppositeMeaningfullyBetter =
            oppositeSideSafe &&
            (!latchedSideSafe ||
             oppositeClearance >= latchedClearance + Config::SIDE_OPEN_SWITCH_MARGIN_MM);

        if (latchedSideSafe && !oppositeMeaningfullyBetter) {
            return avoidDirection;
        }
    }

    avoidDirection = chooseAvoidDirection();
    return avoidDirection;
}

void FSM::fuzzyApproach(float fireOffset, float obstacleCm, float sidePreference,
                        int selectedAvoidDirection, int &vx, int &vy, int &wz) const
{
    float muOffset[OFFSET_COUNT];
    float muDistance[DIST_COUNT];
    float muSide[SIDE_COUNT];

    // fireOffset = A6(left long raw) - A5(right long raw). Positive means the
    // right sensor sees more light, so positive wz turns right/CW on this robot.
    muOffset[OFFSET_NL] = trimf(fireOffset, -1023.0f, -1023.0f, -400.0f);
    muOffset[OFFSET_NM] = trimf(fireOffset,  -650.0f,  -325.0f,    0.0f);
    muOffset[OFFSET_ZE] = trimf(fireOffset,  -150.0f,     0.0f,  150.0f);
    muOffset[OFFSET_PM] = trimf(fireOffset,     0.0f,   325.0f,  650.0f);
    muOffset[OFFSET_PL] = trimf(fireOffset,   400.0f,  1023.0f, 1023.0f);

    muDistance[DIST_VERY_CLOSE] = trimf(obstacleCm,  0.0f,  0.0f, 15.0f);
    muDistance[DIST_CLOSE]      = trimf(obstacleCm,  8.0f, 20.0f, 32.0f);
    muDistance[DIST_MEDIUM]     = trimf(obstacleCm, 25.0f, 40.0f, 55.0f);
    muDistance[DIST_FAR]        = trimf(obstacleCm, 45.0f, 80.0f, 80.0f);

    // sidePreference positive = more clearance on the left; vy positive strafes left.
    muSide[SIDE_CLEAR_RIGHT] = trimf(sidePreference, -500.0f, -300.0f, -80.0f);
    muSide[SIDE_NEUTRAL]     = trimf(sidePreference, -150.0f,    0.0f, 150.0f);
    muSide[SIDE_CLEAR_LEFT]  = trimf(sidePreference,   80.0f,  300.0f, 500.0f);

    float turnStrengths[16];
    float turnValues[16];
    int turnCount = 0;

    addRule(turnStrengths, turnValues, turnCount,
            minf(muDistance[DIST_FAR], muOffset[OFFSET_PL]), Config::FLC_TURN_FAST);
    addRule(turnStrengths, turnValues, turnCount,
            minf(muDistance[DIST_FAR], muOffset[OFFSET_PM]), Config::FLC_TURN_MEDIUM);
    addRule(turnStrengths, turnValues, turnCount,
            minf(muDistance[DIST_FAR], muOffset[OFFSET_ZE]), 0.0f);
    addRule(turnStrengths, turnValues, turnCount,
            minf(muDistance[DIST_FAR], muOffset[OFFSET_NM]), -Config::FLC_TURN_MEDIUM);
    addRule(turnStrengths, turnValues, turnCount,
            minf(muDistance[DIST_FAR], muOffset[OFFSET_NL]), -Config::FLC_TURN_FAST);

    addRule(turnStrengths, turnValues, turnCount,
            minf(muDistance[DIST_MEDIUM], muOffset[OFFSET_PL]), Config::FLC_TURN_MEDIUM);
    addRule(turnStrengths, turnValues, turnCount,
            minf(muDistance[DIST_MEDIUM], muOffset[OFFSET_PM]), Config::FLC_TURN_SLOW);
    addRule(turnStrengths, turnValues, turnCount,
            minf(muDistance[DIST_MEDIUM], muOffset[OFFSET_ZE]), 0.0f);
    addRule(turnStrengths, turnValues, turnCount,
            minf(muDistance[DIST_MEDIUM], muOffset[OFFSET_NM]), -Config::FLC_TURN_SLOW);
    addRule(turnStrengths, turnValues, turnCount,
            minf(muDistance[DIST_MEDIUM], muOffset[OFFSET_NL]), -Config::FLC_TURN_MEDIUM);

    addRule(turnStrengths, turnValues, turnCount,
            minf(muDistance[DIST_CLOSE], muOffset[OFFSET_PL]), Config::FLC_TURN_SLOW);
    addRule(turnStrengths, turnValues, turnCount,
            minf(muDistance[DIST_CLOSE], muOffset[OFFSET_PM]), Config::FLC_TURN_SLOW * 0.5f);
    addRule(turnStrengths, turnValues, turnCount,
            minf(muDistance[DIST_CLOSE], muOffset[OFFSET_ZE]), 0.0f);
    addRule(turnStrengths, turnValues, turnCount,
            minf(muDistance[DIST_CLOSE], muOffset[OFFSET_NM]), -Config::FLC_TURN_SLOW * 0.5f);
    addRule(turnStrengths, turnValues, turnCount,
            minf(muDistance[DIST_CLOSE], muOffset[OFFSET_NL]), -Config::FLC_TURN_SLOW);
    addRule(turnStrengths, turnValues, turnCount,
            muDistance[DIST_VERY_CLOSE], 0.0f);

    float strafeStrengths[10];
    float strafeValues[10];
    int strafeCount = 0;
    float emergencyVy;

    if (sidePreference > 25.0f) emergencyVy = Config::FLC_STRAFE_FAST;
    else if (sidePreference < -25.0f) emergencyVy = -Config::FLC_STRAFE_FAST;
    else emergencyVy = selectedAvoidDirection * Config::FLC_STRAFE_FAST;

    addRule(strafeStrengths, strafeValues, strafeCount,
            minf(muDistance[DIST_MEDIUM], muSide[SIDE_CLEAR_LEFT]), Config::FLC_STRAFE_SLOW);
    addRule(strafeStrengths, strafeValues, strafeCount,
            minf(muDistance[DIST_MEDIUM], muSide[SIDE_CLEAR_RIGHT]), -Config::FLC_STRAFE_SLOW);
    addRule(strafeStrengths, strafeValues, strafeCount,
            minf(muDistance[DIST_MEDIUM], muSide[SIDE_NEUTRAL]), 0.0f);

    addRule(strafeStrengths, strafeValues, strafeCount,
            minf(muDistance[DIST_CLOSE], muSide[SIDE_CLEAR_LEFT]), Config::FLC_STRAFE_FAST);
    addRule(strafeStrengths, strafeValues, strafeCount,
            minf(muDistance[DIST_CLOSE], muSide[SIDE_CLEAR_RIGHT]), -Config::FLC_STRAFE_FAST);
    addRule(strafeStrengths, strafeValues, strafeCount,
            minf(muDistance[DIST_CLOSE], muSide[SIDE_NEUTRAL]),
            selectedAvoidDirection * Config::FLC_STRAFE_MEDIUM);

    addRule(strafeStrengths, strafeValues, strafeCount,
            minf(muDistance[DIST_VERY_CLOSE], muSide[SIDE_CLEAR_LEFT]), Config::FLC_STRAFE_FAST);
    addRule(strafeStrengths, strafeValues, strafeCount,
            minf(muDistance[DIST_VERY_CLOSE], muSide[SIDE_CLEAR_RIGHT]), -Config::FLC_STRAFE_FAST);
    addRule(strafeStrengths, strafeValues, strafeCount,
            minf(muDistance[DIST_VERY_CLOSE], muSide[SIDE_NEUTRAL]), emergencyVy);
    addRule(strafeStrengths, strafeValues, strafeCount,
            muDistance[DIST_FAR], 0.0f);

    float vxStrengths[DIST_COUNT] = {
        muDistance[DIST_VERY_CLOSE],
        muDistance[DIST_CLOSE],
        muDistance[DIST_MEDIUM],
        muDistance[DIST_FAR]
    };
    float vxValues[DIST_COUNT] = {
        0.0f,
        (float)Config::FLC_VX_CLOSE,
        (float)Config::FLC_VX_MEDIUM,
        (float)Config::FLC_VX_FAR
    };

    vx = constrain((int)defuzzify(vxStrengths, vxValues, DIST_COUNT),
                   0, Config::FLC_VX_FAR);
    vy = constrain((int)defuzzify(strafeStrengths, strafeValues, strafeCount),
                   -Config::FLC_STRAFE_FAST, Config::FLC_STRAFE_FAST);
    wz = constrain((int)defuzzify(turnStrengths, turnValues, turnCount),
                   -Config::FLC_TURN_FAST, Config::FLC_TURN_FAST);
}


void FSM::approachLight()
{
    float closeRightV = adcToVolts(lightCloseRightRaw);
    float closeLeftV  = adcToVolts(lightCloseLeftRaw);

    unsigned long now2 = millis();
    if (closeRightV <= Config::LIGHT_CLOSE_SUPPRESS_V ||
        closeLeftV  <= Config::LIGHT_CLOSE_SUPPRESS_V) {
        avoidSuppressUntilMs = now2 + Config::LIGHT_CLOSE_SUPPRESS_MS;
    }
    bool avoidanceSuppressed = now2 < avoidSuppressUntilMs;

    bool closeToLight = avoidanceSuppressed ||
                        closeRightV <= Config::LIGHT_AVOID_DISABLE_V ||
                        closeLeftV  <= Config::LIGHT_AVOID_DISABLE_V;

    if (closeRightV <= Config::LIGHT_STOP_VOLTAGE_V ||
        closeLeftV <= Config::LIGHT_STOP_VOLTAGE_V)
    {
        motors.Stop(true);
        Serial.print(F("# Close range triggered (A4="));
        Serial.print(closeRightV, 3);
        Serial.print(F(" A7="));
        Serial.print(closeLeftV, 3);
        Serial.println(F("). Fine aligning..."));
        fineAlignStartMs = 0;
        state = State::FINE_ALIGN;
        return;
    }

    float fireOffset = (float)lightLongLeftRaw - (float)lightLongRightRaw;
    float obstacleCm = nearestForwardObstacleCm();
    bool obstacleRelevant = obstacleRelevantForApproach(obstacleCm);
    int selectedAvoidDirection = selectAvoidDirectionForApproach(obstacleCm);
    if (closeToLight) {
        avoidDirection = 0;
        selectedAvoidDirection = 0;
    }

    bool avoidingObstacle = !closeToLight &&
                            selectedAvoidDirection != 0 &&
                            obstacleRelevant;
    bool blockedWithoutEscape = !closeToLight &&
                                obstacleRelevant &&
                                selectedAvoidDirection == 0;
    float sidePreference = (avoidingObstacle || blockedWithoutEscape)
                         ? sideClearancePreference()
                         : 0.0f;

    if (avoidingObstacle) {
        sidePreference = selectedAvoidDirection * 300.0f;
    }

    float controlObstacleCm = avoidingObstacle ? obstacleCm : 80.0f;

    int vx, vy, wz;
    fuzzyApproach(fireOffset, controlObstacleCm, sidePreference, selectedAvoidDirection,
                  vx, vy, wz);

    char guardCode = '0';
    if (blockedWithoutEscape) {
        recordObstacleHit();
        enterReverseEscape();
        return;
    } else if (avoidingObstacle) {
        if (abs(vy) < Config::FLC_MIN_ESCAPE_STRAFE) {
            vy = selectedAvoidDirection * Config::FLC_MIN_ESCAPE_STRAFE;
        }
        // wz left as fuzzy output so robot keeps facing the light while strafing
    } else if (vx < Config::FLC_MIN_HOMING_VX) {
        vx = Config::FLC_MIN_HOMING_VX;
    }

    bool leftTooClose  = validDistance(distLeft)  && distLeft  < Config::SIDE_STRAFE_TRIGGER_MM;
    bool rightTooClose = validDistance(distRight) && distRight < Config::SIDE_STRAFE_TRIGGER_MM;

    // Blind-spot latch: the diagonal front sensors see an obstacle just before it enters
    // the gap between them and the side sensors. Each time a sensor sees something within
    // BLIND_SPOT_WARN_MM, the latch is refreshed. The latch stays active for
    // BLIND_SPOT_LATCH_MS after the sensor last saw it, bridging the transition into the
    // blind spot. Suppressed when close to the light to avoid reacting to the fire pedestal.
    if (!closeToLight) {
        if (validDistance(distFrontA) && distFrontA < Config::BLIND_SPOT_WARN_MM)
            blindSpotLeftUntilMs  = now2 + Config::BLIND_SPOT_LATCH_MS;
        if (validDistance(distFrontB) && distFrontB < Config::BLIND_SPOT_WARN_MM)
            blindSpotRightUntilMs = now2 + Config::BLIND_SPOT_LATCH_MS;
    }
    if (now2 < blindSpotLeftUntilMs)  leftTooClose  = true;
    if (now2 < blindSpotRightUntilMs) rightTooClose = true;

    if (leftTooClose || rightTooClose) {
        if (leftTooClose && rightTooClose) {
            // Both sides triggered — if avoidance has a direction, use it rather than
            // zeroing vy and stalling. selectedAvoidDirection reflects which side has
            // more room so it's the best escape vector available.
            vy = (selectedAvoidDirection != 0)
                     ? selectedAvoidDirection * Config::SIDE_STRAFE_SPEED
                     : 0;
            guardCode = 'C';
        } else if (leftTooClose) {
            vy = -Config::SIDE_STRAFE_SPEED;
            guardCode = 'L';
        } else {
            vy = Config::SIDE_STRAFE_SPEED;
            guardCode = 'R';
        }
        // wz left as fuzzy output so robot keeps facing the light while strafing
    }

    motors.drive(vx, vy, wz);

    lastVx = vx;
    lastVy = vy;
    lastWz = wz;
    lastFireOffset = fireOffset;
    lastObstacleCm = obstacleCm;
    lastSidePreference = sidePreference;

    if (now2 - lastLightLogMs >= Config::LIGHT_LOG_INTERVAL_MS) {
        lastLightLogMs = now2;
        Serial.print(F("# approach off=")); Serial.print(fireOffset, 1);
        Serial.print(F(" obs_cm="));        Serial.print(obstacleCm, 1);
        Serial.print(F(" side="));          Serial.print(sidePreference, 1);
        Serial.print(F(" dir="));           Serial.print(selectedAvoidDirection);
        Serial.print(F(" guard="));         Serial.print(guardCode);
        Serial.print(F(" vx="));            Serial.print(vx);
        Serial.print(F(" vy="));            Serial.print(vy);
        Serial.print(F(" wz="));            Serial.print(wz);
        Serial.print(F(" frontA="));        Serial.print(distFrontA, 1);
        Serial.print(F(" frontB="));        Serial.print(distFrontB, 1);
        Serial.print(F(" left="));          Serial.print(distLeft, 1);
        Serial.print(F(" right="));         Serial.print(distRight, 1);
        Serial.print(F(" sonar="));         Serial.print(distSonar, 1);
        Serial.print(F(" A4="));            Serial.print(closeRightV, 3);
        Serial.print(F(" A4raw="));         Serial.print(lightCloseRightRaw);
        Serial.print(F(" A5raw="));         Serial.print(lightLongRightRaw);
        Serial.print(F(" A6raw="));         Serial.print(lightLongLeftRaw);
        Serial.print(F(" A7="));            Serial.print(closeLeftV, 3);
        Serial.print(F(" A7raw="));         Serial.println(lightCloseLeftRaw);
    }
}

void FSM::fineAlignToLight()
{
    if (fineAlignStartMs == 0) fineAlignStartMs = millis();

    float closeRightV = adcToVolts(lightCloseRightRaw);
    float closeLeftV = adcToVolts(lightCloseLeftRaw);
    float diff = closeRightV - closeLeftV;

    Serial.print(F("# fine A4="));
    Serial.print(closeRightV, 3);
    Serial.print(F(" A7="));
    Serial.print(closeLeftV, 3);
    Serial.print(F(" diff="));
    Serial.println(diff, 3);

    bool balanced = fabsf(diff) <= Config::LIGHT_FINE_ALIGN_TOLERANCE_V;
    bool timedOut = (millis() - fineAlignStartMs) > Config::LIGHT_FINE_ALIGN_TIMEOUT_MS;

    if (balanced || timedOut) {
        motors.Stop(true);
        Serial.println(timedOut ? F("# Fine align timeout; declaring ALIGNED") : F("# ALIGNED"));
        state = State::ALIGNED;
        return;
    }

    // diff < 0 → CW (turns right), diff > 0 → CCW (turns left).
    // Skip the pulse if rotating that way would swing into a nearby side obstacle.
    bool rightBlocked = validDistance(distRight) && distRight < Config::SIDE_CLEAR_MIN_MM;
    bool leftBlocked  = validDistance(distLeft)  && distLeft  < Config::SIDE_CLEAR_MIN_MM;
    bool wouldHitObstacle = (diff < 0.0f && rightBlocked) || (diff > 0.0f && leftBlocked);

    if (!wouldHitObstacle) {
        if (diff < 0.0f) motors.RotateCW(Config::LIGHT_FINE_ALIGN_SPEED);
        else             motors.RotateCCW(Config::LIGHT_FINE_ALIGN_SPEED);
        delay(20);
        motors.Stop(true);
        delay(30);
    }
}

void FSM::aligned()
{
    motors.Stop(true);

    if (lightsFound >= Config::FIRES_TO_EXTINGUISH) {
        setFanDuty(0);
        return;
    }

    setFanDuty(Config::FAN_FULL_DUTY);

    if (alignedAtMs == 0) {
        alignedAtMs = millis();
        lastExtinguishLogMs = 0;
        Serial.print(F("# Fire "));
        Serial.print(lightsFound + 1);
        Serial.println(F(" aligned. Fan on. Waiting for A5+A6 > 4V..."));
    }

    float longRightV = adcToVolts(lightLongRightRaw);
    float longLeftV  = adcToVolts(lightLongLeftRaw);
    unsigned long now = millis();

    if (now - lastExtinguishLogMs >= Config::LIGHT_EXTINGUISH_LOG_MS) {
        lastExtinguishLogMs = now;
        Serial.print(F("# extinguish fire="));
        Serial.print(lightsFound + 1);
        Serial.print(F(" A5="));
        Serial.print(longRightV, 3);
        Serial.print(F(" A6="));
        Serial.println(longLeftV, 3);
    }

    if (longRightV > Config::LIGHT_EXTINGUISHED_LONG_V &&
        longLeftV  > Config::LIGHT_EXTINGUISHED_LONG_V)
    {
        setFanDuty(0);
        lightsFound++;
        Serial.print(F("# Fire "));
        Serial.print(lightsFound);
        Serial.println(F(" extinguished (A5+A6 >4V). Fan off."));

        if (lightsFound >= Config::FIRES_TO_EXTINGUISH) {
            Serial.println(F("# All fires extinguished. Stopping."));
            return;
        }

        resetLightScan();
        motors.resetHeading();
        motors.latchHeading();
        Serial.println(F("# Scanning for next light..."));
        state = State::SCANNING;
    }
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

bool FSM::sideClearForDirection(int direction) const {
    if (direction > 0) {
        return validDistance(distLeft) &&
               distLeft >= Config::SIDE_ESCAPE_MIN_MM;
    }
    if (direction < 0) {
        return validDistance(distRight) &&
               distRight >= Config::SIDE_ESCAPE_MIN_MM;
    }
    return false;
}

int FSM::chooseAvoidDirection() const {
    bool frontABlocked = validDistance(distFrontA) &&
                         distFrontA <= Config::OBSTACLE_AVOID_MM;
    bool frontBBlocked = validDistance(distFrontB) &&
                         distFrontB <= Config::OBSTACLE_AVOID_MM;

    bool leftSafe = sideClearForDirection(1);
    bool rightSafe = sideClearForDirection(-1);
    float leftClearance = cappedSideClearance(distLeft);
    float rightClearance = cappedSideClearance(distRight);

    if (frontABlocked != frontBBlocked) {
        bool blockedSideIsLeft = frontABlocked == Config::FRONT_A_IS_LEFT;
        int awayFromBlockedSide = blockedSideIsLeft ? -1 : 1;
        int oppositeSide = -awayFromBlockedSide;

        if (sideClearForDirection(awayFromBlockedSide)) return awayFromBlockedSide;
        if (sideClearForDirection(oppositeSide)) return oppositeSide;
        return 0;
    }

    if (leftSafe && !rightSafe) return 1;
    if (rightSafe && !leftSafe) return -1;

    // Both safe OR both below minimum — always pick the more open side using raw
    // distances rather than stopping. If one side is genuinely more open, go there.
    if (validDistance(distLeft) && validDistance(distRight)) {
        return (distLeft >= distRight) ? 1 : -1;
    }

    // One or both sensors invalid — fall back to whichever side is known safe
    if (leftSafe)  return 1;
    if (rightSafe) return -1;
    return 0;  // truly no information on either side
}

// ─────────────────────────────────────────────────────────────────────────────
//  Reverse escape
// ─────────────────────────────────────────────────────────────────────────────

void FSM::recordObstacleHit()
{
    unsigned long now = millis();
    // lastObstacleHitMs == 0 means no previous hit has been recorded yet
    if (lastObstacleHitMs != 0 && now - lastObstacleHitMs < Config::OSCILLATION_WINDOW_MS) {
        recentObstacleHits++;
    } else {
        recentObstacleHits = 1;
    }
    lastObstacleHitMs = now;
}

void FSM::enterReverseEscape()
{
    bool oscillating = recentObstacleHits >= Config::OSCILLATION_TRIGGER_COUNT;

    unsigned long duration = oscillating ? Config::REVERSE_ESCAPE_LONG_MS
                                         : Config::REVERSE_ESCAPE_MS;
    reverseEscapeUntilMs = millis() + duration;

    // Oscillating: we've hit an obstacle from the same direction repeatedly.
    // Flip the latched avoid direction so we try the other side on the next approach.
    if (oscillating && avoidDirection != 0) {
        avoidDirection = -avoidDirection;
    }

    motors.Stop(true);
    motors.latchHeading();
    state = State::REVERSE_ESCAPE;

    Serial.print(F("# REVERSE_ESCAPE hits="));
    Serial.print(recentObstacleHits);
    Serial.print(F(" dur="));
    Serial.print(duration);
    Serial.println(oscillating ? F("ms [oscillating, dir flipped]") : F("ms"));
}

void FSM::reverseEscape()
{
    // Drive straight backward, heading correction keeps us from spinning.
    // Speed is intentionally conservative — we have no rear sensor.
    int wz = (int)motors.headingCorrection();
    motors.drive(-Config::REVERSE_ESCAPE_SPEED, 0, wz);

    if (millis() >= reverseEscapeUntilMs) {
        motors.Stop(true);
        motors.latchHeading();
        avoidDirection = 0;  // let selectAvoidDirection pick fresh on the next approach
        state = State::APPROACH_LIGHT;
        Serial.println(F("# Reverse escape done, resuming approach"));
    }
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
