#include <Arduino.h>
#include "percepetion.h"
#include "movement.h"
#include "comms.h"

// A4 = close range right of fan   A5 = long range right of fan
// A6 = long range left of fan     A7 = close range left of fan

// ── Tuning ────────────────────────────────────────────────────────────────────
static const int           SPIN_SPEED         = 200;
static const int           GYRO_SAMPLES       = 100;
static const int           GYRO_INTERVAL_MS   = 10;
static const float         SCAN_TARGET_DEG    = 360.0f;
static const unsigned long LOG_INTERVAL_MS    = 50;

static const float DETECT_THRESHOLD_V = 4.5f;   // long range dip = light detected
static const float CLUSTER_GAP_DEG   = 30.0f;   // gap > this = new cluster
static const float HEADING_TOLERANCE  = 3.0f;    // deg, close enough when rotating
static const float STOP_VOLTAGE_V     = 1.0f;    // close range sensor threshold to stop approach
static const int   APPROACH_SPEED     = 150;      // forward drive speed
static const float STEER_KP           = 200.0f;   // proportional gain (tune this)
static const float STEER_KI           = 5.0f;    // integral gain (tune this)
static const float STEER_I_MAX        = 1.0f;    // anti-windup clamp (volts·s)
static const int   MAX_STEER          = 200;      // max steering correction
static const float ALIGN_TOLERANCE_V  = 0.25f;   // close range balance tolerance (tune — sensors have ~0.196V natural offset)
static const int   ALIGN_SPEED        = 100;      // fine alignment rotation speed
// ─────────────────────────────────────────────────────────────────────────────

static const int MAX_SAMPLES = 150;

struct Sample {
    float    heading;
    uint16_t a5, a6;  // long range only needed for coarse analysis
};

static percepetion perception;
static movement    motors(&perception);
static comms       _comms;

static Sample        samples[MAX_SAMPLES];
static int           sampleCount      = 0;
static unsigned long lastLog          = 0;
static float         targetHeading    = 0.0f;
static unsigned long coarseAlignStart = 0;
static unsigned long fineAlignStart   = 0;
static float         steerIntegral    = 0.0f;
static unsigned long lastSteerTime    = 0;

static const unsigned long COARSE_ALIGN_TIMEOUT_MS = 4000;

static float adcToVolts(int raw) { return raw * 5.0f / 1023.0f; }

enum State { SCANNING, ANALYZING, COARSE_ALIGN, APPROACH, FINE_ALIGN, ALIGNED };
static State state = SCANNING;

// ── 360 scan (long range sensors only for analysis, all 4 logged) ─────────────
static void runScan()
{
    motors.RotateCCW(SPIN_SPEED);

    unsigned long now = millis();
    if (now - lastLog < LOG_INTERVAL_MS) return;
    lastLog = now;

    perception.update();
    float heading = motors.getHeading();

    uint16_t rA4 = analogRead(A4);
    uint16_t rA5 = analogRead(A5);
    uint16_t rA6 = analogRead(A6);
    uint16_t rA7 = analogRead(A7);

    Serial.print(heading, 1);           Serial.print(',');
    Serial.print(adcToVolts(rA4), 3);   Serial.print(',');
    Serial.print(adcToVolts(rA5), 3);   Serial.print(',');
    Serial.print(adcToVolts(rA6), 3);   Serial.print(',');
    Serial.println(adcToVolts(rA7), 3);

    if (sampleCount < MAX_SAMPLES) {
        samples[sampleCount++] = { heading, rA5, rA6 };
    }

    if (heading >= SCAN_TARGET_DEG) {
        motors.Stop(true);
        Serial.println(F("# Scan done. Analyzing..."));
        state = ANALYZING;
    }
}

// ── Cluster analysis on long range sensors → coarse heading ───────────────────
static float analyzeScans()
{
    // Find the heading of the lowest-voltage sample for each long-range sensor.
    // Using the voltage minimum rather than dip-window midpoints avoids the
    // failure mode where a loose threshold keeps the dip "open" for most of the
    // scan, making the midpoint land far from the actual light peak.
    float minV5 = 5.0f, bestH5 = -1.0f;
    float minV6 = 5.0f, bestH6 = -1.0f;

    for (int i = 0; i < sampleCount; i++) {
        float v5 = adcToVolts(samples[i].a5);
        float v6 = adcToVolts(samples[i].a6);

        if (v5 < DETECT_THRESHOLD_V && v5 < minV5) { minV5 = v5; bestH5 = samples[i].heading; }
        if (v6 < DETECT_THRESHOLD_V && v6 < minV6) { minV6 = v6; bestH6 = samples[i].heading; }
    }

    bool found5 = (bestH5 >= 0.0f);
    bool found6 = (bestH6 >= 0.0f);

    if (!found5 && !found6) {
        Serial.println(F("# No light source detected."));
        return -1.0f;
    }

    float result;
    if (found5 && found6) result = (bestH5 + bestH6) * 0.5f;
    else if (found5)      result = bestH5;
    else                  result = bestH6;

    Serial.print(F("# a5 peak: ")); Serial.print(bestH5, 1);
    Serial.print(F("°  a6 peak: ")); Serial.print(bestH6, 1);
    Serial.print(F("°  => target: ")); Serial.print(result, 1);
    Serial.println(F("°"));

    return result;
}

// ── Coarse align: rotate to scan heading, timeout if gyro drift prevents settle ─
static void runCoarseAlign()
{
    if (coarseAlignStart == 0) coarseAlignStart = millis();

    perception.update();
    motors.headingCorrection();  // integrates gyro → keeps getHeading() current

    float err = targetHeading - motors.getHeading();
    while (err >  180) err -= 360;
    while (err < -180) err += 360;

    bool settled  = abs(err) < HEADING_TOLERANCE;
    bool timedOut = (millis() - coarseAlignStart) > COARSE_ALIGN_TIMEOUT_MS;

    if (settled || timedOut) {
        motors.Stop(true);
        if (timedOut) Serial.println(F("# Coarse align timeout — approach steering will correct"));
        else          Serial.println(F("# Coarse aligned. Approaching..."));
        lastLog       = 0;
        steerIntegral = 0.0f;
        lastSteerTime = 0;
        state = APPROACH;
        return;
    }

    int speed = (abs(err) > 20) ? SPIN_SPEED / 2 : ALIGN_SPEED;
    if (err > 0) motors.RotateCCW(speed);
    else         motors.RotateCW(speed);
}

// ── Approach: drive forward, steer with long range differential, stop at sonar ─
static void runApproach()
{
    perception.update();

    float vA4close = adcToVolts(analogRead(A4));
    float vA7close = adcToVolts(analogRead(A7));
    if (vA4close <= STOP_VOLTAGE_V || vA7close <= STOP_VOLTAGE_V) {
        motors.Stop(true);
        Serial.print(F("# Close range triggered (A4="));
        Serial.print(vA4close, 3);
        Serial.print(F(" A7="));
        Serial.print(vA7close, 3);
        Serial.println(F("). Fine aligning..."));
        fineAlignStart = 0;
        state = FINE_ALIGN;
        return;
    }

    // PI steering: error = rightV - leftV (A5 - A6)
    // positive error → right voltage higher → turn right (positive wz in drive = CCW, so negate)
    float vA5  = adcToVolts(analogRead(A5));
    float vA6  = adcToVolts(analogRead(A6));
    float error = -vA5 +vA6;

    unsigned long now = millis();
    float dt = (lastSteerTime == 0) ? 0.05f : (now - lastSteerTime) / 1000.0f;
    lastSteerTime = now;

    steerIntegral = constrain(steerIntegral + error * dt, -STEER_I_MAX, STEER_I_MAX);

    float piOut = STEER_KP * error + STEER_KI * steerIntegral;
    int   wz    = constrain((int)piOut, -MAX_STEER, MAX_STEER);

    motors.drive(APPROACH_SPEED, 0, wz);

    if (now - lastLog >= LOG_INTERVAL_MS) {
        lastLog = now;
        Serial.print(F("# A4="));
        Serial.print(vA4close, 3);
        Serial.print(F(" A7="));
        Serial.print(vA7close, 3);
        Serial.print(F(" err="));
        Serial.print(error, 3);
        Serial.print(F(" I="));
        Serial.print(steerIntegral, 3);
        Serial.print(F(" wz="));
        Serial.println(wz);

    }
}

// ── Fine align: nudge using close range A4/A7 differential ───────────────────
static void runFineAlign()
{
    if (fineAlignStart == 0) fineAlignStart = millis();

    perception.update();

    float vA4 = adcToVolts(analogRead(A4));
    float vA7 = adcToVolts(analogRead(A7));
    float diff = vA4 - vA7;

    Serial.print(F("# fine A4="));
    Serial.print(vA4, 3);
    Serial.print(F(" A7="));
    Serial.print(vA7, 3);
    Serial.print(F(" diff="));
    Serial.println(diff, 3);

    bool balanced = abs(diff) <= ALIGN_TOLERANCE_V;
    bool timedOut = (millis() - fineAlignStart) > 5000;

    if (balanced || timedOut) {
        motors.Stop(true);
        Serial.println(timedOut ? F("# Fine align timeout — declaring ALIGNED") : F("# ALIGNED"));
        state = ALIGNED;
        return;
    }

    if (diff < 0) motors.RotateCW(ALIGN_SPEED);
    else          motors.RotateCCW(ALIGN_SPEED);
    delay(20);
    motors.Stop(true);
    delay(30);
}

// ─────────────────────────────────────────────────────────────────────────────

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println(F("# Initialising..."));
    _comms.init(9600);
    perception.init();
    motors.enable();

    Serial.println(F("# Calibrating gyro..."));
    for (int i = 0; i < GYRO_SAMPLES; i++) {
        delay(GYRO_INTERVAL_MS);
        perception.update();
        perception.feedGyroBias();
    }
    perception.freezeGyroBias();
    motors.resetHeading();

    Serial.println(F("# Starting scan in 2s — stand clear"));
    delay(2000);
    Serial.println(F("# Heading_deg,A4_V,A5_V,A6_V,A7_V"));
}

void loop()
{
    switch (state) {
        case SCANNING:
            runScan();
            break;

        case ANALYZING:
            targetHeading = analyzeScans();
            if (targetHeading < 0.0f) {
                state = ALIGNED;
            } else {
                float current = motors.getHeading();
                float diff    = targetHeading - current;
                while (diff >  180) diff -= 360;
                while (diff < -180) diff += 360;
                targetHeading = current + diff;
                motors.setTargetHeading(targetHeading);
                motors.latchHeading();
                Serial.print(F("# Rotating to "));
                Serial.println(targetHeading, 1);
                state = COARSE_ALIGN;
            }
            break;

        case COARSE_ALIGN:
            runCoarseAlign();
            break;

        case APPROACH:
            runApproach();
            break;

        case FINE_ALIGN:
            runFineAlign();
            break;

        case ALIGNED:
            motors.Stop(true);
            break;
    }
}
