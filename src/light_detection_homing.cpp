#include <Arduino.h>
#include "percepetion.h"
#include "movement.h"

// A4 = close range right of fan   A5 = long range right of fan
// A6 = long range left of fan     A7 = close range left of fan

// ── Tuning ────────────────────────────────────────────────────────────────────
static const int           SPIN_SPEED         = 200;
static const int           GYRO_SAMPLES       = 100;
static const int           GYRO_INTERVAL_MS   = 10;
static const float         SCAN_TARGET_DEG    = 360.0f;
static const unsigned long LOG_INTERVAL_MS    = 50;

static const float DETECT_THRESHOLD_V = 4.0f;   // long range dip = light detected
static const float CLUSTER_GAP_DEG   = 30.0f;   // gap > this = new cluster
static const float HEADING_TOLERANCE  = 3.0f;    // deg, close enough when rotating
static const float STOP_DISTANCE_CM   = 15.0f;   // ultrasonic stop threshold (tune this)
static const int   APPROACH_SPEED     = 150;      // forward drive speed
static const int   STEER_GAIN         = 50;       // volts diff → drive correction units
static const int   MAX_STEER          = 100;      // max steering correction
static const float ALIGN_TOLERANCE_V  = 0.05f;   // close range balance tolerance
static const int   ALIGN_SPEED        = 40;       // fine alignment rotation speed
// ─────────────────────────────────────────────────────────────────────────────

static const int MAX_SAMPLES = 150;

struct Sample {
    float    heading;
    uint16_t a5, a6;  // long range only needed for coarse analysis
};

static percepetion perception;
static movement    motors(&perception);

static Sample        samples[MAX_SAMPLES];
static int           sampleCount      = 0;
static unsigned long lastLog          = 0;
static float         targetHeading    = 0.0f;
static unsigned long coarseAlignStart = 0;

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
    struct Cluster { float minVoltage; float bestHeading; };

    static const int MAX_CLUSTERS = 2;
    Cluster clusters[MAX_CLUSTERS];
    int clusterCount = 0;

    bool  inDip        = false;
    float clusterMinV  = 5.0f;
    float clusterBestH = 0.0f;
    float lastDipHdg   = -999.0f;

    for (int i = 0; i < sampleCount; i++) {
        float minV    = adcToVolts(min(samples[i].a5, samples[i].a6));
        bool  detected = (minV < DETECT_THRESHOLD_V);

        if (detected) {
            bool newCluster = !inDip || (samples[i].heading - lastDipHdg > CLUSTER_GAP_DEG);
            if (newCluster) {
                if (inDip && clusterCount < MAX_CLUSTERS) {
                    clusters[clusterCount++] = { clusterMinV, clusterBestH };
                }
                clusterMinV  = minV;
                clusterBestH = samples[i].heading;
                inDip        = true;
            }
            if (minV < clusterMinV) {
                clusterMinV  = minV;
                clusterBestH = samples[i].heading;
            }
            lastDipHdg = samples[i].heading;
        } else if (inDip) {
            if (clusterCount < MAX_CLUSTERS) {
                clusters[clusterCount++] = { clusterMinV, clusterBestH };
            }
            inDip = false;
        }
    }
    if (inDip && clusterCount < MAX_CLUSTERS) {
        clusters[clusterCount++] = { clusterMinV, clusterBestH };
    }

    if (clusterCount == 0) {
        Serial.println(F("# No light source detected."));
        return -1.0f;
    }

    int best = 0;
    for (int i = 1; i < clusterCount; i++) {
        if (clusters[i].minVoltage < clusters[best].minVoltage) best = i;
    }

    Serial.print(F("# Found "));
    Serial.print(clusterCount);
    Serial.print(F(" source(s). Closest at "));
    Serial.print(clusters[best].bestHeading, 1);
    Serial.println(F(" deg"));

    return clusters[best].bestHeading;
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
        lastLog = 0;
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

    float dist = perception.getUltrasonicCm();
    if (dist > 0 && dist < STOP_DISTANCE_CM) {
        motors.Stop(true);
        Serial.print(F("# Obstacle at "));
        Serial.print(dist, 1);
        Serial.println(F(" cm. Fine aligning..."));
        state = FINE_ALIGN;
        return;
    }

    // A5 = long range right, A6 = long range left
    // diff < 0 → A5 lower → more light on right → steer right (negative wz = CW)
    float vA5  = adcToVolts(analogRead(A5));
    float vA6  = adcToVolts(analogRead(A6));
    float diff = vA5 - vA6;
    int   corr = constrain((int)(diff * STEER_GAIN), -MAX_STEER, MAX_STEER);

    motors.drive(APPROACH_SPEED, 0, -corr);  // negative: diff<0 → steer right (CW = negative wz)

    unsigned long now = millis();
    if (now - lastLog >= LOG_INTERVAL_MS) {
        lastLog = now;
        Serial.print(F("# approach dist="));
        Serial.print(dist, 1);
        Serial.print(F(" A5="));
        Serial.print(vA5, 3);
        Serial.print(F(" A6="));
        Serial.print(vA6, 3);
        Serial.print(F(" corr="));
        Serial.println(corr);
    }
}

// ── Fine align: nudge using close range A4/A7 differential ───────────────────
static void runFineAlign()
{
    perception.update();

    float vA4 = adcToVolts(analogRead(A4));
    float vA7 = adcToVolts(analogRead(A7));
    float diff = vA4 - vA7;  // negative = A4 lower = more light right = rotate CW

    Serial.print(F("# fine A4="));
    Serial.print(vA4, 3);
    Serial.print(F(" A7="));
    Serial.print(vA7, 3);
    Serial.print(F(" diff="));
    Serial.println(diff, 3);

    if (abs(diff) <= ALIGN_TOLERANCE_V) {
        motors.Stop(true);
        Serial.println(F("# ALIGNED"));
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
