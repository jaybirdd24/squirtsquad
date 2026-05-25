#include <Arduino.h>
#include <SoftwareSerial.h>
#include "percepetion.h"
#include "movement.h"

// Tune SCAN_SPEED for ~30-45 deg/s; heading gate stops the scan at 360 deg.
static const int           SCAN_SPEED        = 100;
static const unsigned long SONAR_INTERVAL_MS = 60;
static const int           GYRO_SAMPLES      = 100;
static const int           GYRO_INTERVAL_MS  = 10;
static const float         SCAN_TARGET_DEG   = 360.0f;

enum class ScanState : uint8_t { SCANNING, DONE };

static SoftwareSerial radio(10, 11);    // HC-12: RX=10, TX=11
static percepetion    perception;
static movement       motors(&perception);

static ScanState     state     = ScanState::SCANNING;
static unsigned long lastSonar = 0;
static bool          doneSent  = false;

// Raw HC-SR04 read — no EMA filter, no spike rejection.
// Reuses PIN_US_TRIG / PIN_US_ECHO / US_MAX_PULSE_US from percepetion.h.
static float readSonarRawCm()
{
    digitalWrite(PIN_US_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_US_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_US_TRIG, LOW);
    unsigned long pulse = pulseIn(PIN_US_ECHO, HIGH, US_MAX_PULSE_US);
    return (pulse > 0) ? pulse / 58.0f : 0.0f;
}

void setup()
{
    radio.begin(115200);
    perception.init();
    motors.enable();

    // Accumulate gyro bias while stationary (~1 s)
    for (int i = 0; i < GYRO_SAMPLES; i++) {
        delay(GYRO_INTERVAL_MS);
        perception.update();
        perception.feedGyroBias();
    }
    perception.freezeGyroBias();
    motors.resetHeading();

    radio.println(F(">scan_status:0"));   // 0 = calibrated, awaiting start
    delay(2000);                          // pause — position the robot before scan begins
}

void loop()
{
    if (state == ScanState::DONE) {
        if (!doneSent) {
            motors.Stop(true);
            radio.println(F(">scan_done:1"));
            doneSent = true;
        }
        return;
    }

    // Spin CCW every tick; RotateCCW integrates gyroZ x dt into heading each call
    motors.RotateCCW(SCAN_SPEED);

    unsigned long now = millis();
    if (now - lastSonar < SONAR_INTERVAL_MS) return;
    lastSonar = now;

    // Refresh gyroZ for the next integration window.
    // perception.update() also fires an internal sonar shot (ignored here).
    perception.update();

    float heading = motors.getHeading();
    float dist    = readSonarRawCm();     // clean raw shot — no filter, no rejection

    if (dist > 0.0f) {
        float rad = heading * (PI / 180.0f);
        float x   = dist * cosf(rad);
        float y   = dist * sinf(rad);

        // Teleplot format: >name:value  or  >name:x:y
        radio.print(F(">heading:")); radio.println(heading, 2);
        radio.print(F(">dist:"));    radio.println(dist,    1);
        radio.print(F(">polar:"));   radio.print(heading,   2); radio.print(':'); radio.println(dist, 1);
        radio.print(F(">map:"));     radio.print(x,         1); radio.print(':'); radio.println(y,    1);
    }

    if (heading >= SCAN_TARGET_DEG) {
        state = ScanState::DONE;
    }
}
