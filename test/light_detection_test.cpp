#include <Arduino.h>
#include "percepetion.h"
#include "movement.h"

// A4 = close range right of fan   A5 = long range right of fan
// A6 = long range left of fan     A7 = close range left of fan

static const int           SPIN_SPEED       = 100;   // tune for ~30-45 deg/s
static const int           GYRO_SAMPLES     = 100;
static const int           GYRO_INTERVAL_MS = 10;
static const float         SCAN_TARGET_DEG  = 360.0f;
static const unsigned long LOG_INTERVAL_MS  = 50;

static percepetion perception;
static movement    motors(&perception);

static bool          scanDone    = false;
static bool          donePrinted = false;
static unsigned long lastLog     = 0;

static float adcToVolts(int raw) { return raw * 5.0f / 1023.0f; }

void setup()
{
    Serial.begin(115200);
    while (!Serial) {}

    perception.init();
    motors.enable();

    for (int i = 0; i < GYRO_SAMPLES; i++) {
        delay(GYRO_INTERVAL_MS);
        perception.update();
        perception.feedGyroBias();
    }
    perception.freezeGyroBias();
    motors.resetHeading();

    Serial.println(F("# Light Detection 360 Scan — position robot then stand clear"));
    delay(2000);

    Serial.println(F("Heading_deg,A4_raw,A4_V,A5_raw,A5_V,A6_raw,A6_V,A7_raw,A7_V"));
}

void loop()
{
    if (scanDone) {
        motors.Stop(true);
        if (!donePrinted) {
            Serial.println(F("# SCAN COMPLETE"));
            donePrinted = true;
        }
        return;
    }

    motors.RotateCCW(SPIN_SPEED);

    unsigned long now = millis();
    if (now - lastLog < LOG_INTERVAL_MS) return;
    lastLog = now;

    perception.update();

    float heading = motors.getHeading();

    int a4 = analogRead(A4);
    int a5 = analogRead(A5);
    int a6 = analogRead(A6);
    int a7 = analogRead(A7);

    Serial.print(heading, 1);                                            Serial.print(',');
    Serial.print(a4); Serial.print(','); Serial.print(adcToVolts(a4), 3); Serial.print(',');
    Serial.print(a5); Serial.print(','); Serial.print(adcToVolts(a5), 3); Serial.print(',');
    Serial.print(a6); Serial.print(','); Serial.print(adcToVolts(a6), 3); Serial.print(',');
    Serial.print(a7); Serial.print(','); Serial.println(adcToVolts(a7), 3);

    if (heading >= SCAN_TARGET_DEG) {
        scanDone = true;
    }
}
