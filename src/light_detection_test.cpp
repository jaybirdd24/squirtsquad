#include <Arduino.h>
#include "percepetion.h"
#include "movement.h"
#include "comms.h"

// A4 = close range right of fan   A5 = long range right of fan
// A6 = long range left of fan     A7 = close range left of fan


static const int           SPIN_SPEED       = 100;   // tune for ~30-45 deg/s
static const int           GYRO_SAMPLES     = 100;
static const int           GYRO_INTERVAL_MS = 10;
static const float         SCAN_TARGET_DEG  = 360.0f;
static const unsigned long LOG_INTERVAL_MS  = 50;

static percepetion perception;
static movement    motors(&perception);
static comms       _comms;

static bool          scanDone = false;
static unsigned long lastLog  = 0;

static float adcToVolts(int raw) { return raw * 5.0f / 1023.0f; }

void setup()
{
    perception.init();
    _comms.init(115200);
    motors.enable();
    motors.Stop(true);

    for (int i = 0; i < GYRO_SAMPLES; i++) {
        delay(GYRO_INTERVAL_MS);
        perception.update();
        perception.feedGyroBias();
    }
    perception.freezeGyroBias();
    motors.resetHeading();

    delay(2000);
}

void loop()
{
    if (scanDone) {
        motors.Stop(true);
        return;
    }

    motors.RotateCCW(SPIN_SPEED);

    unsigned long now = millis();
    if (now - lastLog < LOG_INTERVAL_MS) return;
    lastLog = now;

    perception.update();

    float heading = motors.getHeading();

    _comms.sendLightData(heading,
                         adcToVolts(analogRead(A4)),
                         adcToVolts(analogRead(A5)),
                         adcToVolts(analogRead(A6)),
                         adcToVolts(analogRead(A7)));

    if (heading >= SCAN_TARGET_DEG) {
        scanDone = true;
    }
}
