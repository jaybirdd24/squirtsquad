#include <Arduino.h>
#include "percepetion.h"
#include "movement.h"
#include "fsm.h"

static percepetion perc;
static movement    mot(&perc);
static FSM         fsm(&perc, &mot);

void setup()
{
    Serial.begin(115200);

    if (!perc.init()) {
        Serial.println("IMU init failed — halting");
        while (true) {}
    }

    mot.enable();
    mot.latchHeading();

    // Collect gyro bias for ~1 second while stationary
    unsigned long t0 = millis();
    while (millis() - t0 < 1000) {
        perc.update();
        perc.feedGyroBias();
    }
    perc.freezeGyroBias();
    mot.resetHeading();

    fsm.init();
}

void loop()
{
    fsm.update();
}
