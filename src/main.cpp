#include <Arduino.h>
#include "percepetion.h"
#include "movement.h"
#include "flc_approach.h"
#include "fsm.h"

static percepetion perception;
static movement    mover(&perception);
static FLCApproach flc(&perception, &mover);
static FSM         fsm(&perception, &mover, &flc);

void setup() {
    Serial.begin(115200);
    while (!perception.init()) {
        Serial.println(F("IMU init failed — retrying"));
        delay(500);
    }
    mover.enable();
    fsm.setup();
    Serial.println(F("Ready"));
}

void loop() {
    fsm.update();
}
