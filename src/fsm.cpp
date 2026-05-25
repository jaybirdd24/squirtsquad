#include "fsm.h"
#include <Arduino.h>

static const unsigned long CALIBRATE_MS  = 3000;  // gyro bias settling time
static const unsigned long EXTINGUISH_MS = 6000;  // fan run duration
static const int SEARCH_SPIN_SPEED       = 100;   // slow CW scan

FSM::FSM(percepetion* p, movement* m, FLCApproach* f)
    : perc(p), mot(m), flc(f), state(CALIBRATE), state_enter_ms(0) {}

void FSM::setup() {
    fan.attach(PIN_FAN);
    fan.writeMicroseconds(1000);  // fan off
    flc->setup();
    enterState(CALIBRATE);
}

void FSM::enterState(State s) {
    state = s;
    state_enter_ms = millis();

    // Always silence the fan on state entry, then re-enable in EXTINGUISH
    fan.writeMicroseconds(1000);

    switch (s) {
        case CALIBRATE:
            mot->Stop(true);
            break;
        case SEARCH:
            mot->resetHeading();
            break;
        case APPROACH:
            // No heading latch — FLC controls wz directly each tick
            break;
        case EXTINGUISH:
            mot->Stop(true);
            fan.writeMicroseconds(2000);  // full speed
            break;
        case DONE:
            mot->Stop(true);
            break;
    }

    Serial.print(F("State -> "));
    Serial.println((int)s);
}

void FSM::update() {
    perc->update();

    switch (state) {
        case CALIBRATE:  runCalibrate();  break;
        case SEARCH:     runSearch();     break;
        case APPROACH:   runApproach();   break;
        case EXTINGUISH: runExtinguish(); break;
        case DONE:       mot->Stop(true); break;
    }
}

void FSM::runCalibrate() {
    // Accumulate gyro bias while the robot sits still
    perc->feedGyroBias();
    if (millis() - state_enter_ms > CALIBRATE_MS) {
        perc->freezeGyroBias();
        enterState(SEARCH);
    }
}

void FSM::runSearch() {
    // Spin slowly CW to scan for fire
    mot->RotateCW(SEARCH_SPIN_SPEED);
    if (flc->fireVisible()) {
        enterState(APPROACH);
    }
}

void FSM::runApproach() {
    // FLC blends light-homing and obstacle avoidance each tick
    flc->update();

    if (flc->atFire()) {
        enterState(EXTINGUISH);
    } else if (!flc->fireVisible()) {
        enterState(SEARCH);
    }
}

void FSM::runExtinguish() {
    if (millis() - state_enter_ms > EXTINGUISH_MS) {
        fan.writeMicroseconds(1000);
        enterState(DONE);
    }
}
