#pragma once
#include <Servo.h>
#include "percepetion.h"
#include "movement.h"
#include "flc_approach.h"

enum State { CALIBRATE, SEARCH, APPROACH, EXTINGUISH, DONE };

class FSM {
public:
    FSM(percepetion* p, movement* m, FLCApproach* flc);
    void setup();
    void update();
    State getState() const { return state; }

private:
    percepetion* perc;
    movement*    mot;
    FLCApproach* flc;
    Servo        fan;
    State        state;
    unsigned long state_enter_ms;

    void enterState(State s);
    void runCalibrate();
    void runSearch();
    void runApproach();
    void runExtinguish();
};
