#include "fsm.h"

FSM fsm; // single robot FSM instance

void setup() {
    fsm.fsmInit();
}

void loop() {
    fsm.fsmUpdate();
}
