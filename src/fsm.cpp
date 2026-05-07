#include "fsm.h"
#include "config.h"

// ── Constructor ───────────────────────────────────────────────────────────

FSM::FSM() : motors(&perception) {} // movement must receive a pointer to the sibling perception instance

// ── Public ────────────────────────────────────────────────────────────────

void FSM::fsmInit() {}

void FSM::fsmUpdate() {}

// ── Top-level state handlers ──────────────────────────────────────────────

void FSM::stateInitialising() {}

void FSM::stateRunning() {}

void FSM::stateStopped() {}

// ── Sensor cache ───────────────────────────────────────────────────────────

void FSM::cacheSensors() {}

// ── Behaviour pipeline ─────────────────────────────────────────────────────

void FSM::clearFlags() {}

void FSM::arbitrate() {}

void FSM::executeMotion(Motion cmd) {}

// ── Individual behaviours ──────────────────────────────────────────────────

void FSM::behaviourCruise() {}

void FSM::behaviourAvoid() {}

void FSM::behaviourSeekFire() {}

void FSM::behaviourExtinguish() {}

void FSM::behaviourEscape() {}

void FSM::behaviourHalt() {}

// ── Cruise sub-state handlers ──────────────────────────────────────────────

void FSM::cruiseFindWall() {}

void FSM::cruiseSweepForward() {}

void FSM::cruiseLaneStrafe() {}

// ── Debug ──────────────────────────────────────────────────────────────────

void FSM::debugPrint() {}
