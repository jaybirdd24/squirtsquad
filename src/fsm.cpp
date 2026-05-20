#include "fsm.h"

FSM::FSM(percepetion *perc, movement *mot)
    : perc(perc), mot(mot),
      state(State::SCAN), sourcesHandled(0),
      scanInitialized(false), peakHeading(0.0f), peakBrightness(0),
      alignInitialized(false), alignTarget(0.0f),
      filteredDiff(0.0f), approachPeakBright(0),
      dimStartMs(0), dimTimerActive(false)
{}

void FSM::init()
{
    pinMode(PIN_FAN, OUTPUT);
    digitalWrite(PIN_FAN, LOW);
}

// ── Sensor helpers ────────────────────────────────────────────────

int FSM::longBrightness()
{
    return (1023 - perc->getPTLongLeft()) + (1023 - perc->getPTLongRight());
}

int FSM::shortBrightness()
{
    return (1023 - perc->getPTCloseLeft()) + (1023 - perc->getPTCloseRight());
}

int FSM::totalBrightness()
{
    return longBrightness() + shortBrightness();
}

int FSM::minShortADC()
{
    return min(perc->getPTCloseLeft(), perc->getPTCloseRight());
}

// ── Main update ───────────────────────────────────────────────────

void FSM::update()
{
    perc->update();

    // ── Obstacle avoidance hook ───────────────────────────────────
    // TODO (teammate): check perc->isObstacleTooClose(threshold_mm) here
    // and override motor commands / pause FSM as needed.

    switch (state) {
        case State::SCAN:     doScan();        break;
        case State::ALIGN:    doAlign();       break;
        case State::APPROACH: doApproach();    break;
        case State::FAN_ON:   doFanOn();       break;
        case State::DONE:     mot->Stop(true); break;
    }
}

// ── SCAN: spin 360° CCW, print all PT readings, record peak ──────

void FSM::doScan()
{
    if (!scanInitialized) {
        mot->resetHeading();
        peakBrightness = 0;
        peakHeading    = 0.0f;
        scanInitialized = true;
    }

    float h = mot->getHeading();  // increases as robot rotates CCW

    // Read all 4 sensors
    int longLeft   = perc->getPTLongLeft();
    int longRight  = perc->getPTLongRight();
    int closeLeft  = perc->getPTCloseLeft();
    int closeRight = perc->getPTCloseRight();

    // Use long-range pair for scan (better at distance)
    int b = (1023 - longLeft) + (1023 - longRight);
    if (b > peakBrightness) {
        peakBrightness = b;
        peakHeading    = h;
    }

    Serial.print("H:"); Serial.print(h, 1);
    Serial.print("  LongL:"); Serial.print(longLeft);
    Serial.print("  LongR:"); Serial.print(longRight);
    Serial.print("  CloseL:"); Serial.print(closeLeft);
    Serial.print("  CloseR:"); Serial.println(closeRight);

    if (h >= 360.0f) {
        mot->Stop();
        scanInitialized  = false;
        alignInitialized = false;

        // Shortest-path rotation to face peak
        alignTarget = peakHeading;
        if (alignTarget > 180.0f) alignTarget -= 360.0f;  // go CW instead

        state = State::ALIGN;
        Serial.print("Scan done — peak at heading ");
        Serial.print(peakHeading, 1);
        Serial.print("deg, rotating ");
        Serial.print(alignTarget, 1);
        Serial.println("deg to align");
    } else {
        mot->RotateCCW(SCAN_ROTATE_SPEED);
    }
}

// ── ALIGN: rotate shortest path to face light source ─────────────

void FSM::doAlign()
{
    if (!alignInitialized) {
        mot->resetHeading();
        alignInitialized = true;
    }

    float h       = mot->getHeading();
    // rotated = how far we've turned in the correct direction (positive)
    float rotated   = (alignTarget >= 0.0f) ? h : -h;
    float remaining = fabsf(alignTarget) - rotated;

    if (remaining < (float)ALIGN_DEG) {
        mot->Stop();
        mot->resetHeading();
        state = State::APPROACH;
        Serial.println("Aligned — approaching");
        return;
    }

    if (alignTarget > 0.0f) {
        mot->RotateCCW(ROTATE_SPEED);
    } else {
        mot->RotateCW(ROTATE_SPEED);
    }
}

// ── APPROACH: drive forward, auto-switch long/short range sensors ─

void FSM::doApproach()
{
    // Snapshot brightness at the start of approach for stray detection
    if (approachPeakBright == 0) {
        approachPeakBright = totalBrightness();
        filteredDiff = 0.0f;
    }

    // Short-range sensors active → within ~25mm, check for 10mm stop
    bool shortActive = (minShortADC() < SHORT_ACTIVE_ADC);

    if (shortActive && minShortADC() < CLOSE_ADC) {
        mot->Stop(true);
        approachPeakBright = 0;
        state = State::DONE;
        Serial.println("Reached light source — stopped");
        return;
    }

    // Stray detection — if brightness drops to less than STRAY_FRACTION of peak, re-scan
    int current = totalBrightness();
    if (current < (int)(approachPeakBright * STRAY_FRACTION)) {
        mot->Stop();
        approachPeakBright = 0;
        state = State::SCAN;
        Serial.println("Lost light — re-scanning");
        return;
    }

    int leftBrightness, rightBrightness;

    if (shortActive) {
        leftBrightness  = 1023 - perc->getPTCloseLeft();
        rightBrightness = 1023 - perc->getPTCloseRight();
        Serial.print("Sensor: SHORT  ");
    } else {
        leftBrightness  = 1023 - perc->getPTLongLeft();
        rightBrightness = 1023 - perc->getPTLongRight();
        Serial.print("Sensor: LONG   ");
    }

    // EMA-smooth the diff to reduce jittery steering
    int rawDiff = rightBrightness - leftBrightness;
    filteredDiff = DIFF_EMA_ALPHA * rawDiff + (1.0f - DIFF_EMA_ALPHA) * filteredDiff;

    int wz = 0;
    if (fabsf(filteredDiff) > STEER_DEADBAND) {
        wz = constrain((int)(-filteredDiff * STEER_SCALE), -MAX_STEER, MAX_STEER);
    }

    Serial.print("diff:"); Serial.print((int)filteredDiff);
    Serial.print(" wz:"); Serial.println(wz);

    mot->drive(APPROACH_SPEED, 0, wz);
}

// ── FAN_ON: run fan, confirm light gone, move to next source ──────

void FSM::doFanOn()
{
    digitalWrite(PIN_FAN, HIGH);

    if (totalBrightness() < DIM_BRIGHTNESS) {
        if (!dimTimerActive) {
            dimTimerActive = true;
            dimStartMs     = millis();
        } else if (millis() - dimStartMs >= (unsigned long)DIM_CONFIRM_MS) {
            digitalWrite(PIN_FAN, LOW);
            dimTimerActive = false;
            sourcesHandled++;

            Serial.print("Source ");
            Serial.print(sourcesHandled);
            Serial.println(" done");

            if (sourcesHandled >= 2) {
                state = State::DONE;
                Serial.println("All sources handled — done");
            } else {
                state = State::SCAN;
            }
        }
    } else {
        dimTimerActive = false;
    }
}
