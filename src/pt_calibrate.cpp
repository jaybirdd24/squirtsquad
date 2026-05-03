// ─────────────────────────────────────────────────────────────────────────────
// Phototransistor Resistor Calibration Sketch
//
// !! SET THE PIN AND WIRING MODE BELOW BEFORE UPLOADING !!
//
// Flash:   pio run -e pt_calibrate -t upload
// Monitor: pio device monitor -e pt_calibrate   (115200 baud)
//
// Commands (Serial Monitor):
//   a  — capture AMBIENT  (no target light source, just room light)
//   f  — capture FAR      (target light on, robot navigating toward it)
//   n  — capture NEAR     (target light on, sensor ~10 cm away)
//   c  — snapshot current reading
//   h  — reprint the header
//
// Captures all three conditions then prints:
//   • Threshold 1 — ambient vs far  (detect light source, start moving)
//   • Threshold 2 — far vs near     (detect 10 cm proximity, stop)
//   • Resistor sizing hint for both
// ─────────────────────────────────────────────────────────────────────────────

#include <Arduino.h>

// ── 1. Select the analog pin under test (uncomment ONE) ──────────────────────
// #define PT_PIN  A1
// #define PT_PIN  A2
// #define PT_PIN  A3
// #define PT_PIN  A4
// #define PT_PIN  A5
// #define PT_PIN  A6
// #define PT_PIN  A7
// #define PT_PIN  A11
// #define PT_PIN  A13
// #define PT_PIN  A14
// #define PT_PIN  A15
#define PT_PIN  A1   // <-- change this to whichever pin you are testing

// ── 2. Select the wiring configuration (uncomment ONE) ───────────────────────
//
// PULL_DOWN:  VCC → collector | emitter → R → GND   (read at emitter)
//             More light  =  transistor conducts more  =  voltage RISES
//
// PULL_UP:    VCC → R → collector | emitter → GND   (read at collector)
//             More light  =  transistor conducts more  =  voltage FALLS
//
#define WIRING_PULLDOWN
// #define WIRING_PULLUP

#if !defined(WIRING_PULLDOWN) && !defined(WIRING_PULLUP)
  #error "Select a wiring mode: uncomment WIRING_PULLDOWN or WIRING_PULLUP"
#endif

// ── Capture settings ─────────────────────────────────────────────────────────
constexpr uint8_t  CAPTURE_SAMPLES  = 100;
constexpr uint8_t  CAPTURE_DELAY_MS = 5;
constexpr uint16_t STREAM_PERIOD_MS = 100;
constexpr float    VREF             = 5.0f;
constexpr float    ADC_MAX          = 1023.0f;

// ─────────────────────────────────────────────────────────────────────────────

struct Stats {
    float avg;
    int   minVal;
    int   maxVal;
    bool  valid;
};

static Stats ambientStats = {0, 0, 0, false};
static Stats farStats     = {0, 0, 0, false};
static Stats nearStats    = {0, 0, 0, false};

static float adcToVolts(int raw) {
    return (float)raw * VREF / ADC_MAX;
}

static int voltsToAdc(float v) {
    return (int)(v * ADC_MAX / VREF);
}

static Stats captureStats() {
    long sum    = 0;
    int  minVal = 1023;
    int  maxVal = 0;

    for (uint8_t i = 0; i < CAPTURE_SAMPLES; i++) {
        int v = analogRead(PT_PIN);
        sum += v;
        if (v < minVal) minVal = v;
        if (v > maxVal) maxVal = v;
        delay(CAPTURE_DELAY_MS);
    }

    Stats s;
    s.avg    = (float)sum / CAPTURE_SAMPLES;
    s.minVal = minVal;
    s.maxVal = maxVal;
    s.valid  = true;
    return s;
}

static void printStats(const char* label, const Stats& s) {
    Serial.print(F("# "));
    Serial.print(label);
    Serial.print(F("  avg="));
    Serial.print(s.avg, 1);
    Serial.print(F(" ("));
    Serial.print(adcToVolts((int)s.avg), 3);
    Serial.print(F(" V)  min="));
    Serial.print(s.minVal);
    Serial.print(F("  max="));
    Serial.print(s.maxVal);
    Serial.print(F("  spread="));
    Serial.println(s.maxVal - s.minVal);
}

static void printSeparationHint(const char* labelLow, float vLow,
                                const char* labelHigh, float vHigh,
                                const char* purpose) {
    float sep = vHigh - vLow;
#if defined(WIRING_PULLUP)
    sep = -sep;
    float tmp = vLow; vLow = vHigh; vHigh = tmp;
#endif
    float mid = (vLow + vHigh) / 2.0f;

    Serial.print(F("#   ["));
    Serial.print(purpose);
    Serial.print(F("]  "));
    Serial.print(labelLow);
    Serial.print(F("="));
    Serial.print(vLow, 3);
    Serial.print(F(" V  "));
    Serial.print(labelHigh);
    Serial.print(F("="));
    Serial.print(vHigh, 3);
    Serial.print(F(" V  sep="));
    Serial.print(fabsf(sep), 3);
    Serial.print(F(" V"));

    if (fabsf(sep) < 0.3f) {
        Serial.println(F("  !! LOW"));
    } else if (fabsf(sep) < 1.0f) {
        Serial.println(F("  ~ MODERATE"));
    } else {
        Serial.print(F("  OK  threshold="));
        Serial.print(mid, 3);
        Serial.print(F(" V (ADC ~"));
        Serial.print(voltsToAdc(mid));
        Serial.println(F(")"));
    }
}

static void printResistorHint(float vAmbient, float vNear) {
#if defined(WIRING_PULLDOWN)
    float vHigh = vNear;
    float vLow  = vAmbient;
#else
    float vHigh = vAmbient;
    float vLow  = vNear;
#endif
    float sep = vHigh - vLow;

    Serial.println(F("# ── Resistor sizing hint ──────────────────────────────────"));
    if (fabsf(sep) < 0.3f) {
        Serial.println(F("# !! Very low overall contrast — resistor change needed."));
#if defined(WIRING_PULLDOWN)
        if (vHigh < 1.5f) {
            Serial.println(F("#    NEAR reading is low  => try a LARGER resistor (e.g. 2x)."));
        } else {
            Serial.println(F("#    AMBIENT reading is high  => try a SMALLER resistor (e.g. 0.5x)."));
        }
#else
        if (vLow > 3.5f) {
            Serial.println(F("#    NEAR reading is high  => try a LARGER resistor (e.g. 2x)."));
        } else {
            Serial.println(F("#    AMBIENT reading is low  => try a SMALLER resistor (e.g. 0.5x)."));
        }
#endif
    } else if (fabsf(sep) < 1.0f) {
        Serial.println(F("# ~  Moderate overall range — aim for >1 V from ambient to near."));
        Serial.println(F("#    Try a slightly larger resistor to increase swing."));
    } else {
        Serial.println(F("# OK Overall range looks good."));
    }
}

static void printFullSummary() {
    if (!ambientStats.valid || !farStats.valid || !nearStats.valid) return;

    float vAmbient = adcToVolts((int)ambientStats.avg);
    float vFar     = adcToVolts((int)farStats.avg);
    float vNear    = adcToVolts((int)nearStats.avg);

    Serial.println(F("# ── Summary ───────────────────────────────────────────────"));
    printSeparationHint("AMBIENT", vAmbient, "FAR",  vFar,  "detect light / start moving");
    printSeparationHint("FAR",     vFar,     "NEAR", vNear, "10cm proximity / stop      ");
    printResistorHint(vAmbient, vNear);
    Serial.println(F("# ──────────────────────────────────────────────────────────"));
}

static void printPartialSummary() {
    if (ambientStats.valid && farStats.valid) {
        float vAmbient = adcToVolts((int)ambientStats.avg);
        float vFar     = adcToVolts((int)farStats.avg);
        Serial.println(F("# ── Partial summary (need NEAR to complete) ───────────────"));
        printSeparationHint("AMBIENT", vAmbient, "FAR", vFar, "detect light / start moving");
        Serial.println(F("# ──────────────────────────────────────────────────────────"));
    } else if (farStats.valid && nearStats.valid) {
        float vFar  = adcToVolts((int)farStats.avg);
        float vNear = adcToVolts((int)nearStats.avg);
        Serial.println(F("# ── Partial summary (need AMBIENT to complete) ────────────"));
        printSeparationHint("FAR", vFar, "NEAR", vNear, "10cm proximity / stop");
        Serial.println(F("# ──────────────────────────────────────────────────────────"));
    }
}

static void printHeader() {
    Serial.println(F("# ── Phototransistor Resistor Calibration ──────────────────"));
#if defined(WIRING_PULLDOWN)
    Serial.println(F("# Wiring: PULL-DOWN  (more light => voltage rises)"));
#else
    Serial.println(F("# Wiring: PULL-UP    (more light => voltage falls)"));
#endif
    Serial.print(F("# Pin: A"));
    Serial.println(PT_PIN - A0);
    Serial.println(F("# Steps: 1) type 'a' with only room light"));
    Serial.println(F("#         2) type 'f' with target light on, robot far away"));
    Serial.println(F("#         3) type 'n' with sensor ~10 cm from target light"));
    Serial.println(F("# Other: c=snapshot  h=header"));
}

void setup() {
    Serial.begin(115200);
    while (!Serial) {}
    pinMode(PT_PIN, INPUT);
    printHeader();
}

static String inputBuf;

void loop() {
    // ── Live stream ────────────────────────────────────────────────────────
    static unsigned long lastStream = 0;
    if (millis() - lastStream >= STREAM_PERIOD_MS) {
        lastStream = millis();
        int raw = analogRead(PT_PIN);
        Serial.print(F("# live  raw="));
        Serial.print(raw);
        Serial.print(F("  V="));
        Serial.println(adcToVolts(raw), 3);
    }

    // ── Serial input ───────────────────────────────────────────────────────
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            inputBuf.trim();
            if (inputBuf.length() == 0) { inputBuf = ""; continue; }

            if (inputBuf.equalsIgnoreCase("h")) {
                printHeader();

            } else if (inputBuf.equalsIgnoreCase("c")) {
                int raw = analogRead(PT_PIN);
                Serial.print(F("# snap  raw="));
                Serial.print(raw);
                Serial.print(F("  V="));
                Serial.println(adcToVolts(raw), 3);

            } else if (inputBuf.equalsIgnoreCase("a")) {
                Serial.println(F("# Capturing AMBIENT (room light, no target)..."));
                ambientStats = captureStats();
                printStats("AMBIENT", ambientStats);
                if (ambientStats.valid && farStats.valid && nearStats.valid)
                    printFullSummary();
                else
                    printPartialSummary();

            } else if (inputBuf.equalsIgnoreCase("f")) {
                Serial.println(F("# Capturing FAR (target light on, robot navigating toward it)..."));
                farStats = captureStats();
                printStats("FAR    ", farStats);
                if (ambientStats.valid && farStats.valid && nearStats.valid)
                    printFullSummary();
                else
                    printPartialSummary();

            } else if (inputBuf.equalsIgnoreCase("n")) {
                Serial.println(F("# Capturing NEAR (sensor ~10 cm from target light)..."));
                nearStats = captureStats();
                printStats("NEAR   ", nearStats);
                if (ambientStats.valid && farStats.valid && nearStats.valid)
                    printFullSummary();
                else
                    printPartialSummary();

            } else {
                Serial.println(F("# Unknown — a=AMBIENT  f=FAR  n=NEAR  c=snapshot  h=header"));
            }
            inputBuf = "";
        } else {
            inputBuf += c;
        }
    }
}
