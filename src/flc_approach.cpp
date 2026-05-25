#include "flc_approach.h"

// Local membership index enums
enum OffIdx  { NL=0, NM, ZE, PM, PL };            // fire_offset: 5 sets
enum DistIdx { VERY_CLOSE=0, CLOSE, MEDIUM, FAR }; // obs_dist:    4 sets
enum SideIdx { CLEAR_RIGHT=0, NEUTRAL, CLEAR_LEFT };// clear_side:  3 sets

FLCApproach::FLCApproach(percepetion* p, movement* m)
    : perc(p), mot(m), photo_l(0), photo_r(0) {}

void FLCApproach::setup() {
    pinMode(PIN_PHOTO_LEFT,  INPUT);
    pinMode(PIN_PHOTO_RIGHT, INPUT);
}

int  FLCApproach::fireOffset()  const { return photo_r - photo_l; }
bool FLCApproach::fireVisible() const { return (photo_l + photo_r) > FIRE_DETECT_THRESH; }

bool FLCApproach::atFire() const {
    bool bright   = (photo_l + photo_r) > FIRE_CLOSE_THRESH;
    float dist    = perc->getUltrasonicCm();
    bool sonar_ok = (dist > 1.0f && dist < FIRE_CLOSE_CM);
    bool centred  = (abs(fireOffset()) < (int)FIRE_CENTRE_THRESH);
    return centred && (bright || sonar_ok);
}

float FLCApproach::trimf(float x, float a, float b, float c) {
    if (x <= a || x >= c) return 0.0f;
    if (x == b)           return 1.0f;
    if (x < b)            return (x - a) / (b - a);
    return                       (c - x) / (c - b);
}

float FLCApproach::defuzz(const float* s, const float* v, int n) {
    float num = 0.0f, den = 0.0f;
    for (int i = 0; i < n; i++) { num += s[i] * v[i]; den += s[i]; }
    return (den > 0.001f) ? num / den : 0.0f;
}

void FLCApproach::update() {
    photo_l = analogRead(PIN_PHOTO_LEFT);
    photo_r = analogRead(PIN_PHOTO_RIGHT);

    // ── Derive FLC inputs ────────────────────────────────────────────
    float offset = (float)(photo_r - photo_l);     // +ve = fire on RIGHT

    // Treat 0-echo (< 2 cm) as "no obstacle" = FAR
    float d = perc->getUltrasonicCm();
    if (d < 2.0f) d = 80.0f;

    // Normalise each side IR to [0,1] before subtracting — the two sensors
    // have different maximum ranges, so raw subtraction would always bias left.
    float norm_l  = constrain(perc->getIRLongLeft(),  0.0f, 800.0f) / 800.0f;
    float norm_r  = constrain(perc->getIRMedRight(),  0.0f, 300.0f) / 300.0f;
    // Positive = more normalised space on LEFT; negative = more space on RIGHT
    float side = (norm_l - norm_r) * 500.0f;

    // ── Fuzzify ──────────────────────────────────────────────────────
    float mu_o[5], mu_d[4], mu_s[3];

    // fire_offset membership: shoulder MFs use a virtual point outside sensor range
    // so the function stays 1.0 across the extreme zone rather than peaking at 0.
    mu_o[NL] = trimf(offset, -1500, -1023, -300);
    mu_o[NM] = trimf(offset,  -600,  -300,    0);
    mu_o[ZE] = trimf(offset,  -200,     0,  200);
    mu_o[PM] = trimf(offset,     0,   300,  600);
    mu_o[PL] = trimf(offset,   300,  1023, 1500);

    // obs_dist membership (cm)
    mu_d[VERY_CLOSE] = trimf(d,  -5,   0,  15);
    mu_d[CLOSE]      = trimf(d,   8,  20,  35);
    mu_d[MEDIUM]     = trimf(d,  25,  42,  60);
    mu_d[FAR]        = trimf(d,  50,  80,  85);

    // clear_side membership (±500 normalised units)
    mu_s[CLEAR_RIGHT] = trimf(side, -600, -300, -100);
    mu_s[NEUTRAL]     = trimf(side, -200,    0,  200);
    mu_s[CLEAR_LEFT]  = trimf(side,  100,  300,  600);

    // ── wz rules — turn (rad/s proxy): +CCW, -CW ────────────────────
    // Fire on RIGHT (PM, PL) → need CW turn → wz negative.
    // As obstacle closes, turn authority reduces and strafe takes over.
    const int N_WZ = 16;
    float wz_s[N_WZ], wz_v[N_WZ];
    // FAR: fire offset has full authority over turn rate
    wz_s[ 0] = fmin(mu_d[FAR], mu_o[PL]); wz_v[ 0] = -200;
    wz_s[ 1] = fmin(mu_d[FAR], mu_o[PM]); wz_v[ 1] = -100;
    wz_s[ 2] = fmin(mu_d[FAR], mu_o[ZE]); wz_v[ 2] =    0;
    wz_s[ 3] = fmin(mu_d[FAR], mu_o[NM]); wz_v[ 3] =  100;
    wz_s[ 4] = fmin(mu_d[FAR], mu_o[NL]); wz_v[ 4] =  200;
    // MEDIUM: reduced turn — strafe starting to contribute
    wz_s[ 5] = fmin(mu_d[MEDIUM], mu_o[PL]); wz_v[ 5] =  -80;
    wz_s[ 6] = fmin(mu_d[MEDIUM], mu_o[PM]); wz_v[ 6] =  -40;
    wz_s[ 7] = fmin(mu_d[MEDIUM], mu_o[ZE]); wz_v[ 7] =    0;
    wz_s[ 8] = fmin(mu_d[MEDIUM], mu_o[NM]); wz_v[ 8] =   40;
    wz_s[ 9] = fmin(mu_d[MEDIUM], mu_o[NL]); wz_v[ 9] =   80;
    // CLOSE: minimal turn, mostly strafe
    wz_s[10] = fmin(mu_d[CLOSE], mu_o[PL]); wz_v[10] =  -30;
    wz_s[11] = fmin(mu_d[CLOSE], mu_o[PM]); wz_v[11] =  -15;
    wz_s[12] = fmin(mu_d[CLOSE], mu_o[ZE]); wz_v[12] =    0;
    wz_s[13] = fmin(mu_d[CLOSE], mu_o[NM]); wz_v[13] =   15;
    wz_s[14] = fmin(mu_d[CLOSE], mu_o[NL]); wz_v[14] =   30;
    // VERY_CLOSE: stop turning entirely — pure sideways escape
    wz_s[15] = mu_d[VERY_CLOSE];             wz_v[15] =    0;

    // ── vy rules — strafe: +left, -right ────────────────────────────
    // CLEAR_LEFT (more space left) → strafe left (+vy).
    // CLEAR_RIGHT (more space right) → strafe right (-vy).
    // FAR: no strafe (pure fire-chasing turn only).
    const int N_VY = 10;
    float vy_s[N_VY], vy_v[N_VY];
    vy_s[0] = mu_d[FAR];                                  vy_v[0] =    0;
    vy_s[1] = fmin(mu_d[MEDIUM], mu_s[CLEAR_LEFT]);       vy_v[1] =   80;
    vy_s[2] = fmin(mu_d[MEDIUM], mu_s[NEUTRAL]);          vy_v[2] =    0;
    vy_s[3] = fmin(mu_d[MEDIUM], mu_s[CLEAR_RIGHT]);      vy_v[3] =  -80;
    vy_s[4] = fmin(mu_d[CLOSE],  mu_s[CLEAR_LEFT]);       vy_v[4] =  150;
    vy_s[5] = fmin(mu_d[CLOSE],  mu_s[NEUTRAL]);          vy_v[5] =    0;
    vy_s[6] = fmin(mu_d[CLOSE],  mu_s[CLEAR_RIGHT]);      vy_v[6] = -150;
    vy_s[7] = fmin(mu_d[VERY_CLOSE], mu_s[CLEAR_LEFT]);   vy_v[7] =  200;
    vy_s[8] = fmin(mu_d[VERY_CLOSE], mu_s[NEUTRAL]);      vy_v[8] =    0;
    vy_s[9] = fmin(mu_d[VERY_CLOSE], mu_s[CLEAR_RIGHT]);  vy_v[9] = -200;

    // ── vx — forward speed decreases as obstacle closes ──────────────
    float vx = 150.0f * mu_d[FAR]
             +  80.0f * mu_d[MEDIUM]
             +  20.0f * mu_d[CLOSE]
             +   0.0f * mu_d[VERY_CLOSE];

    int wz = (int)defuzz(wz_s, wz_v, N_WZ);
    int vy = (int)defuzz(vy_s, vy_v, N_VY);

    mot->drive((int)vx, vy, wz);
}
