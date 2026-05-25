#include "comms.h"
#include <Arduino.h>

// HC-12 wired to SoftwareSerial: RX=10, TX=11 (matches test_movement harness)
comms::comms() : _hc12(10, 11), _last_send_ms(0)
{
}

comms::~comms()
{
}

void comms::init(uint32_t baud)
{
    _hc12.begin(baud);
}

void comms::sendSensorData(float irF, float irL, float irR, float irRR,
                           float sonar, float gyroZ, float battV)
{
    unsigned long now = millis();
    if (now - _last_send_ms < SEND_INTERVAL_MS) return;
    _last_send_ms = now;

    // Use print() chains — snprintf %f is unreliable on AVR libc
    _hc12.print(F("$S,"));
    _hc12.print(irF,   1); _hc12.print(',');
    _hc12.print(irL,   1); _hc12.print(',');
    _hc12.print(irR,   1); _hc12.print(',');
    _hc12.print(irRR,  1); _hc12.print(',');
    _hc12.print(sonar, 1); _hc12.print(',');
    _hc12.print(gyroZ, 4); _hc12.print(',');
    _hc12.println(battV, 2);
}

void comms::sendControlData(float hdgSp, float hdgActual, float hdgErr,
                            float wfSp, float wfActual, float wfErr,
                            int vx, int vy, int wz, int state)
{
    // No rate-limit check — always sent as the second half of the 100 ms pair
    _hc12.print(F("$C,"));
    _hc12.print(hdgSp,     2); _hc12.print(',');
    _hc12.print(hdgActual, 2); _hc12.print(',');
    _hc12.print(hdgErr,    2); _hc12.print(',');
    _hc12.print(wfSp,      1); _hc12.print(',');
    _hc12.print(wfActual,  1); _hc12.print(',');
    _hc12.print(wfErr,     1); _hc12.print(',');
    _hc12.print(vx);           _hc12.print(',');
    _hc12.print(vy);           _hc12.print(',');
    _hc12.print(wz);           _hc12.print(',');
    _hc12.println(state);
}

void comms::sendAvoidanceData(float frontA, float frontB, float left, float right,
                              float sonar, float heading, float targetHeading,
                              float headingError, int state, int phase,
                              int obstacleBlocked, int avoidDirection)
{
    unsigned long now = millis();
    if (now - _last_send_ms < SEND_INTERVAL_MS) return;
    _last_send_ms = now;

    _hc12.print(F("$A,"));
    _hc12.print(frontA, 1);        _hc12.print(',');
    _hc12.print(frontB, 1);        _hc12.print(',');
    _hc12.print(left, 1);          _hc12.print(',');
    _hc12.print(right, 1);         _hc12.print(',');
    _hc12.print(sonar, 1);         _hc12.print(',');
    _hc12.print(heading, 2);       _hc12.print(',');
    _hc12.print(targetHeading, 2); _hc12.print(',');
    _hc12.print(headingError, 2);  _hc12.print(',');
    _hc12.print(state);            _hc12.print(',');
    _hc12.print(phase);            _hc12.print(',');
    _hc12.print(obstacleBlocked);  _hc12.print(',');
    _hc12.println(avoidDirection);

    // Direct Teleplot serial format for the HC-12 receiver stream.
    _hc12.print(F(">front_a_mm:"));       _hc12.println(frontA, 1);
    _hc12.print(F(">front_b_mm:"));       _hc12.println(frontB, 1);
    _hc12.print(F(">left_mm:"));          _hc12.println(left, 1);
    _hc12.print(F(">right_mm:"));         _hc12.println(right, 1);
    _hc12.print(F(">sonar_cm:"));         _hc12.println(sonar, 1);
    _hc12.print(F(">heading:"));          _hc12.println(heading, 2);
    _hc12.print(F(">target_heading:"));   _hc12.println(targetHeading, 2);
    _hc12.print(F(">heading_err:"));      _hc12.println(headingError, 2);
    _hc12.print(F(">state:"));            _hc12.println(state);
    _hc12.print(F(">avoid_phase:"));      _hc12.println(phase);
    _hc12.print(F(">obstacle_blocked:")); _hc12.println(obstacleBlocked);
    _hc12.print(F(">avoid_direction:"));  _hc12.println(avoidDirection);
}

void comms::sendLightData(float heading, float a4, float a5, float a6, float a7)
{
    _hc12.print(F("$L,"));
    _hc12.print(heading, 1); _hc12.print(',');
    _hc12.print(a4, 3);      _hc12.print(',');
    _hc12.print(a5, 3);      _hc12.print(',');
    _hc12.print(a6, 3);      _hc12.print(',');
    _hc12.println(a7, 3);

    _hc12.print(F(">heading:"));  _hc12.println(heading, 1);
    _hc12.print(F(">pt_a4_v:"));  _hc12.println(a4, 3);
    _hc12.print(F(">pt_a5_v:"));  _hc12.println(a5, 3);
    _hc12.print(F(">pt_a6_v:"));  _hc12.println(a6, 3);
    _hc12.print(F(">pt_a7_v:"));  _hc12.println(a7, 3);
}
