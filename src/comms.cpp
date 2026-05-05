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
