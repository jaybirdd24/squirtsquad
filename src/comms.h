#pragma once

#include <SoftwareSerial.h>

class comms
{
    private:
        SoftwareSerial _hc12;
        unsigned long _last_send_ms;
        static const unsigned long SEND_INTERVAL_MS = 100;

    public:
        comms();
        ~comms();

        void init(uint32_t baud = 115200);

        // Stream sensor readings — call every loop(), rate-limited internally
        void sendSensorData(float irF, float irL, float irR, float irRR,
                            float sonar, float gyroZ, float battV);

        // Stream control state for PID tuning — call every loop(), rate-limited internally
        void sendControlData(float hdgSp, float hdgActual, float hdgErr,
                             float wfSp, float wfActual, float wfErr,
                             int vx, int vy, int wz, int state);

        // Stream obstacle-avoidance isolation telemetry
        void sendAvoidanceData(float frontA, float frontB, float left, float right,
                               float sonar, float heading, float targetHeading,
                               float headingError, int state, int phase,
                               int obstacleBlocked, int avoidDirection);
};
