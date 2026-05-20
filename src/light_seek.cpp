#include "light_seek.h"
#include <Arduino.h>

LightSeek::LightSeek(percepetion *perc, movement *mot)
    : perc(perc), mot(mot)
{}

void LightSeek::update()
{
    // Invert ADC so higher value = more light (lower ADC = more light in pull-up circuit)
    int leftBrightness  = (1023 - perc->getPTLongLeft())  + (1023 - perc->getPTCloseLeft());
    int rightBrightness = (1023 - perc->getPTLongRight()) + (1023 - perc->getPTCloseRight());

    // Positive diff = right brighter → rotate CW (negative wz)
    int diff = rightBrightness - leftBrightness;

    int wz = 0;
    if (abs(diff) > STEER_DEADBAND) {
        wz = constrain((int)(-diff * STEER_SCALE), -MAX_STEER, MAX_STEER);
    }

    mot->drive(DRIVE_SPEED, 0, wz);

    Serial.print("L:"); Serial.print(leftBrightness);
    Serial.print(" R:"); Serial.print(rightBrightness);
    Serial.print(" diff:"); Serial.print(diff);
    Serial.print(" wz:"); Serial.println(wz);
}
