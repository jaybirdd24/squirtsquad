#include <Arduino.h>
#include "percepetion.h"

static percepetion perception;

void setup()
{
    Serial.begin(115200);
    delay(1000);
    perception.init();
    Serial.println(F("# Ultrasonic test — dist_cm"));
}

void loop()
{
    perception.update();
    float dist = perception.getUltrasonicCm();
    Serial.println(dist, 1);
    delay(100);
}
