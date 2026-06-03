#include "movement.h"
#include "percepetion.h"
#include <EEPROM.h>

percepetion perception;
movement motors(&perception);

static int servoPos = 90;
static int escState = 0;

static const int EEPROM_ADDR = 0;

void setup() {
    Serial.begin(115200);
    perception.init();

    uint8_t saved = EEPROM.read(EEPROM_ADDR);
    if (saved <= 180) servoPos = saved;

    motors.enable();
    motors.setServoAngle(servoPos);
    Serial.print("Restored position: ");
    Serial.println(servoPos);
    Serial.println("Use left/right arrow keys to move servo");
}

void loop() {
    while (Serial.available()) {
        int c = Serial.read();
        if (escState == 0) {
            if (c == 27) escState = 1;
        } else if (escState == 1) {
            escState = (c == '[') ? 2 : 0;
        } else if (escState == 2) {
            escState = 0;
            if (c == 'C') {
                servoPos = constrain(servoPos - 5, 0, 180);
                motors.setServoAngle(servoPos);
                EEPROM.update(EEPROM_ADDR, (uint8_t)servoPos);
                Serial.println(servoPos);
            } else if (c == 'D') {
                servoPos = constrain(servoPos + 5, 0, 180);
                motors.setServoAngle(servoPos);
                EEPROM.update(EEPROM_ADDR, (uint8_t)servoPos);
                Serial.println(servoPos);
            }
        }
    }
}
