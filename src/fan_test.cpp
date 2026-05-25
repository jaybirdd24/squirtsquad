#include <Arduino.h>

constexpr uint8_t FAN_PIN = 5;
constexpr bool FAN_ACTIVE_HIGH = true;
constexpr unsigned long AUTO_STEP_MS = 3000;

static uint8_t fanDuty = 0;
static bool autoMode = true;
static unsigned long lastAutoStep = 0;
static uint8_t autoStep = 0;

static uint8_t outputForDuty(uint8_t duty)
{
    return FAN_ACTIVE_HIGH ? duty : 255 - duty;
}

static void setFan(uint8_t duty)
{
    fanDuty = duty;
    analogWrite(FAN_PIN, outputForDuty(fanDuty));

    Serial.print(F("fan duty = "));
    Serial.print((fanDuty * 100UL) / 255UL);
    Serial.println(F("%"));
}

static void printHelp()
{
    Serial.println(F("Fan MOSFET test on pin 5"));
    Serial.println(F("Commands: 0=off, 1=full, 5=50%, +=up, -=down, a=auto, h=help"));
}

static void handleSerial()
{
    while (Serial.available() > 0) {
        char c = (char)Serial.read();

        switch (c) {
            case '0':
                autoMode = false;
                setFan(0);
                break;
            case '1':
                autoMode = false;
                setFan(255);
                break;
            case '5':
                autoMode = false;
                setFan(128);
                break;
            case '+':
                autoMode = false;
                setFan(fanDuty > 230 ? 255 : fanDuty + 25);
                break;
            case '-':
                autoMode = false;
                setFan(fanDuty < 25 ? 0 : fanDuty - 25);
                break;
            case 'a':
            case 'A':
                autoMode = true;
                autoStep = 0;
                lastAutoStep = 0;
                Serial.println(F("auto cycle enabled"));
                break;
            case 'h':
            case 'H':
                printHelp();
                break;
            default:
                break;
        }
    }
}

static void runAutoCycle()
{
    unsigned long now = millis();
    if (now - lastAutoStep < AUTO_STEP_MS) return;

    lastAutoStep = now;

    switch (autoStep) {
        case 0:
            setFan(0);
            break;
        case 1:
            setFan(128);
            break;
        default:
            setFan(255);
            break;
    }

    autoStep = (autoStep + 1) % 3;
}

void setup()
{
    pinMode(FAN_PIN, OUTPUT);
    analogWrite(FAN_PIN, outputForDuty(0));

    Serial.begin(115200);
    delay(500);

    printHelp();
    Serial.println(F("Starting auto cycle: off -> 50% -> full."));
}

void loop()
{
    handleSerial();
    if (autoMode) {
        runAutoCycle();
    }
}
