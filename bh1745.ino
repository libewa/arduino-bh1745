#include "BH1745.h"

BH1745 colorSensor = BH1745();

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    BH1745::begin();
    if (!colorSensor.init()) {
        for (;;) {
            digitalWrite(LED_BUILTIN, HIGH);
            delay(50);
            digitalWrite(LED_BUILTIN, LOW);
            delay(950);
        }
    }
}

void loop() {
    // Blink LEDs (on Pimoroni breakout.)
    colorSensor.setLED(true);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
    colorSensor.setLED(false);
    digitalWrite(LED_BUILTIN, LOW);
    delay(200);
}
