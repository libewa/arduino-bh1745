#include "BH1745.h"

BH1745 colorSensor = BH1745(0x39);

uint16_t r, g, b, c;

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
    Serial.begin(9600);
}

void loop() {
    // Blink LEDs (on Pimoroni breakout.)
    colorSensor.setLED(true);
    delay(200);
    colorSensor.setLED(false);
    delay(200);
    colorSensor.getRGBC(r, b, g, c);
    Serial.println(r);
    Serial.println(g);
    Serial.println(b);
    Serial.println(c);
}
