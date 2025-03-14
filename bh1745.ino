#include "BH1745.h"

BH1745 colorSensor = BH1745();

void setup() {
    BH1745::begin();
}

void loop() {
    // Blink LEDs (on Pimoroni breakout.)
    colorSensor.setLED(true);
    delay(200);
    colorSensor.setLED(false);
    delay(200);
}
