Arduino BH1745
==============

The Rohm Semiconductor BH1745 is a color sensor that uses the I²C bus protocol to easily deliver values to a microcontroller such as the Arduino family. Pimoroni offers a breakout board for this chip, but only provides a MicroPython library for RPxxxx microcontrollers (mainly the Raspberry Pi Pico).

This project aims to provide a simple-to-use interface for Arduino users, and attempts to provide more features than older libraries for this sensor.

## Installation/Usage
Using Arduino-CLI:
```shellsession
$ arduino-cli config set library.enable_unsafe_install true
$ arduino-cli lib install --git-url https://codeberg.org/void_panic/arduino-bh1745.git
```

To use, include `BH1745.hpp` in your sketch file, and initialize one or two instances.

```ino
#include <BH1745.hpp>

BH1745 colorSensor = BH1745();

void setup() {
    BH1745::begin();
}

void loop() {
    // Blink LEDs (on Pimoroni breakout.)
    colorSensor.setLED(true);
    sleep(200);
    colorSensor.setLED(false);
    sleep(200)
}
```

See the header file and your IDE's autocompletion for available methods.
