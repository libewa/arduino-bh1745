#include "BH1745.h"

#include <Arduino.h>
#include <Wire.h>

// Register addresses
#define SYSTEM_CONTROL  0x40
#define MODE_CONTROL1   0x41
#define MODE_CONTROL2   0x42
#define MODE_CONTROL3   0x44
#define COLOUR_DATA     0x50
#define INTERRUPT       0x60
#define THRESHOLD       0x62
#define MANUFACTURER    0x92

// Bit masks and settings
#define SW_RESET        0b01000000
#define INT_RESET       0b00100000
#define PART_ID_MASK    0b00001111

#define PART_ID         0b00001011
#define MANUFACTURER_ID 0xE0

void BH1745::begin() {
    Wire.begin();
#ifdef BH1745_DEBUG
    Serial.begin(9600);
    Serial.println("This is arduino-BH1745 debug mode!");
#endif
}

BH1745::BH1745(uint8_t address) {
    address = address;

    setWhiteBalance(2.2, 1.0, 1.8, 10.0);
    enableWhiteBalance = true;

    modeControl2.valid = 0;
    modeControl2.rgb_en = 1;
    modeControl2.adc_gain = 0b00;

}

bool BH1745::init() {
    // "Power on time: t1: t1 should be more than 2ms ..."
    delay(3);
    reset(); // safety

    uint8_t part_id = readRegister(SYSTEM_CONTROL) & PART_ID_MASK;
    uint8_t manufacturer_id = readRegister(MANUFACTURER);
    if (part_id != PART_ID || manufacturer_id != MANUFACTURER_ID) {
        Serial.println("BH1745 not found: Manufacturer or Part ID mismatch!");
        return false;
    }

    reset();
    setBit(SYSTEM_CONTROL, 6, false);

    setMeasurementTime(160);
    writeRegister(MODE_CONTROL2, makeModeControl2());
    writeRegister(MODE_CONTROL3, 0x02); // Copied from documentation
    delay(5);
    return true;
}

void BH1745::setBit(uint8_t reg, uint8_t bit, bool newValue) {
    uint8_t val = readRegister(reg);
    uint8_t mask = ~(1 << bit);
    val &= mask;
    if (newValue == true) {
        val |= 1 << bit;
    }
    writeRegister(reg, val);
}

bool BH1745::readBit(uint8_t reg, uint8_t bit) {
    return readRegister(reg) & (1 << bit);
}

void BH1745::reset() {
    setBit(SYSTEM_CONTROL, 7, true);
    while (readBit(SYSTEM_CONTROL, 7)) {
        delay(100);
    }
}

void BH1745::getRGBC(uint16_t &r, uint16_t &g, uint16_t &b, uint16_t &c) {
    uint8_t data[8];
    readRegisters(COLOUR_DATA, data, 8);

    r = (data[1] << 8) | data[0];
    g = (data[3] << 8) | data[2];
    b = (data[5] << 8) | data[4];
    c = (data[7] << 8) | data[6];

    if (enableWhiteBalance) {
        r = r * channelCompensation[0];
        g = g * channelCompensation[1];
        b = b * channelCompensation[2];
        c = c * channelCompensation[3];
    }
}

void BH1745::getRGBClamped(uint16_t &r, uint16_t &g, uint16_t &b) {
    uint16_t c;
    getRGBC(r, g, b, c);

    uint16_t div = max(r, max(g, b));

    if (div > 0) {
        r = (r * 255) / div;
        g = (g * 255) / div;
        b = (b * 255) / div;
    } else {
        r = g = b = 0;
    }
}

void BH1745::getRGBScaled(uint16_t &r, uint16_t &g, uint16_t &b) {
    uint16_t c;
    getRGBC(r, g, b, c);

    if (c > 0) {
        r = min(255, (r * 255) / c);
        g = min(255, (g * 255) / c);
        b = min(255, (b * 255) / c);
    } else {
        r = g = b = 0;
    }
}

bool BH1745::setMeasurementTime(uint16_t time_ms) {
    uint8_t time_reg;

    switch (time_ms) {
        case 160:
            time_reg = 0b000;
            break;
        case 320:
            time_reg = 0b001;
            break;
        case 640:
            time_reg = 0b010;
            break;
        case 1280:
            time_reg = 0b011;
            break;
        case 2560:
            time_reg = 0b100;
            break;
        case 5120:
            time_reg = 0b101;
            break;
        default:
            return false; // Invalid time_ms
    }

    writeRegister(MODE_CONTROL1, time_reg);

    return true;
}

void BH1745::setLED(bool enable) {
    setBit(INTERRUPT, 0, enable);
}

bool BH1745::setADCGain(uint8_t gain) {
    switch (gain) {
        case 1:
            modeControl2.adc_gain = 0b00;
            break;
        case 2:
            modeControl2.adc_gain = 0b01;
            break;
        case 16:
            modeControl2.adc_gain = 0b10;
            break;
        default:
            return false;
    }
    return true;
}

void BH1745::setWhiteBalance(float r, float g, float b, float c) {
    channelCompensation[0] = r;
    channelCompensation[1] = g;
    channelCompensation[2] = b;
    channelCompensation[3] = c;
}

uint8_t BH1745::readRegister(uint8_t reg) {
#ifdef BH1745_DEBUG
    Serial.print("Reading register ");
    Serial.print(reg);
    Serial.println("…");
#endif
    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(address, (uint8_t)1);
    uint8_t res = Wire.read();
    Wire.endTransmission();
#ifdef BH1745_DEBUG
    Serial.print("Register ");
    Serial.print(reg);
    Serial.print(" has the value ");
    Serial.println(res);
#endif
    return res;
}

void BH1745::readRegisters(uint8_t reg, uint8_t *data, uint8_t length) {
    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(address, length);
    for (uint8_t i = 0; i < length; i++) {
        data[i] = Wire.read();
    }
    Wire.endTransmission();
}

void BH1745::writeRegister(uint8_t reg, uint8_t value) {
#ifdef BH1745_DEBUG
    Serial.print("Writing ");
    Serial.print(value);
    Serial.print(" to register ");
    Serial.println(reg);
#endif
    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

uint8_t BH1745::makeModeControl2() {
    return modeControl2.valid << 7 | modeControl2.rgb_en << 4 | modeControl2.adc_gain;
}
