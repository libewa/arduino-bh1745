#include "BH1745.hpp"
#include <Wire.h>

// Register addresses
#define SYSTEM_CONTROL 0x40
#define MODE_CONTROL1 0x41
#define MODE_CONTROL2 0x42
#define MODE_CONTROL3 0x44
#define COLOUR_DATA 0x50
#define INTERRUPT 0x60
#define THRESHOLD 0x62
#define MANUFACTURER 0x92

// Bit masks and settings
#define SW_RESET 0x80
#define INT_RESET 0x40
#define PART_ID_MASK 0x3F

#define PART_ID 0x0B
#define MANUFACTURER_ID 0xE0
#define ADC_GAIN_X1 0x00
#define RGB_EN 0x10

void BH1745::begin() {
    Wire.begin();
}

BH1745::BH1745(uint8_t address) {
    address = address;
    setChannelCompensation(2.2, 1.0, 1.8, 10.0);
    enableChannelCompensation = true;

    uint8_t part_id = readRegister(SYSTEM_CONTROL) & PART_ID_MASK;
    uint8_t manufacturer_id = readRegister(MANUFACTURER);

    if (part_id != PART_ID || manufacturer_id != MANUFACTURER_ID) {
        Serial.println("BH1745 not found: Manufacturer or Part ID mismatch!");
        while (1)
            ;
    }

    writeRegister(SYSTEM_CONTROL, SW_RESET);
    delay(10);
    writeRegister(SYSTEM_CONTROL, INT_RESET);
    setMeasurementTime(320);
    writeRegister(MODE_CONTROL2, ADC_GAIN_X1 | RGB_EN);
    writeRegister(MODE_CONTROL3, true);
}

void BH1745::getRGBC(uint16_t &r, uint16_t &g, uint16_t &b, uint16_t &c) {
    uint8_t data[8];
    readRegisters(COLOUR_DATA, data, 8);

    r = (data[1] << 8) | data[0];
    g = (data[3] << 8) | data[2];
    b = (data[5] << 8) | data[4];
    c = (data[7] << 8) | data[6];

    if (enableChannelCompensation) {
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

void BH1745::setMeasurementTime(uint16_t time_ms) {
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
            return; // Invalid time_ms
    }

    writeRegister(MODE_CONTROL1, time_reg);
}

void BH1745::setLED(bool enable) {
    writeRegister(INTERRUPT, enable);
}

void BH1745::setADCGain(uint8_t gain) {
    switch (gain) {
        case 1:
            setADCGainRaw(0x00);
            break;
        case 2:
            setADCGainRaw(0x01);
            break;
        case 16:
            setADCGainRaw(0x10) break;
        default:
            break;
    }
}

void BH1745::setChannelCompensation(float r, float g, float b, float c) {
    channelCompensation[0] = r;
    channelCompensation[1] = g;
    channelCompensation[2] = b;
    channelCompensation[3] = c;
}

uint8_t BH1745::readRegister(uint8_t reg) {
    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom(address, 1);
    return Wire.read();
}

void BH1745::readRegisters(uint8_t reg, uint8_t *data, uint8_t length) {
    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom(address, length);
    for (uint8_t i = 0; i < length; i++) {
        data[i] = Wire.read();
    }
}

void BH1745::writeRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

void BH1745::setADCGainRaw(uint8_t value) {
    uint8_t mc2 = readRegister(MODE_CONTROL2);
    mc2 = (mc2 & ~0b11) & value;
    writeRegister(MODE_CONTROL2, mc2);
}
