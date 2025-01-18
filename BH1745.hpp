#ifndef BH1745_hpp
#define BH1745_hpp

#include <stdint.h>
class BH1745
{
public:
    uint8_t address;
    bool enableChannelCompensation;

    BH1745(uint8_t address);
    void begin();
    void getRGBC(uint16_t &r, uint16_t &g, uint16_t &b, uint16_t &c);
    void getRGBClamped(uint16_t &r, uint16_t &g, uint16_t &b);
    void getRGBScaled(uint16_t &r, uint16_t &g, uint16_t &b);
    void setMeasurementTime(uint16_t time_ms);
    void setLED(bool enable);
    void setADCGain(uint8_t gain);
    void setChannelCompensation(float r, float g, float b, float c);

private:
    float channelCompensation[4];

    uint8_t readRegister(uint8_t reg);
    void readRegisters(uint8_t reg, uint8_t *data, uint8_t length);
    void writeRegister(uint8_t reg, uint8_t value);
    void setADCGainRaw(uint8_t value);
};

#endif