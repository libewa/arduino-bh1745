#ifndef BH1745_h
#define BH1745_h

#include <stdint.h>

struct ModeControl2 {
    unsigned char valid : 1;
    unsigned char :2;
    unsigned char rgb_en : 1;
    unsigned char :2;
    unsigned char adc_gain : 2;
};

/** A singular BH1745 color sensor.
* Before using this library, call BH1745::begin() to initialize I2C. This is not needed if you call Wire.begin() beforehand. This also means that the Arduino cannot be signed in as a slave at the same time.
* @see BH1745()
*/
class BH1745 {
public:
	/// Whether to apply the white balance set with setWhiteBalance().
    bool enableWhiteBalance;

	/** Create the BH1745 class. Optionally takes an address, as the chip supports changing it to 0x39 from the default 0x38.
	* @param address The chip's I2C bus address.
	*/
    BH1745(uint8_t address = 0x38);

    /** Establish a connection with the sensor.
     * @returns `true` if initialization was successfull.
     */
    bool init();

    /// Calls `Wire.begin()` to initialize I2C.
    static void begin();

	/** Gets semi-raw values from the I2C sensor.
	* They are adjusted for white balance if `enableChannelCompensation` is true.
	* @param &r A pointer to store the red channel.
	* @param &g A pointer to store the green channel.
	* @param &b A pointer to store the blue channel.
	* @param &c A pointer to store the clear (brightness) channel.
	*/
    void getRGBC(uint16_t &r, uint16_t &g, uint16_t &b, uint16_t &c);

	/** Gets relative values from the I2C sensor.
	* The values are scaled, so that the highest value is now 255.
	*
	* Pimoroni says:
	* > This will clamp/saturate one of the colour channels, providing a clearer idea of what primary colour an object is most likely to be.
	* > However the resulting colour reaidng will not be accurate for other purposes.
	* >
	* > IE: a value of `(255,0,128)` would produce a result of approximately `(1.0, 0.0, 0.5)` since `max(255,0,128) == 255`, `255/255 == 1`, `0/255 = 0` and `128/255 = 0.50196`.
	*
    * They are adjusted for white balance if `enableChannelCompensation` is true.
	* @see getRGBC(uint16_t, uint16_t, uint16_t, uint16_t)
	*
	* @param &r A pointer to store the red channel.
	* @param &g A pointer to store the green channel.
	* @param &b A pointer to store the blue channel.
	*/
    void getRGBClamped(uint16_t &r, uint16_t &g, uint16_t &b);

	/** Gets relative values from the I2C sensor.
	* The values are scaled, so that the brightness value is now 255.
    * They are adjusted for white balance if `enableChannelCompensation` is true.
	* @see getRGBC(uint16_t, uint16_t, uint16_t, uint16_t)
	*
	* @param &r A pointer to store the red channel.
	* @param &g A pointer to store the green channel.
	* @param &b A pointer to store the blue channel.
	*/
    void getRGBScaled(uint16_t &r, uint16_t &g, uint16_t &b);

	/** Sets the "exposure time" of the sensor.
	* A higher exposure time will saturate the measurements.
	* It must be either 160, 320, 640, 1280, 2560 or 5120.
	*
	* @param time_ms The "exposure time", in Milliseconds, to set the sensor to.
	*/
    bool setMeasurementTime(uint16_t time_ms);

	/** Enables or disables the external pin on the BH1745 chip.
	* It is for example wired to two LEDs on the Pimoroni breakout.
	* @param enable The state to set the pin to.
	*/
    void setLED(bool enable);

	/** Sets a fixed multiplier for the chip's Analog-to-Digital-Converter (ADC).
	* Must be either 1, 2 or 16.
	*
	* @param gain The ADC multiplier
	*/
    bool setADCGain(uint8_t gain);

	/** Sets the white balance values. They are multiplied with the raw sensor values if #enableWhiteBalance is true (default).
	* @param r The compensation to apply to the red channel.
	* @param g The compensation to apply to the green channel.
	* @param b The compensation to apply to the blue channel.
	* @param c The compensation to apply to the brightness channel.
	*/
    void setWhiteBalance(float r, float g, float b, float c);

		void setThresholdHigh(uint16_t value);
		void setThresholdLow(uint16_t value);

private:
	uint8_t address;
    float channelCompensation[4];
    ModeControl2 modeControl2;

    uint8_t readRegister(uint8_t reg);
    void readRegisters(uint8_t reg, uint8_t *data, uint8_t length);
    void writeRegister(uint8_t reg, uint8_t value);
	void writeRegisters(uint8_t startReg, uint8_t values[], uint8_t size);
    uint8_t makeModeControl2();

    void reset();

    void setBit(uint8_t reg, uint8_t bit, bool newValue);
    bool readBit(uint8_t reg, uint8_t bit);
};

#endif
