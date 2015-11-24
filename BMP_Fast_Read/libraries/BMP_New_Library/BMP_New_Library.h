#ifndef BMP_New_Library_h
#define BMP_New_Library_h

#include "Arduino.h"

#define BMP085_ADDRESS (0x77)

class BMP_180{
	public:
		BMP_180();
		bool start();
		int update();
		float getTemp();
		float getPressure();
		float getAltitude(const float &sea_level_pressure);
	private:
		void updateTemp();
		void updatePressure();
		void readS16(byte reg, int16_t &value);
		void read16(byte reg, uint16_t &value);
		void read8(byte reg, uint8_t &value);
		void writeCommand(byte reg, byte value);
		int32_t _raw_temp;
		float _temp;
		float _pressure;
		unsigned int _since_last_update;
		bool _temp_next;
		struct{
		int16_t  ac1;
		int16_t  ac2;
		int16_t  ac3;
		uint16_t ac4;
		uint16_t ac5;
		uint16_t ac6;
		int16_t  b1;
		int16_t  b2;
		int16_t  mb;
		int16_t  mc;
		int16_t  md;
		} _bmp085_coeffs;
};

#endif