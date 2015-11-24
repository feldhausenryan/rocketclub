#include "Arduino.h"
#include "BMP_New_Library.h"
#include <Wire.h>


enum{
	BMP085_REGISTER_CAL_AC1            = 0xAA,  // R   Calibration data (16 bits)
	BMP085_REGISTER_CAL_AC2            = 0xAC,  // R   Calibration data (16 bits)
	BMP085_REGISTER_CAL_AC3            = 0xAE,  // R   Calibration data (16 bits)
	BMP085_REGISTER_CAL_AC4            = 0xB0,  // R   Calibration data (16 bits)
	BMP085_REGISTER_CAL_AC5            = 0xB2,  // R   Calibration data (16 bits)
	BMP085_REGISTER_CAL_AC6            = 0xB4,  // R   Calibration data (16 bits)
	BMP085_REGISTER_CAL_B1             = 0xB6,  // R   Calibration data (16 bits)
	BMP085_REGISTER_CAL_B2             = 0xB8,  // R   Calibration data (16 bits)
	BMP085_REGISTER_CAL_MB             = 0xBA,  // R   Calibration data (16 bits)
	BMP085_REGISTER_CAL_MC             = 0xBC,  // R   Calibration data (16 bits)
	BMP085_REGISTER_CAL_MD             = 0xBE,  // R   Calibration data (16 bits)
	BMP085_REGISTER_CHIPID             = 0xD0,
	BMP085_REGISTER_VERSION            = 0xD1,
	BMP085_REGISTER_SOFTRESET          = 0xE0,
	BMP085_REGISTER_CONTROL            = 0xF4,
	BMP085_REGISTER_TEMPDATA           = 0xF6,
	BMP085_REGISTER_PRESSUREDATA       = 0xF6,
	BMP085_REGISTER_READTEMPCMD        = 0x2E,
	BMP085_REGISTER_READPRESSURECMD    = 0x34
};

BMP_180::BMP_180(){
	_temp = 0;
	_pressure = 0;
	_since_last_update = 0;
	_temp_next = true;
	
	Wire.begin(); // Start Wire (I2C)
	sercom3.disableWIRE(); // Disable the I2C bus
	SERCOM3->I2CM.BAUD.bit.BAUD = SystemCoreClock / ( 2 * 400000) - 1 ;// Set the I2C SCL frequency to 400kHz
	sercom3.enableWIRE(); // Restart the I2C bus
	
	readS16(BMP085_REGISTER_CAL_AC1, _bmp085_coeffs.ac1);
	readS16(BMP085_REGISTER_CAL_AC2, _bmp085_coeffs.ac2);
	readS16(BMP085_REGISTER_CAL_AC3, _bmp085_coeffs.ac3);
	read16(BMP085_REGISTER_CAL_AC4, _bmp085_coeffs.ac4);
	read16(BMP085_REGISTER_CAL_AC5, _bmp085_coeffs.ac5);
	read16(BMP085_REGISTER_CAL_AC6, _bmp085_coeffs.ac6);
	readS16(BMP085_REGISTER_CAL_B1, _bmp085_coeffs.b1);
	readS16(BMP085_REGISTER_CAL_B2, _bmp085_coeffs.b2);
	readS16(BMP085_REGISTER_CAL_MB, _bmp085_coeffs.mb);
	readS16(BMP085_REGISTER_CAL_MC, _bmp085_coeffs.mc);
	readS16(BMP085_REGISTER_CAL_MD, _bmp085_coeffs.md);
};

bool BMP_180::start(){
	//Starting data for the temperature sensor
	writeCommand(BMP085_REGISTER_CONTROL, BMP085_REGISTER_READTEMPCMD);
	_since_last_update = micros();
}

int BMP_180::update(){
	if ((micros() + 5000 > _since_last_update) && (_temp_next == true)){
		updateTemp();
		writeCommand(BMP085_REGISTER_CONTROL, BMP085_REGISTER_READPRESSURECMD + (0 << 6));
		_since_last_update = micros();
		_temp_next = false;
		return 1;
	}
	if ((micros() + 5000 > _since_last_update) && (_temp_next == false)){
		updatePressure();
		writeCommand(BMP085_REGISTER_CONTROL, BMP085_REGISTER_READTEMPCMD);
		_since_last_update = micros();
		_temp_next = true;
		return 2;
	}
	return 0;
}

void BMP_180::updatePressure(){
	uint8_t  p8;
	uint16_t p16;
	int32_t  p32;

	read16(BMP085_REGISTER_PRESSUREDATA, p16);
	p32 = (uint32_t)p16 << 8;
	read8(BMP085_REGISTER_PRESSUREDATA+2, p8);
	p32 += p8;
	p32 >>= (8 - 0);
	
	int32_t  up = 0, compp = 0;
	int32_t  x1, x2, b5, b6, x3, b3, p;
	uint32_t b4, b7;

	/* Get the raw pressure and compensated temperature values */
	b5 = _raw_temp;
	up = p32;

	/* Pressure compensation */
	b6 = b5 - 4000;
	x1 = (_bmp085_coeffs.b2 * ((b6 * b6) >> 12)) >> 11;
	x2 = (_bmp085_coeffs.ac2 * b6) >> 11;
	x3 = x1 + x2;
	b3 = (((((int32_t) _bmp085_coeffs.ac1) * 4 + x3) << 0) + 2) >> 2;
	x1 = (_bmp085_coeffs.ac3 * b6) >> 13;
	x2 = (_bmp085_coeffs.b1 * ((b6 * b6) >> 12)) >> 16;
	x3 = ((x1 + x2) + 2) >> 2;
	b4 = (_bmp085_coeffs.ac4 * (uint32_t) (x3 + 32768)) >> 15;
	b7 = ((uint32_t) (up - b3) * (50000 >> 0));

	if (b7 < 0x80000000){
		p = (b7 << 1) / b4;
	}
	else{
		p = (b7 / b4) << 1;
	}

	x1 = (p >> 8) * (p >> 8);
	x1 = (x1 * 3038) >> 16;
	x2 = (-7357 * p) >> 16;
	//IN hPa
	_pressure = (p + ((x1 + x2 + 3791) >> 4)) / 100.0;
}

void BMP_180::updateTemp(){
	uint16_t ut;
	float t;
	read16(BMP085_REGISTER_TEMPDATA, ut);
	int32_t X1 = (ut - (int32_t)_bmp085_coeffs.ac6) * ((int32_t)_bmp085_coeffs.ac5) >> 15;
	int32_t X2 = ((int32_t)_bmp085_coeffs.mc << 11) / (X1+(int32_t)_bmp085_coeffs.md);
	int32_t B5 = X1 + X2;
	_raw_temp = B5;
	t = (B5+8) >> 4;
	t /= 10;
	_temp = t;
}

float BMP_180::getTemp(){
	return _temp;
}

float BMP_180::getPressure(){
	return _pressure;
}

float BMP_180::getAltitude(const float &sea_level_pressure){
	return 44330.0 * (1.0 - pow(_pressure / sea_level_pressure, 0.1903));
}

void BMP_180::readS16(byte reg, int16_t &value)
{
  uint16_t i;
  read16(reg, i);
  value = (int16_t)i;
}

void BMP_180::read16(byte reg, uint16_t &value)
{
  Wire.beginTransmission((uint8_t)BMP085_ADDRESS);
  Wire.write((uint8_t)reg);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)BMP085_ADDRESS, (byte)2);
  value = (Wire.read() << 8) | Wire.read();
  Wire.endTransmission();
}

void BMP_180::read8(byte reg, uint8_t &value)
{
	Wire.beginTransmission((uint8_t)BMP085_ADDRESS);
	Wire.write((uint8_t)reg);
	Wire.endTransmission();
	Wire.requestFrom((uint8_t)BMP085_ADDRESS, (byte)1);
	value = Wire.read();
	Wire.endTransmission();
}

void BMP_180::writeCommand(byte reg, byte value)
{
  Wire.beginTransmission((uint8_t)BMP085_ADDRESS);
  Wire.write((uint8_t)reg);
  Wire.write((uint8_t)value);
  Wire.endTransmission();
}